/**
 * @file tactile_stats_node.cpp
 * @brief Collect tactile data and compute statistical analysis
 *
 * This node collects tactile sensor data over a configurable duration
 * and computes statistics including mean, std dev, min, max, and noise analysis.
 */

#include <cmath>
#include <array>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <wujihandcpp/protocol/handler.hpp>

using namespace std::chrono_literals;

struct PointStats {
    // Raw data storage
    std::vector<double> x_samples;
    std::vector<double> y_samples;
    std::vector<double> z_samples;
    std::vector<double> magnitude_samples;

    // Computed statistics
    struct AxisStats {
        double mean = 0;
        double std_dev = 0;
        double min = 0;
        double max = 0;
        double range = 0;
        double variance = 0;
    };
    AxisStats x, y, z, magnitude;

    void compute() {
        x = compute_axis_stats(x_samples);
        y = compute_axis_stats(y_samples);
        z = compute_axis_stats(z_samples);
        magnitude = compute_axis_stats(magnitude_samples);
    }

private:
    AxisStats compute_axis_stats(const std::vector<double>& samples) {
        AxisStats stats;
        if (samples.empty()) return stats;

        // Mean
        stats.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();

        // Min/Max
        auto [min_it, max_it] = std::minmax_element(samples.begin(), samples.end());
        stats.min = *min_it;
        stats.max = *max_it;
        stats.range = stats.max - stats.min;

        // Variance and Std Dev
        double sq_sum = 0;
        for (const auto& v : samples) {
            sq_sum += (v - stats.mean) * (v - stats.mean);
        }
        stats.variance = sq_sum / samples.size();
        stats.std_dev = std::sqrt(stats.variance);

        return stats;
    }
};

class TactileStatsNode : public rclcpp::Node {
public:
    TactileStatsNode()
        : Node("tactile_stats_node")
        , handler_(0x0483, -1, nullptr, 64, 0)
    {
        // Declare parameters
        declare_parameter("collection_duration", 30.0);  // seconds
        declare_parameter("sample_rate", 10.0);          // Hz (matches tactile data source)
        declare_parameter("output_dir", std::string(TACTILE_VIZ_LOG_DIR));  // Output directory
        declare_parameter("quiet", false);               // Suppress per-point output

        collection_duration_ = get_parameter("collection_duration").as_double();
        sample_rate_ = get_parameter("sample_rate").as_double();
        output_dir_ = get_parameter("output_dir").as_string();
        quiet_ = get_parameter("quiet").as_bool();

        // Create output directory if it doesn't exist
        std::filesystem::create_directories(output_dir_);

        // Generate timestamped filename
        auto now_time = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now_time);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
        timestamp_str_ = ss.str();

        expected_samples_ = static_cast<size_t>(collection_duration_ * sample_rate_);

        RCLCPP_INFO(get_logger(), "=== Tactile Data Statistics Collector ===");
        RCLCPP_INFO(get_logger(), "Collection duration: %.1f seconds", collection_duration_);
        RCLCPP_INFO(get_logger(), "Sample rate: %.1f Hz", sample_rate_);
        RCLCPP_INFO(get_logger(), "Expected samples: %zu", expected_samples_);
        RCLCPP_INFO(get_logger(), "Output directory: %s", output_dir_.c_str());
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Waiting for tactile data...");

        // Create timer for data collection
        auto period = std::chrono::duration<double>(1.0 / sample_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TactileStatsNode::collect_sample, this));

        start_time_ = now();
    }

private:
    void collect_sample() {
        wujihandcpp::protocol::TactileData data;
        if (!handler_.get_tactile_data(data)) {
            return;  // No data yet
        }

        if (!collecting_) {
            collecting_ = true;
            start_time_ = now();
            RCLCPP_INFO(get_logger(), "Data received, starting collection...");
        }

        // Collect samples for each point
        for (int i = 0; i < 36; ++i) {
            double fx = data.points[i].x;
            double fy = data.points[i].y;
            double fz = data.points[i].z;
            double mag = std::sqrt(fx*fx + fy*fy + fz*fz);

            point_stats_[i].x_samples.push_back(fx);
            point_stats_[i].y_samples.push_back(fy);
            point_stats_[i].z_samples.push_back(fz);
            point_stats_[i].magnitude_samples.push_back(mag);
        }

        // Collect resultant force
        double rfx = data.resultant_force.x;
        double rfy = data.resultant_force.y;
        double rfz = data.resultant_force.z;
        double rf_mag = std::sqrt(rfx*rfx + rfy*rfy + rfz*rfz);

        resultant_stats_.x_samples.push_back(rfx);
        resultant_stats_.y_samples.push_back(rfy);
        resultant_stats_.z_samples.push_back(rfz);
        resultant_stats_.magnitude_samples.push_back(rf_mag);

        sample_count_++;

        // Progress update every second
        auto elapsed = (now() - start_time_).seconds();
        if (static_cast<int>(elapsed) > last_progress_sec_) {
            last_progress_sec_ = static_cast<int>(elapsed);
            RCLCPP_INFO(get_logger(), "Collecting... %d/%d seconds, %zu samples",
                        last_progress_sec_, static_cast<int>(collection_duration_), sample_count_);
        }

        // Check if collection is complete
        if (elapsed >= collection_duration_) {
            timer_->cancel();
            compute_and_report();
            rclcpp::shutdown();
        }
    }

    void compute_and_report() {
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "=== Collection Complete ===");
        RCLCPP_INFO(get_logger(), "Total samples collected: %zu", sample_count_);
        RCLCPP_INFO(get_logger(), "Actual sample rate: %.1f Hz",
                    sample_count_ / collection_duration_);
        RCLCPP_INFO(get_logger(), "");

        // Compute statistics for all points
        for (int i = 0; i < 36; ++i) {
            point_stats_[i].compute();
        }
        resultant_stats_.compute();

        // Print summary statistics
        print_summary();

        // Print per-point details if not quiet
        if (!quiet_) {
            print_per_point_stats();
        }

        // Always export to CSV and raw data
        export_csv();
        export_raw_data();
    }

    void print_summary() {
        RCLCPP_INFO(get_logger(), "=== Summary Statistics ===");
        RCLCPP_INFO(get_logger(), "");

        // Calculate overall noise metrics
        double total_x_std = 0, total_y_std = 0, total_z_std = 0, total_mag_std = 0;
        double max_x_std = 0, max_y_std = 0, max_z_std = 0, max_mag_std = 0;
        double max_x_range = 0, max_y_range = 0, max_z_range = 0, max_mag_range = 0;

        for (int i = 0; i < 36; ++i) {
            total_x_std += point_stats_[i].x.std_dev;
            total_y_std += point_stats_[i].y.std_dev;
            total_z_std += point_stats_[i].z.std_dev;
            total_mag_std += point_stats_[i].magnitude.std_dev;

            max_x_std = std::max(max_x_std, point_stats_[i].x.std_dev);
            max_y_std = std::max(max_y_std, point_stats_[i].y.std_dev);
            max_z_std = std::max(max_z_std, point_stats_[i].z.std_dev);
            max_mag_std = std::max(max_mag_std, point_stats_[i].magnitude.std_dev);

            max_x_range = std::max(max_x_range, point_stats_[i].x.range);
            max_y_range = std::max(max_y_range, point_stats_[i].y.range);
            max_z_range = std::max(max_z_range, point_stats_[i].z.range);
            max_mag_range = std::max(max_mag_range, point_stats_[i].magnitude.range);
        }

        RCLCPP_INFO(get_logger(), "Noise Analysis (Standard Deviation):");
        RCLCPP_INFO(get_logger(), "  %-12s %10s %10s %10s %10s", "Axis", "Avg StdDev", "Max StdDev", "Max Range", "Unit");
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10s", "X (shear)",
                    total_x_std / 36, max_x_std, max_x_range, "N");
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10s", "Y (shear)",
                    total_y_std / 36, max_y_std, max_y_range, "N");
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10s", "Z (normal)",
                    total_z_std / 36, max_z_std, max_z_range, "N");
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10s", "Magnitude",
                    total_mag_std / 36, max_mag_std, max_mag_range, "N");

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Resultant Force Statistics:");
        RCLCPP_INFO(get_logger(), "  %-12s %10s %10s %10s %10s %10s", "Axis", "Mean", "StdDev", "Min", "Max", "Range");
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10.3f %10.3f", "X",
                    resultant_stats_.x.mean, resultant_stats_.x.std_dev,
                    resultant_stats_.x.min, resultant_stats_.x.max, resultant_stats_.x.range);
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10.3f %10.3f", "Y",
                    resultant_stats_.y.mean, resultant_stats_.y.std_dev,
                    resultant_stats_.y.min, resultant_stats_.y.max, resultant_stats_.y.range);
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10.3f %10.3f", "Z",
                    resultant_stats_.z.mean, resultant_stats_.z.std_dev,
                    resultant_stats_.z.min, resultant_stats_.z.max, resultant_stats_.z.range);
        RCLCPP_INFO(get_logger(), "  %-12s %10.3f %10.3f %10.3f %10.3f %10.3f", "Magnitude",
                    resultant_stats_.magnitude.mean, resultant_stats_.magnitude.std_dev,
                    resultant_stats_.magnitude.min, resultant_stats_.magnitude.max,
                    resultant_stats_.magnitude.range);

        // Find noisiest and quietest points
        int noisiest_idx = 0, quietest_idx = 0;
        double max_noise = 0, min_noise = std::numeric_limits<double>::max();
        for (int i = 0; i < 36; ++i) {
            double noise = point_stats_[i].magnitude.std_dev;
            if (noise > max_noise) {
                max_noise = noise;
                noisiest_idx = i;
            }
            if (noise < min_noise) {
                min_noise = noise;
                quietest_idx = i;
            }
        }

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Point Analysis:");
        RCLCPP_INFO(get_logger(), "  Noisiest point: #%d (row %d, col %d) - StdDev: %.3f N",
                    noisiest_idx, noisiest_idx / 6, noisiest_idx % 6, max_noise);
        RCLCPP_INFO(get_logger(), "  Quietest point: #%d (row %d, col %d) - StdDev: %.3f N",
                    quietest_idx, quietest_idx / 6, quietest_idx % 6, min_noise);

        // Signal-to-Noise Ratio estimation
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Recommended Filter Settings:");
        double avg_noise = total_mag_std / 36;
        RCLCPP_INFO(get_logger(), "  force_threshold_show: %.1f (3x avg noise)", avg_noise * 3);
        RCLCPP_INFO(get_logger(), "  force_threshold_hide: %.1f (1.5x avg noise)", avg_noise * 1.5);
        RCLCPP_INFO(get_logger(), "  smoothing_alpha: %.2f (based on noise level)",
                    std::min(0.5, std::max(0.1, 1.0 - avg_noise / 10.0)));
    }

    void print_per_point_stats() {
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "=== Per-Point Statistics ===");
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "%5s %8s %8s %8s %8s %8s %8s %8s %8s",
                    "Point", "X_mean", "X_std", "Y_mean", "Y_std", "Z_mean", "Z_std", "Mag_mean", "Mag_std");
        RCLCPP_INFO(get_logger(), "%5s %8s %8s %8s %8s %8s %8s %8s %8s",
                    "-----", "------", "-----", "------", "-----", "------", "-----", "--------", "-------");

        for (int i = 0; i < 36; ++i) {
            RCLCPP_INFO(get_logger(), "%5d %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f",
                        i,
                        point_stats_[i].x.mean, point_stats_[i].x.std_dev,
                        point_stats_[i].y.mean, point_stats_[i].y.std_dev,
                        point_stats_[i].z.mean, point_stats_[i].z.std_dev,
                        point_stats_[i].magnitude.mean, point_stats_[i].magnitude.std_dev);
        }
    }

    void export_csv() {
        std::string stats_file = output_dir_ + "/stats_" + timestamp_str_ + ".csv";
        std::ofstream file(stats_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", stats_file.c_str());
            return;
        }

        // Header
        file << "point,x_mean,x_std,x_min,x_max,x_range,"
             << "y_mean,y_std,y_min,y_max,y_range,"
             << "z_mean,z_std,z_min,z_max,z_range,"
             << "mag_mean,mag_std,mag_min,mag_max,mag_range\n";

        // Data for each point
        for (int i = 0; i < 36; ++i) {
            file << i << ","
                 << point_stats_[i].x.mean << "," << point_stats_[i].x.std_dev << ","
                 << point_stats_[i].x.min << "," << point_stats_[i].x.max << ","
                 << point_stats_[i].x.range << ","
                 << point_stats_[i].y.mean << "," << point_stats_[i].y.std_dev << ","
                 << point_stats_[i].y.min << "," << point_stats_[i].y.max << ","
                 << point_stats_[i].y.range << ","
                 << point_stats_[i].z.mean << "," << point_stats_[i].z.std_dev << ","
                 << point_stats_[i].z.min << "," << point_stats_[i].z.max << ","
                 << point_stats_[i].z.range << ","
                 << point_stats_[i].magnitude.mean << "," << point_stats_[i].magnitude.std_dev << ","
                 << point_stats_[i].magnitude.min << "," << point_stats_[i].magnitude.max << ","
                 << point_stats_[i].magnitude.range << "\n";
        }

        // Resultant force
        file << "resultant,"
             << resultant_stats_.x.mean << "," << resultant_stats_.x.std_dev << ","
             << resultant_stats_.x.min << "," << resultant_stats_.x.max << ","
             << resultant_stats_.x.range << ","
             << resultant_stats_.y.mean << "," << resultant_stats_.y.std_dev << ","
             << resultant_stats_.y.min << "," << resultant_stats_.y.max << ","
             << resultant_stats_.y.range << ","
             << resultant_stats_.z.mean << "," << resultant_stats_.z.std_dev << ","
             << resultant_stats_.z.min << "," << resultant_stats_.z.max << ","
             << resultant_stats_.z.range << ","
             << resultant_stats_.magnitude.mean << "," << resultant_stats_.magnitude.std_dev << ","
             << resultant_stats_.magnitude.min << "," << resultant_stats_.magnitude.max << ","
             << resultant_stats_.magnitude.range << "\n";

        file.close();
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "Statistics exported to: %s", stats_file.c_str());
    }

    void export_raw_data() {
        std::string raw_file = output_dir_ + "/raw_" + timestamp_str_ + ".csv";
        std::ofstream file(raw_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open raw data file: %s", raw_file.c_str());
            return;
        }

        // Header: sample_idx, point0_x, point0_y, point0_z, ..., point35_x, point35_y, point35_z, resultant_x, resultant_y, resultant_z
        file << "sample";
        for (int i = 0; i < 36; ++i) {
            file << ",p" << i << "_x,p" << i << "_y,p" << i << "_z";
        }
        file << ",resultant_x,resultant_y,resultant_z\n";

        // Write all samples
        size_t num_samples = point_stats_[0].x_samples.size();
        for (size_t s = 0; s < num_samples; ++s) {
            file << s;
            for (int i = 0; i < 36; ++i) {
                file << "," << point_stats_[i].x_samples[s]
                     << "," << point_stats_[i].y_samples[s]
                     << "," << point_stats_[i].z_samples[s];
            }
            file << "," << resultant_stats_.x_samples[s]
                 << "," << resultant_stats_.y_samples[s]
                 << "," << resultant_stats_.z_samples[s];
            file << "\n";
        }

        file.close();
        RCLCPP_INFO(get_logger(), "Raw data exported to: %s", raw_file.c_str());
    }

    // Handler
    wujihandcpp::protocol::Handler handler_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Configuration
    double collection_duration_;
    double sample_rate_;
    std::string output_dir_;
    std::string timestamp_str_;
    bool quiet_;
    size_t expected_samples_;

    // State
    bool collecting_ = false;
    size_t sample_count_ = 0;
    rclcpp::Time start_time_;
    int last_progress_sec_ = 0;

    // Statistics storage
    std::array<PointStats, 36> point_stats_;
    PointStats resultant_stats_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TactileStatsNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("tactile_stats"), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

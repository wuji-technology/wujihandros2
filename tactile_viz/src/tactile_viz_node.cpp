/**
 * @file tactile_viz_node.cpp
 * @brief ROS2 node for 3D visualization of tactile sensor data
 *
 * This node reads tactile data from wujihandcpp and publishes MarkerArray
 * for visualization in RViz. Each tactile point is displayed as:
 * - A sphere at the point position
 * - An arrow showing the force vector direction and magnitude
 */

#include <cmath>
#include <array>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <wujihandcpp/protocol/handler.hpp>

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class TactileVizNode : public rclcpp::Node {
public:
    TactileVizNode()
        : Node("tactile_viz_node")
        , handler_(0x0483, -1, nullptr, 64, 0)  // Connect to any wujihand device
    {
        // Declare parameters
        declare_parameter("frame_id", "base_link");  // Match URDF base frame
        declare_parameter("raw_frame_id", "raw_base_link");  // Separate frame for raw data
        declare_parameter("publish_rate", 100.0);
        declare_parameter("position_scale", 10.0);  // Scale factor for point positions (10x to match URDF)
        declare_parameter("arrow_scale", 0.01);    // Scale factor for force to arrow length
        declare_parameter("arrow_shaft_diameter", 0.02);   // Arrow shaft diameter (20mm)
        declare_parameter("arrow_head_diameter", 0.04);    // Arrow head diameter (40mm)
        declare_parameter("arrow_z_offset", 0.02);        // Z offset for arrows to avoid mesh overlap (20mm)
        declare_parameter("smoothing_alpha", 0.3); // Exponential smoothing factor (0-1, lower = smoother)
        declare_parameter("force_threshold_show", 5.0);  // Force threshold to show arrow
        declare_parameter("force_threshold_hide", 2.0);  // Force threshold to hide arrow (hysteresis)
        declare_parameter("max_slope", 500.0);  // Max allowed slope (N/s), reject data with higher rate of change
        declare_parameter("slope_filter_enabled", true);  // Enable/disable slope-based outlier filtering

        // Get parameters
        frame_id_ = get_parameter("frame_id").as_string();
        raw_frame_id_ = get_parameter("raw_frame_id").as_string();
        double publish_rate = get_parameter("publish_rate").as_double();
        position_scale_ = get_parameter("position_scale").as_double();
        arrow_scale_ = get_parameter("arrow_scale").as_double();
        arrow_shaft_diameter_ = get_parameter("arrow_shaft_diameter").as_double();
        arrow_head_diameter_ = get_parameter("arrow_head_diameter").as_double();
        arrow_z_offset_ = get_parameter("arrow_z_offset").as_double();
        smoothing_alpha_ = get_parameter("smoothing_alpha").as_double();
        force_threshold_show_ = get_parameter("force_threshold_show").as_double();
        force_threshold_hide_ = get_parameter("force_threshold_hide").as_double();
        max_slope_ = get_parameter("max_slope").as_double();
        slope_filter_enabled_ = get_parameter("slope_filter_enabled").as_bool();

        // Initialize arrays
        for (int i = 0; i < 36; ++i) {
            smoothed_forces_[i] = {0.0, 0.0, 0.0};
            prev_raw_forces_[i] = {0.0, 0.0, 0.0};
            arrow_visible_[i] = false;
            raw_arrow_visible_[i] = false;
        }
        smoothed_resultant_ = {0.0, 0.0, 0.0};
        prev_raw_resultant_ = {0.0, 0.0, 0.0};
        resultant_visible_ = false;
        raw_resultant_visible_ = false;
        last_update_time_ = now();
        first_sample_ = true;

        // Initialize point positions from URDF definition (36_points.urdf)
        initialize_urdf_positions();

        // Create publishers for raw and filtered markers
        raw_marker_pub_ = create_publisher<MarkerArray>("/tactile_markers_raw", 100);
        filtered_marker_pub_ = create_publisher<MarkerArray>("/tactile_markers_filtered", 100);

        // Create static transform broadcaster for tactile frame
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_transform();

        // Create timer
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TactileVizNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Tactile visualization node started");
        RCLCPP_INFO(get_logger(), "  Filtered frame: %s", frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "  Raw frame: %s", raw_frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", publish_rate);
        RCLCPP_INFO(get_logger(), "  Raw markers topic: /tactile_markers_raw (frame: %s)", raw_frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "  Filtered markers topic: /tactile_markers_filtered (frame: %s)", frame_id_.c_str());
    }

private:
    void initialize_urdf_positions() {
        // Point positions from 36_points.urdf
        // Extracted from joint origins: <origin xyz="x y z" .../>
        // URDF uses point IDs 1-36, array indices are 0-35
        //
        // Row 1 (Points 1-6): Y=0, red points at fingertip base
        // Row 2 (Points 7-12): Y~=4.58mm, orange
        // Row 3 (Points 13-18): Y~=9.25mm, yellow
        // Row 4 (Points 19-24): Y~=13.89mm, green
        // Row 5 (Points 25-30): Y~=16.13mm, cyan
        // Row 6 (Points 31-36): Y~=17.92mm, blue points at fingertip

        // Row 1: Points 1-6 (Red) - Y=0
        point_positions_[0]  = {0.0806,  0.0,     0.0348};  // Point 1
        point_positions_[1]  = {0.0484,  0.0,     0.0778};  // Point 2
        point_positions_[2]  = {0.0161,  0.0,     0.0938};  // Point 3
        point_positions_[3]  = {-0.0162, 0.0,     0.0938};  // Point 4
        point_positions_[4]  = {-0.0484, 0.0,     0.0778};  // Point 5
        point_positions_[5]  = {-0.0807, 0.0,     0.0348};  // Point 6

        // Row 2: Points 7-12 (Orange) - Y~=4.58mm
        point_positions_[6]  = {0.0773,  0.0458,  0.0424};  // Point 7
        point_positions_[7]  = {0.0464,  0.0458,  0.0798};  // Point 8
        point_positions_[8]  = {0.0148,  0.0459,  0.0932};  // Point 9
        point_positions_[9]  = {-0.0165, 0.0459,  0.0926};  // Point 10
        point_positions_[10] = {-0.0478, 0.0459,  0.0780};  // Point 11
        point_positions_[11] = {-0.0791, 0.0460,  0.0394};  // Point 12

        // Row 3: Points 13-18 (Yellow) - Y~=9.25mm
        point_positions_[12] = {0.0756,  0.0925,  0.0407};  // Point 13
        point_positions_[13] = {0.0454,  0.0925,  0.0728};  // Point 14
        point_positions_[14] = {0.0152,  0.0925,  0.0850};  // Point 15
        point_positions_[15] = {-0.0150, 0.0925,  0.0851};  // Point 16
        point_positions_[16] = {-0.0451, 0.0925,  0.0732};  // Point 17
        point_positions_[17] = {-0.0753, 0.0925,  0.0414};  // Point 18

        // Row 4: Points 19-24 (Green) - Y~=13.89mm
        point_positions_[18] = {0.0649,  0.1389,  0.0367};  // Point 19
        point_positions_[19] = {0.0389,  0.1389,  0.0587};  // Point 20
        point_positions_[20] = {0.0129,  0.1389,  0.0667};  // Point 21
        point_positions_[21] = {-0.0130, 0.1389,  0.0667};  // Point 22
        point_positions_[22] = {-0.0390, 0.1389,  0.0587};  // Point 23
        point_positions_[23] = {-0.0651, 0.1388,  0.0357};  // Point 24

        // Row 5: Points 25-30 (Cyan) - Y~=16.13mm
        point_positions_[24] = {0.0486,  0.1614,  0.0365};  // Point 25
        point_positions_[25] = {0.0291,  0.1613,  0.0475};  // Point 26
        point_positions_[26] = {0.0096,  0.1613,  0.0525};  // Point 27
        point_positions_[27] = {-0.0099, 0.1613,  0.0525};  // Point 28
        point_positions_[28] = {-0.0293, 0.1613,  0.0475};  // Point 29
        point_positions_[29] = {-0.0488, 0.1612,  0.0365};  // Point 30

        // Row 6: Points 31-36 (Blue) - Y~=17.92mm (fingertip)
        point_positions_[30] = {0.0266,  0.1792,  0.0311};  // Point 31
        point_positions_[31] = {0.0158,  0.1792,  0.0341};  // Point 32
        point_positions_[32] = {0.0051,  0.1792,  0.0361};  // Point 33
        point_positions_[33] = {-0.0056, 0.1792,  0.0361};  // Point 34
        point_positions_[34] = {-0.0163, 0.1791,  0.0341};  // Point 35
        point_positions_[35] = {-0.0270, 0.1791,  0.0311};  // Point 36

        // Apply position scale to all points
        for (int i = 0; i < 36; ++i) {
            point_positions_[i].x *= position_scale_;
            point_positions_[i].y *= position_scale_;
            point_positions_[i].z *= position_scale_;
        }
    }

    void publish_static_transform() {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // Transform for filtered data: world -> base_link (at origin)
        geometry_msgs::msg::TransformStamped filtered_tf;
        filtered_tf.header.stamp = now();
        filtered_tf.header.frame_id = "world";
        filtered_tf.child_frame_id = frame_id_;
        filtered_tf.transform.translation.x = 0.0;
        filtered_tf.transform.translation.y = 0.0;
        filtered_tf.transform.translation.z = 0.0;
        filtered_tf.transform.rotation.x = 0.0;
        filtered_tf.transform.rotation.y = 0.0;
        filtered_tf.transform.rotation.z = 0.0;
        filtered_tf.transform.rotation.w = 1.0;
        transforms.push_back(filtered_tf);

        // Transform for raw data: world -> raw_base_link (offset to the right)
        geometry_msgs::msg::TransformStamped raw_tf;
        raw_tf.header.stamp = now();
        raw_tf.header.frame_id = "world";
        raw_tf.child_frame_id = raw_frame_id_;
        raw_tf.transform.translation.x = 3.0;  // Offset 3 meters to the right
        raw_tf.transform.translation.y = 0.0;
        raw_tf.transform.translation.z = 0.0;
        raw_tf.transform.rotation.x = 0.0;
        raw_tf.transform.rotation.y = 0.0;
        raw_tf.transform.rotation.z = 0.0;
        raw_tf.transform.rotation.w = 1.0;
        transforms.push_back(raw_tf);

        tf_broadcaster_->sendTransform(transforms);
    }

    void timer_callback() {
        wujihandcpp::protocol::TactileData data;
        if (!handler_.get_tactile_data(data)) {
            // No data available yet
            return;
        }

        MarkerArray raw_markers;
        MarkerArray filtered_markers;
        auto stamp = now();

        // Calculate time delta for slope calculation
        double dt = (stamp - last_update_time_).seconds();
        if (dt <= 0) dt = 0.1;  // Default to 10Hz if time is invalid
        last_update_time_ = stamp;

        // Create markers for each tactile point
        for (int i = 0; i < 36; ++i) {
            // Get raw values (before any filtering)
            double orig_raw_x = data.points[i].x;
            double orig_raw_y = data.points[i].y;
            double orig_raw_z = data.points[i].z;

            // Values after slope filter
            double raw_x = orig_raw_x;
            double raw_y = orig_raw_y;
            double raw_z = orig_raw_z;

            // Apply slope-based outlier filter
            if (slope_filter_enabled_ && !first_sample_) {
                raw_x = filter_by_slope(raw_x, prev_raw_forces_[i].x, dt);
                raw_y = filter_by_slope(raw_y, prev_raw_forces_[i].y, dt);
                raw_z = filter_by_slope(raw_z, prev_raw_forces_[i].z, dt);
            }

            // Store current as previous for next iteration
            prev_raw_forces_[i].x = raw_x;
            prev_raw_forces_[i].y = raw_y;
            prev_raw_forces_[i].z = raw_z;

            // Apply exponential moving average smoothing
            smoothed_forces_[i].x = smoothing_alpha_ * raw_x +
                                    (1.0 - smoothing_alpha_) * smoothed_forces_[i].x;
            smoothed_forces_[i].y = smoothing_alpha_ * raw_y +
                                    (1.0 - smoothing_alpha_) * smoothed_forces_[i].y;
            smoothed_forces_[i].z = smoothing_alpha_ * raw_z +
                                    (1.0 - smoothing_alpha_) * smoothed_forces_[i].z;

            // ===== RAW MARKERS (before filtering) =====
            double raw_magnitude = std::sqrt(orig_raw_x*orig_raw_x + orig_raw_y*orig_raw_y + orig_raw_z*orig_raw_z);

            // Color based on force magnitude (blue -> green -> red)
            double raw_normalized = std::min(raw_magnitude / 500.0, 1.0);
            float raw_color_r = static_cast<float>(raw_normalized);
            float raw_color_g = static_cast<float>(1.0 - std::abs(raw_normalized - 0.5) * 2);
            float raw_color_b = static_cast<float>(1.0 - raw_normalized);

            Marker raw_arrow;
            raw_arrow.header.frame_id = raw_frame_id_;
            raw_arrow.header.stamp = stamp;
            raw_arrow.ns = "raw_tactile_forces";
            raw_arrow.id = i;
            raw_arrow.type = Marker::ARROW;

            // Hysteresis logic for raw arrows
            if (!raw_arrow_visible_[i] && raw_magnitude > force_threshold_show_) {
                raw_arrow_visible_[i] = true;
            } else if (raw_arrow_visible_[i] && raw_magnitude < force_threshold_hide_) {
                raw_arrow_visible_[i] = false;
            }

            if (raw_arrow_visible_[i] && raw_magnitude > 0.001) {
                raw_arrow.action = Marker::ADD;

                geometry_msgs::msg::Point start, end;
                start.x = point_positions_[i].x;
                start.y = point_positions_[i].y;
                start.z = point_positions_[i].z + arrow_z_offset_;

                double arrow_length = raw_magnitude * arrow_scale_;
                end.x = start.x + (orig_raw_x / raw_magnitude) * arrow_length;
                end.y = start.y + (orig_raw_y / raw_magnitude) * arrow_length;
                end.z = start.z + (orig_raw_z / raw_magnitude) * arrow_length;

                raw_arrow.points.push_back(start);
                raw_arrow.points.push_back(end);

                raw_arrow.scale.x = arrow_shaft_diameter_;
                raw_arrow.scale.y = arrow_head_diameter_;
                raw_arrow.scale.z = arrow_head_diameter_;

                raw_arrow.color.r = raw_color_r;
                raw_arrow.color.g = raw_color_g;
                raw_arrow.color.b = raw_color_b;
                raw_arrow.color.a = 1.0f;
            } else {
                raw_arrow.action = Marker::DELETE;
            }
            raw_markers.markers.push_back(raw_arrow);

            // ===== FILTERED MARKERS (after smoothing) =====
            double fx = smoothed_forces_[i].x;
            double fy = smoothed_forces_[i].y;
            double fz = smoothed_forces_[i].z;
            double force_magnitude = std::sqrt(fx*fx + fy*fy + fz*fz);

            // Color based on force magnitude (blue -> green -> red)
            double normalized_force = std::min(force_magnitude / 500.0, 1.0);
            float color_r = static_cast<float>(normalized_force);
            float color_g = static_cast<float>(1.0 - std::abs(normalized_force - 0.5) * 2);
            float color_b = static_cast<float>(1.0 - normalized_force);

            Marker arrow;
            arrow.header.frame_id = frame_id_;
            arrow.header.stamp = stamp;
            arrow.ns = "filtered_tactile_forces";
            arrow.id = i;
            arrow.type = Marker::ARROW;

            // Hysteresis logic for filtered arrows
            if (!arrow_visible_[i] && force_magnitude > force_threshold_show_) {
                arrow_visible_[i] = true;
            } else if (arrow_visible_[i] && force_magnitude < force_threshold_hide_) {
                arrow_visible_[i] = false;
            }

            if (arrow_visible_[i] && force_magnitude > 0.001) {
                arrow.action = Marker::ADD;

                geometry_msgs::msg::Point start, end;
                start.x = point_positions_[i].x;
                start.y = point_positions_[i].y;
                start.z = point_positions_[i].z + arrow_z_offset_;

                double arrow_length = force_magnitude * arrow_scale_;
                end.x = start.x + (fx / force_magnitude) * arrow_length;
                end.y = start.y + (fy / force_magnitude) * arrow_length;
                end.z = start.z + (fz / force_magnitude) * arrow_length;

                arrow.points.push_back(start);
                arrow.points.push_back(end);

                arrow.scale.x = arrow_shaft_diameter_;
                arrow.scale.y = arrow_head_diameter_;
                arrow.scale.z = arrow_head_diameter_;

                arrow.color.r = color_r;
                arrow.color.g = color_g;
                arrow.color.b = color_b;
                arrow.color.a = 1.0f;
            } else {
                arrow.action = Marker::DELETE;
            }
            filtered_markers.markers.push_back(arrow);
        }

        // ===== RAW RESULTANT FORCE =====
        double orig_raw_rfx = data.resultant_force.x;
        double orig_raw_rfy = data.resultant_force.y;
        double orig_raw_rfz = data.resultant_force.z;

        Marker raw_resultant_arrow;
        raw_resultant_arrow.header.frame_id = raw_frame_id_;
        raw_resultant_arrow.header.stamp = stamp;
        raw_resultant_arrow.ns = "raw_resultant_force";
        raw_resultant_arrow.id = 0;
        raw_resultant_arrow.type = Marker::ARROW;

        double raw_rf_magnitude = std::sqrt(orig_raw_rfx*orig_raw_rfx + orig_raw_rfy*orig_raw_rfy + orig_raw_rfz*orig_raw_rfz);

        // Hysteresis for raw resultant force
        if (!raw_resultant_visible_ && raw_rf_magnitude > force_threshold_show_) {
            raw_resultant_visible_ = true;
        } else if (raw_resultant_visible_ && raw_rf_magnitude < force_threshold_hide_) {
            raw_resultant_visible_ = false;
        }

        if (raw_resultant_visible_ && raw_rf_magnitude > 0.001) {
            raw_resultant_arrow.action = Marker::ADD;

            geometry_msgs::msg::Point start, end;
            start.x = point_positions_[17].x;
            start.y = point_positions_[17].y;
            start.z = point_positions_[17].z + arrow_z_offset_;

            double arrow_length = raw_rf_magnitude * arrow_scale_ * 2;
            end.x = start.x + (orig_raw_rfx / raw_rf_magnitude) * arrow_length;
            end.y = start.y + (orig_raw_rfy / raw_rf_magnitude) * arrow_length;
            end.z = start.z + (orig_raw_rfz / raw_rf_magnitude) * arrow_length;

            raw_resultant_arrow.points.push_back(start);
            raw_resultant_arrow.points.push_back(end);

            raw_resultant_arrow.scale.x = arrow_shaft_diameter_ * 2;
            raw_resultant_arrow.scale.y = arrow_head_diameter_ * 2;
            raw_resultant_arrow.scale.z = arrow_head_diameter_ * 2;

            // Orange for raw resultant force
            raw_resultant_arrow.color.r = 1.0f;
            raw_resultant_arrow.color.g = 0.5f;
            raw_resultant_arrow.color.b = 0.0f;
            raw_resultant_arrow.color.a = 1.0f;
        } else {
            raw_resultant_arrow.action = Marker::DELETE;
        }
        raw_markers.markers.push_back(raw_resultant_arrow);

        // ===== FILTERED RESULTANT FORCE =====
        double raw_rfx = orig_raw_rfx;
        double raw_rfy = orig_raw_rfy;
        double raw_rfz = orig_raw_rfz;

        // Apply slope filter to resultant
        if (slope_filter_enabled_ && !first_sample_) {
            raw_rfx = filter_by_slope(raw_rfx, prev_raw_resultant_.x, dt);
            raw_rfy = filter_by_slope(raw_rfy, prev_raw_resultant_.y, dt);
            raw_rfz = filter_by_slope(raw_rfz, prev_raw_resultant_.z, dt);
        }

        prev_raw_resultant_.x = raw_rfx;
        prev_raw_resultant_.y = raw_rfy;
        prev_raw_resultant_.z = raw_rfz;

        // Apply smoothing to resultant force
        smoothed_resultant_.x = smoothing_alpha_ * raw_rfx +
                                (1.0 - smoothing_alpha_) * smoothed_resultant_.x;
        smoothed_resultant_.y = smoothing_alpha_ * raw_rfy +
                                (1.0 - smoothing_alpha_) * smoothed_resultant_.y;
        smoothed_resultant_.z = smoothing_alpha_ * raw_rfz +
                                (1.0 - smoothing_alpha_) * smoothed_resultant_.z;

        Marker resultant_arrow;
        resultant_arrow.header.frame_id = frame_id_;
        resultant_arrow.header.stamp = stamp;
        resultant_arrow.ns = "filtered_resultant_force";
        resultant_arrow.id = 0;
        resultant_arrow.type = Marker::ARROW;

        double rfx = smoothed_resultant_.x;
        double rfy = smoothed_resultant_.y;
        double rfz = smoothed_resultant_.z;
        double rf_magnitude = std::sqrt(rfx*rfx + rfy*rfy + rfz*rfz);

        // Hysteresis for resultant force
        if (!resultant_visible_ && rf_magnitude > force_threshold_show_) {
            resultant_visible_ = true;
        } else if (resultant_visible_ && rf_magnitude < force_threshold_hide_) {
            resultant_visible_ = false;
        }

        if (resultant_visible_ && rf_magnitude > 0.001) {
            resultant_arrow.action = Marker::ADD;

            geometry_msgs::msg::Point start, end;
            start.x = point_positions_[17].x;
            start.y = point_positions_[17].y;
            start.z = point_positions_[17].z + arrow_z_offset_;

            double arrow_length = rf_magnitude * arrow_scale_ * 2;
            end.x = start.x + (rfx / rf_magnitude) * arrow_length;
            end.y = start.y + (rfy / rf_magnitude) * arrow_length;
            end.z = start.z + (rfz / rf_magnitude) * arrow_length;

            resultant_arrow.points.push_back(start);
            resultant_arrow.points.push_back(end);

            resultant_arrow.scale.x = arrow_shaft_diameter_ * 2;
            resultant_arrow.scale.y = arrow_head_diameter_ * 2;
            resultant_arrow.scale.z = arrow_head_diameter_ * 2;

            // Yellow for filtered resultant force
            resultant_arrow.color.r = 1.0f;
            resultant_arrow.color.g = 1.0f;
            resultant_arrow.color.b = 0.0f;
            resultant_arrow.color.a = 1.0f;
        } else {
            resultant_arrow.action = Marker::DELETE;
        }
        filtered_markers.markers.push_back(resultant_arrow);

        // Mark first sample as processed
        first_sample_ = false;

        // Publish both marker arrays
        raw_marker_pub_->publish(raw_markers);
        filtered_marker_pub_->publish(filtered_markers);
    }

    /**
     * @brief Filter value by slope - reject outliers with excessive rate of change
     * @param current Current raw value
     * @param previous Previous raw value
     * @param dt Time delta in seconds
     * @return Filtered value (current if valid, clamped value if outlier)
     */
    double filter_by_slope(double current, double previous, double dt) {
        double slope = (current - previous) / dt;
        double max_change = max_slope_ * dt;

        if (std::abs(slope) > max_slope_) {
            // Slope exceeds threshold, clamp the change
            if (slope > 0) {
                return previous + max_change;
            } else {
                return previous - max_change;
            }
        }
        return current;
    }

    // wujihandcpp handler
    wujihandcpp::protocol::Handler handler_;

    // ROS2 components
    rclcpp::Publisher<MarkerArray>::SharedPtr raw_marker_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr filtered_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // Configuration
    std::string frame_id_;
    std::string raw_frame_id_;
    double position_scale_;
    double arrow_scale_;
    double arrow_shaft_diameter_;
    double arrow_head_diameter_;
    double arrow_z_offset_;
    double smoothing_alpha_;
    double force_threshold_show_;
    double force_threshold_hide_;
    double max_slope_;
    bool slope_filter_enabled_;

    // 36 point positions (6x6 grid)
    struct Position { double x, y, z; };
    std::array<Position, 36> point_positions_;

    // Smoothed force values for filtering
    struct Force { double x, y, z; };
    std::array<Force, 36> smoothed_forces_;
    Force smoothed_resultant_;

    // Previous raw values for slope calculation
    std::array<Force, 36> prev_raw_forces_;
    Force prev_raw_resultant_;

    // Hysteresis state for arrow visibility
    std::array<bool, 36> arrow_visible_;      // Filtered arrows
    std::array<bool, 36> raw_arrow_visible_;  // Raw arrows
    bool resultant_visible_;
    bool raw_resultant_visible_;

    // Time tracking for slope calculation
    rclcpp::Time last_update_time_;
    bool first_sample_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TactileVizNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("tactile_viz"), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

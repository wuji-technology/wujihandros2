// Copyright 2025 Wuji Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Enumerate connected WujiHand serial numbers, one per line on stdout.
//
// Uses wujihandcpp's USB enumeration, which only opens devices to read the
// serial-number descriptor (no libusb_claim_interface), so it never collides
// with a running driver or another enumeration. The dual-hand launch shells
// out to this tool to discover devices, then starts one driver per serial
// number; connecting by serial only claims that one device, so two drivers
// never race on libusb claim (LIBUSB_ERROR_BUSY).

#include <cstdio>
#include <exception>
#include <wujihandcpp/transport/usb_enumerate.hpp>

int main() {
  // Default WujiHand USB identifiers (match wujihandcpp::device::Hand defaults).
  constexpr uint16_t kUsbVid = 0x0483;
  constexpr int32_t kUsbPid = 0x2000;

  try {
    const auto serial_numbers =
        wujihandcpp::transport::list_matching_serial_numbers(kUsbVid, kUsbPid);
    for (const auto& sn : serial_numbers) {
      std::printf("%s\n", sn.c_str());
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to list WujiHand serial numbers: %s\n", e.what());
    return 1;
  }
  return 0;
}

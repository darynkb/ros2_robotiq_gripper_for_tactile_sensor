// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>

#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>

#include "command_line_utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 1;
constexpr auto kSlaveAddress = 0x09;

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;



int getKey() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);            // get old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          // disable buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);   // apply new settings
    ch = getchar();                            // read first character

    if (ch == 27 && getchar() == 91) {         // ESC + '['
        ch = getchar();                        // actual arrow code
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   // restore old settings
    return ch;
}



int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  double timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stod(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave-address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                      Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE                  Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE                   Set the read/write timeout (default " << kTimeout << "ms)\n"
              << "  --slave-address VALUE             Set the slave address (default " << kSlaveAddress << ")\n"
              << "  -h                                Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  try
  {
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("grasp_publisher_node");
    auto start_recording_pub = ros_node->create_publisher<std_msgs::msg::String>("/start_recording", 10);

    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout)));

    auto driver = std::make_unique<DefaultDriver>(std::move(serial));
    driver->set_slave_address(slave_address);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeout: " << timeout << "s" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;

    const bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The gripper is not connected" << std::endl;
      return 1;
    }
    std::cout << "The gripper is connected." << std::endl;
    
    // The gripper position in decimal; Change as needed
    int touch_value = 163;
    int bend_value = 176;
    int third_press = 178;
    int num_trials;
    std::string folder_name;

    std::cout << "Enter the folder name: ";
    std::cin >> folder_name;
    
    do {
        std::cout << "Enter number of trials: ";
        std::cin >> num_trials;

        if (std::cin.fail() || num_trials <= 0) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
            std::cout << "Invalid input! Please enter a positive integer.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            break;
        }
    } while (true);
    

    uint8_t touch_pos = static_cast<uint8_t>(std::min(touch_value, 255));
    uint8_t bend_pos = static_cast<uint8_t>(std::min(bend_value, 255));
    std::cout << "Decreasing gripper speed..." << std::endl;
    driver->set_speed(0x01);

    std_msgs::msg::String start_msg;

    std::mutex driver_mutex;
    std::vector<std::pair<uint64_t, int>> gripper_samples;
    std::atomic<bool> recording_gripper{false};
    std::thread gripper_thread;

    // Helper to get current time in microseconds
    auto now_us = []() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    // Function to sample gripper position
    auto gripper_sampler = [&](DefaultDriver* driver) {
        while (recording_gripper) {
            int pos;
            {
                std::lock_guard<std::mutex> lock(driver_mutex);
                pos = static_cast<int>(driver->get_gripper_position());
            }
            gripper_samples.emplace_back(now_us(), pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
        }
    };

    std::cout << "Closing the gripper to initial position..." << std::endl;
    driver->set_gripper_position(0x64);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (int i = 0; i < num_trials; i++) {
      std::cout << "Trial #" << i+1 << std::endl;
      start_msg.data = "calibrate";
      start_recording_pub->publish(start_msg);
      rclcpp::spin_some(ros_node); // Ensure the message is sent
      std::cout << "Published 'calibrate' to /start_recording" << std::endl;
      // delay for 3 seconds to allow calibration
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Start recording just before gripper movement
      start_msg.data = "start";
      start_recording_pub->publish(start_msg);
      rclcpp::spin_some(ros_node); // Ensure the message is sent
      std::cout << "Published 'start' to /start_recording" << std::endl;

      // Start gripper position recording
      gripper_samples.clear();
      recording_gripper = true;
      gripper_thread = std::thread(gripper_sampler, driver.get());

      // delay for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      std::cout << "Closing the gripper to touch..." << std::endl;
      {
        std::lock_guard<std::mutex> lock(driver_mutex);
        driver->set_gripper_position(touch_pos);
      }
      while (true)
      {
        // auto start = std::chrono::high_resolution_clock::now();
        // int gripper_pos;
        // {
        //   std::lock_guard<std::mutex> lock(driver_mutex);
        //   gripper_pos = static_cast<int>(driver->get_gripper_position());
        // }
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> elapsed = end - start;
        // std::cout << "Gripper Position: " << gripper_pos << " (retrieved in " << elapsed.count() << " ms)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bool moving;
        {
          std::lock_guard<std::mutex> lock(driver_mutex);
          moving = driver->gripper_is_moving();
        }
        if (!moving) break;
      }

      // delay for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      std::cout << "Closing the gripper to cilia bend..." << std::endl;
      {
        std::lock_guard<std::mutex> lock(driver_mutex);
        driver->set_gripper_position(bend_pos);
      }
      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bool moving;
        {
          std::lock_guard<std::mutex> lock(driver_mutex);
          moving = driver->gripper_is_moving();
        }
        if (!moving) break;
      }

      // delay for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      std::cout << "Opening gripper to touch..." << std::endl;
      {
        std::lock_guard<std::mutex> lock(driver_mutex);
        driver->set_gripper_position(touch_pos);
      }
      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bool moving;
        {
          std::lock_guard<std::mutex> lock(driver_mutex);
          moving = driver->gripper_is_moving();
        }
        if (!moving) break;
      }

      // delay for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      std::cout << "Opening gripper..." << std::endl;
      {
        std::lock_guard<std::mutex> lock(driver_mutex);
        driver->set_gripper_position(0x64);
      }
      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bool moving;
        {
          std::lock_guard<std::mutex> lock(driver_mutex);
          moving = driver->gripper_is_moving();
        }
        if (!moving) break;
      }

      // delay for 1 second
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Stop recording after all gripper movements
      start_msg.data = "stop";
      start_recording_pub->publish(start_msg);
      rclcpp::spin_some(ros_node); // Ensure the message is sent
      std::cout << "Published 'stop' to /start_recording" << std::endl;

      // Stop gripper position recording
      recording_gripper = false;
      if (gripper_thread.joinable()) gripper_thread.join();

      // Save gripper positions to CSV
      std::ofstream csv("/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data/" + folder_name + "/gripper_positions_trial_" + std::to_string(i+1) + ".csv");
      csv << "timestamp_us,position\n";
      for (const auto& [ts, pos] : gripper_samples) {
        csv << ts << "," << pos << "\n";
      }
      csv.close();

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper..." << std::endl;
    driver->set_gripper_position(0x00);
    while (driver->gripper_is_moving())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    

    // std::cout << "Decreasing gripper speed..." << std::endl;
    // driver->set_speed(0x0F);

    // std::cout << "Closing gripper...\n";
    // driver->set_gripper_position(0xFF);
    // while (driver->gripper_is_moving())
    // {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }

    // std::cout << "Increasing gripper speed..." << std::endl;
    // driver->set_speed(0xFF);

    // std::cout << "Opening gripper..." << std::endl;
    // driver->set_gripper_position(0x00);
    // while (driver->gripper_is_moving())
    // {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
  }
  catch (const std::exception& e)
  {
    std::cout << "Failed to communicating with the gripper: " << e.what() << std::endl;
    return 1;
  }
  std::cout << "\nExited.\n";
  rclcpp::shutdown();
  return 0;
}

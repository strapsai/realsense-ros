    /******************************************************************************
     * Software License Agreement (BSD License)
     *
     * Copyright (C) 2023, OMRON SENTECH. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without
     * modification, are permitted provided that the following conditions are met:
     *   * Redistributions of source code must retain the above copyright notice,
     *   this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above copyright
     *   notice, this list of conditions and the following disclaimer in the
     *   documentation and/or other materials provided with the distribution.
     *   * Neither the names of OMRON SENTECH nor the names of its
     *   contributors may be used to endorse or promote products derived from
     *   this software without specific prior written permission.
     *
     * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
     * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
     * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
     * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
     * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
     * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
     * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     * POSSIBILITY OF SUCH DAMAGE.
     *****************************************************************************/

    #include <rclcpp/rclcpp.hpp>
    #include "rclcpp_lifecycle/lifecycle_node.hpp"
    #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
    #include "../include/realsense_node_factory.h"
    using namespace realsense2_camera;
    class RealsenseLifecycleNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        RealsenseLifecycleNode(rclcpp::NodeOptions options) : LifecycleNode("realsense_lifecycle_node")
        {
            RCLCPP_INFO(get_logger(),"Hello, world init ROS ");

            //sleep for 30 seconds
            // std::this_thread::sleep_for(std::chrono::seconds(30));
            // printf("Hello, world init!\n");
            realsense_camera_node_ = std::make_shared<realsense2_camera::RealSenseNodeFactory>(options);
            // set_parameters();
        }

        realsense2_camera::RealSenseNodeFactory::SharedPtr realsense_camera_node_;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_INFO(get_logger(), "Configuring...");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_INFO(get_logger(), "Activating...");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_INFO(get_logger(), "Deactivating...");
            // optional_publisher_->
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_INFO(get_logger(), "Cleaning up...");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_INFO(get_logger(), "Shutting down...");
            rclcpp::shutdown(); // Exit the node cleanly
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override
        {
            RCLCPP_ERROR(get_logger(), "Error occurred");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
    };


    int main(int argc, char **argv)
    {
    printf("Hello, world!\n");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<RealsenseLifecycleNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Wait for SIGINT or shutdown event
    rclcpp::on_shutdown([&executor]() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executor stopping...");
        executor.cancel();  // Stops executor spin
    });

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
    }

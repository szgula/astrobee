/* Copyright (c) 2021, Szymon Gula
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/rendering/RenderEngine.hh"
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <thread>

namespace gazebo {
class ModelPrintVisual : public VisualPlugin {
 public:
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) {

    ROS_WARN("[ModelPrintVisual] Initializing");

    if (!_visual || !_sdf) {
        ROS_WARN("[ModelPrintVisual] Invalid visual or SDF element.");
        return;
    }
    this->visual = _visual;


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectPreRender(
        std::bind(&ModelPrintVisual::OnUpdate, this));

    // Create a topic name
    std::string model_print_topicName = "/model_print";

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "model_print_visualiser_rosnode",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("model_print_visualiser_rosnode"));

    // Freq
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
        model_print_topicName, 1,
        boost::bind(&ModelPrintVisual::OnRosMsg, this, _1), ros::VoidPtr(),
        &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&ModelPrintVisual::QueueThread, this));

    ROS_WARN("Loaded ModelPrintVisual Plugin with parent...%s, Print Controll Started ",
             this->visual->GetName().c_str());
  }

  // Called by the world update start event
 public:
  void OnUpdate() {
    ROS_DEBUG("Update Tick...");
  }

 public:
  void PrintStep(const double &_mass_step) {
    this->print_mass += _mass_step;
    ROS_WARN("[Visual] New mass received ");
    ROS_WARN("[Visual] change following link: %s ", this->visual->GetName().c_str());
  }

 public:
  void OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
    this->PrintStep(_msg->data);
  }

  /// \brief ROS helper function that processes messages
 private:
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }


  // Pointer to the model
 private:
  rendering::VisualPtr visual;

  // Pointer to the update event connection
 private:
  event::ConnectionPtr updateConnection;


  // Mas of model
  double print_mass = 0;
  double cross_density = 0.65;  // kg/m: cross-area is 6.5cm, and ABS density ~1000 kg / m^3
  double print_radius = 0.02;  // 2cm
  double initial_x_displacement = 0.4;

  /// \brief A node use for ROS transport
 private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
 private:
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
 private:
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
 private:
  std::thread rosQueueThread;

  /// \brief A ROS subscriber
 private:
  physics::LinkPtr link_to_print;
  physics::LinkPtr link_to_roll;

 private:
  std::string name_link_to_print = "printed_joint";
  std::string name_link_printing_roll = "prinitng_roll";
};

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(ModelPrintVisual)
}  // namespace gazebo

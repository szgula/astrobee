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
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "std_msgs/String.h"

namespace gazebo {
class ModelPrintControl : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    if (_sdf->HasElement("name_link_to_print")) {
      this->name_link_to_print = _sdf->Get<std::string>("name_link_to_print");
    } else {
      ROS_WARN("No name_link_to_print given, setting default name %s",
             this->name_link_to_print.c_str());
    }

    if (_sdf->HasElement("name_link_printing_roll")) {
      this->name_link_printing_roll = _sdf->Get<std::string>("name_link_printing_roll");
    } else {
      ROS_WARN("No name_link_printing_roll given, setting default name %s",
             this->name_link_printing_roll.c_str());
    }

    // Store the pointer to the model
    this->model = _parent;
    this->link_to_print = this->model->GetLink(this->name_link_to_print);
    this->link_to_roll = this->model->GetLink(this->name_link_printing_roll);
    this->print_joint = this->model->GetJoint("printed_joint");



    this->world_ = this->model->GetWorld();
    this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node_->Init(world_->GetName());
    this->pub_visual_ = this->node_->Advertise<gazebo::msgs::Visual>("~/visual");



    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPrintControl::OnUpdate, this));

    // Create a topic name
    std::string model_print_topicName = "/model_print";

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "model_print_controler_rosnode",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("model_print_controler_rosnode"));

    // Freq
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
        model_print_topicName, 1,
        boost::bind(&ModelPrintControl::OnRosMsg, this, _1), ros::VoidPtr(),
        &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&ModelPrintControl::QueueThread, this));


    this->print_status = this->rosNode->advertise<std_msgs::Float32>("print_status", 10);


    ROS_WARN("Loaded ModelPrintControl Plugin with parent...%s, Print Controll Started ",
             this->model->GetName().c_str());
  }

  // Called by the world update start event
 public:
  void OnUpdate() {
    pub_visual_->Publish(visualMsg);
    std_msgs::Float32 msg;
    msg.data = this->length;
    this->print_status.publish(msg);
  }

 public:
  void PrintStep(const double &_length_step) {
    if (_length_step < 0.0f)
    {
      this->print_scale = 1.0;
      this->length = this->init_length;
      this->print_mass = 0.0001;
      ROS_WARN("reset print; scale:  %f len:  %f  mass:  %f", 
                    this->print_scale, this->length, this->print_mass);
    } 
    else {
      this->print_scale += _length_step / this->init_length;
      this->length += _length_step;
      double _mass_step = _length_step * this->cross_density;
      this->print_mass += _mass_step;
    }
    //ROS_WARN("New mass received ");
    //ROS_WARN("change following link: %s ", this->model->GetName().c_str());
    // Changing the mass
    physics::InertialPtr inertial = this->link_to_print->GetInertial();
    //ROS_WARN("current print_mass >> %f", inertial->GetMass());
    inertial->SetMass(this->print_mass);
    this->link_to_print->UpdateMass();

    double inertia_x = std::max(this->print_mass * print_radius * print_radius, 0.001);
    double inertia_y = std::max(this->print_mass * this->length * this->length / 12, 0.001);
    double inertia_z = std::max(this->print_mass * this->length * this->length / 12, 0.001);
    inertial->SetInertiaMatrix(inertia_x, inertia_y, inertia_z, 0, 0, 0);
    this->link_to_print->Update();
    

    ignition::math::Vector3d new_scale(1.0, 1.0, this->print_scale);
    std::string visual_name_ = "link_visual";

    visualMsg = this->link_to_print->GetVisualMessage(visual_name_);
    gazebo::msgs::Vector3d* scale_factor = new gazebo::msgs::Vector3d{gazebo::msgs::Convert(new_scale)};

    visualMsg.set_name(this->link_to_print->GetScopedName());
    visualMsg.set_parent_name(this->model->GetScopedName());
    visualMsg.set_allocated_scale(scale_factor);
    

    ignition::math::Pose3d visual_pos(length / 2.0 + 1.0, 0, 0, 0, 1.57075, 0);
    gazebo::msgs::Pose* pose_visual_mes = new gazebo::msgs::Pose{gazebo::msgs::Convert(visual_pos)};

    pub_visual_->Publish(visualMsg);
  

    
    double visual_orgin_x = this->initial_x_displacement + this->length / 2;
    //ROS_WARN("new mass >> %f, inertial: >> %f %f %f", this->print_mass, inertia_x, inertia_y, inertia_z);

    math::Pose relativePose = this->link_to_print->GetRelativePose();
    math::Vector3 rotEuler = relativePose.rot.GetAsEuler();
    /*ROS_WARN("relative pose >> %f %f %f rot >> %f %f %f, new pose %f, length %f", 
                    relativePose.pos.x, relativePose.pos.y, relativePose.pos.z, 
                    rotEuler.x, rotEuler.y, rotEuler.z,
                    relativePose.pos.x +  _length_step / 2, this->length);*/
    relativePose.pos.x = visual_orgin_x;
    relativePose.pos.y = 0.0;
    relativePose.pos.z = 0.0;
    math::Pose newRelativePose(visual_orgin_x, 0, 0, 0, 0, 0);
    
    this->print_joint->SetPosition(0, this->length / 2);
    this->print_joint->Update();
    this->model->Update();
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
    gazebo::transport::NodePtr node_;
    gazebo::transport::PublisherPtr pub_visual_;    
    gazebo::physics::WorldPtr world_;
    physics::ModelPtr model;
    gazebo::msgs::Visual visualMsg;
    ros::Publisher print_status;

  // Pointer to the update event connection
 private:
  event::ConnectionPtr updateConnection;


  // Mas of model
  double print_mass = 0.0;
  double print_scale = 1.0;
  double init_length = 0.01;
  double length = this->init_length;
  double cross_density = 0.65;  // kg/m: cross-area is 6.5cm, and ABS density ~1000 kg / m^3
  double print_radius = 0.02;  // 2cm
  double initial_x_displacement = 0.5;

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
  physics::JointPtr print_joint;

 private:
  std::string name_link_to_print = "printed_joint";
  std::string name_link_printing_roll = "prinitng_roll";
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPrintControl)
}  // namespace gazebo

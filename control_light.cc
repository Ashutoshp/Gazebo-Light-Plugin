#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Bool.h"
#include <ignition/math/Pose3.hh>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/ModelState.h"

namespace gazebo
{
    class ControlLight : public ModelPlugin {
        private:    
            // Pointer to the model
            physics::ModelPtr mModel;

            // Pointer to the update event connection
            event::ConnectionPtr mUpdateConnection;

            // A node use for ROS transport
            std::unique_ptr<ros::NodeHandle> mRosNode;

            // A ROS subscriber
            ros::Subscriber mRosSub;

            // A ROS callbackqueue that helps process messages
            ros::CallbackQueue mRosQueue;

            // A thread the keeps running the mRosQueue
            std::thread mRosQueueThread;

            // Whether to show the light or not
            bool mShowLight;

            // Pointer to the light in the world
            physics::LightPtr mLightPtr;

            // Name of the light
            std::string mLightName;

            // Which model to attach the light
            std::string mAttchedModel;

            // Height of the light
            double mHeightOffset;

            // ROS topic for this plugin
            std::string mRosTopic;

            // Debug message flag
            bool mDebug;

            // Debug message flag for deeper debug
            bool mDebugMore;

        public:
            ControlLight(): mModel(NULL), 
                        mUpdateConnection(NULL),
                        mShowLight(false),
                        mLightPtr (NULL),
                        mLightName("Turtlebot_headlamp"),
                        mAttchedModel("mobile_base"),
                        mHeightOffset(1),
                        mRosTopic("/toggle_headlamp"),
                        mDebug(true),
                        mDebugMore(false) {
                if (mDebug) {
                    printf("Plugin ControlLight Initialized\n");
                    printf("Show Light = %d\n", mShowLight);
                }
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // Store the pointer to the model
                this->mModel = _parent;
                
                if (mDebug) {
                    printf("Loading ControlLight Plugin\n");
                    printf("Plugin attched to model %s\n",
                            (this->mModel->GetName()).c_str());
                }
      
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ControlLight::OnUpdate, this, _1));

                // Initialize ros, if it has not already bee initialized.
                if (!ros::isInitialized()) {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "gazebo_client",
                    ros::init_options::NoSigintHandler);
                }

                // Create our ROS node. This acts in a similar manner to
                // the Gazebo node
                this->mRosNode.reset(new ros::NodeHandle("gazebo_client"));

                // Create a named topic, and subscribe to it.
                // Command line to send the message
                // rostopic pub /ground_plane_0/toggle_light std_msgs/Empty
                ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<std_msgs::Bool>(
                    "/" + this->mModel->GetName() + mRosTopic, 1,
                    boost::bind(&ControlLight::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->mRosQueue);
       
                this->mRosSub = this->mRosNode->subscribe(so);

                // Spin up the queue helper thread.
                this->mRosQueueThread =
                    std::thread(std::bind(&ControlLight::QueueThread, this));
            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & /*_info*/) {
	            if (mShowLight && mLightPtr != NULL) {
                    mLightPtr->SetWorldPose((GetRobotLocation()).Ign());
                    // TODO explore this APIs.
                    //mLightPtr->PlaceOnEntity(mAttchedModel);
	            }
            }


            // Handle an incoming message from ROS
            // TODO BUG consitency issues between GUI 
            // and command line control of light
            void OnRosMsg(const std_msgs::BoolConstPtr &_msg) {
                if (mDebug) printf("Entered ControlLight::OnRosMsg\n");
	
	            mShowLight = _msg->data;
	            physics::WorldPtr worldPtr = this->mModel->GetWorld();
            
                if (mShowLight) {
                    if (mDebug) printf("Adding Light\n");

                    sdf::SDF point;

                    // TODO make values custmizable e.g., use mLightName
		            point.SetFromString("<?xml version='1.0' ?>\
			            <sdf version='1.6'>\
  			            <!-- Light Source -->\
  			            <light name='Turtlebot_light' type='point'>\
      				            <pose>1 1 1 0 0 0</pose>\
      				            <diffuse>0.5 0.5 0.5 1</diffuse>\
      				            <specular>0.1 0.1 0.1 1</specular>\
     				            <attenuation>\
        				            <range>20</range>\
        				            <constant>0.5</constant>\
        				            <linear>0.01</linear>\
        				            <quadratic>0.001</quadratic>\
      				            </attenuation>\
      				            <cast_shadows>0</cast_shadows>\
  			            </light>\
			            </sdf>");

		            sdf::ElementPtr light = point.Root()->GetElement("light");
		            msgs::Light msg = gazebo::msgs::LightFromSDF(light);
		
		            transport::NodePtr node(new transport::Node());
		            node->Init(worldPtr->GetName());

		            transport::PublisherPtr lightPub 
			            = node->Advertise<msgs::Light>("~/factory/light");
		            lightPub->Publish(msg);
                
                    // Hack Make sure light is attached to the model.
                    sleep(5);

                    mLightPtr = worldPtr->Light(mLightName);
                    assert(mLightPtr != NULL);
                    mLightPtr->PlaceOnEntity(mAttchedModel);
                } else if (mLightPtr != NULL) {
                    if (mDebug) printf("Deleting Light\n");

                    // TODO Explore these APIs rather than deleting the model
                    // mLightPtr->ProcessMsg(msg);
                    // mLightPtr->UpdateParameters(light);
                    // TODO ideally we should not delete the model
                    // Instead tweak the diffusion parameter
                    worldPtr->RemoveModel(mLightName);
                    mLightPtr = NULL;
                } else {
                    assert(false);
                }

                if (mDebug) {
		            unsigned int light_count = worldPtr->Lights().size();
      	            printf("Light count = %d\n", light_count);
                }
            }

            /* // TODO Customize sdf string
            private std::string GetSDFForLight() {
                math::Pose pose = GetRobotLocation();
                std::string pose_x = to_string(pose.position.x);
                std::string pose_y = to_string(pose.position.y);
                std::string pose_z = to_string(pose.position.z + 1);
                std::string orient_y = to_string(pose.orientation.y);
                std::string orient_z = to_string(pose.orientation.z);
                std::string orient_w = to_string(pose.orientation.w);

                std::string pose_str = pose_x + " " + pose_y
                    + pose_z + " " + orient_y
                    + orient_z + " " + orient_w;

                std::string light_sdf = "<?xml version='1.0' ?>\
			            <sdf version='1.6'>\
  			            <!-- Light Source -->\
  			            <light name='Turtlebot_light' type='point'>\
      				            <pose>" + pose_str + "</pose>\
      				            <diffuse>0.5 0.5 0.5 1</diffuse>\
      				            <specular>0.1 0.1 0.1 1</specular>\
     				            <attenuation>\
        				            <range>20</range>\
        				            <constant>0.5</constant>\
        				            <linear>0.01</linear>\
        				            <quadratic>0.001</quadratic>\
      				            </attenuation>\
      				            <cast_shadows>0</cast_shadows>\
  			            </light>\
			            </sdf>";
                printf("pose_str = %s\n", pose_str.c_str());
            }*/

        private:
 
            // Returns the robot location.
            math::Pose GetRobotLocation(void) {
                    geometry_msgs::Pose pose;
                    ros::ServiceClient gms_c 
                            = this->mRosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
            
                    gazebo_msgs::GetModelState getmodelstate;
                    getmodelstate.request.model_name = mAttchedModel;
                    gms_c.call(getmodelstate);
	            
                    double light_height = getmodelstate.response.pose.position.z + mHeightOffset;
 
                    if (mDebugMore) { 
                        printf("p.x = %f\n", getmodelstate.response.pose.position.x);
	                    printf("p.y = %f\n", getmodelstate.response.pose.position.y);
	                    printf("p.z = %f\n", getmodelstate.response.pose.position.z);
	                    printf("o.x = %f\n", getmodelstate.response.pose.orientation.x);
	                    printf("o.y = %f\n", getmodelstate.response.pose.orientation.y);
	                    printf("o.z = %f\n", getmodelstate.response.pose.orientation.z);
	                    printf("o.w = %f\n", getmodelstate.response.pose.orientation.w);
                    }

                    math::Pose light_pose(getmodelstate.response.pose.position.x, 
                            getmodelstate.response.pose.position.y,
                            light_height,
                            getmodelstate.response.pose.orientation.x,
                            getmodelstate.response.pose.orientation.z,
                            getmodelstate.response.pose.orientation.w);

                    return light_pose;
            }
            
            /// ROS helper function that processes messages
            void QueueThread() {
                static const double timeout = 0.01;
                while (this->mRosNode->ok()) {
                    this->mRosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ControlLight)
}


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
    class ModelCar : public ModelPlugin
    {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;
            
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelCar::OnUpdate, this, _1));
            
            this->model->SetLinearVel(math::Vector3(15, 0, 0));
            
//            physics::JointPtr j = this->model->GetJoint("wheel_front_right_steer_spin");
//            j->SetForce(1, 50);
//            
//            physics::JointPtr j2 = this->model->GetJoint("wheel_front_left_steer_spin");
//            j2->SetForce(1, 50);
        }
        
        // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            // Apply a small linear velocity to the model.
//            this->model->SetLinearVel(math::Vector3(1, 0, 0));
        }
        
        // Pointer to the model
    private: physics::ModelPtr model;
        
        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    };
    
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelCar)
}

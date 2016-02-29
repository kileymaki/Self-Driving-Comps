#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
    class ModelPull : public ModelPlugin
    {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ModelPull::OnUpdate, this, _1));
        }

        // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            // Apply a small linear velocity to the model.
            // if(this->model->GetWorld()->GetIterations() > 10000 && this->model->GetWorld()->GetIterations() <= 15000){
            //     this->model->SetLinearVel(math::Vector3(4, -4, 0));
            // } else if (this->model->GetWorld()->GetIterations() > 15000) {
            //     this->model->SetLinearVel(math::Vector3(0, 0, 0));
            // } else {
            if(this->model->GetName() == "box1"){
                this->model->SetLinearVel(math::Vector3(-7, 0, 0));
            }else if(this->model->GetName() == "box2"){
                this->model->SetLinearVel(math::Vector3(11, 0, 0));
            }else if(this->model->GetName() == "box3"){
                this->model->SetLinearVel(math::Vector3(0, 11, 0));
            }
            // }
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPull)
}

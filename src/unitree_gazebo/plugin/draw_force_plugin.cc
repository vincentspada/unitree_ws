#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>
#include <geometry_msgs/msg/wrench_stamped.hpp>  // ROS 2 geometry_msgs
#include <rclcpp/rclcpp.hpp>  // ROS 2 rclcpp
#include <memory>
#include <string>

namespace gazebo
{
    class UnitreeDrawForcePlugin : public VisualPlugin
    {
    public:
        UnitreeDrawForcePlugin() : line(nullptr) {}
        ~UnitreeDrawForcePlugin()
        {
            if (this->line)
                this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
        {
            this->visual = _parent;
            this->visual_namespace = "visual/";
            if (!_sdf->HasElement("topicName"))
            {
                RCLCPP_INFO(ros::get_logger(), "Force draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            }
            else
            {
                this->topic_name = _sdf->Get<std::string>("topicName");
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
            this->line->SetMaterial("Gazebo/Purple");
            this->line->SetVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            // Initialize ROS 2 node
            rclcpp::init(0, nullptr);
            this->rosnode = std::make_shared<rclcpp::Node>("gazebo_visual", "", rclcpp::init_options::NoSigintHandler | rclcpp::init_options::AnonymousName);
            this->force_sub = this->rosnode->create_subscription<geometry_msgs::msg::WrenchStamped>(this->topic_name + "/" + "the_force", 30, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                this->GetForceCallback(*msg);
            });
            this->update_connection = event::Events::ConnectPreRender([this]() { this->OnUpdate(); });
            RCLCPP_INFO(ros::get_logger(), "Load %s Draw Force plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
        }

        void GetForceCallback(const geometry_msgs::msg::WrenchStamped &msg)
        {
            Fx = msg.wrench.force.x / 20.0;
            Fy = msg.wrench.force.y / 20.0;
            Fz = msg.wrench.force.z / 20.0;
        }

    private:
        rclcpp::Node::SharedPtr rosnode;
        std::string topic_name;
        rendering::VisualPtr visual;
        rendering::DynamicLines *line;
        std::string visual_namespace;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub;
        double Fx = 0, Fy = 0, Fz = 0;
        event::ConnectionPtr update_connection;
    };
    GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}

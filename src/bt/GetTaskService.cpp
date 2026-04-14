#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

class GetTaskService : public BT::RosServiceNode<std_srvs::srv::Trigger> {
public:
    GetTaskService(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
        : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<std::string>("manipulator_id", "ID of the assigned manipulator")
        });
    }

    bool setRequest(Request::SharedPtr& /*request*/) override
    {
        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        if (!response->success) return BT::NodeStatus::FAILURE;
        setOutput("manipulator_id", response->message);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "GetTaskService failed with error: %d", error);
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(GetTaskService, "GetTaskService");
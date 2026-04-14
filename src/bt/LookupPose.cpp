#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include <map>
#include <string>
#include <stdexcept>

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
LookupPose(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
    {
        pose_table_ = {
            {"1",  {4.5,  1.5}},
            {"2",  {4.5, -0.5}},
            {"3",  {4.5, -2.5}},
            {"A1", {1.5,  0.5}},
            {"A2", {1.5, -1.5}},
            {"B1", {-0.5,  0.5}},
            {"B2", {-0.5, -1.5}},
            {"C1", {-2.5,  0.5}},
            {"C2", {-2.5, -1.5}},
            {"D1", {-4.5,  0.5}},
            {"D2", {-4.5, -1.5}},
        };
    }
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up (e.g. '1', 'A1')"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

    BT::NodeStatus tick() override 
    {
        auto id = getInput<std::string>("location_id");
        if (!id) return BT::NodeStatus::FAILURE;
        auto it = pose_table_.find(id.value());
        if (it == pose_table_.end()) return BT::NodeStatus::FAILURE;
        setOutput("x", it->second.x);
        setOutput("y", it->second.y);
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
}

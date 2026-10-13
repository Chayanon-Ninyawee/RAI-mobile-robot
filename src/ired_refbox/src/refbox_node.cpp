#include <protobuf_comm/peer.h>

#include "generated/BeaconSignal.pb.h"
#include "generated/GameInfo.pb.h"
#include "generated/GameState.pb.h"
#include "generated/MachineInfo.pb.h"
#include "generated/MachineReport.pb.h"
#include "generated/NavigationChallenge.pb.h"
#include "generated/OrderInfo.pb.h"
#include "generated/RingInfo.pb.h"
#include "generated/RobotInfo.pb.h"

#include <boost/bind/bind.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

using namespace protobuf_comm;
using namespace boost::placeholders;

class RefboxNode : public rclcpp::Node
{
public:
    RefboxNode()
        : Node("ired_refbox") {
        refbox_host_ = declare_parameter("refbox_host", "192.168.122.210");
        crypto_key_ = declare_parameter("crypto_key", "randomkey");
        team_name_ = declare_parameter("team_name", "TEAM_CYAN");
        peer_name_ = declare_parameter("peer_name", "Women");
        team_color_ = declare_parameter("team_color", "CYAN");
        robot_number_ = declare_parameter("robot_number", 1);

        public_send_port_ = declare_parameter("public_send_port", 4444);
        public_recv_port_ = declare_parameter("public_recv_port", 4448);
        private_send_port_ = declare_parameter("private_send_port", 4446);
        private_recv_port_ = declare_parameter("private_recv_port", 4441);

        pose_topic_ = declare_parameter("pose_topic", "/amcl_pose");

        if (team_color_ != "CYAN" && team_color_ != "MAGENTA") {
            throw std::runtime_error("team_color must be CYAN or MAGENTA");
        }

        messages_pub_ = create_publisher<std_msgs::msg::String>("refbox/messages", 10);

        game_state_pub_ = create_publisher<std_msgs::msg::String>("refbox/game_state", 10);

        navigation_routes_pub_ = create_publisher<std_msgs::msg::String>("refbox/navigation_routes", 10);

        machine_report_info_pub_ = create_publisher<std_msgs::msg::String>("refbox/machine_report_info", 10);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_,
            10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                pose_x_ = msg->pose.pose.position.x;
                pose_y_ = msg->pose.pose.position.y;

                const auto &q = msg->pose.pose.orientation;

                pose_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

                pose_received_ = true;
            }
        );

        register_messages();
        create_peers();

        beacon_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { send_beacon(); });

        RCLCPP_INFO(get_logger(), "RefBox communication started. Host: %s", refbox_host_.c_str());

        RCLCPP_INFO(get_logger(), "Pose topic: %s", pose_topic_.c_str());
    }

private:
    void register_messages() {
        mr_.add_message_type<llsf_msgs::GameState>();
        mr_.add_message_type<llsf_msgs::GameInfo>();
        mr_.add_message_type<llsf_msgs::RobotInfo>();
        mr_.add_message_type<llsf_msgs::MachineInfo>();
        mr_.add_message_type<llsf_msgs::MachineReportInfo>();
        mr_.add_message_type<llsf_msgs::NavigationRoutes>();
        mr_.add_message_type<llsf_msgs::OrderInfo>();
        mr_.add_message_type<llsf_msgs::RingInfo>();
        mr_.add_message_type<llsf_msgs::BeaconSignal>();
    }

    void create_peers() {
        private_peer_ =
            std::make_unique<ProtobufBroadcastPeer>(refbox_host_, private_send_port_, private_recv_port_, &mr_, crypto_key_, "aes-128-cbc");

        public_peer_ = std::make_unique<ProtobufBroadcastPeer>(refbox_host_, public_send_port_, public_recv_port_, &mr_);

        private_peer_->signal_received().connect(boost::bind(&RefboxNode::handle_message, this, _1, _2, _3, _4));

        public_peer_->signal_received().connect(boost::bind(&RefboxNode::handle_message, this, _1, _2, _3, _4));

        private_peer_->signal_recv_error().connect(boost::bind(&RefboxNode::handle_recv_error, this, _1, _2));

        public_peer_->signal_recv_error().connect(boost::bind(&RefboxNode::handle_recv_error, this, _1, _2));
    }

    void handle_message(
        boost::asio::ip::udp::endpoint &endpoint,
        uint16_t component_id,
        uint16_t message_id,
        std::shared_ptr<google::protobuf::Message> message
    ) {
        const std::string type = message->GetTypeName();
        const std::string data = message->DebugString();

        RCLCPP_INFO(
            get_logger(),
            "Received %s (%u:%u) from %s:%u",
            type.c_str(),
            component_id,
            message_id,
            endpoint.address().to_string().c_str(),
            endpoint.port()
        );

        std_msgs::msg::String raw_msg;
        raw_msg.data = "component_id: " + std::to_string(component_id) + "\nmessage_id: " + std::to_string(message_id) + "\ntype: " + type +
                       "\ndata:\n" + data;

        messages_pub_->publish(raw_msg);

        if (auto state = std::dynamic_pointer_cast<llsf_msgs::GameState>(message)) {
            publish_debug(game_state_pub_, *state);
        }

        if (auto routes = std::dynamic_pointer_cast<llsf_msgs::NavigationRoutes>(message)) {
            publish_debug(navigation_routes_pub_, *routes);
        }

        if (auto report = std::dynamic_pointer_cast<llsf_msgs::MachineReportInfo>(message)) {
            publish_debug(machine_report_info_pub_, *report);
        }
    }

    template <typename Message>
    void publish_debug(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub, const Message &message) {
        std_msgs::msg::String msg;
        msg.data = message.DebugString();
        pub->publish(msg);
    }

    void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string error) {
        RCLCPP_WARN(get_logger(), "Receive error from %s:%u: %s", endpoint.address().to_string().c_str(), endpoint.port(), error.c_str());
    }

    void send_beacon() {
        auto beacon = std::make_shared<llsf_msgs::BeaconSignal>();

        const auto now = std::chrono::system_clock::now().time_since_epoch();

        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);

        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec);

        auto *time = beacon->mutable_time();
        time->set_sec(sec.count());
        time->set_nsec(nsec.count());

        beacon->set_seq(++sequence_nr_);
        beacon->set_number(robot_number_);
        beacon->set_team_name(team_name_);
        beacon->set_team_color(team_color_ == "CYAN" ? llsf_msgs::CYAN : llsf_msgs::MAGENTA);
        beacon->set_peer_name(peer_name_);

        if (pose_received_) {
            auto *pose = beacon->mutable_pose();

            pose->set_x(pose_x_);
            pose->set_y(pose_y_);
            pose->set_ori(pose_yaw_);

            auto *pose_time = pose->mutable_timestamp();
            pose_time->set_sec(sec.count());
            pose_time->set_nsec(nsec.count());
        }

        private_peer_->send(2000, 1, beacon);
    }

private:
    MessageRegister mr_;

    std::unique_ptr<ProtobufBroadcastPeer> private_peer_;
    std::unique_ptr<ProtobufBroadcastPeer> public_peer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr messages_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr game_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_routes_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr machine_report_info_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::TimerBase::SharedPtr beacon_timer_;

    std::string refbox_host_;
    std::string crypto_key_;
    std::string team_name_;
    std::string peer_name_;
    std::string team_color_;
    std::string pose_topic_;

    int robot_number_;
    int public_send_port_;
    int public_recv_port_;
    int private_send_port_;
    int private_recv_port_;

    uint64_t sequence_nr_ = 0;

    bool pose_received_ = false;
    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_yaw_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<RefboxNode>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("ired_refbox"), "Failed to start RefBox node: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

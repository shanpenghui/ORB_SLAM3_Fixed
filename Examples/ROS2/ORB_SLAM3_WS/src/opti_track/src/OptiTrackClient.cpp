#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <opti_track/msg/track_data.hpp>
#include "OptiTrackClient.hpp"

using opti_track::msg::TrackData;
using opti_track::msg::Marker;
using opti_track::msg::RigidBody;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;


std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Publisher<TrackData>> publisher;
TrackData trackData; // NOLINT(cert-err58-cpp)

template<typename T>
rclcpp::Parameter get_parameter(const std::shared_ptr<rclcpp::Node> & node, const std::string & name, T defaultValue) {
    auto param = rclcpp::Parameter(name, defaultValue);
    node->get_parameter_or(name, param, param);
    return param;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("OptiTrackDriver");

    node->declare_parameter("server_address");
    node->declare_parameter("client_address");
    auto serverAddress = get_parameter(node, "server_address", "192.168.1.12").as_string();
    auto clientAddress = get_parameter(node, "client_address", "").as_string();

    RCLCPP_INFO(node->get_logger(), "OptiTrackClient startups with: serverAddress: %s, clientAddress: %s", serverAddress.data(), clientAddress.data());

    publisher = node->create_publisher<TrackData>("opti_track_data", 10);

    OptiTrackClient client([](sFrameOfMocapData *data) {
        trackData.header.frame_id = std::to_string(data->iFrame);
        trackData.header.stamp = node->now();
        trackData.marker_quantity = data->nLabeledMarkers;
        trackData.rigid_body_quantity = data->nRigidBodies;

        std::vector<Marker> markers;
        for(int i=0; i < data->nLabeledMarkers; i++) {
            const auto &item = data->LabeledMarkers[i];
            Marker marker;
            Point32 position;

            position.x = item.x;
            position.y = item.y;
            position.z = item.z;

            marker.id = item.ID;
            marker.position = position;
            marker.state = item.params;
            marker.residual = item.residual;

            markers.push_back(marker);
        }
        trackData.markers = markers;

        std::vector<RigidBody> rigidBodies;
        for(int i=0; i < data->nRigidBodies; i++) {
            const auto &item = data->RigidBodies[i];
            RigidBody rigidBody;
            Pose pose;
            Point position;
            Quaternion orientation;

            position.x = item.x;
            position.y = item.y;
            position.z = item.z;
            orientation.x = item.qx;
            orientation.y = item.qy;
            orientation.z = item.qz;
            orientation.w = item.qw;

            pose.position = position;
            pose.orientation = orientation;

            rigidBody.id = item.ID;
            rigidBody.pose = pose;
            rigidBody.mean_error = item.MeanError;
            rigidBody.tracking_flag = item.params;

            rigidBodies.push_back(rigidBody);
        }
        trackData.rigid_bodies = rigidBodies;


        publisher->publish(trackData);
    }, serverAddress.c_str(), clientAddress.c_str());

    rclcpp::spin(node);
    rclcpp::shutdown();
}
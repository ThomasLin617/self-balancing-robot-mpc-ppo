#include <map>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <state_estimator/state_estimator.hpp>
#include <navigation/planar_odometry.hpp>
#include <robot_ros/conversions.hpp>

class EstimatorNode : public rclcpp::Node {
public:
    EstimatorNode() : Node("state_estimator_node"), estimator_(.5, .12) {
        timeout_ = robot_ros::setting(*this, "sensor_timeout", .01);
        stride_ = robot_ros::setting(*this, "encoder_stride", 1);
        if (timeout_ <= 0 || stride_ < 1 || stride_ * .001 > timeout_) {
            throw std::runtime_error("Invalid sensor freshness/stride.");
        }
        state_pub_ = create_publisher<robot_msgs::msg::RobotState>("robot_state", robot_ros::reliable());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", robot_ros::reliable());
        ready_ = create_publisher<std_msgs::msg::UInt64>("estimator_ready", robot_ros::retained());
        epoch_sub_ = create_subscription<std_msgs::msg::UInt64>("episode_start_ns", robot_ros::retained(),
                                                                [this](const std_msgs::msg::UInt64& m) {
                                                                    epoch_ = m.data;
                                                                    active_ = true;
                                                                    last_ = -1;
                                                                    imu_.clear();
                                                                    wheels_.clear();
                                                                    estimator_.reset();
                                                                    odometry_.reset();
                                                                    ready_->publish(m);
                                                                });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", robot_ros::reliable(), [this](const sensor_msgs::msg::Imu& m) {
                const auto ns = robot_ros::nanoseconds(m.header.stamp);
                if (!active_ || ns < static_cast<int64_t>(epoch_) || ns <= last_) {
                    return;
                }
                for (double value :
                     {m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z}) {
                    if (!std::isfinite(value)) {
                        RCLCPP_ERROR(get_logger(), "Invalid IMU sample");
                        return;
                    }
                }
                imu_[ns] = m;
                process();
            });
        wheel_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", robot_ros::reliable(), [this](const sensor_msgs::msg::JointState& m) {
                const auto ns = robot_ros::nanoseconds(m.header.stamp);
                if (!active_ || ns < static_cast<int64_t>(epoch_)) {
                    return;
                }
                if (m.name.size() != 2 || m.position.size() != 2 || m.velocity.size() != 2) {
                    RCLCPP_ERROR(get_logger(), "Invalid encoder dimensions");
                    return;
                }
                if (!((m.name[0] == "left_wheel_joint" && m.name[1] == "right_wheel_joint") ||
                      (m.name[1] == "left_wheel_joint" && m.name[0] == "right_wheel_joint"))) {
                    return;
                }
                for (double value : {m.position[0], m.position[1], m.velocity[0], m.velocity[1]}) {
                    if (!std::isfinite(value)) {
                        RCLCPP_ERROR(get_logger(), "Non-finite encoder sample");
                        return;
                    }
                }
                wheels_[ns] = m;
                process();
            });
    }

private:
    void process() {
        while (!imu_.empty()) {
            const auto ns = imu_.begin()->first;
            const int64_t expected = epoch_ + ((ns - epoch_) / (stride_ * 1000000LL)) * (stride_ * 1000000LL);
            const auto found = wheels_.find(expected);
            if (found == wheels_.end()) {
                break;
            }
            if (!robot_ros::fresh(found->first, ns, timeout_)) {
                imu_.erase(imu_.begin());
                continue;
            }
            const auto& imu = imu_.begin()->second;
            const auto& wheel = found->second;
            const int left = wheel.name[0] == "left_wheel_joint" ? 0 : 1, right = 1 - left;
            robot_interface::SensorData sensor;
            sensor.timestamp = (ns - epoch_) * 1e-9;
            sensor.gyroscope = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            sensor.accelerometer = {imu.linear_acceleration.x, imu.linear_acceleration.y,
                                    imu.linear_acceleration.z};
            sensor.left_wheel_position = wheel.position[left];
            sensor.right_wheel_position = wheel.position[right];
            sensor.left_wheel_velocity = wheel.velocity[left];
            sensor.right_wheel_velocity = wheel.velocity[right];
            try {
                const auto state = estimator_.update(sensor);
                const auto pose = odometry_.update(state);
                const auto message = robot_ros::to_message(state, epoch_);
                if (!robot_ros::finite(message)) {
                    throw std::runtime_error("Non-finite estimated state.");
                }
                nav_msgs::msg::Odometry odom;
                odom.header = message.header;
                odom.child_frame_id = "base_link";
                odom.pose.pose.position.x = pose.x;
                odom.pose.pose.position.y = pose.y;
                odom.pose.pose.orientation.z = std::sin(pose.yaw * .5);
                odom.pose.pose.orientation.w = std::cos(pose.yaw * .5);
                odom.twist.twist.linear.x = state.forward_velocity;
                odom.twist.twist.angular.z = state.yaw_rate;
                odom_pub_->publish(odom);
                state_pub_->publish(message);
                last_ = ns;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Estimator: %s", e.what());
            }
            imu_.erase(imu_.begin());
            while (!wheels_.empty() && wheels_.begin()->first < expected) {
                wheels_.erase(wheels_.begin());
            }
        }
        if (imu_.size() > 32 || wheels_.size() > 32) {
            RCLCPP_ERROR(get_logger(), "Sensor pairing stalled; dropping buffered data");
            imu_.clear();
            wheels_.clear();
        }
    }
    uint64_t epoch_ = 0;
    int64_t last_ = -1;
    bool active_ = false;
    double timeout_;
    int stride_;
    state_estimator::StateEstimator estimator_;
    navigation::PlanarOdometry odometry_;
    std::map<int64_t, sensor_msgs::msg::Imu> imu_;
    std::map<int64_t, sensor_msgs::msg::JointState> wheels_;
    rclcpp::Publisher<robot_msgs::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ready_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr epoch_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<EstimatorNode>());
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}

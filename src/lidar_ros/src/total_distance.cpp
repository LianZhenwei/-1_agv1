#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <cmath>

class LocalizationSubscriber {
public:
    LocalizationSubscriber() {
        ros::param::set("total_distance_", init_distance_);
        ros::param::set("veh_distance", init_v_);
        subscriber_ = nh_.subscribe("/localization", 1, &LocalizationSubscriber::callback, this); // 0.5 hz
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point current_position = msg->pose.pose.position;

        if (last_position_valid_) {
            float distance = calculateDistance(last_position_, current_position);
            // 0.5s 的距离大于 3m，此时可能切换了楼层
            if(distance > 3){
                last_position_ = current_position;
                return;
            }
            car_v_ = distance/0.5;
            total_distance_ += distance;
            ros::param::set("veh_distance", car_v_);
            ros::param::set("total_distance_", total_distance_);
            ROS_INFO("Current v: %.2f m/s", car_v_);
            ROS_INFO("Current Total Distance: %.2f meters", total_distance_);
        }

        last_position_ = current_position;
        last_position_valid_ = true;

        loop_rate_.sleep(); // 按照设定的频率等待，以控制循环速率
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    geometry_msgs::Point last_position_;
    bool last_position_valid_ = false;
    float total_distance_ = 0;
    float init_distance_ = 0;
    float car_v_ = 0;
    float init_v_ = 0;
    ros::Rate loop_rate_ = ros::Rate(2); // 设置循环频率(Hz)

    float calculateDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
        return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_subscriber");
    LocalizationSubscriber localization_subscriber;
    ros::spin();
    return 0;
}

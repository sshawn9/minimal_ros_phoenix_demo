#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <condition_variable>
#include <csignal>

std::atomic_bool running{true};
std::condition_variable cv;

void signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
    running.store(false);
    cv.notify_all();
    exit(signum);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_phoenix_pub");

    boost::shared_ptr<ros::NodeHandle> nh = nullptr;
    boost::shared_ptr<ros::AsyncSpinner> spinner = nullptr;

    boost::shared_ptr<ros::Publisher> pub = nullptr;
    boost::shared_ptr<ros::Timer> timer = nullptr;

    bool phoenix_check_pass = false;

    auto publish = [&]() {
        if (not phoenix_check_pass) {
            //// comment the line below to keep publishing after roscore stops.
            return;
        }
        static std_msgs::String msg;
        static uint64_t count = 0;
        msg.data = "hello world " + std::to_string(count);
        pub->publish(msg);
        ROS_INFO_STREAM("Phoenix pub: " << msg.data);
        ++count;
    };

    auto phoenix_check = [&]() {
        auto phoenix = [&]() {
            if (not ros::master::check()) {
                phoenix_check_pass = false;
                return;
            }
            if (phoenix_check_pass) {
                return;
            }
            pub = nullptr;
            nh = boost::make_shared<ros::NodeHandle>();
            spinner = boost::make_shared<ros::AsyncSpinner>(1);
            pub = boost::make_shared<ros::Publisher>(nh->advertise<std_msgs::String>("phoenix_demo", 1));
            timer = boost::make_shared<ros::Timer>(nh->createTimer(ros::Duration(1.0), std::bind(publish)));
            spinner->start();
            phoenix_check_pass = true;
        };

        while (running) {
            phoenix();
            signal(SIGINT, signal_handler);
            if (not phoenix_check_pass) {
                std::cout << "Phoenix check failed: verify if roscore is running." << std::endl;
            }
            static std::mutex mutex;
            std::unique_lock<std::mutex> lock{mutex};
            cv.wait_for(lock, std::chrono::milliseconds(300), []() { return !running.load(); });
        }
    };

    std::thread phoenix_check_thread(phoenix_check);
    phoenix_check_thread.join();

    return 0;
}

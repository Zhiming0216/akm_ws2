#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <array>
#include <arpa/inet.h>   // inet_pton

#include "ackerman_chassis2/srv/id501.hpp"
#include "ackerman_chassis2/msg/id502.hpp"
#include "ackerman_chassis2/msg/id503.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ChassisDriverNode : public rclcpp::Node 
{
public:
    ChassisDriverNode()
    : Node("chassis_driver_node2", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      running_(true)
    {
        // 只读取底盘控制必须的参数
        this->get_parameter("can_dev", can_dev_);
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("steering_ratio", steering_ratio_);
        this->get_parameter("max_steer_wheel_deg", max_steer_wheel_deg_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("kingpin_offset", kingpin_offset);
        this->get_parameter("reduction_ratio", reduction_ratio);
        
        last_cmd_vel_time_ = this->now(); // cmd_vel超时看门狗

        // ========== 打开CAN设备 ==========
        struct ifreq ifr;
        struct sockaddr_can addr;
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) exit(1);
        strcpy(ifr.ifr_name, can_dev_.c_str());
        ioctl(can_socket_,SIOCGIFINDEX,&ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) exit(1);

        // ========== 订阅和服务 ==========
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ChassisDriverNode::cmdVelCallback, this, std::placeholders::_1));

        id501_srv_ = this->create_service<ackerman_chassis2::srv::ID501>(
            "ID501",
            std::bind(&ChassisDriverNode::id501Callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        io_switch_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "io_mode_switch",
            std::bind(&ChassisDriverNode::ioSwitchCallback, this, std::placeholders::_1, std::placeholders::_2));

        // ========== 发布反馈报文 ==========
        id502_pub_ = this->create_publisher<ackerman_chassis2::msg::ID502>("/ID502", 10);
        id503_pub_ = this->create_publisher<ackerman_chassis2::msg::ID503>("/ID503", 10);

        // ========== 定时器：20ms发送控制指令 ==========
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ChassisDriverNode::timerCallback, this)
        );

        // ========== 启动CAN接收线程 ==========
        can_recv_thread_ = std::thread(&ChassisDriverNode::canRecvThread, this);

        // ========== 初始化控制变量 ==========
        current_gear_ = 2;
        current_rpm_ = 0;
        current_steer_wheel_deg_ = 0;
    }

    ~ChassisDriverNode()
    {
        running_ = false;
        if (can_recv_thread_.joinable()) can_recv_thread_.join();
        close(can_socket_);
    }

private:
    // ========== 成员变量 ==========
    int can_socket_;
    std::atomic<bool> running_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex ctrl_mutex_;
    rclcpp::Service<ackerman_chassis2::srv::ID501>::SharedPtr id501_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr io_switch_srv_;
    rclcpp::Publisher<ackerman_chassis2::msg::ID502>::SharedPtr id502_pub_;
    rclcpp::Publisher<ackerman_chassis2::msg::ID503>::SharedPtr id503_pub_;
    std::thread can_recv_thread_;
    rclcpp::Time last_cmd_vel_time_;

    // 控制变量
    int current_gear_ = 2;
    int current_rpm_ = 0;
    int current_steer_wheel_deg_ = 0;
    int throttle_enable_ = 1;
    int steer_enable_ = 1;
    int brake_enable_ = 1;
    int light_enable_ = 1;
    int drive_mode_ = 0;
    int target_speed_kmh_ = 0;
    int target_brake_pressure_ = 0;
    int head_light_ = 0;
    int left_light_ = 0;
    int right_light_ = 0;

    // 车辆参数
    std::string can_dev_;
    double wheelbase_;
    double steering_ratio_;
    int max_steer_wheel_deg_;
    double wheel_radius_;
    double kingpin_offset;
    int reduction_ratio;

    // ========== TCP发送函数（IO切换用）==========
    void sendModbusPacket(const std::array<uint8_t, 12>& packet)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "创建socket失败 ");
            return;
        }

        struct timeval tv{1, 0};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(8234);
        inet_pton(AF_INET, "192.168.99.168", &addr.sin_addr);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "连接失败 ");
            close(sock);
            return;
        }

        send(sock, packet.data(), packet.size(), 0);

        uint8_t resp[12];
        ssize_t recvd = recv(sock, resp, sizeof(resp), 0);
        if (recvd == 12 && memcmp(resp, packet.data(), 12) == 0) {
            RCLCPP_INFO(this->get_logger(), "报文成功并收到正确响应 ");
        } else {
            RCLCPP_INFO(this->get_logger(), "报文已发（响应%s）", recvd > 0 ? "不对" : "超时正常");
        }

        close(sock);
        RCLCPP_INFO(this->get_logger(), "TCP连接干净断开 ");
    }

    void performSwitchCombination()
    {
        RCLCPP_INFO(this->get_logger(), "开始执行模式切换组合：先关 → 断开 → 再开 ");

        const std::array<uint8_t, 12> off_packet = {
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00
        };
        sendModbusPacket(off_packet);

        std::this_thread::sleep_for(100ms);

        const std::array<uint8_t, 12> on_packet = {
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x06, 0x00, 0x00, 0x00, 0x01
        };
        sendModbusPacket(on_packet);

        RCLCPP_INFO(this->get_logger(), "关→开组合完成，模式已切换！");
    }

    void ioSwitchCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        performSwitchCombination();
        response->success = true;
        response->message = "已执行关→开组合，模式已切换";
        RCLCPP_INFO(this->get_logger(), "IO切换服务被调用");
    }

    // ========== ID501服务回调 ==========
    void id501Callback(
        const std::shared_ptr<ackerman_chassis2::srv::ID501::Request> req,
        std::shared_ptr<ackerman_chassis2::srv::ID501::Response> res)
    {
        last_cmd_vel_time_ = this->now();

        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        throttle_enable_ = req->throttle_enable;
        steer_enable_ = req->steer_enable;
        brake_enable_ = req->brake_enable;
        light_enable_ = req->light_enable;
        current_gear_ = req->gear;
        drive_mode_ = req->drive_mode;
        current_rpm_ = req->target_moto_rpm;
        target_speed_kmh_ = req->target_speed_kmh;
        current_steer_wheel_deg_ = req->target_steer_angle;
        target_brake_pressure_ = req->target_brake_pressure;
        head_light_ = req->head_light;
        left_light_ = req->left_light;
        right_light_ = req->right_light;
        res->success = true;
        res->message = "ID501指令已更新";
        RCLCPP_INFO(this->get_logger(), "[ID501服务] 收到新指令: gear=%d, rpm=%d, steer=%d, brake=%d, lights:%d%d%d",
            current_gear_, current_rpm_, current_steer_wheel_deg_, target_brake_pressure_, head_light_, left_light_, right_light_);
    }

    // ========== 发送CAN控制指令（0x501帧）==========
    void sendCanCommand(int gear, uint16_t rpm, int steer_wheel_deg)
    {
        const uint8_t control_method =
            (throttle_enable_ & 0x01) |
            ((steer_enable_ & 0x01) << 1) |
            ((brake_enable_ & 0x01) << 2) |
            ((light_enable_ & 0x01) << 3) |
            ((gear & 0x03) << 4) |
            ((drive_mode_ & 0x01) << 6);

        const uint8_t rpm_h = static_cast<uint8_t>((rpm >> 8) & 0xFF);
        const uint8_t rpm_l = static_cast<uint8_t>(rpm & 0xFF);
        
        const uint8_t target_speed_kmh = static_cast<uint8_t>(target_speed_kmh_ & 0x0F);

        const int steer_deg_clamped = std::clamp(steer_wheel_deg, -250, 250);
        const int16_t steer_raw = static_cast<int16_t>(steer_deg_clamped + 1024);
        const uint8_t steer_h = static_cast<uint8_t>((steer_raw >> 8) & 0xFF);
        const uint8_t steer_l = static_cast<uint8_t>(steer_raw & 0xFF);

        const uint8_t brake_bar = static_cast<uint8_t>(std::clamp(target_brake_pressure_, 0, 80));

        uint8_t lights = 0x00;
        if (steer_wheel_deg > 0) {
            lights |= (1 << 2);
        } else if (steer_wheel_deg < 0) {
            lights |= (1 << 1);
        }
        if (head_light_)  lights |= (1 << 0);
        if (left_light_)  lights |= (1 << 1);
        if (right_light_) lights |= (1 << 2);

        struct can_frame frame;
        frame.can_id  = 0x501;
        frame.can_dlc = 8;
        frame.data[0] = control_method;
        frame.data[1] = rpm_h;
        frame.data[2] = rpm_l;
        frame.data[3] = target_speed_kmh;
        frame.data[4] = steer_h;
        frame.data[5] = steer_l;
        frame.data[6] = brake_bar;
        frame.data[7] = lights;

        const ssize_t n = write(can_socket_, &frame, sizeof(frame));
        if (n != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "write(can) failed, ret=%zd, errno=%d", n, errno);
        }
    }

    // ========== 定时器回调 ==========
    void timerCallback()
    {
        int gear, rpm, steer_wheel_deg;

        double timeout_sec = 0.5;
        bool timeout = (this->now() - last_cmd_vel_time_).seconds() > timeout_sec;

        {
            std::lock_guard<std::mutex> lock(ctrl_mutex_);
            if (timeout)
            {
                gear = 2;
                rpm = 0;
                steer_wheel_deg = 0;
            } else {
                gear = current_gear_;
                rpm = current_rpm_;
                steer_wheel_deg = current_steer_wheel_deg_;
            }
        }
        sendCanCommand(gear, rpm, steer_wheel_deg);
    }

    // ========== cmd_vel逆运动学 ==========
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_time_ = this->now();

        const int min_rpm = 0;
        const int max_rpm = 3000;

        double v = msg->linear.x;
        double omega = msg->angular.z;
    
        int gear = 2;
        int rpm = 0;
    
        if (v > 0.0) {
            gear = 3;
        } else if (v < 0.0) {
            gear = 1;
        } else {
            gear = 2;
        }
        rpm = (gear == 2) ? 0 : std::clamp(speedToRpm(std::abs(v)), min_rpm, max_rpm);
    
        double delta_rad = 0.0;
        if (std::abs(v) > 1e-5 && std::abs(omega) > 1e-5) {
            delta_rad = std::atan(wheelbase_ / ((v / omega) - (kingpin_offset / 2)));
        }
        double front_wheel_deg = delta_rad * 180.0 / M_PI;
        double steer_wheel_deg = -front_wheel_deg * steering_ratio_;
        steer_wheel_deg = std::max(-double(max_steer_wheel_deg_), std::min(double(max_steer_wheel_deg_), steer_wheel_deg));
        int steer_wheel_deg_int = static_cast<int>(std::round(steer_wheel_deg));

        {
            std::lock_guard<std::mutex> lock(ctrl_mutex_);
            current_gear_ = gear;
            current_rpm_ = rpm;
            current_steer_wheel_deg_ = steer_wheel_deg_int;
        }
    }

    // ========== 轮速↔线速度转换 ==========
    int speedToRpm(double speed_mps)
    {
        return static_cast<int>(std::round(speed_mps / (2 * M_PI * wheel_radius_) * 60.0 * reduction_ratio));
    }

    // ========== 解析反馈报文 ==========
    int parse_gear(uint8_t byte0) {
        int gear = (byte0 >> 2) & 0x03;
        if (gear == 1) return 3;
        if (gear == 2) return 2;
        if (gear == 3) return 1;
        return 2;
    }

    int parse_rpm(uint8_t high, uint8_t low) {
        int16_t rpm = (high << 8) | low;
        return rpm;
    }

    int parse_steer_wheel_angle(uint8_t high, uint8_t low) {
        int16_t raw = (high << 8) | low;
        int angle = raw - 1024;
        return angle;
    }

    std::string gearToString(int gear) {
        switch (gear)
        {
            case 1: return "D";
            case 2: return "N";
            case 3: return "R";
            default: return "N";
        }
    }

    void parseCan503Frame(const struct can_frame& frame, ackerman_chassis2::msg::ID503& msg)
    {
        msg.actual_speed = frame.data[0];

        msg.actual_epb_sts  = (frame.data[1] & 0x01) != 0;
        msg.actual_back_sts = ((frame.data[1] >> 1) & 0x01) != 0;
    }

    void parseCan502Frame(const can_frame &frame, ackerman_chassis2::msg::ID502 &msg)
    {
        uint8_t byte0 = frame.data[0];
        msg.actual_status = byte0 & 0x03;
        msg.actual_gear = (byte0 >> 2) & 0x03;
        msg.actual_gear_str = gearToString(msg.actual_gear);        
        msg.actual_left_light = (byte0 >> 4) & 0x01;
        msg.actual_right_light = (byte0 >> 5) & 0x01;
        msg.actual_head_light = (byte0 >> 6) & 0x01;
        msg.actual_brake_light = (byte0 >> 7) & 0x01;

        msg.actual_moto_spd = (int16_t)((frame.data[1] << 8) | frame.data[2]);
        msg.actual_steering_angle = (int16_t)((frame.data[3] << 8) | frame.data[4]) - 1024;
        msg.actual_brake_pressure = frame.data[5];

        uint8_t byte6 = frame.data[6];
        msg.can0_pc501_timeout = byte6 & 0x01;
        msg.can0_rmc301_timeout = (byte6 >> 1) & 0x01;
        msg.can1_eps18f_timeout = (byte6 >> 2) & 0x01;
        msg.can1_ebs142_timeout = (byte6 >> 3) & 0x01;
        msg.can2_motoa10f8109a_timeout = (byte6 >> 4) & 0x01;

        uint8_t byte7 = frame.data[7];
        msg.steer_error = byte7 & 0x01;
        msg.brake_error = (byte7 >> 1) & 0x01;
        msg.moto_error = (byte7 >> 2) & 0x01;
    }

    // ========== CAN接收线程（只发布ID502和ID503）==========
    void canRecvThread() {
        struct can_frame frame;
        while (running_) {
            ssize_t n = read(can_socket_, &frame, sizeof(frame));
            if (n != sizeof(frame)) continue;

            if (frame.can_id == 0x503) {
                ackerman_chassis2::msg::ID503 msg;
                parseCan503Frame(frame, msg);
                id503_pub_->publish(msg);
            }

            if (frame.can_id == 0x502) {
                ackerman_chassis2::msg::ID502 msg;
                parseCan502Frame(frame, msg);
                id502_pub_->publish(msg);
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ChassisDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
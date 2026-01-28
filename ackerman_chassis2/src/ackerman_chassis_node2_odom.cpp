#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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
#include <iomanip>
#include <sstream>
#include <std_msgs/msg/int32.hpp>
#include "ackerman_chassis2/srv/id501.hpp"
#include "ackerman_chassis2/msg/id502.hpp"
#include "ackerman_chassis2/msg/id503.hpp"
#include <cstdlib>
#include "std_srvs/srv/trigger.hpp"
#include <array>         // 因为用了 std::array<uint8_t, 12>
#include <arpa/inet.h>   // 因为用了 inet_pton

//可以直接在代码里用时间字面量
using namespace std::chrono_literals;

//声明一个叫ChassisDriverNode的类，继承ROS2的节点基类，可以发布订阅消息
class ChassisDriverNode : public rclcpp::Node 
{
public:
    ChassisDriverNode()
    : Node("chassis_driver_node2", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      running_(true)
    {
        
        // ========== 1. 声明参数并读取 ==========
        // 声明参数，设置默认值（如果yaml没写就用默认值）
        // this->declare_parameter<std::string>("can_dev", "can0");
        // this->declare_parameter<double>("wheelbase", 1.04);
        // this->declare_parameter<double>("steering_ratio", 12.5);
        // this->declare_parameter<int>("max_steer_wheel_deg", 250);
        // this->declare_parameter<double>("wheel_radius", 0.17);
        // this->declare_parameter<double>("kingpin_offset", 0.61);
        // this->declare_parameter<int>("reduction_ratio", 23);

        // 先检查有没有声明过
        if (!this->has_parameter("pub_odom_tf")) {
            this->declare_parameter<bool>("pub_odom_tf", true);
        }
        // 然后再获取值
        this->get_parameter("pub_odom_tf", pub_odom_tf_);

        RCLCPP_INFO(this->get_logger(), "底盘驱动初始化完成，TF发布状态: %s", pub_odom_tf_ ? "开启" : "关闭");

        // 读取参数
        this->get_parameter("odom_init_x", x_);
        this->get_parameter("odom_init_y", y_);
        this->get_parameter("odom_init_z", z_);
        this->get_parameter("can_dev", can_dev_);
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("steering_ratio", steering_ratio_);
        this->get_parameter("max_steer_wheel_deg", max_steer_wheel_deg_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("kingpin_offset", kingpin_offset);
        this->get_parameter("reduction_ratio", reduction_ratio);
        
        
        last_cmd_vel_time_ = this->now(); // cmd_vel超时看门狗

        // 1.打开CAN设备
        struct ifreq ifr;
        struct sockaddr_can addr;
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) exit(1);
        strcpy(ifr.ifr_name, can_dev_.c_str());
        ioctl(can_socket_,SIOCGIFINDEX,&ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) exit(1);

        // 2.话题订阅及发布
        // 2.1.订阅/cmd_vel话题，接收速度指令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ChassisDriverNode::cmdVelCallback, this, std::placeholders::_1));
        // // 2.2.订阅/motospd话题，接收转速指令
        // motospd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "/motospd", 10, std::bind(&ChassisDriverNode::motospdCallback, this, std::placeholders::_1));
        // motospd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/motospd", 10);
        // // 2.3.订阅/steerangle话题，接收角度指令（方向盘角度）
        // steerangle_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "/steerangle", 10, std::bind(&ChassisDriverNode::steerangleCallback, this, std::placeholders::_1));
        // steerangle_pub_ = this->create_publisher<std_msgs::msg::Int32>("/steerangle", 10);
        // // 2.4.订阅/brake话题，接收刹车指令
        // brake_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "/brake", 10, std::bind(&ChassisDriverNode::brakeCallback, this, std::placeholders::_1));
        // brake_pub_ = this->create_publisher<std_msgs::msg::Int32>("/brake", 10);
        // // 2.5.订阅/light话题，接收灯光指令
        // light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        //     "/light", 10, std::bind(&ChassisDriverNode::lightCallback, this, std::placeholders::_1));
        // light_pub_ = this->create_publisher<std_msgs::msg::Int32>("/light", 10);

        // 3.定时器，20ms周期，定时发送CAN控制指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ChassisDriverNode::timerCallback, this)
        );

        // 4. 发布odom和TF
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 4.0 启动501service服务
        id501_srv_ = this->create_service<ackerman_chassis2::srv::ID501>(
            "ID501",
            std::bind(&ChassisDriverNode::id501Callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 4.1 发布502报文信息
        id502_pub_ = this->create_publisher<ackerman_chassis2::msg::ID502>("/ID502", 10);

        // 4.2 发布503报文信息
        id503_pub_ = this->create_publisher<ackerman_chassis2::msg::ID503>("/ID503", 10);

        // 5. 启动CAN接收线程，实时接收底盘反馈
        can_recv_thread_ = std::thread(&ChassisDriverNode::canRecvThread, this);
        last_time_ = this->now().seconds();

        // 6. 初始化控制变量
        current_gear_ = 2;              // 当前档位，1=倒车，2=空挡，3=前进
        current_rpm_ = 0;               // 当前目标转速（单位： rpm/min）
        current_steer_wheel_deg_ = 0;   // 当前目标方向盘角度(°)

        // 新增：IO模式切换服务（Trigger类型，调用一次切换一次）
        io_switch_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "io_mode_switch",  // 服务名，自己起个好记的
            std::bind(&ChassisDriverNode::ioSwitchCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "IO切换服务已启动！调用一次就切换一次模式（关→开组合）");
    }

    ~ChassisDriverNode()
    {
        running_ = false;
        if (can_recv_thread_.joinable()) can_recv_thread_.join();
        close(can_socket_);
        // ====== 程序退出时执行 ======
        // 关闭io模块接口（模式为手动模式）
        // std::thread io_exit_thread([this]() {
        //     int ret3 = system(R"(echo "00 01 00 00 00 06 01 06 00 00 00 00" | xxd -r -p | nc -w 1 192.168.1.126 8234)");
        //     RCLCPP_INFO(this->get_logger(), "执行io指令3: %d", ret3);
        //     int ret4 = system(R"(echo "00 01 00 00 00 06 01 06 00 00 00 01" | xxd -r -p | nc -w 1 192.168.1.126 8234)");
        //     RCLCPP_INFO(this->get_logger(), "执行io指令4: %d", ret4);
        // });
        // io_exit_thread.detach();

        // RCLCPP_INFO(this->get_logger(), "节点退出，IO服务结束");
    }

private:
    // 主要成员变量
    int can_socket_; //CAN原始套接字
    std::atomic<bool> running_; // 线程运行标志
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // /cmd_vel订阅器
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
    std::mutex ctrl_mutex_; // 控制变量互斥锁
    // 添加service服务
    rclcpp::Service<ackerman_chassis2::srv::ID501>::SharedPtr id501_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr io_switch_srv_;  // IO切换服务

    // 新增：TF发布开关
    bool pub_odom_tf_ = true;
    
    // 控制指令相关变量
    int current_gear_;               // 当前目标档位（1=倒车，2=空挡，3=前进）
    int current_steer_wheel_deg_;    // 当前目标方向盘角度（单位：度）
    int current_rpm_;                // 当前目标转速（单位： rpm/min）
    int current_brake_pressure_ = 0; // 当前目标制动 （单位：bar）
    int light_mode_ = 0;             // 当前目标灯光 （1.左转 2.右转 3.前照灯）
    // 新增服务使用的501控制变量
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
    std::string can_dev_;      // CAN设备名
    double wheelbase_ ;          // 轴距（单位：米），前后轮中心距离
    double steering_ratio_ ;     // 转向传动比（方向盘角度/前轮转角）
    int max_steer_wheel_deg_ ;    // 方向盘最大角度（单位：度）
    double wheel_radius_ ;       // 车轮半径（单位：米）
    double kingpin_offset ;      // 主销中心距（单位：米） 
    int reduction_ratio ;          // 减速比

    // 里程计相关变量
    double x_ = 0.0, y_ = 0.0, z_ = 0.0, yaw_ = 0.0;   // 车辆当前位姿（x, y, z, 航向角yaw）
    double last_time_ = 0.0;                 // 上一次里程计更新时间（秒）
    int last_gear_ = 2;                      // 上一次反馈档位
    int last_steer_wheel_deg_ = 0;           // 上一次反馈方向盘角度（度）
    int last_rpm_ = 0;                       // 上一次反馈转速（rpm/min）
    double last_speed_ms_ = 0;               // 上一次反馈速度（m/s）
    // int last_left_light_ = 0;                // 上一次左转灯状态
    // int last_right_light_ = 0;               // 上一次右转灯状态
    int signed_steer_wheel_deg_ = 0;         // 带符号的角度信息

    // 定义状态机（用于判断左转或右转）
    enum TurnState { STRAIGHT = 0, LEFT = 1, RIGHT = 2 };
    TurnState turn_state_ = STRAIGHT; // 默认为直行

    // ROS2发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 里程计发布器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF广播器
    std::thread can_recv_thread_; // CAN接收线程
    rclcpp::Time last_cmd_vel_time_; // cmd_vel超时看门狗
    rclcpp::Publisher<ackerman_chassis2::msg::ID502>::SharedPtr id502_pub_; // 发布502报文信息
    rclcpp::Publisher<ackerman_chassis2::msg::ID503>::SharedPtr id503_pub_; // 发布503报文信息

    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motospd_sub_;         //转速订阅及发布
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motospd_pub_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steerangle_sub_;      //方向盘订阅及发布
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steerangle_pub_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr brake_sub_;           //刹车订阅及发布
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr brake_pub_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_sub_;           //灯光订阅及发布
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light_pub_;
    
    // TCP发送函数（保持最稳版，1秒超时 + 读响应 + 强制关闭）
    void sendModbusPacket(const std::array<uint8_t, 12>& packet)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "创建socket失败 ");
            return;
        }

        struct timeval tv{1, 0};  // 1秒超时
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(8234);
        inet_pton(AF_INET, "192.168.99.120", &addr.sin_addr);

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

    // 执行一次完整“关→开”组合（切换模式）
    void performSwitchCombination()
    {
        RCLCPP_INFO(this->get_logger(), "开始执行模式切换组合：先关 → 断开 → 再开 🚀");

        // 先发关（0）
        const std::array<uint8_t, 12> off_packet = {
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00
        };
        sendModbusPacket(off_packet);

        std::this_thread::sleep_for(100ms);  // 确保彻底断开

        // 再发开（1）
        const std::array<uint8_t, 12> on_packet = {
            0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x06, 0x00, 0x00, 0x00, 0x01
        };
        sendModbusPacket(on_packet);

        RCLCPP_INFO(this->get_logger(), "关→开组合完成，模式已切换！");
    }

    // 服务回调（调用一次就切换一次）
    void ioSwitchCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        performSwitchCombination();
        response->success = true;
        response->message = "已执行关→开组合，模式已切换";
        RCLCPP_INFO(this->get_logger(), "IO切换服务被调用 👌");
    }

    // 501回调
    void id501Callback(
        const std::shared_ptr<ackerman_chassis2::srv::ID501::Request> req,
        std::shared_ptr<ackerman_chassis2::srv::ID501::Response> res)
    {
        // 刷新看门狗时间
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


    // ========== 发送CAN控制指令（0x501帧） ==========
    /**
     * @brief 发送底盘控制指令到CAN总线
     * @param gear 档位（1=倒车，2=空挡，3=前进）
     * @param rpm 目标转速（rpm/min）
     * @param steer_wheel_deg 方向盘角度（度）
     */
    void sendCanCommand(int gear, uint16_t rpm, int steer_wheel_deg)
    {
        //定义转速是十六位无符号整数类型
        //const uint16_t rpm = 0;

        // 2) 控制字节 Byte0
        // const uint8_t throttle_enable = 1;     // Bit0: 油门使能
        // const uint8_t steer_enable    = 1;     // Bit1: 转向使能
        // const uint8_t brake_enable    = 1;     // Bit2: 刹车使能
        // const uint8_t lgt_enable      = 1;     // Bit3: 灯光使能
        // const uint8_t target_gear = (gear == 1 || gear == 2 || gear == 3) ? gear : 2; // Bit4-5: 档位
        // const uint8_t drive_mode_rpm  = 0;     // Bit6: 0=转速控制, 1=车速控制

        const uint8_t control_method =
            (throttle_enable_ & 0x01) |
            ((steer_enable_ & 0x01) << 1) |
            ((brake_enable_ & 0x01) << 2) |
            ((light_enable_ & 0x01) << 3) |
            ((gear & 0x03) << 4) |
            ((drive_mode_ & 0x01) << 6);

        // 3) 转速 Byte1-Byte2（高字节在前）
        const uint8_t rpm_h = static_cast<uint8_t>((rpm >> 8) & 0xFF);     //向右移8位，然后取最低的8位，其它高位全部清零
        const uint8_t rpm_l = static_cast<uint8_t>( rpm       & 0xFF);     //最低的8位，其它高位全部清零
        
        // 4) 车速 Byte3（仅“车速控制模式”用，这里用转速控制，置0）
        const uint8_t target_speed_kmh = static_cast<uint8_t>(target_speed_kmh_ & 0x0F);;

        // 5) 转向角 Byte4-Byte5（协议：raw = 角度(°) + 1024，左负右正；高在前，低在后）
        const int steer_deg_clamped = std::clamp(steer_wheel_deg, -250, 250);
        const int16_t steer_raw = static_cast<int16_t>(steer_deg_clamped + 1024);
        const uint8_t steer_h = static_cast<uint8_t>((steer_raw >> 8) & 0xFF);
        const uint8_t steer_l = static_cast<uint8_t>( steer_raw       & 0xFF);

        // 6) 刹车压力 Byte6（0~80 bar）
        const uint8_t brake_bar = static_cast<uint8_t>(std::clamp(current_brake_pressure_, 0, 80));; 

        // 7) 灯光 Byte7
        uint8_t lights = 0x00;              // bit0:前照灯, bit1:左转, bit2:右转
        
        // 自动：根据方向盘角度自动点亮转向灯
        if (steer_wheel_deg > 0) {
            lights |= (1 << 2); // 右转灯
        } else if (steer_wheel_deg < 0) {
            lights |= (1 << 1); // 左转灯
        }

        // 手动：服务/界面设置的灯光也能点亮
        if (head_light_)  lights |= (1 << 0);
        if (left_light_)  lights |= (1 << 1);
        if (right_light_) lights |= (1 << 2);

        // 8) 组帧
        struct can_frame frame;
        frame.can_id  = 0x501;
        frame.can_dlc = 8;
        frame.data[0] = control_method;   // Byte0
        frame.data[1] = rpm_h;            // Byte1
        frame.data[2] = rpm_l;            // Byte2
        frame.data[3] = target_speed_kmh; // Byte3
        frame.data[4] = steer_h;          // Byte4
        frame.data[5] = steer_l;          // Byte5
        frame.data[6] = brake_bar;        // Byte6
        frame.data[7] = lights;           // Byte7

        // // 8) 打印可读信息 (控制方式，档位，转速，旋转角度)
        // RCLCPP_INFO(
        //         this->get_logger(),
        //         "[TX 0x501] mode: RPM, gear:%d (1=R,2=N,3=D), rpm:%u, steer_deg:%d (raw:%d)",
        //         gear, rpm, steer_deg_clamped, static_cast<int>(steer_raw)
        // );

        // // 9) 打印candump格式 (发送的can报文)
        // {
        //     std::ostringstream oss;
        //     oss << std::uppercase << std::hex << std::setfill('0');
        //     oss << "501#";
        //     for (int i = 0; i < frame.can_dlc; ++i) {
        //         if (i) oss << ".";
        //         oss << std::setw(2) << static_cast<int>(frame.data[i]);
        //     }
        //     RCLCPP_INFO(this->get_logger(), "[TX BYTES] %s", oss.str().c_str());
        // }

        // 10) 发送CAN帧
        const ssize_t n = write(can_socket_, &frame, sizeof(frame));
        if (n != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "write(can) failed, ret=%zd, errno=%d", n, errno);
        }
    }

    // ========== 定时器回调：定时发送CAN控制指令 ==========
    void timerCallback()
    {
        int gear, rpm, steer_wheel_deg;

        // 判断是否超时
        double timeout_sec = 0.5; // 超时时间（秒）
        bool timeout = false;
        auto now = this->now();
        if ((now - last_cmd_vel_time_).seconds() > timeout_sec) {
            timeout = true;
        }

        {
            std::lock_guard<std::mutex> lock(ctrl_mutex_);
            if (timeout)
            {
                gear = 2; // 空档
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

    // ========== 逆运动学：/cmd_vel回调 ==========
    /**
     * @brief /cmd_vel回调，将速度指令转换为底盘控制参数
     * @param msg ROS2 Twist消息，linear.x为前进速度(m/s)，angular.z为角速度(rad/s)
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_time_ = this->now(); // 记录收到消息的时间

        // 转速阈值
        const int min_rpm = 0;
        const int max_rpm = 3000;

        int rpm = 0;          // 默认转速
        int gear = 2;         // 默认空挡
        //int speed_kmh = 0;    // 默认速度0
    
        double v = msg->linear.x;      // 前进速度（m/s）
        double omega = msg->angular.z; // 角速度（rad/s）
    
        // 计算目标转速（将目标速度转为转速）
        //const int target_rpm = speedToRpm(v);

        // 判定档位和转速
        if (v > 0.0) {
            gear = 3;
        } else if (v < 0.0) {
            gear = 1;
        } else {
            gear = 2;
        }
        rpm = (gear == 2) ? 0 : std::clamp(speedToRpm(std::abs(v)), min_rpm, max_rpm);
    
        // 逆运动学：根据速度和角速度计算前轮转角
        double delta_rad = 0.0; // 前轮转角（弧度）
        if (std::abs(v) > 1e-5) {
            // Ackermann逆运动学公式
            delta_rad = std::atan(wheelbase_ / ((v / omega) - (kingpin_offset / 2)));
        }
        // 将计算完的弧度转为角度以及方向盘旋转的角度
        double front_wheel_deg = delta_rad * 180.0 / M_PI; // 前轮转角（度）
        double steer_wheel_deg = -front_wheel_deg * steering_ratio_; // 方向盘角度（度）
        // 限幅
        steer_wheel_deg = std::max(-double(max_steer_wheel_deg_), std::min(double(max_steer_wheel_deg_), steer_wheel_deg));
        int steer_wheel_deg_int = static_cast<int>(std::round(steer_wheel_deg));  // 方向盘转角

        // 日志输出
        RCLCPP_INFO(this->get_logger(),
            "[逆运动学] /cmd_vel: linear.x=%.3f m/s, angular.z=%.3f rad/s | gear=%d, speed=%.2f m/s, 转速=%d rpm/min, 方向盘=%d deg (前轮=%.2f deg)",
            msg->linear.x, msg->angular.z, gear, rpmToSpeed(rpm), rpm, steer_wheel_deg_int, front_wheel_deg);

        // 更新控制变量
        {
            std::lock_guard<std::mutex> lock(ctrl_mutex_);
            current_gear_ = gear;
            current_rpm_ = rpm;
            current_steer_wheel_deg_ = steer_wheel_deg_int;
        }
    }

    // // ========== 转速：/motospd订阅 ==========
    // void motospdCallback(const std_msgs::msg::Int32::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(ctrl_mutex_);
    //     current_rpm_ = msg->data;
    //     if (msg->data > 0)
    //         current_gear_ = 3; // 前进
    //     else if (msg->data < 0)
    //         current_gear_ = 1; // 倒车
    //     else
    //         current_gear_ = 2; // 空挡
    // }

    // // ========== 方向盘旋转角度：/steerangle订阅 ==========
    // void steerangleCallback(const std_msgs::msg::Int32::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(ctrl_mutex_);
    //     current_steer_wheel_deg_ = msg->data;
    // }

    // // ========== 刹车压力：/brake订阅 ==========
    // void brakeCallback(const std_msgs::msg::Int32::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(ctrl_mutex_);
    //     current_brake_pressure_ = msg->data;
    // }

    // // ========== 灯光：/light订阅 ==========
    // void lightCallback(const std_msgs::msg::Int32::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(ctrl_mutex_);
    //     light_mode_ = msg->data;
    // }

    // ========== 轮速和线速度相互转换 ==========
    /**
     * @brief 轮速和线速度相互转换
     * @param rpm 轮速 (rpm/min)
     * @param wheel_radius_ 车轮半径（m）
     */
    // 轮速(rpm)转线速度(m/s)
    double rpmToSpeed(int rpm)
    {
        return 2 * M_PI * wheel_radius_ * rpm / 60.0 / reduction_ratio;
    }

    // 线速度(m/s)转轮速(rpm)
    int speedToRpm(double speed_mps)
    {
        return static_cast<int>(std::round(speed_mps / (2 * M_PI * wheel_radius_) * 60.0 * reduction_ratio));
    }

    // ========== 解析反馈报文辅助函数 ==========
    // 档位解析
    int parse_gear(uint8_t byte0) {
        int gear = (byte0 >> 2) & 0x03;
        if (gear == 1) return 3; // 前进
        if (gear == 2) return 2; // 空挡
        if (gear == 3) return 1; // 倒车
        return 2;
    }
    // 转速解析
    int parse_rpm(uint8_t high, uint8_t low) {
        int16_t rpm = (high << 8) | low;
        return rpm;
    }
    // 方向盘角度解析
    int parse_steer_wheel_angle(uint8_t high, uint8_t low) {
        int16_t raw = (high << 8) | low;
        int angle = raw - 1024;
        return angle;
    }
    // ======= 档位数字转为字母 =======
    std::string gearToString(int gear) {
        switch (gear)
        {
            case 1: return "D";
            case 2: return "N";
            case 3: return "R";
            default: return "N";
        }
    }

    // 解析503报文
    void parseCan503Frame(const struct can_frame& frame, ackerman_chassis2::msg::ID503& msg)
    {
        // Byte0: ActualSpeed
        msg.actual_speed = frame.data[0]; // 1分辨率，无偏移，单位km/h

        // Byte1: 状态位
        msg.actual_epb_sts  = (frame.data[1] & 0x01) != 0;      // Bit0
        msg.actual_back_sts = ((frame.data[1] >> 1) & 0x01) != 0; // Bit1
    }

    // 解析502报文
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

    // ========== 正运动学：CAN反馈线程 ==========
    /**
     * @brief CAN接收线程，处理底盘反馈报文，解算里程计
     */
    void canRecvThread() {
        // CAN报文接收循环
        struct can_frame frame;
        while (running_) {
            ssize_t n = read(can_socket_, &frame, sizeof(frame));
            if (n != sizeof(frame)) continue;

            // 发布503报文
            if (frame.can_id == 0x503) {
                ackerman_chassis2::msg::ID503 msg;
                parseCan503Frame(frame, msg);
                id503_pub_->publish(msg);
            }

            // 发布502报文
            if (frame.can_id == 0x502) { // 反馈报文，含档位、转角、转速
            	
            	// 发布ID502的信息
            	ackerman_chassis2::msg::ID502 msg;
            	parseCan502Frame(frame, msg);
            	id502_pub_->publish(msg);
            	
                    uint8_t byte0 = frame.data[0];
                    // // 1.解析实际状态
                    // int actual_motospd = (frame.data[1] << 8) | frame.data[2];     // 实际转速
                    // int16_t steer_raw = (frame.data[3] << 8) | frame.data[4];      
                    // int actual_steerangle = steer_raw - 1024;                      // 实际方向盘角度
                    // int actual_brake_pressure = frame.data[5];                     // 实际刹车压力

                    // bool actual_left_light  = ((byte0 >> 4) & 0x01) != 0;          // 实际左转灯
                    // bool actual_right_light = ((byte0 >> 5) & 0x01) != 0;          // 实际右转灯
                    // bool actual_head_light  = ((byte0 >> 6) & 0x01) != 0;          // 实际前照灯

                    // // 0=关，1=左，2=右，3=前照灯，4=左+前，5=右+前，6=左+右，7=全开
                    // int actual_light = 0;
                    // if (actual_left_light && !actual_right_light && !actual_head_light) actual_light = 1;
                    // else if (!actual_left_light && actual_right_light && !actual_head_light) actual_light = 2;
                    // else if (!actual_left_light && !actual_right_light && actual_head_light) actual_light = 3;
                    // else if (actual_left_light && !actual_right_light && actual_head_light) actual_light = 4;
                    // else if (!actual_left_light && actual_right_light && actual_head_light) actual_light = 5;
                    // else if (actual_left_light && actual_right_light && !actual_head_light) actual_light = 6;
                    // else if (actual_left_light && actual_right_light && actual_head_light) actual_light = 7;
                    // else actual_light = 0;

                    // // 发布实际值到原有topic
                    // // 转速
                    // std_msgs::msg::Int32 msg;
                    // msg.data = actual_motospd;
                    // motospd_pub_->publish(msg);

                    // // 角度
                    // msg.data = actual_steerangle;
                    // steerangle_pub_->publish(msg);

                    // // 刹车
                    // msg.data = actual_brake_pressure;
                    // brake_pub_->publish(msg);

                    // // 灯光
                    // msg.data = actual_light;
                    // light_pub_->publish(msg);

                    // 1. 解析当前灯光状态
                    int curr_left_light  = (byte0 >> 4) & 0x01;
                    int curr_right_light = (byte0 >> 5) & 0x01;

                    // 2. 判断左/右转（只要bit4出现1就切换到左转状态，只有bit5出现1才切换到右转状态）
                    if (curr_left_light == 1) {
                        turn_state_ = LEFT;
                    } else if (curr_right_light == 1) {
                        turn_state_ = RIGHT;
                    }

                    // 3. 解析方向盘角度绝对值
                    // int16_t steer_raw = (frame.data[3] << 8) | frame.data[4];
                    // int abs_angle = steer_raw - 1024;
                    last_steer_wheel_deg_ = parse_steer_wheel_angle(frame.data[3], frame.data[4]); // 最终转角

                    // 4. 根据状态机决定带符号方向盘角度
                    if (turn_state_ == LEFT) {
                        signed_steer_wheel_deg_ = -last_steer_wheel_deg_;
                    } else if (turn_state_ == RIGHT) {
                        signed_steer_wheel_deg_ = last_steer_wheel_deg_;
                    } else {
                        signed_steer_wheel_deg_ = 0;
                    }

                    last_gear_ = parse_gear(byte0); // 最终档位
                    last_rpm_ = parse_rpm(frame.data[1],frame.data[2]); // 最终转速
            }
            // 1.速度解算(通过最终转速计算最终速度，也就是线速度)
            last_speed_ms_ = rpmToSpeed(last_rpm_);

            // 1.1 档位修正：D档时速度为正，R档时速度为负
            if (last_gear_ == 1) { // 1 = 倒车
                last_speed_ms_ = -std::abs(last_speed_ms_);
            } else if (last_gear_ == 3) { // 3 = 前进
                last_speed_ms_ = std::abs(last_speed_ms_);
            }
            
            // 线速度标定
            const double linear_cali = 1.021; // 标定系数
            last_speed_ms_ *= linear_cali;    // 标定后的值

            // 2.方向盘角度转前轮转角
            double front_wheel_deg = signed_steer_wheel_deg_ / steering_ratio_; // 前轮转角（度）
            double steering_angle_rad = front_wheel_deg * M_PI / 180.0;       // 前轮转角（弧度）

            // 3.角速度解算
            double w = 0.0; // 角速度（rad/s）
            if (std::abs(std::cos(steering_angle_rad)) > 1e-5)
            {
                // Ackermann正运动学公式
                w = last_speed_ms_ / ((kingpin_offset / 2) + (wheelbase_ / std::tan(steering_angle_rad)));
            }
            w = -w; // 方向修正（左负右正）

            // 角速度标定
            const double angular_cali = 1.00;
            w *= angular_cali;

            // 4. 位姿积分
            // 获取当前时间（单位：秒）
            double now = this->now().seconds();
            // 计算与上一次积分的时间间隔dt
            double dt = now - last_time_;
            // 更新last_time_为当前时间，为下次积分做准备
            last_time_ = now;
            // 计算本周期内的航向角变化量（delta_yaw = 角速度 * 时间间隔）
            double delta_yaw = w * dt;
            // 累加航向角变化，更新当前航向角yaw_（单位：弧度）
            yaw_ += delta_yaw;
            // 计算本周期内在x方向上的位移增量（dx = 线速度 * cos(航向角) * 时间间隔）
            double dx = last_speed_ms_ * std::cos(yaw_) * dt;
            // 计算本周期内在y方向上的位移增量（dy = 线速度 * sin(航向角) * 时间间隔）
            double dy = last_speed_ms_ * std::sin(yaw_) * dt;
            // 累加位移，更新当前位置x_和y_
            x_ += dx;
            y_ += dy;

            // // 日志输出
            // RCLCPP_INFO(this->get_logger(),
            //     "[正运动学] x=%.3f, y=%.3f, yaw=%.3f, v=%.3f m/s, w=%.3f rad/s, 转速=%d rpm/min, 前轮=%.2f deg, 方向盘=%d deg",
            //     x_, y_, yaw_, last_speed_ms_, w, last_rpm_, front_wheel_deg, last_steer_wheel_deg_);

            // 发布里程计和TF
            publishOdomAndTF(last_speed_ms_, w);
        }
    }

    // ========== 发布里程计和TF ==========
    /**
     * @brief 发布nav_msgs/Odometry和TF变换
     * @param v 当前线速度（m/s）
     * @param w 当前角速度（rad/s）
     */
    void publishOdomAndTF(double v, double w) {
        // 获取当前时间戳
        auto now = this->now();
        // 创建并填充里程计消息
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;                  // 设置消息时间戳
        odom.header.frame_id = "odom";             // 参考坐标系为"odom"
        odom.child_frame_id = "base_link";         // 子坐标系为"base_link"
        // 设置位置
        odom.pose.pose.position.x = x_;            // 当前位置x
        odom.pose.pose.position.y = y_;            // 当前位置y
        odom.pose.pose.position.z = z_;            // 当前位置z
        // 设置姿态（四元数，表示航向角yaw_）
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);                      // 只绕z轴旋转（航向角），roll和pitch为0
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        // 设置速度信息
        odom.twist.twist.linear.x = v;             // 线速度（x方向，前进速度）
        odom.twist.twist.angular.z = w;            // 角速度（绕z轴，航向角速度）
        // 发布里程计消息
        odom_pub_->publish(odom);
        if (pub_odom_tf_){
            // 创建并填充TF变换消息
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;                      // 设置时间戳
            t.header.frame_id = "odom";                // 父坐标系为"odom"
            t.child_frame_id = "base_link";            // 子坐标系为"base_link"
            // 设置平移部分
            t.transform.translation.x = x_;            // 当前位置x
            t.transform.translation.y = y_;            // 当前位置y
            t.transform.translation.z = z_;            // 当前位置z
            // 设置旋转部分（四元数）
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            // 发送TF变换
            tf_broadcaster_->sendTransform(t);
        }
    }
};



int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    // 直接构造节点，不用传参
    // 自动判断变量类型+智能指针工厂函数（会在对象不用时自动释放内存）
    auto node = std::make_shared<ChassisDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <Arduino.h>
#include <DFRobot_BMX160.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

// ==== Motor pins ====
#define PUL_PINS   {18, 14, 32, 25}
#define DIR_PINS   {19, 27, 33, 26}
#define CHANNELS   {0, 1, 2, 3}

const int pwm_pins[4] = PUL_PINS;
const int dir_pins[4] = DIR_PINS;
const int channels[4] = CHANNELS;

// ==== Encoder pins (1 pulse per wheel) ====
const int ENC_PINS[4] = {34, 35, 36, 39};   // FL, FR, RL, RR

volatile int32_t enc_ticks[4] = {0,0,0,0};
volatile int last_cmd_dir[4] = {0,0,0,0};

// ==== Encoder ISR (digitalRead style) ====
void IRAM_ATTR enc_isr_0() {
  if (digitalRead(ENC_PINS[0]) == HIGH) enc_ticks[0]++;
}
void IRAM_ATTR enc_isr_1() {
  if (digitalRead(ENC_PINS[1]) == HIGH) enc_ticks[1]++;
}
void IRAM_ATTR enc_isr_2() {
  if (digitalRead(ENC_PINS[2]) == HIGH) enc_ticks[2]++;
}
void IRAM_ATTR enc_isr_3() {
  if (digitalRead(ENC_PINS[3]) == HIGH) enc_ticks[3]++;
}


// ==== BMX160 IMU ====
DFRobot_BMX160 bmx160;

// ==== micro-ROS ====
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_publisher_t log_pub;

rcl_subscription_t freq_sub;
rcl_timer_t imu_timer, odom_timer;
rclc_executor_t executor;

std_msgs__msg__String log_msg;
std_msgs__msg__String input_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf_msg;

// ==== mecanum odometry params ====
const uint32_t TICKS_PER_REV = 5000;
const float WHEEL_RADIUS = 0.1;
const float WHEEL_BASE_X = 0.25;
const float WHEEL_BASE_Y = 0.15;
const float ODOM_HZ = 50.0;
const float ODOM_DT = 1.0 / ODOM_HZ;

// odom state
double pose_x = 0, pose_y = 0, pose_theta = 0;
int32_t last_ticks_snapshot[4] = {0,0,0,0};

// ==== Utils ====
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {while(1){delay(1000);}} }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; }

// Logging
void publish_log(const char* fmt, ...) {
    static char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    strncpy(log_msg.data.data, buf, log_msg.data.capacity-1);
    log_msg.data.size = strlen(log_msg.data.data);

    rcl_publish(&log_pub, &log_msg, NULL);
}

// ==== Motor control ====
void set_motor_freq_and_dir(long freqs[4], int dirs[4]) {
    for (int i=0;i<4;i++) last_cmd_dir[i] = dirs[i];

    for (int i=0;i<4;i++) {
        int res;
        if (freqs[i] >= 500000) res = 6;
        else if (freqs[i] >= 250000) res = 7;
        else if (freqs[i] >= 125000) res = 8;
        else if (freqs[i] >= 60000)  res = 9;
        else res = 10;

        ledcSetup(channels[i], freqs[i], res);
        ledcAttachPin(pwm_pins[i], channels[i]);

        if (freqs[i] == 0)
            ledcWrite(channels[i], 0);
        else
            ledcWrite(channels[i], (1 << res) / 2);

        digitalWrite(dir_pins[i], dirs[i]);
    }
}

// ==== freq_cmd callback ====
void freq_cmd_cb(const void * msgin) {
    auto msg = (std_msgs__msg__String*)msgin;

    long f[4]; int d[4];
    if (sscanf(msg->data.data, "SET %ld %d %ld %d %ld %d %ld %d",
               &f[0], &d[0], &f[1], &d[1], &f[2], &d[2], &f[3], &d[3]) == 8) {
        set_motor_freq_and_dir(f,d);
    } else publish_log("Bad cmd: %s", msg->data.data);
}

// ==== IMU timer (10 Hz) ====
void imu_cb(rcl_timer_t*, int64_t) {
    sBmx160SensorData_t a,g,m;
    bmx160.getAllData(&m,&g,&a);

    uint64_t now_ns = (uint64_t)millis()*1000000ULL;
    imu_msg.header.stamp.sec = now_ns/1000000000ULL;
    imu_msg.header.stamp.nanosec = now_ns%1000000000ULL;

    // frame
    strcpy(imu_msg.header.frame_id.data, "imu_link");
    imu_msg.header.frame_id.size = strlen("imu_link");

    imu_msg.angular_velocity.x = g.x;
    imu_msg.angular_velocity.y = g.y;
    imu_msg.angular_velocity.z = g.z;

    imu_msg.linear_acceleration.x = a.x;
    imu_msg.linear_acceleration.y = a.y;
    imu_msg.linear_acceleration.z = a.z;

    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1;

    // Covariance
    for (int i=0;i<9;i++){
        imu_msg.orientation_covariance[i] = -1;
        imu_msg.angular_velocity_covariance[i] = 0.02;
        imu_msg.linear_acceleration_covariance[i] = 0.1;
    }

    rcl_publish(&imu_pub,&imu_msg,NULL);
}

// ==== publish TF (TransformStamped) ====
void publish_tf(double now_ms) {
    uint64_t now_ns = (uint64_t)now_ms * 1000000ULL;
    tf_msg.header.stamp.sec = now_ns/1000000000ULL;
    tf_msg.header.stamp.nanosec = now_ns%1000000000ULL;

    strcpy(tf_msg.header.frame_id.data, "odom");
    tf_msg.header.frame_id.size = strlen("odom");

    strcpy(tf_msg.child_frame_id.data, "base_link");
    tf_msg.child_frame_id.size = strlen("base_link");

    tf_msg.transform.translation.x = pose_x;
    tf_msg.transform.translation.y = pose_y;
    tf_msg.transform.translation.z = 0;

    double h = pose_theta * 0.5;
    tf_msg.transform.rotation.x = 0;
    tf_msg.transform.rotation.y = 0;
    tf_msg.transform.rotation.z = sin(h);
    tf_msg.transform.rotation.w = cos(h);

    rcl_publish(&tf_pub, &tf_msg, NULL);
}

// ==== ODOM timer (50 Hz) ====
void odom_cb(rcl_timer_t*, int64_t) {
    unsigned long now_ms = millis();
    double dt = ODOM_DT;

    // ===== 1. Lấy snapshot encoder =====
    int32_t cur[4];
    noInterrupts();
    for (int i=0;i<4;i++) cur[i] = enc_ticks[i];
    interrupts();

    // ===== 2. Tính delta tick (tick mới trong vòng dt) =====
    int32_t dticks[4];
    for (int i=0;i<4;i++) {
        dticks[i] = cur[i] - last_ticks_snapshot[i];

        // sửa hướng theo lệnh điều khiển (nếu dùng)
        if (last_cmd_dir[i] == 1) dticks[i] = -dticks[i];

        last_ticks_snapshot[i] = cur[i];
    }

    // ===== 3. Tick -> tốc độ góc bánh =====
    double omega[4];
    for (int i=0;i<4;i++) {
        double rev = (double)dticks[i] / (double)TICKS_PER_REV;
        omega[i] = 2.0 * M_PI * rev / dt;   // rad/s
    }

    // ===== 4. Mecanum kinematics =====
    double r = WHEEL_RADIUS;
    double L = WHEEL_BASE_X + WHEEL_BASE_Y;

    double vx = (r/4.0)*(omega[0] + omega[1] + omega[2] + omega[3]);
    double vy = (r/4.0)*(-omega[0] + omega[1] + omega[2] - omega[3]);
    double wz = (r/(4.0*L))*(-omega[0] + omega[1] - omega[2] + omega[3]);

    // ===== 5. Cập nhật pose =====
    double dx = vx * dt;
    double dy = vy * dt;

    pose_x += dx*cos(pose_theta) - dy*sin(pose_theta);
    pose_y += dx*sin(pose_theta) + dy*cos(pose_theta);
    pose_theta += wz * dt;

    // ===== 6. Publish Odom =====
    uint64_t now_ns = (uint64_t)now_ms * 1000000ULL;

    odom_msg.header.stamp.sec = now_ns / 1000000000ULL;
    odom_msg.header.stamp.nanosec = now_ns % 1000000000ULL;

    strcpy(odom_msg.header.frame_id.data, "odom");
    odom_msg.header.frame_id.size = 4;

    strcpy(odom_msg.child_frame_id.data, "base_link");
    odom_msg.child_frame_id.size = 9;

    // pose
    odom_msg.pose.pose.position.x = pose_x;
    odom_msg.pose.pose.position.y = pose_y;

    double hh = pose_theta * 0.5;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sin(hh);
    odom_msg.pose.pose.orientation.w = cos(hh);

    // twist
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = wz;

    rcl_publish(&odom_pub, &odom_msg, NULL);

    // ===== 7. Publish TF =====
    publish_tf(now_ms);
}


// ==== SETUP ====
void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!bmx160.begin()) while(1);

    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support,0,NULL,&allocator));
    RCCHECK(rclc_node_init_default(&node,"robot_node","",&support));

    // publishers
    RCCHECK(rclc_publisher_init_default(
        &imu_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),
        "/imu/raw"));

    RCCHECK(rclc_publisher_init_default(
        &odom_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),
        "/odom"));

    RCCHECK(rclc_publisher_init_default(
        &tf_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,TransformStamped),
        "/tf"));

    RCCHECK(rclc_publisher_init_default(
        &log_pub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,String),
        "output_log"));

    // subscriber
    RCCHECK(rclc_subscription_init_default(
        &freq_sub,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,String),
        "freq_cmd"));

    // allocate strings
    input_msg.data.data = (char*)malloc(128);
    input_msg.data.capacity = 128;
    input_msg.data.size = 0;

    log_msg.data.data = (char*)malloc(128);
    log_msg.data.capacity = 128;
    log_msg.data.size = 0;

    imu_msg.header.frame_id.data = (char*)malloc(32);
    imu_msg.header.frame_id.capacity = 32;
    imu_msg.header.frame_id.size = 0;

    odom_msg.header.frame_id.data = (char*)malloc(32);
    odom_msg.header.frame_id.capacity = 32;
    odom_msg.header.frame_id.size = 0;
    odom_msg.child_frame_id.data = (char*)malloc(32);
    odom_msg.child_frame_id.capacity = 32;
    odom_msg.child_frame_id.size = 0;

    tf_msg.header.frame_id.data = (char*)malloc(32);
    tf_msg.header.frame_id.capacity = 32;
    tf_msg.child_frame_id.data = (char*)malloc(32);
    tf_msg.child_frame_id.capacity = 32;

    // timers
    RCCHECK(rclc_timer_init_default(
        &imu_timer,&support,
        RCL_MS_TO_NS(100), imu_cb)); // 10 Hz

    RCCHECK(rclc_timer_init_default(
        &odom_timer,&support,
        RCL_MS_TO_NS((int)(1000/ODOM_HZ)), odom_cb)); // 50 Hz

    // executor
    RCCHECK(rclc_executor_init(&executor,&support.context,4,&allocator));
    RCCHECK(rclc_executor_add_subscription(&executor,&freq_sub,&input_msg,freq_cmd_cb,ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor,&imu_timer));
    RCCHECK(rclc_executor_add_timer(&executor,&odom_timer));

    // motor pin init
    for(int i=0;i<4;i++){
        pinMode(dir_pins[i],OUTPUT);
        digitalWrite(dir_pins[i],LOW);
    }

    // encoder pin init
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(36, INPUT_PULLUP);
    pinMode(39, INPUT_PULLUP);

    attachInterrupt(34,enc_isr_0,CHANGE);
    attachInterrupt(35,enc_isr_1,CHANGE);
    attachInterrupt(36,enc_isr_2,CHANGE);
    attachInterrupt(39,enc_isr_3,CHANGE);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

}

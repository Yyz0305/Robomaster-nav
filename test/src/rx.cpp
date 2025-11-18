#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void tactics(MoveBaseClient& ac,uint8_t flag,uint16_t hp,ros::Publisher& pub_flag);
void tactics1(MoveBaseClient& ac,uint8_t flag,uint16_t hp,ros::Publisher& pub_flag);
void move(MoveBaseClient& ac,float x,float y);
int  wxm = 0;
void readSerialData(MoveBaseClient& ac,serial::Serial& ser, ros::Publisher& pub_flag, ros::Publisher& pub_hp,ros::Publisher& pub_wxm) {
    static std::vector<uint8_t> buffer;
    while (ser.available()) {
        uint8_t byte = ser.read(1)[0];
        if (byte == 0xFF && buffer.empty()) { // New frame start
            buffer.push_back(byte);
            ROS_DEBUG("Frame start detected");
        } else if (!buffer.empty()) {
            buffer.push_back(byte);
            if (byte == 0xFE && buffer.size() == 15) { // Frame end
                if (buffer[0] == 0xFF && buffer[14] == 0xFE) {
                    // Extract flag and HP
                    uint8_t flag = buffer[9];
                    uint16_t hp = buffer[10]<<8|buffer[11];


                    std_msgs::UInt8 msg_hp;
                    msg_hp.data = hp;
                    pub_hp.publish(msg_hp);
                    

                    // Print flag and HP
                    // ROS_INFO("Flag: 0x%02X", flag);
                    ROS_WARN("HP: %d", hp);
                    tactics1(ac,flag,hp,pub_flag);

                    std_msgs::UInt8 msg_wxm;
                    msg_wxm.data = wxm;
                    pub_wxm.publish(msg_wxm);
                } else {
                    ROS_WARN("Invalid frame detected");
                }
                buffer.clear();
            } else if (buffer.size() > 15) {
                // ROS_WARN("Buffer overflow detected, clearing buffer");
                buffer.clear();
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_reader");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base",true);

    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Wait For Move_base");

    // Initialize serial port
    serial::Serial ser;
    try {
        ser.setPort("/dev/ser_rx"); // Change this to your device name
        ser.setBaudrate(115200); // Set your baud rate here
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        ROS_ERROR_STREAM("Failed to open serial port");
        return -1;
    }

    ros::Publisher pub_flag = nh.advertise<std_msgs::UInt8>("serial_flag", 1000);
    ros::Publisher pub_hp = nh.advertise<std_msgs::UInt8>("serial_hp", 1000);
    ros::Publisher pub_wxm = nh.advertise<std_msgs::UInt8>("serial_wxm", 1000);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        readSerialData(ac,ser, pub_flag, pub_hp,pub_wxm);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}
void move(MoveBaseClient& ac,float x,float y){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";  
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
}
void send(ros::Publisher& pub_flag,int z){
    std_msgs::UInt8 msg_flag;
    msg_flag.data = z;
    pub_flag.publish(msg_flag);
}
bool get_naving_status(MoveBaseClient& ac){
    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_WARN("succeed");
        //send(pub_flag,0);
        return true;
    }
    else{
        ROS_WARN("Mission failed");
        return false;
    } 
}

int nav_time = 0;
void tactics(MoveBaseClient& ac,uint8_t flag,uint16_t hp,ros::Publisher& pub_flag){  
    static int task_flag = 1;
    ROS_INFO("flag %d", wxm);
    if(flag==4&&wxm == 0){//状态0：开局抢点
        send(pub_flag,1);
        move(ac,4.0,0);//靠边
        move(ac,4.0,3.5);//抢点
        wxm = 1;  //退出条件：抢点完成 行为：wxm=1
    }else if(wxm == 1){//状态1：抢点后自瞄
        send(pub_flag,0);
        if(hp<320){//退出条件：执行完成 行为：wxm=2
            wxm = 2;
        }
    }else if(wxm == 2){//状态2：抢点血量不够，靠边
        send(pub_flag,1);
        nav_time = 0;
        do{
            move(ac,4.0,0);//靠边
            nav_time++;
        }while(get_naving_status(ac)==false&&nav_time<=2);
        
        wxm = 3;
    }else if(wxm == 3){
        send(pub_flag,0);
        if(hp<240){
            wxm = 4;
        }
    }else if(wxm == 4){
        send(pub_flag,1);
        nav_time = 0;
        // move(ac,0,0);//需要重启
        do{
            move(ac,0,0);//靠边
            nav_time++;
        }while(get_naving_status(ac)==false&&nav_time<=2);

        if(hp>390){
            wxm = 2;
        }
    }
}
void tactics1(MoveBaseClient& ac,uint8_t flag,uint16_t hp,ros::Publisher& pub_flag){
    ROS_INFO("flag:%d", flag);
    if(flag==4&&wxm==0){
        send(pub_flag,1);
        move(ac,4.0,0);
        move(ac,4.0,3.5);
        wxm = 1;  
    }
    if(wxm==1){
        send(pub_flag,0);
        if(hp<320) wxm=2;
    }
    if(wxm==2){
        send(pub_flag,1);
        move(ac,4.0,0);
        wxm=3;
    }
    if(wxm==3){
        send(pub_flag,0);
        if(hp<240) wxm=4;
    }
    if(wxm==4){
        send(pub_flag,1);
        move(ac,0,0);
        wxm=5;
    }
    if(wxm==5){
        send(pub_flag,0);
        if(hp>380) wxm=0;
    }
}

    // if(task_flag==1&&flag==1){//flag:开始  task_flag：任务，1：
    //     send(pub_flag,1);
    //     if(wxm==0){
    //         move(ac,4.9,0);
    //         move(ac,5.0,3.8);
    //     // move(ac,4.8,2.8);
    //     }
    //     if(aim(ac,pub_flag)) wxm=1;
    //     if(hp<320){ wxm==2;task_flag=0;}
    // }
    // if(hp>240&&task_flag!=2&&flag==1&&wxm==2){
    //     send(pub_flag,1);
    //     move(ac,5.0,0);
    //     if(aim(ac,pub_flag)) task_flag=2;
    //     else task_flag=2;
    // }
    // else if(hp < 240 && task_flag!=3&&flag==1&&wxm==2){
    //     send(pub_flag,1);
    //     move(ac,0,0);
    //     // move(ac,1,1);
    //     if(aim(ac,pub_flag)) task_flag=3;
    //     else task_flag=0;
    // }

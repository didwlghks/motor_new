#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <my_msgs/CameraData.h>
#include <my_msgs/SensorData.h>
#include <my_msgs/SignalData.h>
#include <iostream>
#include <fstream>

////////////////////////////////////
#define X_BOUND_1 260
#define X_BOUND_2 380       // sense where person is
#define BOUND_MID 320 
#define BIG_SIZE_P 500      // size of person
#define BIG_SIZE_S 250      // size of sign
#define SENSOR_LIMIT 50     // distance, need to interation with person
///////////////////////////////////// define value

/////////////////////////////
bool person_detect = false;
bool sign_detect = false;
float person_x = 0;
float person_size = 0;
float sign_x = 0;
float sign_size = 0;
/////////////////////////////// cam data(need to make more sign data)

///////////////////////////
float FrontSensor = 0;
float LeftSensor = 0;
float RightSensor = 0;
float BackSensor = 0;
////////////////////////// sensor data

///////////////
int state = 0; // display data
///////////////
int sequence = 0; // sequence 0 to 3

void Text_Input(void)
{
    int i = 0;
    std::size_t found;
    std::ifstream inFile;
    inFile.open("/home/ubuntu/catkin_ws/src/motor_node/motor_input.txt");
    for (std::string line; std::getline(inFile, line);)
    {
        found = line.find("=");

        switch (i)
        {
        case 0: PWM_range = atof(line.substr(found + 2).c_str()); break;
        case 1: PWM_frequency = atof(line.substr(found + 2).c_str()); break;
        case 2: PWM_limit = atof(line.substr(found + 2).c_str()); break;
        case 3: Control_cycle = atof(line.substr(found + 2).c_str()); break;
        case 4: Acceleration_ratio = atof(line.substr(found + 2).c_str()); break;
        case 5: Wheel_radius = atof(line.substr(found + 2).c_str()); break;
        case 6: Robot_radius = atof(line.substr(found + 2).c_str()); break;
        case 7: Encoder_resolution = atof(line.substr(found + 2).c_str()); break;
            //case :  = atof(line.substr(found+2).c_str()); break;
        }
        i += 1;
    }
    inFile.close();
}
int Motor_Setup(void)
{
    pinum = pigpio_start(NULL, NULL);

    if (pinum < 0)
    {
        ROS_INFO("Setup failed");
        ROS_INFO("pinum is %d", pinum);
        return 1;
    }

    set_mode(pinum, motor1_DIR, PI_OUTPUT);
    set_mode(pinum, motor2_DIR, PI_OUTPUT);
    set_mode(pinum, motor1_PWM, PI_OUTPUT);
    set_mode(pinum, motor2_PWM, PI_OUTPUT);
    set_mode(pinum, motor1_ENA, PI_INPUT);
    set_mode(pinum, motor1_ENB, PI_INPUT);
    set_mode(pinum, motor2_ENA, PI_INPUT);
    set_mode(pinum, motor2_ENB, PI_INPUT);

    gpio_write(pinum, motor1_DIR, PI_LOW);
    gpio_write(pinum, motor2_DIR, PI_LOW);

    set_PWM_range(pinum, motor1_PWM, PWM_range);
    set_PWM_range(pinum, motor2_PWM, PWM_range);
    set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
    set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
    set_PWM_dutycycle(pinum, motor1_PWM, 0);
    set_PWM_dutycycle(pinum, motor1_PWM, 0);

    set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

    current_PWM1 = 0;
    current_PWM2 = 0;

    current_Direction1 = true;
    current_Direction2 = true;

    acceleration = PWM_limit / (Acceleration_ratio);

    ROS_INFO("Setup Fin");
    return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A++;
    else EncoderCounter1A--;
    EncoderSpeedCounter1++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B++;
    else EncoderCounter1B--;
    EncoderSpeedCounter1++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A--;
    else EncoderCounter2A++;
    EncoderSpeedCounter2++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B--;
    else EncoderCounter2B++;
    EncoderSpeedCounter2++;
}
int Motor1_Encoder_Sum()
{
    EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
    return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
    EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
    return EncoderCounter2;
}
void Init_Encoder(void)
{
    EncoderCounter1 = 0;
    EncoderCounter2 = 0;
    EncoderCounter1A = 0;
    EncoderCounter1B = 0;
    EncoderCounter2A = 0;
    EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
    Text_Input();
    Motor_Setup();
    Init_Encoder();
    Interrupt_Setting();

    Wheel_round = 2 * PI * Wheel_radius;
    Robot_round = 2 * PI * Robot_radius;

    switch_direction = true;
    Theta_Distance_Flag = 0;

    ROS_INFO("PWM_range %d", PWM_range);
    ROS_INFO("PWM_frequency %d", PWM_frequency);
    ROS_INFO("PWM_limit %d", PWM_limit);
    ROS_INFO("Control_cycle %f", Control_cycle);
    ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
    ROS_INFO("Initialize Complete");

    printf("\033[2J");
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
    int local_PWM = Limit_Function(pwm);

    if (motor_num == 1)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor1_DIR, PI_LOW);
            set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
            current_PWM1 = local_PWM;
            current_Direction1 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor1_DIR, PI_HIGH);
            set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
            current_PWM1 = local_PWM;
            current_Direction1 = false;
        }
    }

    else if (motor_num == 2)
    {
        if (direction == true)
        {
            gpio_write(pinum, motor2_DIR, PI_LOW);
            set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
            current_PWM2 = local_PWM;
            current_Direction2 = true;
        }
        else if (direction == false)
        {
            gpio_write(pinum, motor2_DIR, PI_HIGH);
            set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
            current_PWM2 = local_PWM;
            current_Direction2 = false;
        }
    }
}

int Limit_Function(int pwm)
{
    int output;
    if (pwm > PWM_limit * 2)
    {
        output = PWM_limit;
        ROS_WARN("PWM too fast!!!");
    }
    else if (pwm > PWM_limit)output = PWM_limit;
    else if (pwm < 0)
    {
        output = 0;
        ROS_WARN("trash value!!!");
    }
    else output = pwm;
    return output;
}
void RPM_Calculator()
{
    RPM_Value1 = (EncoderSpeedCounter1 * (60 * Control_cycle)) / (Encoder_resolution * 4);
    EncoderSpeedCounter1 = 0;
    RPM_Value2 = (EncoderSpeedCounter2 * (60 * Control_cycle)) / (Encoder_resolution * 4);
    EncoderSpeedCounter2 = 0;
}
void Motor_View()
{
    RPM_Calculator();
    printf("\033[2J");
    printf("\033[1;1H");
    printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
    printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
    printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
    printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
    printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
    printf("Acc  :%10.0d\n", acceleration);
    printf("\n");
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////

void CamDataCallback(const my_msgs::CameraData& msg) { // function which subscribes from camera
    person_detect = msg.msg_person;
    person_x = msg.msg_pcx;
    person_size = msg.msg_psize;
    sign_detect = msg.msg_sign;
    sign_x = msg.msg_scx;
    sign_size = msg.msg_ssize;
}

void SensorDataCallback(const my_msgs::SensorData& msg) {
    FrontSensor = msg.front;
    LeftSensor = msg.left;
    RightSensor = msg.right;
    BackSensor = msg.back;
}


//////////////////////////////////
/*
void Ultra_Sensor(){
        // make ultra sensor function for approaching people
        // if person is nearby, stop and output voice
}
*/
//////////////////////////////////


void SignalDataCallback(const my_msgs::SignalData& msg) {
    state = msg.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    Initialize();
    ros::Publisher signal_pub = nh.advertise<my_msgs::SignalData>("/signal/topic", 10);
    ros::Subscriber signal_sub = nh.subscribe("/signal/topic2", 10, SignalDataCallback);
    ros::Subscriber sensor_sub = nh.subscribe("/sensor/topic", 10, SensorDataCallback);
    ros::Subscriber camera_sub = nh.subscribe("/camera/topic", 10, CamDataCallback); //camera topic sub
    ros::Rate loop_rate(Control_cycle);
    my_msgs::SignalData signal_msg;

    while (ros::ok())
    {

        if (sequence == 0)
        {
            camera_sub;
            sensor_sub; //subscribe camera and sensor data

            if (person_detect) {        // person detected
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                sequence++;    // next level
            }
            Motor_Controller(1, false, 70);
            Motor_Controller(2, true, 70);
        }

        if (sequence == 1)
        {
            camera_sub;
            sensor_sub;
            if (FrontSensor < SENSOR_LIMIT && person_size > BIG_SIZE) {      // Frontsensor on && size_person is big enough
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                signal_msg.data = 1 // when person is in front of robot, pub 1 to display node
                    signal_pub.publish(signal_msg);

                person_detect = 0;
                person_x = 0;
                person_size = 0; // initialize camera data
                sequence++;    // next level
            }

            else
            {
                if (person_x > X_BOUND_1 && person_x < X_BOUND_2) {
                    Motor_Controller(1, false, 70);
                    Motor_Controller(2, true, 70);
                }
                else if (person_x <= X_BOUND_1) {
                    Motor_Controller(1, false, 55);
                    Motor_Controller(2, true, 70);
                    }
                else {
                    Motor_Controller(1, false, 70);
                    Motor_Controller(2, true, 55);
                }
            }
        }
        if (sequence == 2)     // Screen interaction  ���߿� ���ֵ� ��.
        {
            signal_sub;
            if (state == 9999) // 9999 = display's end_signal
            {
                sequence++;; // next level
            }
        }
        if (sequence == 3)
        {
            camera_sub;
            sensor_sub;
            if (sign_detect)     // sign detected
            {
                if (x_sign == BOUND_MID)    // put sign in center of cam
                {
                        Motor_Controller(1, true, 0);
                        Motor_Controller(2, true, 0);
                        person_detect = 0;
                        person_x = 0;
                        person_size = 0; // init camera data
                        sequence = 0; // init sequence
                }
                else
                {
                    Motor_Controller(1, true, 20);
                    Motor_Controller(2, true, 20);     // turn more to put in the center of cam
                }
            }
            else
            {
                Motor_Controller(1, true, 30);
                Motor_Controller(2, true, 30);     // turn ccw before detecting 
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    return 0;
}


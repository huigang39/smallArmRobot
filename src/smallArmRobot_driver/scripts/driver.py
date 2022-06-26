#!/usr/bin/env python
# coding:utf-8

# 导入rospy库
import rospy

# joint_state的msg是属于sensor
from sensor_msgs.msg import JointState

# 调用StandardFirmata协议
from pyfirmata import ArduinoMega, util

# 导入时间函数
import time

# 导入IO配置函数，自定义
from IO_config import *

# 步进电机驱动引脚声明
Joint_STEP = [Joint1_STEP, Joint2_STEP, Joint3_STEP,
              Joint4_STEP, Joint5_STEP, Joint6_STEP]  # 脉冲引脚声明
Joint_DIR = [Joint1_DIR, Joint2_DIR, Joint3_DIR,
             Joint4_DIR, Joint5_DIR, Joint6_DIR]  # 方向引脚声明
Joint_EN = [Joint1_EN, Joint2_EN, Joint3_EN,
            Joint4_EN, Joint5_EN, Joint6_EN]  # 使能驱动引脚声明

# 减速比系数声明，这里有减速带，需要重新计算
joint_pro = [joint1_pro, joint2_pro, joint3_pro,
             joint4_pro, joint5_pro, joint6_pro]

# 标记量声明
joint_pul_flag = [joint1_pul_flag, joint2_pul_flag, joint3_pul_flag,
                  joint4_pul_flag, joint5_pul_flag, joint6_pul_flag]  # 脉冲标记
joint_dir_flag = [joint1_dir_flag, joint2_dir_flag, joint3_dir_flag,
                  joint4_dir_flag, joint5_dir_flag, joint6_dir_flag]  # 方向标记
joint_value = [joint1_value, joint2_value, joint3_value,
               joint4_value, joint5_value, joint6_value]  # 当前角度值

# 脉冲限位
joint_min_pul = [joint1_min_pul, joint2_min_pul, joint3_min_pul,
                 joint4_min_pul, joint5_min_pul, joint6_min_pul]  # 最小脉冲值
joint_max_pul = [joint1_max_pul, joint2_max_pul, joint3_max_pul,
                 joint4_max_pul, joint5_max_pul, joint6_max_pul]  # 最大脉冲值

# 通过StandardFirmata协议来实例化一个ArduinoMega的对象board，传入参数端口号和波特率，默认8N1模式
board = ArduinoMega("/dev/ttyACM0", baudrate=57600)

# 使能驱动处于低电平
board.digital[Joint1_EN].write(0)
board.digital[Joint2_EN].write(0)
board.digital[Joint3_EN].write(0)
board.digital[Joint4_EN].write(0)
board.digital[Joint5_EN].write(0)
board.digital[Joint6_EN].write(0)

'''
函数名称：value_driver
函数功能：控制对应的关节旋转到指定的角度值
输入参数：joint为关节编号，angle为该关节目标角度值
'''


def value_driver(joint, angle):
    # 如果当前的脉冲迭代超出可运行范围，步进电机不执行，输出提示信息。这里是为了避免过转导致机械臂卡住
    if joint_pul_flag[joint] < joint_min_pul[joint] - 30 or joint_pul_flag[joint] > joint_max_pul[joint] + 30:
        print("ERROR.Beyond the limit...")
    else:
        diff_value = 0  # 角度差值变量声明
        step = 0  # 需要的部署变量声明

        # 如果目标角度angle小于当前角度值
        if angle < joint_value[joint]:
            joint_dir_flag[joint] = 0  # 对应关节方向标记为0
            board.digital[Joint_DIR[joint]].write(0)  # 方向引脚为低电平
            diff_value = joint_value[joint] - angle  # 计算角度差值
            print("角度值减小.")  # 输出提示信息
        else:
            joint_dir_flag[joint] = 1  # 对应关节方向标记为1
            board.digital[Joint_DIR[joint]].write(1)  # 方向引脚为高电平
            diff_value = angle - joint_value[joint]  # 计算角度差值
            print("角度值增加.")  # 输出提示信息
        print("角度差值为：" + str(diff_value))  # 输出计算好的角度差值

        step = int(diff_value/joint_pro[joint])  # 步数=角度值/减速比
        print("需要脉冲数量：" + str(step) + ".")  # 输出提示信息，需要的脉冲数量

        # 发送指定数量的脉冲进行驱动执行
        while step > 0:
            board.digital[Joint_STEP[joint]].write(1)
            time.sleep(0.0001)
            board.digital[Joint_STEP[joint]].write(0)
            time.sleep(0.0001)
            step = step - 1
        print("脉冲执行完毕.")  # 输出提示信息

        step = int(diff_value/joint_pro[joint])
        if joint_dir_flag[joint] == 0:
            joint_pul_flag[joint] = joint_pul_flag[joint] - step  # 更新当前脉冲标识
        else:
            joint_pul_flag[joint] = joint_pul_flag[joint] + step
        # 输出提示信息，脉冲累计量
        print("脉冲计数迭代完毕.当前脉冲数为" + str(joint_pul_flag[joint]) + ".")

        joint_value[joint] = angle  # 角度值迭代
        print("角度迭代完毕.当前角度为" + str(joint_value[joint]) + ".")  # 输出提示信息，当前角度值


'''
函数名称：callback
函数功能：作为回调函数执行
'''


def callback(data):
    joint6_angle = data.position[0]*360/6.28+90  # 对应的关节弧度值转角度值值转
    # 判断当前角度值是否越界
    if joint6_angle < 0:
        # 如果当前角度值小于0，则角度值为0，避免出现负数
        joint6_angle = 0
    elif joint6_angle > 180:
        # 如果当前角度值大于180，则角度值为180，避免出现角度超出
        joint6_angle = 180
    rospy.loginfo(rospy.get_caller_id() + ']--->Joint6 Angle :%d',
                  joint6_angle)  # ros下输出提示信息
    value_driver(5, joint6_angle)  # 角度值执行

    # 下同
    joint5_angle = data.position[1]*360/6.28+90
    if joint5_angle < 0:
        joint5_angle = 0
    elif joint5_angle > 180:
        joint5_angle = 180
    rospy.loginfo(rospy.get_caller_id() +
                  ']--->Joint5 Angle :%d', joint5_angle)
    value_driver(4, joint5_angle)

    joint4_angle = data.position[2]*360/6.28+90
    if joint4_angle < 0:
        joint4_angle = 0
    elif joint4_angle > 180:
        joint4_angle = 180
    rospy.loginfo(rospy.get_caller_id() +
                  ']--->Joint4 Angle :%d', joint4_angle)
    value_driver(3, joint4_angle)

    joint3_angle = data.position[3]*360/6.28+90
    if joint3_angle < 0:
        joint3_angle = 0
    elif joint3_angle > 180:
        joint3_angle = 180
    rospy.loginfo(rospy.get_caller_id() +
                  ']--->Joint3 Angle :%d', joint3_angle)
    value_driver(2, joint3_angle)

    joint2_angle = data.position[4]*360/6.28+90
    if joint2_angle < 0:
        joint2_angle = 0
    elif joint2_angle > 180:
        joint2_angle = 180
    rospy.loginfo(rospy.get_caller_id() +
                  ']--->Joint2 Angle :%d', joint2_angle)
    value_driver(1, joint2_angle)

    joint1_angle = data.position[5]*360/6.28+90
    if joint1_angle < 0:
        joint1_angle = 0
    elif joint1_angle > 180:
        joint1_angle = 180
    rospy.loginfo(rospy.get_caller_id() +
                  ']--->Joint1 Angle :%d', joint1_angle)
    value_driver(0, joint1_angle)


def driver():
    # 初始化节点，命名为SmallArmRobot_Driver
    rospy.init_node('SmallArmRobot_Driver', anonymous=True)
    # 订阅joint_states话题，类型为JointState，当订阅到该话题执行callback函数
    rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    driver()
board.exit()

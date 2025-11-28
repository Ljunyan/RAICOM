#!/usr/bin/env python3
# -*- coding: utf-8 -*
 
import  os
import  sys
import  tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)

# global can_grasp
# global can_release

# def grasp_status_cp(msg):
#     global can_release,can_grasp
#     # 物体抓取成功,让机器人回起始点
#     if msg.data=='1':
#         can_release=True
#     if msg.data=='0' or msg.data=='-1':
#         can_grasp=True
# grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=1)

def keyboardLoop():
    rospy.init_node('teleop')
    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 10))
 
    #速度变量
    # 慢速
    walk_vel_ = rospy.get_param('walk_vel', 0.1)
    # 快速
    run_vel_ = rospy.get_param('run_vel', 1.0)
    yaw_rate_ = rospy.get_param('yaw_rate', 0.5)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.0)
    # walk_vel_前后速度
    max_tv = walk_vel_
    # yaw_rate_旋转速度
    max_rv = yaw_rate_
    # 参数初始化
    speed=0
    # global can_release,can_grasp
    # can_grasp=True
    # can_release=False
    
    print ("使用[WASD]控制机器人")
    
    print ("按[h/H]准备抓取")
    print ("按[u/U]抓取 and 第一L")
    print ("按[i/I]抬到第二L")
    print ("按[o/O]抬到第三L")
    print ("按[j/J]放下物体到第一L")
    print ("按[k/K]放下物体到第二L")
    print ("按[l/L]放下物体到第三L")
    print("按[z/Z]放左下推物块")
    print("按[c/C]放右下推物块")
    print ("按[m/M]秘密武器")
    print ("按[[/{]left rotate 180")
    print ("按[]/}]right rotate 180")
    print ("按[b/B]Reset")
    print ("按[n/N]Home")
    print ("按[q]退出" )
 
    #读取按键循环
    while not rospy.is_shutdown():
        # linux下读取键盘按键
        fd = sys.stdin.fileno()
        turn =0
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # ch代表获取的键盘按键
        if ch == 'u' or ch == 'U':
            # if can_grasp:
            msg=String()
            msg.data='7'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'h' or ch == 'H':
            # if can_grasp:
            msg=String()
            msg.data='6'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == ']' or ch == '}':
            # if can_grasp:
            speed = 0
            turn = -20
            # msg=String()
            # msg.data='99'
            # grasp_pub.publish(msg)
        elif ch == '[' or ch == '{':
            # if can_grasp:
            speed = 0
            turn = 20
        elif ch == 'j' or ch == 'J':#第一层
            # if can_release:
            msg=String()
            msg.data='1'
            grasp_pub.publish(msg)
            # can_release=False
            speed = 0
            turn = 0
        elif ch == 'k' or ch == 'K':#第二层
            # if can_grasp:
            msg=String()
            msg.data='2'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'l' or ch == 'L':#第三层
            # if can_grasp:
            msg=String()
            msg.data='3'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'm' or ch == 'M':#第4层
            # if can_grasp:
            msg=String()
            msg.data='5'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'b' or ch == 'B':#reset
            # if can_grasp:
            msg=String()
            msg.data='8'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'n' or ch == 'N':#home
            # if can_grasp:
            msg=String()
            msg.data='9'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'i' or ch == 'I':#抬第2层
            # if can_grasp:
            msg=String()
            msg.data='10'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'o' or ch == 'O':#抬第3层
            # if can_grasp:
            msg=String()
            msg.data='11'
            grasp_pub.publish(msg)
            # can_grasp=False
            speed = 0
            turn = 0
        elif ch == 'z' or ch == 'Z':# 左下方推物块
            # if can_grasp:
            msg=String()
            msg.data='50'
            grasp_pub.publish(msg)
        elif ch == 'c' or ch == 'C':# 右下方推物块
            # if can_grasp:
            msg=String()
            msg.data='51'
            grasp_pub.publish(msg)

        elif ch == 'w':
            max_tv = walk_vel_
            speed = 1
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            turn = 0
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            turn = 0.4
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            turn = -0.4
        elif ch == 'W':
            max_tv = run_vel_
            speed = 1.5
            turn = 0
        elif ch == 'S':
            max_tv = run_vel_
            speed = -1.5
            turn = 0
        elif ch == 'A':
            max_rv = yaw_rate_run_
            speed = 0
            turn = 1.5
        elif ch == 'D':
            max_rv = yaw_rate_run_
            speed = 0
            turn = -1.5
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()
		#停止机器人
        #stop_robot()
 
def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)
 
if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass


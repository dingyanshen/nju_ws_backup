#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import time
import rospy
import Action

if __name__ == '__main__':
    dobot = Action.Dobot()
    # dobot.setHome()
    dobot.setPTPCommonParams(1000.0, 1000.0)
    dobot.setPTPCoordinateParams(1000, 1000, 200, 200)
    dobot.clearAlarmsState()
    dobot.setIOMultiplexing(8, 2)
    dobot.setPose(250,0,0,0)
    rospy.sleep(0.1)
    dobot.setTheta(0)

    while True:
        choice = raw_input("请输入选择 p末端 t吸盘 s吸取 l释放 a解警 r复位 c1抓下层 c2抓上层 q退出：")
        if choice == "p":
            print("请输入末端位置坐标（空格隔开）：")
            pose = raw_input()
            try:
                x, y, z = map(float, pose.split())
            except ValueError:
                print("输入无效，请输入三个数字")
                continue
            r = 0
            dobot.setPose(x, y, z, r)
            print('末端位置设置完成')
        elif choice == "t":
            theta = float(raw_input("请输入舵机角度(0-1)："))
            dobot.setTheta(theta)
            print('舵机角度设置完成')
        elif choice == "s":
            dobot.suckupObject()
            print('吸取快递盒完成')
        elif choice == "l":
            dobot.releaseObject()
            print('释放快递盒完成')
        elif choice == "a":
            dobot.clearAlarmsState()
            print('解除警报完成')
        elif choice == "r":
            dobot.setHome()
            print('复位完成')
        elif choice == "c1":
            print("请输入抓取下层快递盒的误差（x y）：")
            error = raw_input()
            try:
                error_x, error_y = map(float, error.split())
            except ValueError:
                print("输入无效，请输入两个数字")
                continue
            dobot.CatchDOWN(-10, error_x, error_y)
            print('抓取下层快递盒完成')
        elif choice == "c2":
            print("请输入抓取上层快递盒的误差（x y）：")
            error = raw_input()
            try:
                error_x, error_y = map(float, error.split())
            except ValueError:
                print("输入无效，请输入两个数字")
                continue
            dobot.CatchUP(-10, error_x, error_y)
            print('抓取上层快递盒完成')
        elif choice == "q":
            break
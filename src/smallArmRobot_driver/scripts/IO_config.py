# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 19:37:56 2020

@author: 嘉
"""

''''''''''''''''''''''''
'Arduino Mega引脚IO声明'
''''''''''''''''''''''''
Joint1_DIR = 51
Joint1_STEP = 50
Joint1_MS3 = 49
Joint1_MS2 = 48
Joint1_MS1 = 47
Joint1_EN = 46

Joint2_DIR = 45
Joint2_STEP = 44
Joint2_MS3 = 43
Joint2_MS2 = 42
Joint2_MS1 = 41
Joint2_EN = 40

Joint3_EN = 39
Joint3_MS1 = 38
Joint3_MS2 = 37
Joint3_MS3 = 36
Joint3_STEP = 35
Joint3_DIR = 34

Joint4_EN = 32
Joint4_DIR = 31
Joint4_STEP = 30

Joint5_EN = 28
Joint5_DIR = 27
Joint5_STEP = 26

Joint6_EN = 24
Joint6_DIR = 23
Joint6_STEP = 22

''''''''''''''
'减速比例系数'
''''''''''''''
joint1_pro = 1.8
joint2_pro = 0.8
joint3_pro = 0.6
joint4_pro = 0.375
joint5_pro = 0.42
joint6_pro = 0.375

''''''''''''''
'标记变量声明'
''''''''''''''
joint1_pul_flag = 0
joint1_dir_flag = 0
joint1_value = 0

joint2_pul_flag = 105
joint2_dir_flag = 0
joint2_value = 90

joint3_pul_flag = 150
joint3_dir_flag = 0
joint3_value = 90

joint4_pul_flag = 230
joint4_dir_flag = 0
joint4_value = 90

joint5_pul_flag = 210
joint5_dir_flag = 0
joint5_value = 90

joint6_pul_flag = 240
joint6_dir_flag = 0
joint6_value = 90

''''''''''''''
'限位标记声明'
''''''''''''''
joint1_min_pul = 0
joint1_max_pul = 200

joint2_min_pul = 0
joint2_max_pul = 210

joint3_min_pul = 0
joint3_max_pul = 300

joint4_min_pul = 0
joint4_max_pul = 460

joint5_min_pul = 0
joint5_max_pul = 420

joint6_min_pul = 0
joint6_max_pul = 480
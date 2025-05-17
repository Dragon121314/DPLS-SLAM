#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
# COCO类别名称
CLASS_NAMES = ["人", "自行车", "汽车", "摩托车", "飞机", "公交车", "火车", "卡车", "船", 
              "交通灯", "消防栓", "停止标志", "停车计时器", "长椅", "鸟", "猫", "狗", 
              "马", "羊", "牛", "大象", "熊", "斑马", "长颈鹿", "背包", "雨伞", "手提包", 
              "领带", "行李箱", "飞盘", "滑雪板", "滑雪板", "运动球", "风筝", "棒球棒", 
              "棒球手套", "滑板", "冲浪板", "网球拍", "瓶子", "红酒杯", "杯子", "叉子", 
              "刀", "勺子", "碗", "香蕉", "苹果", "三明治", "橙子", "西兰花", "胡萝卜", 
              "热狗", "披萨", "甜甜圈", "蛋糕", "椅子", "沙发", "盆栽植物", "床", "餐桌", 
              "马桶", "电视", "笔记本电脑", "鼠标", "遥控器", "键盘", "手机", "微波炉", 
              "烤箱", "烤面包机", "水槽", "冰箱", "书", "时钟", "花瓶", "剪刀", "泰迪熊", 
              "吹风机", "牙刷"]

# 颜色生成函数
def get_color(class_id):
    colors = [
        [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0],
        [0, 255, 255], [255, 0, 255], [128, 0, 0], [0, 128, 0],
        [0, 0, 128], [128, 128, 0], [0, 128, 128], [128, 0, 128]
    ]
    return colors[class_id % len(colors)]

# 图像处理回调函数
def image_callback(msg, args): 
    bridge, model, mask_pub, stats = args
    try:
        # 转换ROS图像为OpenCV格式
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 执行YOLO推理
        start_time=time.time()
        results = model.predict(cv_img, device='cpu', augment=False, verbose=False)
        end_time=time.time()
        stats['total_time']+=(end_time-start_time)
        stats['count'] +=1
        # 创建结果图像
        vis_img = cv_img.copy()
        mask_img = np.zeros_like(cv_img)
        
        # 处理检测结果
        for result in results:
            if result.masks is not None:
                for i, mask in enumerate(result.masks.data):
                    # 处理掩膜
                    resized_mask = cv2.resize(mask.numpy(), (cv_img.shape[1], cv_img.shape[0]))
                    color = get_color(i)
                    mask_img[resized_mask > 0] = color
                    
                    # 绘制边界框
                    x1, y1, x2, y2 = map(int, result.boxes.xyxy[i])
                    cv2.rectangle(vis_img, (x1, y1), (x2, y2), color, 2)
                    
                    # 添加标签
                    cls_id = int(result.boxes.cls[i])
                    label = f"{CLASS_NAMES[cls_id]}:{result.boxes.conf[i]:.2f}"
                    cv2.putText(vis_img, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 发布结果
        # result_img = cv2.addWeighted(vis_img, 0.7, mask_img, 0.3, 0)
        # result_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        mask_msg = bridge.cv2_to_imgmsg(mask_img, "bgr8")
        mask_msg.header.stamp = msg.header.stamp  # 复制时间戳
        mask_pub.publish(mask_msg)
        
    except Exception as e:
        rospy.logerr(f"图像处理错误: {str(e)}")



  

if __name__ == '__main__':
  # 初始化ROS节点
    rospy.init_node('Instance')
    
    # 初始化CV桥接
    bridge = CvBridge()
    
    # 加载YOLO模型
    model_path = rospy.get_param('~model_path', ' ')
    print(model_path)
    model = YOLO(model_path)
    rospy.loginfo(f"已加载YOLO模型: {model_path}")
    
    # 创建发布者
    # result_pub = rospy.Publisher('/yolo/result', Image, queue_size=100)
    mask_pub = rospy.Publisher('/yolo/mask', Image, queue_size=100)
    
    # 创建订阅者（使用partial传递额外参数）
    # 创建统计变量
    stats = {'total_time': 0.0, 'count': 0}
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback, (bridge, model, mask_pub,stats))
    
    rospy.loginfo("实例分割节点已启动，等待图像输入...")
    rate=rospy.Rate(5000)
    while not rospy.is_shutdown():
        # rospy.spin()
        rate.sleep()

    rospy.loginfo(f"py::处理消息数量: {stats['count'] } ")
    rospy.loginfo(f"平均处理时间: {1000*stats['total_time'] /stats['count'] :.6f} ms")
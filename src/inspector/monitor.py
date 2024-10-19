#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
import re
import matplotlib.pyplot as plt
import os
from std_msgs.msg import Duration
# 全局变量存储检测到的点数和分数
detected_list = []
score_list = []
time_list = []

user_name = os.environ.get("USER")

# 定义图像保存路径
log_dir = f"/home/{user_name}/ws_caric/src/caric_competition_xmu/logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# 定义图像命名递增
def get_next_image_path():
    i = 1
    while os.path.exists(os.path.join(log_dir, f"{i}.png")):
        i += 1
    return os.path.join(log_dir, f"{i}.png")

# 处理Marker消息
last_save_time = 0
save_interval = 4
first_got_time = None
def marker_callback(msg):
    global detected_list, score_list, time_list
    # 提取text字段
    text = msg.text
    
    # 使用正则表达式提取 detected 和 score
    detected_match = re.search(r"Detected:\s+(\d+)", text)
    score_match = re.search(r"Score:\s+([0-9.]+)", text)
    
    if detected_match and score_match:
        detected = int(detected_match.group(1))
        score = float(score_match.group(1))
        timestamp = msg.header.stamp.to_sec()  # 提取消息的时间
        global first_got_time
        if first_got_time is None:
            first_got_time = timestamp
        
        # 保存数据
        detected_list.append(detected)
        score_list.append(score)
        time_list.append(timestamp - first_got_time)

        # rospy.loginfo(f"Detected: {detected}, Score: {score}, Time: {timestamp}")
        global last_save_time
        if timestamp - last_save_time > save_interval:
            save()
            last_save_time = timestamp

time_left = 9999
def duration_callback(msg:Duration):
    global time_left
    time_left = msg.data.to_sec()
    # rospy.loginfo(f"Duration: {msg.data.to_sec()}")



def listener():
    # 初始化ROS节点
    rospy.init_node('marker_listener', anonymous=True)

    # 订阅 visualization_msgs/Marker 消息
    rospy.Subscriber("/viz_score_totalled", Marker, marker_callback)

    # 保持节点运行
    rospy.spin()

def save():
    # 绘制图像
    plt.figure()

    # 创建第一个图：时间 vs 检测到的点数
    plt.subplot(2, 1, 1)
    plt.plot(time_list, detected_list, label="Detected", color='b')
    plt.xlabel("Time (s)")
    plt.ylabel("Detected")
    plt.title("Detected over Time")
    plt.grid(True)

    # 创建第二个图：时间 vs 分数
    plt.subplot(2, 1, 2)
    plt.plot(time_list, score_list, label="Score", color='r')
    plt.xlabel("Time (s)")
    plt.ylabel("Score")
    plt.title("Score over Time")
    plt.grid(True)

    # 调整图像布局
    plt.tight_layout()

    # 保存图像
    plt.savefig(image_path)
    plt.close()

    rospy.loginfo(f"Saved plot to {image_path}")
if __name__ == '__main__':
    try:
        image_path = get_next_image_path()
        listener()
    except rospy.ROSInterruptException:
        save()
    except KeyboardInterrupt:
        save()
    save()

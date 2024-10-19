#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import yaml
import rospy

import sys
sys.path.append('../../')

from sensor_msgs.msg import PointCloud
#绘制
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
# 导入线程模块
import threading
from caric_mission.srv import CreatePPComTopic, CreatePPComTopicRequest
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

def get_face_points(center, size, orientation,resolution):
    # Calculate the half sizes
    half_sizes = size / 2.0
    
    # Define the eight vertices of the rectangle
    vertices = np.array([ [-half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1],  half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1],  half_sizes[2]]
                        ])

    # Rotate the vertices around the center
    rotation_matrix = orientation
    rotated_vertices = np.dot(vertices, rotation_matrix.T)

    # Translate the rotated vertices to the center position
    translated_vertices = rotated_vertices + center

    # Create the mesh, each triang has two sides
    faces = np.array([
                        # Bottom face
                        [0, 1, 2, 3],
                        # Top face
                        [4, 5, 6, 7],
                        # Side faces
                        [0, 4, 5, 1],
                        [1, 5, 6, 2],
                        [2, 6, 7, 3],
                        [3, 7, 4, 0],
                     ])

    resolution=resolution/2.#我采用哪个栅格的一半，保证都覆盖
    face_points=[]
    for i,face in enumerate(faces):#遍历每个面
        start_vetex=translated_vertices[face[0]].copy()#取出面的第一个顶点
        
        aux1_vetex=translated_vertices[face[1]].copy()#这个面上的另一个定点，不是对角那个
        aux2_vetex=translated_vertices[face[3]].copy()

        move1_vetex=np.array([0.,0.,0.])
        move2_vetex=np.array([0.,0.,0.])

        vec1=(aux1_vetex-start_vetex)/(((aux1_vetex-start_vetex).dot(aux1_vetex-start_vetex))**0.5)#获取两个边方向上的单位向量
        vec2=(aux2_vetex-start_vetex)/(((aux2_vetex-start_vetex).dot(aux2_vetex-start_vetex))**0.5)

        while np.linalg.norm(move1_vetex)<np.linalg.norm(aux1_vetex-start_vetex):
            move2_vetex=np.array([0.,0.,0.])
            while np.linalg.norm(move2_vetex)<np.linalg.norm(aux2_vetex-start_vetex):
                face_points.append(start_vetex+move1_vetex+move2_vetex)
                # print(start_vetex+move1_vetex+move2_vetex)
                move2_vetex+=vec2*resolution
                # print(move2_vetex-start_vetex)
            move1_vetex+=vec1*resolution
            # print(move2_vetex-start_vetex)
        print(len(face_points))
    return face_points

def get_face_points(vertices,resolution):
    # Create the mesh, each triang has two sides
    faces = np.array([
                        # Bottom face
                        [0, 1, 2, 3],
                        # Top face
                        [4, 5, 6, 7],
                        # Side faces
                        [0, 4, 5, 1],
                        [1, 5, 6, 2],
                        [2, 6, 7, 3],
                        [3, 7, 4, 0],
                     ])

    resolution=resolution/2.#我采用哪个栅格的一半，保证都覆盖
    face_points=[]
    for i,face in enumerate(faces):#遍历每个面
        start_vetex=vertices[face[0]].copy()#取出面的第一个顶点
        
        aux1_vetex=vertices[face[1]].copy()#这个面上的另一个定点，不是对角那个
        aux2_vetex=vertices[face[3]].copy()

        move1_vetex=np.array([0.,0.,0.])
        move2_vetex=np.array([0.,0.,0.])

        vec1=(aux1_vetex-start_vetex)/(((aux1_vetex-start_vetex).dot(aux1_vetex-start_vetex))**0.5)#获取两个边方向上的单位向量
        vec2=(aux2_vetex-start_vetex)/(((aux2_vetex-start_vetex).dot(aux2_vetex-start_vetex))**0.5)

        while np.linalg.norm(move1_vetex)<np.linalg.norm(aux1_vetex-start_vetex):
            move2_vetex=np.array([0.,0.,0.])
            while np.linalg.norm(move2_vetex)<np.linalg.norm(aux2_vetex-start_vetex):
                face_points.append(start_vetex+move1_vetex+move2_vetex)
                # print(start_vetex+move1_vetex+move2_vetex)
                move2_vetex+=vec2*resolution
                # print(move2_vetex-start_vetex)
            move1_vetex+=vec1*resolution
            # print(move2_vetex-start_vetex)
        print(len(face_points))
    return face_points



def get_all_points(center, size, orientation,resolution):
    
    # Calculate the half sizes
    half_sizes = size / 2.0
    
    # Define the eight vertices of the rectangle
    vertices = np.array([ [-half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1],  half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1],  half_sizes[2]]
                        ])

    # Rotate the vertices around the center
    rotation_matrix = orientation
    rotated_vertices = np.dot(vertices, rotation_matrix.T)

    # Translate the rotated vertices to the center position
    translated_vertices = rotated_vertices + center

    resolution=resolution/2.#我采用哪个栅格的一半，保证都覆盖
    all_points=[]
    start_vetex=translated_vertices[0].copy()#取出面的第一个顶点
    
    aux1_vetex=translated_vertices[1].copy()#这个面上的另一个定点，不是对角那个
    aux2_vetex=translated_vertices[3].copy()
    aux3_vetex=translated_vertices[4].copy()

    move1_vetex=np.array([0.,0.,0.])
    move2_vetex=np.array([0.,0.,0.])
    move3_vetex=np.array([0.,0.,0.])

    vec1=(aux1_vetex-start_vetex)/(((aux1_vetex-start_vetex).dot(aux1_vetex-start_vetex))**0.5)#获取两个边方向上的单位向量
    vec2=(aux2_vetex-start_vetex)/(((aux2_vetex-start_vetex).dot(aux2_vetex-start_vetex))**0.5)
    vec3=(aux3_vetex-start_vetex)/(((aux3_vetex-start_vetex).dot(aux3_vetex-start_vetex))**0.5)

    while np.linalg.norm(move1_vetex)<np.linalg.norm(aux1_vetex-start_vetex):
        move2_vetex=np.array([0.,0.,0.])
        while np.linalg.norm(move2_vetex)<np.linalg.norm(aux2_vetex-start_vetex):
            move3_vetex=np.array([0.,0.,0.])
            while np.linalg.norm(move3_vetex)<np.linalg.norm(aux3_vetex-start_vetex):
                all_points.append(start_vetex+move1_vetex+move2_vetex+move3_vetex)
                move3_vetex+=vec3*resolution
            # print(start_vetex+move1_vetex+move2_vetex)
            move2_vetex+=vec2*resolution
            # print(move2_vetex-start_vetex)
        move1_vetex+=vec1*resolution
        # print(move2_vetex-start_vetex)
    # print(len(all_points))
    return all_points

def get_all_points(vertices,resolution):
    resolution=resolution/2.#我采用哪个栅格的一半，保证都覆盖
    all_points=[]
    start_vetex=vertices[0].copy()#取出面的第一个顶点
    
    aux1_vetex=vertices[1].copy()#这个面上的另一个定点，不是对角那个
    aux2_vetex=vertices[3].copy()
    aux3_vetex=vertices[4].copy()

    move1_vetex=np.array([0.,0.,0.])
    move2_vetex=np.array([0.,0.,0.])
    move3_vetex=np.array([0.,0.,0.])

    vec1=(aux1_vetex-start_vetex)/(((aux1_vetex-start_vetex).dot(aux1_vetex-start_vetex))**0.5)#获取两个边方向上的单位向量
    vec2=(aux2_vetex-start_vetex)/(((aux2_vetex-start_vetex).dot(aux2_vetex-start_vetex))**0.5)
    vec3=(aux3_vetex-start_vetex)/(((aux3_vetex-start_vetex).dot(aux3_vetex-start_vetex))**0.5)

    while np.linalg.norm(move1_vetex)<np.linalg.norm(aux1_vetex-start_vetex):
        move2_vetex=np.array([0.,0.,0.])
        while np.linalg.norm(move2_vetex)<np.linalg.norm(aux2_vetex-start_vetex):
            move3_vetex=np.array([0.,0.,0.])
            while np.linalg.norm(move3_vetex)<np.linalg.norm(aux3_vetex-start_vetex):
                all_points.append(start_vetex+move1_vetex+move2_vetex+move3_vetex)
                move3_vetex+=vec3*resolution
            # print(start_vetex+move1_vetex+move2_vetex)
            move2_vetex+=vec2*resolution
            # print(move2_vetex-start_vetex)
        move1_vetex+=vec1*resolution
        # print(move2_vetex-start_vetex)
    # print(len(all_points))
    return all_points

def get_cube_vertices(center, size, orientation):
    # Calculate the half sizes
    half_sizes = size / 2.0
    
    # Define the eight vertices of the rectangle
    vertices = np.array([ [-half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1], -half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1], -half_sizes[2]],
                          [-half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0], -half_sizes[1],  half_sizes[2]],
                          [ half_sizes[0],  half_sizes[1],  half_sizes[2]],
                          [-half_sizes[0],  half_sizes[1],  half_sizes[2]]
                        ])

    # Rotate the vertices around the center
    rotation_matrix = orientation
    rotated_vertices = np.dot(vertices, rotation_matrix.T)

    # Translate the rotated vertices to the center position
    translated_vertices = rotated_vertices + center
    return translated_vertices

def is_point_inside_cube(cube_vertices, points):
     #若点在立方体的所有六个面的正面（或等于其中一个顶点），则点在立方体内部。
    boxes_num=int(len(cube_vertices))
    return_points=[]
    for point in points:
        in_num=0
        for i in range(boxes_num):
            delete_flag=True
            vectices=cube_vertices[i]
            # 注意右手定则
            faces = np.array([
                                # Bottom face
                                [0, 1, 3],
                                # Top face
                                [4, 7, 5],
                                # Side faces
                                [0, 4, 1],
                                [1, 5, 2],
                                [2, 6, 3],
                                [3, 7, 0],
                            ])
            for b in range(6):
                # 获取当前面的三个顶点
                face_vertices = np.array([vectices[faces[b,0]],
                                          vectices[faces[b,1]],
                                          vectices[faces[b,2]]])

                # 计算当前面的法线
                normal = np.cross(face_vertices[1] - face_vertices[0], face_vertices[2] - face_vertices[0])

                # 计算点到当前面的距离
                distance = np.dot(point - face_vertices[0], normal)

                # 如果点在法线方向上的投影在法线方向上的投影与法线方向上的投影之间，则点在面的正面或等于一个顶点，即点在立方体内部。
                if distance < 0:#有一个小于0
                    delete_flag=False
                    break
            if delete_flag==True:
                in_num+=1
        if in_num<2:
            return_points.append(point)
    # print(len(points))
    # print(len(return_points))
    return return_points


all_face_points=[]
all_points=[]
cube_vertices=[]
cube_vertices_big=[]
callback_start=False
callback_finished=False
def bounding_box_callback(msg):

    global all_face_points
    global all_points
    global cube_vertices
    global callback_finished
    global cube_vertices_big
    global callback_start
    if not callback_start:
        callback_start = True
        all_face_points=[]
        all_points=[]
        cube_vertices=[]

        rospy.loginfo("Please Wait for the bounding box callback to finish")
        # 获取bounding box,每八个点一个立方体
        for i in range(int(len(msg.points)/8)):
            vectices=np.array([[msg.points[i*8].x,msg.points[i*8].y,msg.points[i*8].z],
                                [msg.points[i*8+1].x,msg.points[i*8+1].y,msg.points[i*8+1].z],
                                [msg.points[i*8+2].x,msg.points[i*8+2].y,msg.points[i*8+2].z],
                                [msg.points[i*8+3].x,msg.points[i*8+3].y,msg.points[i*8+3].z],
                                [msg.points[i*8+4].x,msg.points[i*8+4].y,msg.points[i*8+4].z],
                                [msg.points[i*8+5].x,msg.points[i*8+5].y,msg.points[i*8+5].z],
                                [msg.points[i*8+6].x,msg.points[i*8+6].y,msg.points[i*8+6].z],
                                [msg.points[i*8+7].x,msg.points[i*8+7].y,msg.points[i*8+7].z]])
            #center为八个点的中心点
            center=np.array([0.,0.,0.])
            for j in range(8):
                center+=vectices[j]
            center=center/8.
            
            # vectices_big=vectices.copy()
            # for i in range(8):
            #     for j in range(3):
            #         if(vectices[i][j]>center[j]):
            #             # print(vectices_small[j][i])
            #             vectices_big[i][j]=vectices[i][j]+2
            #         else:
            #             vectices_big[i][j]=vectices[i][j]-2
            # cube_vertices_big.append(vectices_big)
            cube_vertices.append(vectices)
        # print(len(cube_vertices))
        volume = 0
        for one_vertices in cube_vertices:
            len1 = np.linalg.norm(np.array(one_vertices[0]) - np.array(one_vertices[1]))
            len2 = np.linalg.norm(np.array(one_vertices[0]) - np.array(one_vertices[3]))
            len3 = np.linalg.norm(np.array(one_vertices[0]) - np.array(one_vertices[4]))
            # print(type(len1))
            volume += len1*len2*len3
            # print("one_vertices: len1={:f}, len2={:f}, len3={:f}".format(len1,len2,len3))
        # print("TOTAL VOLUME:" + str(volume))
        
        global resolution
        if volume < 1000:
            resolution = 1.0
        else:
            resolution = 2.0
        # resolution = 2.0


        print("CHOSEN RESOLUTION:" + str(resolution))
        
        for one_vertices in cube_vertices:
            # one_box_face_points=get_face_points(one_vertices,resolution)
            # all_face_points.extend(one_box_face_points)  
        # for one_vertices in cube_vertices_big:
            one_box_all_points=get_all_points(one_vertices,resolution)
            all_points.extend(one_box_all_points)


        # print('points in faces')
        # print(len(all_face_points))
        # print('all points')
        # print(len(all_points))


        points_filtered=all_face_points.copy()
        # points_filtered=is_point_inside_cube(cube_vertices, points_filtered)
        # print(len(points_filtered))

        points_filtered=np.array(points_filtered)
        all_points=np.array(all_points)
        min_x=min(all_points[:,0])
        max_x=max(all_points[:,0])
        min_y=min(all_points[:,1])
        max_y=max(all_points[:,1])
        min_z=min(all_points[:,2])
        max_z=max(all_points[:,2])
        # print(min_x,max_x,min_y,max_y,min_z,max_z)


        if resolution == 1:
            search_dis_x = 4
            search_dis_y = 5
            search_dis_z = 6
            range_pad_x = 1
            range_pad_y = 1
            range_pad_z = 1

        else :
            search_dis_x = 13
            search_dis_y = 13
            search_dis_z = 11
            range_pad_x = 2
            range_pad_y = 2
            range_pad_z = 3
        if (max_z - min_z) / search_dis_z < 4:
            search_dis_z = int((max_z - min_z) / 5)
        #以x_min x_max y_min y_max z_min z_max为界限，离散化，获取三维点
        # 适当膨胀一下 使得search points 刚好在bbox的外面
        # 因为之后地图会扩张 所以不会越界
        point_z_min = 2
        
        search_points=[]
        x_length=0
        for x in np.arange(min_x-range_pad_x,max_x+range_pad_x,search_dis_x):
            x_length+=1
            y_length=0
            for y in np.arange(min_y-range_pad_y,max_y+range_pad_y,search_dis_y):
                y_length+=1
                z_length=0
                for z in np.arange(min_z + point_z_min,max_z+range_pad_z,search_dis_z):
                    if(z) > 2:
                        z_length+=1
                        search_points.append([x,y,z])
                if max_z+range_pad_z - z > 3:
                    # 给最上层加一层点
                    z_length+=1
                    search_points.append([x,y,max_z+range_pad_z])

        print("preprocess finished, wait 10s...")

        # #讲这些点写入文件,使用相对路径
        # file=open('src/caric_competition_xmu/files/face_points1.txt','w')
        # file.write(str(min_x)+' '+str(max_x)+' '+str(min_y)+' '+str(max_y)+' '+str(min_z)+' '+str(max_z)+'\n')
        # for point in points_filtered:
        #     file.write(str(point[0])+' '+str(point[1])+' '+str(point[2])+'\n')
        # file.close()


        # file=open('src/caric_competition_xmu/files/all_points1.txt','w')
        # for point in all_points:
        #     file.write(str(point[0])+' '+str(point[1])+' '+str(point[2])+'\n')
        # file.close()

        # file=open('src/caric_competition_xmu/files/search_points1.txt','w')
        # file.write(str(x_length)+' '+str(y_length)+' '+str(z_length)+'\n')
        # for point in search_points:
        #     file.write(str(point[0])+' '+str(point[1])+' '+str(point[2])+'\n')
        # file.close()

        # len(np.array(points_filtered)[:,0])
        # # 结束程序
        # # 绘制看看
        # fig = plt.figure()
        # ax = Axes3D(fig)
        # ax.scatter(np.array(points_filtered)[:,0],np.array(points_filtered)[:,1],np.array(points_filtered)[:,2],c='r')
        # ax.scatter(np.array(all_points)[:,0],np.array(all_points)[:,1],np.array(all_points)[:,2],c='b')
        # ax.set_zlabel('Z')
        # ax.set_ylabel('Y')
        # ax.set_xlabel('X')
        # plt.show()
        # #wait key
        # print('yes')

        # 发布msg
        msg_face_points = PoseArray()
        msg_all_points = PoseArray()
        msg_search_points = PoseArray()

        # face_points
        msg_face_points.poses.append(Pose(position=Point(x=min_x-50,y=min_y-50,z=min_z-10)))
        msg_face_points.poses.append(Pose(position=Point(x=max_x+50,y=max_y+50,z=max_z+50)))
        for point in points_filtered:
            msg_face_points.poses.append(Pose(position=Point(x=point[0],y=point[1],z=point[2])))
        #  all_points
        for point in all_points:
            msg_all_points.poses.append(Pose(position=Point(x=point[0],y=point[1],z=point[2])))
        # search_points
        msg_search_points.poses.append(Pose(position=Point(x=x_length,y=y_length,z=z_length)))
        msg_search_points.poses.append(Pose(position=Point(x=resolution+0.1,y=0,z=0)))
        for point in search_points:
            msg_search_points.poses.append(Pose(position=Point(x=point[0],y=point[1],z=point[2])))

        # delay 10 seconds
        rospy.sleep(10)

        pub_face_points.publish(msg_face_points)
        pub_all_points.publish(msg_all_points)
        pub_search_points.publish(msg_search_points)
        
        # 发布done
        msg = String()
        msg.data = "done"
        pub_bounding_box_done.publish(msg)

        callback_finished=True
        print('all pub done')



if __name__ == '__main__':
    # ros初始化
    rospy.init_node('get_points_in_faces', anonymous=True)
    # 发布者
    pub_bounding_box_done = rospy.Publisher("/bounding_box_done",String, queue_size=1)
    pub_face_points = rospy.Publisher("/face_points",PoseArray, queue_size=1)
    pub_all_points = rospy.Publisher("/all_points",PoseArray, queue_size=1)
    pub_search_points = rospy.Publisher("/search_points",PoseArray, queue_size=1)
    rospy.wait_for_service('/create_ppcom_topic')
    # 建立服务客户端
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # 在ppcom路由器上注册话题
    response = create_ppcom_topic('gcs', ['all'], '/bounding_box_done','std_msgs', 'String')
    print(f"Response {response}")
    response = create_ppcom_topic('gcs', ['all'], '/face_points', 'geometry_msgs', 'PoseArray')
    print(f"Response {response}")
    response = create_ppcom_topic('gcs', ['all'], '/all_points', 'geometry_msgs', 'PoseArray')
    print(f"Response {response}")
    response = create_ppcom_topic('gcs', ['all'], '/search_points', 'geometry_msgs', 'PoseArray')
    print(f"Response {response}")

    rospy.Subscriber('/gcs/bounding_box_vertices', PointCloud, bounding_box_callback)

    while not rospy.is_shutdown():
        # print(callback_finished)
        #spin()会一直阻塞在这里，直到节点关闭
        # spin only once
        rospy.spin()

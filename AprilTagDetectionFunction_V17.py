#!/usr/bin/env python

'''
Created on Fri Apr  8 14:49:19 2022.
@author: Raj-Mehta
@email : rajmehta28599v@gmail.com
'''
from sys import stdout


def YawData(yaw, r11_1, r21_1):
    if r11_1 > 0 and r21_1 > 0:
        # stdout.write("1rd Quadrant")
        ApriTagYaw1 = yaw
    if r11_1 < 0 and r21_1 > 0:
        # stdout.write("2rd Quadrant")
        ApriTagYaw1 = 180 - abs(yaw)
    if r11_1 < 0 and r21_1 < 0:
        # stdout.write("3rd Quadrant")
        ApriTagYaw1 = 180 + yaw
    if r11_1 > 0 and r21_1 < 0:
        # stdout.write("4th Quadrant")
        ApriTagYaw1 = 360 - abs(yaw)
    return ApriTagYaw1


def apriltagDetectionFunction(at_x, at_y, at_z, at_yaw, at_tag_id, tag_detection_state, at_detection_currenttag, at_detection_destinationtag, input_process_enableflag, camera_connection_error_flag, apriltagmodule_flag, t_detect0_min, t_detect480_min, b_detecty_min, capture_click, detectedSide480, detectedSide0, detectedSideB, uiSwitch_Flag):
    # global popen, findall, Laplacian, meanStdDev, CV_16S
    stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] MP -> setup start")
    stdout.flush()
    apriltagmodule_flag.value = 3

    from apriltags3 import Detector
    from cv2 import VideoCapture, cvtColor, COLOR_BGR2GRAY, imshow, waitKey, destroyAllWindows, imwrite, bitwise_not
    from cv2 import VideoWriter
    from cv2 import CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH
    from cv2 import CAP_PROP_FPS, CAP_PROP_FOURCC
    from cv2 import fisheye, remap, INTER_LINEAR, BORDER_CONSTANT, CV_16SC2, IMWRITE_JPEG_QUALITY
    from time import time, sleep
    from re import findall
    from os import popen, system
    from math import atan, degrees, sqrt
    # from threading import Thread
    # from sys import exit
    # from psutil import cpu_percent
    # from queue import Queue, Empty
    # from urllib.request import urlretrieve
    # from getpass import getuser
    from pandas import read_csv
    from numpy import array, zeros, float64, asanyarray, nonzero, where, eye
    # from math import radians, cos, sin
    from datetime import datetime
    from pyrealsense2 import pipeline, config, stream, format, option
    from cv2 import applyColorMap, convertScaleAbs, rectangle, circle, FONT_HERSHEY_SIMPLEX, putText, COLORMAP_JET, LINE_AA

    date = datetime.now
    xxx1 = -1
    # depth
    # Top Depth    # box1, box2, ...
    box_sensing_limit480 =  [1170, 1150, 1115, 1080, 955, 960, 1010, 980, 940, 920] # [1200, 1170, 1140, 1100, 975, 970, 1020, 990, 960, 940]
    box_pixel_limit480 = [80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
    
    box_sensing_limit0 = [1400, 1300, 1400, 1405, 1305, 1320, 1180, 760, 750, 738] # [1050, 1000, 960, 920, 880, 790, 770, 760, 750, 738]
    box_pixel_limit0 = [80, 150, 120, 80, 80, 80, 80, 80, 80, 80]
    
    # x range 0 to 640
    
    # x1, x2, x3, x4 = 200, 480, 250, 430
    # x1, x2, x3, x4 = 192, 473, 220, 440
    x1, x2, x3, x4 = 200, 480, 250, 430
    
    # y range 0 to 480
    
    y1, y2, y3, y4 = 300, 475, 10, 300

    # number of partition on path
    
    # near to robot < 500mm
    part_y0 = 7
    part_length_y0 = int((y4-y3)/part_y0)
    # near to robot < 500mm
    part_y480 = 10
    part_length_y480 = int((y2-y1)/part_y480)
    # fonr style
    font = FONT_HERSHEY_SIMPLEX
    t_detect0 = [1111]
    # t_detect0_min = [0]

    t_detect480 = [1111]
    # t_detect480_min = [0]

    # box1, box2, ...
    B_box_sensing_limit0 = [[330, 370, 530, 1050, 1700],
                         [310, 330, 500, 600, 1600],
                         [300, 330, 500, 600, 900],
                         [300, 330, 500, 600, 900],
                         ]
    # B_box_sensing_limit0 = [[330, 370, 530, 1250, 1700],
                         # [310, 330, 500, 600, 1600],
                         # [300, 330, 500, 600, 900],
                         # [300, 330, 500, 600, 900],
                         # ]
    
    B_box_pixel_limit0 = [[80, 80, 80, 80, 80],
                       [80, 80, 80, 80, 80],
                       [80, 80, 80, 80, 80],
                       [80, 80, 80, 80, 80],
                       ]
    # x range 0 to 640
    
    B_x1, B_x2 = 20, 620
    
    # y range 0 to 480
    B_y1, B_y2 = 2, 460

    # number of partition on path
    # near to robot < 500mm
    B_part_x0, B_part_y0 = 5, 4
    B_part_length_y0 = int((B_y2 - B_y1) / B_part_y0)
    B_part_length_x0 = int((B_x2 - B_x1) / (B_part_x0 * 2))
    b_detectx, b_detecty = [], []
    # b_detecty_min = [0]
    # Top Depth Camera
    try:
        # Configure depth and color streams of the intel realsense
        pipelineT = pipeline()
        configT = config()
        configT.enable_device('017322071426')
        # configT.enable_device('045322072728')
        configT.enable_stream(stream.depth, 640, 480, format.z16, 60)  # 14/11/22 4:43pm
        configT.enable_stream(stream.color, 640, 480, format.bgr8, 30)
        cfgT = pipelineT.start(configT)
        devT = cfgT.get_device()
        depth_sensorT = devT.first_depth_sensor()
    except Exception as e:
        stdout.write("\n\n\n[->]rDCTC()[->] check Top camera serial number exception={},".format(e))
        stdout.flush()
        pass

    # Bottom Depth Camera
    try:
        # Configure depth and color streams of the intel realsense
        pipelineB = pipeline()
        configB = config()
        configB.enable_device('827112071410')
        configB.enable_stream(stream.depth, 640, 480, format.z16, 90)
        configB.enable_stream(stream.color, 640, 480, format.bgr8, 30)
        cfgB = pipelineB.start(configB)
        devB = cfgB.get_device()
        depth_sensorB = devB.first_depth_sensor()
    except Exception as e:
        stdout.write("\n\n\n[->]rDCTC()[->] check Bottom camera serial number exception={},".format(e))
        stdout.flush()
        pass

    TagSize0, TagSize1 = 0.145, 145
    fx, fy, cx, cy = 533.055, 529.644, 338.090, 247.039


    '''
    for robot_v2_p2
    DIM = (640, 480)
    K = array([[530.0359882905851, 0.0, 330.4013132922936], [0.0, 526.5470138068929, 236.6505630192782], [0.0, 0.0, 1.0]])
    D = array([[-0.06413230520512765], [-0.11135284675028854], [1.0502311322641782], [-1.917756640935642]])
    '''
    
    # for robot_v2_p3
    DIM=(640, 480)
    K=array([[549.0321586199226, 0.0, 336.42171803267024], [0.0, 544.3735911193245, 248.63719304090202], [0.0, 0.0, 1.0]])
    D=array([[-0.06867322338310829], [-0.22967699618571022], [1.5999572871285395], [-3.024729585156691]])

    PreviousTAG_ID = 0
    AllocatedNode, NeighbourNodeMetadataList = [0], []

    tag_corners_reshape_3d = array([[-TagSize1/2,  TagSize1/2, 0],
                                    [TagSize1/2,  TagSize1/2, 0],
                                    [TagSize1/2, -TagSize1/2, 0],
                                    [-TagSize1/2, -TagSize1/2, 0]], dtype=float64)
    # Camera distortion Matrix, assumed Zero
    distortion = zeros((4, 1))
    camera_params1 = [fx, fy, cx, cy]
    # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] camera_params1={},".format(camera_params1))
    # stdout.flush()
    camera_matrix = array([[camera_params1[0], 0, camera_params1[2]],
                            [0, camera_params1[1], camera_params1[3]],
                            [0, 0, 1]], dtype=float64)
    # parameter functions are redefined on 22/01/2021
    # Reference Link: https://github.com/pupil-labs/apriltags
    # Main Function
    # IgnoreTagList_AtThread = [0]
    CameraIndex = -1
    t1_april = t2_april = time()
    t1_depth = t2_depth = time()
    b1_depth = b2_depth = time()
    apriltagmodule_flag.value = 4
    _startAnyNode_Flag = 1
    stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] _startAnyNode_Flag={}, MP -> setup End".format(_startAnyNode_Flag))
    stdout.flush()
    at_detector = Detector(searchpath=['apriltags/lib', 'apriltags/lib64'], families='tag36h11', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
    prev_uiSwitch_Flag = -1
    stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] prev_uiSwitch_Flag={}, MP -> setup End".format(prev_uiSwitch_Flag))
    stdout.flush()

    while True:
        t2_april = time() - t1_april
        if t2_april > (0.015):  # and t1 < (0.05):
            t1_april = time()
            try:
                if waitKey(1) == 113 or input_process_enableflag.value == 1:  # 16/12/22 12:18pm
                    cap.release()
                    ret = 0
                    pipelineT.stop()
                    configT.disable_all_streams()
                    pipelineB.stop()
                    configB.disable_all_streams()
                    apriltagmodule_flag.value = 1
                    input_process_enableflag.value = 2
                    stdout.write("\n[--->ADF] break: if input_process_enableflag.value == 1: apriltagmodule_flag.value={}, t2_april={},".format(apriltagmodule_flag.value, t2_april))
                    stdout.flush()
                    break
            except Exception as e:
                stdout.write("\n\n\n[--->ADF]input_process_enableflag.value==1 t2_april={}, exception={},".format(t2_april, e))
                stdout.flush()
                pass
            try:
                if CameraIndex != (-1):
                    try:
                        ret, frame = cap.read()

                        if ret:
                            apriltagmodule_flag.value = 6

                            h, w = frame.shape[:2]
                            map1, map2 = fisheye.initUndistortRectifyMap(K, D, eye(3), K, DIM, CV_16SC2)
                            undistorted_img = remap(frame, map1, map2, interpolation=INTER_LINEAR, borderMode=BORDER_CONSTANT)

                            gray = cvtColor(undistorted_img, COLOR_BGR2GRAY)
                            invert = bitwise_not(gray)
                            # imshow('Invert view', invert)
                            # imshow('frame view', frame)
                            # stdout.write("\n[--->ADF] invert")
                            # stdout.flush()
                            tags = at_detector.detect(invert, estimate_tag_pose=True, camera_params=camera_params1, tag_size=TagSize0)
                            tags_length = len(tags)

                            if not tags_length:  # 10/11/22 12:56pm
                                tag_detection_state.value = 0
                                # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]else: tag_detection_state.value={}, detected_parameters.tag_id={},".format(tag_detection_state.value, detected_parameters.tag_id))
                                pass
                            else:
                                tag_list = [i.tag_id for i in tags]
                                # tag_list_tt.value = tag_list
                                # stdout.write("\n[-->ADF]Tag List={},".format(tag_list))
                                # use for Functional Script
                                if uiSwitch_Flag.value == 0:
                                    if uiSwitch_Flag.value != prev_uiSwitch_Flag:
                                        try:
                                            commonVariableFileName = "./Configurations/CommonVariables2.csv"
                                            nodeListFileName = "./Configurations/Data_Assignment_4.csv"
                                            ErrorCodeCsvFile = "./Configurations/ErrorCodeList.csv"
                                            AtCameraDataFile = "./Configurations/AtCamData.csv"
                                            VirtualNodeFileName = "./Configurations/VirtualNodeMapping.csv"

                                            CommonVariableDF = read_csv(commonVariableFileName).drop(columns=['Comment', 'Parameter']).dropna(subset=["Name"]).set_index('Name')

                                            at_detection_currenttag.value = int(CommonVariableDF.loc['HomeNode'])
                                            at_detection_destinationtag.value = int(CommonVariableDF.loc['HomeNode'])
                                            del CommonVariableDF

                                            NodeListDF = read_csv(nodeListFileName).set_index('cnode')
                                            AvailableNodeList = list(dict.fromkeys(NodeListDF.index.tolist()))
                                            # RealNodeList = AvailableNodeList  # list(set(DataAssignmentDF.cnode))
                                            # del NodeListDF

                                            prev_uiSwitch_Flag = uiSwitch_Flag.value
                                        except Exception as e:
                                            stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->] csv exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    # Both Nodes [Current & Destination]
                                    if set([at_detection_currenttag.value, at_detection_destinationtag.value]).issubset(set(tag_list)):
                                        _startAnyNode_Flag = 2
                                        # stdout.write("\nUnnecessaryTag # Both Nodes [Current & Destination]")
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]if set([at_de at_detection_currenttag.value={}, at_detection_destinationtag.value={},".format(at_detection_currenttag.value, at_detection_destinationtag.value))
                                        current_node_distance, destination_node_distance = 0, 0
                                        for value in tags:
                                            if value.tag_id == at_detection_currenttag.value:
                                                current_node_distance = sqrt((value.pose_t[0][0]**2) + (value.pose_t[1][0]**2))
                                            elif value.tag_id == at_detection_destinationtag.value:
                                                destination_node_distance = sqrt((value.pose_t[0][0]**2) + (value.pose_t[1][0]**2))

                                        if current_node_distance < destination_node_distance:
                                            detected_parameters = tags[tag_list.index(at_detection_currenttag.value)]
                                        else:
                                            detected_parameters = tags[tag_list.index(at_detection_destinationtag.value)]

                                        if AllocatedNode[0]: AllocatedNode[0] = 0

                                    # Destination Node
                                    elif at_detection_destinationtag.value in tag_list:
                                        _startAnyNode_Flag = 2
                                        # stdout.write("\nUnnecessaryTag # Destination Node")
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif at_detection_de at_detection_destinationtag.value={}, tag_list={},".format(at_detection_destinationtag.value, tag_list))
                                        detected_parameters = tags[tag_list.index(at_detection_destinationtag.value)]
                                        if AllocatedNode[0]:
                                            AllocatedNode[0] = 0

                                    # Current Node
                                    elif at_detection_currenttag.value in tag_list:
                                        _startAnyNode_Flag = 2
                                        # stdout.write("\nUnnecessaryTag # Current Node")
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif at_detection_cu at_detection_currenttag.value={}, tag_list={},".format(at_detection_currenttag.value, tag_list))
                                        detected_parameters = tags[tag_list.index(at_detection_currenttag.value)]
                                        if AllocatedNode[0]: AllocatedNode[0] = 0

                                    # Neighbour Node on straight path
                                    elif len(NeighbourNodeMetadataList):
                                        _startAnyNode_Flag = 2
                                        # stdout.write("\nUnnecessaryTag # Neighbour Node on straight path")
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif len(NeighbourNodeMetadataList)({}): list={},".format(len(NeighbourNodeMetadataList), NeighbourNodeMetadataList))
                                        # stdout.write("\n[-->ADF] NeighbourNodeMetadataList={}, NeighbourNodeMetadataList[0]={}".format(NeighbourNodeMetadataList, NeighbourNodeMetadataList[0]))
                                        if len(list(set(NeighbourNodeMetadataList[0]) & set(tag_list))):
                                            detected_neighbour_node_list = list(set(NeighbourNodeMetadataList[0]) & set(tag_list))
                                            for i in range(len(NeighbourNodeMetadataList[0])):
                                                if (NeighbourNodeMetadataList[0][i] in detected_neighbour_node_list) and (NeighbourNodeMetadataList[2][i] < 10):
                                                    detected_parameters = tags[tag_list.index(NeighbourNodeMetadataList[0][i])]
                                                    if AllocatedNode[0]:
                                                        AllocatedNode[0] = 0
                                                    # stdout.write("\n[--->ADF] break: elif len(NeighbourNodeMetadataList):")
                                                    # stdout.flush()
                                                    break

                                    # Fix allocated node
                                    elif AllocatedNode[0] and (AllocatedNode[0] in tag_list):
                                        _startAnyNode_Flag = 2
                                        # stdout.write("\nUnnecessaryTag # Fix allocated node")
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif (AllocatedNode[0]): AllocatedNode[0]={}, tag_list={},".format(AllocatedNode[0], tag_list))
                                        detected_parameters = tags[tag_list.index(AllocatedNode[0])]

                                    # # Start Robot from !HomeNode while in start point
                                    elif _startAnyNode_Flag == 1:
                                        try:
                                            # stdout.write("\nUnnecessaryTag # _StartAnyNode_Flag")
                                            # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif _startAnyNode_Flag == 1: tags_length={},".format(tags_length))
                                            if tags_length == 1:
                                                detected_parameters = tags[0]
                                            else:
                                                answer = [sqrt((i.pose_t[0][0] ** 2) + (i.pose_t[1][0] ** 2)) for i in tags]
                                                detected_parameters = tags[answer.index(min(answer))]
                                            AllocatedNode[0] = detected_parameters.tag_id
                                            _startAnyNode_Flag = 2
                                        except Exception as e:
                                            stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->] elif _startAnyNode_Flag: exception={},".format(e))
                                            stdout.flush()
                                            pass

                                    # more then one tag in One Frame and both are not HomeNode and DestinationNode
                                    elif tags_length > 1:
                                        try:
                                            # stdout.write("\nUnnecessaryTag # tags_length > 1")
                                            # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif tags_length > 1: tags_length={},".format(tags_length))
                                            # stdout.flush()

                                            answer = [sqrt((i.pose_t[0][0] ** 2) + (i.pose_t[1][0] ** 2)) for i in tags]
                                            detected_parameters = tags[answer.index(min(answer))]
                                            AllocatedNode[0] = detected_parameters.tag_id

                                        except Exception as e:
                                            stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->]elif tags_length > 1: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        # stdout.write("\nUnnecessaryTag # else:")
                                        pass
                                elif uiSwitch_Flag.value == 1:
                                    # AvailableNodeList = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
                                    AvailableNodeList =  [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50]
                                    if tags_length > 1:
                                        try:
                                            # stdout.write("\nUnnecessaryTag # tags_length > 1")
                                            # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->]elif tags_length > 1: tags_length={},".format(tags_length))
                                            # stdout.flush()
                                            answer = [sqrt((i.pose_t[0][0] ** 2) + (i.pose_t[1][0] ** 2)) for i in tags]
                                            detected_parameters = tags[answer.index(min(answer))]
                                            AllocatedNode[0] = detected_parameters.tag_id
                                        except Exception as e:
                                            stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->]elif tags_length > 1: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    elif tags_length == 1:
                                        detected_parameters = tags[0]
                                if uiSwitch_Flag.value == 0 or uiSwitch_Flag.value == 1:
                                    if (detected_parameters.tag_id in AvailableNodeList):
                                        # tag_detection_state.value = 1  # 10/11/22 12:55pm
                                        at_tag_id.value = detected_parameters.tag_id
                                        # stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] tag_detection_state.value={}, detected_parameters.tag_id={},".format(tag_detection_state.value, detected_parameters.tag_id))
                                        pose_r, pose_T = detected_parameters.pose_R, detected_parameters.pose_t
                                        # r11, r12, r13 = pose_r[0][0], pose_r[0][1], pose_r[0][2]
                                        # r21, r22, r23 = pose_r[1][0], pose_r[1][1], pose_r[1][2]
                                        r11, r21 = pose_r[0][0], pose_r[1][0]
                                        # r31, r32, r33 = pose_r[2][0], pose_r[2][1], pose_r[2][2]
                                        at_x.value = pose_T[0][0]  # *1000
                                        at_y.value = pose_T[1][0]  # *1000
                                        at_z.value = pose_T[2][0]  # *1000
                                        yaw1 = degrees(atan(r21/r11))
                                        at_yaw.value = round(YawData(yaw1, r11, r21), 3)
                                        tag_detection_state.value = 1  # 10/11/22 12:55pm
                                        if at_tag_id.value != PreviousTAG_ID:
                                            PreviousTAG_ID = at_tag_id.value

                        elif input_process_enableflag.value:
                            pass
                        else:
                            CameraIndex = (-1)
                            try:
                                cap.release()
                            except Exception as e:
                                stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->] cap.release() t2_april={}, exception={},".format(t2_april, e))
                                stdout.flush()
                                pass
                    except Exception as e:
                        stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->] if CameraIndex != (-1): t2_april={}, exception={},".format(t2_april, e))
                        stdout.flush()
                        pass
                else:
                    camera_status = popen('v4l2-ctl --list-devices').read()
                    if 'See3CAM_20CUG' in camera_status:
                        camera_status = camera_status.split('\n\n')
                        for i in range(len(camera_status)):
                            if 'See3CAM_20CUG' in camera_status[i]:
                                CameraIndex = list(map(int, findall(r'\d+', camera_status[i].split('/')[-1])))[0]
                                stdout.write("\n[-->ADF] if 'See3CAM_20CUG' in camera_status[i]: t2_april={}, CameraIndex={},".format(t2_april, CameraIndex))
                                stdout.flush()
                    else:
                        CameraIndex = -1

                    if CameraIndex != -1:
                        try:
                            cap = VideoCapture(CameraIndex)
                            # stdout.write("\n[--->ADF]apriltagDetectionFunction()[->] Apriltag Camera Index={},".format(CameraIndex))
                            cap.set(CAP_PROP_FRAME_HEIGHT, 480)  # 480, 1300
                            cap.set(CAP_PROP_FRAME_WIDTH, 640)  # 640, 1600
                            cap.set(CAP_PROP_FOURCC, VideoWriter.fourcc('G', 'R', 'E', 'Y'))
                            cap.set(CAP_PROP_FPS, 60)
                        except Exception as e:
                            stdout.write("\n\n\n[--->ADF] Exception In main AprilTag Thread t2_april={}, exception={},".format(t2_april, e))
                            stdout.flush()
                            pass
                    else:
                        # if not test_mode.value:
                        camera_connection_error_flag.value = 1
                        stdout.write("\n[-->ADF]apriltagDetectionFunction()[->] AT Camera Not Connected t2_april={},".format(t2_april))
                        stdout.flush()
                    apriltagmodule_flag.value = 5
            except Exception as e:
                stdout.write("\n\n\n[-->ADF]apriltagDetectionFunction()[->] while True: t2_april={}, exception={},".format(t2_april, e))
                stdout.flush()
                pass

        # Top Depth
        t2_depth = time() - t1_depth
        if t2_depth > (0.011):
            t1_depth = time()
            # stdout.write("\n[--->ADF_TopDepth]if dataToTeensy[0][0] >= 4: t2_depth={},".format(t2_depth))
            # stdout.flush()
            # stdout.write("\n[->]rDCTC()[->] time={},".format(1/t2_depth))
            # Create a pipeline object. This object configures the streaming camera and owns it's handle
            frames = pipelineT.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            depth_image = asanyarray(depth_frame.get_data())
            depth_click = applyColorMap(convertScaleAbs(depth_image, None, 0.5, 0), COLORMAP_JET)
            tempProjT = depth_sensorT.get_option(option.projector_temperature)
            # loop for number of partition and in that depth detection
            temp_x1, temp_y1, temp_x2, temp_y2 = x1, y1, x2, y1+part_length_y480
            for box in range(0, part_y480):
                a = depth_image[temp_y1:temp_y2, temp_x1:temp_x2]
                depth_click = rectangle(depth_click, (temp_x1, temp_y1), (temp_x2, temp_y2), (0, 0, 0), 4)
                depth_click = rectangle(depth_click, (temp_x1, temp_y1), (temp_x2, temp_y2), (255, 255, 255), 2)

                a1 = a[nonzero(a)]

                box_pixel_count = a1[a1 < box_sensing_limit480[box]].shape[0]
                # print("\n[->]box={}, box_sensing_limit[box]={}, box_pixel_count={},".format(box, box_sensing_limit[box], box_pixel_count))
                if box_pixel_count > box_pixel_limit480[box]:
                    mini = int(a1.min())
                    yy1, xx1 = where(a == mini)[0], where(a == mini)[1]
                    mini, yyy1, xxx1 = mini / 10, temp_y1 + yy1[int(len(yy1) / 2)], temp_x1 + xx1[int(len(xx1) / 2)]
                    depth_click = circle(depth_click, (xxx1, yyy1), 10, (255, 255, 255), 2)
                    t_detect480.append(part_y480 - box)
                    if xxx1 > 320:
                        if xxx1 <= 370: detectedSide480.value = -3
                        else: detectedSide480.value = -2
                    elif xxx1 <= 320:
                        if xxx1 >= 270: detectedSide480.value = 3
                        else: detectedSide480.value = 2
                    else:
                        stdout.write("\n[->]rDCTC()[->] t2_depth={}, xxx1={}, min(t_detect480)={},".format(t2_depth, xxx1, min(t_detect480)))
                        stdout.flush()
                        system("sudo killall -9 python")
                else:
                    t_detect480.append(1111)
                    mini = box_sensing_limit480[box]
                    if min(t_detect480) == 1111: detectedSide480.value = 0

                if min(t_detect480) != 1111 and detectedSide480.value == 0:
                    t_detect480_min.value = 1111
                else:
                    t_detect480_min.value = int(min(t_detect480))

                depth_click = putText(depth_click, str(mini), (500, temp_y1 - 100), font, 0.3, (255, 255, 255), 1, LINE_AA)
                depth_click = putText(depth_click, str(t_detect480[box]), (540, temp_y1 - 100), font, 0.3, (255, 255, 255), 1, LINE_AA)

                temp_y1, temp_y2 = temp_y1 + part_length_y480, temp_y2 + part_length_y480

            temp_x1, temp_y1, temp_x2, temp_y2 = x3, y3, x4, y3+part_length_y0
            for box in range(0, part_y0):
                a = depth_image[temp_y1:temp_y2, temp_x1:temp_x2]
                depth_click = rectangle(depth_click, (temp_x1, temp_y1), (temp_x2, temp_y2), (0, 0, 0), 4)
                depth_click = rectangle(depth_click, (temp_x1, temp_y1), (temp_x2, temp_y2), (255, 255, 255), 2)
                a1 = a[nonzero(a)]
                box_pixel_count = a1[a1 < box_sensing_limit0[box]].shape[0]
                if box_pixel_count > box_pixel_limit0[box]:
                    mini = int(a1.min())
                    yy1, xx1 = where(a == mini)[0], where(a == mini)[1]
                    mini, yyy1, xxx1 = mini / 10, temp_y1 + yy1[int(len(yy1) / 2)], temp_x1 + xx1[int(len(xx1) / 2)]
                    depth_click = circle(depth_click, (xxx1, yyy1), 10, (255, 255, 255), 2)
                    t_detect0.append((part_y480+part_y0) - box)

                    if xxx1 > 320:
                        if xxx1 <= 370: detectedSide0.value = -3
                        else: detectedSide0.value = -2
                    elif xxx1 <= 320:
                        if xxx1 >= 270: detectedSide0.value = 3
                        else: detectedSide0.value = 2
                    else:
                        stdout.write("\n[->]rDCTC()[->] t2_depth={}, xxx1={}, min(t_detect0)={},".format(t2_depth, xxx1, min(t_detect0)))
                        stdout.flush()
                        system("sudo killall -9 python")
                else:
                    # detectedSide0.value = 0
                    t_detect0.append(1111)
                    mini = box_sensing_limit0[box]
                    if min(t_detect0) == 1111: detectedSide0.value = 0

                if min(t_detect0) != 1111 and detectedSide0.value == 0:
                    # stdout.write("\n[->]rDCTC()[->] t2_depth={}, detectedSide0.value={}, xxx1={}, min(t_detect0)={},".format(t2_depth, detectedSide0.value, xxx1, min(t_detect0)))
                    # stdout.flush()
                    # system("sudo killall -9 python")
                    t_detect0_min.value = 1111
                else:
                    t_detect0_min.value = min(t_detect0)

                depth_click = putText(depth_click, str(mini), (10, temp_y1), font, 0.3, (255, 255, 255), 1, LINE_AA)
                depth_click = putText(depth_click, str(t_detect0[box]), (100, temp_y1), font, 0.3, (255, 255, 255), 1, LINE_AA)

                temp_y1, temp_y2 = temp_y1 + part_length_y0, temp_y2 + part_length_y0

            # for capture image
            try:
                if capture_click.value == 1:
                    color_frame = frames.get_color_frame()
                    color_image = asanyarray(color_frame.get_data())
                    imwrite('log/{}_T.jpeg'.format(date().strftime("%d_%m_%Y_%H_%M_%S")), color_image, [IMWRITE_JPEG_QUALITY, 30])  # 30/11/22 12:33pm
                    imwrite('log/{}_TD.jpeg'.format(date().strftime("%d_%m_%Y_%H_%M_%S")), depth_click, [IMWRITE_JPEG_QUALITY, 30])
                    capture_click.value = 2
            except Exception as e:
                stdout.write("\n\n\n[--->ADF_TopDepth]if dataToTeensy[0][0] >= 4: t2_depth={}, exception={},".format(t2_depth, e))
                stdout.flush()
                pass

            # imshow('tracking', color_image)
            imshow('Top DepthImage', depth_click)
            # imwrite('_D.png', depth_click)
            # break the script

            t_detect0.clear()
            t_detect480.clear()

        # Bottom Depth Camera
        b2_depth = time() - b1_depth
        if b2_depth > (0.011):
            b1_depth = time()
            # stdout.write("\n[--->ADF_BottomDepth]if dataToTeensy[0][0] >= 4: b2_depth={},".format(b2_depth))
            # stdout.flush()
            framesB = pipelineB.wait_for_frames()
            depth_frame = framesB.get_depth_frame()
            depth_image = asanyarray(depth_frame.get_data())
            b_depth_click = applyColorMap(convertScaleAbs(depth_image, None, 0.09, 0), COLORMAP_JET)
            tempProjB = depth_sensorB.get_option(option.projector_temperature)
            
            # loop for number of partition and in that depth detection
            
            b_temp_x1y, b_temp_y1y, b_temp_x2y, b_temp_y2y = 0, 0, 0, 0
            b_temp_x1, b_temp_y1, b_temp_x2, b_temp_y2 = B_x1, B_y1, B_x2, B_y1 + B_part_length_y0
            b_temp_x1y, b_temp_y1y, b_temp_x2y, b_temp_y2y = b_temp_x1, b_temp_y1, b_temp_x2, b_temp_y2
            for box_y in range(0, B_part_y0):
                for box_x in range(0, B_part_x0):
                    a = depth_image[b_temp_y1:b_temp_y2, b_temp_x1:b_temp_x2]
                    b_depth_click = rectangle(b_depth_click, (b_temp_x1, b_temp_y1), (b_temp_x2, b_temp_y2), (0, 0, 0), 4)
                    b_depth_click = rectangle(b_depth_click, (b_temp_x1, b_temp_y1), (b_temp_x2, b_temp_y2), (255, 255, 255), 2)

                    a1 = a[nonzero(a)]
                    box_pixel_count = a1[a1 < B_box_sensing_limit0[box_y][box_x]].shape[0]
                    # print("\n[->]box={}, box_sensing_limit[box]={}, box_pixel_count={},".format(box, box_sensing_limit[box], box_pixel_count))
                    if box_pixel_count > B_box_pixel_limit0[box_y][box_x]:
                        mini = int(a1.min())
                        yy1, xx1 = where(a == mini)[0], where(a == mini)[1]
                        mini, yyy1, xxx1 = mini / 10, b_temp_y1 + yy1[int(len(yy1) / 2)], b_temp_x1 + xx1[int(len(xx1) / 2)]
                        b_depth_click = circle(b_depth_click, (xxx1, yyy1), 10, (255, 255, 255), 2)
                        b_detecty.append(box_y)
                        b_detectx.append(box_x)
                        if xxx1 > 320:
                            if xxx1 <= 400: detectedSideB.value = -4
                            elif xxx1 <= 490: detectedSideB.value = -3
                            else: detectedSideB.value = -2
                        elif xxx1 <= 320:
                            if xxx1 >= 250: detectedSideB.value = 4
                            elif xxx1 >= 150: detectedSideB.value = 3
                            else: detectedSideB.value = 2                        
                        else:
                            stdout.write("\n[->]rDCTC()[->] b2_depth={}, xxx1={}, min(b_detecty)={},".format(b2_depth, xxx1, min(b_detecty)))
                            stdout.flush()
                            system("sudo killall -9 python")
                        # stdout.write("\n[->ADF] if box_pixel_count > B_box_pixel_limit0[box_y][box_x]: xxx1 = {}, detectedSideB = {};".format(xxx1, detectedSideB.value))
                        # stdout.flush()
                    else:
                        # detectedSideB.value = 0
                        b_detecty.append(1111)
                        b_detectx.append(1111)
                        mini = B_box_sensing_limit0[box_y][box_x]
                        if min(b_detecty) == 1111: 
                            detectedSideB.value = 0
                    # b_detecty_min[0] = min(b_detecty)
                    if min(b_detecty) != 1111 and detectedSideB.value == 0:
                        # stdout.write("\n[->]rDCTC()[->] b2_depth={}, detectedSideB.value={}, xxx1={}, min(b_detecty)={},".format(b2_depth, detectedSideB.value, xxx1, min(b_detecty)))
                        # stdout.flush()
                        b_detecty_min.value = 1111
                        # system("sudo killall -9 python")
                    else:
                        b_detecty_min.value = min(b_detecty)
                    # print(b_detecty_min[0])

                    b_depth_click = putText(b_depth_click, str(mini), (b_temp_x1, b_temp_y1 + 20), font, 0.5, (0, 0, 0), 2, LINE_AA)
                    b_depth_click = putText(b_depth_click, str(b_detecty[box_y]) + " " + str(b_detectx[box_x]), (b_temp_x1 + 2, b_temp_y1 + 40), font, 0.5, (0, 0, 0), 2, LINE_AA)
                    b_temp_x1, b_temp_y1, b_temp_x2, b_temp_y2 = b_temp_x1 + B_part_length_x0 - 10, b_temp_y1 + 5, b_temp_x2 - B_part_length_x0 + 10, b_temp_y2 - 5
                b_temp_x1, b_temp_y1, b_temp_x2, b_temp_y2 = B_x1, b_temp_y1y + B_part_length_y0, B_x2, b_temp_y2y + B_part_length_y0
                b_temp_y1y, b_temp_y2y = b_temp_y1, b_temp_y2
                b_detectx.clear()

            b_depth_click = putText(b_depth_click, str(B_part_y0) + 'x' + str(B_part_x0), (550, 440), font, 0.8, (255, 255, 255), 2, LINE_AA)
            b_detecty.clear()

            b_depth_click = putText(b_depth_click, str("RAJ"), (440, 30), font, 0.8, (255, 255, 255), 1, LINE_AA)
            # imshow('Bottom DepthImage', b_depth_click)
            # for capture image
            try:
                if capture_click.value == 2:
                    color_frame = framesB.get_color_frame()
                    color_image = asanyarray(color_frame.get_data())
                    imwrite('log/{}_B.jpeg'.format(date().strftime("%d_%m_%Y_%H_%M_%S")), color_image, [IMWRITE_JPEG_QUALITY, 30])
                    imwrite('log/{}_BD.jpeg'.format(date().strftime("%d_%m_%Y_%H_%M_%S")), b_depth_click, [IMWRITE_JPEG_QUALITY, 30])
                    capture_click.value = 3
                    stdout.write("\n[-->ADF_BottomDepth]b2_depth={}, capture_click[0]={}, tempProjT={}, tempProjB={}, filename={},".format(b2_depth, capture_click.value, tempProjT, tempProjB, date().strftime("%d_%m_%Y_%H_%M_%S")))
                    stdout.flush()
            except Exception as e:
                stdout.write("\n\n\n[--->ADF_BottomDepth]if dataToTeensy[0][0] >= 4: b2_depth={}, exception={},".format(b2_depth, e))
                stdout.flush()
                pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on Mon May 10 18:40:20 2022

@author: Raj-Mehta
@email : rajmehta28599v@gmail.com

IMPORTANT DATA :
 1) Data relate to NUC and Teensy4.1 motion communication mode.
  1.1) mode :
   1 : mode = '1' perform motion to set the robot below the AprilTag.
   2 : mode = '2' perform motion between two AprilTag.
   3 : mode = '3' perform turn belowthe AprilTag.
   4 : mode = '4' perform Quick deacc. break while robot in motion and obstable detect in depth camera. (mode = 1,2,3)
   5 : mode = '5' perform deacc smooth break while robot in motion and obstable detect in depth camera or Estop but pressed.
   6 : mode = '6'
   7 : mode = '7' perform turning break in mode '1' while obstacle detected in depth camera.
   8 : mode = '8' perform installation script motion in between two random AprilTag.
 2) Data relate to robot main script to switching in to Charging, Functional, Installation scripts with UI.
 3) Robot ShoutDown :
    IF Estop_FB[0] == '1' and Main_SW_FB[0] == '1':
        Robot shutdown.
 4) Auto start script:
 5) Reset Teensy4.1:  # 23/11/22 11:14am
  5.1) using teensy bluetooth :
   5.1.1) connect with Bluetooth for (robotv2_p1 = BTR4_teensy) (robotv2_p2 = HC05)
   5.1.2) send '101'
 6) Reboot teensy:  # 23/11/22 11:14am
  6.1) using esp32 bluetooth:
   6.1.1) connect with bluetooth (robotv2p2 =Robot_ESP32)
   6.1.2) send '101'
  6.2) reboot button on teensy board.
 7) charging data from teensy bluetooth.  # 23/11/22 11:14am
  7.1) connect with bluetooth (robotv2_p2 = HC05).
  7.2)send '1'.
 8) Teensy relay test using Bluetooth.  # 2/2/23 12:51pm
  8.1) connect with bluetooth (robotv2_p1 = BTR4_teensy)(robotv2_p2 = HC05).
  8.2)send '2'.
change log:
    30-1-23:
        -> add ErrorCode[0] = 121 in all while loops
    08-02-2023:
        -> reduce wifi connnection time 60 to 30sec
'''
from shutil import make_archive, rmtree, move  # disk_usage
from glob import glob
from os import mkdir, remove  # path, listdir, stat, walk, scandir
from os.path import getctime, isdir  # join, , getsize
from os.path import exists, basename
# from datetime import datetime, date
from sys import stdout  # , exit
from apriltags3 import Detector
from time import time, sleep
# from re import findall
from os import system
from pandas import read_csv
# from numpy import array, zeros, float64, mean, multiply
from datetime import datetime
from serial import Serial
# from collections import OrderedDict
from itertools import groupby
from pyttsx3 import init
# from math import acos, cos, sin, radians, atan2, sqrt, tan, pi, exp
from math import atan, degrees
from socket import gethostbyname, create_connection, gethostname
from pickle import load, dump
from google_auth_oauthlib.flow import InstalledAppFlow  # Flow,
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload  # , MediaIoBaseDownload
from google.auth.transport.requests import Request
from base64 import urlsafe_b64encode
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# ServingFlag  #, playpause  # 17/11/22 1:08pm
# from socket import socket, create_connection, gethostbyname, gethostname
from playMusic_Thread import T_playMusic, playEnable
from canOpen_ethernet_Thread import E_Com, dataToTeensy, VoltageValue, RightEncData, Estop_FB, BmiYAW, CurrentValue, ErrorCode, rebootFlag  # , Main_SW_FB
from ApriltagDetectionHandlingThread_1 import ATX, ATY, ATYaw, TagID, TagDetectionState  # ATP, ATR, ATZ
from ApriltagDetectionHandlingThread_1 import T_April  # EstopPressFeedbackFlag
from ApriltagDetectionHandlingThread_1 import ATDetection_CurrentTag, ATDetection_DestinationTag
from ApriltagDetectionHandlingThread_1 import IgnoreTagList_AtThread, InputProcessEnable, ApriltagModuleFlag
from ApriltagDetectionHandlingThread_1 import t_detect0_min, t_detect480_min, b_detecty_min, capture_click
from ApriltagDetectionHandlingThread_1 import detectedSide480, detectedSide0, detectedSideB, UISwitchFlag
from Input_Method_thread import ChargingFlag, DestinationNodeList, DestinationTrayList
from Input_Method_thread import CurrentTagID, is_on, Real_nodeList
from Input_Method_thread import BluetoothInputEnable, CurrentTrayId, T_Input, WaitingForInput

from Functional_UI_asThread import T_FunUI, FinalDest_Path  # rebootFlag
from ErrorCodeANDIndication_Thread import T_ECIC


# from robot2_DepthCamera_TopCam import T_Depth, t_detect0_min, 
robot_name = gethostname()
date = datetime.now
engine = init()
engine.setProperty('rate', 240)
engine.setProperty('voice', 'english+f3')


def straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF, DestinationNodeList, CompulsoryTagList, IgnoreTagCounterMaxLimit):
    """

    Parameters.

    ----------
    VirtualNodeList : TYPE
        DESCRIPTION.
    virtualNodeDF : TYPE
        DESCRIPTION.
    dataAssignmentDF : TYPE
        DESCRIPTION.
    PathAssignmentDF : TYPE
        DESCRIPTION.
    DestinationNodeList : TYPE
        DESCRIPTION.
    CompulsoryTagList : TYPE
        DESCRIPTION.
    IgnoreTagCounterMaxLimit : TYPE
        DESCRIPTION.

    Returns
    -------
    None.
    """
    global FinalPath, IgnoreTagList, NewPathList, IgnoreTagIndexList, IgnoreTagList_AtThread
    try:
        NewPathList, IgnoreTagList, IgnoreTagIndexList = [], [], []
        IgnoreTagList_AtThread = [0]
        final_start_node, final_end_node, final_path_length = 0, 0, 0
        NewPathListIndexCounter, previous_set_point = 0, 0
        # for chageing tag ignorance limit example : len(FinalPath) = 10 and 10/1.2 = 8 IgnoreTagCounterMaxLimit to be ignored.
        # 29/11/22 3:37pm
        try:
            # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif 0 len(FinalPath)={}, IgnoreTagCounterMaxLimit={},".format(len(FinalPath), IgnoreTagCounterMaxLimit))
            # stdout.flush()
            if len(FinalPath) > 4:
                IgnoreTagCounterMaxLimit = int(len(FinalPath)/1.2)
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif 1 len(FinalPath)={}, IgnoreTagCounterMaxLimit={},".format(len(FinalPath), IgnoreTagCounterMaxLimit))
                # stdout.flush()
        except Exception as e:
            stdout.write("\n\n\n[->]straightPa()[->]elif exception={},".format(e))
            stdout.flush()

        for i in range(len(FinalPath) - 1):
            # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]for i={}, in range(len(FinalPath)-1)={},: FinalPath={},".format(i, (len(FinalPath)-1), FinalPath))
            # stdout.flush()
            # temporary not work with virtualNode.
            if FinalPath[i] in VirtualNodeList:
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if FinalPath[i] in VirtualNodeList:")
                # stdout.flush()
                answer = float(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i]) & (virtualNodeDF.RealNode == FinalPath[i+1])]['Angle']) - 180
                narrow_path_flag = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i]) & (virtualNodeDF.RealNode == FinalPath[i+1])]['IsNarrow?'])
                if answer < 0:
                    set_point = 360 + answer
                else:
                    set_point = answer
            elif FinalPath[i+1] in VirtualNodeList:
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif FinalPath[i+1]={}, FinalPath[i]={}, in VirtualNodeList:".format(FinalPath[i+1], FinalPath[i]))
                # stdout.flush()
                set_point = float(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i+1]) & (virtualNodeDF.RealNode == FinalPath[i])]['Angle'])
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif set_point={},".format(set_point))
                # stdout.flush()
                narrow_path_flag = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i+1]) & (virtualNodeDF.RealNode == FinalPath[i])]['IsNarrow?'])
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif narrow_path_flag={},".format(narrow_path_flag))
                # stdout.flush()
            else:
                set_point = float(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[i]) & (dataAssignmentDF.dnode == FinalPath[i+1])]['Tsetpoint'])
                narrow_path_flag = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[i]) & (dataAssignmentDF.dnode == FinalPath[i+1])]['IsNarrow?'])
                # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]else: FinalPath[={},]={}, FinalPath[={},+1]={}, set_point={}, narrow_path_flag={},".format(i, FinalPath[i], i, FinalPath[i+1], set_point, narrow_path_flag))
                # stdout.flush()

            if i:
                try:
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif FinalPath[i-1]={}, FinalPath[i+1]={}, (FinalPath[i+1] not in VirtualNodeList)={}, (FinalPath[i-1] not in VirtualNodeList)={},".format(FinalPath[i-1], FinalPath[i+1], (FinalPath[i+1] not in VirtualNodeList), (FinalPath[i-1] not in VirtualNodeList)))
                    # stdout.flush()
                    try:
                        if (FinalPath[i+1] not in VirtualNodeList) and (FinalPath[i-1] not in VirtualNodeList) and (FinalPath[i-1] != FinalPath[i+1]):
                            pathStraightFlag_01 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[i-1]) & (PathAssignmentDF.dn == FinalPath[i])]['pathStraightFlag'])
                            pathStraightFlag_12 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[i]) & (PathAssignmentDF.dn == FinalPath[i+1])]['pathStraightFlag'])
                            # pathStraightFlag = pathStraightFlag_12
                            # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif pathStraightFlag_01={}, pathStraightFlag_12={}, pathStraightFlag={},".format(pathStraightFlag_01, pathStraightFlag_12, pathStraightFlag))
                            # stdout.flush()
                        else:
                            pathStraightFlag_01, pathStraightFlag_12 = 0, 0
                            # pathStraightFlag = 1
                    except Exception as e:
                        stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->]elif set_point={}, exception={},".format(set_point, e))
                        stdout.flush()

                    try:
                        if type(IgnoreTagList) == list:
                            condition_status = (pathStraightFlag_01 == pathStraightFlag_12) and (len(IgnoreTagList) < IgnoreTagCounterMaxLimit) and (not narrow_path_flag)
                            # stdout.write("\n\t[->]if type(IgnoreTagList) == list: i={}, condition_status={}, pathStraightFlag={}, type(IgnoreTagList)={}, IgnoreTagList={}, IgnoreTagCounterMaxLimit={}, narrow_path_flag={},".format(i, condition_status, pathStraightFlag, type(IgnoreTagList), IgnoreTagList, IgnoreTagCounterMaxLimit, narrow_path_flag))
                            # stdout.flush()
                            # condition_status = FinalPath[FinalPath.index(FinalPath[i])-1] not in IgnoreTagList  # every one teg after tag ignored 29/11/22 3:05pm
                            condition_status = condition_status and FinalPath[FinalPath.index(FinalPath[i])-1] not in IgnoreTagList  # every one teg after tag ignored 30/11/22 11:03am
                            # stdout.write("\n\t[->]if type(IgnoreTagList) == list: i={}, condition_status={}, FinalPath[FinalPath.index(FinalPath[i])-1]={}, FinalPath[i]={},".format(i, condition_status, FinalPath[FinalPath.index(FinalPath[i])-1], FinalPath[i]))
                            # stdout.flush()
                        else:
                            condition_status = False
                            # stdout.write("\n\t[->]straightPathNodeEliminateFunction()[->]if type(IgnoreTagList) == list:else: condition_status = False")
                            # stdout.flush()
                    except Exception as e:
                        stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->]if type(IgnoreTagList) == list: exception={},".format(e))
                        stdout.flush()
                        pass
                except Exception as e:
                    condition_status = 0
                    stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->]pathStraightFlag = int(Pa exception={},".format(e))
                    stdout.flush()
                    pass

                try:
                    if type(DestinationNodeList) == list and type(DestinationNodeList[0]) == list:
                        if len(DestinationNodeList[0]):
                            condition_status = condition_status and (FinalPath[i] not in DestinationNodeList[0])
                except Exception as e:
                    stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->] not in DestinationNodeList[0] exception={},".format(e))
                    stdout.flush()
                    pass
                # stdout.write("\n\t[->]straightPathNodeEliminateFunction()[->] i={}, condition_status 1 == {}, FinalPath[i]={}, DestinationNodeList[0]={},".format(i, condition_status, FinalPath[i], DestinationNodeList[0]))
                # stdout.flush()

                try:
                    condition_status = condition_status and (FinalPath[i-1] not in VirtualNodeList) and (FinalPath[i+1] not in VirtualNodeList)
                except Exception as e:
                    stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->] (FinalPath[i+1] not in VirtualNodeList) exception={},".format(e))
                    stdout.flush()
                    pass
                # stdout.write("\n\t[->]straightPathNodeEliminateFunction()[->] i={}, condition_status 1 == {}, FinalPath[i-1]={}, VirtualNodeList={}, FinalPath[i+1]={},".format(i, condition_status, FinalPath[i-1], VirtualNodeList, FinalPath[i+1]))
                # stdout.flush()
                try:
                    condition_status = condition_status and (FinalPath[i] not in CompulsoryTagList)
                except Exception as e:
                    stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->] (FinalPath[i] not in CompulsoryTagList) exception={},".format(e))
                    stdout.flush()
                    pass
                # stdout.write("\n\t[->]straightPathNodeEliminateFunction()[->] i={}, condition_status 1 == {}, FinalPath[i]={}, CompulsoryTagList={},".format(i, condition_status, FinalPath[i], CompulsoryTagList))
                # stdout.flush()
                if condition_status:
                    # stdout.write("\n\t[->]straightPathNodeEliminateFunction()[->] if condition_status:")
                    # stdout.flush()
                    IgnoreTagList.append(FinalPath[i])
                    IgnoreTagIndexList.append(i)
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if condition_status: IgnoreTagList={}, IgnoreTagIndexList={},".format(IgnoreTagList, IgnoreTagIndexList))
                    # stdout.flush()

                    if FinalPath[i] in VirtualNodeList:
                        path_length = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i]) & (virtualNodeDF.RealNode == FinalPath[i+1])]['Distance'])
                    elif FinalPath[i+1] in VirtualNodeList:
                        path_length = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i+1]) & (virtualNodeDF.RealNode == FinalPath[i])]['Distance'])
                    else:
                        path_length = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[i]) & (dataAssignmentDF.dnode == FinalPath[i+1])]['dist2travel'])
                    final_path_length += path_length
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if condition_status: path_length={}, final_start_node={}, final_path_length={},".format(path_length, final_start_node, final_path_length))
                    # stdout.flush()

                    if not final_start_node:
                        final_start_node = FinalPath[i-1]
                        # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if condition_status: FinalPath[i-1]={}, final_start_node={},".format(FinalPath[i-1], final_start_node))
                        # stdout.flush()
                        if FinalPath[i-1] in VirtualNodeList:
                            path_length = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i-1]) & (virtualNodeDF.RealNode == FinalPath[i])]['Distance'])
                        elif FinalPath[i] in VirtualNodeList:
                            path_length = int(virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[i]) & (virtualNodeDF.RealNode == FinalPath[i-1])]['Distance'])
                        else:
                            path_length = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[i-1]) & (dataAssignmentDF.dnode == FinalPath[i])]['dist2travel'])

                        final_path_length += path_length
                        NewPathList.insert(NewPathListIndexCounter, [final_start_node])
                        stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if condition_status: path_length={}, NewPathList={}, final_path_length={}, NewPathListIndexCounter={},".format(path_length, NewPathList, final_path_length, NewPathListIndexCounter))
                        stdout.flush()
                elif final_start_node:
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->] elif final_start_node:")
                    final_end_node = FinalPath[i]
                    NewPathList[NewPathListIndexCounter].append(final_end_node)
                    NewPathList[NewPathListIndexCounter].append(final_path_length)
                    NewPathListIndexCounter += 1
                    final_start_node, final_path_length = 0, 0
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif final_start_node: path_length={}, NewPathList={}, final_path_length={}, final_end_node={},".format(path_length, NewPathList, final_path_length, final_end_node))
                    # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif final_start_node: NewPathListIndexCounter={},".format(NewPathListIndexCounter))
                    # stdout.flush()
            previous_set_point = set_point
            # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif final_start_node: previous_set_point={}, set_point={},".format(previous_set_point, set_point))
            # stdout.flush()
        if final_start_node:
            # stdout.write("\n[->]straightPathNodeEliminateFunction()[->] if final_start_node:")
            # stdout.flush()
            final_end_node = FinalPath[-1]
            NewPathList[NewPathListIndexCounter].append(final_end_node)
            NewPathList[NewPathListIndexCounter].append(final_path_length)
            final_start_node, final_path_length = 0, 0
            stdout.write("\n[->]straightPathNodeEliminateFunction()[->]if final_start_node: NewPathList={}, final_end_node={}, final_path_length={}, NewPathListIndexCounter={},".format(NewPathList, final_end_node, final_path_length, NewPathListIndexCounter))
            stdout.flush()

        # stdout.write("\n[->]elif TagID[0] != HomeNode:[->] 1 IgnoreTagList_AtThread[0]={}, IgnoreTagList={},".format(IgnoreTagList_AtThread[0], IgnoreTagList))
        # stdout.write("\n[->]elif TagID[0] != HomeNode:[->] 2 here")
        IgnoreTagList_AtThread[0] = IgnoreTagList
        # # IgnoreTagListFlag_AtThread[0] = 1
        stdout.write("\n[->]elif Tagread[0]={}, IgnoreTagList={},".format(IgnoreTagList_AtThread[0], IgnoreTagList))
        stdout.flush()

        if len(IgnoreTagList):
            # Remove all 'ignore nodes' except destination nodes # just for safety
            # updated_ignore_tag_list = list(set(IgnoreTagList)-set(FinalDestinationNodeList))
            # FinalPath = [node_value for node_value in FinalPath if node_value not in updated_ignore_tag_list]
            FinalPath = [FinalPath[i] for i in range(len(FinalPath)) if i not in IgnoreTagIndexList]

        # stdout.write("\n[->]straightPathNodeEliminateFunction()[->]try: FinalPath={}, IgnoreTagList={}, FinalDestinationNodeList={}, NewPathList={},".format(FinalPath, IgnoreTagList, FinalDestinationNodeList, NewPathList))
        stdout.write("\n[->]straightPathNodeEliminateFunction()[->]try: FinalPath={}, IgnoreTagList={}, NewPathList={},".format(FinalPath, IgnoreTagList, NewPathList))
        stdout.flush()
    except Exception as e:
        stdout.write("\n\n\n[->]straightPathNodeEliminateFunction()[->] exception={},".format(e))
        stdout.flush()
        pass
#  straightPathNodeEliminateFunction end


def is_connected():
    try:
        # see if we can resolve the host name -- tells us if there is
        # a DNS listening
        host = gethostbyname("www.google.com")
        # connect to the host -- tells us if the host is actually
        # reachable
        s = create_connection((host, 80), 2)
        stdout.write("\n[->]is_connected()[->] s={},".format(s))
        stdout.flush()
        return True
    except:
        pass
    return False


def Create_Service(client_secret_file, api_name, api_version, *scopes):
    # print(client_secret_file, api_name, api_version, scopes, sep='-')
    CLIENT_SECRET_FILE = client_secret_file
    API_SERVICE_NAME = api_name
    API_VERSION = api_version
    SCOPES = [scope for scope in scopes[0]]
    # print(SCOPES)

    cred = None

    pickle_file = 'token_' + API_SERVICE_NAME + '_' + API_VERSION + '.pickle'
    # print(pickle_file)

    if exists(pickle_file):
        with open(pickle_file, 'rb') as token:
            cred = load(token)

    if not cred or not cred.valid:
        if cred and cred.expired and cred.refresh_token:
            cred.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(CLIENT_SECRET_FILE, SCOPES)
            cred = flow.run_local_server()

        with open(pickle_file, 'wb') as token:
            dump(cred, token)

    try:
        service = build(API_SERVICE_NAME, API_VERSION, credentials=cred)
        # print(API_SERVICE_NAME, 'service created successfully')
        return service
    except Exception as e:
        stdout.write("\n\n\n[->]main()[->]Create_Service exception={},".format(e))
        stdout.flush()
        return None

def send_email(service, to, subject, body):
    for to_mail in to:
        message = MIMEMultipart()
        message['to'] = to_mail
        message['subject'] = subject
        message.attach(MIMEText(body))
        create_message = {'raw': urlsafe_b64encode(message.as_bytes()).decode()}
        send_message = (service.users().messages().send(userId="me", body=create_message).execute())
        stdout.write("\n[-->FUT] sent message to {} Message Id: {};".format(to_mail,send_message["id"]))
        stdout.flush()


def main():
    """

    Execute all other functions.

    Returns
    -------
    None.

    """
    global dataToTeensy, capture_click, WaitingForInput, FinalDest_Path, rebootFlag
    global FinalPath, IgnoreTagList, NewPathList, IgnoreTagIndexList, IgnoreTagList_AtThread
    global CurrentTagID, ErrorCode, DestinationNodeList, DestinationTrayList

    try:
        # system("service bluetooth stop")
        # stdout.write("\n[->]main()[->] service bluetooth stop")
        # stdout.flush()
        system("nmcli radio wifi on")
        stdout.write("\n[->]main()[->] nmcli radio wifi on")
        stdout.flush()
    except Exception as e:
        stdout.write("\n\n\n[->]main()[->] wifi bluetooth off exception={},".format(e))
        stdout.flush()
        pass
    try:
        ESP32_Ser = Serial('/dev/ESP32', 115200, timeout=.1, write_timeout=0)
    except Exception as e:
        stdout.write("\n\n\n[->]main()[->]ESP32_Ser reboot exception={},".format(e))
        stdout.flush()

        T_playMusic[0].start()
        stdout.write("\n[->]main[->] T_playMusic[0].start()")
        stdout.flush()
        sleep(1)
        E_Com[0].start()
        sleep(0.2)
        stdout.write("\n[->]main[->] E_Com[0].start()")
        stdout.flush()
        T_April[0].start()
        sleep(0.4)
        stdout.write("\n[->]main[->] T_April[0].start()")
        stdout.flush()
        T_Input[0].start()
        sleep(0.1)
        stdout.write("\n[->]main[->] T_Input[0].start()")
        stdout.flush()
        T_FunUI[0].start()
        sleep(0.4)
        stdout.write("\n[->]main[->] T_FunUI[0].start()")
        stdout.flush()
        T_ECIC[0].start()
        sleep(0.1)
        stdout.write("\n[->]main[->] T_ECIC[0].start() pause")
        stdout.flush()

        rebootFlag[0] = 1
        ChargingFlag[0] = 0
        conWait_t1 = conWait_t2 = time()
        while not is_connected():
            conWait_t2 = time() - conWait_t1
            if conWait_t2 > 10:
                break

        if is_connected():
            Client_Serect_File = 'client_secret_gmail.json'
            API_NAME = 'gmail'
            API_VERSION = 'v1'
            SCOPES = ['https://www.googleapis.com/auth/gmail.send']
            service_test = Create_Service(Client_Serect_File, API_NAME, API_VERSION, SCOPES)
            subject = 'Noreply: REBOOT ALERT ' + robot_name
            msg = 'This email is system generated to notify developer that, the reboot has been generated. \nrobot_name = ' + robot_name + '\ntime = ' + str(date().strftime("%d-%m-%Y %H:%M:%S:%f")) + '\nerror = ESP32_Ser reboot exception ' + str(e) 
            send_email(service_test,['aanal.patel@petpooja.com', 'raj.mehta@petpooja.com'], subject, msg)
        else:
            stdout.write("\n[-->main]--> internet_connction, no internet connection: esp port,")
            stdout.flush()
        # sleep(2)
        # system("sudo reboot")
        while rebootFlag[0]:
            rebootFlag[0] = 1

    LedData1, prev_LedData1 = [''], ''
    LedData1[0] = '1,3\n'
    if LedData1[0] != prev_LedData1:
        # ESP32_Ser.write(LedData1[0].encode())
        stdout.write("\n[-->ESP32_T] LedData1[0]={},".format(LedData1[0]))
        stdout.flush()
        prev_LedData1 = LedData1[0]

    # stdout.write("\n[->]main()[->] setup1 Start"); stdout.flush()
    # stdout.write("\n[->]main()[->] 148 ")
    BluetoothInputEnable[0] = 1
    # PreviousTAG_ID, ImageClickFlag = 0, 1
    # ImageClickIndex = 0
    ImageClickLimit, ImageClick_count = 3, 0
    # neighbour_node, distance, offset_degree
    # NeighbourNodeMetadataList = []
    # set_point = 0
    # travel()
    FinalPathListCounter, DestinationNode, path_straight_enable = 0, 1, 0
    PreviousSmoothTurnEnable, SmoothTurnEnable, ReschedulePathCase2 = 0, 0, 0
    AchieveLineFuncationFlag, fpath_straight_enable = 0, 0
    # EstopPressFeedbackFlag, ATDetection_CurrentTag, ATDetection_DestinationTag = [0], [0], [0]
    dist2travel = 3000
    FoodServingCase = [0]
    DefaultYaw, previous_node_index = 0, 0
    flagonce, MotionSection = -1, 0
    FinalDestinationNodeList, FinalPath = [], []
    FinalPathDictionary = {}
    # motion
    futureTsetpoint, OrderServeCounter = 0, 0
    OrderServeFile = open("OrderServeDataFile.csv", "a")
    OrderServeFile.write("OrderServeCounter={},TableNumber,CurrentTagID,WCS_Enable,TrayId,total_travel,Voltage,Current,serving_Start,serving_End,serving_time\n".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    OrderServeFile.flush()
    OrderServeFile.close()
    '''
    change as per requirement
    '''
    # CompulsoryTagList = [48, 32, 31, 27, 50, 36, 28, 24]  # 5  29/11/22 7:03pm
    # CompulsoryTagList = [32, 31, 27, 50, 36, 28, 24]  # 5  30/11/22 7:03pm
    # CompulsoryTagList = [24, 27, 28, 29, 30, 31, 32, 36, 38, 50]
    CompulsoryTagList = [12, 13]
    IgnoreTagCounterMaxLimit = 1  # 2  15/11/22 10:51am
    # travel()
    StraightPathNodeAvoidEnableFlag = 1  # nothing important but requre if you want.
    stdout.write("\n[ADHT]ApriltagDetectionHandlingThread_1[->] start 1")
    stdout.flush()

    LedData1[0] = '1,5\n'
    # if LedData1[0] != prev_LedData1:
    # ESP32_Ser.write(LedData1[0].encode())
    stdout.write("\n[-->ESP32_T] LedData1[0]={}, time={},".format(LedData1[0], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    stdout.flush()
    prev_LedData1 = LedData1[0]
    # updated on 10-01-2023 12:50
    stdout.write("\n[-->main]--> zip_(),start time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    stdout.flush()
    try:
        try:
            d1 = date().strftime("%d_%m_%Y_%H_%M")
            list_of_files_out = glob("./*.out")
            full_path_out = ["{0}".format(x) for x in list_of_files_out]
            if len(full_path_out) > 0:
                newest_file = max(full_path_out, key=getctime)
                list_of_files_out.remove(newest_file)
            list_of_files_Img = glob("./log/*.jpeg")
        except Exception as e:
            stdout.write("\n\n\n[-->main]--> zip_():path list, time={}, exception={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), e))
            stdout.flush()
            pass
        directory = "./log/temp"
        if not isdir(directory): mkdir(directory)

        for file in list_of_files_out: move(file, directory)
        for img in list_of_files_Img: move(img, directory)
        try:
            output_zip = "./log/" + str(d1) + "_out_img"
            make_archive(output_zip, 'zip', directory)
            rmtree(directory)
        except Exception as e:
            stdout.write("\n\n\n\n[-->main]--> zip_():make_archive, time={}, exception={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), e))
            stdout.flush()
            pass
        stdout.write("\n[-->main] logs taken; time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
        stdout.flush()
    except Exception as e:
        stdout.write("\n\n\n[-->main]--> zip_(), time={}, exception={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), e))
        stdout.flush()
        pass
    stdout.write("\n[-->main]--> zip_(),end time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    stdout.flush()
    T_playMusic[0].start()
    stdout.write("\n[->]main[->] T_playMusic[0].start()")
    stdout.flush()
    sleep(1)
    E_Com[0].start()
    sleep(0.2)
    stdout.write("\n[->]main[->]618 E_Com[0].start()")
    stdout.flush()
    T_April[0].start()
    sleep(0.4)
    stdout.write("\n[->]main[->] T_April[0].start()")
    stdout.flush()
    T_Input[0].start()
    sleep(0.1)
    stdout.write("\n[->]main[->] T_Input[0].start()")
    stdout.flush()
    T_FunUI[0].start()
    sleep(0.4)
    stdout.write("\n[->]main[->] T_FunUI[0].start()")
    LedData1[0] = '1,7\n'
    # if LedData1[0] != prev_LedData1:
    # ESP32_Ser.write(LedData1[0].encode())
    stdout.write("\n[-->ESP32_T] LedData1[0]={},".format(LedData1[0]))
    stdout.flush()
    prev_LedData1 = LedData1[0]

    T_ECIC[0].start()
    sleep(0.1)
    stdout.write("\n[->]main[->] T_ECIC[0].start() pause")
    stdout.flush()
    stdout.write("\n[->]main[->] start time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    stdout.flush()
    conWait_t1 = conWait_t2 = time()
    while not is_connected():
        # ESP32_Ser.write(LedData1[0].encode())
        conWait_t2 = time() - conWait_t1
        if conWait_t2 > 60:
            break
    stdout.write("\n[->]main[->] end time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
    stdout.flush()

    try:
        successful_upload = []
        failed_upload = []
        response = None
        response_update = None
        if is_connected():
            Client_Serect_File = 'client_secret.json'
            API_NAME = 'drive'
            API_VERSION = 'v3'
            SCOPES = ['https://www.googleapis.com/auth/drive']

            service_test = Create_Service(Client_Serect_File, API_NAME, API_VERSION, SCOPES)

            file_name_csv = 'OrderServeDataFile.csv'
            mime_type_csv = 'text/csv'

            file_metadata = {
                'name': 'OrderServeDataFile.csv',
                'mimeType': 'text/csv',
                'parents': ["1EJy1ztX3uEUhCEZuDvxQy2g8Jxdu2tI8"]
            }

            file_name_csv = file_metadata['name']
            query = "name='{}' and trashed=false and parents='1EJy1ztX3uEUhCEZuDvxQy2g8Jxdu2tI8'".format(file_name_csv)
            results = service_test.files().list(q=query, fields="nextPageToken, files(id, name)").execute()
            filesOnDrive = results.get("files", [])

            if not filesOnDrive:
                media = MediaFileUpload('{}'.format(file_name_csv), mimetype=mime_type_csv)
                response = service_test.files().create(supportsTeamDrives=True, body=file_metadata, media_body=media, fields='id').execute()
            else:
                file_id = filesOnDrive[0].get('id')
                media = MediaFileUpload('{}'.format(file_name_csv), mimetype=mime_type_csv)
                response_update = service_test.files().update(fileId=file_id, media_body=media, supportsTeamDrives=True).execute()
            list_of_files_zip = glob("./log/*.zip")
            full_path = ["{0}".format(x) for x in list_of_files_zip]

            for fileToUpload in full_path:
                service_test = Create_Service(Client_Serect_File, API_NAME, API_VERSION, SCOPES)
                base_file_name = basename(fileToUpload)
                file_name = base_file_name
                mime_type = 'application/zip'

                file_metadata = {
                    'name': base_file_name,
                    'mimeType': 'application/zip',
                    'parents': ["1EJy1ztX3uEUhCEZuDvxQy2g8Jxdu2tI8"]
                }

                file_name = file_metadata['name']
                query = "name='{}' and trashed=false and parents='1EJy1ztX3uEUhCEZuDvxQy2g8Jxdu2tI8'".format(file_name)
                results = service_test.files().list(q=query, fields="nextPageToken, files(id, name)").execute()
                filesOnDrive = results.get("files", [])

                if not filesOnDrive:
                    media = MediaFileUpload('./log/{}'.format(file_name), mimetype=mime_type)
                    response = service_test.files().create(supportsTeamDrives=True, body=file_metadata, media_body=media, fields='id').execute()
                else:
                    file_id = filesOnDrive[0].get('id')
                    media = MediaFileUpload('./log/{}'.format(file_name), mimetype=mime_type)
                    response_update = service_test.files().update(fileId=file_id, media_body=media, supportsTeamDrives=True).execute()

                if (response and 'id' in response) or (response_update and 'id' in response_update):
                    successful_upload.append(fileToUpload)
                else:
                    failed_upload.append(fileToUpload)
            if len(successful_upload) == len(full_path):
                stdout.write("\n[-->main]--> internet_connction, All files uploaded successfully,")
                stdout.flush()
                for fileToDelete in full_path: remove(fileToDelete)
            else:
                stdout.write("\n[-->main]--> internet_connction:some file failed to upload, Successfully uploaded files = {}, Failed to upload files = {};".format(successful_upload, failed_upload))
                stdout.flush()
        else:
            stdout.write("\n[-->main]--> internet_connction, no internet connection: files upload,")
            stdout.flush()
    except Exception as e:
        stdout.write("\n\n\n[-->main]--> internet_connction, time={}, exception={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), e))
        stdout.flush()
        # pass
    if not rebootFlag[0]:
        # system("nmcli radio wifi off")
        # stdout.write("\n[->]main()[->] nmcli radio wifi off")
        # stdout.flush()
        pass
    # main
    InputProcessEnable[0], printFlag = 0, True
    stdout.write("\n[->]main()[->] while UISwitchFlag={}, ApriltagModuleFlag[0]={},".format(UISwitchFlag[0], ApriltagModuleFlag[0]))
    error_allow_yaw, error_allow_y = 1, 10
    voltage_counter1, voltage_counter2 = time(), time()
    prev_UISwitchFlag, main_t1 = -2, time()
    InputProcessEnable_count1, InputProcessEnable_count2 = time(), time()
    total_travel = total_travel_0 = 0
    LowBatteryVoltageLimit = 20.5
    serving_t1 = serving_t2 = serving_t3 = time()
    setx_temp = sety_temp = 0
    while True:
        main_t2 = time() - main_t1
        if main_t2 > (0.00001):   # 0.0001
            main_t1 = time()
            if UISwitchFlag[0] == 1:
                # if (InputProcessEnable[0] or WaitingForInput[0] == 1):
                InputProcessEnable[0] = WaitingForInput[0] = 0
            elif UISwitchFlag[0] == -1:
                if UISwitchFlag[0] != prev_UISwitchFlag:
                    try:
                        # CommonVariables2.csv
                        fileName = "./Configurations/CommonVariables2.csv"
                        CommonVariableDF = read_csv(fileName).drop(columns=['Comment', 'Parameter']).dropna(subset=["Name"]).set_index('Name')
                        ChargedBatteryVoltageLimit = float(CommonVariableDF.loc['ChargedBatteryVoltageLimit'])  # 25
                        del CommonVariableDF
                        flagonce = -1
                        prev_UISwitchFlag = UISwitchFlag[0]
                    except Exception as e:
                        stdout.write("\n\n\n[->]main()[->]elif UISwitchFlag[0] == -1:if not UISwitchFlag[0]: main_t2={}, exception={},".format(main_t2, e))
                        stdout.flush()
                        pass
                if float(CurrentValue[0]) > 0:
                    LedData1[0] = '2,' + str(VoltageValue[0]) + '\n'
                    if LedData1[0] != prev_LedData1:
                        # ESP32_Ser.write(LedData1[0].encode())
                        stdout.write("\n[-->ESP32_T]elif UISwitchFlag[0] == -1: main_t2={}, LedData1[0]={},".format(main_t2, LedData1[0]))
                        stdout.flush()
                        prev_LedData1 = LedData1[0]
                    if float(VoltageValue[0]) > ChargedBatteryVoltageLimit:
                        ChargingFlag[0] = 0
                        InputProcessEnable[0], WaitingForInput[0] = 0, 1
                        stdout.write("\n[->]inputFunction()[->]elif UISwitchFlag[0] == -1:Chrging Station Charged VoltageValue(20.5 to 26v)={}, time={}, main_t2={},".format(VoltageValue[0], date().strftime("%d-%m-%Y %H:%M:%S:%f"), main_t2))
                        stdout.flush()
                    else:
                        InputProcessEnable[0], WaitingForInput[0] = 1, 0
                elif float(VoltageValue[0]) > LowBatteryVoltageLimit and ChargingFlag[0]:
                    ChargingFlag[0] = InputProcessEnable[0] = 0
                    # ErrorCode[0] = 122
                    stdout.write("\n[->]main()[->]elif float(Voltag ChargingFlag[0]={}, main_t2={},".format(ChargingFlag[0], main_t2))
                    stdout.flush()
            elif not UISwitchFlag[0]:
                if UISwitchFlag[0] != prev_UISwitchFlag:
                    try:
                        # path_assignment_2.csv
                        fileName = "./Configurations/path_assignment_2.csv"
                        PathAssignmentDF = read_csv(fileName)
                        # TableFaceAngle_2.csv
                        fileName = "./Configurations/TableFaceAngle_2.csv"
                        tableFaceAngleDF = read_csv(fileName)
                        #  ErrorCodeList.csv
                        fileName = "./Configurations/ErrorCodeList.csv"
                        VoiceFilePath = './Thread/new_voice_pooja'
                        ErrorCodeDF = read_csv(fileName).dropna(subset=["Error Name"]).set_index("Error Name")
                        OrderServingCase_VoiceFile = ErrorCodeDF.loc['Order Serving Case']['Voice File'].replace('(', '').split(')')
                        # UsedFoodPlatesCollectionCase_VoiceFile = ErrorCodeDF.loc['Used Food Plates Collection Case']['Voice File'].split('.')
                        OrderServe_Tray1_VoiceFile = OrderServingCase_VoiceFile[0].split('.')
                        OrderServe_Tray2_VoiceFile = OrderServingCase_VoiceFile[1].split('.')
                        OrderServe_Tray3_VoiceFile = OrderServingCase_VoiceFile[2].split('.')
                        OrderServe_Tray12_VoiceFile = OrderServingCase_VoiceFile[3].split('.')
                        OrderServe_Tray23_VoiceFile = OrderServingCase_VoiceFile[4].split('.')
                        OrderServe_Tray13_VoiceFile = OrderServingCase_VoiceFile[5].split('.')
                        OrderServe_General_VoiceFile = OrderServingCase_VoiceFile[6].split('.')
                        OrderServe_Water_VoiceFile = OrderServingCase_VoiceFile[7].split('.')
                        del ErrorCodeDF
                        # VirtualNodeMapping.csv
                        fileName = "./Configurations/VirtualNodeMapping.csv"
                        virtualNodeDF = read_csv(fileName)
                        VirtualNodeList = list(virtualNodeDF.VirtualNode)
                        stdout.write("\n[->]main()[->] main_t2={},".format(main_t2))
                        stdout.flush()
                        # TagId.csv
                        fileName = "./Configurations/TagId.csv"
                        TagIdDF = read_csv(fileName)
                        # CommonVariables2.csv
                        fileName = "./Configurations/CommonVariables2.csv"
                        CommonVariableDF = read_csv(fileName).drop(columns=['Comment', 'Parameter']).dropna(subset=["Name"]).set_index('Name')
                        HomeNode = int(CommonVariableDF.loc['HomeNode'])
                        ChargingNode = int(CommonVariableDF.loc['ChargingNode'])
                        HypotenusePathLength = int(CommonVariableDF.loc['HypotenusePathLength'])
                        HomeTagId = int(CommonVariableDF.loc['HomeNode'])
                        # CombineTrayNumber = int(CommonVariableDF.loc['CombineTrayNumber'])
                        # PlateCollectionTrayNumber = int(CommonVariableDF.loc['PlateCollectionTrayNumber'])
                        ChargedBatteryVoltageLimit = float(CommonVariableDF.loc['ChargedBatteryVoltageLimit'])  # 25
                        LowBatteryVoltageLimit = float(CommonVariableDF.loc['LowBatteryVoltageLimit'])  # 20.5
                        del CommonVariableDF

                        # Data_Assignment_4.csv
                        fileName = "./Configurations/Data_Assignment_4.csv"
                        dataAssignmentDF = read_csv(fileName)
                        # NodeListDF = read_csv(fileName).set_index('cnode')
                        # AvailableNodeList = list(dict.fromkeys(NodeListDF.index.tolist()))
                        # RealNodeList = AvailableNodeList  # list(set(dataAssignmentDF.cnode))
                        # del NodeListDF
                        AvailableNodeList = RealNodeList = list(set(dataAssignmentDF.cnode))
                        stdout.write("\n[->]main()[->] AvailableNodeList={},".format(AvailableNodeList))
                        stdout.write("\n[->]main()[->] RealNodeList={}, main_t2={},".format(RealNodeList, main_t2))
                        stdout.flush()
                        # flagonce = -1
                        prev_UISwitchFlag = UISwitchFlag[0]
                    except Exception as e:
                        stdout.write("\n\n\n[->]main()[->]if not UISwitchFlag[0]: main_t2={}, exception={},".format(main_t2, e))
                        stdout.flush()
                        pass

                # check battery voltage data.
                try:
                    voltage_counter2 = time() - voltage_counter1
                    # LowBatteryVoltageLimit = 18  # 11/11/22 12:18pm
                    if float(VoltageValue[0]) and not ChargingFlag[0] and float(VoltageValue[0]) < LowBatteryVoltageLimit and voltage_counter2 > 1:
                        ChargingFlag[0] = 1
                        LedData1[0] = '10'
                        if LedData1[0] != prev_LedData1:
                            # ESP32_Ser.write(LedData1[0].encode())
                            stdout.write("\n[-->ESP32_T]if float(VoltageValue[ please charge me LedData1[0]={}, time={},".format(LedData1[0], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                            stdout.flush()
                            prev_LedData1 = LedData1[0]
                        # ErrorCode[0] = 122
                        stdout.write("\n[->]main()[->]if main_t2={}, > (0.0001): ChargingFlag[0]={}, main_t2={},".format(main_t2, ChargingFlag[0], main_t2))
                        stdout.flush()
                    elif float(CurrentValue[0]) > 0:
                        ChargingFlag[0], UISwitchFlag[0] = 1, -1
                        stdout.write("\n[->]main()[->]elif float(CurrentValue[0])>0: time={}, main_t2={}, ChargingFlag[0]={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), main_t2, ChargingFlag[0]))
                        stdout.flush()
                    elif float(VoltageValue[0]) >= LowBatteryVoltageLimit:
                        voltage_counter1 = time()
                except Exception as e:
                    stdout.write("\n\n\n[->]main()[->]voltage_counter main_t2={}, exception={},".format(main_t2, e))
                    stdout.flush()
                    pass
                # stdout.write("\n[->]main()[->] detectedSide480={}, detectedSide0={}, detectedSideB={},".format(detectedSide480[0], detectedSide0[0], detectedSideB[0]))
                # stdout.write("\n[->]main()[->] TagID[0]={}, HomeNode={}, len(FinalPath)={},".format(TagID[0], HomeNode, len(FinalPath)))
                # Robot starting set below to home node
                if ApriltagModuleFlag[0] == 6 and TagID[0] == HomeNode and not len(FinalPath):
                    # stdout.write("\n[->]main()[->]if TagID[0] == HomeNode and not len(FinalPath): TagID[0]={}, HomeNode={}, len(FinalPath)={},".format(TagID[0], HomeNode, len(FinalPath)))
                    # stdout.flush()
                    try:
                        # stdout.write("\n[->]main()[->] flagonce={}, InputProcessEnable[0]={}, CurrentTagID[0]={}, HomeNode={},".format(flagonce, InputProcessEnable[0], CurrentTagID[0], HomeNode))
                        # stdout.flush()
                        if flagonce == -1:
                            flagonce = 0
                            stdout.write("\n[->]inputFunction()[->] while end main_t2={},".format(main_t2))
                            stdout.flush()
                        if flagonce == 2 or flagonce == 0:
                            if TagID[0]:
                                CurrentTagID[0] = TagID[0]
                            if (not InputProcessEnable[0]) and CurrentTagID[0] == HomeNode:
                                AtTableRobotFace = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == HomeNode)].iloc[0, 2])  # 16/11/22 8:08pm
                                setXtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 4])
                                setYtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 3])
                                stdout.write("\n[->]main()[->]main_t2={}, AtTableRobotFace={}, setXtableFaceAngleDF={}, setYtableFaceAngleDF={}, main_t2={},".format(main_t2, AtTableRobotFace, setXtableFaceAngleDF, setYtableFaceAngleDF, main_t2))
                                stdout.flush()
                                # LedData1[0] = '6\n'
                                # if LedData1[0] != prev_LedData1:
                                    # ESP32_Ser.write(LedData1[0].encode())
                                    # stdout.write("\n[-->ESP32_T] main_t2={}, LedData1[0]={},".format(main_t2, LedData1[0]))
                                    # stdout.flush()
                                    # prev_LedData1 = LedData1[0]
                                break_mode_t1 = break_mode_t2 = time()
                                flagonce = ImageClick_count = capture_click[0] = 0
                                mode = pmode = ''

                                while flagonce != 5:

                                    if Estop_FB[0] == '1': ErrorCode[0] = 121
                                    if float(CurrentValue[0]) > 0: break

                                    if flagonce == 0: setYaw, SetY = (AtTableRobotFace - 90) % 360, int(ATY[0] * 1000)
                                    elif flagonce == 1: 
                                        if TagID[0] == HomeNode and TagDetectionState[0]:
                                            setYaw, SetY = ATYaw[0], setXtableFaceAngleDF
                                        else:
                                            setYaw, SetY = ATYaw[0], int(ATY[0] * 1000) - 200
                                    elif flagonce == 2: setYaw, SetY = (AtTableRobotFace) % 360, int(ATY[0] * 1000)
                                    elif flagonce == 3: setYaw, SetY = ATYaw[0], setYtableFaceAngleDF
                                    elif flagonce == 4: setYaw, SetY = (AtTableRobotFace) % 360, int(ATY[0] * 1000)

                                    if UISwitchFlag[0] == 1:
                                        mode = '0'
                                        dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                        break

                                    if TagID[0] == HomeNode and TagDetectionState[0]:
                                        conditionMatch = abs(SetY - (ATY[0] * 1000)) < error_allow_y and abs(setYaw - ATYaw[0]) < error_allow_yaw
                                        tagState = 1
                                    else:
                                        conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)
                                        tagState = 0
                                    # mode = '1'
                                    if flagonce == 0:
                                        mode = '1'
                                        if conditionMatch:
                                            flagonce, conditionMatch, mode = 1, 0, '1'
                                            # stdout.write("\n[->]main()[->] if flagonce == 0 and conditionMatch: flagonce = {}, TagDetectionState[0] = {}, tagState = {}, ATY = {};".format(flagonce, TagDetectionState[0], tagState, round(ATY[0] * 1000,2)))
                                            # stdout.flush()
                                    elif conditionMatch:
                                        if flagonce == 1: 
                                            # if TagID[0] == HomeNode and TagDetectionState[0]:
                                                flagonce, conditionMatch, mode = 2, 0, '1'
                                            # else:
                                                # flagonce, conditionMatch, mode = 1, 0, '1'
                                        elif flagonce == 2: flagonce, conditionMatch, mode = 3, 0, '1'
                                        elif flagonce == 3: flagonce, conditionMatch, mode = 4, 0, '1'
                                        elif flagonce == 4:
                                            flagonce, conditionMatch, mode = 5, 0, '0'
                                            InputProcessEnable_count1 = time()
                                    # depth error image capture  5/12/22 10:57am
                                    # if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                        # capture_click[0] = 1
                                    # elif capture_click[0] == 3:
                                        # ImageClick_count += 1
                                        # capture_click[0] = 0
                                    break_mode_t2 = time() - break_mode_t1
                                    if t_detect480_min[0] < 2:
                                        mode = '7'
                                        if break_mode_t2 > 0.2:
                                            temp_ATY = int(ATY[0] * 1000) + 100
                                            while t_detect480_min[0] < 2 and temp_ATY <= 200:
                                                mode = '1'
                                                setYaw, SetY = ATYaw[0], temp_ATY
                                                if TagID[0] == HomeNode:
                                                    dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                            break_mode_t1 = time()
                                    elif mode == '1': break_mode_t1 = time()
                                    elif mode == '7': mode = '1'
                                    pmode = mode
                                    # mode, TagDetectionState[0], ATYaw[0], setYaw, ATY[0], SetY
                                    # dataToTeensy = 1,1,101.87,105,103.12,108
                                    SetY = round(SetY, 2)
                                    if TagID[0] == HomeNode:
                                        dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                    else:
                                        dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(BmiYAW[0])) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                        
                                flagonce = 0
                                # AprilTag camera on/off
                                # off => InputProcessEnable[0] = 1
                                # InputProcessEnable[0] = 1
                                InputProcessEnable_count2 = time() - InputProcessEnable_count1
                                stdout.write("\n[->]inputFunction()[->]0 InputProcessEnable[0]={}, main_t2={}, InputProcessEnable_count2={},".format(InputProcessEnable[0], main_t2, InputProcessEnable_count2))
                                stdout.flush()
                                if not ChargingFlag[0] and not InputProcessEnable[0] and (InputProcessEnable_count2 > 5):
                                    stdout.write("\n[->]inputFunction()[->]1 InputProcessEnable[0]=1, main_t2={}, InputProcessEnable_count2={},".format(main_t2, InputProcessEnable_count2))
                                    stdout.flush()
                                    InputProcessEnable[0], playEnable[0] = 1, 0
                                    InputProcessEnable_count1 = time()
                                # stdout.write("\n[->]inputFunction()[->] InputProcessEnable[0] = 1")
                                LedData1[0] = '3,25\n'
                                if LedData1[0] != prev_LedData1:
                                    # ESP32_Ser.write(LedData1[0].encode())
                                    stdout.write("\n[-->ESP32_T] LedData1[0]={}, main_t2={},".format(LedData1[0], main_t2))
                                    stdout.flush()
                                    prev_LedData1 = LedData1[0]
                                stdout.write("\n\t\t\t\t\t[->]inputFunction()[->] *****Waiting For Input***** time={}, main_t2={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), main_t2))
                                stdout.flush()
                                WaitingForInput[0], flagonce, MotionSection = 1, 3, 0
                        elif WaitingForInput[0]:
                            InputProcessEnable_count2 = time() - InputProcessEnable_count1
                            if (not InputProcessEnable[0]) and InputProcessEnable_count2 > 5:
                                stdout.write("\n[->]inputFunction()[->]2 InputProcessEnable[0]={}, main_t2={}, InputProcessEnable_count2={},".format(InputProcessEnable[0], main_t2, InputProcessEnable_count2))
                                stdout.flush()
                                InputProcessEnable_count1, InputProcessEnable[0] = time(), 1
                    except Exception as e:
                        stdout.write("\n\n\n[->]main()[->]if TagID[0] == HomeNode: main_t2={}, exception={},".format(main_t2, e))
                        stdout.flush()
                        pass

                if ChargingFlag[0] == 1 and WaitingForInput[0] == 1:
                    stdout.write("\n[->]if ChargingFlag[0] == 1 and WaitingForInput[0] == 1: WaitingForInput[0]={}, ApriltagModuleFlag[0]={}, main_t2={},".format(WaitingForInput[0], ApriltagModuleFlag[0], main_t2))
                    stdout.flush()
                    WaitingForInput[0] = 0
                # inputFunction() 1
                # 1) assign CurrentTagID[0] value
                # 2) InputProcessEnable[0]
                if (TagID[0] or CurrentTagID[0] in VirtualNodeList) and (ApriltagModuleFlag[0] == 6 and not WaitingForInput[0] and (float(CurrentValue[0]) < 0) or (TagID[0] != HomeNode and not len(FinalPath))):
                    # stdout.write("\n[->]if Wait WaitingForInput[0]={}, ApriltagModuleFlag[0]={}, CurrentValue[0]={},".format(WaitingForInput[0], ApriltagModuleFlag[0], CurrentValue[0]))
                    # stdout.flush()
                    # stdout.write("\n[->]0 while not FinalPath={},".format(FinalPath))
                    # stdout.flush()
                    if TagID[0] != HomeNode and not len(FinalPath): AchieveLineFuncationFlag = 1
                    voltage_counter1, voltage_counter2 = time(), time()
                    playEnable[0] = 1
                    # stdout.write("\n[->]1 while not FinalPath={},".format(FinalPath))
                    # stdout.flush()
                    while not len(FinalPath):
                        # InputProcessEnable[0] = 0
                        try:
                            voltage_counter2 = time() - voltage_counter1
                            # LowBatteryVoltageLimit = 18  # 11/11/22 12:18pm
                            if float(VoltageValue[0]) != 0 and float(VoltageValue[0]) < LowBatteryVoltageLimit and voltage_counter2 > 1:
                                stdout.write("\n[->]while not len(FinalPath): VoltageValue[0]={}, voltage_counter2={}, main_t2={},".format(VoltageValue[0], voltage_counter2, main_t2))
                                stdout.flush()
                                ChargingFlag[0] = 1
                                ErrorCode[0] = 122
                                break  # 3/12/22 11:17am
                            elif float(CurrentValue[0]) > 0:
                                ChargingFlag[0], UISwitchFlag[0] = 1, -1
                                stdout.write("\n[->]while not len(FinalPath): CurrentValue[0]={}, main_t2={},".format(CurrentValue[0], main_t2))
                                stdout.flush()
                                break
                            elif float(VoltageValue[0]) >= LowBatteryVoltageLimit:
                                voltage_counter1 = time()
                        except Exception as e:
                            stdout.write("\n\n\n[->]main()[->]while not len(FinalPath): voltage_counter main_t2={}, exception={},".format(main_t2, e))
                            stdout.flush()
                            pass
                        if CurrentTagID[0] in VirtualNodeList:
                            if type(DestinationNodeList[0]) == list and len(DestinationNodeList[0]):
                                try:
                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu: CurrentTagID[0]={}, DestinationNodeList={}, InputProcessEnable[0]={},".format(CurrentTagID[0], DestinationNodeList, InputProcessEnable[0]))
                                    stdout.flush()
                                    try:
                                        for PathListCounter in range(len(DestinationNodeList[0])):
                                            stdout.write("\n[->]inputFunction()[->]if CurrentT Virtu:PathListCounter={},".format(PathListCounter))
                                            stdout.flush()
                                            # right now not use virtual node
                                            if DestinationNodeList[0][PathListCounter] in VirtualNodeList:
                                                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == DestinationNodeList[0][PathListCounter]]['RealNode'])
                                                temp_list = []
                                                if (not PathListCounter) and CurrentTagID[0] != real_node:
                                                    if len(DestinationNodeList[0]) == 1 and CurrentTagID[0] == DestinationNodeList[0][PathListCounter]:
                                                        temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == HomeNode) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                        FinalDest_Path[0] = FinalDestinationNodeList = [HomeNode]
                                                        temp_list = temp_list[::-1]
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:temp_list={},".format(temp_list))
                                                        stdout.flush()
                                                        # temp_list.append(CurrentTagID[0])
                                                    elif not PathListCounter and CurrentTagID[0] != DestinationNodeList[0][PathListCounter]:
                                                        real_node2 = int(virtualNodeDF[virtualNodeDF.VirtualNode == CurrentTagID[0]]['RealNode'])
                                                        if real_node != real_node2:
                                                            temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node2) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:real_node2={}, temp_list={}, real_node={},".format(real_node2, temp_list, real_node))
                                                            stdout.flush()
                                                        else:
                                                            temp_list = [real_node]
                                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagI exception temp_list={},".format(temp_list))
                                                            stdout.flush()
                                                    elif not PathListCounter:
                                                        temp_list = [real_node]
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:PathListCounter={}, temp_list={}, CurrentTagID[0]={}, real_node={},".format(PathListCounter, temp_list, CurrentTagID[0], real_node))
                                                        stdout.flush()
                                                        if CurrentTagID[0] != DestinationNodeList[0][PathListCounter] and temp_list[-1] != DestinationNodeList[0][PathListCounter]:
                                                            temp_list.append(DestinationNodeList[0][PathListCounter])
                                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0]={}, in Virtu:DestinationNodeList={}, PathListCounter={}, temp_list={}, real_node={},".format(CurrentTagID[0], DestinationNodeList, PathListCounter, temp_list, real_node))
                                                            stdout.flush()
                                                elif PathListCounter and DestinationNodeList[0][PathListCounter] in VirtualNodeList:
                                                    stdout.write("\n[->]inputFunction()[->]if FinalPath={}, no temp_list={}, DestinationNodeList={}, PathListCounter={},".format(FinalPath, temp_list, DestinationNodeList, PathListCounter))
                                                    stdout.flush()
                                                    if FinalPath[-1] not in VirtualNodeList and FinalPath[-1] == real_node:
                                                        temp_list.append(DestinationNodeList[0][PathListCounter])
                                                        stdout.write("\n[->]inputFunction()[->]if FinalPath[-1]={}, no temp_list={}, DestinationNodeList={}, PathListCounter={},".format(FinalPath[-1], temp_list, DestinationNodeList, PathListCounter))
                                                        stdout.flush()
                                                    elif FinalPath[-1] not in VirtualNodeList and FinalPath[-1] != real_node:
                                                        temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[-1]) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                        stdout.write("\n[->]inputFunction()[->]elif FinalPath[-1]={}, not i: temp_list={},".format(FinalPath[-1], temp_list))
                                                        stdout.flush()
                                                    elif FinalPath[-1] in VirtualNodeList:
                                                        real_node2 = int(virtualNodeDF[virtualNodeDF.VirtualNode == FinalPath[-1]]['RealNode'])
                                                        stdout.write("\n[->]inputFunction()[->]elif real_node2={}, FinalPath[-1]={}, in Vi temp_list={},".format(real_node2, FinalPath[-1], temp_list))
                                                        stdout.flush()
                                                        if real_node2 != real_node:
                                                            temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node2) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                            stdout.write("\n[->]inputFunction()[->]elif FinalPath[-1]={}, in Vi temp_list={},".format(FinalPath[-1], temp_list))
                                                            stdout.flush()
                                                        else:
                                                            temp_list = [real_node]
                                                            stdout.write("\n[->]inputFunction()[->]else FinalPath[-1]={}, in Vi temp_list={},".format(FinalPath[-1], temp_list))
                                                            stdout.flush()
                                                    elif FinalPath[-1] in VirtualNodeList:
                                                        stdout.write("\n[->]inputFunction()[->] exception elif FinalPath[-1]={}, not i: temp_list={},".format(FinalPath[-1], temp_list))
                                                        stdout.flush()
                                                try:
                                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:1 temp_list={}, FinalPath={}, DestinationNodeList={},".format(temp_list, FinalPath, DestinationNodeList))
                                                    stdout.flush()
                                                    if len(FinalPath) and len(temp_list):
                                                        if temp_list[0] != FinalPath[-1]: FinalPath.extend(temp_list)
                                                        else: FinalPath.extend(temp_list[1:])
                                                    elif len(temp_list): FinalPath.extend(temp_list)
                                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:0 FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(FinalPath, CurrentTagID[0], PathListCounter))
                                                    stdout.flush()
                                                    if len(FinalPath):
                                                        if FinalPath[0] != CurrentTagID[0]:
                                                            FinalPath = [CurrentTagID[0]] + FinalPath
                                                            stdout.write("\n[->]inputFunction()[->]if FinalPath[0] != Cur: FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(FinalPath, CurrentTagID[0], PathListCounter))
                                                            stdout.flush()
                                                        if FinalPath[-1] != DestinationNodeList[0][PathListCounter] and CurrentTagID[0] != DestinationNodeList[0][PathListCounter]:
                                                            FinalPath = FinalPath + [DestinationNodeList[0][PathListCounter]]
                                                            stdout.write("\n[->]inputFunction()[->]if FinalPath[thListCounter={}, FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(DestinationNodeList[0][PathListCounter], FinalPath, CurrentTagID[0], PathListCounter))
                                                            stdout.flush()
                                                    stdout.write("\n[->]inputFunction()[->]if CurrentT: FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(FinalPath, CurrentTagID[0], PathListCounter))
                                                    stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu: temp_list.append exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                            else:
                                                if not PathListCounter:
                                                    real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == CurrentTagID[0]]['RealNode'])
                                                    if len(DestinationNodeList[0]) == 1:
                                                        if real_node != DestinationNodeList[0][PathListCounter]:
                                                            FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                            stdout.write("\n[->]inputFunction()[->]if Curreif len(Destinatin V:else:elif not PathListCounter: CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                        elif real_node != HomeNode:
                                                            FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == HomeNode)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                            stdout.write("\n[->]inputFunction()[->]HOMEif Curreif len(Destinatin V:else:elif not PathListCounter: CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                    elif real_node != DestinationNodeList[0][PathListCounter]:
                                                        FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                        stdout.write("\n[->]inputFunction()[->]else:if elif real_node != Des: CurrentTagID[0]={}, CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                    else:
                                                        FinalPath = FinalPath + [real_node]
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[else:else CurrentTagID[0]={}, CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                else:
                                                    if DestinationNodeList[0][PathListCounter-1] in VirtualNodeList:
                                                        real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == DestinationNodeList[0][PathListCounter-1]]['RealNode'])
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in V:else:else: real_node={}, DestinationNodeList[0][PathListCounter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(real_node, DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                        if real_node != DestinationNodeList[0][PathListCounter]:
                                                            FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in V:DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                        else:
                                                            FinalPath = FinalPath + [real_node]
                                                            # FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == DestinationNodeList[0][DestinationNodeList[0].index(real_node)+1])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0]={}, real_node={}, DestinationNodeList[0][PathListCounter]={}, FinalPath={},".format(CurrentTagID[0], real_node, DestinationNodeList[0][PathListCounter], FinalPath))
                                                            stdout.flush()
                                                    elif DestinationNodeList[0][PathListCounter-1] != DestinationNodeList[0][PathListCounter]:
                                                        FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == DestinationNodeList[0][PathListCounter-1]) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in V:else:else: DestinationNodeList[0][PathListCounter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                    else:
                                                        stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] exception in V:else:elif:else:else: CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu: t: exception={},".format(e))
                                        stdout.flush()
                                        pass
                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:0 main_t2={}, FinalPath={},".format(main_t2, FinalPath))
                                    stdout.flush()
                                    FinalPath = [PathListCounter[0] for PathListCounter in groupby(FinalPath)]
                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:1 main_t2={}, FinalPath={},".format(main_t2, FinalPath))
                                    stdout.flush()
                                    # OriginalFinalPath = FinalPath
                                    DynamicOriginalFinalPath = OriginalFinalPath = FinalPath
                                    FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0]
                                    if len(DestinationNodeList[0]) == 1:
                                        if CurrentTagID[0] == DestinationNodeList[0][0]:
                                            FinalDest_Path[0] = FinalDestinationNodeList = [HomeNode]
                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu: if len(DestinationNodeLi main_t2={}, DestinationNodeList[0][0]={}, CurrentTagID[0]={},".format(main_t2, DestinationNodeList[0][0], CurrentTagID[0]))
                                            stdout.flush()
                                        else:
                                            FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0]
                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:else: if len( else: main_t2={}, DestinationNodeList[0]={},".format(main_t2, DestinationNodeList[0]))
                                            stdout.flush()
                                    else:
                                        if CurrentTagID[0] == DestinationNodeList[0][0]:
                                            FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0][0:]
                                            stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0] in Virtu:else:if len( else: main_t2={}, DestinationNodeList[0][0:]={}, CurrentTagID[0]={},".format(main_t2, DestinationNodeList[0][0:], CurrentTagID[0]))
                                            stdout.flush()
                                    stdout.write("\n[->]inputFunction()[->]if CurrentTagID[0]={}, in Virtu: FinalDestinationNodeList={}, main_t2={}, FinalPath={},".format(CurrentTagID[0], FinalDestinationNodeList, main_t2, FinalPath))
                                    stdout.flush()
                                    # straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF) START
                                    straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF, DestinationNodeList, CompulsoryTagList, IgnoreTagCounterMaxLimit)
                                    stdout.write("\n[->]straightPathNodeEli[->]if CurrentTagID[0] in Virtu: FinalPath={}, main_t2={},".format(FinalPath, main_t2))
                                    stdout.flush()
                                    previous_node_index, FinalPathDictionary = 0, {}
                                    for i in range(len(FinalDestinationNodeList)):
                                        node_index = FinalPath.index(FinalDestinationNodeList[i])
                                        temp_result_list = FinalPath[previous_node_index:node_index+1]
                                        temp_result_list.append(0)
                                        FinalPathDictionary[FinalDestinationNodeList[i]] = temp_result_list
                                        previous_node_index = node_index
                                    stdout.write("\n[->]finalDictionary()[->]elif Bluetoif CurrentTagID[0] in Virtu: FinalPathDictionary={}, FinalPath={}, MotionSection={}, main_t2={},".format(FinalPathDictionary, FinalPath, MotionSection, main_t2))
                                    stdout.flush()
                                    MotionSection = 0
                                except Exception as e:
                                    stdout.write("\n\n\n[->]inputFunction()[->]elif Bluif CurrentTagID[0] in Virtu: main_t2={}, exception={},".format(main_t2, e))
                                    stdout.flush()
                                    pass
                        elif TagDetectionState[0]:
                            CurrentTagID[0] = TagID[0]
                            if ChargingFlag[0] == 1:
                                if CurrentTagID[0] == ChargingNode:
                                    # stdout.write("\n[->]inputFunction()[->] at Chrging Station for Charge")
                                    if float(CurrentValue[0]) > 0:
                                        InputProcessEnable[0] = 0
                                        LedData1[0] = '2,' + str(VoltageValue[0]) + '\n'
                                        if LedData1[0] != prev_LedData1:
                                            # ESP32_Ser.write(LedData1[0].encode())
                                            stdout.write("\n[-->ESP32_T] LedData1[0]={}, time={},".format(LedData1[0], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                            stdout.flush()
                                            prev_LedData1 = LedData1[0]
                                        if float(VoltageValue[0]) > ChargedBatteryVoltageLimit:
                                            ChargingFlag[0] = 0
                                            stdout.write("\n[->]inputFunction()[->]Chrging Station Charged VoltageValue(20.5 to 26v)={}, time={},".format(VoltageValue[0], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                            stdout.flush()
                                    else:
                                        LedData1[0] = '10'
                                        if LedData1[0] != prev_LedData1:
                                            # ESP32_Ser.write(LedData1[0].encode())
                                            stdout.write("\n[-->ESP32_T] please charge me LedData1[0]={}, time={},".format(LedData1[0], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                            stdout.flush()
                                            prev_LedData1 = LedData1[0]
                                        ErrorCode[0] = 122
                                    break
                                    break
                                else:
                                    FinalPath = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == ChargingNode)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                    OriginalFinalPath = FinalPath
                                    DynamicOriginalFinalPath = OriginalFinalPath
                                    FinalDest_Path[0] = FinalDestinationNodeList = [ChargingNode]
                                    stdout.write("\n[->]inputFunction()[->] elif TagDetectionState[0]: finalDictionary() START... {}".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                    stdout.flush()
                                    straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF, DestinationNodeList, CompulsoryTagList, IgnoreTagCounterMaxLimit)
                                    previous_node_index, FinalPathDictionary = 0, {}

                                    for i in range(len(FinalDestinationNodeList)):
                                        node_index = FinalPath.index(FinalDestinationNodeList[i])
                                        temp_result_list = FinalPath[previous_node_index:node_index+1]
                                        temp_result_list.append(0)
                                        FinalPathDictionary[FinalDestinationNodeList[i]] = temp_result_list
                                        previous_node_index = node_index
                                    MotionSection = 0
                                    stdout.write("\n[->]finalDictionary()[->]if ChargingFlag[0] == 1: FinalPathDictionary={},".format(FinalPathDictionary))
                                    stdout.flush()
                            elif type(DestinationNodeList[0]) == list and len(DestinationNodeList[0]):
                                try:
                                    stdout.write("\n[->]inputFunction()[->] CurrentTagID[0]={}, DestinationNodeList={}, InputProcessEnable[0]={},".format(CurrentTagID[0], DestinationNodeList, InputProcessEnable[0]))
                                    stdout.flush()
                                    try:
                                        PathListCounter = 0
                                        # for PathListCounter in range(len(DestinationNodeList[0])):
                                        while PathListCounter < len(DestinationNodeList[0]):
                                            stdout.write("\n[->]inputFunction()[->]:FinalPath={}, PathListCounter={},".format(FinalPath, PathListCounter))
                                            stdout.flush()
                                            # right now not use virtual node
                                            if DestinationNodeList[0][PathListCounter] in VirtualNodeList:
                                                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == DestinationNodeList[0][PathListCounter]]['RealNode'])
                                                temp_list = []
                                                if not PathListCounter and (CurrentTagID[0] != real_node):
                                                    temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                    stdout.write("\n[->]inputFunction()[->]temp_list={}, CurrentTagID[0]={}, real_node={},".format(temp_list, CurrentTagID[0], real_node))
                                                    stdout.flush()
                                                elif not PathListCounter:
                                                    temp_list = [real_node]
                                                    stdout.write("\n[->]inputFunction()[->] PathListCounter={}, CurrentTagID[0]={}, real_node={},".format(PathListCounter, CurrentTagID[0], real_node))
                                                    stdout.flush()
                                                else:
                                                    try:
                                                        stdout.write("\n[->]inputFunction()[->] DestinationNodeList[0][PathListCounter-1]={}, VirtualNodeList={},".format(DestinationNodeList[0][PathListCounter-1], VirtualNodeList))
                                                        stdout.write("\n[->]inputFunction()[->] PathListCounter={}, real_node={},".format(PathListCounter, real_node))
                                                        stdout.flush()
                                                        if DestinationNodeList[0][PathListCounter-1] in VirtualNodeList:
                                                            previous_real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == DestinationNodeList[0][PathListCounter-1]]['RealNode'])
                                                            # stdout.write("\n[->]inputFunction()[->]1 real_node={}, previous_real_node={},".format(real_node, VirtualNodeList))
                                                            # stdout.flush()
                                                            if real_node == previous_real_node: temp_list = [real_node]
                                                            elif previous_real_node != real_node:
                                                                temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == previous_real_node) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                                # stdout.write("\n[->]inputFunction()[->]elif previous_real_node != real temp_list={},".format(temp_list))
                                                                # stdout.flush()
                                                            else:
                                                                stdout.write("\n\n\n[->]inputFunction()[->] else: exception")
                                                                stdout.flush()
                                                                pass
                                                        elif DestinationNodeList[0][PathListCounter-1] != real_node:
                                                            temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == DestinationNodeList[0][PathListCounter-1]) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                                            stdout.write("\n[->]inputFunction()[->]elif DestinationNodeList[0][PathLi: temp_list={},".format(temp_list))
                                                            stdout.flush()
                                                        else:
                                                            temp_list = [real_node]
                                                            stdout.write("\n[->]inputFunction()[->]else temp_list={},".format(temp_list))
                                                            stdout.flush()
                                                    except Exception as e:
                                                        stdout.write("\n\n\n[->]inputFunction()[->] if DestinationNodeList[0][PathListCounter-1] in VirtualNodeList: exception={},".format(e))
                                                        stdout.flush()
                                                        pass
                                                try:
                                                    temp_list.append(DestinationNodeList[0][PathListCounter])
                                                    stdout.write("\n[->]inputFunction()[->]1 temp_list={}, FinalPath={}, DestinationNodeList={},".format(temp_list, FinalPath, DestinationNodeList))
                                                    stdout.flush()
                                                    if len(FinalPath):
                                                        if temp_list[0] != FinalPath[-1]: FinalPath.extend(temp_list)
                                                        else: FinalPath.extend(temp_list[1:])
                                                    else: FinalPath.extend(temp_list)
                                                    # stdout.write("\n[->]inputFunction()[->]0 FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(FinalPath, CurrentTagID[0], PathListCounter))
                                                    # stdout.flush()
                                                    if FinalPath[0] != CurrentTagID[0]: FinalPath = [CurrentTagID[0]] + FinalPath
                                                    stdout.write("\n[->]inputFunction()[->]1 FinalPath={}, CurrentTagID[0]={}, PathListCounter={},".format(FinalPath, CurrentTagID[0], PathListCounter))
                                                    stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]inputFunction()[->]temp_list.append exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                            else:
                                                if not PathListCounter and CurrentTagID[0] != DestinationNodeList[0][PathListCounter]:
                                                    FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                    stdout.write("\n[->]inputFunction()[->]else:if not PathListCo != De CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                    stdout.flush()
                                                elif not PathListCounter:
                                                    if len(DestinationNodeList[0]) == 1:
                                                        FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == HomeNode)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                        stdout.write("\n[->]inputFunction()[->]else:elif not PathListCounter: CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                    else:
                                                        stdout.write("\n[->]inputFunction()[->]else:elif:else CurrentTagID[0]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(CurrentTagID[0], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                        stdout.flush()
                                                        pass
                                                else:
                                                    try:
                                                        if DestinationNodeList[0][PathListCounter-1] in VirtualNodeList:
                                                            real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == DestinationNodeList[0][PathListCounter-1]]['RealNode'])
                                                            stdout.write("\n[->]inputFunction()[->]else:else: len(DestinationNodeList[0])={}, real_node={}, DestinationNodeList[0][PathListCounter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(len(DestinationNodeList[0]), real_node, DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                            if real_node != DestinationNodeList[0][PathListCounter]:
                                                                FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                                stdout.write("\n[->]inputFunction()[->]DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                                stdout.flush()
                                                            elif real_node == DestinationNodeList[0][PathListCounter]:
                                                                FinalPath.append(real_node)
                                                                stdout.write("\n[->]inputFunction()[->]elseelse:: odeList[0])={}, DestinationNodeList[0].index(real_node)={}, real_node={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(len(DestinationNodeList[0]), DestinationNodeList[0].index(real_node), real_node, DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                                stdout.flush()
                                                                pass
                                                        elif DestinationNodeList[0][PathListCounter-1] != DestinationNodeList[0][PathListCounter]:
                                                            stdout.write("\n[->]inputFunction()[->]0else:else: nter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                            FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == DestinationNodeList[0][PathListCounter-1]) & (PathAssignmentDF.dn == DestinationNodeList[0][PathListCounter])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                                            stdout.write("\n[->]inputFunction()[->]1else:else: stCounter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                        else:
                                                            stdout.write("\n[->]inputFunction()[->]else:elif:else:else:ounter-1]={}, DestinationNodeList[0][PathListCounter]={}, PathListCounter={}, FinalPath={},".format(DestinationNodeList[0][PathListCounter-1], DestinationNodeList[0][PathListCounter], PathListCounter, FinalPath))
                                                            stdout.flush()
                                                            pass
                                                    except Exception as e:
                                                        stdout.write("\n\n\n[->]inputFunction()[->]try:else: main_t2={}, exception={},".format(main_t2, e))
                                                        stdout.flush()
                                                        pass
                                            PathListCounter += 1
                                        PathListCounter = 0
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]inputFunction()[->]try : main_t2={}, exception={},".format(main_t2, e))
                                        stdout.flush()
                                        pass
                                    stdout.write("\n[->]inputFunction()[->]0 main_t2={}, FinalPath={},".format(main_t2, FinalPath))
                                    stdout.flush()
                                    FinalPath = [PathListCounter[0] for PathListCounter in groupby(FinalPath)]
                                    stdout.write("\n[->]inputFunction()[->]1 main_t2={}, FinalPath={},".format(main_t2, FinalPath))
                                    stdout.flush()
                                    # OriginalFinalPath = FinalPath
                                    DynamicOriginalFinalPath = OriginalFinalPath = FinalPath
                                    FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0]
                                    if len(DestinationNodeList[0]) == 1:
                                        if CurrentTagID[0] == DestinationNodeList[0][0]:
                                            FinalDest_Path[0] = FinalDestinationNodeList = [HomeNode]
                                            stdout.write("\n[->]inputFunction()[->]if len(DestinationNodeLi main_t2={}, DestinationNodeList[0][0]={},".format(main_t2, DestinationNodeList[0][0]))
                                            stdout.flush()
                                        else:
                                            FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0]
                                            stdout.write("\n[->]inputFunction()[->]if len( else: main_t2={}, DestinationNodeList[0]={},".format(main_t2, DestinationNodeList[0]))
                                            stdout.flush()
                                    else:
                                        if CurrentTagID[0] == DestinationNodeList[0][0]:
                                            FinalDest_Path[0] = FinalDestinationNodeList = DestinationNodeList[0][0:]
                                            stdout.write("\n[->]inputFunction()[->]if len( else: main_t2={}, DestinationNodeList[0][0:]={},".format(main_t2, DestinationNodeList[0][0:]))
                                            stdout.flush()

                                    stdout.write("\n[->]inputFunction()[->] FinalDestinationNodeList={}, main_t2={}, FinalPath={},".format(FinalDestinationNodeList, main_t2, FinalPath))
                                    stdout.flush()
                                    # straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF) START
                                    straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF, DestinationNodeList, CompulsoryTagList, IgnoreTagCounterMaxLimit)
                                    stdout.write("\n[->]straightPathNodeEli[->] FinalPath={}, main_t2={},".format(FinalPath, main_t2))
                                    stdout.flush()
                                    previous_node_index, FinalPathDictionary = 0, {}

                                    for i in range(len(FinalDestinationNodeList)):
                                        node_index = FinalPath.index(FinalDestinationNodeList[i])
                                        if node_index < previous_node_index: node_index = previous_node_index + 1
                                        else:
                                            temp_list_ = [i for i, x in enumerate(FinalPath) if x == FinalPath[previous_node_index]]
                                            for j in range(len(temp_list_)):
                                                if node_index > temp_list_[j]: previous_node_index = temp_list_[j]
                                                elif node_index < temp_list_[j]: break
                                        temp_result_list = FinalPath[previous_node_index:node_index+1]
                                        temp_result_list.append(0)
                                        FinalPathDictionary[FinalDestinationNodeList[i]] = temp_result_list
                                        previous_node_index = node_index
                                    stdout.write("\n[->]finalDictionary()[->]elif BluetoothInputEnable[0]: FinalPathDictionary={}, FinalPath={}, MotionSection={}, main_t2={},".format(FinalPathDictionary, FinalPath, MotionSection, main_t2))
                                    stdout.flush()
                                    MotionSection = 0
                                except Exception as e:
                                    stdout.write("\n\n\n[->]inputFunction()[->]elif BluetoothIn: main_t2={}, exception={},".format(main_t2, e))
                                    stdout.flush()
                                    pass
                            # at last home
                            elif TagID[0] != HomeNode:
                                stdout.write("\n[->]finalDictionary()elif TagID[0] != HomeNode: ATYaw[0]={}, CurrentTagID[0]={}, main_t2={},".format(ATYaw[0], CurrentTagID[0], main_t2))
                                stdout.flush()
                                FinalPath = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == HomeNode)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                                DynamicOriginalFinalPath = OriginalFinalPath = FinalPath
                                FinalDest_Path[0] = FinalDestinationNodeList = [HomeNode]
                                stdout.write("\n[->]inputFunction()[->] elif TagID[0](={},) != HomeNode(={},): FinalPath={}, main_t2={}, START... {}".format(TagID[0], HomeNode, FinalPath, main_t2, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                stdout.write("\n[->]inputFunction()[->] DynamicOriginalFinalPath={}, OriginalFinalPath={},".format(DynamicOriginalFinalPath, OriginalFinalPath))
                                stdout.flush()
                                straightPathNodeEliminateFunction(VirtualNodeList, virtualNodeDF, dataAssignmentDF, PathAssignmentDF, DestinationNodeList, CompulsoryTagList, IgnoreTagCounterMaxLimit)
                                previous_node_index, FinalPathDictionary = 0, {}

                                for i in range(len(FinalDestinationNodeList)):
                                    node_index = FinalPath.index(FinalDestinationNodeList[i])
                                    stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode: node_index={}, FinalPathDictionary={}, main_t2={},".format(node_index, FinalPathDictionary, main_t2))
                                    stdout.flush()
                                    temp_result_list = FinalPath[previous_node_index:node_index+1]
                                    temp_result_list.append(0)
                                    stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode: temp_result_list={}, previous_node_index={}, main_t2={},".format(temp_result_list, previous_node_index, main_t2))
                                    stdout.flush()
                                    FinalPathDictionary[FinalDestinationNodeList[i]] = temp_result_list
                                    stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode: FinalPathDictionary={}, main_t2={},".format(FinalPathDictionary, main_t2))
                                    stdout.flush()
                                    previous_node_index = node_index
                                    stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode: FinalDestinationNodeList={}, FinalPathDictionary={}, main_t2={},".format(FinalDestinationNodeList, FinalPathDictionary, main_t2))
                                    stdout.flush()
                                stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode:1 FinalPathDictionary={}, MotionSection={}, main_t2={},".format(FinalPathDictionary, MotionSection, main_t2))
                                stdout.flush()
                                # Endtag to home Tag
                                MotionSection = 0
                                stdout.write("\n[->]finalDictionary()[->]elif TagID[0] != HomeNode:2 FinalPathDictionary={}, MotionSection={}, main_t2={},".format(FinalPathDictionary, MotionSection, main_t2))
                                stdout.flush()
                            elif not len(FinalPath):
                                # stdout.write("\n[->]inputFunction()elif not len(FinalPath):")
                                BluetoothInputEnable[0] = 1
                            else:
                                stdout.write("\n[->]inputFunction()elif TagDetectionState[0]: main_t2={},".format(main_t2))
                                stdout.flush()
                                break
                        elif (not TagDetectionState[0]) and printFlag:
                            stdout.write("\n[->]inputFunction()[->] ApriltagModuleFlag[0]={},".format(ApriltagModuleFlag[0]))
                            stdout.write("\n\n\n[->]inputFunction()[->] exception ........ TagID[0]={}, not detected ........main_t2={},".format(TagID[0], main_t2))
                            stdout.flush()
                            ErrorCode[0] = 123  # 29/11/22  12:19pm
                            printFlag = False
                if len(FinalPath) and MotionSection == 0:
                    # travel()
                    # 1) FinalPath, in seprated form after 1 or 2 table input
                    # 2) EndNode
                    # 3)
                    # global DestinationNode, VirtualNodeList
                    try:
                        Initium, FinalDestinationNodeListCounter = 1, 0
                        # for FinalDestinationNodeListCounter in range(len(FinalDestinationNodeList)):
                        serving_t1, serving_t3 = time(), date().strftime("%d-%m-%Y %H:%M:%S:%f")
                        while len(FinalDestinationNodeList) > FinalDestinationNodeListCounter:
                            # stdout.write("\n[->]FinalDestinationNodeListCounter={}, CurrentTagID[0]={}, FinalDestinationNodeList={}, FinalPathDictionary={}, main_t2={},".format(FinalDestinationNodeListCounter, CurrentTagID[0], FinalDestinationNodeList, FinalPathDictionary, main_t2))
                            # stdout.flush()
                            if (len(FinalDestinationNodeList) == 1 and CurrentTagID[0] == FinalDestinationNodeList[0]):
                                stdout.write("\n[->]break len(FinalDestinationNodeList)={}, CurrentTagID[0]={}, FinalDestinationNodeList[0]={}, main_t2={},".format(len(FinalDestinationNodeList), CurrentTagID[0], FinalDestinationNodeList[0], main_t2))
                                stdout.flush()
                                break
                            elif CurrentTagID[0] != HomeNode and CurrentTagID[0] in FinalDestinationNodeList:
                                if FinalDestinationNodeListCounter != (FinalDestinationNodeList.index(CurrentTagID[0]) + 1) and len(FinalDestinationNodeList) > (FinalDestinationNodeList.index(CurrentTagID[0]) + 1):
                                    FinalDestinationNodeListCounter = (FinalDestinationNodeList.index(CurrentTagID[0]) + 1)
                                    stdout.write("\n[->]1 FinalDestinationNodeListCounter={}, CurrentTagID[0]={}, FinalDestinationNodeList.index(CurrentTagID[0])={}, main_t2={},".format(FinalDestinationNodeListCounter, CurrentTagID[0], FinalDestinationNodeList.index(CurrentTagID[0]), main_t2))
                                    stdout.flush()

                            FinalPath = FinalPathDictionary[FinalDestinationNodeList[FinalDestinationNodeListCounter]]
                            # stdout.write("\n[->]travelFunction()[->] Initium=1 FinalPath={}, FinalPathDictionary={}, main_t2={},".format(FinalPath, FinalPathDictionary, main_t2))
                            # stdout.flush()
                            if FinalPath[0] in VirtualNodeList:
                                # stdout.write("\n[->]travelFunction()[->] CurrentTagID={}, in VirtualNodeList={}, main_t2={},".format(FinalPath[0], VirtualNodeList, main_t2))
                                # stdout.flush()
                                virtual_node_data_list = virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[0])].dropna(axis=1).iloc[0, 0:].tolist()
                                # stdout.write("\n[->]travelFunction()[->] virtual_node_data_list={}, main_t2={},".format(virtual_node_data_list, main_t2))
                                # stdout.flush()
                                actual_node, distance_between = virtual_node_data_list[0], virtual_node_data_list[2]
                                stdout.write("\n[->]travelFunction()[->] actual_node={}, distance_between={}, OriginalFinalPath={}, main_t2={},".format(actual_node, distance_between, OriginalFinalPath, main_t2))
                                stdout.flush()
                                if FinalPath[0] != OriginalFinalPath[0]:
                                    DynamicOriginalFinalPath = OriginalFinalPath = OriginalFinalPath[OriginalFinalPath.index(FinalPath[0]):]
                                stdout.write("\n[->]travelFunction()[->] DynamicOriginalFinalPath=OriginalFinalPath={}, main_t2={},".format(OriginalFinalPath, main_t2))
                                stdout.flush()
                                if TagDetectionState[0] and TagID[0] == actual_node and distance_between <= 100:
                                    if FinalPath[0] == OriginalFinalPath[0]:
                                        del(OriginalFinalPath[0])
                                        DynamicOriginalFinalPath = OriginalFinalPath
                                    del(FinalPath[0])
                                    stdout.write("\n[->]travelFunction()[->] so finally FinalPath={}, main_t2={}, DynamicOriginalFinalPath=OriginalFinalPath={},".format(FinalPath, main_t2, OriginalFinalPath))
                                    stdout.flush()
                            stdout.write("\n[->]travelFunction()[->] NEW FINAL PATH={}, serving_t1={}, main_t2={}, TagDetectionState[0]={},".format(FinalPath, serving_t1, main_t2, TagDetectionState[0]))
                            stdout.flush()
                            if len(FinalPath) > 1:
                                EndNode = FinalPath[-2]
                                stdout.write("\n[->]travelFunction()[->] EndNode={}, main_t2={},".format(EndNode, main_t2))
                                stdout.flush()
                                AtTableRobotFace = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == EndNode)].iloc[0, 2])
                                stdout.write("\n[->]travelFunction()[->] AtTableRobotFace={}, FinalPath={}, FinalPathListCounter={}, FinalPath[FinalPathListCounter+1]={}, main_t2={},".format(AtTableRobotFace, FinalPath, FinalPathListCounter, FinalPath[FinalPathListCounter+1], main_t2))
                                stdout.flush()
                            # while FinalPath[FinalPathListCounter+1]:
                            # global futureTsetpoint
                            while FinalPath[FinalPathListCounter+1]:
                                DestinationNode, CurrentTagID[0] = FinalPath[FinalPathListCounter + 1], FinalPath[FinalPathListCounter]
                                # stdout.write("\n[->]travelFunction()[->]while FinalPath[FinalPathListCounter+1]: DestinationNode={}, CurrentTagID[0]={},".format(DestinationNode, CurrentTagID[0]))
                                # stdout.flush()
                                try:
                                    # 1 assignment() START
                                    stdout.write("\n[->]travelFunction()[->] while FinalPath[Fina: assignment({}, {}, {}) main_t2={}, START... {}".format(CurrentTagID[0], DestinationNode, FinalPath[FinalPathListCounter+2], main_t2, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                    stdout.flush()
                                    cnode, dnode, fnode = FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], FinalPath[FinalPathListCounter+2]
                                    stdout.write("\n[->]862 assignment(cnode={}, dnode={}, fnode={},) DynamicOriginalFinalPath={}, main_t2={},".format(cnode, dnode, fnode, DynamicOriginalFinalPath, main_t2))
                                    stdout.flush()
                                    try:
                                        try:
                                            stdout.write("\n[->]travelFunction()[->]1313 while Fina cnode={}, DynamicOriginalFinalPath={}, main_t2={},".format(cnode, DynamicOriginalFinalPath, main_t2))
                                            stdout.flush()
                                            if cnode in DynamicOriginalFinalPath:  # 2/12/22 11:34am
                                                del DynamicOriginalFinalPath[:DynamicOriginalFinalPath.index(cnode)]
                                        except Exception as e:
                                            stdout.write("\n\n\n[->]del DynamicOriginalFinalPath={}, cnode={}, main_t2={}, exception={},".format(cnode, DynamicOriginalFinalPath, main_t2, e))
                                            stdout.flush()
                                            pass
                                        # turn_complete, first,
                                        travel = 0
                                        try:
                                            if cnode not in VirtualNodeList:
                                                if fnode and fnode not in VirtualNodeList:
                                                    # and (FinalPath[FinalPathListCounter+2] not in VirtualNodeList):
                                                    if (FinalPath[FinalPathListCounter + 1] not in VirtualNodeList):
                                                        pathStraightFlag_011 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter + 1])]['pathStraightFlag'])
                                                        pathStraightFlag_122 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter + 1]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter + 2])]['pathStraightFlag'])
                                                        stdout.write("\n[->]straightPathNodeEliminateFunction()[->]elif pathStraightFlag_011={}, pathStraightFlag_122={}, main_t2={},".format(pathStraightFlag_011, pathStraightFlag_122, main_t2))
                                                        stdout.flush()
                                                elif dnode not in VirtualNodeList:
                                                    pathStraightFlag_011 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+1])]['pathStraightFlag'])
                                                    pathStraightFlag_122 = pathStraightFlag_011
                                                else: pathStraightFlag_122, pathStraightFlag_011 = 1, 1
                                            else: pathStraightFlag_122, pathStraightFlag_011 = 1, 1
                                        except Exception as e:
                                            stdout.write("\n\n\n[->]867 pathStraightFlag_01 pathS main_t2={}, exception={},".format(main_t2, e))
                                            stdout.flush()
                                            # sleep(1)
                                            # system("sudo killall -9 python")
                                            pass
                                        if pathStraightFlag_011 == pathStraightFlag_122 or fnode == 0: SmoothTurnEnable = 0
                                        else:
                                            SmoothTurnEnable = 1
                                            stdout.write("\n[->]assignment()[->]896 For Hypotenuse Start from homenode SmoothTurnEnable={},".format(SmoothTurnEnable))
                                            stdout.flush()
                                            try:
                                                if cnode == HomeNode:
                                                    A, B = [cnode, dnode], OriginalFinalPath
                                                    stdout.write("\n[->]assignment()[->] A={}, B={},".format(A, B))
                                                    stdout.flush()
                                                    avoided_node_input = any(A == B[i:i + 2] for i in range(len(B)-2 + 1))
                                                    stdout.write("\n[->]assignment()[->] avoided_node_input={},".format(avoided_node_input))
                                                    stdout.flush()
                                                    if not avoided_node_input: path_straight_enable = 2
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]assignment()[->] avoided_node_input exception={},".format(e))
                                                stdout.flush()
                                                pass
                                        fnarrow_path, fpath_straight_enable = 0, 0
                                        ATDetection_CurrentTag[0], ATDetection_DestinationTag[0] = cnode, dnode
                                        pathStraightFlag_0, pathStraightFlag_1 = 0, 0
                                        # stdout.write("\n[->]assignment()[->]whiley:try: FinalPath={}, FinalPathListCounter={}, StraightPathNodeAvoidEnableFlag={}, len(IgnoreTagList)={}, main_t2={},".format(FinalPath, FinalPathListCounter, StraightPathNodeAvoidEnableFlag, len(IgnoreTagList), main_t2))
                                        # stdout.flush()
                                        try:
                                            if cnode in VirtualNodeList:
                                                dataAssignmentList = virtualNodeDF[(virtualNodeDF.VirtualNode == cnode) & (virtualNodeDF.RealNode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                answer = float(dataAssignmentList[3]) - 180
                                                if answer < 0: dataAssignmentList[3] = 360 + answer
                                                else: dataAssignmentList[3] = answer
                                                Tsetpoint = float(virtualNodeDF[(virtualNodeDF.RealNode == dnode) & (virtualNodeDF.VirtualNode == cnode)].iloc[0, 4])
                                            elif dnode in VirtualNodeList:
                                                dataAssignmentList = virtualNodeDF[(virtualNodeDF.RealNode == cnode) & (virtualNodeDF.VirtualNode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                Tsetpoint = float(virtualNodeDF[(virtualNodeDF.RealNode == cnode) & (virtualNodeDF.VirtualNode == dnode)].iloc[0, 3])
                                            elif len(IgnoreTagList):
                                                # stdout.write("\n[->]assignment()[->]elif len(IgnoreTagList): OriginalFinalPath={}, OriginalFinalPath.count(cnode)={}, main_t2={},".format(OriginalFinalPath, OriginalFinalPath.count(cnode), main_t2))
                                                # stdout.flush()
                                                try:
                                                    if OriginalFinalPath.count(cnode) >= 2:
                                                        # stdout.write("\n[->]assignment()[->]if OriginalFina: cnode={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1](next original node)={}, OriginalFinalPath[OriginalFinalPath.index(cnode,OriginalFinalPath.index(cnode)+1)+1](dnode={}) == {}".format(cnode, OriginalFinalPath[OriginalFinalPath.index(cnode)+1], OriginalFinalPath[OriginalFinalPath.index(cnode,OriginalFinalPath.index(cnode)+1)+1], dnode))
                                                        # stdout.write("\n[->]assignment()[->] OriginalFinalPath[OriginalFinalPath.index(cnode)+1]={}, OriginalFinalPath.index(cnode)+1={}, main_t2={},".format(OriginalFinalPath[OriginalFinalPath.index(cnode)+1], OriginalFinalPath.index(cnode)+1, main_t2))
                                                        # stdout.flush()
                                                        if OriginalFinalPath[OriginalFinalPath.index(cnode)+1] in VirtualNodeList:
                                                            temp_OriginalFinalPath = OriginalFinalPath[OriginalFinalPath.index(cnode, OriginalFinalPath.index(cnode)+1)+1]
                                                            # stdout.write("\n[->]assignment()[->]if lNodeList: cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(cnode, temp_OriginalFinalPath, main_t2))
                                                            # stdout.flush()
                                                            if cnode != temp_OriginalFinalPath and temp_OriginalFinalPath not in VirtualNodeList:
                                                                # stdout.write("\n[->]assignment()[->]if cnode != temp_: cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(cnode, temp_OriginalFinalPath, main_t2))
                                                                # stdout.flush()
                                                                dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == temp_OriginalFinalPath)].dropna(axis=1).iloc[0, 0:].tolist()
                                                                Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == temp_OriginalFinalPath)]['Tsetpoint'])
                                                            elif cnode != temp_OriginalFinalPath and temp_OriginalFinalPath in VirtualNodeList:
                                                                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == temp_OriginalFinalPath]['RealNode'])
                                                                # stdout.write("\n[->]assignment()[->]if cnode != temp_:real_node={}, cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(real_node, cnode, temp_OriginalFinalPath, main_t2))
                                                                # stdout.flush()
                                                                if real_node != cnode:
                                                                    # stdout.write("\n[->]assignment()[->]ifcno:")
                                                                    # stdout.flush()
                                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == real_node)].dropna(axis=1).iloc[0, 0:].tolist()
                                                                    Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == real_node)]['Tsetpoint'])
                                                                elif cnode != OriginalFinalPath[OriginalFinalPath.index(dnode)-1] and OriginalFinalPath[OriginalFinalPath.index(dnode)-1] not in VirtualNodeList:
                                                                    # stdout.write("\n[->]assignment()[->]ifcno: OriginalFinalPath[OriginalFinalPath.index(dnode)-1]={},".format(OriginalFinalPath[OriginalFinalPath.index(dnode)-1]))
                                                                    # stdout.flush()
                                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(dnode)-1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                                    Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(dnode)-1])]['Tsetpoint'])
                                                        else:
                                                            # stdout.write("\n[->]assignment()[->]if OriginalFinalPath[OriginalFinalPath.index(cnode)+1]else: condition_status cnode={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1](dnode)={},".format(cnode, OriginalFinalPath[OriginalFinalPath.index(cnode)+1]))
                                                            # stdout.write("\n[->]assignment()[->]Update-> else: condition_status DynamicOriginalFinalPath[0](cnode)={}, DynamicOriginalFinalPath[1](dnode)={}, DynamicOriginalFinalPath={}, main_t2={},".format(DynamicOriginalFinalPath[0], DynamicOriginalFinalPath[1], DynamicOriginalFinalPath, main_t2))
                                                            # stdout.flush()
                                                            # dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0,0:].tolist()
                                                            dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == DynamicOriginalFinalPath[DynamicOriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                            Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == DynamicOriginalFinalPath[DynamicOriginalFinalPath.index(cnode)+1])]['Tsetpoint'])
                                                    elif cnode in OriginalFinalPath:
                                                        # dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0,0:].tolist()
                                                        # stdout.write("\n[->]assignment()[->]else: OriginalFinalPath[OriginalFinalPath.index(cnode)](cnode)={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1]={}, main_t2={},".format(OriginalFinalPath[OriginalFinalPath.index(cnode)], OriginalFinalPath[OriginalFinalPath.index(cnode)+1], main_t2))
                                                        # stdout.flush()
                                                        dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[OriginalFinalPath.index(cnode)]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                        Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[OriginalFinalPath.index(cnode)]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])]['Tsetpoint'])
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->]if Original2: main_t2={}, exception={},".format(main_t2, e))
                                                    stdout.flush()
                                                    pass
                                                # stdout.write("\n[->]assignment()[->] dataAssignmentList 1={}, Tsetpoint={},".format(dataAssignmentList, Tsetpoint))
                                                # stdout.write("\n[->]assignment()[->] NewPathList={}, main_t2={},".format(NewPathList, main_t2))
                                                # stdout.flush()
                                                try:
                                                    # dataAssignmentList[2] = int([data[-1] for data in NewPathList if data[0] == cnode and data[1] == dnode][0])
                                                    for data in NewPathList:
                                                        if (data[0] == cnode and data[1] == dnode): dataAssignmentList[2] = int(data[-1])
                                                        else: pass
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->]dataAssignmentList[2] main_t2={}, exception={},".format(main_t2, e))
                                                    stdout.flush()
                                                    pass
                                                if OriginalFinalPath[0] not in VirtualNodeList:
                                                    if OriginalFinalPath[1] not in VirtualNodeList:
                                                        Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[1])]['Tsetpoint'])
                                                    elif OriginalFinalPath[1] in VirtualNodeList and OriginalFinalPath[2] not in VirtualNodeList and OriginalFinalPath[0] != OriginalFinalPath[2]:
                                                        Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[2])]['Tsetpoint'])
                                                    elif OriginalFinalPath[1] in VirtualNodeList and OriginalFinalPath[3] not in VirtualNodeList and OriginalFinalPath[0] != OriginalFinalPath[3]:
                                                        Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[3])]['Tsetpoint'])
                                                stdout.write("\n[->]assignment()[->] dataAssignmentList 2={}, Tsetpoint={},".format(dataAssignmentList, Tsetpoint))
                                                stdout.flush()
                                            else:
                                                # stdout.write("\n[->]assignment()[->]try:else: OriginalFinalPath={},".format(OriginalFinalPath))
                                                # stdout.flush()
                                                dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                # Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0])&(dataAssignmentDF.dnode == OriginalFinalPath[1])]['Tsetpoint'])
                                                Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == dnode)]['Tsetpoint'])  # 17/11/22 12:59pm
                                                # stdout.write("\n[->]assignment()[->]try:else: Tsetpoint={}, cnode={}, dnode={}, dataAssignmentList={},".format(Tsetpoint, cnode, dnode, dataAssignmentList))
                                                # stdout.flush()
                                            # stdout.write("\n[->]assignment()[->]997try: cnode={}, dnode={}, dataAssignmentList={},".format(cnode, dnode, dataAssignmentList))
                                            # stdout.flush()
                                        except Exception as e:
                                            stdout.write("\n\n\n[->]assignment()[->]1399 dataAssignmentList exception={},".format(e))
                                            stdout.flush()
                                            break
                                        # stdout.write("\n[->]assignment()[->]1003final dataAssignmentList={},".format(dataAssignmentList))
                                        # stdout.flush()
                                        tempdist2travel, narrow_path = dataAssignmentList[2], dataAssignmentList[5]
                                        setX = dataAssignmentList[7]
                                        # setX = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0])&(dataAssignmentDF.dnode == OriginalFinalPath[1])]['setX(mm)'])
                                        smoothExitX, smoothExitY = dataAssignmentList[9], dataAssignmentList[10]
                                        # stdout.write("\n[->]1007 assignment(cnode={}, dnode={}, fnode={},)".format(cnode, dnode, fnode))
                                        # stdout.write("\n[->]assignment()[->] tempdist2travel={}, Tsetpoint={}, narrow_path={}, min_depth_setpoint={},".format(tempdist2travel, Tsetpoint, narrow_path, min_depth_setpoint))
                                        # stdout.write("\n[->]assignment()[->] fnode={}, VirtualNodeList={}, StraightPathNodeAvoidEnableFlag={},".format(fnode, VirtualNodeList, StraightPathNodeAvoidEnableFlag))
                                        # stdout.write("\n[->]assignment()[->] OriginalFinalPath={},".format(OriginalFinalPath))
                                        # stdout.flush()
                                        if fnode != 0:
                                            try:
                                                # dnode in VirtualNodeList or
                                                if fnode in VirtualNodeList:
                                                    dataAssignmentList = virtualNodeDF[(virtualNodeDF.RealNode == dnode) & (virtualNodeDF.VirtualNode == fnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                elif StraightPathNodeAvoidEnableFlag and len(IgnoreTagList):
                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == dnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(dnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                else:
                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == dnode) & (dataAssignmentDF.dnode == fnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]assignment()[->] if fnode != 0: dataAssignmentList exception={}".format(e))
                                                stdout.flush()
                                                pass
                                            stdout.write("\n[->]assignment()[->] if fnode != 0: dnode={}, fnode={}, dataAssignmentList={},".format(dnode, fnode, dataAssignmentList))
                                            stdout.flush()
                                            futureTsetpoint, fnarrow_path = dataAssignmentList[3], dataAssignmentList[5]
                                            stdout.write("\n[->]assignment()[->] futureTsetpoint={}, fnarrow_path={}".format(futureTsetpoint, fnarrow_path))
                                            stdout.flush()
                                        stdout.write("\n[->]assignment()[->] narrow_path={}, SmoothTurnEnable={}, fnarrow_path={}, PreviousSmoothTurnEnable={}, fpath_straight_enable={}, path_straight_enable={},".format(narrow_path, SmoothTurnEnable, fnarrow_path, PreviousSmoothTurnEnable, fpath_straight_enable, path_straight_enable))
                                        stdout.flush()
                                        # dist2travel = abs(int(tempdist2travel * cos(radians(tempsetpoint))))
                                        dist2travel, tTsetpoint = tempdist2travel, Tsetpoint
                                        # stdout.write("\n[->]assignment()[->] 1 tTsetpoint=Tsetpoint={}, TagID[0]={}, FinalPathListCounter={}, FinalPath={},".format(Tsetpoint, TagID[0], FinalPathListCounter, FinalPath))
                                        # stdout.write("\n[->]assignment()[->] 2 tTsetpoint=Tsetpoint={}, TagID[0]={}, FinalPathListCounter={}, FinalPath={}, pathStraightFlag_0={}, pathStraightFlag_1={},".format(Tsetpoint, TagID[0], FinalPathListCounter, FinalPath, pathStraightFlag_0, pathStraightFlag_1))
                                        # stdout.write("\n[->]assignment()[->]1447 1 setX={}, ATX[0]={}, ATY[0]={}, ATYaw[0]={},".format(setX, ATX[0]*1000, ATY[0]*1000, ATYaw[0]))
                                        # stdout.flush()
                                        if (FinalPathListCounter >= 1) and (len(FinalPath) >= 3) and (len(set(FinalPath) & set(VirtualNodeList)) == 0):
                                            pathStraightFlag_0 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter-1]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter])]['pathStraightFlag'])
                                            pathStraightFlag_1 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+1])]['pathStraightFlag'])
                                        # for turn: invert setX value
                                        if pathStraightFlag_0 != pathStraightFlag_1: setX = -setX
                                            # stdout.write("\n[->]assignment()[->]1456 2 setX={},".format(setX))
                                            # stdout.flush()
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]assignment() exception={},".format(e))
                                        stdout.flush()
                                        pass
                                    # assignment() End
                                except Exception as e:
                                    stdout.write("\n\n\n[->]travelFunction()[->] assignment() / exception={},".format(e))
                                    stdout.flush()
                                    pass
                                # if (PreviousSmoothTurnEnable == 0 and path_straight_enable == 0) or Initium == 1:  #(FinalPath[FinalPathListCounter] != 3) and
                                if Initium or (CurrentTagID[0] == HomeNode):
                                    # stdout.write("\n[->]travelFunction()[->] Initium={}, or CurrentTagID[0](={},) == HomeNode(={},)".format(Initium, CurrentTagID[0], HomeNode))
                                    # stdout.flush()
                                    Initium = 0
                                    try:
                                        if CurrentTagID[0] not in VirtualNodeList:
                                            # 1 aprilTagReset(0, 0, 0, 0)
                                            # At_flg, setYaw1, setY2, setY1 = 0, 0, 0, 0
                                            # stdout.write("\n[->]aprilTagReset()[->] aprilTagReset(At_flg={}, setYaw1={}, setY2={}, setY1={}) main_t2={}, START... {}".format(At_flg, setYaw1, setY2, setY1, main_t2, date().strftime("%d-%m-%Y_%H:%M:%S:%f")))
                                            # stdout.write("\n[->]aprilTagReset()[->] FinalPathListCounter={}, DefaultYaw={}, TagID[0]={}, ATYaw[0]={}, ATX[0]*1000={}, ATY[0]*1000={}, ATZ[0]={}, main_t2={},".format(FinalPathListCounter, DefaultYaw, TagID[0], ATYaw[0], ATX[0]*1000, ATY[0]*1000, ATZ[0], main_t2))
                                            # stdout.flush()
                                            flagonce = 0
                                            # At_flg 2 for initialization setup before inputenable mode
                                            # if At_flg == 0:
                                            # dataAssignmentRead()
                                            # stdout.write("\n[->]aprilTagReset()[->] if At_flg == 0:")
                                            # stdout.write("\n[->]dataAssignmentRead()[->]global DefaultYaw, FinalPath={}, FinalPathListCounter={}, OriginalFinalPath={},".format(FinalPath, FinalPathListCounter, OriginalFinalPath))
                                            # stdout.flush()
                                            try:
                                                if FinalPath[FinalPathListCounter] in VirtualNodeList:
                                                    if FinalPath[FinalPathListCounter] != FinalPath[FinalPathListCounter+1]:
                                                        dataAssignmentList = virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter]) & (virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                elif FinalPath[FinalPathListCounter+1] in VirtualNodeList:
                                                    if FinalPath[FinalPathListCounter+1] != FinalPath[FinalPathListCounter]:
                                                        dataAssignmentList = virtualNodeDF[(virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.RealNode == FinalPath[FinalPathListCounter])].dropna(axis=1).iloc[0, 0:].tolist()
                                                elif len(IgnoreTagList):
                                                    try:
                                                        if OriginalFinalPath.count(FinalPath[FinalPathListCounter]) == 2:
                                                            # stdout.write("\n[->]dataAssignmentRead()[->] FinalPath[FinalPathListCounter]={}, 1={}, 2={}, main_t2={},".format(FinalPath[FinalPathListCounter], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter], OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1)+1], main_t2))
                                                            # stdout.flush()
                                                            if OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1] in VirtualNodeList:
                                                                dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter], OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                            elif FinalPath[FinalPathListCounter] != OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]:
                                                                dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                            else:
                                                                stdout.write("\n[->]dataAss[->]ightPat:if OriginalF FinalPath[FinalPathListCounter]={}, != OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]={},".format(FinalPath[FinalPathListCounter], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]))
                                                                stdout.flush()
                                                                pass
                                                        elif FinalPath[FinalPathListCounter] != OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]:
                                                            dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                        else:
                                                            stdout.write("\n[->]dataAss[->]ightPat: FinalPath[FinalPathListCounter]={}, != OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]={},".format(FinalPath[FinalPathListCounter], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]))
                                                            stdout.flush()
                                                            pass
                                                        # stdout.write("\n[->]dataAssignmentRead()[->]if StraightPat: dataAssignmentList={},".format(dataAssignmentList))
                                                        # stdout.write("\n[->]dataAssignmentRead()[->]if StraightPat: OriginalFinalPath.count(FinalPath[FinalPathListCounter])={}, FinalPath={}, FinalPathListCounter={}, main_t2={},".format(OriginalFinalPath.count(FinalPath[FinalPathListCounter]), FinalPath, FinalPathListCounter, main_t2))
                                                        # stdout.flush()
                                                    except Exception as e:
                                                        stdout.write("\n[->]dataAssignmentRead()[->] FinalPath[FinalPathListCounter]={}, OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]={},".format(FinalPath[FinalPathListCounter], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]))
                                                        stdout.write("\n\n\n[->]dataAssignmentRead()[->] if StraightPathNodeAvoidEnableFlag and len(IgnoreTagList): main_t2={}, exception={},".format(main_t2, e))
                                                        stdout.flush()
                                                        pass
                                                elif FinalPath[FinalPathListCounter] != FinalPath[FinalPathListCounter+1]:
                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter]) & (dataAssignmentDF.dnode == FinalPath[FinalPathListCounter+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                    # stdout.write("\n[->]dataAssignmentRead()[->]elif St:else: dataAssignmentList={}, main_t2={},".format(dataAssignmentList, main_t2))
                                                    # stdout.flush()
                                                else:
                                                    stdout.write("\n[->]dataAss[->]ightPat:else: FinalPath[FinalPathListCounter]={}, != OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]={},".format(FinalPath[FinalPathListCounter], OriginalFinalPath[OriginalFinalPath.index(FinalPath[FinalPathListCounter])+1]))
                                                    stdout.flush()
                                                    pass

                                            except Exception as e:
                                                stdout.write("\n\n\n[->]dataAssignmentRead()[->]dataAssignmentRead(): main_t2={}, exception={},".format(main_t2, e))
                                                stdout.flush()
                                                pass
                                            tempdist2travel, setX = dataAssignmentList[2], dataAssignmentList[7]
                                            DefaultYaw = dataAssignmentList[3]
                                            # 13/12/22 10:34am
                                            stdout.write("\n[->]dataAssignmentRead()[->]if StraightP: TagID[0]={}, cnode={},".format(TagID[0], cnode))
                                            stdout.flush()
                                            try:
                                                if TagID[0] != 0:
                                                    setXtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 4])
                                                    setYtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 3])
                                                else:
                                                    setXtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == cnode)].iloc[0, 4])
                                                    setYtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == cnode)].iloc[0, 3])
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]dataAssignmentRead()[->]if TagID[0] != 0: exception={},".format(e))
                                                stdout.flush()
                                                pass
                                            
                                            break_mode_t1 = break_mode_t2 = time()
                                            flagonce = tagState = ImageClick_count = capture_click[0] = 0
                                            mode = pmode = ''
                                            while flagonce != 2:
                                                if Estop_FB[0] == '1': ErrorCode[0] = 121
                                                if flagonce == 0:
                                                    setYaw, SetY = round(Tsetpoint, 2), int(ATY[0] * 1000)
                                                # conditionMatch = ((abs(SetY - (ATY[0] * 1000))) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                if cnode == TagID[0]:
                                                    conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                    tagState = 1
                                                else:
                                                    conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)
                                                    tagState = 0
                                                # mode = '1'

                                                if flagonce == 0:
                                                    mode = '1'
                                                    if conditionMatch: flagonce, conditionMatch, mode = 2, 0, '1'

                                                break_mode_t2 = time() - break_mode_t1
                                                if mode == '1' and break_mode_t2 > 0.2: mode = '1'
                                                elif pmode == '7': mode = '7'

                                                # depth error image capture  5/12/22 10:57am
                                                # if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                    # capture_click[0] = 1
                                                # elif capture_click[0] == 3:
                                                    # ImageClick_count += 1
                                                    # capture_click[0] = 0
                                                break_mode_t2 = time() - break_mode_t1
                                                if t_detect480_min[0] < 2:
                                                    mode = '7'
                                                    if break_mode_t2 > 0.2:
                                                        temp_ATY = int(ATY[0] * 1000) + 100
                                                        if cnode == TagID[0]: tagState = 1
                                                        while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                            mode = '1'
                                                            setYaw, SetY = ATYaw[0], temp_ATY
                                                            if TagID[0] == cnode:
                                                                dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                        break_mode_t1 = time()
                                                elif mode == '1': break_mode_t1 = time()
                                                elif mode == '7': mode = '1'
                                                pmode = mode

                                                # mode, TagDetectionState[0], ATYaw[0], setYaw, ATY[0], SetY
                                                SetY = round(SetY, 2)
                                                if cnode == TagID[0]:
                                                    dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                else:
                                                    dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                            flagonce = tagState = 0
                                            # Hypotenuse motion error detection
                                            if narrow_path:
                                                # disable Hypotenuse motion
                                                total_yaw_error = 0
                                                stdout.write("\n[->]assignment()[->]if narrow_path:total_yaw_error=0")
                                                stdout.flush()
                                                pass
                                            elif path_straight_enable == 1:
                                                stdout.write("\n[->]assignment()[->] elif path_straight_enable == 1:")
                                                stdout.flush()
                                                if cnode in VirtualNodeList: xrror, yrror = 0, 0
                                                else:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] abs(xrror)={}, TagID[0]={},".format(abs(xrror), TagID[0]))
                                                    stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] DefaultYaw={}, xrror={}, yrror={}, PreviousSmoothTurnEnable={},".format(DefaultYaw, xrror, yrror, PreviousSmoothTurnEnable))
                                                    stdout.flush()
                                                    if not PreviousSmoothTurnEnable:dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel: total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel: total_yaw_error = degrees(atan(xrror / dist2travel))

                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8

                                                    stdout.write("\n[->]assignment()[->]1488 xrror={}, HypotenusePathLength({}), dist2travel({}), total_yaw_error={}, Tsetpoint={},".format(xrror, HypotenusePathLength, dist2travel, total_yaw_error, Tsetpoint))
                                                    stdout.flush()

                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)
                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] average_x={}, average_y={},".format(average_x, average_y))
                                                    stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] (ATX[0]*1000)={}, setX={}, xrror={},".format((ATX[0]*1000), setX, xrror))
                                                    stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] xrror={0}, yrror={1}, TagDetectionState[0]={2}, total_yaw_error={3}, dist2travel={4}, Tsetpoint={5}".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                    stdout.flush()
                                            elif AchieveLineFuncationFlag:
                                                try:
                                                    if cnode in VirtualNodeList: xrror, yrror = 0, 0
                                                    else:
                                                        xrror = (ATX[0] * 1000) - setX
                                                        yrror = (ATY[0] * 1000)
                                                        if dist2travel >= 400: dist2travel = dist2travel + yrror

                                                        if HypotenusePathLength <= dist2travel: total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                        elif dist2travel: total_yaw_error = degrees(atan(xrror / dist2travel))

                                                        if total_yaw_error > 8: total_yaw_error = 8
                                                        elif total_yaw_error < -8: total_yaw_error = -8

                                                        # stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]1717 xrror={}, HypotenusePathLength({}), dist2travel({}), total_yaw_error={}, Tsetpoint={},".format(xrror, HypotenusePathLength, dist2travel, total_yaw_error, Tsetpoint))
                                                        # stdout.flush()
                                                        # if dist2travel >= 400:  Tsetpoint = round((Tsetpoint - total_yaw_error ) % 360.0,1)
                                                        if dist2travel >= 400: Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 2)
                                                        # stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]if dist2travel >= 400: global_x_reading={}, global_y_reading={}, ValueAchieve={},".format(global_x_reading, global_y_reading, ValueAchieve))
                                                        # stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]if dist2travel >= 400: xrror={}, yrror={}, TagDetectionState[0]={}, total_yaw_error={}, dist2travel={}, Tsetpoint={},".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                        # stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->]1414 elif AchieveLineFuncationFlag: exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                            elif (CurrentTagID[0] == HomeNode or (PreviousSmoothTurnEnable and TagID[0] == cnode)) and TagDetectionState[0]:
                                                stdout.write("\n[->]assignment()1287[->]elif (CurrentTagID[0] == HomeNode or (PreviousSmoothTurnEnable and TagID[0] == cnode)) and TagDetectionState[0]:")
                                                stdout.flush()
                                                if cnode in VirtualNodeList:xrror, yrror = 0, 0
                                                else:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    stdout.write("\n[->]assignment()[->]elif (CurrentTagID[0][->] TagID[0]={}, abs(xrror)={},".format(TagID[0], abs(xrror)))
                                                    stdout.flush()
                                                    if PreviousSmoothTurnEnable:dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel: total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel: total_yaw_error = degrees(atan(xrror / dist2travel))

                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8

                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)

                                                    # stdout.write("\n[->]assignment()[->] average_x={}, average_y={}, setX={}, xrror={},".format(average_x, average_y, setX, xrror))
                                                    stdout.write("\n[->]assignment()[->] total_yaw_error={}, Tsetpoint={},".format(total_yaw_error, Tsetpoint))
                                                    stdout.write("\n[->]assignment()[->] (ATX[0]*1000)={}, setX={}, xrror={},".format((ATX[0]*1000), setX, xrror))
                                                    stdout.write("\n[->]assignment()[->]1558 xrror={}, yrror={}, TagDetectionState[0]={}, total_yaw_error={}, dist2travel={}, Tsetpoint={},".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                    stdout.flush()
                                            else:
                                                try:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    stdout.write("\n[->]assignment()[->]if At_flg == 0:else TagID[0]={}, setX={},".format(TagID[0], setX))
                                                    stdout.flush()
                                                    if PreviousSmoothTurnEnable: dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel: total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel: total_yaw_error = degrees(atan(xrror / dist2travel))

                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8

                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)
                                                    stdout.write("\n[->]assignment()[->]if At_flg == 0:else setX={}, total_yaw_error={}, Tsetpoint={},".format(setX, total_yaw_error, Tsetpoint))
                                                    stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->] else: exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]travelFunction()[->] aprilTagReset(0,0,0) / exception={},".format(e))
                                        stdout.flush()
                                        pass
                                    stdout.write("\n[->]travelFunction()[->] TagID[0]={}, ATYaw[0]={}, ATX[0]={}, ATY[0]={}, TagDetectionState[0]={},".format(TagID[0], ATYaw[0], (ATX[0]*1000), (ATY[0]*1000), TagDetectionState[0]))
                                    stdout.flush()
                                elif AchieveLineFuncationFlag:
                                    stdout.write("\n[->]travelFunction()[->] AchieveLineFuncationFlag turn={},".format(AchieveLineFuncationFlag))
                                    stdout.flush()
                                    try:
                                        stdout.write("\n[->]travelFunction()[->] elif AchieveLineFuncationFlag: assignment({}, {}, {}) START... {}".format(FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], FinalPath[FinalPathListCounter+2], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                        stdout.flush()
                                        cnode, dnode, fnode = FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], FinalPath[FinalPathListCounter+2]
                                        stdout.write("\n[->]1402 assignment(cnode={}, dnode={}, fnode={}) START... {}".format(cnode, dnode, fnode, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                        stdout.flush()
                                        try:
                                            try:
                                                # if DynamicOriginalFinalPath[DynamicOriginalFinalPath.index(cnode)+1] == dnode:
                                                del DynamicOriginalFinalPath[:DynamicOriginalFinalPath.index(cnode)]
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]del DynamicOriginalFinalPath exception={},".format(e))
                                                stdout.flush()
                                                pass
                                            travel = 0
                                            try:
                                                # if fnode:
                                                if cnode not in VirtualNodeList:
                                                    if fnode and fnode not in VirtualNodeList:
                                                        # and (FinalPath[FinalPathListCounter+2] not in VirtualNodeList):
                                                        if (FinalPath[FinalPathListCounter+1] not in VirtualNodeList):
                                                            pathStraightFlag_011 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+1])]['pathStraightFlag'])
                                                            pathStraightFlag_122 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter+1]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+2])]['pathStraightFlag'])
                                                            # stdout.write("\n[->]1422straightPathNodeEliminateFunction()[->]elif pathStraightFlag_011={}, pathStraightFlag_122={},".format(pathStraightFlag_011, pathStraightFlag_122))
                                                            # stdout.flush()
                                                    elif dnode not in VirtualNodeList:
                                                        pathStraightFlag_011 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+1])]['pathStraightFlag'])
                                                        pathStraightFlag_122 = pathStraightFlag_011
                                                    else: pathStraightFlag_122, pathStraightFlag_011 = 1, 1
                                                else: pathStraightFlag_122, pathStraightFlag_011 = 1, 1
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]1429 pathStraightFlag_01 pathStraightFlag_12 exception={},".format(e))
                                                stdout.flush()
                                                # sleep(1)
                                                # system("sudo killall -9 python")
                                                pass
                                            if pathStraightFlag_011 == pathStraightFlag_122 or fnode == 0: SmoothTurnEnable = 0
                                            else:
                                                SmoothTurnEnable = 1
                                                # stdout.write("\n[->]assignment()[->]1438 For Hypotenuse Start from homenode SmoothTurnEnable={},".format(SmoothTurnEnable))
                                                # stdout.flush()
                                                try:
                                                    if cnode == HomeNode:
                                                        A, B = [cnode, dnode], OriginalFinalPath
                                                        stdout.write("\n[->]assignment()[->] A={}, B={},".format(A, B))
                                                        stdout.flush()
                                                        avoided_node_input = any(A == B[i:i + 2] for i in range(len(B)-2 + 1))
                                                        # stdout.write("\n[->]assignment()[->] avoided_node_input={},".format(avoided_node_input))
                                                        # stdout.flush()
                                                        if not avoided_node_input: path_straight_enable = 2
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->] avoided_node_input exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                            fnarrow_path, fpath_straight_enable = 0, 0
                                            # EstopPressFeedbackFlag[0] = 0
                                            ATDetection_CurrentTag[0], ATDetection_DestinationTag[0] = cnode, dnode
                                            pathStraightFlag_0, pathStraightFlag_1 = 0, 0
                                            # metadataListAssignment(cnode,dnode)
                                            #                  cnode                           dnode                              fnode
                                            # FinalPath[FinalPathListCounter],FinalPath[FinalPathListCounter+1],FinalPath[FinalPathListCounter+2]
                                            # stdout.write("\n[->]assignment()[->] FinalPath={}, FinalPathListCounter={},".format(FinalPath, FinalPathListCounter))
                                            # stdout.flush()
                                            try:
                                                if cnode in VirtualNodeList:
                                                    dataAssignmentList = virtualNodeDF[(virtualNodeDF.VirtualNode == cnode) & (virtualNodeDF.RealNode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                    answer = float(dataAssignmentList[3]) - 180
                                                    if answer < 0: dataAssignmentList[3] = 360 + answer
                                                    else: dataAssignmentList[3] = answer
                                                    Tsetpoint = float(virtualNodeDF[(virtualNodeDF.RealNode == dnode) & (virtualNodeDF.VirtualNode == cnode)].iloc[0, 4])
                                                elif dnode in VirtualNodeList:
                                                    dataAssignmentList = virtualNodeDF[(virtualNodeDF.RealNode == cnode) & (virtualNodeDF.VirtualNode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                    Tsetpoint = float(virtualNodeDF[(virtualNodeDF.RealNode == cnode) & (virtualNodeDF.VirtualNode == dnode)].iloc[0, 3])

                                                elif StraightPathNodeAvoidEnableFlag and len(IgnoreTagList):
                                                    # stdout.write("\n[->]assignment()[->] OriginalFinalPath={}, OriginalFinalPath.count(cnode)={},".format(OriginalFinalPath, OriginalFinalPath.count(cnode)))
                                                    # stdout.flush()
                                                    if OriginalFinalPath.count(cnode) >= 2:
                                                        # stdout.write("\n[->]assignment()[->]if OriginalFina: cnode={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1](next original node)={}, OriginalFinalPath[OriginalFinalPath.index(cnode,OriginalFinalPath.index(cnode)+1)+1](dnode={}) == {}".format(cnode, OriginalFinalPath[OriginalFinalPath.index(cnode)+1], OriginalFinalPath[OriginalFinalPath.index(cnode,OriginalFinalPath.index(cnode)+1)+1], dnode))
                                                        # stdout.write("\n[->]assignmnalFinalPath.index(cnode)+1]={}, OriginalFinalPath.index(cnode)+1={}, main_t2={},".format(OriginalFinalPath[OriginalFinalPath.index(cnode)+1], OriginalFinalPath.index(cnode)+1, main_t2))
                                                        # stdout.flush()
                                                        if OriginalFinalPath[OriginalFinalPath.index(cnode)+1] in VirtualNodeList:
                                                            temp_OriginalFinalPath = OriginalFinalPath[OriginalFinalPath.index(cnode, OriginalFinalPath.index(cnode)+1)+1]
                                                            # stdout.write("\n[->]assignmenodeList: cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(cnode, temp_OriginalFinalPath, main_t2))
                                                            # stdout.flush()
                                                            if cnode != temp_OriginalFinalPath and temp_OriginalFinalPath not in VirtualNodeList:
                                                                # stdout.write("\n[->]assignode != temp_: cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(cnode, temp_OriginalFinalPath, main_t2))
                                                                # stdout.flush()
                                                                dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode, OriginalFinalPath.index(cnode)+1)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                            elif cnode != temp_OriginalFinalPath and temp_OriginalFinalPath in VirtualNodeList:
                                                                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == temp_OriginalFinalPath]['RealNode'])
                                                                # stdout.write("\n[->]assignm!= temp_:real_node={}, cnode={}, temp_OriginalFinalPath={}, main_t2={},".format(real_node, cnode, temp_OriginalFinalPath, main_t2))
                                                                # stdout.flush()
                                                                if real_node != cnode:
                                                                    # stdout.write("\n[->]assit()[->]ifcno:")
                                                                    # stdout.flush()
                                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == real_node)].dropna(axis=1).iloc[0, 0:].tolist()
                                                        else:
                                                            # stdout.write("\n[->]assigOriginalFinalPath[OriginalFinalPath.index(cnode)+1]else: condition_status cnode={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1](dnode)={},".format(cnode, OriginalFinalPath[OriginalFinalPath.index(cnode)+1]))
                                                            # stdout.write("\n[->]assignment()[-> else: condition_status DynamicOriginalFinalPath[0](cnode)={}, DynamicOriginalFinalPath[1](dnode)={}, DynamicOriginalFinalPath={}, main_t2={},".format(DynamicOriginalFinalPath[0], DynamicOriginalFinalPath[1], DynamicOriginalFinalPath, main_t2))
                                                            # stdout.flush()
                                                            # dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0,0:].tolist()
                                                            dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == DynamicOriginalFinalPath[DynamicOriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                    else:
                                                        # dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0,0:].tolist()
                                                        # stdout.write("\n[->]ariginalFinalPath[OriginalFinalPath.index(cnode)](cnode)={}, OriginalFinalPath[OriginalFinalPath.index(cnode)+1]={}, main_t2={},".format(OriginalFinalPath[OriginalFinalPath.index(cnode)], OriginalFinalPath[OriginalFinalPath.index(cnode)+1], main_t2))
                                                        # stdout.flush()
                                                        dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[OriginalFinalPath.index(cnode)]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                    # stdout.write("\n[->]assignment()[->] dataAssignmentList 1={},".format(dataAssignmentList))
                                                    # stdout.write("\n[->]assignment()[->] NewPathList={}, OriginalFinalPath={}, cnode={},".format(NewPathList, OriginalFinalPath, cnode))
                                                    # stdout.flush()

                                                    try:
                                                        # dataAssignmentList[2] = int([data[-1] for data in NewPathList if data[0] == cnode and data[1] == dnode][0])
                                                        for data in NewPathList:
                                                            if (data[0] == cnode and data[1] == dnode):
                                                                dataAssignmentList[2] = int(data[-1])
                                                            else: pass
                                                    except Exception as e:
                                                        stdout.write("\n\n\n[->]assignment()[->]dataAssignmentList[2] exception={},".format(e))
                                                        stdout.flush()
                                                        pass
                                                    # if OriginalFinalPath[1] in VirtualNodeList:
                                                        # Tsetpoint = float(virtualNodeDF[(virtualNodeDF.RealNode == OriginalFinalPath[0]) & (virtualNodeDF.VirtualNode == OriginalFinalPath[1])].iloc[0, 4])
                                                        # # Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[1])]['Tsetpoint'])
                                                    # else:
                                                        # Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[1])]['Tsetpoint'])
                                                    # Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[OriginalFinalPath.index(cnode)]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(cnode) + 1])]['Tsetpoint'])
                                                    if OriginalFinalPath[0] not in VirtualNodeList:
                                                        if OriginalFinalPath[1] not in VirtualNodeList:
                                                            Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[1])]['Tsetpoint'])
                                                        elif OriginalFinalPath[1] in VirtualNodeList and OriginalFinalPath[2] not in VirtualNodeList and OriginalFinalPath[0] != OriginalFinalPath[2]:
                                                            Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[2])]['Tsetpoint'])
                                                        elif OriginalFinalPath[1] in VirtualNodeList and OriginalFinalPath[3] not in VirtualNodeList and OriginalFinalPath[0] != OriginalFinalPath[3]:
                                                            Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[3])]['Tsetpoint'])
                                                    # stdout.write("\n[->]assignment()[->] dataAssignmentList 2={}, Tsetpoint={}, OriginalFinalPath={},".format(dataAssignmentList, Tsetpoint, OriginalFinalPath))
                                                    # stdout.flush()

                                                else:
                                                    dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == dnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                    Tsetpoint = float(dataAssignmentDF[(dataAssignmentDF.cnode == cnode) & (dataAssignmentDF.dnode == dnode)]['Tsetpoint'])  # 17/11/22 1:36pm
                                                    # stdout.write("\n[->]assignment()[->]try:else: ")
                                                # stdout.write("\n[->]assignment()[->]1575try: cnode={}, dnode={}, dataAssignmentList={},".format(cnode, dnode, dataAssignmentList))
                                                # stdout.flush()
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]assignment()[->] dataAssignmentList exception={},".format(e))
                                                stdout.flush()
                                                pass
                                            stdout.write("\n[->]assignment()[->]1580final cnode={}, dnode={}, dataAssignmentList={},".format(cnode, dnode, dataAssignmentList))
                                            stdout.flush()
                                            narrow_path, setX = dataAssignmentList[5], dataAssignmentList[7]
                                            # setX = float(dataAssignmentDF[(dataAssignmentDF.cnode == OriginalFinalPath[0])&(dataAssignmentDF.dnode == OriginalFinalPath[1])]['setX(mm)'])
                                            smoothExitX, smoothExitY = dataAssignmentList[9], dataAssignmentList[10]
                                            # stdout.write("\n[->]assignment()[->] tempdist2travel={}, Tsetpoint={}, narrow_path={}, min_depth_setpoint={}, fnode={}".format(tempdist2travel, Tsetpoint, narrow_path, min_depth_setpoint, fnode))
                                            # stdout.flush()

                                            if fnode != 0:
                                                try:
                                                    # dnode in VirtualNodeList or
                                                    if fnode in VirtualNodeList:
                                                        dataAssignmentList = virtualNodeDF[(virtualNodeDF.RealNode == dnode) & (virtualNodeDF.VirtualNode == fnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                    elif StraightPathNodeAvoidEnableFlag and len(IgnoreTagList):
                                                        dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == dnode) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(dnode)+1])].dropna(axis=1).iloc[0, 0:].tolist()
                                                    else:
                                                        dataAssignmentList = dataAssignmentDF[(dataAssignmentDF.cnode == dnode) & (dataAssignmentDF.dnode == fnode)].dropna(axis=1).iloc[0, 0:].tolist()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->] if fnode != 0: dataAssignmentList exception={}".format(e))
                                                    stdout.flush()
                                                    pass
                                                # stdout.write("\n[->]assignment()[->] if fnode != 0: dnode={}, fnode={}, dataAssignmentList={},".format(dnode, fnode, dataAssignmentList))
                                                # stdout.flush()

                                                futureTsetpoint, fnarrow_path = dataAssignmentList[3], dataAssignmentList[5]
                                                # smoothExitX = dataAssignmentList[9]
                                                # smoothExitY = dataAssignmentList[10]
                                                # stdout.write("\n[->]assignment()[->] futureTsetpoint={}, fnarrow_path={}".format(futureTsetpoint, fnarrow_path))
                                                # stdout.flush()

                                            # stdout.write("\n[->]assignment()[->] narrow_path={}, SmoothTurnEnable={}, fnarrow_path={}, PreviousSmoothTurnEnable={}, fpath_straight_enable={}, path_straight_enable={},".format(narrow_path, SmoothTurnEnable, fnarrow_path, PreviousSmoothTurnEnable, fpath_straight_enable, path_straight_enable))
                                            # stdout.flush()
                                            # dist2travel = abs(int(tempdist2travel * cos(radians(tempsetpoint))))
                                            dist2travel, tTsetpoint = tempdist2travel, Tsetpoint
                                            # stdout.write("\n[->]assignment()[->] Tsetpoint={},".format(Tsetpoint))
                                            # stdout.flush()
                                            # stdout.write("\n[->]assignment()[->] 1 tTsetpoint=Tsetpoint={}, TagID[0]={}, FinalPathListCounter={}, FinalPath={},".format(Tsetpoint, TagID[0], FinalPathListCounter, FinalPath))
                                            # stdout.write("\n[->]assignment()[->] 2 tTsetpoint=Tsetpoint={}, TagID[0]={}, FinalPathListCounter={}, FinalPath={}, pathStraightFlag_0={}, pathStraightFlag_1={},".format(Tsetpoint, TagID[0], FinalPathListCounter, FinalPath, pathStraightFlag_0, pathStraightFlag_1))
                                            # stdout.write("\n[->]assignment()[->] 1 setX={},".format(setX))
                                            # stdout.flush()
                                            if (FinalPathListCounter >= 1) and (len(FinalPath) >= 3) and (len(set(FinalPath) & set(VirtualNodeList)) == 0):
                                                pathStraightFlag_0 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter-1]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter])]['pathStraightFlag'])
                                                pathStraightFlag_1 = int(PathAssignmentDF[(PathAssignmentDF.cn == FinalPath[FinalPathListCounter]) & (PathAssignmentDF.dn == FinalPath[FinalPathListCounter+1])]['pathStraightFlag'])

                                            if (pathStraightFlag_0 != 1) and (pathStraightFlag_0 != pathStraightFlag_1):
                                                # setX = -setX
                                                stdout.write("\n[->]assignment()[->] 2 setX={},".format(setX))
                                                stdout.flush()
                                            # stdout.write("\n[->]assignment()[->] 1867 pathStraightFlag_0={}, pathStraightFlag_1={}, setX={},".format(pathStraightFlag_0, pathStraightFlag_1, setX))
                                            # stdout.flush()
                                            # HypotenusePathLength error detection
                                            if narrow_path: pass
                                            elif path_straight_enable == 1:
                                                stdout.write("\n[->]assignment()[->] elif path_straight_enable == 1:")
                                                stdout.flush()
                                                if cnode in VirtualNodeList: xrror, yrror = 0, 0
                                                else:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] abs(xrror)={},".format(abs(xrror)))
                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] DefaultYaw={}, xrror={}, yrror={}, PreviousSmoothTurnEnable={},".format(DefaultYaw, xrror, yrror, PreviousSmoothTurnEnable))
                                                    # stdout.flush()
                                                    if not PreviousSmoothTurnEnable: dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel:
                                                        total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel:  # 17/11/22  5:10pm
                                                        total_yaw_error = degrees(atan(xrror / dist2travel))

                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8
                                                    # stdout.write("\n[->]assignment()[->]1895 xrror={}, HypotenusePathLength({}), dist2travel({}), total_yaw_error={}, Tsetpoint={},".format(xrror, HypotenusePathLength, dist2travel, total_yaw_error, Tsetpoint))
                                                    # stdout.flush()
                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)

                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] average_x={}, average_y={},".format(average_x, average_y))
                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] (ATX[0]*1000)={}, setX={}, xrror={},".format((ATX[0]*1000), setX, xrror))
                                                    # stdout.write("\n[->]assignment()[->]elif path_straight_enable == 1:[->] xrror={0}, yrror={1}, TagDetectionState[0]={2}, total_yaw_error={3}, dist2travel={4}, Tsetpoint={5}".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                    # stdout.flush()
                                            elif AchieveLineFuncationFlag:
                                                try:
                                                    stdout.write("\n[->]assignment()[->]1660 elif AchieveLineFuncationFlag:[->] Tsetpoint={}, ATYaw[0]={}, SetY={}, ATY[0]={},".format(Tsetpoint, ATYaw[0], SetY, (ATY[0] * 1000)))
                                                    stdout.flush()
                                                    # only for set Tag Entry Angle.
                                                    break_mode_t1 = break_mode_t2 = time()
                                                    flagonce = ImageClick_count = capture_click[0] = 0
                                                    mode = pmode = ''
                                                    while flagonce != 2:
                                                        if Estop_FB[0] == '1': ErrorCode[0] = 121
                                                        if flagonce == 0:
                                                            if TagDetectionState[0]:
                                                                setYaw, SetY = round(Tsetpoint, 2), int(ATY[0] * 1000)
                                                                tagState = 1
                                                            else:
                                                                setYaw, SetY = ATYaw[0], int(ATY[0] * 1000) - 100
                                                        # conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                        # 1/12/22 12:55pm
                                                        if cnode == TagID[0]:
                                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                        else:
                                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)
                                                            tagState = 0
                                                        # mode = '1'
                                                        if flagonce == 0:
                                                            mode = '1'
                                                            if conditionMatch: flagonce, conditionMatch, mode = 2, 0, '1'

                                                        # depth error image capture  5/12/22 10:57am
                                                        if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                            capture_click[0] = 1
                                                        elif capture_click[0] == 3:
                                                            ImageClick_count += 1
                                                            # capture_click[0] = 0
                                                        break_mode_t2 = time() - break_mode_t1
                                                        if t_detect480_min[0] < 2:
                                                            mode = '7'
                                                            if break_mode_t2 > 0.2:
                                                                temp_ATY = int(ATY[0] * 1000) + 100
                                                                while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                                    mode = '1'
                                                                    setYaw, SetY = ATYaw[0], temp_ATY
                                                                    if TagID[0] == cnode:
                                                                        dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                                break_mode_t1 = time()
                                                        elif mode == '1': break_mode_t1 = time()
                                                        elif mode == '7': mode = '1'
                                                        pmode = mode
                                                        if cnode == TagID[0]:
                                                            dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(int(SetY)) + '\n'
                                                        else:
                                                            dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(int(SetY)) + '\n'  # 10/11/22 11:19am

                                                    flagonce = tagState = 0
                                                    stdout.write("\n[->]assignment()[->]1 1660 elif AchieveLineFuncationFlag:[->] Tsetpoint={}, ATYaw[0]={}, SetY={}, ATY[0]={},".format(Tsetpoint, ATYaw[0], SetY, (ATY[0] * 1000)))
                                                    stdout.flush()

                                                    if cnode in VirtualNodeList: xrror, yrror = 0, 0
                                                    else:
                                                        xrror = (ATX[0] * 1000) - setX
                                                        yrror = (ATY[0] * 1000)
                                                        stdout.write("\n[->]assignment()[->]1661 elif AchieveLineFuncationFlag:[->] abs(xrror)={}, (ATX[0]*1000)={}, setX={},".format(abs(xrror), (ATX[0]*1000), setX))
                                                        stdout.flush()

                                                        if dist2travel >= 400: dist2travel = dist2travel + yrror
                                                        if HypotenusePathLength <= dist2travel:
                                                            total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                        elif dist2travel:  # 17/11/22  5:10pm
                                                            total_yaw_error = degrees(atan(xrror / dist2travel))

                                                        # 1/12/22 7:25pm
                                                        if total_yaw_error > 5: total_yaw_error = 5
                                                        elif total_yaw_error < -5: total_yaw_error = -5

                                                        stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]2033 xrror={}, HypotenusePathLength({}), dist2travel({}), total_yaw_error={}, Tsetpoint={},".format(xrror, HypotenusePathLength, dist2travel, total_yaw_error, Tsetpoint))
                                                        stdout.flush()
                                                        if dist2travel >= 400:
                                                            # original
                                                            Tsetpoint = ((Tsetpoint + total_yaw_error) % 360)
                                                        # stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]if dist2travel >= 400: global_x_reading={}, global_y_reading={}, ValueAchieve={},".format(global_x_reading, global_y_reading, ValueAchieve))
                                                        stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:[->]if dist2travel >= 400: xrror={}, yrror={}, TagDetectionState[0]={}, total_yaw_error={}, dist2travel={}, Tsetpoint={},".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                        stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->] elif AchieveLineFuncationFlag: exception={},".format(e))
                                                    stdout.flush()
                                                    pass
                                            elif (CurrentTagID[0] == HomeNode or (PreviousSmoothTurnEnable and TagID[0] == cnode)) and TagDetectionState[0]:
                                                stdout.write("\n[->]assignment()1675[->]elif (CurrentTagID[0] == HomeNode or (PreviousSmoothTurnEnable and TagID[0] == cnode)) and TagDetectionState[0]:")
                                                stdout.flush()
                                                if cnode in VirtualNodeList: xrror, yrror = 0, 0
                                                else:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag: elif (CurrentTagID[0][->] abs(xrror)={},".format(abs(xrror)))
                                                    stdout.flush()

                                                    if PreviousSmoothTurnEnable: dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel:
                                                        total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel:  # 17/11/22  5:10pm
                                                        total_yaw_error = degrees(atan(xrror / dist2travel))

                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8

                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)
                                                    # stdout.write("\n[->]assignment()[->] average_x={}, average_y={}, setX={}, xrror={},".format(average_x, average_y, setX, xrror))
                                                    stdout.write("\n[->]assignment()[->] total_yaw_error={}, Tsetpoint={},".format(total_yaw_error, Tsetpoint))
                                                    stdout.write("\n[->]assignment()[->] (ATX[0]*1000)={}, setX={}, xrror={},".format((ATX[0]*1000), setX, xrror))
                                                    stdout.write("\n[->]assignment()[->]1966 xrror={}, yrror={}, TagDetectionState[0]={}, total_yaw_error={}, dist2travel={}, Tsetpoint={},".format(xrror, yrror, TagDetectionState[0], total_yaw_error, dist2travel, Tsetpoint))
                                                    stdout.flush()
                                            else:
                                                try:
                                                    xrror = (ATX[0] * 1000) - setX
                                                    yrror = (ATY[0] * 1000)
                                                    stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:else setX={},".format(setX))
                                                    stdout.flush()

                                                    if PreviousSmoothTurnEnable: dist2travel = dist2travel + yrror

                                                    if HypotenusePathLength <= dist2travel:
                                                        total_yaw_error = degrees(atan(xrror / HypotenusePathLength))
                                                    elif dist2travel:  # 17/11/22  5:10pm
                                                        total_yaw_error = degrees(atan(xrror / dist2travel))
                                                    if total_yaw_error > 8: total_yaw_error = 8
                                                    elif total_yaw_error < -8: total_yaw_error = -8

                                                    Tsetpoint = round((Tsetpoint + total_yaw_error) % 360.0, 1)
                                                    stdout.write("\n[->]assignment()[->]elif AchieveLineFuncationFlag:else setX={}, total_yaw_error={}, Tsetpoint={},".format(setX, total_yaw_error, Tsetpoint))
                                                    stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[->]assignment()[->]elif AchieveLineFuncationFlag:else: exception={},".format(e))
                                                    stdout.flush()
                                                    pass

                                        except Exception as e:
                                            stdout.write("\n\n\n[->]assignment()[->]1740 exception={},".format(e))
                                            stdout.flush()
                                            pass
                                        # stdout.write("\n[->]travelFunction()[->] assignment({}, {}, {}) End... {}".format(FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], FinalPath[FinalPathListCounter+2], date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]travelFunction()[->] assignment() / exception={},".format(e))
                                        stdout.flush()
                                        pass

                                stdout.write("\n[->]travelFunction()[->] FinalPath={}, total_yaw_error={},".format(FinalPath, total_yaw_error))
                                stdout.write("\n[->]travelFunction()[->] FinalPath[FinalPathListCounter](={},) --> FinalPath[FinalPathListCounter+1](={},) EndNode={}, HomeNode={},".format(FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], EndNode, HomeNode))
                                stdout.write("\n[->]travelFunction()[->] path_straight_enable={}, PreviousSmoothTurnEnable={},".format(path_straight_enable, PreviousSmoothTurnEnable))
                                stdout.write("\n[->]travelFunction()[->] fpath_straight_enable={}, SmoothTurnEnable={},".format(fpath_straight_enable, SmoothTurnEnable))
                                stdout.flush()

                                # if path_straight_enable != 1:
                                if FinalPath[FinalPathListCounter] == FinalPath[0] or FinalPath[FinalPathListCounter] == HomeNode or PreviousSmoothTurnEnable or FinalPath[FinalPathListCounter+1] in VirtualNodeList:
                                    stdout.write("\n[->]travelFunction()[->] FinalPath={}, FinalPath[FinalPathListCounter]={}, HomeNode={}, EndNode={},".format(FinalPath, FinalPath[FinalPathListCounter], HomeNode, EndNode))
                                    stdout.write("\n[->]travelFunction()[->] 0 TagDetectionState[0]={}, Tsetpoint={}, ATYaw[0]={}, ATY[0]={}, SetY={}, BmiYAW[0]={},".format(TagDetectionState[0], Tsetpoint, ATYaw[0], ATY[0] * 1000, SetY, BmiYAW[0]))
                                    stdout.flush()

                                    # only set angle
                                    break_mode_t1 = break_mode_t2 = time()
                                    flagonce = ImageClick_count = capture_click[0] = 0
                                    mode = pmode = ''
                                    while flagonce != 2:
                                        if Estop_FB[0] == '1': ErrorCode[0] = 121
                                        if flagonce == 0:
                                            setYaw, SetY = round(Tsetpoint, 2), int(ATY[0] * 1000)

                                        if FinalPath[FinalPathListCounter] not in VirtualNodeList or TagDetectionState[0]:
                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                        else:
                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)

                                        mode = '1'
                                        if flagonce == 0:
                                            mode = '1'
                                            if conditionMatch: flagonce, conditionMatch, mode = 2, 0, '1'
                                        # depth error image capture  5/12/22 10:57am
                                        if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                            capture_click[0] = 1
                                        elif capture_click[0] == 3:
                                            ImageClick_count += 1
                                            # capture_click[0] = 0
                                        break_mode_t2 = time() - break_mode_t1
                                        if t_detect480_min[0] < 2:
                                            mode = '7'
                                            if break_mode_t2 > 0.2:
                                                temp_ATY = int(ATY[0] * 1000) + 100
                                                while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                    mode = '1'
                                                    setYaw, SetY = ATYaw[0], temp_ATY
                                                    if TagID[0] == FinalPath[FinalPathListCounter] and FinalPath[FinalPathListCounter] not in VirtualNodeList:
                                                        dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                break_mode_t1 = time()
                                        elif mode == '1': break_mode_t1 = time()
                                        elif mode == '7': mode = '1'
                                        pmode = mode
                                        # mode, TagDetectionState[0], ATYaw[0], setYaw, ATY[0], SetY
                                        SetY = int(SetY)
                                        if FinalPath[FinalPathListCounter] not in VirtualNodeList or TagDetectionState[0]:
                                            dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                            # conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                        else:
                                            dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(BmiYAW[0])) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(int(ATY[0] * 1000)) + '\n'
                                            # conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)
                                        # stdout.write("\n[->]main()[->] conditionMatch={}, flagonce={}, setYaw={}, SetY={}, abs(SetY-(ATY[0]*1000))={}, abs(setYaw - ATYaw[0])={}, (ATY[0]*1000)={}, ATYaw[0]={},".format(conditionMatch, flagonce, setYaw, SetY, abs(SetY-(ATY[0]*1000)), round(abs(setYaw - ATYaw[0]), 3), (ATY[0]*1000), ATYaw[0]))
                                        # stdout.write("\n[->]main()[->] conditionMatch={}, flagonce={}, setYaw={}, SetY={}, abs(SetY-(ATY[0]*1000))={}, abs(sYaw-ATYaw[0])={},".format(conditionMatch, flagonce, setYaw, SetY, abs(SetY-(ATY[0]*1000)), round(abs(setYaw - ATYaw[0]), 3)))
                                    flagonce = 0
                                    stdout.write("\n[->]travelFunction()[->] 1 TagDetectionState[0]={}, Tsetpoint={}, ATYaw[0]={}, ATY[0]={}, SetY={}, BmiYAW[0]={},".format(TagDetectionState[0], Tsetpoint, ATYaw[0], ATY[0] * 1000, SetY, BmiYAW[0]))
                                    stdout.flush()
                                else:
                                    setYaw = round(Tsetpoint, 2)
                                    stdout.write("\n[->]travelFunction()[->]else: setYaw={},".format(setYaw))
                                    stdout.flush()
                                try:
                                    dist2travel = round(dist2travel, 2)
                                    if dist2travel >= 100:

                                        # motion()
                                        # stdout.write("\n[->]travelFunction()[->] motion() End... {}".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                        flg = 0
                                        stdout.write("\n[->]motion()[->] while True[->] 0 if dist2travel >= 100: dist2travel={}, TagID[0]={},".format(dist2travel, TagID[0]))
                                        stdout.flush()
                                        if SmoothTurnEnable or PreviousSmoothTurnEnable:
                                            if DestinationNode != HomeNode:
                                                smooth_turn_radius = 400
                                                dist2travel = dist2travel - (smooth_turn_radius * (SmoothTurnEnable + PreviousSmoothTurnEnable))
                                        # stdout.write("\n[->]motion()[->] while True[->] 1 if dist2travel >= 100: dist2travel={}, total_yaw_error={},".format(dist2travel, total_yaw_error))
                                        stdout.write("\n[->]motion()[->] while True[->] 1 if dist2travel >= 100: dist2travel={},".format(dist2travel))
                                        stdout.flush()
                                        MotionSection = 0
                                        break_mode_t1, break_mode_t2, break_mode_t3 = time(), time(), 0.05
                                        break_SOLmode_t1, break_SOLmode_t2, break_SOLmode_t3 = time(), time(), 1
                                        tagDetectionState_t1, tagDetectionState_t2 = time(), time()
                                        v_tagDetectionState_t1, v_tagDetectionState_t2 = time(), time()
                                        tagDetect_flag = capture_click[0] = 0
                                        try:
                                            LedData1[0] = '7\n'
                                            if LedData1[0] != prev_LedData1:
                                                # ESP32_Ser.write(LedData1[0].encode())
                                                stdout.write("\n[-->ESP32_T]if dist2travel >= 100: total_travel={}, total_travel_0={}, LedData1[0]={},".format(total_travel, total_travel_0, LedData1[0]))
                                                stdout.flush()
                                                prev_LedData1 = LedData1[0]
                                        except Exception as e:
                                            stdout.write("\n\n\n[->][-->ESP32_T]if dist2travel >= 100: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                        total_travel += total_travel_0
                                        total_travel_0 = 0
                                        while True:
                                            # stdout.write("\n[->]motion()[->] while True[->] MotionSection={}, InputProcessEnable[0]={},".format(MotionSection, InputProcessEnable[0]))
                                            try:
                                                if MotionSection == 0:
                                                    if not PreviousSmoothTurnEnable and path_straight_enable != 1:
                                                        stdout.write("\n[->]motion()[->]if not PreviousSmoothTurnEnable and path_straight_enable != 1: ")
                                                        stdout.flush()
                                                    SetPoint = tTsetpoint
                                                    temp_setYaw = setYaw
                                                    stdout.write("\n[->]motion()[->]if not PreviousSmoothTurnEnable and path_straight_enable != 1: TagID[0]={}, SetPoint={}, = tTsetpoint={},".format(TagID[0], SetPoint, tTsetpoint))
                                                    stdout.flush()
                                                    MotionSection, capture_click[0] = 1, 0
                                                    Hypotenuse_AngleFlag, travel_depthSol_0 = 0, 0
                                                    depthSol_flag, travel_virtual_0, travel_virtual = 0, 0, 0
                                                    travel_0 = travel_virtual_0 = RightEncData[0]
                                                    Not_mode2_counter1, Not_mode2_counter2 = time(), time()
                                                    mode = pmode = '2'
                                                    # stdout.write("\n[->]MotionSection = 0 END")
                                                elif MotionSection == 1:
                                                    # stdout.write("\n[->]motion()[->] MotionSection = 1 Start")
                                                    # stdout.write("\n[->]motion()[->]elif MotionSection == 1: TagID[0]={}, FinalPath[FinalPathListCounter+1]={},".format(TagID[0], FinalPath[FinalPathListCounter+1]))
                                                    if TagID[0] == FinalPath[FinalPathListCounter+1] and TagDetectionState[0]:
                                                        mode = '2'
                                                        dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'  # 10/11/22 11:47am
                                                        tagDetectionState_t2 = time() - tagDetectionState_t1
                                                        total_travel_0 = RightEncData[0]
                                                        tagDetect_flag = 1  # 11/11/22 11:02am
                                                        # if tagDetectionState_t2 > (0.4) and (abs(ATX[0] * 1000) < 600 and abs(ATY[0] * 1000) < 600):  # 18/11/22 4:30am
                                                        if tagDetectionState_t2 > (0.6) or (abs(ATX[0] * 1000) < 800 and abs(ATY[0] * 1000) < 800):
                                                            MotionSection = 2
                                                            stdout.write("\n[->]MotionSection1->2[->] tagDetectionState_t2={}, TagID[0]={}, FinalPath={}, abs(ATX[0])={}, abs(ATY[0])={},".format(tagDetectionState_t2, TagID[0], FinalPath, abs(ATX[0] * 1000), abs(ATY[0] * 1000)))
                                                            stdout.flush()
                                                    elif (TagID[0] != FinalPath[FinalPathListCounter+1]) or (FinalPath[FinalPathListCounter] in VirtualNodeList):
                                                        # stdout.write("\n[->]MotionSection = 1[->]if TagID[0] != FinalPath[FinalPathLis TagID[0]={}, FinalPath={},".format(TagID[0], FinalPath))
                                                        # Hypotenuse End conditions
                                                        # NOTE: right now hypotenuse angle not implement but not in use due to encoder readings.
                                                        # stdout.write("\n[->]motion()[->]if (not flg={},) and (path_straight_enable={}, or AchieveLineFuncationFlag={},): ATYaw[0]={}, travel={}, travel_0={},".format(flg, path_straight_enable, AchieveLineFuncationFlag, ATYaw[0], travel, travel_0))
                                                        if HypotenusePathLength > 300: HypotenuseBeta = HypotenusePathLength - 300
                                                        else: HypotenuseBeta = HypotenusePathLength
                                                        if (mode == '2' and mode != pmode) or travel_0 > RightEncData[0]: travel_0 = RightEncData[0]
                                                        else: total_travel_0 = RightEncData[0]

                                                        travel = round(abs(RightEncData[0] - travel_0), 2)

                                                        # stdout.write("\n[->]motion()[->] travel={}, RightEncData[0]={}, travel_0={},".format(travel, RightEncData[0], travel_0))
                                                        if (not flg) and (path_straight_enable or AchieveLineFuncationFlag) and (not Hypotenuse_AngleFlag):
                                                            if travel >= HypotenuseBeta:
                                                                # stdout.write("\n[->]motion()[->]if travel(={},)>=HypotenuseBeta(={},): RightEncData[0]={}, travel_0={}, Previous Tsetpoint={}, tTsetpoint={},".format(travel, HypotenuseBeta, RightEncData[0], travel_0, Tsetpoint, tTsetpoint))
                                                                temp_setYaw = setYaw = Tsetpoint = tTsetpoint
                                                                Hypotenuse_AngleFlag = 1
                                                        elif (CurrentTagID[0] == HomeNode) and (not Hypotenuse_AngleFlag):
                                                            # stdout.write("\n[->]motion()[->]elif[->]if travel(={},)>=HypotenuseBeta(={},): RightEncData[0]={}, travel_0={}, Previous Tsetpoint={}, tTsetpoint={},".format(travel, HypotenuseBeta, RightEncData[0], travel_0, Tsetpoint, tTsetpoint))
                                                            if travel >= HypotenuseBeta:
                                                                # stdout.write("\n[->]motion()[->]elif[->]if travel(={},)>=HypotenuseBeta(={},): RightEncData[0]={}, travel_0={}, Previous Tsetpoint={}, tTsetpoint={},".format(travel, HypotenuseBeta, RightEncData[0], travel_0, Tsetpoint, tTsetpoint))
                                                                temp_setYaw = setYaw = Tsetpoint = tTsetpoint
                                                                Hypotenuse_AngleFlag = 1
                                                                # stdout.write("\n[->]motion()[->]elif[->]if travel({})>=HypotenuseBeta({}): Current Tsetpoint={},".format(travel, HypotenuseBeta, Tsetpoint))
                                                                flg = 1
                                                        # mode = '2'
                                                        # Top + Bottom Depth Cam breaking in mode 2
                                                        # break_less_time = 0
                                                        if Estop_FB[0] == '0':
                                                            break_mode_t2 = time() - break_mode_t1
                                                            if dist2travel > 1000:
                                                                if (b_detecty_min[0] <= 2) or (t_detect480_min[0] <= 10):
                                                                    break_mode_t1, break_SOLmode_t2 = time(), time() - break_SOLmode_t1
                                                                    if detectedSide480[0] or detectedSideB[0]:
                                                                        mode, ErrorCode[0] = '4', 120
                                                                        if break_SOLmode_t2 > break_SOLmode_t3:
                                                                            if detectedSide480[0]:
                                                                                setYaw = setYaw + (detectedSide480[0] * 2)
                                                                                setYaw = setYaw % 360
                                                                                break_SOLmode_t1, depthSol_flag = time(), 1
                                                                            elif detectedSideB[0]:
                                                                                setYaw = setYaw + (detectedSideB[0] * 2)
                                                                                setYaw = setYaw % 360
                                                                                break_SOLmode_t1, depthSol_flag = time(), 1
                                                                            else:
                                                                                capture_click[0] = 1
                                                                                stdout.write("\n[->]t_detect480_min[0]={}, detectedSide480[0]={}, b_detecty_min[0]={}, detectedSideB[0]={}, break_SOLmode_t2={},".format(t_detect480_min[0], detectedSide480[0], b_detecty_min[0], detectedSideB[0], break_SOLmode_t2))
                                                                                stdout.flush()
                                                                                mode = '4'
                                                                                dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                                                system("sudo killall -9 python")
                                                                elif (total_yaw_error < 4) and ((b_detecty_min[0] < 2) or (t_detect480_min[0] <= 10) or (t_detect0_min[0] <= 12)):
                                                                    if ((dist2travel - RightEncData[0]) > 500) and (RightEncData[0] > 300):
                                                                        break_mode_t1, break_SOLmode_t2 = time(), time() - break_SOLmode_t1
                                                                        if detectedSide480[0] or detectedSide0[0] or detectedSideB[0]:
                                                                            mode, ErrorCode[0] = '4', 120
                                                                            if break_SOLmode_t2 > break_SOLmode_t3:
                                                                                if detectedSide480[0]:
                                                                                    setYaw = setYaw + (detectedSide480[0] * 2)
                                                                                    setYaw = setYaw % 360
                                                                                    break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                elif detectedSide0[0]:
                                                                                    setYaw = setYaw + (detectedSide0[0] * 2)
                                                                                    setYaw = setYaw % 360
                                                                                    break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                elif detectedSideB[0]:
                                                                                    setYaw = setYaw + (detectedSideB[0] * 2)
                                                                                    setYaw = setYaw % 360
                                                                                    break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                else:
                                                                                    capture_click[0] = 1
                                                                                    stdout.write("\n[->]t_detect480_min[0]={}, detectedSide480[0]={}, b_detecty_min[0]={}, detectedSideB[0]={}, break_SOLmode_t2={},".format(t_detect480_min[0], detectedSide480[0], b_detecty_min[0], detectedSideB[0], break_SOLmode_t2))
                                                                                    stdout.write("\n[->]t_detect0_min[0]={}, detectedSide0[0]={}, total_yaw_error={}, RightEncData[0]={}, dist2travel={},".format(t_detect0_min[0], detectedSide0[0], total_yaw_error, RightEncData[0], dist2travel))
                                                                                    stdout.flush()
                                                                                    mode = '4'
                                                                                    dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                                                    system("sudo killall -9 python")
                                                                elif (pmode != '4') and (abs(setYaw - ATYaw[0]) < error_allow_yaw) and (total_yaw_error < 4) and ((b_detecty_min[0] < 3) or (t_detect0_min[0] <= 16)):
                                                                    if (((dist2travel - RightEncData[0]) > 700) and (RightEncData[0] > 500)):
                                                                        break_mode_t1, break_SOLmode_t2 = time(), time() - break_SOLmode_t1
                                                                        if detectedSide0[0] or detectedSideB[0]:
                                                                            mode, ErrorCode[0] = '5', 120
                                                                            if break_SOLmode_t2 > break_SOLmode_t3:
                                                                                if detectedSide0[0]:
                                                                                    setYaw = (setYaw + (detectedSide0[0] * 2))
                                                                                    setYaw = setYaw % 360
                                                                                    break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                elif detectedSideB[0]:
                                                                                    setYaw = (setYaw + (detectedSideB[0] * 2))
                                                                                    setYaw = setYaw % 360
                                                                                    break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                else:
                                                                                    capture_click[0] = 1
                                                                                    stdout.write("\n[->]pmode={}, b_detecty_min[0]={}, detectedSideB[0]={}, break_SOLmode_t2={},".format(pmode, b_detecty_min[0], detectedSideB[0], break_SOLmode_t2))
                                                                                    stdout.write("\n[->]t_detect0_min[0]={}, detectedSide0[0]={}, total_yaw_error={}, RightEncData[0]={}, dist2travel={},".format(t_detect0_min[0], detectedSide0[0], total_yaw_error, RightEncData[0], dist2travel))
                                                                                    stdout.flush()
                                                                                    mode = '4'
                                                                                    dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                                                    system("sudo killall -9 python")
                                                                elif break_mode_t2 > break_mode_t3: mode = '2'
                                                            else:
                                                                if t_detect480_min[0] <= 4:
                                                                    # travel_depthSol_0, break_mode_t1 = travel, time()
                                                                    break_mode_t1, break_SOLmode_t2 = time(), time() - break_SOLmode_t1
                                                                    if detectedSide480[0]:
                                                                        mode, ErrorCode[0] = '4', 120
                                                                        # if break_SOLmode_t2 > 1.5:  # and Estop_FB[0] == '0':
                                                                        if break_SOLmode_t2 > break_SOLmode_t3:
                                                                            if detectedSide480[0]:
                                                                                setYaw = (setYaw + detectedSide480[0])
                                                                                setYaw = setYaw % 360
                                                                                break_SOLmode_t1, depthSol_flag = time(), 1
                                                                                stdout.write("\n[->]motion()[->]else:480<4 main_t2={}, setYaw={}, temp_setYaw={}, detectedSideB={}, detectedSide0={}, detectedSide480={},".format(main_t2, setYaw, temp_setYaw, detectedSideB[0], detectedSide0[0], detectedSide480[0]))
                                                                                stdout.flush()
                                                                            else:
                                                                                stdout.write("\n[->]pmode={}, t_detect480_min[0]={}, detectedSide480[0]={}, break_SOLmode_t2={},".format(pmode, t_detect480_min[0], detectedSide480[0], break_SOLmode_t2))
                                                                                stdout.write("\n[->]total_yaw_error={}, RightEncData[0]={}, dist2travel={},".format(total_yaw_error, RightEncData[0], dist2travel))
                                                                                stdout.flush()
                                                                                mode = '4'
                                                                                dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                                                system("sudo killall -9 python")
                                                                elif break_mode_t2 > break_mode_t3: mode = '2'
                                                        else: ErrorCode[0], mode = 121, '5'

                                                        # depth detection error angle changes errorInAngle
                                                        errorInAngle = 20  # 20 16/11/22 12:47pm
                                                        if mode != '2' or pmode != '2':
                                                            if temp_setYaw > setYaw and (temp_setYaw - setYaw) > errorInAngle:
                                                                setYaw = temp_setYaw + errorInAngle
                                                                setYaw = setYaw % 360
                                                            elif setYaw > temp_setYaw and (setYaw - temp_setYaw) > errorInAngle:
                                                                setYaw = temp_setYaw + errorInAngle
                                                                setYaw = setYaw % 360
                                                            elif setYaw < 0:
                                                                if setYaw >= -360: setYaw = 360 + setYaw
                                                                stdout.write("\n\n\n[->]motion()[->]exception errorAngCont<0 temp_setYaw={}, setYaw={},".format(temp_setYaw, setYaw))
                                                                stdout.flush()
                                                        # depth error image capture  5/12/22 11:09am
                                                        # if (mode >= '4') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                            # capture_click[0] = 1
                                                        # elif capture_click[0] == 3:
                                                            # ImageClick_count += 1
                                                        if mode == '2' and pmode != mode:
                                                            travel_depthSol_0 = RightEncData[0]
                                                        if depthSol_flag and travel_depthSol_0:
                                                            if (abs(RightEncData[0] - travel_depthSol_0) > 500): # 500 -> v2_p2
                                                                setYaw = tTsetpoint
                                                                depthSol_flag, travel_depthSol_0 = 0, 0
                                                        # timer for controlling depth break and original angle after some sec.
                                                        if mode == '2': Not_mode2_counter1 = break_SOLmode_t1 = time()
                                                        else:
                                                            Not_mode2_counter2 = time() - Not_mode2_counter1
                                                            if Not_mode2_counter2 > 30:  # 16/11/22 5:36pm
                                                                Not_mode2_counter1, setYaw = time(), tTsetpoint
                                                                # stdout.write("\n[->]motion()[->] errorAngCont <0 Not_mode2_counter2={}, setYaw={}, tTsetpoint={},".format(Not_mode2_counter2, setYaw, tTsetpoint))
                                                        if FinalPath[FinalPathListCounter+1] in VirtualNodeList or FinalPath[FinalPathListCounter] in VirtualNodeList:
                                                            travel_virtual = round(abs(RightEncData[0] - travel_virtual_0), 2)
                                                            if travel_virtual >= dist2travel:
                                                                # stdout.write("\n[->]MotionSection==1[->] travel_virtual={}, travel={}, RightEncData[0]={}, dist2travel={},".format(travel_virtual, travel, RightEncData[0], dist2travel))
                                                                # stdout.flush()
                                                                MotionSection = 2
                                                            elif TagID[0] == FinalPath[FinalPathListCounter+1] and TagDetectionState[0]:
                                                                v_tagDetectionState_t2 = time() - v_tagDetectionState_t1
                                                                # stdout.write("\n[->]motion()[->]MotionSection=1[->] tagDetectionState_t2={}, MotionSection={},".format(tagDetectionState_t2, MotionSection))
                                                                if v_tagDetectionState_t2 > (0.5): MotionSection = 2
                                                            else: v_tagDetectionState_t1 = time()
                                                        # AprilTag Data following.
                                                        tagState = 0
                                                        # stdout.write("\n[->]motion()[->]1 MotionSection=1 TagID[0]={}, TagDetectionState[0]={}, CurrentTagID[0]={}, FinalPath={},".format(TagID[0], TagDetectionState[0], CurrentTagID[0], FinalPath))
                                                        # if ((not TagDetectionState[0]) or TagID[0] == FinalPath[FinalPathListCounter]) and not tagDetect_flag:
                                                        if tagDetect_flag == 0:
                                                            # and tagDetect_flag == 0:  # 10-11-22 11:29am
                                                            if ((TagDetectionState[0] == 0) or TagID[0] != FinalPath[FinalPathListCounter]): tagDetect_flag = 1
                                                            elif TagID[0] == FinalPath[FinalPathListCounter+1]: tagDetect_flag = 1
                                                            if TagID[0] == FinalPath[FinalPathListCounter] and TagDetectionState[0]: tagState = 1
                                                        # for smooth breaking after estop relise. 9/12/22 12:21pm
                                                        dist2travel = int(dist2travel)
                                                        #  change due to multiple tag detected.
                                                        if TagID[0] == FinalPath[FinalPathListCounter]:
                                                            dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                        else:
                                                            dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                            # stdout.write("\n[->]motion section 2[->] if TagID[0] == FinalPath[FinalPathListCounter]:else setYaw = {}".format(setYaw))
                                                            # stdout.flush()
                                                        pmode, tagDetectionState_t1 = mode, time()
                                                    else:  # 10-11-22 11:28am
                                                        total_travel_0 = RightEncData[0]
                                                        tagDetectionState_t1, tagDetect_flag = time(), 1  # 11/11/22 11:02am
                                                        dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'  # 10/11/22 11:57am
                                                        pass
                                                elif MotionSection == 2:
                                                    if TagID[0] in [HomeTagId, EndNode]: SmoothTurnEnable = 0
                                                    stdout.write("\n[->]motion()[->]MotionSection=2[->] SmoothTurnEnable={}, TagID[0]={}, HomeTagId={}, EndNode={}, total_travel={}, total_travel_0={},".format(SmoothTurnEnable, TagID[0], HomeTagId, EndNode, total_travel, total_travel_0))
                                                    stdout.flush()
                                                    # if (not fpath_straight_enable) and (not SmoothTurnEnable):
                                                    if FinalPath[FinalPathListCounter+1] in VirtualNodeList:
                                                        mode = '0'
                                                        dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATX[0] * 1000)) + ',' + str(setX) + ',' + str(dist2travel) + '\n'
                                                        break
                                                    elif not SmoothTurnEnable:
                                                        try:
                                                            if (FinalPath[FinalPathListCounter+1] not in VirtualNodeList):
                                                                # if (not PreviousSmoothTurnEnable):# and not path_straight_enable):
                                                                stdout.write("\n[->]motion()[->]MotionSection == 2[->] cnode={}, dnode={}, fnode={}, smoothExitX={}, smoothExitY={}, sp={}, fsp={}".format(FinalPath[FinalPathListCounter], FinalPath[FinalPathListCounter+1], FinalPath[FinalPathListCounter+2], smoothExitX, smoothExitY, tTsetpoint, futureTsetpoint))
                                                                stdout.flush()
                                                                if not narrow_path: AchieveLineFuncationFlag = 1
                                                                # stdout.write("\n\n\n\n\n\n\nbhai1")
                                                                # achieveLine()
                                                                try:
                                                                    # based on csv
                                                                    # to fetch TagID[0] Entery angle from column AprilTagYaw
                                                                    if TagID[0] == FinalPath[FinalPathListCounter+1]:
                                                                        if FinalPath[FinalPathListCounter+2] in VirtualNodeList:
                                                                            # fnode angle :-> fnode in virtual
                                                                            stdout.write("\n[->]achieveLine()[->]if TagID[0] == FinalPath[FinalPathListCounter+1]:elif FinalPath[FinalPathListCounter+2] in VirtualNodeList:")
                                                                            stdout.flush()
                                                                            Tsetpoint = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter+2])].iloc[0, 3])
                                                                            stdout.write("\n[->]achieveLine()[->]if FinalPath[FinalPathListCounter+2] in VirtualNodeList: Tsetpoint={},".format(Tsetpoint))
                                                                            stdout.flush()
                                                                        elif FinalPath[FinalPathListCounter+1] in VirtualNodeList:
                                                                            Tsetpoint = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+2]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter+1])].iloc[0, 4])
                                                                            stdout.write("\n[->]achieveLine()[->]elif FinalPath[FinalPathListCounter+1] in VirtualNodeList: Tsetpoint={},".format(Tsetpoint))
                                                                            stdout.flush()
                                                                        elif FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:
                                                                            # current tag exit angle
                                                                            if FinalPath[FinalPathListCounter+2] and OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1] not in VirtualNodeList:
                                                                                Tsetpoint = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 4])  # 28/11/22 11:36am
                                                                            elif FinalPath[FinalPathListCounter+2]:
                                                                                # stdout.write("\n[->]achieveLine()[->]if FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:if FinalPath[FinalPathListCounter+2]: OriginalFinalPath[-2]={}, OriginalFinalPath={},".format(OriginalFinalPath[-2], OriginalFinalPath))
                                                                                # stdout.flush()
                                                                                Tsetpoint = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])+1])].iloc[0, 4])  # 28/11/22 11:36am
                                                                            # current tag entry angle
                                                                            else:
                                                                                stdout.write("\n[->]achieveLine()[->]if FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:if FinalPath[FinalPathListCounter+2]:else: OriginalFinalPath.index(TagID[0])-1={}, OriginalFinalPath={},".format(OriginalFinalPath.index(TagID[0])-1, OriginalFinalPath))
                                                                                stdout.flush()
                                                                                Tsetpoint = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 4])
                                                                        elif FinalPath[FinalPathListCounter+2]:
                                                                            # dnode to fnode angle
                                                                            stdout.write("\n[->]achieveLine()[->]if TagID[0] == FinalPath[FinalPathListCounter+1]:elif FinalPath[FinalPathListCounter+2]:")
                                                                            stdout.flush()
                                                                            Tsetpoint = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == FinalPath[FinalPathListCounter+2])].iloc[0, 3])
                                                                        else:
                                                                            stdout.write("\n[->]achieveLine()[->]if FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:else: OriginalFinalPath.index(TagID[0])-1={}, OriginalFinalPath={},".format(OriginalFinalPath.index(TagID[0])-1, OriginalFinalPath))
                                                                            stdout.flush()
                                                                            Tsetpoint = int(dataAssignmentDF[(dataAssignmentDF.cnode == TagID[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 4])
                                                                    stdout.write("\n[->]achieveLine()[->] Tsetpoint={},".format(Tsetpoint))
                                                                    stdout.flush()
                                                                except Exception as e:
                                                                    stdout.write("\n\n\n[->]achieveLine()[->] cnode= exception={},".format(e))
                                                                    stdout.flush()
                                                                    pass
                                                                # for smoothY
                                                                stdout.write("\n[->]achieveLine()[->]if TagID[0]={}, == FinalPath[FinalPathListCounter+1]={},:".format(TagID[0], FinalPath[FinalPathListCounter+1]))
                                                                stdout.flush()
                                                                if TagID[0] == FinalPath[FinalPathListCounter+1]:
                                                                    if FinalPath[FinalPathListCounter+2] in VirtualNodeList:
                                                                        # fnode angle :-> fnode in virtual
                                                                        ValueAchieve = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter+2])].iloc[0, 10])
                                                                        stdout.write("\n[->]achieveLine()[->]if FinalPath[FinalPathListCounter+2] in VirtualNodeList: ValueAchieve={},".format(ValueAchieve))
                                                                        stdout.flush()
                                                                    elif FinalPath[FinalPathListCounter] in VirtualNodeList:
                                                                        stdout.write("\n[->]achieveLine()[->]elif FinalPath[FinalPathListCounter+1] in VirtualNodeList: FinalPath={}, FinalPathListCounter={},".format(FinalPath, FinalPathListCounter))
                                                                        stdout.flush()
                                                                        ValueAchieve = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter])].iloc[0, 10])
                                                                        stdout.write("\n[->]achieveLine()[->]elif FinalPath[FinalPathListCounter+1] in VirtualNodeList: ValueAchieve={},".format(ValueAchieve))
                                                                        stdout.flush()
                                                                    elif FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:
                                                                        # current tag exit angle
                                                                        if FinalPath[FinalPathListCounter+2] and OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1] not in VirtualNodeList:
                                                                            ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])  # 28/11/22 11:36am
                                                                        elif FinalPath[FinalPathListCounter+2]:
                                                                            ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])+1])].iloc[0, 10])  # 28/11/22 11:36am
                                                                        else:
                                                                            ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])
                                                                    else:
                                                                        stdout.write("\n[->]achieveLine()[->]if TagID[0] == FinalPath[FinalPathListCounter+1]:else:")
                                                                        stdout.flush()
                                                                        ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == TagID[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])

                                                                exact_distance = int(abs(ValueAchieve - (ATY[0]*1000)))  # - live_offset
                                                                stdout.write("\n[->]achieveLine()[->] int(abs(ATY[0])) ValueAchieve={}, ATX[0]*1000={}, ATY[0]*1000={}, exact_distance={},".format(ValueAchieve, ATX[0]*1000, ATY[0]*1000, exact_distance))
                                                                stdout.write("\n[->]MotionSection == 2[->] flagonce={}, ATYaw[0]={},".format(flagonce, ATYaw[0]))
                                                                stdout.write("\n[->]MotionSection == 2[->] FinalPath[FinalPathListCounter+1]={}, FinalPath[-2]={},".format(FinalPath[FinalPathListCounter+1], FinalPath[-2]))
                                                                stdout.flush()

                                                                if FinalPath[FinalPathListCounter+1] == FinalPath[-2]:
                                                                    try:
                                                                        LedData1[0] = '6\n'
                                                                        if LedData1[0] != prev_LedData1:
                                                                            # # Esp32_ser.write(LedData1[0].encode())
                                                                            stdout.write("\n[-->ESP32_T] LedData1[0]={},".format(LedData1[0]))
                                                                            stdout.flush()
                                                                            prev_LedData1 = LedData1[0]
                                                                    except Exception as e:
                                                                        stdout.write("\n\n\n[-->ESP32_T]ESP32 exception={},".format(e))
                                                                        stdout.flush()
                                                                        pass
                                                                    stdout.write("\n[->]MotionSection == 2[->] step1")
                                                                    stdout.flush()

                                                                    #  16/11/22 5:02pm
                                                                    error_allow_y, error_allow_yaw = 50, 2
                                                                    # if EndNode != HomeNode or EndNode != ChargingNode:
                                                                        # error_allow_y, error_allow_yaw = 50, 2
                                                                        # stdout.write("\n[->] TableFaceAngle normal main_t2={}, error_allow_y={}, error_allow_yaw={},".format(main_t2, error_allow_y, error_allow_yaw))
                                                                        # stdout.flush()
                                                                    # elif EndNode == ChargingNode or EndNode == HomeNode:
                                                                        # error_allow_y, error_allow_yaw = 10, 1
                                                                        # stdout.write("\n[->] TableFaceAngle charging main_t2={}, error_allow_y={}, error_allow_yaw={},".format(main_t2, error_allow_y, error_allow_yaw))
                                                                        # stdout.flush()
                                                                    flagonce, pmode, ImageClick_count = 0, '', 0
                                                                    break_mode_t1 = timeBreak_c1 = timeBreak_c2 = time()
                                                                    tagState = capture_click[0] = 0
                                                                    while flagonce != 3:
                                                                        if Estop_FB[0] == '1': ErrorCode[0] = 121
                                                                        if flagonce == 0:
                                                                            # setYaw, SetY = Tsetpoint, (ATY[0] * 1000)
                                                                            tagState = 1
                                                                            if TagDetectionState[0]: setYaw, SetY = round(Tsetpoint, 2), int(ATY[0] * 1000)
                                                                            else: setYaw, SetY = ATYaw[0], int(ATY[0] * 1000) - 100
                                                                        elif flagonce == 1: setYaw, SetY = ATYaw[0], ValueAchieve
                                                                        elif flagonce == 2: setYaw, SetY = Tsetpoint, ValueAchieve

                                                                        # conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                                        if dnode == TagID[0]:
                                                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                                                            tagState = 1
                                                                        else:
                                                                            conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)
                                                                            tagState = 0
                                                                        # if pmode != '0': mode = '1'
                                                                        try:
                                                                            if flagonce == 0:
                                                                                mode = '1'
                                                                                if conditionMatch: flagonce, conditionMatch = 1, 0
                                                                            elif flagonce == 1 and conditionMatch:
                                                                                flagonce, conditionMatch = 2, 0
                                                                            elif flagonce == 2 and conditionMatch:
                                                                                flagonce, conditionMatch = 3, 0
                                                                                if TagID[0] == HomeNode or TagID[0] == ChargingNode: pass
                                                                                elif type(DestinationNodeList) == list and type(DestinationNodeList[0]) == list:
                                                                                    if len(DestinationNodeList[0]):
                                                                                        if TagID[0] == DestinationNodeList[0][0]: mode = '0'
                                                                        except Exception as e:
                                                                            stdout.write("\n\n\n[-->mo]final flagonce={}, conditionMatch={}, exception={},".format(flagonce, conditionMatch, e))
                                                                            stdout.flush()
                                                                            pass

                                                                        # depth error image capture  5/12/22 10:57am
                                                                        if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                                            capture_click[0] = 1
                                                                        elif capture_click[0] == 3: ImageClick_count += 1
                                                                        break_mode_t2 = time() - break_mode_t1
                                                                        if t_detect480_min[0] < 2:
                                                                            mode = '7'
                                                                            if break_mode_t2 > 0.2:
                                                                                temp_ATY = int(ATY[0] * 1000) + 100
                                                                                while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                                                    mode = '1'
                                                                                    setYaw, SetY = ATYaw[0], temp_ATY
                                                                                    if TagID[0] == dnode:
                                                                                        dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                                                break_mode_t1 = time()
                                                                        elif mode == '1': break_mode_t1 = time()
                                                                        elif mode == '7': mode = '1'
                                                                        # 9/12/22 7:53 pm
                                                                        if flagonce == 1 and dnode != TagID[0] and TagDetectionState[0]:
                                                                            timeBreak_c2 = time() - timeBreak_c1
                                                                            if timeBreak_c2 > 2:
                                                                                stdout.write("\n\n\n[-->]elifnSmoot: flagonce={}, TagID[0]={}, conditionMatch={}, dnode={}, timeBreak_c2={}, break exception main_t2={},".format(flagonce, TagID[0], conditionMatch, dnode, timeBreak_c2, main_t2))
                                                                                stdout.flush()
                                                                                break
                                                                        else: timeBreak_c1 = time()
                                                                        pmode = mode

                                                                        # mode, TagDetectionState[0], ATYaw[0], setYaw, ATY[0], SetY
                                                                        if dnode == TagID[0]:
                                                                            dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(int(SetY)) + '\n'
                                                                        else:
                                                                            dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(int(SetY)) + '\n'
                                                                    flagonce = tagState = 0
                                                                    #  16/11/22 5:02pm
                                                                    error_allow_y, error_allow_yaw = 10, 1
                                                                # stdout.write("\n\n\n\n\n\n\nbhai2")
                                                                stdout.write("\n[->]MotionSection == 2[->] ValueAchieve={},".format(ValueAchieve))
                                                                stdout.write("\n[->]MotionSection == 2[->] ATX[0]={}, ATY[0]={}".format(ATX[0]*1000, ATY[0]*1000))
                                                                stdout.write("\n[->]MotionSection=2[->] Motion Complete time={}, main_t2={}, break".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), main_t2))
                                                                stdout.flush()
                                                                break
                                                            else: break
                                                        except Exception as e:
                                                            stdout.write("\n\n\n[->]MotionSection == 2[->] exception={},".format(e))
                                                            stdout.flush()
                                                            pass
                                                    elif SmoothTurnEnable:
                                                        # if SmoothTurnEnable:  # and (not path_straight_enable):
                                                        try:
                                                            # for smoothY
                                                            stdout.write("\n[->]smoothTurn()[->]if TagID[0]={}, == FinalPath[FinalPathListCounter+1]={},:".format(TagID[0], FinalPath[FinalPathListCounter+1]))
                                                            stdout.flush()
                                                            if TagID[0] == FinalPath[FinalPathListCounter+1]:
                                                                if FinalPath[FinalPathListCounter+2] in VirtualNodeList:
                                                                    # fnode angle :-> fnode in virtual
                                                                    ValueAchieve = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter+2])].iloc[0, 10])
                                                                    stdout.write("\n[->]smoothTurn()[->]if FinalPath[FinalPathListCounter+2] in VirtualNodeList: ValueAchieve={},".format(ValueAchieve))
                                                                    stdout.flush()
                                                                elif FinalPath[FinalPathListCounter] in VirtualNodeList:
                                                                    stdout.write("\n[->]smoothTurn()[->]elif FinalPath[FinalPathListCounter+1] in VirtualNodeList: FinalPath={}, FinalPathListCounter={},".format(FinalPath, FinalPathListCounter))
                                                                    stdout.flush()
                                                                    ValueAchieve = int(virtualNodeDF[(virtualNodeDF.RealNode == FinalPath[FinalPathListCounter+1]) & (virtualNodeDF.VirtualNode == FinalPath[FinalPathListCounter])].iloc[0, 10])
                                                                    stdout.write("\n[->]smoothTurn()[->]elif FinalPath[FinalPathListCounter+1] in VirtualNodeList: ValueAchieve={},".format(ValueAchieve))
                                                                    stdout.flush()
                                                                elif FinalPath[FinalPathListCounter+1] != OriginalFinalPath[-2]:
                                                                    # current tag exit angle
                                                                    if FinalPath[FinalPathListCounter+2] and OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1] not in VirtualNodeList:
                                                                        ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])  # 28/11/22 11:36 am
                                                                    elif FinalPath[FinalPathListCounter+2]:
                                                                        ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])+1])].iloc[0, 10])  # 28/11/22 11:36 am
                                                                    else:
                                                                        ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == FinalPath[FinalPathListCounter+1]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])
                                                                else:
                                                                    stdout.write("\n[->]smoothTurn()[->]if TagID[0] == FinalPath[FinalPathListCounter+1]:else:")
                                                                    stdout.flush()
                                                                    ValueAchieve = int(dataAssignmentDF[(dataAssignmentDF.cnode == TagID[0]) & (dataAssignmentDF.dnode == OriginalFinalPath[OriginalFinalPath.index(TagID[0])-1])].iloc[0, 10])
                                                            stdout.write("\n[->]smoothTurn()[->]0 ValueAchieve={},: ATY={}, ATX={},".format(ValueAchieve, (ATY[0] * 1000), (ATX[0] * 1000)))
                                                            stdout.flush()
                                                            if ValueAchieve > 0 and (ValueAchieve + 100) <= 520:
                                                                stdout.write("\n[->]smoothTurn()[->]2 ValueAchieve={},:".format(ValueAchieve))
                                                                stdout.flush()
                                                                if (ValueAchieve + 100) >= 520: ValueAchieve = 500
                                                                else: ValueAchieve = ValueAchieve + 100
                                                            else: ValueAchieve = ValueAchieve + 100

                                                            stdout.write("\n[->]smoothTurn()[->]1 ValueAchieve={},: ATY={}, ATX={},".format(ValueAchieve, (ATY[0] * 1000), (ATX[0] * 1000)))
                                                            stdout.flush()
                                                            # use only for ValueAchieve
                                                            flagonce, break_mode_t1, pmode, ImageClick_count = 0, time(), '', 0
                                                            timeBreak_c1 = timeBreak_c2 = time()
                                                            flagonce = tagState = capture_click[0] = 0
                                                            while flagonce != 2:
                                                                if Estop_FB[0] == '1': ErrorCode[0] = 121
                                                                if flagonce == 0:
                                                                    setYaw, SetY = ATYaw[0], ValueAchieve
                                                                if TagID[0] == FinalPath[FinalPathListCounter+1]: tagState = 1
                                                                else: tagState = 0

                                                                if (ATY[0]*1000) > 0 and ValueAchieve > (ATY[0]*1000):
                                                                    flagonce, conditionMatch = 2, 0
                                                                    stdout.write("\n[->]smoothTurn()[->]if ATY[0]={}, ValueAchieve={},".format(ATY[0] * 1000, ValueAchieve))
                                                                    stdout.flush()

                                                                conditionMatch = abs(SetY-(ATY[0]*1000)) < error_allow_y and abs(setYaw - ATYaw[0]) < error_allow_yaw
                                                                mode = '1'
                                                                if flagonce == 0:
                                                                    mode = '1'
                                                                    if conditionMatch:
                                                                        stdout.write("\n[->]smoothTurn()[->] while: SetY={}, ATY[0]={}, setYaw={}, ATYaw[0]={},".format(SetY, ATY[0]*1000, setYaw, ATYaw[0]))
                                                                        stdout.flush()
                                                                        flagonce, conditionMatch = 1, 0
                                                                elif flagonce == 1:
                                                                    flagonce, conditionMatch = 2, 0

                                                                # depth error image capture  5/12/22 10:57am
                                                                if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                                    capture_click[0] = 1
                                                                elif capture_click[0] == 3:
                                                                    ImageClick_count += 1
                                                                    # capture_click[0] = 0
                                                                break_mode_t2 = time() - break_mode_t1
                                                                if t_detect480_min[0] < 2:
                                                                    mode = '7'
                                                                    if break_mode_t2 > 0.2:
                                                                        temp_ATY = int(ATY[0] * 1000) + 100
                                                                        while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                                            mode = '1'
                                                                            setYaw, SetY = ATYaw[0], temp_ATY
                                                                            if TagID[0] == FinalPath[FinalPathListCounter+1]:
                                                                                dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                                        break_mode_t1 = time()
                                                                elif mode == '1': break_mode_t1 = time()
                                                                elif mode == '7': mode = '1'
                                                                pmode = mode
                                                                # mode, TagDetectionState[0], ATYaw[0], setYaw, ATY[0], SetY
                                                                setYaw = round(setYaw, 2)
                                                                SetY = round(SetY, 2)
                                                                if TagID[0] == FinalPath[FinalPathListCounter+1]:
                                                                    dataToTeensy[0] = str(mode) + ',' + str(tagState) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                            flagonce = smooth_turn_section = 0
                                                            # 200mm
                                                            smooth_turn_loop_flag = TurnDirection = 1
                                                            exitAng = (int(ATYaw[0]) - 180) % 360
                                                            # stdout.write("\n[->]smoothTurn()[->]TurnDirection=1 int(ATYaw[0])={}, exitAng={}, futureTsetpoint={},".format(int(ATYaw[0]), exitAng, futureTsetpoint))
                                                            # stdout.write("\n[->]smoothTurn()[->] 1 TurnDirection=1 int(ATYaw[0])={},".format(int(ATYaw[0])))
                                                            # stdout.flush()
                                                            # 0 to 90, 90 to 180
                                                            if ATYaw[0] < futureTsetpoint and exitAng > futureTsetpoint: TurnDirection = 1
                                                            elif ATYaw[0] < futureTsetpoint and exitAng < futureTsetpoint:
                                                                # 0 to 270
                                                                if ATYaw[0] < 180: TurnDirection = -1
                                                                # 180 to 270
                                                                else: TurnDirection = 1
                                                            elif ATYaw[0] > futureTsetpoint and exitAng > futureTsetpoint:
                                                                # 90 to 0
                                                                if ATYaw[0] < 180: TurnDirection = -1
                                                                # 270 to 0
                                                                else: TurnDirection = 1
                                                            elif ATYaw[0] > futureTsetpoint and exitAng < futureTsetpoint:
                                                                TurnDirection = -1
                                                            # stdout.write("\n[->]smoothTurn()[->] 1 TurnDirection={}, int(ATYaw[0])={}, futureTsetpoint={}, exitAng={},".format(TurnDirection, int(ATYaw[0]), futureTsetpoint, exitAng))
                                                            # stdout.flush()
                                                            startTurnAngle = int(ATYaw[0])
                                                            breakAngle = (startTurnAngle - futureTsetpoint) % 360
                                                            # stdout.write("\n[->]smoothTurn()[->] 1 breakAngle={},".format(breakAngle))
                                                            # stdout.flush()
                                                            if breakAngle > 180: breakAngle -= 180
                                                            stdout.write("\n[->]smoothTurn()[->] 2 breakAngle={},".format(breakAngle))
                                                            stdout.flush()
                                                            breakAngle = int(breakAngle/2)
                                                            # stdout.write("\n[->]smoothTurn()[->] 3 breakAngle/2={},".format(breakAngle))
                                                            # stdout.write("\n[->]smoothTurn()[->] 2 TurnDirection={}, startTurnAngle={}, exitAng={}, futureTsetpoint={},".format(TurnDirection, startTurnAngle, exitAng, futureTsetpoint))
                                                            # stdout.write("\n[->]smoothTurn()[->] while smooth_turn_loop_flag: start... {}".format(date().strftime("%d-%m-%Y_%H:%M:%S:%f")))
                                                            # stdout.flush()
                                                            radius = 140
                                                            x_In_limit = ATX[0] * 1000
                                                            if x_In_limit > 300: x_In_limit = 300
                                                            elif x_In_limit < -300: x_In_limit = -300
                                                            if TurnDirection == 1:
                                                                new_radius = int(radius + ((x_In_limit/300)*radius))
                                                                LedData1[0] = '5\n'
                                                            else:
                                                                new_radius = int(radius - ((x_In_limit/300)*radius))
                                                                LedData1[0] = '4\n'

                                                            if LedData1[0] != prev_LedData1:
                                                                # ESP32_Ser.write(LedData1[0].encode())
                                                                stdout.write("\n[-->ESP32_T] LedData1[0]={},".format(LedData1[0]))
                                                                stdout.flush()
                                                                prev_LedData1 = LedData1[0]
                                                            stdout.write("\n[->]smoothTurn()[->] x_In_limit={}, TurnDirection={}, new_radius={},".format(x_In_limit, TurnDirection, new_radius))
                                                            stdout.flush()
                                                            futureTsetpoint = int(futureTsetpoint * 10)
                                                            break_mode_t1, pmode,  = time(), ''
                                                            flagonce = ImageClick_count = capture_click[0] = 0
                                                            while smooth_turn_loop_flag:
                                                                try:
                                                                    if smooth_turn_section == 0:
                                                                        # stdout.write("\n[->]smoothTurn()[->] elif smooth_turn_section == 3: TagDetectionState[0]={},".format(TagDetectionState[0]))
                                                                        if TagDetectionState[0]: live_angle_value = int(ATYaw[0] * 10)
                                                                        else: live_angle_value = int(BmiYAW[0])

                                                                        # if abs(startTurnAngle - live_angle_value) >= breakAngle:
                                                                        if abs(live_angle_value - futureTsetpoint) < 200: smooth_turn_loop_flag = 0
                                                                        else:
                                                                            mode = '3'
                                                                            if t_detect480_min[0] < 2:
                                                                                mode = '7'
                                                                                ErrorCode[0] = 120
                                                                                break_mode_t1 = time()
                                                                                # stdout.write("\n[->]main()[->] 2404 flagonce={}, b_detecty_min[0]={}, t_detect480_min[0]={}, type(t_detect480_min[0])={},".format(flagonce, b_detecty_min[0], t_detect480_min[0], type(t_detect480_min[0])))
                                                                            break_mode_t2 = time() - break_mode_t1
                                                                            if mode == '3' and break_mode_t2 > 0.3: mode = '3'
                                                                            elif pmode == '7': mode = '7'

                                                                            # depth error image capture  5/12/22 10:57am
                                                                            if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                                                capture_click[0] = 1
                                                                            elif capture_click[0] == 3:
                                                                                ImageClick_count += 1
                                                                                # capture_click[0] = 0
                                                                            pmode = mode
                                                                            dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(live_angle_value) + ',' + str(futureTsetpoint) + ',' + str(new_radius) + ',' + str(TurnDirection) + '\n'
                                                                except Exception as e:
                                                                    stdout.write("\n\n\n[->]smoothTurn()[->] while smooth_turn_loop_flag: exception={},".format(e))
                                                                    stdout.flush()
                                                                    pass
                                                            # stdout.write("\n[->]smoothTurn()[->] PathEndSpeed={},".format(PathEndSpeed))
                                                            # stdout.write("\n[->]smoothTurn()[->]* Applied motion TurnSpeed=0,")
                                                            # stdout.flush()
                                                            # SmoothTurnProcess = 0
                                                            MotionSection = 0
                                                            # SmoothTurnProcessStartFlag = 0
                                                        except Exception as e:
                                                            stdout.write("\n\n\n[->]travelFunction()[->] smoothTurn() / exception={},".format(e))
                                                            stdout.flush()
                                                            pass
                                                        break
                                                    stdout.write("\n[->]motion()[->]MotionSection=2 End... {}".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                                                    stdout.flush()
                                                # try:
                                                    # if TagDetectionState[0] and Estop_FB[0] == '0':  # or playpause[0]:
                                                        # if TagID[0] != FinalPath[FinalPathListCounter] and TagID[0] != FinalPath[FinalPathListCounter+1]:
                                                            # ReschedulePathCase2 = TagID[0]
                                                            # stdout.write("\n[->]motion()[->]MotionSection ReschedulePathCase2={},".format(ReschedulePathCase2))
                                                            # stdout.flush()
                                                            # break
                                                # except Exception as e:
                                                    # stdout.write("\n\n\n[->]motion()[->] exception={},".format(e)); stdout.flush()
                                                    # pass
                                            except Exception as e:
                                                stdout.write("\n\n\n[->]motion()[->] exception={}".format(e))
                                                stdout.flush()
                                                break
                                except Exception as e:
                                    stdout.write("\n\n\n[->]travelFunction()[->] motion() / exception={}".format(e))
                                    stdout.flush()
                                    pass
                                # not enable right now from 22/2/2022
                                stdout.write("\n[->]travelFunction()[->] SmoothTurnEnable={}, TagID[0]={}, TagID[0] not in [HomeTagId, EndNode]={},".format(SmoothTurnEnable, TagID[0], TagID[0] not in [HomeTagId, EndNode]))
                                stdout.flush()
                                if ReschedulePathCase2:
                                    if ReschedulePathCase2 != EndNode:
                                        FinalPath *= 0
                                        FinalPath.extend(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == ReschedulePathCase2) & (PathAssignmentDF.dn == EndNode)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                                        FinalPath.append(0)
                                        total_travel += total_travel_0
                                        stdout.write("\n[->]travelFunction()[->] ReschedulePathCase2(={},) != EndNode(={},) total_travel={}, total_travel_0={},".format(ReschedulePathCase2, EndNode, total_travel, total_travel_0))
                                        stdout.flush()
                                        total_travel_0 = 0
                                        FinalPathListCounter, ReschedulePathCase2 = 0, 0
                                        PreviousSmoothTurnEnable, SmoothTurnEnable = 0, 0
                                else:
                                    total_travel += total_travel_0
                                    stdout.write("\n[->]travelFunction()[->]0 FinalPathListCounter={}, total_travel={}, total_travel_0={},".format(FinalPathListCounter, total_travel, total_travel_0))
                                    stdout.flush()
                                    FinalPathListCounter += 1
                                    PreviousSmoothTurnEnable = SmoothTurnEnable
                                    total_travel_0 = 0
                                    stdout.write("\n[->]travelFunction()[->]1 FinalPathListCounter={}, total_travel={}, total_travel_0={},".format(FinalPathListCounter, total_travel, total_travel_0))
                                    stdout.flush()
                            if not FinalPath[1]:
                                total_travel += total_travel_0
                                stdout.write("\n[->]travelFunction()[->]0FinalPath={}, DestinationNode={}, total_travel={}, total_travel_0={},".format(FinalPath, DestinationNode, total_travel, total_travel_0))
                                stdout.flush()
                                total_travel_0 = 0
                                dnode = DestinationNode = FinalPath[0]
                                stdout.write("\n[->]travelFunction()[->]1FinalPath={}, dnode=DestinationNode={},".format(FinalPath, DestinationNode))
                                stdout.flush()
                            # TableFaceAngle
                            stdout.write("\n[->]travelFunction()[->] TableFaceAngle DestinationNode={},".format(DestinationNode))
                            stdout.flush()
                            if DestinationNode in VirtualNodeList:
                                CurrentTagID[0] = DestinationNode
                                dataToTeensy[0] = str(0) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                            else:
                                try:
                                    AtTableRobotFace = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == EndNode)].iloc[0, 2])  # 16/11/22 8:08pm
                                    setXtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 4])
                                    setYtableFaceAngleDF = int(tableFaceAngleDF[(tableFaceAngleDF.Tag == TagID[0])].iloc[0, 3])

                                    # error_allow_y, error_allow_yaw = 50, 3
                                    error_allow_y, error_allow_yaw = 10, 2
                                    # if EndNode != ChargingNode:
                                        # error_allow_y, error_allow_yaw = 10, 2
                                        # stdout.write("\n[->] TableFaceAngle normal error_allow_y={}, error_allow_yaw={},".format(error_allow_y, error_allow_yaw))
                                        # stdout.flush()
                                    # else:
                                        # error_allow_y, error_allow_yaw = 2, 0.5
                                        # stdout.write("\n[->] TableFaceAngle charging error_allow_y={}, error_allow_yaw={},".format(error_allow_y, error_allow_yaw))
                                        # stdout.flush()
                                    # temp_TagID = TagID[0]
                                    pmode = ''
                                    break_mode_t1 = timeBreak_c1 = timeBreak_c2 = time()
                                    flagonce = tagState = ImageClick_count = capture_click[0] = 0
                                    try:
                                        while flagonce != 3:
                                            if TagID[0] == HomeNode:
                                                mode = '0'
                                                break
                                            # tagState = TagDetectionState[0] if flagonce != 1 else 1
                                            if flagonce == 0:
                                                if TagDetectionState[0]: setYaw, SetY = AtTableRobotFace, ATY[0]*1000
                                                else: setYaw, SetY = AtTableRobotFace, setYtableFaceAngleDF
                                            elif flagonce == 1:
                                                setYaw, SetY = ATYaw[0], setYtableFaceAngleDF
                                                    # stdout.write("\n[->]main()[->]MotionSection==2: elif flagonce == 1:else: flagonce = {}, setx_temp = {}, sety_temp = {}, sety = {}, ATY[0] = {}, TagDetectionState[0] = {};".format(flagonce, setx_temp, sety_temp, SetY, round(ATY[0] * 1000, 2), TagDetectionState[0]))
                                                    # stdout.flush()
                                                if setYtableFaceAngleDF > ATY[0] * 1000 and TagID[0] != HomeNode:
                                                    flagonce = 2
                                                    stdout.write("\n[->]MotionSection==2 if setYtableFaceAngleDF > ATY[0]*1000: setYtableFaceAngleDF={}, ATY[0]={}, ".format(setYtableFaceAngleDF, ATY[0] * 1000))
                                                    stdout.flush()
                                            elif flagonce == 2: setYaw, SetY = AtTableRobotFace, ATY[0] * 1000
                                            # conditionMatch = abs(SetY-(ATY[0]*1000)) < error_allow_y and abs(setYaw - ATYaw[0]) < error_allow_yaw
                                            if TagDetectionState[0] and dnode == TagID[0]:
                                                conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - ATYaw[0]) < error_allow_yaw)
                                            else:
                                                conditionMatch = (abs(SetY - (ATY[0] * 1000)) < error_allow_y) and (abs(setYaw - (float(BmiYAW[0]) * 0.1)) < error_allow_yaw)

                                            mode = '1'
                                            if flagonce == 0:
                                                mode = '1'
                                                if conditionMatch: flagonce, conditionMatch = 1, 0
                                            elif conditionMatch: 
                                                if flagonce == 1: flagonce, conditionMatch, mode = 2, 0, '1'
                                                elif flagonce == 2: flagonce, conditionMatch, mode = 3, 0, '0'

                                            # depth error image capture  5/12/22 10:57am
                                            # if (mode == '7') and (mode != pmode) and (capture_click[0] == 0) and (ImageClickLimit >= ImageClick_count):
                                                # capture_click[0] = 1
                                            # elif capture_click[0] == 3:
                                                # ImageClick_count += 1
                                                # # capture_click[0] = 0
                                            break_mode_t2 = time() - break_mode_t1
                                            if t_detect480_min[0] < 2:
                                                mode = '7'
                                                if TagID[0] != HomeNode:
                                                    # ErrorCode[0] = 120
                                                    dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                    stdout.write("\n[->]MotionSection==2 [->]except Serving End due to depth.")
                                                    stdout.flush()
                                                    break
                                                if break_mode_t2 > 0.2:
                                                    temp_ATY = int(ATY[0] * 1000) + 100
                                                    while t_detect480_min[0] < 2 and temp_ATY <= 150:
                                                        mode = '1'
                                                        setYaw, SetY = ATYaw[0], temp_ATY
                                                        if TagID[0] == dnode:
                                                            dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                    break_mode_t1 = time()
                                            elif mode == '1': break_mode_t1 = time()
                                            elif mode == '7': mode = '1'
                                            pmode = mode
                                            # 9/12/22 7:53 pm
                                            if flagonce == 1 and TagDetectionState[0] and dnode != TagID[0]:
                                                timeBreak_c2 = time() - timeBreak_c1
                                                if timeBreak_c2 > 2:
                                                    stdout.write("\n\n\n[-->]eif not Smoot: flagonce={}, TagID[0]={}, conditionMatch={}, dnode={}, timeBreak_c2={}, break exception main_t2={},".format(flagonce, TagID[0], conditionMatch, dnode, timeBreak_c2, main_t2))
                                                    stdout.flush()
                                                    break
                                            else: timeBreak_c1 = time()

                                            SetY = round(SetY, 2)
                                            if TagDetectionState[0] and dnode == TagID[0]:
                                                dataToTeensy[0] = str(mode) + ',' + str(TagDetectionState[0]) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                # stdout.write("\n[->]main()[->]MotionSection==2:SetY: if flagonce == 0: flagonce = {}, TagDetectionState = {};".format(flagonce, TagDetectionState[0]))
                                                # stdout.flush()
                                            else:
                                                dataToTeensy[0] = str(mode) + ',' + str(0) + ',' + str(int(ATYaw[0] * 10)) + ',' + str(int(setYaw * 10)) + ',' + str(int(ATY[0] * 1000)) + ',' + str(SetY) + '\n'
                                                # stdout.write("\n[->]main()[->]MotionSection==2:SetY: if flagonce == 0: flagonce = {}, TagDetectionState = {};".format(flagonce, TagDetectionState[0]))
                                                # stdout.flush()
                                    except Exception as e:
                                        stdout.write("\n\n\n[->]travelFunction()[->] while flagonce != 3: exception={},".format(e))
                                        stdout.flush()
                                        pass
                                    flagonce = 0
                                    error_allow_y, error_allow_yaw = 10, 1
                                except Exception as e:
                                    stdout.write("\n\n\n[->]travelFunction()[->] while True / Initium = 1 /aprilTagReset / exception={},".format(e))
                                    stdout.flush()
                                    pass
                                stdout.write("\n[->]travelFunction()[->] 1 while LiveDriveSpeed and ESTOP[0]: CurrentTagID[0]={}, TagID[0]={},".format(CurrentTagID[0], TagID[0]))
                                stdout.flush()
                                CurrentTagID[0] = TagID[0]
                                stdout.write("\n[->]travelFunction()[->] 2 while LiveDriveSpeed and ESTOP[0]: CurrentTagID[0]={}, TagID[0]={},".format(CurrentTagID[0], TagID[0]))
                                stdout.flush()
                            # Order serving case = 2
                            FoodServingCase[0] = 2
                            # total_travel = total_travel_0
                            Serving_Volume = 1000  # 16/11/22 12:50pm
                            # Wait On Table Speak
                            if EndNode != HomeNode and ChargingFlag[0] != 1:
                                waitOntableTime, Wait_On_Table_Delta = time(), 0
                                DestinationDataDictionary = dict(zip(Real_nodeList[0], DestinationTrayList[0]))
                                CurrentTrayId[0] = DestinationDataDictionary[CurrentTagID[0]]
                                stdout.write("\n[->]travelFunction()[->]if CurrentTrayId[0]={},".format(CurrentTrayId[0]))
                                stdout.flush()
                                if CurrentTrayId[0] == 1:
                                    LedData1[0] = '1,1\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    # ArrowSerial.write(ArrowInput_Tray1.encode())
                                    if len(OrderServe_Tray1_VoiceFile) == 2:  # upper tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray1_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray1_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 1: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                elif CurrentTrayId[0] == 2:
                                    LedData1[0] = '1,2\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if len(OrderServe_Tray2_VoiceFile) == 2:  # lower tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray2_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray2_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                elif CurrentTrayId[0] == 3:
                                    LedData1[0] = '1,3\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if len(OrderServe_Tray3_VoiceFile) == 2:  # lower tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray3_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray3_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                elif CurrentTrayId[0] == 12:
                                    LedData1[0] = '1,4\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if len(OrderServe_Tray12_VoiceFile) == 2:  # lower tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray12_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray12_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                elif CurrentTrayId[0] == 23:
                                    LedData1[0] = '1,5\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if len(OrderServe_Tray23_VoiceFile) == 2:  # lower tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray23_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray23_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                elif CurrentTrayId[0] == 13:
                                    LedData1[0] = '1,6\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if len(OrderServe_Tray13_VoiceFile) == 2:  # lower tray
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray13_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Tray13_VoiceFile[0]))
                                            stdout.write("\n\n\n[->]travelFunction()[->]if CurrentTrayId[0] == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                else:  # 9/12/22 1;22pm
                                    LedData1[0] = '1,7\n'
                                    if LedData1[0] != prev_LedData1:
                                        # Esp32_ser.write(LedData1[0].encode())
                                        stdout.write("\n[-->ESP32_T]Wait On Table LedData1[0]={},".format(LedData1[0]))
                                        stdout.flush()
                                        prev_LedData1 = LedData1[0]
                                    if CurrentTrayId[0] == 123:
                                        # general tray
                                        if len(OrderServe_General_VoiceFile) == 2:
                                            try:
                                                system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_General_VoiceFile[0]))
                                            except Exception as e:
                                                system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_General_VoiceFile[0]))
                                                stdout.write("\n\n\n[->]travelFunction()[->]elif CurrentTrayId[0] 123 exception={},".format(e))
                                                stdout.flush()
                                                pass
                                    elif CurrentTrayId[0] == 111:
                                        # # # general tray
                                        if len(OrderServe_Water_VoiceFile) == 2:
                                            # system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(50, VoiceFilePath, OrderServe_Water_VoiceFile[0]))
                                            try:
                                                system("sudo mpg321 -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Water_VoiceFile[0]))
                                            except Exception as e:
                                                system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(Serving_Volume, VoiceFilePath, OrderServe_Water_VoiceFile[0]))
                                                stdout.write("\n\n\n[->]travelFunction()[->]elif CurrentTrayId[0] 111 exception={},".format(e))
                                                stdout.flush()
                                                pass
                                serving_t2 = time() - serving_t1
                                OrderServeCounter += 1
                                OrderServeFile = open("OrderServeDataFile.csv", "a")
                                OrderServeFile.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(OrderServeCounter, float(TagIdDF[(TagIdDF.TagId == CurrentTagID[0])]["TableNumber"]), CurrentTagID[0], is_on[0], CurrentTrayId[0], total_travel, VoltageValue[0], CurrentValue[0], serving_t3, date().strftime("%d-%m-%Y %H:%M:%S:%f"), serving_t2))
                                OrderServeFile.flush()
                                OrderServeFile.close()
                                # next input data
                                stdout.write("\n[->]0 After Wait On Table start={}, serving_t1={}, serving_t2={}, main_t2={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), serving_t1, serving_t2, main_t2))
                                stdout.flush()
                                WaitingForInput[0] = 1
                                stdout.write("\n[->]1 After Wait On Table start={}, main_t2={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f"), main_t2))
                                stdout.flush()
                                total_travel = total_travel_0 = 0

                                WOT_TimeOutLimit = 10
                                while Wait_On_Table_Delta < WOT_TimeOutLimit:
                                    Wait_On_Table_Delta = time() - waitOntableTime
                                    if (WOT_TimeOutLimit > 3) and (Wait_On_Table_Delta >= WOT_TimeOutLimit-3):
                                        LedData1[0] = '6\n'  # 16/11/22 1:24pm
                                        if LedData1[0] != prev_LedData1:
                                            # Esp32_ser.write(LedData1[0].encode())
                                            stdout.write("\n[-->ESP32_T]After Wait On Table LedData1[0]={},".format(LedData1[0]))
                                            stdout.flush()
                                            prev_LedData1 = LedData1[0]
                                        pass
                            FinalPathListCounter = 0
                            # ServingFlag[0] = 0
                            # stdout.write("\n[->]After Wait On Table ServingFlag[0]={}, main_t2={},".format(ServingFlag[0], main_t2))

                            stdout.write("\n[->]After Wait On Table main_t2={},".format(main_t2))
                            stdout.flush()
                            FinalDestinationNodeListCounter += 1
                            serving_t1, serving_t3 = time(), date().strftime("%d-%m-%Y %H:%M:%S:%f")
                            # # Order serving case = 0
                            # FoodServingCase[0] = 0
                        FinalDestinationNodeListCounter = 0
                    except Exception as e:
                        stdout.write("\n\n\n[->]travelFunction() exception={},".format(e))
                        stdout.flush()
                        pass
                    stdout.write("\n[->]1 travelFunction() FinalPath={}, End... ={},".format(FinalPath, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    stdout.flush()
                    FinalPath *= 0
                    stdout.write("\n[->]2 travelFunction() FinalPath={}, main_t2={}, End... ={},\n\n\n".format(FinalPath, main_t2, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    stdout.flush()


def main_step1():
    """
    To verify thread works or not.

    Returns
    -------
    None.

    """
    try:
        main()
    except Exception as e:
        stdout.write("\n\n\n[->] main() exception={},".format(e))
        stdout.flush()
        pass


if __name__ == "__main__":
    try:
        main_step1()
    except Exception as e:
        stdout.write("\n\n\n[->] if __name__ == \"__main__\": exception={},".format(e))
        stdout.flush()
        pass

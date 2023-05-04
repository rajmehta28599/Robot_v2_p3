'''
Created on Fri Apr  8 14:49:19 2022

@author: Raj-Mehta 
@email : rajmehta28599v@gmail.com
'''

from sys import stdout
from threading import Thread
from time import sleep, time
from multiprocessing import Process, Value
from datetime import datetime
from AprilTagDetectionFunction_V17 import apriltagDetectionFunction
date = datetime.now

test_mode, print_flag = 0, 0
BluetoothPresent, ImageFeedbackFlag = 0, 0
test_mode_log_file, fix_pitch_roll = 0, 1
LightCorrectionAlgo, NewProcessCreateFlag = 0, 1
FrameSize = 1  # 1: 640x480 , 2: 1280x720 , 3: 1920x1080

EstopPressFeedbackFlag_ArcThread, AtDataErrorFlag, AtCameraConnectionErrorFlag = [0], [0], [0]
ATX, ATY, ATZ, ATYaw, ATP, ATR = [0], [0], [0], [0], [0], [0]
TagID, TagDetectionState = [0], [0]
ATDetection_CurrentTag, ATDetection_DestinationTag = [0], [0]
MetaDataAssignmentFlag, InputProcessEnable = [0], [0]
IgnoreTagList_AtThread, EstopPressFeedbackFlag = [0], [0]
AprilTagLogFlag, ImageCapture, previous_ImageCapture = 0, 0, 0
reading_update_fps, thread_fps = 0, 0
t_detect0_min, t_detect480_min = [0], [0]
b_detecty_min, ApriltagModuleFlag = [0], [0]
capture_click, UISwitchFlag = [0], [11]
detectedSide480, detectedSide0, detectedSideB = [0], [0], [0]


def mainFunction():
    stdout.write("\n[-->ADHT]mainFunction()[->] Main function start")
    stdout.flush()
    global ATX, ATY, ATZ, ATYaw, TagID, TagDetectionState
    global ImageCapture, previous_ImageCapture, ATDetection_CurrentTag
    global IgnoreTagList_AtThread, ATDetection_DestinationTag
    global NewProcessCreateFlag, ApriltagModuleFlag
    global mp_fps, thread_fps, reading_update_fps, UISwitchFlag

    t11 = time()
    while True:
        # thread_fps = (1 / (time() - t11))
        t22 = time() - t11
        if t22 > (0.033): # 0.033
            t11 = time()
            # stdout.write("\n[-->ADHT] thread_fps={},".format(t22))
            try:
                if NewProcessCreateFlag:

                    ApriltagModuleFlag[0] = 1

                    ApriltagModule_flag = Value('i',3)
                    # stdout.write("\n[-->ADHT] IgnoreTagList_AtThread={}, -> DataErrorFlag NewProcessCreateFlag={}, ApriltagModule_flag={}, ... {}".format(IgnoreTagList_AtThread, NewProcessCreateFlag, ApriltagModule_flag, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    CameraConnectionErrorFlag = Value('i',0); Input_Process_EnableFlag = Value('i',0)
                    # stdout.write("\n[-->ADHT] Input_Process_EnableFlag={},".format(Input_Process_EnableFlag))
                    AT_Yaw = Value('f', 0.0)
                    # stdout.write("\n[-->ADHT] AT_Yaw={}, AT_P={}, AT_R={},".format(AT_Yaw, AT_P, AT_R))
                    AT_X, AT_Y, AT_Z = Value('f', 0.0), Value('f', 0.0), Value('f', 0.0)
                    # stdout.write("\n[-->ADHT] AT_X={}, AT_Y={}, AT_Z={},".format(AT_X, AT_Y, AT_Z))
                    Tag_ID, TagDetection_State = Value('i', 0), Value('i', 0)
                    # stdout.write("\n[-->ADHT] Tag_ID={}, TagDetection_State={}, Fps={},".format(Tag_ID, TagDetection_State, Fps))
                    At_Detection_Currenttag, At_Detection_Destinationtag = Value('i', 0), Value('i', 0)
                    # stdout.write("\n[-->ADHT] At_Detection_Currenttag={}, At_Detection_Destinationtag={},".format(At_Detection_Currenttag, At_Detection_Destinationtag))
                    T_detect0_min, T_detect480_min = Value('i', 0), Value('i', 0)
                    B_detecty_min = Value('i', 0); Capture_click = Value('i', 0)
                    DetectedSide480, DetectedSide0, DetectedSideB = Value('i', 0), Value('i', 0), Value('i', 0)
                    ui_SwitchFlag = Value('i', -1)
                    # stdout.write("\n[-->ADHT] Light_Correction_Algo -> p1 ... {},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    p1 = Process(target=apriltagDetectionFunction, args=(AT_X, AT_Y, AT_Z, AT_Yaw, Tag_ID, TagDetection_State, At_Detection_Currenttag, At_Detection_Destinationtag, Input_Process_EnableFlag, CameraConnectionErrorFlag, ApriltagModule_flag, T_detect0_min, T_detect480_min, B_detecty_min, Capture_click, DetectedSide480, DetectedSide0, DetectedSideB, ui_SwitchFlag))

                    stdout.write("\n[-->ADHT] 0 t22={}, p1 -> p1.start ...{},".format(t22, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    stdout.flush()
                    try:
                        p1.start()
                        stdout.write("\n[-->ADHT] 3 t22={}, p1 -> p1.start ... {},".format(t22, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                        stdout.flush()
                        try:
                            if not p1.is_alive():
                                stdout.write("\n[-->ADHT]: p1 is a goner t22={},".format(t22)); stdout.flush()
                                p1.join(timeout=1.0)
                                stdout.write("\n[-->ADHT] Joined p1 successfully! t22={},".format(t22)); stdout.flush()
                        except Exception as e:
                            stdout.write("\n\n\n[-->ADHT]mainFunction()[->]if not p1.is_alive(): t22={}, exception={},".format(t22, e));
                            stdout.flush()
                            pass
                    except Exception as e:
                        stdout.write("\n\n\n[-->ADHT]mainFunction()[->]p1.start() t22={}, exception={},".format(t22, e));
                        stdout.flush()
                        pass
                    stdout.write("\n[-->ADHT] 1 t22={}, p1.start -> NewProcessCreateFlag ... {},".format(t22, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    stdout.flush()
                    NewProcessCreateFlag = 0
                    ApriltagModuleFlag[0] = 2
                    stdout.write("\n[-->ADHT] New Process t22={}, Create ... {},".format(t22, date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                    stdout.flush()
                if not test_mode:
                    # stdout.write("\n[-->ADHT]if not test_mode: 1 ATDetection_CurrentTag[0]={}, ATDetection_DestinationTag[0]={},".format(ATDetection_CurrentTag[0], ATDetection_DestinationTag[0]))
                    if (not ATDetection_CurrentTag[0]) or (not ATDetection_DestinationTag[0]):
                        ATDetection_CurrentTag[0], ATDetection_DestinationTag[0] = At_Detection_Currenttag.value, At_Detection_Destinationtag.value
                    else:
                        At_Detection_Currenttag.value, At_Detection_Destinationtag.value = ATDetection_CurrentTag[0], ATDetection_DestinationTag[0]
                    # stdout.write("\n[-->ADHT]if not test_mode: 2 ATDetection_CurrentTag[0]={}, ATDetection_DestinationTag[0]={},".format(ATDetection_CurrentTag[0], ATDetection_DestinationTag[0]))

                    # stdout.write("\n[-->ADHT]if not test_mode: 1 InputProcessEnable[0]={}, Input_Process_EnableFlag.value={},".format(InputProcessEnable[0], Input_Process_EnableFlag.value))
                    if InputProcessEnable[0] == 1 and Input_Process_EnableFlag.value == 0:
                        Input_Process_EnableFlag.value = 1
                    elif InputProcessEnable[0] == 0 and Input_Process_EnableFlag.value == 2:
                        Input_Process_EnableFlag.value = 0
                        NewProcessCreateFlag = 1
                    # stdout.write("\n[-->ADHT]if not test_mode: 2 InputProcessEnable[0]={}, Input_Process_EnableFlag.value={},".format(InputProcessEnable[0], Input_Process_EnableFlag.value))

                    ApriltagModuleFlag[0] = ApriltagModule_flag.value

                # stdout.write("\n[-->ADHT]if not test_mode: 1 type(IgnoreTagList_AtThread[0])={}, IgnoreTagList_AtThread={},".format(type(IgnoreTagList_AtThread[0]), IgnoreTagList_AtThread))
                t_detect0_min[0], t_detect480_min[0] = T_detect0_min.value, T_detect480_min.value
                b_detecty_min[0] = B_detecty_min.value
                detectedSide480[0], detectedSide0[0], detectedSideB[0] = DetectedSide480.value, DetectedSide0.value, DetectedSideB.value

                # UISwitchFlag[0] = ui_SwitchFlag.value

                if UISwitchFlag[0] == 0 and ui_SwitchFlag.value != 0:
                    ui_SwitchFlag.value = 0
                    stdout.write("\n[-->ADHT]mainFunction()[->]value != 0: UISwitchFlag[0]={}, ui_SwitchFlag.value={}, t22={},".format(UISwitchFlag[0], ui_SwitchFlag.value, t22))
                    stdout.flush()
                elif UISwitchFlag[0] == 1 and ui_SwitchFlag.value != 1:
                    ui_SwitchFlag.value = 1
                    stdout.write("\n[-->ADHT]mainFunction()[->]value != 1: UISwitchFlag[0]={}, ui_SwitchFlag.value={}, t22={},".format(UISwitchFlag[0], ui_SwitchFlag.value, t22))
                    stdout.flush()

                if Capture_click.value == 0:
                    Capture_click.value = capture_click[0]
                elif capture_click[0] != 0:
                    capture_click[0] = Capture_click.value
                elif capture_click[0] == 0:
                    Capture_click.value = capture_click[0]

                if type(IgnoreTagList_AtThread[0]) == list:
                    if Tag_ID.value not in IgnoreTagList_AtThread[0]:
                        ATX[0], ATY[0], ATZ[0] = AT_X.value, AT_Y.value, AT_Z.value
                        ATYaw[0] = AT_Yaw.value
                        TagID[0], TagDetectionState[0] = Tag_ID.value, TagDetection_State.value
                else:
                    ATX[0], ATY[0], ATZ[0] = AT_X.value, AT_Y.value, AT_Z.value
                    ATYaw[0] = AT_Yaw.value
                    TagID[0], TagDetectionState[0] = Tag_ID.value, TagDetection_State.value
            except Exception as e:
                stdout.write("\n\n\n[-->ADHT]mainFunction()[->] t22={}, exception={},".format(t22, e))
                stdout.write("\n[-->ADHT]mainFunction()[->] NewProcessCreateFlag={}, t22={},".format(NewProcessCreateFlag, t22))
                stdout.flush()
                pass


try:
    T_April = [Thread(target = mainFunction)]
except KeyboardInterrupt:
    T_April[0].join()
    pass

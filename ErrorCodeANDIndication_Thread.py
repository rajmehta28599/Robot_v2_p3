"""
Created on Fri Oct 28 12:13:20 2022.

@author: Raj-Mehta
@email : rajmehta28599v@gmail.com
"""
from sys import stdout
from threading import Thread
from pandas import read_csv
from os import popen, system
from time import time
from datetime import datetime
from canOpen_ethernet_Thread import ErrorCode, Estop_FB
from ApriltagDetectionHandlingThread_1 import UISwitchFlag
# from Functional_UI_asThread import playpause
from pyttsx3 import init
engine = init()
FinalErrorCodeList = [0]
VoiceFilePath = './Thread/VoiceFiles'
date = datetime.now


def mainFunctionECIC():
    """
    mainFunctionECIC.

    Returns
    -------
    None.
    """
    errorCodeDataFile = open("log/errorCodeDataFile.csv", "a")
    errorCodeDataFile.write("time={},ErrorCode_List\n".format(date().strftime("%d-%m-%Y %H_%M_%S_%f")))
    errorCodeDataFile.flush()
    errorCodeDataFile.close()

    prev_ErrorCode, prev_ErrorCode_int, prev_UISwitchFlag = -1, -1, -1
    reSpeak_counter1, reSpeak_counter2 = time(), time()
    ErrorCode_int = 0
    # stdout.write('\n[-->T_ECIC] thread start above while')
    res, temp_ErrorCode_int, ErrorCode_List = [], [], []
    errorT_t1 = errorT_t2 = time()
    vol_IMP, vol_NOM = 80, 60
    while 1:
        errorT_t2 = time() - errorT_t1
        if errorT_t2 > 0.1:
            errorT_t1 = time()
            try:
                # stdout.write('\n[-->T_ECIC]while 1:if not UISwitchFlag[0]={},'.format(UISwitchFlag[0]))
                if not UISwitchFlag[0]:
                    # stdout.write('\n[-->T_ECIC]if not UISwitchFlag[0]={},'.format(UISwitchFlag[0]))
    
                    if UISwitchFlag[0] != prev_UISwitchFlag:
                        # stdout.write('\n[-->T_ECIC] csv config Done')
                        # ErrorCodeList.csv
                        filename = './Configurations/ErrorCodeList.csv'
                        ErrorCodeDF = read_csv(filename).dropna(subset=["Error Name"]).set_index("Error Name")
    
                        ShutdownError_VoiceFile = ErrorCodeDF.loc['Shutdown Error']['Voice File'].split('.')
                        EmergencyStopError_VoiceFile = ErrorCodeDF.loc['Emergency Stop Error']['Voice File'].split('.')
                        ObjectDetectionError_VoiceFile = ErrorCodeDF.loc['Object Detection Error']['Voice File'].split('.')
                        InternalError_VoiceFile = ErrorCodeDF.loc['Internal Error']['Voice File'].split('.')
                        TagDetectionError_VoiceFile = ErrorCodeDF.loc['Tag Detection Error']['Voice File'].split('.')
                        LowBatteryError_VoiceFile = ErrorCodeDF.loc['Low Battery Error']['Voice File'].split('.')
    
                        ShutdownErrorCode = int(ErrorCodeDF.loc['Shutdown Error']['Error Code'])
                        EmergencyStopErrorCode = int(ErrorCodeDF.loc['Emergency Stop Error']['Error Code'])
                        ObjectDetectionErrorCode = int(ErrorCodeDF.loc['Object Detection Error']['Error Code'])
                        # BmsDataErrorCode = int(ErrorCodeDF.loc['BMS Data Error']['Error Code'])
                        BMIDataErrorCode = int(ErrorCodeDF.loc['BMI Data Error']['Error Code'])
                        BMIConnectionErrorCode = int(ErrorCodeDF.loc['BMI Connection Error']['Error Code'])
                        TagDetectionErrorCode = int(ErrorCodeDF.loc['Tag Detection Error']['Error Code'])
                        LowBatteryErrorCode = int(ErrorCodeDF.loc['Low Battery Error']['Error Code'])
                        del ErrorCodeDF
                        prev_UISwitchFlag = UISwitchFlag[0]
    
                    ErrorCode_int = int(ErrorCode[0])
                    try:
                        if ErrorCode_int != prev_ErrorCode_int:
                            ErrorCode_int = int(ErrorCode[0])
                            temp_ErrorCode_int.append(ErrorCode_int)
                            [res.append(x) for x in temp_ErrorCode_int if x not in res]
                            ErrorCode_List = res
                            ErrorCode_List.sort()
                            prev_ErrorCode_int = ErrorCode_int
                            # stdout.write("\n\n\n[-->T_ECIC] ErrorCode_List res={}, ErrorCode_List={},".format(res, ErrorCode_List))
                    except Exception as e:
                        stdout.write("\n\n\n[-->T_ECIC] ErrorCode_List exception={},".format(e)); stdout.flush()
                        pass
    
                    # stdout.write('\n[-->T_ECIC] ErrorCode_int={}, type(ErrorCode_int)={},'.format(ErrorCode_int, type(ErrorCode_int)))
                    reSpeak_counter2 = time() - reSpeak_counter1
    
                    if ErrorCode_int:
                        if ErrorCode_int != 999 and len(ErrorCode_List) and ErrorCode_int == ErrorCode_List[0]:
                            if ErrorCode_List[0] != prev_ErrorCode or reSpeak_counter2 > 10:
                                if ErrorCode_List[0] == ShutdownErrorCode:
                                    if len(ShutdownError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception ShutdownError_VoiceFile')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_IMP, VoiceFilePath, ShutdownError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_IMP, VoiceFilePath, ShutdownError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(ShutdownError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(ShutdownError_VoiceFile[0])
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                # elif ErrorCode_List[0] == EmergencyStopErrorCode and (Estop_FB[0] == '1' or playpause[0] == 0):
                                elif ErrorCode_List[0] == EmergencyStopErrorCode or Estop_FB[0] == '1':
                                    if len(EmergencyStopError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception EmergencyStopError_VoiceFile')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, EmergencyStopError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, EmergencyStopError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(EmergencyStopError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(EmergencyStopError_VoiceFile[0]) 
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                elif ErrorCode_List[0] == ObjectDetectionErrorCode:
                                    if len(ObjectDetectionError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception ObjectDetectionError_VoiceFile')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, ObjectDetectionError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, ObjectDetectionError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(ObjectDetectionError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(ObjectDetectionError_VoiceFile[0])
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                # 16-1-23 11:42am
                                elif ErrorCode_List[0] == LowBatteryErrorCode:
                                    if len(LowBatteryError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception LowBatteryError_VoiceFile')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, LowBatteryError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, LowBatteryError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(LowBatteryError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(TagDetectionError_VoiceFile[0])
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                # 29/11/22 12:20pm
                                elif ErrorCode_List[0] == TagDetectionErrorCode:
                                    if len(TagDetectionError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception TagDetectionError_VoiceFile')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, TagDetectionError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, TagDetectionError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(TagDetectionError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(TagDetectionError_VoiceFile[0])
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                elif ErrorCode_List[0] == BMIDataErrorCode:
                                    if len(InternalError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception InternalError_VoiceFile BMIDataErrorCode')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, InternalError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, InternalError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(InternalError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(InternalError_VoiceFile[0])
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                # 29/11/22 12:31pm
                                elif ErrorCode_List[0] == BMIConnectionErrorCode:
                                    if len(InternalError_VoiceFile) == 2:
                                        # stdout.write('\n[-->T_ECIC]exception InternalError_VoiceFile BMIConnectionErrorCode')
                                        try:
                                            system("sudo mpg321 -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, InternalError_VoiceFile[0]))
                                        except Exception as e:
                                            system("sudo mpg321 -o alsa -g {} {}/{}.mp3".format(vol_NOM, VoiceFilePath, InternalError_VoiceFile[0]))
                                            stdout.write("\n\n\n[-->T_ECIC] if len(InternalError_VoiceFile) == 2: exception={},".format(e))
                                            stdout.flush()
                                            pass
                                    else:
                                        engine.say(InternalError_VoiceFile[0]) 
                                        engine.runAndWait()
                                    prev_ErrorCode = ErrorCode_List[0]
                                elif ErrorCode_List[0] > 0 and ErrorCode_List[0] != EmergencyStopErrorCode:
                                    # stdout.write('\n[-->T_ECIC]elif ErrorCode_List[0] != 999: Not not not ErrorCode_List[0]={},'.format(ErrorCode_List[0]))
                                    engine.say("New              Error      code")
                                    engine.runAndWait()
                                    pass
                                reSpeak_counter1 = time()
                                errorCodeDataFile = open("log/errorCodeDataFile.csv", "a")
                                errorCodeDataFile.write("{},{}\n".format(date().strftime("%d-%m-%Y %H_%M_%S_%f"), ErrorCode_int))
                                errorCodeDataFile.flush()
                            else:
                                if len(ErrorCode_List):
                                    # stdout.write('\n[-->T_ECIC]if len(ErrorCode_List):clear ErrorCode_List={},'.format(ErrorCode_List))
                                    ErrorCode_List.clear()
                                pass
                        else:
                            if len(ErrorCode_List):
                                # stdout.write('\n[-->T_ECIC]else:if len(ErrorCode_List):clear ErrorCode_List={},'.format(ErrorCode_List))
                                prev_ErrorCode = ErrorCode_List[0]
                                ErrorCode_List.clear()
            except Exception as e:
                stdout.write("\n\n\n[-->T_ECIC] def mainFunctionECIC(): exception={},".format(e))
                stdout.flush()
                pass


try:
    T_ECIC = [Thread(target=mainFunctionECIC)]
except Exception as e:
    T_ECIC[0].join()
    stdout.write("\n\n\n[-->T_ECIC] thread exception={},".format(e))
    stdout.flush()
    pass

'''
Created on Wed July 20 12:13:20 2022.

@author: Raj-Mehta.
@email : rajmehta28599v@gmail.com

change log:
    30-1-23:
        -> bmi error poweroff to reboot
        -> connection time 10sec
        -> 
'''

from threading import Thread
from socket import socket, create_connection, gethostbyname, gethostname
from time import time, sleep
from sys import stdout
from datetime import datetime
from serial import Serial
from os import system
from pickle import load, dump
from os.path import exists  # basename
from google_auth_oauthlib.flow import InstalledAppFlow  # Flow
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload  # MediaIoBaseDownload
from google.auth.transport.requests import Request
from base64 import urlsafe_b64encode
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

from ApriltagDetectionHandlingThread_1 import TagID, TagDetectionState

stdout.write("\n[-->cet] WELCOME")
stdout.flush()
dataToTeensy, prev_dataToTeensy = [''], ['']

# 1 = Main_SW_FB, 2 = BmiYAW, 3 = CurrentValue, 4 = VoltageValue, 5 = RightEncData, 6 = Estop_FB
Main_SW_FB, BmiYAW, CurrentValue = [0], [0], [0]
VoltageValue, RightEncData, ErrorCode, Estop_FB = [0], [0], [0], [0]

# ip as per the ethernet 192.168.10.2-255 random
# host = "192.168.2.3"
# port = 80             #ESP32 Server Port
list1 = [0.0]
rebootFlag = [0]
date = datetime.now
robot_name = gethostname()

def is_connected():
    try:
        # see if we can resolve the host name -- tells us if there is
        # a DNS listening
        host = gethostbyname("www.google.com")
        # connect to the host -- tells us if the host is actually
        # reachable
        s = create_connection((host, 80), 2)
        stdout.write("\n[-->cet] create_connection={},".format(s))
        stdout.flush()
        return True
    except Exception as e:
        stdout.write("\n\n\n[-->cet] is_connected() exception={},".format(e))
        stdout.flush()
    return False


def Create_Service(client_secret_file, api_name, api_version, *scopes):
    CLIENT_SECRET_FILE = client_secret_file
    API_SERVICE_NAME = api_name
    API_VERSION = api_version
    SCOPES = [scope for scope in scopes[0]]
    cred = None
    pickle_file = 'token_' + API_SERVICE_NAME + '_' + API_VERSION + '.pickle'

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
        stdout.write("\n[-->cet] Create_Service(): {}, service created successfully".format(API_SERVICE_NAME))
        stdout.flush()
        return service
    except Exception as e:
        stdout.write("\n\n\n[-->cet] Create_Service(): exception={},".format(e))
        stdout.flush()
        return None


def ethernetCom():
    """
    Script work to send data on ehternet.

    Returns
    -------
    None.

    """
    try:
        # s2 = None
        sleep(1)
        sock = socket()
        sock.connect(("192.168.2.3", 80))
    except Exception as e:
        stdout.write("\n\n\n[-->cet]ethernetCom(): reboot exception={},".format(e))
        stdout.flush()
        sleep(10)
        # system("sudo reboot")
        pass

    try:
        # sleep(0.01)
        csvt1 = t1 =  time()
        ShutDown_Count1 = shutDown_t1 = shutDown_t2 = time()
        dataToTeensy[0] = '0,0,101.87,105,103.12,108\n'
        dataToTeensyDataFile = open("log/dataToTeensyDataFile.csv", "a")
        dataToTeensyDataFile.write("time={},Main_SW_FB,_BMI_YAW,bat_current_filtered,voltage_24_filtered,encoder_count0,errorCode,estop_FB\n".format(date().strftime("%d-%m-%Y %H_%M_%S_%f")))
        dataToTeensyDataFile.flush()
        dataToTeensyDataFile.close()

        robotChargingDataFile = open("log/robotChargingDataFile.csv", "a")
        robotChargingDataFile.write("time={},Main_SW_FB,_BMI_YAW,bat_current_filtered,voltage_24_filtered,encoder_count0,errorCode,estop_FB\n".format(date().strftime("%d-%m-%Y %H_%M_%S_%f")))
        robotChargingDataFile.flush()
        robotChargingDataFile.close()

        # dataToTeensyDataFile = open("dataToTeensyDataFile.csv", "a")
    except Exception as e:
        stdout.write("\n\n\n[-->cet]csv  exception={},".format(e))
        stdout.flush()
        pass

    while True:
        try:
            t2 = (time() - t1)
            if t2 >= 0.007:  # 0.007, 0.011  # 11/11/22 11:50am
                t1 = time()
                # stdout.write("\n[-->cet] t2={}, len(dataToTeensy[0])={}, ".format(t2, len(dataToTeensy[0])))
                if len(dataToTeensy[0]):
                    try:
                        # dataToTeensy[0] = '0,0,101.87,105,103.12,108\n'
                        sock.sendall(dataToTeensy[0].encode())
                    except Exception as e:
                        stdout.write("\n\n\n[-->cet]sock.sendall(dataToTee exception={}, 30<shutDown_t2={},".format(e, shutDown_t2))
                        stdout.flush()
                        shutDown_t2 = time() - shutDown_t1
                        if shutDown_t2 > 30: system("sudo reboot")
                        # pass
                # data = sock.recv(28).decode().split(',')
                data = sock.recv(34).decode().split(',')
                # stdout.write("\n[-->cet] len(data)={}, data={}, ".format(len(data), data))
                if len(data) == 9:
                    # if data[0] == '0' and (data[5] == '1' or data[5] == '0'):
                    if data[0] == '<' and data[8] == '>':
                        try:
                            list1 = float(data[5])
                            if type(list1) == float:
                                Main_SW_FB[0], BmiYAW[0], CurrentValue[0] = data[1], data[2], data[3]
                                VoltageValue[0], RightEncData[0], ErrorCode[0], Estop_FB[0] = data[4], round(list1/41.2, 2), data[6], data[7]

                                # 29/11/22 12:45pm
                                if prev_dataToTeensy[0] != dataToTeensy[0]:
                                    stdout.write("\n[-->cet]t={}, TagID={}, TD={}, t2={}, RightEncData[0]={}, data={}, dataToTeensy[0]={},".format(date().strftime("_%H_%M_%S_%f"), TagID[0], TagDetectionState[0], round(t2, 4), RightEncData[0], data, dataToTeensy[0]))
                                    stdout.flush()
                                # data observation csv file.
                                csvt2 = time() - csvt1
                                if csvt2 > 4:
                                    csvt1 = time()
                                    if float(CurrentValue[0]) > 0:
                                        robotChargingDataFile = open("log/robotChargingDataFile.csv", "a")
                                        robotChargingDataFile.write(str(date().strftime("%d-%m-%Y %H:%M:%S:%f")) + "," + str(Main_SW_FB[0]) + "," + str(BmiYAW[0]) + "," + str(CurrentValue[0]) + "," + str(VoltageValue[0]) + "," + str(RightEncData[0]) + "," + str(ErrorCode[0]) + "," + str(Estop_FB[0]) + "\n")
                                        robotChargingDataFile.flush()
                                    # else:
                                        # dataToTeensyDataFile = open("log/dataToTeensyDataFile.csv", "a")
                                        # dataToTeensyDataFile.write(str(date().strftime("%d-%m-%Y %H:%M:%S:%f")) + "," + str(Main_SW_FB[0]) + "," + str(BmiYAW[0]) + "," + str(CurrentValue[0]) + "," + str(VoltageValue[0]) + "," + str(RightEncData[0])  + "," + str(ErrorCode[0]) + "," + str(Estop_FB[0]) + "\n")
                                        # dataToTeensyDataFile.flush()
                            else:
                                stdout.write("\n[-->cet]else: t={}, RightEncData[0]={}, t2={}, data={}, dataToTeensy[0]={},".format(date().strftime("_%H_%M_%S_%f"), RightEncData[0], round(t2, 3), data, dataToTeensy[0]))
                                stdout.flush()
                                pass
                        except Exception as e:
                            stdout.write("\n\n\n[-->cet]RightEncData[0] = list//10.3  exception={},".format(e))
                            stdout.flush()
                            pass

                        # BMI data error
                        if BmiYAW[0] == '44444':
                            dataToTeensy[0] = '0,1,44444,44444,44444,108\n'
                            stdout.write("\n\n\n[-->cet]sock.sendall(dataToTee exception=BmiYAW[0] 44444, reboot")
                            stdout.flush()
                            ErrorCode[0] = 115
                            sleep(2)
                            ErrorCode[0] = 101
                            sleep(2)
                            system("sudo reboot")
                            break

                        # for robot ShutDown from MainSwitch off more then 5 sec.
                        ShutDown_Count2 = time() - ShutDown_Count1
                        if Main_SW_FB[0] == '0' or Estop_FB[0] == '0':
                            ShutDown_Count1 = time()
                        elif Main_SW_FB[0] == '1' and ShutDown_Count2 > 5 and Estop_FB[0] == '1':
                            system("nmcli radio wifi on")
                            sleep(2)
                            stdout.write("\n[-->cet] nmcli radio wifi on")
                            stdout.flush()

                            stdout.write("\n[-->cet] start time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                            stdout.flush()
                            ErrorCode[0] = 101
                            conWait_t1 = conWait_t2 = time()
                            while not is_connected():
                                conWait_t2 = time() - conWait_t1
                                if conWait_t2 > 10:
                                    break

                            stdout.write("\n[-->cet] end time={},".format(date().strftime("%d-%m-%Y %H:%M:%S:%f")))
                            stdout.flush()

                            if is_connected():
                                Client_Serect_File = 'client_secret_test2.json'
                                API_NAME = 'drive'
                                API_VERSION = 'v3'
                                SCOPES = [
                                    'https://www.googleapis.com/auth/drive']

                                try:
                                    service_test = Create_Service(Client_Serect_File, API_NAME, API_VERSION, SCOPES)
                                except Exception as e:
                                    stdout.write("\n\n\n[-->cet]Create_Service exception={},".format(e))
                                    stdout.flush()
                                    pass
                                # updated on 19-01-2023 12:46
                                file_name_csv = 'OrderServeDataFile.csv'
                                mime_type_csv = 'text/csv'
                                folder_ID = '1EJy1ztX3uEUhCEZuDvxQy2g8Jxdu2tI8'

                                file_metadata = {
                                    'name': file_name_csv,
                                    'mimeType': mime_type_csv,
                                    'parents': [folder_ID]
                                }

                                query = "name='{}' and trashed=false and parents='{}'".format(file_name_csv, folder_ID)
                                results = service_test.files().list(q=query, fields="nextPageToken, files(id, name)").execute()
                                filesOnDrive = results.get("files", [])

                                media = MediaFileUpload('{}'.format(file_name_csv), mimetype=mime_type_csv)

                                if not filesOnDrive:
                                    response = service_test.files().create(supportsTeamDrives=True,body=file_metadata, media_body=media, fields='id').execute()
                                else:
                                    file_id = filesOnDrive[0].get('id')
                                    response_update = service_test.files().update(fileId=file_id, media_body=media, supportsTeamDrives=True).execute()

                            else:
                                stdout.write("\n[-->cet]--> internet_connction, no internet connection: cvs file update,")
                                stdout.flush()

                            stdout.write("\n\n\n[-->cet] Robot ShoutDown from Main_SW_FB={}, Estop_FB={}, ShutDown_Count2={}, time={},\n\n".format(Main_SW_FB[0], Estop_FB[0], ShutDown_Count2, date().strftime("%d-%m-%Y_%H:%M:%S:%f")))
                            stdout.flush()
                            if Main_SW_FB[0] == '0':
                                stdout.write("\n\n\n[-->cet] BREAK Robot ShoutDown from Main_SW_FB=0 time={},".format(date().strftime("%d-%m-%Y_%H:%M:%S:%f")))
                                stdout.flush()
                            else:
                                stdout.write("\n\n\n[-->cet] final Robot ShoutDown from Main_SW_FB=1 time={},".format(date().strftime("%d-%m-%Y_%H:%M:%S:%f")))
                                stdout.flush()
                                system("sudo poweroff")
                                pass
                        # 29/11/22 12:45pm
                        prev_dataToTeensy[0] = dataToTeensy[0]
        except Exception as e:
            stdout.write("\n\n\n[-->cet]while  exception={},".format(e))
            stdout.flush()
            pass
    sock.close()


try:
    E_Com = [Thread(target=ethernetCom, args=())]
    # E_Com[0].start()
except Exception as e:
    stdout.write("\n\n\n[-->cet]thrad join exception={},".format(e))
    stdout.flush()
    E_Com[0].join()
    pass

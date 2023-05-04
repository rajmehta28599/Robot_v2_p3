"""
Created on Fri Oct 28 12:13:20 2022.

@author: Raj-Mehta
@email : rajmehta28599v@gmail.com

bt_input_data : is only accept TagID.csv file data.

"""
from time import sleep, time
from threading import Thread
from pandas import read_csv
from serial import Serial
from sys import stdout, exit
from collections import OrderedDict
from datetime import datetime
ChargingFlag, BatteryValue = [-1], [100]
DestinationNodeList, DestinationTrayList = [0], [0]
Real_nodeList, BluetoothInputEnable = [0], [0]
CurrentTrayId, WaitingForInput = [-1], [-1]
CurrentTagID, ScreenInputList, ScreenInputFlag, is_on = [0], [0], [0], [1]
date = datetime.now


def fetchPath(node_list, tray_list, DistanceList, VirtualNodeList, temp_list, HomeTagId, PathAssignmentDF, dataAssignmentDF, PathList, virtualNodeDF):
    """

    Parameters.

    node_list : TYPE
        DESCRIPTION.
    tray_list : TYPE
        DESCRIPTION.
    DistanceList : TYPE
        DESCRIPTION.
    VirtualNodeList : TYPE
        DESCRIPTION.
    temp_list : TYPE
        DESCRIPTION.
    HomeTagId : TYPE
        DESCRIPTION.
    PathAssignmentDF : TYPE
        DESCRIPTION.
    dataAssignmentDF : TYPE
        DESCRIPTION.
    PathList : TYPE
        DESCRIPTION.

    Returns
    -------
    final_destination_node_List : TYPE
        DESCRIPTION.
    final_destination_tray_List : TYPE
        DESCRIPTION.
    node_list : TYPE
        DESCRIPTION.

    """
    try:
        # stdout.write("\n[-->IMT]1node_list={}, CurrentTagID[0]={},".format(node_list, CurrentTagID[0]))
        # stdout.flush()
        if CurrentTagID[0] in node_list:
            # stdout.write("\n[-->IMT]1 node_list={}, tray_list={},".format(node_list, tray_list)); stdout.flush()
            del tray_list[node_list.index(CurrentTagID[0])]
            del node_list[node_list.index(CurrentTagID[0])]
            # stdout.write("\n[-->IMT]2 node_list={}, tray_list={},".format(node_list, tray_list)); stdout.flush()
        # stdout.write("\n[-->IMT]2node_list={}, CurrentTagID[0]={},".format(node_list, CurrentTagID[0]))
        # stdout.flush()

        for i in range(len(node_list)):
            # stdout.write("\n[-->IMT] i={}, node_list[i]={}, len(node_list)={}, PathList={}, CurrentTagID[0]={},".format(i, node_list[i], len(node_list), PathList, CurrentTagID[0]))
            # stdout.flush()
            if node_list[i] in VirtualNodeList:
                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == node_list[i]]['RealNode'])
                if CurrentTagID[0] in VirtualNodeList:
                    real_node2 = int(virtualNodeDF[virtualNodeDF.VirtualNode == CurrentTagID[0]]['RealNode'])
                    if real_node != real_node2:
                        # temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == HomeTagId) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                        temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node2) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                    else:
                        temp_list = [real_node]
                        # stdout.write("\n[-->IMT]:temp_list={}, i={}, node_list[i]={}, len(node_list)={}, PathList={}, real_node={}, real_node2={},".format(temp_list, i, node_list[i], len(node_list), PathList, real_node, real_node2))
                        # stdout.flush()
                elif CurrentTagID[0] != real_node:
                    temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == real_node)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                else:
                    temp_list = [real_node]
                temp_list.append(node_list[i])
                PathList.append(temp_list)
                # stdout.write("\n[-->IMT]if node_lis: real_node={}, temp_list={}, PathList={},".format(real_node, temp_list, PathList))
                # stdout.flush()
            elif CurrentTagID[0] not in VirtualNodeList and CurrentTagID[0] != node_list[i]:
                # stdout.write("\n[-->IMT]elif CurrentTagID[0] not in Vi: CurrentTagID[0]={},".format(CurrentTagID[0])); stdout.flush()
                PathList.append(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == CurrentTagID[0]) & (PathAssignmentDF.dn == node_list[i])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                # stdout.write("\n[-->IMT]elif Curren: PathList={},".format(PathList))
                # stdout.flush()
            elif CurrentTagID[0] != node_list[i]:
                real_node = int(virtualNodeDF[virtualNodeDF.VirtualNode == CurrentTagID[0]]['RealNode'])
                if real_node != node_list[i]:
                    temp_list = list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == real_node) & (PathAssignmentDF.dn == node_list[i])][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist()))
                elif real_node == node_list[i]:
                    temp_list = [real_node]
                # stdout.write("\n[-->IMT]elif CurrentTagID[0]={}, != node_list[i]: real_node={}, temp_list={}, 0".format(CurrentTagID[0], real_node, temp_list))
                # stdout.flush()
                if len(temp_list) and temp_list[-1] != node_list[i]: temp_list.append(node_list[i])
                elif not len(temp_list): temp_list.append(node_list[i])
                PathList.append(temp_list)
                # stdout.write("\n[-->IMT]elif Cu: real_node={}, temp_list={},".format(real_node, temp_list))
                # stdout.flush()
            elif CurrentTagID[0] == node_list[i] and len(node_list) > i:
                # stdout.write("\n[-->IMT] i={}, node_list[i]={}, len(node_list={}, CurrentTagID[0]={},".format(i, node_list[i], len(node_list), CurrentTagID[0]))
                # stdout.write("\n[-->IMT] i={}, node_list={}, len(node_list)={}, CurrentTagID[0]={},".format(i, node_list, len(node_list), CurrentTagID[0]))
                # stdout.flush()
                pass
            elif HomeTagId != node_list[i]:
                # stdout.write("\n[-->IMT]elif i={}, HomeTagId != node_list[i]={}, : HomeTagId={},".format(i, node_list[i], HomeTagId))
                # stdout.flush()
                PathList.append(list(map(int, PathAssignmentDF[(PathAssignmentDF.cn == node_list[i]) & (PathAssignmentDF.dn == HomeTagId)][PathAssignmentDF > 0].dropna(axis=1).iloc[0, 3:].tolist())))
                node_list[i] = HomeTagId
                DestinationTrayList[0] *= 0
                Real_nodeList[0] *= 0
                # stdout.write("\n[-->IMT]elif i={}, HomeTagId != node_list[i]={}, : HomeTagId={},".format(i, node_list[i], HomeTagId)); stdout.flush()
        # stdout.write("\n[-->IMT] PathList={},".format(PathList))
        # stdout.flush()

        try:
            for i in range(len(PathList)):
                temp = []
                for j in range(len(PathList[i])):
                    if j+1 < len(PathList[i]):
                        if PathList[i][j] in VirtualNodeList:
                            distance = int(virtualNodeDF[(virtualNodeDF.VirtualNode == PathList[i][j]) & (virtualNodeDF.RealNode == PathList[i][j+1])]['Distance'])
                            temp.append(distance)
                        elif PathList[i][j+1] in VirtualNodeList:
                            distance = int(virtualNodeDF[(virtualNodeDF.RealNode == PathList[i][j]) & (virtualNodeDF.VirtualNode == PathList[i][j+1])]['Distance'])
                            temp.append(distance)
                        else:
                            # stdout.write("\n[-->IMT] for i={}, j={}, in range(len(PathList)): PathList[i][j]={}, PathList[i][j+1]={},".format(i, j, PathList[i][j], PathList[i][j+1]))
                            # stdout.flush()
                            temp.append(int(dataAssignmentDF[(dataAssignmentDF.cnode == PathList[i][j]) & (dataAssignmentDF.dnode == PathList[i][j+1])].dropna(axis=1).iloc[0, 0:].tolist()[2]))
                            # stdout.write("\n[-->IMT] for i={}, j={}, in range(len(PathList)): temp={},".format(i, j, temp))
                            # stdout.flush()
                DistanceList.append(sum(temp))
                # stdout.write("\n[-->IMT]fori={}, PathList[i]={}, temp={}, sum(temp)={}, DistanceList={},".format(i, PathList[i], temp, sum(temp), DistanceList))
                # stdout.flush()
        except Exception as e:
            stdout.write("\n\n\n[-->IMT] for i in range(len(PathList)): exception={},".format(e))
            stdout.flush()
            pass

        # logic to arrange according distanace
        final_destination_node_List = list(OrderedDict(sorted(dict(zip(node_list, DistanceList)).items(), key=lambda x: x[1])).keys())
        final_destination_node_distance_List = list(OrderedDict(sorted(dict(zip(node_list, DistanceList)).items(), key=lambda x: x[1])).values())
        # stdout.write("\n[-->IMT] Initial_node_list={},".format(node_list))
        # stdout.write("\n[-->IMT] Initial_DistanceList={},".format(DistanceList))
        # stdout.write("\n[-->IMT] final_destination_node_List={},".format(final_destination_node_List))
        # stdout.write("\n[-->IMT] final_destination_node_distance_List={},".format(final_destination_node_distance_List))
        # stdout.flush()
        final_destination_tray_List = tray_list
        return final_destination_node_List, final_destination_tray_List, node_list
    except Exception as e:
        stdout.write("\n\n\n[-->IMT]def fetchPath(node_list, tray_l: exception={},".format(e))
        stdout.flush()
        pass


def inputDataValidation(input_data):
    """

    Parameters.

    input_data : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.
    TYPE
        DESCRIPTION.

    """
    validation_list1 = input_data.split(',')
    data_length = len(validation_list1)
    # stdout.write("\n[-->IMT] input_data={}, data_length={},".format(input_data, data_length))
    # stdout.flush()
    discription_list = ['Table_Number', 'Tray_Number', 'Charging_Input', 'Battery_Percentage', 'Error_Code']

    if data_length:
        validation_list2 = input_data.split('\\r\\n#')
        data2_length = len(validation_list2)
        # stdout.write("\n[-->IMT] validation_list2={}, data2_length={},".format(validation_list2, data2_length))
        # stdout.flush()
        for i in range(data2_length):
            serv_data = validation_list2[i].split(',')
            for j in range(len(serv_data)):
                try:
                    k = float(serv_data[j])
                except:
                    return 0, "discription_list[j={},]={}, of serv_data[j={},]={}, is not a number:".format(j, discription_list[j], j, serv_data[j])
                return 1, 'valid input'
    else:
        return 0, 'invalid input length:={},'.format(data_length)


def mainInputFunction():
    """
    mainInputFunction.

    Returns
    -------
    None.

    """
    BluetoothPresent, charging_flag = 1, -1

    # if BluetoothPresent:
        # try:
            # BlueTooth = Serial('/dev/BLUETOOTH', 9600, timeout=.1, write_timeout=0)
        # except Exception as e:
            # stdout.write("\n\n\n[-->IMT]BLUETOOTH exception={},".format(e))
            # stdout.flush()
            # pass

    # try:
        # ESP32_Ser = Serial('/dev/ESP32', 115200, timeout=.1, write_timeout=0)
    # except Exception as e:
        # stdout.write("\n\n\n[-->IMT]ESP32_Ser exception={},".format(e))
        # stdout.flush()
        # pass

    ErrorCode, BluetoothInputEnable[0] = [0], 1
    from canOpen_ethernet_Thread import VoltageValue
    from ApriltagDetectionHandlingThread_1 import InputProcessEnable, UISwitchFlag
    from Functional_UI_asThread import FinalDest_Path

    # ARCommunication
    stdout.write("\n[-->IMT] ARstart "); stdout.flush()
    input_t1 = input_t2 = Time1 = time()
    prev_UISwitchFlag = -1
    csv_input_data = screen_input_data = ''
    wcs_input_data = final_input_data = ''
    bt_input_data = ''

    global InputProcessEnable, ScreenInputList, ScreenInputFlag

    while 1:
        input_t2 = time() - input_t1
        if input_t2 >= 0.01:
            # stdout.write("\n[-->IMT] in while ARstart ")
            if not UISwitchFlag[0]:
                if UISwitchFlag[0] != prev_UISwitchFlag:
                    try:
                        # CommonVariables2.csv
                        fileName = './Configurations/CommonVariables2.csv'
                        CommonVariableDF = read_csv(fileName).drop(columns=['Comment', 'Parameter']).dropna(subset=["Name"]).set_index('Name')
                        AutoInputFlag = int(CommonVariableDF.loc['AutoInputFlag'])

                        if AutoInputFlag or (not BluetoothPresent):        
                            InputTagList = CommonVariableDF.loc['InputTagList'].values[0]
                            InputTagList = list(map(int, InputTagList[1:len(InputTagList)-1].split(',')))
                            InputTaskList = CommonVariableDF.loc['InputTaskList'].values[0]
                            InputTaskList = list(map(int, InputTaskList[1:len(InputTaskList)-1].split(',')))

                        HomeTagId = int(CommonVariableDF.loc['HomeNode'])
                        AndroidPingTimeout = float(CommonVariableDF.loc['AndroidPingTimeout'])
                        CombineTrayNumber = int(CommonVariableDF.loc['CombineTrayNumber'])
                        del CommonVariableDF

                        # VirtualNodeMapping.csv
                        fileName = './Configurations/VirtualNodeMapping.csv'
                        virtualNodeDF = read_csv(fileName)
                        VirtualNodeList = list(virtualNodeDF.VirtualNode)

                        # path_assignment_2.csv
                        fileName = "./Configurations/path_assignment_2.csv"
                        PathAssignmentDF = read_csv(fileName)

                        # Data_Assignment_4.csv
                        fileName = './Configurations/Data_Assignment_4.csv'
                        dataAssignmentDF = read_csv(fileName)

                        # TagId.csv
                        fileName = "./Configurations/TagId.csv"
                        TagIdDF = read_csv(fileName)
                        WCS_DeviceIDList = list(TagIdDF.WCS_DeviceID)
                        BT_TagList = list(TagIdDF.TagId)

                        prev_UISwitchFlag = UISwitchFlag[0]
                    except Exception as e:
                        stdout.write("\n\n\n[-->IMT] csv input_t2={}, exception={},".format(input_t2, e)); stdout.flush()
                        pass

                # if BluetoothPresent:
                    # DeltaT = time() - Time1
                    # if (DeltaT >= AndroidPingTimeout) and (not BlueTooth.in_waiting):
                        # try:
                            # # BlueTooth.write('{},{},{},{},{}\r\n'.format(CurrentTagID[0], CurrentTrayId[0], ChargingFlag[0], BatteryValue[0], ErrorCode[0]).encode())
                            # BlueTooth.write('{},{},{},{},{},->{}\r\n'.format(CurrentTagID[0], CurrentTrayId[0], ChargingFlag[0], VoltageValue[0], ErrorCode[0], DestinationNodeList[0]).encode())
                        # except Exception as e:
                            # stdout.write("\n\n\n[-->IMT] BlueTooth.write input_t2={}, exception={},".format(input_t2, e)); stdout.flush()
                            # pass
                        # Time1 = time()

                if (InputProcessEnable[0] or WaitingForInput[0] == 1):
                    try:
                        if AutoInputFlag or not BluetoothPresent:
                            csv_input_data = '#{},{}, -1, -1, -1\\r\\n#{},{}, -1, -1, -1\r\n'.format(InputTagList[0], InputTaskList[0], InputTagList[1], InputTaskList[1])
                            csv_input_data = csv_input_data.strip('#').rstrip('\r\n')
                            final_input_data = csv_input_data
                        elif ScreenInputFlag[0]:
                            stdout.write("\n[-->IMT] t={}, ScreenInputList[0]={}, screen_input_data={}, 0".format(date().strftime("%d_%m_%Y_%H_%M_%S"), ScreenInputList[0], screen_input_data))
                            stdout.write("\n[-->IMT] type(ScreenInputList[0])={}, type(screen_input_data)={}, input_t2={},".format(type(ScreenInputList[0]), type(screen_input_data), input_t2))
                            stdout.flush()
                            screen_input_data = ScreenInputList[0].strip('#').rstrip('\r\n')
                            if len(screen_input_data):
                                final_input_data = screen_input_data
                                stdout.write("\n[-->IMT] ScreenInputList[0]={}, screen_input_data={}, 1 input_t2={},".format(ScreenInputList[0], screen_input_data, input_t2)); stdout.flush()
                                ScreenInputFlag[0] = 0
                            else:
                                final_input_data = ''
                                ScreenInputList[0] = 0
                        else:
                            data, data_set = [''], ['']
                            # if is_on[0]:
                                # try:
                                    # while ESP32_Ser.in_waiting:
                                        # # stdout.write("\n[-->IMT] 0 wcs_input_data={}, type(wcs_input_data)={},".format(wcs_input_data, type(wcs_input_data)))
                                        # wcs_input_data = (ESP32_Ser.readline()).decode().rstrip('\r\n')
                                        # if len(wcs_input_data):
                                            # # BlueTooth.write('wcs_input_data={},\r\n'.format(wcs_input_data).encode())
                                            # # wcs_input_data = int(wcs_input_data)
                                            # if int(wcs_input_data) in WCS_DeviceIDList:
                                                # wcs_input_data = str(int(TagIdDF[TagIdDF.WCS_DeviceID == int(wcs_input_data)]["TagId"])) + ",111,-1,-1,-1"
                                                # data.append(wcs_input_data)
                                                # # stdout.write("\n[-->IMT] 1 wcs_input_data={}, type(wcs_input_data)={}, data={}, data_set={}, input_t2={},".format(wcs_input_data, type(wcs_input_data), data, data_set, input_t2))
                                                # # stdout.flush()
                                                # data_set = list(set(data))
                                    # # stdout.write("\n[-->IMT] 2 wcs_input_data={}, type(wcs_input_data)={}, data={}, data_set={},".format(wcs_input_data, type(wcs_input_data), data, data_set))
                                    # # stdout.flush()
                                # except Exception as e:
                                    # stdout.write("\n\n\n[-->IMT] else:wcs_input_data={}, input_t2={}, exception={},".format(wcs_input_data, input_t2, e))
                                    # stdout.flush()
                                    # pass
                            # else:
                                # if ESP32_Ser.in_waiting:
                                    # ESP32_Ser.reset_input_buffer()
                                    # pass
                            # try:
                                # while BlueTooth.in_waiting:
                                    # stdout.write("\n[-->IMT] 0 BlueTooth.in_waiting={}, type(bt_input_data)={},".format(BlueTooth.in_waiting, type(bt_input_data)))
                                    # try:
                                        # bt_input_data = (BlueTooth.readline()).decode().strip('#').rstrip('\r\n')
                                    # except Exception as e:
                                        # stdout.write("\n\n\n[-->IMT]bt_input_data = (BlueT input_t2={}, exception={},".format(input_t2, e))
                                        # stdout.flush()
                                        # pass                                    
                                    # try:
                                        # if len(bt_input_data):
                                            # BlueTooth.write('BT_input_data={},\r\n'.format(bt_input_data).encode())
                                            # stdout.write("\n[-->IMT] bt_input_data={}, type(bt_input_data[0])={}, bt_input_data[0]={},".format(bt_input_data, type(bt_input_data[0]), bt_input_data[0]))
                                            # stdout.flush()
                                            # try:
                                                # if int(bt_input_data[0]) in BT_TagList:
                                                    # data.append(bt_input_data)
                                                    # stdout.write("\n[-->IMT] 1 bt_input_data={}, type(bt_input_data)={}, data={}, data_set={}, input_t2={},".format(bt_input_data, type(bt_input_data), data, data_set, input_t2))
                                                    # stdout.flush()
                                                    # data_set = list(set(data))
                                            # except Exception as e:
                                                # stdout.write("\n\n\n[-->IMT]if int(bt_input_data:input_t2={}, exception={},".format(input_t2, e))
                                                # stdout.flush()
                                                # pass
                                    # except Exception as e:
                                        # stdout.write("\n\n\n[-->IMT]if len(bt_input_data):input_t2={}, exception={},".format(input_t2, e))
                                        # stdout.flush()
                                        # pass
                                # # stdout.write("\n[-->IMT] 2 bt_input_data={}, type(bt_input_data)={}, data={}, data_set={},".format(bt_input_data, type(bt_input_data), data, data_set))
                            # except Exception as e:
                                # stdout.write("\n\n\n[-->IMT] else:bt_input_data input_t2={}, exception={},".format(input_t2, e)); stdout.flush()
                                # pass

                            # stdout.write("\n[-->IMT] 0 final_input_data={}, type(final_input_data)={}, data_set={}, type(data_set)={},".format(final_input_data, type(final_input_data), data_set, type(data_set)))
                            if len(data_set) > 1:
                                final_input_data = "\\r\\n#".join(data_set[1:])
                                stdout.write("\n[-->IMT] 1 len(data_set)={}, final_input_data={}, type(final_input_data)={}, data_set={}, type(data_set)={}, input_t2={},".format(len(data_set), final_input_data, type(final_input_data), data_set, type(data_set), input_t2))
                                stdout.flush()
                            else:
                                final_input_data = ''
                                # stdout.write("\n[-->IMT] clear final_input_data={},".format(final_input_data)); stdout.flush()

                        data_length = len(final_input_data)
                        # if BluetoothInputEnable[0]:
                        if data_length:
                            valid_input_data, validation_message = inputDataValidation(final_input_data)
                            # stdout.write("\n[-->IMT] valid_input_data={}, validation_message={}, 0 input_t2={},".format(valid_input_data, validation_message, input_t2))
                            # stdout.flush()
                            if valid_input_data:
                                # stdout.write("\n[-->IMT]0 DestinationNodeList[0]={}, DestinationTrayList[0]={}, Real_nodeList[0]={}, input_t2={},".format(DestinationNodeList[0], DestinationTrayList[0], Real_nodeList[0], input_t2))
                                # stdout.flush()
                                if CurrentTagID[0] == HomeTagId:
                                    DestinationNodeList[0] *= 0
                                    DestinationTrayList[0] *= 0
                                    Real_nodeList[0] *= 0
                                temporary_node_list, temporary_tray_list = [], []
                                filtered_data1 = final_input_data.split('\\r\\n#')
                                # stdout.write("\n[-->IMT] filtered_data1={}, input_t2={},".format(filtered_data1, input_t2))
                                # stdout.flush()
                                for i in range(len(filtered_data1)):
                                    filtered_data2 = filtered_data1[i].split(',')
                                    data_length = len(filtered_data2)
                                    # stdout.write("\n[-->IMT] data_length={}, filtered_data2={}, input_t2={},".format(data_length, filtered_data2, input_t2))
                                    # stdout.flush()
                                    # filtered_data2 is bluetooth input data if we want to send

                                    if data_length == 5:
                                        table_number, tray_number = int(filtered_data2[0]), int(filtered_data2[1])
                                        charging_flag, battery_percentage, error_code = int(filtered_data2[2]), int(filtered_data2[3]), int(filtered_data2[4])
                                        temporary_node_list.append(table_number)
                                        temporary_tray_list.append(tray_number)

                                        # stdout.write("\n[-->IMT] TableNumber={}, TrayNumber={},".format(table_number, tray_number))
                                        # stdout.write("\n[-->IMT] ChargingFlag={}, BatteryPercentage={},".format(charging_flag, battery_percentage))
                                        # stdout.write("\n[-->IMT] ErrorCode={}, input_t2={},".format(error_code, input_t2))
                                        # stdout.flush()
                                        # if BluetoothPresent:
                                            # if (not i) and len(filtered_data1) == 2:
                                                # BlueTooth.write(('#{},{},{},{},{}\\r\\n'.format(table_number, tray_number, charging_flag, battery_percentage, error_code)).encode())
                                            # else:
                                                # BlueTooth.write(('#{},{},{},{},{}\r\n'.format(table_number, tray_number, charging_flag, battery_percentage, error_code)).encode())
                                            # Time1 = time()
                                        if charging_flag == 1:
                                            ChargingFlag[0] = 1
                                if ChargingFlag[0] != 1:
                                    try:
                                        PathList, DistanceList, temp_list = [], [], []
                                        node_list, tray_list = temporary_node_list, temporary_tray_list

                                        # stdout.write("\n[-->IMT] VirtualNodeList={},".format(VirtualNodeList))
                                        # stdout.write("\n[-->IMT] node_list={}, tray_list={}, input_t2={},".format(node_list, tray_list, input_t2))
                                        # stdout.flush()

                                        final_destination_node_List, final_destination_tray_List, node_list = fetchPath(node_list, tray_list, DistanceList, VirtualNodeList, temp_list, HomeTagId, PathAssignmentDF, dataAssignmentDF, PathList, virtualNodeDF)

                                        # stdout.write("\n[-->IMT]0 DestinationNodeList[0]={}, len(DestinationNodeList)={},".format(DestinationNodeList[0], len(DestinationNodeList)))
                                        # stdout.write("\n[-->IMT]0 DestinationTrayList[0]={}, input_t2={},".format(DestinationTrayList[0], input_t2))
                                        # stdout.flush()

                                        if type(DestinationNodeList) == list and type(DestinationNodeList[0]) == list:  # 6/12/22 12:47pm
                                            if len(DestinationNodeList[0]):
                                                prev_DestinationNodeList, prev_DestinationTrayList, prev_Real_nodeList = DestinationNodeList[0], DestinationTrayList[0], Real_nodeList[0]
                                                final_destination_node_List, final_destination_tray_List, node_list = final_destination_node_List + prev_DestinationNodeList, final_destination_tray_List + prev_DestinationTrayList, node_list + prev_Real_nodeList
                                                try:
                                                    # stdout.write("\n[-->IMT]0final_destination_node_List={}, final_destination_tray_List={}, node_list={},".format(final_destination_node_List, final_destination_tray_List, node_list))
                                                    # stdout.flush()
                                                    temp_final_destination_node_List, temp_final_destination_tray_List = [], []
                                                    for i in range(len(final_destination_node_List)):
                                                        # stdout.write("\n[-->IMT]1final_destination_node_List={}, i={},".format(final_destination_node_List, i))
                                                        # stdout.flush()
                                                        if final_destination_node_List[i] not in temp_final_destination_node_List:
                                                            # stdout.write("\n[-->IMT]i={}, final_destination_node_List[i]={},".format(i, final_destination_node_List[i]))
                                                            # stdout.flush()
                                                            temp_final_destination_node_List.append(final_destination_node_List[i])
                                                            temp_final_destination_tray_List.append(final_destination_tray_List[i])
                                                    final_destination_node_List, final_destination_tray_List = temp_final_destination_node_List, temp_final_destination_tray_List
                                                    node_list = list(set(node_list))
                                                    # stdout.write("\n[-->IMT]1final_destination_node_List={}, final_destination_tray_List={}, node_list={},".format(final_destination_node_List, final_destination_tray_List, node_list))
                                                    # stdout.flush()
                                                except Exception as e:
                                                    stdout.write("\n\n\n[-->IMT]temp_final input_t2={}, exception={},".format(input_t2, e))
                                                    stdout.flush()
                                                    pass
                                                PathList, DistanceList, temp_list = [], [], []
                                                DestinationNodeList[0], DestinationTrayList[0], Real_nodeList[0] = fetchPath(final_destination_node_List, final_destination_tray_List, DistanceList, VirtualNodeList, temp_list, HomeTagId, PathAssignmentDF, dataAssignmentDF, PathList, virtualNodeDF)
                                                # stdout.write("\n[-->IMT]if prev_Destin: final_destination_node_List={}, input_t2={},".format(final_destination_node_List, input_t2))
                                                # stdout.write("\n[-->IMT]if prev_Destin: DestinationNodeList[0]={}, input_t2={},".format(DestinationNodeList[0], input_t2))
                                                # stdout.flush()
                                            else:
                                                DestinationNodeList[0], DestinationTrayList[0], Real_nodeList[0] = final_destination_node_List, final_destination_tray_List, node_list
                                        else:
                                            DestinationNodeList[0], DestinationTrayList[0], Real_nodeList[0] = final_destination_node_List, final_destination_tray_List, node_list
                                        # stdout.write("\n[-->IMT]1 DestinationNodeList[0]={},".format(DestinationNodeList[0]))
                                        # stdout.write("\n[-->IMT]1 DestinationTrayList[0]={}, input_t2={},".format(DestinationTrayList[0], input_t2))
                                        # stdout.flush()
                                    except Exception as e:
                                        stdout.write("\n\n\n[-->IMT]if ChargingFlag[0] != 1: input_t2={}, exception={},".format(input_t2, e)); stdout.flush()
                                        pass
                                InputProcessEnable[0], WaitingForInput[0] = 0, 0
                                stdout.write("\n[-->IMT]END InputProcessEnable[0]={}, WaitingForInput={}, input_t2={},".format(InputProcessEnable[0], WaitingForInput[0], input_t2))
                                stdout.flush()
                            else:
                                # 2/12/22 3:28pm
                                stdout.write("\n\n\n[-->IMT] exception BlueTooth InputData Error Message validation_message={}, input_t2={},".format(validation_message, input_t2))
                                stdout.flush()
                        elif CurrentTagID[0] != HomeTagId:
                            stdout.write("\n[-->IMT]elif CurrentTagID[0]={}, != HomeTagId={}, DestinationNodeList={}, input_t2={},".format(CurrentTagID[0], HomeTagId, DestinationNodeList, input_t2))
                            stdout.flush()
                            if type(DestinationNodeList) == list and type(DestinationNodeList[0]) == list:  # 6/12/22 12:47pm
                                if len(DestinationNodeList[0]):
                                    temp_DestinationNodeList = DestinationNodeList[0]
                                    if CurrentTagID[0] in temp_DestinationNodeList:
                                        del temp_DestinationNodeList[:temp_DestinationNodeList.index(CurrentTagID[0])]
                                        stdout.write("\n[-->IMT] new DestinationNodeList={}, temp_DestinationNodeList={}, CurrentTagID[0]={}, HomeTagId={}, input_t2={},".format(DestinationNodeList, temp_DestinationNodeList, CurrentTagID[0], HomeTagId, input_t2))
                                        stdout.flush()
                            InputProcessEnable[0], WaitingForInput[0] = 0, 0
                    except Exception as e:
                        stdout.write("\n\n\n[-->IMT] while True: input_t2={}, exception={},".format(input_t2, e))
                        stdout.flush()
                        pass


try:
    T_Input = [Thread(target=mainInputFunction)]
except Exception as e:
    T_Input[0].join()
    stdout.write("\n\n\n[-->IMT]T_Input exception={},".format(e))
    stdout.flush()
    pass

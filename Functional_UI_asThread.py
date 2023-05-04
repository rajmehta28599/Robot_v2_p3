# !/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu 2022-Dec-29 13:18:47
@author: Aanal Patel

:UI for Robot using Tkinter for main code with multiple tray selection and table(up to three) with previous button.
:solved the exception--> tableidList=[j for sub in tableidList for j in sub]
:solved the previouse button exception
:the code is under optimazation(29/11/22) for the better perfomance and error solving.
:optimized the code.
:removed the refresh function from functional UI on Wed 2022-Dec-28 10:37:26
:removed shutdown Button on Thu 2022-Dec-29 13:18:47
:implemention motion UI Fri 30-12-2022 12:47
:implemented music button Tue 03-01-2023 12:43
:removed wcs stack from motion UI(will be implemented later) 10-01-2023 10:36
:implemeted not Charging Mode UI
:added reboot UI and Email API to notify developer of reboot 27-01-2023 10:23
:added timer in reboot UI09-02-2023 11:21
"""

# import
from tkinter import ttk, Tk, Button, Label, DISABLED, TOP, BOTH, RIGHT, LEFT, NORMAL, RAISED, SUNKEN, StringVar, Spinbox, W, N, BOTTOM, Toplevel
from PIL import ImageTk, Image, ImageSequence
from tkinter.ttk import Style, Frame
from pandas import read_csv
from os import stat, path, system

from threading import Thread
from sys import stdout
from time import sleep, time

from Input_Method_thread import ScreenInputList, ScreenInputFlag, ChargingFlag, WaitingForInput, is_on
from ApriltagDetectionHandlingThread_1 import UISwitchFlag, TagID, InputProcessEnable
from canOpen_ethernet_Thread import ErrorCode, CurrentValue, VoltageValue, Main_SW_FB, Estop_FB, dataToTeensy, rebootFlag

# variables (functional)
trayList, traybtn, trayList1, trayButtonList = [], [], [], []  # tray lists
buttons = btList = tableButtonList = []  # table list
a = count = frist_ = x_1 = y_1 = table_a = table_b = table_i = 0
tray_str = ''  # siring for saving tray value
strList_data = []  # list of strings
dict = {}  # used for the previous function
musicCount = [0]

# variables (charge)
charge_active = 0

# variables (Motion)
x_stack = y_stack = 0
stack_call, FinalDest_Path = [], [0]
img_iterator = []
ittration, HomeTagId = 0, -1

# variables (wcs password)
x_stack_wcs = y_stack_wcs = 0
btn_lable_num_wcs = btn_num_str_wcs = ''
pwd_nums_wcs, rootclose = [], [0]

global UISwitchFlag
# files try and exceptions
try:
    if float(CurrentValue[0]) > 0:
        UISwitchFlag[0] = -1
        # stdout.write("\n[-->FUT] Charging Mode--> UISwitchFlag={}".format(UISwitchFlag[0]))
    else:
        stdout.write("\n[-->FUT] Fetch Configuration files;")
        stdout.flush()

        Data_Assignment_4 = "./Configurations/Data_Assignment_4.csv"
        TableFaceAngle_2 = "./Configurations/TableFaceAngle_2.csv"
        path_assignment_2 = "./Configurations/path_assignment_2.csv"

        if path.isfile(Data_Assignment_4) and path.isfile(TableFaceAngle_2) and path.isfile(path_assignment_2):
            # stdout.write("\n[-->FUT] Fetch Configuration files UISwitchFlag[0] 0 if")
            if stat(Data_Assignment_4).st_size > 0 and stat(TableFaceAngle_2).st_size > 0 and stat(path_assignment_2).st_size > 0:
                state1 = NORMAL
                fgcolor = "Blue"
                UISwitchFlag[0] = 0

                fileName = "./Configurations/CommonVariables2.csv"
                CommonVariableDF = read_csv(fileName).drop(columns=['Comment', 'Parameter']).dropna(subset=["Name"]).set_index('Name')

                df = read_csv("./Configurations/TagId.csv")  # ./Configurations/
                ChargeNode = [df["TagId"][1]]
                stdout.write("\n[-->FUT] charging Node = {}, type = {};".format(ChargeNode[0], type(ChargeNode)))
                stdout.flush()

        else:
            stdout.write("\n[-->FUT] Fetch Configuration files UISwitchFlag[0] 0 else;")
            stdout.flush()

except Exception as e:
    stdout.write("\n\n\n[-->FUT]exception--> in uploading files, exception={},".format(e))
    stdout.flush()
    pass


def frames():
    Top = Tk()
    Top.title("main frame")
    Top.geometry("200x200")
    Top.attributes('-fullscreen', True)
    Top.protocol("WM_DELETE_WINDOW", False)  # for not letting user close the window
    stdout.write("\n[-->FUT] frames();")
    stdout.flush()
    if not UISwitchFlag[0]: functional(Top)  # UISwitchFlag=0
    elif UISwitchFlag[0] == -1: charge_main(Top)  # UISwitchFlag=-1 and ChargingFlag=1
    Top.mainloop()


def functional(Top):
    def select_frame2(btn, x1=0, x2=0):
        def select_button():
            None_var = tray1(btn, x1) if x1 != 0 else t1(btn, x2)
            btn.config(bg='light Blue', fg='Blue', activebackground='Blue', relief=SUNKEN)
        return select_button

    def tray1(btn_tray, x):
        global tray_str, count
        count = -1
        tray_str = ''
        if x not in trayList1:
            trayList.append(x)
            trayList1.append(x)
            dict[a] = len(trayList)
            trayButtonList.append(btn_tray)
        
            done.config(fg="Blue", state=NORMAL)
            go.config(fg="Black", state=DISABLED)

    def done():
        global tray_str, count

        trayList.sort()
        tray_str = "".join([str(i) for i in trayList])
        trayList.clear()
        done.config(fg="Black", state=DISABLED)
        label3['fg'] = "Black"
        label4['fg'] = "Blue"
        # Use a list comprehension to perform the same action on a list of widgets
        [btn.config(fg="Black", state=DISABLED) for btn in traybtn]
        [btn.config(fg="Blue", state=NORMAL) for btn in buttons]
        clear.config(fg="red", state=NORMAL)
        previous.config(fg="red", state=NORMAL)


    def t1(btn_table, x):
        global tray_str, a, count
        table_str = final_str = final_str_1 = ''  # changed on 29-12-2022 13:32--> now all these strings are local
        count = 1
        fg1 = ''
        if len(btList) < 3:
            tableButtonList.append(btn_table)
            go.config(fg="Blue", state=NORMAL)
            label3['fg'] = "Blue"
            label4['fg'] = "Black"

            if len(btList) < 2 and len(trayList1) < 3:
                fg1, state1 = "Blue", NORMAL
            else:
                fg1, state1 = "Black", DISABLED

            [btn.config(fg=fg1, state=state1) for btn in traybtn]
            [btn.config(fg="Black", state=DISABLED) for btn in buttons]

            tray_str = '1' if tray_str == '' else tray_str
            table_str = str(df["TagId"][x])
            final_str = "".join([',', tray_str, ',-1,-1,-1'])
            final_str_1 = "".join(['#', table_str, final_str])

            try:
                strList_data.append(final_str_1)
                btList.append(int(table_str))
            except Exception as e:
                stdout.write("\n\n\n[-->FUT]exception--> functional():t1() list.append, {};".format(e))
                stdout.flush()
                pass
            tray_str = table_str = final_str = final_str_1 = ''
            a += 1

    def tray(x):
        t1 = (Button(frame21, text=x, fg='Blue', bg='light gray', height=2, width=10, state=NORMAL, relief=RAISED))
        t1.pack(side=TOP, padx=25, pady=20)
        t1['command'] = select_frame2(t1, x)
        return t1

    def table(x):
        global x_1, y_1, table_a, table_b, table_i, frist_
        if frist_ == 0:
            x_1 = (width_main/2)-570
            y_1 = 70
            table_a, table_i, table_b = 5, 1, 5
            frist_ = 1
        y = df["TableNumber"][x]
        t3 = (Button(frame22, text=y, fg='Black', bg='light gray', height=2, width=7, state=DISABLED, relief=RAISED))
        t3.place(x=x_1, y=y_1)
        x_1 += 100
        if (x - 1) % table_b == 0 and x != 0:
            y_1 += 70
            x_1 = (width_main / 2) - 570
            table_i += 1
            table_b = table_a * table_i
        t3['command'] = select_frame2(t3, 0, x)
        return t3

    def go():
        try:
            global tray_str, ScreenInputList, ScreenInputFlag
            go['state'] = DISABLED
            tray_str = str1 = ''

            for x in strList_data:
                str1 = x if x == strList_data[0] else str1 + '\\r\\n' + x
            ScreenInputList[0] = str1
            ScreenInputFlag[0] = 1
            stdout.write("\n[-->FUT] functional():go() str1 = {}, ScreenInputList = {}, ScreenInputFlag = {};\n\n".format(str1, ScreenInputList[0], ScreenInputFlag[0]))
            stdout.flush()
            str1 = ''
            btList.clear()
            trayList1.clear()
            trayList.clear()
            strList_data.clear()
            buttons.clear()
            traybtn.clear()
            tableButtonList.clear()
            trayButtonList.clear()

            frame1.destroy()
            frame2.destroy()
            frame3.destroy()

            functional(Top)

        except Exception as e:
            stdout.write("\n\n\n[-->FUT]exception--> functional():go(), {};".format(e))
            stdout.flush()
            pass

    def previous():
        try:
            global tray_str, count

            # check if count is 1 or -1
            if count == 1:
                if len(tableButtonList) > 0:
                    # set the button color and relief for the last item in tableButtonList
                    tableButtonList[-1].config(fg='Blue', bg='light gray', relief=RAISED)
                    
                    # remove the last item from btList and tableButtonList
                    btList.pop()
                    tableButtonList.pop()

                # set the button colors and states for traybtn and buttons
                s2, s1 = NORMAL, DISABLED
                fg2, fg1 = 'Blue', 'Black'
                count = -1

            elif count == -1:
                tray_str = ''

                # remove the last item from the dictionary and trayButtonList
                temp = list(dict.keys())[-1]
                b = dict.pop(temp)
                for i in range(b):
                    if len(trayButtonList) > 0:
                        trayButtonList[-1].config(fg='Blue', bg='light gray', relief=RAISED)
                        trayList1.pop()
                        trayButtonList.pop()

                # set the button colors and states for traybtn and buttons
                s1, s2 = NORMAL, DISABLED
                fg1, fg2 = 'Blue', 'Black'
                count = 1

            # remove the last item from strList_data
            if len(strList_data) > 0: strList_data.pop()

            # check if trayList1 is empty and update the button states accordingly
            if len(trayList1) == 0: 
                previous.config(fg='Black', bg='light gray', state=DISABLED)
                go.config(fg='Black', bg='light gray', state=DISABLED)
                clear.config(fg='Black', bg='light gray', state=DISABLED)

                s1, s2 = NORMAL, DISABLED
                fg1, fg2 = 'Blue', 'Black'

            # set the label colors
            label3['fg'], label4['fg'] = fg1, fg2

            # set the button colors and states for traybtn and buttons
            [btn.config(fg=fg1, state=s1) for btn in traybtn]
            [btn.config(fg=fg2, state=s2) for btn in buttons]

        except Exception as e:
            stdout.write("\n\n\n[-->FUT]functional():previous(), exception={};".format(e))
            stdout.flush()
            pass

    def clear():
        try:
            go['state'] = previous['state'] = clear['state'] = DISABLED

            btList.clear()
            strList_data.clear()
            trayList1.clear()
            trayList.clear()

            label4['fg'], label3['fg'] = "Black", "Blue"

            [btn.config(fg='Blue', bg='light gray', relief=RAISED, state=NORMAL) for btn in traybtn]
            [btn.config(fg='Black', state=DISABLED, bg='light gray', relief=RAISED) for btn in buttons]

            tableButtonList.clear()
            trayButtonList.clear()
        except Exception as e:
            stdout.write("\n\n\n[-->FUT]exception--> finctional():clear(), {};".format(e))
            stdout.flush()
            pass

    def setup_():
        root_create_wcs(setup, label_wcs, tk_image_on, tk_image_off, 1)

    def button_mode():
        global is_on, rootclose
        is_on[0] = not is_on[0] if not rootclose[0] else is_on[0]
        if is_on[0]:
            root_create_wcs(on_, label_wcs, tk_image_on, tk_image_off, 0)
        else:
            on_.config(image=tk_image_off)
            label_wcs.config(fg="Black")
        rootclose[0] = 0

    def btn_clicked_music():
        global musicCount
        musicCount[0] = (musicCount[0] + 1) % 5
        images = [tk_image_mute, tk_image_music, tk_image_birthday, tk_image_celeb, tk_image_repub]
        musicButtonLable.config(image=images[musicCount[0]])

    def motion_while_wcs_call():
        while 1:
            if float(CurrentValue[0]) > 0 or ChargingFlag[0] == 1 or rebootFlag[0]: break
            Top.update()

        frame1.destroy()
        frame2.destroy()
        frame3.destroy()
        if rebootFlag[0]: reboot_UI(Top)
        else:
            if ChargingFlag[0] == 1 and ChargeNode[0] == TagID[0] and float(CurrentValue[0]) < 0: notChargeMode(Top)
            else:
                UISwitchFlag[0] = -1
                charge_main(Top)

    try:
        global ChargingFlag, is_on, musicCount
        if float(CurrentValue[0]) > 0:
            global UISwitchFlag, charge_active
            UISwitchFlag[0] = -1
            charge_main(Top)
        elif ChargingFlag[0] == 1 and ChargeNode[0] == TagID[0] and float(CurrentValue[0]) < 0:
            notChargeMode(Top)
        else:
            ChargingFlag[0] = 0
            global a, count, frist_, x_1, y_1, table_a, table_b, table_i
            a = count = frist_ = x_1 = y_1 = table_a = table_b = table_i = 0
            df = read_csv("./Configurations/TagId.csv")  # ./Configurations/
            width_main = Top.winfo_screenwidth()

            frame1 = Frame(Top, relief=SUNKEN)
            frame2 = Frame(Top, relief=SUNKEN)
            frame3 = Frame(Top, relief=SUNKEN)
            frame1.pack(fill=BOTH, expand=True)
            frame2.pack(fill=BOTH, expand=True)
            frame3.pack(fill=BOTH, expand=True)

            frame21 = Frame(frame2, relief=SUNKEN)
            frame22 = Frame(frame2, relief=SUNKEN)
            frame21.pack(fill=BOTH, expand=True, side=LEFT)
            frame22.pack(fill=BOTH, expand=True, side=RIGHT)

            try:
                img_files = [
                    ("./Img/bat_100.png", (90, 100), 0),
                    ("./Img/bat_80%.png", (70, 90), 30),
                    ("./Img/bat_40%.png", (50, 70), 30),
                    ("./Img/bat_30%.png", (30, 50), 0),
                    ("./Img/bat_15%.png", (0, 30), 25)
                ]

                x1 = percent = 0
                imgFile = ''
                voltage = float(VoltageValue[0])
                top, bottom = 25.3, 19.0

                percent = int(((voltage - bottom) / (top - bottom)) * 100)
                percent = min(100, max(0, percent))  # clamp percent between 0 and 100

                for file, percent_range, x in img_files:
                    if percent_range[0] <= percent <= percent_range[1]:
                        imgFile = file
                        x1 = x
                        break

                image = Image.open(imgFile)
                img = ImageTk.PhotoImage(image)

                label1 = Label(frame1, image=img)
                label1.image = img
                label1.pack(side=TOP, pady=5)
                label1.pack(side=RIGHT, padx=x1)
            except Exception as e:
                stdout.write("\n\n\n[-->FUT]exception--> finctional():battery(), {};".format(e))
                stdout.flush()
                pass
            try:
                on = Image.open("./Img/on.png")
                tk_image_on = ImageTk.PhotoImage(on)

                off = Image.open("./Img/off.png")
                tk_image_off = ImageTk.PhotoImage(off)

                label_wcs = Label(frame1, text="Remote Call", fg='Blue' if is_on[0] else 'Black', font=("Arial", 25))
                label_wcs.pack(side=LEFT, padx=10, pady=25)
                on_ = Button(frame1, state=NORMAL, fg='white', image=tk_image_on if is_on[0] else tk_image_off, compound='center', command=button_mode,  font=(25), bd=0)
                on_.pack(side=LEFT, padx=10, pady=25)

            except Exception as e:
                stdout.write("\n\n\n[-->FUT]exception--> finctional():Toggle, {};".format(e))
                stdout.flush()
                pass
            label3 = Label(frame21, text="Select a Tray", font=('Arial', 20), fg='Blue')
            label3.pack(side=TOP, pady=15)
            traybtn = [tray(i) for i in range(1, 4)]

            done = (Button(frame21, text="DONE", fg='Blue', bg='light gray', height=2, width=20, state=DISABLED, command=done))
            done.pack(side=TOP, padx=25, pady=20)

            label4 = Label(frame22, text="Select a Table", font=('Arial', 20), fg='Black')
            label4.pack(side=TOP, pady=15)
            buttons = [table(i) for i in range(2, len(df["TableNumber"]))]

            go = Button(frame3, text='GO', fg='Black', bg='light gray', command=go, height=3, width=15, font=('Arial', 12), state=DISABLED, activebackground='Sky Blue')
            go.pack(side=RIGHT, pady=10, padx=20)

            clear = Button(frame3, text='CLEAR', fg='Black', bg='light gray', command=clear, height=3, width=15, font=('Arial', 12), state=DISABLED, activebackground='Pink')
            clear.pack(side=LEFT, pady=10, padx=20)

            previous = Button(frame3, text='PREVIOUS', fg='Black', bg='light gray', command=previous, height=3, width=15, font=('Arial', 12), state=DISABLED, activebackground='Pink')
            previous.pack(side=LEFT, pady=10, padx=20)
            # Create a list of image file paths
            image_paths = ["./Img/mute_emoji_1.png", "./Img/music_emoji.png", "./Img/happy_birthday_emoji.png", "./Img/celebration.png", "./Img/repub_emoji.png"]

            # Create a list to store the PhotoImage objects
            images = []

            # Load and resize the images
            for path in image_paths:
                img = Image.open(path)
                images.append(ImageTk.PhotoImage(img))

            # Assign the images to variables
            tk_image_mute, tk_image_music, tk_image_birthday, tk_image_celeb, tk_image_repub = images

            # Create the button and set its initial image
            musicButtonLable = Button(frame3, font=('Arial', 25), state=NORMAL, fg='white', image=images[musicCount[0]], compound='center', bd=0, command=btn_clicked_music)
            musicButtonLable.pack(side=TOP)
            musicButtonLable.pack(side=RIGHT, padx=20)

            Top.after(100, motion_while_wcs_call)

    except Exception as e:
        stdout.write("\n\n\n[-->FUT]exception--> finctional(), {};".format(e))
        stdout.flush()
        pass


def root_create_wcs(on_, label_wcs, tk_image_on, tk_image_off, switchFlag):
    """
    root.

    Parameters
    ----------
    on_ : TYPE
        DESCRIPTION.
    label_wcs : TYPE
        DESCRIPTION.
    tk_image_on : TYPE
        DESCRIPTION.
    tk_image_off : TYPE
        DESCRIPTION.
    switchFlag : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.

    """
    def select_pwd_wcs(btn, btn_num, other_btn_lable, root):
        def select_button():
            btn.config(bg='light Blue', fg='Blue', activebackground='Blue', relief=SUNKEN)
            global btn_lable_num_wcs, btn_num_str_wcs, pwd_nums_wcs, rootclose
            password = '1234'  # min - 4 digit max - 6 digits
            if btn_num <= 9 or other_btn_lable == 0:
                btn_num_str_wcs = 0 if other_btn_lable == 0 else btn_num
                btn_lable_num_wcs = "".join([btn_lable_num_wcs, str(btn_num_str_wcs)])
                pl1['text'] = btn_lable_num_wcs
            else:
                if other_btn_lable == 'enter':
                    for btns in pwd_nums_wcs:
                        btns.config(relief=RAISED, bg='#fcfcff', fg='Black')
                    if btn_lable_num_wcs == password:
                        root.destroy()
                        if switchFlag:
                            on_.config(fg='green', state=NORMAL)
                            for widgets in Top.winfo_children():
                                widgets.destroy()
                            # Installation(Top)
                        else:
                            on_.config(image=tk_image_on, state=NORMAL)
                            label_wcs.config(fg="Blue")

                    else:
                        pl1['text'] = 'incorrect password'
                        if switchFlag: on_.config(fg='black', state=DISABLED)
                        else:
                            on_.config(image=tk_image_off, state=NORMAL)
                            label_wcs.config(fg="Black")
                elif other_btn_lable == 'close':
                    rootclose[0] = 1
                    root.destroy()
                    if switchFlag: on_.config(fg='black', state=DISABLED)
                    else:
                        on_.config(image=tk_image_off, state=NORMAL)
                        label_wcs.config(fg="Black")
                btn_lable_num_wcs = btn_num_str_wcs = ''
        return select_button

    def pwd_btn(root, i):
        global x_stack_wcs, y_stack_wcs, other_btn
        btn_lable = 0
        if i == 1:
            x_stack_wcs = 5
            y_stack_wcs = 70
            other_btn = 0
        if (i - 1) % 3 == 0 and i > 3:
            x_stack_wcs = 5
            y_stack_wcs += 50

        labels = [i if i <= 9 else 'enter' if i == 10 else 0 if i == 11 else 'close' for i in range(1, 13)]
        pb1 = (Button(root, text=labels[i-1], fg='Black', height=2, width=10, state=NORMAL, relief=RAISED))
        pb1.place(x=x_stack_wcs, y=y_stack_wcs)

        x_stack_wcs += 90
        pb1['command'] = select_pwd_wcs(pb1, i, labels[i-1], root)
        return pb1

    try:
        on_['state'] = DISABLED
        global pwd_nums_wcs
        pwd = 'Enter the password'
        root = Tk()
        root.geometry("290x275+10+10")
        root.attributes("-topmost", True)
        root.overrideredirect(True)
        pl1 = Label(root, text=pwd, font=('Arial', 10), fg='gray')
        pl1.pack(side=TOP, pady=10)
        pwd_nums_wcs = [pwd_btn(root, i) for i in range(1, 13)]

    except Exception as e:
        stdout.write("\n\n[-->FUT] exception-->root_create_wcs(), {};".format(e))
        stdout.flush()
        pass


def reboot_UI(Top):
    """
    # updated on 27-01-2023 12:57(yet to implement at KDH).

    Parameters
    ----------
    Top : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    stdout.write("\n[-->FUT] reboot_UI();")
    stdout.flush()

    def reboot():
        stdout.write("\n[-->FUT] reboot_UI():reboot();")
        stdout.flush()
        system("sudo reboot")
    try:
        if rebootFlag[0]:
            ChargingFlag[0] = 0
            mins = 5
            secs = 5 * 60

            height_main = Top.winfo_screenheight()
            width_main = Top.winfo_screenwidth()
            RebootButtonImg = Image.open("./Img/restart_btn.png")
            tk_image = ImageTk.PhotoImage(RebootButtonImg)
            label = Label(Top, image=tk_image)
            label.image = tk_image

            RebootButtonLable = (Button(Top, text='Internal Error Detected, click on the restart to solve the error\n\nrobot will autometically restart in {}:{}\n'.format(mins, secs), fg='Red', font=('Arial', 25), command=reboot, state=NORMAL, image=tk_image, compound='bottom', height=height_main, width=width_main))
            RebootButtonLable.pack(side=TOP)

            # updated on 09-02-2023 11:17
            temp = secs
            while temp > -1:
                mins, secs = divmod(temp, 60)
                RebootButtonLable['text'] = 'Internal Error Detected, click on the restart to solve the error\n\nrobot will autometically restart in {}:{}\n'.format(mins, secs)
                Top.update()
                sleep(1)
                # stdout.write("\n[-->FUT] playPause(), pause(), while ")
                temp -= 1
            stdout.write("\n[-->FUT] reboot_UI():sudo reboot;")
            stdout.flush()
            system("sudo reboot")
    except Exception as e:
        stdout.write("\n[-->FUT]exception--> reboot_UI(), {};".format(e))
        stdout.flush()


def notChargeMode(Top):
    def changeUI():
        global UISwitchFlag
        while float(CurrentValue[0]) < 0:
            if ChargeNode[0] != TagID[0]: break
        labelnotChargeMode.destroy()
        if (ChargeNode[0] != TagID[0] and float(CurrentValue[0]) < 0) or (not ChargingFlag[0]):
            functional(Top)
        else:
            UISwitchFlag[0] = -1
            charge_main(Top)
    try:
        stdout.write("\n[-->FUT] notChargeMode();")
        stdout.flush()

        imgFile = './Img/notChargeMode.png'
        image = Image.open(imgFile)
        resize_image = image.resize((400, 350))
        img = ImageTk.PhotoImage(resize_image)

        labelnotChargeMode = Label(Top, image=img)
        labelnotChargeMode.image = img
        labelnotChargeMode.pack(side=TOP, pady=100)

        Top.after(500, changeUI)
    except Exception as e:
        stdout.write("\n\n\n[-->FUT]exception--> notChargeMode(), {};".format(e))
        stdout.flush()
        pass


def charge_main(Top):
    global UISwitchFlag, ChargingFlag, charge_active
    
    def refresh():
        percent, img = battery()
        str1 = ''.join([str(percent), '%'])
        label4.config(text=str1)
        label3.image = img
        label3.config(image=img)
        Top.update()
        if float(CurrentValue[0]) < 0 or not ChargingFlag[0] or rebootFlag[0]:
            for widgets in Top.winfo_children():
                widgets.destroy()
            UISwitchFlag[0] = 0
            functional(Top)
        else: Top.after(60000, refresh)

    def start():
        charge_active = 1
        if float(CurrentValue[0]) > 0 and float(VoltageValue[0]) >= 20.5:
            HomeButtonLable['state'] = DISABLED
            ChargingFlag[0] = 0
            for widgets in Top.winfo_children():
                widgets.destroy()
            UISwitchFlag[0] = 0
            functional(Top)

    def battery():
        img_files = {
            (90, 100): "./Img/charge_100.png",
            (70, 90): "./Img/charge_80%.png",
            (50, 70): "./Img/charge_50.png",
            (30, 50): "./Img/charge_30.png",
            (10, 30): "./Img/charge_10.png",
            (0, 10): "./Img/charge_0.png"
        }

        voltage = float(VoltageValue[0])
        top, bottom = 25.3, 19.0

        percent = int(((voltage - bottom) / (top - bottom)) * 100)
        percent = min(100, max(0, percent))  # clamp percent between 0 and 100

        for range_percent, file in img_files.items():
            if range_percent[0] <= percent <= range_percent[1]:
                with Image.open(file) as image:
                    resized_img = image if percent < 90 and percent >= 70 else image.resize((290, 190))
                    img = ImageTk.PhotoImage(resized_img)
                    break
        return percent, img

    try:
        if not rebootFlag[0] and float(CurrentValue[0]) > 0:
            ChargingFlag[0], charge_active = 1, 0
            label1 = Label(Top, text="Charging Mode", fg='Blue', font=('Arial', 50))
            label1.pack(side=TOP, padx=120, pady=10)
            str1, img = battery()

            label3 = Label(Top, image=img)
            label3.image = img
            label3.pack(side=TOP)
            label3.pack(side=RIGHT, padx=5)

            HomeButtonImg = Image.open("./Img/black_circle.png")
            resize_image = HomeButtonImg.resize((300, 250))  # (Width,Height)
            tk_image = ImageTk.PhotoImage(resize_image)
            label = Label(Top, image=tk_image)
            label.image = tk_image

            HomeButtonLable = (Button(Top, text='START', font=('Arial', 25), command=start, state=NORMAL, fg='white', image=tk_image, compound='center'))
            HomeButtonLable.pack(side=TOP)
            HomeButtonLable.pack(side=LEFT, padx=100)

            resize_image1 = HomeButtonImg.resize((150, 100))
            tk_image1 = ImageTk.PhotoImage(resize_image1)
            label_1 = Label(Top, image=tk_image1)
            label_1.image = tk_image1

            str1 = ''.join([str(str1), '%'])
            label4 = Label(Top, text=str1, font=('Arial', 30), image=tk_image1, compound='center', fg='white')
            label4.pack(side=TOP)
            label4.pack(side=RIGHT, padx=20)

            Top.after(60000, refresh)

    except Exception as e:
        stdout.write("\n\n\n[-->FUT]exception--> charge_main(), {};".format(e))
        stdout.flush()
        pass


try:
    T_FunUI = [Thread(target=frames, args=())]
except KeyboardInterrupt:
    T_FunUI[0].join()
    pass

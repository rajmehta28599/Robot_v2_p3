"""
Created on Sat Dec 31 16:12:10 2022.

@author: Raj-Mehta
@email : rajmehta28599v@gmail.com

playMusic : is only accept TagID.csv file data.

modified on Thu 2023-Jan-05 17:36:37 by Aanal Patel.
updates-> used simple audio instead of linux command, can also changes according to the UI input.
note: simpleaudio only supports wav files.
"""

from time import sleep, time
from threading import Thread
from sys import stdout, exit
from datetime import datetime
from os import system
from pandas import read_csv
from ApriltagDetectionHandlingThread_1 import InputProcessEnable, TagID
from Functional_UI_asThread import musicCount
from Input_Method_thread import ChargingFlag
from canOpen_ethernet_Thread import CurrentValue
import simpleaudio as sa

playEnable = [0]
current_song = current_song0 = re_current_song0 = 0

df = read_csv("./Configurations/TagId.csv") # ./Configurations/
ChargeNode = df["TagId"][1]

songs = ["./music_thread/music1.wav", "./music_thread/music2.wav", "./music_thread/music3.wav", "./music_thread/music4.wav", "./music_thread/music5.wav",
        "./music_thread/music15.wav", "./music_thread/music6.wav", "./music_thread/music7.wav", "./music_thread/music8.wav", "./music_thread/music9.wav",
        "./music_thread/music10.wav", "./music_thread/music11.wav", "./music_thread/music12.wav", "./music_thread/music13.wav", "./music_thread/music14.wav"]

celeb_songs = ["./music_thread/BirthDay.wav", "./music_thread/Celebration.wav"]

# updated on 24-01-2023 15:25 for 26th jan
repub_songs = ["./music_thread/resong1.wav", "./music_thread/resong2.wav", "./music_thread/resong3.wav", "./music_thread/resong4.wav", "./music_thread/resong5.wav", "./music_thread/resong6.wav", "./music_thread/resong7.wav"]

play_obj = None

def playMusic():
    prevCount = 0
    count = 0
    mode_change = 0
    play_t1 = play_t2 = time()
    global wave_obj, play_obj, current_song, current_song0, musicCount, re_current_song0
    song_play = ''
    while True:
        play_t2 = time() - play_t1
        if play_t2 > 0.01:
            play_t1 = time()
            current_song = musicCount[0]
            try:
                if ((ChargingFlag[0] == 1) and (ChargeNode == TagID[0] or float(CurrentValue[0]) > 0)) and play_obj is not None:
                    play_obj.stop()
                    play_obj = None
                
                elif playEnable[0] and not InputProcessEnable[0]:
                    if play_obj is not None:
                        play_obj.stop()
                        play_obj = None
                    
                    if musicCount[0] == 1 and count == prevCount:                        
                        if current_song0 == len(songs):
                            current_song0 = 0
                        song_play = songs[current_song0]
                        current_song0 += 1
                        mode_change = 1
                        
                    elif musicCount[0] == 4 and count == prevCount: 
                        if re_current_song0 == len(repub_songs): re_current_song0 = 0
                        song_play = repub_songs[re_current_song0]
                        re_current_song0 += 1
                    
                    elif musicCount[0] != 0 and count == prevCount: # updated 23-01-2023 16:18
                        song_play = celeb_songs[0] if musicCount[0] == 2 else celeb_songs[1] if musicCount[0] == 3 else '' # updated on 23-01-2023 12:59
                    
                    else:
                        song_play = ''
                        mode_change = 0
                   
                    if song_play != '' and count == prevCount and musicCount[0] != 0: # updated on 24-01-2023 14:52
                        wave_obj = sa.WaveObject.from_wave_file(song_play)
                        play_obj = wave_obj.play()
                        if musicCount[0] == 1 or musicCount[0] == 4: count += 1
                    
                    # updated on 11-01-2023 19:32 at KDH
                    try:  
                        if play_obj is not None and musicCount[0] == 0:play_obj.stop() # updated on 24-01-2023 14:56
                        elif play_obj is not None:
                            while play_obj.is_playing(): # updated 09 January 2023 19:18
                                if InputProcessEnable[0]:
                                    play_obj.stop()
                                    break
                    except Exception as e:
                        stdout.write("\n\n\n[-->playMT] play_t2={}  while play_obj.is_playing(), exception={},".format(play_t2, e))
                        stdout.flush()
                        pass
                    
                    # code for changing the celeb_songs after one play (yet to test) 

                elif(not playEnable[0] or InputProcessEnable[0]) and play_obj is not None:
                    play_obj.stop()
                    play_obj = None
                    stdout.write("\n[-->playMT] elif (not playEnable[0] or InputProcessEnable[0]) and play_obj is not None: play_t2={},".format(play_t2))
                    stdout.flush()
                
                if InputProcessEnable[0] and count != prevCount and (musicCount[0] == 1 or musicCount[0] == 4):
                    prevCount += 1
                
                if song_play in celeb_songs and mode_change and InputProcessEnable[0]:
                    musicCount[0] = 1
                    song_play = songs[current_song0]
                elif song_play in celeb_songs and not mode_change and InputProcessEnable[0]:
                    song_play = ''
                    musicCount[0] = 0
                
            except Exception as e:
                stdout.write("\n\n\n[-->playMT] play_t2={}, exception={},".format(play_t2, e))
                stdout.flush()
                pass


try:
    T_playMusic = [Thread(target=playMusic)]
except KeyboardInterrupt:
    T_playMusic[0].join()
    pass

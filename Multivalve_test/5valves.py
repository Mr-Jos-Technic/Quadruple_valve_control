#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

from pybricks.iodevices import UARTDevice
from utime import ticks_ms

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# MIT License: Copyright (c) 2022 Mr Jos for the rest of the code

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~QUADRUPLE VALVE CONTROL BY 2 MOTORS~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
##########~~~~~~~~~~~~~EV3 ADVANCED MACHINERY~~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()                                                                    #Name we will be using to make the brick do tasks

#   Motors definition
valve_actuator  = Motor(Port.A)                                                     #Name for the motor that pumps air and opens valves
carriage_motor  = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)      #Name for the motor that moves the carriage
#   Sensor definition
color_top       = ColorSensor(Port.S3)                                              #Name for the color sensor that uses a default black background


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
valve_pos = [115, 440, 765, 1090, 1415]                                             #Position for each valve after homing
pump_pos  = 1580                                                                    #Very safe position for many air pumping


##########~~~~~~~~~~GEARING~~~~~~~~~~##########


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
valve_actuator.control.limits( 900, 3600, 100)                                      #Default     900,  3600, 100
carriage_motor.control.limits( 900, 3600, 100)                                      #Default     900,  3600, 100


##########~~~~~~~~~~MAXIMUM ACCELERATION AND MAXIMUM ANGLE TO SAY A MOVEMENT IS FINISHED~~~~~~~~~~##########
valve_actuator.control.target_tolerances(1000,  2)                                  #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1000, 10)
carriage_motor.control.target_tolerances(1000, 10)                                  #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1000, 10)


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########                #This is not used in this project I use my standard template to program all my projects
#server = BluetoothMailboxServer()
#commands_bt_text = TextMailbox('commands text', server)                            #Main mailbox for sending commands and receiving feedback to/from other brick
#yaw_base_bt_zeroing = NumericMailbox('zero position yaw', server)                  #Mailbox for sending theta1 homing position


##########~~~~~~~~~~CREATING AND STARTING A TIMER, FOR INVERSE KINEMATIC SMOOTH CONTROL~~~~~~~~~~##########
timer_strike  = StopWatch()                                                         #Creating the timer that will be used for calibration


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
pump_fwd    = True                                                                  #Variable to know the last direction the compressor has been running
cursor_pos  = 0                                                                     #Onscreen cursor position
highscore   = 0                                                                     #Highscore value since program start


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')                                    #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)   #Select speaking language, and a voice (male/female)
small_font = Font(size=6)                                                           # 6 pixel height for text on screen
normal_font = Font(size=10)                                                         #10 pixel height for text on screen
big_font = Font(size=16)                                                            #16 pixel height for text on screen
ev3.screen.set_font(normal_font)                                                    #Choose a preset font for writing next texts
ev3.screen.clear()                                                                  #Make the screen empty (all pixels white)
#ev3.speaker.beep()                                                                 #Brick will make a beep sound 1 time
ev3.light.off()                                                                     #Turn the lights off on the brick


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########       #This is used to store your own last calibration values offline, so it will remember them next startup
#os.remove("calibrationdata.txt")                                                   #This is for removing the file we will make next, this is for debugging for me, keep the # in front of it
#create_file = open("calibrationdata.txt", "a")                                     #Create a file if it does not exist and open it, if it does exist, just open it
#create_file.write("")                                                              #Write the default values to the file, for first ever starttup so it holds values
#create_file.close()                                                                #Close the file again, to be able to call it later again

#with open("calibrationdata.txt") as retrieve_data:                                 #Open the offline data file
#    data_retrieval_string = retrieve_data.read().splitlines()                      #The data is in the Type: String , read the complete file line by line
#if len(data_retrieval_string) < 12: data_background_offline = limits_scanned       #Check if there are 12 values in the string list, if not then it is first start of this program ever
#else:                                                                              #If there are 12 then it will convert the String to a Integer list.
#    data_background_offline = []
#    for x in data_retrieval_string:
#        data_background_offline.append(int(x))
#limits_scanned = data_background_offline                                           #The background color is now defined from the offline file (last calibration done)


##########~~~~~~~~~~CREATING FUNCTIONS THAT CAN BE CALLED TO PERFORM REPETITIVE OR SIMULTANEOUS TASKS~~~~~~~~~~##########
#def save_offline_data():                                                           #This definition will save the current background limits to the offline file, if it is called
#    with open("calibrationdata.txt", "w") as backup_data:
#        for current_data in limits_scanned:
#            backup_data.write(str(current_data) + "\n")


def open_valve(direction):
    if direction == "Out":
        valve_actuator.run_target(900,  50, then=Stop.COAST, wait=True)
        wait(400)
        while color_top.color() != None and cursor_pos == 1: continue
        valve_actuator.run_target(900, -20, then=Stop.HOLD, wait=True)
        wait(100)
        valve_actuator.run_target(900,   0, then=Stop.HOLD, wait=True)
    elif direction == "In":
        valve_actuator.run_target(900, -50, then=Stop.COAST, wait=True)
        wait(400)
        while color_top.color() != None and cursor_pos == 1: continue
        valve_actuator.run_target(900,  15, then=Stop.HOLD, wait=True)
        wait(100)
        valve_actuator.run_target(900,   0, then=Stop.HOLD, wait=True)


def pumping_pressure(pos, length):
    global pump_fwd

    if pos == "Safe":
        ev3.light.on(Color.ORANGE) 
        carriage_motor.run_target(900, pump_pos, then=Stop.COAST, wait=True) 
    if pump_fwd == True:
        valve_actuator.run_target(900, length, then=Stop.HOLD, wait=True)
        wait(50)
        valve_actuator.reset_angle(valve_actuator.angle() - length)
        pump_fwd = False
    else:
        valve_actuator.run_target(900, -length, then=Stop.HOLD, wait=True)
        wait(50)
        valve_actuator.reset_angle(valve_actuator.angle() + length)
        pump_fwd = True
    if pos == "Safe": ev3.light.on(Color.GREEN)


def go_to_valve(pos, operation, pump):
    carriage_motor.run_target(900, valve_pos[pos], then=Stop.HOLD, wait=True)
    if   operation == "Out": open_valve("Out")
    elif operation == "In" : open_valve("In")
    elif operation == "In out":
        open_valve("In")
        open_valve("Out")
    elif operation == "Out in":
        open_valve("Out")
        open_valve("In")
    if pump == True:
        carriage_motor.run_target(900, valve_pos[pos]+162, then=Stop.COAST, wait=True)
        pumping_pressure("Local", 1440)


def preprogrammed():
    pumping_pressure("Safe", 7200)                                                  #Go to the safe location with the carriage and do some pre-pumping to build pressure

    for x in range(5):                                                              #Perform the next task 4 times
        go_to_valve(x, "Out", True)                                                 #Move to valve number 'x', extend the cylinder and do some extra pumping
    go_to_valve(2, "In out", True)                                                  #Move to valve number 3, retract and extend again, then do some extra pumping
    for x in range(5):
        go_to_valve(x, "In", False)                                                 #Move to all 4 valves and retract the cylinders, without pumping
    ev3.light.off()


def sensor_control():
    global cursor_pos 
    pumping_pressure("Safe", 7200)

    while True:                                                                     #Start the loop that checks the visible color on the color sensor
        if color_top.color() == Color.GREEN:                                        #If it sees green
            go_to_valve(0, "Out", False)                                            #Move the carriage to the first valve, turn it to extend the cylinder and don't go extra pumping
            while color_top.color() != None: continue                               #Wait for all colors to be away from the sensor
            go_to_valve(0, "In", True)                                              #Make the first cylinder retract again and do some extra pumping
        elif color_top.color() == Color.YELLOW:
            go_to_valve(1, "Out", False)
            while color_top.color() != None: continue
            go_to_valve(1, "In", True)
        elif color_top.color() == Color.RED:
            go_to_valve(2, "Out", False)
            while color_top.color() != None: continue
            go_to_valve(2, "In", True)
        elif color_top.color() == Color.BLUE:
            go_to_valve(3, "Out", False)
            while color_top.color() != None: continue
            go_to_valve(3, "In", True)
        elif color_top.color() == Color.BLACK:
            go_to_valve(4, "Out", False)
            while color_top.color() != None: continue
            go_to_valve(4, "In", True)
        elif color_top.color() == Color.WHITE:
            pumping_pressure("Safe", 7200)
            while color_top.color() != None: continue
        elif ev3.buttons.pressed() == [Button.DOWN]:
            cursor_pos += 1
            ev3.light.off()
            break
        elif ev3.buttons.pressed() == [Button.UP]:
            cursor_pos -= 1
            ev3.light.off()
            break
        else: continue


def whack_a_mole():
    global highscore
    onscreen_counter_line = "{}: {} {} "                                            #Create a text line with 3 blank spots, to be filled in later
    score = 0                                                                       #Set the score to 0 points
    strikeout = 1000                                                                #ms time for showing the correct color
    
    ev3.speaker.say("Building pressure")
    ev3.screen.draw_text(4, 48, "Pre pumping air pressure              ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
    #pumping_pressure("Safe", 14400)
    #pumping_pressure("Safe", 14400)
    ev3.screen.draw_text(4, 48, "Air pressure ok, game started         ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
    ev3.light.off()
    ev3.speaker.say("Game starting, show the correct color!")
    while True:
        next_valve = choice([0,1,2,3,4])
        go_to_valve(next_valve, "Out", False)
        timer_strike.reset()
        timer_strike.resume()
        ev3.speaker.beep(frequency=10, duration=1000)
        while timer_strike.time() < strikeout:
            whack_clr = color_top.color()
            if (next_valve == 0 and whack_clr == Color.GREEN) or (next_valve == 1 and whack_clr == Color.YELLOW) or (next_valve == 2 and whack_clr == Color.RED) or (next_valve == 3 and whack_clr == Color.BLUE) or (next_valve == 4 and whack_clr == Color.BLACK):
                timer_strike.pause()
                break
        #print(timer_strike.time())
        if timer_strike.time() >= strikeout:
            ev3.screen.draw_text(4, 48, "GAME OVER                                        ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            ev3.light.on(Color.RED)
            ev3.screen.draw_text(4, 59, "                                                                  ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            if score > highscore:
                highscore = score
                ev3.screen.draw_text(4, 70, onscreen_counter_line.format("Highscore: ", int(highscore), "!"), text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            ev3.speaker.say("Game over!")
            break
        score += 1
        ev3.screen.draw_text(4, 59, onscreen_counter_line.format("Correct hits:", int(score), "times   "), text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
        ev3.light.on(Color.GREEN)
        if score < 3: ev3.speaker.say("Correct!")
        go_to_valve(next_valve, "In", False)


        if math.fmod(score, 10) == 0:                                               #After scoring 10points, build up more air pressure
            ev3.screen.draw_text(4, 48, "Extra pumping air pressure                                       ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            pumping_pressure("Safe", 14400)
            pumping_pressure("Safe", 14400)
            ev3.light.off()
            ev3.screen.draw_text(4, 48, "Air pressure ok game continues faster         ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            if strikeout > 200: strikeout -= 200                                    #200ms less time each 10points, until minimal 200ms
        ev3.light.off()


def pushingbuttons():                                                               #Function to wait for a button to be pressed on the EV3 brick, and return which one was pressed
    while True:
        if   ev3.buttons.pressed() == [Button.UP]:
            wait_for_release_buttons()
            return "up"
        if ev3.buttons.pressed() == [Button.DOWN]:
            wait_for_release_buttons()
            return "down"
        elif ev3.buttons.pressed() == [Button.LEFT]:
            wait_for_release_buttons()
            return "left"
        elif ev3.buttons.pressed() == [Button.RIGHT]:
            wait_for_release_buttons()
            return "right"
        elif ev3.buttons.pressed() == [Button.CENTER]:
            wait_for_release_buttons()
            return "center"


def wait_for_release_buttons():                                                     #Function to wait for the EV3 buttons to be all released
    while ev3.buttons.pressed() != []: continue


def draw_text_lines_menu(selected):                                                 #Function to color the selected line in the menu
    if   selected == 0:
        ev3.screen.draw_text(4,  4, "Start the preprogrammed routine", text_color=Color.WHITE, background_color=Color.BLACK)    #Cursor pos 0 selected background showing on screen
        ev3.screen.draw_text(4, 15, "Start the color sensor control", text_color=Color.BLACK, background_color=Color.WHITE)     #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Start the whack a mole game", text_color=Color.BLACK, background_color=Color.WHITE)        #Cursor pos 2
    elif selected == 1:
        ev3.screen.draw_text(4,  4, "Start the preprogrammed routine", text_color=Color.BLACK, background_color=Color.WHITE)    #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Start the color sensor control", text_color=Color.WHITE, background_color=Color.BLACK)     #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Start the whack a mole game", text_color=Color.BLACK, background_color=Color.WHITE)        #Cursor pos 2
    elif selected == 2:
        ev3.screen.draw_text(4,  4, "Start the preprogrammed routine", text_color=Color.BLACK, background_color=Color.WHITE)    #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Start the color sensor control", text_color=Color.BLACK, background_color=Color.WHITE)     #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Start the whack a mole game", text_color=Color.WHITE, background_color=Color.BLACK)        #Cursor pos 2
    else:                                                                           #Nothing selected, sub program running
        ev3.screen.draw_text(4,  4, "Start the preprogrammed routine", text_color=Color.BLACK, background_color=Color.WHITE)    #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Start the color sensor control", text_color=Color.BLACK, background_color=Color.WHITE)     #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Start the whack a mole game", text_color=Color.BLACK, background_color=Color.WHITE)        #Cursor pos 2


def clear_screen():
    ev3.screen.clear()                                                              #Empty the complete screen on the EV3 brick
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)     #Write text on the EV3 screen on the XY grid

    
##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
#sub_white_scanner = Thread(target=check_color_white)                                #Creating a multithread so the definition can run at the same time as the main program, if it's called


##########~~~~~~~~~~MAIN PROGRAM~~~~~~~~~~##########
clear_screen()

carriage_motor.run_until_stalled(-300, then=Stop.COAST, duty_limit=30)              #Start to run the carriage motor with low power, until it stalls
wait(250)                                                                           #Wait for the tension to relax
carriage_motor.reset_angle(0)                                                       #Set the current motor angle as 0 (Homing position)
carriage_motor.run_target(900, valve_pos[0], then=Stop.COAST, wait=True)            #Move to the center of the first valve = [0]


while True:
    draw_text_lines_menu(cursor_pos)                                                #Show on screen the selected mode currently
    lastpress = pushingbuttons()
    if   lastpress == "center":                                                     #If the last button press was the center button;
        if   cursor_pos == 0: preprogrammed()                                       #Start the definition that has a routine set
        elif cursor_pos == 1: sensor_control()                                      #Start the routine that allows you to manual move cylinders by showing a color to the sensor
        elif cursor_pos == 2: whack_a_mole()                                        #Start the routine that allows you to play whack a mole!
    elif lastpress == "down" and cursor_pos < 2: cursor_pos += 1                    #Move the cursor position one line down
    elif lastpress == "up"   and cursor_pos > 0: cursor_pos -= 1                    #Move the cursor position one line up


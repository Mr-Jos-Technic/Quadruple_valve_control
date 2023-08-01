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
pump_pos  = 277                                                                     #Very safe position for many air pumping


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
timer_strike  = StopWatch()                                                         #Creating the timer that will be used for gametime


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
valve_open_time  = 400                                                              #Time a valve needs to stay open before closing again
valve_open_angle =  50                                                              #Angle for the actuator to open a valve completely
pump_fwd    = True                                                                  #Variable to know the last direction the compressor has been running
cursor_pos  = 0                                                                     #Onscreen cursor position
highscore   = 0                                                                     #Highscore value since program start
counters    = [0, 0, 0, 0, 0]                                                       #Counters for amount of times extending a cylinder

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


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########       #This is used to store your counters, so it will remember them next startup
#os.remove("counterdata.txt")                                                       #This is for removing the file we will make next, this is for debugging for me, keep the # in front of it
#create_file = open("counterdata.txt", "a")                                         #Create a file if it does not exist and open it, if it does exist, just open it
#create_file.write("")                                                              #Write the default values to the file, for first ever starttup so it holds values
#create_file.close()                                                                #Close the file again, to be able to call it later again

#with open("counterdata.txt") as retrieve_data:                                     #Open the offline data file
#    data_retrieval_string = retrieve_data.read().splitlines()                      #The data is in the Type: String , read the complete file line by line
#if len(data_retrieval_string) < 5: data_background_offline = counters              #Check if there are 5 values in the string list, if not then it is first start of this program ever
#else:                                                                              #If there are 5 then it will convert the String to a Integer list.
#    data_background_offline = []
#    for x in data_retrieval_string:
#        data_background_offline.append(int(x))
#counters = data_background_offline                                                 #The counters are now defined from the offline file (last running)


##########~~~~~~~~~~CREATING FUNCTIONS THAT CAN BE CALLED TO PERFORM REPETITIVE OR SIMULTANEOUS TASKS~~~~~~~~~~##########
#def save_offline_data():                                                           #This definition will save the current counter values to the offline file, if it is called
#    with open("counterdata.txt", "w") as backup_data:
#        for current_data in counters:
#            backup_data.write(str(current_data) + "\n")


def open_valve(direction):                                                          #Definition to be called to open 1 valve, the direction is given
    if direction == "Out":                                                          #If the given is extending out the cylinder
        valve_actuator.run_target(900,  valve_open_angle, then=Stop.COAST, wait=True)   #Turn the lever 50° with a coast ending, so there's no stress on the motor
        wait(valve_open_time)                                                       #Wait a certain time to allow the cylinder to extend completely
        while color_top.color() != None and cursor_pos == 1: continue               #In play mode, wait for retracting until the color is away from the sensor
        valve_actuator.run_target(900, -20, then=Stop.HOLD, wait=True)              #Run the actuator over center back to put the lever in center position
        #wait(100)                                                                  #TODO check if it keeps working fine without this wait block
        valve_actuator.run_target(900,   0, then=Stop.HOLD, wait=True)              #Align the actuator back to center, no more tension on the lever now
    elif direction == "In":                                                         #If the given is retracting the cylinder
        valve_actuator.run_target(900, -valve_open_angle, then=Stop.COAST, wait=True)
        wait(valve_open_time)
        while color_top.color() != None and cursor_pos == 1: continue
        valve_actuator.run_target(900,  15, then=Stop.HOLD, wait=True)
        #wait(100)                                                                  #TODO check need
        valve_actuator.run_target(900,   0, then=Stop.HOLD, wait=True)


def pumping_pressure(pos, length):                                                  #Definition to pre-pressurize the system, or pump a little extra
    global pump_fwd                                                                 #Global variable to check the next direction to turn

    if pos == "Safe":                                                               #Most safe position to pressurize a long time (near the motor)
        ev3.light.on(Color.ORANGE)                                                  #Illuminate the Red+Green LED (to make Orange)
        carriage_motor.run_target(900, pump_pos, then=Stop.COAST, wait=True)        #Make the carriage go to a safe spot and let it coast (if it would hit anything during pumping, it will just move)
    if pump_fwd == True:                                                            #If the next direction to pump is forward
        valve_actuator.run_target(900, length, then=Stop.HOLD, wait=True)           #Run the compressor for a given duration (Only run in increments of 360°!! to keep the actuator flat, so it passes valves)
        wait(50)                                                                    #Wait for the motor to stand completely still (so the encoder value will not change anymore)
        valve_actuator.reset_angle(valve_actuator.angle() - length)                 #Remove the length turned from the encoder value, so any deviation remains.
        pump_fwd = False                                                            #Overwrite the next direction to turn
    else:
        valve_actuator.run_target(900, -length, then=Stop.HOLD, wait=True)
        wait(50)
        valve_actuator.reset_angle(valve_actuator.angle() + length)
        pump_fwd = True
    if pos == "Safe": ev3.light.on(Color.GREEN)                                     #If it was pumping in the safe spot, with orange light on, make it now green


def go_to_valve(pos, operation, pump):                                              #Definiton to make a complete operation of the valve incl extra pumping
    carriage_motor.run_target(900, valve_pos[pos], then=Stop.HOLD, wait=True)       #Make the carriage go to the desired valve location
    if   operation == "Out": open_valve("Out")                                      #If the operation is extending  the cylinder, run that definition
    elif operation == "In" : open_valve("In")                                       #If the operation is retracting the cylinder, run that definition
    elif operation == "In out":                                                     #If the operation is retract and direct extending, run both definition
        open_valve("In")
        open_valve("Out")
    elif operation == "Out in":
        open_valve("Out")
        open_valve("In")
    if pump == True:                                                                #If extra pumping is required
        carriage_motor.run_target(900, valve_pos[pos]+162, then=Stop.COAST, wait=True)  #Move right next to the current valve
        pumping_pressure("Local", 1440)                                             #Pump for 4 rotations (Only run in increments of 360°!! to keep the actuator flat, so it passes valves)


def preprogrammed():                                                                #Definition with some preset valve operations (menu cursor position 1)
    pumping_pressure("Safe", 7200)                                                  #Go to the safe location with the carriage and do some pre-pumping to build pressure

    for x in range(5):                                                              #Perform the next task 4 times
        go_to_valve(x, "Out", True)                                                 #Move to valve number 'x', extend the cylinder and do some extra pumping
    go_to_valve(2, "In out", True)                                                  #Move to valve number 3, retract and extend again, then do some extra pumping
    for x in range(5):
        go_to_valve(x, "In", False)                                                 #Move to all 4 valves and retract the cylinders, without pumping
    ev3.light.off()


def sensor_control():                                                               #Definition to control the valves by showing colors to the color sensor
    global cursor_pos                                                               #Use the global variable in this local area
    pumping_pressure("Safe", 7200)                                                  #Start with pre-pressurizing

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
        elif color_top.color() == Color.BROWN:
            go_to_valve(4, "Out", False)
            while color_top.color() != None: continue
            go_to_valve(4, "In", True)
        elif color_top.color() == Color.WHITE:
            pumping_pressure("Safe", 7200)
            while color_top.color() != None: continue
        elif ev3.buttons.pressed() == [Button.DOWN]:                                #If you press the down button on the EV3
            cursor_pos += 1                                                         #Make the cursor go down by 1
            ev3.light.off()                                                         #Turn the LED's off
            break                                                                   #Close this definition
        elif ev3.buttons.pressed() == [Button.UP]:
            cursor_pos -= 1
            ev3.light.off()
            break
        else: continue                                                              #Restart this loop


def whack_a_mole():                                                                 #Definition to play a game of whack a mole
    global highscore                                                                #Use the global variable in this local area
    onscreen_counter_line = "{}: {} {} "                                            #Create a text line with 3 blank spots, to be filled in later
    score = 0                                                                       #Set the score to 0 points
    strikeout = 1000                                                                #ms time you have for showing the correct color
    
    ev3.speaker.say("Building pressure")                                            #Make the EV3 speak
    ev3.screen.draw_text(4, 48, "Pre pumping air pressure              ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen
    pumping_pressure("Safe", 14400)                                                 #Start with pre-pressurizing
    pumping_pressure("Safe", 14400)                                                 #Continue with pre-pressurizing, it will be in the other direction now
    ev3.screen.draw_text(4, 48, "Air pressure ok, game started         ", text_color=Color.BLACK, background_color=Color.WHITE)
    ev3.light.off()                                                                 #Turn the LED's off
    ev3.speaker.say("Game starting, show the correct color!")
    while True:                                                                     #Start a forever loop
        next_valve = choice([0,1,2,3,4])                                            #Randomly choose between the 5 valves
        go_to_valve(next_valve, "Out", False)                                       #Run the definition to extend the cylinder
        timer_strike.reset()                                                        #Put the timer back to 0
        timer_strike.resume()                                                       #Restart the timer
        while timer_strike.time() < strikeout:                                      #Whilst the timer is under the strikeout time
            whack_clr = color_top.color()                                           #Check the color in front of the color sensor
            if (next_valve == 0 and whack_clr == Color.GREEN) or (next_valve == 1 and whack_clr == Color.YELLOW) or (next_valve == 2 and whack_clr == Color.RED) or (next_valve == 3 and whack_clr == Color.BLUE) or (next_valve == 4 and whack_clr == Color.BROWN):
                timer_strike.pause()                                                #If it matches the random chosen valve, stop the timer
                break                                                               #Break out of this loop
        if timer_strike.time() >= strikeout:                                        #Check if the player was to late
            ev3.screen.draw_text(4, 48, "GAME OVER                                        ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            ev3.light.on(Color.RED)                                                 #Turn the red LED on
            ev3.screen.draw_text(4, 59, "                                                                  ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            if score > highscore:                                                   #Check if the current score is higher than the highscore
                highscore = score                                                   #If it is, overwrite the highscore
                ev3.screen.draw_text(4, 70, onscreen_counter_line.format("Highscore: ", int(highscore), "!"), text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            ev3.speaker.say("Game over!")                                           #Make the EV3 say "Game over"
            break                                                                   #Stop the main loop, running this game
        score += 1                                                                  #If he was in time, add a scorepoint
        ev3.screen.draw_text(4, 59, onscreen_counter_line.format("Correct hits:", int(score), "times   "), text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
        ev3.light.on(Color.GREEN)                                                   #Turn the green LED on
        if score =< 2: ev3.speaker.say("Correct!")                                  #The EV3 will call out a correct answer for the first 2 points
        go_to_valve(next_valve, "In", False)                                        #Move the current extended cylinder back in

        if math.fmod(score, 10) == 0:                                               #After scoring 10points, build up more air pressure
            ev3.screen.draw_text(4, 48, "Extra pumping air pressure                                       ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            pumping_pressure("Safe", 14400)
            pumping_pressure("Safe", 14400)
            ev3.light.off()
            ev3.screen.draw_text(4, 48, "Air pressure ok game continues faster         ", text_color=Color.BLACK, background_color=Color.WHITE) #This will write on the EV3 screen the scorepoints
            if strikeout > 200: strikeout -= 200                                    #200ms less time each 10points scored, until minimal 200ms
        ev3.light.off()


def pushingbuttons():                                                               #Function to wait for a button to be pressed on the EV3 brick, and return which one was pressed
    while True:                                                                     #Start a forever loop
        if   ev3.buttons.pressed() == [Button.UP]:                                  #If only the up button is currently pressed
            wait_for_release_buttons()                                              #Start the definition that waits for all buttons to be released (to prevent double tapping)
            return "up"                                                             #Answer the definition call with up
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


def clear_screen():                                                                 #Definition to clear everything from the screen
    ev3.screen.clear()                                                              #Empty the complete screen on the EV3 brick
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)     #Write text on the EV3 screen on the XY grid

    
##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########                       #Not used in this program
#sub_white_scanner = Thread(target=check_color_white)                               #Creating a multithread so the definition can run at the same time as the main program, if it's called


##########~~~~~~~~~~MAIN PROGRAM~~~~~~~~~~##########
clear_screen()

carriage_motor.run_until_stalled(-300, then=Stop.COAST, duty_limit=30)              #Start to run the carriage motor with low power, until it stalls
wait(250)                                                                           #Wait for the tension to relax
carriage_motor.reset_angle(0)                                                       #Set the current motor angle as 0 (Homing position)
carriage_motor.run_target(900, valve_pos[0], then=Stop.COAST, wait=True)            #Move to the center of the first valve = [0]


while True:                                                                         #Start a forever loop
    draw_text_lines_menu(cursor_pos)                                                #Show on screen the selected mode currently
    lastpress = pushingbuttons()
    if   lastpress == "center":                                                     #If the last button press was the center button;
        if   cursor_pos == 0: preprogrammed()                                       #Start the definition that has a routine set
        elif cursor_pos == 1: sensor_control()                                      #Start the routine that allows you to manual move cylinders by showing a color to the sensor
        elif cursor_pos == 2: whack_a_mole()                                        #Start the routine that allows you to play whack a mole!
    elif lastpress == "down" and cursor_pos < 2: cursor_pos += 1                    #Move the cursor position one line down
    elif lastpress == "up"   and cursor_pos > 0: cursor_pos -= 1                    #Move the cursor position one line up


#!/usr/bin/python
#from dronekit_sitl import SITL
# Import DroneKit-Python
from dronekit import connect, VehicleMode, CommandSequence
from tkinter import *
import time, sys, struct, os, math, csv, random, tkFileDialog
try:
    import thread
except ImportError:
    import _thread as thread

#----------globals----------
#TK panes which need to be updated
updatePanes = None
info_readout = None  
fault_frame = None  
mission_readout = None
popup_frame = None

#dronekit vehicle
vehicle = None
#TK window root 
root = None
#bool which denotes whether you are connected to SITL/Mavproxy or not
connected = False
#used to ensure two wind gusts don't go off at once
is_gusting = False

#wind vector components - get added together and sent to the simulator every so often
const_wind_x = 0
const_wind_y = 0
const_wind_z = 0

#the actual representations of the wind in the code - two angles and a magnitude (m/s)
gust_wind_spd = 0
gust_wind_dir = 0
gust_wind_pitch = 0

#current build the z wind is called 'pitch', live its 'dir_z'
Z_WIND_NAME = ''

#turbulence
wind_turb = 0

#Button Activity
gcsfs = bfs = tfs = rfs = gpsfs = 'Inactive'

#waypoint list, actual is the real list, custom is being modified.
#if waypoints_been_modified, then custom will be sent to MAV, and actual stored.
#once wps reset to normal, 
mission_actual = None
mission_custom = None
waypoints_been_modified = False
#stores custom waypoint information, controlled with waypoint_listener()
waypoint_info = []
#global var, mission listbox
mission_readout = None

#----------Vehicle Connections----------

#connects to a drone sitting at ip:port and dispatches a thread to display it's
#information to the readout window
def connect_drone(dkip, dkport):
    #Dronekit connection
    global vehicle
    #connect to vehicle at ip:port
    print('Attempting To Connect to MAVLink')
    global info_readout
    update_readout_window(info_readout, 'Attempting To Connect to MAVLink...')
    try:
        vehicle = connect(dkip+':'+dkport, wait_ready=True)
    except:
        print('No drone found!')
        update_readout_window(info_readout, 'Unable to connect!')
        def wait_then_clear(txtbox):
            time.sleep(4)
            global connected
            if not connected:
                update_readout_window(txtbox, 'Use the connect button to connect to a drone!')
        thread.start_new_thread(wait_then_clear, (info_readout,))
        return

    global connected 
    connected = True
    
    global Z_WIND_NAME
    try:
        vehicles.parameters['WIND_DIR_Z']
    except Exception:
        Z_WIND_NAME = 'SIM_WIND_PITCH'
    else:
        Z_WIND_NAME = 'SIM_WIND_DIR_Z'
    
    #initialize globals associated with the vehicle
    #Throttle PWM
    global THR_FS_VAL
    vehicle.parameters['THR_FS_VALUE'] = float(0)
    THR_FS_VAL = vehicle.parameters['THR_FS_VALUE']
    #Battery capacity
    global FS_BATT_MAH
    vehicle.parameters['FS_BATT_MAH'] = float(0)
    FS_BATT_MAH = vehicle.parameters['FS_BATT_MAH'] 
    #ID of the ground control station
    global SYSID_MYGCS_DEF
    SYSID_MYGCS_DEF = vehicle.parameters['SYSID_MYGCS']
    #max radius of the waypoints, stored for close
    global WP_RADIUS
    WP_RADIUS = vehicle.parameters['WP_RADIUS']
    
    # create thread to update readout information in real time
    thread.start_new_thread(update_vehicle_status, (vehicle,))
    
    #create thread to feed wind info to the sim
    thread.start_new_thread(send_wind_params, ())
    
    #read the mission to memory, display it
    thread.start_new_thread(grab_mission, ())

#disconnects the vehicle and cleans the readout
def disconnect():
    global connected
    if connected:
        #close vehicle connection
        reset()
        vehicle.close()
        global info_readout
        update_readout_window(info_readout, 'Disconnected')
        connected = False
    else:
        print('Not connected to any Drone!')

#grabs the mission from MAVProxy, and updates the display.
def grab_mission():
    global vehicle
    if connected:
        mission = vehicle.commands
        mission.download()
        #download is async, so wait for it to finish
        mission.wait_ready()
        mission_readout.delete(0,END)
        command_list = []
        for cmd in mission:
            command_list.append(cmd)
        global mission_actual, mission_custom, waypoints_been_modified
        if waypoints_been_modified:
            mission_custom = command_list
        else:
            mission_actual = command_list
        print('Misssion Grabbed')
        update_mission_readout()
    
#sends list of commands to vehicle (same type as the return from update_mission_readout())
def send_mission():
    global vehicle
    global mission_actual, mission_custom, waypoints_been_modified
    if connected:
        if waypoints_been_modified:
            cmd_list = mission_custom
        else:
            cmd_list = mission_actual
        print('Current command list:')
        for cmd in cmd_list:
            print(str(cmd.x)+', '+str(cmd.y))
        #mission = CommandSequence(vehicle)
        mission = vehicle.commands
        mission.download()
        mission.wait_ready()
        mission.clear()
        print('Updated List:')
        for cmd in cmd_list:
            mission.add(cmd)
        try:
            mission.upload()
        except:
            print('Err. sending of commands to MAVLink failed')
            return False
        return True

#----------TK pane updaters----------

#continually updates the readout with vehicle information
def update_vehicle_status(vehicle):
    while(connected):
        #update the readout with vehicle information  
        updateText = '';
        updateText += ('System Status: %s'     % vehicle.system_status.state + 
                       '\nLast Heartbeat: %s'     %vehicle.last_heartbeat + 
                       '\nMode: %s'                 % vehicle.mode.name +
                       '\nIs Armable?: %s'         % vehicle.is_armable + '\n')
        #Add battery/location/environment info
        updateText += ('\nBattery Capacity: %s MAH' % vehicle.parameters['BATT_CAPACITY'] +
                       '\nGPS Info: %s'             % vehicle.gps_0 + 
                       '\nLattitude: %s '             % vehicle.location.global_relative_frame.lat +
                       '\nLongitude: %s'             % vehicle.location.global_relative_frame.lon +
                       '\nAirspeed: %s'             % vehicle.velocity +
                       '\nAltitude: %s'             % vehicle.location.global_relative_frame.alt +
                       '\nWind Speed: %s'             % vehicle.parameters['SIM_WIND_SPD'] +
                       '\nWind Direction: %s'         % vehicle.parameters['SIM_WIND_DIR'] +
                       '\nWind Pitch: %s'             % vehicle.parameters[Z_WIND_NAME] + 
                       '\nWind Turbulence: %s\n'     % vehicle.parameters['SIM_WIND_TURB'])
        #add failsafes

        updateText += (    '\nGPS Failsafe:      ' + gpsfs +
                        '\nRadio Failsafe:    ' + rfs +
                        '\nThrottle Failsafe: ' + tfs +
                        '\nBattery Failsafe:  ' + bfs +
                        '\nGCS Failsafe:      ' + gcsfs) 

        #update the readout
        global info_readout
        update_readout_window(info_readout, updateText)
        #update root
        root.update()
        #wait for 1 second
        time.sleep(1)

#helper function to write information to the readout
def update_readout_window(textWindow, text):
    #sets the readout window from read only to read/write
    textWindow.config(state=NORMAL)
    #clears the readout window
    textWindow.delete('1.0', END)
    #inserts text into readout window
    textWindow.insert(END, text)
    #sets window back to read only 
    textWindow.config(state=DISABLED)

#name is self explanitory
def update_mission_readout(new_selected=-1):
    global waypoints_been_modified, mission_custom, mission_actual
    if connected:
        #choose the correct wp list
        if waypoints_been_modified:
            cmds = mission_custom
        else:
            cmds = mission_actual
        global mission_readout
        mission_readout.delete(0,END)
        command_list = []
        for x, cmd in enumerate(cmds):
            mission_readout.insert(END,str(x+1) + ': ' + command_key_to_name(cmd.command) + ', ' +str(cmd.x) + ', ' + str(cmd.y) + ', ' + str(cmd.z) + '\n')
            command_list.append(cmd)     
        if new_selected != -1:
            mission_readout.selection_set(new_selected)
    else:
        print('Tried to read wps, but not connected.')

#----------Waypoint functions----------

#removes a wp from the list -- its still stored in memory, to get reset when FI is closed/disconnected
def delete_wp(selected):
    global mission_actual, mission_custom, waypoints_been_modified, waypoint_info
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    mission_custom.pop(selected-1)
    waypoint_info = []
    update_mission_readout()
    send_mission()
    print('Waypoint ' + str(selected) + ' deleted.')

#randomly moves a wp around -this sucks right now  
def vary_wp(selected, mean, stddev=-1):
    if stddev == -1:
        stddev = math.sqrt(mean)
    global mission_actual, mission_custom, waypoints_been_modified
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True 
    mag = random.normalvariate(mean, stddev)
    theta = random.randint(0,359)
    mission_custom[selected-1].x += mag*math.cos(math.radians(theta)) #actually latitude
    mission_custom[selected-1].y += mag*math.sin(math.radians(theta)) #actually longitude
    print('Waypoint ' + str(selected) + ' changed to ' + str((mission_custom[selected-1].x, mission_custom[selected-1].y)))
    send_mission()
    update_mission_readout()

#modifies a waypoints lat/long using a given change in position (in miles)
def change_wp_loc(selected, x_change, y_change):
    global mission_actual, mission_custom, waypoints_been_modified
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True 
    old_x = mission_custom[selected-1].x
    old_y = mission_custom[selected-1].y
    lat_change,long_change = miles_to_coords(x_change, y_change, mission_custom[selected-1].y)
    print((lat_change, long_change))
    mission_custom[selected-1].x += long_change
    mission_custom[selected-1].y += lat_change
    
    print('Waypoint ' + str(selected) + ': ' + str((old_x, old_y)) + ' changed to ' + str((mission_custom[selected-1].x, mission_custom[selected-1].y)))
    send_mission()
    update_mission_readout()

#sets a flag to skip a waypoint as soon as the vehicle starts towards it.
def skip_wp(selected):
    global mission_actual, mission_custom, waypoints_been_modified
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    global waypoint_info
    found = False
    for cmd in waypoint_info:
        if cmd[0] == selected-1:
            if cmd[1] is None:
                cmd[1] = True
            else:
                cmd[1] = not cmd[1]
            found = True
            break
    if not found:
        #if no entries into wp_info, make a new one and start the listener thread
        new_wp_info(selected-1, skip=True)
        thread.start_new_thread(waypoint_listener, ())

#sets the coordinates of a specified waypoint        
def set_wp(selected, lat, lon):
    if connected:
        global mission_actual, mission_custom, waypoints_been_modified
        if not waypoints_been_modified:
            mission_custom = mission_actual[:]
            waypoints_been_modified = True
        mission_custom[selected-1].x = lat
        mission_custom[selected-1].y = lon
        send_mission()
        update_mission_readout()
        print('Waypoint ' + str(selected-1) + ' set to ' + str((mission_custom[selected-1].x, mission_custom[selected-1].y)))
        
#Edit flyby radius (doesnt technically deal with waypoints)
def edit_flyby_wp(selected, radius):
    global mission_actual, mission_custom, waypoints_been_modified
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    global waypoint_info
    found = False
    for cmd in waypoint_info:
        if cmd[0] == selected-1:
            cmd[2] = radius
            found = True
            break
    if not found:
        #if no entries into wp_info, make a new one and start the listener thread
        new_wp_info(selected-1, radius=radius)
        thread.start_new_thread(waypoint_listener, ())
    print( 'WP_RADIUS set to ' + str(radius) + ' for wp #' + str(selected-1))

#moves the selected wp up 1 (backwards in the sequence)  
def repo_up_wp(selected):
    global mission_actual, mission_custom, waypoints_been_modified, waypoint_info
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    if selected-1 == 0:
        print('Error: cant move the top one down!')
        return
    temp = mission_custom[selected-2]
    mission_custom[selected-2] = mission_custom[selected-1]
    mission_custom[selected-1]   = temp
    if waypoint_info:
        for wp in waypoint_info:
            if wp[0] == selected:
                wp[0] -= 1
    send_mission()
    update_mission_readout(selected-2)
    print('Waypoint ' + str(selected) + ' Moved up')

#moves the selected wp down 1 (forwards in the sequence)
def repo_down_wp(selected):
    global mission_actual, mission_custom, waypoints_been_modified, waypoint_info
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    if selected-1 == len(mission_custom)-1:
        #cant move the bottom one down!
        return
    temp = mission_custom[selected]
    mission_custom[selected] = mission_custom[selected-1]
    mission_custom[selected-1]   = temp
    if waypoint_info:
        for wp in waypoint_info:
            if wp[0] == selected-1:
                wp[0] += 1
    send_mission()
    update_mission_readout(selected)
    print('Waypoint ' + str(selected+1) + ' Moved down')
    
#switch the position of two waypoints
def swap_wps(wp1, wp2):
    #set up vars:
    global mission_actual, mission_custom, waypoints_been_modified, waypoint_info
    if not waypoints_been_modified:
        mission_custom = mission_actual[:]
        waypoints_been_modified = True
    #make sure each is in bounds
    if -1 < wp1-1 <= len(mission_custom) and -1 < wp2-1 <= len(mission_custom):
        #do the swapping
        temp = mission_custom[wp1-1]
        mission_custom[wp1-1] = mission_custom[wp2-1]
        mission_custom[wp2-1] = temp
        #fix wp_info
        if waypoint_info:
            for wp in waypoint_info:
                if wp[0] == wp1-1:
                    wp[0] = wp2-1
                if wp[0] == wp2-1:
                    wp[0] = wp1-1
        print('Waypoint ' + str(wp1) + ' swapped with' + str(wp2))

#reset to actual mission, and send old mission to MAV
def reset_wps():
    global mission_actual, mission_custom, waypoints_been_modified
    waypoints_been_modified = False
    send_mission()
    update_mission_readout()
    print('all waypoints reset.')

#new entry to wp_info, allows arguments to be optional
def new_wp_info(index, skip=None, radius=None):
    global waypoint_info
    #make sure the index is not already in wp_info, if so modify it
    for item in waypoint_info:
        if item[0] == index:
            if skip is not None:
                item[1] = skip
            if radius is not None:
                item[2] = radius
        return
    #if the entry is not already in wp_info, make a new entry
    waypoint_info.append([index, skip, radius])

#Listen for wp updates, and see if the new wp has a special condition
#Thread me!
def waypoint_listener():
    global vehicle, connected, waypoint_info
    old_wp = 0
    while connected and waypoint_info:
        new_wp = vehicle.commands.next
        if old_wp != new_wp:
            print('New WP Detected: #' + str(new_wp))
            #each entry in wp_info in the form [<Index>,<Skip True/False>,<Radius>]
            #(Radius of -1 to keep as default)
            for wp in waypoint_info:
                if wp[0] == new_wp:
                    if wp[1] == True:
                        #If this is true, skip this wp
                        vehicle.commands.next = vehicle.commands.next+1
                    elif wp[2] is not None:
                        #set radius if wp[2] is defined
                        vehicle.parameters['WP_RADIUS']  = wp[2]
                        print('WP_RADIUS set to ' + str(wp[2]))
                    elif vehicle.parameters['WP_RADIUS'] != WP_RADIUS:
                        #if the new wp doesn't have a custom radius, set back to default
                        vehicle.parameters['WP_RADIUS'] = WP_RADIUS
        time.sleep(1) #is this too short?
        old_wp = new_wp
    #if wp_info is cleared but vehicle is still connected
    if connected:
        vehicle.parameters['WP_RADIUS'] = WP_RADIUS

#----------TK Elements Setup----------
    
#adds toolbar to root frame
def create_toolbar(root):
    #Creates toolbar frame
    toolbar = Frame(root)
    
    mpLabel = Label(toolbar, text = 'Connect to Drone: ')
    mpLabel.pack(side=LEFT, padx=2, pady=2)
    #creates IP label
    MPipLabel = Label(toolbar, text='IP Address:')
    MPipLabel.pack(side=LEFT, padx=2, pady=2)
    
    #creates IP entry box
    MPipBox = Entry(toolbar)
    MPipBox.delete(0, END)
    MPipBox.insert(0, '127.0.0.1')  
    MPipBox.pack(side=LEFT, padx=2, pady=2)
    
    #creates port label
    MPportLabel = Label(toolbar, text='Port:')
    MPportLabel.pack(side=LEFT, padx=2, pady=2)
    
    #creates port entry box
    MPportBox = Entry(toolbar)
    MPportBox.delete(0, END)
    MPportBox.insert(0, '14551')  
    MPportBox.pack(side=LEFT, padx=2, pady=2)
    def connect_thr():
        thread.start_new_thread(connect_drone,(MPipBox.get(), MPportBox.get()))
    #creates connection button
    MPcon = Button(toolbar, text='Connect', command=connect_thr)
    MPcon.pack(side=LEFT, padx=2, pady=2)  
    #creates disconnect button
    MPdis = Button(toolbar, text='Disconnect', command=disconnect)
    MPdis.pack(side=LEFT, padx=2, pady=2)
    
    #create file read button
    Filebutton = Button(toolbar, text='Read Scenario File...', command= lambda: file_dialog(root))
    Filebutton.pack(side=RIGHT, pady=2)
    
    #pack it up
    toolbar.pack(side=TOP, fill=X, padx=5, pady=3)

#creates a split panned window, with a text box on the left, and fault buttons on the right
def create_window(root):
    window = Frame(root, padx=5)
    window.pack() 
    global info_readout, fault_frame, mission_readout
    #creates a text box on the left side (this is the readout window)
    info_readout = Text(window, height=25, width=50, state=DISABLED, bd=1, relief=RIDGE)
    info_readout.pack(side=LEFT, fill=BOTH, expand=1)
    
    #creates a button frame on the right
    fault_frame = Frame(window, padx=2)
    create_fault_pane(fault_frame)
    fault_frame.pack(side=LEFT, fill=Y, expand=1)
    
    missionFrame = Frame(window, height=25)
    #fill the mission frame, and globalise the readout for easy updates
    mission_readout = create_mission_pane(missionFrame)
    
    missionFrame.pack(side=LEFT, fill=Y)

#adds faults to the window -- gps failure, wind, etc 
def create_fault_pane(pane):
    #top pane for the wind controls
    wind_pane = Frame(pane)
    #load the constant and gust buttons
    create_const_buttons(wind_pane).pack(side=TOP, fill=X, expand=1)
    create_gust_buttons(wind_pane).pack(side=TOP, fill=X, expand=1)
    wind_pane.pack(fill=BOTH, expand=1)
    
    #everything else
    not_wind_pane = Frame(pane, padx=2, pady=2, bd=2, relief=GROOVE)
    
    #add gps Frame and button
    gps_rc_frame = Frame(not_wind_pane)
    gpsButton = Button(gps_rc_frame, text = 'Disable GPS', command=lambda: gps(gpsButton))
    gpsButton.pack(side=LEFT, fill=BOTH, expand=1)

    #add rc Frame and button
    rcButton = Button(gps_rc_frame, text = 'Disable RC', command=lambda: rc(rcButton))
    rcButton.pack(side=RIGHT, fill=BOTH, expand=1);
    gps_rc_frame.pack(fill=BOTH, expand=1)

    #add throttle frame and button
    thrButton = Button(not_wind_pane, text = 'Activate Throttle Failsafe', width = 20, command=lambda: throttle(thrButton))
    thrButton.pack(fill=BOTH, expand=1)

    #add battery frame and button
    battButton = Button(not_wind_pane, text = 'Activate Battery Failsafe', width = 20, command=lambda: battery(battButton))
    battButton.pack(fill=BOTH, expand=1)
    
    #add gps frame and button
    GCSButton = Button(not_wind_pane, text = 'Disconnect GCS', width = 20, command=lambda: gcs(GCSButton))
    GCSButton.pack(fill=BOTH, expand=1)
    
    not_wind_pane.pack(side=TOP, fill=BOTH, expand=1)

#helper functions for fault pane 
def create_const_buttons(pane):
    pop = Frame(pane, bd=2, relief=GROOVE)
    msg = Label(pop, text='Set Global Wind:')
    msg.pack()

    #speed
    spdPane = Frame(pop)
    spdLabel = Label(spdPane, text='Windspeed (m/s):')
    spdLabel.pack(side=LEFT)

    spdBox = Entry(spdPane, width=10)
    spdBox.pack(side=LEFT)
    spdBox.delete(0, END)
    spdBox.insert(0, '0')
    spdPane.pack(side=TOP, padx=5)

    #dir
    dirPane = Frame(pop)
    dirLabel = Label(dirPane, text='Yaw (degrees):      ')
    dirLabel.pack(side=LEFT)

    dirBox = Entry(dirPane, width=10)
    dirBox.pack(side=LEFT)
    dirBox.delete(0, END)
    dirBox.insert(0, '0')
    dirPane.pack(side=TOP, padx=5)

    #pitch
    pitchPane = Frame(pop)
    pitchLabel = Label(pitchPane, text='Pitch (degrees):    ')
    pitchLabel.pack(side=LEFT)

    pitchBox = Entry(pitchPane, width=10)
    pitchBox.pack(side=LEFT)
    pitchBox.delete(0, END)
    pitchBox.insert(0, '0')
    pitchPane.pack(side=TOP, padx=5)
    
    #turbulence
    turbPane = Frame(pop)
    turbLabel = Label(turbPane, text='Turbulence:          ')
    turbLabel.pack(side=LEFT)

    turbBox = Entry(turbPane, width=10)
    turbBox.pack(side=LEFT)
    turbBox.delete(0, END)
    turbBox.insert(0, '0')
    turbPane.pack(side=TOP, padx=5)

    #buttons
    btnPane = Frame(pop, pady=3)
    btnStart = Button(btnPane, text='Enter', width=7, command= lambda : wind(float(spdBox.get()), float(dirBox.get()), float(pitchBox.get()), float(turbBox.get())))
    btnStart.pack(side=LEFT)

    #btnClose = Button(btnPane, text='Close', command = pop.destroy)
    #btnClose.pack(side=LEFT)

    btnPane.pack(side=TOP, padx=5)
    return pop
    
def create_gust_buttons(pane):
    pop2 = Frame(pane, bd=2, relief=GROOVE)
    msg = Label(pop2, text='Send Wind Gust:')
    msg.pack()
    #speed
    spdPane = Frame(pop2)
    spdLabel = Label(spdPane, text='Windspeed (m/s): ')
    spdLabel.pack(side=LEFT)
    
    spdBox = Entry(spdPane, width=10)
    spdBox.pack(side=RIGHT)
    spdBox.delete(0, END)
    spdBox.insert(0, '0')
    spdPane.pack(side=TOP, padx=5)
    
    #dir
    dirPane = Frame(pop2)
    dirLabel = Label(dirPane, text='Yaw (degrees):      ')
    dirLabel.pack(side=LEFT)
    
    dirBox = Entry(dirPane, width=10)
    dirBox.pack(side=RIGHT)
    dirBox.delete(0, END)
    dirBox.insert(0, '0')
    dirPane.pack(side=TOP, padx=5)
    
    #pitch
    pitchPane = Frame(pop2)
    pitchLabel = Label(pitchPane, text='Pitch (degrees):    ')
    pitchLabel.pack(side=LEFT)
    
    pitchBox = Entry(pitchPane, width=10)
    pitchBox.pack(side=RIGHT)
    pitchBox.delete(0, END)
    pitchBox.insert(0, '0')
    pitchPane.pack(side=TOP, padx=5)
    
    #period
    perPane = Frame(pop2)
    perLabel = Label(perPane, text='Gust Period (s):     ')
    perLabel.pack(side=LEFT)
    
    perBox = Entry(perPane, width=10)
    perBox.pack(side=RIGHT)
    perBox.delete(0, END)
    perBox.insert(0, '10')
    perPane.pack(side=TOP, padx=5)
    
    #buttons
    btnPane = Frame(pop2, pady=3)
    btnStart = Button(btnPane, text='Start', width=7, command= lambda : make_gust(float(spdBox.get()), float(dirBox.get()), float(pitchBox.get()), int(perBox.get())))
    btnStart.pack(side=LEFT)
    
    #btnClose = Button(btnPane, text='Close', command = pop2.destroy)
    #btnClose.pack(side=LEFT)
    
    btnPane.pack(side=TOP, padx=5)
    return pop2

#add elements to waypoint-centric area
def create_mission_pane(pane):
    msg_frame = Frame(pane)
    msg = Label(msg_frame, text='Waypoints:').pack(side=LEFT)
    update_button = Button(msg_frame, text='Update Waypoints', command=lambda: thread.start_new_thread(reset_wps, ()))
    update_button.pack(side=RIGHT)
    msg_frame.pack(fill=X)
    info = None
    
    #GUI creation code for the various waypoint functions. 
    def flyby_frame(panel):
        global popup_frame
        if popup_frame != None:
            popup_frame.destroy()
        popup_frame    = Frame(panel, bd=2, relief=GROOVE, padx=3, pady=3)
        mess_frame = Frame(popup_frame)
        msg = Label(mess_frame, text='New Radius?')
        msg.pack(side=LEFT)
        mess_frame.pack(fill=X)
        input_frame = Frame(popup_frame)
        input_box = Entry(input_frame)
        input_box.pack(side=LEFT, fill=X, expand=1)
        global vehicle
        #reads inputs and sends to be changed by the actual fb_wp()
        def fly_wrapper():
            rad = int(input_box.get())
            try:
                selected = int(info.curselection()[0]) + 1 #Commands start at 1 :-(
                print('selected wp =' + str(selected))
            except IndexError:
                print('It seems no waypoint was selected.')
                return
            edit_flyby_wp(selected, rad)
        input_button = Button(input_frame, text='Enter', command=fly_wrapper)
        input_button.pack(side=LEFT)
        input_frame.pack(fill=X)
        desc = Message(popup_frame, width=220, text='This will set the radius a plane must fly through to hit a waypoint.')
        desc.pack()
        popup_frame.pack()
    
    def reorder_frame(panel):
        global popup_frame
        if popup_frame != None:
            popup_frame.destroy()
        popup_frame     = Frame(panel, bd=2, relief=GROOVE, padx=3, pady=3)
        button_frame = Frame(popup_frame)
        up_button    = Button(button_frame, text='/\\', command=lambda: thread.start_new_thread(lambda: repo_up_wp(int(info.curselection()[0])), ()))
        down_button  = Button(button_frame, text='\\/', command=lambda: thread.start_new_thread(lambda: repo_down_wp(int(info.curselection()[0])), ()))
        description  = Message(popup_frame, width=200, text='Use the left buttons to reorder the currently selected waypoint.')
        up_button.pack()
        down_button.pack()
        button_frame.pack(side=LEFT)
        description.pack(side=LEFT)
        popup_frame.pack()
        return popup_frame
      
    def skip_frame(panel):
        global popup_frame
        if popup_frame != None:
            popup_frame.destroy()
        popup_frame     = Frame(panel, bd=2, relief=GROOVE, padx=3, pady=3)
        skip_button = Button(popup_frame, text='Skip', command=lambda: thread.start_new_thread(lambda: skip_wp(int(info.curselection()[0])), ()))
        skip_button.pack()
        description  = Message(popup_frame, width=200, text='Skipped waypoints will stay in memory, but not be flown to. They will be greyed out in the waypoint window.')
        description.pack(side=LEFT)
        popup_frame.pack()
        return popup_frame
    
    def vary_cords_frame(panel):
        global popup_frame
        if popup_frame != None:
            popup_frame.destroy()
        popup_frame    = Frame(panel, bd=2, relief=GROOVE, padx=3, pady=3)
        mess_frame = Frame(popup_frame)
        msg = Label(mess_frame, text='Variation Distance?')
        msg.pack(side=LEFT)
        mess_frame.pack(fill=X)
        input_frame = Frame(popup_frame)
        input_box = Entry(input_frame)
        input_box.pack(side=LEFT, fill=X, expand=1)
        def vary_wrapper():
            rad = int(input_box.get())
            try:
                selected = int(info.curselection()[0]) #Commands start at 1 :-(
                print('selected wp =' + str(selected))
            except IndexError:
                print('It seems no waypoint was selected.')
                return
            vary_wp(selected, rad)
        input_button = Button(input_frame, text='Enter', command=vary_wrapper)
        input_button.pack(side=LEFT)
        input_frame.pack(fill=X)
        desc = Message(popup_frame, width=220, text='This will vary the selected coordinate by the given amount in degrees (gps), in a random direction. Leave blank to also randomize distance.')
        desc.pack()
        popup_frame.pack()
        return popup_frame
        
    #Listbox for selecting waypoints and scroll bar
    textFrame = Frame(pane)
    info = Listbox(textFrame, height=10, width=37)
    info.insert(END, 'connect to drone!')
    info.pack(side=LEFT)
    info_scroll = Scrollbar(textFrame, command=info.yview)
    info.config(yscrollcommand=info_scroll.set)
    info_scroll.pack(side=RIGHT, fill=Y, expand=1)
    textFrame.pack(side=TOP)
    
    #where the waypoint options go
    bottom_frame = Frame(pane, pady=3)
    
    #the words 'With selection', left justified
    with_frame = Frame(bottom_frame)
    with_label = Label(with_frame, text='With Selection...').pack(side=LEFT)
    with_frame.pack(fill=X)
    
    #first row of fault buttons
    button_frame = Frame(bottom_frame)
    button_sub_frame1 = Frame(button_frame)
    randomise_button  = Button(button_sub_frame1, text='Vary Coords', command=lambda: vary_cords_frame(pane))
    randomise_button.pack(side=LEFT, fill=X, expand=1)
    skip_button       = Button(button_sub_frame1, text='Skip', command=lambda: skip_frame(pane)).pack(side=LEFT, fill=X, expand=1)
    button_sub_frame1.pack(fill=X)

    #second row of fault buttons
    button_sub_frame2 = Frame(button_frame)
    radius_button     = Button(button_sub_frame2, text='Change flyby radius', command=lambda: flyby_frame(pane)).pack(side=LEFT, fill=X, expand=1)
    reorder_button    = Button(button_sub_frame2, text='Reorder waypoint', command=lambda: reorder_frame(pane)).pack(side=LEFT, fill=X, expand=1)
    button_sub_frame2.pack(fill=X)
    
    button_frame.pack(fill=X)
    bottom_frame.pack(fill=X)
    return info

#Popup Scenario File chooser -- possibly add custom wrapper dialog?
def file_dialog(root):
    file = str(tkFileDialog.askopenfilename(parent=root, initialdir=sys.path[0], title='Select Scenario'))
    if type(file) == type('str') and file != '':
        thread.start_new_thread(command_thread, (file,))

#threaded function to update wind
def send_wind_params():
    global connected
    global const_wind_x,  const_wind_y,  const_wind_z
    global gust_wind_spd, gust_wind_dir, gust_wind_pitch
    global wind_turb
    while connected:
        [gust_wind_x, gust_wind_y, gust_wind_z] = angles_to_vector(gust_wind_spd,gust_wind_dir,gust_wind_pitch)
        wind_x = gust_wind_x + const_wind_x
        wind_y = gust_wind_y + const_wind_y
        wind_z = gust_wind_z + const_wind_z
        [spd, dir, pitch] = vector_to_angles(wind_x, wind_y, wind_z)
        vehicle.parameters['SIM_WIND_DIR'] = float(dir)
        vehicle.parameters['SIM_WIND_SPD'] = float(spd)
        vehicle.parameters[Z_WIND_NAME] = float(pitch)
        vehicle.parameters['SIM_WIND_TURB'] = float(wind_turb)
        time.sleep(0.5)
    
#----------helper/math functions---------- 

#returns array of [spd, dir, pitch] based on given vector
def vector_to_angles(x, y, z):
    x = float(x)
    y = float(y)
    z = float(z)
    #this fixes unexpected angles at 0 spd
    if 0 == x == y == z:
        return[0, 0, 0]
    pitch = math.degrees(math.atan2(z,math.sqrt(x*x + y*y)))
    if y==0:
        if x>0:
            yaw =  90.0
        else:
            yaw = -90.0
    else:
        yaw = math.degrees(math.atan2(x,y))
    spd   = (x*x + y*y + z*z)**(1./2.)

    pitch = round(pitch,1)
    yaw   = round(yaw,1)
    spd   = round(spd, 1)
    return [spd, yaw, pitch]

#returns array of [x, y, z] based on given speed and angles
def angles_to_vector(spd, yaw, pitch):
    wx = math.sin(math.radians(yaw))*math.cos(math.radians(pitch))*spd
    wy = math.cos(math.radians(yaw))*math.cos(math.radians(pitch))*spd
    wz =                             math.sin(math.radians(pitch))*spd
    #we round to fix floating point errs
    wx = round(wx, 2)
    wy = round(wy, 2)
    wz = round(wz, 2)
    return [wx, wy, wz]    
   
#converts vector in miles to vector in degress longitude and latitude    
def miles_to_coords(x, y, curr_lat):
    lat_equ = 69.172
    long = math.cos(math.radians(curr_lat))*lat_equ
    return (float(x)/lat_equ,float(y)/long)
    
#Returns Str of command name given key represents.
#Or, returns they key in string form if not found.
def command_key_to_name(key):
    if key == 16:
        return 'WAYPOINT'
    if key == 19:
        return 'LOITER_TIME'
    if key == 20:
        return 'RTL'
    if key == 21:
        return 'LAND'
    if key == 22:
        return 'TAKEOFF'
    #Unrecognized code? return key
    return str(key)
    
#----------vehicle properties functions---------- 

#The following 6 functions are called when their corresponding Fault Buttons are pressed
def wind(windSPD, windDIR, windPITCH, windTURB):
    global connected
    if connected:
        global const_wind_x, const_wind_y, const_wind_z, wind_turb
        [const_wind_x,const_wind_y,const_wind_z] = angles_to_vector(windSPD, windDIR, windPITCH)
        wind_turb = windTURB
    else:
        print('Connect to Drone first!')

def gps(button):
    global connected
    if not connected:
        print('Connect to drone first!')
        return
    #get global button and failsafe readout text
    global gpsfs
    #create param dictionary
    #mav_param = mavparm.MAVParmDict()
    # if button's text is to Disable GPS
    if button['text'] == 'Disable GPS':
        print('Disabling GPS')
        #set SITL to disable gps
        vehicle.parameters['SIM_GPS_DISABLE'] = float(1)
        #change button text
        button.configure(text='Enable GPS')
        #change readout text
        gpsfs = 'Active'
        #if text is Enable gps
    else:
        #set SITL to enable gps
        print('Enabling GPS')
        vehicle.parameters['SIM_GPS_DISABLE'] = float(0) 
        #change readout text
        gpsfs = 'Inactive'
        #change button text
        button.configure(text='Disable GPS')

def rc(button):
    #get global button and failsafe readout text
    #create param dictionary
    #mav_param = mavparm.MAVParmDict()
    #if button text says Disable RC
    global rfs
    if button['text'] == 'Disable RC':
        print('Disabling RC')
        #Send param to disable RC
        vehicle.parameters['SIM_RC_FAIL'] = float(1)
        #change readout text
        rfs = 'Active'
        #change button text
        button.configure(text='Enable RC')
    else:
        print('Enabling RC')
        #reactivate RC
        vehicle.parameters['SIM_RC_FAIL'] = float(0)
        #change readout text
        rfs = 'Inactive'
        #change button text
        button.configure(text='Disable RC')

def throttle(button):
    #create parameter dictionary
    #mav_param = mavparm.MAVParmDict()
    #get global dronekit vehiclea, button, current throttle PWM failsafe value, and failsafe readout text
    global vehicle, tfs
    #if button says to Activate Throttle Failsafe
    if button['text'] == 'Activate Throttle Failsafe':
        print('Activating Throttle Failsafe')
        #set throttle failsafe
        vehicle.parameters['THR_FS_VALUE'] = float(2000)
        #change readout
        tfs = 'Active'
        #change button text
        button.configure(text='Deactivate Throttle Failsafe')
    else:
        print('Deactivating Throttle Failsafe')
        #Set Throttle pwn failsafe value back to normal
        vehicle.parameters['THR_FS_VALUE'] = float(0)
        #change button text
        button.configure(text='Activate Throttle Failsafe')
        #change readout text
        tfs = 'Inactive'

def battery(button):
    #create param dictionary
    #mav_param = mavparm.MAVParmDict()
    #get global dronekit vehicle, button, battert MAH,and failsafe readout text
    global vehicle, FS_BATT_MAH, bfs
    #if button text says to activate battery failsafe
    if button['text'] == 'Activate Battery Failsafe':
        print('Activating Battery Failsafe')
        #set battery FS value above current capacity
        vehicle.parameters['FS_BATT_MAH'] = float(4000)
        #set readout text
        bfs = 'Active'
        button.configure(text='Deactivate Battery Failsafe')
    else:
        print('Deactivating Battery Failsafe')
        #set battery capacity back to normal
        vehicle.parameters['FS_BATT_MAH'] = float(0)
        #set readout
        bfs = 'Inactive'
        button.configure(text='Activate Battery Failsafe')

def gcs(button):
    global vehicle, gcsfs, SYSID_MYGCS
    #if button says Disconnect GCS
    if button['text'] == 'Disconnect GCS':
        #store GCS ID
        SYSID_MYGCS = vehicle.parameters['SYSID_MYGCS']
        #set GCS ID to something unrecognizable to the vehicle
        vehicle.parameters['SYSID_MYGCS'] = float(0)
        #change button text
        button.configure(text='Reconnect GCS')
        #change readout
        gcsfs = 'Active'
        print('GCS disconnected')
    else:
        #set GCS id back
        vehicle.parameters['SYSID_MYGCS'] = float(SYSID_MYGCS)    
        #change button text
        button.configure(text='Disconnect GCS')
        #change readout
        gcsfs = 'Inactive'
        print('GCS reconnected')
#set all wind values to 0, enable sensors, disable fail safes, and run the waypoint reset code
def reset():
    wind(0, 0, 0, 0)
    global gcsfs, bfs, tfs, rfs, gpsfs, vehicle, WP_RADIUS
    if gpsfs == 'Active':
        gps()
    if rfs == 'Active':
        rc()
    if bfs == 'Active':
        battery()
    if tfs == 'Active':
        throttle()
    if gcsfs == 'Active':
        gcs()
    vehicle.parameters['SIM_WIND_DIR']   = 0
    vehicle.parameters['SIM_WIND_SPD']   = 0
    vehicle.parameters[Z_WIND_NAME] = 0
    
    vehicle.parameters['SIM_WIND_TURB']  = 0
    vehicle.parameters['WP_RADIUS']  = WP_RADIUS
    
    #Reset mission to the original mission if needed
    reset_wps()
    
#Send gust, changing the wind over time
def make_gust(speed, dir, pitch, period):
    global is_gusting
    #check if a gust is already happening, so there is never two at once
    if is_gusting:
        print('Wind gust already ongoing!')
        return
    is_gusting = True
    
    #set the direction
    global gust_wind_dir, gust_wind_pitch
    gust_wind_dir = dir
    gust_wind_pitch = pitch
    
    #threaded function to control the gust speed over time
    def gust_thread():
        print('Gusting for ' + str(period) + ' seconds')
        global gust_wind_spd, gust_wind_dir, gust_wind_pitch
        pi2 = 3.1416*2 #so we don't have to keep computing this
        #Follow a -cos curve, starting and ending at 0, and curving up to target_spd
        for x in range(0, period):
            gust_wind_spd = (speed/2)*(-math.cos(x*pi2/period) + 1)
            time.sleep(1)
        #Reset vals, so they don't keep getting used post-gust
        gust_wind_spd = 0
        gust_wind_dir = 0
        gust_wind_pitch = 0
        global is_gusting
        is_gusting = False
        print('Gust completed')
    thread.start_new_thread(gust_thread, ())
    
#----------command_thread() helper functions----------

def set_turb(newturb):
    print('Setting turbulence to ' + str(newturb))
    global const_wind_x, const_wind_y, const_wind_z
    a = vector_to_angles(const_wind_x, const_wind_y, const_wind_z)
    wind(a[0], a[1], a[2], newturb)

def set_wspd(newspd):
    print('Setting windspeed to ' + str(newspd) + ' (m/s)')
    global const_wind_x, const_wind_y, const_wind_z, wind_turb
    a = vector_to_angles(const_wind_x, const_wind_y, const_wind_z)
    wind(newspd, a[1], a[2], wind_turb)

def set_wdir(newdir):
    print('Setting wind yaw to ' + str(newdir) + ' (degrees)')
    global const_wind_x, const_wind_y, const_wind_z, wind_turb
    a = vector_to_angles(const_wind_x, const_wind_y, const_wind_z)
    wind(a[0], newdir, a[2], wind_turb)

def set_pitch(newpitch):
    print('Setting wind pitch to ' + str(newpitch) + ' (degrees)')
    global const_wind_x; global const_wind_y; global const_wind_z; global wind_turb
    a = vector_to_angles(const_wind_x, const_wind_y, const_wind_z)
    wind(a[0], a[1], newpitch, wind_turb)

def command_thread(input):
    commands = list(csv.reader(open(input, 'r')))
    t = 0
    print('Running testing scenario..')
    #loop repeatedly, executing commands when they occur and removing them from list.
    #end when list is empty.
    if not connected:
        print('Scenario stopped: Connect to drone first!')
        return
    while commands != []:
        print('Time: ' + str(t))
        
        for command in commands[:]: #in python, "foo[:]" returns a copy of the list foo
            if command[0] == '#':
                #clear out the comments
                commands.remove(command)
                continue
            if int(command[0]) <= t:
                if not connected:
                    print('Scenario stopped: Connection to drone lost!')
                    return
                if type(command[1]) != type('str'):
                    print('Can\'t read event! Ensure each line is formatted as <Timestamp>,<Name>,<arg0>,...,<argN>')
                    #command will end up deleted afterwards
                else:
                    if   command[1] == 'TURBULENCE'      or command[1] == 'TURB':
                        set_turb(float(command[2]))
                    elif command[1] == 'WINDSPEED'       or command[1] == 'WSPD':
                        set_wspd(float(command[2]))
                    elif command[1] == 'WIND_DIR'        or command[1] == 'WDIR':
                        set_wdir(int(command[2]))
                    elif command[1] == 'WIND_DIR_Z'      or command[1] == 'ZDIR':
                        set_pitch(int(command[2]))
                    elif command[1] == 'GPS_TOGGLE'      or command[1] == 'GPS':
                        gps()
                    elif command[1] == 'RC_TOGGLE'       or command[1] == 'RC':
                        rc()
                    elif command[1] == 'THROTTLE_TOGGLE' or command[1] == 'THRT':
                        throttle()
                    elif command[1] == 'EASTER_EGG':
                        print('This program has 3 buttons on the toolbar? A Triangle has 3 sides?? Illuminati confirmed!')
                    elif command[1] == 'BATTERY_TOGGLE'  or command[1] == 'BAT':
                        battery()
                    elif command[1] == 'GCS_TOGGLE'      or command[1] == 'GCS':
                        gcs()
                    elif command[1] == 'WIND':
                        print('Setting wind')    #Printed here to keep wind() clean
                        #make sure they have enough args
                        try:
                            command[5]
                        except IndexError:
                            print('Error: Not enough args for WIND event at time ' + str(t))
                            commands.remove(command)
                            continue
                        else:
                            wind(float(command[2]), int(command[3]), int(command[4]), float(command[5]))
                    elif command[1] == 'GUST':
                        make_gust(float(command[2]), int(command[3]), int(command[4]), int(command[5]))
                    elif command[1] == 'WP_MOVE_UP' or command[1] == 'UP':
                        repo_up_wp(int(command[2]))
                    elif command[1] == 'WP_MOVE_DOWN' or command[1] == 'DOWN':
                        repo_down_wp(int(command[2]))
                    elif command[1] == 'WP_SWAP' or command[1] == 'SWAP':
                        swap_wps(int(command[2]), int(command[3])+1)
                    elif command[1] == 'WP_SKIP' or command[1] == 'SKIP':
                        skip_wp(int(command[2]))
                    elif command[1] == 'WP_RADIUS' or command[1] == 'RAD':
                        edit_flyby_wp(int(command[2]), int(command[3]))
                    elif command[1] == 'WP_OFFSET' or command[1] == 'OFFSET':
                        change_wp_loc(int(command[2]), int(command[3]), int(command[4]))
                    elif command[1] == 'RESET':
                        reset()
                    else:
                        print('Invalid event \'' + command[1] + '\' at time: ' + str(command[0]))
                #after executing the command, remove it. Command is also removed if it is invalid
                commands.remove(command)
        t += 1
        time.sleep(1)
    print('Scenario completed.')
    
#take an iterator of commands and execute them at given times. this is called for file input
def command_thread_old(input):
    commands = csv.reader(open(input, 'r'))
    #increment up in time, executing commands when they arrive
    t = 0
    print('Running testing scenario..')
    if not connected:
        print('Scenario stopped: Connect to drone first!')
        return
    for command in commands:
        #ignore line if comment
        if command[0] == '#':
            continue
        #wait until time to execute command, we are assuming they are in chronological order (!)
        while t < int(command[0]):
            time.sleep(1)
            t = t+1
        #check to make sure the command is valid
        if type(command[1]) != type('str'):
            print('Can\'t read event! Ensure each line is formatted as <Timestamp>,<Name>,<arg0>,...,<argN>')
        else:
            #run the appropriate command: descriptions found in readme.txt
            if   command[1] == 'TURBULENCE'      or command[1] == 'TURB':
                set_turb(float(command[2]))
            elif command[1] == 'WINDSPEED'       or command[1] == 'WSPD':
                set_wspd(float(command[2]))
            elif command[1] == 'WIND_DIR'        or command[1] == 'WDIR':
                set_wdir(int(command[2]))
            elif command[1] == 'WIND_DIR_Z'      or command[1] == 'ZDIR':
                set_pitch(int(command[2]))
            elif command[1] == 'GPS_TOGGLE'      or command[1] == 'GPS':
                gps()
            elif command[1] == 'RC_TOGGLE'       or command[1] == 'RC':
                rc()
            elif command[1] == 'THROTTLE_TOGGLE' or command[1] == 'THRT':
                throttle()
            elif command[1] == 'FUN':
                print('This program has 3 buttons on the toolbar? A Triangle has 3 sides?? Illuminati confirmed!')
            elif command[1] == 'BATTERY_TOGGLE'  or command[1] == 'BAT':
                battery()
            elif command[1] == 'GCS_TOGGLE'      or command[1] == 'GCS':
                gcs()
            elif command[1] == 'WIND':
                print('Setting wind')    #Printed here to keep wind() clean
                #make sure they have enough args
                try:
                    command[5]
                except IndexError:
                    print('Not enough args for WIND at time ' + str(t))
                    print('Exiting Scenario...')
                    reset()
                    break
                else:
                    wind(float(command[2]), int(command[3]), int(command[4]), float(command[5]))
            elif command[1] == 'GUST':
                make_gust(float(command[2]), int(command[3]), int(command[4]), int(command[5]))
            elif command[1] == 'WP_MOVE_UP' or command[1] == 'UP':
                repo_up_wp(int(command[2]))
            elif command[1] == 'WP_MOVE_DOWN' or command[1] == 'DOWN':
                repo_down_wp(int(command[2]))
            elif command[1] == 'WP_SWAP' or command[1] == 'SWAP':
                swap_wps(int(command[2]), int(command[3]))
            elif command[1] == 'WP_SKIP' or command[1] == 'SKIP':
                skip_wp(int(command[2]))
            elif command[1] == 'WP_RADIUS' or command[1] == 'RAD':
                edit_flyby_wp(int(command[2]), int(command[3]))
            elif command[1] == 'WP_OFFSET' or command[1] == 'OFFSET':
                change_wp_loc(int(command[2]), int(command[3]), int(command[4]))
            elif command[1] == 'RESET':
                reset()
            else:
                print('Invalid event \'' + command[1] + '\' at time: ' + str(command[0]))
    print('Scenario completed.')

def main():
    global root
    #create root window
    root = Tk()
    root.resizable(width=False, height=False)
    def on_close():
        disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    
    root.title('Fault Injector')
    #set window size
    #root.geometry('840x440')
    #add the connections toolbar onto the root window
    create_toolbar(root)
     
    #create the panes for the readout window and the fault buttons
    updatePanes = create_window(root)
    global info_readout
    update_readout_window(info_readout,'Use the connect button to connect to a drone!')
    
    #loop 
    root.mainloop()

if __name__ == '__main__':
    main()
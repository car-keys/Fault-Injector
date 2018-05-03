FAULT INJECTOR - Usage Guide/Information
ver 2

Original Developer: Jayson Boubin
Current Developer: Nicholas Engle

Any questions? email me (Nicholas Engle) at englens@mail.uc.edu

Fault Injector is a program which triggers failsafes in fixed wing ardupilot mega aircraft in SITL using dronekit, mavproxy, and mavlink.
It provides the ability to set up testing scenarios, running a set of faults over a specified time interval.
It can also be modified to inject failures into other craft that can be simulated in SITL, or to change 
any variables in the simulation over mavlink.

FaultInjector runs on python version 2.7. The following python libraries are dependencies:
dronekit, Tkinter, tkFileDialog

Setting up a SITL instance:
Windows: http://ardupilot.org/dev/docs/sitl-native-on-windows.html
Linux:   http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

There are three main areas of the FaultInjector Window:
    Connection Toolbar (Top of window):
	Use this are to connect to a vehicle. Input the correct Mavproxy IP and Port 
	(Use default if ran from same computer), and press connect. The readout window will
	populate with information if connection is successful.
    Readout Pane (Left Panel):
	This area is populated with information about the vehicle in SITL. It also displays
	the status of the various fail safes (Active or Inactive).
    Manual Fault Control (Middle area):
	This area provides manual controls for the various faults supported by FaultInjector.
	These include:
	    Constant Wind:
		Allows the user to set a constant (global) wind vector, using Yaw and Pitch to
		control the direction. This area also allows the input a Turbulence value,
		simulating a random deviation of the wind vector.
	    Wind Gust:
		Simulates a simple wind gust, adding an additional vector to the constant wind.
		The user can input direction and max magnitude, as well as the cycle period.
		FaultInjector will increase the gust magnitude from 0 to the specified magnitude, and
		return to 0 in the set time. This magnitude will follow a sine curve.
	    Fail safes:
		These buttons allow the user to control the various vehicle failsaves and
		simulate various system faults: GPS, Radio, Throttle, Battery, and GCS.
    Mission Waypoint Control (Rightmost Area):
        Displays current mission waypoints or commands, and some mission controls. 
	Mission controls do not notify MissionPlanner of changes.

FaultInjector allows the user to write scenario files, and run custom automated test scenarios.
This functionality is accessed by pressing the "Read from file..." button, and selecting a valid file.

Scenario files are comma-separated values (csv) files, made up of rows with the following syntax:
<Timestamp>,<EventName>,<arg1>,...,<argN>
The timestamp represents the time the event is to be executed (in seconds, with 0 representing the beginning
of execution). Timestamps must be in chronological order, or they will produce unexpected results.
EventName is the event to be executed at the given timestamp. Some events have special arguments to be specified
afterwards. 

The events are listed as follows:
Name____________| Description____________________________________________________| Args_________________________________
WINDSPEED:      | Set the constant windspeed. 				         | 1 arg: value (m/s)
WIND_DIR:       | Set the yaw of the const windspeed. 			         | 1 arg: value (degrees)
WIND_DIR_Z:     | Set the pitch of the const windspeed. 		         | 1 arg: value (degrees)
TURBULENCE:     | Set the value of the turbulence. 			         | 1 arg: value
WIND:           | Set all const wind values at once. 			         | 4 args: speed, yaw, pitch, turbulence
GUST:           | Start a gust. 					         | 4 args: max_speed, yaw, pitch, period
GPS_TOGGLE:     | Toggle the state of the Gps failstate. 		         | 0 args.
RC_TOGGLE:      | Toggle the state of the RC failstate. 		         | 0 args.
THROTTLE_TOGGLE:| Toggle the state of the Throttle failstate. 		         | 0 args.
BATTERY_TOGGLE: | Toggle the state of the Battery failstate. 		         | 0 args.
GCS_TOGGLE:     | Toggle the state of the GCS failsafe. 		         | 0 args.
RESET:          | Resets all values to default. 			         | 0 args.
#,: 		| Signifies a comment. (Ignore all characters after the comma; dont use a timestamp)
WP_MOVE_UP:	| Move the given mission waypoint up (backwards in the mission)  | 1 arg:  waypoint.
WP_MOVE_DOWN:	| Move the given waypoint down (fowards in the mission) 	 | 1 arg:  waypoint.
WP_SWAP:	| Swap two given waypoints. 				         | 2 args: waypoint A, waypoint B
WP_SKIP:	| Flag a (future) waypoint to be skipped. 		         | 1 arg:  waypoint
WP_RADIUS:	| Set the radius the vehicle must fly in to reach the waypoint.  | 2 args: waypoint, radius             Note: This does not seem to work.
WP_OFFSET:	| Offset the location of a waypoint by given distance (miles).   | 3 args: waypoint, x_change, y_change

Shortened versions of commands are included for typing convenience:
WINDSPEED ->       WSPD
WIND_DIR ->        WDIR
WIND_DIR_Z ->      ZDIR
TURBULENCE ->      TURB
GPS_TOGGLE ->      GPS
RC_TOGGLE ->       RC
THROTTLE_TOGGLE -> THRT
BATTERY_TOGGLE ->  BAT
GCS_TOGGLE ->      GCS
WP_MOVE_UP ->	   UP
WP_MOVE_DOWN ->	   DOWN
WP_SWAP ->         SWAP
WP_SKIP ->	   SKIP
WP_RADIUS ->	   RAD
WP_OFFSET ->	   OFFSET

Example Scenario:
1,TURB,4
1,GPS
3,WSPD,10
3,ZDIR,5
6,WDIR,60
10,SPD,15
15,GUST",5,200,0,6
#, gps resumes just after gust finishes
23,GPS
25,RESET

NOTE: If RESET is not executed at the end of a scenario, the conditions will
continue even after the scenario ends. This could be desired, for example if 
a scenario is designed to set up conditions for further testing with manual controls.

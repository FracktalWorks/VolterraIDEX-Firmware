# VolterraIDEX-Firmware
This Repository consists of the methodology and syntax of the Klipper '.cfg' file, and the methodology of defining values and macros in it. Along with it, the initial steps of flashing the Firmware in the BTT Octopus V1.1 motherboard by interfacing with Raspberry Pi and flashing the image file alomg with the BIN file of the configuration file is mentioned below. Apart from this, the plugin for accessing the 'OctoKlipper' is also mentioned below in order for quick access for the '.cfg' and instant modifications.

## Octoprint configuration in Raspberry PI

* Connect a SD card to the system  using the SD card reader 
* Open the win32 disk imager in the system and then use the Octoprint image file in the software and write the image file in the SD card
* After writing it safely eject it from the system

## OCTOprint TOUCHUI configuration 
* Refer this documentation for the intial setup of the Touch ui in RaspberryPI octoprint server [Julia-Touch-UI-Documentation](https://github.com/FracktalWorks/Julia-Touch-UI-Documentation)

## Klipper installation in octoprint 

* Click on the ```setting Icon``` of the octoprint and in that search fo the ```plugin manager``` and click on ```get more``` and then search fo ```OctoKlipper ``` 
* Click on install and restart the Octoprint and then you can see the Klipper option .

## Klipper Installation 

Refer this documentation and steps for seting up of klipper and configuration with Micro Controller [Klipper3d](https://www.klipper3d.org/Installation.html)

## Volterra IDEX Klipper Firmware '.cfg' file
### Methodology and Syntax ###
The basic structure and defining of macros and variables are explained in the following documentation below:
#### [stepper_#] ####
* The particular functions **‘step_pin’**, **‘dir_pin’** and **'enable_pin’** are the various pin configurations for the motor drivers which are found in the pin mapping of the motherboard. In the pin mapping, they are usually defined with abbreviations like **'STEP'**, **'DIR'** and **'EN'** respectively.
*	The **‘microsteps’** by default has the value of 16. The **’rotation_distance’** of the stepper can be calculated using the formula of **‘rotation_distance = full_steps_per_rotation * microsteps / steps_per_mm’**. 
*	The **‘endstop_pin’** is the limit pin that is defined as per the name of the pin on the motherboard. On the motherboard pin mapping they are usually defined by **'DIAG#'** pins.
* **‘position_min’** and **‘position_max’** are the minimum and maximum positions of the axis. The **‘position_endstop’** is the actual position of the endstop, which usually determines its home position and can be defined within either of the above two functions. However, the value should lie within the limits of the two values.
* **‘homing_speed’** is the speed of the travel speed when it homes.

#### [dual_carriage] ####

Dual carriage has the exact characteristics of that of the **[stepper_#]** macro where the axis of the carriage which it acts as the secondary tool is to be defined, in order to get integrated with that axis, and enable the macros and variables of switching tools and other dual carriage functions.
* **'axis'** is the additional variable that is to be mentioned that it needs to work on which particular axis.

#### [extruder] / [extruder#] ####

The similar stepper function that is used is the **[extruder]** function, which consists of common stepper functions, along with separate extruder functionalities.

* **‘nozzle_diameter’** is to be specified with the diameter of the nozzle is being used.
* **‘filament_diameter’** is the diameter of the filament that is regularly being used.
* The variables of **‘heater_pin’** and **‘sensor_pin’** are the heater pin and the thermistor pin names that are to be defined for the specific  extruder. 
* **‘sensor_type’** is the model name of the thermistor that is being used.
* The **‘control’**, **‘pid_Kp’**, **‘pid_Ki’** and **‘pid_Kd’** are the calibration values for the heater pin, in order to remain the temperature that is entered to be constant. This can however be automatically be calibrated using PID_CALIBRATE_HEATER command in the terminal, or the PID calibrate option in the OctoKlipper plugin.
* **‘min_temp’** and **‘max_temp’** are the minimum and maximum temperature targets that the hotend is allowed to proceed. **‘min_extrude_temp’** is the function to extrude the filament at the particular minimum temperature.
* **‘max_extrude_only_distance’** is the maximum distance that is extruded at once.
* **'pressure_advance'** is theamount of raw filament to push into the extruder during extruder acceleration. An equal amount of filament is retracted during deceleration. In order to get the right value in millimeters per millimeter/second, the pressure advance calibration part is to be printed with the klipper gcode input in the gcode file. For more information, refer to [Klipper pressure advanced documentation](https://www.klipper3d.org/Pressure_Advance.html?h=pressure#pressure-advance)
* **'smooth_time'** is the time value (in seconds) over which temperature measurements will time value (in seconds) over which temperature measurements will be smoothed to reduce the impact of measurement noise.
* In the electronics component selection, the thermistor that is being used for the hotend is the **PT100 amplifier** which works on the paramters of ADC conversion of voltage. Hence to get the right reading of the temperatures, the **'voltage_offset'** variable is used to vary the voltage supply to the amplifier that has the resistance of around 100 ohms.

#### [extruder_stepper] ####
This is used for the support for additional steppers synchronized to the movement of an extruder
* **'[extruder_stepper]'** has only few of the common **'[stepper]'** and **'[extruder]'** that are used.

#### [adc_temperature] ####
Custom ADC temperature sensors (one may define any number of sections with an "adc_temperature" prefix). This allows one to define a custom temperature sensor that measures a voltage on an Analog to Digital Converter (ADC) pin and uses linear interpolation between a set of configured temperature/voltage (or temperature/resistance) measurements to determine the temperature. The resulting sensor can be used as a sensor_type in a heater section. Be sure to place the sensor section in the config file above its first use in a heater section.
* **'temperature#'**, **'voltage#'** is used for mentioning a set of temperatures (in Celsius) and voltages (in Volts) to use as reference when converting a temperature. A heater section using this sensor may also specify adc_voltage and voltage_offset parameters to define the ADC voltage.


#### [verify_heater heater_config_name] ####
Heater and temperature sensor verification. Heater verification is automatically enabled for each heater that is configured on the printer. It is basically for the protection of the hotends and the printer in cases of the heater malfunction.
* **'max_error'** is the maximum "cumulative temperature error" before raising an error. Smaller values result in stricter checking and larger values allow for more time before an error is reported.
* **'check_gain_time'** controls heater verification during initial heating. Smaller values result in stricter checking and larger values allow for more time before an error is reported. Specifically, during initial heating, as long as the heater increases in temperature within this time frame (specified in seconds) then the internal "error counter" is reset.
* **'hysteresis'** is the maximum temperature difference (in Celsius) to a target temperature that is considered in range of the target. This controls the max_error range check.
* **'heating_gain'** is the minimum temperature (in Celsius) that the heater must increase by during the check_gain_time check.

#### [mcu] ####
* [mcu] function saves and activates the serial number of the motherboard and the port that is generated when the octoprint, RPi and Octopus are initiated using the SSH server terminal. 
* **'serial'** is where the serial ID is mentioned which was generated during the process of flashing of the firmware

#### [printer] ####

This function is used to save the basic printer configuration such as the acceleration, velocity in each of the axes and the kinematics of the printer profile.
* **'kinematics'** is the type of printer that is in use. In cases of IDEX, the type used is **cartesian**.
* **'max_accel'** and **'max_velocity'** are the acceleration and velocities of the toolhead in (mm/s^2) and (mm/s) respectively. 
* **'max_accel_to_decel'** is the pseudo acceleration (in mm/s^2) controlling how fast the toolhead may go from acceleration to deceleration. It is used to reduce the top speed of short zig-zag moves (and thus reduce printer vibration from these moves). The default is half of max_accel.
* **'square_corner_velocity'** is the maximum velocity (in mm/s) that the toolhead may travel a 90 degree corner at. A non-zero value can reduce changes in extruder flow rates by enabling instantaneous velocity changes of the toolhead during cornering. This value configures the internal centripetal velocity cornering algorithm; corners with angles larger than 90 degrees will have a higher cornering velocity while corners with angles less than 90 degrees will have a lower cornering velocity. If this is set to zero then the toolhead will decelerate to zero at each corner.

#### [heater_bed] ####
The heater_bed section describes a heated bed. It uses the same heater settings described in the "extruder" section.

#### [tmc2209 stepper_# / extruder#] ####
This macro is for configuring a TMC2209 stepper motor driver via single wire UART. 
* To define the paticular uart pin that the driver needs to interface with the stepper, use **'uart_pin'** and in order to refer with the pin mapping of the BTT Octopus V1.1 one needs to check the **'CS'** section of the motor pins section.
* **'run_current'** is used to define the amount of current (in amps RMS) to configure the driver to use during stepper movement. Usually the range lies between 1.000 to 1.300.

#### [fan] ####
It is the macro for defining basic print cooling fan. In order for the G-codes **'M106'** and **'M107'** to work, this section is to be enabled.
* **'pin'** is used for defining the pin that the fan is connected to on the BTT Octopus board.
* **'max_power'** is the maximum power (expressed as a value from 0.0 to 1.0) that the heater_pin may be set to. The value 1.0 allows the pin to be set fully enabled for extended periods
* **'kick_start_time'** is the time to run the fan at full speed when either first enabling or increasing it by more than 50%
* **'off_below'** is the minimum input speed which will power the fan (expressed as a value from 0.0 to 1.0). When a speed lower than off_below is requested the fan will instead be turned off. This setting may be used to prevent fan stalls and to ensure kick starts are effective.

#### [heater_fan] ####
It has the similar variables as that of the [fan] macro, along with where we have to mention the heater that the fan is associated with in order to the operate with the heaters proximity. These extra functions include:
* **'heater'** where we mention the heater that it associates with on the printer.
* **'heater_temp'** is the temperature of the associated heater that the fan should turn off below that temperature.

#### [probe] ####
This is the Z height probe macro. One may define this section to enable Z height probing hardware. When this section is enabled, PROBE and QUERY_PROBE extended g-code commands become available.
* **'pin'** is the probe detection pin. It is defined by the **'Z_OUT'** pin which triggers on the Arduino Nano based load cell probe.
* **'x_offset'**, **'y_offset'** and **'z_offset'** is the distance between the probe and the nozzle along the x, y and the z-axis.
* **'sample_retract_distance'** is the distance to lift the toolhead between each sample (if sampling more than once)
* **'speed'** is the speed of the z-axis motor when it probes.
* **'lift_speed'** is the speed of the Z-axis of the when lifting the probe between samples.
* **'samples_tolerance'** is the maximum Z distance (in mm) that a sample may differ from other samples. If this tolerance is exceeded then either an error is reported or the attempt is restarted (see samples_tolerance_retries).
* **'samples'** is the number of times to probe each point. The probed z-values will be averaged.
* **'samples_result'** is the calculation method when sampling more than once - either "median" or "average".
* **'samples_tolerance_retries'** is the number of times to retry if a sample is found that exceeds samples_tolerance. On a retry, all current samples are discarded and the probe attempt is restarted. If a valid set of samples are not obtained in the given number of retries then an error is reported. The default is zero which causes an error to be reported on the first sample that exceeds samples_tolerance.

#### [bed_mesh] ####
One may define a bed_mesh config section to enable move transformations that offset the z axis based on a mesh generated from probed points.
* **'horizontal_move_z'** is the height (in mm) that the head should be commanded to move to just prior to starting a probe operation.
* **'mesh_min'** defines the minimum X, Y coordinate of the mesh for rectangular beds. This coordinate is relative to the probe's location. This will be the first point probed, nearest to the origin.
* **'mesh_max'** defines the maximum X, Y coordinate of the mesh for rectangular beds. Adheres to the same principle as mesh_min, however this will be the furthest point probed from the bed's origin.
* **'probe_count'** are the integer values X, Y defining the number of points to probe along each axis. A single value is also valid, in which case that value will be applied to both axes.
* **'algorithm'** is the interpolation algorithm to use.

#### [gcode_macro] ####
This command is to add a custom G-code with any name, under which the behaviour of that g-code is to be defined by the particular or set of g-codes that is identifiable by Klipper. It is defined in the format **'[gcode_macro custom_name]'**.
* **'gcode'** is the list of G-Code commands to execute in place of "custom_name".
* **'rename_existing'** is the option that will cause the macro to override an existing G-Code command and provide the previous definition of the command via the name provided here. This can be used to override builtin G-Code commands. Care should be taken when overriding commands as it can cause complex and unexpected results.

#### [respond] ####
This macro is used in order to enable the gcode **'M118'** which is used to display a message or an echo in the terminal of the host.
* **'default_type'** sets the default prefix output to **'echo: "echo: "'** or **'command: "// "'** or **'error: "!! "'**.

#### [pause_resume] ####
Pause/Resume functionality with support of position capture and restore. It enables the gcodes **'PAUSE'**, **'RESUME'**, **'CANCEL_PRINT'** and **'CLEAR_PAUSE'**
* **'recover_velocity'** is the speed at which to return to the captured position (in mm/s)

#### [save_variables] ####
Support saving variables to disk so that they are retained across restarts. It creates a seperate file to store the additional variables in order to re-use them after restarting the firmware.

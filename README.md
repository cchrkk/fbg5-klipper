# Klipper+ Fluidd su Flyingbear Ghost 5 + RPI3B
Guida fatta _davvero_ male e incompleta per klipper+ fluidd su Flyingbear Ghost 5
![image](https://user-images.githubusercontent.com/12370126/154228285-e5f54d5d-913e-4d7f-8536-2bb8a0b2fa33.png)

# In pratica

Personalmente uso DietPi invece di Raspbian su un vecchio RPI3B

In pratica:
- Ho disabilitato i container docker di octoprint/mjpg-streamer già presenti sul raspberry 
- Ho scaricato KIAU per poi installare Klipper, Moonraker e Fluidd https://klipper-italia.xyz/installazione-rpi/installazione-kiauh/
- Una volta installato tutto, ho compilato il firmware usando le impostazioni per la mia scheda madre (Nel mio caso STM32F103 quindi https://flyingbear.info/firmware/klipper/mcu_firmware#mks-robin-nano-v11 )
- Ho criptato e rinominato il fw con 
```
~/klipper/scripts/update_mks_robin.py ~/klipper/out/klipper.bin ~/klipper/out/Robin_nano35.bin
``` 
(pare necessario per Robin nano 1.1)

- Ho messo il fw ~/klipper/out/Robin_nano35.bin su microsd, ho riavviato la stampante ed ha flashato il nuovo firmware bloccandosi al 100%
- Ho iniziato a creare la cfg della stampante con fluidd andando a http://IP-LOCALE-RASPBERRY
- Ho seguito questa guida russa con google translate per creare la config stando attento alle posizioni dei pin per i miei driver XYZE (Avendo TMC2208 su X / Y e A4988 su Z / E ) https://flyingbear.info/ru/firmware/klipper
- Ho riavviato la stampante, dopo il beep, schermo nero quindi tutto perfetto
- Nel mio caso ho avuto problemi ad accedere a /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 anche se esisteva, (forse problema di DietPi/permessi) ma non ho avuto problemi impostando /dev/ttyUSB0
- Ho graffiato il piatto magnetico 20€, quindi FARE CALIBRAZIONE VITI PER PRIMA COSA 🐷😇,😇🐷!
- Ho fatto stampa di prova e nonostante mi sia sembrato che andasse a 200 km/h sembrava tutto ok

![image](https://user-images.githubusercontent.com/12370126/154228285-e5f54d5d-913e-4d7f-8536-2bb8a0b2fa33.png)

# Printer.cfg
[QUI ALLEGO IL FIRMWARE PRECOMPILATO](https://github.com/cherokee93/fbg5-klipper/raw/main/Robin_nano35.bin) da mettere su microsd nel caso tu abbia una STM32F103


QUI SOTTO ALLEGO LA MIA printer.cfg

*N.B. Ho driver TMC2208 su X / Y e A4988 su Z / E e scheda madre STM32F103*


```
[stepper_x]
step_pin: PE3
dir_pin: PE2                            # !PE2 for A4988, PE2 for TMC22**
enable_pin: !PE4
microsteps: 16
rotation_distance: 40
endstop_pin: !PA15
position_endstop: 0
position_max: 255                       # X-axis bed size
homing_speed: 50

[stepper_y]
step_pin: PE0
dir_pin: PB9                            # !PB9 for A4988, PB9 for TMC22**
enable_pin: !PE1
microsteps: 16
rotation_distance: 40
endstop_pin: !PA12
position_endstop: 0
position_max: 210                       # Y-axis bed size
homing_speed: 50

[stepper_z]
step_pin: PB5
dir_pin: PB4                           # PB4 for A4988, !PB4 for TMC22**
enable_pin: !PB8
microsteps: 16
rotation_distance: 8
endstop_pin: !PA11
position_endstop: 0.5
position_max: 200                       # Z-axis print volume size
homing_speed: 20

[extruder]
step_pin: PD6
dir_pin: PD3                           # PD3 for A4988, !PD3 for TMC22**
enable_pin: !PB3
microsteps: 16
rotation_distance: 7.880
nozzle_diameter: 0.400
filament_diameter: 1.750
max_extrude_only_distance: 100
pressure_advance: 0.000
heater_pin: PC3
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PC1
control: pid
pid_Kp: 14.669
pid_Ki: 0.572
pid_Kd: 94.068
min_temp: 0
max_temp: 260

[heater_bed]
heater_pin: PA0
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PC0
control: pid
pid_Kp: 325.10
pid_Ki: 63.35
pid_Kd: 417.10
min_temp: 0
max_temp: 130

[fan]
pin: PB1

[heater_fan heater_fan]
pin: PB0

[output_pin BEEPER_pin]
pin: PC5
pwm: True
value: 0
shutdown_value: 0
cycle_time: 0.001
scale: 1000

[filament_switch_sensor filament_sensor]
switch_pin: PA4
runout_gcode:
    BEEP P=1500

[virtual_sdcard]
path: ~/gcode_files

[pause_resume]

[display_status]

[firmware_retraction]
retract_length: 0
retract_speed: 35

[respond]

[bed_screws]
screw1: 25,30
screw1_name: front left screw
screw2: 230,30
screw2_name: front right screw
screw3: 230,180
screw3_name: back right screw
screw4: 25,180
screw4_name: back left screw
speed: 150

[mcu]
#serial: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
serial: /dev/ttyUSB0
restart_method: command

[printer]
kinematics: cartesian
max_velocity: 250
max_accel: 3000
max_accel_to_decel: 2000
max_z_velocity: 20
max_z_accel: 100

#MACROS
[gcode_macro START_PRINT]
variable_retract: 5
gcode:
    {% set extruder_temp = params.EXTRUDER_TEMP|default(240)|float %}
    {% set bed_temp = params.BED_TEMP|default(70)|float %}
    {% set E = printer["gcode_macro START_PRINT"].retract|float %}
    CLEAR_PAUSE
    M220 S100                                                                   # reset feedrate
    M221 S100                                                                   # reset flowrate
    SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET={bed_temp}                  # set bed t℃
    TEMPERATURE_WAIT SENSOR=heater_bed MINIMUM={bed_temp * 0.75}                # wait until bed is partially heated
    SET_HEATER_TEMPERATURE HEATER=extruder TARGET={extruder_temp}               # set nozzle t℃
    G90                                                                         # absolute positioning
    M82                                                                         # absolute extrusion mode
    TEMPERATURE_WAIT SENSOR=heater_bed MINIMUM={bed_temp}                       # wait until
    TEMPERATURE_WAIT SENSOR=extruder MINIMUM={extruder_temp}                    # wait until
    G28                                                                         # home
    G0 Z10 F1500                                                                # raise Z
    G92 E0                                                                      # reset extruder
    G1 E{E} F1500                                                               # prime
    G92 E0                                                                      # reset extruder

[gcode_macro END_PRINT]
gcode:
    {% set E = printer["gcode_macro START_PRINT"].retract|float %}
    TURN_OFF_HEATERS
    M107                                                                        # turn off fan
    G91                                                                         # relative positioning
    G1 E-{E} F1500                                                              # retract
    G0 X5 Y5 Z0.2 F5000                                                         # wipe
    G0 Z2 F1500                                                                 # raise Z
    G90                                                                         # absolute positioning
    PARK
    M84                                                                         # turn off all motors
    BEEP P=200 S=250

[gcode_macro PARK]
gcode:
    {% set x_park = params.X|default(0)|float %}
    {% set y_park = params.Y|default(-4)|float %}
    {% set z_park = params.Z|default(10)|float + printer.toolhead.position.z|float %}
    {% set x_max = printer.toolhead.axis_maximum.x|float %}
    {% set y_max = printer.toolhead.axis_maximum.y|float %}
    {% set z_max = printer.toolhead.axis_maximum.z|float %}
    {% if x_park > x_max %}
        {% set x_park = x_max %}
    {% endif %}
    {% if y_park > y_max %}
        {% set y_park = y_max %}
    {% endif %}
    {% if z_park > z_max %}
        {% set z_park = z_max %}
    {% endif %}
    SAVE_GCODE_STATE NAME=PARK_STATE
    G90                                                                         # absolute positioning
    G1 Z{z_park} F1500
    G1 X{x_park} Y{y_park} F5000
    RESTORE_GCODE_STATE name=PARK_STATE

[gcode_macro PAUSE]
rename_existing: BASE_PAUSE
gcode:
    {% set E = printer["gcode_macro START_PRINT"].retract|float %}
    SAVE_GCODE_STATE NAME=PAUSE_STATE
    BASE_PAUSE
    G91
    G1 E-{E} F1500                                                              # retract
    G90
    PARK

[gcode_macro RESUME]
rename_existing: BASE_RESUME
gcode:
    {% set E = printer["gcode_macro START_PRINT"].retract|float %}
    G91
    G1 E{E} F1500                                                               # unretract
    G90
    RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1
    BASE_RESUME

[gcode_macro CANCEL_PRINT]
rename_existing: BASE_CANCEL_PRINT
gcode:
    {% set E = printer["gcode_macro START_PRINT"].retract|float %}
    TURN_OFF_HEATERS
    M107                                                                        # turn off fan
    G91
    G1 E-{E} F1500                                                              # retract
    G90
    CLEAR_PAUSE
    SDCARD_RESET_FILE
    BASE_CANCEL_PRINT
    PARK

[gcode_macro BEEP]
gcode:
    {% set frequency = params.S|default(1000)|float %}
    {% set duration = params.P|default(100)|float %}
    SET_PIN PIN=BEEPER_pin VALUE={frequency}
    G4 P{duration}
    SET_PIN PIN=BEEPER_pin VALUE=0

[gcode_macro FILAMENT_CHANGE]
gcode:
    SAVE_GCODE_STATE NAME=FILAMENT_CHANGE_STATE
    {% set timer = params.T|default(300)|float %}
    {% set unload = params.U|default(100)|float %}
    {% set load = params.L|default(100)|float %}
    {% if printer.pause_resume.is_paused %}
        M118 Already paused
    {% else %}
        {% if printer.toolhead.homed_axes != "xyz" %}
            M118 Homing
            G28                                                                 # home if not homed
        {% else %}
            M118 Pausing print
            PAUSE
        {% endif %}
    {% endif %}
    M118 Changing filament
    SET_IDLE_TIMEOUT TIMEOUT=7200
    FILAMENT_UNLOAD U={unload}
    COUNTDOWN TIME={timer} MSG="Change filament! Time left: "
    FILAMENT_LOAD L={load}
    RESTORE_GCODE_STATE NAME=FILAMENT_CHANGE_STATE
    {% if printer.pause_resume.is_paused %}
        M118 Resuming print
        RESUME
    {% endif %}

[gcode_macro FILAMENT_LOAD]
gcode:
    {% set load = params.L|default(100)|float * 0.5 %}
    {% set extruder_temp = params.T|default(240)|float %}
    SAVE_GCODE_STATE NAME=FILAMENT_LOAD_STATE
    LOW_TEMP_CHECK T={extruder_temp}
    M118 Loading filament
    M83                                                                         # relative extrusion
    G1 E{load} F1500                                                            # extrude fast
    G4 P1000                                                                    # wait 1 second
    G1 E{load} F200                                                             # extrude slow
    BEEP
    RESTORE_GCODE_STATE NAME=FILAMENT_LOAD_STATE

[gcode_macro FILAMENT_UNLOAD]
gcode:
    {% set unload = params.U|default(100)|float %}
    {% set extruder_temp = params.T|default(180)|float %}
    SAVE_GCODE_STATE NAME=FILAMENT_UNLOAD_STATE
    LOW_TEMP_CHECK T={extruder_temp}
    M118 Unloading filament
    M83                                                                         # relative extrusion
    G1 E2  F200                                                                 # extrude a little
    G1 E-10  F200                                                               # retract a little
    G1 E-{unload} F1500                                                         # retract a lot
    BEEP
    RESTORE_GCODE_STATE NAME=FILAMENT_UNLOAD_STATE

[gcode_macro LOW_TEMP_CHECK]
gcode:
    {% set extruder_temp = params.T|default(240)|float %}
    {% if printer.extruder.target > extruder_temp %}                            # if there is a setpoint for extruder
        {% set extruder_temp = printer.extruder.target %}
    {% endif %}
    {% if printer.extruder.temperature < extruder_temp %}                       # heat to the target
        M118 Heating to {extruder_temp}
        SET_HEATER_TEMPERATURE HEATER=extruder TARGET={extruder_temp}
        TEMPERATURE_WAIT SENSOR=extruder MINIMUM={extruder_temp}
    {% endif %}

[gcode_macro COUNTDOWN]
gcode:
    {% set timer = params.TIME|default(10)|int %}
    {% set message = params.MSG|default("Time: ") %}
    # countdown
    {% if timer > 60 %}
        {% for s in range(timer, 60, -10) %}
            M118 {message} {s}s
            G4 P10000                                                           # dwell 10 seconds
        {% endfor %}
        {% set timer = 60 %}
    {% endif %}
    {% if timer > 10 %}
        {% for s in range(timer, 10, -5) %}
            M118 {message} {s}s
            G4 P5000                                                           # dwell 5 seconds
        {% endfor %}
        {% set timer = 10 %}
    {% endif %}
    {% if timer > 0 %}
        {% for s in range(timer, 0, -1) %}
            M118 {message} {s}s
            G4 P1000                                                           # dwell 1 second
        {% endfor %}
    {% endif %}
    BEEP

[gcode_macro M76]                                                               # Marlin Compatibility
gcode:
    PAUSE

[gcode_macro M601]                                                              # Prusa Compatibility
gcode:
    PAUSE

[gcode_macro G27]                                                               # Marlin Compatibility
gcode:
    PARK

[gcode_macro M125]                                                              # Marlin Compatibility
gcode:
    PARK

[gcode_macro M300]                                                              # Marlin Compatibility
gcode:
    BEEP

[gcode_macro M600]                                                              # Marlin Compatibility
gcode:
    FILAMENT_CHANGE

[gcode_macro M701]                                                              # Marlin Compatibility
gcode:
    FILAMENT_LOAD

[gcode_macro M702]                                                              # Marlin Compatibility
gcode:
    FILAMENT_UNLOAD

[gcode_macro M92]
gcode:
    {% set extruder_steps = params.E|default(0.0025)|float %}
    SET_EXTRUDER_STEP_DISTANCE DISTANCE={extruder_steps}

[gcode_macro M500]
gcode:
    SAVE_CONFIG

[gcode_macro M204]
rename_existing: M204.1
gcode:
    {% set factor = params.F|default(0.5)|float %}
    {% if 'S' in params %}
        SET_VELOCITY_LIMIT ACCEL={S} ACCEL_TO_DECEL={ S|float * factor }
    {% else %}
        {% if 'P' in params %}
            {% if 'T' in params %}
                {% if P|int < T|int %}
                    SET_VELOCITY_LIMIT ACCEL={P} ACCEL_TO_DECEL={ P|float * factor }
                {% else %}
                    SET_VELOCITY_LIMIT ACCEL={T} ACCEL_TO_DECEL={ T|float * factor }
                {% endif %}
            {% else %}
                SET_VELOCITY_LIMIT ACCEL={P} ACCEL_TO_DECEL={ P|float * factor }
            {% endif %}
        {% elif 'T' in params %}
            SET_VELOCITY_LIMIT ACCEL={T} ACCEL_TO_DECEL={ T|float * factor }
        {% endif %}
    {% endif %}

[gcode_macro M205]
gcode:
    {% if 'J' in params %}
        SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={J}
    {% elif 'X' in params %}
        SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={X}
    {% elif 'Y' in params %}
        SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={Y}
    {% endif %}

[gcode_macro M207]
gcode:
    {% set length = params.S|default(0.5)|float %}
    {% set speed = params.F|default(25)|float %}
    SET_RETRACTION RETRACT_LENGTH={length} RETRACT_SPEED={speed}

[gcode_macro M900]
gcode:
    {% if 'K' in params %}
        SET_PRESSURE_ADVANCE ADVANCE={ params.K|float }
    {% endif %}

[gcode_macro M303]
gcode:
    {% if 'E' in params %}
        {% set heater = params.E|default(0)|int %}
        {% set temp = params.T|default(0)|float %}
        {% if heater == 0 %}                                                    # extruder
            {% if temp >= printer.configfile.settings.extruder.min_extrude_temp|float %}
                {% if temp <= printer.configfile.settings.extruder.max_temp|float %}
                    PID_CALIBRATE HEATER=extruder TARGET={T}
                {% endif %}
            {% endif %}
        {% elif heater == -1 %}                                                 # bed
            {% if temp <= printer.configfile.settings.heater_bed.max_temp|float %}
                PID_CALIBRATE HEATER=heater_bed TARGET={T}
            {% endif %}
        {% endif %}
    {% endif %}

[gcode_macro M486]
gcode:
    # Do nothing

[gcode_macro PRIME_LINE]
gcode:
    {% set feedrate = params.F|default(10)|float * 60 %}
    {% set length = 100.0 %}
    {% set width = printer.configfile.settings.extruder.nozzle_diameter|float %}
    {% set height = ( (width / 0.04)|int - (width / 0.04 / 4)|int )|float * 0.04 %}
    {% set extrude = length * width * height / 1.6 %}
    SAVE_GCODE_STATE NAME=PRIME_LINE_STATE
    SET_IDLE_TIMEOUT TIMEOUT=7200
    {% if 'Y' in params %}
        {% set x_start = 1.0 %}
        {% set y_start = (printer.toolhead.axis_maximum.y|float - 100) / 2 %}
        G0 X{x_start} Y{y_start} F5000                                          # move to start position
        G0 Z{height} F1500
        G91                                                                     # relative positioning
        G1 Y100 E{extrude} F{feedrate}                                          # draw the 1st line
        G0 X{width} F5000                                                       # move to the next line
        G1 Y-100 E{extrude} F{feedrate}                                         # draw the 2nd line
    {% else %}
        {% set x_start = (printer.toolhead.axis_maximum.x|float - 100) / 2 %}
        {% set y_start = 1.0 %}
        G0 X{x_start} Y{y_start} F5000                                          # move to start position
        G0 Z{height} F1500
        G91                                                                     # relative positioning
        G1 E4 F{feedrate}                                                       # prime
        G1 X100 E{extrude} F{feedrate}                                          # draw the 1st line
        G0 Y{width} F5000                                                       # move to the next line
        G1 X-100 E{extrude} F{feedrate}                                         # draw the 2nd line
    {% endif %}
    RESTORE_GCODE_STATE NAME=PRIME_LINE_STATE
```

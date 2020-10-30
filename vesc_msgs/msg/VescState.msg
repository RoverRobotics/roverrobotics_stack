# Vedder VESC open source motor controller state (telemetry)

# fault codes
int FAULT_CODE_NONE=0
int FAULT_CODE_OVER_VOLTAGE=1
int FAULT_CODE_UNDER_VOLTAGE=2
int FAULT_CODE_DRV8302=3
int FAULT_CODE_ABS_OVER_CURRENT=4
int FAULT_CODE_OVER_TEMP_FET=5
int FAULT_CODE_OVER_TEMP_MOTOR=6

double voltage_input        # input voltage (volt)
double temperature_pcb      # temperature of printed circuit board (degrees Celsius)
double current_motor        # motor current (ampere)
double current_input        # input current (ampere)
double speed                # motor electrical speed (revolutions per minute) 
double duty_cycle           # duty cycle (0 to 1)
double charge_drawn         # electric charge drawn from input (ampere-hour)
double charge_regen         # electric charge regenerated to input (ampere-hour)
double energy_drawn         # energy drawn from input (watt-hour)
double energy_regen         # energy regenerated to input (watt-hour)
double displacement         # net tachometer (counts)
double distance_traveled    # total tachnometer (counts)
int   fault_code

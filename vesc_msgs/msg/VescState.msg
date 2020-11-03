# Vedder VESC open source motor controller state (telemetry)

# fault codes
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV8302=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

int32 motorid #Id of which vesc the data is coming from
float64 temp_mos        
float64 temp_motor      
float64 current_motor        # motor current (ampere)
float64 current_input        # input current (ampere)
float64 id
float64 iq
float64 duty_now            
float64 rpm          
float64 v_in                  
float64 amp_hours    
float64 amp_hours_charged
float64 watt_hours
float64 watt_hours_charged
int32 tachometer   
int32 tachometer_abs
float64 position  
uint8   fault_code
float64 temp_mos_1
float64 temp_mos_2
float64 temp_mos_3
float64 vd
float64 vq
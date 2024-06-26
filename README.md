Serial Comms to GSM90 magnetometer for std absolute observations with TFT touch screen display

assumes

        : Arduino Mega 2560 R3
        : TFT LCD TouchScreen 240 x 320  (Jaycar XC4630)
        : DFRobot RS-232 shield (Core Electronics DFR0258)
        : DFRobot DS1307 "Gravity" RTC module
        
        : Adafruit_GFX library; TouchScreen library; EEPROM Library; DFRobit RTC library
        : MCUFriend-kbv library
        : The RT clock module can be powered from VCC and GND pins accessible on RS-232 shield but this
        : requires and angled 2 or 4-pin header block to connect onto the RS-232 pins.
        
 Menu control via TouchScreen buttons to set RTC clock time, adjust Baud_rate, 
 adjust magnetometer_tune_value (in microTesla) and adjust number_of_obs_per_run. 
 The current value of the latter three variables are stored to persistent EEPROM
 whenever they are updated and are used after reboot/power cycle.
 
 RS-232 parity, data_bits, stop_bits are hardwired into code as N81
 Magnetometer sampling interval is hardwired at about 10 seconds

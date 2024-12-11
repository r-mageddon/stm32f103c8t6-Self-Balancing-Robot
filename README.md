PID controlled self balancing robot using stm32cubeIDE on a Wheeltec standard edition self balancing trolley. Robot can be found here: https://www.aliexpress.com/item/1005004991832304.html

All the code (MPU6050 set up and reads, PID controller etc.) is contained in the main.c file.

A couple of notes to take advice of:
- the board is programmed at 72MHz HSE
- I2C is in fast mode
- I2C is remapped to pins PB8 and PB9
- Timer 4 activates a 500ns interrupt (prescaler = 720, Preload register = 50)
- no encoders are used
- robot uses 2 12V encoded brushed DC motors
- MPU6050 falls across the y-axis direction
- OLED screen is not used due to Wheeltec made code programming it using software SPI
- PID controller works best with dt being sampling frequency over sampling time (dt = 2000), meaning I gain is divided by dt and D gain is multiplied by dt
- motors worked faster when moving forward, this is why there is a 0.9 product in the output to the motors during forward movement
- PWM is set to work at period of 100 (prescaler = 9, ARR = 100)
  
End result was robot not balancing due to PID values but it is functional, hopefully this is able to help you with your robot :)

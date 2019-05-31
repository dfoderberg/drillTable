cmd option U to upload
cmd option R to verify
cmd shift P then arduino to go to menu
must select board and serial port to upload

calibrate all the speed settings
recieve which speed setting is coming across

add in switch for manual state
add in h bridge for both motors
add davis's special menu switch
add direction to motor in and motor out function and serial communication
change all strings to c strings == https://hackingmajenkoblog.wordpress.com/2016/02/04/the-evils-of-arduino-strings/

calculate jam speeds for all settings
sync motor distance
fix communication between boards - need to check that both motors started? or just make sure that if only 1 motor is moving it stops to wait for the other one that is not moving. during the wait it checks to see if other is moving if it doesnt i need to send special error message.

maybe add a possitive exchange of comunication between boards. It sends X then waits to recieve X then verifies that The returning transmision was correct.

Do i need to open up multiple serial ports?

System Diagnostic When Manual state on and Start is held for 5 seconds.









Relay Tutorial - https://howtomechatronics.com/tutorials/arduino/control-high-voltage-devices-arduino-relay-tutorial/

LCD Tutorial - http://theelectromania.blogspot.com/2016/01/clock-and-calendar-using-arduino-due.html

Serial Commuication Tutorial - https://learn.sparkfun.com/tutorials/serial-communication/all

Serial Communication Part 2 - https://forum.arduino.cc/index.php?topic=396450


Parts 20x4 i2c display Amazon- https://www.amazon.com/JANSANE-Arduino-Display-Interface-Raspberry/dp/B07D7ZQVDR/ref=sr_1_5?keywords=20x4+i2c&qid=1559062736&s=gateway&sr=8-5


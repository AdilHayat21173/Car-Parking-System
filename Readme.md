Which components are used to make a car?
- Arduino Uno: A popular microcontroller board in the Arduino family, featuring an ATmega328P chip and various I/O pins, used for prototyping electronics projects with USB connectivity for programming and power.
- Motor Controller: An electronic device or circuit enabling control of motor speed and direction, commonly applied in robotics and automation for precise motor manipulation.
- Bluetooth Controller: A Bluetooth module facilitating wireless communication between Arduino boards and external devices like smartphones or computers or mobile , enabling remote control and data exchange.

You can control the car through a Bluetooth controller, connecting it to a mobile phone to perform forward, backward, left, and right movements remotely.

For car parking, we utilize the following components:
- Arduino LCD I2C: This module features an LCD (Liquid Crystal Display) with an integrated I2C interface, requiring only four wires (VCC, GND, SDA, SCL) to connect to an Arduino board. It simplifies wiring and allows for the display of text and graphics on the screen.

- Breadboard: A rectangular board with a grid of holes, utilized for prototyping electronic circuits. It enables easy insertion and connection of components without soldering, providing a temporary platform for circuit experimentation.

- IR Sensor: An electronic device capable of detecting infrared radiation emitted or reflected by objects. IR sensors are commonly employed in Arduino projects for tasks such as proximity sensing, object detection, and remote control applications.

- Servo Motor: A specialized motor capable of rotating to a precise angle based on the control signal it receives. Servo motors find extensive use in Arduino projects for accurately controlling motion in applications such as robotic systems and remote-controlled vehicles.

- Jumper Wires: Short wires equipped with connector pins at each end, employed to create temporary connections between components on a breadboard or between a breadboard and external elements. Jumper wires facilitate rapid prototyping and experimentation by enabling swift and simple connections in electronic circuits.

How the Car parking system are working ?
In the car parking system, two IR sensors and a liquid crystal display (LCD) are utilized. Initially, the LCD displays the total number of available slots, with 5 slots being occupied and 5 slots remaining.

As a car enters the parking area, the first sensor detects the car, prompting the corresponding rod to open, allowing the car to enter. Subsequently, the second sensor senses the car's presence, closes the rod, and decrements the available slot count to 4.

When another car approaches the parking area, the available slot count continues to decrease as each car enters. Conversely, when a car exits the parking area, the sensors detect its departure, incrementing the available slot count and indicating an increase in available space.

If a car attempts to enter but no space is available in the parking area, the sensors prevent the corresponding rod from opening due to the lack of available space.

Overall, the system effectively manages the entry and exit of cars from the parking area, accurately displaying the available slots on the LCD.


Explanation of the Code?
Code Explanation of the Car:
1. **Including Libraries**:
   - The `#include <SoftwareSerial.h>` statement at the beginning of the code imports the SoftwareSerial library. This library enables the Arduino to communicate with devices using serial communication (UART) on digital pins other than the hardware serial pins (usually pins 0 and 1).

2. **Defining Pin Assignments**:
   - The code assigns specific digital pins on the Arduino board for various purposes:
     - Pins `leftMotorForward`, `leftMotorBackward`, `rightMotorForward`, and `rightMotorBackward` are used to control the direction of the motors connected to the L298N motor driver. These pins determine whether the motors spin forward or backward.
     - Pins `S_A` and `S_B` are designated as speed control pins for motor A and motor B, respectively. By modulating the PWM (Pulse Width Modulation) signals on these pins, the speed of the motors can be adjusted.
     - `bluetoothRx` and `bluetoothTx` represent the receive (RX) and transmit (TX) pins of the Bluetooth module. These pins are used to establish serial communication between the Arduino and the Bluetooth module.

3. **Initializing SoftwareSerial and Motor Pins**:
   - In the `setup()` function, the code configures the previously defined pins:
     - The `pinMode()` function sets the motor control pins (`leftMotorForward`, `leftMotorBackward`, `rightMotorForward`, `rightMotorBackward`) as outputs, indicating that these pins will be used to send control signals to the L298N motor driver.
     - The `analogWrite()` function sets an initial speed for both motors by applying a PWM signal to the speed control pins (`S_A` and `S_B`). This allows the motors to start at a predefined speed.
     - Serial communication is initialized using the `begin()` function for both the hardware serial (via `Serial.begin()`) and the software serial (via `bluetooth.begin()`) to communicate with the Bluetooth module.

4. **Main Loop**:
   - The `loop()` function, which runs continuously after the setup, constantly checks if there are any incoming commands from the Bluetooth module.
   - If the Bluetooth module has data available (`bluetooth.available() > 0`), it reads the received command using `bluetooth.read()` and passes it to the `handleBluetoothCommand()` function for further processing.

5. **Command Handling**:
   - The `handleBluetoothCommand()` function receives a single character command from the Bluetooth module and interprets it using a `switch` statement.
   - Depending on the received command (`'F'`, `'B'`, `'L'`, `'R'`, or `'S'`), the corresponding movement function (`moveForward()`, `moveBackward()`, `turnLeft()`, `turnRight()`, or `stopMotors()`) is called.

6. **Movement Functions**:
   - These functions control the direction of the motors based on the received commands:
     - `moveForward()`, `moveBackward()`, `turnLeft()`, and `turnRight()` functions set the appropriate motor control pins to make the robot move forward, backward, turn left, or turn right.
     - The `stopMotors()` function sets all motor control pins low, effectively stopping the movement of the robot.

Code Explanation of the CarParing:

1. **Including Libraries**:
   - The code includes two libraries:
     - `LiquidCrystal.h`: This library enables communication with LCD displays.
     - `Servo.h`: This library facilitates servo motor control.

2. **Initializing LCD and Servo Motor**:
   - An instance of the `LiquidCrystal` class is created, initializing the LCD with the interface pin numbers (A0-A5).
   - An instance of the `Servo` class is created for controlling a servo motor.

3. **Defining Pin Assignments**:
   - Pin numbers for the two IR sensors (`ir_s1` and `ir_s2`) are defined.

4. **Variable Declaration**:
   - `Total` represents the total number of parking spaces available.
   - `Space` represents the number of available parking spaces.
   - `flag1` and `flag2` are flags used to track the status of the IR sensors.

5. **Setup Function**:
   - Pin modes for the IR sensors are set as inputs.
   - The servo motor is attached to a pin and initialized to an initial position.
   - The LCD is initialized with the desired dimensions and a welcome message is displayed.
   - The initial value of `Space` is set to `Total`.

6. **Loop Function**:
   - The `loop()` function continuously monitors the status of the IR sensors and updates the LCD display accordingly.
   - If the IR sensor detects an object and there is available space, the servo motor opens the parking space and updates the `Space` variable.
   - If a car exits and there are no flags set, the servo motor closes the parking space and updates the `Space` variable.
   - If both entry and exit flags are set, indicating a car has entered and exited, the servo motor resets to its initial position and the flags are reset.

This code creates a simple parking system using infrared sensors, a servo motor, and an LCD display to monitor and display the availability of parking spaces.


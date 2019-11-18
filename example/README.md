## Simple example

Example, tested with `NUCLEO F411RE`,  to create a `Publisher` (`/chatter`: send text) and a `Subscriber` (`/toggle_led`: toggle the integrated LED) :

1. Create [`mainpp.h`](./mainpp.h) and [`mainpp.cpp`](./mainpp.cpp) files in the project :
  - Copy [`mainpp.h`](./mainpp.h) content in `Inc/mainpp.h` :
  - Copy [`mainpp.cpp`](./mainpp.cpp) content in `Src/mainpp.cpp` :

2. Add in `main.c` :
- Include `mainpp.h` :
    ```c++
    /* USER CODE BEGIN Includes */
    #include "mainpp.h"
    /* USER CODE END Includes */
    ```
  
- Add `setup()` and `loop()` functions :
    ```cpp
    /* USER CODE BEGIN 2 */
    setup();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      loop();
    }
    /* USER CODE END 3 */
    ```

3. Compile program and flash it to microcontroller

4. Launch `roscore`, `rosserial` and test :
- First terminal :
    ```sh
    $ roscore
    ```
- Second terminal, launch `rosserial` with the correct `serial port` (`ttyS3` here):
    ```sh
    $ rosrun rosserial_python serial_node.py _port:=/dev/ttyS3 _baud:=115200
    ```
- Third terminal:
  - List the topic of the STM32 :
    ```sh
    $ rostopic list
    ```
  - Echo what the node has to say :
    ```sh
    $ rostopic echo /chatter
    ```
  - Toggle the integrated LED :
    ```sh
    $ rostopic pub /toggle_led -1 std_msgs/Empty
    $ rostopic pub /toggle_led -r 2 std_msgs/Empty  # Rate 2 Hz
    ```

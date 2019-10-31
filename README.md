# ROSserial on STM32F4

ROSserial implementation for `STM32F4`, developed to work with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) projects.

Heavily based on [yoneken's rosserial_stm32](https://github.com/yoneken/rosserial_stm32), which is in part based on [rosserial_mbed](https://github.com/ros-drivers/rosserial/tree/melodic-devel/rosserial_mbed).
Also exists for [STM32F7](https://github.com/fdila/rosserial_stm32f7), but has not been tested.


## Generate code

```sh
$ cd path/to/your/stm32/project/Core
$ rosrun rosserial_stm32f4 make_libraries.py .
```

## Usage

1. Create a new STM32CubeIDE project :
    - Choose `C++` as `Targeted Language`
    - Choose `Yes` for "Initialize all peripherals with their default Mode ?"

2. Configure microcontroller :
    - Enable `USART2` global interrupt
    - Enable `DMA` for `USART2_TX` and `USART2_RX` and set their priority to `HIGH`
    - Generate code (+ [checking](#check-generated-code))

3. Create `ROS` libraries in your project :
    ```sh
    $ cd your/catkin/workspace/src
    $ git clone https://github.com/......./rosserial_stm32f4
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    $ cd path/to/your/stm32/project/Core
    $ rosrun rosserial_stm32f4 make_libraries.py .
    ```
    (`rosserial` should already be installed, if not : `sudo apt-get install ros-<distro>-rosserial`)

4. Add `ros_lib` to the default paths for compilation :
- Open `Project / Properties` window
- Add in `C/C++ / Settings / Tool Settings / Include paths`: `../Core/Inc/ros_lib`

### Check generated code

This implementation use [`DMA`](https://embedds.com/using-direct-memory-access-dma-in-stm23-projects/) for the serial interface.

Sometimes, the generated code initialises `DMA` and `USART` in the wrong order.

To correcly work, `MX_DMA_Init()` should be before `MX_USART2_UART_Init()`. If not, add `MX_DMA_Init()` between the `Init` brackets :
```cpp
  /* USER CODE BEGIN Init */
  MX_DMA_Init();
  /* USER CODE END Init */
```

---
## Exemples

See a [simple example](./example) with a `Publisher` and a `Subscriber`.

Also, see [yoneken's examples](https://github.com/yoneken/rosserial_stm32/tree/master/src/ros_lib/examples)

---
## Usage for other STM32 series

This package can easily be modified to be used for other STM32 series, by updating only one file :

In `ros_lib/STM32Hardware.h`, change `?` to the number of the serie or the `USART` used for the project :

- Change the library files :
```c
#include "stm32f?xx_hal.h"
#include "stm32f?.xx_hal_uart.h"
#include "stm32f?.xx_hal_tim.h"
```

- Change the number of the UART used :
```cpp
extern UART_HandleTypeDef huart?;
```

- And in the class constructor:
```c
STM32Hardware():
  huart(&huart?), rind(0), twind(0), tfind(0){
}
```
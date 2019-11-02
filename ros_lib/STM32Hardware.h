/*
 * STM32Hardware.h
 *
 *  Created on: Nov 2, 2019
 *      Author: xav-jann1
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "BufferedSerial.hpp"
extern UART_HandleTypeDef huart2;

// Create Serial Buffer with UART2:
BufferedSerial buff_serial(huart2);

class STM32Hardware {
 public:
  STM32Hardware() : serial(&buff_serial) {}

  // Any initialization code necessary to use the serial port:
  void init() { serial->init(); }

  // Read a byte from the serial port. -1 = failure:
  int read() { return serial->read(); }

  // Write data to the connection to ROS:
  void write(uint8_t* data, int length) { serial->write(data, length); }

  // Returns milliseconds since start of program:
  unsigned long time() { return HAL_GetTick(); };

 protected:
  BufferedSerial* serial;
};

// DMA callbacks:
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  // Comparing pointers: (remove equality if only one UART is used)
  if (huart->Instance == buff_serial.get_handle()->Instance) {
    buff_serial.flush_tx_buffer();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  buff_serial.reset_rx_buffer();  // Can be commented if DMA mode for RX is Circular
}

#endif

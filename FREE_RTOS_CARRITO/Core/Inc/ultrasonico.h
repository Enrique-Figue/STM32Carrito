#ifndef INC_ULTRASONICO_H_
#define INC_ULTRASONICO_H_

#include "stm32f1xx_hal.h" // Si tienes otra familia de micro, reemplaza el f1 por f3, f4, etc.

// Definir los pines utilizados para el sensor ultrasónico
#define TRIG_PIN GPIO_PIN_6 // PA6
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_5 // PA5
#define ECHO_PORT GPIOA

// Prototipos de funciones
void ultrasonic_init(void);
uint16_t ultrasonic_measure_distance(void);

#endif /* INC_ULTRASONICO_H_ */

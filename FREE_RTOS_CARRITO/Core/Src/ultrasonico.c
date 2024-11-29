#include "ultrasonico.h"

#define timmer htim3 // Reemplaza 'htim3' por el timer que estés utilizando

extern TIM_HandleTypeDef timmer;

void ultrasonic_init(void) {
    HAL_TIM_Base_Start(&timmer);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // Poner el trigger en bajo
}

uint16_t ultrasonic_measure_distance(void) {
    uint32_t pMillis = HAL_GetTick();
    uint32_t Value1 = 0;
    uint32_t Value2 = 0;

    uint16_t Distance  = 0;  // cm

    // Generar pulso de trigger
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // Poner el trigger en alto
    __HAL_TIM_SET_COUNTER(&timmer, 0);
    while (__HAL_TIM_GET_COUNTER(&timmer) < 10);  // Esperar 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // Poner el trigger en bajo nuevamente.

    pMillis = HAL_GetTick();
    // Esperar que el echo reciba el pulso
    while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && (HAL_GetTick() - pMillis) < 10);
    Value1 = __HAL_TIM_GET_COUNTER(&timmer);

    pMillis = HAL_GetTick();
    // Esperar que el pin echo esté en bajo
    while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && (HAL_GetTick() - pMillis) < 50);
    Value2 = __HAL_TIM_GET_COUNTER(&timmer);

    Distance = (Value2 - Value1) * 0.034 / 2; // Calcular distancia en cm
    return Distance;
}

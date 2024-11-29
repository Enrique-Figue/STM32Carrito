/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * Este software está licenciado por ST bajo la licencia Ultimate Liberty
  * SLA0044, la "Licencia"; No puedes usar este archivo excepto en cumplimiento
  * con la Licencia. Puedes obtener una copia de la Licencia en:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "MPU9250-DMP.h"
#include "math.h"
#include "ultrasonico.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* No se realizaron cambios aquí */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ChannelA_Pin GPIO_PIN_3
#define ChannelA_GPIO_Port GPIOA
#define ChannelB_Pin GPIO_PIN_4
#define ChannelB_GPIO_Port GPIOA

// Definiciones de constantes
#define PWM_MIN               1000    // PWM para 0 grados
#define PWM_MAX               2000    // PWM para 180 grados
#define PWM_UPDATE_PERIOD_MS  10      // Periodo de actualización del PWM en ms
#define DISTANCE_THRESHOLD    30      // Umbral de distancia en cm
#define PULSES_PER_REV        56      // Ajusta este valor según tu encoder

#define VELOCIDAD_MAX         65535   // Velocidad máxima (PWM)
#define VELOCIDAD_MIN         20000   // Velocidad mínima para arrancar el motor (PWM)
#define SPEED_SCALE           1000    // Escala de velocidad (0 a 1000)
#define PWM_STEP              100     // Paso de ajuste del PWM para cambios graduales

#define ANGLE_CENTER          80      // �?ngulo central donde el carrito está recto
#define MAX_DELTA_ANGLE       80      // Máximo desplazamiento angular desde el centro (±80 grados)
#define PWM_STEP_SERVO        50       // Paso de ajuste de PWM para el servo

// Buffer circular para UART
#define UART_BUFFER_SIZE      64

// Constantes para el PID de dirección
#define INTEGRAL_MAX_DIR      1.0f
#define INTEGRAL_MIN_DIR     -1.0f

#define SIDE_DISTANCE_CM       1200      // Distancia de cada lado del cuadrado en cm (ajusta según necesidad)
#define TURN_ANGLE_DEGREES     90		// �?ngulo de giro para cada esquina

#define WHEEL_CIRCUMFERENCE_CM 6.0f
#define PULSES_PER_CM 		   2.8f
#define DISTANCE_PER_PULSE     2.8f    // cm por pulso del encoder (ajusta según tu hardware)
#define SIDE_ENCODER_COUNT     (int32_t)(SIDE_DISTANCE_CM / DISTANCE_PER_PULSE)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* No se realizaron cambios aquí */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId UART_TaskHandle;
osThreadId Servo_TaskHandle;
osThreadId Motor_TaskHandle;
osThreadId Ultrasonic_TaskHandle;
osThreadId Encoder_TaskHandle;
osThreadId MPU_TaskHandle;
osThreadId Bluetooth_TaskHandle;
/* USER CODE BEGIN PV */
// Variables globales
const char mensaje_constante[] = "RPM: ";

volatile int pwm_val = PWM_MIN + (140 * (PWM_MAX - PWM_MIN) / 180);
volatile int Grados = 0;

int32_t var_X = 0;

uint8_t rxData;

volatile int velocidad = 0;       // Valor PWM actual
volatile int speed_scale = 0;

uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_rx_head = 0;
volatile uint8_t uart_rx_tail = 0;

volatile int32_t posicion = 0;

int32_t current_position = 0;
int32_t rpm_scaled = 0;

// Constantes del PID
float kp = 15.3546;
float ki = 15.2377;
float kd = -0.03721;
float cv = 20000.0;
float cv_n1 = 20000.0;
uint32_t miDeltaT = 10;
float error, error_n1;
uint16_t valor_deseado; // Valor del proceso (RPM)
float sumatoria_error = 0;
float rate_error = 0;
char uart_buf[50];
int uart_buf_len;

// Variables para el PID de dirección
volatile float desired_heading = 0; // Inicializado al ángulo central

float kp_dir = 2.0f;       // Ganancia proporcional
float ki_dir = 0.5f;       // Ganancia integral
float kd_dir = 1.0f;       // Ganancia derivativa

float error_dir = 0.0f;
float prev_error_dir = 0.0f;
float integral_dir = 0.0f;
float derivative_dir = 0.0f;
float output_dir = 0.0f;

// COMPASS VARIABLES
float heading_degree = 0.0f;

int i2cVar = 0;

uint32_t previousTick = 0;

volatile uint8_t uart_data_ready = 0;

float error_correction = 0.0f;

float angulo_deseado = 0;

float pos_x = 0.0f;
float pos_y = 0.0f;
volatile float target_x = 0.0f;
volatile float target_y = 0.0f;
volatile int  velocidad_actual = 0;

volatile int  velocidad_manual = 0;
volatile int  grados_manual = 0;



char square_msg[100];


typedef enum {
    MODE_AUTONOMOUS,
    MODE_MANUAL
} OperationMode;

volatile OperationMode current_mode = MODE_AUTONOMOUS;

// Variables para comandos
volatile uint8_t command_received = 0;
char command_buffer[UART_BUFFER_SIZE];
volatile uint8_t command_index = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartUartTask(void const * argument);
void StartServoTask(void const * argument);
void StartMotorTask(void const * argument);
void StartUltrasonicoTask(void const * argument);
void StartEncoderTask(void const * argument);
void StartMPU9250Task(void const * argument);
void StartBluetoothTask(void const * argument);

/* USER CODE BEGIN PFP */
// Funciones de usuario
void update_servo_position(void);
void set_motor_direction(int direction);
void adjust_speed(int step);
void process_uart_command(uint8_t command);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void I2C_Scan(I2C_HandleTypeDef *hi2c);
void intToStr(int32_t num, char *str);
int32_t float_to_scaled_int(float number, uint8_t decimals);
void float_to_int_frac(float number, int32_t *int_part, int32_t *frac_part, uint8_t decimals);
void process_received_command(void);
void send_data_over_bluetooth(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback de recepción UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        static uint8_t receiving_command = 0;
        static uint8_t command_started = 0;

        if (rxData == 0x02) // STX
        {
            command_index = 0;
            command_started = 1;
        }
        else if (rxData == 0x03 && command_started) // ETX
        {
            command_buffer[command_index] = '\0'; // Terminar la cadena
            command_received = 1;
            command_started = 0;

            // Opcional: Echo del comando recibido para depuración
            HAL_UART_Transmit(&huart1, (uint8_t*)"Comando Recibido: ", 19, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, (uint8_t*)command_buffer, command_index, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
        }
        else if (command_started)
        {
            if (command_index < UART_BUFFER_SIZE - 1)
            {
                command_buffer[command_index++] = rxData;
            }
            else
            {
                // Buffer overflow, resetear
                command_started = 0;
                command_index = 0;
                HAL_UART_Transmit(&huart1, (uint8_t*)"Error: Buffer Overflow\r\n", 23, HAL_MAX_DELAY);
            }
        }

        // Reiniciar la recepción
        HAL_UART_Receive_IT(&huart1, &rxData, 1);
    }
}


float calculate_angle_error(float desired, float current) {
    float error = desired - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void move_forward(int32_t encoder_target) {
    // Reiniciar el conteo del encoder
    posicion = 0;

    // Configurar la dirección del motor hacia adelante
    set_motor_direction(2); // Asumiendo que '2' es adelante

    // Establecer una velocidad de avance
    set_speed(800); // Ajusta la velocidad según sea necesario

    // Esperar hasta que se alcance el conteo del encoder
    while (posicion < encoder_target) {
        osDelay(10); // Verificar cada 10 ms
    }

    // Detener el motor
    set_motor_direction(0);
    set_speed(0);

    // Opcional: Enviar mensaje de depuración
    sprintf(square_msg, "Avanzó %ld pulsos\r\n", encoder_target);
    HAL_UART_Transmit(&huart1, (uint8_t*)square_msg, strlen(square_msg), HAL_MAX_DELAY);
}

// Función para girar un ángulo específico utilizando el PID de dirección
// Función para girar un ángulo específico utilizando el PID de dirección
void turn_degrees(float angle)
{
    // Guardar el heading inicial
    float initial_heading = heading_degree;

    // Calcular el heading deseado
    float desired_heading_turn = angulo_deseado + angle;
    angulo_deseado = desired_heading_turn;

    // Normalizar el heading deseado entre 0 y 360 grados
    while (desired_heading_turn >= 360.0f) desired_heading_turn -= 360.0f;
    while (desired_heading_turn < 0.0f) desired_heading_turn += 360.0f;

    // Establecer el heading deseado
    desired_heading = desired_heading_turn;



    // Detener el motor después del giro
    set_motor_direction(0);
    set_speed(0);

    // Opcional: Enviar mensaje de depuración
}




// Función para escanear el bus I2C y detectar dispositivos
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    char msg[50];
    HAL_UART_Transmit(&huart1, (uint8_t*)"Scanning I2C bus...\r\n", 20, HAL_MAX_DELAY);
    for(uint16_t i=0; i<128; i++)
    {
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i<<1), 1, 10);
        if(result == HAL_OK)
        {
            i2cVar = i;
            sprintf(msg, "Device found at 0x%X\r\n", i);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"I2C scan completed\r\n", 21, HAL_MAX_DELAY);
}

// Configurar dirección del motor
void set_motor_direction(int direction)
{
    switch (direction)
    {
    case 1:  // Motor A encendido
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    case 2:  // Motor C encendido
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        break;
    case 0:  // Apagar ambos motores
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}

// Ajustar la velocidad del motor
void adjust_speed(int step)
{
    // 1. Actualizar la escala de velocidad
    speed_scale += step;

    // 2. Limitar la escala de velocidad dentro de 0 a 1000
    if (speed_scale > SPEED_SCALE)
        speed_scale = SPEED_SCALE;
    else if (speed_scale < 0)
        speed_scale = 0;

    // 3. Mapear la escala de velocidad a VELOCIDAD_MIN a VELOCIDAD_MAX
    if (speed_scale == 0)
    {
        // Motor apagado
        velocidad = 0;
    }
    else
    {
        // Calcula el PWM correspondiente a la escala de velocidad
        // Usamos 'long' para evitar desbordamientos en la multiplicación
        velocidad = VELOCIDAD_MIN + ((long)speed_scale * (VELOCIDAD_MAX - VELOCIDAD_MIN)) / SPEED_SCALE;

        // Asegurarse de que 'velocidad' no exceda los límites
        if (velocidad > VELOCIDAD_MAX)
            velocidad = VELOCIDAD_MAX;
        else if (velocidad < VELOCIDAD_MIN)
            velocidad = VELOCIDAD_MIN;
    }

    // 4. Ajustar 'velocidad' hacia 'velocidad_target' en pasos definidos por PWM_STEP
    // Esto proporciona un cambio suave en la velocidad
    static int velocidad_target = 0;

    // Actualizar el objetivo de velocidad
    velocidad_target = velocidad;

    // Ajuste gradual hacia 'velocidad_target'
    if (velocidad < velocidad_target)
    {
        velocidad += PWM_STEP;
        if (velocidad > velocidad_target)
            velocidad = velocidad_target;
    }
    else if (velocidad > velocidad_target)
    {
        velocidad -= PWM_STEP;
        if (velocidad < velocidad_target)
            velocidad = velocidad_target;
    }

    // 5. Actualizar el PWM del motor
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, velocidad);
}

// Establecer velocidad directamente
// Actualizar la velocidad en la función set_speed
void set_speed(int desired_speed)
{
    if (desired_speed > SPEED_SCALE)
        desired_speed = SPEED_SCALE;
    else if (desired_speed < 0)
        desired_speed = 0;

    speed_scale = desired_speed;

    if (speed_scale == 0)
    {
        velocidad = 0;
    }
    else
    {
        velocidad = VELOCIDAD_MIN + ((long)speed_scale * (VELOCIDAD_MAX - VELOCIDAD_MIN)) / SPEED_SCALE;

        if (velocidad > VELOCIDAD_MAX)
            velocidad = VELOCIDAD_MAX;
        else if (velocidad < VELOCIDAD_MIN)
            velocidad = VELOCIDAD_MIN;
    }

    // Actualizar la velocidad actual
    velocidad_actual = velocidad;

    // Actualizar el PWM del motor
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, velocidad);
}

// Procesar comandos UART
void process_uart_command(uint8_t command)
{
    switch (command)
    {
    case 'A':
        var_X = 0;
        set_motor_direction(2);
        set_speed(800);
        break;
    case 'B':
        adjust_speed(100);
        break;
    case 'C':
        var_X = 0;
        set_motor_direction(2);
        set_speed(650);
        break;
    case 'D':
        adjust_speed(-100);
        break;
    case 'E':
        set_motor_direction(0);
        break;
    case 'F':
        Grados = 60;
        break;
    case 'G':
        Grados = 40;
        break;
    default:
        break;
    }
}

// Actualizar posición del servo
void update_servo_position(void)
{
    // 1. Limitar el desplazamiento angular al rango permitido (-80° a +80°)
    if (Grados > MAX_DELTA_ANGLE)
        Grados = MAX_DELTA_ANGLE;
    else if (Grados < -MAX_DELTA_ANGLE)
        Grados = -MAX_DELTA_ANGLE;

    // 2. Calcular el ángulo real del servo sumando el desplazamiento al ángulo central
    int servo_angle = ANGLE_CENTER + Grados;

    // 3. Calcular el valor PWM objetivo usando la fórmula ajustada
    // La fórmula mapea el ángulo (0° a 180°) al rango PWM_MIN a PWM_MAX
    int pwm_target = PWM_MIN + ((servo_angle) * (PWM_MAX - PWM_MIN)) / 180;

    // 4. Asegurarse de que pwm_target esté dentro de los límites PWM_MIN y PWM_MAX
    if (pwm_target > PWM_MAX)
        pwm_target = PWM_MAX;
    else if (pwm_target < PWM_MIN)
        pwm_target = PWM_MIN;

    // 5. Ajustar pwm_val hacia pwm_target en pasos definidos por PWM_STEP_SERVO para un movimiento suave
    if (pwm_val < pwm_target)
    {
        pwm_val += PWM_STEP_SERVO;
        if (pwm_val > pwm_target)
            pwm_val = pwm_target;
    }
    else if (pwm_val > pwm_target)
    {
        pwm_val -= PWM_STEP_SERVO;
        if (pwm_val < pwm_target)
            pwm_val = pwm_target;
    }

    // 6. Actualizar PWM del servo
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);
}

// Función simple para convertir entero a cadena
void intToStr(int32_t num, char *str) {
    int i = 0;
    int isNegative = 0;

    // Manejar 0 explícitamente
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    // Manejar números negativos
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // Procesar dígitos
    while (num != 0 && i < 9) { // Limitar a 9 dígitos para evitar overflow
        int rem = num % 10;
        str[i++] = rem + '0';
        num = num / 10;
    }

    // Añadir signo negativo si es necesario
    if (isNegative) {
        str[i++] = '-';
    }

    str[i] = '\0';

    // Invertir la cadena
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

// Función auxiliar para convertir float a enteros escalados
int32_t float_to_scaled_int(float number, uint8_t decimals) {
    float scaling_factor = 1.0f;
    for(uint8_t i = 0; i < decimals; i++) {
        scaling_factor *= 10.0f;
    }
    return (int32_t)(number * scaling_factor);
}

// Función auxiliar para dividir float en parte entera y fraccionaria
void float_to_int_frac(float number, int32_t *int_part, int32_t *frac_part, uint8_t decimals) {
    int32_t scaled = float_to_scaled_int(number, decimals);
    int32_t divisor = 1;
    for(uint8_t i = 0; i < decimals; i++) {
        divisor *= 10;
    }
    *int_part = scaled / divisor;
    *frac_part = scaled % divisor;
    if (*frac_part < 0) {
        *frac_part = -*frac_part;
    }
}

// Callback de interrupción GPIO
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ChannelA_Pin)
    {
        if (HAL_GPIO_ReadPin(ChannelB_GPIO_Port, ChannelB_Pin))
            posicion--;
        else
            posicion++;
    }

    // Calcular la distancia recorrida por este pulso
   float distance_per_pulse = 1.0f / PULSES_PER_CM; // cm por pulso

   // Obtener el ángulo actual en radianes
   float angle_rad = heading_degree * (3.14159265359f / 180.0f);

   // Actualizar las posiciones X e Y
   pos_x += distance_per_pulse * cosf(angle_rad);
   pos_y += distance_per_pulse * sinf(angle_rad);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // No se realizaron cambios aquí
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* No se realizaron cambios aquí */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  // Inicializar PWM para el servo
  pwm_val = PWM_MIN + (90 * (PWM_MAX - PWM_MIN) / 180);

  // Inicializar sensores y módulos
  I2C_Scan(&hi2c2);
  MPU9250_DMP();

  // Inicializar MPU9250 con reintentos
  int result = INV_ERROR; // Inicializamos con un valor de error
  int attempts = 0;
  const int max_attempts = 5; // Máximo de 5 intentos

  while (result != INV_SUCCESS && attempts < max_attempts) {
      result = MPU9250_begin();

      if (result != INV_SUCCESS) {
          char error_msg[64];
          snprintf(error_msg, sizeof(error_msg), "MPU9250 attempt %d failed\r\n", attempts + 1);
          HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
          HAL_Delay(200); // Retraso antes de reintentar
          attempts++;
      }
  }

  if (result != INV_SUCCESS) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"MPU9250 failed after max attempts\r\n", 35, HAL_MAX_DELAY);
      Error_Handler(); // Manejar el error (reiniciar o notificar)
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"MPU9250 initialized successfully\r\n", 35, HAL_MAX_DELAY);
  }

  result = MPU9250_setSensors(INV_XYZ_COMPASS | INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (result != INV_SUCCESS) {
      // Manejar error
      HAL_UART_Transmit(&huart1, (uint8_t*)"MPU9250_setSensors failed\r\n", 27, HAL_MAX_DELAY);
      Error_Handler();
  }

  result = MPU9250_dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL, 10);
  if (result != INV_SUCCESS) {
      // Manejar error
      HAL_UART_Transmit(&huart1, (uint8_t*)"MPU9250_dmpBegin failed\r\n", 25, HAL_MAX_DELAY);
      Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart1, &rxData, 1) != HAL_OK)
  {
      HAL_UART_Transmit(&huart1, (uint8_t*)"HAL UART IT failed\r\n", 25, HAL_MAX_DELAY);
      Error_Handler();
  }


  // Inicializar sensor ultrasónico
  ultrasonic_init();

  // Habilitar interrupciones UART
  HAL_UART_Receive_IT(&huart1, &rxData, 1);

  // Iniciar PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Servo

  // Establecer el valor inicial del PWM del servo
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);

  HAL_Delay(25000); // Esperar 20 segundos antes de iniciar las tareas

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* No se añadieron mutexes */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* No se añadieron semáforos */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* No se añadieron timers */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* No se añadieron colas */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, StartUartTask, osPriorityNormal, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of Servo_Task */
  osThreadDef(Servo_Task, StartServoTask, osPriorityHigh, 0, 128);
  Servo_TaskHandle = osThreadCreate(osThread(Servo_Task), NULL);

  /* definition and creation of Motor_Task */
  osThreadDef(Motor_Task, StartMotorTask, osPriorityNormal, 0, 128);
  Motor_TaskHandle = osThreadCreate(osThread(Motor_Task), NULL);

  /* definition and creation of Ultrasonic_Task */
  osThreadDef(Ultrasonic_Task, StartUltrasonicoTask, osPriorityIdle, 0, 64);
  Ultrasonic_TaskHandle = osThreadCreate(osThread(Ultrasonic_Task), NULL);

  /* definition and creation of Encoder_Task */
  osThreadDef(Encoder_Task, StartEncoderTask, osPriorityLow, 0, 256);
  Encoder_TaskHandle = osThreadCreate(osThread(Encoder_Task), NULL);

  /* definition and creation of MPU_Task */
  osThreadDef(MPU_Task, StartMPU9250Task, osPriorityHigh, 0, 256);
  MPU_TaskHandle = osThreadCreate(osThread(MPU_Task), NULL);

  /* definition and creation of Bluetooth_Task */
  osThreadDef(Bluetooth_Task, StartBluetoothTask, osPriorityNormal, 0, 256);
  Bluetooth_TaskHandle = osThreadCreate(osThread(Bluetooth_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* No se añadieron más threads */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */


  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0); // Prioridad 5 (ajústala si es necesario)
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Triger_GPIO_Port, Triger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INI2_Pin|INI1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Enc_A_Pin */
  GPIO_InitStruct.Pin = Enc_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc_B_Pin */
  GPIO_InitStruct.Pin = Enc_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Enc_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Triger_Pin */
  GPIO_InitStruct.Pin = Triger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Triger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INI2_Pin INI1_Pin */
  GPIO_InitStruct.Pin = INI2_Pin|INI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

// Función para extraer dos enteros de un buffer
int parse_command(const char *buffer, int *angulo, int *velocidad) {
    char *comma_pos = strchr(buffer, ','); // Buscar la coma
    if (comma_pos == NULL) {
        return 0; // Formato inválido
    }

    // Extraer los valores antes y después de la coma
    *comma_pos = '\0'; // Reemplazar coma por terminador nulo temporalmente
    *angulo = atoi(buffer); // Convertir la primera parte a entero
    *comma_pos = ',';       // Restaurar la coma
    *velocidad = atoi(comma_pos + 1); // Convertir la segunda parte a entero

    return 1; // Éxito
}


void process_received_command(void)
{
    if (command_received)
    {
        command_received = 0;

        // Verificar si el comando comienza con "MODE:"
        if (strncmp(command_buffer, "MODE:", 5) == 0)
        {
            char mode[7]; // "MANUAL" o "AUTO"
            strncpy(mode, command_buffer + 5, 6);
            mode[6] = '\0'; // Asegurar terminación de cadena

            if (strcmp(mode, "MANUAL") == 0)
            {
                current_mode = MODE_MANUAL;
                set_motor_direction(0); // Detener motores al cambiar a manual
                set_speed(0);
                HAL_UART_Transmit(&huart1, (uint8_t*)"Modo Manual Activado\r\n", 22, HAL_MAX_DELAY);
            }
            else if (strcmp(mode, "AUTO") == 0)
            {
                current_mode = MODE_AUTONOMOUS;
                HAL_UART_Transmit(&huart1, (uint8_t*)"Modo Autonómico Activado\r\n", 25, HAL_MAX_DELAY);
            }
            else
            {
                HAL_UART_Transmit(&huart1, (uint8_t*)"Modo Desconocido\r\n", 19, HAL_MAX_DELAY);
            }
        }
        // Verificar si el comando comienza con 'M' para Manual Control
        else if (command_buffer[0] == 'M')
        {
            if (current_mode != MODE_MANUAL)
            {
                // Ignorar comandos manuales si no estamos en modo manual
                HAL_UART_Transmit(&huart1, (uint8_t*)"No está en modo manual\r\n", 23, HAL_MAX_DELAY);
                return;
            }

            // Parsear los valores de ángulo y velocidad
            int angulo = 0;
            int velocidad_m = 0;

            // Usar sscanf para extraer los valores
            if (parse_command(command_buffer + 1, &angulo, &velocidad_m))
            {

              velocidad_manual = velocidad_m;
		      grados_manual = angulo;


		      if (velocidad_m < 0)
		      {
		    	  velocidad_m = -velocidad_m;
		          set_motor_direction(2);
		      }
		      else if (velocidad_m > 0)
			  {
				  set_motor_direction(1);
			  }
		      else
		      {
			        set_motor_direction(0);
		      }


		      set_speed(velocidad_m);
		      Grados = angulo;

                // Actualizar la velocidad y el ángulo
              //  set_speed(velocidad_manual);
              //  Grados = angulo;


            }
            else
            {
                //HAL_UART_Transmit(&huart1, (uint8_t*)"Formato de comando inválido\r\n", 28, HAL_MAX_DELAY);
            }
        }


    }
}

void send_data_over_bluetooth(void)
{
    char data_packet[100]; // Ajusta el tamaño según sea necesario
    // Escalar los valores flotantes a enteros (×100)
    int velocidad_scaled = velocidad_actual; // Asegúrate de que velocidad_actual ya está en enteros
    int pos_x_scaled = (int)(pos_x * 100);
    int pos_y_scaled = (int)(pos_y * 100);
    int yaw_scaled = (int)(heading_degree * 100);

    // Formatear el paquete con enteros
    int length = snprintf(data_packet, sizeof(data_packet), "D,%d,%d,%d,%d", velocidad_scaled, pos_x_scaled, pos_y_scaled, yaw_scaled);

    if (length > 0 && length < sizeof(data_packet))
    {
        uint8_t start_byte = 0x02;
        uint8_t end_byte = 0x03;

        // Enviar el paquete por UART
        HAL_UART_Transmit(&huart1, &start_byte, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t*)data_packet, length, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &end_byte, 1, HAL_MAX_DELAY);


    }


}




/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (command_received)
	        {
	            process_received_command();
	        }

	        osDelay(100); // Retard
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the Servo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */
	uint32_t antes = HAL_GetTick();
	uint32_t ahora = antes;
	float intervalo = 0.0f;
  /* Infinite loop */
  for(;;)
  {

	  if (current_mode != MODE_MANUAL)
	  {
	  // Implementación del PID para la dirección
	        // 1. Calcular el error

	        error_dir = calculate_angle_error(desired_heading, heading_degree);

	        // 2. Calcular la integral del error (anti-windup)

	        antes = ahora;
	        ahora = HAL_GetTick();
	        intervalo = (float)(ahora - antes) / 1000.0f;

	        if (intervalo <= 0.0f)
			{
				intervalo = 0.001f; // Asignar un valor pequeño
			}

	        integral_dir += error_dir * intervalo; // Integración en segundos


	        if (integral_dir > INTEGRAL_MAX_DIR)
	            integral_dir = INTEGRAL_MAX_DIR;
	        else if (integral_dir < INTEGRAL_MIN_DIR)
	            integral_dir = INTEGRAL_MIN_DIR;

	        // 3. Calcular la derivada del error
	        derivative_dir = (error_dir - prev_error_dir) / (PWM_UPDATE_PERIOD_MS / 1000.0f);

	        // 4. Calcular la salida del PID
	        output_dir = kp_dir * error_dir + ki_dir * integral_dir + kd_dir * derivative_dir;

	        // 5. Actualizar el error previo
	        prev_error_dir = error_dir;

	        // 6. Ajustar Grados basado en la salida del PID
	        Grados= output_dir;

	        // 7. Asegurar que Grados esté dentro de los límites
	        if (Grados > MAX_DELTA_ANGLE)
	            Grados = MAX_DELTA_ANGLE;
	        else if (Grados < -MAX_DELTA_ANGLE)
	            Grados = -MAX_DELTA_ANGLE;

	        // 8. Actualizar la posición del servo

	  }
	  update_servo_position();

	        osDelay(PWM_UPDATE_PERIOD_MS); // Actualiza cada PWM_UPDATE_PERIOD_MS ms
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
      if (current_mode == MODE_AUTONOMOUS)
      {
          // Secuencia autónoma
          move_forward(SIDE_ENCODER_COUNT - 115);
          osDelay(5000);

          // Girar 90 grados
          turn_degrees(TURN_ANGLE_DEGREES);
          osDelay(5000);

          for(int side = 0; side < 2; side++)
          {
              move_forward(SIDE_ENCODER_COUNT-10);
              osDelay(5000);

              turn_degrees(TURN_ANGLE_DEGREES);
              osDelay(5000);
          }

          move_forward(SIDE_ENCODER_COUNT + 15);
          osDelay(5000);

          turn_degrees(TURN_ANGLE_DEGREES);
          osDelay(5000);

          // Detener el motor después de completar la secuencia
          set_motor_direction(0);
          set_speed(0);

          // Esperar antes de repetir la secuencia
          osDelay(1000);
      }
      else if (current_mode == MODE_MANUAL)
      {
          // En modo manual, permitir que las tareas de comandos controlen el motor
          // No realizar ninguna acción específica aquí
          osDelay(100); // Ajusta el delay según sea necesario
      }

      osDelay(100); // Ajusta el delay según sea necesario
  }
  /* USER CODE END StartMotorTask */
}


/* USER CODE BEGIN Header_StartUltrasonicoTask */
/**
* @brief Function implementing the Ultrasonic_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicoTask */
void StartUltrasonicoTask(void const * argument)
{
  /* USER CODE BEGIN StartUltrasonicoTask */
  /* Infinite loop */
  for(;;)
  {
      if(ultrasonic_measure_distance() < DISTANCE_THRESHOLD)
          set_motor_direction(0);
      osDelay(400);
  }
  /* USER CODE END StartUltrasonicoTask */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the Encoder_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void const * argument)
{
  /* USER CODE BEGIN StartEncoderTask */

  const uint32_t t_in_ms = 500;  // Intervalo de medición en milisegundos

  char msg[20]; // Aumenta el tamaño si es necesario


  /* Infinite loop */
  for(;;)
  {

	  int pos_x_scaled = (int)(pos_x  * (180.0f / 3.141592f) * 100); // Escalar por 100
	  int pos_y_scaled = (int)(pos_y  * (180.0f / 3.141592f) * 100); // Escalar por 100

	  //sprintf(msg, "x%d y%02d\r\n", pos_x_scaled / 100, pos_y_scaled/100);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      // Capturar la posición actual
//      posicion = 0;
//
//      osDelay(t_in_ms);
//
//      current_position = posicion;
//
//      // Calcular RPM escaladas
//      rpm_scaled = (current_position * 60000) / (PULSES_PER_REV * t_in_ms);

      // Puedes implementar aquí la lógica adicional si es necesario
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartMPU9250Task */
/**
* @brief Function implementing the MPU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPU9250Task */
void StartMPU9250Task(void const * argument)
{
  /* USER CODE BEGIN StartMPU9250Task */
    /* Infinite loop */
	float gyro_heading = 0.0f;
	    float mag_heading = 0.0f;
	    char msg[10]; // Aumenta el tamaño si es necesario
	    float current_heading = 0;

	    static uint8_t is_calibrated = 0;     // Bandera de calibración

	    // Función para normalizar ángulos a 0-360 grados
	    float normalize_angle(float angle)
	    {
	        while (angle < 0.0f)
	            angle += 360.0f;
	        while (angle >= 360.0f)
	            angle -= 360.0f;
	        return angle;
	    }

	    for(;;)
	    {
	    	if (current_mode == MODE_AUTONOMOUS)
	    	{
	        // COMPASS SENSOR
	        if (MPU9250_dataReady())
	        {
	            MPU9250_update(UPDATE_COMPASS | UPDATE_ACCEL | UPDATE_GYRO);

	            if (MPU9250_updateCompass() == INV_SUCCESS)
	            {
	                mag_heading = MPU9250_computeCompassHeading();
	            }
	        }

	        if (MPU9250_fifoAvailable())
	        {
	            // Usar dmpUpdateFifo para actualizar los valores ax, gx, mx, etc.
	            if (MPU9250_dmpUpdateFifo() == INV_SUCCESS)
	            {


	            	if (!is_calibrated && current_heading != 0 )
					{
						error_correction = current_heading; // Almacenar el ángulo inicial como 0
						is_calibrated = 1;
					}
	                // computeEulerAngles puede ser usado para estimar roll, pitch y yaw
	                MPU9250_computeEulerAngles(false);
	                gyro_heading = yaw_inside;

	                // Calcular el heading en grados
	                current_heading = gyro_heading * (180.0f / 3.141592f); // Convertir a grados

	                // Ajustar el heading restando el ángulo de calibración
					heading_degree = current_heading - error_correction;

					// Normalizar heading_degree a 0-360 grados
					heading_degree = normalize_angle(heading_degree);

					//int degrees_scaled = (int)(gyro_heading * (180.0f / 3.141592f) * 100); // Escalar por 100
					//sprintf(msg, "rec%d.%02d\r\n", degrees_scaled / 100, degrees_scaled % 100);
					//HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

					 // Realizar la calibración si no se ha hecho aún


	            }
	        }


	    	}

	        osDelay(10); // Ajusta según la frecuencia deseada (por ejemplo, 10 ms)
	    }
  /* USER CODE END StartMPU9250Task */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the Bluetooth_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void const * argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  /* Infinite loop */
  for(;;)
  {
	  if (current_mode == MODE_AUTONOMOUS)
	        {
	  send_data_over_bluetooth();
	        }

	  osDelay(100); // Enviar datos cada 100 ms
  }
  /* USER CODE END StartBluetoothTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Usuario puede agregar su propio código para manejar el error */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Usuario puede agregar su propio código para manejar el error */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

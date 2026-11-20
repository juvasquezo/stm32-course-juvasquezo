/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Pablo Vasquez Ospina
 * @brief          : Control de un LED RGB (PWM) y de un DAC simple (PWM
 *                   filtrado) con un encoder rotativo, más captura de
 *                   frecuencia, usando 5 timers. Un display de 7 segmentos de
 *                   4 dígitos muestra el duty del modo actual o la frecuencia
 *                   medida.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * DESCRIPCIÓN DEL EQUIPO (HARDWARE)
 * - Tarjeta: NUCLEO-F411RE (STMicroelectronics) con MCU STM32F411RE
 *   (Cortex-M4), reloj interno HSI de 16 MHz, programado en bare-metal con
 *   CMSIS (acceso directo a registros). La tarjeta incluye el programador
 *   ST-LINK integrado.
 * - LED de estado: conectado a PH1, genera un blinky que indica que el MCU
 *   está funcionando.
 * - LED RGB: cada color se maneja con un canal PWM de TIM3 y lleva una
 *   resistencia en serie (___ Ohm) para limitar la corriente.
 * - Encoder rotativo: señales CLK (PA1) y DT (PA0) leídas por TIM5 en modo
 *   encoder, y botón SW (PA4) leído con la interrupción EXTI4.
 * - DAC simple: el cuarto canal PWM de TIM3 (PC9) pasa por un filtro
 *   pasa-bajos y un buffer de voltaje, generando una salida analógica de
 *   0 V a 3.3 V.
 * - Entrada de captura: PB10 (TIM2 CH3), con pull-down interno, para medir
 *   la frecuencia de una señal externa.
 * - Display de 7 segmentos de 4 dígitos multiplexado: los 7 segmentos (a-g)
 *   son comunes a los 4 dígitos; cada dígito se habilita con su propio pin
 *   (PIN 1-4). Cada segmento lleva una resistencia de 330 Ohm en serie.
 *   Cada dígito se maneja con un transistor PNP 2N3906 (base con una
 *   resistencia de 2 kOhm), porque un solo pin del MCU no soporta la corriente
 *   de un dígito completo. Lógica activa en bajo: un 0 enciende el segmento
 *   y un 0 en la base del transistor enciende el dígito.
 * - Botón de reset: pulsador conectado al pin NRST, disponible para ser
 *   presionado en cualquier momento.
 *
 * DISTRIBUCIÓN DE LOS PINES
 *   Dígitos (habilitación)      Segmentos
 *   PIN 1  ----- PC5            PIN a ----- PB12
 *   PIN 2  ----- PC13           PIN b ----- PA12
 *   PIN 3  ----- PC6            PIN c ----- PC11
 *   PIN 4  ----- PB7            PIN d ----- PC12
 *                               PIN e ----- PC10
 *   TIMER 3 (PWM)               PIN f ----- PA11
 *   PIN ROJO(CH3) --- PC8       PIN g ----- PD2
 *   PIN VERDE(CH1) -- PA6
 *   PIN AZUL(CH2) --- PA7       TIMER 2 (captura)
 *   PIN DAC(CH4) ---- PC9       PIN Cap(CH3) --- PB10
 *
 *   TIMER 5 (encoder)           GPIO GENERALES
 *   PIN CLK(CH2) ---- PA1       PIN BLINKY ----- PH1
 *   PIN DT(CH1) ----- PA0       PIN SW --------- PA4
 *
 * FUNCIONAMIENTO DEL SISTEMA
 * - TIM9 (interrupción cada 500 ms): levanta blink_flag y la FSM invierte
 *   PH1, generando un blinky de 1 Hz.
 * - TIM3 (PWM a 10 kHz, 100 pasos de duty): CH1 verde, CH2 azul, CH3 rojo y
 *   CH4 para el DAC. El voltaje del DAC es 3.3 V * (CCR4 / 100).
 * - TIM5 (encoder, 32 bits): cuenta 4 por cada detent del encoder. La FSM
 *   lee la diferencia de cuentas y la convierte en pasos de duty (CW suma,
 *   CCW resta).
 * - TIM2 (captura, 1 us por cuenta): captura el flanco de subida de PB10. La
 *   ISR guarda el valor capturado y cuenta los flancos. La FSM mide en
 *   ventanas de 100 ms: frecuencia = flancos de la ventana / tiempo de la
 *   ventana. Si pasa 1 s sin flancos se considera que no hay señal.
 * - TIM4 (debounce del SW, 30 ms): EXTI4 detecta el flanco de bajada de PA4,
 *   su ISR deshabilita la línea EXTI4 y dispara TIM4 en modo one-pulse. Al
 *   cumplirse los 30 ms, si PA4 sigue en bajo se acepta la pulsación y se
 *   vuelve a habilitar EXTI4. Así se ignoran los rebotes.
 * - TIM1 (interrupción cada 3 ms): levanta cambio_digito_flag para que la
 *   FSM refresque el siguiente dígito. Los 4 dígitos se barren en 12 ms
 *   (~83 Hz), por lo que el ojo los percibe encendidos a la vez (efecto POV).
 *   En cada cambio se apaga todo antes de escribir el siguiente dígito para
 *   evitar el "fantasma".
 * - Total: 5 interrupciones (TIM9, TIM2, EXTI4, TIM4, TIM1).
 *
 * DISPLAY
 * - Si hay señal en la entrada de captura, muestra la frecuencia dividida
 *   entre 10 (ej: 1000 = 10 kHz). El máximo es 9999 (99.99 kHz).
 * - Si no hay señal, muestra el duty en porcentaje (0 a 100) del modo actual.
 *   En MODE_ALL muestra el duty del rojo.
 * - Los ceros a la izquierda se apagan.
 *
 * MODOS (cada pulsación válida de SW pasa al siguiente)
 *   MODE_ROJO  : el encoder modifica solo el duty del rojo
 *   MODE_VERDE : el encoder modifica solo el duty del verde
 *   MODE_AZUL  : el encoder modifica solo el duty del azul
 *   MODE_ALL   : el encoder modifica los tres colores a la vez
 *   MODE_DAC   : el encoder modifica el duty del canal del DAC
 *
 * MÁQUINA DE ESTADOS (main)
 *   IDLE            : espera un evento sin bloquear (get_next_event)
 *   NEXT_MODE       : cambia al siguiente modo (EV_SW)
 *   UPDATE_PWM      : aplica el giro del encoder al duty del modo actual
 *                     (EV_ENCODER)
 *   UPDATE_FREQ     : calcula la frecuencia con las capturas (EV_CAPTURE)
 *   NO_SIGNAL       : la señal desapareció, vuelve a mostrar el duty
 *                     (EV_NO_SIGNAL)
 *   BLINK           : invierte el LED de estado (EV_BLINK)
 *   REFRESH_DISPLAY : refresca un dígito del display (EV_DISPLAY)
 *   ERROR           : estado inválido, se recupera hacia IDLE
 *
 *   Todos los estados vuelven a IDLE al terminar su acción.
 ******************************************************************************
 */

// Librerías necesarias para los tipos enteros y los registros del microcontrolador
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f411xe.h"

// Prototipos de las funciones del programa
void gpio_init(void);
void tim9_init(void);
void tim3_init(void);
void tim5_init(void);
void tim2_init(void);
void tim4_init(void);
void tim1_init(void);
void exti_init(void);
void cambiar_modo(void);
void cambiar_pwm(void);
void actualizar_digitos(void);
void apagar(void);
void escribir(void);
void cambiar_digito(void);

static int32_t clamp_pwm(int32_t val, int32_t max_val);

#define SW_DEBOUNCE_MS     30U         // tiempo de debounce del SW
#define FREQ_WINDOW_US     100000UL    // ventana de medida de frecuencia (100 ms)
#define FREQ_TIMEOUT_US    1000000UL   // sin flancos durante 1 s = no hay señal
#define FREQ_DIV           10U         // la frecuencia se muestra dividida entre 10
#define DIGITO_APAGADO     10          // índice de tabla_7seg con todos los segmentos apagados

typedef enum {
    STATE_IDLE,             // Reposo: espera un evento sin bloquear
    STATE_NEXT_MODE,
    STATE_UPDATE_PWM,
    STATE_UPDATE_FREQ,
    STATE_NO_SIGNAL,
    STATE_BLINK,
    STATE_REFRESH_DISPLAY,
    STATE_ERROR,            // Estado inválido: se recupera hacia STATE_IDLE
    NUM_STATES              // Centinela: cantidad de estados, valida el rango
} State_t;

typedef enum {
    EV_NONE,
    EV_ENCODER,
    EV_SW,
    EV_CAPTURE,
    EV_NO_SIGNAL,
    EV_BLINK,
    EV_DISPLAY
} Event_t;

typedef enum {
    MODE_ROJO,   // 0: el encoder modifica CCR3 (PC8, rojo)
    MODE_VERDE,  // 1: el encoder modifica CCR1 (PA6, verde)
    MODE_AZUL,   // 2: el encoder modifica CCR2 (PA7, azul)
    MODE_ALL,    // 3: el encoder modifica CCR1, CCR2 y CCR3 juntos
    MODE_DAC,    // 4: el encoder modifica CCR4 (PC9, salida analógica filtrada)
    NUM_MODES    // Centinela = 5, se usa en (modo + 1) % NUM_MODES
} Mode_t;

typedef struct{
    GPIO_TypeDef*puerto;
    uint8_t pin;
}PinSegmento;

// Mapa de segmentos: cada posición guarda el puerto y el pin de un segmento (A a G)
const PinSegmento mapa_segmentos[7]={
    {GPIOB, 12},  // Segmento A -> PB12
    {GPIOA, 12},  // Segmento B -> PA12
    {GPIOC, 11},  // Segmento C -> PC11
    {GPIOC, 12},  // Segmento D -> PC12
    {GPIOC, 10},  // Segmento E -> PC10
    {GPIOA, 11},  // Segmento F -> PA11
    {GPIOD, 2}    // Segmento G -> PD2
};

// Patrón de cada número: bit 0 = A ... bit 6 = G, bit 7 = DP. Un 0 enciende el segmento
const uint8_t tabla_7seg[11] = {
    0b11000000, // 0 (A,B,C,D,E,F encendidos; G y DP apagados)
    0b11111001, // 1 (B,C encendidos)
    0b10100100, // 2 (A,B,D,E,G encendidos)
    0b10110000, // 3 (A,B,C,D,G encendidos)
    0b10011001, // 4 (B,C,F,G encendidos)
    0b10010010, // 5 (A,C,D,F,G encendidos)
    0b10000010, // 6 (A,C,D,E,F,G encendidos)
    0b11111000, // 7 (A,B,C encendidos)
    0b10000000, // 8 (A,B,C,D,E,F,G encendidos)
    0b10010000, // 9 (A,B,C,D,F,G encendidos)
    0b11111111  // apagado (ningún segmento encendido)
};

// Variables globales. Las que modifican las ISR son volatile para que el
// compilador no las guarde en registros y siempre lea su valor real de memoria
int32_t arr_value = 100;                 // pasos del PWM (duty de 0 a arr_value)
int32_t ccr_verde = 0;                   // CH1 -> PA6
int32_t ccr_azul  = 0;                   // CH2 -> PA7
int32_t ccr_rojo  = 0;                   // CH3 -> PC8
int32_t ccr_dac   = 0;                   // CH4 -> PC9
int32_t mode_values[NUM_MODES] = {0};    // duty actual de cada modo

volatile uint8_t blink_flag = 0;         // TIM9 la pone en 1 cada 500 ms
volatile uint8_t sw_pressed_flag = 0;    // TIM4 la pone en 1 al validar el SW
volatile uint8_t capture_flag = 0;       // TIM2 la pone en 1 al capturar un flanco
volatile uint8_t cambio_digito_flag = 0; // TIM1 la pone en 1 cada 3 ms
volatile uint32_t captured_ticks = 0;    // valor crudo de CCR3 capturado en el flanco
volatile uint32_t capture_count = 0;     // cantidad total de flancos capturados

uint32_t last_tim5_cnt = 0;              // última lectura del contador del encoder
int32_t encoder_delta = 0;               // cuentas nuevas del encoder
int32_t encoder_accum = 0;               // cuentas acumuladas (4 cuentas = 1 detent)

uint32_t last_capture = 0;               // instante del último flanco procesado
uint32_t ref_ticks = 0;                  // instante en que empezó la ventana de medida
uint32_t ref_count = 0;                  // flancos acumulados al empezar la ventana
uint8_t first_capture = 1;               // 1 = no hay ventana de medida en curso
uint8_t freq_activa = 0;                 // 1 = hay una frecuencia válida para mostrar
uint32_t period_us = 0;                  // período promedio de la señal capturada
uint32_t frequency_hz = 0;               // frecuencia de la señal capturada

uint8_t digito_actual = 0;               // dígito encendido en este momento (0 a 3)
uint8_t digito[4] = {0,0,0,0};           // índice de tabla_7seg de cada posición: miles, centenas, decenas, unidades

uint8_t state_entry_flag = 1;            // 1 mientras se entra a un estado nuevo

Mode_t current_mode = MODE_ROJO;
State_t current_state = STATE_IDLE;
Event_t current_event = EV_NONE;

Event_t get_next_event(void);

/******************************************************************************
 * PROGRAMA PRINCIPAL
******************************************************************************/
int main(void)
{
    gpio_init();
    tim9_init();
    tim3_init();
    tim5_init();
    tim2_init();
    tim4_init();
    tim1_init();
    exti_init();

    actualizar_digitos();   // el display arranca mostrando el duty (0)

    // Si el estado inicial no es válido se pasa a ERROR
    if (current_state >= NUM_STATES) {
        current_state = STATE_ERROR;
        state_entry_flag = 1;
    }

    while(1){
        switch(current_state){
            case STATE_IDLE:
                current_event = get_next_event();

                // Según el evento se pasa al estado que lo atiende
                if (current_event == EV_SW){
                    current_state = STATE_NEXT_MODE;
                    state_entry_flag = 1;
                }else if (current_event == EV_DISPLAY) {
                    current_state = STATE_REFRESH_DISPLAY;
                    state_entry_flag = 1;
                }else if (current_event == EV_ENCODER) {
                    current_state = STATE_UPDATE_PWM;
                    state_entry_flag = 1;
                }else if (current_event == EV_CAPTURE) {
                    current_state = STATE_UPDATE_FREQ;
                    state_entry_flag = 1;
                }else if (current_event == EV_NO_SIGNAL) {
                    current_state = STATE_NO_SIGNAL;
                    state_entry_flag = 1;
                }else if (current_event == EV_BLINK) {
                    current_state = STATE_BLINK;
                    state_entry_flag = 1;
                }
                break;

            case STATE_NEXT_MODE:
                if (state_entry_flag) {
                    cambiar_modo();
                    actualizar_digitos();
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_UPDATE_PWM:
                if (state_entry_flag) {
                    cambiar_pwm();
                    actualizar_digitos();
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_UPDATE_FREQ:
                if (state_entry_flag) {
                    uint32_t ticks, count;

                    // Lectura atómica: la ISR de TIM2 no puede cambiar uno de
                    // los dos valores entre las dos lecturas
                    __disable_irq();
                    ticks = captured_ticks;
                    count = capture_count;
                    __enable_irq();

                    if (first_capture) {
                        // Se abre la ventana de medida
                        ref_ticks = ticks;
                        ref_count = count;
                        first_capture = 0;
                    }
                    else {
                        uint32_t elapsed = ticks - ref_ticks;   // tiempo de la ventana (us)
                        uint32_t pulsos = count - ref_count;    // flancos dentro de la ventana

                        if (elapsed >= FREQ_WINDOW_US) {
                            period_us = elapsed / pulsos;
                            frequency_hz = (uint32_t)(((uint64_t)pulsos * 1000000ULL) / elapsed);
                            freq_activa = 1;
                            actualizar_digitos();

                            // Empieza la siguiente ventana
                            ref_ticks = ticks;
                            ref_count = count;
                        }
                    }
                    last_capture = ticks;
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_NO_SIGNAL:
                if (state_entry_flag) {
                    first_capture = 1;
                    freq_activa = 0;
                    frequency_hz = 0;
                    actualizar_digitos();
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_BLINK:
                if (state_entry_flag) {
                    GPIOH->ODR ^= (GPIO_ODR_OD1);
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_REFRESH_DISPLAY:
                if (state_entry_flag) {
                    apagar();
                    escribir();
                    cambiar_digito();
                    digito_actual = (digito_actual + 1) % 4;
                    state_entry_flag = 0;
                }
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;

            case STATE_ERROR:
            default:
                current_state = STATE_IDLE;
                state_entry_flag = 1;
                break;
        }
    }
}

// Devuelve el siguiente evento pendiente. Orden de prioridad: SW, display,
// encoder, captura, señal perdida y blinky
Event_t get_next_event(void){

    if (sw_pressed_flag == 1){
        sw_pressed_flag = 0;
        return EV_SW;
    }

    if (cambio_digito_flag == 1){
        cambio_digito_flag = 0;
        return EV_DISPLAY;
    }

    // El encoder no usa interrupción: se compara el contador de TIM5 con la última lectura
    uint32_t now_cnt = TIM5->CNT;
    int32_t diff = (int32_t)(now_cnt - last_tim5_cnt);
    if (diff != 0) {
        last_tim5_cnt = now_cnt;
        encoder_delta = diff;
        return EV_ENCODER;
    }

    if (capture_flag == 1){
        capture_flag = 0;
        return EV_CAPTURE;
    }

    // Hay una medida en curso pero pasó más de 1 s desde el último flanco
    if (!first_capture && ((TIM2->CNT - last_capture) > FREQ_TIMEOUT_US)){
        return EV_NO_SIGNAL;
    }

    if (blink_flag == 1){
        blink_flag = 0;
        return EV_BLINK;
    }

    return EV_NONE;
}

void gpio_init(void){
    // Se encienden las señales de reloj de los puertos A, B, C, D y H
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOHEN);

    // PH1: LED de estado (blinky), salida push-pull que arranca apagada
    GPIOH->MODER  &= ~(GPIO_MODER_MODE1);
    GPIOH->MODER |= (GPIO_MODER_MODE1_0);
    GPIOH->OTYPER &= ~(GPIO_OTYPER_OT1);
    GPIOH->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED1);
    GPIOH->PUPDR &= ~(GPIO_PUPDR_PUPD1);
    GPIOH->ODR &= ~(GPIO_ODR_OD1);

    // PA6 (verde, TIM3 CH1) y PA7 (azul, TIM3 CH2): función alterna AF2
    GPIOA->MODER &= ~(GPIO_MODER_MODE6|GPIO_MODER_MODE7);
    GPIOA->MODER |= (GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6|GPIO_AFRL_AFRL7);
    GPIOA->AFR[0] |= (GPIO_AFRL_AFRL6_1|GPIO_AFRL_AFRL7_1);

    // PC8 (rojo, TIM3 CH3) y PC9 (DAC, TIM3 CH4): función alterna AF2
    GPIOC->MODER &= ~(GPIO_MODER_MODE8|GPIO_MODER_MODE9);
    GPIOC->MODER |= (GPIO_MODER_MODE8_1|GPIO_MODER_MODE9_1);
    GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH0|GPIO_AFRH_AFRH1);
    GPIOC->AFR[1] |= (GPIO_AFRH_AFRH0_1|GPIO_AFRH_AFRH1_1);

    // PA0 (DT, TIM5 CH1) y PA1 (CLK, TIM5 CH2): encoder, función alterna AF2
    GPIOA->MODER &= ~(GPIO_MODER_MODE0|GPIO_MODER_MODE1);
    GPIOA->MODER |= (GPIO_MODER_MODE0_1|GPIO_MODER_MODE1_1);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL0|GPIO_AFRL_AFRL1);
    GPIOA->AFR[0] |= (GPIO_AFRL_AFRL0_1|GPIO_AFRL_AFRL1_1);

    // PB10 (captura, TIM2 CH3): función alterna AF1 con pull-down interno
    GPIOB->MODER &= ~(GPIO_MODER_MODE10);
    GPIOB->MODER |= (GPIO_MODER_MODE10_1);
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH2);
    GPIOB->AFR[1] |= (GPIO_AFRH_AFRH2_0);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_1);

    // PA4: botón SW del encoder, entrada sin pull interno
    GPIOA->MODER &= ~(GPIO_MODER_MODE4);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4);

    // Salidas del display (segmentos y dígitos, activos en bajo).
    // Se escribe 1 en ODR antes de pasar a salida para que arranquen apagados.
    // Configuración de cada pin: MODER = 01 (salida), OTYPER = 0 (push-pull),
    // OSPEEDR = 00 (velocidad baja), PUPDR = 00 (sin pull-up ni pull-down)

    // Puerto A: PA12 (segmento B) y PA11 (segmento F)
    GPIOA->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD11);
    GPIOA->MODER  &= ~(GPIO_MODER_MODE12|GPIO_MODER_MODE11);
    GPIOA->MODER |= (GPIO_MODER_MODE12_0|GPIO_MODER_MODE11_0);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT12|GPIO_OTYPER_OT11);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12|GPIO_OSPEEDR_OSPEED11);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD12|GPIO_PUPDR_PUPD11);

    // Puerto C: PC10, PC11, PC12 (segmentos E, C, D) y PC5, PC13, PC6 (dígitos 1, 2, 3)
    GPIOC->ODR |= (GPIO_ODR_OD10|GPIO_ODR_OD12|GPIO_ODR_OD11|GPIO_ODR_OD5|GPIO_ODR_OD13|GPIO_ODR_OD6);
    GPIOC->MODER  &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE12|GPIO_MODER_MODE11|GPIO_MODER_MODE5|GPIO_MODER_MODE13|GPIO_MODER_MODE6);
    GPIOC->MODER |= (GPIO_MODER_MODE10_0|GPIO_MODER_MODE12_0|GPIO_MODER_MODE11_0|GPIO_MODER_MODE5_0|GPIO_MODER_MODE13_0|GPIO_MODER_MODE6_0);
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10|GPIO_OTYPER_OT12|GPIO_OTYPER_OT11|GPIO_OTYPER_OT5|GPIO_OTYPER_OT13|GPIO_OTYPER_OT6);
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10|GPIO_OSPEEDR_OSPEED12|GPIO_OSPEEDR_OSPEED11|GPIO_OSPEEDR_OSPEED5|GPIO_OSPEEDR_OSPEED13|GPIO_OSPEEDR_OSPEED6);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10|GPIO_PUPDR_PUPD12|GPIO_PUPDR_PUPD11|GPIO_PUPDR_PUPD5|GPIO_PUPDR_PUPD13|GPIO_PUPDR_PUPD6);

    // Puerto B: PB12 (segmento A) y PB7 (dígito 4)
    GPIOB->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD7);
    GPIOB->MODER  &= ~(GPIO_MODER_MODE12|GPIO_MODER_MODE7);
    GPIOB->MODER |= (GPIO_MODER_MODE12_0|GPIO_MODER_MODE7_0);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT12|GPIO_OTYPER_OT7);
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12|GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD12|GPIO_PUPDR_PUPD7);

    // Puerto D: PD2 (segmento G)
    GPIOD->ODR |= (GPIO_ODR_OD2);
    GPIOD->MODER  &= ~(GPIO_MODER_MODE2);
    GPIOD->MODER |= (GPIO_MODER_MODE2_0);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT2);
    GPIOD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2);
    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD2);
}

void tim9_init(void){
    RCC->APB2ENR |= (RCC_APB2ENR_TIM9EN);

    TIM9->CNT = 0;
    TIM9->PSC = (16000-1);   // 16 MHz / 16000 = 1 kHz (1 ms por cuenta)
    TIM9->ARR = (500-1);     // 500 cuentas = 500 ms entre interrupciones
    TIM9->DIER |= (TIM_DIER_UIE);
    TIM9->CR1 &= ~(TIM_CR1_CKD);
    TIM9->CR1 &= ~(TIM_CR1_ARPE);
    TIM9->SR = ~(TIM_SR_UIF);
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    TIM9->CR1 |= (TIM_CR1_CEN);
}

// ISR de TIM9: avisa a la FSM que debe invertir el LED de estado (blinky)
void TIM1_BRK_TIM9_IRQHandler(void){
    if (TIM9->SR & TIM_SR_UIF){
        blink_flag = 1;
        TIM9->SR = ~(TIM_SR_UIF);
    }
}

void tim3_init(void){
    RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);

    TIM3->PSC = (16-1);              // 16 MHz / 16 = 1 MHz (1 us por cuenta)
    TIM3->ARR = (arr_value-1);       // 100 cuentas = PWM de 10 kHz
    TIM3->CCR1 = (ccr_verde);
    TIM3->CCR2 = (ccr_azul);
    TIM3->CCR3 = (ccr_rojo);
    TIM3->CCR4 = (ccr_dac);

    TIM3->CR1 |= (TIM_CR1_ARPE);

    // Modo PWM 1 (0b110) en los 4 canales, con precarga del CCR
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M|TIM_CCMR1_OC2M);
    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M|TIM_CCMR2_OC4M);
    TIM3->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos|0b110 << TIM_CCMR1_OC2M_Pos);
    TIM3->CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos|0b110 << TIM_CCMR2_OC4M_Pos);
    TIM3->CCMR1 |= (TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE);
    TIM3->CCMR2 |= (TIM_CCMR2_OC3PE|TIM_CCMR2_OC4PE);

    TIM3->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E);
    TIM3->EGR |= (TIM_EGR_UG);       // carga PSC, ARR y CCR

    TIM3->CR1 |= (TIM_CR1_CEN);
}

void tim5_init(void){
    RCC->APB1ENR |= (RCC_APB1ENR_TIM5EN);

    TIM5->ARR = 0xFFFFFFFF;          // contador de 32 bits completo

    // CH1 y CH2 como entradas, con filtro digital máximo contra ruido
    TIM5->CCMR1 |= (TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0);
    TIM5->CCMR1 |= (TIM_CCMR1_IC1F|TIM_CCMR1_IC2F);
    TIM5->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC2E);

    // Modo encoder 3: cuenta en los flancos de ambas señales (4 cuentas por detent)
    TIM5->SMCR &= ~(TIM_SMCR_SMS);
    TIM5->SMCR |= (TIM_SMCR_SMS_0|TIM_SMCR_SMS_1);
    TIM5->CR1 |= (TIM_CR1_CEN);
}

void tim2_init(void){
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);

    TIM2->PSC = (16-1);              // 16 MHz / 16 = 1 MHz (1 us por cuenta)
    TIM2->ARR = (0xFFFFFFFF);

    TIM2->EGR |= (TIM_EGR_UG);       // carga el PSC desde ya
    TIM2->SR = 0;                    // borra la bandera que genera el UG
    // CH3 como entrada (TI3) con filtro digital
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S);
    TIM2->CCMR2 |= (TIM_CCMR2_CC3S_0);
    TIM2->CCMR2 &= ~(TIM_CCMR2_IC3F);
    TIM2->CCMR2 |= (0b0011 << TIM_CCMR2_IC3F_Pos);

    // Captura en flanco de subida
    TIM2->CCER &= ~(TIM_CCER_CC3P|TIM_CCER_CC3NP);
    TIM2->CCER |= (TIM_CCER_CC3E);

    TIM2->DIER |= (TIM_DIER_CC3IE);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= (TIM_CR1_CEN);
}

// ISR de TIM2: guarda el valor capturado, cuenta el flanco y avisa a la FSM
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_CC3IF) {
        captured_ticks = TIM2->CCR3;   // leer CCR3 baja CC3IF automáticamente
        capture_count++;
        capture_flag = 1;
    }
}

void tim4_init(void){
    RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN);

    TIM4->CNT = 0;
    TIM4->PSC = (16000-1);               // 16 MHz / 16000 = 1 kHz (1 ms por cuenta)
    TIM4->ARR = (SW_DEBOUNCE_MS-1);      // 30 cuentas = 30 ms de debounce
    TIM4->CR1 |= (TIM_CR1_OPM);          // one-pulse: se detiene solo al terminar la cuenta
    TIM4->EGR = (TIM_EGR_UG);            // carga PSC y ARR
    TIM4->SR = ~(TIM_SR_UIF);            // borra la bandera que genera el UG
    TIM4->DIER |= (TIM_DIER_UIE);
    NVIC_EnableIRQ(TIM4_IRQn);
}

// ISR de TIM4: terminó el tiempo de debounce. Si el SW sigue presionado (PA4
// en bajo) la pulsación es válida. Luego se vuelve a habilitar EXTI4
void TIM4_IRQHandler(void){
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR = ~(TIM_SR_UIF);

        if (!(GPIOA->IDR & GPIO_IDR_ID4)) {
            sw_pressed_flag = 1;
        }

        EXTI->PR = (EXTI_PR_PR4);        // descarta rebotes pendientes
        EXTI->IMR |= (EXTI_IMR_MR4);
    }
}

void tim1_init(void){
    RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);

    TIM1->CNT = 0;
    TIM1->PSC = (1600-1);    // 16 MHz / 1600 = 10 kHz (100 us por cuenta)
    TIM1->ARR = (30-1);      // 30 cuentas = 3 ms por dígito
    TIM1->DIER |= (TIM_DIER_UIE);
    TIM1->CR1 &= ~(TIM_CR1_CKD);
    TIM1->CR1 &= ~(TIM_CR1_ARPE);
    TIM1->CR1 &= ~(TIM_CR1_DIR);
    TIM1->SR = ~(TIM_SR_UIF);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    TIM1->CR1 |= (TIM_CR1_CEN);
}

// ISR de TIM1: avisa a la FSM que debe refrescar el siguiente dígito
void TIM1_UP_TIM10_IRQHandler(void){
    if (TIM1->SR & TIM_SR_UIF){
        TIM1->SR = ~(TIM_SR_UIF);
        cambio_digito_flag = 1;
    }
}

void exti_init(void){
    // Reloj de SYSCFG: necesario para asignar un pin a cada línea EXTI
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN);

    // EXTI4 <- puerto A (PA4). EXTICR[1] agrupa las líneas 4 a 7
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4);
    SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PA);

    // Se habilita la línea 4 con flanco de bajada (SW presionado)
    EXTI->IMR |= (EXTI_IMR_MR4);
    EXTI->RTSR &= ~(EXTI_RTSR_TR4);
    EXTI->FTSR |= (EXTI_FTSR_TR4);

    // Se borra la bandera pendiente (se borra escribiendo 1)
    EXTI->PR = (EXTI_PR_PR4);

    NVIC_EnableIRQ(EXTI4_IRQn);
}

// ISR de EXTI4 (flanco de bajada de SW): deshabilita la línea para ignorar
// los rebotes y arranca TIM4, que valida la pulsación pasados 30 ms
void EXTI4_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR = (EXTI_PR_PR4);
        EXTI->IMR &= ~(EXTI_IMR_MR4);

        TIM4->CNT = 0;
        TIM4->CR1 |= (TIM_CR1_CEN);
    }
}

// Pasa al siguiente modo (ROJO -> VERDE -> AZUL -> ALL -> DAC -> ROJO)
void cambiar_modo(void) {
    current_mode = (Mode_t)(((uint32_t)current_mode + 1U) % NUM_MODES);
}

// Limita val al rango 0 a max_val
static int32_t clamp_pwm(int32_t val, int32_t max_val) {
    if (val < 0) return 0;
    if (val > max_val) return max_val;
    return val;
}

// Convierte el giro del encoder en pasos de duty y los aplica al modo actual
void cambiar_pwm(void){
    encoder_accum += encoder_delta;
    encoder_delta = 0;

    // 4 cuentas = 1 detent = 1 paso de duty
    int32_t step = encoder_accum / 4;

    if (step == 0) {
        return;
    }

    encoder_accum -= step * 4;

    switch (current_mode) {
        case MODE_ROJO:
            mode_values[MODE_ROJO] = clamp_pwm(mode_values[MODE_ROJO] + step, arr_value);
            ccr_rojo = mode_values[MODE_ROJO];
            TIM3->CCR3 = (uint32_t)ccr_rojo;
            break;

        case MODE_VERDE:
            mode_values[MODE_VERDE] = clamp_pwm(mode_values[MODE_VERDE] + step, arr_value);
            ccr_verde = mode_values[MODE_VERDE];
            TIM3->CCR1 = (uint32_t)ccr_verde;
            break;

        case MODE_AZUL:
            mode_values[MODE_AZUL] = clamp_pwm(mode_values[MODE_AZUL] + step, arr_value);
            ccr_azul = mode_values[MODE_AZUL];
            TIM3->CCR2 = (uint32_t)ccr_azul;
            break;

        case MODE_ALL:
            mode_values[MODE_ROJO]  = clamp_pwm(mode_values[MODE_ROJO]  + step, arr_value);
            mode_values[MODE_VERDE] = clamp_pwm(mode_values[MODE_VERDE] + step, arr_value);
            mode_values[MODE_AZUL]  = clamp_pwm(mode_values[MODE_AZUL]  + step, arr_value);

            ccr_rojo  = mode_values[MODE_ROJO];
            ccr_verde = mode_values[MODE_VERDE];
            ccr_azul  = mode_values[MODE_AZUL];

            TIM3->CCR3 = (uint32_t)ccr_rojo;
            TIM3->CCR1 = (uint32_t)ccr_verde;
            TIM3->CCR2 = (uint32_t)ccr_azul;
            break;

        case MODE_DAC:
            mode_values[MODE_DAC] = clamp_pwm(mode_values[MODE_DAC] + step, arr_value);
            ccr_dac = mode_values[MODE_DAC];
            TIM3->CCR4 = (uint32_t)ccr_dac;
            break;

        default:
            current_mode = MODE_ROJO;
            break;
    }
}

// Decide qué se muestra (frecuencia/10 si hay señal, duty en % si no) y lo
// separa en los 4 dígitos
void actualizar_digitos(void){
    uint32_t valor;

    if (freq_activa) {
        valor = frequency_hz / FREQ_DIV;
        if (valor > 9999) {
            valor = 9999;
        }
    }
    else {
        // En MODE_ALL se toma como referencia el duty del rojo
        uint32_t idx = (current_mode == MODE_ALL) ? MODE_ROJO : current_mode;
        valor = ((uint32_t)mode_values[idx] * 100U) / (uint32_t)arr_value;
    }

    // Se guarda en un arreglo el número correspondiente a cada dígito
    digito[0] = (valor / 1000) % 10;
    digito[1] = (valor / 100) % 10;
    digito[2] = (valor / 10) % 10;
    digito[3] = valor % 10;

    // Se apagan los ceros a la izquierda (las unidades siempre se muestran)
    for(uint8_t i = 0; i < 3 && digito[i] == 0; i++){
        digito[i] = DIGITO_APAGADO;
    }
}

// Apaga todos los segmentos y dígitos (activos en bajo, por eso se escribe 1)
void apagar(void){
    GPIOA->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD11);
    GPIOC->ODR |= (GPIO_ODR_OD10|GPIO_ODR_OD12|GPIO_ODR_OD11|GPIO_ODR_OD5|GPIO_ODR_OD13|GPIO_ODR_OD6);
    GPIOB->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD7);
    GPIOD->ODR |= (GPIO_ODR_OD2);
}

// Escribe en los 7 segmentos el patrón del dígito que se va a mostrar
void escribir(void){
    for(uint8_t i = 0; i < 7; i++){

        // Se extrae el bit i del patrón (0 = segmento encendido, 1 = apagado)
        uint8_t bit = (tabla_7seg[digito[digito_actual]] >> i) & 1;

        if(bit == 0){
            // Pin en 0: enciende el segmento
            mapa_segmentos[i].puerto->ODR &= ~(0b1 << mapa_segmentos[i].pin);
        }
        else{
            // Pin en 1: apaga el segmento
            mapa_segmentos[i].puerto->ODR |= (0b1 << mapa_segmentos[i].pin);
        }
    }
}

// Enciende el dígito que corresponde a digito_actual (escribiendo 0 en su pin)
void cambiar_digito(void){
    switch(digito_actual){
        case 0:
            GPIOC->ODR &= ~(GPIO_ODR_OD5);  // Enciende PIN 1 (PC5) -> Miles
            break;
        case 1:
            GPIOC->ODR &= ~(GPIO_ODR_OD13); // Enciende PIN 2 (PC13) -> Centenas
            break;
        case 2:
            GPIOC->ODR &= ~(GPIO_ODR_OD6);  // Enciende PIN 3 (PC6) -> Decenas
            break;
        case 3:
            GPIOB->ODR &= ~(GPIO_ODR_OD7);  // Enciende PIN 4 (PB7) -> Unidades
            break;
    }
}
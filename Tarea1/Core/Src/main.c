/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Pablo Vasquez Ospina
 * @brief          : Contador en display de 7 segmentos de 4 dígitos
 *                   (multiplexado POV), controlado por dos fotointerruptores
 *                   con interrupciones EXTI.
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
 * - Display de 7 segmentos de 4 dígitos multiplexado: los 7 segmentos (a-g)
 *   son comunes a los 4 dígitos; cada dígito se habilita con su propio pin
 *   (PIN 1-4). Cada segmento lleva una resistencia de 330 Ohm en serie.
 *   Cada dígito se maneja con un transistor PNP 2N3906 (base con una
 *   resistencia de 2 kOhm), porque un solo pin del MCU no soporta la corriente
 *   de un dígito completo. Lógica activa en bajo: un 0 enciende el segmento
 *   y un 0 en la base del transistor enciende el dígito.
 * - Dos fotointerruptores: PC3 (+1) y PA4 (-1), conectados a EXTI3 y EXTI4.
 *   Cada uno lleva una resistencia de 6,8 kOhm en su salida.
 * - Botón de reset: pulsador táctil (tact switch) de la board tactica
 *   montada sobre la Nucleo, conectado al pin NRST, disponible
 *   para ser presionado en cualquier momento.
 *
 * DISTRIBUCIÓN DE LOS PINES
 *   Dígitos (habilitación)      Segmentos
 *   PIN 1  ----- PC5            PIN a ----- PB12
 *   PIN 2  ----- PC13           PIN b ----- PA12
 *   PIN 3  ----- PC6            PIN c ----- PC11
 *   PIN 4  ----- PB7            PIN d ----- PC12
 *                               PIN e ----- PC10
 *   Fotointerruptores           PIN f ----- PA11
 *   PIN +1 ----- PC3            PIN g ----- PD2
 *   PIN -1 ----- PA4
 *
 *   LED de estado ----- PH1
 *
 * FUNCIONAMIENTO DEL SISTEMA
 * - TIM9 (interrupción cada 500 ms): invierte PH1, generando un blinky de
 *   1 Hz que indica que el MCU está funcionando.
 * - TIM3 (interrupción cada 3 ms): avanza al siguiente dígito y levanta
 *   cambio_digito_flag. Los 4 dígitos se barren en 12 ms (~83 Hz), por lo
 *   que el ojo los percibe encendidos a la vez (efecto POV). En cada cambio
 *   se apaga todo antes de escribir el siguiente dígito para evitar el
 *   "fantasma".
 * - EXTI3 (flanco de subida) y EXTI4 (flanco de bajada): cada ISR solo
 *   incrementa un contador de eventos pendientes (ev_inc, ev_dec), de modo
 *   que ningún pulso se pierda.
 * - Total: 4 interrupciones (TIM9, TIM3, EXTI3, EXTI4).
 *
 * MÁQUINA DE ESTADOS (main)
 *   REPOSO     : refresca el display cuando TIM3 lo indica.
 *   ACTUALIZAR : aplica los eventos pendientes al contador (de forma
 *                atómica) y recalcula los 4 dígitos.
 *
 *   REPOSO     --(ev_inc || ev_dec)--> ACTUALIZAR
 *   ACTUALIZAR --(siempre)-----------> REPOSO
 *
 *   El multiplexado (digito_actual, 4 posiciones) es una segunda máquina
 *   independiente, gobernada por TIM3.
 *
 * NOTA: contador es uint16_t; al pasar de 0 hacia abajo se desborda a 65535
 * (el display muestra los 4 dígitos bajos).
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
void exti_init(void);
void actualizar_valores(void);
void cambiar_digito(void);
void escribir(void);
void apagar(void);

typedef enum {
    REPOSO,
    ACTUALIZAR
} EstadoSistema;

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
const uint8_t tabla_7seg[10] = {
    0b11000000, // 0 (A,B,C,D,E,F encendidos; G y DP apagados)
    0b11111001, // 1 (B,C encendidos)
    0b10100100, // 2 (A,B,D,E,G encendidos)
    0b10110000, // 3 (A,B,C,D,G encendidos)
    0b10011001, // 4 (B,C,F,G encendidos)
    0b10010010, // 5 (A,C,D,F,G encendidos)
    0b10000010, // 6 (A,C,D,E,F,G encendidos)
    0b11111000, // 7 (A,B,C encendidos)
    0b10000000, // 8 (A,B,C,D,E,F,G encendidos)
    0b10010000  // 9 (A,B,C,D,F,G encendidos)
};

// Variables globales. Las que modifican las ISR (cambio_digito_flag, digito_actual,
// ev_inc, ev_dec) son volatile para que el compilador no las guarde en registros
// y siempre lea su valor real de memoria
volatile EstadoSistema Estado = REPOSO;
volatile uint16_t contador = 0;          // número mostrado (0 a 65535)
volatile uint8_t cambio_digito_flag = 0; // TIM3 la pone en 1 para pedir el cambio de dígito
volatile uint8_t digito_actual = 0;      // dígito encendido en este momento (0 a 3)
volatile uint8_t ev_inc = 0;             // pulsos pendientes de sumar (EXTI3)
volatile uint8_t ev_dec = 0;             // pulsos pendientes de restar (EXTI4)
uint8_t bit = 0;                         // bit del patrón que se está escribiendo
uint8_t digito[4] = {0,0,0,0};           // número de cada posición: miles, centenas, decenas, unidades

/******************************************************************************
 * PROGRAMA PRINCIPAL
******************************************************************************/
int main(void)
{   
    gpio_init();
    tim9_init();
    tim3_init();
    exti_init();

    while(1){

        switch (Estado) {

            case(REPOSO):
                // Si hay pulsos pendientes de los fotointerruptores se pasa a ACTUALIZAR
                if (ev_inc || ev_dec) 
                { 
                    Estado = ACTUALIZAR;
                }
                // Si TIM3 pidió cambiar de dígito, se refresca el display
                if (cambio_digito_flag) 
                { 
                    apagar();
                    escribir();
                    cambiar_digito();
                    cambio_digito_flag = 0;
                }
                break;
            case(ACTUALIZAR):

                actualizar_valores();
                Estado = REPOSO;  // ACTUALIZAR siempre vuelve a REPOSO
                break;
            default:
                Estado = REPOSO; 
                break;
        }
    }
}

void gpio_init(void){
    // Se encienden las señales de reloj de los puertos A, B, C, D y H
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOHEN);

    // PH1: LED de estado (blinky), salida push-pull que arranca apagada
    GPIOH->MODER  &= ~(GPIO_MODER_MODE1);
    GPIOH->MODER |= (GPIO_MODER_MODE1_0);
    GPIOH->OTYPER &= ~(GPIO_OTYPER_OT1);    
    GPIOH->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED1);
    GPIOH->PUPDR &= ~(GPIO_PUPDR_PUPD1);
    GPIOH->ODR &= ~(GPIO_ODR_OD1);

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

    // Entradas de los fotointerruptores: MODER = 00 (entrada) y PUPDR = 00 (sin pull interno)
    // PC3: fotointerruptor +1
    GPIOC->MODER &= ~(GPIO_MODER_MODE3);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD3);

    // PA4: fotointerruptor -1
    GPIOA->MODER &= ~(GPIO_MODER_MODE4);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4);
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

// ISR de TIM9: invierte el LED de estado (blinky)
void TIM1_BRK_TIM9_IRQHandler(void){
    if (TIM9->SR & TIM_SR_UIF){
        GPIOH->ODR ^= (GPIO_ODR_OD1);
        TIM9->SR = ~(TIM_SR_UIF);
    }
}

void tim3_init(void){
    RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);
    
    TIM3->CNT = 0;
    TIM3->PSC = (1600-1);    // 16 MHz / 1600 = 10 kHz (100 us por cuenta)
    TIM3->ARR = (30-1);      // 30 cuentas = 3 ms por dígito
    TIM3->DIER |= (TIM_DIER_UIE);
    TIM3->CR1 &= ~(TIM_CR1_CKD);
    TIM3->CR1 &= ~(TIM_CR1_ARPE);
    TIM3->CR1 &= ~(TIM_CR1_DIR);
    TIM3->SR = ~(TIM_SR_UIF);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= (TIM_CR1_CEN);
}

// ISR de TIM3: pasa al siguiente dígito (0 -> 1 -> 2 -> 3 -> 0) y avisa a main
void TIM3_IRQHandler(void){
    if(TIM3->SR & TIM_SR_UIF){
        TIM3->SR = ~(TIM_SR_UIF);
        digito_actual = (digito_actual+1) % 4;
        cambio_digito_flag = 1;
    }
}

void exti_init(void){
    // Reloj de SYSCFG: necesario para asignar un pin a cada línea EXTI
    RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN);
    
    // EXTI3 <- puerto C (PC3). EXTICR[0] agrupa las líneas 0 a 3
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
    SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PC);
    // EXTI4 <- puerto A (PA4). EXTICR[1] agrupa las líneas 4 a 7
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI4);
    SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI4_PA);

    // Se habilitan (desenmascaran) las líneas 3 y 4
    EXTI->IMR |= (EXTI_IMR_MR3|EXTI_IMR_MR4);
    // Flancos: EXTI3 con flanco de subida (+1), EXTI4 con flanco de bajada (-1)
    EXTI->RTSR &= ~(EXTI_RTSR_TR4);
    EXTI->FTSR &= ~(EXTI_FTSR_TR3);
    EXTI->FTSR |= (EXTI_FTSR_TR4);
    EXTI->RTSR |= (EXTI_RTSR_TR3);

    // Se borran banderas pendientes (se borran escribiendo 1)
    EXTI->PR = (EXTI_PR_PR3|EXTI_PR_PR4);

    // Se habilitan las interrupciones en el NVIC
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);

}

// ISR de EXTI3 (flanco de subida): registra un pulso para sumar
void EXTI3_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR3){
        EXTI->PR = (EXTI_PR_PR3);

        ev_inc++;
    }
}

// ISR de EXTI4 (flanco de bajada): registra un pulso para restar
void EXTI4_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR4){
        EXTI->PR = (EXTI_PR_PR4);

        ev_dec++;
    }
}

// Apaga todos los segmentos y dígitos (activos en bajo, por eso se escribe 1)
void apagar(void){
    GPIOA->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD11);
    GPIOC->ODR |= (GPIO_ODR_OD10|GPIO_ODR_OD12|GPIO_ODR_OD11|GPIO_ODR_OD5|GPIO_ODR_OD13|GPIO_ODR_OD6);
    GPIOB->ODR |= (GPIO_ODR_OD12|GPIO_ODR_OD7);
    GPIOD->ODR |= (GPIO_ODR_OD2);

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

// Aplica los pulsos pendientes al contador y separa el número en sus 4 dígitos
void actualizar_valores(void){
    uint8_t i, d;

    __disable_irq();          // lectura atómica: ninguna EXTI se cuela entre leer y borrar
    i = ev_inc;  ev_inc = 0;
    d = ev_dec;  ev_dec = 0;
    __enable_irq();

    // Un incremento o decremento por cada pulso registrado
    while(i--){ contador++; } 
    while(d--){ contador--; }

    // Se guarda en un arreglo el número correspondiente a cada dígito
    digito[0] = (contador / 1000) % 10;
    digito[1] = (contador / 100) % 10;
    digito[2] = (contador / 10) % 10;
    digito[3] = contador % 10;
}

// Escribe en los 7 segmentos el patrón del dígito que se va a mostrar
void escribir(void){
    for(uint8_t i = 0; i < 7; i++){

        // Se extrae el bit i del patrón (0 = segmento encendido, 1 = apagado)
        bit = (tabla_7seg[digito[digito_actual]] >> i) & 1;

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
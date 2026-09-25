/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Pablo Vasquez
 * @version        : V1.0.0
 * @date           : 2025-06-14
 * @brief          : Main program body
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
 */



#include <stm32f4xx.h>
#include <stdint.h>

int main(void)
{
    /* 
     * HABILITAR RELOJES PERIFERICOS (RCC) AHB1ENR
     */

    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= ~(0b11 << (2 * 5)); // Limpiar bits de modo para PA5
    GPIOA->MODER |= (0b01 << (2 * 5)); // Configurar PA5 como salida

    GPIOA->OTYPER &= ~(0b1 << 5); // Configurar PA5 como salida push-pull

    GPIOA->OSPEEDR &= ~(0b11 << (2 * 5)); // Configurar PA5 como baja velocidad
    
    GPIOA->PUPDR &= ~(0b11 << (2 * 5)); // Configurar PA5 como entrada sin pull-up ni pull-down
    
    GPIOA->ODR &= ~(1 << 5); // Apagar LED en PA5
    GPIOA->ODR |= (1 << 5); // Encender LED en PA5

    // BUcle principal
    while (1)
    {
        // Main loop code here
    }
}

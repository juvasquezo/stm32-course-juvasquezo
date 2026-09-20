/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Juan Pablo Vasquez Ospina
 * @brief          : Clculadora de prueba
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

#include <stdint.h>

/**------------------------Variables---------------------------*/
uint8_t numero1 = 10;
uint8_t numero2 = 3;
char operacion = '+';
uint8_t resultado;
/**----------------------------Cabeceras de funciones----------------------------*/
uint8_t sumar(uint8_t a, uint8_t b);
uint8_t restar(uint8_t a, uint8_t b);
uint8_t multiplicar(uint8_t a, uint8_t b);
uint8_t dividir(uint8_t a, uint8_t b);

int main(void)
{
    while(1)
    {
        if (operacion == '+')
        {   
            resultado = sumar(numero1, numero2);
        }
        else if (operacion == '-')
        {
            resultado = restar(numero1, numero2);
        }
        else if (operacion =='*')
        {
            resultado = multiplicar(numero1, numero2);
        }
        else if (operacion == '/')
        {
            if (numero2 != 0){
                resultado = dividir(numero1, numero2);
            }
        }

    }
}
/**--------------- Funciones------------------*/
uint8_t sumar(uint8_t a, uint8_t b){
    return a + b;
}
uint8_t restar(uint8_t a, uint8_t b){
    return a - b;
}
uint8_t multiplicar(uint8_t a, uint8_t b){
    return a * b;
}
uint8_t dividir(uint8_t a, uint8_t b){
    return a / b;
}
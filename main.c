// Definición de la frecuencia del CPU
#define F_CPU 16000000

// Inclusión de las bibliotecas necesarias
#include <util/delay.h>
#include <avr/io.h>
#include "PWM.h"
#include "PWM0.h"

// Declaración de variables globales
float ValorADC1 = 0;
float ValorADC2 = 0;
float ValorADC3 = 0;
float dutyCycle = 0;

// Declaración de funciones
void ADC_init(void);
uint16_t adcRead(uint8_t);

// Función principal
int main(void)
{
	// Configuración del puerto D como salida
	DDRD = 0xFF;

	// Inicialización del ADC y los PWM
	ADC_init();
	PWM_init();
	PWM0_init();

	// Bucle infinito
	while (1)
	{
		// Lectura del ADC para el canal 0 y control del servo A
		ValorADC1 = adcRead(0);
		servo_writeA(ValorADC1);
		_delay_ms(10);

		// Lectura del ADC para el canal 1 y control del servo B
		ValorADC2 = adcRead(1);
		servo_writeB(ValorADC2);
		_delay_ms(10);

		// Lectura del ADC para el canal 2 y ajuste del ciclo de trabajo del PWM
		ValorADC3 = adcRead(2);
		dutyCycle = map(ValorADC3, 0, 1023, 0, 100);
		PWM0_dcb(dutyCycle, NO_INVERTING);
	}
}

// Función de inicialización del ADC
void ADC_init(void){
	ADMUX |= (1<<REFS0);    // Referencia de voltaje en VCC
	ADMUX &= ~(1<<REFS1);
	ADMUX &= ~(1<<ADLAR);   // Resolución de 10 bits
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    // Preescalador 128 para 125KHz
	ADCSRA |= (1<<ADEN);    // Habilitación del ADC
}

// Función para leer el ADC
uint16_t adcRead(uint8_t canal){
	ADMUX = (ADMUX & 0xF0)|canal;    // Selección del canal
	ADCSRA |= (1<<ADSC);    // Inicia la conversión
	while((ADCSRA)&(1<<ADSC));    // Espera hasta que la conversión finalice
	return(ADC);
}

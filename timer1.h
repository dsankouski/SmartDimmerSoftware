/**
  * Timer 1 is used for determining zero crossing period
  * todo: switch control pins to control it with 1 timer.
  */

#include <util/atomic.h>

#define OUTPUT_COMPARE_1A_INT_ENABLE_OR_MASK 1 << OCIE1A
#define OUTPUT_COMPARE_1A_INT_DISABLE_AND_MASK ~(1 << OCIE1A)
#define SET_ON_COMPARE_1A_OR_MASK (1 << COM3A1)
#define CLEAR_COMPARE_MATCH_1A_MASK 1 << OCF1A
#define TOGGLE_ON_COMPARE_1A_AND_MASK ~(1 << COM3A1)
#define PRESCALER_MASK_1 (1 << CS10) + (1 << CS12)
#define WAVEFORM_GENERATOR_MASK_REG_1A ~((1 << WGM10) + (1 << WGM11))

void timer_1_setup() {
	/* prescaler 1/1024 */
    /* CSn2:0 = 101 */
    TCCR1B = PRESCALER_MASK_1;

    /* Normal mode TOP = MAX*/
	TCCR1A &= ~(1 << WGM10);
	TCCR1A &= ~(1 << WGM11);
	TCCR1B &= ~(1 << WGM12);
	TCCR1B &= ~(1 << WGM13);

	TCCR1A |= 1 << COM1A0;
}

ISR(TIMER1_COMPA_vect){
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        TIMSK1 &= OUTPUT_COMPARE_1A_INT_DISABLE_AND_MASK;
        TCCR1A |= SET_ON_COMPARE_1A_OR_MASK;
		OCR1A += 30;
	}
}


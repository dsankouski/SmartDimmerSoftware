#include <util/atomic.h>


#define OUTPUT_COMPARE_3A_INT_ENABLE_OR_MASK 1 << OCIE3A
#define OUTPUT_COMPARE_3A_INT_DISABLE_OR_MASK ~(1 << OCIE3A)
#define SET_ON_COMPARE_3A_OR_MASK (1 << COM3A1)
#define TOGGLE_ON_COMPARE_3A_AND_MASK ~(1 << COM3A1)
#define CLEAR_COMPARE_MATCH_3A_MASK 1 << OCF3A
#define PRESCALER_MASK_3 (1 << CS10) + (1 << CS12)
#define WAVEFORM_GENERATOR_MASK_REG_1A ~((1 << WGM10) + (1 << WGM11))


/* *
 * This timer controls AC phase on channel 1
 */
void timer_3_setup() {
    	/* prescaler 1/1024 */
    	/* CSn2:0 = 101 */
    	TCCR3B = PRESCALER_MASK_3;

    	/* Normal mode TOP = MAX*/
        TCCR3A &= ~(1 << WGM30);
        TCCR3A &= ~(1 << WGM31);
        TCCR3B &= ~(1 << WGM32);
        TCCR3B &= ~(1 << WGM33);


	/* Toggle OCnA/OCnB/OCnC on compare match*/
    	TCCR3A |= 1 << COM3A0;
}

ISR(TIMER3_COMPA_vect){
ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        TIMSK3 &= OUTPUT_COMPARE_3A_INT_DISABLE_OR_MASK;
        TCCR3A |= SET_ON_COMPARE_3A_OR_MASK;
		OCR3A += 1;
	}
}
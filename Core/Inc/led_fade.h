#ifndef LED_FADE_H_
#define LED_FADE_H_ 1


#include "main.h"

typedef enum FadeMode_ {
	FADE_MODE_AUTO,
	FADE_MODE_MANUAL
} FadeMode;

typedef struct LedFadeState_ {
	FadeMode mode;
	int8_t dir;
	uint8_t val;
	uint8_t target;
} LedFadeState;

void set_duty_cycle(uint8_t d);

void fade_init(LedFadeState *state);
void fade_tick(LedFadeState *state);
void fade_manual(LedFadeState* state);
void fade_set_target(LedFadeState* state, uint8_t target);
void fade_auto(LedFadeState* state);

// reference to global instance
extern volatile LedFadeState led_fade_state;

#endif

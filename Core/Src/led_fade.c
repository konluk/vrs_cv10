#include "led_fade.h"


void set_duty_cycle(uint8_t d) {
	LL_TIM_OC_SetCompareCH1(TIM2, d);
}

// init the state of the fader
void fade_init(LedFadeState *state) {
	state->mode = FADE_MODE_AUTO;
	state->dir = 1;
	state->val = 0;
	state->target = 0;
}

// called from interrupt, sets the PWM duty cycle based on state
void fade_tick(LedFadeState *state) {

	if(state->mode == FADE_MODE_AUTO) {
		if(state->val >= 99) {
			state->dir = -1;
		}
		else if (state->val == 0) {
			state->dir = 1;
		}
		state->val += state->dir;
	}
	else if(state->mode == FADE_MODE_MANUAL) {
		if(state->val < state->target) {
			state->val++;
		}
		else if(state->val > state->target) {
			state->val--;
		}
	}
	set_duty_cycle(state->val);
}

// set manual mode, the target is the current state
void fade_manual(LedFadeState* state) {
	state->target = state->val;
	state->mode = FADE_MODE_MANUAL;
}

// sets the target for manual mode, doesn't enable manual mode
void fade_set_target(LedFadeState* state, uint8_t target) {
	if(target > 99) {
		target = 99;
	}
	state->target = target;
}

//sets auto mode
void fade_auto(LedFadeState* state) {
	state->mode = FADE_MODE_AUTO;
}


// global instance of state
volatile LedFadeState led_fade_state;

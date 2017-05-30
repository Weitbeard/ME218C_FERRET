;==========================================================================
;FERRET Controller - 218C Project Infrastructure - Spring '17
;History:
;Author	-   Date -	Notes -
;LW	    05/28/17	Added second servo and refined sequencing
;LW	    05/12/17	Core functionality tested and debugged with hardware
;LW	    05/11/17	Implemented remainder of requirements & added
;			    mpasm relocatable code directives
;LW	    05/09/17	Implemented timers, run button, heartbeat LED,
;			    and lift fan control
;LW	    04/28/17	Revised template for PWM on CCP (Timer2)
;LW	    04/20/17	Initial copy of old template code  
;==========================================================================

;==========================================================================
;Defines, PIC configuration, and code organization
;==========================================================================

;PIC configuration
    list P=PIC16F690
#include "P16F690.inc"
    __config(_CP_OFF & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _INTOSCIO)
    
;Pin mapping
#define HB_LED	    RA0		;Heartbeat LED to show PIC activity
#define RUN_IND	    RA1		;LED to indicate RUNNING status
#define RUN_BTN	    RA2		;Button to start a run routine
#define	SRVB_CNTRL  RC1		;Servo B (Right) control signal output
#define	SRVA_CNTRL  RC2		;Servo A (Left) control signal output
#define	TF_DIR	    RC4		;Thrust fan direction output pin
#define	TF_PWM	    RC5		;Thrust fan PWM output pin
#define LIFT_CNTRL  RC6		;Lift fan control pin

;Timer 0 defines (Servo timing values)
#define	TWENTY_MS   H'4E'	;time = hex value * 0.256ms
#define	SRVA_ENG    H'08'	;Pulse width ~ 2ms
#define	SRVA_DIS    H'06'	;Pulse width ~ 1.5ms
#define	SRVB_ENG    H'08'	;Pulse width ~ 1.5ms
#define	SRVB_DIS    H'06'	;Pulse width ~ 2ms

;Timer 1 defines (Approximate time values)
#define	HALF_S	    H'0F'	;time = hex value * 32.8ms
#define ONE_S	    H'1E'
#define TWO_S	    H'3D'	    
#define THREE_S	    H'5C'	    
#define FOUR_S	    H'7A'	    
#define FIVE_S	    H'99'	    
#define SIX_S	    H'B7'
#define	SEVEN_S	    H'D6'
#define EIGHT_S	    H'F4'
#define	HB_TIME	    HALF_S
#define	DEB_TIME    FIVE_S
    
;Timer 2 defines (Thrust fan PWM configurations)
#define	TF_PERIOD   H'65'	;Period for thrust fan PWM (PR2 value)
#define	TF_FULL     TF_PERIOD	;100% duty cycle on thrust fan
#define TF_HALF     H'32'	;50% duty cycle on thrust fan
#define TF_OFF	    H'00'	;0% duty cycle

;Flag defines
#define	RUNNING	    0		;flag to note if FERRET is in RUNNING state
#define DEBOUNCING  1		;flag to note if FERRET is in DEBOUNCING state
#define SRVA_ENGD   2		;flag to note if servo A is ENGAGED
#define SRVB_ENGD   3		;flag to note if servo B is ENGAGED
    
;Misc. & readability defines
#define	HB_ON	    H'01'	;bit value in PORTA to turn the HB LED on
#define	NUM_CYCLS   H'04'	;number of RUN cycles to repeat per routine
#define NUM_STEPS   H'0C'	;number of steps per RUN cycle - 1
    
;Variable assignment (stored in common access bank registers)
.vars	udata_shr   H'70'
; Temporary register storage for ISR
W_TEMP	    res 1   ;temporary storage for W register
STATUS_TEMP res 1   ;temporary storage for STATUS register
PCLATH_TEMP res 1   ;temporary storage for PCLATH register
; Software counters/timers
CYCL_CNT    res	1   ;RUN sequence cycle counter
SRV_CNT	    res 1   ;servo tick counter
SRVA_PW	    res 1   ;servo A control signal PW counter
SRVB_PW	    res 1   ;servo B control signal PW counter
SRV_TMR	    res 1   ;general servo control signal timer
RUN_TMR	    res 1   ;RUN step timer
HB_TMR	    res 1   ;heartbeat blink timer
DEB_TMR	    res 1   ;button debounce timer
; Misc. vars
FLAGS	    res 1   ;register to hold various boolean flags
RUN_INDX    res	1   ;RUN sequence index
;End of variable assignment

;Reset vector code organization
.reset	code	H'00'	    ;unconditional branch to MAIN at (0) memory location
	goto	MAIN_SETUP  ;

;Interrupt vector code organization
.ivect	code	H'04'	    ;unconditional branch to ISR at (4) memory location
	goto	ISR_PUSH    ;

;Lookup table code organization
.tables	code	H'05'	    ;start lookup tables immediately following int. vec.
	
;==========================================================================
;RUN_STEP_SEQ lookup table - hardcoded routine steps
;==========================================================================
RUN_STEP_SEQ:			    ;
	addwf	PCL,F		    ;
	goto	RUN_HRD_BRAKE	    ;1st
	goto	RUN_SFT_BRAKE	    ;2
	goto	RUN_DRIFT	    ;3
	goto	RUN_HALF_FWD	    ;4
	goto	RUN_FULL_FWD	    ;5th
	goto	RUN_DRIFT	    ;6
	goto	RUN_BRAKE_LEFT	    ;7
	goto	RUN_HALF_LEFT	    ;8
	goto	RUN_FULL_LEFT	    ;9
	goto	RUN_BRAKE_RIGHT	    ;10th
	goto	RUN_HALF_RIGHT	    ;11
	goto	RUN_FULL_RIGHT	    ;12
	goto	RUN_OFF		    ;13

;==========================================================================
;RUN_STEP_TIME lookup table - hardcoded routine times for each step
;==========================================================================
RUN_STEP_TIME:		    ;
	addwf	PCL,F	    ;
	retlw	THREE_S	    ;1st
	retlw   THREE_S	    ;2
	retlw   EIGHT_S	    ;3
	retlw   THREE_S	    ;4
	retlw   THREE_S	    ;5th
	retlw   THREE_S	    ;6
	retlw   THREE_S	    ;7
	retlw   THREE_S	    ;8
	retlw   THREE_S	    ;9
	retlw   THREE_S	    ;10th
	retlw   THREE_S	    ;11
	retlw   THREE_S	    ;12
	retlw   THREE_S	    ;13
	;total time should be ~44 seconds (x5 cycles)
	
;==========================================================================
;MAIN setup code
;==========================================================================
.main	code			;begin main body code
	
MAIN_SETUP:			;
	;Clock configuration
	banksel	OSCCON		;configure for 8MHz internal osc speed
	movlw	B'01110000'	;
	movwf	OSCCON		;
WAIT_4_CLOCK:			;wait for stable clock
	btfss	OSCCON,HTS	;
	goto	WAIT_4_CLOCK	;

	;Pin configuration (Port A & C Basic Preparation)
	; Port Clearing
	banksel PORTA		;clear port A (PORTA register)
	clrf	PORTA		;
	banksel PORTC		;clear port C (PORTC register)
	clrf	PORTC		;
	; Mode Configuration
	banksel	ANSEL		;set all I/O pins as digital (ANSEL(H) register)
	clrf	ANSEL		;
	banksel	ANSELH		;
	clrf	ANSELH		;
	; Direction Configuration
	banksel	TRISA		;set all A pins as outputs (TRISA register)
	clrf	TRISA		;and set the Run button pin to an input
	bsf	TRISA,RUN_BTN	;
	banksel	TRISC		;set all C pins as outputs (TRISC register)
	clrf	TRISC		;
	
	;Run Button IOC configuration
	banksel	INTCON		;enable IOC in control register
	bcf	INTCON,RABIF	; - clear flag
	bsf	INTCON,RABIE	; - enable interrupt
	banksel	IOCA		;locally enable IOC for run button
	bsf	IOCA,RUN_BTN	;
	
	;Timer 0 counter configuration (multi-servo control)
	banksel	OPTION_REG	;
	bcf	OPTION_REG,T0CS	;set clock source as FOSC/4 (TOCS in OPTION_REG)
	bcf	OPTION_REG,PSA	;assign prescaler to Timer 0
	bcf	OPTION_REG,PS0	;set min. prescale value (PS<2:0> in OPTION_REG)
	bcf	OPTION_REG,PS1	; -> (Prescale) * (256 * 4) / FOSC = Period
	bcf	OPTION_REG,PS2	; -> (2) * (256 * 4) / FOSC = 0.256ms
	banksel	INTCON		;
	bcf	INTCON,T0IF	;clear any existing Timer 0 interrupt flags
	bsf	INTCON,T0IE	;enable Timer 0 overflow interrupt
	
	;Timer 1 configuration (heartbeat & random timers)
	banksel	T1CON		;set clock source to FOSC/4 (TMR1CS in T1CON)
	bcf	T1CON,TMR1CS	;
	bcf	T1CON,T1CKPS0	;set prescale to 1:1 (T1CKPS<1:0> in T1CON
	bcf	T1CON,T1CKPS1	;
	banksel	PIR1		;enable rollover interrupt:
	bcf	PIE1,TMR1IF	; - clear interrupt flag (TMR1IF in PIR1)
	banksel	PIE1		; - local enable (TMR1IE in PIE1)
	bsf	PIE1,TMR1IE	;
	banksel INTCON		; - peripheral enable (PEIE in INTCON)
	bsf	INTCON,PEIE	;
	banksel	T1CON		;turn on the timer (TMR1ON in T1CON)
	bsf	T1CON,TMR1ON	;
	
	;PWM configuration (Thrust Fan Control):
	;   Using Method from Datasheet:
	;	1. Disable PWM pin (CCP1) by setting associated TRIS bit
	;	2. Set PWM period by loading PR2 register
	;	3. Configure CCP module for PWM by loading CCP1CON
	;	4. Set duty cycle by loading CCPR1L & DC1B<1:0> of CCP1CON
	;	5. Configure and start Timer2
	;	    a. Clear TMR2IF interrupt flag in PIR1
	;	    b. Set Timer2 prescale value by loading T2CKPS in T2CON
	;	    c. Enable Timer2 by setting TMR2ON in T2CON
	;	6. Enable PWM ouput
	;	    a. Wait until Timer2 overflows (TMR2IF interrupt flag)
	;	    b. Enable CCP1 pin by clearing associated TRIS bit
	
	;   Example Calculations:
	;   PWM Period = [(PR2)+1]*(4/FOSC)*TMR2PreScale
	;       -> PR2 = [PWM Period]*FOSC/(4*TMR2PreScale)-1
	;       ->     = [1/5kHz]*8MHz/(4*4)-1 = 99 (0x63)
	;   Duty Cycle = (CCPR1L:CCP1CON<5:4>)/[4*(PR2+1)]
	;	-> CCPR1L:CCP1CON<5:4> = [Duty Cycle]*[4*(PR2+1)]
	;	->		       = [0.5]*[4*99+1] = 50 (0x32)
	
	banksel TRISC		; Step 1
	bsf	TRISC,TF_PWM	;
	banksel	PR2		; Step 2
	movlw	TF_PERIOD	;
	movwf	PR2		;
	banksel	CCP1CON		; Step 3
	movlw	B'00001100'	; *include 2 LSBs of duty cycle here (00)*
	movwf	CCP1CON		;
	call	TF_SET_OFF	; Step 4
	banksel PIR1		; Step 5a
	bcf	PIR1,TMR2IF	;
	banksel T2CON		; Step 5b
	movlw	B'00000001'	;
	movwf	T2CON		;
	bsf	T2CON,TMR2ON	; Step 5c
	banksel PIR1		; Step 6a
WAIT_4_CYCLE:			;
	btfss	PIR1,TMR2IF	;
	goto	WAIT_4_CYCLE	;
	banksel TRISC		; Step 6b
	bcf	TRISC,TF_PWM	;
	
	;ADC configuration (Thrust Fan Speed Adjust);
	;   Using Method from Datasheet:
	;	1. Configure the analog input pin
	;	    a. Set the associated ANSEL bit
	;	    b. Set the associated TRIS bit
	;	2. Set the appropriate input channel (CHS bit in ADCON0)
	;	.......etc
	; ********TODO********
	
	;Init software timer/counter values
	call	INIT_VARS	;
	
	;Enable global interrupts
	bsf	INTCON,GIE	;
	
	goto 	MAIN_LOOP	;once setup complete, go to the run LOOP

;==========================================================================
;MAIN run code LOOP
;==========================================================================
MAIN_LOOP:			;
	nop			;put any run code here (i.e. event checkers)	
	goto	MAIN_LOOP	;return to start of run LOOP

;==========================================================================
;INIT_VARS routine code
;==========================================================================
INIT_VARS:			;
	movlw	HB_TIME		;load heartbeat blink time
	movwf	HB_TMR		;
	clrf	FLAGS		;clear all program flags
	goto	RUN_OFF		;disable all motors (turn FERRET OFF)
	
;==========================================================================
;ISR routine code
;==========================================================================
ISR_PUSH:			    ;
	movwf	W_TEMP		    ;start by storing W, STATUS, and PCLATH
	swapf	STATUS,W	    ;  in TEMP registers
	movwf	STATUS_TEMP	    ;
	movf	PCLATH,W	    ;
	movwf	PCLATH_TEMP	    ;
	
ISR_IOCA:			    ;
	banksel INTCON		    ;check if IOC fired on port A
	btfsc	INTCON,RABIF	    ; if the RUN button interrupt flag is high
	call	IOC_RESP	    ; call IOC response routine
	
ISR_T0:				    ;
	banksel	INTCON		    ;check Timer 0 interrupt
	btfss	INTCON,T0IE	    ; check local interrupt enable status
	goto	ISR_T1		    ;  if clear, skip to next section of ISR
	btfsc	INTCON,T0IF	    ;  else, check interrupt flag status
	call	TMR0_OVFL	    ;    if set, call the Timer 0 ovrflw routine

ISR_T1:				    ;
	banksel	PIE1		    ;check Timer 1 interrupt
	btfss	PIE1,TMR1IE	    ; check local interrupt enable status
	goto	ISR_POP		    ;  if clear, skip to next section of ISR
	banksel	PIR1		    ;
	btfsc	PIR1,TMR1IF	    ; check interrupt flag status
	call	TMR1_OVFL	    ;  if set, call the Timer 0 overflow routine

ISR_POP:
	movf	PCLATH_TEMP,W	    ;finish by pulling back the original W,
	movwf	PCLATH		    ; STATUS, and PCLATH from the TEMP registers
	swapf	STATUS_TEMP,W	    ;
	movwf	STATUS		    ;
	swapf	W_TEMP,F	    ;
	swapf	W_TEMP,W	    ;
	retfie			    ;return from the ISR
	
;==========================================================================
;IOC_RESP routine code
;==========================================================================
IOC_RESP:			    ;
	call	DEBOUNCE_START	    ;start the extended debouncing routine
	btfss	FLAGS,RUNNING	    ;if not currently RUNNING,
	goto	START_CHASE	    ; start the chase sequence
	goto	END_CHASE	    ;else, end the chase sequence
	
;==========================================================================
;DEBOUNCE_START routine code
;==========================================================================
DEBOUNCE_START:			    ;
	banksel	PORTA		    ;sample Port A (clears mismatch condition)
	movf	PORTA,W		    ;
	banksel	IOCA		    ;locally disable IOC for run button
	bcf	IOCA,RUN_BTN	    ;
	banksel	INTCON		    ;clear INTCON IOC flag
	bcf	INTCON,RABIF	    ;
	bsf	FLAGS,DEBOUNCING    ;set debouncing flag
	movlw	DEB_TIME	    ;load debounce time into timer register
	movwf	DEB_TMR		    ;
	return			    ;
	
;==========================================================================
;DEBOUNCE_END routine code
;==========================================================================
DEBOUNCE_END:			    ;
	bcf	FLAGS,DEBOUNCING    ;clear debouncing flag
	banksel	INTCON		    ;
	bcf	INTCON,RABIF	    ;clear IOC flag in INTCON
	banksel	IOCA		    ;locally enable IOC for run button
	bsf	IOCA,RUN_BTN	    ;
	return			    ;
	
;==========================================================================
;START_CHASE branch code
;==========================================================================
START_CHASE:			    ;
	banksel	INTCON		    ;clear the port A IOC flag
	bcf	INTCON,RABIF	    ;
	bsf	FLAGS,RUNNING	    ;change RUNNING state to true
	banksel	PORTA		    ;enable RUNNING indicator LED
	bsf	PORTA,RUN_IND	    ;
	clrf	RUN_INDX	    ;reset RUN sequence index
	movlw	NUM_CYCLS	    ;reset cycle count
	movwf	CYCL_CNT	    ;
	return			    ;return from the subroutine
	
;==========================================================================
;END_CHASE branch code
;==========================================================================
END_CHASE:			    ;
	bcf	FLAGS,RUNNING	    ;change RUNNING state to false
	banksel	PORTA		    ;disable RUNNING indicator
	bcf	PORTA,RUN_IND	    ;
	goto	RUN_OFF		    ;disable all motors (turn FERRET OFF)

;==========================================================================
;TMR0_OVFL routine code (servo PWM)
;==========================================================================
TMR0_OVFL:			    ;
	banksel	INTCON		    ;clear Timer 0 interrupt flag
	bcf	INTCON,T0IF	    ;
	
	;Servo control output timing
	decf	SRV_CNT,W	    ;decrement the counter
	btfsc	STATUS,Z	    ;if zero,
	movlw	TWENTY_MS	    ; load 20 ms back into counter
	movwf	SRV_CNT		    ;
	
	;Servo A
	banksel	PORTC		    ;check if SRV_CNT is above SRVA_PW
	subwf	SRVA_PW,W	    ;
	btfss	STATUS,C	    ; if above, clear SRV_CNTRL pin
	bcf	PORTC,SRVA_CNTRL    ;
	btfsc	STATUS,C	    ; else, set SRV_CNTRL pin
	bsf	PORTC,SRVA_CNTRL    ;
	
	;Servo B
	banksel	PORTC		    ;check if SRV_CNT is above SRVB_PW
	subwf	SRVB_PW,W	    ;
	btfss	STATUS,C	    ; if above, clear SRV_CNTRL pin
	bcf	PORTC,SRVB_CNTRL    ;
	btfsc	STATUS,C	    ; else, set SRV_CNTRL pin
	bsf	PORTC,SRVB_CNTRL    ;
	   
	return			    ;return from the subroutine
	
	
;==========================================================================
;TMR1_OVFL routine code (software timers/counters)
;==========================================================================
TMR1_OVFL:			    ;
	banksel	PIR1		    ;clear Timer 1 interrupt flag
	bcf	PIR1,TMR1IF	    ;
	
	; Heartbeat control timing
	decf	HB_TMR,F	    ;increment heartbeat LED timer
	btfsc	STATUS,Z	    ;
	call	TOGGLE_HB	    ; if cleared, toggle the heartbeat

DEBOUNCING_LOOP:		    ;DEBOUNCING sequence loop
	btfss	FLAGS,DEBOUNCING    ;check if in DEBOUNCING state
	goto	RUNNING_LOOP	    ; if not, go to next part of Timer 1 ISR
	decf	DEB_TMR,F	    ;decrement debounce timer
	btfsc	STATUS,Z	    ; if debounce time is over
	call	DEBOUNCE_END	    ;	end the debouncing
	
RUNNING_LOOP:			    ;RUNNING sequence loop
	btfss	FLAGS,RUNNING	    ;check if in RUNNING state
	goto	END_TMR1_OVFL	    ; if not, go to next part of Timer 1 ISR
	decf	RUN_TMR,F	    ;decrement RUN step timer
	btfsc	STATUS,Z	    ;
	call	RUN_CNTRL	    ; if cleared, call the RUN control routine
	
END_TMR1_OVFL:
	return			    ;return from the subroutine
	
;==========================================================================
;TOGGLE_HB routine code
;==========================================================================
TOGGLE_HB:		        ;
	banksel PORTA		;xor PORTA with the heartbeat LED bit value
	movlw	HB_ON		;
	xorwf	PORTA,F		;
	movlw	HB_TIME		;reset HB_TIME
	movwf	HB_TMR		;
	return			;	
	
;==========================================================================
;RUN_CNTRL routine code
;==========================================================================
RUN_CNTRL:			    ;
	incf	RUN_INDX,F	    ;begin next step in RUN sequence:
	movf	RUN_INDX,W	    ; - increment index
	sublw	NUM_STEPS	    ;
	btfss	STATUS,C	    ; - if a cycle is complete
	goto	NEXT_CYCLE	    ;	   begin a new cycle
	goto	RUN_STEP	    ; - otherwise, run the next step

NEXT_CYCLE:
	clrf	RUN_INDX	    ;reset RUN step index
	decfsz	CYCL_CNT,W	    ;decrement cycle counter
	goto	RUN_STEP	    ;  if not 0, run the first sequence step
	goto	END_CHASE	    ;  if 0, stop the CHASE routine
    
RUN_STEP:			    ;	
	call	RUN_STEP_SEQ	    ;call nex RUN step from sequence
	movf	RUN_INDX,W	    ;get the corresponding step run time
	call	RUN_STEP_TIME	    ;put it into the step timer
	movwf	RUN_TMR		    ;
	return			    ;

;==========================================================================
;TF_SET_FULL routine code
;==========================================================================
TF_SET_FULL:			    ;
    banksel	CCPR1L		    ;set thrust fan duty cycle to FULL
    movlw	TF_FULL		    ;
    movwf	CCPR1L		    ;
    return			    ;

;==========================================================================
;TF_SET_HALF routine code
;==========================================================================
TF_SET_HALF:			    ;
    banksel	CCPR1L		    ;set thrust fan duty cycle to HALF
    movlw	TF_HALF		    ;
    movwf	CCPR1L		    ;
    return			    ;
    
;==========================================================================
;TF_SET_OFF routine code
;==========================================================================
TF_SET_OFF:			    ;
    banksel	CCPR1L		    ;set thrust fan duty cycle to OFF
    movlw	TF_OFF		    ;
    movwf	CCPR1L		    ;
    return			    ;
	
;==========================================================================
;SRVA_ON routine code
;==========================================================================
SRVA_ON:			    ;
	movlw	SRVA_ENG	    ;set servo A pw to ENGAGED
	movwf	SRVA_PW		    ;
	bsf	FLAGS,SRVA_ENGD	    ;
	return			    ;
	
;==========================================================================
;SRVA_OFF routine code
;==========================================================================
SRVA_OFF:			    ;
	movlw	SRVA_DIS	    ;set servo A pw to DISENGAGED
	movwf	SRVA_PW		    ;
	bcf	FLAGS,SRVA_ENGD	    ;
	return			    ;

;==========================================================================
;SRVB_ON routine code
;==========================================================================
SRVB_ON:			    ;
	movlw	SRVB_ENG	    ;set servo B pw to ENGAGED
	movwf	SRVB_PW		    ;
	bsf	FLAGS,SRVB_ENGD	    ;
	return			    ;
	
;==========================================================================
;SRVB_OFF routine code
;==========================================================================
SRVB_OFF:			    ;
	movlw	SRVB_DIS	    ;set servo B pw to DISENGAGED
	movwf	SRVB_PW		    ;
	bcf	FLAGS,SRVB_ENGD	    ;
	return			    ;
	
;==========================================================================
;LIFT_ON routine code
;==========================================================================
LIFT_ON:			    ;
	banksel	PORTC		    ;turn ON the lift fan
	bsf	PORTC,LIFT_CNTRL    ;
	return			    ;
    
;==========================================================================
;LIFT_OFF routine code
;==========================================================================
LIFT_OFF:			    ;
	banksel	PORTC		    ;turn OFF the lift fan
	bcf	PORTC,LIFT_CNTRL    ;
	return			    ;
	
	
;==========================================================================
;**** RUN STEP BEHAVIOR FUNCTIONS ****
;==========================================================================	
RUN_OFF:			    ;case: FERRET off
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_OFF	    ;lift	-	OFF
	return			    ;
	
RUN_HRD_BRAKE:			    ;case: hard brake
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_ON		    ;servo A    -	ON
	call	SRVB_ON		    ;servo B    -	ON
	call	LIFT_OFF	    ;lift	-	OFF
	return			    ;
	
RUN_SFT_BRAKE:			    ;case: soft brake
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_ON		    ;servo A    -	ON
	call	SRVB_ON		    ;servo B    -	ON
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_DRIFT:			    ;case: idle drifting
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_FULL_FWD:			    ;case: full thrust forward
	call	TF_SET_FULL	    ;thrust	-	FULL
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_HALF_FWD:			    ;case: half thrust forward
	call	TF_SET_HALF	    ;thrust	-	HALF
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_FULL_LEFT:			    ;case: full thrust left turn
	call	TF_SET_FULL	    ;thrust	-	FULL
	call	SRVA_ON		    ;servo A    -	ON
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_HALF_LEFT:			    ;case: half thrust left turn
	call	TF_SET_HALF	    ;thrust	-	HALF
	call	SRVA_ON		    ;servo A    -	ON
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;

RUN_BRAKE_LEFT:			    ;case: no thrust left turn
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_ON		    ;servo A    -	ON
	call	SRVB_OFF	    ;servo B    -	OFF
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_FULL_RIGHT:			    ;case: full thrust right turn
	call	TF_SET_FULL	    ;thrust	-	FULL
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_ON		    ;servo B    -	ON
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_HALF_RIGHT:			    ;case: half thrust right turn
	call	TF_SET_HALF	    ;thrust	-	HALF
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_ON		    ;servo B    -	ON
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
RUN_BRAKE_RIGHT:		    ;case: no thrust right turn
	call	TF_SET_OFF	    ;thrust	-	OFF
	call	SRVA_OFF	    ;servo A    -	OFF
	call	SRVB_ON		    ;servo B    -	ON
	call	LIFT_ON		    ;lift	-	ON
	return			    ;
	
;==========================================================================
;Utility functions
;==========================================================================	   
    
    	;******** TEST TOGGLE METHOD *************
TOGGLE_TEST_LED:		;
	banksel PORTC		; Using pin RC1
	movlw	H'02'		;
	xorwf	PORTC,F		;
	return			;
	;******** END TEST TOGGLE ****************
    
;==========================================================================
;END of code
;==========================================================================
    END				;
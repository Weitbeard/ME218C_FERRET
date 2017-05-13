;==========================================================================
;FERRET Controller - 218C Project Infrastructure - Spring '17
;History:
;Author	-   Date -	Notes -
;LW	    05/12/17	Core functionality tested and debugged with hardware
;LW	    05/11/17	Implemented remainder of requirements & added
;			    mpasm relocatable code directives
;LW	    05/09/17	Implemented timers, run button, and heartbeat LED,
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
#define	SRVA_CNTRL  RC2		;Servo A control signal output
#define	TF_DIR	    RC4		;Thrust fan direction output pin
#define	TF_PWM	    RC5		;Thrust fan PWM output pin
#define LIFT_CNTRL  RC6		;Lift fan control pin

;Timer 0 defines (Servo timing values)
#define	TWENTY_MS   H'4E'	;time = hex value * 0.256ms
#define	SRVA_ENG    H'08'	;Pulse width ~ 2ms
#define	SRVA_DIS    H'06'	;Pulse width ~ 1.5ms

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
#define	TF_DEF_DC   H'0D'	;Default thrust fan PWM duty cycle (~50%)
#define	FULL_FWD    H'00'	;100% duty cycle forward
#define HALF_FWD    H'0D'	;50% duty cycle forward
#define STOPPED	    H'00'	;0% duty cycle
#define HALF_REV    H'00'	;50% duty cycle reverse
#define FULL_REV    H'00'	;100% duty cycle reverse

;Flag defines
#define	RUNNING	    0		;flag to note if FERRET is in RUNNING state
#define DEBOUNCING  1		;flag to note if FERRET is in DEBOUNCING state
#define SRVA_ENGD   2		;flag to note if servo A is ENGAGED
    
;Misc. & readability defines
#define	HB_ON	    H'01'	;bit value in PORTA to turn the HB LED on 
    
;Variable assignment (stored in common access bank registers)
.vars	udata_shr   H'70'
; Temp register storage for ISR
W_TEMP	    res 1   ;temporary storage for W register
STATUS_TEMP res 1   ;temporary storage for STATUS register
PCLATH_TEMP res 1   ;temporary storage for PCLATH register
; Software timers/counters
SRV_CNT	    res 1   ;servo tick counter
SRV_TMR	    res 1   ;general servo control signal timer
SRVA_PW	    res 1   ;servo A control signal PW
SRVA_TMR    res 1   ;servo A ON/OFF timer
TF_TMR	    res 1   ;thrust fan ON/OFF timer
HB_TMR	    res 1   ;heartbeat blink timer
DEB_TMR	    res 1   ;button debounce timer
; Misc. vars
FLAGS	    res 1   ;register to hold various boolean flags
SRVA_SIND   res	1   ;servo A sequence index
TF_SIND	    res	1   ;thrust fan sequence index
TF_DC	    res 1   ;temporary DC value storage for thrust fan
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
;SRVA_SEQ lookup table - hardcoded routine times for servo A
;==========================================================================
SRVA_SEQ:		    ;ordered time values to be used with servo A
	addwf	PCL,F	    ;
	retlw	SIX_S	    ;1st
	retlw   FIVE_S	    ;2
	retlw   FIVE_S	    ;3
	retlw   FIVE_S	    ;4
	retlw   FIVE_S	    ;5th
	retlw   FIVE_S	    ;6
	retlw   FIVE_S	    ;7
	retlw   FIVE_S	    ;8
	retlw   FIVE_S	    ;9
	retlw   FIVE_S	    ;10th
	; ***** TODO *****
	;total time should be ~138 seconds

;==========================================================================
;TF_TIME_SEQ lookup table - hardcoded routine times for thrust fan
;==========================================================================
TF_TIME_SEQ:		    ;ordered time values to be used with thrust fan
	addwf	PCL,F	    ;
	retlw	THREE_S	    ;1st
	retlw   FIVE_S	    ;2
	retlw   FIVE_S	    ;3
	retlw   FIVE_S	    ;4
	retlw   FIVE_S	    ;5th
	retlw   FIVE_S	    ;6
	retlw   FIVE_S	    ;7
	retlw   FIVE_S	    ;8
	retlw   FIVE_S	    ;9
	retlw   FIVE_S	    ;10th
	; ***** TODO *****
	;total time should be ~138 seconds
	
;==========================================================================
;TF_PW_SEQ lookup table - hardcoded routine times for thrust fan PWM DC
;==========================================================================
TF_DC_SEQ:		    ;ordered DC values to be used with thrust fan
	addwf	PCL,F	    ;
	retlw	STOPPED	    ;1st
	retlw   HALF_FWD    ;2
	retlw	STOPPED	    ;3
	retlw   HALF_FWD    ;4
	retlw	STOPPED	    ;5th
	retlw   HALF_FWD    ;6
	retlw	STOPPED	    ;7
	retlw   HALF_FWD    ;8
	retlw	STOPPED	    ;9
	retlw   HALF_FWD    ;10th
	; ***** TODO *****
	
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
	banksel	TRISC		;set all C pins as outputs (TRISA register)
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
	banksel	CCPR1L		; Step 4
	movlw	STOPPED		;
	movwf	CCPR1L		;
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
	movlw	H'00'		;load 1st sequence OFF time for servo
	call	SRVA_SEQ	;
	movwf	SRV_TMR		;
	movlw	H'00'		;load 1st sequence OFF time for thrust fan
	call	TF_TIME_SEQ	;
	movwf	TF_TMR		;
	call	SRVA_OFF	;disable servo A
	return			;
	
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
	btfsc	INTCON,T0IF	    ; check interrupt flag status
	call	TMR0_OVFL	    ;  if set, call the Timer 0 overflow routine

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
	banksel	PORTA		    ;enable RUNNING indicator
	bsf	PORTA,RUN_IND	    ;
	banksel	PORTC		    ;enable lift fan
	bsf	PORTC,LIFT_CNTRL    ;
	clrf	SRVA_SIND	    ;reset sequence indeces
	clrf	TF_SIND		    ;
	return			    ;return from the subroutine
	
;==========================================================================
;END_CHASE branch code
;==========================================================================
END_CHASE:			    ;
	bcf	FLAGS,RUNNING	    ;change RUNNING state to false
	banksel	PORTA		    ;disable RUNNING indicator
	bcf	PORTA,RUN_IND	    ;
	banksel	PORTC		    ;disable lift fan
	bcf	PORTC,LIFT_CNTRL    ;
	movlw	STOPPED		    ;disable thrust fan
	movwf	TF_DC		    ;
	call	TF_SET_DC	    ;
	call	SRVA_OFF	    ;disengage servo
	return			    ;return from the subroutine

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
	banksel	PORTC		    ;check if SRV_CNT is above SRV_PW
	subwf	SRVA_PW,W	    ;
	btfss	STATUS,C	    ; if above, clear SRV_CNTRL pin
	bcf	PORTC,SRVA_CNTRL    ;
	btfsc	STATUS,C	    ; else, set SRV_CNTRL pin
	bsf	PORTC,SRVA_CNTRL    ;
	   
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
	; Servo ON/OFF control timing
	decf	SRV_TMR,F	    ;decrement servo timer
	btfsc	STATUS,Z	    ;
	call	SRV_CNTRL	    ; if cleared, call the servo control rout.
	; Thrust fan ON/OFF control timing
	decfsz	TF_TMR,F	    ;decrement thrust fan timer
	btfsc	STATUS,Z	    ;
	call	TF_CNTRL	    ; if cleared, toggle the thrust fan
	
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
;SRV_CNTRL routine code
;==========================================================================
SRV_CNTRL:			    ;
	incf	SRVA_SIND,F	    ;load new value into srvo timer
	movf	SRVA_SIND,W	    ; - increment index ****TODO**** NO OVERFLOW PROTECTION!
	call	SRVA_SEQ	    ; - get next time from sequence
	movwf	SRV_TMR		    ; - put it in timer
	btfss	FLAGS,SRVA_ENGD	    ;if servo A is not engaged,
	goto	SRVA_ON		    ; engage servo A
	goto	SRVA_OFF	    ;else, disengage servo A
	
;==========================================================================
;SRVA_ON routine code
;==========================================================================
SRVA_ON:			;
	movlw	SRVA_ENG        ;set servo A pw to ENGAGED (pw ~2ms)
	movwf	SRVA_PW		;
	bsf	FLAGS,SRVA_ENGD	;
	return			;
	
;==========================================================================
;SRVA_OFF routine code
;==========================================================================
SRVA_OFF:		        ;
	movlw	SRVA_DIS	;set servo A pw to DISENGAGED (~1ms)
	movwf	SRVA_PW		;
	bcf	FLAGS,SRVA_ENGD	;
	return			;

;==========================================================================
;TF_CNTRL routine code
;==========================================================================
TF_CNTRL:			    ;
	incf	TF_SIND,F	    ;load new value into tf timer
	movf	TF_SIND,W	    ; - increment index ****TODO**** NO OVERFLOW PROTECTION!
	call	TF_TIME_SEQ	    ; - get next time from sequence
	movwf	TF_TMR		    ; - put it in timer
	movf	TF_SIND,W	    ;
	call	TF_DC_SEQ	    ; - get next duty cycle from sequence
	movwf	TF_DC		    ;
	call	TF_SET_DC	    ;set the thrust fan PWM duty cycle
	return			    ;
	
;==========================================================================
;TF_SET_DC routine code
;==========================================================================
TF_SET_DC:			    ;
    
    ;***** TODO **** check for positive/negative dc values & adjust direction pin
    
    ;if forward:
    banksel PORTC	        ;adjust direction pin
    bcf	    PORTC,TF_DIR        ; 
    banksel	CCPR1L		;load new dc
    movf	TF_DC,W		;
    movwf	CCPR1L		;
    
;    ;if reverse:
;    banksel PORTC
;    bsf	    PORTC,TF_DIR
    
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
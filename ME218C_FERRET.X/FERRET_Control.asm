;==========================================================================
;FERRET Controller - 218C Project Infrastructure - Spring '17
;by Luke Weitkemper
;04/20/17
;==========================================================================

;==========================================================================
;Defines, PIC configuration, and code organization
;==========================================================================

;Readability defines
#define	bABit		RA0		;H-Bridge signal A output on pin A0
#define	bBBit		RA1		;H-Bridge signal B output on pin A1
#define	stepBit 	RA2		;Step signal input on pin A2
#define	modeBit		RA4		;Mode signal input on pin A4
#define	dirBit		RA5		;Direction signal input on pin A5
#define	stepSpeed	H'C3'		;Match value for Timer2 (195 * 2.56*10e-5 s) ~= 50ms

W_TEMP			equ	H'40'	;temporary storage for W register
STATUS_TEMP		equ	H'41'	;temporary storage for STATUS register

;PIC Configuration
	list	P=PIC12F752
#include "P12F752.inc"
	__config(_CP_OFF & _WDT_OFF & _PWRTE_ON & _CKLOUTEN_OFF & _FOSC_INT & _MCLRE_OFF)

;MAIN code organization
	org		0		;unconditional branch to MAIN at (0) memory location
	goto	MAIN	;

;ISR code organization
	org		4		;unconditional branch to ISR at (4) memory location
	goto	ISR		;

;==========================================================================
;MAIN setup code
;==========================================================================
MAIN:				;

	;Clock configuration
	;leave default int osc speed of 1 MHz
	
	;Pin configuration
	banksel PORTA		;clear port A (PORTA register)
	clrf	PORTA		;
	banksel	LATA		;clear latch A (LATA register)
	clrf	LATA		;
	banksel	ANSELA		;set all I/O pins as digital (ANSEL register)
	clrf	ANSELA		;
	banksel	TRISA		;set pins 0,1 as outputs (low) and 2,4,6 as inputs
	movlw	B'00111100'	; (high) - (TRIS register / pins default to high)
	movwf	TRISA		;internal weak pull-ups are auto-configured

	;Interrupt configuration
	banksel	INTCON			;enable interrupts in control register
	bsf		INTCON,GIE		; - global interrupts
	bsf		INTCON,IOCIE	; - IOC interrupts
	bsf		INTCON,PEIE		; - peripheral interrupts
	banksel	IOCAP			;locally enable rising edge interrupt on 
	bsf		IOCAP,stepBit	; pin 2 (step) - (IOCAP register)
	banksel	IOCAN			;locally enable falling edge interrupt on
	bsf		IOCAN,modeBit	; pin 4 (mode) - (IOCAN register)
	banksel	PIE1			;locally enable Timer2 interrupt
	bsf		PIE1,TMR2IE		; - (PIE1 register)

	;Timer2 configuration
	banksel	PR2				;default Timer2 control register values are good
	movlw	stepSpeed		;set match value for Timer2 (auto-step speed)
	movwf	PR2				;
	movlw	B'01111011'		;set prescaler and postscaler values to 1:16
	iorwf	T2CON,f			; for Timer2
	banksel PORTA			;
	btfsc	PORTA,modeBit	;if MODE_SELECT is currently low (auto-step enabled)
	call 	AUTO_STEP		;setup Timer2 for automatic stepping

	goto 	LOOP	;once setup complete, go to the run LOOP

;==========================================================================
;MAIN run code LOOP
;==========================================================================
LOOP:					;
	nop					;put any run code here (i.e. event checkers)
	goto	LOOP		;return to start of run LOOP
	
;==========================================================================
;AUTO_STEP mode setup routine
;==========================================================================
AUTO_STEP:
	banksel T2CON			;clear current count on Timer2(all TMR2 
	clrf	TMR2			; registers on same bank)
	bsf		T2CON,TMR2ON	;turn on Timer2
	banksel	IOCAP			;disable STEP_SIGNAL interrupt (IOCAP register)
	bcf		IOCAP,stepBit	;
	return					;

;==========================================================================
;MAN_STEP mode setup routine
;==========================================================================
MAN_STEP:
	banksel T2CON			;
	bcf		T2CON,TMR2ON	;turn off Timer2
	banksel	IOCAP			;enable STEP_SIGNAL interrupt (IOCAP register)
	bsf		IOCAP,stepBit	;
	return					;

;==========================================================================
;ISR routine code
;==========================================================================
ISR:						;start by storing W and STATUS in TEMP
	banksel STATUS			; registers
	movwf	W_TEMP			;
	swapf	STATUS,w		;
	movwf	STATUS_TEMP		;
	banksel	INTCON			;clear high-level IOC interrupt flag
	bcf		INTCON,IOCIF	;
	banksel PIR1			;determine source of interrupt and branch as needed
	btfsc	PIR1,TMR2IF		;if Timer2 match interrupt flag is high
	call	TMR2_ISR		;call TMR2_ISR
	banksel IOCAF			;
	btfsc	IOCAF,modeBit	;else if MODE_SIGNAL interrupt flag is high
	call	MODE_ISR		;call MODE_ISR
	btfsc	IOCAF,stepBit	;else if STEP_SIGNAL interrupt flag is high
	call	STEP_ISR		;call STEP_ISR
	banksel	STATUS			;
	swapf	STATUS_TEMP,w	;finish by pulling back the original W
	movwf	STATUS			; and STATUS from the TEMP registers
	swapf	W_TEMP,f		;
	swapf	W_TEMP,w		;
	retfie					;return from the ISR

TMR2_ISR:					;for Timer2 match interrupt
	banksel	PIR1			;clear interrupt source (PIR1 register)
	bcf		PIR1,TMR2IF		;
	btfss	PORTA,modeBit	;if auto-step mode has been disabled
	goto	MODE_LOW		;skip this &
	call	MAN_STEP		;call MAN_STEP mode setup routine
	return					;
MODE_LOW:					;else
	call	STEP			;call STEP routine
	return					;

MODE_ISR:					;for MODE_SELECT interrupt
	bcf		IOCAF,modeBit	;clear interrupt source (IOCAF register)
	bcf		IOCAF,stepBit	;clear interrupt flag for STEP_SIGNAL if it exists
	call	AUTO_STEP		;enable AUTO_STEP mode
	bcf		IOCAP,stepBit	;disable STEP_SIGNAL interrupt (IOCAP register)
	return					;

STEP_ISR:					;for STEP_SIGNAL interrupt
	bcf		IOCAF,stepBit	;clear interrupt source (IOCAF register)
	call	STEP			;call STEP routine
	return					;

;==========================================================================
;STEP routine code
;==========================================================================
STEP:						;
	banksel	PORTA			;check STEP_DIRECTION signal state
	btfss	PORTA,dirBit	;if STEP_DIRECTION is CW (high)
	goto	DEC_INDEX		;skip this &
	incf	StepIndex,f		;increment StepIndex
	goto	IND_CHANGED		;
DEC_INDEX:					;else
	decf	StepIndex,f		;decrement StepIndex
IND_CHANGED:				;
	movlw	B'00000011'		;mask off lower two bounds of StepIndex and
	andwf	StepIndex,w		; move StepIndex to w
	call	STEP_TABLE		;perform table lookup
	banksel	LATA			;
	movwf	LATA			;write returned literal to PORTA
	return					;

	END



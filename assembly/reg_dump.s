/******************************************************************************
* @FILE mod.s
* @BRIEF register print example
*
* Simple example of printing register values to terminal for debugging
*
* @AUTHOR Christopher D. McMurrough
******************************************************************************/
 
    .global main
    .func main
   
main:
    MOV R1, #100        @ set a constant value for printing
	MOV R2, #200        @ set a constant value for printing
	MOV R3, #300        @ set a constant value for printing
	MOV R4, #400        @ set a constant value for printing
	MOV R5, #500        @ set a constant value for printing
	MOV R6, #600        @ set a constant value for printing
	MOV R7, #700        @ set a constant value for printing
	MOV R8, #800        @ set a constant value for printing
	MOV R9, #900        @ set a constant value for printing
	MOV R10, #1000      @ set a constant value for printing
	MOV R11, #1100      @ set a constant value for printing
	MOV R12, #1200      @ set a constant value for printing
	MOV R13, #1300      @ set a constant value for printing
    MOV R2, #75         @ set a constant value for printing
    BL  _reg_dump       @ print register contents
    B   _exit           @ branch to exit procedure with no return
   
_exit:  
    MOV R7, #4          @ write syscall, 4
    MOV R0, #1          @ output stream to monitor, 1
    MOV R2, #21         @ print string length
    LDR R1,=exit_str    @ string at label exit_str:
    SWI 0               @ execute syscall
    MOV R7, #1          @ terminate syscall, 1
    SWI 0               @ execute syscall
       
_reg_dump:
    PUSH {LR}           @ backup registers
	PUSH {R0}           @ backup registers
	PUSH {R1}           @ backup registers
	PUSH {R2}           @ backup registers
	PUSH {R3}           @ backup registers
	
	PUSH {R14}          @ push registers for printing
	PUSH {R13}          @ push registers for printing
	PUSH {R12}          @ push registers for printing
	PUSH {R11}          @ push registers for printing
	PUSH {R10}          @ push registers for printing
	PUSH {R9}           @ push registers for printing
	PUSH {R8}           @ push registers for printing
	PUSH {R7}           @ push registers for printing
	PUSH {R6}           @ push registers for printing
	PUSH {R5}           @ push registers for printing
	PUSH {R4}           @ push registers for printing
	PUSH {R3}           @ push registers for printing
	PUSH {R2}           @ push registers for printing
	PUSH {R1}           @ push registers for printing
    PUSH {R0}           @ push registers for printing
	
	LDR R0,=debug_str   @ prepare register print
	
	MOV R1, #0          @ prepare R0 print
	POP {R2}            @ prepare R0 print
	MOV R3, R2          @ prepare R0 print
	BL printf           @ print R0 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R1 print
	POP {R2}            @ prepare R1 print
	MOV R3, R2          @ prepare R1 print
	BL printf           @ print R1 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R2 print
	POP {R2}            @ prepare R2 print
	MOV R3, R2          @ prepare R2 print
	BL printf           @ print R2 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R3 print
	POP {R2}            @ prepare R3 print
	MOV R3, R2          @ prepare R3 print
	BL printf           @ print R3 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R4 print
	POP {R2}            @ prepare R4 print
	MOV R3, R2          @ prepare R4 print
	BL printf           @ print R4 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R5 print
	POP {R2}            @ prepare R5 print
	MOV R3, R2          @ prepare R5 print
	BL printf           @ print R5 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R6 print
	POP {R2}            @ prepare R6 print
	MOV R3, R2          @ prepare R6 print
	BL printf           @ print R6 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R7 print
	POP {R2}            @ prepare R7 print
	MOV R3, R2          @ prepare R7 print
	BL printf           @ print R7 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R8 print
	POP {R2}            @ prepare R8 print
	MOV R3, R2          @ prepare R8 print
	BL printf           @ print R8 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R9 print
	POP {R2}            @ prepare R9 print
	MOV R3, R2          @ prepare R9 print
	BL printf           @ print R9 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R10 print
	POP {R2}            @ prepare R10 print
	MOV R3, R2          @ prepare R10 print
	BL printf           @ print R10 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R11 print
	POP {R2}            @ prepare R11 print
	MOV R3, R2          @ prepare R11 print
	BL printf           @ print R11 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R12 print
	POP {R2}            @ prepare R12 print
	MOV R3, R2          @ prepare R12 print
	BL printf           @ print R12 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R13 print
	POP {R2}            @ prepare R13 print
	MOV R3, R2          @ prepare R13 print
	BL printf           @ print R13 value prior to reg_dump call
	
	MOV R1, #0          @ prepare R14 print
	POP {R2}            @ prepare R14 print
	MOV R3, R2          @ prepare R14 print
	BL printf           @ print R14 value prior to reg_dump call
	
	POP {R3}            @ restore register
	POP {R2}            @ restore register
	POP {R1}            @ restore register
	POP {R0}            @ restore regsiter
	POP {PC}            @ return
 
.data
debug_str:
.asciz "R%d:    %X    %d \n"
exit_str:
.ascii "Terminating program.\n"
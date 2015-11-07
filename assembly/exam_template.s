/******************************************************************************
* @FILE exam_template.ss
* @BRIEF code template for CSE 2312 final exam
*
* Program prompts the user for 10 postive integers, then computes and prints 
* the sum, minimum, and maximum values before terminating.
*
* @AUTHOR Christopher D. McMurrough
******************************************************************************/
 
    .global main
    .func main
   
main:    
    BL  _print_name     @ print programmer name
    BL  _print_prompt   @ print user prompt
    BL  _get_input      @ retrieve 10 user input values
    BL  _calc_result    @ compute the sum, min, and max of the top 10 values on the stack
    BL  _print_result   @ print print result
    B   _exit           @ branch to exit procedure with no return
 
 _print_name:
    PUSH {LR}            @ store LR since printf call overwrites
    LDR R0,=name_str     @ R0 contains formatted string address
    BL printf            @ call printf
    POP {PC}             @ return
    
 _print_prompt:
    PUSH {LR}            @ store LR since printf call overwrites
    LDR R0,=prompt_str   @ R0 contains formatted string address
    BL printf            @ call printf
    POP {PC}             @ return
    
_get_input:
    MOV R4, LR           @ store LR since _scanf call overwrites LR, R0, R1
    MOV R5, #0           @ initialize loop counter
    _loop1: BL _scanf            @ get a number from console
            PUSH {R0}            @ push the number to the stack
            ADD R5, R5, #1       @ increment loop counter
            CMP R5, #10          @ check for end of loop
            BNE _loop1           @ loop if necessary
    MOV PC, R4           @ return
    
_calc_result:
    MOV R5, #0           @ initialize loop counter
    _loop2: POP {R1}             @ remove a value from the stack
            ADD R5, R5, #1       @ increment loop counter
            CMP R5, #10          @ check for end of loop
            BNE _loop2           @ loop if necessary
    MOV R7, #70         @ store the sum in R7
    MOV R8, #80         @ store the min in R8
    MOV R9, #90         @ store the max in R9
    MOV PC, LR          @ return

 _print_result:
    PUSH {LR}           @ store LR since printf call overwrites
    MOV R1, R7          @ move stored sum to R7 for printing
    MOV R2, R8          @ move stored sum to R8 for printing
    MOV R3, R9          @ move stored sum to R9 for printing
    LDR R0,=result_str  @ R0 contains formatted string address
    BL printf           @ call printf
    POP {PC}            @ return
    
_exit:  
    MOV R7, #4          @ write syscall, 4
    MOV R0, #1          @ output stream to monitor, 1
    MOV R2, #21         @ print string length
    LDR R1,=exit_str    @ string at label exit_str:
    SWI 0               @ execute syscall
    MOV R7, #1          @ terminate syscall, 1
    SWI 0               @ execute syscall
    
_scanf:
    PUSH {LR}               @ store LR since scanf call overwrites
    SUB SP, SP, #4          @ make room on stack
    LDR R0, =format_str     @ R0 contains address of format string
    MOV R1, SP              @ move SP to R1 to store entry on stack
    BL scanf                @ call scanf
    LDR R0, [SP]            @ load value at SP into R0
    ADD SP, SP, #4          @ restore the stack pointer
    POP {PC}                @ return
       
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

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #1          @ prepare R1 print
    POP {R2}            @ prepare R1 print
    MOV R3, R2          @ prepare R1 print
    BL printf           @ print R1 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #2          @ prepare R2 print
    POP {R2}            @ prepare R2 print
    MOV R3, R2          @ prepare R2 print
    BL printf           @ print R2 value prior to reg_dump call
 
    LDR R0,=debug_str   @ prepare register print
    MOV R1, #3          @ prepare R3 print
    POP {R2}            @ prepare R3 print
    MOV R3, R2          @ prepare R3 print
    BL printf           @ print R3 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #4          @ prepare R4 print
    POP {R2}            @ prepare R4 print
    MOV R3, R2          @ prepare R4 print
    BL printf           @ print R4 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #5          @ prepare R5 print
    POP {R2}            @ prepare R5 print
    MOV R3, R2          @ prepare R5 print
    BL printf           @ print R5 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #6          @ prepare R6 print
    POP {R2}            @ prepare R6 print
    MOV R3, R2          @ prepare R6 print
    BL printf           @ print R6 value prior to reg_dump call
 
    LDR R0,=debug_str   @ prepare register print
    MOV R1, #7          @ prepare R7 print
    POP {R2}            @ prepare R7 print
    MOV R3, R2          @ prepare R7 print
    BL printf           @ print R7 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #8          @ prepare R8 print
    POP {R2}            @ prepare R8 print
    MOV R3, R2          @ prepare R8 print
    BL printf           @ print R8 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #9          @ prepare R9 print
    POP {R2}            @ prepare R9 print
    MOV R3, R2          @ prepare R9 print
    BL printf           @ print R9 value prior to reg_dump call
    
    LDR R0,=debug_str   @ prepare register print
    MOV R1, #10          @ prepare R10 print
    POP {R2}            @ prepare R10 print
    MOV R3, R2          @ prepare R10 print
    BL printf           @ print R10 value prior to reg_dump call
    
    LDR R0,=debug_str   @ prepare register print
    MOV R1, #11         @ prepare R11 print
    POP {R2}            @ prepare R11 print
    MOV R3, R2          @ prepare R11 print
    BL printf           @ print R11 value prior to reg_dump call
    
    LDR R0,=debug_str   @ prepare register print
    MOV R1, #12         @ prepare R12 print
    POP {R2}            @ prepare R12 print
    MOV R3, R2          @ prepare R12 print
    BL printf           @ print R12 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #13         @ prepare R13 print
    POP {R2}            @ prepare R13 print
    MOV R3, R2          @ prepare R13 print
    BL printf           @ print R13 value prior to reg_dump call

    LDR R0,=debug_str   @ prepare register print
    MOV R1, #14         @ prepare R14 print
    POP {R2}            @ prepare R14 print
    MOV R3, R2          @ prepare R14 print
    BL printf           @ print R14 value prior to reg_dump call
    
    POP {R3}            @ restore register
    POP {R2}            @ restore register
    POP {R1}            @ restore register
    POP {R0}            @ restore regsiter
    POP {PC}            @ return
    
.data
name_str:
.asciz "Sarah Connor: 1000123456 \n"
prompt_str:
.asciz "Please enter 10 positive integers: \n"
format_str:     
.asciz "%d"
result_str:
.asciz "Sum: %10d     Min: %10d     Max: %10d \n"
debug_str:
.asciz "R%-2d   0x%08X  %011d \n"
exit_str:
.ascii "Terminating program.\n"

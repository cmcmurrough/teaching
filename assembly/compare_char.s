/******************************************************************************
* @FILE compare_char.s
* @BRIEF simple scanf and compare
*
* Simple example of invoking scanf to retrieve a char from keyboard input, and
* testing to see if it is equal to a given value
*
* @AUTHOR Christopher D. McMurrough
******************************************************************************/
 
    .global main
    .func main
   
main:
    BL  _prompt             @ branch to printf procedure with return
    BL  _getchar            @ branch to scanf procedure with return
    MOV R1, R0              @ move return value R0 to argument register R1
    BL  _compare            @ check the scanf input
    B   _exit               @ branch to exit procedure with no return
   
_exit:  
    MOV R7, #4              @ write syscall, 4
    MOV R0, #1              @ output stream to monitor, 1
    MOV R2, #21             @ print string length
    LDR R1, =exit_str       @ string at label exit_str:
    SWI 0                   @ execute syscall
    MOV R7, #1              @ terminate syscall, 1
    SWI 0                   @ execute syscall
 
_prompt:
    MOV R7, #4              @ write syscall, 4
    MOV R0, #1              @ output stream to monitor, 1
    MOV R2, #23             @ print string length
    LDR R1, =prompt_str     @ string at label prompt_str:
    SWI 0                   @ execute syscall
    MOV PC, LR              @ return
   
_getchar:
    MOV R4, LR              @ store LR since scanf call overwrites
    SUB SP, SP, #4          @ make room on stack
    LDR R0, =format_str     @ R0 contains address of format string
    MOV R1, SP              @ move SP to R1 to store entry on stack
    BL scanf                @ call scanf
    LDR R0, [SP]            @ load value at SP into R0
    ADD SP, SP, #4          @ restore the stack pointer
    MOV PC, R4              @ return
 
_compare:
    MOV R2, #64
    CMP R1, R2
    BEQ _correct            @ branch to equal handler
    BNE _incorrect          @ branch to not equal handler
    MOV PC, R4
 
_correct:
    MOV R5, LR              @ store LR since printf call overwrites
    LDR R0, =equal_str      @ R0 contains formatted string address
    BL printf               @ call printf
    MOV PC, R5              @ return
 
_incorrect:
    MOV R5, LR              @ store LR since printf call overwrites
    LDR R0, =nequal_str     @ R0 contains formatted string address
    BL printf               @ call printf
    MOV PC, R5              @ return
 
.data
format_str:     .asciz      "%d"
prompt_str:     .ascii      "Enter the @ character: "
equal_str:      .asciz      "CORRECT \n"
nequal_str:     .asciz      "INCORRECT: %d \n"
exit_str:       .ascii      "Terminating program.\n"
/******************************************************************************
* @FILE scanf.s
* @BRIEF simple scanf example
*
* Simple example of invoking scanf to retrieve a number from keyboard input
*
* @AUTHOR Christopher D. McMurrough
******************************************************************************/
 
    .global main
    .func main
   
main:
    BL  _scan               @ branch to scan procedure with return
    MOV R1, R0              @ move return value R0 to argument register R1
    BL  _print              @ branch to print procedure with return
    B   _exit               @ branch to exit procedure with no return
   
_exit:  
    MOV R7, #4              @ write syscall, 4
    MOV R0, #1              @ output stream to monitor, 1
    MOV R2, #21             @ print string length
    LDR R1, =exit_str       @ string at label exit_str:
    SWI 0                   @ execute syscall
    MOV R7, #1              @ terminate syscall, 1
    SWI 0                   @ execute syscall
       
_print:
    MOV R4, LR              @ store LR since printf call overwrites
    LDR R0, =print_str      @ R0 contains formatted string address
    MOV R1, R1              @ R1 contains printf argument (redundant line)
    BL printf               @ call printf
    MOV PC, R4              @ return
    
_scan:
    MOV R4, LR              @ store LR since scanf call overwrites
    SUB SP, SP, #4          @ make room on stack
    LDR R0, =addr_format    @ R0 contains address of format string
    MOV R1, SP              @ move SP to R1 to store entry on stack
    BL scanf                @ call scanf
    LDR R0, [SP]            @ load value at SP into R0
    @LDR R2, [SP]            @ load value at SP into R2
    @LDR R3, addr_number     @ load value at addr_number into R3
    @STR R2, [R3]            @ store value in R2 into address in R3
    ADD SP, SP, #4          @ restore the stack pointer
    MOV PC, R4              @ return
 
@ scanf requires addresses of strings in the text memory area.
addr_number:    .word       number
addr_format:    .word       scanformat
 
.data
number:         .word       0
format_str:     .asciz      "%d"
print_str:      .asciz      "The number entered was: %d\n"
exit_str:       .ascii      "Terminating program.\n"
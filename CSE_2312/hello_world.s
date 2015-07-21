/******************************************************************************
* @FILE hello_world.s
* @BRIEF Simple example of printing text to console
* @AUTHOR Christopher D. McMurrough
******************************************************************************/

	.global  _start
    
_start:
	MOV R7, #4	        @ write syscall
 	MOV R0, #1	        @ output stream 1 for monitor
	MOV R2, #19	        @ print string length
	LDR R1,=hello_str   @ string located at string:
	SWI 0	            @ execute syscall
	B   _exit           @ branch to exit procedure
    
_exit:   
	MOV R7,             @ terminate syscall
	SWI 0               @ execute syscall

.data
hello_str:
.ascii "Hello World!\n"


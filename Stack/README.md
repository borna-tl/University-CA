This project is a Multi-Cycle implementation of a Stack-Based MIPS processor.
The processor has an 8-bit data bus and a 5-bit address bus. It can store 32x8 bits. Each instruction is an 8-bit input with the three leftmost bits as an opcode.

The opcode table is as follows: 

![Q](https://user-images.githubusercontent.com/70484744/130335021-347c7342-6227-4c98-9798-3b059e94ce7c.PNG)

For example, the ADD instruction will pop the first and the second elements of the stack, and then push their sum onto the top! Or the JMP instruction will make the PC jump
to the specified address.

To test the processor's functionality we have written a simple prorgram to add and subtract random numbers (previously stored in the memory).
 

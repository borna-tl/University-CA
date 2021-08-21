In this project we have implemented the MIPS processor using pipelines!

The following are the instructions supported by our processor:
* **Arithmetic/Logical Instructions:** add, addi, sub, slt, slti
* **Memory Reference Instruction:** lw, sw
* **Control Flow Instructions:** j, jal, jr, beq, bne

We have removed the data hazards with the _Forwarding_ technique. Control hazards can also be resolved using _NOP_ instructions. 

To check the processor's functionality, we have written a program that calculates the sum of the elements
of an array of size 10 (first element located at _address=1000_) and stores the result at _address=2000_.

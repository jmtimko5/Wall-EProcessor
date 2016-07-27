lw $r25, 1($r0)
addi $r27, $r25, 100
lw $r26, 1($r0)
sw $r2, 300($r26)
addi $r1, $r0, 10
addi $r2, $r0, 2
addi $r3, $r0, 3
add $r4, $r2, $r3
add $r5, $r4, $r1
add $r6, $r1, $r3
addi $r7, $r6, 100
add $r8, $r6, $r1
addi $r10, $r0, 1
lw $r9, 0($r10)
sw $r9, 1($r10)
add $r21, $r1, $r0
addi $r21, $r21, 100
sub $r22, $r21, $r5
and $r11, $r9, $r10
or $r12, $r10, $r9
sll $r13, $r1, 4
sra $r14, $r13, 2
bne $r1, $r2, 1
sub $r0, $r0, $r0
bne $r0, $r0, 200
j 26
jal 28
addi $r0, $r0, 0
blt $r1, $r3, 1000
blt $r3, $r1, 1
sub $r0, $r0, $r0
addi $r31, $r0, 1000
jr $r31

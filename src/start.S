/*
 * Entry point. The A3 header occupies the first 32 bytes of the image and is
 * followed by 32 bytes of guard, so this code is linked at 0xFE020040 and the
 * loader's entry is 0xFE020041.
 */

    .syntax unified
    .arch armv7-a
    .thumb

    .section .a3hdr, "a", %progbits
    .global _a3_hdr
_a3_hdr:
    .space 32

    .section .text.start, "ax", %progbits
    .global _start
    .type _start, %function
    .thumb_func
_start:
    /* Leave the free-running counter in a known state. 0xFFFF0E96 is Thumb
     * code (its callers at 0xFFFF0D64 use a Thumb bl, and its pc-relative
     * literal only resolves to 0xF2403000 at 0xFFFF0F44 under Thumb rules),
     * so the pointer must carry bit0 or blx switches to ARM state and dies
     * on an undefined instruction.  33 MB is beyond Thumb bl's +-16 MB
     * range, hence the register call. */
    ldr     r0, =0xFFFF0E97
    blx     r0

    ldr     r1, =__stack_top
    mov     sp, r1

    /*
     * Take the exception vectors over before anything else can fault.
     *
     * Until now the CPU has been using the BootROM's table at 0xFFFF0000,
     * whose handlers simply park the core -- on the bench that shows up as the
     * unexplained "2 short" loop that takes the lamp away from this payload.
     * With our own table every unexpected exception reports which one it was,
     * so a fault after the host attaches can be identified instead of guessed.
     */
    ldr     r0, =_vectors
    mcr     p15, 0, r0, c12, c0, 0      @ VBAR = _vectors
    mrc     p15, 0, r0, c1, c0, 0       @ SCTLR
    bic     r0, r0, #(1 << 13)          @ V = 0: vectors come from VBAR
    /*
     * Caches off, same as the ZV-E10 toolkit does on this core.
     *
     * The DWC3 reads its TRBs and the event buffer over AXI.  With the
     * caches on, a TRB write can sit in L1/L2 and the controller sees a
     * stale descriptor -- and, the other way round, event records the
     * controller wrote appear as the stale (all zero) cache lines we
     * filled, so every counter reads 0 even though the host is talking to
     * us.  The ROM's MMU state is left untouched.
     */
    bic     r0, r0, #(1 << 12)          @ I = 0
    bic     r0, r0, #(1 << 2)           @ C = 0
    mcr     p15, 0, r0, c1, c0, 0
    dsb
    isb

    ldr     r2, =__bss_start
    ldr     r3, =__bss_end
    movs    r0, #0
1:
    cmp     r2, r3
    bhs     2f
    str     r0, [r2], #4
    b       1b

2:
    bl      main

3:
    b       3b

/*
 * Exception vector table.  Every entry reports a distinct code through
 * exc_report() in main.c:
 *
 *   1 reset (spurious)   2 undefined   3 supervisor
 *   4 prefetch abort     5 data abort  6 IRQ        7 FIQ
 *
 * A reported code means the payload really did take that exception; before
 * this table existed the BootROM's handler swallowed it and all the bench saw
 * was an unexplained 2-short loop.
 */
    .section .text.vectors, "ax"
    .align 5
    .global _vectors
_vectors:
    b   exc_reset
    nop
    b   exc_undef
    nop
    b   exc_svc
    nop
    b   exc_pabt
    nop
    b   exc_dabt
    nop
    b   exc_hyp
    nop
    b   exc_irq
    nop
    b   exc_fiq
    nop

exc_reset:  movs r0, #1
            b   exc_report
exc_undef:  movs r0, #2
            b   exc_report
exc_svc:    movs r0, #3
            b   exc_report
exc_pabt:   movs r0, #4
            b   exc_report
exc_dabt:   movs r0, #5
            b   exc_report
exc_hyp:    movs r0, #6
            b   exc_report
exc_irq:    movs r0, #6
            b   exc_report
exc_fiq:    movs r0, #7
            b   exc_report

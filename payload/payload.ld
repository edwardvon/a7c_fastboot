/*
 * Sony A7C (ILCE-7C) bare-metal payload.
 *
 * Everything lives in the 64 KB window the BootROM loads into. There is no
 * second SRAM region to put the DWC3 DMA objects in, so they are ordinary
 * .bss and the whole image is one loadable blob.
 */

ENTRY(_start)

MEMORY
{
    ESRAM (rwx) : ORIGIN = 0xFE020000, LENGTH = 64K
}

STACK_SIZE = 4K;

SECTIONS
{
    /* 32 zero bytes; tools/a3image.py patches the real header in afterwards.
     * The extra 32 bytes are a guard: the BootROM hands over with whatever SP
     * its own loader left in place, and a single push below the entry point
     * would land on the header's CRC word at +0x1c -- the one word whose
     * corruption explains A3 error 3. With _start at +0x40 that write falls
     * into the guard instead. */
    .a3hdr : ALIGN(4)
    {
        KEEP(*(.a3hdr))
        . += 32;
    } > ESRAM

    .text : ALIGN(4)
    {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > ESRAM

    .rodata : ALIGN(8)
    {
        *(.rodata .rodata.*)
    } > ESRAM

    .data : ALIGN(8)
    {
        *(.data .data.*)
    } > ESRAM

    .bss (NOLOAD) : ALIGN(16)
    {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(16);
        __bss_end = .;
    } > ESRAM

    .stack (NOLOAD) : ALIGN(16)
    {
        . += STACK_SIZE;
        __stack_top = .;
    } > ESRAM

    ASSERT(ADDR(.text) == ORIGIN(ESRAM) + 64, "A3 header + guard must be exactly 64 bytes")
    ASSERT(. <= ORIGIN(ESRAM) + LENGTH(ESRAM), "payload overflows the 64K ESRAM window")

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
        *(.ARM.attributes)
    }
}

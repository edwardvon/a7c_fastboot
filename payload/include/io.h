#ifndef A7C_IO_H
#define A7C_IO_H

typedef unsigned char      u8;
typedef signed char        s8;
typedef unsigned short     u16;
typedef signed short       s16;
typedef unsigned int       u32;
typedef signed int         s32;
typedef unsigned long long u64;
typedef signed long long   s64;

#define BIT(n)  (1u << (n))

static inline void write32(u32 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  read32(u32 addr)           { return *(volatile u32 *)addr; }
static inline void write16(u32 addr, u16 val) { *(volatile u16 *)addr = val; }
static inline u16  read16(u32 addr)           { return *(volatile u16 *)addr; }
static inline void write8(u32 addr, u8 val)   { *(volatile u8 *)addr = val; }
static inline u8   read8(u32 addr)            { return *(volatile u8 *)addr; }

static inline void setbits32(u32 addr, u32 bits) { write32(addr, read32(addr) | bits); }
static inline void clrbits32(u32 addr, u32 bits) { write32(addr, read32(addr) & ~bits); }
static inline void clrsetbits32(u32 addr, u32 clr, u32 set)
{
    write32(addr, (read32(addr) & ~clr) | set);
}

/* ARMv7-A: bare `dsb` is DSB SY; the AArch64 `dsb sy` spelling is not valid
 * here. */
static inline void dsb(void) { __asm__ volatile("dsb" ::: "memory"); }
static inline void isb(void) { __asm__ volatile("isb" ::: "memory"); }

#define dma_wmb() dsb()

void udelay(u32 us);
void mdelay(u32 ms);

#endif /* A7C_IO_H */

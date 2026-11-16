/*

A7C (ILCE-7C) bootrom UART transmitter.

The host side is unmodified host_control.py: same 3-byte packet framing, same
CMD_PING/CARRIER/BLOB/BAUD/UART numbers, same "uart_boot 1" version string.
CMD_TIMING (0x07) is the only addition; it retunes the waveform at runtime.

Waveform (reverse engineered from the dumped A7C bootrom, 2026-10-21):

  * One PIO cycle is half a "quantum".  The quantum is the UART bit period.
    Compile-time default is 56.25 us (17778 baud), but every host-controlled
    run sends CMD_TIMING; the hardware-verified value is q = 20833 ns, i.e.
    20.833 us/bit = 48001 baud.

  * Tick scale: the ROM samples bit k of a byte at ROM-table offsets
    [0xFFFF2F84 + 4k] (strap bit5 = 1) = 0x1c, 0x70, 0xc4, ... -- a first
    sample at 28 ticks and a constant 84-tick step.  Since that step is one
    bit period, one tick = q/84 = 248 ns at q = 20833.  All ROM thresholds
    below are timer ticks at that scale.

  * Sync accept window: an edge is counted only if it lands 909..1184 ticks
    (225.4..293.6 us) after the previous accepted edge.  Earlier => status
    0x3003, which is non-fatal but zeroes the consecutive-edge counter, so a
    run of them never reaches 60 and the 1.45 s timer A expires as 0x2004
    (1 long + 4 short).  Later than 1184 ticks (level unchanged) => 0x3002 =>
    0x3001, i.e. a normal boot.

  * Carrier: 4 PIO cycles low + 4 cycles high = 2 quanta per half period
    (41.7 us at q = 20833).  That is far below 909 ticks, so every carrier
    edge is "too early" (0x3003): it keeps the sync FSM parked with the
    counter at 0 instead of counting edges or timing out.

  * Preamble: a train of fast half-period cells (one quantum each) followed by
    60 slow blocks of `slow_q` quanta each, alternating low/high starting low.
    The fast train ends high; its high->low edge into block 0 is only one
    quantum after the previous edge, so it is rejected as "too early" and
    resets the counter.  Then block 0->1, 1->2, ... 58->59 raise the counter
    to 59, and block 59 high -> start bit low is the 60th counted edge, which
    makes the bootrom start parsing the frame immediately.

    `slow_q * q` is therefore the parameter that has to fit the window:
      slow_q = 11 -> 229.2 us, only 1.65 % above the floor: *fails* (0x2004)
      slow_q = 12 -> 250.0 us: verified good on hardware 2026-10-21
      slow_q = 13 -> 270.8 us: still inside, 8 % below the ceiling
      slow_q = 14 -> 291.7 us: 0.65 % below the ceiling, do not use
    slow_q never changes the bit period, only the block length.

  * Frame: start bit low, 8 data bits LSB first, odd parity, stop bit high,
    11 quanta per byte.  After every byte (including the last one) the bootrom
    requires the next start edge 830..1802 ticks (206..447 us) after the
    previous one -- a much wider window than the sync window, so a 250 us
    frame passes with ample margin.  The padding after the last stop bit
    starts low and then alternates.

  * Probe mode (CMD_PROBE): a free running square wave; its half period has to
    sit inside the same 225.4..293.6 us sync window for every edge to be
    counted.  NOTE: PROBE_HALF_US is still 675 us, which predates the tick
    scale above and lies outside that window -- pass an explicit period
    (~250 us) rather than relying on the default.

*/

#include <string.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/structs/sio.h>
#include <hardware/timer.h>

#include "a7c_serialboot.pio.h"

#define MULTI_TX  28
#define MULTI_RX  29
#define MULTI_RST 27  /* not wired on the A7C; kept for pin-map compatibility */

#define LINK_BAUD 48000u

#define RESET_PULSE_US 10u
#define STARVE_MS      1200u

#ifndef UART_TX
// Serial1 falls back to its variant default when its tx pin is left unset, which must not be the multi-port line.
static_assert(MULTI_TX != PIN_SERIAL1_TX, "MULTI_TX collides with the default Serial1 tx pin");
#endif

#define VERSION "uart_boot 1"

enum {
  CMD_PING    = 0x01,
  CMD_RESET   = 0x02,
  CMD_CARRIER = 0x03,
  CMD_BLOB    = 0x04,
  CMD_BAUD    = 0x05,
  CMD_UART    = 0x06,
  CMD_TIMING  = 0x07,
  CMD_PROBE   = 0x08,
  CMD_PROBE_STAT = 0x09,
  CMD_SNIFF   = 0x0A,
  /* A6700-style hardware trigger: sample the line instead of guessing a hold
     time.  Numbers differ from the A6700 build (where they are 0x07/0x08)
     because CMD_TIMING/CMD_PROBE already own those here. */
  CMD_SENSE   = 0x0B,
  CMD_PULL    = 0x0C,

  EVT_ACK  = 0x81,
  EVT_ERR  = 0x82,
  EVT_UART = 0x83,
  EVT_SNIFF = 0x84,
};

enum {
  MODE_IDLE,
  MODE_CARRIER,
  MODE_UART,
  MODE_PROBE,
};

#define MAX_RECORD   4100u
#define MAX_PKT      MAX_RECORD
#define UART_CHUNK   512u
#define BITS_PER_BYTE 11u

/* Sniffer: PIO samples the data line once per instruction and DMA fills this
   buffer with 32 samples per word.  16384 words = 64 KiB = 524288 samples. */
#define SNIFF_WORDS  16384u
#define MAX_RUNS     1024u
#define SNIFF_MAX_MS 5000u

/* one bit = one quantum = two PIO cycles.  The range is deliberately wide
   because the accept window is narrow and has to be hit by sweeping CMD_TIMING
   on hardware: it is 225.4..293.6 us of edge spacing (909..1184 ROM ticks at
   the verified 248 ns/tick), which is a slow block of 11..13 quanta at
   q = 20833 ns. */
#define QUANTUM_MIN_NS 4000u
#define QUANTUM_MAX_NS 200000u

#define SLOW_BLOCKS  60u
#define MAX_SLOW_Q   255u
#define MAX_FAST_Q   255u
#define MAX_PREAMBLE_BITS (MAX_FAST_Q + SLOW_BLOCKS * MAX_SLOW_Q)

#define MAX_BITWORDS (1u + (MAX_RECORD * BITS_PER_BYTE + MAX_PREAMBLE_BITS + 31u) / 32u)

static uint8_t  rx_buf[MAX_PKT];
static uint8_t  uart_buf[UART_CHUNK];
static uint32_t bitbuf[MAX_BITWORDS];

static uint8_t  mode;
static bool     blob_pending;
static uint32_t last_blob_ms;

/* waveform knobs, tuned by CMD_TIMING */
static uint32_t quantum_ns = 56250u;  /* compile-time default only; the host
                                         sends CMD_TIMING 20833 on every run */
static uint32_t slow_q     = 12u;     /* verified 2026-10-21: 250 us per slow
                                         block, inside the 225.4..293.6 us
                                         sync window (11 = 229.2 us fails) */
static uint32_t fast_q     = 129u;    /* odd, so the fast train ends high */

/* 0 = derive the carrier square wave from quantum_ns (2 * quantum_ns half
   period); probe mode overrides it with an explicit half period in ns. */
static uint32_t carrier_half_ns = 0;

#define PROBE_HALF_US 675u   /* legacy value: outside the 225.4..293.6 us sync
                                window, so pass an explicit ~250 us period */

/* Probe self-check: measure the real edge spacing on MULTI_TX with a GPIO
   interrupt, so the waveform can be validated without the camera. */
static volatile uint32_t edge_cnt;
static volatile uint32_t edge_min_us;
static volatile uint32_t edge_max_us;
static volatile uint32_t edge_last_us;
static volatile uint32_t edge_lvl;

static void probe_edge_isr(uint gpio, uint32_t events) {
  (void)gpio;
  (void)events;
  uint32_t now = time_us_32();
  uint32_t d   = now - edge_last_us;
  edge_last_us = now;
  edge_lvl     = gpio_get(MULTI_TX);
  if (edge_cnt) {
    if (d < edge_min_us) edge_min_us = d;
    if (d > edge_max_us) edge_max_us = d;
  }
  edge_cnt++;
}

static PIO      pio = pio0;
static int      pio_sm = -1;
static uint     pio_off;
static int      dma_chan = -1;

static uint32_t sniff_buf[SNIFF_WORDS];
static uint16_t sniff_runs[MAX_RUNS];
static uint8_t  sniff_out[12u + MAX_RUNS * 2u];
static int      sniff_sm = -1;
static uint     sniff_off;
static int      sniff_dma = -1;
static uint32_t sniff_words;

static uint8_t  rx_hdr_got;
static uint8_t  rx_type;
static uint16_t rx_len;
static uint16_t rx_body_got;
static uint16_t rx_skip;

static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void send_pkt(uint8_t type, const void *data, uint16_t len) {
  uint8_t hdr[3] = { type, (uint8_t)len, (uint8_t)(len >> 8) };
  Serial.write(hdr, sizeof(hdr));
  if (len) {
    Serial.write((const uint8_t *)data, len);
  }
}

static void send_text(uint8_t type, const char *s) {
  send_pkt(type, s, (uint16_t)strlen(s));
}

static void pulse_reset(void) {
  sio_hw->gpio_clr    = 1u << MULTI_RST;
  sio_hw->gpio_oe_set = 1u << MULTI_RST;
  busy_wait_us_32(RESET_PULSE_US);
  sio_hw->gpio_oe_clr = 1u << MULTI_RST;
}

static void carrier_start(void) {
#ifdef OPEN_DRAIN
  pio_sm_config c = a7c_serialboot_od_program_get_default_config(pio_off);
#else
  pio_sm_config c = a7c_serialboot_pp_program_get_default_config(pio_off);
#endif
  sm_config_set_set_pins(&c, MULTI_TX, 1);
  sm_config_set_out_pins(&c, MULTI_TX, 1);
  sm_config_set_out_shift(&c, true, true, 32);
  /* one instruction = a quarter of a carrier half period */
  uint32_t half_ns = carrier_half_ns ? carrier_half_ns : quantum_ns * 2u;
  sm_config_set_clkdiv(&c, (float)half_ns * (float)clock_get_hz(clk_sys) / 4.0e9f);
  sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);

  pio_gpio_init(pio, MULTI_TX);
#ifdef OPEN_DRAIN
  pio_sm_set_pins_with_mask(pio, pio_sm, 0, 1u << MULTI_TX);
  pio_sm_set_consecutive_pindirs(pio, pio_sm, MULTI_TX, 1, false);
#else
  /* latch low first, then enable the driver (same order as uart_boot_probe:
     digitalWrite(LOW) before pinMode(OUTPUT)) so the switch-over glitch is
     not taken as the first edge by the ROM */
  pio_sm_set_pins_with_mask(pio, pio_sm, 0, 1u << MULTI_TX);
  pio_sm_set_consecutive_pindirs(pio, pio_sm, MULTI_TX, 1, true);
#endif
  pio_sm_init(pio, pio_sm, pio_off, &c);
  pio_sm_clear_fifos(pio, pio_sm);
  pio_interrupt_clear(pio, 0);
  pio_sm_set_enabled(pio, pio_sm, true);
}

static void carrier_stop(void) {
  gpio_set_irq_enabled(MULTI_TX, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
  if (dma_channel_is_busy(dma_chan)) {
    dma_channel_abort(dma_chan);
  }
  pio_sm_set_enabled(pio, pio_sm, false);
  pinMode(MULTI_TX, INPUT);
  blob_pending = false;
}

static inline void put_high(uint32_t n) {
  bitbuf[1u + (n >> 5)] |= 1u << (n & 31u);
}

/* Returns the number of 32-bit words to push (including the length word), or 0
   if the record does not fit. */
static uint32_t build_bits(const uint8_t *rec, uint32_t len) {
  uint32_t total = fast_q + SLOW_BLOCKS * slow_q + len * BITS_PER_BYTE;
  uint32_t nbits = (total + 31u) & ~31u;
  uint32_t nwords = nbits / 32u;

  if (nwords + 1u > MAX_BITWORDS) {
    return 0;
  }

  memset(bitbuf + 1, 0, nwords * sizeof(uint32_t));
  bitbuf[0] = nbits - 1u;

  uint32_t n = 0;

  /* fast train: one quantum per cell, high first; fast_q is odd so it ends high */
  for (uint32_t i = 0; i < fast_q; i++, n++) {
    if (!(i & 1u)) {
      put_high(n);
    }
  }

  /* slow blocks: even blocks low (already zero), odd blocks high */
  for (uint32_t b = 0; b < SLOW_BLOCKS; b++) {
    if (b & 1u) {
      for (uint32_t k = 0; k < slow_q; k++, n++) {
        put_high(n);
      }
    } else {
      n += slow_q;
    }
  }

  for (uint32_t i = 0; i < len; i++) {
    uint8_t v = rec[i];
    uint32_t frame = 1u << 10;
    uint32_t x = 0;
    for (uint32_t b = 0; b < 8; b++) {
      uint32_t bit = (v >> b) & 1u;
      x ^= bit;
      frame |= bit << (1u + b);
    }
    frame |= (x ? 0u : 1u) << 9;

    for (uint32_t b = 0; b < BITS_PER_BYTE; b++, n++) {
      if (frame & (1u << b)) {
        put_high(n);
      }
    }
  }

  /* pad to a whole number of words: alternate, starting low, so no level is
     ever held longer than one quantum (a long high would trip 0x3002) */
  for (uint32_t p = 0; n < nbits; p++, n++) {
    if (p & 1u) {
      put_high(n);
    }
  }

#ifdef OPEN_DRAIN
  // the state machine drives pindirs, so the wire level is the complement of each bit
  for (uint32_t w = 1; w <= nwords; w++) {
    bitbuf[w] = ~bitbuf[w];
  }
#endif

  return 1u + nwords;
}

static void blob_send(uint32_t nwords) {
  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, pio_get_dreq(pio, pio_sm, true));
  dma_channel_configure(dma_chan, &c, &pio->txf[pio_sm], bitbuf, nwords, true);
}

/* ---------------- sniffer: capture whatever the camera drives on D- -------- */

static void sniff_begin(uint32_t duration_ms, uint32_t period_ns) {
  pio_sm_config c = a7c_sniff_program_get_default_config(sniff_off);
  sm_config_set_in_pins(&c, MULTI_TX);
  sm_config_set_in_shift(&c, true, true, 32);   /* shift right: first sample -> bit 0 */
  /* system clock is 200 MHz here, not 125: derive the divider from the real rate */
  sm_config_set_clkdiv(&c, (float)period_ns * (float)clock_get_hz(clk_sys) / 1.0e9f);

  pio_gpio_init(pio, MULTI_TX);
  gpio_set_pulls(MULTI_TX, false, false);
  pio_sm_init(pio, sniff_sm, sniff_off, &c);
  pio_sm_clear_fifos(pio, sniff_sm);

  uint32_t words = (uint32_t)(((uint64_t)duration_ms * 1000000ull) /
                              ((uint64_t)period_ns * 32ull));
  if (words > SNIFF_WORDS) {
    words = SNIFF_WORDS;
  }
  if (words == 0) {
    words = 1;
  }
  sniff_words = words;

  dma_channel_config dc = dma_channel_get_default_config(sniff_dma);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, false);
  channel_config_set_write_increment(&dc, true);
  channel_config_set_dreq(&dc, pio_get_dreq(pio, sniff_sm, false));
  dma_channel_configure(sniff_dma, &dc, sniff_buf, &pio->rxf[sniff_sm], words, true);

  pio_sm_set_enabled(pio, sniff_sm, true);
}

static uint32_t sniff_end(void) {
  dma_channel_abort(sniff_dma);
  uint32_t left = dma_channel_hw_addr(sniff_dma)->transfer_count;
  pio_sm_set_enabled(pio, sniff_sm, false);
  return (left <= sniff_words) ? (sniff_words - left) : sniff_words;
}

/* Turn the raw samples into level runs and hand them to the host.  Each run is
   one uint16 of microseconds; runs alternate in level, starting at the level of
   the very first sample. */
static void sniff_report(uint32_t nwords, uint32_t period_ns) {
  uint32_t nsamp = nwords * 32u;
  uint32_t nrun  = 0;
  uint32_t run   = 1;
  uint8_t  lvl   = (uint8_t)(sniff_buf[0] & 1u);
  bool     trunc = false;

  for (uint32_t s = 1; s < nsamp; s++) {
    uint8_t b = (uint8_t)((sniff_buf[s >> 5] >> (s & 31u)) & 1u);
    if (b == lvl) {
      run++;
      continue;
    }
    if (nrun >= MAX_RUNS) {
      trunc = true;
      break;
    }
    uint64_t us = ((uint64_t)run * period_ns) / 1000ull;
    sniff_runs[nrun++] = (uint16_t)(us > 65535ull ? 65535ull : us);
    lvl = b;
    run = 1;
  }
  if (!trunc && nrun < MAX_RUNS) {
    uint64_t us = ((uint64_t)run * period_ns) / 1000ull;
    sniff_runs[nrun++] = (uint16_t)(us > 65535ull ? 65535ull : us);
  }

  uint32_t total_us = (uint32_t)(((uint64_t)nsamp * period_ns) / 1000ull);
  sniff_out[0]  = (uint8_t)total_us;
  sniff_out[1]  = (uint8_t)(total_us >> 8);
  sniff_out[2]  = (uint8_t)(total_us >> 16);
  sniff_out[3]  = (uint8_t)(total_us >> 24);
  sniff_out[4]  = (uint8_t)period_ns;
  sniff_out[5]  = (uint8_t)(period_ns >> 8);
  sniff_out[6]  = (uint8_t)(period_ns >> 16);
  sniff_out[7]  = (uint8_t)(period_ns >> 24);
  sniff_out[8]  = (uint8_t)(sniff_buf[0] & 1u);
  sniff_out[9]  = trunc ? 1u : 0u;
  sniff_out[10] = (uint8_t)nrun;
  sniff_out[11] = (uint8_t)(nrun >> 8);

  for (uint32_t i = 0; i < nrun; i++) {
    sniff_out[12u + i * 2u] = (uint8_t)sniff_runs[i];
    sniff_out[13u + i * 2u] = (uint8_t)(sniff_runs[i] >> 8);
  }
  send_pkt(EVT_SNIFF, sniff_out, (uint16_t)(12u + nrun * 2u));
}

static void uart_start(uint32_t baud) {
  Serial1.setFIFOSize(1024);
#ifdef UART_TX
  Serial1.setTX(MULTI_TX);
#endif
  Serial1.setRX(MULTI_RX);
  Serial1.begin(baud);
}

static void uart_stop(void) {
  Serial1.end();
  pinMode(MULTI_TX, INPUT);
}

static void mode_set(uint8_t m, uint32_t baud) {
  switch (mode) {
  case MODE_CARRIER:
  case MODE_PROBE:   carrier_stop(); break;
  case MODE_UART:    uart_stop();    break;
  }
  switch (m) {
  case MODE_CARRIER: carrier_start();  break;
  case MODE_UART:    uart_start(baud); break;
  }
  mode = m;
}

static bool timing_valid(uint32_t q, uint32_t slow, uint32_t fast) {
  if (q < QUANTUM_MIN_NS || q > QUANTUM_MAX_NS) {
    return false;
  }
  if (slow < 1u || slow > MAX_SLOW_Q) {
    return false;
  }
  uint32_t block = slow * q;
  if (block < 20000u || block > 3000000u) {
    return false;
  }
  if (fast < 1u || fast > MAX_FAST_Q || !(fast & 1u)) {
    return false;
  }
  return true;
}

static void handle_pkt(uint8_t type, uint8_t *d, uint16_t len) {
  switch (type) {
  case CMD_PING:
    send_text(EVT_ACK, VERSION);
    break;

  case CMD_RESET:
    /* The A7C multi-port has no reset line on this adapter.  Acknowledge
       instead of pulsing so the stock host script's default path keeps
       working; nothing is driven. */
    send_text(EVT_ACK, "no reset pin");
    break;

  case CMD_CARRIER:
    if (len < 1) {
      send_text(EVT_ERR, "short CMD_CARRIER");
      break;
    }
    if (d[0]) {
      carrier_half_ns = 0;
      if (mode != MODE_CARRIER) {
        mode_set(MODE_CARRIER, 0);
      }
      last_blob_ms = millis();
    } else {
      mode_set(MODE_IDLE, 0);
    }
    send_pkt(EVT_ACK, NULL, 0);
    break;

  case CMD_BLOB: {
    if (mode != MODE_CARRIER) {
      send_text(EVT_ERR, "carrier off");
      break;
    }
    if (blob_pending) {
      send_text(EVT_ERR, "slot busy");
      break;
    }
    if (len > MAX_RECORD) {
      send_text(EVT_ERR, "record too long");
      break;
    }
    uint32_t nwords = build_bits(d, len);
    if (!nwords) {
      send_text(EVT_ERR, "record too long");
      break;
    }
    blob_send(nwords);
    blob_pending = true;
    last_blob_ms = millis();
    break;
  }

  case CMD_BAUD: {
    if (len < 4) {
      send_text(EVT_ERR, "short CMD_BAUD");
      break;
    }
    uint32_t baud = le32(d);
    if (baud && mode == MODE_CARRIER) {
      send_text(EVT_ERR, "carrier on");
      break;
    }
    mode_set(baud ? MODE_UART : MODE_IDLE, baud);
    send_pkt(EVT_ACK, NULL, 0);
    break;
  }

  case CMD_UART:
    if (mode != MODE_UART) {
      send_text(EVT_ERR, "uart not up");
      break;
    }
#ifdef UART_TX
    Serial1.write(d, len);
#else
    send_text(EVT_ERR, "uart tx disabled");
#endif
    break;

  case CMD_TIMING: {
    if (len < 6) {
      send_text(EVT_ERR, "short CMD_TIMING");
      break;
    }
    if (blob_pending) {
      send_text(EVT_ERR, "slot busy");
      break;
    }
    uint32_t q     = le32(d);
    uint32_t slow  = d[4];
    uint32_t fastq = d[5];
    if (!timing_valid(q, slow, fastq)) {
      send_text(EVT_ERR, "bad timing");
      break;
    }
    quantum_ns = q;
    slow_q     = slow;
    fast_q     = fastq;
    if (mode == MODE_CARRIER) {
      carrier_stop();
      carrier_start();
      last_blob_ms = millis();
    }
    uint8_t echo[6] = {
      (uint8_t)q, (uint8_t)(q >> 8), (uint8_t)(q >> 16), (uint8_t)(q >> 24),
      (uint8_t)slow, (uint8_t)fastq
    };
    send_pkt(EVT_ACK, echo, sizeof(echo));
    break;
  }

  case CMD_PROBE: {
    /* Free running square wave, sending only.  Optional 2-byte half period
       in microseconds; defaults to PROBE_HALF_US. */
    if (blob_pending) {
      send_text(EVT_ERR, "slot busy");
      break;
    }
    uint32_t half_us = PROBE_HALF_US;
    if (len >= 2) {
      half_us = (uint32_t)d[0] | ((uint32_t)d[1] << 8);
    }
    if (half_us < 200u || half_us > 2000u) {
      send_text(EVT_ERR, "bad probe period");
      break;
    }
    if (mode != MODE_IDLE) {
      mode_set(MODE_IDLE, 0);
    }
    carrier_half_ns = half_us * 1000u;
    carrier_start();
    mode = MODE_PROBE;

    /* start measuring what actually appears on the pin */
    edge_cnt     = 0;
    edge_min_us  = 0xFFFFFFFFu;
    edge_max_us  = 0;
    edge_last_us = time_us_32();
    edge_lvl     = (uint32_t)gpio_get(MULTI_TX);
    gpio_set_irq_enabled_with_callback(MULTI_TX,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &probe_edge_isr);

    uint8_t echo[2] = { (uint8_t)half_us, (uint8_t)(half_us >> 8) };
    send_pkt(EVT_ACK, echo, sizeof(echo));
    break;
  }

  case CMD_PROBE_STAT: {
    uint32_t v[4] = { edge_cnt, edge_min_us, edge_max_us, edge_last_us };
    uint8_t out[18];
    for (uint32_t i = 0; i < 4; i++) {
      out[i * 4 + 0] = (uint8_t)v[i];
      out[i * 4 + 1] = (uint8_t)(v[i] >> 8);
      out[i * 4 + 2] = (uint8_t)(v[i] >> 16);
      out[i * 4 + 3] = (uint8_t)(v[i] >> 24);
    }
    out[16] = (uint8_t)edge_lvl;
    out[17] = mode;
    send_pkt(EVT_ACK, out, sizeof(out));
    break;
  }

  case CMD_SNIFF: {
    /* Capture the data line instead of driving it.  d = u16 duration_ms,
       optional u16 sample period in us (default 20).  When the probe square
       wave is still running the line stays driven, which is the loopback
       self-test path; otherwise the pin is released and left high-Z. */
    if (len < 2) {
      send_text(EVT_ERR, "short CMD_SNIFF");
      break;
    }
    uint32_t duration_ms = (uint32_t)d[0] | ((uint32_t)d[1] << 8);
    uint32_t period_us   = 20u;
    if (len >= 4) {
      period_us = (uint32_t)d[2] | ((uint32_t)d[3] << 8);
    }
    if (duration_ms < 1u || duration_ms > SNIFF_MAX_MS) {
      send_text(EVT_ERR, "bad sniff time");
      break;
    }
    if (period_us < 2u || period_us > 500u) {
      send_text(EVT_ERR, "bad sniff period");
      break;
    }
    if (blob_pending) {
      send_text(EVT_ERR, "slot busy");
      break;
    }

    if (mode != MODE_PROBE) {
      if (mode != MODE_IDLE) {
        mode_set(MODE_IDLE, 0);
      }
      pinMode(MULTI_TX, INPUT);
    }

    sniff_begin(duration_ms, period_us * 1000u);
    uint8_t echo[4] = {
      (uint8_t)duration_ms, (uint8_t)(duration_ms >> 8),
      (uint8_t)period_us,   (uint8_t)(period_us >> 8)
    };
    send_pkt(EVT_ACK, echo, sizeof(echo));

    busy_wait_ms(duration_ms);
    sniff_report(sniff_end(), period_us * 1000u);
    break;
  }

  case CMD_SENSE: {
    /* Sample the existing line for 2 ms and report whether it ever went high.
       Reads only: the PIO state machine and the pin direction are untouched,
       so this works while the open-drain carrier is running. */
    uint8_t high = 0;
    const uint32_t start = time_us_32();
    while ((uint32_t)(time_us_32() - start) < 2000u) {
      if (gpio_get(MULTI_TX)) {
        high = 1;
      }
    }
    send_pkt(EVT_ACK, &high, 1);
    break;
  }

  case CMD_PULL: {
    /* Bench pull-up, only while idle (the PIO owns the pad otherwise). */
    if (len < 1) {
      send_text(EVT_ERR, "short CMD_PULL");
      break;
    }
    if (mode != MODE_IDLE) {
      send_text(EVT_ERR, "pull requires idle");
      break;
    }
    gpio_set_pulls(MULTI_TX, d[0] != 0, false);
    send_pkt(EVT_ACK, NULL, 0);
    break;
  }

  default:
    send_text(EVT_ERR, "unknown command");
    break;
  }
}

static void poll_host(void) {
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) {
      break;
    }

    if (rx_skip) {
      rx_skip--;
      continue;
    }

    if (rx_hdr_got < 3) {
      if (rx_hdr_got == 0) {
        rx_type = (uint8_t)c;
      } else if (rx_hdr_got == 1) {
        rx_len = (uint16_t)c;
      } else {
        rx_len |= (uint16_t)c << 8;
      }
      rx_hdr_got++;
      if (rx_hdr_got == 3) {
        rx_body_got = 0;
        if (rx_len > MAX_PKT) {
          send_text(EVT_ERR, "packet too long");
          rx_skip    = rx_len;
          rx_hdr_got = 0;
        } else if (rx_len == 0) {
          handle_pkt(rx_type, rx_buf, 0);
          rx_hdr_got = 0;
        }
      }
    } else {
      rx_buf[rx_body_got++] = (uint8_t)c;
      if (rx_body_got == rx_len) {
        handle_pkt(rx_type, rx_buf, rx_len);
        rx_hdr_got = 0;
      }
    }
  }
}

void setup(void) {
  pinMode(MULTI_TX,  INPUT);
  pinMode(MULTI_RX,  INPUT);
  pinMode(MULTI_RST, INPUT);

  /* No internal pull-up/down on the data line: the open-drain carrier must let
     the camera decide the idle level, and CMD_SENSE has to see that level. */
  gpio_disable_pulls(MULTI_TX);

  pio_sm   = pio_claim_unused_sm(pio, true);
#ifdef OPEN_DRAIN
  pio_off  = pio_add_program(pio, &a7c_serialboot_od_program);
#else
  pio_off  = pio_add_program(pio, &a7c_serialboot_pp_program);
#endif
  dma_chan = dma_claim_unused_channel(true);

  /* receive side: a second state machine samples the data line into RAM */
  sniff_sm  = pio_claim_unused_sm(pio, true);
  sniff_off = pio_add_program(pio, &a7c_sniff_program);
  sniff_dma = dma_claim_unused_channel(true);

  Serial.begin(115200);
}

void loop(void) {
  poll_host();

  if (blob_pending && pio_interrupt_get(pio, 0)) {
    pio_interrupt_clear(pio, 0);
    blob_pending = false;
    /* A record can take well over STARVE_MS to play out, so the idle window
       has to start when the waveform ends, not when it started.  Otherwise
       the very next pass declares "starved" and kills the carrier. */
    last_blob_ms = millis();
    send_pkt(EVT_ACK, NULL, 0);
  }

  if (mode == MODE_CARRIER && !blob_pending &&
      (uint32_t)(millis() - last_blob_ms) > STARVE_MS) {
    mode_set(MODE_IDLE, 0);
    send_text(EVT_ERR, "starved");
  }

  if (mode == MODE_UART) {
    int n = Serial1.available();
    if (n > 0) {
      if (n > (int)sizeof(uart_buf)) {
        n = (int)sizeof(uart_buf);
      }
      for (int i = 0; i < n; i++) {
        uart_buf[i] = (uint8_t)Serial1.read();
      }
      send_pkt(EVT_UART, uart_buf, (uint16_t)n);
    }
  }
}

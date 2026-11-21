/* NUCLEO-F103RB: replace Core/Src/main.c in the existing project.
 * Use with the accompanying STM2/main.c and uno/uno.ino summary format.
 * Existing wiring: USART1 PA9/PA10 -> STM2, USART3 PB10/PB11 -> UNO;
 * FSR PA0/PA1/PA4, pedal PC7, rack PC8/PC9, servos PA6/PA7.
 */
/* Shared wire format: A5 5A type id(LE32) a..g(LE32) CRC16-CCITT(LE16).
- 37 bytes; all peers must use this revision. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>



/* ===== STM3 ADC comparison: change ONLY TEST_ADC_DMA between runs ===== */
#ifndef TEST_ADC_DMA
#define TEST_ADC_DMA 0             /* 0=Polling, 1=DMA */
#endif
#define TEST_ADC_PERIOD_MS 5u
#define TEST_WARMUP_MS 2000u
#define TEST_DURATION_MS 10000u
#define TEST_QUIET 1               /* suppress U2 logs inside measurement */
#define TEST_TRACE 1               /* PC0=acquisition, PC1=request, PC2=rack */
static bool test_quiet;

#define FRAME_N 37u
#define T_RACK 1u
#define T_CONTROL 2u
#define T_MEASURE 3u
#define T_UI 4u
#define T_KEY 5u
#define F_ACTIVE 1u
#define F_ASSIST 2u
#define F_RACKED 4u
#define F_UNRACKED 8u
#define F_PARTIAL 16u
#define F_PARKED 32u
#define F_WEIGHT 64u
#define F_FSR_OK 128u
#define F_LINK 256u
#define F_SUMMARY 512u
#define F_WAIT_FINAL 1024u
#define F_MODEL_ENABLED 2048u
#define M_CAL 1u
#define M_DISTANCE 2u
#define M_ACTIVE 4u
#define M_FINAL 8u
#define M_BASE 16u
#define M_TILT 32u
#define M_FAULT 64u
#define I_WEIGHT 1u
#define I_IMU 2u
#define I_MODEL 4u
#define I_LINK 8u
#define I_FSR 16u
#define I_FINAL 32u
#define I_DISTANCE 64u
#define I_ASSIST 128u
#define I_BASE 256u
#define REC_NONE 0u
#define REC_KEEP 1u
#define REC_REDUCE 2u
#define REC_INCREASE 3u
#define REASON_PEDAL 1u
#define REASON_FSR1 2u
#define U32(v) ((uint32_t)(v))


typedef struct {
  uint8_t type;
  uint32_t id, a, b, c, d, e, f, g;
} Packet;


typedef struct {
  uint8_t b[FRAME_N];
  unsigned n;
} Parser;


static uint16_t crc16(const uint8_t * p, unsigned n) {
  uint16_t c = 0xffff;
  while (n--) {
      c ^= (uint16_t) * p++ << 8;
      for (unsigned j = 0; j < 8; j++) c = (uint16_t)((c & 0x8000u) ? ((uint32_t) c << 1) ^ 0x1021u : (uint32_t) c << 1);
  }
  return c;
}
static void put32(uint8_t * p, uint32_t x) {
  for (unsigned j = 0; j < 4; j++) p[j] = (uint8_t)(x >> (8 * j));
}
static uint32_t get32(const uint8_t * p) {
  return U32(p[0]) | (U32(p[1]) << 8) | (U32(p[2]) << 16) | (U32(p[3]) << 24);
}
static void encode(const Packet * p, uint8_t b[FRAME_N]) {
  b[0] = 0xa5;
  b[1] = 0x5a;
  b[2] = p->type;
  uint32_t w[8] = { p->id, p->a, p->b, p->c, p->d, p->e, p->f, p->g };
  for (unsigned j = 0; j < 8; j++) put32(b + 3 + 4 * j, w[j]);
  uint16_t c = crc16(b + 2, 33);
  b[35] = (uint8_t) c;
  b[36] = (uint8_t)(c >> 8);
}
static bool parse(Parser * s, uint8_t c, Packet * p) {
  if (!s->n) {
      if (c == 0xa5) s->b[s->n++] = c;
      return false;
  }
  if (s->n == 1 && c != 0x5a) {
      s->n = (c == 0xa5) ? 1 : 0;
      return false;
  }
  s->b[s->n++] = c;
  if (s->n < FRAME_N) return false;
  uint8_t * b = s->b;
  s->n = 0;
  if (crc16(b + 2, 33) != ((uint16_t) b[35] | (uint16_t) b[36] << 8)) {
      if (c == 0xa5) {
          b[0] = c;
          s->n = 1;
      }
      return false;
  }
  p->type = b[2];
  p->id = get32(b + 3);
  p->a = get32(b + 7);
  p->b = get32(b + 11);
  p->c = get32(b + 15);
  p->d = get32(b + 19);
  p->e = get32(b + 23);
  p->f = get32(b + 27);
  p->g = get32(b + 31);
  return true;
}


#define F_CONTACT 4096u /* both rack contacts raw pressed: suspend new rep completion */
#define F_M_READY 8192u
#define F_M_TILT 16384u
#define F_M_BASE 32768u
#define F_M_WARN 65536u
#define M_WARN 128u
#define M_REP_VALID 256u
/* V2 calibration transaction; all four boards must be updated together. */
#define T_CAL_STATUS 6u
#define F_CAL_BUSY (1u << 17)
#define F_CAL_FAIL (1u << 18)
#define F_CAL_REQUIRED (1u << 19)
#define CAL_NONE 0u
#define CAL_BUSY 1u
#define CAL_READY 2u
#define CAL_FAIL 3u
#define I_CAL 512u
#define SUMMARY_INVALID_SHIFT 20u
#define SUMMARY_INVALID_MASK 0x3ffu


/* STM32F103RB, HSI 8 MHz, register-only. USART1/3 IRQ RX; USART2 debug TX. */
#define R(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define B(n) (1u<<(n))
#define RA 0x40010800u
#define RB 0x40010c00u
#define RC 0x40011000u
#define U1 0x40013800u
#define U3 0x40004800u
#define U2 0x40004400u
#define T4 0x40000800u
#define APB2 R(0x40021018u)
#define APB1 R(0x4002101cu)


static void pin(uint32_t port, unsigned n, unsigned cfg) {
  uint32_t a = port + (n >= 8 ? 4 : 0), s = (n % 8) * 4;
  R(a) = (R(a) & ~(15u << s)) | (cfg << s);
}
static void pin_high(uint32_t port, unsigned n) {
  R(port + 0x10) = B(n);
}
static void base_init(void) {
  R(0x40021000) |= 1;
  while (!(R(0x40021000) & 2)) {
  }
  R(0x40021004) &= ~3u;
  while (R(0x40021004) & 12u) {
  }
  R(0x40021004) &= ~((15u << 4) | (7u << 8) | (7u << 11));
  APB2 |= B(0) | B(2) | B(3) | B(4) | B(14);
  APB1 |= B(2) | B(17) | B(18);
  R(T4) = 0;
  R(T4 + 0x28) = 7999;
  R(T4 + 0x2c) = 65535;
  R(T4 + 0x14) = 1;
  R(T4 + 0x24) = 0;
  R(T4) = 1;
}
static uint32_t ms(void) {
  static uint16_t last;
  static uint32_t total;
  uint16_t cur = (uint16_t) R(T4 + 0x24);
  total += (uint16_t)(cur - last);
  last = cur;
  return total;
}


#define RING_N 256u
typedef struct {
  volatile uint8_t q[RING_N], head, tail, err;
} Ring;
static Ring r1, r3;
static void irq_rx(uint32_t u, Ring * r) {
  uint32_t s = R(u);
  if (s & (B(5) | 15u)) {
      uint8_t c = (uint8_t) R(u + 4);
      uint8_t next = (uint8_t)(r->head + 1);
      if ((s & 15u) || next == r->tail) r->err = 1;
      else if (s & B(5)) {
          r->q[r->head] = c;
          r->head = next;
      }
  }
}
void USART1_IRQHandler(void) {
  irq_rx(U1, & r1);
}
void USART3_IRQHandler(void) {
  irq_rx(U3, & r3);
}
static void uart_init(uint32_t u, unsigned baud) {
  if (u == U1) {
      pin(RA, 9, 11);
      pin(RA, 10, 4);
  }
  else if (u == U3) {
      pin(RB, 10, 11);
      pin(RB, 11, 4);
  }
  else {
      pin(RA, 2, 11);
      pin(RA, 3, 4);
  }
  R(u + 8) = (8000000u + baud / 2) / baud;
  R(u + 0xc) = B(13) | B(3) | B(2) | (u != U2 ? B(5) : 0);
  if (u == U1) R(0xe000e104) = B(5);
  if (u == U3) R(0xe000e104) = B(7);
}
static bool take(Ring * r, Parser * s, Packet * p) {
  if (r->err) {
      r->err = 0;
      s->n = 0;
      r->tail = r->head;
  }
  while (r->tail != r->head) {
      uint8_t c = r->q[r->tail];
      r->tail = (uint8_t)(r->tail + 1);
      if (parse(s, c, p)) return true;
  }
  return false;
}
static void sendp(uint32_t u, const Packet * p) {
  uint8_t b[FRAME_N];
  encode(p, b);
  for (unsigned i = 0; i < FRAME_N; i++) {
      while (!(R(u) & B(7))) {
      }
      R(u + 4) = b[i];
  }
}
static void logstr(const char * s) {
  if (test_quiet) return;
  while (* s) {
      while (!(R(U2) & B(7))) {
      }
      R(U2 + 4) = (uint8_t) * s++;
  }
}


/* STM3: rack owner, 3-channel ADC1 DMA, local assist, TWO mirrored SG90 servos. MOCK MODEL ONLY. */
#ifndef SERVO_ENABLED
#define SERVO_ENABLED 1
#endif


#define SERVO1_DEPLOY_US   1500u
#define SERVO1_RETRACT_US  2500u
#define SERVO2_DEPLOY_US   1500u
#define SERVO2_RETRACT_US   500u
#define FSR_CONFIRM_MS     3000u
#define SERVO_MIN_US        500u
#define SERVO_MAX_US       2500u
/* 1000 us travel / 25 us per 20 ms = about 0.8 s in either direction.
 * Keep the existing PWM period and endpoint positions. */
#define SERVO_STEP_US         25u
#define SERVO_STEP_MS         20u


#ifndef FSR_CALIBRATED
#define FSR_CALIBRATED 1
#endif
#define FSR1_THRESHOLD 2000u
#define FSR2_THRESHOLD 2000u
#define FSR3_THRESHOLD 2000u
#define FSR_RELEASE_MARGIN 100u


#define ADC  0x40012400u
#define DMA  0x40020000u
#define TIM2 0x40000000u
#define TIM3 0x40000400u



/* ADC sampling stays PA0/PA1/PA4, ADC clock 4MHz, 239.5 cycles/channel.
 * DMA: one hardware scan. Polling: three single-channel conversions, so
 * each DR value is read before another channel is started. The latter
 * adds software inter-channel gaps; this is a driver comparison.
 */
static volatile uint16_t adc_buf[3], adc_latest[3];
static volatile uint16_t adc_tick;
static volatile uint32_t adc_seq;
static volatile bool adc_fault, adc_busy;
static volatile uint32_t adc_isr_us, dma_tc_count, dma_te_count;
static uint32_t adc_swstart_count, adc_timeouts;
static volatile uint32_t adc_completed_count;
static const uint16_t threshold[3] = {
  FSR1_THRESHOLD, FSR2_THRESHOLD, FSR3_THRESHOLD
};
static uint16_t fsr[3];
static uint32_t consumed, fsr_at, adc_skips, adc_cpu_us, adc_reads;
static bool fsr_seen;
static uint32_t adc_started_ms, adc_last_request_ms;

typedef struct { uint32_t n, sum, max; } TestStat;
static TestStat test_request_us, test_gap_ms, test_rack_ms;
static uint8_t test_stage; /* 0=warmup, 1=measure, 2=drain, 3=done */
static uint32_t test_boot_ms, test_begin_ms, test_end_ms;
static uint32_t test_prev_request, test_prev_rack;
static bool test_have_request, test_have_rack;
static uint32_t test_req0, test_done0, test_read0, test_skip0;
static uint32_t test_te0, test_timeout0, test_tc0;

static uint16_t test_us16(void) { return (uint16_t)R(0x40012c24u); }
static uint16_t test_elapsed(uint16_t begin) {
  return (uint16_t)(test_us16() - begin);
}
static void test_pin(unsigned n, bool high) {
#if TEST_TRACE
  R(RC + 0x10u) = high ? B(n) : B(n + 16u);
#else
  (void)n; (void)high;
#endif
}
static void test_trace_init(void) {
#if TEST_TRACE
  APB2 |= B(4);
  for (unsigned n = 0; n < 3; n++) {
    test_pin(n, false);
    pin(RC, n, 2u); /* general purpose push-pull, 2MHz */
  }
#endif
}
static void test_stat_add(TestStat *s, uint32_t v) {
  s->n++; s->sum += v;
  if (v > s->max) s->max = v;
}
static void test_request_start(uint32_t t) {
  if (test_stage != 1u) return;
  if (test_have_request) test_stat_add(&test_gap_ms, t - test_prev_request);
  test_prev_request = t;
  test_have_request = true;
}
static void test_rack_start(uint32_t t) {
  if (test_stage != 1u) return;
  if (test_have_rack) test_stat_add(&test_rack_ms, t - test_prev_rack);
  test_prev_rack = t;
  test_have_rack = true;
}
static uint32_t test_avg(TestStat s, uint32_t scale) {
  return s.n ? (uint32_t)((uint64_t)s.sum * scale / s.n) : 0u;
}
/* Begin only with no old acquisition or unconsumed result. */
static void test_before_adc(uint32_t t) {
  if (test_stage == 0u && t - test_boot_ms >= TEST_WARMUP_MS &&
      !adc_busy && adc_seq == consumed) {
    test_req0 = adc_swstart_count;
    test_done0 = adc_completed_count;
    test_read0 = adc_reads; test_skip0 = adc_skips;
    test_te0 = dma_te_count; test_tc0 = dma_tc_count;
    test_timeout0 = adc_timeouts;
    test_begin_ms = t;
    test_quiet = TEST_QUIET != 0;
    test_stage = 1u;
  }
  if (test_stage == 1u && t - test_begin_ms >= TEST_DURATION_MS) {
    test_end_ms = t;
    test_stage = 2u; /* stop new requests until the last sample is consumed */
  }
}
/* Called after adc_update and rack_task. No serial output inside the window. */
static void test_finish(void) {
  if (test_stage != 2u || adc_busy || adc_seq != consumed) return;
  test_stage = 3u;
  test_quiet = false;
  char b[256];
  snprintf(b, sizeof b,
    "\r\n[AB] mode=%s duration_ms=%lu req=%lu done=%lu read=%lu skip=%lu\r\n",
    TEST_ADC_DMA ? "DMA" : "POLLING", (unsigned long)(test_end_ms-test_begin_ms),
    (unsigned long)(adc_swstart_count-test_req0),
    (unsigned long)(adc_completed_count-test_done0),
    (unsigned long)(adc_reads-test_read0), (unsigned long)(adc_skips-test_skip0));
  logstr(b);
  snprintf(b, sizeof b,
    "[AB_TIME_US] n=%lu avg=%lu max=%lu\r\n[AB_GAP] avg_us=%lu max_ms=%lu rack_avg_us=%lu rack_max_ms=%lu\r\n",
    (unsigned long)test_request_us.n, (unsigned long)test_avg(test_request_us,1u),
    (unsigned long)test_request_us.max, (unsigned long)test_avg(test_gap_ms,1000u),
    (unsigned long)test_gap_ms.max, (unsigned long)test_avg(test_rack_ms,1000u),
    (unsigned long)test_rack_ms.max);
  logstr(b);
  snprintf(b, sizeof b, "[AB_ERR] te=%lu timeout=%lu tc=%lu pending=0\r\n",
    (unsigned long)(dma_te_count-test_te0),
    (unsigned long)(adc_timeouts-test_timeout0),
    (unsigned long)(dma_tc_count-test_tc0));
  logstr(b);
}

void DMA1_Channel1_IRQHandler(void) {
#if TEST_ADC_DMA
  uint16_t start = test_us16();
  uint32_t isr = R(DMA);
  R(DMA + 0x04u) = 15u;
  if (isr & B(3)) { /* transfer error takes priority over completion */
    adc_fault = true; dma_te_count++; adc_busy = false;
    test_pin(0u, false);
  } else if (isr & B(1)) {
    adc_latest[0] = adc_buf[0]; adc_latest[1] = adc_buf[1];
    adc_latest[2] = adc_buf[2];
    adc_tick = (uint16_t)R(T4 + 0x24u);
    adc_seq++; dma_tc_count++; adc_completed_count++;
    adc_busy = false;
    test_pin(0u, false);
  }
  adc_isr_us += test_elapsed(start);
#endif
}
static bool adc_wait(uint32_t reg, uint32_t mask, bool set) {
  uint32_t t = ms();
  while (((R(reg) & mask) != 0u) != set) {
    if ((uint32_t)(ms() - t) > 25u) return false;
  }
  return true;
}
static bool adc_init(void) {
  APB2 |= B(2) | B(9) | B(11); /* GPIOA, ADC1, TIM1 */
  R(0x40021014u) |= B(0);
  R(0x40021004u) &= ~(3u << 14); /* PCLK2/2 = 4MHz ADC */
  pin(RA,0,0); pin(RA,1,0); pin(RA,4,0);
  R(0x40012c00u)=0u; R(0x40012c28u)=7u;
  R(0x40012c2cu)=65535u; R(0x40012c14u)=1u; R(0x40012c00u)=1u;
  R(ADC + 0x08u)=0u;
  R(ADC + 0x04u)=TEST_ADC_DMA ? B(8) : 0u;
  R(ADC + 0x10u)=(7u<<0)|(7u<<3)|(7u<<12);
  R(ADC + 0x2cu)=TEST_ADC_DMA ? (2u<<20) : 0u;
  R(ADC + 0x30u)=0u;
  R(ADC + 0x34u)=TEST_ADC_DMA ? ((1u<<5)|(4u<<10)) : 0u;
  R(ADC + 0x08u)=B(0);
  uint32_t t=ms(); while (ms()-t<2u) {}
  R(ADC + 0x08u)|=B(3);
  if (!adc_wait(ADC+0x08u,B(3),false)) return false;
  R(ADC + 0x08u)|=B(2);
  if (!adc_wait(ADC+0x08u,B(2),false)) return false;
  R(DMA+0x08u)=0u; R(DMA+0x04u)=15u;
#if TEST_ADC_DMA
  R(DMA+0x0cu)=3u; R(DMA+0x10u)=ADC+0x4cu;
  R(DMA+0x14u)=(uint32_t)(uintptr_t)adc_buf;
  R(DMA+0x08u)=B(1)|B(3)|B(5)|B(7)|B(8)|B(10)|B(12);
  R(0xe000e100u)=B(11);
  R(DMA+0x08u)|=B(0);
#endif
  R(ADC+0x08u)=B(0)|B(20)|(7u<<17)|(TEST_ADC_DMA ? B(8) : 0u);
  return true;
}
static void adc_consume(void) {
  uint32_t seq, after;
  uint16_t val[3], stamp;
  do {
    seq=adc_seq; val[0]=adc_latest[0]; val[1]=adc_latest[1];
    val[2]=adc_latest[2]; stamp=adc_tick; after=adc_seq;
  } while (seq != after);
  if (seq==consumed) return;
  if (consumed && (uint32_t)(seq-consumed)>1u) adc_skips+=seq-consumed-1u;
  consumed=seq;
  fsr[0]=val[0]; fsr[1]=val[1]; fsr[2]=val[2];
  uint16_t age=(uint16_t)((uint16_t)R(T4+0x24u)-stamp);
  fsr_at=ms()-age; fsr_seen=true; adc_reads++;
}
static void adc_update(uint32_t t) {
  if (adc_busy && t-adc_started_ms>25u) {
    R(DMA+0x08u)&=~B(0);
    adc_busy=false; adc_fault=true; adc_timeouts++;
    test_pin(0u,false);
  }
  bool request = test_stage != 2u && !adc_fault && !adc_busy &&
                 (uint32_t)(t-adc_last_request_ms)>=TEST_ADC_PERIOD_MS;
  uint16_t begin=0;
  if (request) {
    test_pin(1u,true);
    begin=test_us16();
    test_request_start(t);
    adc_last_request_ms=t; adc_started_ms=t;
    adc_swstart_count++; /* one request = one THREE-channel group */
    adc_busy=true;
    test_pin(0u,true);
#if TEST_ADC_DMA
    R(ADC+0x08u)|=B(22);
#else
    static const uint8_t channels[3]={0u,1u,4u};
    uint16_t values[3]; bool ok=true;
    for (unsigned i=0;i<3;i++) {
      R(ADC+0x34u)=channels[i];
      R(ADC)=0u;
      R(ADC+0x08u)|=B(22);
      if (!adc_wait(ADC,B(1),true)) { ok=false; break; }
      values[i]=(uint16_t)R(ADC+0x4cu);
    }
    if (ok) {
      adc_latest[0]=values[0]; adc_latest[1]=values[1]; adc_latest[2]=values[2];
      adc_tick=(uint16_t)R(T4+0x24u); adc_seq++; adc_completed_count++;
    } else { adc_fault=true; adc_timeouts++; }
    adc_busy=false;
    test_pin(0u,false);
#endif
  }
  adc_consume();
  if (request) {
    uint16_t elapsed=test_elapsed(begin);
    /* elapsed is wall time, not total CPU use. ISR costs remain separate. */
    if (test_stage==1u) test_stat_add(&test_request_us,elapsed);
    adc_cpu_us+=elapsed;
    test_pin(1u,false);
  }
}
static bool fsr_good(uint32_t t) {
  return FSR_CALIBRATED && !adc_fault && fsr_seen && t - fsr_at <= 50 && threshold[0] > FSR_RELEASE_MARGIN
          && threshold[1] > FSR_RELEASE_MARGIN && threshold[2] > FSR_RELEASE_MARGIN && threshold[0] < 4095
          && threshold[1] < 4095 && threshold[2] < 4095;
}


typedef struct {
  bool raw, stable;
  uint32_t change;
} Deb;
static void deb_init(Deb * d, bool v, uint32_t t) {
  d->raw = d->stable = v;
  d->change = t;
}
static bool deb(Deb * d, bool v, uint32_t t) {
  if (v != d->raw) {
      d->raw = v;
      d->change = t;
  }
  if (d->stable != d->raw && t - d->change >= 30) {
      d->stable = d->raw;
      return true;
  }
  return false;
}
static bool pressed(unsigned n) {
  return !(R(RC + 8) & B(n));
}


static Deb pedal, left, right;
static uint32_t id, weight, set_weight, invalid, cause, pattern_at, fsr_since[3], last_fsr_used,
      last_s2, last_report, last_ui, last_log, last_uno;
static bool uno_seen, suppress_summary;
static uint32_t cal_token;
static uint16_t key_ack;
static bool over[3], rack_history, active_set, assisted, hold, has_confirmed, model_on, parked,
      adc_setup, link_seen;
static unsigned candidate = 9, confirmed = 9;
static uint16_t pwm_us, pwm2_us;
static uint16_t servo1_target_us, servo2_target_us;
static uint32_t servo_step_at;
static Parser par1, par3;
static Packet ui;
static uint32_t ui_at;
static Packet cal_status;
static uint32_t cal_status_at;
static uint8_t txbuf[FRAME_N];
static unsigned txidx = FRAME_N;


static void uno_tx_service(void) {
  if (txidx < FRAME_N && (R(U3) & B(7))) R(U3 + 4) = txbuf[txidx++];
}


static void servo_init(void) {
   APB2 |= B(0) | B(2);
   APB1 |= B(1);
   R(0x40010004u) &= ~(3u << 10);
   pin(RA, 6, 11);
   pin(RA, 7, 11);
   R(TIM3) = 0u;
   R(TIM3 + 0x28u) = 7u;
   R(TIM3 + 0x2cu) = 19999u;
   R(TIM3 + 0x34u) = SERVO2_DEPLOY_US;
   R(TIM3 + 0x38u) = SERVO1_DEPLOY_US;
   R(TIM3 + 0x18u) = (6u << 4) | B(3) | (6u << 12) | B(11);
   R(TIM3 + 0x20u) = 0u;
   R(TIM3 + 0x14u) = 1u;
   R(TIM3) = B(7) | B(0);
   /* No position feedback at power-up: retain the original initial command. */
   pwm_us = servo1_target_us = SERVO1_DEPLOY_US;
   pwm2_us = servo2_target_us = SERVO2_DEPLOY_US;
   servo_step_at = ms();
}
static bool servo_config(void) {
   return SERVO_ENABLED &&
          SERVO_MIN_US >= 500u && SERVO_MAX_US <= 2500u &&
          SERVO1_DEPLOY_US >= SERVO_MIN_US && SERVO1_DEPLOY_US <= SERVO_MAX_US &&
          SERVO1_RETRACT_US >= SERVO_MIN_US && SERVO1_RETRACT_US <= SERVO_MAX_US &&
          SERVO2_DEPLOY_US >= SERVO_MIN_US && SERVO2_DEPLOY_US <= SERVO_MAX_US &&
          SERVO2_RETRACT_US >= SERVO_MIN_US && SERVO2_RETRACT_US <= SERVO_MAX_US &&
          SERVO1_DEPLOY_US != SERVO1_RETRACT_US &&
          SERVO2_DEPLOY_US != SERVO2_RETRACT_US &&
          (SERVO1_RETRACT_US > SERVO1_DEPLOY_US) &&
          (SERVO2_RETRACT_US < SERVO2_DEPLOY_US);
}
static void servo_set(bool retract) {
   if (!servo_config()) return;
   uint16_t s1 = retract ? SERVO1_RETRACT_US : SERVO1_DEPLOY_US;
   uint16_t s2 = retract ? SERVO2_RETRACT_US : SERVO2_DEPLOY_US;
   if (servo1_target_us != s1 || servo2_target_us != s2) {
       servo1_target_us = s1;
       servo2_target_us = s2;
       servo_step_at = ms();
   }
   if (!model_on) {
       R(TIM3 + 0x20u) |= B(0) | B(4);
       model_on = true;
   }
}
static uint16_t servo_approach(uint16_t current, uint16_t target) {
   if (current < target) {
       uint32_t next = (uint32_t)current + SERVO_STEP_US;
       return next < target ? (uint16_t)next : target;
   }
   if (current > target) {
       return (uint32_t)(current - target) > SERVO_STEP_US ?
              (uint16_t)(current - SERVO_STEP_US) : target;
   }
   return target;
}
static void servo_move_task(uint32_t t) {
   if (!model_on || !servo_config()) return;
   if ((uint32_t)(t - servo_step_at) < SERVO_STEP_MS) return;
   servo_step_at = t;
   /* One small step per call; do not jump to catch up after a delayed loop. */
   pwm_us = servo_approach(pwm_us, servo1_target_us);
   pwm2_us = servo_approach(pwm2_us, servo2_target_us);
   R(TIM3 + 0x38u) = pwm_us;
   R(TIM3 + 0x34u) = pwm2_us;
}
static void rack_report(uint32_t t);
static void assist(uint32_t why) {
  if (!hold) {
      hold = true;
      cause = why;
      if (active_set) assisted = true;
      logstr("[SERVO] Failure -> DEPLOY, latched until rack cycle\r\n");
  }
  servo_set(false);
  parked = true;
}
static bool clear_to_retract(uint32_t t) {
  if (!fsr_good(t) || pedal.raw || pedal.stable) return false;
  for (unsigned i = 0; i < 3; ++i) {
      if (fsr[i] >= threshold[i] - FSR_RELEASE_MARGIN) return false;
  }
  return true;
}
static void servo_task(uint32_t t, bool raw_both) {
  (void)raw_both;
  if (!servo_config()) return;
  if (active_set && !hold && !fsr_good(t)) {
      invalid |= I_FSR | I_ASSIST;
      assist(0);
      rack_report(t);
  }
  servo_move_task(ms());
}
static uint32_t flags(uint32_t t) {
  return (active_set ? F_ACTIVE : 0)
      | (assisted ? F_ASSIST : 0)
      | ((confirmed == 0 && candidate == 0) ? F_RACKED : 0)
      | ((confirmed == 1 && candidate == 1) ? F_UNRACKED : 0)
      | ((candidate == 2) ? F_PARTIAL : 0)
      | (parked ? F_PARKED : 0)
      | (weight ? F_WEIGHT : 0)
      | (fsr_good(t) ? F_FSR_OK : 0)
      | ((link_seen && t - last_s2 <= 1000) ? F_LINK : 0)
      | (servo_config() ? F_MODEL_ENABLED : 0)
      | ((left.raw && right.raw) ? F_CONTACT : 0);
}
static void rack_report(uint32_t t) {
  Packet p = {
      T_RACK,
      id,
      flags(t),
      active_set || id ? set_weight : weight,
      cause,
      invalid,
      (uint32_t)fsr[0] | ((uint32_t)fsr[1] << 16),
      fsr[2],
      cal_token
  };
  if (!active_set && !id) p.b = weight;
  sendp(U1, & p);
  last_report = t;
}
static void request_calibration(void) {
  /* One calibration generation per power cycle, reused by every set. */
  if (!cal_token) cal_token = 1;
}
static bool local_racked(void) {
  return has_confirmed && confirmed == 0 && candidate == 0 && left.raw && right.raw;
}
static bool register_weight(Packet p, uint32_t t) {
  if (p.a < 1 || p.a > 9999 || p.b < 1 || p.b > 65535 ||
      p.b == key_ack || active_set) return false;
  key_ack = (uint16_t)p.b;
  weight = p.a;
  suppress_summary = true;
  if (local_racked()) request_calibration();
  logstr("[중량 등록] 최초 거치 보정 기준을 사용합니다.\r\n");
  rack_report(t);
  return true;
}
static void start_set(uint32_t t) {
  if (!weight || !cal_token || id == UINT32_MAX) return;
  id++;
  active_set = true;
  assisted = false;
  set_weight = weight;
  invalid = (!weight ? I_WEIGHT : 0) | (!parked ? I_MODEL : 0) | (!fsr_good(t) ? I_FSR : 0)
          | (!(link_seen && t - last_s2 <= 1000) ? I_LINK : 0);
  cause = 0;
  if (!servo_config() || !clear_to_retract(t)) {
      hold = true;
      invalid |= (!servo_config() ? I_MODEL : I_FSR);
      logstr("[SERVO] Set started; retraction blocked (servo/FSR/pedal)\r\n");
  } else {
      hold = false;
      parked = false;
      servo_set(true);
      logstr("[SERVO] Set started -> RETRACT\r\n");
  }
  suppress_summary = false;
  for (unsigned i = 0; i < 3; i++) over[i] = false;
  rack_report(t);
  logstr("[세트 시작]\r\n");
}
static void rack_task(uint32_t t) {
  bool v = pressed(7), l = pressed(8), r = pressed(9);
  bool pedal_change = deb(& pedal, v, t);
  deb(& left, l, t);
  deb(& right, r, t);
  unsigned next = left.stable && right.stable ? 0 : !left.stable && !right.stable ? 1 : 2;
  if (next != candidate) {
      candidate = next;
      pattern_at = t;
  }
  if (next != 2 && t - pattern_at >= (next == 0 ? 500u : 100u) && (!has_confirmed || confirmed != next)) {
      confirmed = next;
      has_confirmed = true;
      if (next == 0) {
          rack_history = true;
          if (servo_config()) {
              servo_set(false);
              parked = true;
          }
          hold = false;
          /* Rack entry ends a set without changing the calibration generation. */
          bool ended = active_set;
          active_set = false;
          rack_report(t);
          logstr(ended ? "[재거치 종료] 최초 보정 유지, 다음 세트 대기\r\n" :
                         "[거치 확인] 중량 등록 후 최초 보정\r\n");
      }
      else if (rack_history && !active_set) start_set(t);
  }
  if (!active_set && weight && local_racked() && !cal_token) {
      request_calibration();
      rack_report(t);
      logstr("[최초 보정] MPU/ToF 기준 저장 요청\r\n");
  }
  bool monitor = (confirmed == 1 && candidate == 1) || active_set;
  if (pedal_change && pedal.stable && monitor && !hold) {
      assist(REASON_PEDAL);
      invalid |= active_set ? I_ASSIST : 0;
      rack_report(t);
  }
  bool good = fsr_good(t);
  if (active_set && !good) invalid |= I_FSR;
  if (active_set && (!link_seen || t - last_s2 > 1000 || !uno_seen || t - last_uno > 1200)) invalid |= I_LINK;
  if (!monitor || !good) {
      for (unsigned i = 0; i < 3; i++) over[i] = false;
  }
  else if (fsr_at != last_fsr_used) {
      if (last_fsr_used && fsr_at - last_fsr_used > 50) for (unsigned i = 0; i < 3; i++) over[i] = false;
      for (unsigned i = 0; i < 3; i++) {
          if (fsr[i] > threshold[i]) {
              if (!over[i]) {
                  over[i] = true;
                  fsr_since[i] = fsr_at;
              }
              else if (fsr_at - fsr_since[i] >= FSR_CONFIRM_MS && !hold) {
                  assist(REASON_FSR1 + i);
                  if (active_set) invalid |= I_ASSIST;
                  rack_report(t);
              }
          }
          else over[i] = false;
      }
  }
  last_fsr_used = fsr_at;
  servo_task(t, l && r);
}


static void send_ui(uint32_t t) {
  if (txidx < FRAME_N) return;
  Packet p = { T_UI, id, flags(t), active_set ? set_weight : weight, 0, 0, 0, 0, 0 };
  if (!active_set) p.a |= cal_token ? F_CAL_BUSY : F_CAL_REQUIRED;


  bool has_valid_ui = (ui.type == T_UI && ui.id == id && t - ui_at < 1000);
  if (has_valid_ui) {
      p.a |= ui.a & (F_SUMMARY | F_WAIT_FINAL | F_M_READY | F_M_TILT | F_M_BASE |
                     F_M_WARN | F_CAL_BUSY | F_CAL_FAIL | F_CAL_REQUIRED);
      p.c = ui.c;
      p.d = ui.d;
      p.e = ui.e;
      p.f = ui.f;
      p.g = ui.g;
      if ((p.a & F_SUMMARY) && !active_set && !suppress_summary) p.b = set_weight;
      else if (suppress_summary) {
          p.a &= ~(F_SUMMARY | F_WAIT_FINAL);
          /* Convert summary errors before restoring the normal ACK layout. */
          if (ui.a & F_SUMMARY)
              p.g = ((ui.a >> SUMMARY_INVALID_SHIFT) & SUMMARY_INVALID_MASK) << 16;
      }
  } else if (id) {
      p.g = (I_LINK << 16);
  }


  if (!active_set) {
      p.a &= ~(F_M_READY | F_CAL_BUSY | F_CAL_FAIL | F_CAL_REQUIRED);
      if (!cal_token) p.a |= F_CAL_REQUIRED;
      else if (has_valid_ui && cal_status.type == T_CAL_STATUS &&
               cal_status.id == id && cal_status.a == cal_token &&
               t - cal_status_at <= 500u) {
          p.a |= cal_status.b == CAL_READY ? F_M_READY :
                 cal_status.b == CAL_FAIL ? F_CAL_FAIL : F_CAL_BUSY;
      } else p.a |= F_CAL_REQUIRED; /* No fresh status is not proof of calibration activity. */
  }


  /* Summary: retain the exact 0.1 kg weight in g[31:16] and all errors
   * in a[29:20], including I_BASE/I_CAL. Normal ACK layout is unchanged. */
  if ((p.a & F_SUMMARY) && !active_set) {
      uint32_t errors = ((ui.a >> SUMMARY_INVALID_SHIFT) | invalid) & SUMMARY_INVALID_MASK;
      p.a |= errors << SUMMARY_INVALID_SHIFT;
      if (cause) p.g = (p.g & ~0xff00u) | ((cause & 255u) << 8);
  } else {
      /* 대기 중이거나 세트 진행 중일 때는 기존 방식대로 key_ack 및 invalid 플래그 패킹 */
      p.g = (p.g & 0x0000ffffu) | (((p.g >> 16) | invalid) & 0xffffu) << 16;
      if (cause) p.g = (p.g & ~0xff00u) | ((cause & 255u) << 8);
      p.f = (p.f & 0xffffu) | ((uint32_t)key_ack << 16);
  }


  encode(&p, txbuf);
  txidx = 0;
  last_ui = t;
}


int main(void) {
  base_init();
  uart_init(U1, 115200);
  uart_init(U3, 9600);
  uart_init(U2, 115200);
  for (unsigned n = 7; n <= 9; n++) {
      pin_high(RC, n);
      pin(RC, n, 8);
  }
  servo_init();
  uint32_t t = ms();
  deb_init(& pedal, pressed(7), t);
  deb_init(& left, pressed(8), t);
  deb_init(& right, pressed(9), t);
  test_trace_init();
  adc_setup = adc_init();
  adc_fault = !adc_setup;
  test_boot_ms = ms();


  if (servo_config()) {
      servo_set(false);
      parked = true;
      logstr("[SERVO] Startup -> DEPLOY\r\n");
  }
  logstr("[STM3] 랙/FSR ADC A-B target 200 Hz/SG90 서보 2개 + AI 코칭 중계. 모형 전용.\r\n");
  if (!servo_config()) logstr("[설정] 서보 출력 OFF. 무부하 펄스 교정 후 활성화.\r\n");
  if (!FSR_CALIBRATED) logstr("[설정] FSR calibration OFF: retract inhibited, FSR triggers disabled.\r\n");


  for (;;) {
      uno_tx_service();
      t = ms();
      test_before_adc(t);
      adc_update(t);
      /* adc_update timestamps the newly consumed sample with a fresh ms(). */
      t = ms();
      test_rack_start(t);
      test_pin(2u, true);
      rack_task(t);
      test_pin(2u, false);
      test_finish();
      t = ms();
      t = ms();
      Packet p;
      while (take(& r1, & par1, & p)) {
          if (p.type == T_UI) {
              link_seen = true;
              last_s2 = t;
              ui = p;
              ui_at = t;
          } else if (p.type == T_CAL_STATUS && p.id == id) {
              cal_status = p;
              cal_status_at = t;
          }
      }
      while (take(& r3, & par3, & p)) {
          if (p.type == T_KEY && p.id == id) {
              uno_seen = true;
              last_uno = t;
              (void)register_weight(p, t);
          }
      }
      if (t - last_report >= 100) rack_report(t);
      if (t - last_ui >= 250) send_ui(t);
      if (!test_quiet && t - last_log >= 1000) {
          last_log = t;
          char b[360];
          uint8_t rx_rec = (uint8_t)(ui.g & 0xffu);
          uint16_t rx_w  = (uint16_t)(ui.g >> 16);
          uint8_t rx_rep = (uint8_t)((ui.f >> 24) & 0xffu);
          uint8_t rx_scr = (uint8_t)((ui.f >> 16) & 0xffu);
          snprintf(b, sizeof b,
                   "[STM3] ID %lu | flags %lx | FSR %u,%u,%u | PWM D11:%u D12:%u | samples %lu lost %lu dma_tc %lu | REC:%u (%u.%ukg x %u, score:%u)\r\n",
                   (unsigned long) id, (unsigned long) flags(t), fsr[0], fsr[1], fsr[2],
                   pwm_us, pwm2_us, (unsigned long) adc_reads, (unsigned long) adc_skips,
                   (unsigned long)dma_tc_count, rx_rec, rx_w / 10u, rx_w % 10u, rx_rep, rx_scr);
          logstr(b);
      }
  }
}







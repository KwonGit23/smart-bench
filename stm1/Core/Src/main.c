/* PRJ2 STM1 DEBUG V4: V2 37-byte wire protocol, STM2 DEBUG V3 compatible.
* Default: existing wiring, VL53L0X software reset via I2C.
* Optional hardware reset: TOF_USE_XSHUT=1, PB0/A3 open-drain to XSHUT
* (module must provide pull-up to its own logic supply). No push-pull high.
* Sensor retry only outside known active sets. A reset does not restore a set.
*/
/* NUCLEO-F103RB / VL53L0X + MPU6050 통합 테스트 / UTF-8
* Core/Src/main.c 전체 교체. sysmem.c 등은 수정하지 말 것.
* PB6=SCL, PB7=SDA, 3.3V module, GND shared; USART2 PA2/PA3 VCP 115200.
* GPIO open-drain software I2C (register only), no HAL; external pull-ups required.
* Automatic start height, distance-based rep counting. MPU attitude independent.
* Integrated build keeps raw ToF (no angle correction); beam parallel to +X.
* Mock-up test only, not a safety controller.
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define BIT(n) (1u<<(n))
#define RCC_CR REG32(0x40021000u)
#define RCC_CFGR REG32(0x40021004u)
#define RCC_APB2ENR REG32(0x40021018u)
#define RCC_APB1ENR REG32(0x4002101cu)
#define GPIOA_CRL REG32(0x40010800u)
#define GPIOB_CRL REG32(0x40010c00u)
#define GPIOB_IDR REG32(0x40010c08u)
#define GPIOB_BSRR REG32(0x40010c10u)
#define TIM2_CR1 REG32(0x40000000u)
#define TIM2_EGR REG32(0x40000014u)
#define TIM2_CNT REG32(0x40000024u)
#define TIM2_PSC REG32(0x40000028u)
#define TIM2_ARR REG32(0x4000002cu)
#define TIM3_CR1 REG32(0x40000400u)
#define TIM3_EGR REG32(0x40000414u)
#define TIM3_CNT REG32(0x40000424u)
#define TIM3_PSC REG32(0x40000428u)
#define TIM3_ARR REG32(0x4000042cu)
#define U2_SR REG32(0x40004400u)
#define U2_DR REG32(0x40004404u)
#define U2_BRR REG32(0x40004408u)
#define U2_CR1 REG32(0x4000440cu)
#define TOF_USE_XSHUT 0
#define SCL BIT(6)
#define SDA BIT(7)
static void platform_init(void) {
  RCC_CR |= BIT(0);
  while (!(RCC_CR & BIT(1))) {
  }
  RCC_CFGR &= ~3u;
  while (RCC_CFGR &(3u << 2)) {
  }
  RCC_CFGR &= ~((15u << 4) | (7u << 8) | (7u << 11));
  RCC_APB2ENR |= BIT(0) | BIT(2) | BIT(3);
  RCC_APB1ENR |= BIT(0) | BIT(1) | BIT(17);
  TIM2_CR1 = 0;
  TIM2_PSC = 7999;
  TIM2_ARR = 65535;
  TIM2_EGR = 1;
  TIM2_CNT = 0;
  TIM2_CR1 = 1;
  TIM3_CR1 = 0;
  TIM3_PSC = 7;
  TIM3_ARR = 65535;
  TIM3_EGR = 1;
  TIM3_CNT = 0;
  TIM3_CR1 = 1;
  GPIOA_CRL = (GPIOA_CRL & ~((15u << 8) | (15u << 12))) | (11u << 8) | (4u << 12);
  U2_BRR = 69;
  U2_CR1 = BIT(13) | BIT(3) | BIT(2);
  /* release both lines before selecting 2 MHz open-drain outputs */
  GPIOB_BSRR = SCL | SDA;
  GPIOB_CRL = (GPIOB_CRL & ~((15u << 24) | (15u << 28))) | (6u << 24) | (6u << 28);
}
static uint32_t now_ms(void) {
  static uint16_t last;
  static uint32_t total;
  uint16_t cur = (uint16_t) TIM2_CNT;
  total += (uint16_t)(cur - last);
  last = cur;
  return total;
}
static bool sensor_recovering;
static void service_recovery_link(void);
static bool recovery_interrupted(void);
static void delay_ms(uint32_t n) {
  uint32_t t = now_ms();
  while (now_ms() - t < n) {
      if (sensor_recovering) service_recovery_link();
  }
}
static void delay_us(uint16_t n) {
  uint16_t t = (uint16_t) TIM3_CNT;
  while ((uint16_t)((uint16_t) TIM3_CNT - t) < n) {
  }
}
static void out(const char * s) {
  while (*s) {
      while (!(U2_SR & BIT(7))) {
      }
      U2_DR = (uint8_t) * s++;
  }
}
static char msg[256];
static void low(uint32_t p) {
  GPIOB_BSRR = p << 16;
}
static void release(uint32_t p) {
  GPIOB_BSRR = p;
}
static const char * bus_step = "없음";
static bool scl_high(void) {
  release(SCL);
  uint32_t t = now_ms();
  while (!(GPIOB_IDR & SCL)) {
      if (now_ms() - t >= 5) {
          bus_step = "SCL LOW 고정/시간 초과";
          return false;
      }
  }
  delay_us(6);
  return true;
}
static bool start(void) {
  release(SDA);
  delay_us(6);
  if (!scl_high()) return false;
  if (!(GPIOB_IDR & SDA)) {
      bus_step = "SDA LOW 고정";
      return false;
  }
  low(SDA);
  delay_us(6);
  low(SCL);
  return true;
}
static bool stop(void) {
  low(SCL);
  low(SDA);
  delay_us(6);
  if (!scl_high()) {
      release(SDA);
      return false;
  }
  release(SDA);
  delay_us(6);
  if (!(GPIOB_IDR & SDA)) {
      bus_step = "STOP 후 SDA LOW";
      return false;
  }
  return true;
}
static void abort_bus(void) {
  const char * why = bus_step;
  (void) stop();
  release(SDA | SCL);
  bus_step = why;
}
static bool tx(uint8_t b, const char * nack) {
  for (unsigned i = 0; i < 8; i++) {
      low(SCL);
      if (b & 0x80) release(SDA);
      else low(SDA);
      delay_us(6);
      if (!scl_high()) return false;
      low(SCL);
      b <<= 1;
  }
  release(SDA);
  delay_us(6);
  if (!scl_high()) return false;
  bool ack = !(GPIOB_IDR & SDA);
  low(SCL);
  if (!ack) bus_step = nack;
  return ack;
}
static bool rx(uint8_t * b, bool ack) {
  uint8_t v = 0;
  release(SDA);
  for (unsigned i = 0; i < 8; i++) {
      delay_us(6);
      if (!scl_high()) return false;
      v = (uint8_t)((v << 1) | ((GPIOB_IDR & SDA) != 0));
      low(SCL);
  }
  if (ack) low(SDA);
  else release(SDA);
  delay_us(6);
  if (!scl_high()) return false;
  low(SCL);
  release(SDA);
  * b = v;
  return true;
}
/* start/repeated start; all transfers use 7-bit addresses. */
static bool bus_write(uint8_t addr, uint8_t reg, uint8_t data) {
  if (start() && tx((uint8_t)(addr << 1), "쓰기 주소 NACK") && tx(reg, "레지스터 NACK") && tx(data, "데이터 NACK") && stop()) return true;
  abort_bus();
  return false;
}
static bool bus_read(uint8_t addr, uint8_t reg, uint8_t * out, unsigned n) {
  if (!n) return false;
  if (!start() || !tx((uint8_t)(addr << 1), "쓰기 주소 NACK") || !tx(reg, "레지스터 NACK") || !start() || !tx((uint8_t)((addr << 1) | 1u),
      "읽기 주소 NACK")) goto fail;
  for (unsigned i = 0; i < n; i++) if (!rx(out + i, i + 1 < n)) goto fail;
  if (stop()) return true;
  fail: abort_bus();
  return false;
}
static bool probe(uint8_t addr) {
  if (start() && tx((uint8_t)(addr << 1), "주소 NACK") && stop()) return true;
  abort_bus();
  return false;
}
/* VL53L0X register initialization adapted from Pololu vl53l0x-arduino
* https://github.com/pololu/vl53l0x-arduino ; default tuning derived from ST API.
* Copyright (c) 2017-2022 Pololu Corporation.
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Copyright (c) 2016, STMicroelectronics International N.V. All rights reserved.
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
* - Neither the name of STMicroelectronics nor the names of its contributors
*   may be used to endorse or promote products derived from this software
*   without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#define TOF_ADDR 0x29u
static uint8_t diag_reg, diag_value, diag_expected, diag_spad_count, diag_spad_enabled;
static const char * diag_why = "없음";
static bool tw(uint8_t r, uint8_t d) {
  if (sensor_recovering) {
      service_recovery_link();
      if (recovery_interrupted()) {
          diag_why = "세트 진행 중: 재초기화 중단";
          return false;
      }
  }
  diag_reg = r;
  if (bus_write(TOF_ADDR, r, d)) return true;
  diag_why = "I2C 쓰기 실패";
  return false;
}
static bool tr(uint8_t r, uint8_t * d) {
  if (sensor_recovering) {
      service_recovery_link();
      if (recovery_interrupted()) {
          diag_why = "세트 진행 중: 재초기화 중단";
          return false;
      }
  }
  diag_reg = r;
  if (bus_read(TOF_ADDR, r, d, 1)) return true;
  diag_why = "I2C 읽기 실패";
  return false;
}
static bool twait(uint8_t r, uint8_t mask) {
  uint32_t start = now_ms();
  uint8_t v;
  do {
      if (!tr(r, &v)) return false;
      if (v & mask) return true;
      delay_ms(2);
  } while (now_ms() - start < 500u);
  diag_why = "센서 내부 응답 시간 초과";
  diag_reg = r;
  diag_value = v;
  return false;
}
static unsigned tof_stage;
#define W(r,v) do{if(!tw((r),(v)))return false;}while(0)
#define R(r,v) do{if(!tr((r),&(v)))return false;}while(0)
static bool tof_init(void) {
  uint8_t v, stop, spad[6], count;
  unsigned first, enabled = 0;
  tof_stage = 1;
  R(0xc0, v);
  if (v != 0xee) {
      diag_why = "모델 ID 불일치";
      diag_value = v;
      diag_expected = 0xee;
      return false;
  }
  /* 2.8 V I/O mode; breakout level shifting / supply per module. */
  R(0x89, v);
  W(0x89, v | 1u);
  W(0x88, 0);
  W(0x80, 1);
  W(0xff, 1);
  W(0, 0);
  R(0x91, stop);
  W(0, 1);
  W(0xff, 0);
  W(0x80, 0);
  R(0x60, v);
  W(0x60, v | 0x12);
  W(0x44, 0);
  W(0x45, 0x20);
  W(0x01, 0xff);
  tof_stage = 2;
  W(0x80, 1);
  W(0xff, 1);
  W(0, 0);
  W(0xff, 6);
  R(0x83, v);
  W(0x83, v | 4u);
  W(0xff, 7);
  W(0x81, 1);
  W(0x80, 1);
  W(0x94, 0x6b);
  W(0x83, 0);
  if (!twait(0x83, 0xff)) return false;
  W(0x83, 1);
  R(0x92, v);
  count = v & 0x7f;
  first = (v & 0x80) ? 12u: 0u;
  W(0x81, 0);
  W(0xff, 6);
  R(0x83, v);
  W(0x83, v & ~4u);
  W(0xff, 1);
  W(0, 1);
  W(0xff, 0);
  W(0x80, 0);
  diag_reg = 0xb0;
  if (!bus_read(TOF_ADDR, 0xb0, spad, 6)) {
      diag_why = "SPAD 맵 읽기 실패";
      return false;
  }
  W(0xff, 1);
  W(0x4f, 0);
  W(0x4e, 0x2c);
  W(0xff, 0);
  W(0xb6, 0xb4);
  for (unsigned i = 0; i < 48; i++) {
      if (i < first || enabled == count) spad[i / 8] &= (uint8_t) ~(1u << (i % 8));
      else if (spad[i / 8] &(1u << (i % 8))) enabled++;
  }
  if (!count || enabled != count) {
      diag_why = "SPAD 맵 불일치";
      diag_spad_count = count;
      diag_spad_enabled = (uint8_t) enabled;
      return false;
  }
  for (unsigned i = 0; i < 6; i++) W(0xb0 + i, spad[i]);
  tof_stage = 3;
  static const uint8_t tune[][2] = {
      {
          0xff, 1
      }, {
          0, 0
      }, {
          0xff, 0
      }, {
          9, 0
      }, {
          0x10, 0
      }, {
          0x11, 0
      }, {
          0x24, 1
      }, {
          0x25, 0xff
      }, {
          0x75, 0
      }, {
          0xff, 1
      }, {
          0x4e, 0x2c
      }, {
          0x48, 0
      }, {
          0x30, 0x20
      }, {
          0xff, 0
      }, {
          0x30, 9
      }, {
          0x54, 0
      }, {
          0x31, 4
      }, {
          0x32, 3
      }, {
          0x40, 0x83
      }, {
          0x46, 0x25
      }, {
          0x60, 0
      }, {
          0x27, 0
      }, {
          0x50, 6
      }, {
          0x51, 0
      }, {
          0x52, 0x96
      }, {
          0x56, 8
      }, {
          0x57, 0x30
      }, {
          0x61, 0
      }, {
          0x62, 0
      }, {
          0x64, 0
      }, {
          0x65, 0
      }, {
          0x66, 0xa0
      }, {
          0xff, 1
      }, {
          0x22, 0x32
      }, {
          0x47, 0x14
      }, {
          0x49, 0xff
      }, {
          0x4a, 0
      }, {
          0xff, 0
      }, {
          0x7a, 0x0a
      }, {
          0x7b, 0
      }, {
          0x78, 0x21
      }, {
          0xff, 1
      }, {
          0x23, 0x34
      }, {
          0x42, 0
      }, {
          0x44, 0xff
      }, {
          0x45, 0x26
      }, {
          0x46, 5
      }, {
          0x40, 0x40
      }, {
          0x0e, 6
      }, {
          0x20, 0x1a
      }, {
          0x43, 0x40
      }, {
          0xff, 0
      }, {
          0x34, 3
      }, {
          0x35, 0x44
      }, {
          0xff, 1
      }, {
          0x31, 4
      }, {
          0x4b, 9
      }, {
          0x4c, 5
      }, {
          0x4d, 4
      }, {
          0xff, 0
      }, {
          0x44, 0
      }, {
          0x45, 0x20
      }, {
          0x47, 8
      }, {
          0x48, 0x28
      }, {
          0x67, 0
      }, {
          0x70, 4
      }, {
          0x71, 1
      }, {
          0x72, 0xfe
      }, {
          0x76, 0
      }, {
          0x77, 0
      }, {
          0xff, 1
      }, {
          0x0d, 1
      }, {
          0xff, 0
      }, {
          0x80, 1
      }, {
          1, 0xf8
      }, {
          0xff, 1
      }, {
          0x8e, 1
      }, {
          0, 1
      }, {
          0xff, 0
      }, {
          0x80, 0
      }
  };
  for (unsigned i = 0; i < sizeof tune / sizeof tune[0]; i++) W(tune[i][0], tune[i][1]);
  W(0x0a, 4);
  R(0x84, v);
  W(0x84, v & ~0x10u);
  W(0x0b, 1);
  /* Use default tuning timeout values, no custom timing budget. */
  tof_stage = 4;
  W(1, 1);
  W(0, 0x41);
  if (!twait(0x13, 7)) return false;
  W(0x0b, 1);
  W(0, 0);
  W(1, 2);
  W(0, 1);
  if (!twait(0x13, 7)) return false;
  W(0x0b, 1);
  W(0, 0);
  W(1, 0xe8);
  tof_stage = 5;
  W(0x80, 1);
  W(0xff, 1);
  W(0, 0);
  W(0x91, stop);
  W(0, 1);
  W(0xff, 0);
  W(0x80, 0);
  W(0, 2);
  return true;
  /* back-to-back continuous */
}
#undef W
#undef R
/* 0 no new result, 1 new result, -1 bus error. Raw device status 11 is range complete;
* not equivalent to full ST API RangeStatus (which includes software checks). */
static int tof_poll(uint16_t * mm, uint8_t * status, uint16_t * signal) {
  uint8_t v, b[12];
  if (!tr(0x13, &v)) return - 1;
  if (!(v & 7)) return 0;
  if (!bus_read(TOF_ADDR, 0x14, b, 12) || !tw(0x0b, 1)) return - 1;
  * status = (b[0] >> 3) & 15u;
  * mm = ((uint16_t) b[10] << 8) | b[11];
  * signal = ((uint16_t) b[6] << 8) | b[7];
  /* Q9.7 MCPS */
  return 1;
}
/* I2C bus clear (UM10204): release SDA and send up to nine clocks.
* Never drive either line HIGH; a physical short or held SCL must not be forced. */
static void print_lines(void) {
  unsigned pins = GPIOB_IDR;
  snprintf(msg, sizeof msg, "[I2C] SCL=%u | SDA=%u\r\n", (unsigned)((pins >> 6) & 1u), (unsigned)((pins >> 7) & 1u));
  out(msg);
}
static bool recover_bus(void) {
  release(SCL | SDA);
  delay_ms(5);
  print_lines();
  if ((GPIOB_IDR &(SCL | SDA)) == (SCL | SDA)) return true;
  if (!(GPIOB_IDR & SCL)) {
      out("[복구 실패] SCL LOW. 전원/배선/풀업 확인.\r\n");
      return false;
  }
  out("[버스 복구] SDA LOW. SCL 클록 최대 9회 전송.\r\n");
  unsigned pulses = 0;
  while (!(GPIOB_IDR & SDA) && pulses < 9u) {
      low(SCL);
      delay_us(10);
      if (!scl_high()) {
          release(SCL | SDA);
          out("[복구 실패] SCL 상승 불가.\r\n");
          print_lines();
          return false;
      }
      delay_us(10);
      pulses++;
  }
  if (!(GPIOB_IDR & SDA)) {
      release(SCL | SDA);
      out("[복구 실패] 9회 클록 후에도 SDA LOW.\r\n");
      print_lines();
      return false;
  }
  /* End the interrupted transaction with an explicit STOP. */
  if (!stop()) {
      release(SCL | SDA);
      out("[복구 실패] STOP 생성 실패.\r\n");
      print_lines();
      return false;
  }
  delay_ms(5);
  print_lines();
  if ((GPIOB_IDR &(SCL | SDA)) != (SCL | SDA)) return false;
  snprintf(msg, sizeof msg, "[복구 성공] %u회 클록 후 주소 검색.\r\n", pulses);
  out(msg);
  return true;
}
/* ST API ResetDevice sequence: page 0, reset low, model ID goes zero,
* release reset and wait for 0xEE. All waits bounded. Even on timeout try
* releasing reset. Never leave a deliberately asserted software reset.
*/
static bool wait_model(uint8_t expected, uint32_t timeout) {
  uint32_t t = now_ms();
  uint8_t id;
  do {
      if (bus_read(TOF_ADDR, 0xc0, &id, 1) && id == expected) return true;
      delay_ms(3);
  } while (now_ms() - t < timeout);
  return false;
}
static bool tof_reset_device(void) {
#if TOF_USE_XSHUT
  /* PB0: 2MHz GPIO open-drain; latch high releases line, never drives it.
    Use only when XSHUT is wired and pulled up at the module. */
  GPIOB_BSRR = BIT(0);
  GPIOB_CRL = (GPIOB_CRL & ~15u) | 6u;
  low(BIT(0));
  delay_ms(10);
  release(BIT(0));
  delay_ms(30);
  bool ok = wait_model(0xee, 150);
  out(ok ? "[ToF RESET] XSHUT / ID OK\r\n": "[ToF RESET] XSHUT / ID FAIL\r\n");
  return ok;
#else
  if (!bus_write(TOF_ADDR, 0xff, 0x00)) {
      out("[ToF RESET] page select FAIL\r\n");
      return false;
  }
  bool asserted = bus_write(TOF_ADDR, 0xbf, 0);
  bool zero = asserted && wait_model(0, 120);
  delay_ms(5);
  bool released = bus_write(TOF_ADDR, 0xbf, 1);
  delay_ms(20);
  bool awake = wait_model(0xee, 200);
  snprintf(msg, sizeof msg, "[ToF RESET] SW assert=%u zero=%u release=%u ID=%s\r\n", (unsigned) asserted, (unsigned) zero,
      (unsigned) released, awake ? "EE": "FAIL");
  out(msg);
  /* If model responds, full initialization is still worth trying on an
    unconfirmed reset; the log makes this best-effort path explicit. */
  if (awake && !(asserted && zero && released)) out("[ToF RESET] reset not confirmed; try full init\r\n");
  return awake;
#endif
}
static bool scan(void) {
  unsigned found = 0;
  bool tof = false;
  if (!recover_bus()) return false;
  for (unsigned a = 0x08; a <= 0x77; a++) {
      if (sensor_recovering) {
          service_recovery_link();
          if (recovery_interrupted()) return false;
      }
      if (probe((uint8_t) a)) {
          found++;
          if (a == 0x29) tof = true;
          snprintf(msg, sizeof msg, "[주소 발견] 0x%02X\r\n", a);
          out(msg);
      }
  }
  snprintf(msg, sizeof msg, "[검색 완료] %u개\r\n", found);
  out(msg);
  if (!tof) out("[오류] VL53L0X(0x29) 응답 없음. 전원, 핀, XSHUT 확인.\r\n");
  return tof;
}
/* MPU6050: +X down (rest Ax ~ -1g), Y parallel to bar, Z transverse.
* Estimate the upward unit vector in sensor coordinates, no position integration. */
#define MPU_ADDR 0x68u
#define IMU_PERIOD_MS 20u
#define IMU_FRESH_MS 150u
#define IMU_CAL_N 100u
#define DEG_F 57.2957795f
#define RAD_F 0.01745329252f
#define BEAM_MIN_COS 0.9063078f /* 25 degrees */
static bool imu_ok, imu_cal_ok, imu_calibrating, imu_fresh, tilt_warn;
static bool imu_notice;
static float imu_up[3] = {
  - 1, 0, 0
}, gyro_bias[3], last_a[3], last_w[3];
static float tilt_zero, tilt_abs, tilt_rel, beam_cos = 1.0f;
static float ca[3], ca2[3], cw[3], cw2[3];
static unsigned cal_n;
static uint32_t imu_at, cal_start, imu_errors;
static float clampf1(float x, float lo, float hi) {
  return x < lo ? lo: x > hi ? hi: x;
}
static float vnorm(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
static bool vunit(float v[3]) {
  float n = vnorm(v);
  if (n < 0.001f) return false;
  for (unsigned i = 0; i < 3; i++) v[i] /= n;
  return true;
}
static int16_t be16(const uint8_t * p) {
  return (int16_t)(((uint16_t) p[0] << 8) | p[1]);
}
static int round_i(float x) {
  return (int)(x +(x < 0 ? - 0.5f: 0.5f));
}
static bool mpu_init(void) {
  uint8_t id = 0, g = 0, a = 0;
  if (!bus_read(MPU_ADDR, 0x75, &id, 1) || id != 0x68u) return false;
  if (!bus_write(MPU_ADDR, 0x6b, 0x01)) return false;
  delay_ms(100);
  if (!bus_write(MPU_ADDR, 0x6c, 0) || !bus_write(MPU_ADDR, 0x1a, 3) || !bus_write(MPU_ADDR, 0x19, 9) || !bus_write(MPU_ADDR, 0x1b,
      8) || !bus_write(MPU_ADDR, 0x1c, 8)) return false;
  /* 100 Hz, +/-500 dps, +/-4g */
  return bus_read(MPU_ADDR, 0x1b, &g, 1) && bus_read(MPU_ADDR, 0x1c, &a, 1) && (g & 0x18u) == 8u && (a & 0x18u) == 8u;
}
static bool mpu_read_sample(float a[3], float w[3]) {
  uint8_t b[14];
  if (!bus_read(MPU_ADDR, 0x3b, b, 14)) return false;
  for (unsigned i = 0; i < 3; i++) {
      int16_t ar = be16(b + 2 * i), wr = be16(b + 8 + 2 * i);
      if (ar >= 32700 || ar <= - 32700 || wr >= 32700 || wr <= - 32700) return false;
      a[i] = (float) ar / 8192.0f;
      w[i] = (float) wr / 65.5f;
  }
  return true;
}
static void imu_cal_begin(uint32_t t) {
  cal_n = 0;
  cal_start = t;
  imu_calibrating = true;
  imu_cal_ok = false;
  imu_fresh = false;
  tilt_warn = false;
  imu_at = 0;
  for (unsigned i = 0; i < 3; i++) ca[i] = ca2[i] = cw[i] = cw2[i] = 0;
  out("[MPU 보정] 바벨을 수평으로 놓고 약 2~3초간 움직이지 마세요.\r\n");
}
static void imu_cal_sample(const float a[3], const float w[3]) {
  for (unsigned i = 0; i < 3; i++) {
      ca[i] += a[i];
      ca2[i] += a[i] * a[i];
      cw[i] += w[i];
      cw2[i] += w[i] * w[i];
  }
  if (++cal_n < IMU_CAL_N) return;
  float avg[3];
  bool ok = true;
  for (unsigned i = 0; i < 3; i++) {
      avg[i] = ca[i] / cal_n;
      gyro_bias[i] = cw[i] / cal_n;
      float av = ca2[i] / cal_n - avg[i] * avg[i], wv = cw2[i] / cal_n - gyro_bias[i] * gyro_bias[i];
      if (av > 0.05f * 0.05f || wv > 3.0f * 3.0f || fabsf(gyro_bias[i]) > 30.0f) ok = false;
  }
  float n = vnorm(avg);
  imu_calibrating = false;
  if (n < 0.8f || n > 1.2f || !vunit(avg) || avg[0] > - 0.8f || fabsf(avg[1]) > 0.15f) ok = false;
  if (!ok) {
      out("[MPU 보정 실패] 움직임 또는 축 방향 확인. +X 아래, Y 수평. 안정 거치 후 자동 재시도. 이번 세트는 준비 완료 전 시작하지 마세요.\r\n");
      return;
  }
  for (unsigned i = 0; i < 3; i++) imu_up[i] = avg[i];
  tilt_abs = asinf(clampf1(imu_up[1], - 1, 1)) * DEG_F;
  tilt_zero = tilt_abs;
  tilt_rel = 0;
  beam_cos = - imu_up[0];
  imu_cal_ok = true;
  imu_fresh = true;
  snprintf(msg, sizeof msg, "[MPU 보정 완료] 영점 오프셋(0.1도/s)=%d,%d,%d | 수평 기준 저장\r\n", round_i(gyro_bias[0] * 10),
      round_i(gyro_bias[1] * 10), round_i(gyro_bias[2] * 10));
  out(msg);
}
/* Complementary filter on gravity direction. Body-frame derivative u'=u x omega. */
static void imu_filter(const float a[3], const float w[3], float dt) {
  float x = imu_up[0], y = imu_up[1], z = imu_up[2];
  float gx = (w[0] - gyro_bias[0]) * RAD_F, gy = (w[1] - gyro_bias[1]) * RAD_F, gz = (w[2] - gyro_bias[2]) * RAD_F;
  float pred[3] = {
      x +(y * gz - z * gy) * dt, y +(z * gx - x * gz) * dt, z +(x * gy - y * gx) * dt
  };
  if (!vunit(pred)) return;
  float an = vnorm(a);
  if (an > 0.88f && an < 1.12f) {
      float k = dt / (0.65f + dt);
      /* accel correction, weaker during motion */
      for (unsigned i = 0; i < 3; i++) pred[i] = (1 - k) * pred[i] + k * a[i] / an;
      if (!vunit(pred)) return;
  }
  for (unsigned i = 0; i < 3; i++) imu_up[i] = pred[i];
  tilt_abs = asinf(clampf1(imu_up[1], - 1, 1)) * DEG_F;
  tilt_rel = tilt_abs - tilt_zero;
  beam_cos = clampf1(- imu_up[0], - 1, 1);
}
static void imu_update(uint32_t t) {
  if (!imu_ok) return;
  if (imu_calibrating && t - cal_start > 6000u) {
      imu_calibrating = false;
      out("[MPU 보정 실패] 시간 초과. 안정 거치 후 자동 재시도.\r\n");
  }
  float a[3], w[3];
  if (!mpu_read_sample(a, w)) {
      imu_errors++;
      imu_fresh = false;
      if (imu_calibrating) {
          imu_calibrating = false;
          imu_cal_ok = false;
      }
      if (!imu_notice) {
          imu_notice = true;
          out("[MPU 오류] 읽기/측정 범위 확인. 거리와 별도 상태입니다.\r\n");
      }
      return;
  }
  t = now_ms();
  if (imu_notice) {
      imu_notice = false;
      out("[MPU] 데이터 수신 복구.\r\n");
  }
  for (unsigned i = 0; i < 3; i++) {
      last_a[i] = a[i];
      last_w[i] = w[i];
  }
  uint32_t elapsed = t - imu_at;
  imu_at = t;
  if (imu_calibrating) {
      imu_cal_sample(a, w);
      return;
  }
  if (!imu_cal_ok) return;
  if (elapsed > IMU_FRESH_MS) {
      /* Don't integrate across missing samples; re-anchor only near static. */
      float n = vnorm(a), rate[3];
      for (unsigned i = 0; i < 3; i++) rate[i] = w[i] - gyro_bias[i];
      if (n < 0.92f || n > 1.08f || vnorm(rate) > 8.0f) {
          imu_fresh = false;
          imu_cal_ok = false;
          out("[MPU] 자세 추정 중단. 안정 거치 후 재보정합니다.\r\n");
          return;
      }
      for (unsigned i = 0; i < 3; i++) imu_up[i] = a[i] / n;
      elapsed = 0;
  }
  imu_filter(a, w, elapsed * 0.001f);
  imu_fresh = true;
  if (!tilt_warn && fabsf(tilt_rel) >= 5.0f) {
      tilt_warn = true;
      out("[기울기 주의] 기준 대비 좌우 기울기 5도 이상.\r\n");
  } else if (tilt_warn && fabsf(tilt_rel) < 4.0f) {
      tilt_warn = false;
      out("[기울기] 경고 해제.\r\n");
  }
}
static bool attitude_valid(uint32_t t) {
  return imu_ok && imu_cal_ok && imu_fresh && t - imu_at <= IMU_FRESH_MS;
}
/* Rep counting: only valid filtered distance, no inertial integration. */
#define MIN_DROP_MM 60
#define RETURN_MM 30
#define MIN_REP_MS 220u
#define MAX_GAP_MS 350u
#define MIN_RANGE_MM 40u
#define MAX_RANGE_MM 1600u
#define CONFIRM_SAMPLES 2u
typedef enum {
  IDLE, ARM_TOP, WAIT_DOWN, WAIT_RETURN, DONE
}
Phase;
typedef enum {
  EV_NONE, EV_ARM, EV_DOWN, EV_REP, EV_DONE, EV_SHORT
}
Event;
typedef struct {
  Phase phase;
  int baseline, lowest;
  unsigned hits, reps, goal;
  uint32_t down_at, low_at, ascent_ms;
}
Counter;
static Counter counter;
static void change_phase(Counter * c, Phase p) {
  c->phase = p;
  c->hits = 0;
}
static bool active(const Counter * c) {
  return c->phase == ARM_TOP || c->phase == WAIT_DOWN || c->phase == WAIT_RETURN;
}
static bool confirm(Counter * c, bool yes) {
  if (!yes) {
      c->hits = 0;
      return false;
  }
  return++c->hits >= CONFIRM_SAMPLES;
}
static void counter_begin(Counter * c, int baseline, unsigned goal) {
  * c = (Counter) {
      .phase = WAIT_DOWN, .baseline = baseline, .lowest = baseline, .goal = goal
  };
}
static bool counter_gap(Counter * c) {
  if (active(c)) {
      bool changed = c->phase != ARM_TOP;
      change_phase(c, ARM_TOP);
      return changed;
  }
  return false;
}
static Event counter_step(Counter * c, int h, uint32_t t) {
  switch (c->phase) {
      case ARM_TOP:
      if (confirm(c, h >= c->baseline - RETURN_MM)) {
          change_phase(c, WAIT_DOWN);
          return EV_ARM;
      }
      break;
      case WAIT_DOWN:
      if (confirm(c, h <= c->baseline - MIN_DROP_MM)) {
          c->down_at = t;
          c->low_at = t;
          c->lowest = h;
          change_phase(c, WAIT_RETURN);
          return EV_DOWN;
      }
      break;
      case WAIT_RETURN:
      if (h < c->lowest - 3) {
          c->lowest = h;
          c->low_at = t;
      }
      if (confirm(c, h >= c->baseline - RETURN_MM)) {
          if (t - c->down_at < MIN_REP_MS) {
              change_phase(c, WAIT_DOWN);
              return EV_SHORT;
          }
          c->ascent_ms = t - c->low_at;
          c->reps++;
          if (c->reps >= c->goal) {
              change_phase(c, DONE);
              return EV_DONE;
          }
          change_phase(c, WAIT_DOWN);
          return EV_REP;
      }
      break;
      default:
      break;
  }
  return EV_NONE;
}
static const char * phase_name(Phase p) {
  switch (p) {
      case ARM_TOP:
      return "시작 높이 복귀 대기";
      case WAIT_DOWN:
      return "하강 대기";
      case WAIT_RETURN:
      return "상단 복귀 대기";
      case DONE:
      return "완료";
      default:
      return "대기";
  }
}
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
      uint16_t t = a;
      a = b;
      b = t;
  }
  if (b > c) {
      uint16_t t = b;
      b = c;
      c = t;
  }
  return a > b ? a: b;
}
static uint16_t dist_samples[3], height, raw_mm, signal_q;
static uint8_t raw_status = 255;
static unsigned sn, si;
static uint32_t last_good, last_result, errors;
static bool valid;
static const char * reason = "새 거리값 대기";
static void invalidate(const char * why) {
  reason = why;
  valid = false;
  sn = si = 0;
  if (counter_gap(&counter)) out("[거리 오류] 진행 중 반복 제외. 시작 높이로 복귀하세요.\r\n");
}
/* Shared wire format: A5 5A type id(LE32) a..g(LE32) CRC16-CCITT(LE16).
* 37 bytes; all peers must use this revision. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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
#define REASON_PEDAL 1u
#define REASON_FSR1 2u
#define U32(v) ((uint32_t)(v))
typedef struct {
  uint8_t type;
  uint32_t id, a, b, c, d, e, f, g;
}
Packet;
typedef struct {
  uint8_t b[FRAME_N];
  unsigned n;
  uint32_t crc_errors, sync_drops;
}
Parser;
static uint16_t crc16(const uint8_t * p, unsigned n) {
  uint16_t c = 0xffff;
  while (n--) {
      c ^= (uint16_t) * p++ << 8;
      for (unsigned j = 0; j < 8; j++) c = (uint16_t)((c & 0x8000u) ? ((uint32_t) c << 1) ^ 0x1021u: (uint32_t) c << 1);
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
  uint32_t w[8] = {
      p->id, p->a, p->b, p->c, p->d, p->e, p->f, p->g
  };
  for (unsigned j = 0; j < 8; j++) put32(b + 3 + 4 * j, w[j]);
  uint16_t c = crc16(b + 2, 33);
  b[35] = (uint8_t) c;
  b[36] = (uint8_t)(c >> 8);
}
static bool parse(Parser * s, uint8_t c, Packet * p) {
  if (!s->n) {
      if (c == 0xa5) s->b[s->n++] = c;
      else s->sync_drops++;
      return false;
  }
  if (s->n == 1 && c != 0x5a) {
      s->sync_drops++;
      s->n = (c == 0xa5) ? 1: 0;
      return false;
  }
  s->b[s->n++] = c;
  if (s->n < FRAME_N) return false;
  uint8_t * b = s->b;
  s->n = 0;
  if (crc16(b + 2, 33) != ((uint16_t) b[35] | (uint16_t) b[36] << 8)) {
      s->crc_errors++;
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
#define F_CAL_BUSY (1u<<17)
#define F_CAL_FAIL (1u<<18)
#define F_CAL_REQUIRED (1u<<19)
#define CAL_NONE 0u
#define CAL_BUSY 1u
#define CAL_READY 2u
#define CAL_FAIL 3u
#define I_CAL 512u
/* STM32F103RB, HSI 8 MHz, register-only. USART1/3 IRQ RX; USART2 debug TX. */
#include <stdio.h>
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
  uint32_t a = port +(n >= 8 ? 4: 0), s = (n % 8) * 4;
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
#define RING_N 2048u
#define RING_MASK (RING_N-1u)
typedef struct {
  volatile uint8_t q[RING_N];
  volatile uint16_t head, tail;
  volatile uint32_t loss;
  uint32_t seen_loss;
}
Ring;
static Ring r1, r3;
static volatile uint32_t dbg_rx, dbg_err, dbg_ovf, dbg_fe, dbg_ne, dbg_ore;
static uint32_t dbg_tx;
static uint32_t dbg_packets, dbg_controls, dbg_req, dbg_flags, dbg_id, dbg_at;
static void irq_rx(uint32_t u, Ring * r) {
  uint32_t s = R(u);
  if (s &(B(5) | 15u)) {
      uint8_t c = (uint8_t) R(u + 4);
      uint16_t next = (uint16_t)((r->head + 1u) & RING_MASK);
      if (u == U1) {
          if (s & B(5)) dbg_rx++;
          if (s & 15u) dbg_err++;
          if (s & B(1)) dbg_fe++;
          if (s & B(2)) dbg_ne++;
          if (s & B(3)) dbg_ore++;
          if ((s & B(5)) && next == r->tail) dbg_ovf++;
      }
      if ((s & 15u) || next == r->tail) r->loss++;
      else if (s & B(5)) {
          r->q[r->head] = c;
          r->head = next;
      }
  }
}
void USART1_IRQHandler(void) {
  irq_rx(U1, &r1);
}
void USART3_IRQHandler(void) {
  irq_rx(U3, &r3);
}
static void uart_init(uint32_t u, unsigned baud) {
  if (u == U1) {
      pin(RA, 9, 11);
      pin_high(RA, 10);
      pin(RA, 10, 8);
  } else if (u == U3) {
      pin(RB, 10, 11);
      pin_high(RB, 11);
      pin(RB, 11, 8);
  } else {
      pin(RA, 2, 11);
      pin(RA, 3, 4);
  }
  /* Explicit reset of UART settings, then 8N1, no flow control. */
  uint32_t rr = u == U1 ? 0x4002100cu: 0x40021010u;
  uint32_t mask = u == U1 ? B(14): u == U3 ? B(18): B(17);
  R(rr) |= mask;
  R(rr) &= ~mask;
  R(u + 0xc) = 0;
  R(u + 0x10) = 0;
  R(u + 0x14) = 0;
  R(u + 8) = (8000000u + baud / 2) / baud;
  R(u + 0xc) = B(13) | B(3) | B(2) | (u != U2 ? B(5): 0);
  if (u == U1) R(0xe000e104) = B(5);
  if (u == U3) R(0xe000e104) = B(7);
}
static bool take(Ring * r, Parser * s, Packet * p) {
  for (;;) {
      uint32_t loss = r->loss;
      if (r->seen_loss != loss) {
          r->seen_loss = loss;
          s->n = 0;
          r->tail = r->head;
          continue;
      }
      if (r->tail == r->head) return false;
      uint8_t c = r->q[r->tail];
      r->tail = (uint16_t)((r->tail + 1u) & RING_MASK);
      if (parse(s, c, p) && r->loss == loss) return true;
  }
}
/* ---- USART1 TX DMA (STM1 <-> STM2): DMA1 Channel 4, normal mode ----
* Eight 37-byte packets are buffered so consecutive MEASURE/CAL_STATUS
* messages cannot overwrite an in-flight DMA transfer.
* USART1 RX remains interrupt-driven; all other UARTs are unchanged.
* The DMA IRQ advances the queue; the main loop never waits for USART1 TXE.
*/
#define U1_DMA_BASE       0x40020000u
#define U1_DMA_CH4_CCR    (U1_DMA_BASE + 0x44u)
#define U1_DMA_SLOTS      8u
static uint8_t u1_dma_frames[U1_DMA_SLOTS][FRAME_N];
static volatile uint8_t u1_dma_head, u1_dma_tail, u1_dma_pending, u1_dma_busy;
static volatile uint32_t u1_dma_queued, u1_dma_completed;
static volatile uint32_t u1_dma_overflow, u1_dma_errors;
static uint32_t u1_irq_lock(void) {
   uint32_t previous;
   __asm__ volatile ("mrs %0, primask" : "=r" (previous) :: "memory");
   __asm__ volatile ("cpsid i" ::: "memory");
   return previous;
}
static void u1_irq_unlock(uint32_t previous) {
   __asm__ volatile ("msr primask, %0" :: "r" (previous) : "memory");
}
/* Call with interrupts masked or from DMA1_Channel4_IRQHandler. */
static void u1_dma_start_locked(void) {
   if (u1_dma_busy || u1_dma_pending == 0u) return;
   R(U1_DMA_CH4_CCR) &= ~B(0);  /* DMA channel must be off for reprogramming */
   R(U1_DMA_BASE + 0x04u) = (0xFu << 12); /* IFCR: clear CH4 GIF/TC/HT/TE */
   R(U1_DMA_CH4_CCR + 0x04u) = FRAME_N; /* CNDTR4 */
   R(U1_DMA_CH4_CCR + 0x08u) = U1 + 4u; /* CPAR4 = USART1_DR */
   R(U1_DMA_CH4_CCR + 0x0cu) =
       (uint32_t)(uintptr_t)&u1_dma_frames[u1_dma_tail][0]; /* CMAR4 */
   u1_dma_busy = 1u;
   /* TCIE, TEIE, DIR=memory->peripheral, MINC, PL=high, EN;
    * MSIZE=PSIZE=8-bit; normal (not circular) transfer. */
   R(U1_DMA_CH4_CCR) = B(13) | B(12) | B(7) | B(4) | B(3) | B(1) | B(0);
}
static bool u1_dma_enqueue(const uint8_t bytes[FRAME_N]) {
   uint32_t irq_state = u1_irq_lock();
   if (u1_dma_pending == U1_DMA_SLOTS) {
       u1_dma_overflow++;
       u1_irq_unlock(irq_state);
       return false;
   }
   for (unsigned i = 0; i < FRAME_N; i++)
       u1_dma_frames[u1_dma_head][i] = bytes[i];
   u1_dma_head = (uint8_t)((u1_dma_head + 1u) % U1_DMA_SLOTS);
   u1_dma_pending++;
   u1_dma_queued++;
   u1_dma_start_locked();
   u1_irq_unlock(irq_state);
   return true;
}
void DMA1_Channel4_IRQHandler(void) {
   uint32_t status = R(U1_DMA_BASE); /* DMA1_ISR */
   R(U1_DMA_BASE + 0x04u) = (0xFu << 12); /* DMA1_IFCR */
   if ((status & (B(13) | B(15))) == 0u) return;
   R(U1_DMA_CH4_CCR) &= ~B(0);
   if (status & B(15)) u1_dma_errors++;
   if (status & B(13)) u1_dma_completed++;
   if (u1_dma_busy && u1_dma_pending) {
       u1_dma_tail = (uint8_t)((u1_dma_tail + 1u) % U1_DMA_SLOTS);
       u1_dma_pending--;
   }
   u1_dma_busy = 0u;
   u1_dma_start_locked();
}
static void u1_dma_init(void) {
   R(0x40021014u) |= B(0);  /* RCC_AHBENR.DMA1EN */
   R(U1_DMA_CH4_CCR) = 0u;
   R(U1_DMA_BASE + 0x04u) = (0xFu << 12);
   R(U1 + 0x14u) |= B(7); /* USART1_CR3.DMAT */
   R(0xE000E100u) = B(14); /* DMA1_Channel4_IRQn = 14 */
}
static void sendp(uint32_t u, const Packet * p) {
   uint8_t b[FRAME_N];
   encode(p, b);
   if (u == U1) {
       if (u1_dma_enqueue(b)) dbg_tx++; /* queued, not necessarily transmitted */
       return;
   }
   /* Other USARTs: preserve original blocking behavior. */
   for (unsigned i = 0; i < FRAME_N; i++) {
       while (!(R(u) & B(7))) { }
       R(u + 4) = b[i];
   }
}
static void logstr(const char * s) {
  while (*s) {
      while (!(R(U2) & B(7))) {
      }
      R(U2 + 4) = (uint8_t) * s++;
  }
}
/* STM1 integrated: rack control from STM2, raw ToF counts, independent MPU tilt.
* Rack calibration supplies the rep baseline; unracking starts counting immediately. */
static uint32_t sid, control_at, report_at;
static uint32_t cal_token, cal_state, cal_attempt_at, cal_dist_sum, cal_dist_n;
static uint16_t rack_height, cal_dist_min, cal_dist_max;
static bool allowed_rack, cal_dist_fault;
static bool cal_completed_once;
#define RACK_CAL_SPREAD_MM 45u
#define RACK_CAL_TIMEOUT_MS 6000u
static bool controlled, session, finalized, assisted, contact, pause_count, stat_fault, rep_tilt_valid, base_saved;
static uint32_t sum_up, first_up, last_up, max_tilt, last_drop, rep_peak;
static bool rep_quality = true;
static bool tof_ok;
static uint32_t recovery_at, tof_attempts, tof_recoveries, tof_bus_streak;
#define TOF_RETRY_MS 2000u
#define TOF_STALL_MS 1200u
#define TOF_BUS_RETRY_N 3u
static Parser control_parser;
static uint32_t idle_count;
static uint32_t last_peak;
static void clear_stats(void) {
  counter = (Counter) {
      .phase = IDLE
  };
  sum_up = first_up = last_up = max_tilt = last_drop = rep_peak = 0;
  stat_fault = false;
  base_saved = false;
  rep_tilt_valid = false;
  last_peak = 0;
  rep_quality = true;
}
static bool cal_ready(uint32_t token, uint32_t t) {
  return token && cal_token == token && cal_state == CAL_READY && attitude_valid(t) && valid && t - last_good <= MAX_GAP_MS;
}
static void cal_fail(const char * why) {
  imu_calibrating = false;
  imu_cal_ok = false;
  imu_fresh = false;
  cal_state = CAL_FAIL;
  out(why);
}
static void cal_begin(uint32_t token, uint32_t t) {
  cal_token = token;
  cal_state = CAL_BUSY;
  cal_attempt_at = t;
  rack_height = 0;
  cal_dist_sum = cal_dist_n = 0;
  cal_dist_min = 65535;
  cal_dist_max = 0;
  cal_dist_fault = false;
  sn = si = 0;
  valid = false;
  if (!tof_ok || !imu_ok) {
      cal_fail("[보정 실패] 센서 초기화 확인.\r\n");
      return;
  }
  out("[거치 보정] 기울기와 거리 기준 동시 저장. 움직이지 마세요.\r\n");
  imu_cal_begin(t);
}
static void cal_distance(uint16_t h) {
  if (cal_state != CAL_BUSY) return;
  if (h < cal_dist_min) cal_dist_min = h;
  if (h > cal_dist_max) cal_dist_max = h;
  cal_dist_sum += h;
  cal_dist_n++;
  if ((unsigned)(cal_dist_max - cal_dist_min) > RACK_CAL_SPREAD_MM) cal_dist_fault = true;
}
static void cal_task(uint32_t t) {
  if (cal_state != CAL_BUSY) return;
  if (!allowed_rack || t - control_at > 500u) {
      cal_fail("[보정 실패] 거치 해제 또는 통신 두절.\r\n");
      return;
  }
  if (cal_dist_fault || t - cal_attempt_at > RACK_CAL_TIMEOUT_MS) {
      cal_fail("[보정 실패] 거리 변동/누락 또는 시간 초과.\r\n");
      return;
  }
  if (imu_calibrating) return;
  if (!attitude_valid(t) || !valid || t - last_good > MAX_GAP_MS || cal_dist_n < 5u) {
      cal_fail("[보정 실패] MPU/ToF 정지 기준 미확립.\r\n");
      return;
  }
  rack_height = (uint16_t)((cal_dist_sum + cal_dist_n / 2) / cal_dist_n);
  if (rack_height < MIN_RANGE_MM + MIN_DROP_MM || rack_height > MAX_RANGE_MM) {
      cal_fail("[보정 실패] 거치 높이에서 반복 측정 범위를 확보할 수 없습니다.\r\n");
      return;
  }
  cal_state = CAL_READY;
  cal_completed_once = true;
  snprintf(msg, sizeof msg, "[보정 완료] 거치 기준 %u mm | 들면 즉시 측정\r\n", rack_height);
  out(msg);
}
static void report(void) {
  uint32_t t = now_ms();
  uint32_t f = (attitude_valid(t) ? M_CAL | M_TILT: 0) | (valid && t - last_good <= MAX_GAP_MS ? M_DISTANCE: 0) | (session
      ? M_ACTIVE: 0) | (finalized ? M_FINAL: 0) | (base_saved ? M_BASE: 0) | (stat_fault ? M_FAULT: 0) | (tilt_warn ? M_WARN: 0)
      | (rep_tilt_valid ? M_REP_VALID: 0);
  Packet p = {
      T_MEASURE, sid, f, counter.reps, sum_up, first_up, last_up, max_tilt, (last_drop & 0xffffu) | ((last_peak & 0xffffu) << 16)
  };
  sendp(U1, &p);
  uint32_t state = cal_state;
  if (state == CAL_READY && !cal_ready(cal_token, t)) state = CAL_FAIL;
  Packet cp = {
      T_CAL_STATUS, sid, cal_token, state, rack_height, 0, 0, 0, 0
  };
  sendp(U1, &cp);
  report_at = now_ms();
}
static void on_control(Packet p) {
  uint32_t t = now_ms();
  dbg_packets++;
  if (p.type != T_CONTROL) return;
  dbg_controls++;
  dbg_req = p.c;
  dbg_flags = p.a;
  dbg_id = p.id;
  if (p.id < sid) return;
  controlled = true;
  control_at = t;
  bool rack = (p.a & F_RACKED) != 0;
  contact = (p.a & F_CONTACT) != 0;
  bool running = (p.a & F_ACTIVE) != 0, freeze = (p.a & F_ASSIST) != 0;
  allowed_rack = rack && contact && !running;
  if (running && p.id && p.id > sid) {
      bool ready = !(p.a & F_CAL_FAIL) && cal_ready(p.c, t);
      /* exact calibration generation, not just IMU alive */
      sid = p.id;
      clear_stats();
      session = true;
      finalized = false;
      assisted = false;
      pause_count = false;
      if (imu_calibrating) cal_fail("[보정 중단] 준비 완료 전 언랙.\r\n");
      if (ready) {
          /* Use the completed J-cup calibration, with no post-unrack capture. */
          counter_begin(&counter, rack_height, 65535);
          base_saved = true;
          out("[세트 시작] 거치 기준으로 즉시 측정합니다.\r\n");
          report();
      } else {
          stat_fault = true;
          change_phase(&counter, IDLE);
          out("[세트 시작] 보정 미완료. 이번 세트 측정 보류. 재거치하세요.\r\n");
      }
  }
  if (session && p.id == sid) {
      if (freeze) {
          assisted = true;
          change_phase(&counter, IDLE);
      }
      if (!running) {
          if (!base_saved) stat_fault = true;
          session = false;
          finalized = true;
          change_phase(&counter, IDLE);
          out("[세트 종료] 최종 기록 전송\r\n");
          report();
      } else if (contact && !pause_count) {
          pause_count = true;
          counter_gap(&counter);
      } else if (!contact && pause_count) {
          pause_count = false;
      }
  }
  if (!session && !sensor_recovering) {
      if (cal_state == CAL_BUSY && !allowed_rack) cal_fail("[보정 중단] 안정 거치 필요.\r\n");
      if (!cal_completed_once && allowed_rack && p.c && p.c != cal_token) cal_begin(p.c, t);
      else if (allowed_rack && p.c && p.c == cal_token && cal_state == CAL_READY && !cal_ready(p.c,
          t)) cal_fail("[보정 실패] 센서 데이터 단절.\r\n");
      /* Retry only the initial calibration. After success, faults require a
       * restart; do not silently replace the stored reference between sets. */
      if (!cal_completed_once && allowed_rack && p.c && p.c == cal_token && cal_state == CAL_FAIL && tof_ok && imu_ok && valid
          && t - last_good <= MAX_GAP_MS && t - cal_attempt_at >= 3000u) cal_begin(p.c, t);
  }
}
/* Service UART between I2C register operations and during bounded waits.
* on_control never enters user calibration while recovery is in progress.
* Report invalid sensor state, not a cached READY, until fresh samples return. */
static void service_recovery_link(void) {
  Packet p;
  while (take(&r1, &control_parser, &p)) on_control(p);
  if (now_ms() - report_at >= 100u) report();
}
static bool recovery_interrupted(void) {
  return session || (controlled && ((dbg_flags & F_ACTIVE) != 0 || now_ms() - control_at > 500u));
}
static bool recovery_allowed(uint32_t t) {
  return !session && (!controlled || (!(dbg_flags & F_ACTIVE) && t - control_at <= 500u));
}
static void tof_mark_fault(const char * why) {
  tof_ok = false;
  raw_mm = signal_q = height = 0;
  raw_status = 255;
  invalidate(why);
  if (session) stat_fault = true;
  if (cal_state == CAL_BUSY || cal_state == CAL_READY) {
      cal_attempt_at = now_ms();
      cal_fail("[보정 실패] 거리센서 통신 단절. 재거치 후 복구합니다.\r\n");
  }
}
static void try_tof_recovery(bool first) {
  if (!recovery_allowed(now_ms())) return;
  sensor_recovering = true;
  tof_ok = false;
  valid = false;
  sn = si = 0;
  if (cal_state == CAL_READY || cal_state == CAL_BUSY) cal_fail("[보정 중단] ToF 재초기화\r\n");
  ++tof_attempts;
  tof_stage = 0;
  diag_reg = 0;
  diag_why = "주소/리셋 확인";
  snprintf(msg, sizeof msg, "[ToF RECOVERY] attempt=%lu mode=%s\r\n", (unsigned long) tof_attempts,
      TOF_USE_XSHUT ? "XSHUT": "I2C SW RESET");
  out(msg);
  service_recovery_link();
  bool bus_ok = !recovery_interrupted() && (first ? scan(): recover_bus());
#if TOF_USE_XSHUT
  /* Hardware reset may also free a stuck bus; do not require prior scan. */
  bool reset_ok = !recovery_interrupted() && tof_reset_device();
  bus_ok = reset_ok && recover_bus();
#else
  bool reset_ok = bus_ok && !recovery_interrupted() && tof_reset_device();
#endif
  bool ok = bus_ok && reset_ok && !recovery_interrupted() && tof_init();
  if (recovery_interrupted()) ok = false;
  tof_ok = ok;
  tof_bus_streak = 0;
  raw_mm = signal_q = height = 0;
  raw_status = 255;
  sn = si = 0;
  valid = false;
  last_result = now_ms();
  reason = ok ? "새 거리값 대기": "ToF 초기화/복구 실패";
  if (ok) {
      ++tof_recoveries;
      out("[ToF 초기화 성공] 거리 측정 재개\r\n");
  } else {
      snprintf(msg, sizeof msg, "[ToF 초기화 실패] 단계 %u 레지스터 %02x %s\r\n", tof_stage, diag_reg, diag_why);
      out(msg);
  }
  sensor_recovering = false;
  recovery_at = now_ms();
}
static void measure_distance(void) {
  if (!tof_ok) return;
  uint16_t mm = 0, sig = 0;
  uint8_t st = 255;
  int r = tof_poll(&mm, &st, &sig);
  uint32_t t = now_ms();
  if (r < 0) {
      if (cal_state == CAL_BUSY) cal_dist_fault = true;
      errors++;
      invalidate("I2C 통신 실패");
      if (session) stat_fault = true;
      if (++tof_bus_streak >= TOF_BUS_RETRY_N) tof_mark_fault("ToF I2C 연속 오류");
      return;
  }
  tof_bus_streak = 0;
  if (!r) {
      if (t - last_result >= TOF_STALL_MS) {
          tof_mark_fault("ToF 새 결과 타임아웃");
          return;
      }
      if (t - last_result > MAX_GAP_MS) {
          if (cal_state == CAL_BUSY && t - cal_attempt_at > MAX_GAP_MS) cal_dist_fault = true;
      }
      if (t - last_result > MAX_GAP_MS && (valid || sn)) {
          invalidate("새 거리값 없음");
          if (session) stat_fault = true;
      }
      return;
  }
  last_result = t;
  raw_mm = mm;
  raw_status = st;
  signal_q = sig;
  if (st != 11u || mm < MIN_RANGE_MM || mm > MAX_RANGE_MM || !sig) {
      if (cal_state == CAL_BUSY) cal_dist_fault = true;
      errors++;
      invalidate("거리/원시 상태/반사 신호 확인");
      if (session) stat_fault = true;
      return;
  }
  if (sn && t - last_good > MAX_GAP_MS) {
      if (cal_state == CAL_BUSY) cal_dist_fault = true;
      invalidate("측정 간격 초과");
      if (session) stat_fault = true;
  }
  last_good = t;
  dist_samples[si] = mm;
  si = (si + 1u) % 3u;
  if (sn < 3u) sn++;
  if (sn < 3) return;
  height = median3(dist_samples[0], dist_samples[1], dist_samples[2]);
  valid = true;
  reason = "정상";
  cal_distance(height);
  if (!session || assisted || pause_count) return;
  if (!active(&counter)) return;
  Event ev = counter_step(&counter, height, t);
  if (ev == EV_DOWN) {
      rep_peak = attitude_valid(t) ? (uint32_t) round_i(fabsf(tilt_rel) * 10): 0;
      rep_quality = attitude_valid(t);
      out("[하강 인정]\r\n");
  }
  if (counter.phase == WAIT_RETURN || ev == EV_REP || ev == EV_DONE) {
      if (attitude_valid(t)) {
          uint32_t mag = (uint32_t) round_i(fabsf(tilt_rel) * 10);
          if (mag > rep_peak) rep_peak = mag;
      } else rep_quality = false;
  }
  if (ev == EV_REP || ev == EV_DONE) {
      last_up = counter.ascent_ms;
      sum_up += last_up;
      if (counter.reps <= 2) first_up += last_up;
      last_drop = (uint32_t)(counter.baseline - counter.lowest);
      last_peak = rep_peak;
      rep_tilt_valid = rep_quality;
      if (!rep_quality) stat_fault = true;
      if (rep_peak > max_tilt) max_tilt = rep_peak;
      snprintf(msg, sizeof msg, "[횟수] %u회 | 상승 %lu ms | 하강 %lu mm | 최대 기울기 %lu.%lu도\r\n", counter.reps, (unsigned long) last_up,
          (unsigned long) last_drop, (unsigned long)(rep_peak / 10), (unsigned long)(rep_peak % 10));
      out(msg);
      report();
  }
}
/* V3 UART diagnostics: same wire format, sensor and motion code. */
static void uart_debug(uint32_t t) {
  if (t - dbg_at < 1000u) return;
  dbg_at = t;
  const char * cs = cal_state == CAL_BUSY ? "BUSY": cal_state == CAL_READY ? "READY": cal_state == CAL_FAIL ? "FAIL": "NONE";
  snprintf(msg, sizeof msg,
       "[RXDBG] RX=%lu ERR=%lu OVF=%lu PKT=%lu CTRL=%lu LINK=%s\r\nREQ=%lu TOKEN=%lu RACK=%u CONTACT=%u ACTIVE=%u ID=%lu/%lu CAL=%s\r\n",
       (unsigned long) dbg_rx, (unsigned long) dbg_err, (unsigned long) dbg_ovf, (unsigned long) dbg_packets,
       (unsigned long) dbg_controls, controlled && t - control_at <= 500u ? "OK": "LOST", (unsigned long) dbg_req, (unsigned long) cal_token,
       (unsigned)((dbg_flags & F_RACKED) != 0), (unsigned)((dbg_flags & F_CONTACT) != 0), (unsigned)((dbg_flags & F_ACTIVE) != 0), (unsigned long) dbg_id,
      (unsigned long) sid, cs);
  out(msg);
  snprintf(msg, sizeof msg, "[U1] TX=%lu CRC=%lu SYNC=%lu FE=%lu NE=%lu ORE=%lu PIN=%u\r\n", (unsigned long) dbg_tx,
      (unsigned long) control_parser.crc_errors, (unsigned long) control_parser.sync_drops, (unsigned long) dbg_fe,
      (unsigned long) dbg_ne, (unsigned long) dbg_ore, (unsigned)((R(RA + 8) & B(10)) != 0));
  out(msg);
  snprintf(msg, sizeof msg,
      "[U1 DMA TX] queued=%lu done=%lu pending=%u busy=%u drops=%lu errors=%lu\r\n",
      (unsigned long)u1_dma_queued, (unsigned long)u1_dma_completed,
      (unsigned)u1_dma_pending, (unsigned)u1_dma_busy,
      (unsigned long)u1_dma_overflow, (unsigned long)u1_dma_errors);
  out(msg);
  const char * why = !controlled || t - control_at > 500u ? "NO_CONTROL": session ? "ACTIVE": !dbg_req
      ? "NO_REQUEST": !(dbg_flags & F_RACKED) ? "NOT_RACKED": !(dbg_flags & F_CONTACT) ? "NO_CONTACT": (dbg_flags & F_ACTIVE)
      ? "UNRACKED": !tof_ok || !imu_ok ? "SENSOR_INIT_FAIL": cal_state == CAL_BUSY ? "CAL_BUSY": cal_state == CAL_FAIL
      ? "CAL_FAIL": cal_state == CAL_READY ? "READY": "WAIT_CAL";
  snprintf(msg, sizeof msg, "[CAL] REASON=%s | samples=%lu spread=%u mm\r\n", why, (unsigned long) cal_dist_n,
      cal_dist_n ? (unsigned)(cal_dist_max - cal_dist_min): 0u);
  out(msg);
  snprintf(msg, sizeof msg, "[SENSOR] TOF_INIT=%u MPU_INIT=%u DIST_VALID=%u RAW=%u STATUS=%u SIGNAL=%u | %s\r\n", (unsigned) tof_ok,
      (unsigned) imu_ok, (unsigned)(valid && t - last_good <= MAX_GAP_MS), (unsigned) raw_mm, (unsigned) raw_status,
      (unsigned) signal_q, reason);
  out(msg);
  snprintf(msg, sizeof msg, "[RECOVERY] attempts=%lu success=%lu retry=%s | RXBUF=%u/%u\r\n", (unsigned long) tof_attempts,
      (unsigned long) tof_recoveries, tof_ok ? "NOT_NEEDED": recovery_allowed(t) ? "WAIT/RETRY": "BLOCKED_ACTIVE_OR_LINK",
      (unsigned)((r1.head - r1.tail) & RING_MASK), (unsigned)(RING_N - 1u));
  out(msg);
}
int main(void) {
  base_init();
  platform_init();
  uart_init(U1, 115200);
  u1_dma_init();
  out("\r\n[STM1] PRJ2 DEBUG V4 / V2 PACKET 37B / UART1 TX DMA CH4\r\n[STM1] ToF + MPU6050 / 랙 연동 / AUTO RECOVERY\r\n");
  /* Receive fresh controls even while sensor power stabilizes. */
  sensor_recovering = true;
  delay_ms(300);
  sensor_recovering = false;
  try_tof_recovery(true);
  sensor_recovering = true;
  imu_ok = mpu_init();
  sensor_recovering = false;
  out(imu_ok ? "[MPU 연결 성공]\r\n": "[MPU 연결 실패]\r\n");
  uint32_t poll = now_ms(), imu = poll, print = poll;
  last_result = poll;
  recovery_at = poll;
  for (;;) {
      Packet p;
      while (take(&r1, &control_parser, &p)) on_control(p);
      uint32_t t = now_ms();
      if (controlled && t - control_at > 1000u && session) {
          stat_fault = true;
          counter_gap(&counter);
          pause_count = true;
      }
      if (controlled && t - control_at > 500u) allowed_rack = false;
      if (!tof_ok && t - recovery_at >= TOF_RETRY_MS && recovery_allowed(t)) try_tof_recovery(false);
      t = now_ms();
      if (t - imu >= IMU_PERIOD_MS) {
          imu = t;
          imu_update(t);
          if (session && !assisted && base_saved) {
              if (!attitude_valid(now_ms())) stat_fault = true;
              else {
                  uint32_t a = (uint32_t) round_i(fabsf(tilt_rel) * 10);
                  if (a > max_tilt) max_tilt = a;
              }
          }
      }
      t = now_ms();
      if (t - poll >= 10) {
          poll = t;
          measure_distance();
      }
      t = now_ms();
      cal_task(t);
      if (t - report_at >= 100) report();
      if (t - print >= 500) {
          print = t;
          snprintf(msg, sizeof msg, "[측정] 세트 %lu | %u회 | 높이 %u mm | 기울기(0.1도) %d | %s | %s\r\n", (unsigned long) sid, counter.reps,
              height, round_i(tilt_rel * 10), phase_name(counter.phase), valid ? "거리 정상": "거리 오류");
          out(msg);
          idle_count++;
      }
      uart_debug(now_ms());
      delay_ms(1);
  }
}


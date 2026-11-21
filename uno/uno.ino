// Arduino UNO R3 (ATmega328P @ 16MHz) - Pure Register-Level Firmware (Arduino IDE Compatible)
// - Uses Hardware TWI (I2C) registers for 16x2 PCF8574 LCD.
// - Uses PCINT0 (D10/PB2) + Timer2 (9600 bps) for STM3 37-byte packet link.
// - Uses Timer1 (1 ms tick) to avoid Timer0 vector collision in Arduino IDE.
// - Open this uno/uno.ino sketch with board "Arduino Uno" (ATmega328P, 16 MHz).
// - Use the accompanying STM2/STM3 files for exact summary weights and errors.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "Select Arduino Uno (ATmega328P at 16 MHz)."
#endif

/* Shared wire format: A5 5A type id(LE32) a..g(LE32) CRC16-CCITT(LE16). - 37 bytes */
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
#define F_SESSION_END (1UL<<30)
#define F_MODEL_ENABLED 2048u
#define F_CONTACT 4096u
#define F_M_READY 8192u
#define F_M_TILT 16384u
#define F_M_BASE 32768u
#define F_M_WARN 65536UL
#define F_CAL_BUSY (1UL<<17)
#define F_CAL_FAIL (1UL<<18)
#define F_CAL_REQUIRED (1UL<<19)

#define I_WEIGHT 1u
#define I_IMU 2u
#define I_MODEL 4u
#define I_LINK 8u
#define I_FSR 16u
#define I_FINAL 32u
#define I_DISTANCE 64u
#define I_ASSIST 128u
#define I_BASE 256u
#define I_CAL 512u
#define SUMMARY_INVALID_SHIFT 20u
#define SUMMARY_INVALID_MASK 0x3ffUL

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

/* ================================================================
 * 1. Timer1 (1 ms System Tick) & Microsecond Busy-Wait
 * ================================================================ */
static volatile uint32_t timer1_ms = 0;

ISR(TIMER1_COMPA_vect) {
    timer1_ms++;
}

static void timer1_init(void) {
    cli();
    TIMSK0 = 0;
    TCCR0A = 0;
    TCCR0B = 0;

    /* Timer1 CTC mode: 16 MHz / 64 = 250 kHz -> OCR1A = 249 (1.000 ms) */
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    TCNT1  = 0;
    OCR1A  = 249;
    TIFR1  = (1 << OCF1A);
    TIMSK1 = (1 << OCIE1A);
}

static uint32_t sys_ms(void) {
    uint32_t t;
    uint8_t sreg = SREG;
    cli();
    t = timer1_ms;
    SREG = sreg;
    return t;
}

static void delay_us_reg(uint16_t us) {
    while (us--) {
        __asm__ __volatile__ (
            "nop\n\t nop\n\t nop\n\t nop\n\t"
            "nop\n\t nop\n\t nop\n\t nop\n\t"
            "nop\n\t nop\n\t nop\n\t nop\n\t"
        );
    }
}

static void delay_ms_reg(uint16_t ms) {
    uint32_t start = sys_ms();
    while ((uint32_t)(sys_ms() - start) < ms) { }
}

/* ================================================================
 * 2. Hardware USART0 (PD0/PD1, 115200 bps) for PC Debug Output
 * ================================================================ */
static void uart0_init(void) {
    UCSR0A = (1 << U2X0);
    UBRR0H = 0;
    UBRR0L = 16; /* 115200 bps @ 16 MHz */
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart0_puts(const char *s) {
    while (*s) {
        while (!(UCSR0A & (1 << UDRE0))) { }
        UDR0 = (uint8_t)*s++;
    }
}

/* ================================================================
 * 3. Register-Level 9600 bps UART on D10 (PB2 RX) & D11 (PB3 TX)
 * ================================================================ */
#define RX_RING_N 256u
static volatile uint8_t rx_q[RX_RING_N];
static volatile uint8_t rx_head = 0, rx_tail = 0;

static volatile uint8_t rx_state = 0; /* 0: idle, 1..8: data bits, 9: stop bit */
static volatile uint8_t rx_shift = 0;
static volatile bool rx_error = false;

static void sw_uart_init(void) {
    /* D11 (PB3) TX: Output High */
    DDRB  |= (1 << DDB3);
    PORTB |= (1 << PORTB3);

    /* D10 (PB2) RX: Input with Pull-up */
    DDRB  &= ~(1 << DDB2);
    PORTB |= (1 << PORTB2);

    /* Timer2: CTC mode */
    TCCR2A = (1 << WGM21);
    TCCR2B = 0;
    OCR2A  = 51;
    TIMSK2 = 0;

    /* Enable Pin Change Interrupt on PB2 (PCINT2) */
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT2);
}

ISR(PCINT0_vect) {
    if (rx_state == 0 && !(PINB & (1 << PINB2))) {
        rx_state = 1;
        rx_shift = 0;
        TCCR2B = 0;
        TCNT2  = 0;
        OCR2A  = 74;  /* 150 us plus ISR entry latency: center of bit 0. */
        TIFR2  = (1 << OCF2A);
        TCCR2B = (1 << CS21) | (1 << CS20); /* Prescaler 32 -> 2.0 us/tick */
        TIMSK2 = (1 << OCIE2A);
        PCMSK0 &= ~(1 << PCINT2);
    }
}

ISR(TIMER2_COMPA_vect) {
    if (rx_state >= 1 && rx_state <= 8) {
        OCR2A = 51; /* 52 * 2.0 us = 104.0 us interval */
        if (PINB & (1 << PINB2)) {
            rx_shift |= (1 << (rx_state - 1));
        }
        rx_state++;
    } else if (rx_state == 9) {
        uint8_t next = (uint8_t)(rx_head + 1u);
        if ((PINB & (1 << PINB2)) && next != rx_tail) {
            rx_q[rx_head] = rx_shift;
            rx_head = next;
        } else rx_error = true; /* Bad stop bit or ring overflow. */
        rx_state = 0;
        TIMSK2 = 0;
        TCCR2B = 0;
        PCIFR  = (1 << PCIF0);
        PCMSK0 |= (1 << PCINT2);
    }
}

static bool sw_uart_getc(uint8_t *out) {
    if (rx_head == rx_tail) return false;
    *out = rx_q[rx_tail];
    rx_tail = (uint8_t)(rx_tail + 1u);
    return true;
}

/* IRQs are masked during TX; poll each millisecond flag before it can repeat.
 * ATmega328P datasheet: TIFR1/OCF1A is a flag, not an accumulated tick count.
 * https://docs.arduino.cc/resources/datasheets/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
 */
static inline void sw_uart_wait_bit(void) {
    while (!(TIFR2 & (1 << OCF2A))) {
        if (TIFR1 & (1 << OCF1A)) {
            TIFR1 = (1 << OCF1A);
            timer1_ms++;
        }
    }
}

static void sw_uart_write_frame(const uint8_t *buf, uint8_t len) {
    uint8_t sreg = SREG;
    cli();
    uint8_t rx_mask = PCMSK0;
    PCMSK0 &= ~(1 << PCINT2);
    TIMSK2 = 0;
    TCCR2B = (1 << CS21) | (1 << CS20);
    OCR2A  = 51;
    for (uint8_t idx = 0; idx < len; idx++) {
        uint8_t b = buf[idx];
        PORTB &= ~(1 << PORTB3);
        TCNT2 = 0; TIFR2 = (1 << OCF2A);
        sw_uart_wait_bit();

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (b & (1u << bit)) PORTB |= (1 << PORTB3);
            else PORTB &= ~(1 << PORTB3);
            TCNT2 = 0; TIFR2 = (1 << OCF2A);
            sw_uart_wait_bit();
        }

        PORTB |= (1 << PORTB3);
        TCNT2 = 0; TIFR2 = (1 << OCF2A);
        sw_uart_wait_bit();
    }
    TCCR2B = 0;
    rx_state = 0;
    PCIFR = (1 << PCIF0);
    PCMSK0 = rx_mask | (1 << PCINT2);
    SREG = sreg;
}

/* ================================================================
 * 4. Hardware TWI (I2C) Master Registers for PCF8574 16x2 LCD
 * ================================================================ */
static void twi_init(void) {
    PORTC |= (1 << PORTC4) | (1 << PORTC5);
    TWSR = 0x00;
    TWBR = 72; /* 100 kHz SCL */
    TWCR = (1 << TWEN);
}

static bool twi_wait(void) {
    uint16_t guard = 2000;
    while (!(TWCR & (1 << TWINT))) {
        if (--guard == 0) return false;
    }
    return true;
}

static bool twi_start_addr(uint8_t addr_rw) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_wait()) return false;
    uint8_t st = TWSR & 0xF8;
    if (st != 0x08 && st != 0x10) return false;

    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return false;
    st = TWSR & 0xF8;
    return (st == 0x18 || st == 0x40);
}

static bool twi_write_byte(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return false;
    return ((TWSR & 0xF8) == 0x28);
}

static void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint16_t guard = 2000;
    while (TWCR & (1 << TWSTO)) {
        if (--guard == 0) {
            TWCR = 0;
            TWCR = (1 << TWEN);
            return;
        }
    }
}

static uint8_t lcd_addr = 0;
static const uint8_t LCD_COLS = 16, LCD_ROWS = 2;
static char cache[LCD_ROWS][LCD_COLS + 1];

static bool ack(uint8_t a) {
    bool ok = twi_start_addr((uint8_t)(a << 1));
    twi_stop();
    return ok;
}

static void io_lcd(uint8_t v) {
    if (!lcd_addr) return;
    if (twi_start_addr((uint8_t)(lcd_addr << 1))) {
        twi_write_byte((uint8_t)(v | 0x08));
    }
    twi_stop();
}

static void nibble(uint8_t v, bool rs) {
    uint8_t x = (v & 0xF0) | (rs ? 1u : 0u);
    io_lcd(x | 0x04);
    delay_us_reg(2);
    io_lcd(x);
}

static void lcd_byte(uint8_t v, bool rs) {
    nibble(v, rs);
    nibble((uint8_t)(v << 4), rs);
    delay_us_reg(45);
}

static void lcd_init(void) {
    if (ack(0x27)) lcd_addr = 0x27;
    else if (ack(0x3F)) lcd_addr = 0x3F;
    else {
        for (uint8_t a = 0x20; a <= 0x27 && !lcd_addr; a++) if (ack(a)) lcd_addr = a;
        for (uint8_t a = 0x38; a <= 0x3F && !lcd_addr; a++) if (ack(a)) lcd_addr = a;
    }
    if (!lcd_addr) {
        uart0_puts("LCD not found\r\n");
        return;
    }
    delay_ms_reg(50);
    nibble(0x30, false); delay_ms_reg(5);
    nibble(0x30, false); delay_ms_reg(5);
    nibble(0x30, false); delay_us_reg(150);
    nibble(0x20, false);
    lcd_byte(0x28, false);
    lcd_byte(0x0C, false);
    lcd_byte(0x06, false);
    lcd_byte(0x01, false);
    delay_ms_reg(2);
    memset(cache, 0, sizeof cache);
}

static void line(uint8_t r, const char *s) {
    if (r >= LCD_ROWS) return;
    char b[LCD_COLS + 1];
    memset(b, ' ', LCD_COLS);
    b[LCD_COLS] = 0;
    for (uint8_t i = 0; i < LCD_COLS && s[i]; i++) b[i] = s[i];
    if (!memcmp(cache[r], b, sizeof b)) return;
    for (uint8_t i = 0; i < LCD_COLS;) {
        if (cache[r][i] == b[i]) { i++; continue; }
        lcd_byte((uint8_t)(0x80 | (r ? 0x40 : 0) | i), false);
        do {
            lcd_byte((uint8_t)b[i], true);
            i++;
        } while (i < LCD_COLS && cache[r][i] != b[i]);
    }
    memcpy(cache[r], b, sizeof b);
}

/* ================================================================
 * 5. Wire Protocol (37-byte Frame, CRC16-CCITT)
 * ================================================================ */
static uint16_t crc16(const uint8_t *p, unsigned n) {
    uint16_t c = 0xffff;
    while (n--) {
        c ^= (uint16_t)*p++ << 8;
        for (unsigned j = 0; j < 8; j++)
            c = (uint16_t)((c & 0x8000u) ? ((uint32_t)c << 1) ^ 0x1021u : (uint32_t)c << 1);
    }
    return c;
}
static void put32(uint8_t *p, uint32_t x) {
    for (unsigned j = 0; j < 4; j++) p[j] = (uint8_t)(x >> (8 * j));
}
static uint32_t get32(const uint8_t *p) {
    return U32(p[0]) | (U32(p[1]) << 8) | (U32(p[2]) << 16) | (U32(p[3]) << 24);
}
static void encode(const Packet *p, uint8_t b[FRAME_N]) {
    b[0] = 0xa5;
    b[1] = 0x5a;
    b[2] = p->type;
    uint32_t w[8] = { p->id, p->a, p->b, p->c, p->d, p->e, p->f, p->g };
    for (unsigned j = 0; j < 8; j++) put32(b + 3 + 4 * j, w[j]);
    uint16_t c = crc16(b + 2, 33);
    b[35] = (uint8_t)c;
    b[36] = (uint8_t)(c >> 8);
}
static bool parse(Parser *s, uint8_t c, Packet *p) {
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
    uint8_t *b = s->b;
    s->n = 0;
    if (crc16(b + 2, 33) != ((uint16_t)b[35] | (uint16_t)b[36] << 8)) {
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

/* ================================================================
 * 6. Buzzer (D12 = PB4) & 4x4 Keypad (Rows: D2..D5, Cols: D6..D9)
 * ================================================================ */
static Parser parser;
static Packet ui = {};
static uint32_t last_rx, render_at, pending_weight;
static uint32_t pending_end_id;
static uint16_t key_seq, pending_seq;
static bool seen, was_active, was_assist, was_summary, was_ready;
static uint32_t last_id, last_reps;
static char entry[7] = {0};
static uint8_t en;

static uint32_t beep_at;
static uint16_t beep_len;
static uint8_t beep_left;
static bool beep_on;

static const char keys[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static void gpio_init_reg(void) {
    DDRB  |= (1 << DDB4);
    PORTB &= ~(1 << PORTB4);

    DDRD  &= ~((1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5));
    PORTD &= ~((1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5));

    DDRD  &= ~((1 << DDD6) | (1 << DDD7));
    PORTD |=  ((1 << PORTD6) | (1 << PORTD7));
    DDRB  &= ~((1 << DDB0) | (1 << DDB1));
    PORTB |=  ((1 << PORTB0) | (1 << PORTB1));
}

static void buzz(uint8_t count, uint16_t len) {
    beep_left = count;
    beep_len  = len;
    beep_on   = true;
    beep_at   = sys_ms();
    PORTB    |= (1 << PORTB4);
}

static void buzzer_task(uint32_t t) {
    if (beep_on && (uint32_t)(t - beep_at) >= beep_len) {
        PORTB  &= ~(1 << PORTB4);
        beep_on = false;
        beep_at = t;
        if (beep_left) beep_left--;
    } else if (!beep_on && beep_left && (uint32_t)(t - beep_at) >= 100u) {
        beep_on = true;
        beep_at = t;
        PORTB  |= (1 << PORTB4);
    }
}

static char raw_key(void) {
    char found = 0;
    unsigned count = 0;
    for (uint8_t row = 0; row < 4; row++) {
        uint8_t rmask = (uint8_t)(1u << (row + 2u));
        PORTD &= ~rmask;
        DDRD  |= rmask;
        delay_us_reg(4);

        if (!(PIND & (1 << PIND6))) { found = keys[row][0]; count++; }
        if (!(PIND & (1 << PIND7))) { found = keys[row][1]; count++; }
        if (!(PINB & (1 << PINB0))) { found = keys[row][2]; count++; }
        if (!(PINB & (1 << PINB1))) { found = keys[row][3]; count++; }

        DDRD  &= ~rmask;
        PORTD &= ~rmask;
    }
    return (count == 1u) ? found : 0;
}

static bool weight_parse(uint32_t &v) {
    if (!en || entry[en - 1] == '.') return false;
    uint32_t whole = 0, frac = 0;
    bool dot = false;
    for (uint8_t i = 0; i < en; i++) {
        char c = entry[i];
        if (c == '.') {
            if (dot || i == 0) return false;
            dot = true;
        } else if (c >= '0' && c <= '9') {
            if (dot) {
                if (i != en - 1) return false;
                frac = (uint32_t)(c - '0');
            } else {
                whole = whole * 10u + (uint32_t)(c - '0');
                if (whole > 999u) return false;
            }
        } else return false;
    }
    v = whole * 10u + frac;
    return v >= 1u && v <= 9999u;
}

static void key(char c) {
    if (c == 'D') {
        if (seen && (uint32_t)(sys_ms()-last_rx)<=1200u && ui.id &&
            !(ui.a & F_ACTIVE) && (ui.a & F_RACKED) && !pending_weight && !en) {
            if (!(ui.a & F_SESSION_END)) pending_end_id=ui.id;
            buzz(1,60);
        } else buzz(2,80);
        return;
    }
    if ((seen && ((uint32_t)(sys_ms() - last_rx) > 1200u ||
                  (ui.a & F_ACTIVE))) || pending_weight) return;

    if (c == '*') { en = 0; entry[0] = 0; return; }
    if (c == 'B') { if (en) entry[--en] = 0; return; }
    if (c == '#') {
        uint32_t w;
        if (weight_parse(w)) {
            pending_weight = w;
            if (++key_seq == 0) ++key_seq;
            pending_seq = key_seq;
            en = 0;
            entry[0] = 0;
            buzz(1, 60);
        } else {
            buzz(2, 80);
        }
        return;
    }
    if (c == 'A') {
        if (en && en < 5 && !strchr(entry, '.')) {
            entry[en++] = '.';
            entry[en] = 0;
        }
        return;
    }
    if (c >= '0' && c <= '9' && en < 5) {
        char *d = strchr(entry, '.');
        if (d && d[1]) return;
        if (!d && en >= 3) return;
        entry[en++] = c;
        entry[en] = 0;
    }
}

static void keypad_task(uint32_t t) {
    static uint32_t last, changed;
    static char old, stable;
    if ((uint32_t)(t - last) < 10u) return;
    last = t;
    char k = raw_key();
    if (k != old) { old = k; changed = t; }
    if (stable != old && (uint32_t)(t - changed) >= 30u) {
        stable = old;
        if (stable) key(stable);
    }
}

/* ================================================================
 * 7. Packet Reception & LCD Rendering
 * ================================================================ */
static void receive(void) {
    uint8_t byte_in;
    for (;;) {
        uint8_t sreg = SREG;
        cli();
        if (rx_error) {
            rx_error = false;
            rx_tail = rx_head;
            parser.n = 0;
        }
        SREG = sreg;
        if (!sw_uart_getc(&byte_in)) break;
        Packet p;
        if (!parse(&parser, byte_in, &p) || p.type != T_UI) continue;
        seen = true;
        last_rx = sys_ms();
        bool active  = (p.a & F_ACTIVE) != 0;
        bool assist  = (p.a & F_ASSIST) != 0;
        bool summary = (p.a & F_SUMMARY) != 0;
        bool ready   = (p.a & F_M_READY) != 0;

        if (active) {
            pending_weight = 0;
        } else if (!summary && pending_weight && (uint16_t)(p.f >> 16) == pending_seq) {
            pending_weight = 0;
        }

        if (active || p.id != pending_end_id || (p.a & F_SESSION_END)) pending_end_id=0;
        Packet answer = { T_KEY, p.id, pending_weight, pending_weight ? (uint32_t)pending_seq : 0u,
                          pending_end_id ? 1u : 0u, 0, 0, 0, 0 };
        if (!summary) p.f &= 0xffffu;

        uint8_t b[FRAME_N];
        encode(&answer, b);
        delay_us_reg(1500);
        sw_uart_write_frame(b, FRAME_N);

        ui = p;
        if (assist && (!was_assist || last_id != p.id)) buzz(3, 250);
        else if (summary && !was_summary) buzz(2, 150);
        else if (active && (!was_active || p.id != last_id)) buzz((p.g >> 16) & I_CAL ? 2 : 1, 200);
        else if (active && p.id == last_id && p.c > last_reps) buzz(1, 70);
        else if (!active && ready && !was_ready) buzz(1, 50);

        was_active  = active;
        was_assist  = assist;
        was_summary = summary;
        was_ready   = ready;
        last_id     = p.id;
        last_reps   = p.c;
    }
}

static void render(uint32_t t) {
    char a[40], b[40];
    if (!seen) {
        if (en) {
            snprintf(a, sizeof a, "Input: %s kg", entry);
            line(0, a);
            line(1, "#:Save *:Clear");
        } else if (pending_weight) {
            line(0, "Weight queued");
            line(1, "Wait STM3 link");
        } else {
            line(0, "Enter weight kg");
            line(1, "#:Save A:Decimal");
        }
        return;
    }
    if ((uint32_t)(t - last_rx) > 1200u) {
        line(0, "LINK LOST / WAIT");
        line(1, "Check STM3 UART");
        return;
    }

    uint32_t fl  = ui.a;
    uint32_t why = (ui.g >> 8) & 255u;
    uint32_t rec = ui.g & 255u;

    uint32_t inv         = (fl & F_SUMMARY) ?
                           ((ui.a >> SUMMARY_INVALID_SHIFT) & SUMMARY_INVALID_MASK) : (ui.g >> 16);
    uint8_t instab_score = (uint8_t)((ui.f >> 16) & 0xffu);
    uint8_t next_reps    = (uint8_t)((ui.f >> 24) & 0xffu);
    /* Exact value from STM2 in 0.1 kg; do not guess missing weight bits. */
    uint32_t nw_x10 = ui.g >> 16;

    unsigned long w    = (unsigned long)(ui.b / 10u);
    unsigned long wf   = (unsigned long)(ui.b % 10u);
    unsigned long reps = (unsigned long)ui.c;
    unsigned long up   = (unsigned long)ui.e;
    unsigned long avg  = (unsigned long)ui.d;
    unsigned long tilt = (unsigned long)(ui.f & 0xffffu);

    if (fl & F_ACTIVE) {
        snprintf(a, sizeof a, "%lu.%lukg R:%lu", w, wf, reps);
        if (fl & F_ASSIST) {
            snprintf(a, sizeof a, "ASSIST R:%lu", reps);
            snprintf(b, sizeof b, "%s", why == REASON_PEDAL ? "PEDAL DETECTED" : (why >= 2 && why <= 4) ? "FSR DETECTED" : "MODEL NOT READY");
        } else if (inv & I_CAL) {
            snprintf(b, sizeof b, "CAL FAIL - RACK");
        } else if (!(fl & F_M_BASE)) {
            snprintf(b, sizeof b, "Syncing start...");
        } else if (fl & F_M_WARN) {
            if (fl & F_M_TILT) snprintf(b, sizeof b, "TILT MAX:%lu.%lu", tilt / 10u, tilt % 10u);
            else snprintf(b, sizeof b, "TILT WARNING");
        } else if (inv && (t / 2000u) % 2u) {
            snprintf(b, sizeof b, "RUN ERR:%lX", (unsigned long)inv);
        } else if (fl & F_M_TILT) {
            snprintf(b, sizeof b, "U:%lu.%02lus M:%lu.%lu", up / 1000u, (up % 1000u) / 10u, tilt / 10u, tilt % 10u);
        } else {
            snprintf(b, sizeof b, "U:%lu.%02lus M:--", up / 1000u, (up % 1000u) / 10u);
        }
    } else if (pending_weight) {
        snprintf(a, sizeof a, "Sending %lu.%lukg", (unsigned long)(pending_weight / 10u), (unsigned long)(pending_weight % 10u));
        snprintf(b, sizeof b, "Waiting STM3 ACK");
    } else if (en) {
        snprintf(a, sizeof a, "Input: %s kg", entry);
        snprintf(b, sizeof b, "#:Save *:Clear");
    } else if (!(fl & F_WEIGHT)) {
        snprintf(a, sizeof a, "Enter weight kg");
        snprintf(b, sizeof b, "#:Save A:Decimal");
    } else if (fl & F_SESSION_END) {
        snprintf(a, sizeof a, "Workout ended");
        snprintf(b, sizeof b, "See laptop / USB");
    } else if (pending_end_id) {
        snprintf(a, sizeof a, "Finishing...");
        snprintf(b, sizeof b, "Waiting STM2");
    } else if (fl & F_WAIT_FINAL) {
        snprintf(a, sizeof a, "Set %lu ended", (unsigned long)ui.id);
        snprintf(b, sizeof b, "Waiting data");
    } else if ((fl & F_SUMMARY) && reps == 0) {
        snprintf(a, sizeof a, "Set ended: 0 rep");
        if (fl & F_CAL_FAIL) snprintf(b, sizeof b, "CAL FAIL: check");
        else if (inv) snprintf(b, sizeof b, "Result ERR:%lX", (unsigned long)inv);
        else snprintf(b, sizeof b, "%s", (fl & F_M_READY) ? "READY: Unrack" : "Check sensors");
    } else if (fl & F_SUMMARY) {
        static bool showing = false;
        static uint32_t result_id, summary_at;
        if (!showing || result_id != ui.id) {
            showing = true;
            result_id = ui.id;
            summary_at = t;
        }
        switch (((t - summary_at) / 3000u) % 4u) {
            case 0:
                snprintf(a, sizeof a, "%lu.%lukg R:%lu", w, wf, reps);
                if (reps) snprintf(b, sizeof b, "Avg up:%lu.%02lus", avg / 1000u, (avg % 1000u) / 10u);
                else snprintf(b, sizeof b, "No valid reps");
                break;
            case 1:
                if (fl & F_M_TILT) snprintf(a, sizeof a, "Max tilt:%lu.%lu", tilt / 10u, tilt % 10u);
                else snprintf(a, sizeof a, "Max tilt: --");
                if (reps) snprintf(b, sizeof b, "Last up:%lu.%02lus", up / 1000u, (up % 1000u) / 10u);
                else snprintf(b, sizeof b, "Last up: --");
                break;
            case 2:
                if (rec != REC_NONE && nw_x10 > 0) {
                    const char *tag = (rec == REC_REDUCE) ? "REDC" :
                                      (rec == REC_INCREASE) ? "INCR" : "KEEP";
                    snprintf(a, sizeof a, "Instab:%u%% [%s]", (unsigned)instab_score, tag);
                    snprintf(b, sizeof b, "Next:%lu.%lukg x%u", (unsigned long)(nw_x10 / 10u), (unsigned long)(nw_x10 % 10u), (unsigned)next_reps);
                } else {
                    snprintf(a, sizeof a, "Next set advice");
                    snprintf(b, sizeof b, "%s", reps < 3 ? "Advice needs 3R" : "Advice skipped");
                }
                break;
            default:
                if (inv) snprintf(a, sizeof a, "Result ERR:%lX", (unsigned long)inv);
                else snprintf(a, sizeof a, "%s", (fl & F_ASSIST) ? "Assisted set" : "Set complete");
                snprintf(b, sizeof b, "%s", (fl & F_CAL_FAIL) ? "CAL FAIL: check" : (fl & F_CAL_BUSY) ? "Calibrating..." : (fl & F_M_READY) ? "Unrack / D:End" : "Wait sensor link");
                break;
        }
    } else if (!(fl & F_RACKED) || !(fl & F_CONTACT)) {
        snprintf(a, sizeof a, "%lu.%lukg %s", w, wf, (fl & F_UNRACKED) ? "OUT" : "WAIT");
        snprintf(b, sizeof b, "Rack bar first");
    } else if (fl & F_CAL_FAIL) {
        snprintf(a, sizeof a, "CALIBRATION FAIL");
        snprintf(b, sizeof b, "Check sensors");
    } else if (fl & F_CAL_BUSY) {
        snprintf(a, sizeof a, "Calibrating...");
        snprintf(b, sizeof b, "Do not move bar");
    } else if (fl & F_M_READY) {
        snprintf(a, sizeof a, "%lu.%lukg READY", w, wf);
        snprintf(b, sizeof b, "%s", ui.id ? "Unrack / D:End" : "Unrack to start");
    } else if (fl & F_CAL_REQUIRED) {
        snprintf(a, sizeof a, "Wait sensor link");
        snprintf(b, sizeof b, "Check STM1/STM2");
    } else {
        snprintf(a, sizeof a, "Weight:%lu.%lukg", w, wf);
        snprintf(b, sizeof b, "Enter weight+#");
    }

    line(0, a);
    line(1, b);
}

/* ================================================================
 * 8. Arduino Sketch Entry Points (Register-Only Execution)
 * ================================================================ */
void setup(void) {
    timer1_init();
    uart0_init();
    gpio_init_reg();
    twi_init();
    sw_uart_init();
    sei();

    lcd_init();
    uart0_puts("PRJ2 UNO Register-Only Ready\r\n");
}

void loop(void) {
    receive();
    uint32_t t = sys_ms();
    keypad_task(t);
    buzzer_task(t);
    if ((uint32_t)(t - render_at) >= 220u) {
        render_at = t;
        render(t);
    }
    receive();
}

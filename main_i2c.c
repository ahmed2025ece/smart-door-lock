/*
 * SMART DOOR LOCK  -  ATmega32A
 * ---------------------------------------------------------------
 * Modified version:
 *   - LCD 16x2 with I2C backpack (PCF8574): VCC, GND, SDA, SCL
 *   - Clock 8 MHz  (change F_CPU below if you use another one: 16000000UL or 20000000UL)
 *
 * Wiring
 *   LCD SDA  -> PC1 (pin 23)
 *   LCD SCL  -> PC0 (pin 22)
 *   Keypad 4x4: rows PA0..PA3 (outputs), columns PA4..PA7 (inputs)
 *   Servo signal -> PD5 (OC1A)
 *   UART (voice module / PC): RXD PD0, TXD PD1 @ 9600 8N1
 *
 * Build (avr-gcc, optimisation -Os is REQUIRED for _delay_ms):
 *   avr-gcc -mmcu=atmega32a -Os -DF_CPU=8000000UL main.c -o main.elf
 *   avr-objcopy -O ihex -R .eeprom main.elf main.hex
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

/* ================= Configuration ================= */
#define BAUD            9600UL
#define PIN_MIN         4
#define PIN_MAX         6
#define EE_LEN_ADDR     6
#define EE_MAGIC_ADDR   7
#define EE_MAGIC        0xA5

#define T_PIN_ENTRY_MS  20000UL   /* waiting for PIN after voice OK        */
#define T_CHANGE_MS     20000UL   /* change-PIN screen timeout             */
#define T_UNLOCK_MS     10000UL   /* door stays unlocked, then auto-locks  */
#define T_LOCKOUT_MS    30000UL   /* lockout after 5 wrong PINs            */
#define T_MSG_MS         2000UL   /* temporary message on LCD              */
#define MAX_FAILS       5

#define SERVO_LOCK_DEG    0
#define SERVO_UNLOCK_DEG  90

/* PCF8574 -> LCD pin mapping (most common I2C backpacks) */
#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08            /* backlight on */
/* D4..D7 are on P4..P7 */

/* ================= Timer0 : 1 ms tick ================= */
#if F_CPU == 20000000UL
  #define T0_CS   (1 << CS02)                    /* /256 */
  #define T0_OCR  77                             /* 1.0016 ms */
#elif F_CPU == 16000000UL
  #define T0_CS   ((1 << CS01) | (1 << CS00))    /* /64  */
  #define T0_OCR  249
#elif F_CPU == 8000000UL
  #define T0_CS   ((1 << CS01) | (1 << CS00))    /* /64  */
  #define T0_OCR  124
#else
  #error "Add Timer0 settings for this F_CPU"
#endif

static volatile uint32_t ms_ticks;

ISR(TIMER0_COMP_vect)
{
    ms_ticks++;
}

static void timer0_init(void)
{
    TCCR0 = (1 << WGM01) | T0_CS;     /* CTC */
    OCR0  = T0_OCR;
    TIMSK |= (1 << OCIE0);
}

static uint32_t millis(void)
{
    uint8_t s = SREG;
    cli();
    uint32_t m = ms_ticks;
    SREG = s;
    return m;
}

static uint8_t expired(uint32_t t)
{
    return ((int32_t)(millis() - t) >= 0);
}

/* ================= UART (ring buffers) ================= */
#define UBRR_VAL ((F_CPU + 8UL * BAUD) / (16UL * BAUD) - 1)

#define RXQ 64
#define TXQ 64
static volatile uint8_t rx_buf[RXQ], tx_buf[TXQ];
static volatile uint8_t rx_head, rx_tail, tx_head, tx_tail;
static volatile uint8_t rx_overflow;

ISR(USART_RXC_vect)
{
    uint8_t st = UCSRA;
    uint8_t d  = UDR;
    uint8_t next = (rx_head + 1) & (RXQ - 1);
    if (st & ((1 << FE) | (1 << DOR) | (1 << PE)))
        d = 0xFF;
    if (next == rx_tail) {
        rx_overflow = 1;
    } else {
        rx_buf[rx_head] = d;
        rx_head = next;
    }
}

ISR(USART_UDRE_vect)
{
    if (tx_head == tx_tail) {
        UCSRB &= ~(1 << UDRIE);
    } else {
        UDR = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) & (TXQ - 1);
    }
}

static void uart_init(void)
{
    UBRRH = (uint8_t)(UBRR_VAL >> 8);
    UBRRL = (uint8_t)UBRR_VAL;
    UCSRA = 0;
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   /* 8N1 */
}

static void uart_putc(uint8_t c)
{
    uint8_t next = (tx_head + 1) & (TXQ - 1);
    if (next == tx_tail) return;          /* buffer full: drop */
    tx_buf[tx_head] = c;
    tx_head = next;
    UCSRB |= (1 << UDRIE);
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static int16_t uart_getc(void)
{
    if (rx_head == rx_tail) return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (RXQ - 1);
    return c;
}

/* Collects a text line (max 15 chars). Returns 1 when a full line is in out[] */
static char    rl_buf[16];
static uint8_t rl_len, rl_skip;

static uint8_t uart_readline(char *out)
{
    uint8_t s = SREG;
    cli();
    uint8_t ov = rx_overflow;
    rx_overflow = 0;
    SREG = s;
    if (ov) rl_skip = 1;

    for (;;) {
        int16_t c = uart_getc();
        if (c < 0) return 0;

        if (c == '\n' || c == '\r') {
            uint8_t ok = (!rl_skip && rl_len);
            if (ok) {
                rl_buf[rl_len] = 0;
                memcpy(out, rl_buf, rl_len + 1);
            }
            rl_len = 0;
            rl_skip = 0;
            if (ok) return 1;
        } else if (c >= 0x20 && c <= 0x7E) {
            if (rl_len < 15) rl_buf[rl_len++] = (char)c;
            else             rl_skip = 1;
        } else {
            rl_skip = 1;
        }
    }
}

/* ================= TWI (I2C) master ================= */
#define TW_ST_START      0x08
#define TW_ST_REP_START  0x10
#define TW_ST_SLAW_ACK   0x18
#define TW_ST_DATA_ACK   0x28
#define TW_STATUS_MASK   0xF8

static uint8_t twi_wait(void)
{
    uint16_t t = 30000;
    while (!(TWCR & (1 << TWINT))) {
        if (--t == 0) return 0;
    }
    return 1;
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint16_t t = 30000;
    while ((TWCR & (1 << TWSTO)) && --t) ;
}

static void twi_reset(void)
{
    TWCR = 0;
    TWCR = (1 << TWEN);
}

static void twi_init(void)
{
    PORTC |= (1 << PC0) | (1 << PC1);       /* internal pull-ups (harmless) */
    TWSR = 0;                               /* prescaler 1 */
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);   /* 100 kHz */
    TWCR = (1 << TWEN);
}

static uint8_t twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_wait()) return 0;
    uint8_t s = TWSR & TW_STATUS_MASK;
    return (s == TW_ST_START || s == TW_ST_REP_START);
}

static uint8_t twi_sla_w(uint8_t addr7)
{
    TWDR = (uint8_t)(addr7 << 1);           /* write */
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return 0;
    return ((TWSR & TW_STATUS_MASK) == TW_ST_SLAW_ACK);
}

static uint8_t twi_put(uint8_t d)
{
    TWDR = d;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return 0;
    return ((TWSR & TW_STATUS_MASK) == TW_ST_DATA_ACK);
}

/* ================= LCD 16x2 over PCF8574 ================= */
static uint8_t lcd_addr;     /* 7-bit address found by probe (0x20..0x3F) */
static uint8_t lcd_ok;

/* Finds the backpack automatically (usually 0x27 or 0x3F) */
static uint8_t lcd_probe(void)
{
    for (uint8_t a = 0x20; a <= 0x3F; a++) {
        if (!twi_start()) { twi_reset(); return 0; }   /* bus problem */
        uint8_t ack = twi_sla_w(a);
        twi_stop();
        if (ack) { lcd_addr = a; return 1; }
    }
    return 0;
}

static void lcd_nibble(uint8_t hi, uint8_t rs)
{
    if (!lcd_ok) return;
    uint8_t b = (hi & 0xF0) | LCD_BL | (rs ? LCD_RS : 0);
    if (!twi_start() || !twi_sla_w(lcd_addr) ||
        !twi_put(b | LCD_EN) || !twi_put(b)) {
        lcd_ok = 0;                 /* LCD lost: keep the lock working */
        twi_reset();
        return;
    }
    twi_stop();
}

static void lcd_send(uint8_t v, uint8_t rs)
{
    lcd_nibble(v & 0xF0, rs);
    lcd_nibble((uint8_t)(v << 4), rs);
    _delay_us(50);
}

static void lcd_cmd(uint8_t c)  { lcd_send(c, 0); }
static void lcd_data(uint8_t d) { lcd_send(d, 1); }

static void lcd_clear(void)
{
    lcd_cmd(0x01);
    _delay_ms(2);
}

static void lcd_init(void)
{
    twi_init();
    _delay_ms(50);                  /* LCD power-up time */
    wdt_reset();
    lcd_ok = lcd_probe();
    if (!lcd_ok) return;

    lcd_nibble(0x30, 0); _delay_ms(5);
    lcd_nibble(0x30, 0); _delay_ms(1);
    lcd_nibble(0x30, 0); _delay_ms(1);
    lcd_nibble(0x20, 0); _delay_ms(1);   /* 4-bit mode */
    lcd_cmd(0x28);                       /* 4-bit, 2 lines, 5x8 */
    lcd_cmd(0x08);                       /* display off */
    lcd_clear();
    lcd_cmd(0x06);                       /* entry mode */
    lcd_cmd(0x0C);                       /* display on, no cursor */
}

static void lcd_goto(uint8_t row, uint8_t col)
{
    if (col > 15) col = 15;
    lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col));
}

static void lcd_line(uint8_t row, const char *s)
{
    lcd_goto(row, 0);
    for (uint8_t i = 0; i < 16; i++) {
        if (*s) lcd_data((uint8_t)*s++);
        else    lcd_data(' ');
    }
}

static void lcd2(const char *l1, const char *l2)
{
    lcd_line(0, l1);
    lcd_line(1, l2);
}

/* ================= Servo (Timer1, 50 Hz, OC1A = PD5) ================= */
#define SERVO_TICKS(us) ((uint16_t)(((uint32_t)(us) * (F_CPU / 1000000UL)) / 8UL))

static void servo_init(void)
{
    ICR1  = (uint16_t)(F_CPU / 8UL / 50UL - 1UL);     /* 20 ms */
    OCR1A = SERVO_TICKS(600);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);   /* Fast PWM, /8 */
    DDRD  |= (1 << PD5);
}

static void servo_set(uint8_t deg)
{
    if (deg > 180) deg = 180;
    OCR1A = SERVO_TICKS(600 + (uint16_t)deg * 10);    /* 0.6 .. 2.4 ms */
}

/* ================= Keypad 4x4 on PORTA ================= */
static const char keymap[16] = {
    '7','8','9','A',
    '4','5','6','B',
    '1','2','3','C',
    '*','0','#','D'
};

static uint8_t kp_last_raw, kp_last_acc, kp_stable;

static void keypad_init(void)
{
    DDRA  = 0x0F;       /* PA0..PA3 rows = outputs */
    PORTA = 0xFF;       /* columns pulled up       */
}

static char keypad_raw(void)
{
    char key = 0;
    uint8_t cnt = 0;

    for (uint8_t r = 0; r < 4; r++) {
        PORTA = (uint8_t)((PORTA | 0x0F) & ~(1 << r));
        _delay_us(10);
        uint8_t cols = (uint8_t)((~PINA) >> 4) & 0x0F;
        for (uint8_t c = 0; c < 4; c++) {
            if (cols & (1 << c)) {
                key = keymap[r * 4 + c];
                cnt++;
            }
        }
    }
    PORTA |= 0x0F;
    return (cnt == 1) ? key : 0;
}

/* Debounced: returns a key once per press, otherwise 0 */
static char keypad_get(void)
{
    char raw = keypad_raw();
    if ((uint8_t)raw != kp_last_raw) {
        kp_last_raw = (uint8_t)raw;
        kp_stable = 0;
        return 0;
    }
    if (kp_stable < 4) kp_stable++;
    if (kp_stable != 4) return 0;
    if (raw == 0) { kp_last_acc = 0; return 0; }
    if ((uint8_t)raw == kp_last_acc) return 0;
    kp_last_acc = (uint8_t)raw;
    return raw;
}

/* ================= Application state ================= */
typedef enum {
    ST_IDLE = 0,        /* waiting for voice            */
    ST_WAIT_PIN = 1,    /* voice OK, waiting for PIN    */
    ST_UNLOCKED = 2,
    ST_CHANGE_PIN = 3
} state_t;

static state_t state;
static char    stored_pin[PIN_MAX];
static uint8_t pin_len;
static char    in_buf[PIN_MAX];
static uint8_t in_len;
static uint8_t voice_ok;
static uint8_t fail_count, locked_out;
static uint8_t msg_active;
static uint32_t lockout_until, msg_until, next_scan;
static uint32_t pin_until, change_until, unlock_until;

static void reset_input(void)
{
    voice_ok = 0;
    in_len = 0;
    memset(in_buf, 0, sizeof(in_buf));
}

static void show_home(void)
{
    msg_active = 0;
    lcd2("SMART DOOR LOCK", "WAITING VOICE");
}

static void show_message(const char *l1, const char *l2)
{
    lcd2(l1, l2);
    msg_active = 1;
    msg_until = millis() + T_MSG_MS;
}

static void show_masked(void)
{
    lcd_goto(1, 10);
    for (uint8_t i = 0; i < PIN_MAX; i++)
        lcd_data(i < in_len ? '*' : ' ');
}

static void send_status(void)
{
    switch (state) {
    case ST_UNLOCKED:    uart_puts("UNLOCKED\n");     break;
    case ST_CHANGE_PIN:  uart_puts("CHANGING_PIN\n"); break;
    case ST_WAIT_PIN:    uart_puts("WAITING_PIN\n");  break;
    default:             uart_puts("LOCKED\n");       break;
    }
}

static void go_locked_idle(void)       /* back to idle + LOCKED message */
{
    reset_input();
    state = ST_IDLE;
    uart_puts("LOCKED\n");
    show_home();
}

static void enter_unlocked(const char *l1, const char *l2)
{
    reset_input();
    state = ST_UNLOCKED;
    unlock_until = millis() + T_UNLOCK_MS;
    uart_puts("UNLOCKED\n");
    lcd2(l1, l2);
}

static void load_pin(void)
{
    uint8_t len = eeprom_read_byte((uint8_t *)EE_LEN_ADDR);
    uint8_t ok = (eeprom_read_byte((uint8_t *)EE_MAGIC_ADDR) == EE_MAGIC) &&
                 (len >= PIN_MIN) && (len <= PIN_MAX);
    if (ok) {
        for (uint8_t i = 0; i < len; i++) {
            uint8_t c = eeprom_read_byte((uint8_t *)(uint16_t)i);
            if (c < '0' || c > '9') { ok = 0; break; }
            stored_pin[i] = (char)c;
        }
    }
    if (!ok) {
        memcpy(stored_pin, "1234", 4);
        len = 4;
    }
    pin_len = len;
}

static void save_pin(void)
{
    eeprom_update_byte((uint8_t *)EE_MAGIC_ADDR, 0xFF);   /* invalidate first */
    for (uint8_t i = 0; i < in_len; i++)
        eeprom_update_byte((uint8_t *)(uint16_t)i, (uint8_t)in_buf[i]);
    eeprom_update_byte((uint8_t *)EE_LEN_ADDR, in_len);
    eeprom_update_byte((uint8_t *)EE_MAGIC_ADDR, EE_MAGIC);
    memcpy(stored_pin, in_buf, in_len);
    pin_len = in_len;
}

/* ================= UART commands ================= */
static void on_voice_ok(void)
{
    if (locked_out) {
        if (!expired(lockout_until)) {
            uart_puts("ACCESS_DENIED\n");
            show_message("TOO MANY TRIES", "TRY LATER");
            return;
        }
        locked_out = 0;
        fail_count = 0;
    }
    reset_input();
    voice_ok = 1;
    state = ST_WAIT_PIN;
    pin_until = millis() + T_PIN_ENTRY_MS;
    msg_active = 0;
    uart_puts("WAITING_PIN\n");
    lcd2("VOICE VERIFIED", "ENTER PIN:");
}

static void on_uart_line(const char *line)
{
    if (!strcmp(line, "VOICE_OK")) {
        if (state == ST_IDLE) on_voice_ok();
        else                  send_status();
    } else if (!strcmp(line, "VOICE_FAIL")) {
        if (state == ST_IDLE) {
            uart_puts("ACCESS_DENIED\n");
            show_message("VOICE FAILED", "ACCESS DENIED");
        } else {
            send_status();
        }
    } else if (!strcmp(line, "LOCK")) {
        servo_set(SERVO_LOCK_DEG);
        reset_input();
        state = ST_IDLE;
        uart_puts("LOCKED\n");
        show_home();
    } else if (!strcmp(line, "STATUS")) {
        send_status();
    }
}

/* ================= Keypad handling ================= */
static uint8_t is_digit(char k) { return (k >= '0' && k <= '9'); }

static void on_key(char k)
{
    switch (state) {

    case ST_UNLOCKED:
        if (k == 'A') {
            reset_input();
            state = ST_CHANGE_PIN;
            change_until = millis() + T_CHANGE_MS;
            uart_puts("CHANGING_PIN\n");
            lcd2("CHANGE PIN", "NEW PIN:");
        }
        break;

    case ST_CHANGE_PIN:
        if (is_digit(k)) {
            if (in_len < PIN_MAX) {
                in_buf[in_len++] = k;
                change_until = millis() + T_CHANGE_MS;
                show_masked();
            }
        } else if (k == '*') {
            in_len = 0;
            memset(in_buf, 0, sizeof(in_buf));
            show_masked();
        } else if (k == 'D') {
            enter_unlocked("DOOR UNLOCKED", "A=CHANGE PIN");
        } else if (k == '#') {
            if (in_len < PIN_MIN) {
                in_len = 0;
                memset(in_buf, 0, sizeof(in_buf));
                lcd2("MIN 4 DIGITS", "NEW PIN:");
            } else {
                save_pin();
                fail_count = 0;
                locked_out = 0;
                enter_unlocked("PIN CHANGED", "DOOR UNLOCKED");
                uart_puts("PIN_CHANGED\n");
            }
        }
        break;

    case ST_WAIT_PIN:
        if (!voice_ok) { in_len = 0; break; }

        if (is_digit(k)) {
            if (in_len < PIN_MAX) {
                in_buf[in_len++] = k;
                show_masked();
            }
        } else if (k == '*') {
            in_len = 0;
            memset(in_buf, 0, sizeof(in_buf));
            show_masked();
        } else if (k == '#' && in_len != 0) {
            /* compare all PIN_MAX positions (no early exit) */
            uint8_t diff = in_len ^ pin_len;
            for (uint8_t i = 0; i < PIN_MAX; i++) {
                uint8_t a = (i < in_len)  ? (uint8_t)in_buf[i]    : 0;
                uint8_t b = (i < pin_len) ? (uint8_t)stored_pin[i] : 0;
                diff |= (a ^ b);
            }
            reset_input();

            if (diff == 0) {                          /* correct */
                fail_count = 0;
                locked_out = 0;
                servo_set(SERVO_UNLOCK_DEG);
                state = ST_UNLOCKED;
                unlock_until = millis() + T_UNLOCK_MS;
                uart_puts("UNLOCKED\n");
                lcd2("ACCESS GRANTED", "A=CHANGE PIN");
            } else {                                  /* wrong */
                state = ST_IDLE;
                if (fail_count < 255) fail_count++;
                if (fail_count >= MAX_FAILS) {
                    locked_out = 1;
                    lockout_until = millis() + T_LOCKOUT_MS;
                }
                uart_puts("ACCESS_DENIED\n");
                show_message("WRONG PIN", "ACCESS DENIED");
            }
        }
        break;

    default:            /* ST_IDLE: keys ignored */
        in_len = 0;
        break;
    }
}

/* ================= Timeouts ================= */
static void check_timeouts(void)
{
    switch (state) {
    case ST_UNLOCKED:
        if (expired(unlock_until)) {
            servo_set(SERVO_LOCK_DEG);
            go_locked_idle();
        }
        break;
    case ST_CHANGE_PIN:
        if (expired(change_until)) {
            servo_set(SERVO_LOCK_DEG);
            go_locked_idle();
        }
        break;
    case ST_WAIT_PIN:
        if (expired(pin_until)) go_locked_idle();
        break;
    default:
        if (msg_active && expired(msg_until)) show_home();
        break;
    }
}

/* ================= main ================= */
int main(void)
{
    char line[16];

    wdt_enable(WDTO_2S);
    servo_init();
    timer0_init();
    uart_init();
    keypad_init();
    sei();

    lcd_init();
    state = ST_IDLE;
    reset_input();
    show_home();
    load_pin();

    uart_puts("LOCKED\n");
    next_scan = millis() + 5;

    for (;;) {
        wdt_reset();

        if (uart_readline(line))
            on_uart_line(line);

        if (expired(next_scan)) {
            next_scan = millis() + 5;
            char k = keypad_get();
            if (k) on_key(k);
        }

        check_timeouts();
    }
}

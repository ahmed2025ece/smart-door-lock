/*
 * SMART DOOR LOCK - ATmega32 @ 8 MHz external crystal - SINGLE FILE VERSION
 * Bluetooth (HC-05) voice result + keypad PIN + SG90 servo + LCD (4-bit parallel).
 *
 * Pins (DIP-40):
 *   HC-05 TXD->PD0(14)  HC-05 RXD<-PD1(15) via 1k/2k divider
 *   Servo signal PD5/OC1A (19), servo on its own 5 V supply, common GND
 *   Keypad rows PA0..PA3 (40..37), columns PA4..PA7 (36..33)
 *   LCD 4-bit: RS=PC0 E=PC1 D4..D7=PC4..PC7 (LM016L)
 * Fuses: lfuse=0xFF hfuse=0xC9 (external 8 MHz crystal) - program BEFORE flashing.
 * Build: Microchip Studio, device ATmega32, optimisation -Os.
 *
 * The phone verifies the speaker; VOICE_OK only opens ONE window for a PIN attempt.
 */

/* ============================================================
 * CONFIGURATION (edit here)
 * ============================================================ */
#ifndef F_CPU
#define F_CPU 8000000UL            /* external 8 MHz crystal (fuses must match!) */
#endif

/* ---------------- Bluetooth / UART (HC-05) ---------------- */
#define UART_BAUD             9600UL   /* HC-05 default data-mode baud rate        */
#define UART_RX_BUF_SIZE      64       /* must be a power of two (<=128)           */
#define UART_TX_BUF_SIZE      64       /* must be a power of two (<=128)           */
#define CMD_MAX_LEN           15       /* longest accepted line (without '\n')     */
#define CR_TERMINATES         1        /* 1 = also treat '\r' as end of line       */

/* ---------------- PIN ---------------- */
#define DEFAULT_PIN           "1234"
#define PIN_MIN_LEN           4        /* shortest PIN accepted when changing it    */
#define PIN_MAX_LEN           6        /* digits; 6 is the most that fits on the LCD */

/* ---------------- Timing (milliseconds) ---------------- */
#define PIN_TIMEOUT_MS        20000UL  /* VOICE_OK is valid this long for PIN entry */
#define UNLOCK_DURATION_MS    10000UL  /* door stays unlocked this long             */
#define MESSAGE_HOLD_MS       2000UL   /* "VOICE FAILED"/"WRONG PIN" screen time    */
#define KEYPAD_SCAN_MS        5        /* keypad scan period                        */
#define KEYPAD_DEBOUNCE_SCANS 4        /* identical scans needed (4 x 5 ms = 20 ms) */

/* ---------------- Brute-force lock-out (added safety, set 0 to disable) -------- */
#define MAX_WRONG_ATTEMPTS    5        /* consecutive wrong PINs before lock-out    */
#define LOCKOUT_MS            30000UL  /* VOICE_OK is refused for this long         */

/* ---------------- Servo (SG90) ---------------- */
#define SERVO_MIN_US          600      /* pulse width at 0 degrees   (calibrate)    */
#define SERVO_MAX_US          2400     /* pulse width at 180 degrees (calibrate)    */
#define LOCK_POSITION         0        /* degrees, 0..180                           */
#define UNLOCK_POSITION       90       /* degrees, 0..180                           */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

/* ============================================================
 * TIMEBASE (Timer0, 1 ms tick)
 * ============================================================ */
static volatile uint32_t ms_count;

/* Timer0 compare-match: fires every 1 ms */
ISR(TIMER0_COMP_vect)
{
    ms_count++;
}

void timebase_init(void)
{
    /* CTC mode, prescaler 64: 8 MHz / 64 = 125 kHz -> 125 counts = 1 ms */
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0  = (uint8_t)((F_CPU / 64UL / 1000UL) - 1UL);   /* 124 */
    TIMSK |= (1 << OCIE0);
}

uint32_t millis(void)
{
    uint8_t  s = SREG;
    uint32_t v;
    cli();
    v = ms_count;
    SREG = s;
    return v;
}

uint8_t time_reached(uint32_t deadline)
{
    return ((int32_t)(millis() - deadline) >= 0) ? 1 : 0;
}

/* ============================================================
 * UART / HC-05 (interrupt driven)
 * ============================================================ */
#define UBRR_VAL  ((F_CPU + 8UL * UART_BAUD) / (16UL * UART_BAUD) - 1UL)   /* 51 @ 8 MHz, 9600 */
#define RX_MASK   (UART_RX_BUF_SIZE - 1)
#define TX_MASK   (UART_TX_BUF_SIZE - 1)

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head, rx_tail, rx_overflow;
static volatile uint8_t tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t tx_head, tx_tail;

static char    line_buf[CMD_MAX_LEN + 1];
static uint8_t line_len, line_bad;

void uart_init(void)
{
    UBRRH = (uint8_t)(UBRR_VAL >> 8);
    UBRRL = (uint8_t)(UBRR_VAL & 0xFF);
    UCSRA = 0;
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);     /* 8 data bits, no parity, 1 stop */
}

/* Receive complete */
ISR(USART_RXC_vect)
{
    uint8_t status = UCSRA;          /* read status BEFORE reading UDR */
    uint8_t data   = UDR;
    uint8_t next   = (uint8_t)((rx_head + 1) & RX_MASK);

    if (status & ((1 << FE) | (1 << DOR) | (1 << PE)))
        data = 0xFF;                 /* marker: parser will reject the line */

    if (next == rx_tail) {           /* buffer full: drop byte, remember it */
        rx_overflow = 1;
        return;
    }
    rx_buf[rx_head] = data;
    rx_head = next;
}

/* Data register empty: feed next byte or switch itself off */
ISR(USART_UDRE_vect)
{
    if (tx_head == tx_tail) {
        UCSRB &= (uint8_t)~(1 << UDRIE);
        return;
    }
    UDR = tx_buf[tx_tail];
    tx_tail = (uint8_t)((tx_tail + 1) & TX_MASK);
}

int16_t uart_getc(void)
{
    uint8_t d;
    if (rx_head == rx_tail)
        return -1;
    d = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) & RX_MASK);
    return d;
}

void uart_putc(char c)
{
    uint8_t next = (uint8_t)((tx_head + 1) & TX_MASK);
    if (next == tx_tail)
        return;                      /* full: never block the lock logic */
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = next;
    UCSRB |= (1 << UDRIE);
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

uint8_t uart_rx_overflow_take(void)
{
    uint8_t s = SREG, f;
    cli();
    f = rx_overflow;
    rx_overflow = 0;
    SREG = s;
    return f;
}

uint8_t uart_poll_line(char *out)
{
    int16_t c;

    if (uart_rx_overflow_take())
        line_bad = 1;

    while ((c = uart_getc()) >= 0) {
        uint8_t ch = (uint8_t)c;

        if (ch == '\n' || (CR_TERMINATES && ch == '\r')) {
            uint8_t ok = (!line_bad && line_len > 0) ? 1 : 0;
            if (ok) {
                line_buf[line_len] = '\0';
                memcpy(out, line_buf, (size_t)line_len + 1);
            }
            line_len = 0;
            line_bad = 0;
            if (ok)
                return 1;
            continue;
        }
        if (ch == '\r')
            continue;                               /* tolerate CR LF */
        if (ch < 0x20 || ch > 0x7E) {               /* binary junk / UART error marker */
            line_bad = 1;
            continue;
        }
        if (line_len < CMD_MAX_LEN)
            line_buf[line_len++] = (char)ch;
        else
            line_bad = 1;                           /* too long: discard until '\n' */
    }
    return 0;
}

/* ============================================================
 * LCD 16x2 HD44780, 4-bit PARALLEL (LM016L)
 *   RS=PC0, E=PC1, D4..D7=PC4..PC7, RW=GND, D0..D3 unused
 * ============================================================ */
#define LCD_PORT      PORTC
#define LCD_DDR       DDRC
#define LCD_RS        (1 << PC0)
#define LCD_E         (1 << PC1)
#define LCD_DATA_MASK 0xF0
#define LCD_PINS      (LCD_RS | LCD_E | LCD_DATA_MASK)

static void lcd_nibble(uint8_t n)
{
    LCD_PORT = (uint8_t)((LCD_PORT & ~LCD_DATA_MASK) | (n & LCD_DATA_MASK));
    LCD_PORT |= LCD_E;
    _delay_us(2);
    LCD_PORT &= (uint8_t)~LCD_E;
    _delay_us(50);
}

static void lcd_send(uint8_t v, uint8_t rs)
{
    if (rs) LCD_PORT |= LCD_RS; else LCD_PORT &= (uint8_t)~LCD_RS;
    lcd_nibble(v);
    lcd_nibble((uint8_t)(v << 4));
}

static void lcd_cmd(uint8_t c) { lcd_send(c, 0); }

void lcd_clear(void)
{
    lcd_cmd(0x01);
    _delay_ms(2);
}

void lcd_init(void)
{
    LCD_DDR  |= LCD_PINS;
    LCD_PORT &= (uint8_t)~LCD_PINS;
    _delay_ms(50);
    lcd_nibble(0x30); _delay_ms(5);
    lcd_nibble(0x30); _delay_ms(1);
    lcd_nibble(0x30); _delay_ms(1);
    lcd_nibble(0x20); _delay_ms(1);     /* 4-bit mode */
    lcd_cmd(0x28);                      /* 2 lines, 5x8 */
    lcd_cmd(0x08);
    lcd_clear();
    lcd_cmd(0x06);
    lcd_cmd(0x0C);                      /* display on, cursor off */
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    lcd_cmd((uint8_t)(0x80 | ((row ? 0x40 : 0x00) + col)));
}

void lcd_putc(char c) { lcd_send((uint8_t)c, 1); }

void lcd_print_line(uint8_t row, const char *s)
{
    uint8_t i;
    lcd_set_cursor(row, 0);
    for (i = 0; i < 16; i++)
        lcd_putc(*s ? *s++ : ' ');
}


/* ============================================================
 * SERVO (Timer1 PWM)
 * ============================================================ */
/*
 * SG90 on OC1A (PD5, DIP pin 19). Timer1, Fast PWM mode 14 (TOP = ICR1).
 *   Prescaler 8  -> timer clock = 8 MHz / 8 = 1 MHz -> 1 tick = 1 us
 *   ICR1 = 19999 -> period = 20000 ticks = 20 ms = 50 Hz
 *   OCR1A = pulse width in microseconds (SERVO_MIN_US .. SERVO_MAX_US)
 */

#define TICKS_PER_US   (F_CPU / 8UL / 1000000UL)          /* 1 at 8 MHz */
#define PWM_TOP        ((F_CPU / 8UL / 50UL) - 1UL)       /* 19999      */

static uint16_t angle_to_ticks(uint8_t deg)
{
    uint32_t us;
    if (deg > 180) deg = 180;
    us = SERVO_MIN_US + ((uint32_t)deg * (SERVO_MAX_US - SERVO_MIN_US)) / 180UL;
    return (uint16_t)(us * TICKS_PER_US);
}

void servo_init(void)
{
    ICR1  = (uint16_t)PWM_TOP;
    OCR1A = angle_to_ticks(LOCK_POSITION);                 /* safe state BEFORE the pin is driven */
    TCCR1A = (1 << COM1A1) | (1 << WGM11);                 /* non-inverting, WGM1[3:0] = 1110 */
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    /* prescaler 8 */
    DDRD  |= (1 << PD5);                                   /* OC1A output */
}

void servo_set_angle(uint8_t degrees) { OCR1A = angle_to_ticks(degrees); }
void servo_lock(void)                 { servo_set_angle(LOCK_POSITION); }
void servo_unlock(void)               { servo_set_angle(UNLOCK_POSITION); }

/* ============================================================
 * KEYPAD 4x4
 * ============================================================ */
#define KEY_ENTER '#'
#define KEY_CLEAR '*'
#define KEY_CHANGE 'A'    /* "/" button: change PIN (only while door is unlocked) */
#define KEY_CANCEL 'D'    /* "+" button: cancel PIN change                        */

/*
 * 4x4 matrix keypad on PORTA.
 *   Rows    R1..R4 = PA0..PA3 (outputs, driven low one at a time)
 *   Columns C1..C4 = PA4..PA7 (inputs with internal pull-ups)
 */

#define KP_PORT      PORTA
#define KP_DDR       DDRA
#define KP_PIN       PINA
#define KP_ROW_MASK  0x0F

static const char keymap[4][4] = {
    { '7', '8', '9', 'A' },
    { '4', '5', '6', 'B' },
    { '1', '2', '3', 'C' },
    { '*', '0', '#', 'D' }
};

static char    candidate, last_reported;
static uint8_t stable;

void keypad_init(void)
{
    KP_DDR  = KP_ROW_MASK;     /* PA0-PA3 outputs, PA4-PA7 inputs     */
    KP_PORT = 0xFF;            /* rows idle high, column pull-ups ON  */
}

/* One raw pass over the matrix. Returns the key, or 0 for none / more than one. */
static char raw_scan(void)
{
    char    found = 0;
    uint8_t count = 0, r, c, cols;

    for (r = 0; r < 4; r++) {
        KP_PORT = (uint8_t)((KP_PORT | KP_ROW_MASK) & (uint8_t)~(1 << r));
        _delay_us(5);                                   /* let the lines settle */
        cols = (uint8_t)(((uint8_t)~KP_PIN) >> 4);      /* 1 = pressed          */
        for (c = 0; c < 4; c++) {
            if (cols & (1 << c)) {
                found = keymap[r][c];
                count++;
            }
        }
    }
    KP_PORT |= KP_ROW_MASK;
    return (count == 1) ? found : 0;
}

char keypad_scan(void)
{
    char k = raw_scan();

    if (k != candidate) {                  /* reading changed: restart debounce */
        candidate = k;
        stable = 0;
        return 0;
    }
    if (stable < KEYPAD_DEBOUNCE_SCANS)
        stable++;

    if (stable == KEYPAD_DEBOUNCE_SCANS) { /* reading is stable */
        if (candidate == 0) {
            last_reported = 0;             /* all keys released: re-arm */
            return 0;
        }
        if (candidate != last_reported) {  /* new press, report exactly once */
            last_reported = candidate;
            return candidate;
        }
    }
    return 0;
}

/* ============================================================
 * MAIN APPLICATION (state machine)
 * ============================================================ */
/*
 * Smart Door Lock - ATmega32 @ 8 MHz
 * Bluetooth (HC-05) voice-result + keypad PIN + SG90 servo + 4-bit LCD.
 *
 * The phone verifies the speaker. The ATmega32 only trusts the text result,
 * and VOICE_OK merely OPENS ONE WINDOW for a PIN attempt. The door can only
 * be unlocked by the correct PIN typed on the keypad inside that window.
 */


/* compile-time sanity checks (negative array size = build error) */
typedef char chk_pin_fits_buffer[(sizeof(DEFAULT_PIN) - 1 <= PIN_MAX_LEN) ? 1 : -1];
typedef char chk_pin_fits_lcd[(PIN_MAX_LEN <= 6) ? 1 : -1];

typedef enum {
    ST_LOCKED,        /* door locked, waiting for VOICE_OK              */
    ST_PIN_ENTRY,     /* voice accepted, keypad enabled, timer running  */
    ST_UNLOCKED,      /* door open, auto-relock timer running           */
    ST_CHANGE_PIN     /* door open, typing a NEW PIN (key A to start)   */
} state_t;

static state_t  state = ST_LOCKED;
static uint8_t  voice_verified;               /* true ONLY in ST_PIN_ENTRY */
static char     pin_buf[PIN_MAX_LEN];
static uint8_t  pin_len;

static uint32_t change_deadline, pin_deadline, unlock_deadline, msg_deadline, lockout_until, next_scan;
static uint8_t  msg_active, lockout_active, wrong_attempts;

/* ---------------- PIN storage (EEPROM, survives power loss) ---------------- */
#define PIN_MAGIC 0xA5
static uint8_t ee_magic EEMEM;
static uint8_t ee_len   EEMEM;
static char    ee_pin[PIN_MAX_LEN] EEMEM;

static char    stored_pin[PIN_MAX_LEN];
static uint8_t stored_len;

static void pin_load(void)
{
    uint8_t i, len, ok = 1;

    len = eeprom_read_byte(&ee_len);
    if (eeprom_read_byte(&ee_magic) != PIN_MAGIC || len < PIN_MIN_LEN || len > PIN_MAX_LEN) {
        ok = 0;
    } else {
        for (i = 0; i < len; i++) {
            char c = (char)eeprom_read_byte((const uint8_t *)&ee_pin[i]);
            if (c < '0' || c > '9') { ok = 0; break; }
            stored_pin[i] = c;
        }
    }
    if (!ok) {                                  /* blank / corrupt EEPROM -> default PIN */
        len = (uint8_t)(sizeof(DEFAULT_PIN) - 1);
        memcpy(stored_pin, DEFAULT_PIN, len);
    }
    stored_len = len;
}

static void pin_save(const char *p, uint8_t len)
{
    uint8_t i;
    eeprom_update_byte(&ee_magic, 0xFF);        /* invalidate first: a power cut mid-write -> default PIN */
    for (i = 0; i < len; i++)
        eeprom_update_byte((uint8_t *)&ee_pin[i], (uint8_t)p[i]);
    eeprom_update_byte(&ee_len, len);
    eeprom_update_byte(&ee_magic, PIN_MAGIC);   /* valid again only when everything is written */
    memcpy(stored_pin, p, len);
    stored_len = len;
}

/* ------------------------------ LCD screens ------------------------------ */
static void ui_show(const char *l1, const char *l2)
{
    lcd_print_line(0, l1);
    lcd_print_line(1, l2);
}

static void ui_idle(void)
{
    msg_active = 0;
    ui_show("SMART DOOR LOCK", "WAITING VOICE");
}

/* temporary message that falls back to the idle screen after MESSAGE_HOLD_MS */
static void ui_message(const char *l1, const char *l2)
{
    ui_show(l1, l2);
    msg_active = 1;
    msg_deadline = millis() + MESSAGE_HOLD_MS;
}

static void ui_pin_stars(void)                /* "ENTER PIN:" is 10 chars -> stars from col 10 */
{
    uint8_t i;
    lcd_set_cursor(1, 10);
    for (i = 0; i < PIN_MAX_LEN; i++)
        lcd_putc(i < pin_len ? '*' : ' ');
}

/* ------------------------------ helpers ------------------------------ */
static void clear_auth(void)                  /* wipe voice permission AND typed PIN */
{
    voice_verified = 0;
    pin_len = 0;
    memset(pin_buf, 0, sizeof(pin_buf));
}

static void send_status(void)
{
    switch (state) {
    case ST_PIN_ENTRY: uart_puts("WAITING_PIN\n"); break;
    case ST_UNLOCKED:  uart_puts("UNLOCKED\n");    break;
    case ST_CHANGE_PIN: uart_puts("CHANGING_PIN\n"); break;
    default:           uart_puts("LOCKED\n");      break;
    }
}

static uint8_t is_locked_out(void)
{
    if (!lockout_active)
        return 0;
    if (time_reached(lockout_until)) {
        lockout_active = 0;
        wrong_attempts = 0;
        return 0;
    }
    return 1;
}

/* constant-time comparison: no early exit, length checked too */
static uint8_t pin_matches(void)
{
    const char *ref = stored_pin;
    const uint8_t ref_len = stored_len;
    uint8_t diff = (uint8_t)(pin_len ^ ref_len), i;
    char a, b;

    for (i = 0; i < PIN_MAX_LEN; i++) {
        a = (i < pin_len) ? pin_buf[i] : 0;
        b = (i < ref_len) ? ref[i] : 0;
        diff |= (uint8_t)(a ^ b);
    }
    return (diff == 0) ? 1 : 0;
}

/* ------------------------------ Bluetooth commands ------------------------------ */
static void cmd_voice_ok(void)
{
    if (state != ST_LOCKED) {                 /* repeated / mistimed: change nothing, do NOT extend the timer */
        send_status();
        return;
    }
    if (is_locked_out()) {
        uart_puts("ACCESS_DENIED\n");
        ui_message("TOO MANY TRIES", "TRY LATER");
        return;
    }
    clear_auth();
    voice_verified = 1;
    state = ST_PIN_ENTRY;
    pin_deadline = millis() + PIN_TIMEOUT_MS;
    msg_active = 0;
    uart_puts("WAITING_PIN\n");
    ui_show("VOICE VERIFIED", "ENTER PIN:");
}

static void cmd_voice_fail(void)
{
    if (state == ST_UNLOCKED || state == ST_CHANGE_PIN) {   /* door already open: ignore, report truth */
        send_status();
        return;
    }
    clear_auth();
    state = ST_LOCKED;                        /* servo is already locked in these states */
    uart_puts("ACCESS_DENIED\n");
    ui_message("VOICE FAILED", "ACCESS DENIED");
}

static void cmd_lock(void)                    /* works in EVERY state */
{
    servo_lock();
    clear_auth();
    state = ST_LOCKED;
    msg_active = 0;
    uart_puts("LOCKED\n");
    ui_show("DOOR LOCKED", "WAITING VOICE");
}

static void handle_command(const char *cmd)
{
    if      (strcmp(cmd, "VOICE_OK")   == 0) cmd_voice_ok();
    else if (strcmp(cmd, "VOICE_FAIL") == 0) cmd_voice_fail();
    else if (strcmp(cmd, "LOCK")       == 0) cmd_lock();
    else if (strcmp(cmd, "STATUS")     == 0) send_status();
    /* anything else: rejected silently, no state change */
}

/* ------------------------------ keypad ------------------------------ */
static void submit_pin(void)
{
    uint8_t ok = pin_matches();

    clear_auth();                             /* voice permission is single-use, win or lose */

    if (ok) {
        wrong_attempts = 0;
        lockout_active = 0;
        servo_unlock();
        state = ST_UNLOCKED;
        unlock_deadline = millis() + UNLOCK_DURATION_MS;
        uart_puts("UNLOCKED\n");
        ui_show("ACCESS GRANTED", "A=CHANGE PIN");
    } else {
        state = ST_LOCKED;                    /* servo never moved */
        if (wrong_attempts < 255)
            wrong_attempts++;
#if MAX_WRONG_ATTEMPTS > 0
        if (wrong_attempts >= MAX_WRONG_ATTEMPTS) {
            lockout_active = 1;
            lockout_until = millis() + LOCKOUT_MS;
        }
#endif
        uart_puts("ACCESS_DENIED\n");
        ui_message("WRONG PIN", "ACCESS DENIED");
    }
}

static void enter_change_pin(void)
{
    clear_auth();
    state = ST_CHANGE_PIN;
    change_deadline = millis() + PIN_TIMEOUT_MS;
    uart_puts("CHANGING_PIN\n");
    ui_show("CHANGE PIN", "NEW PIN:");
}

static void back_to_unlocked(const char *l1, const char *l2)
{
    clear_auth();
    state = ST_UNLOCKED;
    unlock_deadline = millis() + UNLOCK_DURATION_MS;
    uart_puts("UNLOCKED\n");
    ui_show(l1, l2);
}

static void change_pin_key(char k)
{
    if (k >= '0' && k <= '9') {
        if (pin_len < PIN_MAX_LEN) {
            pin_buf[pin_len++] = k;
            change_deadline = millis() + PIN_TIMEOUT_MS;
            ui_pin_stars();
        }
    } else if (k == KEY_CLEAR) {
        pin_len = 0;
        memset(pin_buf, 0, sizeof(pin_buf));
        ui_pin_stars();
    } else if (k == KEY_CANCEL) {
        back_to_unlocked("DOOR UNLOCKED", "A=CHANGE PIN");
    } else if (k == KEY_ENTER) {
        if (pin_len < PIN_MIN_LEN) {
            pin_len = 0;
            memset(pin_buf, 0, sizeof(pin_buf));
            ui_show("MIN 4 DIGITS", "NEW PIN:");
        } else {
            pin_save(pin_buf, pin_len);
            wrong_attempts = 0;
            lockout_active = 0;
            back_to_unlocked("PIN CHANGED", "DOOR UNLOCKED");
            uart_puts("PIN_CHANGED\n");
        }
    }
}

static void on_key(char k)
{
    if (state == ST_UNLOCKED) {               /* only reachable after voice + correct PIN */
        if (k == KEY_CHANGE)
            enter_change_pin();
        return;
    }
    if (state == ST_CHANGE_PIN) {
        change_pin_key(k);
        return;
    }
    /* Both checks on purpose: keys are discarded unless voice was verified. */
    if (state != ST_PIN_ENTRY || !voice_verified) {
        pin_len = 0;
        return;
    }
    if (k >= '0' && k <= '9') {
        if (pin_len < PIN_MAX_LEN) {
            pin_buf[pin_len++] = k;
            ui_pin_stars();
        }
    } else if (k == KEY_CLEAR) {
        pin_len = 0;
        memset(pin_buf, 0, sizeof(pin_buf));
        ui_pin_stars();
    } else if (k == KEY_ENTER) {
        if (pin_len > 0)
            submit_pin();
    }
    /* A-D: ignored */
}

/* ------------------------------ timeouts ------------------------------ */
static void check_timeouts(void)
{
    switch (state) {
    case ST_PIN_ENTRY:
        if (time_reached(pin_deadline)) {     /* VOICE_OK expired */
            clear_auth();
            state = ST_LOCKED;
            uart_puts("LOCKED\n");
            ui_idle();
        }
        break;
    case ST_UNLOCKED:
        if (time_reached(unlock_deadline)) {  /* automatic relock */
            servo_lock();
            clear_auth();
            state = ST_LOCKED;
            uart_puts("LOCKED\n");
            ui_idle();
        }
        break;
    case ST_CHANGE_PIN:
        if (time_reached(change_deadline)) {  /* gave up: relock for safety */
            servo_lock();
            clear_auth();
            state = ST_LOCKED;
            uart_puts("LOCKED\n");
            ui_idle();
        }
        break;
    default:
        if (msg_active && time_reached(msg_deadline))
            ui_idle();
        break;
    }
}

/* ------------------------------ main ------------------------------ */
int main(void)
{
    char cmd[CMD_MAX_LEN + 1];
    char k;

    wdt_enable(WDTO_2S);          /* a hang resets into the SAFE (locked) state */

    servo_init();                 /* 1st: servo is driven to LOCK_POSITION at power-up */
    timebase_init();
    uart_init();
    keypad_init();
    sei();

    lcd_init();                   /* auto-detects 0x27 / 0x3F; lock works even if absent */

    state = ST_LOCKED;
    clear_auth();
    pin_load();                   /* PIN from EEPROM (default if blank) */
    ui_idle();
    uart_puts("LOCKED\n");
    next_scan = millis() + KEYPAD_SCAN_MS;

    for (;;) {
        wdt_reset();

        if (uart_poll_line(cmd))
            handle_command(cmd);

        if (time_reached(next_scan)) {
            next_scan = millis() + KEYPAD_SCAN_MS;   /* no catch-up: keeps debounce honest */
            k = keypad_scan();
            if (k)
                on_key(k);
        }

        check_timeouts();
    }
}

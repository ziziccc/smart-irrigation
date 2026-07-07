// I2C Address: 0x21 (Pump actuator node)
// Receives PWM value from controller (0x25) and drives water pump via L9110s H-bridge
// Timer0: Fast PWM on OC0A (PD6) ~976Hz | Timer2: 3-second auto-off watchdog

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

#define MY_ADDRESS 0x21

volatile unsigned long timer2_overflow_count = 0;
const unsigned long PUMP_OFF_COUNT = 183; // ~3 seconds at Timer2 prescaler 1024

const int UBRR = 103;

volatile bool pwm_is_active = false;
volatile uint8_t pwm_val = 0;
volatile uint8_t sender_id = 0;
volatile bool expecting_sender_id = true;

void timer2_init(void) {
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
    // TOIE2 enabled only when pump is active
}

ISR(TIMER2_OVF_vect) {
    if (pwm_is_active) {
        timer2_overflow_count++;
        if (timer2_overflow_count >= PUMP_OFF_COUNT) {
            // Auto-off: disable Timer0 PWM output
            TCCR0A &= ~(1 << COM0A1);
            OCR0A = 0;
            TIMSK2 &= ~(1 << TOIE2);
            timer2_overflow_count = 0;
            pwm_is_active = false;
        }
    }
}

void timer0_pwm_init(void) {
    DDRD |= (1 << PD6) | (1 << PD5); // OC0A, OC0B as output
    // Fast PWM, non-inverting on OC0A, prescaler 64 → ~976Hz
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 0;
}

void twi_slave_listen(void) {
    uint8_t status = TWSR & 0xF8;

    if (!(TWCR & (1 << TWINT))) return;

    switch (status) {
        case 0x60: // SLA+W received, ACK returned
            expecting_sender_id = true;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        case 0x80: // Data byte received, ACK returned
            if (expecting_sender_id) {
                sender_id = TWDR;
                expecting_sender_id = false;
            } else {
                uint8_t actual_data = TWDR;
                // Only accept from controller (0x25)
                if (sender_id == 0x25) {
                    pwm_val = actual_data;
                    OCR0A = pwm_val;
                    TCCR0A |= (1 << COM0A1); // enable PWM output
                    TCNT2 = 0;
                    timer2_overflow_count = 0;
                    TIMSK2 |= (1 << TOIE2);  // start watchdog
                    pwm_is_active = true;
                }
            }
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        case 0xA0: // STOP or REPEATED START received
            expecting_sender_id = true;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        default:
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;
    }
}

void UART_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void string_transmit(const char data[]) {
    for (int i = 0; data[i] != '\0'; i++) UART_transmit(data[i]);
}

void send_number(uint16_t num) {
    char buf[10];
    itoa(num, buf, 10);
    string_transmit(buf);
}

void send_number_hex(uint8_t num) {
    char buf[4];
    itoa(num, buf, 16);
    string_transmit(buf);
}

void setup() {
    UART_init(UBRR);
    string_transmit("Pump actuator node initialized. My address is 0x");
    send_number_hex(MY_ADDRESS);
    string_transmit("\n");

    PORTC |= (1 << PC4) | (1 << PC5); // SDA, SCL pull-up

    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWAR = (MY_ADDRESS << 1);
    TWCR = (1 << TWEN) | (1 << TWEA);

    timer0_pwm_init();
    timer2_init();
    SREG |= (1 << 7);
}

void loop() {
    twi_slave_listen();

    static bool prev_active = false;
    if (pwm_is_active != prev_active) {
        prev_active = pwm_is_active;
        if (pwm_is_active) {
            string_transmit("Pump ON: PWM=");
            send_number(pwm_val);
            string_transmit("\n");
        } else {
            string_transmit("Pump OFF (auto-off or PWM=0)\n");
        }
    }
}

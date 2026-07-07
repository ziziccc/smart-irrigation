// I2C Address: 0x25 (Controller node) — Main P-controller for smart irrigation
//
// This node acts as BOTH slave (receives sensor data from 0x20 and 0x28)
// AND master (transmits PWM command to pump actuator at 0x21).
//
// Control law:
//   err = FC - hum_data
//   k   = K_DRY (0.40) if err > 0, else K_WET (0.80)
//   pwm += k * err       [clamped to 0-254]
//
// Blocking conditions:
//   night_block  : light_data == 1 (nighttime) → pump inhibited
//   soil_block   : hum_data >= TRIGGER (soil sufficiently moist) → pump inhibited

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

#define SOIL_ADDRESS    0x20
#define LIGHT_ADDRESS   0x28
#define MY_ADDRESS      0x25
#define TARGET_ADDRESS  0x21

// Soil water constants (% volumetric water content)
#define FC      47.8f       // Field Capacity
#define TRIGGER 37.87f      // FC - RAW  (47.8 - 9.93)
#define K_DRY   0.40f       // Proportional gain when soil is dry  (err > 0)
#define K_WET   0.80f       // Proportional gain when soil is wet   (err < 0)

const unsigned long PWM_INTERVAL    = 61 * 15;  // 15 seconds: send PWM to actuator
const unsigned long UPDATE_INTERVAL = 61 * 5;   // 5 seconds:  update PWM via P-controller
const int UBRR = 103;

volatile unsigned long timer2_overflow_count = 0;

volatile float   err        = 0.0f;
volatile float   pwm        = 150.0f; // initial PWM
volatile uint16_t hum_data  = 0;
volatile uint8_t  light_data = 0;
volatile bool night_block   = true;
volatile bool soil_block    = true;

volatile uint8_t sender_id = 0;
volatile bool expecting_sender_id = true;

static unsigned long last_pwm_count    = 0;
static unsigned long last_update_count = 0;

void timer2_init(void) {
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
    TIMSK2 |= (1 << TOIE2);
}

ISR(TIMER2_OVF_vect) {
    timer2_overflow_count++;
}

// Non-blocking TWI slave poll — called from loop()
void twi_slave_listen(void) {
    uint8_t status = TWSR & 0xF8;

    if (!(TWCR & (1 << TWINT))) return;

    switch (status) {
        case 0x60: // Own SLA+W received, ACK returned
            expecting_sender_id = true;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        case 0x80: // Data byte received, ACK returned
            if (expecting_sender_id) {
                sender_id = TWDR;
                expecting_sender_id = false;
            } else {
                uint8_t actual_data = TWDR;
                switch (sender_id) {
                    case SOIL_ADDRESS:   hum_data   = actual_data; break;
                    case LIGHT_ADDRESS:  light_data = actual_data; break;
                }
            }
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        case 0xA0: // STOP or REPEATED START
            expecting_sender_id = true;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;

        default:
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
            break;
    }
}

bool twi_master_transmit(unsigned char* data, uint8_t length) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != 0x08) return false;

    TWDR = (TARGET_ADDRESS << 1) | 0;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    switch (TWSR & 0xF8) {
        case 0x18: break;
        case 0x20:
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            return false;
        case 0x38:
            TWCR = (1 << TWINT) | (1 << TWEN);
            return false;
        default:
            return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        TWDR = data[i];
        TWCR = (1 << TWINT) | (1 << TWEN);
        while (!(TWCR & (1 << TWINT)));
        if ((TWSR & 0xF8) != 0x28) {
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            return false;
        }
    }

    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    return true;
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

void send_float(float f) {
    char buf[16];
    dtostrf(f, 4, 2, buf);
    string_transmit(buf);
}

void send_number_hex(uint8_t num) {
    char buf[4];
    itoa(num, buf, 16);
    string_transmit(buf);
}

void setup() {
    UART_init(UBRR);
    string_transmit("Controller node initialized. My address is 0x");
    send_number_hex(MY_ADDRESS);
    string_transmit("\n");

    PORTC |= (1 << PC4) | (1 << PC5); // SDA, SCL pull-up

    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWAR = (MY_ADDRESS << 1);
    TWCR = (1 << TWEN) | (1 << TWEA);

    timer2_init();
    SREG |= (1 << 7);
}

void loop() {
    unsigned long now = timer2_overflow_count;

    // Listen for incoming sensor data (non-blocking)
    twi_slave_listen();

    // UPDATE_INTERVAL (5s): recalculate PWM via P-controller
    if (now - last_update_count >= UPDATE_INTERVAL) {
        last_update_count = now;

        night_block = (light_data == 1);
        soil_block  = ((float)hum_data >= TRIGGER);

        err = FC - (float)hum_data;
        float k = (err > 0.0f) ? K_DRY : K_WET;
        pwm += k * err;

        if (pwm < 0.0f)   pwm = 0.0f;
        if (pwm > 254.0f) pwm = 254.0f;

        string_transmit("[UPDATE] hum=");
        send_number(hum_data);
        string_transmit("% light=");
        send_number(light_data);
        string_transmit(" err=");
        send_float(err);
        string_transmit(" pwm=");
        send_float(pwm);
        string_transmit(" night_block=");
        send_number(night_block);
        string_transmit(" soil_block=");
        send_number(soil_block);
        string_transmit("\n");
    }

    // PWM_INTERVAL (15s): send PWM command to pump actuator
    if (now - last_pwm_count >= PWM_INTERVAL) {
        last_pwm_count = now;

        if (night_block) {
            string_transmit("[PWM] Blocked: nighttime\n");
        } else if (soil_block) {
            string_transmit("[PWM] Blocked: soil moisture above trigger\n");
        } else {
            unsigned char send_data[2];
            send_data[0] = MY_ADDRESS;
            send_data[1] = (unsigned char)pwm;

            string_transmit("[PWM] Sending PWM=");
            send_number((uint16_t)pwm);
            string_transmit(" to 0x");
            send_number_hex(TARGET_ADDRESS);
            string_transmit("\n");

            if (twi_master_transmit(send_data, 2)) {
                string_transmit("  -> Send success\n");
            } else {
                string_transmit("  -> Send failed\n");
            }

            // Switch back to slave mode after master TX
            TWAR = (MY_ADDRESS << 1);
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
        }
    }
}

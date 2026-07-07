// I2C Address: 0x28 (Light sensor node)
// Reads LDR (photoresistor) via ADC and transmits day/night flag to controller (0x25)
// Multi-Master I2C using AVR TWI hardware registers

#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

#define MY_ADDRESS      0x28
#define TARGET_ADDRESS  0x25

volatile unsigned long timer2_overflow_count = 0;
const unsigned long TRANSMIT_INTERVAL = 61 * 3;
const int UBRR = 103;

volatile uint8_t received_sender_id;
volatile bool expecting_sender_id = true;

int sensorValue;
int previousSensorValue;

void timer2_init(void) {
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
    TIMSK2 |= (1 << TOIE2);
}

ISR(TIMER2_OVF_vect) {
    timer2_overflow_count++;
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

uint16_t readADC(uint8_t channel) {
    ADMUX  = (1 << REFS0) | (channel & 0x07);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    uint16_t result = ADCL;
    result |= ((uint16_t)ADCH << 8);
    return result;
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

void string_transmit_ln(const char data[]) {
    for (int i = 0; data[i] != '\0'; i++) UART_transmit(data[i]);
    UART_transmit('\n');
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
    string_transmit_ln("Multi-Slave Polling Master Initialized.");

    PORTC |= (1 << PC4) | (1 << PC5); // SDA, SCL pull-up

    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWAR = (MY_ADDRESS << 1);
    TWCR = (1 << TWEN) | (1 << TWEA);

    timer2_init();
    SREG |= (1 << 7); // global interrupt enable

    string_transmit("Node initialized. My address is 0x");
    send_number_hex(MY_ADDRESS);
    string_transmit("\n");
}

void loop() {
    static unsigned long previous_overflow_count = 0;

    int aboveValue;
    int diffValue;
    unsigned char scaledValue;
    int a;

    const int dayOrNight       = 700;
    const int absNight         = 500;
    const int absDay           = 800;
    const int compareSensorValue = 300;

    if (timer2_overflow_count >= TRANSMIT_INTERVAL) {
        previousSensorValue = sensorValue;
        sensorValue = readADC(0);

        aboveValue = (sensorValue > dayOrNight) ? 0 : 1; // 0=day, 1=night
        diffValue  = (abs(sensorValue - previousSensorValue) > compareSensorValue) ? 1 : 0;

        if ((sensorValue < absNight) || (diffValue && aboveValue)) {
            a = 1; // night
        } else if ((sensorValue > absDay) || (diffValue && !aboveValue)) {
            a = 0; // day
        }
        scaledValue = a;

        string_transmit("Master: Attempting to send [ID, Value] -> [0x");
        send_number_hex(MY_ADDRESS);
        string_transmit(", ");
        send_number(scaledValue);
        string_transmit("]\nLight Flux: ");
        send_number(sensorValue);
        string_transmit("\n");
        string_transmit_ln(scaledValue ? "night" : "day");

        unsigned char data_packet[2];
        data_packet[0] = MY_ADDRESS;
        data_packet[1] = scaledValue;

        if (twi_master_transmit(data_packet, 2)) {
            string_transmit("Master: Send success!\n");
        } else {
            string_transmit("Master: Send failed (Bus busy, NACK, or Arbitration Lost).\n");
        }
        string_transmit("\n");

        TIMSK2 &= ~(1 << TOIE2);
        timer2_overflow_count = 0;
        TIMSK2 |= (1 << TOIE2);

        TWAR = (MY_ADDRESS << 1);
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
    }
}

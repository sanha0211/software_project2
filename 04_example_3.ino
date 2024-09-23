#define PIN_LED 13
unsigned int count = 0;
int toggle = 0;

void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200); // Initialize serial port
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    Serial.println("Hello World!");
    digitalWrite(PIN_LED, LOW); // start with LED off.
}

void loop() {
    Serial.println(++count);
    toggle = !toggle; // toggle LED value (invert 0/1).
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(1000); // wait for 1,000 milliseconds
}

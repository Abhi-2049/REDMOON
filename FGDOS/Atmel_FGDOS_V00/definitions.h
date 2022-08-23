// Micro pins SPI
// pins for slave selection SPI
#define SS1 9 
#define SS2 8
// I2C slave address definition
#define SLAVE_ADDRESS 0x25
// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF

// Register definitions
#define TEMP 0x00
#define RECHARGE_COUNT 0x01
#define REF_WINDOW 0x0B
#define CHARGE_SENS 0x0C
#define RECHARGE 0x0D
#define NIRQ_ENGATE 0x0E // NIRQ pin not used for now
#define TARGET 0x09
#define THRESHOLD 0x0A

// sensor constant definitions (these depend on settings in Arduino and sensor!)
#define THRESHOLD_FREQ 30000 // these values are not exact! Since only 8 MSBs are used
#define TARGET_FREQ 50000
#define CK_FREQ 31250 // depends on the settings of the PWM
#define WINDOW_PULSES 8192.0f // depends on window register settings!
#define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)
#define BYTES_SEND 13 // amount of bytes to send over I2C

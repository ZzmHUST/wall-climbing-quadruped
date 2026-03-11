#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===================== User Configuration ===================== */
// Select the UART handle (defined in main.c)
#define SERVO_UART_HANDLE   huart1

// Enable direction control pin for half-duplex mode (refer to 74HC126 in protocol doc)
#define SERVO_USE_DIR_CTRL  0       
#if SERVO_USE_DIR_CTRL
    #define SERVO_DIR_PORT      GPIOA
    #define SERVO_DIR_PIN       GPIO_PIN_8
    #define SERVO_DIR_TX()      HAL_GPIO_WritePin(SERVO_DIR_PORT, SERVO_DIR_PIN, GPIO_PIN_SET)
    #define SERVO_DIR_RX()      HAL_GPIO_WritePin(SERVO_DIR_PORT, SERVO_DIR_PIN, GPIO_PIN_RESET)
#endif

/* ===================== Protocol Constants ===================== */
#define SERVO_FRAME_HEADER1     0x55
#define SERVO_FRAME_HEADER2     0x55
#define SERVO_BROADCAST_ID      0xFE        // Broadcast ID 254

// Angle resolution: 0-1000 corresponds to 0-240 degrees, i.e., 0.24 deg/unit
#define ANGLE_TO_VALUE(angle)   ((uint16_t)((angle) / 0.24f + 0.5f))
#define VALUE_TO_ANGLE(value)   ((float)(value) * 0.24f)

/* Command codes (refer to protocol document Table 2) */
typedef enum {
    CMD_MOVE_TIME_WRITE     = 0x01,     // Immediate move, Length=7
    CMD_MOVE_TIME_READ      = 0x02,     // Read angle/time, Length=3
    CMD_MOVE_WAIT_WRITE     = 0x07,     // Buffered move (wait for start), Length=7
    CMD_MOVE_WAIT_READ      = 0x08,     // Read buffered values, Length=3
    CMD_MOVE_START          = 0x0B,     // Start buffered action, Length=3
    CMD_MOVE_STOP           = 0x0C,     // Immediate stop, Length=3
    CMD_ID_WRITE            = 0x0D,     // Change ID (saved to EEPROM), Length=4
    CMD_ID_READ             = 0x0E,     // Read ID, Length=3
    CMD_ANGLE_OFFSET_ADJUST = 0x11,     // Adjust angle offset, Length=4
    CMD_ANGLE_OFFSET_WRITE  = 0x12,     // Save offset, Length=3
    CMD_ANGLE_LIMIT_WRITE   = 0x14,     // Set angle limits, Length=7
    CMD_ANGLE_LIMIT_READ    = 0x15,     // Read angle limits, Length=3
    CMD_VIN_LIMIT_WRITE     = 0x16,     // Set voltage limits, Length=7
    CMD_VIN_LIMIT_READ      = 0x17,     // Read voltage limits, Length=3
    CMD_TEMP_MAX_LIMIT_WRITE= 0x18,     // Set temperature limit, Length=4
    CMD_TEMP_MAX_LIMIT_READ = 0x19,     // Read temperature limit, Length=3
    CMD_TEMP_READ           = 0x1A,     // Read current temperature, Length=3
    CMD_VIN_READ            = 0x1B,     // Read current voltage, Length=3
    CMD_POS_READ            = 0x1C,     // Read current position, Length=3
    CMD_OR_MOTOR_MODE_WRITE = 0x1D,     // Switch servo/motor mode, Length=7
    CMD_OR_MOTOR_MODE_READ  = 0x1E,     // Read mode, Length=3
    CMD_LOAD_OR_UNLOAD_WRITE= 0x1F,     // Power on/off control, Length=4
    CMD_LOAD_OR_UNLOAD_READ = 0x20,     // Read power state, Length=3
    CMD_LED_CTRL_WRITE      = 0x21,     // LED control, Length=4
    CMD_LED_CTRL_READ       = 0x22,     // Read LED state, Length=3
    CMD_LED_ERROR_WRITE     = 0x23,     // Error alarm config, Length=4
    CMD_LED_ERROR_READ      = 0x24      // Read error state, Length=3
} ServoCmd_t;

/* Packet structure (supports up to 4 bytes of parameters) */
typedef struct {
    uint8_t header[2];      // 0x55 0x55
    uint8_t id;             // Servo ID
    uint8_t length;         // Data length (=3 + parameter bytes)
    uint8_t cmd;            // Command code
    uint8_t params[4];      // Parameters (max 4 bytes, varies by command)
    uint8_t checksum;       // Checksum byte
} ServoPacket_t;

/* ===================== Function Declarations ===================== */

// Initialization (pass UART handle pointer)
void Servo_Init(UART_HandleTypeDef *huart);

// Basic control
void Servo_MoveToAngle(uint8_t id, float angle, uint16_t time_ms);      // Immediate move
void Servo_MoveToAngle_Wait(uint8_t id, float angle, uint16_t time_ms); // Buffered move (requires Start)
void Servo_Start(uint8_t id);                                            // Start buffered action (can use broadcast ID)
void Servo_Stop(uint8_t id);                                             // Immediate stop

// Multi-servo synchronized control (for quadruped gait)
void Servo_SyncMove(uint8_t *ids, float *angles, uint16_t *times, uint8_t count);

// Configuration
void Servo_SetID(uint8_t old_id, uint8_t new_id);                        // Change servo ID (saved to EEPROM)
void Servo_Load(uint8_t id);                                             // Servo power on (torque enabled)
void Servo_Unload(uint8_t id);                                           // Servo power off (torque disabled, free to move)
void Servo_SetAngleLimit(uint8_t id, uint16_t min_angle_val, uint16_t max_angle_val); // Set angle limits

// Status reading (blocking with timeout)
bool Servo_ReadPosition(uint8_t id, float *angle_out);                   // Read current angle
bool Servo_ReadTemperature(uint8_t id, uint8_t *temp_out);               // Read current temperature (Celsius)
bool Servo_ReadVoltage(uint8_t id, float *voltage_out);                  // Read current voltage (Volts)

#endif /* __SERVO_H */

#include "servo.h"

/* Internal UART handle pointer */
static UART_HandleTypeDef *servo_huart = NULL;

/* Internal static functions */
static uint8_t CalcChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t *params, uint8_t param_len);
static void SendPacket(ServoPacket_t *pkt, uint8_t param_len);
static void FillPacket(ServoPacket_t *pkt, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);

/**
 * @brief Initialize servo control handle
 * @param huart Pass the UART handle configured in CubeMX (e.g., &huart1)
 */
void Servo_Init(UART_HandleTypeDef *huart) {
    servo_huart = huart;
    
#if SERVO_USE_DIR_CTRL
    // Configure direction control pin as push-pull output
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SERVO_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SERVO_DIR_PORT, &GPIO_InitStruct);
    SERVO_DIR_RX(); // Default to receive mode
#endif
}

/**
 * @brief Calculate checksum: ~(ID + Length + Cmd + Params...)
 */
static uint8_t CalcChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t *params, uint8_t param_len) {
    uint16_t sum = id + length + cmd;
    for (uint8_t i = 0; i < param_len; i++) {
        sum += params[i];
    }
    return ~(uint8_t)(sum & 0xFF); // Keep low byte only, then bitwise NOT
}

/**
 * @brief Fill packet structure
 * @note  Length field = 3 + parameter byte count (according to protocol doc)
 */
static void FillPacket(ServoPacket_t *pkt, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len) {
    pkt->header[0] = SERVO_FRAME_HEADER1;
    pkt->header[1] = SERVO_FRAME_HEADER2;
    pkt->id = id;
    pkt->cmd = cmd;
    pkt->length = param_len + 3; // 1(Length itself) + 1(Cmd) + 1(Checksum placeholder)
    
    if (params != NULL && param_len > 0) {
        memcpy(pkt->params, params, param_len);
    }
    
    pkt->checksum = CalcChecksum(id, pkt->length, cmd, params, param_len);
}

/**
 * @brief Send packet (blocking mode, can be replaced with DMA)
 * @param param_len Actual parameter byte count N, total send length = 6 + N
 */
static void SendPacket(ServoPacket_t *pkt, uint8_t param_len) {
    // Total length calculation: Header(2) + ID(1) + Length(1) + Cmd(1) + Params(N) + Checksum(1) = 6+N
    uint8_t total_len = 6 + param_len;
    
    if (servo_huart == NULL) return;
    
#if SERVO_USE_DIR_CTRL
    SERVO_DIR_TX();
    // Direction switch delay, adjust based on 74HC126 switching time, usually 1-5us is enough, use 1ms for stability
    HAL_Delay(1); 
#endif

    HAL_UART_Transmit(servo_huart, (uint8_t*)pkt, total_len, 100);
    // To use DMA, comment out the line above and use:
    // HAL_UART_Transmit_DMA(servo_huart, (uint8_t*)pkt, total_len);
    
#if SERVO_USE_DIR_CTRL
    // Wait for transmission complete flag (TC)
    while(__HAL_UART_GET_FLAG(servo_huart, UART_FLAG_TC) == RESET);
    SERVO_DIR_RX();
#endif
}

/* ===================== Basic Control Functions ===================== */

/**
 * @brief Move servo to specified angle immediately
 * @param id: Servo ID (0-253), 254 for broadcast
 * @param angle: Target angle 0.0 ~ 240.0 degrees
 * @param time_ms: Move time 0 ~ 30000 ms (controls speed, 0 for maximum speed)
 */
void Servo_MoveToAngle(uint8_t id, float angle, uint16_t time_ms) {
    ServoPacket_t pkt;
    uint16_t angle_val;
    
    // Limit protection
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 240.0f) angle = 240.0f;
    if (time_ms > 30000) time_ms = 30000;
    
    angle_val = ANGLE_TO_VALUE(angle);
    
    uint8_t params[4] = {
        (uint8_t)(angle_val & 0xFF),        // Angle low byte
        (uint8_t)((angle_val >> 8) & 0xFF), // Angle high byte
        (uint8_t)(time_ms & 0xFF),          // Time low byte
        (uint8_t)((time_ms >> 8) & 0xFF)    // Time high byte
    };
    
    FillPacket(&pkt, id, CMD_MOVE_TIME_WRITE, params, 4);
    SendPacket(&pkt, 4);
}

/**
 * @brief Buffer angle command (does not execute immediately, requires Servo_Start)
 * @note  Used for multi-servo synchronization: send this to all servos first, then broadcast Start to move simultaneously
 */
void Servo_MoveToAngle_Wait(uint8_t id, float angle, uint16_t time_ms) {
    ServoPacket_t pkt;
    uint16_t angle_val;
    
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 240.0f) angle = 240.0f;
    if (time_ms > 30000) time_ms = 30000;
    
    angle_val = ANGLE_TO_VALUE(angle);
    
    uint8_t params[4] = {
        (uint8_t)(angle_val & 0xFF),
        (uint8_t)((angle_val >> 8) & 0xFF),
        (uint8_t)(time_ms & 0xFF),
        (uint8_t)((time_ms >> 8) & 0xFF)
    };
    
    FillPacket(&pkt, id, CMD_MOVE_WAIT_WRITE, params, 4);
    SendPacket(&pkt, 4);
}

/**
 * @brief Start buffered action (used with Wait command)
 * @param id: Can be single ID or broadcast ID (254) to start all servos simultaneously
 */
void Servo_Start(uint8_t id) {
    ServoPacket_t pkt;
    FillPacket(&pkt, id, CMD_MOVE_START, NULL, 0);
    SendPacket(&pkt, 0);
}

/**
 * @brief Stop movement immediately
 */
void Servo_Stop(uint8_t id) {
    ServoPacket_t pkt;
    FillPacket(&pkt, id, CMD_MOVE_STOP, NULL, 0);
    SendPacket(&pkt, 0);
}

/* ===================== Advanced Functions ===================== */

/**
 * @brief Multi-servo synchronized movement (for quadruped robot gait)
 * @param ids: Array of servo IDs
 * @param angles: Array of target angles (degrees)
 * @param times: Array of move times (ms, can be NULL for default time)
 * @param count: Number of servos
 */
void Servo_SyncMove(uint8_t *ids, float *angles, uint16_t *times, uint8_t count) {
    if (ids == NULL || angles == NULL || count == 0) return;
    
    uint16_t default_time = 500;
    
    // Phase 1: Send buffered commands
    for (uint8_t i = 0; i < count; i++) {
        uint16_t t = (times != NULL) ? times[i] : default_time;
        Servo_MoveToAngle_Wait(ids[i], angles[i], t);
        // According to protocol, delay between commands to avoid bus collision
        HAL_Delay(2); 
    }
    
    // Phase 2: Broadcast start command, all servos begin movement simultaneously
    HAL_Delay(5); // Ensure all buffered commands are received
    Servo_Start(SERVO_BROADCAST_ID);
}

/**
 * @brief Change servo ID (saved to EEPROM, use with caution)
 * @param old_id: Current ID (can use broadcast if unknown, but ensure only one servo on bus)
 * @param new_id: New ID (0-253)
 */
void Servo_SetID(uint8_t old_id, uint8_t new_id) {
    ServoPacket_t pkt;
    uint8_t params[1] = {new_id};
    
    FillPacket(&pkt, old_id, CMD_ID_WRITE, params, 1);
    SendPacket(&pkt, 1);
    HAL_Delay(100); // Wait for EEPROM write to complete
}

/**
 * @brief Power on servo (torque enabled)
 */
void Servo_Load(uint8_t id) {
    ServoPacket_t pkt;
    uint8_t params[1] = {1}; // 1 = Load motor
    FillPacket(&pkt, id, CMD_LOAD_OR_UNLOAD_WRITE, params, 1);
    SendPacket(&pkt, 1);
}

/**
 * @brief Power off servo (torque disabled, can be moved by hand)
 */
void Servo_Unload(uint8_t id) {
    ServoPacket_t pkt;
    uint8_t params[1] = {0}; // 0 = Unload motor
    FillPacket(&pkt, id, CMD_LOAD_OR_UNLOAD_WRITE, params, 1);
    SendPacket(&pkt, 1);
}

/**
 * @brief Set angle limits (prevent mechanical damage from over-travel)
 * @param min_angle_val: Minimum angle value (0-1000)
 * @param max_angle_val: Maximum angle value (0-1000)
 */
void Servo_SetAngleLimit(uint8_t id, uint16_t min_angle_val, uint16_t max_angle_val) {
    ServoPacket_t pkt;
    uint8_t params[4] = {
        (uint8_t)(min_angle_val & 0xFF),
        (uint8_t)((min_angle_val >> 8) & 0xFF),
        (uint8_t)(max_angle_val & 0xFF),
        (uint8_t)((max_angle_val >> 8) & 0xFF)
    };
    FillPacket(&pkt, id, CMD_ANGLE_LIMIT_WRITE, params, 4);
    SendPacket(&pkt, 4);
    HAL_Delay(50); // Wait for EEPROM save
}

/* ===================== Status Reading Functions ===================== */

/**
 * @brief Read current angle (blocking reception)
 * @return true if success, false if timeout or checksum error
 */
bool Servo_ReadPosition(uint8_t id, float *angle_out) {
    ServoPacket_t pkt;
    FillPacket(&pkt, id, CMD_POS_READ, NULL, 0);
    
#if SERVO_USE_DIR_CTRL
    SERVO_DIR_TX();
    HAL_Delay(1);
#endif
    
    // Send read command (6 bytes)
    HAL_UART_Transmit(servo_huart, (uint8_t*)&pkt, 6, 50);
    
#if SERVO_USE_DIR_CTRL
    while(__HAL_UART_GET_FLAG(servo_huart, UART_FLAG_TC) == RESET);
    SERVO_DIR_RX();
#endif
    
    // Receive response: Header(2)+ID(1)+Length(1)+Cmd(1)+AngleLow(1)+AngleHigh(1)+Checksum(1) = 8 bytes
    uint8_t rx_buf[8];
    if (HAL_UART_Receive(servo_huart, rx_buf, 8, 100) == HAL_OK) {
        // Simple header check
        if (rx_buf[0] == SERVO_FRAME_HEADER1 && rx_buf[1] == SERVO_FRAME_HEADER2) {
            uint16_t angle_val = rx_buf[5] | (rx_buf[6] << 8); // Low byte first
            *angle_out = VALUE_TO_ANGLE(angle_val);
            return true;
        }
    }
    return false;
}

/**
 * @brief Read internal temperature of servo
 * @param temp_out: Output temperature value (Celsius)
 */
bool Servo_ReadTemperature(uint8_t id, uint8_t *temp_out) {
    ServoPacket_t pkt;
    FillPacket(&pkt, id, CMD_TEMP_READ, NULL, 0);
    
    uint8_t rx_buf[7]; // Returns 7 bytes
    HAL_UART_Transmit(servo_huart, (uint8_t*)&pkt, 6, 50);
    if (HAL_UART_Receive(servo_huart, rx_buf, 7, 100) == HAL_OK) {
        if (rx_buf[0] == SERVO_FRAME_HEADER1) {
            *temp_out = rx_buf[5]; // Temperature value at parameter 1 position
            return true;
        }
    }
    return false;
}

/**
 * @brief Read input voltage
 * @param voltage_out: Output voltage value (Volts)
 */
bool Servo_ReadVoltage(uint8_t id, float *voltage_out) {
    ServoPacket_t pkt;
    FillPacket(&pkt, id, CMD_VIN_READ, NULL, 0);
    
    uint8_t rx_buf[8]; // Returns 8 bytes (voltage is 2 bytes, unit: mV)
    HAL_UART_Transmit(servo_huart, (uint8_t*)&pkt, 6, 50);
    if (HAL_UART_Receive(servo_huart, rx_buf, 8, 100) == HAL_OK) {
        if (rx_buf[0] == SERVO_FRAME_HEADER1) {
            uint16_t mv = rx_buf[5] | (rx_buf[6] << 8);
            *voltage_out = mv / 1000.0f; // Convert to V
            return true;
        }
    }
    return false;
}

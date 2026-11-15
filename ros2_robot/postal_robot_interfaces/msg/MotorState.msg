# Raw, per-motor feedback from the single serial-bus owner. No value is
# refreshed artificially: "fresh" is false when the last valid reply is older
# than the configured maximum age, and the numbers are then the last valid
# reply, never zeros substituted for missing bytes.
uint8 motor_id
string role                 # arm | gripper | wheel_left | wheel_back | wheel_right | dispenser
bool ever_received
bool fresh
bool responding             # last transaction returned a valid packet
float64 age_s               # since last valid reply; -1 when never received
int32 position_ticks
int32 speed_ticks_s         # signed, bit 15 decoded
int32 load_raw              # signed, bit 10 decoded
int32 current_raw
float32 voltage_v
int32 temperature_c
uint8 servo_status          # register 0x41 (servo error flags)
uint8 packet_error          # status byte of the reply packet
bool torque_enabled         # register 0x28
bool moving                 # register 0x42
uint8 mode                  # register 0x21 read at start-up: 0 position, 1 wheel
string fault                # empty when healthy

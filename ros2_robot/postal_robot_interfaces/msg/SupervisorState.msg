uint8 WAITING_FOR_STATION=0   # no lease since start-up; actuator commands refused
uint8 ACTIVE=1                # lease valid and acknowledged
uint8 LINK_LOST=2             # lease expired: base and dispenser stopped, arm held
uint8 AWAITING_ACK=3          # lease back (or new session): explicit acknowledge required
uint8 FAULT=4                 # latched fault: base/dispenser stopped, arm held
std_msgs/Header header
uint8 state
string state_name
string session_id
float64 lease_age_s
bool arm_torque
bool base_enabled
bool dispenser_enabled
bool arm_busy
bool base_moving
bool dispenser_running
float64 dispenser_remaining_s
string[] faults
string[] warnings
string last_event

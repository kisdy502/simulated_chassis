# Asynchronous load_map/relocalize operation. Terminal result is immutable.
std_msgs/Header header
string operation_id
string operation_type          # LOAD_MAP | RELOCALIZE
string map_name
string state                   # RUNNING | SUCCEEDED | FAILED | TIMED_OUT
string stage                   # ACCEPTED | STOPPING_LOCALIZATION | STARTING_LOCALIZATION | WAITING_LOCALIZATION | FINISHED
bool finished
bool success
string message
bool pose_initialized          # snapshot, not proof of geometric localization accuracy
bool navigation_ready          # snapshot only; SUCCEEDED means localization completed, not Nav2 ready
float64 elapsed_sec             # steady-clock duration, fixed at completion

# Current mapped cursor sample in task coordinates.
#
# Consumers must reject samples whose session_id/trial_id do not match their
# current StudyTask. input_valid=false is an explicit invalidation sample; its
# position must not be used for interaction, estimation, or control.
string session_id
uint32 trial_id
builtin_interfaces/Time stamp
geometry_msgs/Point position
bool input_valid

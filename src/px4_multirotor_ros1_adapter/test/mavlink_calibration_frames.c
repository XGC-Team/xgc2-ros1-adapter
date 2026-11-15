#include <stdio.h>
#include "common/mavlink.h"
static void output(mavlink_message_t *msg) { uint8_t b[300]; unsigned n=mavlink_msg_to_send_buffer(b,msg); for(unsigned i=0;i<n;i++)printf("%02x",b[i]); puts(""); }
int main(void) { mavlink_message_t m; mavlink_msg_heartbeat_pack(7,1,&m,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_PX4,0,0,MAV_STATE_STANDBY);output(&m); mavlink_msg_heartbeat_pack(7,100,&m,MAV_TYPE_CAMERA,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);output(&m);mavlink_msg_command_ack_pack(7,1,&m,511,MAV_RESULT_ACCEPTED,0,0,0,0);output(&m);return 0; }

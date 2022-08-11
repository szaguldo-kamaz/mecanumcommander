#
# NLAB-MecanumCommander for Linux, a simple bridge to control VStone MecanumRover 2.1 / VStone MegaRover 3
# by David Vincze, vincze.david@webcode.hu
# at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021-2022
# version 0.60
# https://github.com/szaguldo-kamaz/
#
# Example client
#

import time
from mecanumcommander_client import MecanumCommanderClient

mecacomm_ip = "127.0.0.1";
mecacomm_protocol = 1;  # 0 - TCP, 1 - UDP
mecacomm_port = 3475;
mecacomm_passwd = "PASSWORD";

print("Connecting to mecanumcommander at %s:%d"%(mecacomm_ip, mecacomm_port));
mc = MecanumCommanderClient(mecacomm_protocol, mecacomm_ip, mecacomm_port, mecacomm_passwd);

# rotate speed 600 for 2 seconds (5*0.4s)
# repeat command every <500ms - otherwise mecanumcommander will think that the client got disconnected
for n in range(1,5):
    mc.setXYrot(0, 0, 600);
    time.sleep(0.4);  # 400ms

mc.STOProbot();

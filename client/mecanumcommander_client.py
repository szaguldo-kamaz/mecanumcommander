#
# NLAB-MecanumCommander for Linux, a simple bridge to control VStone MecanumRover 2.1 / VStone MegaRover 3
# by David Vincze, vincze.david@webcode.hu
# at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021-2022
# version 0.60
# https://github.com/szaguldo-kamaz/
#
# Example client class
#

import socket, errno, time, sys, struct
from crc16pure import crc16xmodem as crc16_ccitt


class MecanumCommanderClient:

    def __init__(self, mecacom_protocol = 0, mecacom_ip = "127.0.0.1", mecacom_port = 3475, mecacom_pass = "PASSWORD"):

        self.param__commandsendmintime = 0.025;
        self.protocol = mecacom_protocol;

        if self.protocol == 0:  # TCP
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
            try:
                self.sock.connect((mecacom_ip, mecacom_port));
            except socket.error as serr:
                if serr.errno != errno.ECONNREFUSED:
                    raise serr
                print("MECACOM: Connection to MecanumCommander (%s:%d) failed! Is it running?" % (mecacom_ip, mecacom_port) );
                sys.exit(2);

            banner = self.sock.recv(1024);
            bannerstr = banner.decode('UTF-8').strip();
            if (bannerstr != "I'm NLAB-MecanumCommander. Please authenticate yourself."):
                print("MECACOM: Connected to something, but it is not MecanumCommander: %s\n"%(bannerstr));
                sys.exit(2);
            self.sock.send((mecacom_pass + "\r\n").encode());
            welcome = self.sock.recv(1024);
            welcomestr = welcome.decode('UTF-8').strip();
            print("MECACOM:", welcomestr, "-", welcomestr[0:21], "-", welcomestr[-6:], '--', sep='');
            if welcomestr[0:21] != 'NLAB-MecanumCommander' or welcomestr[-6:] != 'Ready.':
                print("MECACOM: Wrong password for NLAB-MecanumCommander?: %s\n"%(welcomestr));
                sys.exit(2);
            print("MECACOM: Connected to NLAB-MecanumCommander");

        elif self.protocol == 1:  # UDP
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
            self.udpdestination = (mecacom_ip, mecacom_port);
            self.udppacketno = 0;

        else:
            print("MECACOM: Unknown protocol: ", self.protocol);
            sys.exit(2);

        self.lastcmdsent_time = time.time() - 0.5;
        self.lastcmdwasSTOPZERO = False;


    def __del__(self):
        self.sock.close();


    def sendudpcommand(self, commandtosend):
        if self.udppacketno == 0xFFFF:
            self.udppacketno = 0;
        else:
            self.udppacketno += 1;
        packetdata = struct.pack(">H", self.udppacketno) + bytes(commandtosend, 'ascii');
        checksum = crc16_ccitt(packetdata);
        packetdata += struct.pack(">H", checksum);
        self.sock.sendto(packetdata, self.udpdestination);
        print("MECACOM: UDP packet sent: packetno: %d cmd: %s cksum: %x"%(self.udppacketno, commandtosend, checksum));


    def setXYrot(self, speedX, speedY, rotation):

        timenow = time.time();
        timediff = timenow - self.lastcmdsent_time;
        if timediff >= self.param__commandsendmintime:

            if self.protocol == 0:  # TCP
                commandtosend = "SPX%05d\r\nSPY%05d\r\nROT%05d\r\n"%(speedX, speedY, rotation);
                self.sock.send(bytes(commandtosend, 'ascii'));
                print("MECACOM: TCP: cmd sent:", str(commandtosend).replace('\r', '\\r').replace('\n', '\\n'));

            elif self.protocol == 1:  # UDP

                self.sendudpcommand("SPX%05d"%(speedX));
                self.sendudpcommand("SPY%05d"%(speedY));
                self.sendudpcommand("ROT%05d"%(rotation));

            else:
                print("MECACOM: Unknown protocol, this should not happen!");
                sys.exit(2);

            self.lastcmdsent_time = time.time();
            self.lastcmdwasSTOPZERO = False;

        else:

            print("MECACOM: too fast - SPX/SPY/ROT cmd was not sent");


    def STOProbot(self):
        if self.lastcmdwasSTOPZERO:
            print("MECACOM: last command was also STOPZERO - new STOPZERO cmd was not sent");
            return
        timenow = time.time();
        timediff = timenow - self.lastcmdsent_time;
        if timediff >= self.param__commandsendmintime:

            if self.protocol == 0:  # TCP
                self.sock.send(bytes("STOPZERO\r\n", "ascii"));
                print("MECACOM: TCP: STOPZERO sent");

            elif self.protocol == 1:  # UDP
                self.sendudpcommand("STOPZERO");

            else:
                print("MECACOM: Unknown protocol, this should not happen!");
                sys.exit(2);

            self.lastcmdsent_time = time.time();
            self.lastcmdwasSTOPZERO = True;

        else:
            print("MECACOM: too fast - STOPZERO cmd was not sent");


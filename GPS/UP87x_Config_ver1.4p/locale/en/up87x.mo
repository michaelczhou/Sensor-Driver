��    q      �  �   ,      �	     �	  	   �	     �	  
   �	     �	     �	  	   �	     �	  	   �	  
   �	     �	  	   �	     
     
  
   
  
   '
  	   2
  
   <
  
   G
     R
     Z
     b
     j
     s
     �
     �
  
   �
     �
  	   �
  	   �
     �
     �
     �
     �
     �
     �
                         &  
   2  
   =  
   H     S     _  
   k  
   v  
   �  
   �  
   �  
   �  
   �  
   �     �     �     �     �     �                    %     1     =     I  	   U     _     h     v  	   ~  
   �  	   �  
   �     �     �  	   �     �     �  
   �  	   �  	   �  	   �                 
     	   *     4     @     O     [     c     l     t     �     �     �     �     �     �     �     �     �     �     �     �     �     �     �     �     �  H  �     E     V     k     s     �     �     �     �     �     �     �     �     �     �     �     �     �       
                  #  	   (     2     A     T     ]     f     o     u  	   �     �     �  
   �     �  
   �     �  
   �     �     �     �     �     �                  	   1     ;     K     Y     g     t     �     �    �    �  6  �  2  �  F    �   ^  q  �    m    !  �  �$  �  ]'  	  6)     @+     F+     K+     [+     c+     j+     r+     y+  	   �+     �+     �+     �+     �+     �+     �+     �+     �+     �+     �+     �+     �+     �+     �+  	   �+     �+  	   �+  
   	,     ,  	   %,  	   /,     9,     H,     W,     h,  
   n,     y,     �,  
   �,     �,     �,     �,     �,     �,  	   �,  
   �,     -                +       X   K                      a   $              J   W   q       f              *   /   S   ]          #   F       N          l   	       )       U   m   (   `      9   <   =   >   ?      A   B   !   M   h      _                      %   P       0   ^   1       e           @      E   ,   -         2   3   4   5   6   7         H                           8       k          '   G   d          C   Q           
      O   T      Y   i      L   n   I      b         D   R   V   g   p   [       "       c       .   o   :   ;   j   Z       &      \            L_7Para L_EllPara bbt_collect bbt_freset bbt_gga bbt_readCfg bbt_reset bbt_set bcb_ntrip bcb_udpReg bgrp_basePos bgrp_corr bgrp_ipSetting brd_base brd_netTcp brd_netUdp brd_rover brd_serial brd_udpCom bss_Alt bss_Lat bss_Lon bss_Port bss_baudrate bss_deviceinfo bss_gate bss_ipAddr bss_mask bss_meter bss_mount bss_password bss_user bss_vectorDistence bss_workMode bt_Get bt_Set bt_about bt_break bt_exit bt_link btt_waiting cb_logfile help_cmd_0 help_cmd_1 help_cmd_10 help_cmd_11 help_cmd_2 help_cmd_3 help_cmd_4 help_cmd_5 help_cmd_6 help_cmd_7 help_cmd_8 help_cmd_9 help_info_0 help_info_1 help_info_10 help_info_11 help_info_2 help_info_3 help_info_4 help_info_5 help_info_6 help_info_7 help_info_8 help_info_9 lbt_table lbt_veiw pg_Coordinate pg_help pg_manual pg_obsView pg_serial pg_setting pg_statView pg_view sbt_clear sbt_display sbt_joff sbt_noSbas sbt_pause sbt_query sbt_reset sbt_send sbt_test scb_grid scb_marker scb_range sgrp_seting sgrp_trackview srd_current srd_fix srd_mean ss_lang sss_trackcnt sss_viewtype st_CS st_CS1 st_L st_Scale st_X st_Xro st_Xtran st_Y st_Yro st_Ytran st_Zro st_Ztran st_a st_ell st_f st_rate Project-Id-Version: e2687_cfg
POT-Creation-Date: 2014-05-10 11:00+0800
PO-Revision-Date: 2015-04-15 09:39+0800
Last-Translator: walter chen <walnutcy@163.com>
Language-Team: LANGUAGE <www.gnss8.com>
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
Language: en
X-Generator: Poedit 1.6.5
 Seven Parameters Ellipsoid Parameters Collect Factory Reset GGA Pos Read Config Reset SET Ntrip Protocol UDP Register Base Station WGS84 Corr Way Static IP Setting Base net TCP net UDP Rover Serial UDP+Serial Height: Lat: Lon: Net Port: Diff Baudrate: Device Infomation: NetGate: IP Addr: NetMask: Meter Mountpoint: Password: User: Vector Antenna Gap: Work Mode: Read Config Set Config About Disconnect Exit Connect Waiting LogFile Basic Commands Network Config UDP Reg-Mode Trouble Shooting Work Mode Current Setting Device Status BeiDou Status Base Station Rover Station Quick Config UDP Broadcast Device can be configed by Control serial port, which default setting is 115200,No parity,8bit data,1bit stop.
Command format: cmd [para...] <CRLF>
cmd and para are described in ASCII, CRLF means enter and new line.
cmd words are not case sensitive, but some para are case sensitive, such as password, mountpoint. cmd and para are space-delimited

eg. read device config: 
RTU INFO<CRLF>
When base station work with TCP Ntrip Link, we need config the mountpoint name and user, password.
BASE MOUNTPOINT <mount> <password> <CRLF>
note: <mount> means ntrip server's mountpoint name,  <password> means the password for ntrip client login,each para need less than 10 chars, and all case sensitive.

Note: Need reset the device after change the base or rover station setting. Ethernet static IP address setting: ETH IPSTATIC <ip_addr> <ip_mask> <net_gate> <CRLF>
eg.  ETH IPSTATIC 192.168.2.10 255.255.255.0 192.168.2.254 <CRLF>

Ethernet auto DHCP setting: ETH IPDHCP [ON,OFF] <CRLF>
Read Ethernet MAC address: ETH MAC <CRLF> 
  Note: UDP Reg-Mode support after version 6.01.
Base station quick setup:
1) ETH IPDHCP OFF<CRLF> 
2) ETH IPSTATIC 192.168.2.6 255.255.255.0 192.168.2.1<CRLF>
3) RTKSRC UDP<CRLF>
4) RTU MODE BASE<CRLF>
5) BASE UDPPORT 6001<CRLF>
6) BASE POSITION 34.19147681 108.85722886 399.328<CRLF>
7) BASE DIFF RTCM3<CRLF>
8) RTU RESET<CRLF>

Rover station quick setup:
1) ETH IPDHCP OFF<CRLF> 
2) ETH IPSTATIC 192.168.2.21 255.255.255.0 192.168.2.1<CRLF>
3) RTKSRC UDP<CRLF>
4) RTU MODE ROVER<CRLF>
5) ROVER UDPPORT 192.168.2.6:6001<CRLF>
6) RTU RESET<CRLF>
    Simple trouble shooting.


Control serial port's baudrate is 115200.
1)  Lock base station postion：
        $JMODE,FIXLOC,YES<CRLF>
        $JAVE<CRLF>
2)  Connect radio link with diff serial port, and output GGA data from control serial port:
        $JASC,GPGGA,5<CRLF>
        $JAVE<CRLF>
   The Device work mode is based on the correction data link, currently supported link interface included RJ45 Ethernet, 3PIN RS232 serial. Ethernet interface support standard TCP/IP v4 protocol, Application can work on TCP or UDP by setting.
Device can work as base station (broadcast correction data) or rover station (receive correction data).
Base station can work as UDP base, serial base, Ntrip Caster(TCP), Ntrip Server(TCP). Rover station can work as UDP rover, serial rover, Ntrip Client.

There are two commands used to config the work mode: RTU MODE [BASE,ROVER,SERVER] <CRLF>
RTKSRC [TCP,UDP,COM,UDPCOM] <CRLF>
TCP/UDP use the ethernet port, COM use the diff serial port, SERVER only work on TCP link. UDPCOM means UDP and Serial work at the same time, used to support RF-Radio, GGA output from control port currently.
    Read device config information: 
RTU INFO <CRLF> 
Read base position setting: BASE POSITION <CRLF>
Read base correction data format: BASE DIFF <CRLF>
   Commonly used command list:
Read CPU ID: RTU UDID <CRLF>
Read ethernet MAC: ETH MAC <CRLF> 
Read Firmware&Hardware info: TOP<CRLF>
Read Control&Diff serial port configuration: BAUD <CRLF> 
Change Diff serial port baudrate: BAUD DIFF [9600,19200,38400,57600,115200]<CRLF>
Read GNSS board serial No, Firmware version: $JI<CRLF>

Reset device: RTU RESET<CRLF>
    There are 2 version hardware which support BeiDou. 
version 1: Only support BDS B1 freq, and not included in RTK solution; support output position and satillites info that get from BDS;
                  BDMSG GGA [0,1]<CRLF> --- ouput GBGGA or not from control serial port
                  BDMSG RMC [0,1]<CRLF> --- ouput GBRMC or not from control serial port
                  BDMSG GSV [0,1]<CRLF> --- ouput GBGSV or not from control serial port
                  BDMSG ALL [0,1]<CRLF> 
version 2: Support BDS B1,B2,B3, and included in RTK solution;
                  $JASC,PSAT,RTKSTAT,[1,0]<CRLF> --- output RTKSTAT or not
RTKSTAT format reference:
$PSAT,RTKSTAT,FIX,RTCM3,1,01FF,0.1,(,L1,L2,G1,G2,B1,B2,B3,)(,5,4,4,3,8,8,8,)(,A,A,A,A,A,A,A,),952,0,0.064,014,0*11
      Base station setup need  WGS84 position, diff correction type and Link interface and protocol.
Change the base station position with WGS84 format:  BASE POSITION lat lon alt<CRLF>
lat and lon are all descripted in degree format,  alt  in meter.  
eg:  BASE POSITION 34.19147681 108.85722886 399.328<CRLF>
Correction protocol set:  BASE DIFF [RTCM3,CMR]<CRLF>1) Serial Base station:  RTKSRC COM <CRLF>
2) UDP Base Station: RTKSRC UDP<CRLF>
                    BASE UDPPORT [a.b.c.d:]port <CRLF>3) Ntrip Caster: RTKSRC TCP<CRLF>
                    BASE TCPPORT port<CRLF>                    BASE MOUNTPOINT mount password<CRLF>4) Ntrip Server:  RTKSRC TCP<CRLF>                   SERVER TCPPORT a.b.c.d:port<CRLF>                   SERVER MOUNTPOINT mount password<CRLF>     Ref to different interface and protocol, work mode have some little different.
1) UDP Rover: RTKSRC UDP<CRLF>  --- use UDP Link;
             ROVER UDPPORT [a.b.c.d:]port <CRLF> --- a.b.c.d define the server IP address
2) Ntrip Client(TCP)：RTKSRC TCP<CRLF>  --- use TCP Link;
             ROVER TCPPORT a.b.c.d:port<CRLF> --- a.b.c.d define the server IP address
             ROVER MOUNTPOINT mount password<CRLF> --- mount means server mountpoint, password means ntripclient password which used to login server, each para need less than 10 chars, and case sensitive.
3) Serial Rover：RTKSRC COM<CRLF>  --- use diff serial Link, default baudrate is 19200, can be changed by command 'BAUD DIFF baud <CRLF> 
     Quick Config describe how to config the base or rover to broadcast or get correction data by UDP Link,mainly for Driving School Users. UDP Link support broadcast in local area network, or Reg-mode in all network.


Setting step:
1) Config Ethernet，using DHCP or static IP Address;
2) Select Link by command 'RTKSRC';
3) Select work mode by command 'RTU MODE';
4) Set detail para with work mode;
5) Reset device.

Note: UDP Reg-Mode support after version6.01.   Base station quick setup:
1) ETH IPDHCP OFF<CRLF> 
2) ETH IPSTATIC 192.168.2.6 255.255.255.0 192.168.2.1<CRLF>
3) RTKSRC UDP<CRLF>
4) RTU MODE BASE<CRLF>
5) BASE UDPPORT 192.168.2.255:6001<CRLF>
6) BASE POSITION 34.19147681 108.85722886 399.328<CRLF>
7) BASE DIFF RTCM3<CRLF>
8) RTU RESET<CRLF>
"
Rover station quick setup:
1) ETH IPDHCP OFF<CRLF> 
2) ETH IPSTATIC 192.168.2.21 255.255.255.0 192.168.2.1<CRLF>
3) RTKSRC UDP<CRLF>
4) RTU MODE ROVER<CRLF>
5) ROVER UDPPORT 6001<CRLF>
6) RTU RESET<CRLF>
    Table View Coordinate para History Manual ObsView Serial Setting TrackView View Clear display $JOFF No SBAS Pause Query Reset Send Test Grid Marker Range TrackSetting TrackView Current Point Fix Point Mean Point Lang(Need reset) TrackCnt: ViewType: Current Config Current Config Central Meridian Scale Constant X X Rotation(s) X Translation(m) Constant Y Y Rotation(s) Y Translation(m) Z Rotation(s) Z Translation(m) Major Radius(m) Ellipsoid Flattening Output rate(Hz) 
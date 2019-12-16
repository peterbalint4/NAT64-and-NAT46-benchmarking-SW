# NAT64-and-NAT46-benchmarking-SW
Benchmarking Software for NAT64 and NAT46 tests


Summary:

Simple easy-to-use RFC 8219 compliant application for NAT64 and NAT46 Benchmarking "nat64bench"

Installation:

1, GCC, Install DPDK, DPDK devel and DPDK tool
2, Verify dpdk with a sample app https://doc.dpdk.org/guides/sample_app_ug/hello_world.html
3, Copy NAT64/46 benchmarking app to a folder (f.e.: folder of examples inside dpdk) 
4, Compiling App. Easiest way with the use of the Makefile in one folder of example apps. Suggested to modify the name to "APP = nat64_tester"
5, Bind NICs verify with "dpdk-devbind --status"
6, Run App

Configuration:

nat64bench use the following input
parameters:
• -s IPv6 frame size (in bytes, 84-1518), IPv4 frames are automatically calculated by the Tester.
• -t time of testing (in seconds, 1-250)
• -r frame rate (frames per second)
• -l number of the latency packets per seconds.
• -o timeout (in milliseconds), the tester stops receiving when this timeout elapsed after sending finished. (Late frames will be accounted as lost ones.)
• -p Sending repeat allowed in case the NIC is busy (0 - deny repeat; 1 - permit repeat)
• -n and -m specifying the proportion of foreground and background traffic. Traffic proportion is expressed by two relative prime numbers n and m, where m packets form every n packet belong to the foreground traffic and the rest (m-n) packets belong to the background traffic.


Configuration File parameters:

IPv6-Real 2001:1::2;
IPv6-Virtual 2001:2:0:1::192.19.0.2;
IPv4-Real 192.19.0.2;
IPv4-Virtual 192.18.0.2;
IPv6-Background 2001:3:0:8000::2;
Tester_MAC_L 00:1e:68:37:91:96;
Tester_MAC_R 00:1e:68:37:91:97;
DUT_MAC_L 00:15:17:d8:07:60;
DUT_MAC_R 00:15:17:d8:07:61;
CPU-Left-Send 4;
CPU-Right-Receive 5;
CPU-Right-Send 6;
CPU-Left-Receive 7;



Example Run:

build/nat64bench -s 84 -t 200 -r 2000000 -l 0 -o 10 -n 2 -m 2
Forward frames sent: 400000000
Forward frames received: 400000000
Reverse frames sent: 400000000
Reverse frames received: 400000000

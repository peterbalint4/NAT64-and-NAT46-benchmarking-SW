# NAT64-and-NAT46-benchmarking-SW
Benchmarking Software for NAT64 and NAT46


Summary:<br/><br/>

Simple easy-to-use RFC 8219 compliant application for NAT64 and NAT46 Benchmarking "nat64bench"<br/><br/>

Installation:<br/><br/>

1, Install DPDK, DPDK devel and DPDK tool<br/>
2, Verify dpdk with a sample app https://doc.dpdk.org/guides/sample_app_ug/hello_world.html<br/>
3, Download NAT64/46 benchmarking app  <br/>
4, Compiling App. Easiest way with the usage of an Example App Makefile. Suggested to modify the app name in the Makefile "APP = nat64_tester"<br/>
5, If needed bind NICs and verify with "dpdk-devbind --status"<br/>
6, Run App<br/><br/>

Configuration:<br/><br/>

nat64bench use the following input
parameters:<br/>
• -s IPv6 frame size (in bytes, 84-1518), IPv4 frames are automatically calculated by the Tester.<br/>
• -t time of testing (in seconds, 1-250)<br/>
• -r frame rate (frames per second)<br/>
• -l number of the latency packets per seconds.<br/>
• -o timeout (in milliseconds), the tester stops receiving when this timeout elapsed after sending finished. (Late frames will be accounted as lost ones.)<br/>
• -n and -m specifying the proportion of foreground and background traffic. Traffic proportion is expressed by two relative prime numbers n and m, where m packets form every n packet belong to the foreground traffic and the rest (m-n) packets belong to the background traffic.<br/><br/>


Configuration File parameters:<br/><br/>

IPv6-Real 2001:1::2;<br/>
IPv6-Virtual 2001:2:0:1::192.19.0.2;<br/>
IPv4-Real 192.19.0.2;<br/>
IPv4-Virtual 192.18.0.2;<br/>
IPv6-Background 2001:3:0:8000::2;<br/>
Tester_MAC_L 00:1e:68:37:91:96;<br/>
Tester_MAC_R 00:1e:68:37:91:97;<br/>
DUT_MAC_L 00:15:17:d8:07:60;<br/>
DUT_MAC_R 00:15:17:d8:07:61;<br/>
CPU-Left-Send 4;<br/>
CPU-Right-Receive 5;<br/>
CPU-Right-Send 6;<br/>
CPU-Left-Receive 7;<br/><br/>



Example Run:<br/><br/>

build/nat64bench -s 84 -t 200 -r 2000000 -l 0 -o 10 -n 2 -m 2<br/>

Example Results:<br/><br/>

Forward frames sent: 400000000<br/>
Forward frames received: 400000000<br/>
Reverse frames sent: 400000000<br/>
Reverse frames received: 400000000<br/>

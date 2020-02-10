/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2019 Peter Balint
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>

#include <time.h>
#include <rte_timer.h>

#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_hash_crc.h>
#include <rte_ip.h>
#include <arpa/inet.h>

static volatile bool force_quit; /* Ctrl-C was pressed by the user */


#define NB_MBUF 2000 /* Maximum permitted number of message buffers */
#define MAX_PKT_BURST 10 /* Maximum number of packets recived in a burst */
#define MAX_PPS 1800000 /* Maximum packet/sec */
#define MEMPOOL_CACHE_SIZE 256 /* EAL parameter ... */
#define MAX_TEST_DURATION 200 /* Max test duration in second*/
#define MAX_LATENCY_PKTPS 800 /* Max number of Latency packets in a second */
#define MAX_PACKET_SIZE 1532 /* Max packet size */
#define MAX_RX_QUEUE_PER_LCORE 16 
#define MAX_TX_QUEUE_PER_PORT 16
#define RTE_TEST_RX_DESC_DEFAULT 128
#define RTE_TEST_TX_DESC_DEFAULT 512
static uint16_t nb_rxd = RTE_TEST_RX_DESC_DEFAULT;
static uint16_t nb_txd = RTE_TEST_TX_DESC_DEFAULT;

/* packet template */

uint8_t l2[14] = {//L2
    0x00,0x15,0x17,0x54,0xb6,0x60, 	/* Destination MAC address */ 
    0x00,0x15,0x17,0x54,0x98,0xe8,	/* Source MAC address */
    0x08,0x00,						/* Ether Type: IPv4 */

};  

 uint8_t l2_v6[14] = {//L2 v6
    0x00,0x14,0x5e,0xc2,0xfe,0x50,	/* Destination MAC address */ 
    0x08,0x00,0x27,0xbe,0x78,0x77,	/* Source MAC address */
    0x86,0xdd,						/* Ether Type: IPv6 */

};     

uint8_t l3_v4[20] = {//L3v4
	0x45,0x00,0x00,0x30,
    0x66,0xaf,0x40,0x00,
    0x40,0x11,0x00,0x00,
    0xbe,0x12,0x01,0x02,//SIP
    0xc0,0x13,0x01,0x02,//DIP

};

 uint8_t l3_v6[40] = {//L3v6
    0x60,0x30,0x00,0x00,
    0x00,0x1c,0x11,0x80,
    0xfe,0x80,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x02,0x01,0x00,0xff,
    0xfe,0x00,0x00,0x00,
    0x20,0x01,0x00,0x02,
    0x00,0x01,0x00,0x01,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x01,
};

uint8_t l4_data[26]= {//L4	
    0x83,0xc3,0x07,0xd1,	/* UDP header */ 
    0x00,0x1a,0x00,0x00,	/* UDP header */
    0x49,0x44,0x45,0x4e, 	/* IDEN */
    0x00,0x00,0x00,0x00, 	/* latency packet ID */
    0x00,0x00,0x00,0x00,	/* time stamp */
    0xAA,0xBB,0xCC,0xDD,	/* time stamp */
    0x00,0x00,
};



uint32_t sent1 = 0,sent2 = 0,received1 = 0,received2 = 0,hz, sending_ring_full1 = 0,sending_ring_full2 = 0; /* storing the count of sended and recived packets and system frequency*/
uint16_t lat_received1 = 0, lat_received2 = 0; /* count of recived latency packets */
uint32_t delay1[MAX_TEST_DURATION*MAX_LATENCY_PKTPS] = {0}, delay2[MAX_TEST_DURATION*MAX_LATENCY_PKTPS] = {0}; /*storing delay */
int recv_ctrl = 0; /* used to stop reciving functions  */

char cpu_list[20];
uint8_t ctrl1 = 0; /* used at the sending syncronisation */

uint32_t bgd_pkt = 999, lat_pkt = 999; /* background and latency scheduler (invalid values) */
uint64_t p_send_cyc; /*  the number of cycles between packets (1/frequency) */
int flr=0,flr1=0,flr2=0; /*  failure indicator */

uint8_t fgd_1_IPv6[16]; /*  IPv6 address, foreground and backgorund Tester Left side */
uint8_t fgd_2_IPv6[16]; /*  foreground IPv6 destination address (IPv4-embedded IPv6 address) */
uint8_t fgd_1_IPv4[4];  /*  foreground IPv4 address, Tester side */
uint8_t fgd_2_IPv4[4];  /*  foreground IPv4 address, virtually assigned to Tester Left side */
uint8_t bgd_1_IPv6[16]; /*  background IPv6 address, Tester Right side */

uint8_t Tester_MAC_R[6]; /*  general (fore- and background) Tester Right MAC address */
uint8_t DUT_MAC_R[6]; /*  general (fore- and background) DUT Right MAC address */
uint8_t Tester_MAC_L[6]; /*  general (fore- and background) Tester Left MAC address */
uint8_t DUT_MAC_L[6]; /*  general (fore- and background) DUT Left MAC address */

uint16_t v4s_snd_CPU = 99; /* Right side CPU IDs (invalid values)*/ 
uint16_t v4s_rcv_CPU = 99; /* Right side CPU IDs (invalid values)*/
uint16_t v6s_snd_CPU = 99; /* Left side CPU IDs (invalid values)*/
uint16_t v6s_rcv_CPU = 99; /* Left side CPU IDs (invalid values)*/




struct rte_mempool * pktmbuf_pool = NULL;


static const struct rte_eth_conf port_conf = {
	.rxmode = {
		.split_hdr_size = 0,
	},
	.txmode = {
		.mq_mode = ETH_MQ_TX_NONE,
		
	},
};

uint64_t timer_period = 60; /* default period is 60 seconds */
uint32_t pkt_size = 64; /* default packet size 64 byte */
uint32_t pkt_sizev6 = 84; /* default packet size 64 byte */
uint64_t pktps = 20; /* default packet per sec */
uint16_t lat_rate = 0; /* Latency packet /100 packet */
uint16_t time_out = 100; /* timeout in milliseconds */
uint8_t repeat = 0; /* Sending repeat allowed/disallowed in case the NIC is busy */
uint8_t warm_up = 0; /* unmeasured sec before test */
uint32_t mod_n = 100; /* traffic proportion modulo n */
uint32_t mod_m = 10; /* traffic proportion: if ( packet_count % mod_n < m ) then foreground */

float median(uint32_t n, uint32_t x[]);
int sender1(void *dummy);
int sender2(void *dummy);
int receive2(void *dummy);
int receive1(void *dummy);
float percentile(uint32_t n, uint32_t x[]); 
int conf_rd(void);
void nat64_tester_bp_conf_usage(void);

/* display usage */
static void
nat64_tester_bp_usage(const char *prgname)
{
	printf("Input Error %s [EAL options] -- \n"
	       "  -s v6 frame size in bytes 84-1518\n"
		   "  -t time of testing 1-%d s\n"
		   "  -r Packet/s max: %d \n"			   
		   "  -l Latency packet/s  max: %d\n"
		   "  -o Waiting time after sending finished in ms\n max 1000"
		   "  -p Sending repeat allowed in case the NIC is busy (0 - deny repeat; 1 - permit repeat)"
		   "  -n Background traffic proportion modulo n  \n"
		   "  -m traffic proportion: if ( packet_count % mod_n < m ) then foreground  \n",
	       prgname,MAX_TEST_DURATION,MAX_PPS,MAX_LATENCY_PKTPS);
}

/* config file usage */
void
nat64_tester_bp_conf_usage(void)
{
	printf("Input Error"
		   "IPv6-Real 2001:db8:85a3:0:0:8a2e:370:7554 \n"
	       "IPv6-Virtual 2001:0db8:85a3:0000:0000:8a2e:0370:7664 \n"
		   "IPv4-Real 192.168.10.1 \n"
		   "IPv4-Virtual 192.168.10.1 \n"	
		   "IPv6-Background 2001:0db8:85a3:0000:0000:8a2e:0370:7774 \n"
		   "Tester_MAC_L FF:FF:FF:FF:FF:FF \n"	
		   "Tester_MAC_R FF:FF:FF:FF:FF:FF \n"	
		   "DUT_MAC_L FF:FF:FF:FF:FF:FF \n"	
		   "DUT_MAC_R FF:FF:FF:FF:FF:FF \n"
		   "CPU-Left-Send  Integer 0-90 \n"	
		   "CPU-Left-Receive  Integer 0-90 \n"	
		   "CPU-Right-Send  Integer 0-90 \n"	
		   "CPU-Right-Receive  Integer 0-90 \n");
}
static unsigned int
nat64_tester_bp_parse_size(const char *q_arg)
{
	char *end = NULL;
	unsigned long n;

	/* parse string */
	n = strtoul(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return 0;
	if (n == 0)
		return 0;


	return n;
}

static int
nat64_tester_bp_parse_timer_period(const char *q_arg)
{
	char *end = NULL;
	int n;

	/* parse number string */
	n = strtol(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return -1;
	if (n >= MAX_TEST_DURATION)
		return -1;

	return n;
}

static int
nat64_tester_bp_parse_gen(const char *q_arg)
{
	char *end = NULL;
	uint64_t n;

	/* parse number string */
	n = strtol(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
		return -1;


	return n;
}


/* Parse the arguments*/
static int
nat64_tester_bp_parse_args(int argc, char **argv)
{
	int opt, chk, timer_secs, nat64_tester_bp_pkt_size;
	uint64_t nat64_tester_bp_pktps;
	char **argvopt;
	char *prgname = argv[0];


	argvopt = argv;

	while ((opt = getopt(argc, argvopt, "s:t:r:l:o:w:n:m:p:")) != EOF) {

		switch (opt) {
		/* size */
		case 's':
			nat64_tester_bp_pkt_size = nat64_tester_bp_parse_size(optarg);
			if ((nat64_tester_bp_pkt_size < 84) || (nat64_tester_bp_pkt_size >1538)){
				printf("invalid packet size\n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
				pkt_size = nat64_tester_bp_pkt_size - 20;
				pkt_sizev6 = nat64_tester_bp_pkt_size;
			break;

		/* timer period */
		case 't':
			timer_secs = nat64_tester_bp_parse_timer_period(optarg);
			if ((timer_secs < 1) || (timer_secs >MAX_TEST_DURATION)) {
				printf("invalid timer period\n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
			timer_period = timer_secs;
			break;
		/* pktps */
		case 'r':
			nat64_tester_bp_pktps = nat64_tester_bp_parse_gen(optarg);
			if (nat64_tester_bp_pktps < 1 || nat64_tester_bp_pktps > MAX_PPS) {
				printf("invalid pktps \n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
			pktps = nat64_tester_bp_pktps;
			break;
			
		/* Latency rate /s*/
		case 'l':
			lat_rate = nat64_tester_bp_parse_gen(optarg);
			if (lat_rate > MAX_LATENCY_PKTPS) {
				printf("invalid Latency rate\n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
			
		/* Timeout */
		case 'o':
			time_out = nat64_tester_bp_parse_gen(optarg);
			if (time_out > 100000) {
				printf("invalid Timeout value\n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
			break;

		/* Repeat */
		case 'p':
			repeat = nat64_tester_bp_parse_gen(optarg);
			if (repeat > 1) {
				printf("invalid Repeat value\n");
				nat64_tester_bp_usage(prgname);
				return -1;
			}
			break;			

		/* Background traffic proportion modulo n  */
		case 'n':
			mod_n = nat64_tester_bp_parse_gen(optarg);
			
		
			
			
			break;			
		/* Traffic proportion: if ( packet_count % mod_n < m ) then foreground */
		case 'm':
			mod_m = nat64_tester_bp_parse_gen(optarg);
			
			
			
			break;			
		
		
		case 0:
			break;

		default:
			nat64_tester_bp_usage(prgname);
			return -1;
		}
	}


	

	chk = optind-1;  /* error if the count of options is <1 */
	optind = 0; /* reset getopt */
	
	return chk;
}


/* Signal handling*/
static void
signal_handler(int signum) 
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\n %d Signal received, preparing to exit...\n",
				signum);
		force_quit = true;
	}
}



/* Reciving packets 1 */
int
receive1(void *dummy) 
{
	uint8_t eth_id = 1;
	int recv, incr, r_i; /* work variables  */
	uint8_t *pointr;  /* pointer to the start of the data in the mbuf */
	uint64_t *tstmp, stmp;
	struct rte_mbuf *pktr_burst1[MAX_PKT_BURST]; /* Store the rte_mbufs  */
	struct rte_mbuf *pktr; /* Store the rte_mbuf for processing  */
	

	while (recv_ctrl<1){
		
		if (force_quit) break;
		recv = rte_eth_rx_burst(eth_id, 0, pktr_burst1, MAX_PKT_BURST); 
		for (r_i = 0; r_i < recv; r_i++){
			pktr = pktr_burst1[r_i];
			pointr = rte_pktmbuf_mtod(pktr, uint8_t *);
			incr = 0;
			
			if ((*(pointr+12) == 0x86)  && (*(pointr+13)  == 0xdd)) { /* check IP version  */
				incr=20;
			}
			if ((*(pointr+incr+42) == 0x49)  && (*(pointr+incr+43)  == 0x44)) { /* check "IDen" */
				received1++;
				
				if (*(pointr+incr+46) == 0xAA) {
					tstmp= (uint64_t*)(pointr+(50+incr));
					stmp=*tstmp;					
			
					delay1[lat_received1] = rte_rdtsc() - stmp; lat_received1++;
				}
				
				rte_pktmbuf_free(pktr);
			}
			
		}
	}
	return 0;
}



/* Reciving packets 2 */
int
receive2(void *dummy) 
{
	uint8_t eth_id = 0;
	int recv, incr, r_i; /* work variables  */
	uint8_t *pointr;  /* pointer to the start of the data in the mbuf */
	uint64_t *tstmp, stmp;
	struct rte_mbuf *pktr_burst2[MAX_PKT_BURST]; /* Store the rte_mbufs  */
	struct rte_mbuf *pktr; /* Store the rte_mbuf for processing  */

	while (recv_ctrl<1){

		if (force_quit) break;
		recv = rte_eth_rx_burst(eth_id, 0, pktr_burst2, MAX_PKT_BURST); 
		for (r_i = 0; r_i < recv; r_i++){
			pktr = pktr_burst2[r_i];
			pointr = rte_pktmbuf_mtod(pktr, uint8_t *);
			incr = 0;
			
		
			if ((*(pointr+12) == 0x86)  && (*(pointr+13)  == 0xdd)) { /* check IP version  */
				incr=20;
			}
			
			if ((*(pointr+incr+42) == 0x48)  && (*(pointr+incr+43)  == 0x44)) { /* check "IDen" */
				received2++;
				
				if (*(pointr+incr+46) == 0xAA) {
					
					tstmp= (uint64_t*)(pointr+(50+incr));
					stmp=*tstmp;		
		
					delay2[lat_received2] = rte_rdtsc() - stmp; lat_received2++;
				}			
				rte_pktmbuf_free(pktr);
			}
			
		}
	}
	return 0;
}

/* Sending packets 1  */
int
sender1(void *dummy) 
{
	uint8_t eth_id = 1;
	uint8_t *pointr4, *pointr6; /* pointer to the start of the data in the mbuf */
	uint8_t sent; /* work variables  */
	struct rte_mbuf *pkts, *pkts_back; /* mbufs for foregound and background */
	uint16_t size1=0x0000, size2=0x0000, sizechks,sizechks_v4; /* L2 and L3 sizes */
	uint16_t reset = 0x0000; /* reset checksum */
	uint16_t ip_chks; /* IPv4 checksum */
	uint32_t cksum,cksum2; /* UDP checksum */
	uint16_t *magic1,*magic2; /* work variables  */
	uint8_t *magic5,*magic6; /* work variables  */
	uint32_t count = 0, countb=0, countc = 1, countd; /* work variables  */
	uint8_t l_id = 0xAA; /*latency packet id */
	uint8_t l_reset = 0x00; /*latency packet id reset */
	uint16_t count2 = 0x0000; /* latency packet counter */
    uint8_t *magic3,*magic4; /*  direction ID */
    uint64_t strt; /* starting TSC register value at the beginning of the test */
    uint64_t cur_tsc = 0; /* current TSC register values */
	uint64_t st_tsc; /* TSC register values */
	uint64_t *time_stmp1,*time_stmp2;
	struct ipv4_hdr ipv4_hdr;
	struct ipv6_hdr ipv6_hdr;
	/*Allocate a new mbufs from a mempool.*/   
	pkts = rte_pktmbuf_alloc(pktmbuf_pool);
	pkts_back = rte_pktmbuf_alloc(pktmbuf_pool);
	 /* update packet template with pckt size: substract the size of the CRC */
	pkts->data_len = (pkt_size-4);
	pkts->pkt_len = (pkt_size-4);
	pkts_back->data_len = (pkt_sizev6-4);
	pkts_back->pkt_len = (pkt_sizev6-4);
	/*pointer to the start of the data in the mbuf*/
	pointr4 = rte_pktmbuf_mtod(pkts, uint8_t *);
	pointr6 = rte_pktmbuf_mtod(pkts_back, uint8_t *);
	
	printf("Sender 1 OK on %d Lcore\n",rte_lcore_id());
	
	/*  v4 side fgd */
	/* calculate ip checksum ad update packet template */
		
	size1=htons(pkt_size-(14+4)); // Ethernet header: 14, Ethernet CRC: 4
	

	/* copy packet to the mbuf*/
	rte_memcpy(pointr4, &l2, 14);
	rte_memcpy(pointr4, &DUT_MAC_R, 6);
	rte_memcpy(pointr4+6, &Tester_MAC_R, 6);
	rte_memcpy((pointr4+14), &l3_v4, 20);
	rte_memcpy((pointr4+26), &fgd_1_IPv4, 4);
	rte_memcpy((pointr4+30), &fgd_2_IPv4, 4);
	rte_memcpy((pointr4+16), &size1, 2);
	
	rte_memcpy(&ipv4_hdr,(pointr4+14),20); 
	
	ip_chks=rte_ipv4_cksum(&ipv4_hdr);

	rte_memcpy((pointr4+24), &ip_chks, 2);
	rte_memcpy((pointr4+34), &l4_data, 26);
	if ((pkt_size-64)>0){
		memset((pointr4+60), 0xFF, (pkt_size-64));
	}

	size2=htons(pkt_size-(14+20+4)); // UDP size
	sizechks_v4=rte_be_to_cpu_16(size2);
	rte_memcpy(pointr4+38, &size2, 2); // update the packet size in the UDP header
	magic3=pointr4+42;
	magic1= (uint16_t*)pointr4+48;		// pointer to the second counter in the UDP packet
	magic5=pointr4+46;
	time_stmp1= (uint64_t*)pointr4+50;
	rte_memcpy(&ipv4_hdr,(pointr4+14),20); // prepare for checksum calculation
		
	
	/*  v4 side	bgd v6 */
	/* copy packet to the mbuf*/
	rte_memcpy(pointr6, &l2_v6, 14);
	rte_memcpy(pointr6, &DUT_MAC_R, 6);
	rte_memcpy(pointr6+6, &Tester_MAC_R, 6);
	rte_memcpy((pointr6+14), &l3_v6, 40);
	rte_memcpy((pointr6+22), &bgd_1_IPv6, 16);
	rte_memcpy((pointr6+38), &fgd_1_IPv6, 16);
	rte_memcpy((pointr6+54), &l4_data, 26);
	if ((pkt_sizev6-84)>0){
		memset((pointr6+80), 0xFF, (pkt_sizev6-84));
	}
	size1=htons(pkt_sizev6-(14+40+4));//ip header
	size2=htons(pkt_sizev6-(14+40+4));//udp header
	sizechks=rte_be_to_cpu_16(size1);
	rte_memcpy(pointr6+18, &size1, 2); 
	rte_memcpy(pointr6+58, &size2, 2); 
	magic4=pointr6+62;
	magic2= (uint16_t*)pointr6+68;
	magic6=pointr6+66;
	time_stmp2= (uint64_t*)pointr6+70;
	
	rte_memcpy(&ipv6_hdr,(pointr6+14),40); 
	//rte_memcpy(&l3_v6_w,(pointr6+14),40);
	
	rte_delay_ms(100); 	// wait for sender2
	ctrl1 = 1;		// synchronize with sender2
	
	
	for (count = 0; count < timer_period; count++){ //Main cycle for the seconds

		if (flr1 == 1) break;
		if (force_quit) break;

		*magic3 = 0x48;
		*magic4 = 0x48;
		
	 
		countd = 0;
		strt = rte_rdtsc(); 	// Starting TSC of the test
		
		for (countb = 0; countb < pktps; countb++){ //repeats in 1 sec
			
			if (force_quit) break;
			if (flr1 == 1) break;
			if (countc == 2) {
				*magic5=l_reset; 
				*magic6=l_reset;
			}		
						
			if (countc >= lat_pkt) {
				countc=1; 
				
				*magic5=l_id; 
				*magic6=l_id;
			
				*magic1=count2; 
				*magic2=count2;
				st_tsc = rte_rdtsc();
				*time_stmp1=st_tsc;
				*time_stmp2=st_tsc;
				count2++;
				
			}
			if (lat_rate>0) countc++;
		
            if ( countd % mod_n  < mod_m ) {
				/* send foreground packets*/
				
				rte_memcpy((pointr4+40), &reset, 2); // reset UDP checksum
				// calculate UDP checksum
				cksum = rte_raw_cksum((pointr4+34), sizechks_v4); 
				cksum += rte_ipv4_phdr_cksum(&ipv4_hdr, 0);
				cksum = ((cksum & 0xffff0000) >> 16) + (cksum & 0xffff);
				cksum = (~cksum) & 0xffff;
				cksum = (uint16_t) cksum;
				if (cksum == 0)  cksum = 0xffff;

				rte_memcpy((pointr4+40), &cksum, 2);
				sent=0;
				sent = rte_eth_tx_burst(eth_id, 0, &pkts, 1);
				while (sent == 0 && repeat == 1){
					sent = rte_eth_tx_burst(eth_id, 0, &pkts, 1);
				}	
				if (sent==0) sending_ring_full1++;
				else sent1++;	
				countd++;
				cur_tsc = rte_rdtsc();

			}
			else {
				/* send background packets*/				
				rte_memcpy((pointr6+60), &reset, 2);
				cksum2 = rte_raw_cksum((pointr6+54), sizechks);
				cksum2 += rte_ipv6_phdr_cksum(&ipv6_hdr, 0);
				cksum2 = ((cksum2 & 0xffff0000) >> 16) + (cksum2 & 0xffff);
				cksum2 = (~cksum2) & 0xffff;
				cksum2 = (uint16_t) cksum2;
				if (cksum2 == 0)  cksum2 = 0xffff;
				
				rte_memcpy((pointr6+60), &cksum2, 2);	
				
				sent=0;
				sent = rte_eth_tx_burst(eth_id, 0, &pkts_back, 1);
				while (sent == 0 && repeat == 1){
					sent = rte_eth_tx_burst(eth_id, 0, &pkts_back, 1);
				}
				if (sent==0) sending_ring_full1++;
				else sent1++;
				countd++;
				cur_tsc = rte_rdtsc();
				
			}

			/* calculate gap between packets*/
			if (countd < pktps ){
				
				while ((strt+(p_send_cyc*countd))>cur_tsc){
				
					cur_tsc = rte_rdtsc();
				}
				
			}
			
			cur_tsc = rte_rdtsc();
			
		}
		/* stop program if it can't perform the required pkt/s*/
		if ( (cur_tsc-strt) > ((hz/100)*101)){  // more than 5% inaccuracy
			
			printf("The system can't send a packet in this period of time!  \n");
			flr1=1;
			break;
		}
		/* wait until the end of sec*/
		while (((cur_tsc)-(strt)) <hz){
			
			cur_tsc = rte_rdtsc();
		}

		
	}
	printf("Stop sending 1  \n");
	return 0;
	}

/* Sending packets 2  */

int
sender2(void *dummy) 
{
	uint8_t eth_id = 0;
	uint8_t *pointr6f, *pointr6b; /* pointer to the start of the data in the mbuf */
	uint8_t sent; /* work variables  */
	struct rte_mbuf *pkts, *pkts_back; /* mbuf */
	uint16_t size1=0x0000, size2=0x0000,sizechks; /* L2 and L3 sizes */
	uint32_t cksum,cksum2; /* L4 checksum */
	uint16_t reset = 0x0000; /* reset checksum */
	uint16_t *magic1,*magic2; /* pointers to the second couter in the packet (in which second the packet was sent) */
	uint8_t *magic5,*magic6; 
	uint32_t count = 0, countb=0, countc = 0, countd; /* work variables  */
	uint8_t l_id = 0xAA; /*latency packet id */
	uint8_t l_reset = 0x00; /*latency packet id reset */
	uint16_t count2 = 0x0000; /* latency packet counter */
	uint8_t *magic3; /*direction ID */
	uint8_t *magic4; /*direction ID */
	uint64_t strt; /* starting TSC register value at the beginning of the test */
	uint64_t cur_tsc = 0; /* current TSC register values */
	uint64_t st_tsc; /* TSC register values */
	uint64_t *time_stmp1,*time_stmp2;
	struct ipv6_hdr ipv6_hdr1, ipv6_hdr2;
	
	/*Allocate a new mbufs from a mempool.*/   
	pkts = rte_pktmbuf_alloc(pktmbuf_pool); // foreground traffic
	pkts_back = rte_pktmbuf_alloc(pktmbuf_pool); // backgorund traffic
	/* update packet template with pckt size: substract the size of the CRC */
	pkts->data_len = (pkt_sizev6 -4);
	pkts->pkt_len = (pkt_sizev6 -4);
	pkts_back->data_len = (pkt_sizev6-4);
	pkts_back->pkt_len = (pkt_sizev6-4);
	/*pointer to the start of the data in the mbuf*/
	pointr6f = rte_pktmbuf_mtod(pkts, uint8_t *);
	pointr6b = rte_pktmbuf_mtod(pkts_back, uint8_t *);
	
	printf("Sender 2 OK on %d Lcore \n",rte_lcore_id());
	
	/*  v6 side fgd */
	/* copy packet to the mbuf*/
	rte_memcpy(pointr6f, &l2_v6, 14);
	rte_memcpy(pointr6f, &DUT_MAC_L, 6);
	rte_memcpy(pointr6f+6, &Tester_MAC_L, 6);
	rte_memcpy((pointr6f+14), &l3_v6, 40);
	rte_memcpy((pointr6f+22), &fgd_1_IPv6, 16);
	rte_memcpy((pointr6f+38), &fgd_2_IPv6, 16);		
	rte_memcpy((pointr6f+54), &l4_data, 26);
	
	if ((pkt_sizev6 -84)>0){
		memset((pointr6f+80), 0xFF, (pkt_sizev6-84));
	}	
	size1=htons(pkt_sizev6-(14+40+4));//ip size
	size2=htons(pkt_sizev6-(14+40+4));//udp size
	sizechks=rte_be_to_cpu_16(size1);
	rte_memcpy(pointr6f+18, &size1, 2); // update the size in the IP header
	rte_memcpy(pointr6f+58, &size2, 2); // update the size in the UDP header
	magic1=(uint16_t*)pointr6f+68; 
	magic5=pointr6f+66; 
	magic3=pointr6f+62;
	time_stmp1=(uint64_t*)pointr6f+70;
	
	rte_memcpy(&ipv6_hdr1,(pointr6f+14),40);
	
	
	/*  v6 side	bgd  */
	/* copy packet to the mbuf*/
	rte_memcpy(pointr6b, &l2_v6, 14);
	rte_memcpy(pointr6b, &DUT_MAC_L, 6);
	rte_memcpy(pointr6b+6, &Tester_MAC_L, 6);
	rte_memcpy((pointr6b+14), &l3_v6, 40);
	rte_memcpy((pointr6b+22), &fgd_1_IPv6, 16);
	rte_memcpy((pointr6b+38), &bgd_1_IPv6, 16);		
	rte_memcpy((pointr6b+54), &l4_data, 26);
	if ((pkt_sizev6-84)>0){
		memset((pointr6b+80), 0xFF, (pkt_sizev6-84));
	}
	size1=htons(pkt_sizev6-(14+40+4));//ip size
	size2=htons(pkt_sizev6-(14+40+4));//udp size
	rte_memcpy(pointr6b+18, &size1, 2); // update the size in the IP header
	rte_memcpy(pointr6b+58, &size2, 2); // update the size in the UDP header
	magic2=(uint16_t*)pointr6b+68;	
	magic6=pointr6b+66;
	magic4=pointr6b+62;
	time_stmp2=(uint64_t*)pointr6b+70;
	rte_memcpy(&ipv6_hdr2,(pointr6b+14),40);
	
	
	while (ctrl1 == 0){		//synchronize with sender1
		rte_delay_ms(1);
		
	}
	

	for (count = 0; count < timer_period; count++){ //Main cycle for the seconds
		if (flr2 == 1) break;
		if (force_quit) break;
		countd = 0;
		*magic3 = 0x49;
		
		*magic4 = 0x49;
		
		 
		strt = rte_rdtsc(); // Starting TSC of the test
		


		for (countb = 0; countb < pktps; countb++){ //repeats in 1 sec
			
			if (force_quit) break;
			if (flr2 == 1) break;
			
			if (countc == 2) {
				*magic5=l_reset; 
				*magic6=l_reset;
			}
			
			if (countc >= lat_pkt) {
				countc=1; 
				
				*magic5=l_id; 
				*magic6=l_id;
			
				*magic1=count2; 
				*magic2=count2;
				st_tsc = rte_rdtsc();
				*time_stmp1=st_tsc;
				*time_stmp2=st_tsc;
				count2++;
			}
			if (lat_rate>0) countc++;
			

			if ( countd % mod_n < mod_m ) {
				/* send foreground packets*/

				rte_memcpy((pointr6f+60), &reset, 2);
				cksum = 0;
				cksum = rte_raw_cksum((pointr6f+54), sizechks);
			
				cksum += rte_ipv6_phdr_cksum(&ipv6_hdr1, 0);
			
				cksum = ((cksum & 0xffff0000) >> 16) + (cksum & 0xffff);
				cksum = (~cksum) & 0xffff;
				if (cksum == 0)  cksum = 0xffff;

				rte_memcpy((pointr6f+60), &cksum, 2);
				sent=0;
				sent = rte_eth_tx_burst(eth_id, 0, &pkts, 1);
				while (sent == 0 && repeat == 1){
					sent = rte_eth_tx_burst(eth_id, 0, &pkts, 1);
				}	
				if (sent==0) sending_ring_full2++;
				else sent2++;
				countd++;
				cur_tsc = rte_rdtsc();
				
			}
			else {
			
				/* send background packets*/				
				rte_memcpy((pointr6b+60), &reset, 2);
				cksum2 = 0;
				cksum2 = rte_raw_cksum((pointr6b+54), sizechks);
				cksum2 += rte_ipv6_phdr_cksum(&ipv6_hdr2, 0);
				cksum2 = ((cksum2 & 0xffff0000) >> 16) + (cksum2 & 0xffff);
				cksum2 = (~cksum2) & 0xffff;
				if (cksum2 == 0)  cksum2 = 0xffff;
				rte_memcpy((pointr6b+60), &cksum2, 2);
				sent=0;
				sent = rte_eth_tx_burst(eth_id, 0, &pkts_back, 1);
				while (sent == 0 && repeat == 1){
					sent = rte_eth_tx_burst(eth_id, 0, &pkts_back, 1);
				}
				if (sent==0) sending_ring_full2++;
				else sent2++;
				countd++;
				cur_tsc = rte_rdtsc();
							
			}

			/* calculate gap between packets*/
			if (countd < pktps ){
				while ((strt+(p_send_cyc*countd))>cur_tsc){
						
					cur_tsc = rte_rdtsc();
				} 
			}
		
			cur_tsc = rte_rdtsc();
			
		}
	
		/* stop program if it can't perform the required pkt/s*/
		if ( (cur_tsc-strt) > ((hz/100)*101)){  // more than 5% inaccuracy
			printf("The system can't send a packet in this period of time!!  \n");
			flr2=1;
			break;
		}
		/* wait until the end of sec*/
		while (((cur_tsc)-(strt)) <hz){
			cur_tsc = rte_rdtsc();
		}

		
	}

	printf("Stop sending 2  \n");
	return 0;
}





/* Calculate median */
float median(uint32_t n, uint32_t x[]) {
    uint32_t temp;
    uint32_t i, j;
   
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                 /* swap elements */
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(n%2==0) {
        
        return(((x[n/2]/(float)hz)*1000 + (x[n/2 - 1]/(float)hz)*1000) / 2.0);
    } else {
        
        return (x[n/2]/(float)hz)*1000;
    }
}


/* Calculate percentile */
float percentile(uint32_t n, uint32_t x[]) {
    uint32_t temp;
    uint32_t i, j;
	int vegp;
	float per=0.999;
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                /* swap elements */
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

	vegp=(int)(per*n);
    return((x[vegp]/(float)hz)*1000.0);
  
}

/* Processing config file*/
int conf_rd(void) {
    FILE* file = fopen("nat64.conf", "r");
    char line[100];
	char v6_add[40], v4_add[20],macstr[17] = "AA:00:00:00:00:01", cpustr[3];
	int k, c, s, domain, c_list = 0;
	
    while (fgets(line, 65, file)) {
		
		/* IPv6-Real  */
        if(line[3] == '6' && line[5] == 'R'  && line[9] != '\0' && line[10] != '\0')
        {
			
			k = 10;
			c = 0;
			domain = AF_INET6;
			while (line[k]!='\0' && line[k]!=';' && k<50){
					
				v6_add[c]=line[k];
				k++;
				c++;
			}
			v6_add[c] = '\0';
			
			s = inet_pton(AF_INET6, v6_add, fgd_1_IPv6);
			if (s == 0){
				printf("Wrong format in config file! IPv6-Real \n");
				flr=1;
				break;
			}
			memset(v6_add, '\0', sizeof(v6_add));
        }
		
		/* IPv6-Virtual */
        if(line[3] == '6' && line[5] == 'V'  && line[12] != '\0' && line[13] != '\0')
        {
			
			k = 13;
			c = 0;
			domain = AF_INET6;
			while (line[k]!='\0' && line[k]!=';' && k<50){
					
				v6_add[c]=line[k];
					
				k++;
				c++;
			}
			v6_add[c]='\0';
			s = inet_pton(domain, v6_add, fgd_2_IPv6);
			if (s == 0){
				printf("Wrong format in config file! IPv6-Virtual \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(v6_add, '\0', sizeof(v6_add));
        }
	
		/* IPv4-Real */
        if(line[3] == '4' && line[5] == 'R'  && line[9] != '\0' && line[10] != '\0')
        {
			
			k = 10;
			c = 0;
			domain = AF_INET;
			while (line[k]!='\0' && line[k]!=';' && k<30){
					
				v4_add[c]=line[k];
					
				k++;
				c++;
			}
			v4_add[c]='\0';
			s = inet_pton(domain, v4_add, fgd_1_IPv4);
			
			if (s == 0){
				printf("Wrong format in config file! IPv4-Real \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(v4_add, '\0', sizeof(v4_add));
        }	
		
		/* IPv4-Virtual */
        if(line[3] == '4' && line[5] == 'V'  && line[12] != '\0' && line[13] != '\0')
        {
			
			k = 13;
			c = 0;
			domain = AF_INET;
			while (line[k]!='\0' && line[k]!=';' && k<30){
					
				v4_add[c]=line[k];
					
				k++;
				c++;
			}
			v4_add[c]='\0';
			s = inet_pton(domain, v4_add, fgd_2_IPv4);
			if (s == 0){
				printf("Wrong format in config file! IPv4-Virtual  \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(v4_add, '\0', sizeof(v4_add));
        }	
		
		/* Background IPv6 */
        if(line[3] == '6' && line[5] == 'B'  && line[15] != '\0' && line[16] != '\0')
        {
			
			k = 16;
			c = 0;
			domain = AF_INET6;
			while (line[k]!='\0' && line[k]!=';' && k<60){
					
				v6_add[c]=line[k];
					
				k++;
				c++;
			}
			v6_add[c]='\0';
			s = inet_pton(domain, v6_add, bgd_1_IPv6);
			if (s == 0){
				printf("Wrong format in config file! IPv6 Background \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(v6_add, '\0', sizeof(v6_add));
        }
		
		/* Tester MAC Left  */
        if(line[0] == 'T' && line[7] == 'M' && line[11] == 'L' && line[12] != '\0' && line[13] != '\0')
        {
			
			k = 13;
			c = 0;
			
			while (line[k]!='\0' && line[k]!=';' && k<33){
					
				macstr[c]=line[k];
					
				k++;
				c++;
			}
			macstr[c]='\0';
			if( 6 != sscanf( macstr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&Tester_MAC_L[0], &Tester_MAC_L[1], &Tester_MAC_L[2], &Tester_MAC_L[3], &Tester_MAC_L[4], &Tester_MAC_L[5] ) ){
				printf("Wrong format in config file!  Tester MAC Left \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(macstr, '\0', sizeof(macstr));
        }
		/* DUT MAC Left  */
        if(line[0] == 'D' && line[4] == 'M' && line[8] == 'L' && line[9] != '\0' && line[10] != '\0')
        {
			

			k = 10;
			c = 0;
			
			while (line[k]!='\0' && line[k]!=';' && k<33){
					
				macstr[c]=line[k];
					
				k++;
				c++;
			}
			if( 6 != sscanf( macstr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&DUT_MAC_L[0], &DUT_MAC_L[1], &DUT_MAC_L[2], &DUT_MAC_L[3], &DUT_MAC_L[4], &DUT_MAC_L[5] ) ){
				printf("Wrong format in config file! DUT MAC Left \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}

			/*
			memset(macstr, '\0', 17);*/
        }	
		
		/* Tester MAC Right  */
        if(line[0] == 'T' && line[7] == 'M' && line[11] == 'R' && line[12] != '\0' && line[13] != '\0')
        {
			
			k = 13;
			c = 0;
			
			while (line[k]!='\0' && line[k]!=';' && k<33){
					
				macstr[c]=line[k];
					
				k++;
				c++;
			}
			macstr[c]='\0';
			if( 6 != sscanf( macstr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&Tester_MAC_R[0], &Tester_MAC_R[1], &Tester_MAC_R[2], &Tester_MAC_R[3], &Tester_MAC_R[4], &Tester_MAC_R[5] ) ){
				printf("Wrong format in config file! Tester MAC Right \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(macstr, '\0', sizeof(macstr));
        }
		/* DUT MAC Right  */
        if(line[0] == 'D' && line[4] == 'M' && line[8] == 'R' && line[9] != '\0' && line[10] != '\0')
        {
			
			k = 10;
			c = 0;
			
			while (line[k]!='\0' && line[k]!=';' && k<33){
					
				macstr[c]=line[k];
					
				k++;
				c++;
			}
			macstr[c]='\0';
			if( 6 != sscanf( macstr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&DUT_MAC_R[0], &DUT_MAC_R[1], &DUT_MAC_R[2], &DUT_MAC_R[3], &DUT_MAC_R[4], &DUT_MAC_R[5] ) ){
				printf("Wrong format in config file! DUT MAC Right \n");
				nat64_tester_bp_conf_usage;
				flr=1;
				break;
			}
			memset(macstr, '\0', sizeof(macstr));
        }	
		
		/* Right side sender CPU  */
        if(line[0] == 'C' && line[4] == 'R' && line[10] == 'S' && line[14] != '\0' && line[15] != '\0')
        {
			
			k = 15;
			c = 0;
			
			while (k<17 && line[k]!='\0' && line[k]!=';'){
					
				cpustr[c]=line[k];
				cpu_list[c_list]=line[k];
				c_list++;
				cpu_list[c_list]=',';
				c_list++;
				k++;
				c++;
			}
			
			
			v4s_snd_CPU = strtol(cpustr, NULL, 10);
			
			
        }
		
		/* Right side reciveing CPU  */
        if(line[0] == 'C' && line[4] == 'R' && line[10] == 'R' && line[17] != '\0' && line[18] != '\0')
        {
			
			k = 18;
			c = 0;
			
			while (k<20 && line[k]!='\0' && line[k]!=';'){
					
				cpustr[c]=line[k];
				cpu_list[c_list]=line[k];
				c_list++;
				cpu_list[c_list]=',';
				c_list++;	
				k++;
				c++;
			}
			
			
			v4s_rcv_CPU = strtol(cpustr, NULL, 10);
			
			
			
        }
		
		/* Left side sender CPU  */
        if(line[0] == 'C' && line[4] == 'L' && line[9] == 'S' && line[13] != '\0' && line[14] != '\0')
        {
			
			k = 14;
			c = 0;
			
			while (k<16 && line[k]!='\0' && line[k]!=';'){
					
				cpustr[c]=line[k];
				cpu_list[c_list]=line[k];
				c_list++;
				cpu_list[c_list]=',';
				c_list++;
				k++;
				c++;
			}
			
			
			v6s_snd_CPU = strtol(cpustr, NULL, 10);
			
			
        }
		
		/* Left side reciveing CPU  */
        if(line[0] == 'C' && line[4] == 'L' && line[9] == 'R' && line[16] != '\0' && line[17] != '\0')
        {
			
			k = 17;
			c = 0;
			
			while (k<19 && line[k]!='\0' && line[k]!=';'){
					
				cpustr[c]=line[k];
				cpu_list[c_list]=line[k];
				c_list++;
				cpu_list[c_list]=',';
				c_list++;	
				k++;
				c++;
			}
			
			
			v6s_rcv_CPU = strtol(cpustr, NULL, 10);
			
			
        }
		
		
    }
	cpu_list[c_list-1]='\0';
    fclose(file);
 
    return 0;
}


int
main(int argc, char **argv)
{

	int chk = 0; /* work variables  */
	uint8_t portid; /* work variables  */
	uint32_t wrk; /* work variables  */
	#define CHECK_INTERVAL 100 /* 100ms */
	#define MAX_CHECK_TIME 90  /* repeat */
	uint8_t port_chk[2] = {0,0}; /* port status  */
	
	/* Processing config file*/
	conf_rd();
	if (flr != 1) {
	/* parse  arguments  */
	chk = nat64_tester_bp_parse_args(argc, argv);
	if (chk < 0)
		rte_exit(EXIT_FAILURE, "Invalid nat64_tester_bp arguments\n");
	
	/* init EAL */
	argv[1] = "-l";
	argv[2] = cpu_list;
	chk = 3;
	chk = rte_eal_init(chk, argv);
	if (chk < 0)
		rte_exit(EXIT_FAILURE, "EAL init failed \n");

	force_quit = false;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* create the mbuf pool */
	pktmbuf_pool = rte_pktmbuf_pool_create("mbuf_pool", NB_MBUF,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE,
		rte_socket_id());
	if (pktmbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");

	
	if (rte_eth_dev_count_avail() != 2)
		rte_exit(EXIT_FAILURE, " 2 Ethernet ports should be available - bye\n");



	/* Initialise each port */
	printf("Start port initialisation \n");
	for (portid = 0; portid < 2; portid++) {
	
		/* init port */
		
		chk = rte_eth_dev_configure(portid, 1, 1, &port_conf);
		if (chk < 0)
			rte_exit(EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",chk, (unsigned) portid);

		/* init RX queue */
		chk = rte_eth_rx_queue_setup(portid, 0, nb_rxd,
					     rte_eth_dev_socket_id(portid),
					     NULL,
					     pktmbuf_pool);
		if (chk < 0)
			rte_exit(EXIT_FAILURE, "RX queue init error \n");
		
		
		/* init TX queue */
		chk = rte_eth_tx_queue_setup(portid, 0, nb_txd,
				rte_eth_dev_socket_id(portid),
				NULL);
		if (chk < 0)
			rte_exit(EXIT_FAILURE, "TX queue init error u\n");



		/* Start device */
		chk = rte_eth_dev_start(portid);
		if (chk < 0)
			rte_exit(EXIT_FAILURE, "Device start error\n");
		rte_eth_promiscuous_enable(portid);

}
	
	/* waiting links are up */
	struct rte_eth_link link;
	for (portid = 0; portid < 2; portid++) {
	chk = 0;
	
		while (chk <= MAX_CHECK_TIME){
		rte_eth_link_get_nowait(0, &link);	
			if (link.link_status == ETH_LINK_UP) {printf("Port %d Up - speed %u "
						"Mbps - %s\n", (uint8_t)portid,
						(unsigned)link.link_speed,
				(link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
					("full-duplex") : ("half-duplex\n"));
			chk = MAX_CHECK_TIME+1;
			port_chk[portid] = 1;
			

			}
			chk++;
			rte_delay_ms(CHECK_INTERVAL);
			
		}
	}
	if (port_chk[0] != 1 || port_chk[1] != 1) rte_exit(EXIT_FAILURE, "Port error\n");
	
	
	printf("Port initialisation finished \n");


	
 
	 


	/* get the number of cycles/s */
	hz = (rte_get_timer_hz()); 

	/* calculate the number of cycles between packets */
	p_send_cyc=hz/pktps; 





	/* skip the first sec from measurment */
	timer_period = timer_period + warm_up; 

	/* latency scheduler */
	if (lat_rate>0) lat_pkt = (unsigned long) pktps/lat_rate;
	
	
	
	chk = 0;
	
	
	if ((v4s_snd_CPU == 99)) ctrl1 = 1;		// turn off synchronization with sender2
	
	if ((v4s_snd_CPU != 99)) chk++;
	if ((v4s_rcv_CPU != 99)) chk++;
	if ((v6s_snd_CPU != 99)) chk++;
	if ((v6s_rcv_CPU != 99)) chk++;
	
	if ((rte_lcore_count()<(uint16_t)chk)) rte_exit(EXIT_FAILURE, " The perited cpu range and CPU cnfiguration in confug file should be similar \n");
	if ((v4s_rcv_CPU != 99) && (v4s_rcv_CPU == rte_get_master_lcore())) rte_exit(EXIT_FAILURE, " The reciving CPU can't be on the master core \n");
	if ((v6s_rcv_CPU != 99) && (v6s_rcv_CPU == rte_get_master_lcore())) rte_exit(EXIT_FAILURE, " The reciving CPU can't be on the master core \n");	
	
	if ((v4s_snd_CPU != 99)) { if ((v4s_snd_CPU == v4s_rcv_CPU) || (v4s_snd_CPU == v6s_rcv_CPU) || 	(v4s_snd_CPU == v6s_snd_CPU)){ rte_exit(EXIT_FAILURE, "  CPUs should be different \n"); }}
	if ((v4s_rcv_CPU != 99)) { if ((v4s_snd_CPU == v4s_rcv_CPU) || (v4s_rcv_CPU == v6s_rcv_CPU) || 	(v4s_rcv_CPU == v6s_snd_CPU)){ rte_exit(EXIT_FAILURE, "  CPUs should be different\n"); }}
	if ((v6s_snd_CPU != 99)) { if ((v6s_snd_CPU == v4s_rcv_CPU) || (v6s_snd_CPU == v6s_rcv_CPU) || 	(v4s_snd_CPU == v6s_snd_CPU)){ rte_exit(EXIT_FAILURE, "  CPUs should be different\n"); }}
	if ((v6s_rcv_CPU != 99)) { if ((v6s_rcv_CPU == v4s_rcv_CPU) || (v4s_snd_CPU == v6s_rcv_CPU) || 	(v6s_rcv_CPU == v6s_snd_CPU)){ rte_exit(EXIT_FAILURE, "  CPUs should be different\n"); }}
	

	
	printf("Start testing \n");
		
	/* start on the remote cores */
		
	/* start reciveing */
	if ((v4s_rcv_CPU != 99) && (v4s_rcv_CPU != rte_lcore_id())) rte_eal_remote_launch(receive1, NULL, v4s_rcv_CPU);
	if ((v6s_rcv_CPU != 99) && (v6s_rcv_CPU != rte_lcore_id())) rte_eal_remote_launch(receive2, NULL, v6s_rcv_CPU);
	
	
	/* start sending */
	if ((v6s_snd_CPU != 99) && (v6s_snd_CPU != rte_lcore_id())) rte_eal_remote_launch(sender2, NULL, v6s_snd_CPU);
	if ((v4s_snd_CPU != 99) && (v4s_snd_CPU != rte_lcore_id())) rte_eal_remote_launch(sender1, NULL, v4s_snd_CPU);
		 
		 
	/* start on the main core */
			
	/* start sending */
	
	if (v6s_snd_CPU == rte_lcore_id()) sender2(NULL);
	if (v4s_snd_CPU == rte_lcore_id()) sender1(NULL);
	
	
	// wait for the remote senders	
	
	if ((v4s_snd_CPU != 99) &&(v4s_snd_CPU != rte_lcore_id())) rte_eal_wait_lcore(v4s_snd_CPU);
	if ((v6s_snd_CPU != 99) &&(v6s_snd_CPU != rte_lcore_id())) rte_eal_wait_lcore(v6s_snd_CPU);
		

		printf("Stop sending \n  \n");	
		
	if (!force_quit) rte_delay_ms(time_out);
	
	
	printf("Stop recv \n  \n");	
	recv_ctrl=1;

	// wait for the remote receivers	
		
	if ((v6s_rcv_CPU != 99) &&(v6s_rcv_CPU != rte_lcore_id())) rte_eal_wait_lcore(v6s_rcv_CPU);
	if ((v4s_rcv_CPU != 99) &&(v4s_rcv_CPU != rte_lcore_id())) rte_eal_wait_lcore(v4s_rcv_CPU);

	

		
		
	printf("Test(s) finished. Processing results \n");



	/* calculate and summarize results*/
	float dmin1=999999.0, dmax1=0.0,dmin2=999999.0, dmax2=0.0,szamol; /* work variables  */
	
	
	
	if (!force_quit) {
  	
  	
  		for (wrk = warm_up; wrk < lat_received1; wrk++){
			szamol = (delay1[wrk]/(float)hz)*1000.0;
			
			if (dmax1<szamol) dmax1=szamol;
			if (dmin1>szamol) dmin1=szamol;
			
	
		}

	


		for (wrk = warm_up; wrk < lat_received2; wrk++){
			
			szamol = (delay2[wrk]/(float)hz)*1000.0;
			
			if (dmax2<szamol) dmax2=szamol;
			if (dmin2>szamol) dmin2=szamol;

				

		}

		
		printf("\n");
		printf("\n");

		if ((sent1 < received2) || (sent2 < received1)) rte_exit(EXIT_FAILURE, " The system can't send a packet in this period of time!!! \n");
		
		printf("Forward frames sent: %d droped %d \n", sent2,sending_ring_full2);
		printf("Forward frames received: %d \n", received1);
		printf("Reverse frames sent: %d droped %d \n", sent1,sending_ring_full1);
		printf("Reverse frames received: %d \n", received2);


		//median ,percentile
		if ((lat_received2>0) || (lat_received1>0)) {
			printf("\n");
			printf("Latency results (ms)\n");
		}
		if (lat_received1>0) {
			
			float m;		
			m=median(lat_received1, delay1);
			printf("\nForward TL=%f\n",m);
			printf("Forward Dmin2=%f\n",dmin1);
			printf("Forward Dmax2=%f\n",dmax1);
			printf("Forward WCL=%f\n",percentile(lat_received1, delay1));
			

		}
		if (lat_received2>0) {

			
			float m;
			m=median(lat_received2, delay2);
			printf("\nReverse TL=%f\n",m);			
			printf("Reverse Dmin1=%f\n",dmin2);
			printf("Reverse Dmax1=%f\n",dmax2);
			printf("Reverse WCL=%f\n",percentile(lat_received2, delay2));

			

		}		
		
		printf("\n");printf("\n");


	}
	}

/* close ports */
	for (portid = 0; portid < 2; portid++) {

		printf("Closing port %d...", portid);
		rte_eth_dev_stop(portid);
		rte_eth_dev_close(portid);
		printf(" Done\n");
	}
	printf("Good bye!\n");

	return chk;
}

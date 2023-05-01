/*
   For LG NAS Detector & FW Update (Enclosing)
   anyong	2010/06/03
 */

#include <nas_detector.h>

/* #define	DEBUG_NASD */

#ifdef DEBUG_NASD
#define debug_nasd(fmt,args...)	printf (fmt ,##args)
#else
#define debug_nasd(fmt,args...)
#endif


/* Globals */
int nasd_state = NASD_INIT;
uchar * g_img_ptr = NULL;		/* Pointer to the location to copy the uImage file */

uchar broad_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
#define BROAD_IP	 0xffffffff

#define NASD_REQ_PORT	24988	// from PC
#define NASD_REP_PORT	24987	// to PC

void setenv_ipaddr(char * name, IPaddr_t ip)
{
	char ips[20];

	ip_to_string(ip, ips);
	setenv(name, ips);
}
	
char uimage[64];
char initrd[64];
char firmware[64];
char g_packet[1500];
int g_len=0;

int filter_mac( struct SETUP_MSG * req )
{

	char test_mac[20]={0};
	sprintf(test_mac, "%02X:%02X:%02X:%02X:%02X:%02X",
			NetOurEther[0] & 0xff, NetOurEther[1] & 0xff,
			NetOurEther[2] & 0xff, NetOurEther[3] & 0xff,
			NetOurEther[4] & 0xff, NetOurEther[5] & 0xff);
	
	printf(" opcode:%c nas's mac  %s: ",req->op_code, test_mac);
	printf(" opcode:%c detector send mac %s: " , req->op_code, req->mac );

	if( strncmp( test_mac, req->mac , 17 ) ){
		printf(" different mac add....\n");
		return 0;
	}

	return 1;
}


void do_handle (struct SETUP_MSG * req, unsigned len, int src_ip)
{
	int i;
	volatile uchar *pkt;
	struct SETUP_MSG *reply;
	IPaddr_t new_ip, netmask;
	unsigned short src_port;
	char *ptr;
	char test_mac[20]={0};
	
	pkt = NetTxPacket;
	
	reply = pkt + NetEthHdrSize() + IP_HDR_SIZE;
			
	strcpy(reply->srv_name, "LG-NAS - Enclosure");
	netmask = getenv_IPaddr("netmask");


	switch ( req->op_code ) 
	{
		case 'r': 	// Info Request(SCAN)
			printf("<Request Info [%c]>\n", req->op_code);

			reply->op_code = 'r';

			ip_to_string(NetOurIP, reply->srv_ip);
			ip_to_string(netmask, reply->netmask);

			break;

		case 'd':	// DHCP Setup
			if( !filter_mac( req ) ){
				return;
			}
			printf("<DHCP Setup [%c]>\n", req->op_code);

			reply->op_code = 'o';

			nasd_state = NASD_DHCP;
			NetState = NETLOOP_SUCCESS;
			break;

		case 'w':	// Static IP Setup
			if( !filter_mac( req ) ){
				return;
			}
			printf("<Static IP Setup [%c]>\n", req->op_code);

			reply->op_code = 'g'; 

			new_ip = string_to_ip(req->srv_ip);
			printf("New IP :"); print_IPaddr(new_ip); printf("\n");

			setenv("ipaddr", req->srv_ip);
			setenv("netmask", req->netmask);
			NetOurIP = getenv_IPaddr("ipaddr");

			break;

		case 'U':

			if( !filter_mac( req ) ){
				return;
			}
		
			printf("<FW Update Start [%c]>\n", req->op_code);

			reply->op_code = 'r';

			ip_to_string(NetOurIP, reply->srv_ip);
			ip_to_string(netmask, reply->netmask);
			printf("srv_desc: %s\n", req->srv_desc);
			
			ptr = &req->srv_desc;
			strncpy(uimage, strsep(&ptr, " "), sizeof(uimage));
			if( !ptr ) {
				printf("Invalid image name\n");
				break;
			}
			strncpy(initrd, strsep(&ptr, " "), sizeof(initrd));
			if( !ptr ) {
				printf("Invalid initrd name\n");
				break;
			}
			strncpy(firmware, ptr, sizeof(firmware));

			printf("uimage: %s\n", uimage);
			printf("initrd: %s\n", initrd);
			printf("firmware: %s\n", firmware);

			if( strlen(uimage) > 0 && strlen(initrd) > 0 && strlen(firmware) > 0 ) {
				g_len = len;
				memcpy(g_packet, pkt, len);
				setenv_ipaddr("serverip", src_ip);
				setenv("firmware", firmware);

				nasd_state = NASD_DO_TFTP;
				NetState = NETLOOP_SUCCESS;
			}
			else {
				printf("Invalid FW Update packet!\n");
			}

			break;

		default:
			printf("No OP [%c]\n", req->op_code);
			return;
	}

	ip_to_string(getenv_IPaddr("netmask"), reply->netmask); 

	src_port = 1024 + (get_timer(0) % 3072);
    
	sprintf(reply->mac, "%02X:%02X:%02X:%02X:%02X:%02X",
			NetOurEther[0] & 0xff, NetOurEther[1] & 0xff,
			NetOurEther[2] & 0xff, NetOurEther[3] & 0xff,
			NetOurEther[4] & 0xff, NetOurEther[5] & 0xff);

	NetSendUDPPacket(broad_mac, BROAD_IP, NASD_REP_PORT, src_port, sizeof(struct SETUP_MSG));
}


static void
nasd_handler(uchar * pkt, unsigned dest, unsigned src, unsigned len)
{
	IPaddr_t src_ip;
	IPaddr_t dest_ip;
	volatile IP_t *ip = (volatile IP_t *)(pkt - IP_HDR_SIZE) ;
		    
	src_ip = (NetReadIP((void *)&ip->ip_src));
	dest_ip = (NetReadIP((void *)&ip->ip_dst));

#if 1
	printf("\n[Packet len=%d] ", len); 
	printf("IP ["); print_IPaddr((src_ip));
	printf(" -> "); print_IPaddr((dest_ip));
	printf("] (%d->%d)\n", src, dest);
#endif

	if( htonl(dest_ip) != BROAD_IP || ip->udp_dst != htons(NASD_REQ_PORT) ) {
		return;
	}

	do_handle( (struct SETUP_MSG*) pkt, len, src_ip );
}

static void
nasd_timeout(void)
{
	switch (nasd_state)
	{
		case NASD_WAIT_SCAN:
			NetSetTimeout (3600 * CFG_HZ, nasd_timeout);
			break;

		case NASD_FINISH:
			debug_nasd("Finished successfully.\n");
			NetSetTimeout(0, (thand_f *)0);
			NetState = NETLOOP_SUCCESS;
			break;

		default:
			debug_nasd("Invalid state received in the Timeout routine~\n");
	}
}


void nasd_start(void)
{
	NetOurIP = getenv_IPaddr("ipaddr");

	NetServerIP = BROAD_IP;

    NetSetHandler(nasd_handler);

    nasd_state = NASD_WAIT_SCAN;
	NetState = NETLOOP_CONTINUE;

//    NetSetTimeout( 5 * CFG_HZ, nasd_timeout);
}

int nasd_main(void)
{
	int ret;

	udelay (5000000);	// eth link status not reported properly

	nasd_start();

	while ( nasd_state != NASD_FINISH ) 
	{
		if( nasd_state == NASD_DHCP ) {
			ret = NetLoop(DHCP);
			if( ret < 0 ) break;
			if( NetState == NETLOOP_SUCCESS ) {
				nasd_state = NASD_INIT;

				setenv_ipaddr("ipaddr", NetOurIP);
				setenv_ipaddr("netmask", NetOurSubnetMask);
			}
		}
		else if( nasd_state == NASD_DO_TFTP ) {
			
			copy_filename (BootFile, uimage, sizeof(BootFile));
			setenv("loadaddr", "0x1000000");
			ret = NetLoop(TFTP);
			if( ret < 0 ) break;
			if( NetState == NETLOOP_SUCCESS ) {
				copy_filename (BootFile, initrd, sizeof(BootFile));
				setenv("loadaddr", "0x2000000");
				NetState = NETLOOP_CONTINUE;

				ret = NetLoop(TFTP);
				if( ret < 0 ) { 
					unsigned short	src_port = 1024 + (get_timer(0) % 3072);
					struct SETUP_MSG *reply;
	
					reply = g_packet + NetEthHdrSize() + IP_HDR_SIZE;
					reply->op_code = 'U';
					strcpy(reply->srv_desc, "ERR_TFTP_FAIL");

					memcpy((uchar *)NetTxPacket, g_packet, g_len);

					NetSendUDPPacket(broad_mac, BROAD_IP, NASD_REP_PORT, src_port, sizeof(struct SETUP_MSG));
					nasd_state = NASD_WAIT_SCAN;
					continue;
				}
			
				if( NetState == NETLOOP_SUCCESS ) {
					nasd_state = NASD_FINISH;
				}
			}
		}
		else {
			ret = NetLoop(NASDETECT);
			if( ret < 0 ) break;
		}
	}

	if( nasd_state == NASD_FINISH ) {
		run_command("run bootcmd_netenc", 0);
	}

	//setenv("ipaddr", "192.168.10.33");
	//setenv("serverip", "192.168.10.2");

	return ret;
}

int do_nasd (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	return nasd_main();
}

U_BOOT_CMD(
	nasd,	2,	1,	do_nasd,
	"nasd\t- Interface with LG NAS Detector\n",
	"\n"
);




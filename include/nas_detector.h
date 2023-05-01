/*
 *	This file provides the typedefs and constants for the TFTP server and Distress 
 *	Beacn implimentation
 */

#ifndef __NASD_H__
#define __NASD_H__

#include <common.h>
#include <command.h>
#include <net.h>

extern int nasd_state;

/* NAS Detector state-machine */
typedef enum
{
	NASD_INIT,
    NASD_WAIT_SCAN,
	NASD_DHCP,
    NASD_WAIT_TFTP,
    NASD_DO_TFTP,
	NASD_FINISH
}nasd_state_t;


#pragma pack (1)
struct SETUP_MSG
{
	//IP information
	char    srv_ip[20];
	char    cli_ip[20];
	char    netmask[20];
	char    gateway[20];
	char    pri_dns[20];
	char    sec_dns[20];
	//END of IP information

	char    mac[20];

	//NetBIOS information
	char    srv_name[20];
	char    srv_desc[256];
	char    workgroup[20];
	//END of NetBIOS informaiton

	//WINS  information
	char    wins;
	char    wins_ip[20];
	//END of WINS information

	//Etc. information
	char    srv_pass[24];
	char    srv_model_name[24];
	char    srv_serial[24];
	char    op_code;  
	// r:read w:write f:ip/gw mismatch, d:dhcp request, o:dhcp ok, x:dhcp fail, g:ip setting ok, i:ip taken, p: password mismatch
	// Added 2010/02
	char    op_result;
	char  DDNS_UserName[24];        // from PC
	char  DDNS_PassWord[24];        // from PC
	char    srv_DDNS_name[24];
	char    model_name[24];
	char    model_firmver[24];
	char    upnp_enable;
	char    ftp_enable;
	char    ftp_port[8];
	char  ddns_enable;
	//END of Etc. information
};

#pragma pack ()

#endif /* __RCVR_H__ */

/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or 
modify this File in accordance with the terms and conditions of the General 
Public License Version 2, June 1991 (the "GPL License"), a copy of which is 
available along with the File in the license.txt file or by writing to the Free 
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or 
on the worldwide web at http://www.gnu.org/licenses/gpl.txt. 

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED 
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY 
DISCLAIMED.  The GPL License provides additional details about this warranty 
disclaimer.

*******************************************************************************/

#include <config.h>
#include <common.h>
#include <command.h>
#include <pci.h>

#include "mvTypes.h"
#include "mvCtrlEnvLib.h"
#include "uart/mvUart.h"
#include "twsi/mvTwsi.h"

#if defined(MV_INC_BOARD_NOR_FLASH)
#include "norflash/mvFlash.h"
#endif

#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)
#include "eth-phy/mvEthPhy.h"
#endif

#if defined(MV_INCLUDE_PEX)
#include "pex/mvPex.h"
#endif

#if defined(MV_INCLUDE_IDMA)
#include "idma/mvIdma.h"
#include "sys/mvSysIdma.h"
#endif

#if defined(CFG_NAND_BOOT) || defined(CFG_CMD_NAND)
#include <nand.h>

/* references to names in cmd_nand.c */
#define NANDRW_READ		0x01
#define NANDRW_WRITE	0x00
#define NANDRW_JFFS2	0x02
//extern struct nand_chip nand_dev_desc[];
extern nand_info_t nand_info[];       /* info for NAND chips */
/* int nand_rw (struct nand_chip* nand, int cmd,
	    size_t start, size_t len,
	    size_t * retlen, u_char * buf);
 int nand_erase(struct nand_chip* nand, size_t ofs,
				size_t len, int clean);
*/
extern int nand_erase_opts(nand_info_t *meminfo, const nand_erase_options_t *opts);
extern int nand_write_opts(nand_info_t *meminfo, const nand_write_options_t *opts);


#endif /* CFG_NAND_BOOT */

#if (CONFIG_COMMANDS & CFG_CMD_FLASH)
#if !defined(CFG_NAND_BOOT)
static unsigned int flash_in_which_sec(flash_info_t *fl,unsigned int offset)
{
	unsigned int sec_num;
	if(NULL == fl)
		return 0xFFFFFFFF;

	for( sec_num = 0; sec_num < fl->sector_count ; sec_num++){
		/* If last sector*/
		if (sec_num == fl->sector_count -1)
		{
			if((offset >= fl->start[sec_num]) && 
			   (offset <= (fl->size + fl->start[0] - 1)) )
			{
				return sec_num;
			}

		}
		else
		{
			if((offset >= fl->start[sec_num]) && 
			   (offset < fl->start[sec_num + 1]) )
			{
				return sec_num;
			}

		}
	}
	/* return illegal sector Number */
	return 0xFFFFFFFF;

}

#endif /* !defined(CFG_NAND_BOOT) */


/*******************************************************************************
burn a u-boot.bin on the Boot Flash
********************************************************************************/
extern flash_info_t flash_info[];       /* info for FLASH chips */
#include <net.h>
#include "bootstrap_def.h"
#if (CONFIG_COMMANDS & CFG_CMD_NET)
/* 
 * 8 bit checksum 
 */
static u8 checksum8(u32 start, u32 len,u8 csum)
{
    register u8 sum = csum;
	volatile u8* startp = (volatile u8*)start;

    if (len == 0)
	return csum;

    do{
	  	sum += *startp;
		startp++;
    }while(--len);

    return (sum);
}

/*
 * Check the extended header and execute the image
 */
static MV_U32 verify_extheader(ExtBHR_t *extBHR)
{
	MV_U8	chksum;


	/* Caclulate abd check the checksum to valid */
	chksum = checksum8((MV_U32)extBHR , EXT_HEADER_SIZE -1, 0);
	if (chksum != (*(MV_U8*)((MV_U32)extBHR + EXT_HEADER_SIZE - 1)))
	{
		printf("Error! invalid extende header checksum\n");
		return MV_FAIL;
	}
	
    return MV_OK;
}
/*
 * Check the CSUM8 on the main header
 */
static MV_U32 verify_main_header(BHR_t *pBHR, MV_U8 headerID)
{
	MV_U8	chksum;

	/* Verify Checksum */
	chksum = checksum8((MV_U32)pBHR, sizeof(BHR_t) -1, 0);

	if (chksum != pBHR->checkSum)
	{
		printf("Error! invalid image header checksum\n");
		return MV_FAIL;
	}

	/* Verify Header */
	if (pBHR->blockID != headerID)
	{
		printf("Error! invalid image header ID\n");
		return MV_FAIL;
	}
	
	/* Verify Alignment */
	if (pBHR->blockSize & 0x3)
	{
		printf("Error! invalid image header alignment\n");
		return MV_FAIL;
	}

	if ((cpu_to_le32(pBHR->destinationAddr) & 0x3) && (cpu_to_le32(pBHR->destinationAddr) != 0xffffffff))
	{
		printf("Error! invalid image header destination\n");
		return MV_FAIL;
	}

	if ((cpu_to_le32(pBHR->sourceAddr) & 0x3) && (pBHR->blockID != IBR_HDR_SATA_ID))
	{
		printf("Error! invalid image header source\n");
		return MV_FAIL;
	}

    return MV_OK;
}

int nandenvECC = 0;
#if defined(CFG_NAND_BOOT)
/* Boot from NAND flash */
/* Write u-boot image into the nand flash */
int nand_burn_uboot_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int filesize, eccEnv = 0;
	MV_U32 ret = 0;
	extern char console_buffer[];
	nand_info_t *nand = &nand_info[0];
	nand_erase_options_t er_opts;
	nand_write_options_t wr_opts;

	load_addr = CFG_LOAD_ADDR; 
	if(argc == 2) {
		copy_filename (BootFile, argv[1], sizeof(BootFile));
	}
	else { 
		copy_filename (BootFile, "u-boot.bin", sizeof(BootFile));
		printf("using default file \"u-boot.bin\" \n");
	}
 
	if ((filesize = NetLoop(TFTP)) < 0)
		return 0;

#ifdef MV_BOOTROM
    	BHR_t*           	tmpBHR = (BHR_t*)load_addr;
	ExtBHR_t*   		extBHR = (ExtBHR_t*)(load_addr + BHR_HDR_SIZE);
	MV_U32			errCode=0;

	/* Verify Main header checksum */
	errCode = verify_main_header(tmpBHR, IBR_HDR_NAND_ID);
	if (errCode) 
		return 0;

	/* Verify that the extended header is valid */
	errCode = verify_extheader(extBHR);
	if (errCode) 
		return 0;
#endif
    /* The environment has Reed-Solomon ECC protection in the NAND */
    nandenvECC =  1;

	printf("\n**Warning**\n");
	printf("If U-Boot Endiannes is going to change (LE->BE or BE->LE), Then Env parameters should be overriden..\n");
	printf("Override Env parameters? (y/n)");
	readline(" ");
	if( strcmp(console_buffer,"Y") == 0 || 
	    strcmp(console_buffer,"yes") == 0 ||
	    strcmp(console_buffer,"y") == 0 ) {

		printf("Erase Env parameters sector %d... ",CFG_ENV_OFFSET);
		memset(&er_opts, 0, sizeof(er_opts));
		er_opts.offset = CFG_ENV_OFFSET;
		er_opts.length = CFG_ENV_SECT_SIZE;
		er_opts.quiet  = 1;

		nand_erase_opts(nand, &er_opts);
		//nand_erase(nand_dev_desc + 0, CFG_ENV_OFFSET, CFG_ENV_SECT_SIZE, 0);
		printf("\n");
	}

	printf("Erase %d - %d ... ",CFG_MONITOR_BASE, CFG_MONITOR_LEN);
	memset(&er_opts, 0, sizeof(er_opts));
	er_opts.offset = CFG_MONITOR_BASE;
	er_opts.length = CFG_MONITOR_LEN;
	er_opts.quiet  = 1;
	nand_erase_opts(nand, &er_opts);
	//nand_erase(nand_dev_desc + 0, CFG_MONITOR_BASE, CFG_MONITOR_LEN, 0);
	
	printf("\nCopy to Nand Flash... ");
	memset(&wr_opts, 0, sizeof(wr_opts));
	wr_opts.buffer	= (u_char*) load_addr;
	wr_opts.length	= CFG_MONITOR_LEN;
	wr_opts.offset	= CFG_MONITOR_BASE;
	/* opts.forcejffs2 = 1; */
	wr_opts.pad	= 1;
	wr_opts.blockalign = 1;
	wr_opts.quiet      = 1;
	ret = nand_write_opts(nand, &wr_opts);
	/* ret = nand_rw(nand_dev_desc + 0,
				  NANDRW_WRITE | NANDRW_JFFS2, CFG_MONITOR_BASE, CFG_MONITOR_LEN,
			      &total, (u_char*)0x100000 + CFG_MONITOR_IMAGE_OFFSET);
	*/
  	if (ret)
		printf("Error - NAND burn faild!\n");
	else
		printf("\ndone\n");	

    /* The environment has Reed-Solomon ECC protection in the NAND */
    nandenvECC =  0;

    return 1;
}

U_BOOT_CMD(
        bubt,      2,     1,      nand_burn_uboot_cmd,
        "bubt	- Burn an image on the Boot Nand Flash.\n",
        " file-name \n"
        "\tBurn a binary image on the Boot Nand Flash, default file-name is u-boot.bin .\n"
);

/* Write nboot loader image into the nand flash */
int nand_burn_nboot_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int filesize;
	MV_U32 ret = 0;
	extern char console_buffer[];
	nand_info_t *nand = &nand_info[0];
	nand_erase_options_t er_opts;
	nand_write_options_t wr_opts;


	load_addr = CFG_LOAD_ADDR; 
	if(argc == 2) {
		copy_filename (BootFile, argv[1], sizeof(BootFile));
	}
	else { 
		copy_filename (BootFile, "nboot.bin", sizeof(BootFile));
		printf("using default file \"nboot.bin\" \n");
	}
 
	if ((filesize = NetLoop(TFTP)) < 0)
		return 0;
 
	printf("Erase %d - %d ... ",CFG_NBOOT_BASE, CFG_NBOOT_LEN);
	memset(&er_opts, 0, sizeof(er_opts));
	er_opts.offset = CFG_NBOOT_BASE;
	er_opts.length = CFG_NBOOT_LEN;
	er_opts.quiet  = 1;

	nand_erase_opts(nand, &er_opts);
	//nand_erase(nand_dev_desc + 0, CFG_NBOOT_BASE, CFG_NBOOT_LEN , 0);
	
	printf("\nCopy to Nand Flash... ");
	memset(&wr_opts, 0, sizeof(wr_opts));
	wr_opts.buffer	= (u_char*) load_addr;
	wr_opts.length	= CFG_NBOOT_LEN;
	wr_opts.offset	= CFG_NBOOT_BASE;
	/* opts.forcejffs2 = 1; */
	wr_opts.pad	= 1;
	wr_opts.blockalign = 1;
	wr_opts.quiet      = 1;
	ret = nand_write_opts(nand, &wr_opts);
	/* ret = nand_rw(nand_dev_desc + 0,
				  NANDRW_WRITE | NANDRW_JFFS2, CFG_NBOOT_BASE, CFG_NBOOT_LEN,
			      &total, (u_char*)0x100000);
	*/
  	if (ret)
		printf("Error - NAND burn faild!\n");
	else
		printf("\ndone\n");	

	return 1;
}

U_BOOT_CMD(
        nbubt,      2,     1,      nand_burn_nboot_cmd,
        "nbubt	- Burn a boot loader image on the Boot Nand Flash.\n",
        " file-name \n"
        "\tBurn a binary boot loader image on the Boot Nand Flash, default file-name is nboot.bin .\n"
);

#else
/* Boot from Nor flash */
int nor_burn_uboot_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int filesize;
	MV_U32 s_first,s_end,env_sec;
	extern char console_buffer[];


	s_first = flash_in_which_sec(&flash_info[BOOT_FLASH_INDEX], CFG_MONITOR_BASE);
	s_end = flash_in_which_sec(&flash_info[BOOT_FLASH_INDEX], CFG_MONITOR_BASE + CFG_MONITOR_LEN -1);

	env_sec = flash_in_which_sec(&flash_info[BOOT_FLASH_INDEX], CFG_ENV_ADDR);


	load_addr = CFG_LOAD_ADDR; 
	if(argc == 2) {
		copy_filename (BootFile, argv[1], sizeof(BootFile));
	}
	else { 
		copy_filename (BootFile, "u-boot.bin", sizeof(BootFile));
		printf("using default file \"u-boot.bin\" \n");
	}
 
	if ((filesize = NetLoop(TFTP)) < 0)
		return 0;
 
#ifdef MV_BOOTROM
    	BHR_t*           	tmpBHR = (BHR_t*)load_addr;
	ExtBHR_t*   		extBHR = (ExtBHR_t*)(load_addr + BHR_HDR_SIZE);
	MV_U32			errCode=0;

	/* Verify Main header checksum */
	errCode = verify_main_header(tmpBHR, IBR_HDR_SPI_ID);
	if (errCode) 
		return 0;

	/* Verify that the extended header is valid */
	errCode = verify_extheader(extBHR);
	if (errCode) 
		return 0;
#endif

	printf("Un-Protect Flash Monitor space\n");
	flash_protect (FLAG_PROTECT_CLEAR,
		       CFG_MONITOR_BASE,
		       CFG_MONITOR_BASE + CFG_MONITOR_LEN - 1,
		       &flash_info[BOOT_FLASH_INDEX]);

	printf("\n**Warning**\n");
	printf("If U-Boot Endiannes is going to change (LE->BE or BE->LE), Then Env parameters should be overriden..\n");
	printf("Override Env parameters? (y/n)");
	readline(" ");
	if( strcmp(console_buffer,"Y") == 0 || 
	    strcmp(console_buffer,"yes") == 0 ||
	    strcmp(console_buffer,"y") == 0 ) {

		flash_protect (FLAG_PROTECT_CLEAR,
				   flash_info[BOOT_FLASH_INDEX].start[env_sec],
				   flash_info[BOOT_FLASH_INDEX].start[env_sec] + CFG_ENV_SECT_SIZE - 1,
				   &flash_info[BOOT_FLASH_INDEX]);

		printf("Erase Env parameters sector %d... ",env_sec);
		flash_erase(&flash_info[BOOT_FLASH_INDEX], env_sec, env_sec);
	
		if ((mvCtrlModelGet() != MV_6082_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6183_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6183L_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6281_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6192_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6190_DEV_ID) &&
		    (mvCtrlModelGet() != MV_6180_DEV_ID))
		flash_protect (FLAG_PROTECT_SET,
				   flash_info[BOOT_FLASH_INDEX].start[env_sec],
				   flash_info[BOOT_FLASH_INDEX].start[env_sec] + CFG_ENV_SECT_SIZE - 1,
				   &flash_info[BOOT_FLASH_INDEX]);

	}

	printf("Erase %d - %d sectors... ",s_first,s_end);
	flash_erase(&flash_info[BOOT_FLASH_INDEX], s_first, s_end);

	printf("Copy to Flash... ");

	flash_write ( (uchar *)(CFG_LOAD_ADDR + CFG_MONITOR_IMAGE_OFFSET),
				  (ulong)CFG_MONITOR_BASE,
				  (ulong)(filesize - CFG_MONITOR_IMAGE_OFFSET));

	printf("done\nProtect Flash Monitor space\n");
	flash_protect (FLAG_PROTECT_SET,
		       CFG_MONITOR_BASE,
		       CFG_MONITOR_BASE + CFG_MONITOR_LEN - 1,
		       &flash_info[BOOT_FLASH_INDEX]);

	return 1;
}

U_BOOT_CMD(
        bubt,      2,     1,      nor_burn_uboot_cmd,
        "bubt	- Burn an image on the Boot Flash.\n",
        " file-name \n"
        "\tBurn a binary image on the Boot Flash, default file-name is u-boot.bin .\n"
);
#endif /* defined(CFG_NAND_BOOT) */
#endif /* (CONFIG_COMMANDS & CFG_CMD_NET) */



/*******************************************************************************
Reset environment variables.
********************************************************************************/
extern flash_info_t flash_info[];       /* info for FLASH chips */
int resetenv_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(CFG_NAND_BOOT)
	printf("Erase Env parameters offset 0x%x... ",CFG_ENV_OFFSET);
	nand_info_t *nand = &nand_info[0];
	nand_erase_options_t er_opts;
	memset(&er_opts, 0, sizeof(er_opts));
	er_opts.offset = CFG_ENV_OFFSET;
	er_opts.length = CFG_ENV_SECT_SIZE;
	er_opts.quiet  = 1;

	nand_erase_opts(nand, &er_opts);
	//nand_erase(nand_dev_desc + 0, CFG_ENV_OFFSET, CFG_ENV_SECT_SIZE, 0);
	printf("done");
#else
	MV_U32 env_sec = flash_in_which_sec(&flash_info[0], CFG_ENV_ADDR);
	
	if (env_sec == -1)
	{
		printf("Could not find ENV Sector\n");
		return 0;
	}

	printf("Un-Protect ENV Sector\n");

	flash_protect(FLAG_PROTECT_CLEAR,
				  CFG_ENV_ADDR,CFG_ENV_ADDR + CFG_ENV_SECT_SIZE - 1,
				  &flash_info[0]);

	
	printf("Erase sector %d ... ",env_sec);
	flash_erase(&flash_info[0], env_sec, env_sec);
	printf("done\nProtect ENV Sector\n");
	
	flash_protect(FLAG_PROTECT_SET,
				  CFG_ENV_ADDR,CFG_ENV_ADDR + CFG_ENV_SECT_SIZE - 1,
				  &flash_info[0]);

#endif /* defined(CFG_NAND_BOOT) */
	printf("\nWarning: Default Environment Variables will take effect Only after RESET \n\n");
	return 1;
}

U_BOOT_CMD(
        resetenv,      1,     1,      resetenv_cmd,
        "resetenv	- Return all environment variable to default.\n",
        " \n"
        "\t Erase the environemnt variable sector.\n"
);

#endif /* #if (CONFIG_COMMANDS & CFG_CMD_FLASH) */
#if CONFIG_COMMANDS & CFG_CMD_BSP

/******************************************************************************
* Category     - General
* Functionality- The commands allows the user to view the contents of the MV
*                internal registers and modify them.
* Need modifications (Yes/No) - no
*****************************************************************************/
int ir_cmd( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[] )
{
	MV_U32 regNum = 0x0, regVal, regValTmp, res;
	MV_8 regValBin[40];
	MV_8 cmd[40];
	int i,j = 0, flagm = 0;
	extern MV_8 console_buffer[];

	if( argc == 2 ) {
		regNum = simple_strtoul( argv[1], NULL, 16 );
	}
	else { 
		printf( "Usage:\n%s\n", cmdtp->usage );
		return 0;
	}                                                                                                        

	regVal = MV_REG_READ( regNum );
	regValTmp = regVal;
	printf( "Internal register 0x%x value : 0x%x\n ",regNum, regVal );
	printf( "\n    31      24        16         8         0" );
	printf( "\n     |       |         |         |         |\nOLD: " );

	for( i = 31 ; i >= 0 ; i-- ) {
		if( regValTmp > 0 ) {
			res = regValTmp % 2;
			regValTmp = (regValTmp - res) / 2;
			if( res == 0 )
				regValBin[i] = '0';
			else
				regValBin[i] = '1';
		}
		else
			regValBin[i] = '0';
	}

	for( i = 0 ; i < 32 ; i++ ) {
		printf( "%c", regValBin[i] );
		if( (((i+1) % 4) == 0) && (i > 1) && (i < 31) )
			printf( "-" );
	}

	readline( "\nNEW: " );
	strcpy(cmd, console_buffer);
	if( (cmd[0] == '0') && (cmd[1] == 'x') ) {
		regVal = simple_strtoul( cmd, NULL, 16 );
		flagm=1;
	}
	else {
		for( i = 0 ; i < 40 ; i++ ) {
			if(cmd[i] == '\0')
				break;
			if( i == 4 || i == 9 || i == 14 || i == 19 || i == 24 || i == 29 || i == 34 )
				continue;
			if( cmd[i] == '1' ) {
				regVal = regVal | (0x80000000 >> j);
				flagm = 1;
			}
			else if( cmd[i] == '0' ) {
				regVal = regVal & (~(0x80000000 >> j));
				flagm = 1;
			}
			j++;
		}
	}

	if( flagm == 1 ) {
		MV_REG_WRITE( regNum, regVal );
		printf( "\nNew value = 0x%x\n\n", MV_REG_READ(regNum) );
	}
	return 1;
}

U_BOOT_CMD(
	ir,      2,     1,      ir_cmd,
	"ir	- reading and changing MV internal register values.\n",
	" address\n"
	"\tDisplays the contents of the internal register in 2 forms, hex and binary.\n"
	"\tIt's possible to change the value by writing a hex value beginning with \n"
	"\t0x or by writing 0 or 1 in the required place. \n"
    	"\tPressing enter without any value keeps the value unchanged.\n"
);

/******************************************************************************
* Category     - General
* Functionality- Display the auto detect values of the TCLK and SYSCLK.
* Need modifications (Yes/No) - no
*****************************************************************************/
int clk_cmd( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	printf( "TCLK %dMhz, SYSCLK %dMhz (UART baudrate %d)\n",
		mvTclkGet()/1000000, mvSysClkGet()/1000000, CONFIG_BAUDRATE);
	return 1;
}

U_BOOT_CMD(
	dclk,      1,     1,      clk_cmd,
	"dclk	- Display the MV device CLKs.\n",
	" \n"
	"\tDisplay the auto detect values of the TCLK and SYSCLK.\n"
);

/******************************************************************************
* Category     - LgDiag
* Functionality- Check Main Board
* Need modifications (Yes/No) - no
*****************************************************************************/
//#define  LG_DEBUG
#ifdef  LG_DEBUG
#define dprintf(fmt, args...)   printf(fmt ,##args)
#else
#define dprintf(fmt, args...)
#endif

#define mdelay(n)       udelay((n)*1000)
#define SSS_TWSI_ID_RTC                 0x32
#define SSS_TWSI_ID_MICOM               0x58


int i2c_write(uchar dev_addr, unsigned int offset, int alen, uchar* data, int len);
int i2c_read(MV_U8 dev_addr, unsigned int offset, int alen, MV_U8* data, int len);

int LgDiag_IoWrite( u8 C0, u8 C1, u8 C2, u8 C3, u8 C4, u8 C5, u8 C6 )
{
        u8 cData[8];
        int ret;

        memset(cData, 0, 8);
        cData[0] = C0; cData[1] = C1; cData[2] = C2; cData[3] = C3;
        cData[4] = C4; cData[5] = C5; cData[6] = C6;
        cData[7] = C0 ^ C1 ^ C2 ^ C3 ^ C4 ^ C5 ^ C6;
        ret = i2c_write( SSS_TWSI_ID_MICOM, 0, 0, cData, 8 );

        mdelay(100);
        return ret;
}
int LgDiag_IoRead(u8 bId, u8* data)     // jcho ODD-boot
{
        int retry, wretry, iretry, fcode;
        unsigned char msg_buf[256];
        unsigned long dwTmp;

        retry = 10; wretry = iretry = 0;
        while(retry--){
                memset(data,0x00,8);
                if( LgDiag_IoWrite(0x80,bId,0,0,0,0,0) == MV_FAIL )
                {
                        wretry++;
                        mdelay(200);    
                        continue;
                }
                mdelay(200);    
                if(i2c_read( SSS_TWSI_ID_MICOM, 0, 0,data,8) == MV_OK ){
                        if(data[0] != bId) continue;
                        if(data[7] == (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]))
                        {
                                //printf("1:bId:%02x, Retry:%d, Data:%02x%02x%02x%02x %02x%02x%02x%02x\n",
                                //      bId, retry, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                                return MV_TRUE;
                        }
                }
                else
                {
                        fcode = data[0];
                        iretry++;
                        if(getenv("twsi_rd0") == NULL)
                        {
                                memset(msg_buf, 0x00, 256);
                                sprintf(msg_buf, "TC:%02x, TS:%02x, TD:%02x, TA:%02x, TAE:%02x, d[0]:%02x, d[1]:%02x, d[2]:%02x, d[3]:%02x, d[4]:%02x, d[5]:%02x, d[6]:%02x, d[7]:%02x",
                                        (u8) MV_REG_READ(TWSI_CONTROL_REG(0)), (u8) MV_REG_READ(TWSI_STATUS_BAUDE_RATE_REG(0)),
                                        (u8) MV_REG_READ(TWSI_DATA_REG(0)), (u8) MV_REG_READ(TWSI_SLAVE_ADDR_REG(0)),
                                        (u8) MV_REG_READ(TWSI_EXTENDED_SLAVE_ADDR_REG(0)),
                                        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                                setenv("twsi_rd0", msg_buf);
                        }
                        i2c_init(CFG_I2C_SPEED,0);
                        if(getenv("twsi_rd1") == NULL)
                        {
                                memset(msg_buf, 0x00, 256);
                                sprintf(msg_buf, "TC:%02x, TS:%02x, TD:%02x, TA:%02x, TAE:%02x, d[0]:%02x, d[1]:%02x, d[2]:%02x, d[3]:%02x, d[4]:%02x, d[5]:%02x, d[6]:%02x, d[7]:%02x",
                                        (u8) MV_REG_READ(TWSI_CONTROL_REG(0)), (u8) MV_REG_READ(TWSI_STATUS_BAUDE_RATE_REG(0)),
                                        (u8) MV_REG_READ(TWSI_DATA_REG(0)), (u8) MV_REG_READ(TWSI_SLAVE_ADDR_REG(0)),
                                        (u8) MV_REG_READ(TWSI_EXTENDED_SLAVE_ADDR_REG(0)),
                                        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                                setenv("twsi_rd1", msg_buf);
                        }
                        //twsiIntFlgClr();
                mvOsDelay(1);
                        /* clear the int flag bit */
                        dwTmp = MV_REG_READ(TWSI_CONTROL_REG(0));
        MV_REG_WRITE(TWSI_CONTROL_REG(0),dwTmp & ~TWSI_CONTROL_INT_FLAG_SET);

                        /* wait for 1 mili sec for the clear to take effect */
                mvOsDelay(1);

                        if(getenv("twsi_rd2") == NULL)
                        {
                                memset(msg_buf, 0x00, 256);
                                sprintf(msg_buf, "TC:%02x, TS:%02x, TD:%02x, TA:%02x, TAE:%02x, d[0]:%02x, d[1]:%02x, d[2]:%02x, d[3]:%02x, d[4]:%02x, d[5]:%02x, d[6]:%02x, d[7]:%02x",
                                        (u8) MV_REG_READ(TWSI_CONTROL_REG(0)), (u8) MV_REG_READ(TWSI_STATUS_BAUDE_RATE_REG(0)),
                                        (u8) MV_REG_READ(TWSI_DATA_REG(0)), (u8) MV_REG_READ(TWSI_SLAVE_ADDR_REG(0)),
                                        (u8) MV_REG_READ(TWSI_EXTENDED_SLAVE_ADDR_REG(0)),
                                        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                                setenv("twsi_rd2", msg_buf);
                        }

                        mdelay(200);
                }
        }
        //printf("0:bId:%02x, Retry:%d, Data:%02x%02x%02x%02x %02x%02x%02x%02x\n",
        //      bId, retry, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        return MV_FALSE;
}

int LgDiag_Msg2Lcd(char* msg, int bSetMode)    // jcho ODD-boot 
{
        char cTemp[134];
        unsigned char dTemp[8];
        u8 bCheckSum;
        int iLen, i, ret, rcnt;

        if(bSetMode)
        {
                //booted
                //LgDiag_IoWrite(0x86,0,1,0,0,0,0 );
                //shutting down....
                //LgDiag_IoWrite(0x86,0,2,0,0,0,0);
                //halted
                //LgDiag_IoWrite(0x86,0,3,0,0,0,0);
        }

        memset(cTemp, 0x00, 134);
        cTemp[0] = 0xd1;
        strcpy(&cTemp[4],msg);
        iLen = strlen((char*)(cTemp+4));

        cTemp[1] = (u8)(iLen + 2);
        iLen += 4;
        bCheckSum = 0;
        for(i=0; i<iLen; i++){
                bCheckSum ^= cTemp[i];
        }
        cTemp[i] = bCheckSum;
        cTemp[i+1] = bCheckSum;
        iLen += 2;

        rcnt = 0;
retry_d1_msg:
        ret = i2c_write(SSS_TWSI_ID_MICOM, 0, 0, cTemp, iLen);
        if(ret == MV_FAIL)
        {
                printf("Msg2Lcd_0:rcnt:%d, ret:%d, chksum:%02x, iLen:%d\n",
                        rcnt, ret, bCheckSum, iLen);
                return MV_FALSE;
        }
        ret = LgDiag_IoRead(0x7e, (u8*)dTemp);
        if((dTemp[1] == 0xd1) && (dTemp[3] == bCheckSum)) return MV_TRUE;
        if(rcnt++ < 3)
        {
                printf("Msg2Lcd_1:rcnt:%d, ret:%d, Data:%02x%02x%02x%02x %02x%02x%02x%02x\n",
                        rcnt, ret, dTemp[0], dTemp[1], dTemp[2], dTemp[3], dTemp[4], dTemp[5], dTemp[6], dTemp[7]);
                goto retry_d1_msg;
        }

        return MV_FALSE;
}

int LgDiag_Msg2LcdKeep(char* msg, int bSetMode)
{
        char cTemp[134];
        unsigned char dTemp[8];
        u8 bCheckSum;
        int iLen, i, ret, rcnt;

        if(bSetMode)
        {
                //booted
                //LgDiag_IoWrite(0x86,0,1,0,0,0,0 );
                //shutting down....
                //LgDiag_IoWrite(0x86,0,2,0,0,0,0);
                //halted
                //LgDiag_IoWrite(0x86,0,3,0,0,0,0);
        }

        memset(cTemp, 0x00, 134);
        cTemp[0] = 0xd2;
        strcpy(&cTemp[4],msg);
        iLen = strlen((char*)(cTemp+4));

        cTemp[1] = (u8)(iLen + 2);
        iLen += 4;
        bCheckSum = 0;
        for(i=0; i<iLen; i++){
                bCheckSum ^= cTemp[i];
        }
        cTemp[i] = bCheckSum;
        cTemp[i+1] = bCheckSum;
        iLen += 2;

        rcnt = 0;
retry_d2_msg:
        ret = i2c_write(SSS_TWSI_ID_MICOM, 0, 0, cTemp, iLen);
        if(ret == MV_FAIL) return MV_FALSE;
        ret = LgDiag_IoRead(0x7e, (u8*)dTemp);
        if((dTemp[1] == 0xd2) && (dTemp[3] == bCheckSum)) return MV_TRUE;
        if(rcnt++ < 3) goto retry_d2_msg;

        return MV_FALSE;
}
int LgDiag_GetRpm(u16 *wRpm)
{
        u8 bData[8];

        if(LgDiag_IoRead(0x0b, bData) == MV_TRUE){
                *wRpm = ((u16)bData[1] << 8) + (u16)bData[2];
                dprintf("LgDiag_GetRpm : %d \n",((u16)bData[1] << 8) + (u16)bData[2]);
                return MV_TRUE;
        }else{
                *wRpm = 0;
                return MV_FALSE;
        }
}
int LgDiag_Beep(u8 bB1,u8 bB2,u8 bB3,u8 bB4,u8 b10mB, u8 b10mNoB, u8 bCount)
{
        int i;

        for (i=0; i<bCount; i++){
                LgDiag_IoWrite( 0xa1,bB1,bB2,bB3,bB4,b10mB,b10mNoB);
                mdelay(1000);
        }

        return MV_TRUE;
}
int LgDiag_Check_MainIC(void)
{
        char name[15];
        //check chip id and revision number
        //     ("1234567890123456789012345678901");
        dprintf (" Check Main IC ................");
        if(mvCtrlModelRevNameGet(name) == MV_OK){
                dprintf ("Soc(%s).",name);
                if((strcmp (name, MV_5281_D0_NAME) == 0)||
                        (strcmp (name, MV_5281_D2_NAME) == 0)){
                        setenv("LGDIAG_MAIN","OK");
                        dprintf ("[OK]\n");
                        return MV_TRUE;
                }
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_MAIN","NG");
        return MV_FALSE;
}
extern int i2c_probe (uchar chip);
int LgDiag_Check_Twsi( void )
{
        int i;
        uchar lg_i2c[] = CFG_I2C_PROBES;
        //check twsi rtc(0x32), micom(0x58)
        //     ("1234567890123456789012345678901");
        dprintf (" Check Twsi ...................");
        for (i = 0; i < sizeof(lg_i2c); i++){
                dprintf ("(0x%x).",lg_i2c[i]);
                if(i2c_probe(lg_i2c[i])){
                        break;
                }
        }
        if(i == sizeof(lg_i2c)){
                setenv("LGDIAG_TWSI","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_TWSI","NG");
        return MV_FALSE;
}


#define LGDIAG_MAX_MICOM 10
int LgDiag_Check_Micom( void )
{
        char cData[16];
        unsigned char msg_buf[256];
        int iRetry;

        // red led all off
        LgDiag_IoWrite(0x8e,0,0,0,0,0,0 );

        iRetry = 0;
        while( iRetry++ < LGDIAG_MAX_MICOM){

                //     ("1234567890123456789012345678901");
                dprintf (" Check Micom ..................");

                LgDiag_IoRead(0x7f, (u8*)cData);
                cData[7] = (char)NULL;
                dprintf( "%s.",&cData[1]);

                if(strcmp (&cData[1], "LGDS3S") == 0){

                        printf ("MCU Test Count : %03d\r",iRetry);
                        if(iRetry == LGDIAG_MAX_MICOM){
                                setenv("LGDIAG_MICOM","OK");
                                dprintf ("[OK]\n");
                                return MV_TRUE;
                        }else
                                continue;
                }else{
                        memset(msg_buf, 0x00, 256);
                        sprintf(msg_buf, "Error_0, Retry:%d, cData:%02x%02x%02x%02x %02x%02x%02x%02x\n",
                                iRetry, cData[0], cData[1], cData[2], cData[3], cData[4], cData[5], cData[6], cData[7]);
                        setenv("mc_test", msg_buf);
                        if(getenv("twsi_mc") == NULL)
                        {
                                memset(msg_buf, 0x00, 256);
                                sprintf(msg_buf, "TWSI Con:%08x, TWSI Sts:%08x",
                                        MV_REG_READ(TWSI_CONTROL_REG(0)), MV_REG_READ(TWSI_STATUS_BAUDE_RATE_REG(0)));
                                setenv("twsi_mc", msg_buf);
                        }
                        printf("Error_0, Retry:%d, cData:%02x%02x%02x%02x %02x%02x%02x%02x\n",
                                iRetry, cData[0], cData[1], cData[2], cData[3], cData[4], cData[5], cData[6], cData[7]);
                        break;
                }

                LgDiag_IoRead(0x02, (u8*)cData);
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_MICOM","NG");
        return MV_FALSE;
}
int LgDiag_Check_MicomVer( u8 * pbData )
{
        //u8 bTemp[8];
        u8 bTemp[16];
        int ret;

        memset(bTemp,0x0, 16);
        if((ret = LgDiag_IoRead(0x2, bTemp)) == MV_TRUE)
                memcpy(pbData,(u8*)(&bTemp[1]),6);
        return ret;
}

int LgDiag_Check_Rtc( void )
{
        u8              data[8];
        //     ("1234567890123456789012345678901");
        dprintf (" Check RTC ....................");

        // read second count
        // int i2c_write(uchar dev_addr, unsigned int offset, int alen, uchar* data, int len)
        if(i2c_read( SSS_TWSI_ID_RTC, 0, 1,data,1) == MV_OK){
                setenv("LGDIAG_RTC","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_RTC","NG");
        return MV_FALSE;
}

int LgDiag_Check_Dram( void )
{
        //do_mem_mtest
        vu_long *addr, *start, *end;
        ulong   val;
        ulong   readback;
        ulong   incr;
        ulong   pattern = 0xaaaa5555;
        int     rcode = 0,i;

        //     ("1234567890123456789012345678901");
        dprintf (" Check Dram ...................");

        start = (ulong *)CFG_MEMTEST_START;
        end = (ulong *)CFG_MEMTEST_END;

        incr = 1;
        for (i=0;i<10;i++) {

                //dprintf ("\rPattern %08lX  Writing..."
                //      "%12s"
                //      "\b\b\b\b\b\b\b\b\b\b",
                //      pattern, "");

                for (addr=start,val=pattern; addr<end; addr++) {
                        *addr = val;
                        val  += incr;
                }

                //puts ("Reading...");

                for (addr=start,val=pattern; addr<end; addr++) {
                        readback = *addr;
                        if (readback != val) {
                                dprintf ("\nMem error @ 0x%08X: "
                                        "found %08lX, expected %08lX\n",
                                        (uint)addr, readback, val);
                                rcode = 1;
                        }
                        val += incr;
                }

                /*
                 * Flip the pattern each time to make lots of zeros and
                 * then, the next time, lots of ones.  We decrement
                 * the "negative" patterns and increment the "positive"
                 * patterns to preserve this feature.
                 */
                if(pattern & 0x80000000) {
                        pattern = -pattern;     /* complement & increment */
                }
                else {
                        pattern = ~pattern;
                }
                incr = -incr;

                if(rcode) break;
        }

        if(!rcode){
                setenv("LGDIAG_DRAM","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_DRAM","NG");
        return MV_FALSE;
}
int LgDiag_Check_Buzzer( u8 bMode )
{
        //     ("1234567890123456789012345678901234567");
        dprintf (" Check Buzzer .................");
        if(bMode == 0)//start
                LgDiag_Beep(0x11,0x13,0x15,0x21,20, 1, 1);
        else if(bMode == 1){//check error!
                LgDiag_Beep(0x15,0x11,0x15,0x11, 8, 5, 4);
        }else if(bMode == 2){//check ok1!
                LgDiag_Beep(0x21,0x15,0x13,0x11,20, 1, 1);
        }else if(bMode == 3){//check ok2!
                LgDiag_Beep(0x13,0x11,   0,   0, 8, 5, 1);
        }else if(bMode == 4){//check error2!
                LgDiag_Beep(0x11,0x15,0x11,0x15, 8, 5, 2);
        }
        dprintf ("[OK]\n");
        return MV_TRUE;
}

typedef struct table_fan {
        u8              bRpm;
        u16             wMinRpm;
        u16             wMaxRpm;
        char    *cMode;
} table_fan_t;

table_fan_t lgdiag_fan_mode[] = {
    {   0,              0,              100,    "LGDIAG_FAN_STOP",      },
        {  17,    700,     1100,        "LGDIAG_FAN_SILENT",},//900,
    {  27,   1000,     1400,    "LGDIAG_FAN_NORMAL",},//1200,
    { 255,   1300,         2000,        "LGDIAG_FAN_COOL",      },//1560, 12V
};

int LgDiag_Check_Fan( u8 bMode, u16* wRpm )
{
        //     ("1234567890123456789012345678901234567");
        dprintf (" Check Fan ....................");
        LgDiag_IoWrite( 0x8f,lgdiag_fan_mode[bMode].bRpm,0,0,0,0,0 );
        dprintf ("[%d].",lgdiag_fan_mode[bMode].bRpm);
        mdelay(4000);
        if( !LgDiag_GetRpm(wRpm) ){
                setenv("LGIAG_FAN_GETRPM","NG");
                LgDiag_IoWrite( 0x8f,lgdiag_fan_mode[3].bRpm,0,0,0,0,0 );
                return MV_FALSE;
        }

        if(     ( *wRpm < lgdiag_fan_mode[bMode].wMinRpm)||
                ( *wRpm > lgdiag_fan_mode[bMode].wMaxRpm)){
                dprintf ("[NG]\n");
                setenv(lgdiag_fan_mode[bMode].cMode,"NG");
                LgDiag_IoWrite( 0x8f,lgdiag_fan_mode[3].bRpm,0,0,0,0,0 );
                return MV_FALSE;
        }

        LgDiag_IoWrite( 0x8f,lgdiag_fan_mode[3].bRpm,0,0,0,0,0 );
        setenv("LGDIAG_FAN","OK");
        dprintf ("[OK]\n");
        return MV_TRUE;
}

int LgDiag_Check_Button( void )
{
        u8 data[8];
        int i;

        //12345678901234567890123456789012345678");
        dprintf (" Check Button .................");
        // test 3 seconds
        i = 0;
        while (i++ < 100){
                mdelay(100);
                i2c_read( SSS_TWSI_ID_MICOM, 0, 0,data,8);

                if((data[0] != 1)||(data[2] != 0)) continue;
                mdelay(100);
                dprintf("[BTN:");
                if(LgDiag_IoRead(0, data) != MV_TRUE) {

                        dprintf(" Reason Error]");
                        break;
                }
                dprintf("0x%08x",*(u32*)data);

                if(data[1] == 0x01)     dprintf("Power].");
                else if(data[1] == 0x02) dprintf("Menu]."); // no use
                else if(data[1] == 0x04) dprintf("Backup].");
                else if(data[1] == 0x08) dprintf("Eject].");
                else if(data[1] == 0x10) dprintf("Burn].");
                else if(data[1] == 0x20) dprintf("USB].");
                else if(data[1] == 0x40) dprintf("Cancel].");
                else {dprintf("Unknown:0x%x].",data[1]);continue;}
        }

        setenv("LGDIAG_BUTTON","OK");
        dprintf ("[OK]\n");
        return MV_TRUE;
}

int LgDiag_Check_Phy( void )
{
        u16 wTemp;
        //     ("1234567890123456789012345678901234567");
        dprintf (" Check Phy ....................");
        mvEthPhyRegRead(0x08, 0x0, &wTemp);

        if(wTemp == 0x1140){
                setenv("LGDIAG_PHY","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_PHY","NG");
        return MV_FALSE;
}

int LgDiag_Check_Pci( void )
{
        unsigned short DeviceID;
        pci_dev_t dev;


        //     ("1234567890123456789012345678901234567");
        dprintf (" Check Pci ....................");
        // scan only bus 0 functon 0
        // Bus.Dev.Fun 0.0.0 : 5281 Memory controller
        // Bus.Dev.Fun 0.1.0 : 7042 Mass storage controller
        // Bus.Dev.Fun 1.7.0 : 6081 Mass storage controller --> new board
        //dev = ((b) << 16 | (d) << 11 | (f) << 8)
        dev = ((0) << 16 | (1) << 11 | (0) << 8);
        pci_read_config_word(dev, PCI_DEVICE_ID, &DeviceID);
        dprintf("[DEV 0x%x].",DeviceID);
        if(DeviceID == 0x7042){
                setenv("LGDIAG_PCI","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dev = ((1) << 16 | (7) << 11 | (0) << 8);
        pci_read_config_word(dev, PCI_DEVICE_ID, &DeviceID);
        dprintf("[DEV 0x%x].",DeviceID);
        if(DeviceID == 0x6081){
                setenv("LGDIAG_PCI","OK");
                dprintf ("[OK]\n");
                return MV_TRUE;
        }
        dprintf ("[NG]\n");
        setenv("LGDIAG_PCI","NG");
        return MV_FALSE;

}

int LgDiag_Check_Mac( void )
{
        //     ("1234567890123456789012345678901234567");
        dprintf ("LgDiag Check Mac ...................");
        if((getenv("ethaddr") == NULL) ||(strcmp (SSS_ETHADDR, getenv("ethaddr")) == 0)){
                printf("Mac NG [ Default Mac ] \n");
                LgDiag_Msg2LcdKeep("Check Mac Address", 0);
                return MV_FALSE;
        }
        else
                return MV_TRUE;
}
int LgDiag_Check_Sata( void )
{
        //     ("1234567890123456789012345678901234567");
        dprintf ("LgDiag Check Sata ...................");

        return MV_TRUE;
}
int LgDiag_Check_Serial( void )
{
        //     ("1234567890123456789012345678901234567");
        dprintf ("LgDiag Check Serial .................");

        return MV_TRUE;
}
int LgDiag_Check_Usb( void )
{
        //     ("1234567890123456789012345678901234567");
        dprintf ("LgDiag Check usb ....................");

        return MV_TRUE;
}

int LgDiag_CheckAll( void )
{
        u16 wRpm;

        LgDiag_Check_Buzzer(0);

        if(LgDiag_Check_MainIC() != MV_TRUE ) {
                printf("MainIC NG\n");
                return MV_FALSE;
        }
        if(LgDiag_Check_Twsi() != MV_TRUE ){
                printf("Twsi NG\n");
                return MV_FALSE;
        }
        if(LgDiag_Check_Micom() != MV_TRUE ){
                printf("Micom NG\n");
                return MV_FALSE;
        }
        if(LgDiag_Check_Rtc() != MV_TRUE ){
                printf("Rtc NG\n");
                return MV_FALSE;
        }
        if(LgDiag_Check_Dram() != MV_TRUE ){
                printf("Dram NG\n");
                return MV_FALSE;
        }
        if(LgDiag_Check_Pci() != MV_TRUE ){
                printf("Pci NG\n");
                return MV_FALSE;
        }

        if(LgDiag_Check_Fan(3, &wRpm) != MV_TRUE ){
                printf("Fan NG [ Rpm : %d ] \n",wRpm);
                return MV_FALSE;
        }
        if(LgDiag_Check_Button() != MV_TRUE ){
                printf("Button NG\n");
                return MV_FALSE;
        }
        LgDiag_Check_Buzzer(2);
        return MV_TRUE;
}

#define SCODE_MAINIC    0x11
#define SCODE_TWSI              0x12
#define SCODE_DDR2              0x13
#define SCODE_SATACONT  0x14
#define SCODE_MCU               0x15
#define SCODE_RTC               0x16
#define SCODE_GPHY              0x17
#define SCODE_USBHUB    0x18
#define SCODE_FAN               0x19
#define SCODE_KERNEL    0x1A
int LgDiag_ScodeDisp(u8 bEnc)
{
        static char sTemp[10];

        sprintf (sTemp, "SVC_CODE[%02X_00]", bEnc);
        LgDiag_Msg2LcdKeep(sTemp, 0);
        return MV_TRUE;
}
int LgDiag_CheckSystem( void )
{
        u16 wRpm;

        dprintf ("LgDiag_CheckSystem \n");
        //LgDiag_IoWrite( 0x86,0,1,0,0,0,0 );
        LgDiag_Msg2LcdKeep("Checking LG-NAS...", 1);

        //LgDiag_Check_Buzzer(0);//start

        if(LgDiag_Check_MainIC() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check MAIN-IC", 0);
                LgDiag_ScodeDisp(SCODE_MAINIC);
                return MV_FALSE;
        }
        if(LgDiag_Check_Twsi() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check TWSI", 0);
                LgDiag_ScodeDisp(SCODE_TWSI);
                return MV_FALSE;
        }
        if(LgDiag_Check_Micom() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check MCU", 0);
                LgDiag_ScodeDisp(SCODE_MCU);
                return MV_FALSE;
        }
        if(LgDiag_Check_Rtc() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check RTC", 0);
                LgDiag_ScodeDisp(SCODE_RTC);
                return MV_FALSE;
        }
        if(LgDiag_Check_Dram() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check DRAM", 0);
                LgDiag_ScodeDisp(SCODE_DDR2);
                return MV_FALSE;
        }
        if(LgDiag_Check_Phy() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check PHY", 0);
                LgDiag_ScodeDisp(SCODE_GPHY);
                return MV_FALSE;
        }
        if(LgDiag_Check_Pci() != MV_TRUE ){
                //LgDiag_Msg2LcdKeep("Check PCI", 0);
                LgDiag_ScodeDisp(SCODE_SATACONT);
                return MV_FALSE;
        }
        if(LgDiag_Check_Fan(3, &wRpm) != MV_TRUE ){
                printf("Fan NG [ Rpm : %d ] \n",wRpm);
                //LgDiag_Msg2LcdKeep("Check FAN Status", 0);
                LgDiag_ScodeDisp(SCODE_FAN);
                return MV_FALSE;
        }

        LgDiag_Msg2LcdKeep("Loading Kernel...", 0);
        LgDiag_Check_Buzzer(3);//end ok2
        return MV_TRUE;
}

int LgDiag_cmd( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
        char c;
        int err, i;
        extern char version_string[];
        u16 wRpm;
        u8 bData[16];

        LgDiag_Check_MicomVer(bData);
        LgDiag_IoWrite( 0x86,0,1,0,0,0,0 );

        while(1) {
                printf( "===========================================================\n" );
                printf( "LGE Board Diagnostic \n" );
                printf( "Uboot : %s\n", version_string );
                printf( "MCU   : %02d-%d-%d %d/%d #%d\n",bData[0],bData[1],bData[2],bData[3],bData[4],bData[5]);
                printf( "MAC   : %s\n", getenv("ethaddr"));
                printf( "(0) Check All.         \t(1) Check MainIC \n" );
                printf( "(2) Check Twsi.        \t(3) Check Micom \n" );
                printf( "(4) Check Rtc.         \t(5) Check Dram \n" );
                printf( "(6) Check Pci.         \t(7) Check Buzzer \n" );
                printf( "(8) Check Fan.         \t(9) Check Button \n" );
                printf( "(a) Check LCD(D1)      \t(b) Check LCD(D2) \n" );
                printf( "(.) Exit LGE Diag.\n" );
                printf( "===========================================================\n" );

                c = getc();
                printf( "%c\n", c );

                if( c == '.')
                        break;

                err = MV_FALSE;

                switch( c ) {
                        case '0': /* Enter */
                                printf("Testing All...\n");
                                LgDiag_Msg2Lcd("LGDiag Testing All...", 1);
                                err = LgDiag_CheckAll();
                                if(err)
                                        printf("All OK\n");
                                break;
                        case '1':
                                printf("Testing MainIC...\n");
                                err = LgDiag_Check_MainIC();
                                if(!err){
                                        printf("MainIC NG\n");
                                        LgDiag_Msg2LcdKeep("MainIC NG",0);
                                }else
                                        printf("MainIC OK\n");
                                break;
                        case '2':
                                printf("Testing Twsi...\n");
                                err = LgDiag_Check_Twsi();
                                if(!err){
                                        printf("Twsi NG\n");
                                        LgDiag_Msg2LcdKeep("MainIC NG",0);
                                }else
                                        printf("Twsi OK\n");
                                break;
                        case '3':
                                printf("Testing Micom...\n");
                                err = LgDiag_Check_Micom();
                                if(!err){
                                        printf("Micom NG\n");
                                        LgDiag_Msg2LcdKeep("Micom  NG",0);
                                }else
                                        printf("Micom OK\n");
                                break;
                        case '4':
                                printf("Testing Rtc...\n");
                                err = LgDiag_Check_Rtc();
                                if(!err){
                                        printf("Rtc NG\n");
                                        LgDiag_Msg2LcdKeep("Rtc  NG",0);
                                }else
                                        printf("Rtc OK\n");
                                break;
                        case '5':
                                printf("Testing Dram...\n");
                                err = LgDiag_Check_Dram();
                                if(!err){
                                        printf("Dram NG\n");
                                        LgDiag_Msg2LcdKeep("Dram  NG",0);
                                }else
                                        printf("Dram OK\n");
                                break;
                        case '6':
                                printf("Testing Pci...\n");
                                err = LgDiag_Check_Pci();
                                if(!err){
                                        printf("Pci NG\n");
                                        LgDiag_Msg2LcdKeep("Dram  NG",0);
                                }else
                                        printf("Pci OK\n");
                                break;
                        case '7':
                                printf("Testing Buzzer...\n");
                                err = LgDiag_Check_Buzzer(0);//start
                                err = LgDiag_Check_Buzzer(2);//end1
                                err = LgDiag_Check_Buzzer(3);//end2
                                printf("Buzzer OK\n");
                                break;
                        case '8':
                                printf("Testing Fan...\n");
                                for(i=3; i>=0; i--){
                                        err = LgDiag_Check_Fan((u8)i, &wRpm);
                                        if(!err){
                                                printf("Fan NG [mode:%d ,rpm:%d]\n",i, wRpm);
                                                LgDiag_Msg2LcdKeep("Fan  NG",0);
                                                break;
                                        }else
                                                printf("Fan OK [mode:%d ,rpm:%d]\n",i, wRpm);
                                }
                                break;
                        case '9':
                                printf("Testing Button...\n");
                                err = LgDiag_Check_Button();
                                if(!err){
                                        printf("Button NG\n");
                                        LgDiag_Msg2LcdKeep("Button  NG",0);
                                }else
                                        printf("Button OK\n");
                                break;
                        case 'a':
                                printf("Testing Lcd D1...\n");
                                err = LgDiag_Msg2Lcd("LCD TEST D1",0);
                                if(!err)
                                        printf("LCD NG\n");
                                else
                                        printf("LCD OK\n");
                                break;
                        case 'b':
                                printf("Testing Lcd D2...\n");
                                err = LgDiag_Msg2LcdKeep("LCD TEST D2",0);
                                if(!err)
                                        printf("LCD NG\n");
                                else
                                        printf("LCD OK\n");
                                break;

                        default:
                                printf( "Bad value !!!\n" );
                                err = MV_TRUE;
                }

                if( !err ) {
                        LgDiag_Msg2Lcd("LGDiag Test NG",0);
                        break;
                }
                //LgDiag_Msg2Lcd("LGDiag Test OK",0);
        }

        return 1;
}

static inline void LgDiag_readline(unsigned char *buf, int bufsiz)
{
        char c;
        char *p;
        int n;

        n = 0;
        p = (char*)buf;
        for(;;) {
                c = mvUartGetc(CFG_DUART_CHAN);

                switch(c) {
                case '\r':                              /* Enter                */
                case '\n':
                        *p = '\0';
                        //puts ("\r\n");
                        return;

                default:
                        if(n++ > bufsiz) {
                                *p = '\0';
                                return; /* sanity check */
                        }
                        *p = c;
                        p++;
                        break;
                }
        }
}
//mvUartPutc(CFG_DUART_CHAN, c);
//serial_puts (s);
#define LGDIAG_BUFSIZE 20
#define LGDIAG_DATASIZE 10
typedef struct lgdiag_buf_s {
        u8      bCmd;
        u8      bId;
        u8      bRsv1;
        u8      bRslt;
        u8      bData[14];
        u16 wErCode;
}LGDIAG_BUF;

void LgDiag_puts (const u8 *s)
{
        int cnt = LGDIAG_BUFSIZE;

        while ( cnt-- > 0)
                serial_putc (*s++);
}
#define LG_OK 0
#define LG_NG 1
#define LG_END 2

//error code
#define LG_ER_COMM                              0x0001
#define LG_ER_NOTSUPPORT                0x0002
#define LG_ER_CHEKSUM                   0x0003
#define LG_ER_COMPARE                   0x0004
#define LG_ER_NULLPOINTER               0x0005

extern char version_string[];
extern void reset_cpu(void);
int LgDiag_SMT( void )
{

        LGDIAG_BUF InBuf, OutBuf;

        int ret;
        u16 wTemp;
        pci_dev_t dev;
        char cTemp[16];
        char *cpTemp;

        ret = LG_NG;
        //LgDiag_Msg2Lcd("SMT Test Mode : Waiting Host Connection...", 0);
        dprintf("SMT Test Mode : Waiting Host Connection...\n");
        while(1) {
                LgDiag_readline ( (u8*)&InBuf, LGDIAG_BUFSIZE);
                dprintf("LgDiag_readline bCmd : %02x\n",InBuf.bCmd);
                dprintf("LgDiag_readline bId : %02x\n",InBuf.bId);
                memset((u8*)&OutBuf,0x00,LGDIAG_BUFSIZE);

                switch(InBuf.bCmd){

                // confirm connection
                case 0x40:
                        LgDiag_Msg2Lcd("Host Connected!", 0);
                        // uart init
                        serial_setbrg();
                        udelay(100000);

                        putc(0x04);
                        putc('J');putc('W');putc('B');
                        putc('L');putc('S');putc('Y');
                        continue;
                        break;
                case 0x53:
                        OutBuf.bCmd     = InBuf.bCmd;
                        OutBuf.bId      = InBuf.bId;
                        OutBuf.bRsv1= InBuf.bRsv1;
                        switch(InBuf.bId){
                        //main ic
                        case 0x01:
                                LgDiag_Msg2Lcd("[MainIC] Test", 0);
                                if(mvCtrlModelRevNameGet(cTemp) == MV_OK){
                                        memcpy(OutBuf.bData, cTemp, 10);
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //flash
                        case 0x02:
                                LgDiag_Msg2Lcd("[Flash ]Test", 0);
                                memcpy(OutBuf.bData, version_string, 10);
                                ret = LG_OK;
                                break;
                        //ddr
                        case 0x03:
                                LgDiag_Msg2Lcd("[DDR   ]Test", 0);
                                if(LgDiag_Check_Dram() == MV_TRUE){
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //sata host controller
                        case 0x04:
                                LgDiag_Msg2Lcd("[SATA  ]Test", 0);
                                dev = ((1) << 16 | (7) << 11 | (0) << 8);
                                pci_read_config_word(dev, PCI_DEVICE_ID, &wTemp);
                                if(wTemp == 0x6081){
                                        memcpy(OutBuf.bData, (u8 *)&wTemp, 2);
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //micom version
                        case 0x05:
                                LgDiag_Msg2Lcd("[MICOM ]Test", 0);
                                if(LgDiag_Check_MicomVer(OutBuf.bData) == MV_TRUE){
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //rtc
                        case 0x06:
                                LgDiag_Msg2Lcd("[RTC   ]Test", 0);
                                if(i2c_read( SSS_TWSI_ID_RTC, 0, 1,OutBuf.bData,1) == MV_OK){
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //giga phy
                        case 0x07:
                                LgDiag_Msg2Lcd("[PHY   ]Test", 0);
                                mvEthPhyRegRead(0x08, 0x0, &wTemp);
                                if(wTemp == 0x1140){
                                        memcpy(OutBuf.bData, (u8 *)&wTemp, 2);
                                        ret = LG_OK;
                                }else{
                                        ret = LG_NG;
                                }
                                break;
                        //usb hub
                        case 0x08:
                                LgDiag_Msg2Lcd("[HUB   ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //fan
                        case 0x09:
                                LgDiag_Msg2Lcd("[FAN   ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //hdd1
                        case 0x1a:
                                LgDiag_Msg2Lcd("[HDD1  ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //hdd2
                        case 0x1b:
                                LgDiag_Msg2Lcd("[HDD2  ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //hdd3
                        case 0x1c:
                                LgDiag_Msg2Lcd("[HDD3  ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //hdd4
                        case 0x1d:
                                LgDiag_Msg2Lcd("[HDD4  ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //odd
                        case 0x1e:
                                LgDiag_Msg2Lcd("[ODD1  ]Test", 0);
                                OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                ret = LG_NG;
                                break;
                        //write serial
                        case 0x3b:
                                LgDiag_Msg2Lcd("[Ser# W]Test", 0);
                                memcpy(cTemp, InBuf.bData, 16);
                                setenv("serialNo",cTemp);
                                //OutBuf.wErCode = LG_ER_NOTSUPPORT;
                                if(strcmp (cTemp, getenv("serialNo")) == 0)
                                        ret = LG_OK;
                                else{
                                        OutBuf.wErCode = LG_ER_COMPARE;
                                        ret = LG_NG;
                                }
                                break;
                        //read serial
                        case 0x3c:
                                LgDiag_Msg2Lcd("[Ser# R]Test", 0);
                                cpTemp = (char*)getenv("serialNo");
                                if(cpTemp != NULL){
                                        ret = LG_OK;
                                        memcpy(OutBuf.bData, cpTemp, 16);
                                }else{
                                        OutBuf.wErCode = LG_ER_NULLPOINTER;
                                        ret = LG_NG;
                                }
                                break;
                        //reset
                        case 0x44:
                                LgDiag_Msg2Lcd("[Reset]Test", 0);
                                disable_interrupts ();
                                reset_cpu();
                                ret = LG_OK;
                                break;
                        //poweroff
                        case 0x43:
                                LgDiag_Msg2Lcd("[Down ]Test", 0);
                                //booted
                                //LgDiag_IoWrite(0x86,0,1,0,0,0,0 );
                                //shutting down....
                                LgDiag_IoWrite(0x86,0,2,0,0,0,0);
                                //halted
                                LgDiag_IoWrite(0x86,0,3,0,0,0,0);
                                ret = LG_OK;
                                break;
                        //buzzeron
                        case 0x21:
                                LgDiag_Msg2Lcd("[Buzzer]Test", 0);
                                LgDiag_Beep(0x11,0x13,0x15,0x21,20, 1, 1);
                                LgDiag_Beep(0x21,0x15,0x13,0x11,20, 1, 1);
                                break;
                        //flash save
                        case 0xf0:
                                saveenv();
                                LgDiag_Msg2Lcd("flash saved", 0);
                                ret = LG_OK;
                                break;
                        default:
                                break;
                        }

                        if(ret == LG_OK)        OutBuf.bRslt= 'O';//ok
                        else                            OutBuf.bRslt= 'N';//ng

                        break;
                case 0xf0:
                        saveenv();
                        ret = LG_END;
                        LgDiag_Msg2Lcd("SMT Test Mode : Complete", 0);
                        break;
                default:
                        break;
                }
                if(ret == LG_END) break;

                LgDiag_puts ((u8*)&OutBuf);
                //putc('/n');

        }
        return 0;
}

U_BOOT_CMD(
        lgdiag,      4,     1,      LgDiag_cmd,
        "lgdiag - Check the LG Board Diag.\n",
        " \n"
        "\tDisplay Result of LG Board Diagnostic.\n"
);

/******************************************************************************
* Functional only when using Lauterbach to load image into DRAM
* Category     - DEBUG
* Functionality- Display the array of registers the u-boot write to.
*
*****************************************************************************/
#if defined(REG_DEBUG)
int reg_arry[REG_ARRAY_SIZE][2];
int reg_arry_index = 0;
int print_registers( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int i;
	printf("Register display\n");

	for (i=0; i < reg_arry_index; i++)
		printf("Index %d 0x%x=0x%08x\n", i, (reg_arry[i][0] & 0x000fffff), reg_arry[i][1]);
	
	/* Print DRAM registers */	
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1500, MV_REG_READ(0x1500));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1504, MV_REG_READ(0x1504));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1508, MV_REG_READ(0x1508));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x150c, MV_REG_READ(0x150c));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1510, MV_REG_READ(0x1510));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1514, MV_REG_READ(0x1514));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1518, MV_REG_READ(0x1518));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x151c, MV_REG_READ(0x151c));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1400, MV_REG_READ(0x1400));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1404, MV_REG_READ(0x1404));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1408, MV_REG_READ(0x1408));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x140c, MV_REG_READ(0x140c));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1410, MV_REG_READ(0x1410));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x141c, MV_REG_READ(0x141c));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1420, MV_REG_READ(0x1420));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1424, MV_REG_READ(0x1424));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1428, MV_REG_READ(0x1428));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x147c, MV_REG_READ(0x147c));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1494, MV_REG_READ(0x1494));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x1498, MV_REG_READ(0x1498));
	printf("Index %d 0x%x=0x%08x\n", i++, 0x149c, MV_REG_READ(0x149c));

	printf("Number of Reg %d \n", i);

	return 1;
}

U_BOOT_CMD(
	printreg,      1,     1,      print_registers,
	"printreg	- Display the register array the u-boot write to.\n",
	" \n"
	"\tDisplay the register array the u-boot write to.\n"
);
#endif

#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)
/******************************************************************************
* Category     - Etherent
* Functionality- Display PHY ports status (using SMI access).
* Need modifications (Yes/No) - No
*****************************************************************************/
int sg_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(MV_INC_BOARD_QD_SWITCH)
		printf( "Switch status not supported\n");
#else
	MV_U32 port;
	for( port = 0 ; port < mvCtrlEthMaxPortGet(); port++ ) {

		printf( "PHY %d :\n", port );
		printf( "---------\n" );

		mvEthPhyPrintStatus( mvBoardPhyAddrGet(port) );

		printf("\n");
	}
#endif
	return 1;
}

U_BOOT_CMD(
	sg,      1,     1,      sg_cmd,
	"sg	- scanning the PHYs status\n",
	" \n"
	"\tScan all the Gig port PHYs and display their Duplex, Link, Speed and AN status.\n"
);
#endif /* #if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH) */

#if defined(MV_INCLUDE_IDMA)

/******************************************************************************
* Category     - DMA
* Functionality- Perform a DMA transaction
* Need modifications (Yes/No) - No
*****************************************************************************/
int mvDma_cmd( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[] )
{
	MV_8 cmd[20], c;
	extern MV_8 console_buffer[];
	MV_U32 chan, src, dst, byteCount, ctrlLo;
	MV_DMA_DEC_WIN win;
	MV_BOOL err;

	/* IDMA channel */
	if( argc == 2 ) 
		chan = simple_strtoul( argv[1], NULL, 16 );
	else
		chan = 0;

	/* source address */
	while(1) {
		readline( "Source Address: " );
		strcpy( cmd, console_buffer );
		src = simple_strtoul( cmd, NULL, 16 );
		if( src == 0xffffffff ) printf( "Bad address !!!\n" );
		else break;
	}

	/* desctination address */
	while(1) {
		readline( "Destination Address: " );
		strcpy(cmd, console_buffer);
		dst = simple_strtoul( cmd, NULL, 16 );
		if( dst == 0xffffffff ) printf("Bad address !!!\n");
		else break;
	}

	/* byte count */
	while(1) {
		readline( "Byte Count (up to 16M (0xffffff-1)): " );
		strcpy( cmd, console_buffer );
		byteCount = simple_strtoul( cmd, NULL, 16 );
		if( (byteCount > 0xffffff) || (byteCount == 0) ) printf("Bad value !!!\n");
		else break;
	}

	/* compose the command */
	ctrlLo = ICCLR_BLOCK_MODE | ICCLR_NON_CHAIN_MODE | ICCLR_SRC_INC | ICCLR_DST_INC;


	if (byteCount > _64K)
	{
		ctrlLo |= ICCLR_DESC_MODE_16M;
	}

	/* set data transfer limit */
	while(1) {
		printf( "Data transfer limit:\n" );
		printf( "(1) 8   bytes at a time.\n" );
		printf( "(2) 16  bytes at a time.\n" );
		printf( "(3) 32  bytes at a time.\n" );
		printf( "(4) 64  bytes at a time.\n" );
		printf( "(5) 128 bytes at a time.\n" );

		c = getc(); 
		printf( "%c\n", c );

		err = MV_FALSE;

		switch( c ) {
			case 13: /* Enter */
				ctrlLo |= (ICCLR_DST_BURST_LIM_32BYTE | ICCLR_SRC_BURST_LIM_32BYTE);
				printf( "32 bytes at a time.\n" );
				break;
			case '1':
				ctrlLo |= (ICCLR_DST_BURST_LIM_8BYTE | ICCLR_SRC_BURST_LIM_8BYTE);
				break;
			case '2':
				ctrlLo |= (ICCLR_DST_BURST_LIM_16BYTE | ICCLR_SRC_BURST_LIM_16BYTE);
				break;
			case '3':
				ctrlLo |= (ICCLR_DST_BURST_LIM_32BYTE | ICCLR_SRC_BURST_LIM_32BYTE);
				break;
			case '4':
				ctrlLo |= (ICCLR_DST_BURST_LIM_64BYTE | ICCLR_SRC_BURST_LIM_64BYTE);
				break;
			case '5':
				ctrlLo |= (ICCLR_DST_BURST_LIM_128BYTE | ICCLR_SRC_BURST_LIM_128BYTE);
				break;
			default:
				printf( "Bad value !!!\n" );
				err = MV_TRUE;
		}

		if( !err ) break;
	}

	/* set ovveride source option */
	while(1) {
		printf( "Override Source:\n" ); 
		printf( "(0) - no override\n" );
		mvDmaWinGet( 1, &win );
		printf( "(1) - use Win1 (%s)\n",mvCtrlTargetNameGet(win.target));
		mvDmaWinGet( 2, &win );
		printf( "(2) - use Win2 (%s)\n",mvCtrlTargetNameGet(win.target));
		mvDmaWinGet( 3, &win );
		printf( "(3) - use Win3 (%s)\n",mvCtrlTargetNameGet(win.target));

		c = getc(); 
		printf( "%c\n", c );

		err = MV_FALSE;

		switch( c ) {
			case 13: /* Enter */
			case '0':
				printf( "No override\n" );
				break;
			case '1':
				ctrlLo |= ICCLR_OVRRD_SRC_BAR(1);
				break;
			case '2':
				ctrlLo |= ICCLR_OVRRD_SRC_BAR(2);
				break;
			case '3':
				ctrlLo |= ICCLR_OVRRD_SRC_BAR(3);
				break;
			default:
				printf("Bad value !!!\n");
				err = MV_TRUE;
		}

		if( !err ) break;
	}

	/* set override destination option */
	while(1) {
		printf( "Override Destination:\n" ); 
		printf( "(0) - no override\n" );
		mvDmaWinGet( 1, &win );
		printf( "(1) - use Win1 (%s)\n",mvCtrlTargetNameGet(win.target));
		mvDmaWinGet( 2, &win );
		printf( "(2) - use Win2 (%s)\n",mvCtrlTargetNameGet(win.target));
		mvDmaWinGet( 3, &win );
		printf( "(3) - use Win3 (%s)\n",mvCtrlTargetNameGet(win.target));

		c = getc(); 
		printf( "%c\n", c );

		err = MV_FALSE;

	        switch( c ) {
			case 13: /* Enter */
			case '0':
				printf( "No override\n" );
				break;
			case '1':
				ctrlLo |= ICCLR_OVRRD_DST_BAR(1);
				break;
			case '2':
				ctrlLo |= ICCLR_OVRRD_DST_BAR(2);
				break;
			case '3':
				ctrlLo |= ICCLR_OVRRD_DST_BAR(3);
				break;
			default:
				printf("Bad value !!!\n");
				err = MV_TRUE;
		}

		if( !err ) break;
	}

	/* wait for previous transfer completion */
	while( mvDmaStateGet(chan) != MV_IDLE );

	/* issue the transfer */
	mvDmaCtrlLowSet( chan, ctrlLo ); 
	mvDmaTransfer( chan, src, dst, byteCount, 0 );

	/* wait for completion */
	while( mvDmaStateGet(chan) != MV_IDLE );

	printf( "Done...\n" );
	return 1;
}

U_BOOT_CMD(
	dma,      2,     1,      mvDma_cmd,
	"dma	- Perform DMA\n",
	" \n"
	"\tPerform DMA transaction with the parameters given by the user.\n"
);

#endif /* #if defined(MV_INCLUDE_IDMA) */

/******************************************************************************
* Category     - Memory
* Functionality- Displays the MV's Memory map
* Need modifications (Yes/No) - Yes
*****************************************************************************/
int displayMemoryMap_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	mvCtrlAddrDecShow();
	return 1;
}

U_BOOT_CMD(
	map,      1,     1,      displayMemoryMap_cmd,
	"map	- Diasplay address decode windows\n",
	" \n"
	"\tDisplay controller address decode windows: CPU, PCI, Gig, DMA, XOR and COMM\n"
);



#include "ddr2/spd/mvSpd.h"
#if defined(MV_INC_BOARD_DDIM)

/******************************************************************************
* Category     - Memory
* Functionality- Displays the SPD information for a givven dimm
* Need modifications (Yes/No) - 
*****************************************************************************/
              
int dimminfo_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
        int num = 0;
 
        if (argc > 1) {
                num = simple_strtoul (argv[1], NULL, 10);
        }
 
        printf("*********************** DIMM%d *****************************\n",num);
 
        dimmSpdPrint(num);
 
        printf("************************************************************\n");
         
        return 1;
}
 
U_BOOT_CMD(
        ddimm,      2,     1,      dimminfo_cmd,
        "ddimm  - Display SPD Dimm Info\n",
        " [0/1]\n"
        "\tDisplay Dimm 0/1 SPD information.\n"
);

/******************************************************************************
* Category     - Memory
* Functionality- Copy the SPD information of dimm 0 to dimm 1
* Need modifications (Yes/No) - 
*****************************************************************************/
              
int spdcpy_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
 
        printf("Copy DIMM 0 SPD data into DIMM 1 SPD...");
 
        if (MV_OK != dimmSpdCpy())
        	printf("\nDIMM SPD copy fail!\n");
 	else
        	printf("Done\n");
         
        return 1;
}
 
U_BOOT_CMD(
        spdcpy,      2,     1,      spdcpy_cmd,
        "spdcpy  - Copy Dimm 0 SPD to Dimm 1 SPD \n",
        ""
        ""
);
#endif /* #if defined(MV_INC_BOARD_DDIM) */

/******************************************************************************
* Functionality- Go to an address and execute the code there and return,
*    defualt address is 0x40004
*****************************************************************************/
extern void cpu_dcache_flush_all(void);
extern void cpu_icache_flush_invalidate_all(void);

void mv_go(unsigned long addr,int argc, char *argv[])
{
	int rc;
	addr = MV_CACHEABLE(addr);
	char* envCacheMode = getenv("cacheMode");
 
	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
	 */

    if(envCacheMode && (strcmp(envCacheMode,"write-through") == 0))
	{	
		int i=0;

		/* Flush Invalidate I-Cache */
		cpu_icache_flush_invalidate_all();

		/* Drain write buffer */
		asm ("mcr p15, 0, %0, c7, c10, 4": :"r" (i));
		

	}
	else /*"write-back"*/
	{
		int i=0;

		/* Flush Invalidate I-Cache */
		cpu_icache_flush_invalidate_all();

		/* Drain write buffer */
		asm ("mcr p15, 0, %0, c7, c10, 4": :"r" (i));
		

		/* Flush invalidate D-cache */
		cpu_dcache_flush_all();


    }


	rc = ((ulong (*)(int, char *[]))addr) (--argc, &argv[1]);
 
        return;
}

int g_cmd (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
        ulong   addr;

	if(!enaMonExt()){
		printf("This command can be used only if enaMonExt is set!\n");
		return 0;
	}

	addr = 0x40000;

        if (argc > 1) {
		addr = simple_strtoul(argv[1], NULL, 16);
        }
	mv_go(addr,argc,&argv[0]);
	return 1;
}                                                                                                                     

U_BOOT_CMD(
	g,      CFG_MAXARGS,     1,      g_cmd,
        "g	- start application at cached address 'addr'(default addr 0x40000)\n",
        " addr [arg ...] \n"
	"\tStart application at address 'addr'cachable!!!(default addr 0x40004/0x240004)\n"
	"\tpassing 'arg' as arguments\n"
	"\t(This command can be used only if enaMonExt is set!)\n"
);

/******************************************************************************
* Functionality- Searches for a value
*****************************************************************************/
int fi_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    MV_U32 s_address,e_address,value,i,tempData;
    MV_BOOL  error = MV_FALSE;

    if(!enaMonExt()){
	printf("This command can be used only if enaMonExt is set!\n");
	return 0;}

    if(argc == 4){
	value = simple_strtoul(argv[1], NULL, 16);
	s_address = simple_strtoul(argv[2], NULL, 16);
	e_address = simple_strtoul(argv[3], NULL, 16);
    }else{ printf ("Usage:\n%s\n", cmdtp->usage);
    	return 0;
    }     

    if(s_address == 0xffffffff || e_address == 0xffffffff) error = MV_TRUE;
    if(s_address%4 != 0 || e_address%4 != 0) error = MV_TRUE;
    if(s_address > e_address) error = MV_TRUE;
    if(error)
    {
	printf ("Usage:\n%s\n", cmdtp->usage);
        return 0;
    }
    for(i = s_address; i < e_address ; i+=4)
    {
        tempData = (*((volatile unsigned int *)i));
        if(tempData == value)
        {
            printf("Value: %x found at ",value);
            printf("address: %x\n",i);
            return 1;
        }
    }
    printf("Value not found!!\n");
    return 1;
}

U_BOOT_CMD(
	fi,      4,     1,      fi_cmd,
	"fi	- Find value in the memory.\n",
	" value start_address end_address\n"
	"\tSearch for a value 'value' in the memory from address 'start_address to\n"
	"\taddress 'end_address'.\n"
	"\t(This command can be used only if enaMonExt is set!)\n"
);

/******************************************************************************
* Functionality- Compare the memory with Value.
*****************************************************************************/
int cmpm_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    MV_U32 s_address,e_address,value,i,tempData;
    MV_BOOL  error = MV_FALSE;

    if(!enaMonExt()){
	printf("This command can be used only if enaMonExt is set!\n");
	return 0;}

    if(argc == 4){
	value = simple_strtoul(argv[1], NULL, 16);
	s_address = simple_strtoul(argv[2], NULL, 16);
	e_address = simple_strtoul(argv[3], NULL, 16);
    }else{ printf ("Usage:\n%s\n", cmdtp->usage);
    	return 0;
    }     

    if(s_address == 0xffffffff || e_address == 0xffffffff) error = MV_TRUE;
    if(s_address%4 != 0 || e_address%4 != 0) error = MV_TRUE;
    if(s_address > e_address) error = MV_TRUE;
    if(error)
    {
	printf ("Usage:\n%s\n", cmdtp->usage);
        return 0;
    }
    for(i = s_address; i < e_address ; i+=4)
    {
        tempData = (*((volatile unsigned int *)i));
        if(tempData != value)
        {
            printf("Value: %x found at address: %x\n",tempData,i);
        }
    }
    return 1;
}

U_BOOT_CMD(
	cmpm,      4,     1,      cmpm_cmd,
	"cmpm	- Compare Memory\n",
	" value start_address end_address.\n"
	"\tCompare the memory from address 'start_address to address 'end_address'.\n"
	"\twith value 'value'\n"
	"\t(This command can be used only if enaMonExt is set!)\n"
);



#if 0
/******************************************************************************
* Category     - Etherent
* Functionality- Display PHY ports status (using SMI access).
* Need modifications (Yes/No) - No
*****************************************************************************/
int eth_show_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ethRegs(argv[1]);
	ethPortRegs(argv[1]);
	ethPortStatus(argv[1]);
	ethPortQueues(argv[1],0,0,1);
	return 1;
}

U_BOOT_CMD(
	ethShow,      2,    2,      eth_show_cmd,
	"ethShow	- scanning the PHYs status\n",
	" \n"
	"\tScan all the Gig port PHYs and display their Duplex, Link, Speed and AN status.\n"
);
#endif

#if defined(MV_INCLUDE_PEX)

#include "pci/mvPci.h"

int pcie_phy_read_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	MV_U16 phyReg;

    	mvPexPhyRegRead(simple_strtoul( argv[1], NULL, 16 ),
	                simple_strtoul( argv[2], NULL, 16), &phyReg);

	printf ("0x%x\n", phyReg);

	return 1;
}

U_BOOT_CMD(
	pciePhyRead,      3,     3,      pcie_phy_read_cmd,
	"phyRead	- Read PCI-E Phy register\n",
	" PCI-E_interface Phy_offset. \n"
	"\tRead the PCI-E Phy register. \n"
);


int pcie_phy_write_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	mvPexPhyRegWrite(simple_strtoul( argv[1], NULL, 16 ),
					 simple_strtoul( argv[2], NULL, 16 ),
					 simple_strtoul( argv[3], NULL, 16 ));

	return 1;
}

U_BOOT_CMD(
	pciePhyWrite,      4,     4,      pcie_phy_write_cmd,
	"pciePhyWrite	- Write PCI-E Phy register\n",
	" PCI-E_interface Phy_offset value.\n"
	"\tWrite to the PCI-E Phy register.\n"
);

#endif /* #if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH) */
#if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH)

#include "eth-phy/mvEthPhy.h"

int phy_read_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    	MV_U16 phyReg;

    	mvEthPhyRegRead(simple_strtoul( argv[1], NULL, 16 ),
	                simple_strtoul( argv[2], NULL, 16), &phyReg);

	printf ("0x%x\n", phyReg);

	return 1;
}

U_BOOT_CMD(
	phyRead,      3,     3,      phy_read_cmd,
	"phyRead	- Read Phy register\n",
	" Phy_address Phy_offset. \n"
	"\tRead the Phy register. \n"
);


int phy_write_cmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	mvEthPhyRegWrite(simple_strtoul( argv[1], NULL, 16 ),
					 simple_strtoul( argv[2], NULL, 16 ),
					 simple_strtoul( argv[3], NULL, 16 ));

	return 1;
}

U_BOOT_CMD(
	phyWrite,      4,     4,      phy_write_cmd,
	"phyWrite	- Write Phy register\n",
	" Phy_address Phy_offset value.\n"
	"\tWrite to the Phy register.\n"
);

#endif /* #if defined(MV_INCLUDE_UNM_ETH) || defined(MV_INCLUDE_GIG_ETH) */

#endif /* MV_TINY */

int _4BitSwapArry[] = {0,8,4,0xc,2,0xa,6,0xe,1,9,5,0xd,3,0xb,7,0xf};
int _3BitSwapArry[] = {0,4,2,6,1,5,3,7};

int do_satr(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	char *cmd, *s;
	MV_U8 data0=0, data1=0, devNum0=0, devNum1=0;
	MV_U8 moreThenOneDev=0, regNum = 0;
	MV_U8 mask0=0, mask1=0, shift0=0, shift1=0;
	MV_U8 val=0, width=0;

	/* at least two arguments please */
	if (argc < 2)
		goto usage;

	cmd = argv[1];

	if (strncmp(cmd, "read", 4) != 0 && strncmp(cmd, "write", 5) != 0)
		goto usage;

	/* read write */
	if (strncmp(cmd, "read", 4) == 0 || strncmp(cmd, "write", 5) == 0) {
		int read;


		read = strncmp(cmd, "read", 4) == 0; /* 1 = read, 0 = write */

		/* In write mode we have additional value */
		if (!read)
		{
			if (argc < 3)
				goto usage;
			else
				/* Value for write */
				val = (ulong)simple_strtoul(argv[2], NULL, 16);
		}

		printf("\nS@R %s: ", read ? "read" : "write");
		s = strchr(cmd, '.');
		if ((s != NULL) && (strcmp(s, ".cpu") == 0))
		{
#ifdef DB_88F6180A
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 0;
			mask0 = 0x7;
			shift0 = 0;
			mask1 = 0x0;
			shift1 = 0;
			width = 3;
#else
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 0;
			mask0 = 0xf;
			shift0 = 0;
			mask1 = 0x0;
			shift1 = 0;
			width = 4;
#endif
		}

		if ((s != NULL) && (strcmp(s, ".cpu2ddr") == 0))
		{
			moreThenOneDev = 1;
			regNum = 0;
			devNum0 = 0;
			devNum1 = 1;
			mask0 = 0x10;
			shift0 = 4;
			mask1 = 0x7;
			shift1 = 1;
			width = 4;
		}

		if ((s != NULL) && (strcmp(s, ".cpu2L2") == 0))
		{
			moreThenOneDev = 1;
			regNum = 0;
			devNum0 = 1;
			devNum1 = 2;
			mask0 = 0x18;
			shift0 = 3;
			mask1 = 0x1;
			shift1 = 2;
			width = 3;
		}

		if ((s != NULL) && (strcmp(s, ".SSCG") == 0)) 
		{
#ifdef DB_88F6180A
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 0;
			mask0 = 0x8;
			shift0 = 3;
			mask1 = 0x0;
			shift1 = 0;
#else
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 2;
			mask0 = 0x4;
			shift0 = 2;
			mask1 = 0x0;
			shift1 = 0;
#endif
		}

		if ((s != NULL) && (strcmp(s, ".PEXCLK") == 0)) 
		{
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 2;
			mask0 = 0x10;
			shift0 = 4;
			mask1 = 0x0;
			shift1 = 0;
		}

		if ((s != NULL) && ((strcmp(s, ".MPP18") == 0) || (strcmp(s, ".TCLK") == 0)) )
		{

#ifdef DB_88F6180A
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 0;
			mask0 = 0x10;
			shift0 = 4;
			mask1 = 0x0;
			shift1 = 0;
#else
			moreThenOneDev = 0;
			regNum = 0;
			devNum0 = 2;
			mask0 = 0x8;
			shift0 = 3;
			mask1 = 0x0;
			shift1 = 0;
#endif
		}
			
		if (read) {
			/* read */
			data0 = mvBoarTwsiSatRGet(devNum0, regNum);
			if (moreThenOneDev)
				data1 = mvBoarTwsiSatRGet(devNum1, regNum);

			data0 = ((data0 & mask0) >> shift0);

			if (moreThenOneDev)
			{
				data1 = ((data1 & mask1) << shift1);
				data0 |= data1;
			}
			
			/* Swap value */
			switch(width)
			{
				case 4:
					data0 = _4BitSwapArry[data0];
					break;
				case 3:
					data0 = _3BitSwapArry[data0];
					break;
				case 2:
					data0 = (((data0 & 0x1) << 0x1) | ((data0 & 0x2) >> 0x1));
					break;
			}

			printf("Read S@R val %x\n", data0);

		} else {

			/* Swap value */
			switch(width)
			{
				case 4:
					val = _4BitSwapArry[val];
					break;
				case 3:
					val = _3BitSwapArry[val];
					break;
				case 2:
					val = (((val & 0x1) << 0x1) | ((val & 0x2) >> 0x1));
					break;
			}

			/* read modify write */
			data0 = mvBoarTwsiSatRGet(devNum0, regNum);
			data0 = (data0 & ~mask0);
			data0 |= ((val << shift0) & mask0);
			if (mvBoarTwsiSatRSet(devNum0, regNum, data0) != MV_OK)
			{
				printf("Write S@R first device val %x fail\n", data0);
				return 1;
			}
			printf("Write S@R first device val %x succeded\n", data0);

			if (moreThenOneDev)
			{
				data1 = mvBoarTwsiSatRGet(devNum1, regNum);
				data1 = (data1 & ~mask1);
				data1 |= ((val >> shift1) & mask1);
				if (mvBoarTwsiSatRSet(devNum1, regNum, data1) != MV_OK)
				{
					printf("Write S@R second device val %x fail\n", data1);
					return 1;
				}
				printf("Write S@R second device val %x succeded\n", data1);
			}
		}

		return 0;
	}

usage:
	printf("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

#ifdef DB_88F6180A
U_BOOT_CMD(SatR, 5, 1, do_satr,
	"SatR - sample at reset sub-system, relevent for DB only\n",
	"SatR read.cpu 		- read cpu/L2/DDR clock from S@R devices\n"
    "SatR read.SSCG		- read SSCG state from S@R devices [0 ~ en]\n"
    "SatR read.MPP18	- reserved\n"
	"SatR write.cpu val	- write cpu/L2/DDR clock val to S@R devices [0,1,..,7]\n"
    "SatR write.SSCG val	- write SSCG state val to S@R devices [0 ~ en]\n"
    "SatR write.MPP18	- reserved\n"                       
);
#elif defined(DB_88F6192A) || defined(DB_88F6281A)
U_BOOT_CMD(SatR, 5, 1, do_satr,
	"SatR - sample at reset sub-system, relevent for DB only\n",
	"SatR read.cpu 		- read cpu clock from S@R devices\n"
	"SatR read.cpu2ddr	- read cpu2ddr clock ratio from S@R devices\n"
	"SatR read.cpu2L2	- read cpu2L2 clock ratio from S@R devices\n"
	"SatR read.SSCG		- read SSCG state from S@R devices [0 ~ en]\n"
    "SatR read.PEXCLK   - read PCI-E clock state from S@R devices [0 ~ input]\n"
#if defined(DB_88F6281A)
   "SatR read.TCLK	    - read TCLK value (0 = 200MHz, 1 = 166MHz)\n"
#else
   "SatR read.MPP18	    - reserved\n"
#endif
	"SatR write.cpu val	- write cpu clock val to S@R devices [0,1,..,F]\n"
	"SatR write.cpu2ddr val	- write cpu2ddr clock ratio val to S@R devices [0,1,..,F]\n"
	"SatR write.cpu2L2 val	- write cpu2L2 clock ratio val to S@R devices [0,1,..,7]\n"
	"SatR write.SSCG val	- write SSCG state val to S@R devices [0 ~ en]\n"
    "SatR write.PEXCLK	- write PCI-E clock state from S@R devices [0 ~ input]\n"
#if defined(DB_88F6281A)
   "SatR write.TCLK	    - write TCLK value (0 = 200MHz, 1 = 166MHz)\n"
#else
   "SatR write.MPP18	    - reserved\n"
#endif
);
#endif

#if (CONFIG_COMMANDS & CFG_CMD_RCVR)
extern void recoveryHandle(void);
int do_rcvr (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	recoveryHandle();
	return 1;
}

U_BOOT_CMD(
	rcvr,	3,	1,	do_rcvr,
	"rcvr\t- Satrt recovery process (Distress Beacon with TFTP server)\n",
	"\n"
);
#endif	/* CFG_CMD_RCVR */

#ifdef CFG_DIAG

#include "../diag/diag.h"
extern diag_func_t *diag_sequence[];

int mv_diag (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
        int test_no = 0, no_of_tests = 0; 
        diag_func_t **diag_func_ptr;


        for (diag_func_ptr = diag_sequence; *diag_func_ptr; ++diag_func_ptr)
                no_of_tests++;

        if (argc > 1) 
        {
                test_no = simple_strtoul(argv[1], NULL, 10); 
                if (test_no > no_of_tests)
                {
                        printf("There are only %d tests\n", no_of_tests);
                        printf("Usage: %s\n", cmdtp->help);
                        return 0;
                }

                test_no--;
                (*diag_sequence[test_no])();
                return 0;
        }

        for (diag_func_ptr = diag_sequence; *diag_func_ptr; ++diag_func_ptr)
        {
                printf("\n");
                if((*diag_func_ptr)())
                        break;
        }

        if(*diag_func_ptr == NULL)
                printf("\nDiag completed\n");
        else
                printf("\nDiag FAILED\n");

        return 0;
}

U_BOOT_CMD(
        mv_diag, 2, 0, mv_diag,
        "mv_diag - perform board diagnostics\n"
        "mv_diag - run all available tests\n"
        "mv_diag [1|2|...]\n"
        "        - run specified test number\n",
        "mv_diag - perform board diagnostics\n"
        "mv_diag - run all available tests\n"
        "mv_diag [1|2|...]\n"
        "        - run specified test number\n"
);
#endif /*CFG_DIAG*/

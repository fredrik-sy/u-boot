/*
 * (C) Copyright 2000-2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */
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

/*
 * IDE support
 */
#include <common.h>
#include <config.h>
#include <watchdog.h>
#include <command.h>
#include <image.h>
#include <asm/byteorder.h>
#include <ide.h>
#include <ata.h>
#include <pci.h>
#include "mvCtrlEnvLib.h"

#if (CONFIG_COMMANDS & CFG_CMD_IDE)

#include "sata/CoreDriver/mvOsS.h"
#include "sata/CoreDriver/mvSata.h"
#include "sata/CoreDriver/mvStorageDev.h"

extern unsigned int whoAmI(void);

#ifdef MV_LOGGER
char *szModules[] = {"Core Driver",
"BIOS IAL"
};
#endif

#undef	IDE_DEBUG

#ifdef	IDE_DEBUG
#define	DP(fmt,args...)	printf (fmt ,##args)
#else
#define DP(fmt,args...)
#endif

#define SHOW_BOOT_PROGRESS(arg)

#if defined(MV_INCLUDE_INTEG_SATA)
extern MV_STATUS mvSataWinInit(void);
#endif

#ifndef MRVL_SATA_BUFF_BOUNDARY
#define MRVL_SATA_BUFF_BOUNDARY (1 << 24)
#endif /* MRVL_SATA_BUFF_BOUNDARY */

#define MRVL_SATA_BOUNDARY_MASK (MRVL_SATA_BUFF_BOUNDARY - 1)
 
#if (__BE)
void swapATABuffer(unsigned short *buffer, ulong count);
#endif
/* ------------------------------------------------------------------------- */

/* Current I/O Device	*/
static int curr_device = -1;


#define	ATA_CURR_BASE(dev)	(CFG_ATA_BASE_ADDR+ide_bus_offset[IDE_BUS(dev)])
#define MV_PM 0xF

typedef struct mvSataPMDeviceInfo
{
	MV_U16      vendorId;
	MV_U16      deviceId;
	MV_U8       productRevision;
	MV_U8       PMSpecRevision:4;
	MV_U8       numberOfPorts:4;
} MV_SATA_PM_DEVICE_INFO;

typedef struct _MV_CHANNEL_INFO
{
	MV_SATA_DEVICE_TYPE	deviceType;	
	MV_U8			numberOfPorts;
	MV_U16			connected;
}MV_CHANNEL_INFO;
typedef struct _HW_ADAPTER_DESCRIPTION
{
    MV_BOOLEAN          valid;
    int		 	devno;
    MV_SATA_ADAPTER     mvSataAdapter;  /* CoreDriver Adapter data structure*/
    MV_SATA_CHANNEL	mvSataChannels[MV_SATA_CHANNELS_NUM];
    MV_CHANNEL_INFO	channelInfo[MV_SATA_PM_MAX_PORTS];
} HW_ADAPTER_DESCRIPTION, *PHW_ADAPTER_DESCRIPTION;

/* Data structure describing mvSata adapter and channels */
/* The data structure describes 4 adapters */
#define MAX_NUM_OF_ADAPTERS 4
HW_ADAPTER_DESCRIPTION sataAdapters[CFG_IDE_MAXBUS];
static  block_dev_desc_t ide_dev_desc[CFG_IDE_MAXDEVICE];
static  unsigned int ide_initiated=0,ide_detected=0;

MV_SATA_EDMA_PRD_ENTRY prd_table[2]  __attribute__ ((aligned(32)));
unsigned char request_q[MV_EDMA_REQUEST_ENTRY_SIZE] __attribute__ ((aligned(1024)));
unsigned char response_q[MV_EDMA_RESPONSE_ENTRY_SIZE] __attribute__ ((aligned(256)));

/* ------------------------------------------------------------------------- */

static MV_BOOLEAN StartPM(HW_ADAPTER_DESCRIPTION *sataAdapter, MV_U8 channelIndex);

static MV_BOOLEAN initDisk(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex,
			   MV_U8 port, block_dev_desc_t *dev_desc);

static void ident_cpy (unsigned char *dest, unsigned char *src, unsigned int len);

static ulong pio_read_write (MV_SATA_ADAPTER *pSataAdapter, 
			     unsigned int channelIndex,
			     MV_U8 port,
			     lbaint_t blknr, ulong blkcnt, ulong *buffer, int read);
static
ulong dma_read_write (MV_SATA_ADAPTER *pSataAdapter, 
		      unsigned int channelIndex,
		      MV_U8 port,
		      lbaint_t blknr, ulong blkcnt, ulong *buffer, int read);
static
ulong dma_read_write_cmd (MV_SATA_ADAPTER *pSataAdapter, 
			  unsigned int channelIndex,
			  MV_U8 port,
			  lbaint_t blknr, ulong blkcnt, ulong *buffer, int read);
/* ------------------------------------------------------------------------- */

#ifdef CONFIG_PCI

static struct pci_device_id supported[] = {
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_5080},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_5081},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_5040},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_5041},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_6041},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_6081},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_6042},
        {MV_SATA_VENDOR_ID, MV_SATA_DEVICE_ID_7042},
};

#endif /* CONFIG_PCI */

		
int g_ide_reset_ok;
// jcho ODD-boot
extern int LgDiag_IoRead(u8 bId, u8* data);     
extern int LgDiag_Msg2Lcd(char* msg, int bSetMode);

#if (CONFIG_COMMANDS & CFG_CMD_IDE)
int do_ide (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int rcode = 0;

	/* Check if we have already called reset or not */
	if (ide_initiated == 0)
	{
		/* If this is not a call to reset !!!! */
		if (!((argc==2)&&(strncmp(argv[1],"res",3) == 0)))
		{
			puts ("\nWarning: Please run 'ide reset' before running other ide commands \n\n");
			return 1;
		}
	}
	else /* We have already called reset */
	{
		if (ide_detected == 0)
		{
			puts ("\nno IDE devices available\n");
			return 1;
		}
	}

    switch (argc) {
    case 0:
    case 1:
	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    case 2:
	if (strncmp(argv[1],"res",3) == 0) {
		puts ("\nReset IDE"
#ifdef CONFIG_IDE_8xx_DIRECT
			" on PCMCIA " PCMCIA_SLOT_MSG
#endif
			": ");

		g_ide_reset_ok = 1;

		ide_init ();

		if( g_ide_reset_ok == 0 ) {
			hdd_power_restart();
			ide_init ();
		}
                //udelay(3000000);
		return 0;
	} else if (strncmp(argv[1],"inf",3) == 0) {
		int i;

		putc ('\n');

		for (i=0; i<CFG_IDE_MAXDEVICE; ++i) {
			if (ide_dev_desc[i].type==DEV_TYPE_UNKNOWN)
				continue; /* list only known devices */
			printf ("IDE device %x: ", i);
			dev_print(&ide_dev_desc[i]);
		}
		return 0;

	} else if (strncmp(argv[1],"dev",3) == 0) {
		if ((curr_device < 0) || (curr_device >= CFG_IDE_MAXDEVICE)) {
			puts ("\nno IDE devices available\n");
			return 1;
		}
		printf ("\nIDE device %x: ", curr_device);
		dev_print(&ide_dev_desc[curr_device]);
		return 0;
	} else if (strncmp(argv[1],"part",4) == 0) {
		int dev, ok;

		for (ok=0, dev=0; dev<CFG_IDE_MAXDEVICE; ++dev) {
			if (ide_dev_desc[dev].part_type!=PART_TYPE_UNKNOWN) {
				++ok;
				if (dev)
					putc ('\n');
				print_part(&ide_dev_desc[dev]);
			}
		}
		if (!ok) {
			puts ("\nno IDE devices available\n");
			rcode ++;
		}
		return rcode;
	} else if (strncmp(argv[1],"load",4) == 0) {    // jcho ODD-boot

          printf ("\nBoot from Optical Disc\n");
          ide_load(0xffffffff, 0xffffffff, 0);

                return 0;
        }
	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    case 3:
	if (strncmp(argv[1],"dev",3) == 0) {
		int dev = (int)simple_strtoul(argv[2], NULL, 10);

		printf ("\nIDE device %x: ", dev);
		if (dev >= CFG_IDE_MAXDEVICE) {
			puts ("unknown device\n");
			return 1;
		}
		dev_print(&ide_dev_desc[dev]);
		/*ide_print (dev);*/

		if (ide_dev_desc[dev].type == DEV_TYPE_UNKNOWN) {
			return 1;
		}

		curr_device = dev;

		puts ("... is now current device\n");

		return 0;
	} else if (strncmp(argv[1],"part",4) == 0) {
		int dev = (int)simple_strtoul(argv[2], NULL, 10);

		if (ide_dev_desc[dev].part_type!=PART_TYPE_UNKNOWN) {
				print_part(&ide_dev_desc[dev]);
		} else {
			printf ("\nIDE device %x not available\n", dev);
			rcode = 1;
		}
		return rcode;
	} else if (strncmp(argv[1],"test",4) == 0) {    // pch_080709
            unsigned char io_data[8], bId;
            int ret_val;

            printf ("\nTest Command [%s]\n", argv[2]);

            if(argv[2][0] == '1')
            {
                bId = 0x09;

                LgDiag_IoRead(bId, io_data);
                printf("IO_DATA: %02x%02x%02x%02x %02x%02x%02x%02x\n",
                        io_data[0], io_data[1], io_data[2], io_data[3],
                        io_data[4], io_data[5], io_data[6], io_data[7]);
            }
            else if(argv[2][0] == '2')
            {
                if(ide_disc_ready(0xffffffff, 0xffffffff))
                        printf("uImage Disc is Ready\n");
                else
                        printf("uImage Disc is not Ready\n");
            }
            else if(argv[2][0] == '3')
            {
                //ret_val = LgDiag_IoWrite(0x86,0,4,0,0,0,0 ); udelay(100000);
                //printf("Ret_val_1:%d\n", ret_val);
                ret_val = LgDiag_Msg2Lcd("Aging start...", 0); udelay(100000);
                printf("Ret_val_2:%d\n", ret_val);
            }

                return 0;
            }

	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
    default:
	/* at least 4 args */

	if (strcmp(argv[1],"read") == 0) {
		ulong addr = simple_strtoul(argv[2], NULL, 16);
		ulong cnt  = simple_strtoul(argv[4], NULL, 16);
		ulong n;
		
		if (curr_device == -1)
		{
			printf ("\n\nIDE device not available\n");
			return 1;
		}
#ifdef CFG_64BIT_STRTOUL
		lbaint_t blk  = simple_strtoull(argv[3], NULL, 16);

		printf ("\nIDE read: device %x block # %qd, count %ld ... ",
			curr_device, blk, cnt);
#else
		lbaint_t blk  = simple_strtoul(argv[3], NULL, 16);

		printf ("\nIDE read: device %x block # %ld, count %ld ... ",
			curr_device, blk, cnt);
#endif

		n = ide_dev_desc[curr_device].block_read (curr_device,
							  blk, cnt,
							  (ulong *)addr);
		/* flush cache after read */
		flush_cache (addr, cnt*ide_dev_desc[curr_device].blksz);

		printf ("%ld blocks read: %s\n",
			n, (n==cnt) ? "OK" : "ERROR");
		if (n==cnt) {
			return 0;
		} else {
			return 1;
		}
	} else if (strcmp(argv[1],"write") == 0) {
		ulong addr = simple_strtoul(argv[2], NULL, 16);
		ulong cnt  = simple_strtoul(argv[4], NULL, 16);
		ulong n;

		if (curr_device == -1)
		{
			printf ("\n\nIDE device not available\n");
			return 1;
		}
#ifdef CFG_64BIT_STRTOUL
		lbaint_t blk  = simple_strtoull(argv[3], NULL, 16);

		printf ("\nIDE write: device %x block # %qd, count %ld ... ",
			curr_device, blk, cnt);
#else
		lbaint_t blk  = simple_strtoul(argv[3], NULL, 16);

		printf ("\nIDE write: device %x block # %ld, count %ld ... ",
			curr_device, blk, cnt);
#endif

		n = ide_write (curr_device, blk, cnt, (ulong *)addr);

		printf ("%ld blocks written: %s\n",
			n, (n==cnt) ? "OK" : "ERROR");
		if (n==cnt) {
			return 0;
		} else {
			return 1;
		}
	} else if (strncmp(argv[1],"load",4) == 0) {    // jcho ODD-boot

            ulong addr = simple_strtoul(argv[2], NULL, 16);
            ulong length  = simple_strtoul(argv[3], NULL, 16);
            ulong target  = simple_strtoul(argv[4], NULL, 16);
            printf ("\nBoot from Optical Disc, Addr:%08x, Size:%08x, Target:%08x\n", addr, length, target);
            ide_load(addr, length, target);

            return 0;
        } else {
		printf ("Usage:\n%s\n", cmdtp->usage);
		rcode = 1;
	}

	return rcode;
    }
}
#endif

int do_diskboot (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *boot_device = NULL;
	char *ep;
	int dev, part = 0;
	ulong cnt;
	ulong addr;
	disk_partition_t info;
	image_header_t *hdr;
	int rcode = 0;

	switch (argc) {
	case 1:
		addr = CFG_LOAD_ADDR;
		boot_device = getenv ("bootdevice");
		break;
	case 2:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = getenv ("bootdevice");
		break;
	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = argv[2];
		break;
	default:
		printf ("Usage:\n%s\n", cmdtp->usage);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (!boot_device) {
		puts ("\n** No boot device **\n");
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	dev = simple_strtoul(boot_device, &ep, 16);

	if (ide_dev_desc[dev].type==DEV_TYPE_UNKNOWN) {
		printf ("\n** Device %x not available\n", dev);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (*ep) {
		if (*ep != ':') {
			puts ("\n** Invalid boot device, use `dev[:part]' **\n");
			SHOW_BOOT_PROGRESS (-1);
			return 1;
		}
		part = simple_strtoul(++ep, NULL, 16);
	}
	if (get_partition_info (&ide_dev_desc[dev], part, &info)) {
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}
	if ((strncmp((const char *)info.type, BOOT_PART_TYPE, sizeof(info.type)) != 0) &&
	    (strncmp((const char *)info.type, BOOT_PART_COMP, sizeof(info.type)) != 0)) {
		printf ("\n** Invalid partition type \"%.32s\""
			" (expect \"" BOOT_PART_TYPE "\")\n",
			info.type);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	printf ("\nLoading from IDE device %x, partition %d: "
		"Name: %.32s  Type: %.32s\n",
		dev, part, info.name, info.type);

	DP ("First Block: %ld,  # of blocks: %ld, Block Size: %ld\n",
		info.start, info.size, info.blksz);

	if (ide_dev_desc[dev].block_read (dev, info.start, 1, (ulong *)addr) != 1) {
		printf ("** Read error on %x:%d\n", dev, part);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	hdr = (image_header_t *)addr;

	if (ntohl(hdr->ih_magic) == IH_MAGIC) {

		print_image_hdr (hdr);

		cnt = (ntohl(hdr->ih_size) + sizeof(image_header_t));
		cnt += info.blksz - 1;
		cnt /= info.blksz;
		cnt -= 1;
	} else {
		printf("\n** Bad Magic Number **\n");
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}

	if (ide_dev_desc[dev].block_read (dev, info.start+1, cnt,
		      (ulong *)(addr+info.blksz)) != cnt) {
		printf ("** Read error on %x:%d\n", dev, part);
		SHOW_BOOT_PROGRESS (-1);
		return 1;
	}


	/* Loading ok, update default load address */

	load_addr = addr;

	/* Check if we should attempt an auto-start */
	if (((ep = getenv("autostart")) != NULL) && (strcmp(ep,"yes") == 0)) {
		char *local_args[2];
		extern int do_bootm (cmd_tbl_t *, int, int, char *[]);

		local_args[0] = argv[0];
		local_args[1] = NULL;

		printf ("Automatic boot of image at addr 0x%08lX ...\n", addr);

		do_bootm (cmdtp, 0, 1, local_args);
		rcode = 1;
	}
	return rcode;
}

/* ------------------------------------------------------------------------- */
MV_SATA_DEVICE_TYPE mvGetSataDeviceType(
					MV_STORAGE_DEVICE_REGISTERS *mvStorageDevRegisters)
{
	if (((mvStorageDevRegisters->sectorCountRegister & 0xff) != 1) ||
	    ((mvStorageDevRegisters->lbaLowRegister & 0xff) != 1))
		{
			return MV_SATA_DEVICE_TYPE_UNKNOWN;
		}
	if (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0) &&
	    ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0))
		{
			return MV_SATA_DEVICE_TYPE_ATA_DISK;
		}
	if ((((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x14) &&
	     ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0xEB))/* ||
         (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x69) &&
         ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0x96))*/)
		{
			return MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;
		}
	if (((mvStorageDevRegisters->lbaMidRegister & 0xff) == 0x69) &&
	    ((mvStorageDevRegisters->lbaHighRegister & 0xff) == 0x96))
		{
			return MV_SATA_DEVICE_TYPE_PM;
		}
	return MV_SATA_DEVICE_TYPE_UNKNOWN;
}	

MV_BOOLEAN  mvGetPMDeviceInfo(MV_SATA_ADAPTER   *pSataAdapter,
                              MV_U8 channelIndex,
                              MV_SATA_PM_DEVICE_INFO *pPMDeviceInfo)
{	
	MV_U32  regVal;

	if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
			   MV_SATA_GSCR_ID_REG_NUM, &regVal, NULL) == MV_FALSE)
		{	
			printf("Error [%d %d]: Failed to read PM GSCR ID register\n",
			       pSataAdapter->adapterId, channelIndex);
			return MV_FALSE;
		}
	pPMDeviceInfo->vendorId = (MV_U16)(regVal & 0xffff);
	pPMDeviceInfo->deviceId = (MV_U16)((regVal & 0xffff0000) >> 16);

	if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
			   MV_SATA_GSCR_REVISION_REG_NUM, &regVal, NULL)== MV_FALSE)
		{
			printf("Error:[%d %d]: Failed to read PM GSCR REVISION register\n", pSataAdapter->adapterId,
			       channelIndex);
			return MV_FALSE;
		}
	pPMDeviceInfo->PMSpecRevision = (MV_U8)(regVal & 0xff);
	pPMDeviceInfo->productRevision = (MV_U8)((regVal & 0xff00) >> 8);

	if (mvPMDevReadReg(pSataAdapter, channelIndex, MV_SATA_PM_CONTROL_PORT,
			   MV_SATA_GSCR_INFO_REG_NUM, &regVal, NULL)== MV_FALSE)
		{
			printf("Error: [%d %d]: Failed to read PM GSCR INFO\n", pSataAdapter->adapterId,
				 channelIndex);
			return MV_FALSE;
		}
	pPMDeviceInfo->numberOfPorts = (MV_U8)(regVal & 0xf);
	return MV_TRUE;
}

static MV_BOOLEAN
StartPM(HW_ADAPTER_DESCRIPTION *sataAdapter,
	MV_U8 channelIndex)
{
	MV_SATA_ADAPTER *pSataAdapter = &sataAdapter->mvSataAdapter;
	MV_U8               PMPort;
	MV_SATA_PM_DEVICE_INFO PMInfo;
	
	if (mvGetPMDeviceInfo(pSataAdapter, channelIndex, &PMInfo) == MV_FALSE)	{
		printf("Error: Failed to get PortMultiplier Info\n",
		       pSataAdapter->adapterId, channelIndex);
		return MV_FALSE;
	}
	printf("Port Multiplier found @ %d %d. Vendor: %04x ports: %d\n",
	       pSataAdapter->adapterId, channelIndex, PMInfo.vendorId, 
	       PMInfo.numberOfPorts);
	
	for (PMPort = 0; PMPort < PMInfo.numberOfPorts; PMPort++){
		MV_U32  SStatus;
		/* 
		 * Skip PMPort #0 - this should be already enabled due to legacy mode
		 * transition of PM
		 */
		if (PMPort >= 0)	{
			if (mvPMDevEnableStaggeredSpinUp(pSataAdapter, channelIndex, 
							 PMPort) == MV_FALSE){
				printf("Error:[%d %d %d]: EnableStaggeredSpinUp Failed\n",
				       pSataAdapter->adapterId, channelIndex, PMPort);
				if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
								   MV_SATA_PM_CONTROL_PORT, NULL) == MV_FALSE)
					{
						printf("Error: [%d %d]: failed to Soft Reset PM control port\n"
						       , pSataAdapter->adapterId, channelIndex);
					}
				continue;
			}
		}
		if (mvPMDevReadReg(pSataAdapter, channelIndex, PMPort,
				   MV_SATA_PSCR_SSTATUS_REG_NUM, &SStatus, NULL) ==
		    MV_FALSE)
			{
				printf("Error:[%d %d %d]: mvPMDevReadReg Failed\n",
				       pSataAdapter->adapterId, channelIndex, PMPort);
				if (mvStorageDevATASoftResetDevice(pSataAdapter, channelIndex,
								   MV_SATA_PM_CONTROL_PORT, NULL) == MV_FALSE)
					{
						printf("Error: [%d %d]: failed to Soft Reset PM control port\n"
						       , pSataAdapter->adapterId, channelIndex);
					}
				continue;
			}
		DP("[%d %d %x]: S-Status: 0x%x\n", pSataAdapter->adapterId,
				   channelIndex, PMPort, SStatus);
		if ((SStatus & 0xf) == 3){
            MV_STORAGE_DEVICE_REGISTERS ATARegs;
            if (mvPMDevWriteReg(pSataAdapter, channelIndex, PMPort,
                 MV_SATA_PSCR_SERROR_REG_NUM, 0xffffffff, NULL) == MV_FALSE)
            {
                printf("Error [%d %d %d]: PM Write SERROR Failed\n",
                 pSataAdapter->adapterId, channelIndex, PMPort);
                continue;
    		}
		     
            if (mvStorageDevATASoftResetDevice(pSataAdapter,
                        channelIndex, PMPort, &ATARegs) == MV_FALSE)
            {
                printf("Error - SRST for port %d on channel %d failed\n", PMPort, channelIndex);
				g_ide_reset_ok = 0;
                return MV_FALSE;
            }
            if(mvGetSataDeviceType(&ATARegs) == MV_SATA_DEVICE_TYPE_ATA_DISK)
            {
                sataAdapter->channelInfo[channelIndex].connected |= 1 << PMPort;
            }		     
		}
	}
	sataAdapter->channelInfo[channelIndex].numberOfPorts = PMInfo.numberOfPorts;
	return MV_TRUE;
}

#if 0 // jcho ODD-boot
static MV_BOOLEAN StartChannel(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex, block_dev_desc_t *dev_desc)
{
    MV_STORAGE_DEVICE_REGISTERS inATARegs;
    MV_SATA_CHANNEL     *pSataChannel;
    ulong identifyBuffer[ATA_SECTORWORDS];
    hd_driveid_t *iop = (hd_driveid_t *)identifyBuffer;
    MV_U16_PTR      iden = (MV_U16_PTR) identifyBuffer;
    MV_U8 PIOMode;
    MV_BUS_ADDR_T   ioBaseAddr;
    MV_U32          eDmaRegsOffset;

    pSataChannel = pSataAdapter->sataChannel[channelIndex];
    if (!pSataChannel)
    {
        printf ("Error in StartChannel - Channel data structure is null\n");
        return MV_FALSE;
    }
    if (mvStorageDevATASoftResetDevice(pSataAdapter,
                                      channelIndex, MV_PM, 0) == MV_FALSE)
    {
        printf("Error - Failed initializing(SRST) drive on channel %d\n", channelIndex);
        return MV_FALSE;
    }
        /* identify device*/
    memset(&inATARegs, 0, sizeof(inATARegs));
    inATARegs.commandRegister = MV_ATA_COMMAND_IDENTIFY;

        // jcho ODD-boot
    if(channelIndex == 1)
    {
        ioBaseAddr = pSataAdapter->adapterIoBaseAddress;
        eDmaRegsOffset = pSataAdapter->sataChannel[4]->eDmaRegsOffset;
        printf("SEC_CNT:%02x, SEC_NUM:%02x, CYL_LOW:%02x, CYL_HIGH:%02x\n",
        MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset + MV_ATA_DEVICE_SECTOR_COUNT_REG_OFFSET),
        MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset + MV_ATA_DEVICE_LBA_LOW_REG_OFFSET),    /* SECTOR NUM */
        MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset + MV_ATA_DEVICE_LBA_MID_REG_OFFSET),    /* CYL LOW */
        MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset + MV_ATA_DEVICE_LBA_HIGH_REG_OFFSET));   /* CYL HIGH */

        inATARegs.commandRegister = MV_ATA_COMMAND_ATAPI_IDENTIFY;
        pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;
        if(mvStorageDevATAPIIdentifyDevice(pSataAdapter, channelIndex, 0,
            (unsigned short *)identifyBuffer) == MV_FALSE)
        {
            printf("ATAPI Indentify Command Failed\n");
            return MV_FALSE;
        }
        printf("ATAPI Indentify Command Done\n");
    }
    if (mvStorageDevATAIdentifyDevice(pSataAdapter, channelIndex, 0,
                                     (unsigned short *)identifyBuffer) == MV_FALSE)
    {
        printf("[%d %d]: failed to perform ATA Identify command\n", pSataAdapter->adapterId,
                channelIndex);
        return MV_FALSE;
    }
        /* Check if read look ahead is supported. If so enable it */
    if (iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT6)
    {
        if (mvStorageDevATASetFeatures(pSataAdapter, channelIndex, 0,
                                        MV_ATA_SET_FEATURES_ENABLE_RLA,0,0,0, 0) == MV_FALSE)
        {
            printf("[%d %d]: failed to perform Enable RLA command\n", pSataAdapter->adapterId,
                     channelIndex);
            return MV_FALSE;
        }
    }
    if ((iden[IDEN_VALID] & MV_BIT1) == 0)
    {
        printf("[%d %d]: Unable to find PIO Mode\n", pSataAdapter->adapterId, channelIndex);
        return MV_FALSE;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT0)
    {
        PIOMode = MV_ATA_TRANSFER_PIO_3;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT1)
    {
        PIOMode = MV_ATA_TRANSFER_PIO_4;
    }
    else
    {
        printf("[%d %d]: PIO modes 3 and 4 are not supported\n", pSataAdapter->adapterId, channelIndex);
        PIOMode = MV_ATA_TRANSFER_PIO_SLOW;
    }
    if (mvStorageDevATASetFeatures(pSataAdapter, channelIndex, 0,
                                MV_ATA_SET_FEATURES_TRANSFER,PIOMode,0,0, 0) == MV_FALSE)
    {
        printf("[%d %d]: failed to perform Enable RLA command\n", pSataAdapter->adapterId,
             channelIndex);
        return MV_FALSE;
    }

    /* Continue parsing identify buffer*/
    int device;
    device=dev_desc->dev;
    printf ("  Device %d: ", device);

    ident_cpy (dev_desc->revision, iop->fw_rev, sizeof(dev_desc->revision));
    ident_cpy (dev_desc->vendor, iop->model, sizeof(dev_desc->vendor));
    ident_cpy (dev_desc->product, iop->serial_no, sizeof(dev_desc->product));

    if ((iop->config & 0x0080)==0x0080)
        dev_desc->removable = 1;
    else
        dev_desc->removable = 0;

#ifdef __BIG_ENDIAN
    /* swap shorts */
    dev_desc->lba = (iop->lba_capacity << 16) | (iop->lba_capacity >> 16);
#else   /* ! __BIG_ENDIAN */
    dev_desc->lba = iop->lba_capacity;
#endif  /* __BIG_ENDIAN */

#ifdef CONFIG_LBA48
    if (iop->command_set_2 & 0x0400) { /* LBA 48 support */
        dev_desc->lba48 = 1;
        dev_desc->lba = (unsigned long long)iop->lba48_capacity[0] |
                                      ((unsigned long long)iop->lba48_capacity[1] << 16) |
                                      ((unsigned long long)iop->lba48_capacity[2] << 32) |
                                      ((unsigned long long)iop->lba48_capacity[3] << 48);
    } else {
        dev_desc->lba48 = 0;
    }
#endif /* CONFIG_LBA48 */

        /* assuming HD */
    dev_desc->type=DEV_TYPE_HARDDISK;
    dev_desc->blksz=ATA_BLOCKSIZE;
    dev_desc->lun=0; /* just to fill something in... */

    // jcho ODD-boot
    if(channelIndex == 4)
    {
        dev_desc->type = DEV_TYPE_CDROM;
        dev_desc->blksz = 2048;
        dev_desc->if_type = IF_TYPE_ATAPI;
    }

    return MV_TRUE;
}

#else // jcho ODD-boot
static MV_BOOLEAN StartChannel(HW_ADAPTER_DESCRIPTION *sataAdapter,
			       MV_U8 channelIndex, block_dev_desc_t *dev_desc)
{
	MV_SATA_ADAPTER *pSataAdapter = &sataAdapter->mvSataAdapter;
    	MV_STORAGE_DEVICE_REGISTERS ATARegs;
    	MV_SATA_CHANNEL     *pSataChannel;
        ulong identifyBuffer[ATA_SECTORWORDS];
	MV_SATA_DEVICE_TYPE deviceType;
        MV_BUS_ADDR_T   ioBaseAddr;
        MV_U32          eDmaRegsOffset;

    	pSataChannel = pSataAdapter->sataChannel[channelIndex];
    	if (!pSataChannel)
    	{
        	printf ("Error in StartChannel - Channel data structure is null\n");
        	return MV_FALSE;
    	}
    	if (mvStorageDevATASoftResetDevice(pSataAdapter,
					   channelIndex, MV_PM, &ATARegs) == MV_FALSE)
		{
        	printf("Error - Failed initializing(SRST) drive on channel %d\n", channelIndex);
        	return MV_FALSE;
    	}
	deviceType = mvGetSataDeviceType(&ATARegs);
	mvStorageDevSetDeviceType(pSataAdapter, channelIndex,deviceType);
	sataAdapter->channelInfo[channelIndex].deviceType = deviceType;
	sataAdapter->channelInfo[channelIndex].numberOfPorts = 0;
	sataAdapter->channelInfo[channelIndex].connected = 0;
	switch(deviceType)
	{	
	case MV_SATA_DEVICE_TYPE_UNKNOWN:
		printf("Error - unknown device at channel %d\n", channelIndex);	
		return MV_FALSE;  
	case MV_SATA_DEVICE_TYPE_ATAPI_DEVICE:
                ioBaseAddr = pSataAdapter->adapterIoBaseAddress;
                eDmaRegsOffset = pSataAdapter->sataChannel[1]->eDmaRegsOffset;
                printf("eDmaRegsOffset1 = %x, eDmaRegsOffset4 = %x,\n SEC_CNT:%02x, SEC_NUM:%02x, CYL_LOW:%02x, CYL_HIGH:%02x\n",
                pSataAdapter->sataChannel[1]->eDmaRegsOffset, 
                pSataAdapter->sataChannel[4]->eDmaRegsOffset,
                MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset+MV_ATA_DEVICE_SECTOR_COUNT_REG_OFFSET),
                MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset+MV_ATA_DEVICE_LBA_LOW_REG_OFFSET),    /* SECTOR NUM */
                MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset+MV_ATA_DEVICE_LBA_MID_REG_OFFSET),    /* CYL LOW */
                MV_REG_READ_BYTE(ioBaseAddr, eDmaRegsOffset+MV_ATA_DEVICE_LBA_HIGH_REG_OFFSET));   /* CYL HIGH */

                ATARegs.commandRegister = MV_ATA_COMMAND_ATAPI_IDENTIFY;
                pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;
                if(mvStorageDevATAPIIdentifyDevice(pSataAdapter, channelIndex, 0,
                    (unsigned short *)identifyBuffer) == MV_FALSE)
                {
                    printf("ATAPI Indentify Command Failed\n");
                    return MV_FALSE;
                }
                printf("ATAPI Indentify Command Done\n");
		return MV_TRUE;  
	case MV_SATA_DEVICE_TYPE_ATA_DISK:
		printf("This is ATA Disk at channel %d\n", channelIndex);	
		sataAdapter->channelInfo[channelIndex].numberOfPorts = 1;
		sataAdapter->channelInfo[channelIndex].connected = 1;
		return MV_TRUE;
	case MV_SATA_DEVICE_TYPE_PM:
		return StartPM(sataAdapter, channelIndex);
	}
	
	return MV_TRUE;
}
#endif

static MV_BOOLEAN initDisk(MV_SATA_ADAPTER *pSataAdapter, MV_U8 channelIndex,
			   MV_U8 port, block_dev_desc_t *dev_desc)
{
    	MV_STORAGE_DEVICE_REGISTERS ATARegs;
  	ulong identifyBuffer[ATA_SECTORWORDS];
    	hd_driveid_t *iop = (hd_driveid_t *)identifyBuffer;
	MV_U16_PTR	iden = (MV_U16_PTR) identifyBuffer;
	MV_U8 PIOMode;
	MV_U8 UDMAMode;
	MV_SATA_DEVICE_TYPE deviceType = mvStorageDevGetDeviceType(pSataAdapter,channelIndex);
	if (deviceType == MV_SATA_DEVICE_TYPE_PM)
	{
#if 0
		if (mvStorageDevATASoftResetDevice(pSataAdapter,
						   channelIndex, port, &ATARegs) == MV_FALSE)
		{
			printf("Error - SRST for port %d on channel %d failed\n", port, channelIndex);
			return MV_FALSE;
		}
		if(mvGetSataDeviceType(&ATARegs) != MV_SATA_DEVICE_TYPE_ATA_DISK)
		{
			return MV_FALSE;
		}
#endif
	}
	
	/* identify device*/
    memset(&ATARegs, 0, sizeof(ATARegs));
    ATARegs.commandRegister = MV_ATA_COMMAND_IDENTIFY;
    if (mvStorageDevATAIdentifyDevice(pSataAdapter, channelIndex, port,
                                  (unsigned short *)identifyBuffer) == MV_FALSE)
    {
        printf("[%d %d %d]: failed to perform ATA Identify command\n", pSataAdapter->adapterId,
           channelIndex, port);
        return MV_FALSE;
    }
	/* Check if read look ahead is supported. If so enable it */
    if (iden[IDEN_SUPPORTED_COMMANDS1] & MV_BIT6)
    {
        if (mvStorageDevATASetFeatures(pSataAdapter, channelIndex, port,
			MV_ATA_SET_FEATURES_ENABLE_RLA,0,0,0, 0) == MV_FALSE)
        {
            printf("[%d %d %d]: failed to perform Enable RLA command\n", pSataAdapter->adapterId,
		       channelIndex, port);
	    return MV_FALSE;
	}
    }
    if ((iden[IDEN_VALID] & MV_BIT1) == 0)
    {
        printf("[%d %d %d]: Unable to find PIO Mode\n", pSataAdapter->adapterId, channelIndex, port);
        return MV_FALSE;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT0)
    {
	PIOMode = MV_ATA_TRANSFER_PIO_3;
    }
    else if (iden[IDEN_PIO_MODE_SPPORTED] & MV_BIT1)
    {
	PIOMode = MV_ATA_TRANSFER_PIO_4;
    }
    else
    {
	printf("[%d %d %d]: PIO modes 3 and 4 are not supported\n", pSataAdapter->adapterId, channelIndex, port);
	PIOMode = MV_ATA_TRANSFER_PIO_SLOW;
    }
    if (mvStorageDevATASetFeatures(pSataAdapter, channelIndex, port,
                                    MV_ATA_SET_FEATURES_TRANSFER,PIOMode,0,0, 0) == MV_FALSE)
    {
        printf("[%d %d %d]: failed to enable PIO mode\n",
        pSataAdapter->adapterId, channelIndex, port);
        return MV_FALSE;
    }

        if (iden[IDEN_UDMA_MODE] & MV_BIT6)
        {
            UDMAMode = MV_ATA_TRANSFER_UDMA_6;
        }
        else if (iden[IDEN_UDMA_MODE] & MV_BIT5)
        {
            UDMAMode = MV_ATA_TRANSFER_UDMA_5;
        }
        else if (iden[IDEN_UDMA_MODE] & MV_BIT4)
        {
            UDMAMode = MV_ATA_TRANSFER_UDMA_4;
        }
        else
        {
            UDMAMode = MV_ATA_TRANSFER_UDMA_4;
        }

        if (mvStorageDevATASetFeatures(pSataAdapter, channelIndex, port,
				       MV_ATA_SET_FEATURES_TRANSFER,
				       UDMAMode,
				       0,0, 0) == MV_FALSE)
        {
                printf("[%d %d %d]: failed to perform Enable DMA mode\n",
		       pSataAdapter->adapterId, channelIndex, port);
                return MV_FALSE;
        }
	printf("[%d %d %d]: Enable DMA mode (%d)\n",		       pSataAdapter->adapterId, channelIndex, port, UDMAMode - MV_ATA_TRANSFER_UDMA_0);

    	/* Continue parsing identify buffer*/
	int device;
	device=dev_desc->dev;
	printf ("  Device %d @ %d %d", device, pSataAdapter->adapterId, channelIndex);
	if(deviceType == MV_SATA_DEVICE_TYPE_PM)
		printf(" %d:\n", port);
	else
		printf(":\n");

	ident_cpy (dev_desc->revision, iop->fw_rev, sizeof(dev_desc->revision));
	ident_cpy (dev_desc->vendor, iop->model, sizeof(dev_desc->vendor));
	ident_cpy (dev_desc->product, iop->serial_no, sizeof(dev_desc->product));

	if ((iop->config & 0x0080)==0x0080)
		dev_desc->removable = 1;
	else
		dev_desc->removable = 0;

#ifdef __BIG_ENDIAN
        /* swap shorts */
        dev_desc->lba = (iop->lba_capacity << 16) | (iop->lba_capacity >> 16);
#else   /* ! __BIG_ENDIAN */
        dev_desc->lba = iop->lba_capacity;
#endif  /* __BIG_ENDIAN */

#ifdef CONFIG_LBA48
        if (iop->command_set_2 & 0x0400) { /* LBA 48 support */
                dev_desc->lba48 = 1;
                dev_desc->lba = (unsigned long long)iop->lba48_capacity[0] |
                                  ((unsigned long long)iop->lba48_capacity[1] << 16) |
                                  ((unsigned long long)iop->lba48_capacity[2] << 32) |
                                  ((unsigned long long)iop->lba48_capacity[3] << 48);
        } else {
                dev_desc->lba48 = 0;
        }
#endif /* CONFIG_LBA48 */

	/* assuming HD */
	dev_desc->type=DEV_TYPE_HARDDISK;
	dev_desc->blksz=ATA_BLOCKSIZE;
	dev_desc->lun=0; /* just to fill something in... */

        if(channelIndex == 1)
        {
            dev_desc->type = DEV_TYPE_CDROM;
            dev_desc->blksz = 2048;
            dev_desc->if_type = IF_TYPE_ATAPI;
        }

    	return MV_TRUE;
}

void
InitChannel(
           PHW_ADAPTER_DESCRIPTION HwDeviceExtension,
           MV_U8 channelIndex)
{
    	MV_SATA_ADAPTER     *pSataAdapter = &(HwDeviceExtension->mvSataAdapter);
    	MV_SATA_CHANNEL     *pSataChannel;

   	pSataChannel = &HwDeviceExtension->mvSataChannels[channelIndex];
    	pSataAdapter->sataChannel[channelIndex] = pSataChannel;
    	pSataChannel->channelNumber = channelIndex;
    	pSataChannel->requestQueue = (struct mvDmaRequestQueueEntry  *)request_q;
    	pSataChannel->requestQueuePciLowAddress = (MV_U32)request_q;
    	pSataChannel->requestQueuePciHiAddress = 0;
    	pSataChannel->responseQueue = (struct mvDmaResponseQueueEntry *) response_q;
    	pSataChannel->responseQueuePciLowAddress = (MV_U32)response_q;
    	pSataChannel->responseQueuePciHiAddress = 0;
}

MV_BOOLEAN
mvSataEventNotify(MV_SATA_ADAPTER *pSataAdapter, MV_EVENT_TYPE eventType,
                  MV_U32 param1, MV_U32 param2)
{

    switch (eventType)
    {
    case MV_EVENT_TYPE_SATA_CABLE:
	    switch(param1)
	    {
	    case MV_SATA_CABLE_EVENT_CONNECT:
		    printf("[%d,%d]: device connected event received\n",
			   pSataAdapter->adapterId, param2);
		    break;
	    case MV_SATA_CABLE_EVENT_DISCONNECT:
		    printf("[%d,%d]: device disconnected event received \n",
			   pSataAdapter->adapterId, param2);
		    break;
	    case MV_SATA_CABLE_EVENT_PM_HOT_PLUG:
		    printf("[%d,%d]: Port Multiplier hotplug event received \n",
			   pSataAdapter->adapterId, param2);
		    break;
	    default:
		    printf( "illegal value for param1(%d) at "
			    "connect/disconect event, host=%d\n", param1,
			    pSataAdapter->adapterId );
	    }
	    break;
    case MV_EVENT_TYPE_ADAPTER_ERROR:
	    printf("DEVICE error event received, pci cause "
		   "reg=%x, don't know how to handle this\n", param1);
	    return MV_TRUE;
    case MV_EVENT_TYPE_SATA_ERROR:
	    switch (param1)
		{
		case MV_SATA_RECOVERABLE_COMMUNICATION_ERROR:
			printf(" [%d %d] sata recoverable error occured\n",
			       pSataAdapter->adapterId, param2);
			break;
		case MV_SATA_UNRECOVERABLE_COMMUNICATION_ERROR:
			printf(" [%d %d] sata unrecoverable error occured, restart channel\n",
			       pSataAdapter->adapterId, param2);
			break;
		case MV_SATA_DEVICE_ERROR:
			printf( " [%d %d] device error occured\n",
				pSataAdapter->adapterId, param2);
			break;
		}
	    break;
    default:
	    printf(" adapter %d unknown event %d"
		   " param1= %x param2 = %x\n", pSataAdapter->adapterId,
		   eventType - MV_EVENT_TYPE_ADAPTER_ERROR, param1, param2);
	    return MV_FALSE;
	    
    }/*switch*/
    
    return MV_TRUE;
}

/* ------------------------------------------------------------------------- */
#define bus_to_phys(devno, a)	pci_mem_to_phys(devno, a)

///////////////  >>> enclosure
extern void lg_Usb_Led_Ctrl(int mode);
extern void lg_Odd_Led_Ctrl(int mode);
extern void lg_Hdd_Led_Ctrl(int mode);
#define ENC_LED_OFF 	0
#define ENC_LED_ON  	1
#define ENC_LED_BLINK	2
//////////////  <<< enclosure

void ide_init (void)
{
	int j, numOfDrives;
	int idx = 0;
	unsigned int temp, initAdapterResult, pciCommand;
	unsigned short stemp;
	MV_SATA_ADAPTER *pMvSataAdapter;
	int devno = -1;
	unsigned char channelIndex;
	unsigned int numOfIdeDev = 0;
	unsigned int numOfAdapters = 0;
	MV_BOOLEAN integratedSataDetected = MV_FALSE;
	MV_BOOLEAN integratedSataInitialized = MV_FALSE;
	int intSataAccess = 0;
	int pexSataAccess = 0;
	char *env;

#if defined(CONFIG_MV78200)
	/* Check in dual CPU system which CPU use integrated SATA */
	if (mvSocUnitIsMappedToThisCpu(SATA))
		intSataAccess = 1;

	/* Check in dual CPU system which CPU use integrated SATA */
	if ( (mvSocUnitIsMappedToThisCpu(PEX00)) || (mvSocUnitIsMappedToThisCpu(PEX10)) )
		pexSataAccess = 1;
#else
	/* When working in none dual Linux mode all CPU have access */
	intSataAccess = 1;
	pexSataAccess = 1;
#endif

#ifdef MV_LOGGER
#if defined (MV_LOG_DEBUG)
	mvLogRegisterModule(MV_CORE_DRIVER_LOG_ID, 0x1FF,
			szModules[MV_CORE_DRIVER_LOG_ID]);
#elif defined (MV_LOG_ERROR)
	mvLogRegisterModule(MV_CORE_DRIVER_LOG_ID, MV_DEBUG_ERROR,
			szModules[MV_CORE_DRIVER_LOG_ID]);
#endif
#endif
	printf("\nMarvell Serial ATA Adapter, MAX_DEVICE=%d\n", CFG_IDE_MAXDEVICE);
	curr_device = -1;

	memset(sataAdapters,0, sizeof(HW_ADAPTER_DESCRIPTION)*CFG_IDE_MAXBUS);
	/* Init ide structure */
	for(j = 0; j < CFG_IDE_MAXDEVICE; j++)
	{
		ide_dev_desc[j].type=DEV_TYPE_UNKNOWN;
		ide_dev_desc[j].if_type=IF_TYPE_IDE;
		ide_dev_desc[j].dev=j;
		ide_dev_desc[j].part_type=PART_TYPE_UNKNOWN;
		ide_dev_desc[j].blksz=0;
		ide_dev_desc[j].lba=0;
		ide_dev_desc[j].block_read=ide_read;
	}

	while ((numOfIdeDev < CFG_IDE_MAXDEVICE) && (numOfAdapters < CFG_IDE_MAXBUS)) {      
		MV_BOOLEAN integratedSataDevice = MV_FALSE;
		numOfDrives = 0;

#if defined(MV_INCLUDE_INTEG_SATA)
		if(intSataAccess)
			if (integratedSataInitialized != MV_TRUE)
			{
				if (MV_FALSE == mvCtrlPwrClckGet(SATA_UNIT_ID, 0))
				{
					printf("Warning Integrated SATA is Powered Off\n");
				}
				else
				{
					integratedSataDetected = MV_TRUE;
					integratedSataDevice = MV_TRUE;

					mvSataWinInit();
					printf("Integrated Sata device found\n");
				}
			}
#endif


#ifdef CONFIG_PCI
		if(pexSataAccess)
		{
			if ((integratedSataDetected == MV_FALSE) ||
					(integratedSataInitialized == MV_TRUE))
			{
				/* Find PCI device(s) */
				if ((devno = pci_find_devices(supported, idx++)) < 0) { 
					if(!numOfAdapters) printf("no device found \n");
					break;
				}   
			}
		}
#endif /* CONFIG_PCI */

		integratedSataInitialized = MV_TRUE;
		sataAdapters[numOfAdapters].devno = devno;
		sataAdapters[numOfAdapters].valid = MV_TRUE;

		memset (&sataAdapters[numOfAdapters].mvSataAdapter, 0, sizeof(MV_SATA_ADAPTER));
		pMvSataAdapter = &sataAdapters[numOfAdapters].mvSataAdapter;

		if (integratedSataDevice == MV_TRUE){
			pMvSataAdapter->adapterIoBaseAddress = INTER_REGS_BASE + SATA_REG_BASE - 0x20000;
			pMvSataAdapter->pciConfigDeviceId = mvCtrlModelGet();
			pMvSataAdapter->pciConfigRevisionId = 0;
		}
#ifdef CONFIG_PCI
		else{
			if(pexSataAccess)
			{
				pci_read_config_dword(devno, PCI_BASE_ADDRESS_0 ,&temp);
				pMvSataAdapter->adapterIoBaseAddress = bus_to_phys(devno, (temp & 0xfffffff0));
				pci_read_config_word(devno, PCI_DEVICE_ID, &stemp);
				pMvSataAdapter->pciConfigDeviceId = stemp;
				pci_read_config_word(devno, PCI_REVISION_ID, &stemp);	
				pMvSataAdapter->pciConfigRevisionId = stemp & 0xff;
			}
		}
#endif /*CONFIG_PCI */

		pMvSataAdapter->adapterId = numOfAdapters;

		pMvSataAdapter->intCoalThre[0]= 4;
		pMvSataAdapter->intCoalThre[1]= 4;
		pMvSataAdapter->intTimeThre[0] = 150*50 ;
		pMvSataAdapter->intTimeThre[1] = 150*50;
		pMvSataAdapter->mvSataEventNotify = mvSataEventNotify; 
		pMvSataAdapter->IALData = (void *)&sataAdapters[numOfAdapters];
		pMvSataAdapter->pciSerrMask = MV_PCI_SERR_MASK_REG_ENABLE_ALL;
		pMvSataAdapter->pciInterruptMask = 0;
		pMvSataAdapter->pciCommand = MV_PCI_COMMAND_REG_DEFAULT;

#ifdef CONFIG_PCI
		if (integratedSataDevice != MV_TRUE) {
			if(pexSataAccess)
			{
				/* Enable master / IO / Memory accesses */
				pci_read_config_dword(devno, PCI_COMMAND ,&pciCommand);
				pci_write_config_dword(devno, PCI_COMMAND, pciCommand | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

				printf("Found adapter at bus %d, device %d ... Scanning channels\n",PCI_BUS(devno), PCI_DEV(devno));	
			}
		}
#endif /*CONFIG_PCI */

		/* Check if found adapter */
		if( (integratedSataDetected == MV_FALSE) && (devno == -1) )
		{
			printf("no device found \n");
			break;
		}

		/* Check if found adapter over PEX/PCI */
		if( (numOfAdapters > 0) && (devno == -1) )
			break;

		/* Init adapter */
		initAdapterResult = mvSataInitAdapter(pMvSataAdapter);
		if (initAdapterResult == MV_FALSE){
			printf("Error Initializing SATA Adapter\n");
			sataAdapters[numOfAdapters].valid = MV_FALSE;
			numOfAdapters ++;
			continue;
		}

		/* Enable staggered spin-up of all SATA channels */
		if (mvSataEnableStaggeredSpinUpAll(pMvSataAdapter) == MV_FALSE) {
			printf("Error in mvSataEnableStaggeredSpinUpAll\n");
			sataAdapters[numOfAdapters].valid = MV_FALSE;
			numOfAdapters ++;
			continue;
		}
		/* when sending DMA command we poll interrupt for completion*/
		mvSataUnmaskAdapterInterrupt(pMvSataAdapter);

		/* find and init channels */
		for (channelIndex = 0 ; channelIndex < pMvSataAdapter->numberOfChannels ; channelIndex++) {
			if (mvSataIsStorageDeviceConnected(pMvSataAdapter, channelIndex, NULL) == 
					MV_TRUE){
				printf("Channel %x/%x is connected ... \n", channelIndex, pMvSataAdapter->numberOfChannels);
				///////////  >>> enclosure
				//                        if (channelIndex == 1)
				//                                lg_Odd_Led_Ctrl(ENC_LED_BLINK);
				//                        else
				//                                lg_Odd_Led_Ctrl(ENC_LED_ON);
				///////////  <<< enclosure

				InitChannel(&sataAdapters[numOfAdapters], channelIndex);
				if (mvSataConfigureChannel(pMvSataAdapter, channelIndex) == 
						MV_FALSE)
				{
					printf("Error in mvSataConfigureChannel on channel %d\n",channelIndex);
					pMvSataAdapter->sataChannel[channelIndex] = NULL;
					numOfIdeDev++;
					///////////  >>> enclosure
					//                                if (channelIndex == 0)
					//                                        lg_Hdd_Led_Ctrl(ENC_LED_ON);
					///////////  <<< enclosure
				}	
				else if (StartChannel(&sataAdapters[numOfAdapters],
							channelIndex,&ide_dev_desc[numOfIdeDev]) == MV_FALSE){
					printf ("numOfAdapters= %d, channelIndex= %d, numOfIdeDev= %d\n",
							numOfAdapters, channelIndex, numOfIdeDev);
					printf ("Failed initializing storage deivce connected "
							"to SATA channel %d\n",channelIndex);
					pMvSataAdapter->sataChannel[channelIndex] = NULL;
					numOfIdeDev++;
					///////////  >>> enclosure
					//                                if (channelIndex == 0)
					//                                        lg_Hdd_Led_Ctrl(ENC_LED_ON);
					///////////  <<< enclosure
				}else{
					int port;
					MV_CHANNEL_INFO *channelInfo=&sataAdapters[numOfAdapters].channelInfo[channelIndex];
					for(port = 0; port < channelInfo->numberOfPorts; port++){
						if(!(channelInfo->connected & (1 << port)))
						{
							numOfIdeDev++;
							continue;
						}

						if(initDisk(pMvSataAdapter, channelIndex, port, &ide_dev_desc[numOfIdeDev]) == MV_FALSE){
							/* mark it as not connected */
							sataAdapters[numOfAdapters].channelInfo[channelIndex].connected &= ~(1 << port);
							numOfIdeDev++;
							///////////  >>> enclosure
							//                                                lg_Hdd_Led_Ctrl(ENC_LED_ON);
							///////////  <<< enclosure
							continue;
						}
						///////////  >>> enclosure
						//                                        lg_Hdd_Led_Ctrl(ENC_LED_BLINK);
						///////////  <<< enclosure
						dev_print(&ide_dev_desc[numOfIdeDev]);
						if ((ide_dev_desc[numOfIdeDev].lba > 0) && 
								(ide_dev_desc[numOfIdeDev].blksz > 0)) 
						{
							/* initialize partition type */
							init_part (&ide_dev_desc[numOfIdeDev]);
							if (curr_device < 0)
								curr_device = numOfIdeDev;
						}
						numOfDrives++;
						numOfIdeDev++;
					}
				}
			}else{
				pMvSataAdapter->sataChannel[channelIndex] = NULL;
				sataAdapters[numOfAdapters].channelInfo[channelIndex].numberOfPorts = 1;
				numOfIdeDev++;
			}

		}

		numOfAdapters++;	
	}

	ide_detected = numOfIdeDev;
	ide_initiated=1;

	putc ('\n');
}

/* ------------------------------------------------------------------------- */

int ide_disc_ready(ulong src_addr, ulong src_size)      // pch_070809
{
    MV_SATA_ADAPTER *pSataAdapter = NULL;
    MV_SATA_CHANNEL     *pSataChannel;
    MV_U8   cdb[12];
    MV_U8   bTmp;
    MV_U8   buf[32768], io_data[8], msg_buf[128];
    MV_U16_PTR pbuf = (MV_U16_PTR) buf;
    MV_U32      addr, data_len, sense_code, dwTmp;
    MV_U8_PTR ptgt_addr;
    MV_BOOLEAN  rslt;
    MV_U8       bTrayLoaded = 0;
    int         ret_val, ret_val_io;

    ptgt_addr = (MV_U8_PTR) 0x00400000;

    if(sataAdapters[0].valid == MV_FALSE)
    {
        printf("Error_in_ide_test[No valid adapter]\n");
        return MV_FALSE;
    }
    pSataAdapter = &(sataAdapters[0].mvSataAdapter);
    if(pSataAdapter->sataChannel[4] == NULL)
    {
        printf("Error_in_ide_test[No valid channel]\n");
        return MV_FALSE;
    }

    pSataChannel = pSataAdapter->sataChannel[4];

        // Inquiry start =========================
    memset(buf, 0x20, 2048);
    memset(cdb, 0, 12);
    cdb[0] = 0x12;
    cdb[4] = 96;
    printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    mvOsSemTake(&pSataAdapter->sataChannel[4]->semaphore);
    if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
        0, cdb, 12, buf, 96, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command Failed_PIO_dr0\n");
        //return MV_FALSE;
    }
    mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);
    printf("ATAPI Packet Command Done\n");

    for(bTmp=0; bTmp<28; bTmp++)
    {
        if(buf[8+bTmp] < 0x20) buf[8+bTmp] = 0x20;
    }
    buf[8+28] = 0x00;

     printf("Inquiry : %s\r\n", &buf[8]);
    // Inquiry end =============================

                // pch_081111 : Tray Eject First
    //goto tur_retry_0;   // pch_080820 : check if there is setup disc in drive first

tray_eject_0:
        bTrayLoaded = 0;
        src_addr = 0xffffffff;
        src_size = 0xffffffff;
        ret_val_io = LgDiag_IoRead(0x00, io_data);  // pch_080820 : clear key_value

        // Tray Eject start =========================
    memset(cdb, 0, 12);
    cdb[0] = 0x1b;
    cdb[1] = 0x01;
    cdb[4] = 0x02;
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    //mvOsSemTake(&pSataAdapter->sataChannel[4]->semaphore);
    if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_NON_DATA,
        0, cdb, 12, buf, 0, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command (Tray Eject) Failed_PIO(%08x)\n", sense_code);
        memset(cdb, 0, 12);
        cdb[0] = 0x03;
        cdb[4] = 0x30;
        rslt = executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 0x30, &sense_code);
        printf("Rslt_1b_0[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x\n",
            0x1b, rslt, buf[0], buf[2], buf[12], buf[13]);
        sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
        if(sense_code == 0x062900)
        {
                goto tray_eject_0;
        }
        //udelay(1000000);
    }
    //mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);
    printf("ATAPI Packet Command (Tray Eject) Done\n");
    // Tray Eject end =============================

                // pch_081125 : Work around for drive bug
    memset(cdb, 0, 12);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_NON_DATA,
        0, cdb, 12, buf, 0, &sense_code) == MV_TRUE)
    {
        udelay(200000);
        goto tray_eject_0;
    }

tur_retry_0:
    memset(cdb, 0, 12);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_NON_DATA,
        0, cdb, 12, buf, 0, &sense_code) == MV_FALSE)
    {
        //printf("ATAPI Packet Command (TUR) Failed_PIO_dr(%08x)\n", sense_code);
        //mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);
        // Read Sense Code
        memset(cdb, 0, 12);
        cdb[0] = 0x03;
        cdb[4] = 0x30;
        rslt = executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 0x30, &sense_code);
        //printf("Rslt_00[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x\n",
        //    0x00, rslt, buf[0], buf[2], buf[12], buf[13]);
        sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
        if((sense_code == 0x020401) && !bTrayLoaded)
        {
                bTrayLoaded = 1;
            ret_val = LgDiag_Msg2Lcd("Checking Disc Contents...", 0); udelay(100000);
            printf("Ret_val_in_ide_disc_ready_0:%d\n", ret_val);
        }
        //else if(bTrayLoaded && ((sense_code == 0x023a00) || (sense_code == 0x023a01)))
        else if((sense_code == 0x023a00) || (sense_code == 0x023a01))
        {
            ret_val = LgDiag_Msg2Lcd("Insert Setup Disc Please...", 0); udelay(100000);
            printf("Ret_val_in_ide_disc_ready_1:%d\n", ret_val);
                goto tray_eject_0;
        }

                ret_val_io = LgDiag_IoRead(0x00, io_data);  // pch_080820 : check key_state
        if(ret_val_io == MV_TRUE)
        {
            if((io_data[0] == 0x00) && (io_data[1]&0x08))
            {
                // Tray Load =========================
                memset(cdb, 0, 12);
                cdb[0] = 0x1b;
                cdb[1] = 0x01;
                cdb[4] = 0x03;
                pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

                rslt = executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_NON_DATA,
                    0, cdb, 12, buf, 0, &sense_code);
            }
        }

        udelay(1000000);
        goto tur_retry_0;
        //return MV_FALSE;
    }
    //mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);
    printf("ATAPI Packet Command (TUR) Done\n");
    // TUR end =============================

        //if(src_size == 0xffffffff) src_size = 0x352;
        //if(src_addr == 0xffffffff) src_addr = 0x1658e;
    // Get Address, Size information (pch_080901)
    if((src_size == 0xffffffff) || (src_addr == 0xffffffff))
    {
        data_len = 1;
get_info_retry_0:
        memset(buf, 0x00, 2048);
        memset(cdb, 0, 12);
        sense_code = 0;
            addr = 0;

        cdb[0] = 0x28;
            cdb[2] = ((addr>>24)&0x00ff);
        cdb[3] = ((addr>>16)&0x00ff);
        cdb[4] = ((addr>>8)&0x00ff);
        cdb[5] = ((addr>>0)&0x00ff);
        cdb[7] = ((data_len>>8)&0x00ff);
        cdb[8] = ((data_len>>0)&0x00ff);

        pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

        if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
        {
            printf("ATAPI Packet Command Failed_PIO_dr1\n");
            memset(cdb, 0, 12);
            cdb[0] = 0x03;
                cdb[4] = 0x30;
            rslt = executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 0x30, &sense_code);
                printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
                0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
            sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
                if(sense_code == 0x062800)
                {
                        goto get_info_retry_0;
                }
                memset(msg_buf, 0x00, 128);
                sprintf(msg_buf, "Rd Err0 %08x", sense_code);
                setenv("enc_err", msg_buf);
                saveenv();
            ret_val = LgDiag_Msg2Lcd(msg_buf, 0); udelay(100000);
                        //ret_val = LgDiag_Msg2Lcd("Invalid Disc... ", 0); udelay(100000);
            printf("Ret_val_in_ide_disc_ready_4:%d[addr:%08x, size:%08x]\n", ret_val, src_addr, src_size);
            udelay(3000000);
            ret_val = LgDiag_Msg2Lcd("Insert Setup Disc Please...", 0); udelay(100000);
            printf("Ret_val_in_ide_disc_ready_5:%d\n", ret_val);
                goto tray_eject_0;
        }

        src_addr = buf[2040]*0x1000000 + buf[2041]*0x10000 + buf[2042]*0x100 + buf[2043];
        src_size = buf[2044]*0x1000000 + buf[2045]*0x10000 + buf[2046]*0x100 + buf[2047];

        if(!src_addr || !src_size)
        {
            src_size = 0x352;
            src_addr = 0x1658e;
        }
    }

        data_len = 1;
read_retry_0:
    memset(buf, 0x00, 2048);
    memset(cdb, 0, 12);
    sense_code = 0;
        addr = src_addr;
    //data_len = 1;
    cdb[0] = 0x28;
        cdb[2] = ((addr>>24)&0x00ff);
    cdb[3] = ((addr>>16)&0x00ff);
    cdb[4] = ((addr>>8)&0x00ff);
        cdb[5] = ((addr>>0)&0x00ff);
    cdb[7] = ((data_len>>8)&0x00ff);
    cdb[8] = ((data_len>>0)&0x00ff);

    //printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    //mvOsSemTake(&pSataAdapter->sataChannel[4]->semaphore);
    if(executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command Failed_PIO_dr2\n");
        memset(cdb, 0, 12);
            cdb[0] = 0x03;
        cdb[4] = 0x30;
        rslt = executePacketCommand(pSataAdapter, 4, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 0x30, &sense_code);
        printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
            0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
            sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
            if(sense_code == 0x062800)
        {
                goto read_retry_0;
        }
        //udelay(1000000);
        memset(msg_buf, 0x00, 128);
        sprintf(msg_buf, "Rd Err1 %08x:%08x", sense_code, addr);
        setenv("enc_err", msg_buf);
        saveenv();
        ret_val = LgDiag_Msg2Lcd(msg_buf, 0); udelay(100000);
        //ret_val = LgDiag_Msg2Lcd("Invalid Disc... ", 0); udelay(100000);
        printf("Ret_val_in_ide_disc_ready_4:%d\n", ret_val);
        udelay(3000000);
        ret_val = LgDiag_Msg2Lcd("Insert Setup Disc Please...", 0); udelay(100000);
        printf("Ret_val_in_ide_disc_ready_5:%d\n", ret_val);
        goto tray_eject_0;
        //return 0;
            //return MV_FALSE;
    }
    //mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);

        printf("Addr: %08x, Data : %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
            addr,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                buf[256+0], buf[256+1], buf[256+2], buf[256+3], buf[256+4], buf[256+5], buf[256+6], buf[256+7]);

        if((buf[0] != 0x27) || (buf[1] != 0x05) || (buf[2] != 0x19) || (buf[3] != 0x56))
        {
                memset(msg_buf, 0x00, 128);
                sprintf(msg_buf, "Addr: %08x, Data : %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
            addr,
                        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                        buf[256+0], buf[256+1], buf[256+2], buf[256+3], buf[256+4], buf[256+5], buf[256+6], buf[256+7]);
                setenv("enc_err", msg_buf);
                saveenv();
        ret_val = LgDiag_Msg2Lcd("Invalid Disc... ", 0); udelay(100000);
        printf("Ret_val_in_ide_disc_ready_2:%d\n", ret_val);
        udelay(3000000);
        ret_val = LgDiag_Msg2Lcd("Insert Setup Disc Please...", 0); udelay(100000);
        printf("Ret_val_in_ide_disc_ready_3:%d\n", ret_val);
                goto tray_eject_0;
                //return 0;
        }

        return 1;
}

void ide_load(ulong src_addr, ulong src_size, ulong target)
{
    MV_SATA_ADAPTER *pSataAdapter = NULL;
    MV_SATA_CHANNEL     *pSataChannel;
    MV_U8   cdb[12];
    MV_U8   bTmp; 
    MV_U8   buf[32768];
    //MV_U16_PTR pbuf = (MV_U16_PTR) buf;
    MV_U32      addr, data_len, sense_code, dwTmp, locate, locate2, kernel_addr, kernel_size;
    MV_U8_PTR ptgt_addr;
    MV_BOOLEAN  rslt;
    int         ret_val;

    locate = locate2 = ret_val =0;
    if(target == 0)
        ptgt_addr = (MV_U8_PTR) 0x00400000;
    else 
        ptgt_addr = (MV_U8_PTR) target;

    if(sataAdapters[0].valid == MV_FALSE)
    {
        printf("Error_in_ide_test[No valid adapter]\n");
        return;
    }
    pSataAdapter = &(sataAdapters[0].mvSataAdapter);
    if(pSataAdapter->sataChannel[1] == NULL)
    {
        printf("Error_in_ide_test[No valid channel]\n");
        return;
    }

    pSataChannel = pSataAdapter->sataChannel[1];

        // Inquiry start =========================
    memset(buf, 0x20, 2048);
    memset(cdb, 0, 12);
    cdb[0] = 0x12;
    cdb[4] = 96;
    printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
    if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
        0, cdb, 12, buf, 96, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command Failed_PIO_dl0\n");
        //return MV_FALSE;
    }
    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
    printf("ATAPI Packet Command Done\n");

    for(bTmp=0; bTmp<28; bTmp++)
    {
        if(buf[8+bTmp] < 0x20) buf[8+bTmp] = 0x20;
    }
    buf[8+28] = 0x00;

    printf("Inquiry : %s\r\n", &buf[8]);
    // Inquiry end =============================

//tur_retry:
    // TUR start =========================
    memset(cdb, 0, 12);
    //printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
    if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_NON_DATA,
        0, cdb, 12, buf, 0, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command (TUR) Failed_PIO(%08x)\n", sense_code);
        if((sense_code&0x00ff0000) == 0x00060000)
        {
                 ;      //mvOsSemRelease(&pSataAdapter->sataChannel[4]->semaphore);
                 ;      //goto tur_retry;
        }
        //return MV_FALSE;
    }
    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
    printf("ATAPI Packet Command (TUR) Done\n");
    // TUR end =============================

        if(src_size == 0xffffffff) udelay(3000000);
        if(src_addr == 0xffffffff) udelay(3000000);

/*
    data_len = src_size;
    memset(buf, 0x00, 2048);
    memset(cdb, 0, 12);
    sense_code = 0;
    addr = src_addr;

    cdb[0] = 0x28;
    cdb[2] = ((addr>>24)&0x00ff);
    cdb[3] = ((addr>>16)&0x00ff);
    cdb[4] = ((addr>>8)&0x00ff);
    cdb[5] = ((addr>>0)&0x00ff);
    cdb[7] = ((data_len>>8)&0x00ff);
    cdb[8] = ((data_len>>0)&0x00ff);

    pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

    mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
    if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
    {
        printf("ATAPI Packet Command Failed_PIO_dl1\n");
        memset(cdb, 0, 12);
        cdb[0] = 0x03;
        cdb[4] = 0x30;
        rslt = executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 0x30, &sense_code);
        printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
                0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
        sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
        if(sense_code == 0x062800)
        {
            mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
            goto get_info_retry;
        }
        printf("Ret_val_in_ide_load_1:%d[addr:%08x, size:%08x, target:%08x]\n", ret_val, src_addr, src_size, target);
        udelay(3000000);
        printf("Ret_val_in_ide_load_ready_5:%d\n", ret_val);

        src_addr = src_size = 0;
    }else {
        printf("[addr:0x%08x, size:0x%08x, sense code:0x%08x]\n", src_addr, src_size,sense_code);
        printf("[addr]: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f [ascii]");
        int i,j;
        for( i=0; i < 2048; i+=16){
            printf("\n0x%04x:", i);
            for( j=0; j<16; j++) {
                printf(" %02x", buf[i+j]);
            }
            printf(" ");
            for( j=0; j<16; j++) {
                if((buf[i+j] >= 0x20) && (buf[i+j] <= 0x7e)) printf("%c", (char)buf[i+j]);
                else printf(".");
            }
        }
        printf("\n");
        
    }
*/
    // search for kernel image & rootfs image
    if((src_size == 0xffffffff) || (src_addr == 0xffffffff))
    {
        data_len = 1;
get_pathtable_retry:
        memset(buf, 0x00, 2048);
        memset(cdb, 0, 12);
        sense_code = 0;
        addr = 0x10;

        cdb[0] = 0x28;
        cdb[2] = ((addr>>24)&0x00ff);
        cdb[3] = ((addr>>16)&0x00ff);
        cdb[4] = ((addr>>8)&0x00ff);
        cdb[5] = ((addr>>0)&0x00ff);
        cdb[7] = ((data_len>>8)&0x00ff);
        cdb[8] = ((data_len>>0)&0x00ff);

        pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

        mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
        // search for path table
        if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
        {
            printf("ATAPI Packet Command Failed_PIO_dl1\n");
            memset(cdb, 0, 12);
            cdb[0] = 0x03;
            cdb[4] = 0x30;
            rslt = executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 0x30, &sense_code);
            printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
                0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
            sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
            if(sense_code == 0x062800)
            {
                mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                goto get_pathtable_retry;
            }
            printf("Ret_val_in_ide_load_1:%d[addr:%08x, size:%08x, target:%08x]\n", ret_val, addr, data_len, target);
            udelay(3000000);
            printf("Ret_val_in_ide_load_ready_5:%d\n", ret_val);
            if(ret_val < 5) 
            {
                ret_val++;
                goto get_pathtable_retry;
            }

            src_addr = src_size = 0;
        }
        else
        {
            src_addr = buf[0x8f]*0x1000000 + buf[0x8e]*0x10000 + buf[0x8d]*0x100 + buf[0x8c];
            src_size = 1;
            printf("[addr:0x%08x, size:0x%08x, sense code:0x%08x]\n", addr, data_len,sense_code);
            /*printf("[addr]: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f [ascii]");
            int i,j;
            for( i=0; i < 2048; i+=16){
                printf("\n0x%04x:", i);
                for( j=0; j<16; j++) {
                    printf(" %02x", buf[i+j]);
                }
                printf(" ");
                for( j=0; j<16; j++) {
                    if((buf[i+j] >= 0x20) && (buf[i+j] <= 0x7e)) printf("%c", (char)buf[i+j]);
                    else printf(".");
                }
            }
            printf("\n");*/
            printf("[path table addr:0x%08x]\n", src_addr);
            printf("\n");
        }
        mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);

        // find casper
        addr = src_addr;
        data_len = src_size;
get_casper_retry:
        memset(buf, 0x00, 2048);
        memset(cdb, 0, 12);
        sense_code = 0;

        cdb[0] = 0x28;
        cdb[2] = ((addr>>24)&0x00ff);
        cdb[3] = ((addr>>16)&0x00ff);
        cdb[4] = ((addr>>8)&0x00ff);
        cdb[5] = ((addr>>0)&0x00ff);
        cdb[7] = ((data_len>>8)&0x00ff);
        cdb[8] = ((data_len>>0)&0x00ff);

        pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

        mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
        // search for CASPER 
        if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
        {
            printf("ATAPI Packet Command Failed_PIO_dl2\n");
            memset(cdb, 0, 12);
            cdb[0] = 0x03;
                cdb[4] = 0x30;
            rslt = executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 0x30, &sense_code);
                printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
                0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
            sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
                if(sense_code == 0x062800)
                {
                    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                        goto get_casper_retry;
                }
            printf("Ret_val_in_ide_load_1:%d[addr:%08x, size:%08x]\n", ret_val, src_addr, src_size);
            udelay(3000000);
            printf("Ret_val_in_ide_load_ready_5:%d\n", ret_val);

            src_addr = src_size = 0;
        }
        else
        {
            printf("[addr:0x%08x, size:0x%08x, sense code:0x%08x]\n", addr, data_len,sense_code);
            printf("[addr]: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f [ascii]");
            int i,j;
            // find CASPER string
            for( i=0; i < 2048; i++) {

                if(i%16 == 0) printf("\n0x%04x:", i);
                printf(" %02x", buf[i]);
                if(i%16 == 15) {
                    printf(" ");
                    for(j=15; j>=0; j--) {
                        if((buf[i-j] >= 0x20) && (buf[i-j] <= 0x7e)) printf("%c", (char)buf[i-j]);
                        else printf(".");
                    }
                }

                if(buf[i] == 'C') {
                    if(strncmp(&buf[i], "CASPER", 5) == 0)
                        locate = i;
//                    printf("[i=%02x, casper locate index:0x%02x, value:%c%c%c%c%c%c]\n",
//                        i, locate, buf[i],buf[i+1],buf[i+2],buf[i+3],buf[i+4],buf[i+5]);
                }
            }
            printf("\n");
            addr = buf[locate-3]*0x1000000 + buf[locate-4]*0x10000 + buf[locate-5]*0x100 + buf[locate-6];
            printf("[casper dir locate index:0x%02x, casper sector addr:0x%08x]\n", locate, addr);
            printf("\n");
//           src_addr = buf[2040]*0x1000000 + buf[2041]*0x10000 + buf[2042]*0x100 + buf[2043];
//           src_size = buf[2044]*0x1000000 + buf[2045]*0x10000 + buf[2046]*0x100 + buf[2047];
        }

        mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);

        data_len = 1;
get_image_retry:
        memset(buf, 0x00, 2048);
        memset(cdb, 0, 12);
        sense_code = 0;

        cdb[0] = 0x28;
        cdb[2] = ((addr>>24)&0x00ff);
        cdb[3] = ((addr>>16)&0x00ff);
        cdb[4] = ((addr>>8)&0x00ff);
        cdb[5] = ((addr>>0)&0x00ff);
        cdb[7] = ((data_len>>8)&0x00ff);
        cdb[8] = ((data_len>>0)&0x00ff);

        pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

        mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
        // search for image at CASPER dir
        if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
        {
            printf("ATAPI Packet Command Failed_PIO_dl3\n");
            memset(cdb, 0, 12);
            cdb[0] = 0x03;
                cdb[4] = 0x30;
            rslt = executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 0x30, &sense_code);
                printf("Rslt[%02x]:%d, EC:%02x, SK:%02x, ASC:%02x, ASCQ:%02x, ADDR:%08x, LENGTH:%08x\n",
                0x28, rslt, buf[0], buf[2], buf[12], buf[13], addr, data_len);
            sense_code = buf[2]*0x10000 + buf[12]*0x100 + buf[13];
                if(sense_code == 0x062800)
                {
                    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                        goto get_image_retry;
                }
            printf("Ret_val_in_ide_load_1:%d[addr:%08x, size:%08x]\n", ret_val, src_addr, src_size);
            udelay(3000000);
            printf("Ret_val_in_ide_load_ready_5:%d\n", ret_val);

            src_addr = src_size = 0;
        }
        else
        {
            printf("[addr:0x%08x, size:0x%08x, sense code:0x%08x]\n", addr, data_len, sense_code);
            //printf("[addr]: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f [ascii]");
            int i;//,j;
            // find VMLINUZ, INITRD.GZ string
            for( i=0; i < 2048; i++) {
                /*if(i%16 == 0) printf("\n0x%04x:", i);
                printf(" %02x", buf[i]);
                if(i%16 == 15) {
                    printf(" ");
                    for(j=15; j>=0; j--) {
                        if((buf[i-j] >= 0x20) && (buf[i-j] <= 0x7e)) printf("%c", (char)buf[i-j]);
                        else printf(".");
                    }
                }*/
                if(buf[i] == 'V') {
                    if(strncmp(&buf[i], "VMLINUZ", 7) == 0)
                        locate = i;
//                    printf("[i=%02x, VMLINUZ locate index:0x%02x, value:%c%c%c%c%c%c%c]\n",
//                        i, locate, buf[i],buf[i+1],buf[i+2],buf[i+3],buf[i+4],buf[i+5],buf[i+6]);
                }
                if(buf[i] == 'I') {
                    if(strncmp(&buf[i], "INITRD.GZ", 9) == 0)
                        locate2= i;
//                    printf("[i=%02x, INITRD.GZ locate index:0x%02x, value:%c%c%c%c%c%c%c%c%c]\n",
//                        i, locate2, buf[i],buf[i+1],buf[i+2],buf[i+3],buf[i+4],buf[i+5],buf[i+6],buf[i+7],buf[i+8]);
                }
            }
            printf("\n");
            addr = buf[locate-0x1c]*0x1000000 + buf[locate-0x1d]*0x10000 + buf[locate-0x1e]*0x100 + buf[locate-0x1f];
            data_len = buf[locate-0x14]*0x1000000 + buf[locate-0x15]*0x10000 + buf[locate-0x16]*0x100 + buf[locate-0x17];
            src_addr = buf[locate2-0x1c]*0x1000000 + buf[locate2-0x1d]*0x10000 + buf[locate2-0x1e]*0x100 + buf[locate2-0x1f];
            src_size = buf[locate2-0x14]*0x1000000 + buf[locate2-0x15]*0x10000 + buf[locate2-0x16]*0x100 + buf[locate2-0x17];
            printf("[VMLINUZ locate index:0x%04x, addr:0x%08x, size:0x%08x, sec_size:0x%08x]\n", locate, addr, data_len, data_len/2048+1);
            printf("[INITRD.GZ locate index:0x%04x, addr:0x%08x, size:0x%08x, sec_size:0x%08x]\n", locate2, src_addr, src_size, src_size/2048+1);
        }

        // load kernel
        kernel_addr = addr;
        kernel_size = data_len/2048+1;
        ptgt_addr = 0x1000000;
        data_len = 1;
        for(dwTmp = 0; dwTmp < kernel_size; dwTmp += data_len)
        {
read_kernel_retry:
            memset(buf, 0x00, sizeof(buf));
            memset(cdb, 0, 12);
            sense_code = 0;
            addr = kernel_addr + dwTmp;
            cdb[0] = 0x28;
            cdb[2] = ((addr>>24)&0x00ff);
            cdb[3] = ((addr>>16)&0x00ff);
            cdb[4] = ((addr>>8)&0x00ff);
            cdb[5] = ((addr>>0)&0x00ff);
            cdb[7] = ((data_len>>8)&0x00ff);
            cdb[8] = ((data_len>>0)&0x00ff);

            //printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
            pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

            mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
            if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
                0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
            {
                printf("ATAPI Packet Command Failed_PIO_dl4, count=%d, sensecode=%x\n", dwTmp, sense_code);
                if((sense_code&0x00ff0000) == 0x00060000)
                {
                    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                    goto read_kernel_retry;
                }
                mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                break;
            //return MV_FALSE;
            }
            mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
 // MV_NON_UDMA_PROTOCOL_PIO_DATA_IN, MV_NON_UDMA_PROTOCOL_PACKET_DMA
            if(!(dwTmp&0x0f))
            {
                printf("Addr: %08x, Data : %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
                  addr, buf[0], buf[1], buf[2], buf[3], buf[512], buf[513], buf[514], buf[515],
                  buf[1024], buf[1025], buf[1026], buf[1027], buf[1536], buf[1537], buf[1538], buf[1539]);
            }

            memcpy((MV_U8_PTR) (ptgt_addr + dwTmp*2048), buf, data_len*2048);
        }

        if(!sense_code)
        {
                flush_cache(ptgt_addr, kernel_size*2048);
        }

        src_size = src_size/2048+1;
        ptgt_addr = 0x2000000;
    }


        mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);

        // load ramdisk
        data_len = 1;
        for(dwTmp = 0; dwTmp < src_size; dwTmp += data_len)
        {
read_retry:
            memset(buf, 0x00, sizeof(buf));
            memset(cdb, 0, 12);
            sense_code = 0;
            addr = src_addr + dwTmp;
            cdb[0] = 0x28;
            cdb[2] = ((addr>>24)&0x00ff);
            cdb[3] = ((addr>>16)&0x00ff);
            cdb[4] = ((addr>>8)&0x00ff);
            cdb[5] = ((addr>>0)&0x00ff);
            cdb[7] = ((data_len>>8)&0x00ff);
            cdb[8] = ((data_len>>0)&0x00ff);

            //printf("dtype:%d[%d]\n", pSataChannel->deviceType, MV_SATA_DEVICE_TYPE_ATAPI_DEVICE);
            pSataChannel->deviceType = MV_SATA_DEVICE_TYPE_ATAPI_DEVICE;

            mvOsSemTake(&pSataAdapter->sataChannel[1]->semaphore);
            if(executePacketCommand(pSataAdapter, 1, MV_NON_UDMA_PROTOCOL_PACKET_PIO_DATA_IN,
            0, cdb, 12, buf, 2048*data_len, &sense_code) == MV_FALSE)
            {
                printf("ATAPI Packet Command Failed_PIO_dl5, sensecode=0x%08x\n", sense_code);
                if((sense_code&0x00ff0000) == 0x00060000)
                {
                    mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                    goto read_retry;
                }
                mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);
                break;
            //return MV_FALSE;
            }
            mvOsSemRelease(&pSataAdapter->sataChannel[1]->semaphore);

            if(!(dwTmp&0x0f))
            {
                printf("Addr: %08x, Data : %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
                        addr,
                        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                        buf[256+0], buf[256+1], buf[256+2], buf[256+3], buf[256+4], buf[256+5], buf[256+6], buf[256+7]);
            }

                memcpy((MV_U8_PTR) (ptgt_addr + dwTmp*2048), buf, data_len*2048);
        }

        if(!sense_code)
        {
                flush_cache(ptgt_addr, src_size*2048);
        }
}

block_dev_desc_t * ide_get_dev(int dev)
{
	return ((block_dev_desc_t *)&ide_dev_desc[dev]);
}

/* ------------------------------------------------------------------------- */

ulong ide_read_write (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer, int read)
{
        //MV_STORAGE_DEVICE_REGISTERS inATARegs;
        //MV_STORAGE_DEVICE_REGISTERS outATARegs;
        //MV_BOOLEAN isExt = 0;
    	MV_SATA_ADAPTER *pSataAdapter = NULL;
    	unsigned int channelIndex = 0;
	unsigned int adapter; //, command;
	int port;
	char *env;
        // ulong blk_transferred = 0;

	/* read count is 0 no read is needed */
	if(blkcnt == 0) {
		return 0;
	}

	/* data buffer must be 2 bytes aligned */
	if((long)buffer & 1) {
		printf("\nide error: address (0x%08p) is not 2 bytes aligned!\n",buffer);
		return 0;
	}


    	DP("ide %s: device %x, blknr %x blkcnt %x, buffer %x\n",
		((read)?"read":"write"),device, (unsigned int)blknr, blkcnt, (unsigned int)buffer);
    	/* find device */
    	for(adapter = 0 ; adapter < CFG_IDE_MAXBUS; adapter++)
    	{
		if(sataAdapters[adapter].valid == MV_FALSE)
			continue;

		pSataAdapter = &(sataAdapters[adapter].mvSataAdapter);

		for(channelIndex = 0; channelIndex < pSataAdapter->numberOfChannels; channelIndex++)
		{
			for(port = 0; port < sataAdapters[adapter].channelInfo[channelIndex].numberOfPorts; port++)
			{
				if(device == 0)
					goto found_device;
				
				device--;
			}
		}
    	}

found_device:
    	if(adapter == CFG_IDE_MAXBUS)
    	{
	  printf("Error didn't find requested device (%d).\n", device);
		return 0;
    	}    
    	if(pSataAdapter->sataChannel[channelIndex] == NULL)
    	{
		printf("Error No HD connection \n");
		return 0;
    	}

	env = getenv("sata_dma_mode");
	if(( (strcmp(env,"yes") == 0) || (strcmp(env,"Yes") == 0) )){
		return dma_read_write(pSataAdapter, channelIndex, port, 
				      blknr, blkcnt, buffer, read);
	}else {
		return pio_read_write(pSataAdapter, channelIndex, port,
				      blknr, blkcnt, buffer, read);
	}
}

static
ulong pio_read_write (MV_SATA_ADAPTER *pSataAdapter, 
		      unsigned int channelIndex,
		      MV_U8 port,
		      lbaint_t blknr, ulong blkcnt, ulong *buffer, int read)
{
    	MV_STORAGE_DEVICE_REGISTERS inATARegs;
    	MV_STORAGE_DEVICE_REGISTERS outATARegs;
    	MV_BOOLEAN isExt = MV_FALSE;
	unsigned int command;
	ulong blk_transferred = 0;


#if (__BE)
	if(read == 0)
		swapATABuffer((unsigned short *)buffer, blkcnt * ATA_SECTOR_SIZE);
#endif

    	/* read all blocks */
    	while(blkcnt > 0) {
		ulong cmd_blkcnt = (blkcnt > 128 ) ? 128 : blkcnt;

    		/* Set transfer mode */
    		memset(&inATARegs, 0, sizeof(inATARegs));
    		memset(&outATARegs, 0, sizeof(outATARegs));
    		inATARegs.commandRegister = (read)? MV_ATA_COMMAND_READ_SECTORS: MV_ATA_COMMAND_WRITE_SECTORS; 
    		inATARegs.sectorCountRegister = cmd_blkcnt; 
    		inATARegs.lbaLowRegister = ((blknr >> 0) & 0xFF); 
    		inATARegs.lbaMidRegister = ((blknr >> 8) & 0xFF);
    		inATARegs.lbaHighRegister = ((blknr >> 16) & 0xFF);	
    		inATARegs.deviceRegister = BIT6 | ((blknr >> 24) & 0xF); 

#ifdef CONFIG_LBA48
		/* more than 28 bits used, use 48bit mode */
     		if (blknr & IDE_BLOCK_NUMBER_MASK) {
			inATARegs.commandRegister = (read)? MV_ATA_COMMAND_READ_SECTORS_EXT : MV_ATA_COMMAND_WRITE_SECTORS_EXT;
			inATARegs.lbaLowRegister |=  ((blknr >> LBA_LOW_REG_SHIFT) & 0xFF) << 8;
			inATARegs.lbaMidRegister |=  ((blknr >> LBA_MID_REG_SHIFT) & 0xFF) << 8;
			inATARegs.lbaHighRegister |= ((blknr >> LBA_HIGH_REG_SHIFT) & 0xFF) << 8;
			inATARegs.deviceRegister = BIT6;
			isExt = MV_TRUE;
		}
#endif
		command = (read)? MV_NON_UDMA_PROTOCOL_PIO_DATA_IN: MV_NON_UDMA_PROTOCOL_PIO_DATA_OUT;
    		if (mvStorageDevExecutePIO(pSataAdapter, channelIndex, port, command,
                               isExt, (MV_U16 *)buffer, (cmd_blkcnt * 512)/2 , &inATARegs, &outATARegs) == MV_FALSE)
    		{
        		printf("[%d %d]: %s Fail for PIO\n", 
				pSataAdapter->adapterId, channelIndex, ((read)?"Read":"Write"));
        		return 0;
    		}
    		blknr += cmd_blkcnt;
		blkcnt = blkcnt - cmd_blkcnt;
    		buffer += (cmd_blkcnt * ATA_SECTORWORDS);
		blk_transferred += cmd_blkcnt;
    	}
#if (__BE)
		swapATABuffer((unsigned short *)buffer, blkcnt * ATA_SECTOR_SIZE);
#endif

    	return blk_transferred;
}
static int command_completed;
static MV_COMPLETION_TYPE completion_type;
static 
MV_BOOLEAN cmd_callback(struct mvSataAdapter *pSataAdapter,
			MV_U8 channel_index,
			MV_COMPLETION_TYPE type,
			MV_VOID_PTR command_id, MV_U16 edma_error_cause,
			MV_U32 time_stamp,
			struct mvStorageDevRegisters *deviceRegs){
	command_completed = 1;
	completion_type = type;
	return MV_TRUE;
}
ulong dma_read_write (MV_SATA_ADAPTER *pSataAdapter, 
		      unsigned int channelIndex,
		      MV_U8 port,
		      lbaint_t blknr, ulong blkcnt, ulong *buffer, int read)
{
	int res = 0;
	int transfered_blks = 0;

	/* we use single PRD table entry (limited to 64KB - 128 sector) */
	while(blkcnt){
		int chunk = (blkcnt > 128) ? 128: blkcnt;

		res = dma_read_write_cmd(pSataAdapter, channelIndex, port, blknr, chunk, 
					 buffer, read) ;
		
		if(res != chunk)
			return transfered_blks;
		
		transfered_blks += res;
		buffer += res << 7; //buffer is in long
		blknr += res;
		blkcnt -= res;
	}
	return transfered_blks;
}

static
ulong dma_read_write_cmd (MV_SATA_ADAPTER *pSataAdapter, 
			  unsigned int channelIndex,
			  MV_U8 port,
			  lbaint_t blknr, ulong blkcnt, ulong *buffer, int read)
{
	MV_QUEUE_COMMAND_INFO q_cmd_info;
	MV_UDMA_COMMAND_PARAMS      *udmaCommand = &q_cmd_info.commandParams.udmaCommand;
	MV_SATA_DEVICE_TYPE	deviceType;
	MV_U32			byte_count = blkcnt << 9;
	MV_U32 buffer_addr = (MV_U32) buffer;

	if ( blkcnt > 128){
		printf("error in %s: blk count %d exceeded max limit\n", 
		       __func__, blkcnt);
	}
		
	//	mvSataDisableChannelDma(pSataAdapter, channelIndex);
	/* reset the request/response pointers as we allcated one entry*/
	/* but some info need to be preserved */
	deviceType = mvStorageDevGetDeviceType(pSataAdapter, channelIndex);
	mvSataConfigureChannel(pSataAdapter, channelIndex);
	mvStorageDevSetDeviceType(pSataAdapter, channelIndex,deviceType);
	mvSataConfigEdmaMode(pSataAdapter, channelIndex,
			     MV_EDMA_MODE_NOT_QUEUED, 2);
	mvSataEnableChannelDma(pSataAdapter, channelIndex);


	memset(&q_cmd_info, 0, sizeof(MV_QUEUE_COMMAND_INFO));
	memset(prd_table, 0, 2 * sizeof(MV_SATA_EDMA_PRD_ENTRY));

	/* buffer must not cross address decode window */
	if(((buffer_addr & MRVL_SATA_BOUNDARY_MASK) + byte_count) > MRVL_SATA_BUFF_BOUNDARY)
	{
	     MV_U32 chunk_len = MRVL_SATA_BUFF_BOUNDARY - 
		  (buffer_addr & MRVL_SATA_BOUNDARY_MASK); 
	     
	     memset(prd_table + 1, 0, sizeof(MV_SATA_EDMA_PRD_ENTRY));
	     prd_table[0].lowBaseAddr = cpu_to_le32(buffer_addr);
	     prd_table[0].byteCount = cpu_to_le16(chunk_len & 0xFFFF);

	     byte_count -= chunk_len;
	     buffer_addr += chunk_len;
 	     prd_table[1].lowBaseAddr = cpu_to_le32(buffer_addr);
	     prd_table[1].byteCount = cpu_to_le16(byte_count & 0xFFFF);
	     prd_table[1].flags = cpu_to_le16(MV_EDMA_PRD_EOT_FLAG);
	     
	}
	else
	{
 	     prd_table[0].lowBaseAddr = cpu_to_le32(buffer_addr);
	     prd_table[0].byteCount = cpu_to_le16(byte_count & 0xFFFF);
	     prd_table[0].flags = cpu_to_le16(MV_EDMA_PRD_EOT_FLAG);
	}

	q_cmd_info.type = MV_QUEUED_COMMAND_TYPE_UDMA;
	q_cmd_info.PMPort = port;
	udmaCommand->readWrite = (read)? MV_UDMA_TYPE_READ : MV_UDMA_TYPE_WRITE;
	udmaCommand->isEXT = MV_FALSE;

#ifdef CONFIG_LBA48
	/* more than 28 bits used, use 48bit mode */
	if (blknr & IDE_BLOCK_NUMBER_MASK) {
	  udmaCommand->isEXT = MV_TRUE;
	}
#endif

	udmaCommand->FUA = MV_FALSE;
	udmaCommand->lowLBAAddress = blknr & 0xFFFFFFFF;
	udmaCommand->highLBAAddress = (blknr >> 32 )& 0xFFFFFFFF;
	udmaCommand->numOfSectors = blkcnt;
	udmaCommand->prdLowAddr = (MV_U32)prd_table;
	udmaCommand->callBack = cmd_callback;
#if 0	
	printf("send dma command: %s lba %x sect %x addr %x\n", read?"read":"write", 
	       udmaCommand->lowLBAAddress, udmaCommand->numOfSectors, 
	       prd_table[0].lowBaseAddr);
#endif
	command_completed = 0;
	if(mvSataQueueCommand(pSataAdapter,
			      channelIndex,
			      &q_cmd_info) != MV_QUEUE_COMMAND_RESULT_OK){
		printf("error in mv_ide: queue command failed\n");
		mvSataDisableChannelDma(pSataAdapter, channelIndex);
		return 0;
	}

	do{
		mvSataInterruptServiceRoutine(pSataAdapter);
		udelay(1000);
		
	}while(!command_completed);

	mvSataDisableChannelDma(pSataAdapter, channelIndex);
	if(completion_type == MV_COMPLETION_TYPE_NORMAL)
		return blkcnt;
	else
		return 0;
}

/* ------------------------------------------------------------------------- */
#if (__BE)
void swapATABuffer(unsigned short *buffer, ulong count)
{
	count >>= 1;
	while(count--)
	{
		buffer[count] = MV_CPU_TO_LE16(buffer[count]);	
	}
}
#endif

ulong ide_read (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
	return ide_read_write (device, blknr, blkcnt, buffer, MV_TRUE);
}

ulong ide_write (int device, lbaint_t blknr, ulong blkcnt, ulong *buffer)
{
	return ide_read_write (device, blknr, blkcnt, buffer, MV_FALSE);
}

/* ------------------------------------------------------------------------- */

/*
 * copy src to dest, skipping leading and trailing blanks and null
 * terminate the string
 * "len" is the size of available memory including the terminating '\0'
 */
static void ident_cpy (unsigned char *dst, unsigned char *src, unsigned int len)
{
	unsigned char *end, *last;

	last = dst;

	/* Make sure lenis multiple of 2 + 1 for '\0'*/
	if( !(len % 2) || (len == 0))
		goto OUT;

	end  = src + len - 1;

	while ((*src) && (src<end))  {
		if(((unsigned int)src % 2) == 0)
		{
			#if defined(__BE)
			*dst = *src;
			#else
			*(dst+1) = *src;
			#endif
		}
		else 
		{
			#if defined(__BE)
			*(dst+1) = *src;
			#else
			*dst = *src;
			#endif
			
			dst+=2;
			last = dst;
			
		}
		src++;
	}
OUT:
	*last = '\0';
}



/* ------------------------------------------------------------------------- */


U_BOOT_CMD(
	ide,  5,  1,  do_ide,
	"ide     - IDE sub-system\n",
	"reset - reset IDE controller\n"
	"ide info  - show available IDE devices\n"
	"ide device [dev] - show or set current device\n"
	"ide part [dev] - print partition table of one or all IDE devices\n"
	"ide read  addr blk# cnt\n"
	"ide write addr blk# cnt - read/write `cnt'"
	" blocks starting at block `blk#'\n"
	"    to/from memory address `addr'\n"
);

U_BOOT_CMD(
	diskboot,	3,	1,	do_diskboot,
	"diskboot- boot from IDE device\n",
	"loadAddr dev:part\n"
);

#endif	/* CONFIG_COMMANDS & CFG_CMD_IDE */

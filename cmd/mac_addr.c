/*
* (C) Copyright 2008-2014 STMicroelectronics.
*
* Jean Michel SIMON <jean-michel.simon@st.com>
*
* SPDX-License-Identifier:	GPL-2.0+
*************************************************************/

#include <common.h>
#include <command.h>

#define NVS_LITE_PUBLIC_0_BASE_ADDRESS       0x08A60000   // Cannes2
#define NVS_LITE_FUSE_START                  0x800        // Cannes2
#define OTP_MAC_ADDRESS 0x20
#define TEST_PASS         0     /* Test executed with result = PASS */
#define TEST_FAIL         1     /* Test executed with result = FAIL */

int OTPread(unsigned int WordNum, unsigned int *WordValue)
{
  *WordValue = *(volatile unsigned int*)(long)(NVS_LITE_PUBLIC_0_BASE_ADDRESS+ NVS_LITE_FUSE_START+WordNum);
  return(TEST_PASS);
}

int ReadMacAddress(char *StringMacAddress)
{
  int result = TEST_PASS;
  unsigned int MacAddress=0;

  result = OTPread(OTP_MAC_ADDRESS,&MacAddress);
  memset(StringMacAddress,0,sizeof(StringMacAddress));

  if((MacAddress == 0) || MacAddress == 0xffffffff )
  {
    printf("MAC address not burnt\n");
    sprintf(StringMacAddress,"00:00:%02x:%02x:%02x:%02x",(MacAddress >> 24) ,(MacAddress >> 16) & 0xFF ,(MacAddress >> 8) & 0xFF ,(MacAddress) & 0xFF );
    return(TEST_FAIL);
  }
  if((MacAddress >> 24) != 0xe1)
  {
    printf("It is not a ST Micro MAC address\n");
    printf("The prefix for a ST Micro MAC address has to be 00:80:e1:........ \n");
    result = TEST_FAIL;
  }
  sprintf(StringMacAddress,"00:80:%02x:%02x:%02x:%02x",(MacAddress >> 24) ,(MacAddress >> 16) & 0xFF ,(MacAddress >> 8) & 0xFF ,(MacAddress) & 0xFF );
  return(result);
}

int do_set_mac_address(cmd_tbl_t * cmd, int flag, int argc, char * const argv[])
{
  int result = TEST_PASS;
  char StringMacAddressToRead[18];

  result = ReadMacAddress(StringMacAddressToRead);

  if(result == TEST_PASS)
  {
    setenv("ethaddr",StringMacAddressToRead);
    printf("The MAC address set for ethaddr is: %s\n",StringMacAddressToRead);
  }
  else
  {
    printf("Not possible to set ethaddr\n");
  }
  return result;
}

/******************************************************************************
* gpio command declaration
******************************************************************************/
U_BOOT_CMD(readmac, 1,1, do_set_mac_address,"Read MAC address and set it to ethaddr","")

/****************************************************************************
 **
 ** Copyright (C) 2010 Greg Green
 ** Contact: ggreen@bit-builder.com
 **
 ** GNU General Public License
 ** This file may be used under the terms of the GNU
 ** General Public License version 3.0 as published by the Free Software
 ** Foundation and appearing in the file LICENSE.GPL included in the
 ** packaging of this file.  Please review the following information to
 ** ensure the GNU General Public License version 3.0 requirements will be
 ** met: http://www.gnu.org/copyleft/gpl.html.
 **
 ****************************************************************************/

#ifndef CANAERO_DEFS_H_
#define CANAERO_DEFS_H_

// CAN Aerospace id definitions
#define CANAERO_EMERG_END 0x7f
#define CANAERO_HI_SVC_START 0x80
#define CANAERO_HI_SVC_END 0xc7
#define CANAERO_LO_SVC_START 0x7d0
#define CANAERO_LO_SVC_END 0x7ef

// data types
#define NODATA     0x00
#define ERROR      0x01
#define FLOAT      0x02
#define LONG       0x03
#define ULONG      0x04
#define BLONG      0x05
#define SHORT      0x06
#define USHORT     0x07
#define BSHORT     0x08
#define CHAR       0x09
#define UCHAR      0x0A
#define BCHAR      0x0B
#define SHORT2     0x0C
#define USHORT2    0x0D
#define BSHORT2    0x0E
#define CHAR4      0x0F
#define UCHAR4     0x10
#define BCHAR4     0x11
#define CHAR2      0x12
#define UCHAR2     0x13
#define BCHAR2     0x14
#define MEMID      0x15
#define CHKSUM     0x16
#define ACHAR      0x17
#define ACHAR2     0x18
#define ACHAR4     0x19
#define CHAR3      0x1A
#define UCHAR3     0x1B
#define BCHAR3     0x1C
#define ACHAR3     0x1D
#define DOUBLEH    0x1E
#define DOUBLEL    0x1F
/*
 #define RESVD      0x20
 ....
 #define RESVD      0x63

 #define UDEF       0x64
 ....
 #define UDEF       0xFF
*/

// classify message types
enum CanAeroMsgType {EED, NOD, NSH, NSL, DSD, UDL};

// return the size in bytes of a CAN Aerospace data type
inline int canaero_data_len(int ty) 
{
	switch (ty) {
	case NODATA:
		return 0;
	case CHAR:
	case UCHAR:
	case BCHAR:
	case ACHAR:
		return 1;
	case SHORT:
	case USHORT:
	case BSHORT:
	case CHAR2:
	case UCHAR2:
	case BCHAR2:
	case ACHAR2:
		return 2;
	case CHAR3:
	case UCHAR3:
	case BCHAR3:
	case ACHAR3:
		return 3;
	case ERROR:
	case FLOAT:
	case LONG:
	case ULONG:
	case BLONG:
	case SHORT2:
	case USHORT2:
	case BSHORT2:
	case CHAR4:
	case UCHAR4:
	case BCHAR4:
	case MEMID:
	case CHKSUM:
	case ACHAR4:
	case DOUBLEH:
	case DOUBLEL:
		return 4;
	}
	return 0;
}

#endif	// CANAERO_DEFS_H_

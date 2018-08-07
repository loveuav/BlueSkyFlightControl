#ifndef _ULOG_H_
#define _ULOG_H_

#include "mathTool.h"

enum ULogMessageType
{
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	INFO_MULTIPLE = 'M',
	PARAMETER = 'P',
	ADD_LOGGED_MSG = 'A',
	REMOVE_LOGGED_MSG = 'R',
	SYNC = 'S',
	DROPOUT = 'O',
	LOGGING = 'L',
	FLAG_BITS = 'B',
};

#pragma pack (1) 

typedef struct 
{
	uint8_t  magic[8];
	uint64_t timestamp;
}ulog_file_header_s;

#define ULOG_MSG_HEADER_LEN 3
typedef struct 
{
	uint16_t msg_size;
	uint8_t  msg_type;
}ulog_message_header_s;

#pragma pack () 

void UlogWriteHeader(void);

#endif



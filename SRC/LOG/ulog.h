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
} ulog_file_header_s;

#define ULOG_MSG_HEADER_LEN 3
typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
} ulog_message_header_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    char     format[2096];
} ulog_message_format_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  multi_id;
    uint16_t msg_id;
    char     message_name[255];
} ulog_message_add_logged_s;

typedef struct {
    uint16_t msg_size;
    uint8_t  msg_type;
    uint16_t msg_id;
} ulog_message_remove_logged_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  sync_magic[8];
} ulog_message_sync_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint16_t duration;
} ulog_message_dropout_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint16_t msg_id;
} ulog_message_data_header_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  key_len;
    char     key[255];
} ulog_message_info_header_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type ;
    uint8_t  is_continued;
    uint8_t  key_len;
    char     key[255];
} ulog_message_info_multiple_header_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  log_level;
    uint64_t timestamp;
    char     message[128];
} ulog_message_logging_s;

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  key_len;
    char     key[255];
} ulog_message_parameter_header_s;


#define ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK (1<<0)

typedef struct
{
    uint16_t msg_size;
    uint8_t  msg_type;
    uint8_t  compat_flags[8];
    uint8_t  incompat_flags[8];
    uint64_t appended_offsets[3];
} ulog_message_flag_bits_s;

typedef struct
{
    char* data_type;
    char* data_name;
} ULOG_FORMAT_t;

#pragma pack ()

void UlogWriteHeader(void);
void UlogWriteFlag(void);
void UlogWriteFormat(char* formatType, ULOG_FORMAT_t* ulog_format, int16_t dataNum);
void UlogWriteAddLogged(char* logType, uint8_t msg_id);

void UlogWriteData_Flight(void);
void UlogWriteData_GPS(void);

#endif



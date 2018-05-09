/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "string.h"
#include "version.h"


extern uint8_t BlockBuf[];

// Pointers to substitution strings
const char *fw_version = (const char *)FW_BUILD;


uint8_t string_auth[25 + 4];
uint8_t string_auth_descriptor[2+25*2];


uint8_t get_len_string_interface(void) {
    return 2 + strlen((const char *)(string_auth+4))*2;
}

uint8_t * get_uid_string_interface(void) {
    return string_auth_descriptor;
}

const uint8_t board_secret[9] = "k4flu5fs";

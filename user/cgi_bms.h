#ifndef __CGI_BMS_HH__
#define __CGI_BMS_HH__

uint16_t ICACHE_FLASH_ATTR bms_value_get(uint8_t wordno);
void bms_rx_data(uint8_t *, int);

#endif


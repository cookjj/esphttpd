/*
Some random cgi routines. Used in the LED example and the page that returns the entire
flash as a binary. Also handles the hit counter on the main page.
*/

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */


#include <esp8266.h>
#include "cgi.h"
#include "comm_bms.h"
#include "io.h"


//cause I can't be bothered to write an ioGetLed()
static char currLedState=0;

//Cgi that turns the LED on or off according to the 'led' param in the POST data
int ICACHE_FLASH_ATTR cgiLed(HttpdConnData *connData) {
	int len;
	char buff[1024];
	
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "led", buff, sizeof(buff));
	if (len!=0) {
		currLedState=atoi(buff);
		ioLed(currLedState);
	}

	httpdRedirect(connData, "led.tpl");
	return HTTPD_CGI_DONE;
}



//Template code for the led page.
int ICACHE_FLASH_ATTR tplLed(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	os_strcpy(buff, "Unknown");
	if (os_strcmp(token, "ledstate")==0) {
		if (currLedState) {
			os_strcpy(buff, "on");
		} else {
			os_strcpy(buff, "off");
		}
	}
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}



int ICACHE_FLASH_ATTR
cgiBms(HttpdConnData *connData) {
	int len_tot;
	char buff[1024];
    char single_val_buf[8];

	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}


    httpdStartResponse(connData, 200);
    httpdHeader(connData, "Content-Type", "text/json");
    httpdEndHeaders(connData);

    /* new JSON structure:
     * { "bms_values" : [ 5, 6, 7, 8, 999 ] }
     */
    len_tot = sprintf(buff, "{\"bms_values\":[");
    int i, numel;
    numel = bms_numel_get();
    for(i = 0; i < numel - 1; i++) {
        len_tot += sprintf(single_val_buf, "%d,", bms_value_get(i));
        strcat(buff, single_val_buf);
    }
    if(numel > 1) {
        // no comma after printed number for last element.
        len_tot += sprintf(single_val_buf, "%d", bms_value_get(numel-1));
        strcat(buff, single_val_buf);
    }
    len_tot += sprintf(single_val_buf, "]}");
    strcat(buff, single_val_buf);

    httpdSend(connData, buff, len_tot);
    return HTTPD_CGI_DONE;
}


static long hitCounter=0;

//Template code for the counter on the index page.
int ICACHE_FLASH_ATTR tplCounter(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	if (os_strcmp(token, "counter")==0) {
		hitCounter++;
		os_sprintf(buff, "%ld", hitCounter);
	}
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

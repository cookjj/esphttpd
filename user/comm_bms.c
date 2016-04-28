/* JJC 2016 */

#include <stdint.h>
#include <esp8266.h>
#include "httpd.h"
#include "comm_bms.h"
#include "io.h"

/* this attribute is essential for global storage arrays */
#define ICACHE_STORE_ATTR __attribute__((aligned(4)))


/* How many param data _words_ we can deal with max: keeps Rx buffer under 256 bytes */
#define BMS_DATA_PARM_VALS_MAX 120

/* 2 bytes per word and 2 chars per byte in serial ascii received data */
#define MAX_DATA_LEN_BYTES (BMS_DATA_PARM_VALS_MAX * 4)

/* Double buffer of param data words */
uint16_t ICACHE_STORE_ATTR param_values[2][BMS_DATA_PARM_VALS_MAX];
static uint8_t actual_numel[2] = {0}; /* actual num elements in buf 0, 1 */
static uint8_t the_good_buffer = 0; /* 0 or 1 for which param_values buffer is complete */

struct ip_info ipconfig;
//struct mdns_info info;// = (struct mdns_info *)os_zalloc(sizeof(struct mdns_info));

#define RX_SOT '<' /* start of text for serial protocol */
#define RX_EOT '>' /* end of text for serial protocol */

#define ARRAY_OFST_WIFI_CFG 0 /* 0th byte in message array for wifi options */
#define WIFICFG_IN_MSB true

#define SSID_MAX_CHARS 32u
#define SSID_PW_MAX_CHARS 64u
//#define ARRAY_OFST_SSID_CFG 14
//#define ARRAY_OFST_SSID_PW_CFG (ARRAY_OFST_SSID_CFG + 16)

#define SZ_NODE_PREFIX "vinci_"
//static char my_nodename[SSID_MAX_CHARS];
static char join_ssid[SSID_MAX_CHARS+1]; // max + a null byte for sys usage
static char join_ssid_pw[SSID_PW_MAX_CHARS+1]; // max + a null byte for internal use

/* this structure reflects wifi config byte of BMS->ESP communication */
#if 0
typedef union {
    struct __attribute__((packed)) {
        unsigned dev_num : 7;
        unsigned dev_mode : 1;
    };
    uint8_t byte;
} dev_wifi_cfg_t;
/* We get these two options from BMS for AP v. STA mode */
#define MSG_DEVMODE_STATION 0
#define MSG_DEVMODE_AP 1
#endif
typedef union {
    uint8_t changing_wifi_mode;
    struct __attribute__((packed)) {
        unsigned ofst_or_chan : 7;
        unsigned sending_ssid : 1; // sending SSID means we station joining SSID
    };
} bms_wifi_cfg_t;
#define BMS_MSG_DEVMODE_STA 1 // .sending_ssid is flagged basically
#define BMS_MSG_DEVMODE_AP  0




enum sys_wifi_mode {
    WM_NULL_MODE,   // NULL
    WM_STATION_MODE,    // Station
    WM_SOFTAP_MODE,     // Access Point
    WM_STATIONAP_MODE,  // Station + Access Point
};

enum {
    CONNTRY_IDLE,
    CONNTRY_WORKING,
    CONNTRY_SUCCESS,
    CONNTRY_FAIL
};

static int sta_connect_status = CONNTRY_IDLE;
static struct station_config stconf; // Station setup struct
static os_timer_t raTimer;
static os_timer_t rstTimer;

uint8_t static hexchar_to_nyb(uint8_t);
void static convert_rx_to_bin_and_store(uint8_t *, uint8_t);
static void ICACHE_FLASH_ATTR bms_wifi_sta_connect(void);
void ICACHE_FLASH_ATTR bms_wifi_cfg_update(void);

static void ICACHE_FLASH_ATTR rstTimerCb(void *arg);
static void ICACHE_FLASH_ATTR raTimerCb(void *arg);

bool once = false;

/******************************************************************************/

uint8_t ICACHE_FLASH_ATTR
bms_numel_get(void)
{
    return actual_numel[the_good_buffer];
}

uint16_t
ICACHE_FLASH_ATTR 
bms_value_get(uint8_t wordno)
{
    if(wordno > BMS_DATA_PARM_VALS_MAX) {return 0;}
    if(the_good_buffer == 0) {
        return param_values[0][wordno];
    } else {
        return param_values[1][wordno];
    }
}


uint8_t
hexchar_to_nyb(uint8_t hc)
/* behaviour: return 0 if not a valid char for hexadecimal */
/* return value guaranteed to not exceed 0xF */
{
    if(hc >= '0' && hc <= '9') {
        return (hc - '0');
    } else if(hc >= 'A' && hc <= 'F') {
        return 10 + (hc - 'A');
    } else if(hc >= 'a' && hc <= 'f') {
        return 10 + (hc - 'a');
    } else {
        return 0;
    }
}


void
convert_rx_to_bin_and_store(uint8_t *buf, uint8_t len)
{
    // len must be divisibe by 4
    if((len & 0b11) != 0) {
        printf("BMS: convert: len not equal 0 (mod 4)");
        return;
    }

    uint8_t i, dirty_buffer;
    uint8_t parm_vals_idx;

    // We'll double buffer, writing to 'dirty_buffer',
    // and not the valid one which may be read out to web by cgi.
    if(the_good_buffer == 0) {
        dirty_buffer = 1;
    } else {
        dirty_buffer = 0;
    }

    parm_vals_idx = 0;

    for(i = 0; i < len; i += 4) { // increment 4 chars for 4 nybbles for i word
        uint16_t build_a_word; // we'll construct saved word here
        uint8_t nybble; // temporary

        if(parm_vals_idx >= BMS_DATA_PARM_VALS_MAX) {
            // Break from loop to avoid overflow buffer if
            // we've gone past the end of param_values
            break;
        }

        // Most significant data comes first.

        nybble = hexchar_to_nyb(buf[i]);
        // promote nybble to 16 bits for shift and assignment
        build_a_word = ((uint16_t)nybble) << (12); // assign shifted most signif nybble

        nybble = hexchar_to_nyb(buf[i+1]);
        build_a_word |= ((uint16_t)nybble) << (8); // OR shifted next signif nybble

        nybble = hexchar_to_nyb(buf[i+2]);
        build_a_word |= ((uint16_t)nybble) << (4); // OR shifted penult signif nybble

        nybble = hexchar_to_nyb(buf[i+3]);
        build_a_word |= (uint16_t)nybble;   // OR least signif nybble

        param_values[dirty_buffer][parm_vals_idx] = build_a_word;
        parm_vals_idx++; // increase to next position to write
    }
    actual_numel[dirty_buffer] = parm_vals_idx;

    /* Switch the_good_buffer to the freshly written one;
     * This should be the only place it is modified,
     * so it can be read safely elsewhere without locks. */
    if(the_good_buffer == 0) {
        the_good_buffer = 1;
    } else {
        the_good_buffer = 0;
    }

    bms_wifi_cfg_update();

    return;
}


void
ICACHE_FLASH_ATTR
bms_wifi_cfg_update(void)
{
//    uint8_t cur_mode, msg_mode;
    struct softap_config c;

    bms_wifi_cfg_t bwc;
    bwc.changing_wifi_mode = (uint8_t)(param_values[the_good_buffer][ARRAY_OFST_WIFI_CFG] >> 8);
    if(bwc.changing_wifi_mode == 0) { // zero indicates no change
        printf("No mode change req.");
        return;
    }

    printf("cmd byte is %02x with sending_ssid .eq. %d and else %d\n", bwc.changing_wifi_mode, bwc.sending_ssid, bwc.ofst_or_chan);

    if(bwc.sending_ssid != 0) { // force chg to STAtion mode, look for SSID to join

        printf("\nstation plz.\n");

        // ensure the offset to read SSID from actually came thru in msg
        if(bwc.ofst_or_chan > actual_numel[the_good_buffer]) {
            printf("Skipping SSID join, actual numel is %d, but offset is %d\n", actual_numel[the_good_buffer], bwc.ofst_or_chan);
            return;
        }

        char *ssid_begin, *pw_begin;
        ssid_begin = (char *) &param_values[the_good_buffer][bwc.ofst_or_chan];
        pw_begin   = (char *) &param_values[the_good_buffer][bwc.ofst_or_chan+(SSID_MAX_CHARS / 2)];

        memcpy(join_ssid, ssid_begin, SSID_MAX_CHARS);
        memcpy(join_ssid_pw, pw_begin, SSID_PW_MAX_CHARS);
        join_ssid[SSID_MAX_CHARS] = 0; // null terminate last position (@ index 32)
        join_ssid_pw[SSID_PW_MAX_CHARS] = 0; // null terminate just in case (id)
        printf("saving in RAM the SSID: '%s' w/ pw '%s'\n", join_ssid, join_ssid_pw); 

        /* change to station mode if not busy trying already */
        //if(sta_connect_status == CONNTRY_IDLE) {
            printf("station connect idle, try connect...\n");
            bms_wifi_sta_connect(); // makes use of arrays 'join_ssid'
        //}

    } else { // become soft AP
        printf("softAP mode plz.  ");
        /* Try get AP config details */
        if(wifi_softap_get_config(&c)) {
            printf("softAP:got config. Make channel %d\n", bwc.ofst_or_chan);

            // adjust config
            c.channel = bwc.ofst_or_chan; // assign desired wifi channel

            /* set softAP config */
            wifi_softap_set_config(&c);
            /* change major mode */
            wifi_set_opmode(WM_SOFTAP_MODE);
            /* restart wifi */
            system_restart();
        }
    }

    return;
}

//This routine is run some time after a connection attempt to an access point. If
//the connect succeeds, this gets the module in STA-only mode.
static void ICACHE_FLASH_ATTR
rstTimerCb(void *arg)
{
    int x = wifi_station_get_connect_status();
    if (x == STATION_GOT_IP) {
        //Go to STA mode. This needs a reset, so do that.
        httpd_printf("Got IP. Going into STA mode..\n");

        wifi_station_set_reconnect_policy(true);
        wifi_station_set_auto_connect(true);
        system_restart();
    } else {
//        sta_connect_status=CONNTRY_FAIL;
        sta_connect_status = CONNTRY_IDLE; // will trigger retry in main communications function --jjc
        httpd_printf("Connect fail.\n");
    }
}



static void ICACHE_FLASH_ATTR
raTimerCb(void *arg)
{
    int x;
    httpd_printf("Try to connect to AP %s\n", stconf.ssid);

    wifi_set_opmode(WM_STATION_MODE);

    wifi_station_disconnect();

    wifi_station_set_config(&stconf);
    wifi_station_connect();

    sta_connect_status = CONNTRY_WORKING;

    x = wifi_get_opmode();
    if(x != WM_STATION_MODE) {
        //Schedule disconnect/connect
        os_timer_disarm(&rstTimer);
        os_timer_setfn(&rstTimer, rstTimerCb, NULL);
        os_timer_arm(&rstTimer, 15000, 0); //time out after 15 secs of trying to connect
    }
    return;
}


static void ICACHE_FLASH_ATTR
bms_wifi_sta_connect(void)
/* start trying to connect to an access point */
{
    sta_connect_status = CONNTRY_WORKING;
    printf("Saving station values to stconf...\n");
    memcpy((char *)stconf.ssid, join_ssid, 32); // need zero termination string for OS
    memcpy((char *)stconf.password, join_ssid_pw, 64);// need zero term str
    stconf.bssid_set = false; // see user_interface.h for more info

    //Schedule disconnect/connect
    os_timer_disarm(&raTimer);
    os_timer_setfn(&raTimer, raTimerCb, NULL);
    os_timer_arm(&raTimer, 500, 0);

    return;
}

void
bms_rx_data(uint8_t *data, int len)
{
    static uint8_t rx_buf[512];  /* Serial protocol receive buffer. Only data text and not text control values are stored */
    static uint8_t rx_idx = 0;  /* byte count for stored data text values in rx_buf */
    static uint8_t got_eot = 0; /* flag if EOT char has been received */
    static uint8_t check;       /* xor additive checksum of rx_buf bytes */

    uint8_t i;
    for(i = 0; i < len; i++) {
        printf("%c", data[i]);
        if(data[i] == RX_SOT) {
            // Reset everything on SOT
            rx_idx = 0;
            check = 0;
            got_eot = 0;
            //printf("BMS: got SOT: %c", data[i]);
        } else if(data[i] == RX_EOT) {
            // Just flag that we got EOT
            got_eot = 1;
            //printf("BMS: got EOT");
        } else {
            // Got a non SOT/EOT value...
            // and EOT hasn't happened
            if(got_eot == 0) {
                // Let's see if its a good data char
                if((data[i] >= '0' && data[i] <= '9')
                || (data[i] >= 'A' && data[i] <= 'F')) { // Require capitals A~F for hex, no lower

                    //printf("BMS: got data char %c", data[i]);

                    // Store, add to checksum, increment buffer index; conversion on full message rxd.
                    // Only increment to last valid place
                    if(rx_idx < MAX_DATA_LEN_BYTES) {
                        check ^= data[i];
                        rx_buf[rx_idx] = data[i];
                        rx_idx++;
                    } else {
                        //printf("BMS: avoided buffer overflow");
                        // remain on last place in buffer.
                        // ignore byte here instead of overflowing buffer
                    }
                } else {
                    //printf("BMS: ascii hex out of range");
                    // Out of range for ascii hex, ignore it.
                }
            } else { // got end of text, this must be check byte
                printf("BMS: Getting check byte");

                if(data[i] == check) { // and good check sum
                    // convert ascii to binary and copy from rx_buf
                    // to param buffer; switch good buffer.
                    // note that we send rx_buf, rx_idx,
                    // and definitely not the partial message loop parameters data,len
                    printf("Good check.\n");
                    convert_rx_to_bin_and_store(rx_buf, rx_idx);
                } else {
                    printf("Bad check.\n");
                }

#if 0
                if(rx_idx == EXPECTED_DATA_LEN_BYTES) { // and enough data bytes came in
                    //printf("BMS: got expected data len");
                    if(data[i] == check) { // and good check sum
                        // convert ascii to binary and copy from rx_buf
                        // to param buffer; switch good buffer.
                        // note that we send rx_buf, rx_idx,
                        // and definitely not the partial message loop parameters data,len
                        printf("Good check.\n");
                        convert_rx_to_bin_and_store(rx_buf, rx_idx);
                    } else {
                        // bad: nothing will happen until SOT again
                        //printf("BMS: got bad checksum sent:%02x vs. calc:%02x", data[i], check);
                    }
                } else {
                    // bad: nothing will happen until SOT again
                    //printf("BMS: got unexpected data len, try again!");
                }
#endif
            }
        }
    }

    return;
}



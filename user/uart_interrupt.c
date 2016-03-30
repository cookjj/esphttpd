#include <esp8266.h>
#include "ets_sys.h"
#include "osapi.h"
//#include "driver/uart.h"
#include "uart_hw.h"
#include "osapi.h"
//#include "driver/uart_register.h"
#include "user_config.h"
#include "mem.h"
#include "os_type.h"

#define DEBUG_UART 0

// UartDev is defined and initialized in rom code.
extern UartDevice UartDev;

#define UART_RX_INTR_DISABLE(uart) CLEAR_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA|UART_RXFIFO_TOUT_INT_ENA)
#define UART_RX_INTR_ENABLE(uart) SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_RXFIFO_FULL_INT_ENA|UART_RXFIFO_TOUT_INT_ENA)
#define UART_TX_INTR_DISABLE(uart) CLEAR_PERI_REG_MASK(UART_INT_ENA(uart), UART_TXFIFO_EMPTY_INT_ENA)
#define UART_TX_INTR_ENABLE(uart) SET_PERI_REG_MASK(UART_CONF1(uart), (UART_TX_EMPTY_THRESH_VAL & UART_TXFIFO_EMPTY_THRHD)<<UART_TXFIFO_EMPTY_THRHD_S); \
                           SET_PERI_REG_MASK(UART_INT_ENA(uart), UART_TXFIFO_EMPTY_INT_ENA)

#define UART_RESET_FIFO(uart) SET_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST);   \
                       CLEAR_PERI_REG_MASK(UART_CONF0(uart), UART_RXFIFO_RST | UART_TXFIFO_RST)

#define UART_CLEAR_ALL_INTR(uart) WRITE_PERI_REG(UART_INT_CLR(uart), 0xffff)
#define UART_CLEAR_INTR(uart,INTERRUPT) WRITE_PERI_REG(UART_INT_CLR(uart), INTERRUPT)
#define UART_INTERRUPT_IS(uart,INTERRUPT) INTERRUPT == (READ_PERI_REG(UART_INT_ST(uart)) & INTERRUPT)

#define UART_RX_FIFO_COUNT(uart) (READ_PERI_REG(UART_STATUS(uart))>>UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT
#define UART_TX_FIFO_COUNT(uart) (READ_PERI_REG(UART_STATUS(uart))>>UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT

#define UART0_READ_CHAR() READ_PERI_REG(UART_FIFO(UART0)) & 0xFF
#define UART_WRITE_CHAR(uart,c) WRITE_PERI_REG(UART_FIFO(uart), c)

static uart0_data_received_callback_t uart0_data_received_callback=NULL;

void uart_rx_intr_handler(void *para);
void uart0_data_received(void);


//UART TRANSMIT ------------------------------------------------------------

void ICACHE_FLASH_ATTR
uart_config(uint8_t uart_no)
{
    if (uart_no == UART1){
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    }else{
        /* rcv_buff size if 0x100 */
        ETS_UART_INTR_ATTACH(uart_rx_intr_handler,  &(UartDev.rcv_buff));
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
        #if UART_HW_RTS
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);   //HW FLOW CONTROL RTS PIN
            #endif
        #if UART_HW_CTS
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_U0CTS);   //HW FLOW CONTROL CTS PIN
        #endif
    }
    uart_div_modify(uart_no, UART_CLK_FREQ / (UartDev.baut_rate));//SET BAUDRATE
    
    WRITE_PERI_REG(UART_CONF0(uart_no), ((UartDev.exist_parity & UART_PARITY_EN_M)  <<  UART_PARITY_EN_S) //SET BIT AND PARITY MODE
                                                                        | ((UartDev.parity & UART_PARITY_M)  <<UART_PARITY_S )
                                                                        | ((UartDev.stop_bits & UART_STOP_BIT_NUM) << UART_STOP_BIT_NUM_S)
                                                                        | ((UartDev.data_bits & UART_BIT_NUM) << UART_BIT_NUM_S));
    
    //clear rx and tx fifo,not ready
    UART_RESET_FIFO(uart_no);
    
    if (uart_no == UART0){
        //set rx fifo trigger
        WRITE_PERI_REG(UART_CONF1(uart_no),
        ((100 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
        #if UART_HW_RTS
        ((110 & UART_RX_FLOW_THRHD) << UART_RX_FLOW_THRHD_S) |
        UART_RX_FLOW_EN |   //enbale rx flow control
        #endif
        (0x60 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S |
        UART_RX_TOUT_EN|
        ((0x10 & UART_TXFIFO_EMPTY_THRHD)<<UART_TXFIFO_EMPTY_THRHD_S));//wjl 
        #if UART_HW_CTS
        SET_PERI_REG_MASK( UART_CONF0(uart_no),UART_TX_FLOW_EN);  //add this sentense to add a tx flow control via MTCK( CTS )
        #endif
        SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_TOUT_INT_ENA |UART_FRM_ERR_INT_ENA);
    }else{
        WRITE_PERI_REG(UART_CONF1(uart_no),((UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));//TrigLvl default val == 1
    }
    //clear all interrupt
    WRITE_PERI_REG(UART_INT_CLR(uart_no), 0xffff);
    //enable rx_interrupt
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_FULL_INT_ENA|UART_RXFIFO_OVF_INT_ENA);
}
 

void
uart0_data_received(void)
{
    uint8_t data_len;
    data_len = UART_RX_FIFO_COUNT(0);
    
    if(data_len < 1u) {
        return;
    }

    uint8_t *tmp_data = (uint8_t *)os_malloc(data_len);
    int i;
    uint8_t c;
    for(i=0; i < data_len; i++) {
        c = UART0_READ_CHAR();
        tmp_data[i]=c;
    }

    if(uart0_data_received_callback != NULL) {
        uart0_data_received_callback(tmp_data, data_len);
    }
    os_free(tmp_data);

    return;
}


void
uart_rx_intr_handler(void *para)
{    

    if( UART_INTERRUPT_IS(0,UART_FRM_ERR_INT_ST) ){
        //just clear intr
        UART_CLEAR_INTR(0,UART_FRM_ERR_INT_CLR);
        return;
    }
    if( UART_INTERRUPT_IS(1,UART_FRM_ERR_INT_ST) ){
        //just clear intr
        UART_CLEAR_INTR(1,UART_FRM_ERR_INT_CLR);
        return;
    }


    
    if( UART_INTERRUPT_IS(0,UART_RXFIFO_FULL_INT_ST) ){
        
        //got data on rx fifo
        UART_RX_INTR_DISABLE(0); //disable rx interrupt

        uart0_data_received();

        UART_CLEAR_INTR(0,UART_RXFIFO_FULL_INT_CLR); //clear interrupt
        UART_RX_INTR_ENABLE(0); //enable interrupt back
        return;
    }
    
    if( UART_INTERRUPT_IS(0,UART_RXFIFO_TOUT_INT_ST) ){
        
        //got data on uart 0 rx fifo, timeout for fifo full
        UART_RX_INTR_DISABLE(0); //disable rx interrupt

        uart0_data_received();

        UART_CLEAR_INTR(0,UART_RXFIFO_TOUT_INT_CLR); //clear interrupt
        UART_RX_INTR_ENABLE(0); //enable interrupt back
        return;
    }

    if( UART_INTERRUPT_IS(0,UART_RXFIFO_OVF_INT_ST) ){      
        
        UART_CLEAR_INTR(0,UART_RXFIFO_OVF_INT_CLR); //clear interrupt
        return;
    }

    

}

void uart_register_data_callback(uart0_data_received_callback_t callback){

    uart0_data_received_callback=callback;  
}
void uart_clear_data_callback(){

    uart0_data_received_callback=NULL;  
}


void uart_init(UartBautRate uart0_br, UartBautRate uart1_br)
{
    UartDev.exist_parity = STICK_PARITY_EN;
    UartDev.parity = EVEN_BITS;
    UartDev.stop_bits = ONE_STOP_BIT;
    UartDev.data_bits = EIGHT_BITS;

    UartDev.baut_rate = uart0_br;
    uart_config(UART0);

    #ifndef ETS_UART_INTR_ENABLE
    #error "Macro function uart int enab not included, please fixme in user_main."
    #endif
    ETS_UART_INTR_ENABLE();

    return;
}




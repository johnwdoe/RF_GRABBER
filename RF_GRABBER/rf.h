/*
 * rf.h
 *
 *  Created on: Apr 23, 2018
 *      Author: william
 */

#ifndef RF_H_
#define RF_H_

#include <stdint.h>
#define BUFFER_LEN (128)

#define SILENCE_BEFORE_START_US (5000UL) //less than minimum sync period - 11520us
#define SILENCE_BEFORE_STOP_US (30000UL) //more than maximum sync period - 25200us
#define T_DIV_CLK (8)

#define F_GRAB (1)
#define F_WAIT_START_SIL (2)
#define F_RX_COMPLETE (4)
#define F_TX_COMPLETE (8)

volatile uint8_t rf_flags;
uint16_t rf_buf[BUFFER_LEN]; //rx/tx buffer
volatile uint8_t rf_buf_cursor; //position on last sig. buffer cell
/*
 * asynchronous
 * wait for silence and start writing ticks to buf (HLHL...)
 * stops when:
 * 1. buffer full
 * 2. long silence after writing tick begin
 * 3. executing rf_grab_stop manually
 * */
void rf_grab_start(void);

/*
 * manually stop scanning
 */
void rf_grab_stop(void);

/*
 * asynchronous start transfer periods from buf: Hbuf[0], Lbuf[1], Hbuf[2], ...
 * repeat - number of packet transmissions.
 * */
void rf_tx_start(uint8_t repeat);

/*
 * external function
 */
extern void rf_on_complete(void);

#endif /* RF_H_ */

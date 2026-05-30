/*
 * app.h
 *
 *  Created on: May 29, 2026.
 *      Author: david
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/* Runs the full bring-up sequence (peripherals must already be MX_*_Init'd). */
void App_Init(void);

void App_Loop(void);

#endif /* INC_APP_H_ */

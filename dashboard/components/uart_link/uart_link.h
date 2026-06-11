#ifndef UART_LINK_H_
#define UART_LINK_H_

#include <stddef.h>

// Start the UART driver and the background task that reads telemetry lines.
void uart_link_init(void);

// Copy the most recently received telemetry line into 'out' (NUL-terminated).
// Returns the number of bytes copied (excluding the NUL). 'out' gets "{}" if
// nothing has been received yet.
size_t uart_link_get_telemetry(char *out, size_t out_size);
void uart_link_send(const char *line);   // sends 'line' + '\n' to the robot

#endif /* UART_LINK_H_ */
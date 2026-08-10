#include "bsp_debug.h"
#include "bsp_itcallback.h"

#include "stdarg.h"

// 串口五
void DebugSendString(uint8_t *buf, uint8_t len)
{
#if DEBUG_OUTPUT_ENABLED
	/* Supply a non-UART5 transport before enabling debug output. */
#else
	(void)buf;
	(void)len;
#endif
}

void DebugPrintf(const char *format, ...)
{
	va_list args2;//调用#include "stdarg.h"
	uint32_t length2;
	uint8_t tx2buf[BUFF_MAX] = {0};

	va_start(args2, format);
	length2 = vsnprintf((char *)tx2buf, sizeof(tx2buf), (char *)format, args2);
	va_end(args2);
	DebugSendString(tx2buf,length2);
	memset(tx2buf, 0, BUFF_MAX);//调用#include "string.h"
}

















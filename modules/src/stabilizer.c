#include <stdbool.h>

#include "stabilizer.h"
#include "imu.h"

static bool isInit = false;

void stabilizerInit(void)
{
	if(isInit)
		return;
	imuInit();

	isInit = true;
}

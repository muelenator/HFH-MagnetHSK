#pragma once

#include <SPI.h>  // include the SPI library
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"

#include <configConstants.h>
#include <supportFunctions.h>

// ******************************
// ALL support functions
// ******************************
void configure_channels();
void configure_memory_table();
void configure_global_parameters();

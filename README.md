# mxusartusbcan
Test and development of sujpport routines for DiscoveryF4 w STM32CubeMX and FreeRTOS

The objective was to determine how the STM32CubeMX w FreeRTOS could be used for the GliderWinch project, and if modifications to the HAL routines would be needed.  It appears that the HAL routines along with interface routines will work, though there is significant overhead incurred compared to our earlier bare-metal.  With the processors getting faster and with more memory the overhead issue becomes less of an issue, and the ease of moving between STM32 processor lines via STM32CubeMX makes it appear--at this point in time--that the approach is viable.


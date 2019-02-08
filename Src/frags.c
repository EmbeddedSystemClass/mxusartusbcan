#ifdef BAREBONESTESTING
extern char hal_flag;
extern uint32_t Xerrors;
char cx[128];
char ctest[128];
strcpy (&ctest[0], "\n\rHELLO TEST:");
int cz = strlen(ctest);
int i;

  /* Infinite loop */
  for(;;)
  {
//while( (HAL_UART_Receive_IT(&huart6, (uint8_t*)cx,10)) != HAL_OK) osDelay(2);
HAL_UART_Receive_IT(&huart6, (uint8_t*)cx,10);
while (hal_flag == 0) osDelay(1);

for (i = 0; i < 10; i++) 
  ctest[cz+i] = cx[i];
ctest[cz+i] = 0;


//HAL_UART_Transmit_IT(&huart6, ctest,(cz+10));
//for (i = 0; i < 1; i++)
//yprintf(&bufy21, "\n\rHELLO:");
yprintf(&bufy21, "%s %c 0X%08X",ctest,hal_flag, Xerrors );
Xerrors =  0;
hal_flag =  0;
#endif

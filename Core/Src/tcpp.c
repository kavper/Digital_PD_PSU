#include "tcpp.h"
#include "main.h"

// Funkcja inicjująca TCPP01
void TCPP_Init() {
  HAL_GPIO_WritePin(DB_GPIO_Port, DB_Pin, 1);
  HAL_Delay(10);
}

void TCPP_SetPDOSelectionMethod(USBPD_DPM_PDO_SelectionMethodTypeDef method) {
  DPM_SetPDOSelectionMethod(method);
}
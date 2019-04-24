#ifndef _NUBL2LIB_H_
#define _NUBL2LIB_H_


extern int32_t (*OTA_Init)(uint32_t u32HSI, ISP_INFO_T *pISPInfo);
extern uint8_t (*OTA_SysTickProcess)(uint32_t u32Ticks);
extern void (*OTA_WiFiProcess)(void);
extern int8_t (*OTA_TaskProcess)(void);
extern int32_t (*OTA_GetBLxFwVer)(uint32_t * pu32FwVer, uint8_t u8Mode);
extern int32_t (*OTA_ForceUpdate)(void);
extern void (*OTA_SDH_Process)(void);



#endif //_NUBL2LIB_H_
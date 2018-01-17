#ifndef _AT_APP_H_
#define _AT_APP_H_

/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/

/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes
 *********************************************************************/
extern void vATApp_Init(void);
extern bool vATApp_is3GBeingUsed(void);
extern void vATApp_Make_Emergency_call(void);
extern void vATApp_Make_TestVoice_call(void);
extern void vATApp_HangUp_Call(void);
extern void vATApp_IPSEND_Excute(uint8_t const* ipData,uint16_t len,callBack cb);

extern void vATApp_Init_Active_Check(void);
extern void vATApp_Init_Deactive_Check(void);

extern void vATApp_Loop_Check(void);
extern void vATApp_GoSleep(void);
extern void vATApp_Sent_Callback(void);
extern void vATApp_WakeUp(void);

extern void vATApp_Retry_Clear(void);
//IP Send ACK to server.
extern void vATApp_IPSEND_Ack(uint8_t const* ipData,uint16_t len,callBack cb);
extern void vATApp_SMS_SEND_Excute(uint8_t const* sms_data, uint16_t len, callBack cb);
//Restart OTA check.
extern void vATApp_Restart_OTA_Check(void);

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif	/* _AT_APP_H_ */

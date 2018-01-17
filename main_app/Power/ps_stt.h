/* $Header:   ps_stt.h     $*/
/***********************************************************************

   Title                      : ps_stt.h

   Module Description         : Contains the state transition table
                                for power/operation mode synchronization
                                . This file is included at 3
                                different positions in psync.c and is used
                                to let the precompiler build 3 different tables
                                (1 enum, 2 arrays).

   Author                     : 

   Created                    : 

   Configuration ID           : 

 **********************************************************************/
TREE (tree_psync)

/*========================== 0 - PS_ROOT =========================*/

STATE (PS_ROOT,				0,				ps_cs_root)
TRANS (ENTRY,				INTERNAL,		ps_entry_root)
TRANS (START,				PS_ROOT,		ps_start_action)
STATE_END

/*========================== 1 - PS_IDLE =========================*/

STATE (PS_IDLE,				PS_ROOT,		ps_cs_idle)
TRANS (ENTRY,				INTERNAL,		ps_entry_idle)
STATE_END

/*========================== 2 - PS_AWAKE ========================*/

STATE (PS_AWAKE,			PS_ROOT,		ps_cs_awake)
TRANS (ENTRY,				INTERNAL,		ps_entry_awake)
TRANS (PS_EVT_IGN_ON,		PS_ENG_ON,	    no_action)
TRANS (PS_EVT_GO_IDLE,		PS_IDLE,		no_action)
TRANS (EXIT,				INTERNAL,		ps_exit_awake)
STATE_END

/*========================== 3 - PS_ENG_ON =====================*/

STATE (PS_ENG_ON,			PS_AWAKE,		ps_cs_eng_on)
TRANS (ENTRY,				INTERNAL,		ps_entry_eng_on)
TRANS (PS_EVT_IGN_OFF,		PS_AWAKE,	    no_action)
TRANS (EXIT,				INTERNAL,		ps_exit_eng_on)
STATE_END
TREE_END (tree_psync)

/**********************************************************************
*                                                                     *
* REVISION RECORDS                                                    *
*                                                                     *
**********************************************************************/
/*********************************************************************/
/* $Log:   ps_stt.h  $
 *
 *********************************************************************/

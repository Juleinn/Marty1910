#ifndef BM64_STRINGS_H
#define BM64_STRINGS_H

static const char * EVENT_NAMES[] = {
"Command_ACK", 
"BTM_Status", 
"Call_Status", 
"Caller_ID", 
"SMS_Received_Indication", 
"Missed_Call_Indication", 
"Phone_Max_Battery_Level", 
"Phone_Current_Battery_Level", 
"Roaming_Status", 
"Phone_Max_Signal_Strength_Level", 
"Phone_Current_Signal_Strength_Level", 
"Phone_Service_Status", 
"BTM_Battery_Status", 
"BTM_Charging_Status", 
"Reset_To_Default", 
"Report_HF_Gain_Level", 
"EQ_Mode_Indication", 
"PBAP_Missed_Call_History",
"PBAP_Received_Call_History",
"PBAP_Dialed_Call_History",
"PBAP_Combine_Call_History",
"Phonebook_Contacts",
"PBAP_Access_Finish",
"Read_Linked_Device_Information_Reply", 
"Read_BTM_Version_Reply", 
"Call_List_Report", 
"AVC_Specific_Rsp", 
"BTM_Utility_Req", 
"Vendor_AT_Cmd_Reply", 
"Report_Vendor_AT_Event", 
"Read_Link_Status_Reply", 
"Read_Paired_Device_Record_Reply", 
"Read_Local_BD_Address_Reply", 
"Read_Local_Device_Name_Reply", 
"Report_SPP/iAP_Data", 
"Report_Link_Back_Status", 
"REPORT_RING_TONE_STATUS", 
"User_Confrim_SSP_Req", 
"Report_AVRCP_Vol_Ctrl", 
"Report_Input_Signal_Level", 
"Report_iAP_Info", 
"REPORT_AVRCP_ABS_VOL_CTRL", 
"Report_Voice_Prompt_Status", 
"Report_MAP_Data",
"Security_Bonding_Res", 
"Report_Type_Codec", 
"Report_Type_BTM_Setting", 
"Report_MCU_Update_Reply", 
"Report_BTM_Initial_Status", 
"LE_ANCS_Service_Event", 
"LE_Signaling_Event", 
"Report_nSPK_Link_Status", 
"Report_nSPK_Vendor_Event", 
"Report_nSPK_Audio_Setting", 
"Report_Sound_Effect_Status", 
"Report_Vendor_EEPROM_Data", 
"REPORT_IC_VERSION_INFO", 
"REPORT_LE_GATT_EVENT"};


static const char * BTM_STATUSES[] = {
"Power OFF state"
,"Pairing state (discoverable mode)"
,"Power ON state"
,"Pairing successful"
,"Pairing failed"
,"HF/HS link established"
,"A2DP link established"
,"HF link disconnected"
,"A2DP link disconnected"
,"SCO link connected"
,"SCO link disconnected"
,"AVRCP link established"
,"AVRCP link disconnected"
,"Standard SPP connected"
,"Standard_SPP / iAP disconnected"
,"Standby state"
,"iAP connected"
,"ACL disconnected"
,"MAP connected"
,"MAP operation forbidden"
,"MAP disconnected"
,"ACL connected"};

#endif

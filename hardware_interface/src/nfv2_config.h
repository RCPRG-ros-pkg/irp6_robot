/*
 * External config file.
 * Uncomment and update buffers' size [no of param set instances] for commands
 * to be supported by module:
 */

#define NF_BroadcastAddress 0xff
#define NF_RobotAddress     0x00
#define NF_MasterAddress  	0xfe

/*
 * Uncomment and update buffers' size [no of param set instances] for commands
 * to be supported by module:
 */



#define NF_BUFSZ_NumberOfDrives 16

#define NF_BUFSZ_ReadDeviceStatus	NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDeviceVitals	NF_BUFSZ_NumberOfDrives

#define NF_BUFSZ_SetDrivesMode			NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetDrivesSpeed			NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesCurrent		NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetDrivesPosition		NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesPWM			NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesMaxCurrent	NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesCurrent		NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesPosition		NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesMisc			NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesStatus		NF_BUFSZ_NumberOfDrives

#define NF_BUFSZ_SetCurrentRegulator	NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetSpeedRegulator		NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetPositionRegulator	NF_BUFSZ_NumberOfDrives

//	#define NF_BUFSZ_SetServosMode		NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetServosPosition	NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_SetServosSpeed		NF_BUFSZ_NumberOfDrives

//	#define NF_BUFSZ_SetDigitalOutputs	NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_ReadDigitalInputs	NF_BUFSZ_NumberOfDrives
//	#define NF_BUFSZ_ReadAnalogInputs	NF_BUFSZ_NumberOfDrives

/*
 * Remember to declare:
 * extern NF_STRUCT_ComBuf	NFComBuf;
 */

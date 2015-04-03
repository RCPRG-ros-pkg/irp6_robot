/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * External config file.
 * Uncomment and update buffers' size [no of param set instances] for commands
 * to be supported by module:
 */

#ifndef NFV2_CONFIG_H_
#define NFV2_CONFIG_H_

#define NF_BroadcastAddress 0xff
#define NF_RobotAddress     0x00
#define NF_MasterAddress   0xfe

/*
 * Uncomment and update buffers' size [no of param set instances] for commands
 * to be supported by module:
 */

#define NF_BUFSZ_NumberOfDrives 16

#define NF_BUFSZ_ReadDeviceStatus NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDeviceVitals NF_BUFSZ_NumberOfDrives

#define NF_BUFSZ_SetDrivesMode   NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetDrivesSpeed   NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesCurrent  NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetDrivesPosition  NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesPWM   NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesMaxCurrent NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesCurrent  NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesPosition  NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_SetDrivesMisc   NF_BUFSZ_NumberOfDrives
#define NF_BUFSZ_ReadDrivesStatus  NF_BUFSZ_NumberOfDrives

#define NF_BUFSZ_SetCurrentRegulator NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetSpeedRegulator  NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetPositionRegulator NF_BUFSZ_NumberOfDrives

// #define NF_BUFSZ_SetServosMode  NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetServosPosition NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_SetServosSpeed  NF_BUFSZ_NumberOfDrives

// #define NF_BUFSZ_SetDigitalOutputs NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_ReadDigitalInputs NF_BUFSZ_NumberOfDrives
// #define NF_BUFSZ_ReadAnalogInputs NF_BUFSZ_NumberOfDrives

/*
 * Remember to declare:
 * extern NF_STRUCT_ComBuf NFComBuf;
 */

#endif  // NFV2_CONFIG_H_

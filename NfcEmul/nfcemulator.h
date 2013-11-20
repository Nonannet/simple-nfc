/*****************************************************************************
Written and Copyright (C) by Nicolas Kruse

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*****************************************************************************/

//#define F_CPU 13560000UL
//#define F_CPU 22000000UL

#define RFID_FREQU 13560000UL
#define CLC_PBIT (uint16_t)(128.0 * F_CPU / RFID_FREQU + 0.5)
#define CLCS CLC_PBIT * 5 / 4
#define CLCM CLC_PBIT * 7 / 4
#define CLCL CLC_PBIT * 9 / 4

#define FDT_DELAY_BD9 (uint16_t)(9.0 * 128 * F_CPU / RFID_FREQU - 1) //Nr of cycles-1 for 9 bit durations
#define FDT_DELAY_BIT0 (FDT_DELAY_BD9 + 20 - CLCL)
#define FDT_DELAY_BIT1 (FDT_DELAY_BD9 + 84 - CLCL)

#define SUBC_OVF (uint8_t)(F_CPU / 847500.0 / 2.0 + 0.5 - 1) //847500 Hz Subcarrier

#define S_IDLE 0
#define S_READY 1
#define S_ACTIVE 2
#define S_HALT 8
#define S_READY_H 9
#define S_ACTIVE_H 10

void setupNfcEmulator(uint8_t *storage, uint16_t storageSize);
void checkForNfcReader();/*****************************************************************************
Written and Copyright (C) by Nicolas Kruse

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*****************************************************************************/

//#define F_CPU 13560000UL
//#define F_CPU 22000000UL

//For ATiny*4:
#define AIN1_PORT 2       //Port for OC0A on ATiny*4 is PA2
#define AIN1_PRNG PORTA  

//For ATiny*5:
//#define AIN1_PORT 1      //Port for OC0A on ATiny*5 is PB1
//#define AIN1_PRNG PORTB

#define RFID_FREQU 13560000UL
#define CLC_PBIT (uint16_t)(128.0 * F_CPU / RFID_FREQU + 0.5)
#define CMIN CLC_PBIT * 3 / 4
#define CLCS CLC_PBIT * 5 / 4
#define CLCM CLC_PBIT * 7 / 4
#define CLCL CLC_PBIT * 9 / 4

#define FDT_DELAY_BD9 (uint16_t)(9.0 * 128 * F_CPU / RFID_FREQU - 1) //Nr of cycles-1 for 9 bit durations
#define FDT_DELAY_BIT0 (FDT_DELAY_BD9 + 20 - CLCL)
#define FDT_DELAY_BIT1 (FDT_DELAY_BD9 + 84 - CLCL)

#define SUBC_OVF (uint8_t)(F_CPU / 847500 / 2 - 1) //847500 Hz Subcarrier

#define S_IDLE 0
#define S_READY 1
#define S_ACTIVE 2
#define S_HALT 8
#define S_READY_H 9
#define S_ACTIVE_H 10

void setupNfcEmulator(uint8_t *storage, uint16_t storageSize);
void checkForNfcReader();
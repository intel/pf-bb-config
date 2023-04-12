/******************************************************************************
*
*   Copyright (c) 2022 Intel.
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*******************************************************************************/

#ifndef _VRB1_FFT_LUT_H_
#define _VRB1_FFT_LUT_H_

uint32_t fft_lut[VRB1_FFT_RAM_SIZE] = {
	0x1FFFF, 0x1FFFF, 0x1FFFE, 0x1FFFA, 0x1FFF6, 0x1FFF1, 0x1FFEA, 0x1FFE2,
	0x1FFD9, 0x1FFCE, 0x1FFC2, 0x1FFB5, 0x1FFA7, 0x1FF98, 0x1FF87, 0x1FF75,
	0x1FF62, 0x1FF4E, 0x1FF38, 0x1FF21, 0x1FF09, 0x1FEF0, 0x1FED6, 0x1FEBA,
	0x1FE9D, 0x1FE7F, 0x1FE5F, 0x1FE3F, 0x1FE1D, 0x1FDFA, 0x1FDD5, 0x1FDB0,
	0x1FD89, 0x1FD61, 0x1FD38, 0x1FD0D, 0x1FCE1, 0x1FCB4, 0x1FC86, 0x1FC57,
	0x1FC26, 0x1FBF4, 0x1FBC1, 0x1FB8D, 0x1FB58, 0x1FB21, 0x1FAE9, 0x1FAB0,
	0x1FA75, 0x1FA3A, 0x1F9FD, 0x1F9BF, 0x1F980, 0x1F93F, 0x1F8FD, 0x1F8BA,
	0x1F876, 0x1F831, 0x1F7EA, 0x1F7A3, 0x1F75A, 0x1F70F, 0x1F6C4, 0x1F677,
	0x1F629, 0x1F5DA, 0x1F58A, 0x1F539, 0x1F4E6, 0x1F492, 0x1F43D, 0x1F3E7,
	0x1F38F, 0x1F337, 0x1F2DD, 0x1F281, 0x1F225, 0x1F1C8, 0x1F169, 0x1F109,
	0x1F0A8, 0x1F046, 0x1EFE2, 0x1EF7D, 0x1EF18, 0x1EEB0, 0x1EE48, 0x1EDDF,
	0x1ED74, 0x1ED08, 0x1EC9B, 0x1EC2D, 0x1EBBE, 0x1EB4D, 0x1EADB, 0x1EA68,
	0x1E9F4, 0x1E97F, 0x1E908, 0x1E891, 0x1E818, 0x1E79E, 0x1E722, 0x1E6A6,
	0x1E629, 0x1E5AA, 0x1E52A, 0x1E4A9, 0x1E427, 0x1E3A3, 0x1E31F, 0x1E299,
	0x1E212, 0x1E18A, 0x1E101, 0x1E076, 0x1DFEB, 0x1DF5E, 0x1DED0, 0x1DE41,
	0x1DDB1, 0x1DD20, 0x1DC8D, 0x1DBFA, 0x1DB65, 0x1DACF, 0x1DA38, 0x1D9A0,
	0x1D907, 0x1D86C, 0x1D7D1, 0x1D734, 0x1D696, 0x1D5F7, 0x1D557, 0x1D4B6,
	0x1D413, 0x1D370, 0x1D2CB, 0x1D225, 0x1D17E, 0x1D0D6, 0x1D02D, 0x1CF83,
	0x1CED8, 0x1CE2B, 0x1CD7E, 0x1CCCF, 0x1CC1F, 0x1CB6E, 0x1CABC, 0x1CA09,
	0x1C955, 0x1C89F, 0x1C7E9, 0x1C731, 0x1C679, 0x1C5BF, 0x1C504, 0x1C448,
	0x1C38B, 0x1C2CD, 0x1C20E, 0x1C14E, 0x1C08C, 0x1BFCA, 0x1BF06, 0x1BE42,
	0x1BD7C, 0x1BCB5, 0x1BBED, 0x1BB25, 0x1BA5B, 0x1B990, 0x1B8C4, 0x1B7F6,
	0x1B728, 0x1B659, 0x1B589, 0x1B4B7, 0x1B3E5, 0x1B311, 0x1B23D, 0x1B167,
	0x1B091, 0x1AFB9, 0x1AEE0, 0x1AE07, 0x1AD2C, 0x1AC50, 0x1AB73, 0x1AA95,
	0x1A9B6, 0x1A8D6, 0x1A7F6, 0x1A714, 0x1A631, 0x1A54D, 0x1A468, 0x1A382,
	0x1A29A, 0x1A1B2, 0x1A0C9, 0x19FDF, 0x19EF4, 0x19E08, 0x19D1B, 0x19C2D,
	0x19B3E, 0x19A4E, 0x1995D, 0x1986B, 0x19778, 0x19684, 0x1958F, 0x19499,
	0x193A2, 0x192AA, 0x191B1, 0x190B8, 0x18FBD, 0x18EC1, 0x18DC4, 0x18CC7,
	0x18BC8, 0x18AC8, 0x189C8, 0x188C6, 0x187C4, 0x186C1, 0x185BC, 0x184B7,
	0x183B1, 0x182AA, 0x181A2, 0x18099, 0x17F8F, 0x17E84, 0x17D78, 0x17C6C,
	0x17B5E, 0x17A4F, 0x17940, 0x17830, 0x1771E, 0x1760C, 0x174F9, 0x173E5,
	0x172D1, 0x171BB, 0x170A4, 0x16F8D, 0x16E74, 0x16D5B, 0x16C41, 0x16B26,
	0x16A0A, 0x168ED, 0x167CF, 0x166B1, 0x16592, 0x16471, 0x16350, 0x1622E,
	0x1610B, 0x15FE8, 0x15EC3, 0x15D9E, 0x15C78, 0x15B51, 0x15A29, 0x15900,
	0x157D7, 0x156AC, 0x15581, 0x15455, 0x15328, 0x151FB, 0x150CC, 0x14F9D,
	0x14E6D, 0x14D3C, 0x14C0A, 0x14AD8, 0x149A4, 0x14870, 0x1473B, 0x14606,
	0x144CF, 0x14398, 0x14260, 0x14127, 0x13FEE, 0x13EB3, 0x13D78, 0x13C3C,
	0x13B00, 0x139C2, 0x13884, 0x13745, 0x13606, 0x134C5, 0x13384, 0x13242,
	0x130FF, 0x12FBC, 0x12E78, 0x12D33, 0x12BEE, 0x12AA7, 0x12960, 0x12819,
	0x126D0, 0x12587, 0x1243D, 0x122F3, 0x121A8, 0x1205C, 0x11F0F, 0x11DC2,
	0x11C74, 0x11B25, 0x119D6, 0x11886, 0x11735, 0x115E3, 0x11491, 0x1133F,
	0x111EB, 0x11097, 0x10F42, 0x10DED, 0x10C97, 0x10B40, 0x109E9, 0x10891,
	0x10738, 0x105DF, 0x10485, 0x1032B, 0x101D0, 0x10074, 0x0FF18, 0x0FDBB,
	0x0FC5D, 0x0FAFF, 0x0F9A0, 0x0F841, 0x0F6E1, 0x0F580, 0x0F41F, 0x0F2BD,
	0x0F15B, 0x0EFF8, 0x0EE94, 0x0ED30, 0x0EBCC, 0x0EA67, 0x0E901, 0x0E79A,
	0x0E633, 0x0E4CC, 0x0E364, 0x0E1FB, 0x0E092, 0x0DF29, 0x0DDBE, 0x0DC54,
	0x0DAE9, 0x0D97D, 0x0D810, 0x0D6A4, 0x0D536, 0x0D3C8, 0x0D25A, 0x0D0EB,
	0x0CF7C, 0x0CE0C, 0x0CC9C, 0x0CB2B, 0x0C9B9, 0x0C847, 0x0C6D5, 0x0C562,
	0x0C3EF, 0x0C27B, 0x0C107, 0x0BF92, 0x0BE1D, 0x0BCA8, 0x0BB32, 0x0B9BB,
	0x0B844, 0x0B6CD, 0x0B555, 0x0B3DD, 0x0B264, 0x0B0EB, 0x0AF71, 0x0ADF7,
	0x0AC7D, 0x0AB02, 0x0A987, 0x0A80B, 0x0A68F, 0x0A513, 0x0A396, 0x0A219,
	0x0A09B, 0x09F1D, 0x09D9E, 0x09C20, 0x09AA1, 0x09921, 0x097A1, 0x09621,
	0x094A0, 0x0931F, 0x0919E, 0x0901C, 0x08E9A, 0x08D18, 0x08B95, 0x08A12,
	0x0888F, 0x0870B, 0x08587, 0x08402, 0x0827E, 0x080F9, 0x07F73, 0x07DEE,
	0x07C68, 0x07AE2, 0x0795B, 0x077D4, 0x0764D, 0x074C6, 0x0733E, 0x071B6,
	0x0702E, 0x06EA6, 0x06D1D, 0x06B94, 0x06A0B, 0x06881, 0x066F7, 0x0656D,
	0x063E3, 0x06258, 0x060CE, 0x05F43, 0x05DB7, 0x05C2C, 0x05AA0, 0x05914,
	0x05788, 0x055FC, 0x0546F, 0x052E3, 0x05156, 0x04FC9, 0x04E3B, 0x04CAE,
	0x04B20, 0x04992, 0x04804, 0x04676, 0x044E8, 0x04359, 0x041CB, 0x0403C,
	0x03EAD, 0x03D1D, 0x03B8E, 0x039FF, 0x0386F, 0x036DF, 0x0354F, 0x033BF,
	0x0322F, 0x0309F, 0x02F0F, 0x02D7E, 0x02BEE, 0x02A5D, 0x028CC, 0x0273B,
	0x025AA, 0x02419, 0x02288, 0x020F7, 0x01F65, 0x01DD4, 0x01C43, 0x01AB1,
	0x0191F, 0x0178E, 0x015FC, 0x0146A, 0x012D8, 0x01147, 0x00FB5, 0x00E23,
	0x00C91, 0x00AFF, 0x0096D, 0x007DB, 0x00648, 0x004B6, 0x00324, 0x00192};

#endif /* _VRB1_FFT_LUT_H_ */
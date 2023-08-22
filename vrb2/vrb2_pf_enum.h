/******************************************************************************
*
*   Copyright (c) 2020 Intel.
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

#ifndef _VRB2_PF_ENUM_H_
#define _VRB2_PF_ENUM_H_

/*
 * VRB2 Register mapping on PF BAR0
 * This is automatically generated from RDL, format may change with new RDL
 * Release.
 * Variable names are as is
 */
enum {
	HWPfQmgrEgressQueuesTemplate             = 0x0007FC00,
	HWPfAramControlStatus                    = 0x00810000,
	HWPfQmgrIngressAq                        = 0x00100000,
	HWPfQmgrArbQAvail                        = 0x00A00010,
	HWPfQmgrArbQBlock                        = 0x00A00020,
	HWPfQmgrAqueueDropNotifEn                = 0x00A00024,
	HWPfQmgrAqueueDisableNotifEn             = 0x00A00028,
	HWPfQmgrSoftReset                        = 0x00A00034,
	HWPfQmgrGrpSoftReset                     = 0x00A00038,
	HWPfQmgrInitStatus                       = 0x00A0003C,
	HWPfQmgrAramWatchdogCount                = 0x00A00040,
	HWPfQmgrAramWatchdogCounterEn            = 0x00A00044,
	HWPfQmgrAxiWatchdogCount                 = 0x00A00048,
	HWPfQmgrAxiWatchdogCounterEn             = 0x00A0004C,
	HWPfQmgrProcessWatchdogCount             = 0x00A00050,
	HWPfQmgrProcessWatchdogCounterEn         = 0x00A00054,
	HWPfQmgrProcessWatchdogCounterFunc       = 0x00A00060,
	HWPfQmgrMsiOverflowUpperVf               = 0x00A00080,
	HWPfQmgrMsiOverflowLowerVf               = 0x00A00084,
	HWPfQmgrMsiWatchdogOverflow              = 0x00A00088,
	HWPfQmgrMsiOverflowEnable                = 0x00A0008C,
	HWPfQmgrAramAllocEn	                 = 0x00A000a0,
	HWPfQmgrAramAllocSetupN0                 = 0x00A000b0,
	HWPfQmgrAramAllocSetupN1                 = 0x00A000b4,
	HWPfQmgrAramAllocSetupN2                 = 0x00A000b8,
	HWPfQmgrAramAllocSetupN3                 = 0x00A000bc,
	HWPfQmgrAramAllocStatusN0	         = 0x00A000c0,
	HWPfQmgrAramAllocStatusN1	         = 0x00A000c4,
	HWPfQmgrAramAllocStatusN2	         = 0x00A000c8,
	HWPfQmgrAramAllocStatusN3	         = 0x00A000cc,
	HWPfQmgrDepthLog2Grp                     = 0x00A00200,
	HWPfQmgrTholdGrp                         = 0x00A00300,
	HWPfQmgrGrpTmplateReg0Indx               = 0x00A00600,
	HWPfQmgrGrpTmplateReg1Indx               = 0x00A00700,
	HWPfQmgrGrpTmplateReg2Indx               = 0x00A00800,
	HWPfQmgrGrpTmplateReg3Indx               = 0x00A00900,
	HWPfQmgrGrpTmplateReg4Indx               = 0x00A00A00,
	HWPfQmgrGrpTmplateReg5Indx               = 0x00A00B00,
	HWPfQmgrGrpTmplateReg6Indx               = 0x00A00C00,
	HWPfQmgrGrpTmplateReg7Indx               = 0x00A00D00,
	HWPfQmgrGrpTmplateEnRegIndx              = 0x00A00E00,
	HWPfQmgrUl4GWeightRrVf                   = 0x00A02000,
	HWPfQmgrDl4GWeightRrVf                   = 0x00A02100,
	HWPfQmgrUl5GWeightRrVf                   = 0x00A02200,
	HWPfQmgrDl5GWeightRrVf                   = 0x00A02300,
	HWPfQmgrMldWeightRrVf                    = 0x00A02400,
	HWPfQmgrArbQDepthGrp                     = 0x00A02F00,
	HWPfQmgrGrpFunction0                     = 0x00A02F80,
	HWPfQmgrGrpFunction1                     = 0x00A02F84,
	HWPfQmgrGrpFunction2                     = 0x00A02F88,
	HWPfQmgrGrpFunction3                     = 0x00A02F8C,
	HWPfQmgrGrpPriority                      = 0x00A02FC0,
	HWPfQmgrWeightSync                       = 0x00A03000,
	HWPfQmgrPasidAtControlAdi		 = 0x00A04000,
	HWPfQmgrVfBaseAddr                       = 0x00A08000,
	HwPfQmgrIrqDebug0                        = 0x00A0F240,
	HwPfQmgrIrqDebug1                        = 0x00A0F244,
	HWPfQmgrAdrGenEcc                        = 0x00A0F320,
	HWPfQmgrPtrEccGrp                        = 0x00A0F380,
	HWPfQmgrAqEnableVf                       = 0x00A10000,
	HWPfQmgrAqResetVf                        = 0x00A20000,
	HWPfQmgrRingSizeVf                       = 0x00A20010,
	HWPfQmgrGrpDepthLog20Vf                  = 0x00A20020,
	HWPfQmgrGrpDepthLog21Vf                  = 0x00A20024,
	HWPfQmgrGrpDepthLog22Vf                  = 0x00A20028,
	HWPfQmgrGrpDepthLog23Vf                  = 0x00A2002C,
	HWPfQmgrGrpFunction0Vf                   = 0x00A20030,
	HWPfQmgrGrpFunction1Vf                   = 0x00A20034,
	HWPfFabricM2iBufferReg                   = 0x00B30000,
	HWPfFabricI2mRouterCoreWeight            = 0x00B31004,
	HWPfFabricI2mRouteDmaDataWeight          = 0x00B31044,
	HWPfFabricI2mRouterCore0                 = 0x00B31800,
	HWPfFabricI2mRouterCore1                 = 0x00B31804,
	HWPfFabricI2mRouterCore2                 = 0x00B31808,
	HWPfFecul5GCntrlReg                      = 0x00B40000,
	HWPfFecul5GI2MThreshReg                  = 0x00B40004,
	HWPfFecul5GVersionReg                    = 0x00B40100,
	HWPfFecul5GFcwStatusReg                  = 0x00B40104,
	HWPfFecul5GWarnReg                       = 0x00B40108,
	HwPfFecUl5gIbDebug0Reg                   = 0x00B401FC,
	HwPfFecUl5gIbDebug1Reg                   = 0x00B40200,
	HwPfFecUl5gObLlrDebugReg                 = 0x00B40204,
	HwPfFecUl5gObHarqDebugReg                = 0x00B40208,
	HwPfFecUl5gObSoftDebugReg                = 0x00B4020C,
	HWPfpFecdl5GCntrlReg                     = 0x00B4B000,
	HWPfpFecdl5GI2MThreshReg                 = 0x00B4B004,
	HWPfpFecdl5GVersionReg                   = 0x00B4B100,
	HWPfpFecdl5GFcwStatusReg                 = 0x00B4B104,
	HWPfpFecdl5GWarnReg                      = 0x00B4B108,
	HWPfFeculVersionReg                      = 0x00B4C000,
	HWPfFeculControlReg                      = 0x00B4C004,
	HWPfFeculStatusReg                       = 0x00B4C008,
	HWPfFecdlVersionReg                      = 0x00B57000,
	HWPfFecdlClusterConfigReg                = 0x00B57004,
	HWPfFecdlBurstThres                      = 0x00B5700C,
	HWPfFecdlClusterStatusReg0               = 0x00B57040,
	HWPfFecdlClusterStatusReg1               = 0x00B57044,
	HWPfFecdlClusterStatusReg2               = 0x00B57048,
	HWPfFecdlClusterStatusReg3               = 0x00B5704C,
	HWPfFecdlClusterStatusReg4               = 0x00B57050,
	HWPfFecdlClusterStatusReg5               = 0x00B57054,
	HWPfFftConfig0                           = 0x00B58004,
	HWPfFftConfig1                           = 0x00B58008,
	HWPfFftRamPageAccess                     = 0x00B5800C,
	HWPfFftParityMask0                       = 0x00B5801C,
	HWPfFftParityMask1                       = 0x00B58020,
	HWPfFftParityMask2                       = 0x00B58024,
	HWPfFftParityMask3                       = 0x00B58028,
	HWPfFftParityMask4                       = 0x00B5802C,
	HWPfFftParityMask5                       = 0x00B58030,
	HWPfFftParityMask6                       = 0x00B58034,
	HWPfFftParityMask7                       = 0x00B58038,
	HWPfFftParityMask8                       = 0x00B5803C,
	HwPfFftStatus0                           = 0x00B58040,
	HwPfFftStatus1                           = 0x00B58044,
	HwPfFftStatus2                           = 0x00B58048,
	HwPfFftStatus3                           = 0x00B5804C,
	HwPfFftStatus4                           = 0x00B58050,
	HwPfFftStatus5                           = 0x00B58054,
	HwPfFftStatus6                           = 0x00B58058,
	HwPfFftStatus7                           = 0x00B5805C,
	HwPfFftStatus8                           = 0x00B58060,
	HwPfFftStatus9                           = 0x00B58064,
	HwPfFftStatus10                          = 0x00B58068,
	HwPfFftStatus11                          = 0x00B5806C,
	HwPfFftStatus12                          = 0x00B58070,
	HwPfFftStatus13                          = 0x00B58074,
	HwPfFftStatus14                          = 0x00B58078,
	HwPfFftStatus15                          = 0x00B5807C,
	HwPfFftParityStatus0                     = 0x00B58080,
	HwPfFftParityStatus1                     = 0x00B58084,
	HwPfFftParityStatus2                     = 0x00B58088,
	HwPfFftParityStatus3                     = 0x00B5808C,
	HwPfFftParityStatus4                     = 0x00B58090,
	HwPfFftParityStatus5                     = 0x00B58094,
	HwPfFftParityStatus6                     = 0x00B58098,
	HwPfFftParityStatus7                     = 0x00B5809C,
	HwPfFftParityStatus8                     = 0x00B580A0,
	HWPfFftRamOff                            = 0x00B58800,
	HWPfDmaConfig0Reg                        = 0x00B80000,
	HWPfDmaConfig1Reg                        = 0x00B80004,
	HWPfDmaQmgrAddrReg                       = 0x00B80008,
	HWPfDmaSoftResetReg                      = 0x00B8000C,
	HWPfDmaAxcacheReg                        = 0x00B80010,
	HWPfDmaVersionReg                        = 0x00B80014,
	HWPfDmaFrameThreshold                    = 0x00B80018,
	HWPfDmaTimestampLo                       = 0x00B8001C,
	HWPfDmaTimestampHi                       = 0x00B80020,
	HWPfDmaAxiStatus                         = 0x00B80028,
	HWPfDmaAxiControl                        = 0x00B8002C,
	HWPfDmaNoQmgr                            = 0x00B80030,
	HWPfDmaQosScale                          = 0x00B80034,
	HWPfDmaQmanen                            = 0x00B80040,
	HWPfDmaQmanenSelect                      = 0x00B80044,
	HWPfDmaSlideFft                          = 0x00B80050,
	HWPfDmaFftModeThreshold                  = 0x00B80054,
	HWPfDmaQmgrQosBase                       = 0x00B80060,
	HWPfDmaFecClkGatingEnable                = 0x00B80080,
	HWPfDmaWeightedSwitchingSmall            = 0x00B800B0,
	HWPfDmaWeightedSwitchingLarge            = 0x00B800B4,
	HWPfDmaWeightedSwitchingStream           = 0x00B800B8,
	HWPfDmaInboundCbMaxSize                  = 0x00B800BC,
	HWPfDmaInboundDrainDataSize              = 0x00B800C0,
	HWPfDmaEngineTypeSmall                   = 0x00B800C4,
	HWPfDmaIbThreshold                       = 0x00B800C8,
	HWPfDmaStreamIbThreshold                 = 0x00B800CC,
	HWPfDmaVfDdrBaseRw                       = 0x00B80400,
	HWPfDmaAxiStreamDebugErrInj              = 0x00B804A4,
	HWPfDmaPtbCtrl0                          = 0x00B80500,
	HWPfDmaPtbStatus0                        = 0x00B80558,
	HWPfDmaCmplTmOutCnt                      = 0x00B80800,
	HWPfDmaStatusRrespBresp                  = 0x00B80810,
	HWPfDmaCfgRrespBresp                     = 0x00B80814,
	HWPfDmaStatusMemParErr                   = 0x00B80818,
	HWPfDmaCfgMemParErrEn                    = 0x00B8081C,
	HWPfDmaStatusDmaHwErr                    = 0x00B80820,
	HWPfDmaCfgDmaHwErrEn                     = 0x00B80824,
	HWPfDmaStatusFecCoreErr                  = 0x00B80828,
	HWPfDmaCfgFecCoreErrEn                   = 0x00B8082C,
	HWPfDmaStatusFcwDescrErr                 = 0x00B80830,
	HWPfDmaCfgFcwDescrErrEn                  = 0x00B80834,
	HWPfDmaStatusBlockTransmit               = 0x00B80838,
	HWPfDmaBlockOnErrEn                      = 0x00B8083C,
	HWPfDmaStatusFlushDma                    = 0x00B80840,
	HWPfDmaFlushDmaOnErrEn                   = 0x00B80844,
	HWPfDmaStatusSdoneFifoFull               = 0x00B80848,
	HWPfDmaStatusDescriptorErrLoVf           = 0x00B8084C,
	HWPfDmaStatusDescriptorErrHiVf           = 0x00B80850,
	HWPfDmaStatusFcwErrLoVf                  = 0x00B80854,
	HWPfDmaStatusFcwErrHiVf                  = 0x00B80858,
	HWPfDmaStatusDataErrLoVf                 = 0x00B8085C,
	HWPfDmaStatusDataErrHiVf                 = 0x00B80860,
	HWPfDmaCfgMsiEnSoftwareErr               = 0x00B80864,
	HWPfDmaDescriptorSignature               = 0x00B80868,
	HWPfDmaFcwSignature                      = 0x00B8086C,
	HWPfDmaErrorDetectionEn                  = 0x00B80870,
	HWPfDmaErrCntrlFifoDebug                 = 0x00B8087C,
	HWPfDmaStatusToutData                    = 0x00B80880,
	HWPfDmaStatusToutDesc                    = 0x00B80884,
	HWPfDmaStatusToutUnexpData               = 0x00B80888,
	HWPfDmaStatusToutUnexpDesc               = 0x00B8088C,
	HWPfDmaStatusToutProcess                 = 0x00B80890,
	HWPfDmaConfigCtoutOutDataEn              = 0x00B808A0,
	HWPfDmaConfigCtoutOutDescrEn             = 0x00B808A4,
	HWPfDmaConfigUnexpComplDataEn            = 0x00B808A8,
	HWPfDmaConfigUnexpComplDescrEn           = 0x00B808AC,
	HWPfDmaConfigPtoutOutEn                  = 0x00B808B0,
	HWPfDmaFatalParityErrorEn		 = 0x00B808B4,
	HWPfDmaNonFatalParityErrorEn		 = 0x00B808B8,
	HWPfDmaErrcntrlDbg			 = 0x00B808BC,
	HWPfDmaChrCtrl				 = 0x00B80C00,
	HWPfDmaChrCleanupTshld			 = 0x00B80C04,
	HWPfDmaChrStatus			 = 0x00B80C0C,
	HWPfDmaClusterCtrl			 = 0x00B80E00,
	HWPfDmaClusterHangThld			 = 0x00B80E04,
	HWPfDmaClusterHangStatus		 = 0x00B80E08,
	HWPfDmaProcTmOutCnt			 = 0x00B80F00,
	HWPfDmaFec5GulDescBaseLoRegVf            = 0x00B88020,
	HWPfDmaFec5GulDescBaseHiRegVf            = 0x00B88024,
	HWPfDmaFec5GulRespPtrLoRegVf             = 0x00B88028,
	HWPfDmaFec5GulRespPtrHiRegVf             = 0x00B8802C,
	HWPfDmaFec5GdlDescBaseLoRegVf            = 0x00B88040,
	HWPfDmaFec5GdlDescBaseHiRegVf            = 0x00B88044,
	HWPfDmaFec5GdlRespPtrLoRegVf             = 0x00B88048,
	HWPfDmaFec5GdlRespPtrHiRegVf             = 0x00B8804C,
	HWPfDmaFec4GulDescBaseLoRegVf            = 0x00B88060,
	HWPfDmaFec4GulDescBaseHiRegVf            = 0x00B88064,
	HWPfDmaFec4GulRespPtrLoRegVf             = 0x00B88068,
	HWPfDmaFec4GulRespPtrHiRegVf             = 0x00B8806C,
	HWPfDmaFec4GdlDescBaseLoRegVf            = 0x00B88080,
	HWPfDmaFec4GdlDescBaseHiRegVf            = 0x00B88084,
	HWPfDmaFec4GdlRespPtrLoRegVf             = 0x00B88088,
	HWPfDmaFec4GdlRespPtrHiRegVf             = 0x00B8808C,
	HWPfDmaFftDescBaseLoRegVf                = 0x00B880A0,
	HWPfDmaFftDescBaseHiRegVf                = 0x00B880A4,
	HWPfDmaFftRespPtrLoRegVf                 = 0x00B880A8,
	HWPfDmaFftRespPtrHiRegVf                 = 0x00B880AC,
	HWPfDmaMldDescBaseLoRegVf                = 0x00B880C0,
	HWPfDmaMldDescBaseHiRegVf                = 0x00B880C4,
	HWPfDmaMldRespPtrLoRegVf                 = 0x00B880C8,
	HWPfDmaMldRespPtrHiRegVf                 = 0x00B880CC,
	HWPfDmaVfDdrBaseRangeRo                  = 0x00B880A0,
	HWPfQosmonACntrlReg                      = 0x00B90000,
	HWPfQosmonAEvalOverflow0                 = 0x00B90008,
	HWPfQosmonAEvalOverflow1                 = 0x00B9000C,
	HWPfQosmonADivTerm                       = 0x00B90010,
	HWPfQosmonATickTerm                      = 0x00B90014,
	HWPfQosmonAEvalTerm                      = 0x00B90018,
	HWPfQosmonAAveTerm                       = 0x00B9001C,
	HWPfQosmonAForceEccErr                   = 0x00B90020,
	HWPfQosmonAEccErrDetect                  = 0x00B90024,
	HWPfQosmonAIterationConfig0Low           = 0x00B90060,
	HWPfQosmonAIterationConfig0High          = 0x00B90064,
	HWPfQosmonAIterationConfig1Low           = 0x00B90068,
	HWPfQosmonAIterationConfig1High          = 0x00B9006C,
	HWPfQosmonAIterationConfig2Low           = 0x00B90070,
	HWPfQosmonAIterationConfig2High          = 0x00B90074,
	HWPfQosmonAIterationConfig3Low           = 0x00B90078,
	HWPfQosmonAIterationConfig3High          = 0x00B9007C,
	HWPfQosmonAEvalMemAddr                   = 0x00B90080,
	HWPfQosmonAEvalMemData                   = 0x00B90084,
	HWPfQosmonAXaction                       = 0x00B900C0,
	HWPfQosmonARemThres1Vf                   = 0x00B90400,
	HWPfQosmonAThres2Vf                      = 0x00B90404,
	HWPfQosmonAWeiFracVf                     = 0x00B90408,
	HWPfQosmonARrWeiVf                       = 0x00B9040C,
	HWPfPermonACntrlRegVf                      = 0x00B98000,
	HWPfPermonACountVf                         = 0x00B98008,
	HWPfPermonAKCntLoVf                        = 0x00B98010,
	HWPfPermonAKCntHiVf                        = 0x00B98014,
	HWPfPermonADeltaCntLoVf                    = 0x00B98020,
	HWPfPermonADeltaCntHiVf                    = 0x00B98024,
	HWPfPermonAVersionReg                      = 0x00B9C000,
	HWPfPermonACbControlFec                    = 0x00B9C0F0,
	HWPfPermonADltTimerLoFec                   = 0x00B9C0F4,
	HWPfPermonADltTimerHiFec                   = 0x00B9C0F8,
	HWPfPermonACbCountFec                      = 0x00B9C100,
	HWPfPermonAAccExecTimerLoFec               = 0x00B9C104,
	HWPfPermonAAccExecTimerHiFec               = 0x00B9C108,
	HWPfPermonAExecTimerMinFec                 = 0x00B9C200,
	HWPfPermonAExecTimerMaxFec                 = 0x00B9C204,
	HWPfPermonAControlBusMon                   = 0x00B9C400,
	HWPfPermonAConfigBusMon                    = 0x00B9C404,
	HWPfPermonASkipCountBusMon                 = 0x00B9C408,
	HWPfPermonAMinLatBusMon                    = 0x00B9C40C,
	HWPfPermonAMaxLatBusMon                    = 0x00B9C500,
	HWPfPermonATotalLatLowBusMon               = 0x00B9C504,
	HWPfPermonATotalLatUpperBusMon             = 0x00B9C508,
	HWPfPermonATotalReqCntBusMon               = 0x00B9C50C,
	HWPfQosmonBCntrlReg                        = 0x00BA0000,
	HWPfQosmonBEvalOverflow0                   = 0x00BA0008,
	HWPfQosmonBEvalOverflow1                   = 0x00BA000C,
	HWPfQosmonBDivTerm                         = 0x00BA0010,
	HWPfQosmonBTickTerm                        = 0x00BA0014,
	HWPfQosmonBEvalTerm                        = 0x00BA0018,
	HWPfQosmonBAveTerm                         = 0x00BA001C,
	HWPfQosmonBForceEccErr                     = 0x00BA0020,
	HWPfQosmonBEccErrDetect                    = 0x00BA0024,
	HWPfQosmonBIterationConfig0Low             = 0x00BA0060,
	HWPfQosmonBIterationConfig0High            = 0x00BA0064,
	HWPfQosmonBIterationConfig1Low             = 0x00BA0068,
	HWPfQosmonBIterationConfig1High            = 0x00BA006C,
	HWPfQosmonBIterationConfig2Low             = 0x00BA0070,
	HWPfQosmonBIterationConfig2High            = 0x00BA0074,
	HWPfQosmonBIterationConfig3Low             = 0x00BA0078,
	HWPfQosmonBIterationConfig3High            = 0x00BA007C,
	HWPfQosmonBEvalMemAddr                     = 0x00BA0080,
	HWPfQosmonBEvalMemData                     = 0x00BA0084,
	HWPfQosmonBXaction                         = 0x00BA00C0,
	HWPfQosmonBRemThres1Vf                     = 0x00BA0400,
	HWPfQosmonBThres2Vf                        = 0x00BA0404,
	HWPfQosmonBWeiFracVf                       = 0x00BA0408,
	HWPfQosmonBRrWeiVf                         = 0x00BA040C,
	HWPfPermonBCntrlRegVf                      = 0x00BA8000,
	HWPfPermonBCountVf                         = 0x00BA8008,
	HWPfPermonBKCntLoVf                        = 0x00BA8010,
	HWPfPermonBKCntHiVf                        = 0x00BA8014,
	HWPfPermonBDeltaCntLoVf                    = 0x00BA8020,
	HWPfPermonBDeltaCntHiVf                    = 0x00BA8024,
	HWPfPermonBVersionReg                      = 0x00BAC000,
	HWPfPermonBCbControlFec                    = 0x00BAC0F0,
	HWPfPermonBDltTimerLoFec                   = 0x00BAC0F4,
	HWPfPermonBDltTimerHiFec                   = 0x00BAC0F8,
	HWPfPermonBCbCountFec                      = 0x00BAC100,
	HWPfPermonBAccExecTimerLoFec               = 0x00BAC104,
	HWPfPermonBAccExecTimerHiFec               = 0x00BAC108,
	HWPfPermonBExecTimerMinFec                 = 0x00BAC200,
	HWPfPermonBExecTimerMaxFec                 = 0x00BAC204,
	HWPfPermonBControlBusMon                   = 0x00BAC400,
	HWPfPermonBConfigBusMon                    = 0x00BAC404,
	HWPfPermonBSkipCountBusMon                 = 0x00BAC408,
	HWPfPermonBMinLatBusMon                    = 0x00BAC40C,
	HWPfPermonBMaxLatBusMon                    = 0x00BAC500,
	HWPfPermonBTotalLatLowBusMon               = 0x00BAC504,
	HWPfPermonBTotalLatUpperBusMon             = 0x00BAC508,
	HWPfPermonBTotalReqCntBusMon               = 0x00BAC50C,
	HWPfQosmonCCntrlReg                        = 0x00BB0000,
	HWPfQosmonCEvalOverflow0                   = 0x00BB0008,
	HWPfQosmonCEvalOverflow1                   = 0x00BB000C,
	HWPfQosmonCDivTerm                         = 0x00BB0010,
	HWPfQosmonCTickTerm                        = 0x00BB0014,
	HWPfQosmonCEvalTerm                        = 0x00BB0018,
	HWPfQosmonCAveTerm                         = 0x00BB001C,
	HWPfQosmonCForceEccErr                     = 0x00BB0020,
	HWPfQosmonCEccErrDetect                    = 0x00BB0024,
	HWPfQosmonCIterationConfig0Low             = 0x00BB0060,
	HWPfQosmonCIterationConfig0High            = 0x00BB0064,
	HWPfQosmonCIterationConfig1Low             = 0x00BB0068,
	HWPfQosmonCIterationConfig1High            = 0x00BB006C,
	HWPfQosmonCIterationConfig2Low             = 0x00BB0070,
	HWPfQosmonCIterationConfig2High            = 0x00BB0074,
	HWPfQosmonCIterationConfig3Low             = 0x00BB0078,
	HWPfQosmonCIterationConfig3High            = 0x00BB007C,
	HWPfQosmonCEvalMemAddr                     = 0x00BB0080,
	HWPfQosmonCEvalMemData                     = 0x00BB0084,
	HWPfQosmonCXaction                         = 0x00BB00C0,
	HWPfQosmonCRemThres1Vf                     = 0x00BB0400,
	HWPfQosmonCThres2Vf                        = 0x00BB0404,
	HWPfQosmonCWeiFracVf                       = 0x00BB0408,
	HWPfQosmonCRrWeiVf                         = 0x00BB040C,
	HWPfPermonCCntrlRegVf                      = 0x00BB8000,
	HWPfPermonCCountVf                         = 0x00BB8008,
	HWPfPermonCKCntLoVf                        = 0x00BB8010,
	HWPfPermonCKCntHiVf                        = 0x00BB8014,
	HWPfPermonCDeltaCntLoVf                    = 0x00BB8020,
	HWPfPermonCDeltaCntHiVf                    = 0x00BB8024,
	HWPfPermonCVersionReg                      = 0x00BBC000,
	HWPfPermonCCbControlFec                    = 0x00BBC0F0,
	HWPfPermonCDltTimerLoFec                   = 0x00BBC0F4,
	HWPfPermonCDltTimerHiFec                   = 0x00BBC0F8,
	HWPfPermonCCbCountFec                      = 0x00BBC100,
	HWPfPermonCAccExecTimerLoFec               = 0x00BBC104,
	HWPfPermonCAccExecTimerHiFec               = 0x00BBC108,
	HWPfPermonCExecTimerMinFec                 = 0x00BBC200,
	HWPfPermonCExecTimerMaxFec                 = 0x00BBC204,
	HWPfPermonCControlBusMon                   = 0x00BBC400,
	HWPfPermonCConfigBusMon                    = 0x00BBC404,
	HWPfPermonCSkipCountBusMon                 = 0x00BBC408,
	HWPfPermonCMinLatBusMon                    = 0x00BBC40C,
	HWPfPermonCMaxLatBusMon                    = 0x00BBC500,
	HWPfPermonCTotalLatLowBusMon               = 0x00BBC504,
	HWPfPermonCTotalLatUpperBusMon             = 0x00BBC508,
	HWPfPermonCTotalReqCntBusMon               = 0x00BBC50C,
	HWPfQosmonDCntrlReg                        = 0x00BC0000,
	HWPfQosmonDEvalOverflow0                   = 0x00BC0008,
	HWPfQosmonDEvalOverflow1                   = 0x00BC000C,
	HWPfQosmonDDivTerm                         = 0x00BC0010,
	HWPfQosmonDTickTerm                        = 0x00BC0014,
	HWPfQosmonDEvalTerm                        = 0x00BC0018,
	HWPfQosmonDAveTerm                         = 0x00BC001C,
	HWPfQosmonDForceEccErr                     = 0x00BC0020,
	HWPfQosmonDEccErrDetect                    = 0x00BC0024,
	HWPfQosmonDIterationConfig0Low             = 0x00BC0060,
	HWPfQosmonDIterationConfig0High            = 0x00BC0064,
	HWPfQosmonDIterationConfig1Low             = 0x00BC0068,
	HWPfQosmonDIterationConfig1High            = 0x00BC006C,
	HWPfQosmonDIterationConfig2Low             = 0x00BC0070,
	HWPfQosmonDIterationConfig2High            = 0x00BC0074,
	HWPfQosmonDIterationConfig3Low             = 0x00BC0078,
	HWPfQosmonDIterationConfig3High            = 0x00BC007C,
	HWPfQosmonDEvalMemAddr                     = 0x00BC0080,
	HWPfQosmonDEvalMemData                     = 0x00BC0084,
	HWPfQosmonDXaction                         = 0x00BC00C0,
	HWPfQosmonDRemThres1Vf                     = 0x00BC0400,
	HWPfQosmonDThres2Vf                        = 0x00BC0404,
	HWPfQosmonDWeiFracVf                       = 0x00BC0408,
	HWPfQosmonDRrWeiVf                         = 0x00BC040C,
	HWPfPermonDCntrlRegVf                      = 0x00BC8000,
	HWPfPermonDCountVf                         = 0x00BC8008,
	HWPfPermonDKCntLoVf                        = 0x00BC8010,
	HWPfPermonDKCntHiVf                        = 0x00BC8014,
	HWPfPermonDDeltaCntLoVf                    = 0x00BC8020,
	HWPfPermonDDeltaCntHiVf                    = 0x00BC8024,
	HWPfPermonDVersionReg                      = 0x00BCC000,
	HWPfPermonDCbControlFec                    = 0x00BCC0F0,
	HWPfPermonDDltTimerLoFec                   = 0x00BCC0F4,
	HWPfPermonDDltTimerHiFec                   = 0x00BCC0F8,
	HWPfPermonDCbCountFec                      = 0x00BCC100,
	HWPfPermonDAccExecTimerLoFec               = 0x00BCC104,
	HWPfPermonDAccExecTimerHiFec               = 0x00BCC108,
	HWPfPermonDExecTimerMinFec                 = 0x00BCC200,
	HWPfPermonDExecTimerMaxFec                 = 0x00BCC204,
	HWPfPermonDControlBusMon                   = 0x00BCC400,
	HWPfPermonDConfigBusMon                    = 0x00BCC404,
	HWPfPermonDSkipCountBusMon                 = 0x00BCC408,
	HWPfPermonDMinLatBusMon                    = 0x00BCC40C,
	HWPfPermonDMaxLatBusMon                    = 0x00BCC500,
	HWPfPermonDTotalLatLowBusMon               = 0x00BCC504,
	HWPfPermonDTotalLatUpperBusMon             = 0x00BCC508,
	HWPfPermonDTotalReqCntBusMon               = 0x00BCC50C,
	HWPfQosmonECntrlReg                        = 0x00BD0000,
	HWPfQosmonEEvalOverflow0                   = 0x00BD0008,
	HWPfQosmonEEvalOverflow1                   = 0x00BD000C,
	HWPfQosmonEDivTerm                         = 0x00BD0010,
	HWPfQosmonETickTerm                        = 0x00BD0014,
	HWPfQosmonEEvalTerm                        = 0x00BD0018,
	HWPfQosmonEAveTerm                         = 0x00BD001C,
	HWPfQosmonEForceEccErr                     = 0x00BD0020,
	HWPfQosmonEEccErrDetect                    = 0x00BD0024,
	HWPfQosmonEIterationConfig0Low             = 0x00BD0060,
	HWPfQosmonEIterationConfig0High            = 0x00BD0064,
	HWPfQosmonEIterationConfig1Low             = 0x00BD0068,
	HWPfQosmonEIterationConfig1High            = 0x00BD006C,
	HWPfQosmonEIterationConfig2Low             = 0x00BD0070,
	HWPfQosmonEIterationConfig2High            = 0x00BD0074,
	HWPfQosmonEIterationConfig3Low             = 0x00BD0078,
	HWPfQosmonEIterationConfig3High            = 0x00BD007C,
	HWPfQosmonEEvalMemAddr                     = 0x00BD0080,
	HWPfQosmonEEvalMemData                     = 0x00BD0084,
	HWPfQosmonEXaction                         = 0x00BD00C0,
	HWPfQosmonERemThres1Vf                     = 0x00BD0400,
	HWPfQosmonEThres2Vf                        = 0x00BD0404,
	HWPfQosmonEWeiFracVf                       = 0x00BD0408,
	HWPfQosmonERrWeiVf                         = 0x00BD040C,
	HWPfPermonECntrlRegVf                      = 0x00BD8000,
	HWPfPermonECountVf                         = 0x00BD8008,
	HWPfPermonEKCntLoVf                        = 0x00BD8010,
	HWPfPermonEKCntHiVf                        = 0x00BD8014,
	HWPfPermonEDeltaCntLoVf                    = 0x00BD8020,
	HWPfPermonEDeltaCntHiVf                    = 0x00BD8024,
	HWPfPermonEVersionReg                      = 0x00BDC000,
	HWPfPermonECbControlFec                    = 0x00BDC0F0,
	HWPfPermonEDltTimerLoFec                   = 0x00BDC0F4,
	HWPfPermonEDltTimerHiFec                   = 0x00BDC0F8,
	HWPfPermonECbCountFec                      = 0x00BDC100,
	HWPfPermonEAccExecTimerLoFec               = 0x00BDC104,
	HWPfPermonEAccExecTimerHiFec               = 0x00BDC108,
	HWPfPermonEExecTimerMinFec                 = 0x00BDC200,
	HWPfPermonEExecTimerMaxFec                 = 0x00BDC204,
	HWPfPermonEControlBusMon                   = 0x00BDC400,
	HWPfPermonEConfigBusMon                    = 0x00BDC404,
	HWPfPermonESkipCountBusMon                 = 0x00BDC408,
	HWPfPermonEMinLatBusMon                    = 0x00BDC40C,
	HWPfPermonEMaxLatBusMon                    = 0x00BDC500,
	HWPfPermonETotalLatLowBusMon               = 0x00BDC504,
	HWPfPermonETotalLatUpperBusMon             = 0x00BDC508,
	HWPfPermonETotalReqCntBusMon               = 0x00BDC50C,
	HWPfQosmonFCntrlReg                        = 0x00BE0000,
	HWPfQosmonFEvalOverflow0                   = 0x00BE0008,
	HWPfQosmonFEvalOverflow1                   = 0x00BE000C,
	HWPfQosmonFDivTerm                         = 0x00BE0010,
	HWPfQosmonFTickTerm                        = 0x00BE0014,
	HWPfQosmonFEvalTerm                        = 0x00BE0018,
	HWPfQosmonFAveTerm                         = 0x00BE001C,
	HWPfQosmonFForceEccErr                     = 0x00BE0020,
	HWPfQosmonFEccErrDetect                    = 0x00BE0024,
	HWPfQosmonFIterationConfig0Low             = 0x00BE0060,
	HWPfQosmonFIterationConfig0High            = 0x00BE0064,
	HWPfQosmonFIterationConfig1Low             = 0x00BE0068,
	HWPfQosmonFIterationConfig1High            = 0x00BE006C,
	HWPfQosmonFIterationConfig2Low             = 0x00BE0070,
	HWPfQosmonFIterationConfig2High            = 0x00BE0074,
	HWPfQosmonFIterationConfig3Low             = 0x00BE0078,
	HWPfQosmonFIterationConfig3High            = 0x00BE007C,
	HWPfQosmonFEvalMemAddr                     = 0x00BE0080,
	HWPfQosmonFEvalMemData                     = 0x00BE0084,
	HWPfQosmonFXaction                         = 0x00BE00C0,
	HWPfQosmonFRemThres1Vf                     = 0x00BE0400,
	HWPfQosmonFThres2Vf                        = 0x00BE0404,
	HWPfQosmonFWeiFracVf                       = 0x00BE0408,
	HWPfQosmonFRrWeiVf                         = 0x00BE040C,
	HWPfPermonFCntrlRegVf                      = 0x00BE8000,
	HWPfPermonFCountVf                         = 0x00BE8008,
	HWPfPermonFKCntLoVf                        = 0x00BE8010,
	HWPfPermonFKCntHiVf                        = 0x00BE8014,
	HWPfPermonFDeltaCntLoVf                    = 0x00BE8020,
	HWPfPermonFDeltaCntHiVf                    = 0x00BE8024,
	HWPfPermonFVersionReg                      = 0x00BEC000,
	HWPfPermonFCbControlFec                    = 0x00BEC0F0,
	HWPfPermonFDltTimerLoFec                   = 0x00BEC0F4,
	HWPfPermonFDltTimerHiFec                   = 0x00BEC0F8,
	HWPfPermonFCbCountFec                      = 0x00BEC100,
	HWPfPermonFAccExecTimerLoFec               = 0x00BEC104,
	HWPfPermonFAccExecTimerHiFec               = 0x00BEC108,
	HWPfPermonFExecTimerMinFec                 = 0x00BEC200,
	HWPfPermonFExecTimerMaxFec                 = 0x00BEC204,
	HWPfPermonFControlBusMon                   = 0x00BEC400,
	HWPfPermonFConfigBusMon                    = 0x00BEC404,
	HWPfPermonFSkipCountBusMon                 = 0x00BEC408,
	HWPfPermonFMinLatBusMon                    = 0x00BEC40C,
	HWPfPermonFMaxLatBusMon                    = 0x00BEC500,
	HWPfPermonFTotalLatLowBusMon               = 0x00BEC504,
	HWPfPermonFTotalLatUpperBusMon             = 0x00BEC508,
	HWPfPermonFTotalReqCntBusMon               = 0x00BEC50C,
	HWPfHiVfToPfDbellVf                        = 0x00C80000,
	HWPfHiPfToVfDbellVf                        = 0x00C80008,
	HWPfHiInfoRingBaseLoVf                     = 0x00C80010,
	HWPfHiInfoRingBaseHiVf                     = 0x00C80014,
	HWPfHiInfoRingPointerVf                    = 0x00C80018,
	HWPfHiInfoRingIntWrEnVf                    = 0x00C80020,
	HWPfHiInfoRingPf2VfWrEnVf                  = 0x00C80024,
	HWPfHiMsixVectorMapperVf                   = 0x00C80060,
	HWPfHiModuleVersionReg                     = 0x00C84000,
	HWPfHiIosf2axiErrLogReg                    = 0x00C84004, /* RW/1C */
	HWPfHiHardResetReg                         = 0x00C84008,
	HWPfHi5GHardResetReg                       = 0x00C8400C,
	HWPfHiCoresHardResetReg                    = 0x00C84010,
	HWPfHiInfoRingBaseLoRegPf                  = 0x00C84014,
	HWPfHiInfoRingBaseHiRegPf                  = 0x00C84018,
	HWPfHiInfoRingPointerRegPf                 = 0x00C8401C,
	HWPfHiInfoRingIntWrEnRegPf                 = 0x00C84020,
	HWPfHiInfoRingVf2pfLoWrEnReg               = 0x00C84024,
	HWPfHiInfoRingVf2pfHiWrEnReg               = 0x00C84028,
	HWPfHiLogParityErrStatusReg                = 0x00C8402C,
	HWPfHiLogDataParityErrorVfStatusLo         = 0x00C84030,
	HWPfHiLogDataParityErrorVfStatusHi         = 0x00C84034,
	HWPfHiBlockTransmitOnErrorEn               = 0x00C84038,
	HWPfHiCfgMsiIntWrEnRegPf                   = 0x00C84040,
	HWPfHiCfgMsiVf2pfLoWrEnReg                 = 0x00C84044,
	HWPfHiCfgMsiVf2pfHighWrEnReg               = 0x00C84048,
	HWPfHiMsixVectorMapperPf                   = 0x00C84060,
	HWPfHiApbWrWaitTime                        = 0x00C84100,
	HWPfHiXCounterMaxValue                     = 0x00C84104,
	HWPfHiPfMode                               = 0x00C84108,
	HWPfHiClkGateHystReg                       = 0x00C8410C,
	HWPfHiSnoopBitsReg                         = 0x00C84110,
	HWPfHiMsiDropEnableReg                     = 0x00C84114,
	HWPfHiMsiStatReg                           = 0x00C84120,
	HWPfHiFifoOflStatReg                       = 0x00C84124,
	HWPfHiSectionPowerGatingReq                = 0x00C84128,
	HWPfHiSectionPowerGatingAck                = 0x00C8412C,
	HWPfHiSectionPowerGatingWaitCounter        = 0x00C84130,
	HWPfHiHiDebugReg                           = 0x00C841F4,
	HWPfHiDebugMemSnoopMsiFifo                 = 0x00C841F8,
	HWPfHiDebugMemSnoopInputFifo               = 0x00C841FC,
	HWPfHiMsixMappingConfig                    = 0x00C84200,
	HWPfHiErrInjectReg			   = 0x00C84204,
	HWPfHiErrStatusReg			   = 0x00C84208, /* RO/C */
	HWPfHiErrMaskReg		           = 0x00C8420C,
	HWPfHiErrFatalReg		           = 0x00C84210,
	HWPfHiJunkReg                              = 0x00C8FF00,
	HWPfHiMSIXBaseLoRegPf                      = 0x00D20000,
	HWPfHiMSIXBaseHiRegPf                      = 0x00D20004,
	HWPfHiMSIXBaseDataRegPf                    = 0x00D20008,
	HWPfHiMSIXBaseMaskRegPf                    = 0x00D2000c,
	HWPfHiMSIXPBABaseLoRegPf                   = 0x00E01000,
};

/* TIP PF Interrupt numbers */
enum {
	VRB2_PF_INT_QMGR_AQ_OVERFLOW = 0,
	VRB2_PF_INT_DOORBELL_VF_2_PF = 1,
	VRB2_PF_INT_ILLEGAL_FORMAT = 2,
	VRB2_PF_INT_QMGR_DISABLED_ACCESS = 3,
	VRB2_PF_INT_QMGR_AQ_OVERTHRESHOLD = 4,
	VRB2_PF_INT_DMA_DL_DESC_IRQ = 5,
	VRB2_PF_INT_DMA_UL_DESC_IRQ = 6,
	VRB2_PF_INT_DMA_FFT_DESC_IRQ = 7,
	VRB2_PF_INT_DMA_UL5G_DESC_IRQ = 8,
	VRB2_PF_INT_DMA_DL5G_DESC_IRQ = 9,
	VRB2_PF_INT_DMA_MLD_DESC_IRQ = 10,
	VRB2_PF_INT_ARAM_ACCESS_ERR = 11,
	VRB2_PF_INT_ARAM_ECC_1BIT_ERR = 12,
	VRB2_PF_INT_PARITY_ERR = 13,
	VRB2_PF_INT_QMGR_OVERFLOW = 14,
	VRB2_PF_INT_QMGR_ERR = 15,
	VRB2_PF_INT_ATS_ERR = 22,
	VRB2_PF_INT_ARAM_FUUL = 23,
	VRB2_PF_INT_EXTRA_READ = 24,
	VRB2_PF_INT_COMPLETION_TIMEOUT = 25,
	VRB2_PF_INT_CORE_HANG = 26,
	VRB2_PF_INT_DMA_HANG = 28,
	VRB2_PF_INT_DS_HANG = 27,
};

enum {
	VRB2_QMGR_AQ_OVERFLOW = 0,
	VRB2_DOORBELL_VFTOPF = 1,
	VRB2_ILLEGAL_FORMAT = 2,
	VRB2_QMGR_DISABLED_ACCESS = 3,
	VRB2_QMGR_AQ_OVER_THRESHOLD = 4,
	VRB2_4G_DMA_DL_DESC_IRQ = 5,
	VRB2_4G_DMA_UL_DESC_IRQ = 6,
	VRB2_FFT_DESC_IRQ = 7,
	VRB2_5G_DMA_UL_DESC_IRQ = 8,
	VRB2_5G_DMA_DL_DESC_IRQ = 9,
	VRB2_MLD_DESCRIPTOR_IRQ = 10,
	VRB2_PAGE_FAULT_ATS_ERROR = 11,
	VRB2_ARAM_ECC_1BIT_ERROR = 12,
	VRB2_PARITY_ERR = 13,
	VRB2_QMGR_INTREQ_FIFO_OVERFLOW = 14,
	VRB2_QMGR_ERROR_DETECTED = 15,
	VRB2_ARAM_ACCESS_ERROR = 22,
	VRB2_QMGR_ARAM_ALMOST_FULL = 23,
	VRB2_5G_EXTRA_COMPLETION_RECVD = 24,
	VRB2_5G_COMPLETION_READ_TIMEOUT = 25,
	VRB2_CORE_HANG_DETECTED = 26,
	VRB2_DMA_CLUSTER_HANG_DETECTED = 27,
	VRB2_DOWN_STREAM_HANG_DETECTED = 28,
	VRB2_ITR_UNDEFINED = 31
};
#endif /* _VRB2_PF_ENUM_H_ */

#pragma once

// RIFF ('sfbk')
//   LIST ('INFO')
//      <ifil-ck>   ; Refers to the version of the Sound Font RIFF file 
//         ifil(<iver-rec>)
//      <isng-ck>   ; Refers to the target Sound Engine
//         isng(szSoundEngine:ZSTR)
//      <INAM-ck>   ; Refers to the Sound Font Bank Name
//         INAM(szName:ZSTR) 
//      [<irom-ck>] ; Refers to the Sound ROM Name
//         irom(szROM:ZSTR) 
//      [<iver-ck>] ; Refers to the Sound ROM Version
//         iver(<iver-rec>)
//            WORD wMajor;
//            WORD wMinor;
//      [<ICRD-ck>] ; Refers to the Date of Creation of the Bank
//         ICRD(szDate:ZSTR) 
//      [<IENG-ck>] ; Sound Designers and Engineers for the Bank
//         IENG(szName:ZSTR)
//      [<IPRD-ck>] ; Product for which the Bank was intended
//         IPRD(szProduct:ZSTR)
//      [<ICOP-ck>] ; Contains any Copyright message
//         ICOP(szCopyright:ZSTR) 
//      [<ICMT-ck>] ; Contains any Comments on the Bank
//         ICMT(szComment:ZSTR) 
//      [<ISFT-ck>] ; The SoundFont tools used to create and alter the bank 
//         ISFT(szTools:ZSTR) 
//   LIST('sdta')
//      [<smpl-ck>] ; The Digital Audio Samples for the upper 16 bits
//         smpl(<sample:SHORT>) 
//      [<sm24-ck>] ; The Digital Audio Samples for the lower 8 bits 
//   LIST('pdta')
//      <phdr-ck>  ; The Preset Headers
//         phdr(<phdr-rec>)
//            CHAR achPresetName[20];
//            WORD wPreset;
//            WORD wBank;
//            WORD wPresetBagNdx;
//            DWORD dwLibrary;
//            DWORD dwGenre;
//            DWORD dwMorphology
//      <pbag-ck>  ; The Preset Index list
//         pbag(<pbag-rec>) 
//      <pmod-ck>  ; The Preset Modulator list
//         pmod(<pmod-rec>)
//      <pgen-ck>  ; The Preset Generator list
//         pgen(<pgen-rec>)
//      <inst-ck>  ; The Instrument Names and Indices
//         inst (<inst -rec>)
//      <ibag-ck>  ; The Instrument Index list
//         ibag(<ibag-rec>)
//      <imod-ck>  ; The Instrument Modulator list
//         imod(<imod-rec>)
//      <igen-ck>  ; The Instrument Generator list
//         igen(<igen-rec>)
//      <shdr-ck>  ; The Sample Headers 
//         shdr(<shdr-rec>)

#ifndef packed_struct
	#ifdef __GNUC__
		#include <arpa/inet.h>
		#define packed_struct __attribute__((__packed__))
		#define path_div "/"
	#else
		#include <Winsock2.h>
		#define packed_packed_struct _Pragma("pack(1)") struct
		#define path_div "\\"
	#endif
#endif

typedef enum sf2enum {
	sfg_startAddrsOffset = 0,
	sfg_endAddrsOffset = 1,
	sfg_startloopAddrsOffset = 2,
	sfg_endloopAddrsOffset = 3,
	sfg_startAddrsCoarseOffset = 4,
	sfg_modLfoToPitch = 5,
	sfg_vibLfoToPitch = 6,
	sfg_modEnvToPitch = 7,
	sfg_initialFilterFc = 8,
	sfg_initialFilterQ = 9,
	sfg_modLfoToFilterFc = 10,
	sfg_modEnvToFilterFc = 11,
	sfg_modLfoToVolume = 13,
	sfg_unused1 = 14,
	sfg_chorusEffectsSend = 15,
	sfg_reverbEffectsSend = 16,
	sfg_pan = 17,
	sfg_unused2 = 18,
	sfg_unused3 = 19,
	sfg_unused4 = 20,
	sfg_delayModLFO = 21,
	sfg_freqModLFO = 22,
	sfg_delayVibLFO = 23,
	sfg_freqVibLFO = 24,
	sfg_delayModEnv = 25,
	sfg_attackModEnv = 26,
	sfg_holdModEnv = 27,
	sfg_decayModEnv = 28,
	sfg_sustainModEnv = 29,
	sfg_releaseModEnv = 30,
	sfg_keynumToModEnvHold = 31,
	sfg_keynumToModEnvDecay = 32,
	sfg_delayVolEnv = 33,
	sfg_attackVolEnv = 34,
	sfg_holdVolEnv = 35,
	sfg_decayVolEnv = 36,
	sfg_sustainVolEnv = 37,
	sfg_releaseVolEnv = 38,
	sfg_keynumToVolEnvHold = 39,
	sfg_keynumToVolEnvDecay = 40,
	sfg_instrument = 41,
	sfg_reserved1 = 42,
	sfg_keyRange = 43,
	sfg_velRange = 44,
	sfg_startloopAddrsCoarseOffset = 45,
	sfg_keynum = 46,
	sfg_initialAttenuation = 48,
	sfg_reserved2 = 49,
	sfg_endloopAddrsCoarseOffset = 50,
	sfg_coarseTune = 51,
	sfg_fineTune = 52,
	sfg_sampleID = 53,
	sfg_sampleModes = 54,
	sfg_reserved3 = 55,
	sfg_scaleTuning = 56,
	sfg_exclusiveClass = 57,
	sfg_overridingRootKey = 58,
	sfg_unused5 = 59,
	sfg_endOper = 60,
} SFGenerator;

typedef enum sfModulator {
	sfm_noController = 0,
	sfm_noteOnVolume = 2,
	sfm_noteOnKeyNumber = 3,
	sfm_polyPressure = 10,
	sfm_channelPressure = 13,
	sfm_pitchWheel = 14,
	sfm_pitchWheelSensitivity = 16,
	sfm_link = 127
} SFModulator;

typedef enum SFSampleLink {
	sfsl_MonoSample = 1,
	sfsl_RightSample = 2,
	sfsl_LeftSample = 4,
	sfsl_LinkedSample = 8,
	sfsl_RomMonoSample = 0x8001,
	sfsl_RomRightSample = 0x8002,
	sfsl_RomLeftSample = 0x8004,
	sfsl_RomLinkedSample = 0x8008
} SFSampleLink;

typedef enum SFGeneralController {
	sfgc_NoController = 0, 
	sfgc_NoteOnVelocity = 2, 
	sfgc_NoteOnKeyNumber = 3, 
	sfgc_PolyPressure = 10,
	sfgc_ChannelPressure = 13, 
	sfgc_PitchWheel = 14, 
	sfgc_PitchWheelSensitivity = 16, 
	sfgc_Link = 127
} SFGeneralController;

typedef enum SFMidiController {
	sfmc_BankSelect = 0,
	sfmc_ModulationDepth,
	sfmc_Controller2,
	sfmc_Controller3,
	sfmc_Controller4,
	sfmc_PortamentoTime,
	sfmc_DataEntry,
	sfmc_ChannelVolume,
	sfmc_Controller8,
	sfmc_Controller9,
	sfmc_Pan,
	sfmc_Expression,
	sfmc_Controller12,
	sfmc_Controller13,
	sfmc_Controller14,
	sfmc_Controller15,
	sfmc_Controller16,
	sfmc_Controller17,
	sfmc_Controller18,
	sfmc_Controller19,
	sfmc_Controller20,
	sfmc_Controller21,
	sfmc_Controller22,
	sfmc_Controller23,
	sfmc_Controller24,
	sfmc_Controller25,
	sfmc_Controller26,
	sfmc_Controller27,
	sfmc_Controller28,
	sfmc_Controller29,
	sfmc_Controller30,
	sfmc_Controller31,
	sfmc_BankSelectLSB,
	sfmc_Controller33,
	sfmc_Controller34,
	sfmc_Controller35,
	sfmc_Controller36,
	sfmc_Controller37,
	sfmc_DataEntryLSB,
	sfmc_Controller39,
	sfmc_Controller40,
	sfmc_Controller41,
	sfmc_Controller42,
	sfmc_Controller43,
	sfmc_Controller44,
	sfmc_Controller45,
	sfmc_Controller46,
	sfmc_Controller47,
	sfmc_Controller48,
	sfmc_Controller49,
	sfmc_Controller50,
	sfmc_Controller51,
	sfmc_Controller52,
	sfmc_Controller53,
	sfmc_Controller54,
	sfmc_Controller55,
	sfmc_Controller56,
	sfmc_Controller57,
	sfmc_Controller58,
	sfmc_Controller59,
	sfmc_Controller60,
	sfmc_Controller61,
	sfmc_Controller62,
	sfmc_Controller63,
	sfmc_Hold,
	sfmc_Portamento,
	sfmc_Sostenuto,
	sfmc_Soft,
	sfmc_Controller68,
	sfmc_Controller69,
	sfmc_Controller70,
	sfmc_FilterResonance,
	sfmc_ReleaseTime,
	sfmc_AttackTime,
	sfmc_Brightness,
	sfmc_DecayTime,
	sfmc_VibratoRate,
	sfmc_VibratoDepth,
	sfmc_VibratoDelay,
	sfmc_Controller79,
	sfmc_Controller80,
	sfmc_Controller81,
	sfmc_Controller82,
	sfmc_Controller83,
	sfmc_Controller84,
	sfmc_Controller85,
	sfmc_Controller86,
	sfmc_Controller87,
	sfmc_Controller88,
	sfmc_Controller89,
	sfmc_Controller90,
	sfmc_ReverbSendLevel,
	sfmc_Controller92,
	sfmc_ChorusSendLevel,
	sfmc_Controller94,
	sfmc_Controller95,
	sfmc_Controller96,
	sfmc_Controller97,
	sfmc_NRPNLSB,
	sfmc_NRPNMSB,
	sfmc_RPNLSB,
	sfmc_RPNMSB,
	sfmc_Controller102,
	sfmc_Controller103,
	sfmc_Controller104,
	sfmc_Controller105,
	sfmc_Controller106,
	sfmc_Controller107,
	sfmc_Controller108,
	sfmc_Controller109,
	sfmc_Controller110,
	sfmc_Controller111,
	sfmc_Controller112,
	sfmc_Controller113,
	sfmc_Controller114,
	sfmc_Controller115,
	sfmc_Controller116,
	sfmc_Controller117,
	sfmc_Controller118,
	sfmc_Controller119,
	sfmc_AllSoundOff,
	sfmc_ResetAllController,
	sfmc_Controller122,
	sfmc_AllNotesOff,
	sfmc_OmniModeOff,
	sfmc_OmniModeOn,
	sfmc_MonoModeOn,
	sfmc_PolyModeOn
} SFMidiController;

typedef enum SFTransform { 
	sft_Linear = 0,
	sft_AbsoluteValue = 2
} SFTransform;

typedef enum SampleMode { 
	sfsm_NoLoop = 0, 
	sfsm_LoopContinuously, 
	sfsm_UnusedNoLoop, 
	sfsm_LoopEndsByKeyDepression
} SampleMode;

typedef enum SFControllerPolarity { 
	sfcp_Unipolar = 0, 
	sfcp_Bipolar = 1
} SFControllerPolarity;

typedef enum SFControllerType { 
	sfct_Linear = 0, 
	sfct_Concave = 1, 
	sfct_Convex = 2, 
	sfct_Switch = 3 
} SFControllerType;

typedef enum SFControllerPalette { 
	sfcpa_GeneralController = 0, 
	sfcpa_MidiController = 1 
} SFControllerPalette;

typedef enum SFControllerDirection { 
	sfcd_Increase = 0, 
	sfcd_Decrease = 1
} SFControllerDirection;

#ifndef DWORD
	#define DWORD uint32_t
#endif

#ifndef WORD
	#define WORD uint16_t
#endif

#ifndef SHORT
	#define SHORT int16_t
#endif

#ifndef BYTE
	#define BYTE uint8_t
#endif

#ifndef CHAR
	#define CHAR int8_t
#endif

packed_struct sfVersionTag {
	WORD wMajor;
	WORD wMinor;
}; 

packed_struct sfPresetHeader {
	CHAR achPresetName[20];
	WORD wPreset;
	WORD wBank;
	WORD wPresetBagNdx;
	DWORD dwLibrary;
	DWORD dwGenre;
	DWORD dwMorphology;
}; 

packed_struct sfPresetBag {
	WORD wGenNdx;
	WORD wModNdx;
}; 

packed_struct sfModList {
	WORD sfModSrcOper;
	WORD sfModDestOper;
	SHORT modAmount;
	WORD sfModAmtSrcOper;
	WORD sfModTransOper;
}; 

typedef packed_struct {
	BYTE byLo;
	BYTE byHi;
} rangesType;

typedef union {
	rangesType ranges;
	SHORT shAmount;
	WORD wAmount;
} genAmountType; 

packed_struct sfGenList {
	WORD sfGenOper;
	genAmountType genAmount;
}; 

packed_struct sfInst {
	CHAR achInstName[20];
	WORD wInstBagNdx;
}; 

packed_struct sfInstBag {
	WORD wInstGenNdx;
	WORD wInstModNdx;
};

packed_struct sfInstGenList {
	WORD sfGenOper;
	genAmountType genAmount;
}; 

packed_struct sfSample {
	CHAR achSampleName[20];
	DWORD dwStart;
	DWORD dwEnd;
	DWORD dwStartloop;
	DWORD dwEndloop;
	DWORD dwSampleRate;
	BYTE byOriginalPitch;
	CHAR chPitchCorrection;
	WORD wSampleLink;
	WORD sfSampleType;
}; 


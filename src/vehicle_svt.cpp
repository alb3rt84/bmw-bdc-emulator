/**
 * @file vehicle_svt.cpp
 * @brief G20 SVT-IST baked into the emulator (WBA5V510X0FJ28775).
 *
 * Source: pc_companion/data/SVT.xml. Each ECU answers UDS 22 F1 01
 * with SVK version, the programming-dependency flag, a big-endian
 * count, then 8-byte SGBMIDs (class, id, main, sub, patch).
 */

#include "vehicle_svt.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>
#include <string.h>

namespace vehicle_svt {

namespace {

struct Sgbm {
  uint8_t klass;
  uint8_t mainVersion;
  uint8_t subVersion;
  uint8_t patchVersion;
  uint32_t id;
};

struct Ecu {
  uint8_t addr;
  uint8_t svkVersion;
  uint8_t progDep;
  uint8_t count;
  const Sgbm* parts;
  const char* name;
};

const Sgbm k_BDC_GW3_10[] = {
    {0x01, 13, 0, 2, 0x00004159u},
    {0x02, 255, 255, 255, 0x00001E5Cu},
    {0x02, 255, 255, 255, 0x00001E5Du},
    {0x02, 255, 255, 255, 0x00001E62u},
    {0x02, 255, 255, 255, 0x00004162u},
    {0x02, 255, 255, 255, 0x00004163u},
    {0x02, 255, 255, 255, 0x00004165u},
    {0x02, 255, 255, 255, 0x00004564u},
    {0x02, 255, 255, 255, 0x00004565u},
    {0x04, 12, 58, 1, 0x0000413Bu},
    {0x06, 142, 10, 20, 0x00004139u},
    {0x08, 142, 10, 20, 0x0000413Au},
};

const Sgbm k_BDC_BODY3_40[] = {
    {0x01, 13, 0, 2, 0x00004159u},
    {0x02, 255, 255, 255, 0x00000F2Bu},
    {0x02, 255, 255, 255, 0x00000F2Du},
    {0x02, 255, 255, 255, 0x00000F2Fu},
    {0x02, 255, 255, 255, 0x00000F32u},
    {0x02, 255, 255, 255, 0x00001F7Eu},
    {0x02, 255, 255, 255, 0x0000415Bu},
    {0x02, 255, 255, 255, 0x0000415Cu},
    {0x02, 255, 255, 255, 0x00004518u},
    {0x05, 203, 70, 2, 0x000017BDu},
    {0x05, 16, 71, 0, 0x00001DF8u},
    {0x05, 8, 34, 241, 0x000044EDu},
    {0x05, 16, 59, 3, 0x00004694u},
    {0x05, 17, 74, 7, 0x000051DDu},
    {0x05, 13, 77, 1, 0x00005665u},
    {0x05, 32, 83, 11, 0x00007083u},
    {0x06, 142, 11, 20, 0x00005F9Eu},
    {0x08, 142, 11, 20, 0x00005F9Fu},
};

const Sgbm k_DCS_45[] = {
    {0x01, 2, 12, 0, 0x00004766u},
    {0x05, 1, 20, 3, 0x0000475Cu},
    {0x06, 1, 20, 3, 0x00006183u},
    {0x08, 1, 20, 3, 0x00006184u},
    {0x08, 1, 20, 3, 0x00006185u},
    {0x08, 1, 20, 3, 0x00006186u},
    {0x08, 1, 20, 3, 0x00006187u},
    {0x08, 1, 20, 3, 0x00006188u},
};

const Sgbm k_KAFAS4_5D[] = {
    {0x01, 3, 1, 2, 0x00004902u},
    {0x05, 13, 2, 13, 0x000040F8u},
    {0x05, 13, 5, 26, 0x000040F9u},
    {0x06, 4, 20, 149, 0x00004901u},
    {0x08, 194, 80, 110, 0x00004903u},
    {0x08, 194, 80, 110, 0x00004904u},
    {0x08, 144, 50, 101, 0x00004905u},
};

const Sgbm k_FRR2_21[] = {
    {0x01, 4, 10, 0, 0x0000223Fu},
    {0x05, 45, 0, 45, 0x0000469Au},
    {0x06, 40, 15, 3, 0x00004769u},
    {0x08, 40, 72, 1, 0x00004699u},
    {0x08, 40, 9, 1, 0x0000476Au},
};

const Sgbm k_RAM_37[] = {
    {0x01, 0, 6, 0, 0x00004236u},
    {0x05, 18, 4, 1, 0x00004224u},
    {0x06, 19, 20, 15, 0x00005ED7u},
    {0x08, 19, 20, 5, 0x00005EDBu},
    {0x08, 19, 20, 10, 0x00005EDFu},
    {0x08, 19, 20, 15, 0x00005EE1u},
    {0x08, 7, 25, 0, 0x00005EE6u},
    {0x08, 16, 100, 40, 0x00005EE8u},
    {0x0D, 15, 5, 53, 0x00006117u},
};

const Sgbm k_DME_BAC2_12[] = {
    {0x01, 1, 4, 0, 0x00003DAEu},
    {0x02, 255, 255, 255, 0x000042C2u},
    {0x05, 0, 23, 86, 0x000029B7u},
    {0x06, 7, 33, 0, 0x000040C4u},
    {0x08, 7, 33, 0, 0x000040C5u},
    {0x08, 7, 33, 18, 0x000078E6u},
};

const Sgbm k_EPS3_30[] = {
    {0x01, 4, 3, 1, 0x000033CCu},
    {0x01, 2, 0, 0, 0x0000450Au},
    {0x02, 255, 255, 255, 0x000033CDu},
    {0x05, 21, 4, 2, 0x000033C9u},
    {0x06, 10, 10, 8, 0x00004E96u},
    {0x08, 10, 240, 0, 0x00004E97u},
    {0x08, 1, 240, 0, 0x00005875u},
    {0x0D, 1, 240, 0, 0x0000587Au},
};

const Sgbm k_DSC_VIP_IB4_39[] = {
    {0x01, 7, 3, 0, 0x000040C3u},
    {0x05, 10, 3, 3, 0x00003D78u},
    {0x06, 4, 5, 2, 0x00007FC9u},
    {0x08, 9, 18, 20, 0x00005B86u},
    {0x08, 9, 18, 20, 0x00005B87u},
};

const Sgbm k_ICAM3_06[] = {
    {0x01, 4, 7, 1, 0x0000807Du},
    {0x05, 6, 23, 6, 0x00008081u},
    {0x06, 4, 7, 11, 0x00008C15u},
    {0x08, 1, 2, 5, 0x0000833Du},
    {0x08, 3, 5, 5, 0x00008C17u},
    {0x08, 20, 2, 1, 0x00008C19u},
    {0x08, 4, 1, 1, 0x00008F16u},
};

const Sgbm k_ACSM5_01[] = {
    {0x01, 6, 3, 0, 0x000046B2u},
    {0x05, 9, 5, 4, 0x00001B29u},
    {0x05, 9, 13, 0, 0x00001B2Au},
    {0x05, 5, 24, 0, 0x00001B2Bu},
    {0x05, 2, 10, 41, 0x00001B2Cu},
    {0x05, 2, 12, 2, 0x00001B2Du},
    {0x05, 4, 2, 0, 0x00002ABFu},
    {0x06, 6, 82, 0, 0x00001B2Eu},
    {0x08, 6, 82, 0, 0x00001B2Fu},
};

const Sgbm k_EGS7_18[] = {
    {0x01, 30, 14, 3, 0x00004326u},
    {0x05, 1, 1, 23, 0x000040EBu},
    {0x06, 14, 26, 2, 0x00005F6Du},
    {0x08, 50, 93, 0, 0x00005F6Fu},
    {0x08, 19, 93, 0, 0x00005F70u},
    {0x0D, 93, 178, 14, 0x00007622u},
};

const Sgbm k_SRR_08[] = {
    {0x01, 1, 4, 0, 0x00004141u},
    {0x05, 3, 37, 4, 0x00004146u},
    {0x06, 3, 0, 9, 0x00007023u},
    {0x08, 2, 37, 1, 0x00005ED0u},
    {0x08, 2, 37, 0, 0x00005ED1u},
    {0x08, 2, 37, 0, 0x00005ED2u},
};

const Sgbm k_SAS3_23[] = {
    {0x01, 1, 3, 2, 0x000042D3u},
    {0x05, 3, 10, 2, 0x000042C1u},
    {0x06, 3, 3, 1, 0x0000702Bu},
    {0x08, 3, 172, 160, 0x00005943u},
    {0x08, 3, 172, 160, 0x00005944u},
};

const Sgbm k_DSC_BRS_29[] = {
    {0x01, 7, 3, 0, 0x000040C3u},
    {0x05, 9, 2, 17, 0x00003D72u},
    {0x06, 4, 3, 2, 0x00005B7Eu},
    {0x08, 9, 18, 31, 0x00005B85u},
    {0x0D, 9, 18, 31, 0x00005ED5u},
    {0x0D, 9, 18, 31, 0x00009916u},
};

const Sgbm k_DKOMBI4_60[] = {
    {0x01, 3, 4, 0, 0x0000467Fu},
    {0x05, 8, 3, 25, 0x00004508u},
    {0x06, 160, 2, 2, 0x00005EA5u},
    {0x08, 160, 2, 2, 0x00005C57u},
    {0x08, 160, 2, 2, 0x00005C58u},
    {0x08, 160, 2, 2, 0x00005C59u},
    {0x08, 160, 2, 2, 0x00005C5Au},
    {0x07, 160, 2, 2, 0x00005C56u},
};

const Sgbm k_ATM2_61[] = {
    {0x01, 103, 4, 4, 0x0000451Bu},
    {0x05, 1, 10, 8, 0x00004514u},
    {0x06, 16, 2, 20, 0x00004A51u},
    {0x08, 16, 2, 20, 0x00004A53u},
    {0x08, 16, 2, 20, 0x00004A54u},
    {0x08, 16, 2, 20, 0x00004A55u},
    {0x08, 16, 2, 20, 0x00004A56u},
    {0x08, 16, 2, 20, 0x00004A57u},
    {0x08, 16, 2, 20, 0x00004A59u},
    {0x08, 16, 2, 20, 0x00004A5Au},
    {0x07, 16, 2, 20, 0x00004A52u},
};

const Sgbm k_HU_MGU_63[] = {
    {0x01, 1, 1, 4, 0x00005F32u},
    {0x05, 8, 10, 124, 0x00003E52u},
    {0x06, 30, 16, 6, 0x00003F36u},
    {0x08, 30, 16, 2, 0x00005684u},
    {0x08, 30, 16, 2, 0x00005685u},
    {0x08, 30, 16, 2, 0x00005686u},
    {0x08, 30, 16, 2, 0x00005687u},
    {0x08, 30, 16, 2, 0x000056E5u},
    {0x08, 30, 16, 2, 0x000056E6u},
    {0x08, 30, 16, 2, 0x000056E7u},
    {0x08, 30, 16, 2, 0x000056E8u},
    {0x08, 30, 16, 2, 0x000056E9u},
    {0x08, 30, 16, 2, 0x000056EAu},
    {0x08, 30, 16, 2, 0x000056EBu},
    {0x08, 30, 16, 2, 0x000056ECu},
    {0x08, 30, 16, 2, 0x000056EEu},
    {0x08, 30, 16, 2, 0x00005CC5u},
    {0x0D, 30, 16, 2, 0x00005688u},
    {0x0D, 30, 16, 2, 0x00005689u},
    {0x0D, 30, 16, 2, 0x0000568Au},
    {0x0D, 30, 16, 2, 0x0000568Bu},
    {0x0D, 30, 16, 2, 0x0000568Cu},
    {0x0D, 30, 16, 2, 0x0000568Du},
    {0x0D, 30, 16, 2, 0x0000568Eu},
    {0x0D, 30, 16, 2, 0x0000568Fu},
    {0x0D, 30, 16, 2, 0x0000569Bu},
    {0x0D, 30, 16, 2, 0x000056AAu},
    {0x0D, 30, 16, 2, 0x000056B5u},
    {0x0D, 30, 16, 2, 0x000056B8u},
    {0x0D, 30, 16, 2, 0x000056B9u},
    {0x0D, 30, 16, 2, 0x000056BEu},
    {0x0D, 30, 16, 2, 0x000056BFu},
    {0x0D, 30, 16, 2, 0x000056C0u},
    {0x0D, 30, 16, 2, 0x000056C1u},
    {0x0D, 30, 16, 2, 0x000056C4u},
    {0x0D, 30, 16, 2, 0x000056C6u},
    {0x0D, 30, 16, 2, 0x000056C7u},
    {0x0D, 30, 16, 2, 0x000056C8u},
    {0x0D, 30, 16, 2, 0x000056C9u},
    {0x0D, 30, 16, 2, 0x000056CAu},
    {0x0D, 30, 16, 2, 0x000056CBu},
    {0x0D, 30, 16, 2, 0x000056D0u},
    {0x0D, 30, 16, 2, 0x000056D1u},
    {0x0D, 30, 16, 2, 0x000056D4u},
    {0x0D, 30, 16, 2, 0x000056D5u},
    {0x0D, 30, 16, 2, 0x000056D7u},
    {0x0D, 30, 16, 2, 0x000056D8u},
    {0x0D, 30, 16, 2, 0x000056D9u},
    {0x0D, 30, 16, 2, 0x000056DAu},
    {0x0D, 30, 16, 2, 0x000056DBu},
    {0x0D, 30, 16, 2, 0x000056DCu},
    {0x0D, 30, 16, 2, 0x000056DEu},
    {0x0D, 30, 16, 2, 0x000056E0u},
    {0x0D, 30, 16, 2, 0x00005A94u},
    {0x0D, 30, 16, 2, 0x00005A95u},
    {0x0D, 30, 16, 2, 0x00005A96u},
    {0x0D, 30, 16, 2, 0x00005CC6u},
    {0x0D, 30, 16, 2, 0x00005CC7u},
    {0x0D, 30, 16, 2, 0x00005CCAu},
    {0x0D, 30, 16, 2, 0x00006949u},
    {0x0D, 30, 16, 2, 0x000073B7u},
    {0x0D, 30, 16, 2, 0x00007CFCu},
    {0x0D, 30, 16, 2, 0x000083B6u},
    {0x0D, 30, 16, 2, 0x000083B7u},
    {0x0D, 30, 16, 2, 0x000083B8u},
    {0xA0, 11, 19, 3, 0x000055EBu},
    {0xA1, 255, 9, 232, 0x000046A0u},
};

const Sgbm k_SRR_28[] = {
    {0x01, 1, 4, 0, 0x00004141u},
    {0x05, 3, 23, 15, 0x00004146u},
    {0x06, 3, 0, 9, 0x00007023u},
    {0x08, 2, 23, 3, 0x00005ED0u},
    {0x08, 2, 23, 0, 0x00005ED1u},
    {0x08, 2, 23, 0, 0x00005ED2u},
};

const Sgbm k_SRR_2A[] = {
    {0x01, 1, 4, 0, 0x00004141u},
    {0x05, 3, 37, 4, 0x00004146u},
    {0x06, 3, 0, 9, 0x00007023u},
    {0x08, 2, 37, 1, 0x00005ED0u},
    {0x08, 2, 37, 0, 0x00005ED1u},
    {0x08, 2, 37, 0, 0x00005ED2u},
};

const Sgbm k_SRR_20[] = {
    {0x01, 1, 4, 0, 0x00004141u},
    {0x05, 3, 37, 4, 0x00004146u},
    {0x06, 3, 0, 9, 0x00007023u},
    {0x08, 2, 37, 1, 0x00005ED0u},
    {0x08, 2, 37, 0, 0x00005ED1u},
    {0x08, 2, 37, 0, 0x00005ED2u},
};

const Sgbm k_IHKA4_78[] = {
    {0x01, 1, 0, 0, 0x000051C7u},
    {0x05, 4, 31, 1, 0x000051C9u},
    {0x06, 1, 9, 14, 0x0000AC95u},
    {0x08, 1, 60, 0, 0x000051C4u},
};

const Sgbm k_SCR2_0B[] = {
    {0x01, 3, 0, 1, 0x00002BB8u},
    {0x06, 4, 6, 2, 0x000031E2u},
    {0x08, 9, 4, 0, 0x00003306u},
    {0x0D, 9, 4, 7, 0x00005B78u},
};

const Sgbm k_USS_2C[] = {
    {0x01, 200, 5, 1, 0x000042D6u},
    {0x05, 49, 4, 3, 0x0000430Au},
    {0x06, 1, 6, 10, 0x00005D59u},
    {0x08, 1, 35, 1, 0x00005D5Au},
};

const Sgbm k_ZBE4_67[] = {
    {0x01, 5, 0, 3, 0x0000428Bu},
    {0x02, 255, 255, 255, 0x00005BF6u},
    {0x06, 5, 8, 0, 0x000058A1u},
    {0x08, 5, 8, 0, 0x000058A2u},
    {0x08, 5, 6, 0, 0x000058A3u},
};

const Sgbm k_FLM2_43[] = {
    {0x01, 5, 0, 0, 0x00005CFFu},
    {0x05, 16, 17, 0, 0x000043AFu},
    {0x06, 100, 0, 2, 0x00005CFAu},
    {0x08, 104, 0, 1, 0x00005CFBu},
    {0x0D, 1, 9, 9, 0x00005CFCu},
};

const Sgbm k_GWS2_5E[] = {
    {0x01, 3, 2, 3, 0x00004885u},
    {0x02, 255, 255, 255, 0x00005938u},
    {0x05, 6, 3, 1, 0x000042DAu},
    {0x06, 3, 14, 0, 0x00005897u},
    {0x08, 3, 20, 0, 0x00005898u},
};

const Sgbm k_FLM2_44[] = {
    {0x01, 5, 0, 0, 0x00005CFFu},
    {0x05, 16, 17, 0, 0x000043AFu},
    {0x06, 100, 0, 2, 0x00005CFAu},
    {0x08, 104, 0, 1, 0x00005CFBu},
    {0x0D, 1, 9, 9, 0x00005CFCu},
};

const Sgbm k_FZD2_56[] = {
    {0x01, 5, 7, 0, 0x00001D95u},
    {0x01, 5, 7, 0, 0x00001D97u},
    {0x01, 5, 7, 0, 0x00001D98u},
    {0x01, 5, 7, 0, 0x00001D99u},
    {0x01, 5, 7, 0, 0x0000318Cu},
    {0x01, 5, 7, 0, 0x00003CDCu},
    {0x02, 255, 255, 255, 0x00002C7Du},
    {0x05, 6, 50, 40, 0x00001D92u},
    {0x05, 7, 25, 21, 0x00001D93u},
    {0x06, 3, 130, 0, 0x00002F47u},
    {0x08, 3, 130, 0, 0x00002F48u},
};

const Ecu kEcus[] = {
    {0x10, 1, 1, 12, k_BDC_GW3_10, "BDC_GW3"},
    {0x40, 1, 1, 18, k_BDC_BODY3_40, "BDC_BODY3"},
    {0x45, 1, 1, 8, k_DCS_45, "DCS"},
    {0x5D, 1, 1, 7, k_KAFAS4_5D, "KAFAS4"},
    {0x21, 1, 1, 5, k_FRR2_21, "FRR2"},
    {0x37, 1, 1, 9, k_RAM_37, "RAM"},
    {0x12, 1, 1, 6, k_DME_BAC2_12, "DME_BAC2"},
    {0x30, 1, 1, 8, k_EPS3_30, "EPS3"},
    {0x39, 1, 1, 5, k_DSC_VIP_IB4_39, "DSC_VIP_IB4"},
    {0x06, 1, 1, 7, k_ICAM3_06, "ICAM3"},
    {0x01, 1, 1, 9, k_ACSM5_01, "ACSM5"},
    {0x18, 1, 1, 6, k_EGS7_18, "EGS7"},
    {0x08, 1, 1, 6, k_SRR_08, "SRR"},
    {0x23, 1, 1, 5, k_SAS3_23, "SAS3"},
    {0x29, 1, 1, 6, k_DSC_BRS_29, "DSC_BRS"},
    {0x60, 1, 1, 8, k_DKOMBI4_60, "DKOMBI4"},
    {0x61, 1, 1, 11, k_ATM2_61, "ATM2"},
    {0x63, 1, 1, 67, k_HU_MGU_63, "HU_MGU"},
    {0x28, 1, 1, 6, k_SRR_28, "SRR"},
    {0x2A, 1, 1, 6, k_SRR_2A, "SRR"},
    {0x20, 1, 1, 6, k_SRR_20, "SRR"},
    {0x78, 1, 1, 4, k_IHKA4_78, "IHKA4"},
    {0x0B, 1, 1, 4, k_SCR2_0B, "SCR2"},
    {0x2C, 1, 1, 4, k_USS_2C, "USS"},
    {0x67, 1, 1, 5, k_ZBE4_67, "ZBE4"},
    {0x43, 1, 1, 5, k_FLM2_43, "FLM2"},
    {0x5E, 1, 1, 5, k_GWS2_5E, "GWS2"},
    {0x44, 1, 1, 5, k_FLM2_44, "FLM2"},
    {0x56, 1, 1, 11, k_FZD2_56, "FZD2"},
};

constexpr int kMaxEcu = 40;
constexpr int kMaxPart = 360;
constexpr int kNameLen = 20;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t g_part[kMaxPart][8];
struct LiveEcu {
  uint8_t addr;
  uint8_t ver;
  uint8_t dep;
  uint8_t count;
  uint16_t index;
  char name[kNameLen];
};
LiveEcu g_ecu[kMaxEcu];
int g_ecuN = 0;
int g_partN = 0;

uint8_t g_stagePart[kMaxPart][8];
LiveEcu g_stage[kMaxEcu];
int g_stageN = 0;
int g_stagePartN = 0;
bool g_staging = false;

void packPart(uint8_t out[8], const Sgbm& part) {
  out[0] = part.klass;
  out[1] = (uint8_t)(part.id >> 24);
  out[2] = (uint8_t)(part.id >> 16);
  out[3] = (uint8_t)(part.id >> 8);
  out[4] = (uint8_t)part.id;
  out[5] = part.mainVersion;
  out[6] = part.subVersion;
  out[7] = part.patchVersion;
}

void seedFactory() {
  g_ecuN = 0;
  g_partN = 0;
  const int n = (int)(sizeof(kEcus) / sizeof(kEcus[0]));
  for (int i = 0; i < n && g_ecuN < kMaxEcu; i++) {
    const Ecu& src = kEcus[i];
    if (g_partN + src.count > kMaxPart) break;
    LiveEcu& dst = g_ecu[g_ecuN];
    dst.addr = src.addr;
    dst.ver = src.svkVersion;
    dst.dep = src.progDep;
    dst.count = src.count;
    dst.index = (uint16_t)g_partN;
    snprintf(dst.name, sizeof(dst.name), "%s", src.name);
    for (uint8_t p = 0; p < src.count; p++) packPart(g_part[g_partN + p], src.parts[p]);
    g_partN += src.count;
    g_ecuN++;
  }
}

int findLive(uint8_t addr) {
  for (int i = 0; i < g_ecuN; i++) {
    if (g_ecu[i].addr == addr) return i;
  }
  return -1;
}

bool unpackNvs(const uint8_t* buf, size_t len) {
  if (len < 5 || buf[0] != 0x53 || buf[1] != 1) return false;
  const int ecuN = buf[2];
  const int partN = ((int)buf[3] << 8) | buf[4];
  if (ecuN <= 0 || ecuN > kMaxEcu || partN < 0 || partN > kMaxPart) return false;
  size_t i = 5;
  int filled = 0;
  LiveEcu ecus[kMaxEcu];
  for (int e = 0; e < ecuN; e++) {
    if (i + 5 > len) return false;
    LiveEcu& dst = ecus[e];
    dst.addr = buf[i++];
    dst.ver = buf[i++];
    dst.dep = buf[i++];
    dst.count = buf[i++];
    const uint8_t nameLen = buf[i++];
    if (nameLen == 0 || nameLen >= kNameLen || i + nameLen > len) return false;
    memcpy(dst.name, buf + i, nameLen);
    dst.name[nameLen] = '\0';
    i += nameLen;
    dst.index = (uint16_t)filled;
    filled += dst.count;
  }
  if (filled != partN || i + (size_t)partN * 8 != len) return false;
  memcpy(g_part, buf + i, (size_t)partN * 8);
  memcpy(g_ecu, ecus, sizeof(LiveEcu) * (size_t)ecuN);
  g_ecuN = ecuN;
  g_partN = partN;
  return true;
}

void saveNvs() {
  uint8_t buf[3900];
  size_t n = 0;
  buf[n++] = 0x53;
  buf[n++] = 1;
  buf[n++] = (uint8_t)g_ecuN;
  buf[n++] = (uint8_t)(g_partN >> 8);
  buf[n++] = (uint8_t)g_partN;
  for (int e = 0; e < g_ecuN; e++) {
    const size_t nameLen = strlen(g_ecu[e].name);
    if (n + 5 + nameLen > sizeof(buf)) return;
    buf[n++] = g_ecu[e].addr;
    buf[n++] = g_ecu[e].ver;
    buf[n++] = g_ecu[e].dep;
    buf[n++] = g_ecu[e].count;
    buf[n++] = (uint8_t)nameLen;
    memcpy(buf + n, g_ecu[e].name, nameLen);
    n += nameLen;
  }
  if (n + (size_t)g_partN * 8 > sizeof(buf)) return;
  memcpy(buf + n, g_part, (size_t)g_partN * 8);
  n += (size_t)g_partN * 8;
  Preferences prefs;
  prefs.begin("bdcsvt", false);
  prefs.putBytes("bin", buf, n);
  prefs.end();
}

}  // namespace

const char* name(uint8_t addr) {
  portENTER_CRITICAL(&g_mux);
  const int i = findLive(addr);
  const char* out = i < 0 ? nullptr : g_ecu[i].name;
  portEXIT_CRITICAL(&g_mux);
  return out;
}

size_t listAddrs(uint8_t* out, size_t max) {
  if (out == nullptr || max == 0) return 0;
  portENTER_CRITICAL(&g_mux);
  size_t n = (size_t)g_ecuN;
  if (n > max) n = max;
  for (size_t i = 0; i < n; i++) out[i] = g_ecu[i].addr;
  portEXIT_CRITICAL(&g_mux);
  return n;
}

size_t answerSvk(uint8_t addr, const uint8_t* req, size_t reqLen,
                 uint8_t* out, size_t outMax) {
  if (!req || !out || reqLen < 3 || req[0] != 0x22) return 0;
  const uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  if (did != 0xF101) return 0;
  uint8_t ver = 0;
  uint8_t dep = 0;
  uint8_t count = 0;
  uint8_t packed[80 * 8];
  portENTER_CRITICAL(&g_mux);
  const int i = findLive(addr);
  if (i >= 0) {
    ver = g_ecu[i].ver;
    dep = g_ecu[i].dep;
    count = g_ecu[i].count;
    if (count <= 80) memcpy(packed, &g_part[g_ecu[i].index], (size_t)count * 8);
  }
  portEXIT_CRITICAL(&g_mux);
  if (i < 0) return 0;
  const size_t need = 7u + (size_t)count * 8u;
  if (outMax < 3) return 0;
  if (count > 80 || outMax < need) {
    out[0] = 0x7F;
    out[1] = 0x22;
    out[2] = 0x10;
    return 3;
  }
  out[0] = 0x62;
  out[1] = 0xF1;
  out[2] = 0x01;
  out[3] = ver;
  out[4] = dep;
  out[5] = 0;
  out[6] = count;
  memcpy(out + 7, packed, (size_t)count * 8);
  return need;
}

void load() {
  seedFactory();
  uint8_t buf[3900];
  Preferences prefs;
  prefs.begin("bdcsvt", true);
  const size_t n = prefs.getBytesLength("bin");
  size_t got = 0;
  if (n > 0 && n <= sizeof(buf)) got = prefs.getBytes("bin", buf, n);
  prefs.end();
  if (got == n && n > 0) {
    portENTER_CRITICAL(&g_mux);
    if (!unpackNvs(buf, n)) seedFactory();
    portEXIT_CRITICAL(&g_mux);
  }
}

void beginReplace() {
  g_staging = true;
  g_stageN = 0;
  g_stagePartN = 0;
}

bool addEcu(uint8_t addr, const char* ecuName, uint8_t ver, uint8_t dep,
            const uint8_t* wire, uint8_t count) {
  if (!g_staging || !ecuName || !wire || count == 0 || count > 80) return false;
  if (addr == 0 || addr == 0xDF) return false;
  const size_t nameLen = strlen(ecuName);
  if (nameLen == 0 || nameLen >= kNameLen) return false;
  for (size_t c = 0; c < nameLen; c++) {
    const char ch = ecuName[c];
    const bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
    if (!ok) return false;
  }
  for (int i = 0; i < g_stageN; i++) {
    if (g_stage[i].addr == addr) return false;
  }
  if (g_stageN >= kMaxEcu || g_stagePartN + count > kMaxPart) return false;
  LiveEcu& dst = g_stage[g_stageN];
  dst.addr = addr;
  dst.ver = ver;
  dst.dep = dep;
  dst.count = count;
  dst.index = (uint16_t)g_stagePartN;
  memcpy(dst.name, ecuName, nameLen);
  dst.name[nameLen] = '\0';
  memcpy(&g_stagePart[g_stagePartN], wire, (size_t)count * 8);
  g_stagePartN += count;
  g_stageN++;
  return true;
}

bool commitReplace() {
  if (!g_staging || g_stageN == 0) return false;
  portENTER_CRITICAL(&g_mux);
  memcpy(g_ecu, g_stage, sizeof(LiveEcu) * (size_t)g_stageN);
  memcpy(g_part, g_stagePart, (size_t)g_stagePartN * 8);
  g_ecuN = g_stageN;
  g_partN = g_stagePartN;
  portEXIT_CRITICAL(&g_mux);
  g_staging = false;
  saveNvs();
  return true;
}

}  // namespace vehicle_svt

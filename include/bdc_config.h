/**
 * @file bdc_config.h
 * @brief Editable BDC identity: VIN, FA, I-Stufe, serial, DoIP LA (NVS).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bdc_config {

constexpr size_t kVinLen     = 17;
constexpr size_t kFaMax      = 240;   // ASCII FA paste from E-Sys / FAFP
constexpr size_t kIStufeMax  = 31;
constexpr size_t kSerialMax  = 31;
constexpr size_t kModelMax   = 15;    // e.g. G30 / G20

struct Identity {
  char     vin[kVinLen + 1];
  char     fa[kFaMax + 1];
  char     iStufe[kIStufeMax + 1];
  char     serial[kSerialMax + 1];
  char     model[kModelMax + 1];
  uint16_t logicalAddress;  // DoIP LA (default 0x0010)
};

bool init();                 // load NVS or defaults
const Identity& get();
Identity copy();

bool setVin(const char* vin17);
bool setFa(const char* fa);
bool setIStufe(const char* s);
bool setSerial(const char* s);
bool setModel(const char* s);
bool setLogicalAddress(uint16_t la);

/** Replace many fields at once; empty/null string = leave unchanged. */
bool applyPatch(const Identity& patch, bool hasVin, bool hasFa, bool hasIStufe,
                bool hasSerial, bool hasModel, bool hasLa);

bool save();                 // persist to NVS
bool load();
void resetDefaults();

}  // namespace bdc_config

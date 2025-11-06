#include "TMC5160_HW_Abstraction.h"

#include <string.h>

#define TMC5160_MAX_INSTANCES 8

typedef struct {
  bool initialized;
  TMC5160TypeDef ic;
  ConfigurationTypeDef config;
} TMC5160_InstanceState;

static TMC5160_InstanceState g_instances[TMC5160_MAX_INSTANCES];

static TMC5160_InstanceState *get_instance(uint16_t icID) {
  if (icID >= TMC5160_MAX_INSTANCES) {
    return NULL;
  }
  return &g_instances[icID];
}

static TMC5160_InstanceState *ensure_instance(uint16_t icID) {
  TMC5160_InstanceState *instance = get_instance(icID);
  if (!instance) {
    return NULL;
  }
  if (!instance->initialized) {
    memset(instance, 0, sizeof(*instance));
    tmc5160_init(&instance->ic, (uint8_t)icID, &instance->config,
                 tmc5160_defaultRegisterResetState);
    instance->initialized = true;
  }
  return instance;
}

void tmc5160_initCache(void) {
  memset(g_instances, 0, sizeof(g_instances));
}

int32_t tmc5160_readRegister(uint16_t icID, uint8_t address) {
  TMC5160_InstanceState *instance = ensure_instance(icID);
  if (!instance) {
    return 0;
  }
  return tmc5160_readInt(&instance->ic, address);
}

void tmc5160_writeRegister(uint16_t icID, uint8_t address, int32_t value) {
  TMC5160_InstanceState *instance = ensure_instance(icID);
  if (!instance) {
    return;
  }
  tmc5160_writeInt(&instance->ic, address, value);
}

void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length) {
  tmc5160_readWriteSPI((uint16_t)channel, data, length);
}

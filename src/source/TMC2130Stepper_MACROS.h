#ifndef TMC2130Stepper_MACROS_H
#define TMC2130Stepper_MACROS_H
#include "../TMC2130Stepper_REGDEFS.h"
#include "TMC2130Stepper.h"

// TODO for Gods's sake, it's 2020. Rewrite to some constexpr/template functions.

#define TMC_WRITE_REG(R) writeCmd (TMC2130_WRITE | REG_##R, R##_sr);

// TODO check the status.
#define TMC_READ_REG(R)                                                                                                                         \
        R##_sr = readCmd (TMC2130_READ | REG_##R).first;                                                                                        \
        return R##_sr

// TODO check the status.
#define TMC_READ_REG_R(R) return readCmd (TMC2130_READ | REG_##R).first;

#define TMC_MOD_REG(REG, SETTING)                                                                                                               \
        REG##_sr &= ~SETTING##_bm;                                                                                                              \
        REG##_sr |= ((uint32_t)B << SETTING##_bp) & SETTING##_bm;                                                                               \
        TMC_WRITE_REG (REG);

#define TMC_GET_BYTE(REG, SETTING) return (REG () & SETTING##_bm) >> SETTING##_bp;

#define TMC_GET_BYTE_R(REG, SETTING) return (REG () & SETTING##_bm) >> SETTING##_bp;

#define TMC_GET_BIT(REG, SETTING) return (bool)((REG () & SETTING##_bm) >> SETTING##_bp);

#endif

#pragma once
#include <cstdint>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <utility>

#define TMC2130STEPPER_VERSION 0x020501 // v2.5.1

/**
 * A rudimentary type defining a pin in a gpio bank.
 */
struct Pin {
        const struct device *gpio{};
        uint8_t pin{};
};

/**
 *
 */
class TMC2130Stepper {
public:
        TMC2130Stepper (device const *spi, spi_config const *spiCfg);

        void checkStatus ();
        void rms_current (uint16_t mA, float multiplier = 0.5, float RS = 0.11);
        uint16_t rms_current ();
        void SilentStepStick2130 (uint16_t mA);
        void setCurrent (uint16_t mA, float Rsense, float multiplier);
        uint16_t getCurrent ();
        bool checkOT ();
        bool getOTPW ();
        void clear_otpw ();
        void push ();
        uint8_t test_connection ();
        // GCONF
        uint32_t GCONF ();
        void GCONF (uint32_t value);
        void I_scale_analog (bool B);
        void internal_Rsense (bool B);
        void en_pwm_mode (bool B);
        void enc_commutation (bool B);
        void shaft (bool B);
        void diag0_error (bool B);
        void diag0_otpw (bool B);
        void diag0_stall (bool B);
        void diag1_stall (bool B);
        void diag1_index (bool B);
        void diag1_onstate (bool B);
        void diag1_steps_skipped (bool B);
        void diag0_int_pushpull (bool B);
        void diag1_pushpull (bool B);
        void small_hysteresis (bool B);
        void stop_enable (bool B);
        void direct_mode (bool B);
        bool I_scale_analog ();
        bool internal_Rsense ();
        bool en_pwm_mode ();
        bool enc_commutation ();
        bool shaft ();
        bool diag0_error ();
        bool diag0_otpw ();
        bool diag0_stall ();
        bool diag1_stall ();
        bool diag1_index ();
        bool diag1_onstate ();
        bool diag1_steps_skipped ();
        bool diag0_int_pushpull ();
        bool diag1_pushpull ();
        bool small_hysteresis ();
        bool stop_enable ();
        bool direct_mode ();
        // IHOLD_IRUN
        void IHOLD_IRUN (uint32_t input);
        uint32_t IHOLD_IRUN ();
        void ihold (uint8_t B);
        void irun (uint8_t B);
        void iholddelay (uint8_t B);
        uint8_t ihold ();
        uint8_t irun ();
        uint8_t iholddelay ();
        // GSTAT
        void GSTAT (uint8_t input);
        uint8_t GSTAT ();
        bool reset ();
        bool drv_err ();
        bool uv_cp ();
        // IOIN
        uint32_t IOIN ();
        bool step ();
        bool dir ();
        bool dcen_cfg4 ();
        bool dcin_cfg5 ();
        bool drv_enn_cfg6 ();
        bool dco ();
        uint8_t version ();
        // TPOWERDOWN
        uint8_t TPOWERDOWN ();
        void TPOWERDOWN (uint8_t input);
        // TSTEP
        uint32_t TSTEP ();
        // TPWMTHRS
        uint32_t TPWMTHRS ();
        void TPWMTHRS (uint32_t input);
        // TCOOLTHRS
        uint32_t TCOOLTHRS ();
        void TCOOLTHRS (uint32_t input);
        // THIGH
        uint32_t THIGH ();
        void THIGH (uint32_t input);
        // XDRIRECT
        uint32_t XDIRECT ();
        void XDIRECT (uint32_t input);
        void coil_A (int16_t B);
        void coil_B (int16_t B);
        int16_t coil_A ();
        int16_t coil_B ();
        // VDCMIN
        uint32_t VDCMIN ();
        void VDCMIN (uint32_t input);
        // MSLUT0..MSLUT7
        uint32_t MSLUT0 ();
        void MSLUT0 (uint32_t input);
        uint32_t MSLUT1 ();
        void MSLUT1 (uint32_t input);
        uint32_t MSLUT2 ();
        void MSLUT2 (uint32_t input);
        uint32_t MSLUT3 ();
        void MSLUT3 (uint32_t input);
        uint32_t MSLUT4 ();
        void MSLUT4 (uint32_t input);
        uint32_t MSLUT5 ();
        void MSLUT5 (uint32_t input);
        uint32_t MSLUT6 ();
        void MSLUT6 (uint32_t input);
        uint32_t MSLUT7 ();
        void MSLUT7 (uint32_t input);
        // MSLUTSEL
        uint32_t MSLUTSEL ();
        void MSLUTSEL (uint32_t input);
        // MSLUTSTART
        uint32_t MSLUTSTART ();
        void MSLUTSTART (uint32_t input);
        // MSCNT
        uint16_t MSCNT ();
        // MSCURACT
        uint32_t MSCURACT ();
        int16_t cur_a ();
        int16_t cur_b ();
        // CHOPCONF
        uint32_t CHOPCONF ();
        void CHOPCONF (uint32_t value);
        void toff (uint8_t B);
        void hstrt (uint8_t B);
        void hend (uint8_t B);
        void fd (uint8_t B);
        void disfdcc (bool B);
        void rndtf (bool B);
        void chm (bool B);
        void tbl (uint8_t B);
        void vsense (bool B);
        void vhighfs (bool B);
        void vhighchm (bool B);
        void sync (uint8_t B);
        void mres (uint8_t B);
        void intpol (bool B);
        void dedge (bool B);
        void diss2g (bool B);
        uint8_t toff ();
        uint8_t hstrt ();
        uint8_t hend ();
        uint8_t fd ();
        bool disfdcc ();
        bool rndtf ();
        bool chm ();
        uint8_t tbl ();
        bool vsense ();
        bool vhighfs ();
        bool vhighchm ();
        uint8_t sync ();
        uint8_t mres ();
        bool intpol ();
        bool dedge ();
        bool diss2g ();
        // COOLCONF
        void COOLCONF (uint32_t value);
        uint32_t COOLCONF ();
        void semin (uint8_t B);
        void seup (uint8_t B);
        void semax (uint8_t B);
        void sedn (uint8_t B);
        void seimin (bool B);
        void sgt (int8_t B);
        void sfilt (bool B);
        uint8_t semin ();
        uint8_t seup ();
        uint8_t semax ();
        uint8_t sedn ();
        bool seimin ();
        int8_t sgt ();
        bool sfilt ();
        // PWMCONF
        void PWMCONF (uint32_t value);
        uint32_t PWMCONF ();
        void pwm_ampl (uint8_t B);
        void pwm_grad (uint8_t B);
        void pwm_freq (uint8_t B);
        void pwm_autoscale (bool B);
        void pwm_symmetric (bool B);
        void freewheel (uint8_t B);
        uint8_t pwm_ampl ();
        uint8_t pwm_grad ();
        uint8_t pwm_freq ();
        bool pwm_autoscale ();
        bool pwm_symmetric ();
        uint8_t freewheel ();
        // DRVSTATUS
        uint32_t DRV_STATUS ();
        uint16_t sg_result ();
        bool fsactive ();
        uint8_t cs_actual ();
        bool stallguard ();
        bool ot ();
        bool otpw ();
        bool s2ga ();
        bool s2gb ();
        bool ola ();
        bool olb ();
        bool stst ();
        // PWM_SCALE
        uint8_t PWM_SCALE ();
        // ENCM_CTRL
        uint8_t ENCM_CTRL ();
        void ENCM_CTRL (uint8_t input);
        void inv (bool B);
        void maxspeed (bool B);
        bool inv ();
        bool maxspeed ();
        // LOST_STEPS
        uint32_t LOST_STEPS ();

        // Helper functions
        void microsteps (uint16_t ms);
        uint16_t microsteps ();
        void blank_time (uint8_t value);
        uint8_t blank_time ();
        void hysteresis_end (int8_t value);
        int8_t hysteresis_end ();
        void hysteresis_start (uint8_t value);
        uint8_t hysteresis_start ();
        void sg_current_decrease (uint8_t value);
        uint8_t sg_current_decrease ();

        // Aliases

        // RW: GCONF
        bool external_ref () { return I_scale_analog (); }
        bool internal_sense_R () { return internal_Rsense (); }
        bool stealthChop () { return en_pwm_mode (); }
        bool commutation () { return enc_commutation (); }
        bool shaft_dir () { return shaft (); }
        bool diag0_errors () { return diag0_error (); }
        bool diag0_temp_prewarn () { return diag0_otpw (); }
        bool diag1_chopper_on () { return diag1_onstate (); }
        bool diag0_active_high () { return diag0_int_pushpull (); }
        bool diag1_active_high () { return diag1_pushpull (); }
        void external_ref (bool value) { I_scale_analog (value); }
        void internal_sense_R (bool value) { internal_Rsense (value); }
        void stealthChop (bool value) { en_pwm_mode (value); }
        void commutation (bool value) { enc_commutation (value); }
        void shaft_dir (bool value) { shaft (value); }
        void diag0_errors (bool value) { diag0_error (value); }
        void diag0_temp_prewarn (bool value) { diag0_otpw (value); }
        void diag1_chopper_on (bool value) { diag1_onstate (value); }
        void diag0_active_high (bool value) { diag0_int_pushpull (value); }
        void diag1_active_high (bool value) { diag1_pushpull (value); }
        // RC
        uint8_t status_flags () { return GSTAT (); }
        // R
        uint32_t input () { return IOIN (); }
        // W: IHOLD_IRUN
        uint8_t hold_current () { return ihold (); }
        uint8_t run_current () { return irun (); }
        uint8_t hold_delay () { return iholddelay (); }
        void hold_current (uint8_t value) { ihold (value); }
        void run_current (uint8_t value) { irun (value); }
        void hold_delay (uint8_t value) { iholddelay (value); }
        // W
        uint8_t power_down_delay () { return TPOWERDOWN (); }
        void power_down_delay (uint8_t value) { TPOWERDOWN (value); }
        // R
        uint32_t microstep_time () { return TSTEP (); }
        // W
        uint32_t stealth_max_speed () { return TPWMTHRS (); }
        void stealth_max_speed (uint32_t value) { TPWMTHRS (value); }
        // W
        uint32_t coolstep_min_speed () { return TCOOLTHRS (); }
        void coolstep_min_speed (uint32_t value) { TCOOLTHRS (value); }
        // W
        uint32_t mode_sw_speed () { return THIGH (); }
        void mode_sw_speed (uint32_t value) { THIGH (value); }
        // RW: XDIRECT
        int16_t coil_A_current () { return coil_A (); }
        void coil_A_current (int16_t value) { coil_A (value); }
        int16_t coil_B_current () { return coil_B (); }
        void coil_B_current (int16_t value) { coil_B (value); }
        // W
        uint32_t DCstep_min_speed () { return VDCMIN (); }
        void DCstep_min_speed (uint32_t value) { VDCMIN (value); }
        // W
        uint32_t lut_mslutstart () { return MSLUTSTART (); }
        void lut_mslutstart (uint32_t value) { MSLUTSTART (value); }
        uint32_t lut_msutsel () { return MSLUTSEL (); }
        void lut_msutsel (uint32_t value) { MSLUTSEL (value); }
        uint32_t ms_lookup_0 () { return MSLUT0 (); }
        void ms_lookup_0 (uint32_t value) { MSLUT0 (value); }
        uint32_t ms_lookup_1 () { return MSLUT1 (); }
        void ms_lookup_1 (uint32_t value) { MSLUT1 (value); }
        uint32_t ms_lookup_2 () { return MSLUT2 (); }
        void ms_lookup_2 (uint32_t value) { MSLUT2 (value); }
        uint32_t ms_lookup_3 () { return MSLUT3 (); }
        void ms_lookup_3 (uint32_t value) { MSLUT3 (value); }
        uint32_t ms_lookup_4 () { return MSLUT4 (); }
        void ms_lookup_4 (uint32_t value) { MSLUT4 (value); }
        uint32_t ms_lookup_5 () { return MSLUT5 (); }
        void ms_lookup_5 (uint32_t value) { MSLUT5 (value); }
        uint32_t ms_lookup_6 () { return MSLUT6 (); }
        void ms_lookup_6 (uint32_t value) { MSLUT6 (value); }
        uint32_t ms_lookup_7 () { return MSLUT7 (); }
        void ms_lookup_7 (uint32_t value) { MSLUT7 (value); }
        // RW: CHOPCONF
        uint8_t off_time () { return toff (); }
        //		 uint8_t 	hysteresis_start() __attribute__((always_)) { return hstrt();
        //} 		 int8_t
        // hysteresis_low()
        //__attribute__((always_)) { return hend(); 										}
        int8_t hysteresis_low () { return hysteresis_end (); }
        uint8_t fast_decay_time () { return fd (); }
        bool disable_I_comparator () { return disfdcc (); }
        bool random_off_time () { return rndtf (); }
        bool chopper_mode () { return chm (); }
        //		 uint8_t 	blank_time()
        //__attribute__((always_)) { return tbl(); 											}
        bool high_sense_R () { return vsense (); }
        bool fullstep_threshold () { return vhighfs (); }
        bool high_speed_mode () { return vhighchm (); }
        uint8_t sync_phases () { return sync (); }
        //		 uint16_t microsteps()
        //__attribute__((always_)) { return mres(); 										}
        bool interpolate () { return intpol (); }
        bool double_edge_step () { return dedge (); }
        bool disable_short_protection () { return diss2g (); }
        void off_time (uint8_t value) { toff (value); }
        //		 void 		hysteresis_start(	 uint8_t value)	__attribute__((always_)) {
        // hstrt(value); 							} 		 void 		hysteresis_low(
        // int8_t value)
        //__attribute__((always_)) {				hend(value); 								}
        void hysteresis_low (int8_t value) { hysteresis_end (value); }
        void fast_decay_time (uint8_t value) { fd (value); }
        void disable_I_comparator (bool value) { disfdcc (value); }
        void random_off_time (bool value) { rndtf (value); }
        void chopper_mode (bool value) { chm (value); }
        //		 void 		blank_time(				 uint8_t value)	__attribute__((always_)) {
        // tbl(value); 								}
        void high_sense_R (bool value) { vsense (value); }
        void fullstep_threshold (bool value) { vhighfs (value); }
        void high_speed_mode (bool value) { vhighchm (value); }
        void sync_phases (uint8_t value) { sync (value); }
        //		 void			microsteps(				uint16_t value)	__attribute__((always_)) {
        // mres(value); 								}
        void interpolate (bool value) { intpol (value); }
        void double_edge_step (bool value) { dedge (value); }
        void disable_short_protection (bool value) { diss2g (value); }
        // W: COOLCONF
        uint8_t sg_min () { return semin (); }
        uint8_t sg_step_width () { return seup (); }
        uint8_t sg_max () { return semax (); }
        //		 uint8_t 	sg_current_decrease()							__attribute__((always_)) {
        // return sedn(); 										}
        uint8_t smart_min_current () { return seimin (); }
        int8_t sg_stall_value () { return sgt (); }
        bool sg_filter () { return sfilt (); }
        void sg_min (uint8_t value) { semin (value); }
        void sg_step_width (uint8_t value) { seup (value); }
        void sg_max (uint8_t value) { semax (value); }
        //		 void 		sg_current_decrease(uint8_t value)__attribute__((always_)) {
        // sedn(value); 								}
        void smart_min_current (uint8_t value) { seimin (value); }
        void sg_stall_value (int8_t value) { sgt (value); }
        void sg_filter (bool value) { sfilt (value); }
        // W: PWMCONF
        uint8_t stealth_amplitude () { return pwm_ampl (); }
        uint8_t stealth_gradient () { return pwm_grad (); }
        uint8_t stealth_freq () { return pwm_freq (); }
        bool stealth_autoscale () { return pwm_autoscale (); }
        bool stealth_symmetric () { return pwm_symmetric (); }
        uint8_t standstill_mode () { return freewheel (); }
        void stealth_amplitude (uint8_t value) { pwm_ampl (value); }
        void stealth_gradient (uint8_t value) { pwm_grad (value); }
        void stealth_freq (uint8_t value) { pwm_freq (value); }
        void stealth_autoscale (bool value) { pwm_autoscale (value); }
        void stealth_symmetric (bool value) { pwm_symmetric (value); }
        void standstill_mode (uint8_t value) { freewheel (value); }
        // W: ENCM_CTRL
        bool invert_encoder () { return inv (); }
        void invert_encoder (bool value) { inv (value); }
        // R: DRV_STATUS
        uint32_t DRVSTATUS () { return DRV_STATUS (); }

        // Backwards compatibility
        void hysterisis_end (int8_t value) { hysteresis_end (value); }
        int8_t hysterisis_end () { return hysteresis_end (); }
        void hysterisis_start (uint8_t value) { hysteresis_start (value); }
        uint8_t hysterisis_start () { return hysteresis_start (); }

        uint8_t getLastStatusResponse () const { return lastStatusResponse; }

        float Rsense = 0.11;
        bool flag_otpw = false;

private:
        device const *spi;
        spi_config const *spiCfg;
        uint8_t lastStatusResponse;

        // Shadow registers
        uint32_t GCONF_sr = 0x00000000UL, IHOLD_IRUN_sr = 0x00000000UL, TSTEP_sr = 0x00000000UL, TPWMTHRS_sr = 0x00000000UL,
                 TCOOLTHRS_sr = 0x00000000UL, THIGH_sr = 0x00000000UL, XDIRECT_sr = 0x00000000UL, VDCMIN_sr = 0x00000000UL,
                 MSLUT0_sr = 0xAAAAB554UL, MSLUT1_sr = 0x4A9554AAUL, MSLUT2_sr = 0x24492929UL, MSLUT3_sr = 0x10104222UL,
                 MSLUT4_sr = 0xFBFFFFFFUL, MSLUT5_sr = 0xB5BB777DUL, MSLUT6_sr = 0x49295556UL, MSLUT7_sr = 0x00404222UL,
                 MSLUTSEL_sr = 0xFFFF8056UL, CHOPCONF_sr = 0x00000000UL, COOLCONF_sr = 0x00000000UL, DCCTRL_sr = 0x00000000UL,
                 PWMCONF_sr = 0x00050480UL, tmp_sr = 0x00000000UL, TPOWERDOWN_sr = 0x00000000UL, ENCM_CTRL_sr = 0x00000000UL,
                 GSTAT_sr = 0x00000000UL, MSLUTSTART_sr = 0x00F70000UL;

        /**
         * Writes to a register. Stores the TMC status in the lastStatusResponse field,
         * which can be accessed using getLastStatusResponse method. Returns 0 in case
         * of a success, and errno otherwise.
         */
        int writeCmd (uint8_t cmd, uint32_t value);

        /**
         * First return value is the register value itself, while the second is the status
         * code from SPI operation. 0 means a success, anything other means an error.
         */
        std::pair<uint32_t, int> readCmd (uint8_t cmd);

        uint16_t val_mA = 0;
};

#![allow(dead_code)]

use crate::drivers::tof::vl53l1x_constants::*;
use core::marker::PhantomData;
use cortex_m::asm::delay;
use stm32f4xx_hal::i2c::{Error, I2c, Instance, NoAcknowledgeSource, Pins};

pub enum DistanceMode {
    Short = 1,
    Long = 2,
}

pub enum TimingBudget {
    TbError = 0,
    Tb15ms = 15,
    Tb20ms = 20,
    Tb33ms = 33,
    Tb50ms = 50,
    Tb100ms = 100,
    Tb200ms = 200,
    Tb500ms = 500,
}

pub struct VL53L1<I2C, SCL, SDA> {
    device_address: u8,
    _i2c: PhantomData<I2C>,
    _scl: PhantomData<SCL>,
    _sda: PhantomData<SDA>,
}

impl<I2C, SCL, SDA> VL53L1<I2C, SCL, SDA>
where
    I2C: Instance,
    (SCL, SDA): Pins<I2C>,
{
    pub fn new(
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        i2c_addr: u8,
    ) -> Result<VL53L1<I2C, SCL, SDA>, Error> {
        let mut dev = VL53L1::default();
        match dev.get_sensor_id(i2c) {
            Ok(_) => {
                // sensor cold boot, init and set new address
                dev.sensor_init(i2c)?;
                dev.set_i2c_address(i2c, i2c_addr)?;
            }
            Err(e) => {
                match e {
                    Error::NoAcknowledge(NoAcknowledgeSource::Address) => {
                        // MCU has been reset without powering down sensor
                        // no need to reinit, just return handle with correct address
                        dev.device_address = i2c_addr;
                        if let Err(_) = dev.get_sensor_id(i2c) {
                            // return error if sensor doesn't respond to new address either
                            return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }
                    _ => {
                        return Err(e);
                    }
                };
            }
        }
        Ok(dev)
    }

    fn set_i2c_address(
        &mut self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        new_address: u8,
    ) -> Result<(), Error> {
        self.write_byte(i2c, I2C_SLAVE__DEVICE_ADDRESS, new_address & 0x7f)?;
        self.device_address = new_address;
        Ok(())
    }

    // reloads the set i2c address on software resets using hardcoded default address
    fn reload_i2c_address(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        let buffer: [u8; 3] = [
            I2C_SLAVE__DEVICE_ADDRESS[0],
            I2C_SLAVE__DEVICE_ADDRESS[1],
            self.device_address & 0x7F,
        ];
        i2c.write(DEFAULT_I2C_ADDR, &buffer)?;
        Ok(())
    }

    fn sensor_init(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        // ensure fresh software state
        self.software_reset(i2c)?;
        while self.get_boot_state(i2c).unwrap_or(0) & 0x01 != 0x01 {
            delay(100000)
        }

        // sanity check
        if self.get_sensor_id(i2c).unwrap_or(1) != SENSOR_ID {
            return Err(Error::NoAcknowledge(NoAcknowledgeSource::Unknown));
        }

        // set config
        let hv_extsup_config = self.read_word(i2c, PAD_I2C_HV__EXTSUP_CONFIG)?;
        self.write_word(i2c, PAD_I2C_HV__EXTSUP_CONFIG, hv_extsup_config | 0x01)?;
        self.write_word(i2c, DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE)?;
        self.write_byte(i2c, GPIO__TIO_HV_STATUS, 0x02)?;
        self.write_byte(i2c, SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8)?; // tuning parm default
        self.write_byte(i2c, SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16)?; // tuning parm default
        self.write_byte(i2c, ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01)?;
        self.write_byte(i2c, ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF)?;
        self.write_byte(i2c, ALGO__RANGE_MIN_CLIP, 0)?; // tuning parm default
        self.write_byte(i2c, ALGO__CONSISTENCY_CHECK__TOLERANCE, 2)?; // tuning parm default

        // general config
        self.write_word(i2c, SYSTEM__THRESH_RATE_HIGH, 0x0000)?;
        self.write_word(i2c, SYSTEM__THRESH_RATE_LOW, 0x0000)?;
        self.write_byte(i2c, DSS_CONFIG__APERTURE_ATTENUATION, 0x38)?;

        // timing config
        // most of these settings will be determined later by distance and timing
        // budget configuration
        self.write_word(i2c, RANGE_CONFIG__SIGMA_THRESH, 360)?; // tuning parm default
        self.write_word(i2c, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192)?; // tuning parm default

        // dynamic config
        self.write_byte(i2c, SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01)?;
        self.write_byte(i2c, SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01)?;
        self.write_byte(i2c, SD_CONFIG__QUANTIFIER, 2)?; // tuning parm default

        self.write_byte(i2c, SYSTEM__GROUPED_PARAMETER_HOLD, 0x00)?;
        self.write_byte(i2c, SYSTEM__SEED_CONFIG, 1)?; // tuning parm default

        self.write_byte(i2c, SYSTEM__SEQUENCE_CONFIG, 0x8B)?; // VHV, PHASECAL, DSS1, RANGE
        self.write_word(i2c, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8)?;
        self.write_byte(i2c, DSS_CONFIG__ROI_MODE_CONTROL, 2)?; // REQUESTED_EFFFECTIVE_SPADS
        let outer_offset_mm: u16 = self.read_word(i2c, MM_CONFIG__OUTER_OFFSET_MM)?;
        self.write_word(i2c, ALGO__PART_TO_PART_RANGE_OFFSET_MM, outer_offset_mm * 4)?;

        self.start_ranging(i2c, Some(DEFAULT_DM), Some(DEFAULT_TB), Some(DEFAULT_IM_MS))?;
        while self.check_for_data_ready(i2c)? == false {
            delay(100000);
        }
        self.clear_interrupt(i2c)?;
        self.stop_ranging(i2c)?;

        // update temperature compensation
        self.write_byte(i2c, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09)?; /* two bounds VHV */
        self.write_byte(i2c, [0x00, 0x0b], 0x00)?; /* start VHV from the previous temperature */
        Ok(())
    }

    pub fn software_reset(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        self.write_byte(i2c, SOFT_RESET, 0x00)?;
        delay(1000000);
        self.write_to_address(i2c, &SOFT_RESET, &[0x01], DEFAULT_I2C_ADDR)?;
        // self.write_byte(i2c, SOFT_RESET, 0x01)?;
        delay(1000000);
        self.reload_i2c_address(i2c)?;
        Ok(())
    }

    pub fn clear_interrupt(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        self.write_byte(i2c, SYSTEM__INTERRUPT_CLEAR, 0x01)?;
        Ok(())
    }

    pub fn start_ranging(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        distance_mode: Option<DistanceMode>,
        timing_budget: Option<TimingBudget>,
        inter_measure_ms: Option<u32>,
    ) -> Result<(), Error> {
        if let Some(dm) = distance_mode {
            self.set_distance_mode(i2c, dm)?;
        }
        if let Some(tb) = timing_budget {
            self.set_timing_budget_ms(i2c, tb)?;
        }
        if let Some(val) = inter_measure_ms {
            self.set_inter_measurement_ms(i2c, val)?;
        }
        self.clear_interrupt(i2c)?;
        self.write_byte(i2c, SYSTEM__MODE_START, 0x40)?; // Enable Measurements
        Ok(())
    }

    pub fn stop_ranging(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        self.write_byte(i2c, SYSTEM__MODE_START, 0x00)?; // Disaable Measurements
        Ok(())
    }

    pub fn check_for_data_ready(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<bool, Error> {
        if (self.read_byte(i2c, GPIO__TIO_HV_STATUS)? & 0x01) == 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn set_timing_budget_ms(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        timing_budget: TimingBudget,
    ) -> Result<(), Error> {
        let dm: DistanceMode = self.get_distance_mode(i2c)?;
        match dm {
            DistanceMode::Short => {
                Ok(match timing_budget {
                    // 15 ms only available in short distance mode
                    TimingBudget::Tb15ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027)?;
                    }
                    TimingBudget::Tb20ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E)?;
                    }
                    TimingBudget::Tb33ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E)?;
                    }
                    TimingBudget::Tb50ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8)?;
                    }
                    TimingBudget::Tb100ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388)?;
                    }
                    TimingBudget::Tb200ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496)?;
                    }
                    TimingBudget::Tb500ms => {
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591)?;
                        self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1)?;
                    }
                    TimingBudget::TbError => {}
                })
            }
            DistanceMode::Long => Ok(match timing_budget {
                // 15 ms only available in short distance mode
                TimingBudget::Tb15ms => {}
                TimingBudget::Tb20ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022)?;
                }
                TimingBudget::Tb33ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E)?;
                }
                TimingBudget::Tb50ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6)?;
                }
                TimingBudget::Tb100ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA)?;
                }
                TimingBudget::Tb200ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8)?;
                }
                TimingBudget::Tb500ms => {
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F)?;
                    self.write_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4)?;
                }
                TimingBudget::TbError => {}
            }),
        }
    }

    pub fn get_timing_budget_ms(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
    ) -> Result<TimingBudget, Error> {
        Ok(
            match self.read_word(i2c, RANGE_CONFIG__TIMEOUT_MACROP_A_HI)? {
                0x001D => TimingBudget::Tb15ms,
                0x0051 => TimingBudget::Tb20ms,
                0x001E => TimingBudget::Tb20ms,
                0x00D6 => TimingBudget::Tb33ms,
                0x0060 => TimingBudget::Tb33ms,
                0x01AE => TimingBudget::Tb50ms,
                0x00AD => TimingBudget::Tb50ms,
                0x02E1 => TimingBudget::Tb100ms,
                0x01CC => TimingBudget::Tb100ms,
                0x03E1 => TimingBudget::Tb200ms,
                0x02D9 => TimingBudget::Tb200ms,
                0x0591 => TimingBudget::Tb500ms,
                0x048F => TimingBudget::Tb500ms,
                _ => TimingBudget::TbError,
            },
        )
    }

    pub fn set_distance_mode(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        dm: DistanceMode,
    ) -> Result<(), Error> {
        let tb: TimingBudget = self.get_timing_budget_ms(i2c)?;
        match dm {
            DistanceMode::Short => {
                self.write_byte(i2c, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14)?;
                self.write_byte(i2c, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07)?;
                self.write_byte(i2c, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05)?;
                self.write_byte(i2c, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38)?;
                self.write_word(i2c, SD_CONFIG__WOI_SD0, 0x0705)?;
                self.write_word(i2c, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606)?;
            }
            DistanceMode::Long => {
                self.write_byte(i2c, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A)?;
                self.write_byte(i2c, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F)?;
                self.write_byte(i2c, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D)?;
                self.write_byte(i2c, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8)?;
                self.write_word(i2c, SD_CONFIG__WOI_SD0, 0x0F0D)?;
                self.write_word(i2c, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E)?;
            }
        }
        self.set_timing_budget_ms(i2c, tb)?;
        Ok(())
    }

    pub fn get_distance_mode(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<DistanceMode, Error> {
        match self.read_byte(i2c, PHASECAL_CONFIG__TIMEOUT_MACROP)? {
            0x14 => Ok(DistanceMode::Short),
            0x0A => Ok(DistanceMode::Long),
            _ => Err(Error::NoAcknowledge(NoAcknowledgeSource::Data)),
        }
    }

    fn set_inter_measurement_ms(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        inter_meas_ms: u32,
    ) -> Result<(), Error> {
        let clock_pll: u16 = self.read_word(i2c, RESULT__OSC_CALIBRATE_VAL)? & 0x3FF;
        self.write_double_word(
            i2c,
            SYSTEM__INTERMEASUREMENT_PERIOD,
            ((clock_pll as u32 * inter_meas_ms) as f32 * 1.075) as u32,
        )?;
        Ok(())
    }

    pub fn get_inter_measurement_ms(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<u16, Error> {
        let clock_pll: u16 = self.read_word(i2c, RESULT__OSC_CALIBRATE_VAL)? & 0x3FF;
        let temp: u32 = self.read_double_word(i2c, SYSTEM__INTERMEASUREMENT_PERIOD)?;
        Ok(((temp as f32) / (clock_pll as f32 * 1.065)) as u16)
    }

    pub fn get_boot_state(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<u8, Error> {
        Ok(self.read_byte(i2c, FIRMWARE__SYSTEM_STATUS)?)
    }

    pub fn get_distance(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<u16, Error> {
        Ok(self.read_word(i2c, RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)?)
    }

    pub fn get_sensor_id(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<u16, Error> {
        Ok(self.read_word(i2c, IDENTIFICATION__MODEL_ID)?)
    }

    pub fn set_offset(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        offset_value: i16,
    ) -> Result<(), Error> {
        self.write_word(
            i2c,
            ALGO__PART_TO_PART_RANGE_OFFSET_MM,
            (offset_value * 4) as u16,
        )?;
        self.write_word(i2c, MM_CONFIG__INNER_OFFSET_MM, 0x0)?;
        self.write_word(i2c, MM_CONFIG__OUTER_OFFSET_MM, 0x0)?;
        Ok(())
    }

    pub fn get_offset(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<i16, Error> {
        let temp: u16 = (self.read_word(i2c, ALGO__PART_TO_PART_RANGE_OFFSET_MM)? << 3) >> 5;
        if temp as i16 > 1024 {
            Ok((temp as i16) - 2048)
        } else {
            Ok(temp as i16)
        }
    }

    pub fn start_temperature_update(&self, i2c: &mut I2c<I2C, (SCL, SDA)>) -> Result<(), Error> {
        self.write_byte(i2c, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81)?; /* full VHV */
        self.write_byte(i2c, [0x00, 0x0B], 0x92)?;
        self.start_ranging(i2c, Some(DEFAULT_DM), Some(DEFAULT_TB), Some(DEFAULT_IM_MS))?;
        while self.check_for_data_ready(i2c)? == false {}
        self.clear_interrupt(i2c)?;
        self.stop_ranging(i2c)?;
        self.write_byte(i2c, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09)?; /* two bounds VHV */
        self.write_byte(i2c, [0x00, 0x0B], 0)?; /* start VHV from the previous temperature */
        Ok(())
    }

    fn read(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: &[u8],
        data: &mut [u8],
    ) -> Result<(), Error> {
        i2c.write(self.device_address & 0x7F, index)?;
        i2c.read(self.device_address & 0x7F, data)?;
        Ok(())
    }

    fn write(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: &[u8],
        payload: &[u8],
    ) -> Result<(), Error> {
        let mut buffer: [u8; 6] = [0; 6];
        for (idx, val) in index.iter().chain(payload.iter()).enumerate() {
            buffer[idx] = *val;
        }
        i2c.write(
            self.device_address & 0x7F,
            &buffer[0..index.len() + payload.len()],
        )?;
        Ok(())
    }

    fn write_to_address(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: &[u8],
        payload: &[u8],
        i2c_addr: u8,
    ) -> Result<(), Error> {
        let mut buffer: [u8; 6] = [0; 6];
        for (idx, val) in index.iter().chain(payload.iter()).enumerate() {
            buffer[idx] = *val;
        }
        i2c.write(i2c_addr & 0x7F, &buffer[0..index.len() + payload.len()])?;
        Ok(())
    }

    fn read_byte(&self, i2c: &mut I2c<I2C, (SCL, SDA)>, index: [u8; 2]) -> Result<u8, Error> {
        let mut temp: [u8; 1] = [0x00];
        self.read(i2c, &index, &mut temp)?;
        Ok(temp[0])
    }

    fn write_byte(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: [u8; 2],
        payload: u8,
    ) -> Result<(), Error> {
        self.write(i2c, &index, &[payload])?;
        Ok(())
    }

    fn read_word(&self, i2c: &mut I2c<I2C, (SCL, SDA)>, index: [u8; 2]) -> Result<u16, Error> {
        let mut temp: [u8; 2] = [0x00, 0x00];
        self.read(i2c, &index, &mut temp)?;
        Ok(((temp[0] as u16) << 8) | (temp[1] as u16))
    }

    fn write_word(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: [u8; 2],
        payload: u16,
    ) -> Result<(), Error> {
        let temp: [u8; 2] = [(payload >> 8) as u8, (payload & 0xFF) as u8];
        self.write(i2c, &index, &temp)?;
        Ok(())
    }

    fn read_double_word(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: [u8; 2],
    ) -> Result<u32, Error> {
        let mut temp: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
        self.read(i2c, &index, &mut temp)?;
        Ok(((temp[0] as u32) << 24)
            | ((temp[1] as u32) << 16)
            | ((temp[2] as u32) << 8)
            | (temp[3] as u32))
    }

    fn write_double_word(
        &self,
        i2c: &mut I2c<I2C, (SCL, SDA)>,
        index: [u8; 2],
        payload: u32,
    ) -> Result<(), Error> {
        let temp: [u8; 4] = [
            (payload >> 24) as u8,
            ((payload & 0x00FF0000) >> 16) as u8,
            ((payload & 0x0000FF00) >> 8) as u8,
            (payload & 0x000000FF) as u8,
        ];
        self.write(i2c, &index, &temp)?;
        Ok(())
    }
}

impl<I2C, SCL, SDA> Default for VL53L1<I2C, SCL, SDA>
where
    I2C: Instance,
    (SCL, SDA): Pins<I2C>,
{
    fn default() -> Self {
        VL53L1 {
            device_address: DEFAULT_I2C_ADDR,
            _i2c: PhantomData,
            _scl: PhantomData,
            _sda: PhantomData,
        }
    }
}

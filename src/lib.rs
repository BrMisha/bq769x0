#![cfg_attr(not(std), no_std)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]

pub mod util;
pub mod registers;

use crc_any::CRCu8;

pub const BQ76920: usize = 5;
pub const BQ76930: usize = 10;
pub const BQ76940: usize = 15;

#[derive(Debug, Copy, Clone)]
pub enum Error {
    // #[cfg(crc)]
    CRCMismatch,
    I2CError,
    BufTooLarge,
    Uninitialized,
    VerifyError(u8),
    OCDSCDRangeMismatch,
    UVThresholdUnobtainable(util::MilliVolts, util::MilliVolts),
    OVThresholdUnobtainable(util::MilliVolts, util::MilliVolts),
}

pub struct BQ769x0<const X: usize> {
    dev_address: u8, // 7bit address
    // crc: CRCu8, // x8 + x2 + x + 1
    init_complete: bool,
    adc_gain: u16,  // uV / LSB
    adc_offset: i8, // mV
    shunt: util::MicroOhms,
    cell_count: u8,
    cells: [util::MilliVolts; X],
    use_crc: bool,
}

impl<const X: usize> BQ769x0<X>
where
    [(); X * 2]: Sized,
    [(); X * 4]: Sized,
{
    pub const fn new(dev_address: u8, cell_count: u8, use_crc: bool) -> Option<Self> {
        match X {
            BQ76920 | BQ76930 | BQ76940 => {
                match X {
                    BQ76920 => {
                        if cell_count < 3 || cell_count > 5 {
                            return None;
                        }
                    }
                    BQ76930 => {
                        if cell_count < 6 || cell_count > 10 {
                            return None;
                        }
                    }
                    BQ76940 => {
                        if cell_count < 9 || cell_count > 15 {
                            return None;
                        }
                    }
                    _ => unreachable!(),
                }
                Some(BQ769x0 {
                    dev_address,
                    init_complete: false,
                    adc_gain: 0,
                    adc_offset: 0,
                    shunt: util::MicroOhms(0),
                    cell_count,
                    cells: [util::MilliVolts(0); X],
                    use_crc,
                })
            }
            _ => None,
        }
    }

    fn check_communication<I2C>(i2c: &mut I2C, dev_address: u8, use_crc: bool) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        const TEST_REG: u8 = 0x0a;
        let mut buf = [0u8; 1];
        if use_crc {
            Self::write_raw_crc(i2c, dev_address, TEST_REG, &[0xaa])?;
            Self::read_raw_crc(i2c, dev_address, TEST_REG, &mut buf)?;
        } else {
            Self::write_raw_nocrc(i2c, dev_address, TEST_REG, &[0xaa])?;
            Self::read_raw_nocrc(i2c, dev_address, TEST_REG, &mut buf)?;
        }
        if buf[0] == 0xaa {
            Ok(())
        } else {
            Err(Error::I2CError)
        }
    }

    pub fn new_detect<I2C>(i2c: &mut I2C, cell_count: u8) -> Option<Self>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        if Self::check_communication(i2c, 0x18, false).is_ok() {
            Self::new(0x18, cell_count, false)
        } else if Self::check_communication(i2c, 0x18, true).is_ok() {
            return Self::new(0x18, cell_count, true);
        } else if Self::check_communication(i2c, 0x08, false).is_ok() {
            return Self::new(0x08, cell_count, false);
        } else if Self::check_communication(i2c, 0x08, true).is_ok() {
            return Self::new(0x08, cell_count, true);
        } else {
            None
        }
    }

    pub fn i2c_address(&self) -> u8 {
        self.dev_address
    }

    pub fn is_crc_used(&self) -> bool {
        self.use_crc
    }

    pub fn adc_gain(&self) -> u16 {
        self.adc_gain
    }

    pub fn adc_offset(&self) -> i8 {
        self.adc_offset
    }

    fn read_raw_nocrc<I2C>(
        i2c: &mut I2C,
        dev_address: u8,
        reg_address: u8,
        data: &mut [u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        #[cfg(no_std)]
        {
            cortex_m::asm::delay(10000);
        }

        match i2c.write_read(dev_address, &[reg_address], data) {
            Ok(_) => Ok(()),
            Err(_) => Err(Error::I2CError),
        }
    }

    fn read_raw_crc<I2C>(
        i2c: &mut I2C,
        dev_address: u8,
        reg_address: u8,
        data: &mut [u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        if data.len() > X * 2 {
            // max 5/10/15 cell voltages * 2 bytes
            return Err(Error::BufTooLarge);
        } else if data.is_empty() {
            return Ok(());
        }
        let mut buf = [0u8; X * 4]; // byte,crc,byte,crc,...
        let r = i2c.write_read(dev_address, &[reg_address], &mut buf[0..data.len() * 2]);
        let mut crc = CRCu8::crc8();
        crc.reset();
        crc.digest(&[(dev_address << 1) | 0b0000_0001, buf[0]]);
        if crc.get_crc() != buf[1] {
            return Err(Error::CRCMismatch);
        }
        if data.len() > 1 {
            for i in (3..data.len() * 2).step_by(2) {
                crc.reset();
                crc.digest(&[buf[i - 1]]);
                if crc.get_crc() != buf[i] {
                    return Err(Error::CRCMismatch);
                }
            }
        }
        return if r.is_ok() {
            for (i, b) in data.iter_mut().enumerate() {
                *b = buf[i * 2];
            }
            Ok(())
        } else {
            Err(Error::I2CError)
        };
    }

    pub fn read_raw<I2C>(
        &mut self,
        i2c: &mut I2C,
        reg_address: u8,
        data: &mut [u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        if self.use_crc {
            Self::read_raw_crc(i2c, self.dev_address, reg_address, data)
        } else {
            Self::read_raw_nocrc(i2c, self.dev_address, reg_address, data)
        }
    }

    fn write_raw_nocrc<I2C>(
        i2c: &mut I2C,
        dev_address: u8,
        reg_address: u8,
        data: &[u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        #[cfg(no_std)]
        {
            cortex_m::asm::delay(10000);
        }

        if data.len() > 8 {
            return Err(Error::BufTooLarge);
        } else if data.is_empty() {
            return Ok(());
        }
        let mut buf = [0u8; 8 + 1]; // reg,byte,byte,...
        buf[0] = reg_address;
        for (i, b) in data.iter().enumerate() {
            buf[i + 1] = *b;
        }

        i2c.write(dev_address, &buf[0..data.len() + 1])
            .map_err(|_| Error::I2CError)?;
        // i2c.write_read(self.dev_address, &[reg_address], &mut buf[0..data.len()]).map_err(|_| Error::I2CError)?;
        // for (i, x) in data.iter().zip(buf).enumerate() {
        //     if *x.0 != x.1 {
        //         return Err(Error::VerifyError(reg_address + i as u8));
        //     }
        // }
        Ok(())
    }

    fn write_raw_crc<I2C>(
        i2c: &mut I2C,
        dev_address: u8,
        reg_address: u8,
        data: &[u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        //#[cfg(no_std)] {
        //cortex_m::asm::delay(10000000);
        //}

        if data.len() > 8 {
            return Err(Error::BufTooLarge);
        } else if data.is_empty() {
            return Ok(());
        }
        let mut buf = [0u8; 8 * 2 + 1]; // reg,byte,crc,byte,crc,...
        buf[0] = reg_address;
        for (i, b) in data.iter().enumerate() {
            buf[i * 2 + 1] = *b;
        }
        let mut crc = CRCu8::crc8();
        crc.reset();
        crc.digest(&[(dev_address << 1), reg_address, data[0]]);
        buf[2] = crc.get_crc();
        for i in (4..data.len() * 2 + 1).step_by(2) {
            crc.reset();
            crc.digest(&[buf[i - 1]]);
            buf[i] = crc.get_crc();
        }
        i2c.write(dev_address, &buf[0..data.len() * 2 + 1])
            .map_err(|_| Error::I2CError)?;

        Ok(())
    }

    pub fn write_raw<I2C>(
        &mut self,
        i2c: &mut I2C,
        reg_address: u8,
        data: &[u8],
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        if self.use_crc {
            Self::write_raw_crc(i2c, self.dev_address, reg_address, data)
        } else {
            Self::write_raw_nocrc(i2c, self.dev_address, reg_address, data)
        }
    }

    fn read_adc_characteristics<I2C>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut gain1_offset = [0u8; 2];
        let mut gain2 = [0u8; 1];
        self.read_raw(i2c, registers::ADCGAIN1, &mut gain1_offset)?;
        self.read_raw(i2c, registers::ADCGAIN2, &mut gain2)?;
        self.adc_gain = 365 + (((gain1_offset[0] << 1) & 0b0001_1000) | (gain2[0] >> 5)) as u16;
        self.adc_offset = gain1_offset[1] as i8;

        Ok(())
    }

    pub fn is_initialized(&self) -> bool {
        self.init_complete
    }

    pub fn cell_voltages<I2C>(&mut self, i2c: &mut I2C) -> Result<&[util::MilliVolts], Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        if !self.is_initialized() {
            return Err(Error::Uninitialized);
        }
        let mut buf = [0u8; X * 2];
        self.read_raw(i2c, registers::VC1_HI_BYTE, &mut buf)?;
        let adc_tf = self.adc_transfer_function();
        for (i, cell) in self.cells.iter_mut().enumerate() {
            let adc_reading = ((buf[i * 2] as u16) << 8) | buf[i * 2 + 1] as u16;
            *cell = adc_tf.apply(adc_reading);
        }

        let cc = self.cell_count;

        if cc == 3 || cc == 6 || cc == 9 {
            self.cells[2] = self.cells[4];
        } else if cc == 4 || cc == 7 || cc == 8 || cc == 10 || cc == 11 || cc == 12 {
            self.cells[3] = self.cells[4];
        }

        if (X == BQ76930 || X == BQ76940) && (cc == 6 || cc == 7 || cc == 9 || cc == 10) {
            self.cells[7] = self.cells[9];
        }

        if (X == BQ76930 || X == BQ76940)
            && (cc == 8 || cc == 9 || cc == 11 || cc == 12 || cc == 13)
        {
            self.cells[8] = self.cells[9];
        }

        if (X == BQ76940) && (cc == 9 || cc == 10 || cc == 11) {
            self.cells[12] = self.cells[14];
        }

        if (X == BQ76940) && (cc == 12 || cc == 13 || cc == 14) {
            self.cells[13] = self.cells[14];
        }

        Ok(&self.cells[..self.cell_count as usize])
    }

    pub fn enable_balancing<I2C>(&mut self, i2c: &mut I2C, cells: u16) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        self.write_raw(i2c, registers::CELLBAL1, &[(cells & 0b11111) as u8])?;
        if self.cell_count > 5 {
            self.write_raw(i2c, registers::CELLBAL2, &[((cells >> 5) & 0b11111) as u8])?;
        }
        if self.cell_count > 10 {
            self.write_raw(i2c, registers::CELLBAL3, &[((cells >> 10) & 0b11111) as u8])?;
        }

        Ok(())
    }

    pub fn balancing_state<I2C>(&mut self, i2c: &mut I2C) -> Result<u16, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut data = [0u8; 1];
        self.read_raw(i2c, registers::CELLBAL1, &mut data)?;
        let mut val = data[0] as u16;

        if self.cell_count > 5 {
            self.read_raw(i2c, registers::CELLBAL2, &mut data)?;
            val |= (data[0] as u16) << 5;
        }
        if self.cell_count > 10 {
            self.read_raw(i2c, registers::CELLBAL3, &mut data)?;
            val |= (data[0] as u16) << 10;
        }

        Ok(val)
    }

    pub fn current<I2C>(&mut self, i2c: &mut I2C) -> Result<util::MilliAmperes, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        // let mut sys_ctrl2 = [0u8; 1];
        // self.read_raw(i2c, 0x05, &mut sys_ctrl2)?;
        // sys_ctrl2[0] = sys_ctrl2[0] | 0b0010_0000;
        // self.write_raw(i2c, 0x05, &sys_ctrl2)?;
        // delay(8_000_000);
        let mut cc = [0u8; 2];
        self.read_raw(i2c, registers::CC_HI_BYTE, &mut cc)?;
        let cc = i16::from_be_bytes(cc);
        let vshunt = cc as i32 * 8440; // nV
        let current = vshunt / self.shunt.0 as i32;
        Ok(util::MilliAmperes(current))
    }

    pub fn voltage<I2C>(&mut self, i2c: &mut I2C) -> Result<util::MilliVolts, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        // let mut sys_ctrl2 = [0u8; 1];
        // self.read_raw(i2c, 0x05, &mut sys_ctrl2)?;
        // sys_ctrl2[0] = sys_ctrl2[0] | 0b0010_0000;
        // self.write_raw(i2c, 0x05, &sys_ctrl2)?;
        // delay(8_000_000);
        let mut vv = [0u8; 2];
        self.read_raw(i2c, registers::BAT_HI_BYTE, &mut vv)?;
        let vv = u16::from_be_bytes(vv);
        let voltage =
            4 * (self.adc_gain as i32) * (vv as i32) + 5 * (self.adc_offset as i32) * 1000;
        Ok(util::MilliVolts((voltage / 1000) as u32))
    }

    pub fn temperature<I2C>(&mut self, i2c: &mut I2C) -> Result<util::Temperature, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut ts = [0u8; 2];
        self.read_raw(i2c, registers::TS1_HI_BYTE, &mut ts)?;
        let ts = u16::from_be_bytes(ts);
        let vtsx = (ts as i32) * 382; // µV/LSB
        match self.temperature_source(i2c)? {
            util::TemperatureSource::InternalDie => Ok(util::Temperature::InternalDie(
                util::DegreesCentigrade(vtsx),
            )),
            util::TemperatureSource::ExternalThermistor => Ok(
                util::Temperature::ExternalThermistor(util::DegreesCentigrade(vtsx)),
            ),
        }
        // match source {
        //     TemperatureSource::InternalDie => {
        //         let v25 = 1200000; // µV at 25degC
        //         let t = 25 - ((vtsx - v25) * 238);
        //         Ok(DegreesCentigrade( t as i16 ))
        //     }
        //     TemperatureSource::ExternalThermistor => {
        //         // let rts = (10_000 * vtsx)
        //         Ok(DegreesCentigrade(0))
        //     }
        // }
    }

    pub fn sys_stat<I2C>(&mut self, i2c: &mut I2C) -> Result<util::SysStat, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut data = [0u8; 1];
        self.read_raw(i2c, registers::SYS_STAT, &mut data)?;
        Ok(util::SysStat::from_bits_truncate(data[0]))
        //Ok(util::Stat { bits: data[0] })
    }

    pub fn sys_stat_reset<I2C>(&mut self, i2c: &mut I2C, flags: util::SysStat) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        self.write_raw(i2c, registers::SYS_STAT, &[flags.bits()])
    }

    pub fn discharge<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sys_ctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sys_ctrl2)?;
        let mut sys_ctrl2 = util::SysCtrl2::from_bits_truncate(sys_ctrl2[0]);

        let already_enabled = sys_ctrl2.contains(util::SysCtrl2::DSG_ON);
        if enable == already_enabled {
            return Ok(());
        }
        sys_ctrl2.set(util::SysCtrl2::DSG_ON, enable);

        self.write_raw(i2c, registers::SYS_CTRL2, &[sys_ctrl2.bits()])
    }

    pub fn is_discharge_enabled<I2C>(&mut self, i2c: &mut I2C) -> Result<bool, Error>
        where
            I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sys_ctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sys_ctrl2)?;
        let sys_ctrl2 = util::SysCtrl2::from_bits_truncate(sys_ctrl2[0]);

        Ok(sys_ctrl2.contains(util::SysCtrl2::DSG_ON))
    }

    pub fn charge<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sys_ctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sys_ctrl2)?;
        let mut sys_ctrl2 = util::SysCtrl2::from_bits_truncate(sys_ctrl2[0]);

        let already_enabled = sys_ctrl2.contains(util::SysCtrl2::CHG_ON);
        if enable == already_enabled {
            return Ok(());
        }
        sys_ctrl2.set(util::SysCtrl2::CHG_ON, enable);

        self.write_raw(i2c, registers::SYS_CTRL2, &[sys_ctrl2.bits()])
    }

    pub fn is_charge_enabled<I2C>(&mut self, i2c: &mut I2C) -> Result<bool, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sys_ctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sys_ctrl2)?;
        let sys_ctrl2 = util::SysCtrl2::from_bits_truncate(sys_ctrl2[0]);

        Ok(sys_ctrl2.contains(util::SysCtrl2::CHG_ON))
    }

    pub fn ship_enter<I2C>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        self.write_raw(i2c, registers::SYS_CTRL1, &[0b0000_0000])?;
        self.write_raw(i2c, registers::SYS_CTRL1, &[0b0000_0001])?;
        self.write_raw(i2c, registers::SYS_CTRL1, &[0b0000_0010])?;
        Ok(())
    }

    fn adc_transfer_function(&self) -> util::AdcTransferFunction {
        util::AdcTransferFunction {
            gain: self.adc_gain,
            offset: self.adc_offset,
        }
    }

    fn ov_voltage_range(&self) -> (util::MilliVolts, util::MilliVolts) {
        let min_adc_reading = 0b10_0000_0000_1000;
        let max_adc_reading = 0b10_1111_1111_1000;
        (
            self.adc_transfer_function().apply(min_adc_reading),
            self.adc_transfer_function().apply(max_adc_reading),
        )
    }

    fn uv_voltage_range(&self) -> (util::MilliVolts, util::MilliVolts) {
        let min_adc_reading = 0b01_0000_0000_0000;
        let max_adc_reading = 0b01_1111_1111_0000;
        (
            self.adc_transfer_function().apply(min_adc_reading),
            self.adc_transfer_function().apply(max_adc_reading),
        )
    }

    pub fn init<I2C>(
        &mut self,
        i2c: &mut I2C,
        config: &util::Config,
    ) -> Result<util::CalculatedValues, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        self.read_adc_characteristics(i2c)?;

        let scd_threshold = util::SCDThreshold::from_current(config.scd_threshold, config.shunt);
        let ocd_threshold = util::OCDThreshold::from_current(config.ocd_threshold, config.shunt);
        let scd_range = scd_threshold.range();
        let ocd_range = ocd_threshold.range();
        if (scd_range == util::OCDSCDRange::Lower && ocd_range == util::OCDSCDRange::Upper)
            || (scd_range == util::OCDSCDRange::Upper && ocd_range == util::OCDSCDRange::Lower)
        {
            return Err(Error::OCDSCDRangeMismatch);
        }
        let range_to_use = if scd_range == util::OCDSCDRange::Unknown {
            if ocd_range == util::OCDSCDRange::Unknown {
                util::OCDSCDRange::Lower
            } else {
                ocd_range
            }
        } else if ocd_range == util::OCDSCDRange::Unknown {
            if scd_range == util::OCDSCDRange::Unknown {
                util::OCDSCDRange::Lower
            } else {
                scd_range
            }
        } else {
            ocd_range // both ranges are equal
        };
        let scd_bits = scd_threshold.bits(range_to_use.clone());
        let ocd_bits = ocd_threshold.bits(range_to_use.clone());

        let mut regs = [0u8; 6];
        regs[0] = range_to_use.bits() | config.scd_delay.bits() | scd_bits; // PROTECT1 (0x06)
        regs[1] = config.ocd_delay.bits() | ocd_bits; // PROTECT2 (0x07)
        regs[2] = config.uv_delay.bits() | config.ov_delay.bits(); // PROTECT3 (0x08)

        let ov_limits = self.ov_voltage_range();
        if !(config.ov_threshold >= ov_limits.0 && config.ov_threshold <= ov_limits.1) {
            return Err(Error::OVThresholdUnobtainable(ov_limits.0, ov_limits.1));
        }
        let ov_trip_full =
            ((config.ov_threshold.0 as i32 - self.adc_offset as i32) * 1000) / self.adc_gain as i32; // ADC value * 1000
        let ov_bits = (((ov_trip_full as u16) >> 4) & 0xff) as u8;

        let uv_limits = self.uv_voltage_range();
        if !(config.uv_threshold >= uv_limits.0 && config.uv_threshold <= uv_limits.1) {
            return Err(Error::UVThresholdUnobtainable(uv_limits.0, uv_limits.1));
        }
        let uv_trip_full =
            ((config.uv_threshold.0 as i32 - self.adc_offset as i32) * 1000) / self.adc_gain as i32; // ADC value * 1000
        let uv_bits = (((uv_trip_full as u16) >> 4) & 0xff) as u8;

        regs[3] = ov_bits; // (0x09)
        regs[4] = uv_bits; // (0xA)
        regs[5] = 0x19; // (0xB)

        self.write_raw(i2c, registers::PROTECT1, &regs)?;
        self.shunt = config.shunt;
        self.init_complete = true;

        let mut sysctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sysctrl2)?;
        sysctrl2[0] |= 0b0100_0000; // !!CC_EN!!
        self.write_raw(i2c, registers::SYS_CTRL2, &sysctrl2)?;

        Ok(util::CalculatedValues {
            ocdscd_range_used: range_to_use,
            scd_threshold: util::Amperes(((scd_threshold as u32) * 1000) / config.shunt.0),
            ocd_threshold: util::Amperes(((ocd_threshold as u32) * 1000) / config.shunt.0),
            uv_threshold: self
                .adc_transfer_function()
                .apply(0b01_0000_0000_0000 | ((uv_bits as u16) << 4)),
            ov_threshold: self
                .adc_transfer_function()
                .apply(0b10_0000_0000_1000 | ((ov_bits as u16) << 4)),
        })
    }

    pub fn enable_adc<I2C>(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sysctrl1 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL1, &mut sysctrl1)?;
        sysctrl1[0] &= !(1 << 4);
        sysctrl1[0] |= (enable as u8) << 4;
        self.write_raw(i2c, registers::SYS_CTRL1, &sysctrl1)
    }

    pub fn set_temperature_source<I2C>(
        &mut self,
        i2c: &mut I2C,
        source: util::TemperatureSource,
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sysctrl1 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL1, &mut sysctrl1)?;
        sysctrl1[0] &= !(1 << 3);
        let is_external = source == util::TemperatureSource::ExternalThermistor;
        sysctrl1[0] |= (is_external as u8) << 3;
        self.write_raw(i2c, registers::SYS_CTRL1, &sysctrl1)
    }

    pub fn temperature_source<I2C>(
        &mut self,
        i2c: &mut I2C,
    ) -> Result<util::TemperatureSource, Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sysctrl1 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL1, &mut sysctrl1)?;
        sysctrl1[0] &= !(1 << 3);
        let is_external = sysctrl1[0] & (1 << 3) != 0;
        if is_external {
            Ok(util::TemperatureSource::ExternalThermistor)
        } else {
            Ok(util::TemperatureSource::InternalDie)
        }
    }

    pub fn coulomb_counter_mode<I2C>(
        &mut self,
        i2c: &mut I2C,
        mode: util::CoulombCounterMode,
    ) -> Result<(), Error>
    where
        I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    {
        let mut sysctrl2 = [0u8; 1];
        self.read_raw(i2c, registers::SYS_CTRL2, &mut sysctrl2)?;
        sysctrl2[0] &= !0b0110_0000;
        match mode {
            util::CoulombCounterMode::Disabled => {}
            util::CoulombCounterMode::OneShot => {
                sysctrl2[0] |= 1 << 5;
            }
            util::CoulombCounterMode::Continuous => {
                sysctrl2[0] |= 1 << 6;
            }
        }
        self.write_raw(i2c, registers::SYS_CTRL2, &sysctrl2)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    struct DummyI2C {
        pub regs: [u8; 255],
    }

    impl DummyI2C {
        pub fn new() -> Self {
            let mut regs = [0u8; 255];
            regs[0x50] = 0x15;
            regs[0x51] = 0x2b;
            regs[0x59] = 0xa3;
            DummyI2C { regs }
        }
    }

    impl embedded_hal::blocking::i2c::Write for DummyI2C {
        type Error = ();

        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            std::println!("-----------");
            std::println!("write: {:#04x}", addr);
            let base_reg_addr = bytes[0] as usize;
            for (i, b) in bytes.iter().skip(1).enumerate() {
                let reg_addr = base_reg_addr + i;
                self.regs[reg_addr] = *b;
                std::println!(
                    "{}/{:#04x}\t<= {:#04x}={:#010b}",
                    reg_addr,
                    reg_addr,
                    *b,
                    *b
                );
            }

            Ok(())
        }
    }

    impl embedded_hal::blocking::i2c::WriteRead for DummyI2C {
        type Error = ();

        fn write_read(
            &mut self,
            address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            std::println!("----------------");
            std::println!("write_read: {:#04x}", address);
            let base_reg_addr = bytes[0] as usize;
            for (i, b) in buffer.iter_mut().enumerate() {
                let reg_addr = base_reg_addr + i;
                let reg_value = self.regs[reg_addr];
                *b = reg_value;
                std::println!(
                    "{}/{:#04x}\t== {:#04x}={:#010b}",
                    reg_addr,
                    reg_addr,
                    reg_value,
                    reg_value
                );
            }

            Ok(())
        }
    }

    #[test]
    fn it_works() {
        use crate::*;

        let mut i2c = DummyI2C::new();
        let mut bq769x0 = BQ769x0::new(0x08);
        let config = util::Config {
            shunt: util::MicroOhms(667),
            scd_delay: util::SCDDelay::_400uS,
            scd_threshold: util::Amperes(200),
            ocd_delay: util::OCDDelay::_1280ms,
            ocd_threshold: util::Amperes(100),
            uv_delay: util::UVDelay::_4s,
            uv_threshold: util::MilliVolts(2000),
            ov_delay: util::OVDelay::_4s,
            ov_threshold: util::MilliVolts(4175),
        };
        match bq769x0.init(&mut i2c, &config) {
            Ok(actual) => {
                std::println!("bq769x0 init ok");
                std::println!(
                    "adc gain:{}uV/LSB offset:{}mV",
                    bq769x0.adc_gain(),
                    bq769x0.adc_offset()
                );
                std::println!(
                    "SCD: {}, OCD: {}, range: {:?}",
                    actual.scd_threshold,
                    actual.ocd_threshold,
                    actual.ocdscd_range_used
                );
                std::println!("UV: {}, OV: {}", actual.uv_threshold, actual.ov_threshold);
            }
            Err(e) => {
                std::println!("bq769x0 init err: {:?}", e);
            }
        }
    }
}

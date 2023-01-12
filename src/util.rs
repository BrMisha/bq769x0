use core::fmt;
use core::fmt::Formatter;
use serde::{Deserialize, Serialize};
/*
pub struct Stat {
    pub(crate) bits: u8,
}

impl Stat {
    pub fn cc_ready_is_set(&self) -> bool {
        self.bits & (1u8 << 7) != 0
    }
    pub fn device_xready_is_set(&self) -> bool {
        self.bits & (1u8 << 5) != 0
    }
    pub fn ovrd_alert_is_set(&self) -> bool {
        self.bits & (1u8 << 4) != 0
    }
    pub fn undervoltage_is_set(&self) -> bool {
        self.bits & (1u8 << 3) != 0
    }
    pub fn overvoltage_is_set(&self) -> bool {
        self.bits & (1u8 << 2) != 0
    }
    pub fn scd_is_set(&self) -> bool {
        self.bits & (1u8 << 1) != 0
    }
    pub fn ocd_is_set(&self) -> bool {
        self.bits & (1u8 << 0) != 0
    }

    pub fn is_ok(&self) -> bool {
        self.bits & 0b0011_1111 == 0
    }

impl fmt::Debug for Stat {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let _ = write!(f, "(");
        if self.cc_ready_is_set() {
            let _ = write!(f, "CC_READY, ");
        };
        if self.device_xready_is_set() {
            let _ = write!(f, "XREADY, ");
        };
        if self.ovrd_alert_is_set() {
            let _ = write!(f, "ALERT, ");
        };
        if self.undervoltage_is_set() {
            let _ = write!(f, "UV, ");
        };
        if self.overvoltage_is_set() {
            let _ = write!(f, "OV, ");
        };
        if self.scd_is_set() {
            let _ = write!(f, "SCD, ");
        };
        if self.ocd_is_set() {
            let _ = write!(f, "OCD, ");
        };
        write!(f, ")")
    }
}
}*/

pub enum SCDDelay {
    _70uS,
    _100uS,
    _200uS,
    _400uS,
}

impl SCDDelay {
    pub fn bits(&self) -> u8 {
        match self {
            SCDDelay::_70uS => 0x0 << 3,
            SCDDelay::_100uS => 0x1 << 3,
            SCDDelay::_200uS => 0x2 << 3,
            SCDDelay::_400uS => 0x3 << 3,
        }
    }
}

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct Amperes(pub u32);

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy, Serialize, Deserialize)]
pub struct MilliAmperes(pub i32);

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct MicroOhms(pub u32);

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Serialize, Deserialize)]
pub struct MilliVolts(pub u32);

impl core::ops::Sub for MilliVolts {
    type Output = MilliVolts;

    fn sub(self, rhs: Self) -> Self::Output {
        MilliVolts(self.0 - rhs.0)
    }
}

impl fmt::Display for Amperes {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}A", self.0)
    }
}

impl fmt::Display for MilliAmperes {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}mA", self.0)
    }
}

impl fmt::Display for MilliVolts {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}mV", self.0)
    }
}
/*
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub struct DegreesCentigrade(pub i32);

impl fmt::Display for DegreesCentigrade {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}degC", self.0)
    }
}*/

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub enum TemperatureChannel {
    _1,
    _2,
    _3,
}

#[derive(Copy, Clone)]
pub enum SCDThreshold {
    // Lower range (RSNS = 0)
    _22mV = 22,
    _33mV = 33,
    _44mV = 44,
    _56mV = 56,
    _67mV = 67,
    _78mV = 78,
    _89mV = 89,
    _100mV = 100,
    // Upper range (RSNS = 1)
    //_44mV, same
    //_67mV, same
    //_89mV, same
    _111mV = 111,
    _133mV = 133,
    _155mV = 155,
    _178mV = 178,
    _200mV = 200,
}

#[derive(PartialEq, Clone, Debug)]
pub enum OCDSCDRange {
    Lower,
    Upper,
    Unknown,
}

impl OCDSCDRange {
    pub fn bits(&self) -> u8 {
        match self {
            OCDSCDRange::Lower => 0 << 7,
            OCDSCDRange::Upper => 1 << 7,
            OCDSCDRange::Unknown => {
                unreachable!()
            }
        }
    }
}

impl SCDThreshold {
    pub fn range(&self) -> OCDSCDRange {
        use SCDThreshold::{_111mV, _133mV, _155mV, _178mV, _200mV, _44mV, _67mV, _89mV};

        match self {
            _44mV | _67mV | _89mV => OCDSCDRange::Unknown,
            _111mV | _133mV | _155mV | _178mV | _200mV => OCDSCDRange::Upper,
            _ => OCDSCDRange::Lower,
        }
    }

    pub fn bits(&self, range: OCDSCDRange) -> u8 {
        use OCDSCDRange::{Lower, Upper};
        use SCDThreshold::{
            _100mV, _111mV, _133mV, _155mV, _178mV, _200mV, _22mV, _33mV, _44mV, _56mV, _67mV,
            _78mV, _89mV,
        };

        match range {
            Lower => {
                match self {
                    _22mV => 0x0,
                    _33mV => 0x1,
                    _44mV => 0x2,
                    _56mV => 0x3,
                    _67mV => 0x4,
                    _78mV => 0x5,
                    _89mV => 0x6,
                    _100mV => 0x7,
                    _ => 0x0, // upper range values, should not happen, 0 to be safe
                }
            }
            Upper => match self {
                _44mV => 0x0,
                _67mV => 0x1,
                _89mV => 0x2,
                _111mV => 0x3,
                _133mV => 0x4,
                _155mV => 0x5,
                _178mV => 0x6,
                _200mV => 0x7,
                _ => 0x0,
            },
            _ => {
                unreachable!()
            }
        }
    }

    pub fn from_mv(mv_threshold: u8) -> Self {
        use SCDThreshold::*;
        let thresholds = [
            _22mV, _33mV, _44mV, _56mV, _67mV, _78mV, _89mV, _100mV, _111mV, _133mV, _155mV,
            _178mV, _200mV,
        ];
        if mv_threshold < 22 {
            return _22mV;
        } else if mv_threshold > 200 {
            return _200mV;
        } else {
            for t in thresholds.iter() {
                if mv_threshold <= *t as u8 {
                    return *t;
                }
            }
        }
        unreachable!();
    }

    pub fn from_current(threshold: Amperes, shunt: MicroOhms) -> Self {
        let mv_threshold = threshold.0 * shunt.0 / 1000;
        Self::from_mv(mv_threshold as u8)
    }
}

pub enum OCDDelay {
    _8ms = 0x0,
    _20ms = 0x1,
    _40ms = 0x2,
    _80ms = 0x3,
    _160ms = 0x4,
    _320ms = 0x5,
    _640ms = 0x6,
    _1280ms = 0x7,
}

impl OCDDelay {
    pub fn bits(&self) -> u8 {
        match self {
            OCDDelay::_8ms => 0x0 << 4,
            OCDDelay::_20ms => 0x1 << 4,
            OCDDelay::_40ms => 0x2 << 4,
            OCDDelay::_80ms => 0x3 << 4,
            OCDDelay::_160ms => 0x4 << 4,
            OCDDelay::_320ms => 0x5 << 4,
            OCDDelay::_640ms => 0x6 << 4,
            OCDDelay::_1280ms => 0x7 << 4,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum OCDThreshold {
    // Lower range (RSNS = 0)
    _8mV = 8,
    _11mV = 11,
    _14mV = 14,
    _17mV = 17,
    _19mV = 19,
    _22mV = 22,
    _25mV = 25,
    _28mV = 28,
    _31mV = 31,
    _33mV = 33,
    _36mV = 36,
    _39mV = 39,
    _42mV = 42,
    _44mV = 44,
    _47mV = 47,
    _50mV = 50,
    // Upper range (RSNS = 1)
    //_17mV, same
    //_22mV,
    //_28mV,
    //_33mV,
    //_39mV,
    //_44mV,
    //_50mV,
    _56mV = 56,
    _61mV = 61,
    _67mV = 67,
    _72mV = 72,
    _78mV = 78,
    _83mV = 83,
    _89mV = 89,
    _94mV = 94,
    _100mV = 100,
}

impl OCDThreshold {
    pub fn range(&self) -> OCDSCDRange {
        use OCDThreshold::{
            _100mV, _17mV, _22mV, _28mV, _33mV, _39mV, _44mV, _50mV, _56mV, _61mV, _67mV, _72mV,
            _78mV, _83mV, _89mV, _94mV,
        };

        match self {
            _17mV | _22mV | _28mV | _33mV | _39mV | _44mV | _50mV => OCDSCDRange::Unknown,
            _56mV | _61mV | _67mV | _72mV | _78mV | _83mV | _89mV | _94mV | _100mV => {
                OCDSCDRange::Upper
            }
            _ => OCDSCDRange::Lower,
        }
    }

    pub fn bits(&self, range: OCDSCDRange) -> u8 {
        use OCDSCDRange::{Lower, Upper};
        use OCDThreshold::{
            _100mV, _11mV, _14mV, _17mV, _19mV, _22mV, _25mV, _28mV, _31mV, _33mV, _36mV, _39mV,
            _42mV, _44mV, _47mV, _50mV, _56mV, _61mV, _67mV, _72mV, _78mV, _83mV, _89mV, _8mV,
            _94mV,
        };

        match range {
            Lower => match self {
                _8mV => 0x0,
                _11mV => 0x1,
                _14mV => 0x2,
                _17mV => 0x3,
                _19mV => 0x4,
                _22mV => 0x5,
                _25mV => 0x6,
                _28mV => 0x7,
                _31mV => 0x8,
                _33mV => 0x9,
                _36mV => 0xa,
                _39mV => 0xb,
                _42mV => 0xc,
                _44mV => 0xd,
                _47mV => 0xe,
                _50mV => 0xf,
                _ => 0x0,
            },
            Upper => match self {
                _17mV => 0x0,
                _22mV => 0x1,
                _28mV => 0x2,
                _33mV => 0x3,
                _39mV => 0x4,
                _44mV => 0x5,
                _50mV => 0x6,
                _56mV => 0x7,
                _61mV => 0x8,
                _67mV => 0x9,
                _72mV => 0xa,
                _78mV => 0xb,
                _83mV => 0xc,
                _89mV => 0xd,
                _94mV => 0xe,
                _100mV => 0xf,
                _ => 0x0,
            },
            _ => {
                unreachable!()
            }
        }
    }

    pub fn from_mv(mv_threshold: u8) -> Self {
        use OCDThreshold::*;
        let thresholds = [
            _8mV, _11mV, _14mV, _17mV, _19mV, _22mV, _25mV, _28mV, _31mV, _33mV, _36mV, _39mV,
            _42mV, _44mV, _47mV, _50mV, _56mV, _61mV, _67mV, _72mV, _78mV, _83mV, _89mV, _94mV,
            _100mV,
        ];
        if mv_threshold < 8 {
            return _8mV;
        } else if mv_threshold > 100 {
            return _100mV;
        } else {
            for t in thresholds.iter() {
                if mv_threshold <= *t as u8 {
                    return *t;
                }
            }
        }
        unreachable!();
    }

    pub fn from_current(threshold: Amperes, shunt: MicroOhms) -> Self {
        let mv_threshold = threshold.0 * shunt.0 / 1000;
        Self::from_mv(mv_threshold as u8)
    }
}

pub enum UVDelay {
    _1s = 0x0,
    _4s = 0x1,
    _8s = 0x2,
    _16s = 0x3,
}

impl UVDelay {
    pub fn bits(&self) -> u8 {
        match self {
            UVDelay::_1s => 0x0 << 6,
            UVDelay::_4s => 0x1 << 6,
            UVDelay::_8s => 0x2 << 6,
            UVDelay::_16s => 0x3 << 6,
        }
    }
}

pub enum OVDelay {
    _1s = 0x0,
    _4s = 0x1,
    _8s = 0x2,
    _16s = 0x3,
}

impl OVDelay {
    pub fn bits(&self) -> u8 {
        match self {
            OVDelay::_1s => 0x0 << 4,
            OVDelay::_4s => 0x1 << 4,
            OVDelay::_8s => 0x2 << 4,
            OVDelay::_16s => 0x3 << 4,
        }
    }
}

pub struct Config {
    pub shunt: MicroOhms,

    // Short Circuit in Discharge
    pub scd_delay: SCDDelay,
    pub scd_threshold: Amperes,

    // Overcurrent in Discharge
    pub ocd_delay: OCDDelay,
    pub ocd_threshold: Amperes,

    // Undervoltage
    pub uv_delay: UVDelay,
    pub uv_threshold: MilliVolts,

    // Overvoltage
    pub ov_delay: OVDelay,
    pub ov_threshold: MilliVolts,
}

#[derive(Debug)]
pub struct CalculatedValues {
    pub ocdscd_range_used: OCDSCDRange,
    pub scd_threshold: Amperes,   // Short Circuit in Discharge
    pub ocd_threshold: Amperes,   // Overcurrent in Discharge
    pub uv_threshold: MilliVolts, // Undervoltage
    pub ov_threshold: MilliVolts, // Overvoltage
}

#[derive(Copy, Clone)]
pub struct AdcTransferFunction {
    pub(crate) gain: u16,
    pub(crate) offset: i8,
}

impl AdcTransferFunction {
    pub(crate) fn apply(&self, adc_reading: u16) -> MilliVolts {
        let adc_reading = adc_reading as i32;
        let uv = adc_reading * self.gain as i32 + self.offset as i32 * 1000;
        MilliVolts((uv / 1000) as u32)
    }
}

pub enum CoulombCounterMode {
    Disabled,
    OneShot,
    Continuous,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum TemperatureSource {
    InternalDie,
    ExternalThermistor,
}
/*
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum Temperature {
    InternalDie(DegreesCentigrade),
    ExternalThermistor(DegreesCentigrade),
}*/

bitflags::bitflags! {
    pub struct SysStat: u8 {
        const CC_READY      = 0b1000_0000;
        const DEVICE_XREADY = 0b0010_0000;
        const OVRD_ALERT    = 0b0001_0000;
        const UNDERVOLTAGE  = 0b0000_1000;
        const OVERVOLTAGE   = 0b0000_0100;
        const SHORTCIRCUIT  = 0b0000_0010;
        const OVERCURRENT   = 0b0000_0001;
        const ALL           = 0b1011_1111;
    }
}

impl SysStat {
    /*pub fn cc_ready_is_set(&self) -> bool {
        self.contains(SysStat::CC_READY)
    }
    pub fn device_xready_is_set(&self) -> bool {
        self.contains(SysStat::DEVICE_XREADY)
    }
    pub fn ovrd_alert_is_set(&self) -> bool {
        self.contains(SysStat::OVRD_ALERT)
    }
    pub fn undervoltage_is_set(&self) -> bool {
        self.contains(SysStat::UNDERVOLTAGE)
    }
    pub fn overvoltage_is_set(&self) -> bool {
        self.contains(SysStat::OVERVOLTAGE)
    }
    pub fn scd_is_set(&self) -> bool {
        self.contains(SysStat::SHORTCIRCUIT)
    }
    pub fn ocd_is_set(&self) -> bool {
        self.contains(SysStat::OVERCURRENT)
    }

    pub fn is_ok(&self) -> bool {
        self.bits & 0b0011_1111 == 0
    }*/
}

bitflags::bitflags! {
    pub struct SysCtrl1: u8 {
        const LOAD_PRESENT = 1 << 7;
        const ADC_EN = 1 << 4;
        const TEMP_SEL = 1 << 3;
        const SHUT_A = 1 << 1;
        const SHUT_B = 1 << 0;
    }
}

bitflags::bitflags! {
    pub struct SysCtrl2: u8 {
        const DELAY_DIS = 1 << 7;
        const CC_EN = 1 << 6;
        const CC_ONESHOT = 1 << 5;
        const DSG_ON = 1 << 1;
        const CHG_ON = 1 << 0;
    }
}
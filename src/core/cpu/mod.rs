use std::fmt::{Debug, Formatter};

pub(super) mod inst;

/// The Sharp SM83 CPU Core.
#[derive(Clone, Default, Debug)]
pub struct Cpu {
    gr: GeneralRegisters,
    sp: u16,
    pc: u16,
    ir: u8,
    ime: bool,
    if_: u8,
    ie: u8,
    ime_next: bool,
    cycle_count: usize,

    /// Cpu is frozen, but the rest of the system works. Only
    /// set by illegal instructions.
    hang: bool,

    halted: bool,
    halt_bug: bool,
}

#[derive(Copy, Clone)]
union GeneralRegisters {
    rp: RegisterPairs,
    sr: SingleRegisters,
}

impl Default for GeneralRegisters {
    fn default() -> Self {
        Self {
            rp: RegisterPairs::default(),
        }
    }
}

impl Debug for GeneralRegisters {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let sr = unsafe { &self.sr };

        let mut st = f.debug_struct("GeneralRegisters");

        macro_rules! field {
            ($($name: ident),*) => {
                {
                    $(
                    st.field(stringify!($name), &sr.$name);
                    )*
                }
            };
        }

        field!(a, f, b, c, d, e, h, l);

        st.finish()
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
struct RegisterPairs {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
#[cfg(target_endian = "little")]
struct SingleRegisters {
    f: u8,
    a: u8,
    c: u8,
    b: u8,
    e: u8,
    d: u8,
    l: u8,
    h: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
#[cfg(target_endian = "big")]
struct SingleRegisters {
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
}

#[cfg(test)]
mod tests {
    use crate::core::cpu::{GeneralRegisters, SingleRegisters};

    #[test]
    fn test_register_pairs() {
        let mut gr = GeneralRegisters::default();

        // SAFETY: Union fields in Rust have defined behavior. u8 and u16 don't have any illegal bit
        // patterns, and the structs we're using don't have any padding. We know these assertions
        // should pass if the struct is implemented correctly.

        unsafe {
            gr.rp.hl = 0x1234;
            assert_eq!(gr.sr.h, 0x12);
            assert_eq!(gr.sr.l, 0x34);
        }

        unsafe {
            gr.sr.b = 0x56;
            gr.sr.c = 0x78;
            assert_eq!(gr.rp.bc, 0x5678);
        }
    }

    #[test]
    fn test_gpr_debug() {
        let mut registers = GeneralRegisters::default();

        let rp = unsafe { &mut registers.rp };
        rp.af = 0x1234;
        rp.bc = 0x5678;
        rp.de = 0x9abc;
        rp.hl = 0xdef0;

        let SingleRegisters {
            f,
            a,
            c,
            b,
            e,
            d,
            l,
            h,
        } = SingleRegisters {
            a: 0x12,
            f: 0x34,
            b: 0x56,
            c: 0x78,
            d: 0x9a,
            e: 0xbc,
            h: 0xde,
            l: 0xf0,
        };

        assert_eq!(format!("{registers:?}"), format!("GeneralRegisters {{ a: {a}, f: {f}, b: {b}, c: {c}, d: {d}, e: {e}, h: {h}, l: {l} }}"))
    }
}

#[cfg(test)]
mod auto_test {
    use crate::core::mem::MemoryBus;
    use crate::core::GameBoy;
    use anyhow::anyhow;
    use serde::de::Error;
    use serde::{Deserialize, Deserializer};
    use std::fs::File;
    use std::io::BufReader;
    use std::path::Path;

    struct TestBus {
        ram: [u8; 0x10000],
        events: Vec<CycleEvent>,
    }

    impl Default for TestBus {
        fn default() -> Self {
            Self {
                ram: [0; 0x10000],
                events: Vec::new(),
            }
        }
    }

    impl MemoryBus for TestBus {
        fn m_cycle(&mut self) {
            self.events.push(CycleEvent::default());
        }

        fn pure_read(&self, address: u16) -> u8 {
            self.ram[address as usize]
        }

        fn pure_write(&mut self, address: u16, value: u8) {
            self.ram[address as usize] = value;
        }

        fn cycle_read(&mut self, address: u16) -> u8 {
            let value = self.pure_read(address);
            self.events.push(CycleEvent::new_read(address, value));
            value
        }

        fn cycle_write(&mut self, address: u16, value: u8) {
            self.events.push(CycleEvent::new_write(address, value));
            self.pure_write(address, value);
        }
    }

    #[derive(Default, Copy, Clone, Eq, PartialEq, Debug)]
    struct MemoryLines {
        read: bool,
        write: bool,
        memory: bool,
    }

    impl<'de> Deserialize<'de> for MemoryLines {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let value = String::deserialize(deserializer)?;

            let chars = value.chars().collect::<Vec<_>>();

            if chars.len() != 3 {
                return Err(Error::custom("expected 3-character-long string"));
            }

            let mut result = Self::default();

            match chars[0] {
                '-' => result.read = false,
                'r' => result.read = true,
                _ => {
                    return Err(Error::custom(
                        "expected '-' or 'r' to denote whether a read was active",
                    ))
                }
            }

            match chars[1] {
                '-' => result.write = false,
                'w' => result.write = true,
                _ => {
                    return Err(Error::custom(
                        "expected '-' or 'w' to denote whether a write was active",
                    ))
                }
            }

            match chars[2] {
                '-' => result.memory = false,
                'm' => result.memory = true,
                _ => {
                    return Err(Error::custom(
                        "expected '-' or 'm' to denote whether a memory access was active",
                    ))
                }
            }

            if !result.memory && (result.write || result.read) {
                return Err(Error::custom(
                    "if a memory read is not active, then neither is a read or write",
                ));
            }

            Ok(result)
        }
    }

    #[derive(Deserialize, Default, Copy, Clone, Eq, PartialEq, Debug)]
    struct CycleEvent {
        address: Option<u16>,
        value: Option<u8>,
        lines: MemoryLines,
    }

    impl CycleEvent {
        fn new_read(address: u16, value: u8) -> Self {
            CycleEvent {
                address: Some(address),
                value: Some(value),
                lines: MemoryLines {
                    read: true,
                    memory: true,
                    ..MemoryLines::default()
                },
            }
        }

        fn new_write(address: u16, value: u8) -> Self {
            CycleEvent {
                address: Some(address),
                value: Some(value),
                lines: MemoryLines {
                    write: true,
                    memory: true,
                    ..MemoryLines::default()
                },
            }
        }
    }

    #[derive(Deserialize)]
    struct TestState {
        pc: u16,
        sp: u16,
        a: u8,
        b: u8,
        c: u8,
        d: u8,
        e: u8,
        f: u8,
        h: u8,
        l: u8,
        ram: Vec<(u16, u8)>,
    }

    impl TestState {
        fn initialize(&self, game_boy: &mut GameBoy<TestBus>) {
            game_boy.cpu.pc = self.pc;
            game_boy.cpu.sp = self.sp;
            game_boy.cpu.ime = false;
            game_boy.cpu.ime_next = false;

            let sr = unsafe { &mut game_boy.cpu.gr.sr };

            sr.a = self.a;
            sr.b = self.b;
            sr.c = self.c;
            sr.d = self.d;
            sr.e = self.e;
            sr.f = self.f & 0xf0;
            sr.h = self.h;
            sr.l = self.l;

            for &(address, value) in &self.ram {
                game_boy.mem.pure_write(address, value);
            }
        }

        fn validate(&self, game_boy: &GameBoy<TestBus>) -> anyhow::Result<()> {
            if self.pc != game_boy.cpu.pc {
                return Err(anyhow!(
                    "PC values differ: expected {:04X}, got {:04X}",
                    self.pc,
                    game_boy.cpu.pc
                ));
            }

            if self.sp != game_boy.cpu.sp {
                return Err(anyhow!(
                    "SP values differ: expected {:04X}, got {:04X}",
                    self.sp,
                    game_boy.cpu.sp
                ));
            }

            let sr = unsafe { &game_boy.cpu.gr.sr };

            if self.a != sr.a {
                return Err(anyhow!(
                    "A values differ: expected {:02X}, got {:02X}",
                    self.a,
                    sr.a
                ));
            }

            if self.b != sr.b {
                return Err(anyhow!(
                    "B values differ: expected {:02X}, got {:02X}",
                    self.b,
                    sr.b
                ));
            }

            if self.c != sr.c {
                return Err(anyhow!(
                    "C values differ: expected {:02X}, got {:02X}",
                    self.c,
                    sr.c
                ));
            }

            if self.d != sr.d {
                return Err(anyhow!(
                    "D values differ: expected {:02X}, got {:02X}",
                    self.d,
                    sr.d
                ));
            }

            if self.e != sr.e {
                return Err(anyhow!(
                    "E values differ: expected {:02X}, got {:02X}",
                    self.e,
                    sr.e
                ));
            }

            if self.f != sr.f {
                return Err(anyhow!(
                    "F values differ: expected {:02X}, got {:02X}",
                    self.f,
                    sr.f
                ));
            }

            if self.h != sr.h {
                return Err(anyhow!(
                    "H values differ: expected {:02X}, got {:02X}",
                    self.h,
                    sr.h
                ));
            }

            if self.l != sr.l {
                return Err(anyhow!(
                    "L values differ: expected {:02X}, got {:02X}",
                    self.l,
                    sr.l
                ));
            }

            for &(address, value) in &self.ram {
                let read_value = game_boy.mem.pure_read(address);
                if value != read_value {
                    return Err(anyhow!("RAM values differ at address {address:04X}: expected {value:02X}, got {read_value:02X}"));
                }
            }

            Ok(())
        }
    }

    #[derive(Deserialize)]
    struct TestCase {
        name: String,
        #[serde(rename = "initial")]
        start: TestState,
        #[serde(rename = "final")]
        end: TestState,
        cycles: Vec<CycleEvent>,
    }

    fn run_test(file_path: impl AsRef<Path>) -> anyhow::Result<()> {
        let file = BufReader::new(File::open(&file_path).inspect_err(|_| eprintln!("File '{}'", file_path.as_ref().display()))?);

        let test_cases: Vec<TestCase> = serde_json::from_reader(file)?;

        test_cases
            .iter()
            .map(|case| -> anyhow::Result<()> {
                let mut game_boy = GameBoy::new(TestBus::default());

                case.start.initialize(&mut game_boy);

                game_boy.step();

                case.end
                    .validate(&game_boy)
                    .inspect_err(|_| eprintln!("Test {} failed", case.name))?;

                for (i, (expected_cycle, real_cycle)) in
                    case.cycles.iter().zip(&game_boy.mem.events).enumerate()
                {
                    if !expected_cycle.lines.memory && !real_cycle.lines.memory {
                        // Assume that the address/data lines are meaningless when the processor isn't trying to use them.
                        continue;
                    }

                    if expected_cycle != real_cycle {
                        eprintln!("Test {} failed", case.name);
                        return Err(anyhow!(
                            "Cycle {expected_cycle:?} expected, got {real_cycle:?} on cycle {i}"
                        ));
                    }
                }

                Ok(())
            })
            .collect::<anyhow::Result<Vec<()>>>()?;

        Ok(())
    }

    #[test]
    fn test_instructions() {
        for opcode in 0..=255u8 {
            if [
                0xd3, 0xdb, 0xdd, 0xe3, 0xe4, 0xeb, 0xec, 0xed, 0xf4, 0xfc, 0xfd,
            ]
            .contains(&opcode)
            {
                // illegal opcodes
                continue;
            }

            if opcode == 0xcb {
                // prefix
                continue;
            }

            let path = format!("./CpuTests/v1/{opcode:02x}.json");
            run_test(&path).unwrap();
        }

        for opcode in 0..=255u8 {
            let path = format!("./CpuTests/v1/cb {opcode:02x}.json");
            run_test(&path).unwrap();
        }
    }
}

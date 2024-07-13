//! This module defines the "backend" of the emulator. It should not be coupled to the frontend
//! in any way, and only contains code relevant to the functioning of the core GameBoy architecture.

use crate::core::cpu::Cpu;
use crate::core::mem::MemoryBus;

pub mod cpu;
pub mod mem;

pub struct GameBoy<M> {
    pub cpu: Cpu,
    pub mem: M,
    pending_m_cycles: isize,
}

impl<M: MemoryBus> GameBoy<M> {
    pub fn new(mem: M) -> Self {
        Self {
            cpu: Cpu::default(),
            mem,
            pending_m_cycles: 0,
        }
    }

    pub fn step_n(&mut self, num_cycles: usize) {
        self.pending_m_cycles += num_cycles as isize;

        while self.pending_m_cycles > 0 {
            self.pending_m_cycles -= cpu::inst::step(&mut self.cpu, &mut self.mem) as isize;
        }
    }

    pub fn step(&mut self) {
        self.pending_m_cycles = 0;
        cpu::inst::step(&mut self.cpu, &mut self.mem);
    }
}
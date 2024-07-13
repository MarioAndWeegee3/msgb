
/// The memory bus allows for memory-mapped I/O with the various components in the system.
pub trait MemoryBus {
    /// Advance one t-cycle (2**22 Hz). Called four times by the [m_cycle] routine.
    fn t_cycle(&mut self) {}
    /// Advance one m-cycle (2**20 Hz). Best to override [t_cycle] instead unless no device advances
    /// on t-cycles.
    fn m_cycle(&mut self) {
        for _ in 0..4 {
            self.t_cycle();
        }
    }
    /// Read one byte at the given address. Should not have side effects.
    fn pure_read(&self, address: u16) -> u8;
    /// Write one byte at the given address. Should have no side effects other than writing the byte.
    fn pure_write(&mut self, address: u16, value: u8);

    /// Read one byte at the given address and advance the system by one m-cycle.
    fn cycle_read(&mut self, address: u16) -> u8 {
        let result = self.pure_read(address);
        self.m_cycle();
        result
    }
    /// Write one byte at the given address and advance the system by one m-cycle.
    fn cycle_write(&mut self, address: u16, value: u8) {
        self.pure_write(address, value);
        self.m_cycle();
    }
}
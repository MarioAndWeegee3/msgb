//! The implementation of all CPU instructions
// #![allow(unused_variables)]

use crate::core::cpu::Cpu;
use crate::core::mem::MemoryBus;

type Instruction = fn(&mut Cpu, &mut dyn MemoryBus);

#[rustfmt::skip]
static INSTRUCTIONS: &[Instruction] = &[
    //        xx0          xx1         xx2        xx3       xx4          xx5       xx6       xx7
    /* 00x */ nop,         ld_r16_n16, ld_dr16_a, inc_r16,  inc_r8,      dec_r8,   ld_r8_n8, rlca,
    /* 01x */ ld_dn16_sp,  add_hl_r16, ld_a_dr16, dec_r16,  inc_r8,      dec_r8,   ld_r8_n8, rrca,
    /* 02x */ illegal,     ld_r16_n16, ld_dr16_a, inc_r16,  inc_r8,      dec_r8,   ld_r8_n8, rla,
    /* 03x */ jr_o8,       add_hl_r16, ld_a_dr16, dec_r16,  inc_r8,      dec_r8,   ld_r8_n8, rra,
    /* 04x */ jr_cc_o8,    ld_r16_n16, ld_dr16_a, inc_r16,  inc_r8,      dec_r8,   ld_r8_n8, daa,
    /* 05x */ jr_cc_o8,    add_hl_r16, ld_a_dr16, dec_r16,  inc_r8,      dec_r8,   ld_r8_n8, cpl,
    /* 06x */ jr_cc_o8,    ld_r16_n16, ld_dr16_a, inc_r16,  inc_r8,      dec_r8,   ld_r8_n8, scf,
    /* 07x */ jr_cc_o8,    add_hl_r16, ld_a_dr16, dec_r16,  inc_r8,      dec_r8,   ld_r8_n8, ccf,
    /* 10x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 11x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 12x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 13x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 14x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 15x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 16x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, halt,     ld_r8_r8,
    /* 17x */ ld_r8_r8,    ld_r8_r8,   ld_r8_r8,  ld_r8_r8, ld_r8_r8,    ld_r8_r8, ld_r8_r8, ld_r8_r8,
    /* 20x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 21x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 22x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 23x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 24x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 25x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 26x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 27x */ alu_a_r8,    alu_a_r8,   alu_a_r8,  alu_a_r8, alu_a_r8,    alu_a_r8, alu_a_r8, alu_a_r8,
    /* 30x */ ret_cc,      pop_r16,    jp_cc_n16, jp_n16,   call_cc_n16, push_r16, alu_a_n8, rst,
    /* 31x */ ret_cc,      ret,        jp_cc_n16, prefix,   call_cc_n16, call_n16, alu_a_n8, rst,
    /* 32x */ ret_cc,      pop_r16,    jp_cc_n16, illegal,  call_cc_n16, push_r16, alu_a_n8, rst,
    /* 33x */ ret_cc,      reti,       jp_cc_n16, illegal,  call_cc_n16, illegal,  alu_a_n8, rst,
    /* 34x */ ld_dn8_a,    pop_r16,    ld_dc_a,   illegal,  illegal,     push_r16, alu_a_n8, rst,
    /* 35x */ add_sp_o8,   jp_hl,      ld_dn16_a, illegal,  illegal,     illegal,  alu_a_n8, rst,
    /* 36x */ ld_a_dn8,    pop_r16,    ld_a_dc,   di,       illegal,     push_r16, alu_a_n8, rst,
    /* 37x */ ld_hl_sp_o8, ld_sp_hl,   ld_a_dn16, ei,       illegal,     illegal,  alu_a_n8, rst,
];

/// Advances one instruction and returns the number of elapsed m-cycles.
pub fn step(cpu: &mut Cpu, mem: &mut dyn MemoryBus) -> usize {
    cpu.cycle_count = 0;

    if cpu.hang {
        cycle(cpu, mem);
        return cpu.cycle_count;
    }

    let mut duplicate_read = false;

    if cpu.halt_bug {
        if cpu.halted {
            if cpu.ie & cpu.if_ & 0x1f != 0 {
                cpu.halted = false;
                cpu.halt_bug = false;
            } else {
                return 0;
            }
        } else {
            cpu.halt_bug = false;
            duplicate_read = true;
        }
    } else if cpu.halted {
        todo!("handle halt mode properly")
    }

    cpu.ir = read(cpu, mem, cpu.pc);

    if !duplicate_read {
        cpu.pc += 1;
    }

    let instruction = INSTRUCTIONS[cpu.ir as usize];

    instruction(cpu, mem);

    cpu.cycle_count
}

const FLAG_Z: u8 = 0b1000_0000;
const FLAG_N: u8 = 0b0100_0000;
const FLAG_H: u8 = 0b0010_0000;
const FLAG_C: u8 = 0b0001_0000;

fn get_flag(cpu: &Cpu, flag: u8) -> bool {
    let f = unsafe { cpu.gr.sr.f };

    f & flag != 0
}

fn set_flag(cpu: &mut Cpu, flag: u8, value: bool) {
    let f = unsafe { &mut cpu.gr.sr.f };

    if value {
        *f |= flag;
    } else {
        *f &= !flag;
    }
}

fn get_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus, select: u8) -> u8 {
    let sr = unsafe { &cpu.gr.sr };

    match select {
        0 => sr.b,
        1 => sr.c,
        2 => sr.d,
        3 => sr.e,
        4 => sr.h,
        5 => sr.l,
        6 => {
            let address = unsafe { cpu.gr.rp.hl };
            read(cpu, mem, address)
        }
        7 => sr.a,
        _ => unreachable!(),
    }
}

fn set_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus, select: u8, value: u8) {
    let sr = unsafe { &mut cpu.gr.sr };

    match select {
        0 => sr.b = value,
        1 => sr.c = value,
        2 => sr.d = value,
        3 => sr.e = value,
        4 => sr.h = value,
        5 => sr.l = value,
        6 => {
            let address = unsafe { cpu.gr.rp.hl };
            write(cpu, mem, address, value);
        }
        7 => sr.a = value,
        _ => unreachable!(),
    }
}

fn get_cc(cpu: &Cpu, select: u8) -> bool {
    match select {
        0 => !get_flag(cpu, FLAG_Z),
        1 => get_flag(cpu, FLAG_Z),
        2 => !get_flag(cpu, FLAG_C),
        3 => get_flag(cpu, FLAG_C),
        _ => unreachable!(),
    }
}

fn pop(cpu: &mut Cpu, mem: &mut dyn MemoryBus) -> u16 {
    let low = read(cpu, mem, cpu.sp);
    cpu.sp += 1;
    let high = read(cpu, mem, cpu.sp);
    cpu.sp += 1;
    u16::from_be_bytes([high, low])
}

fn push(cpu: &mut Cpu, mem: &mut dyn MemoryBus, value: u16) {
    let [high, low] = value.to_be_bytes();

    cpu.sp -= 1;
    write(cpu, mem, cpu.sp, high);
    cpu.sp -= 1;
    write(cpu, mem, cpu.sp, low);
}

fn read(cpu: &mut Cpu, mem: &mut dyn MemoryBus, address: u16) -> u8 {
    cpu.cycle_count += 1;
    mem.cycle_read(address)
}

fn write(cpu: &mut Cpu, mem: &mut dyn MemoryBus, address: u16, value: u8) {
    cpu.cycle_count += 1;
    mem.cycle_write(address, value);
}

fn cycle(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    cpu.cycle_count += 1;
    mem.m_cycle();
}

/// Read an 8-bit value at the program counter. Takes 1 m-cycle.
fn read_pc(cpu: &mut Cpu, mem: &mut dyn MemoryBus) -> u8 {
    let value = read(cpu, mem, cpu.pc);
    cpu.pc += 1;

    value
}

/// Read a 16-bit value at the program counter. Takes 2 m-cycles.
fn read_n16_pc(cpu: &mut Cpu, mem: &mut dyn MemoryBus) -> u16 {
    let low = read_pc(cpu, mem);
    let high = read_pc(cpu, mem);
    u16::from_be_bytes([high, low])
}

fn get_p(opcode: u8) -> u8 {
    (opcode >> 4) & 0b11
}

fn get_x(opcode: u8) -> u8 {
    (opcode >> 6) & 0b11
}

fn get_y(opcode: u8) -> u8 {
    (opcode >> 3) & 0b111
}

fn get_z(opcode: u8) -> u8 {
    opcode & 0b111
}

fn get_addr_indirect(cpu: &mut Cpu) -> u16 {
    let select = get_p(cpu.ir);

    let rp = unsafe { &mut cpu.gr.rp };

    match select {
        0 => rp.bc,
        1 => rp.de,
        2 => {
            let v = rp.hl;
            rp.hl += 1;
            v
        }
        3 => {
            let v = rp.hl;
            rp.hl -= 1;
            v
        }
        _ => unreachable!(),
    }
}

fn nop(_: &mut Cpu, _: &mut dyn MemoryBus) {}
fn illegal(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    cpu.hang = true;
}

fn ld_r16_n16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = read_n16_pc(cpu, mem);

    let opcode = cpu.ir;

    let select = get_p(opcode);

    let rp = unsafe { &mut cpu.gr.rp };

    match select {
        0 => rp.bc = value,
        1 => rp.de = value,
        2 => rp.hl = value,
        3 => cpu.sp = value,
        _ => unreachable!(),
    }
}

fn ld_dr16_a(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = unsafe { cpu.gr.sr.a };

    let address = get_addr_indirect(cpu);

    write(cpu, mem, address, value);
}

fn inc_r16(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let opcode = cpu.ir;
    let select = get_p(opcode);

    let rp = unsafe { &mut cpu.gr.rp };

    match select {
        0 => rp.bc += 1,
        1 => rp.de += 1,
        2 => rp.hl += 1,
        3 => cpu.sp += 1,
        _ => unreachable!(),
    }
}

fn inc_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let opcode = cpu.ir;
    let select = get_y(opcode);

    let value = get_r8(cpu, mem, select);
    let result = value + 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, (value & 0x0f) + 1 > 0x0f);

    set_r8(cpu, mem, select, result);
}

fn dec_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let opcode = cpu.ir;
    let select = get_y(opcode);

    let value = get_r8(cpu, mem, select);
    let result = value - 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, true);
    set_flag(cpu, FLAG_H, (value & 0x0f) < 1);

    set_r8(cpu, mem, select, result);
}

fn ld_r8_n8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let opcode = cpu.ir;
    let select = get_y(opcode);

    let value = read_pc(cpu, mem);

    set_r8(cpu, mem, select, value);
}

//noinspection SpellCheckingInspection
fn rlca(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let sr = unsafe { &mut cpu.gr.sr };

    let value = sr.a;
    let result = value.rotate_left(1);

    sr.a = result;

    set_flag(cpu, FLAG_Z | FLAG_N | FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 0x80 != 0);
}

fn ld_dn16_sp(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = read_n16_pc(cpu, mem);

    let value = cpu.sp;

    let [high, low] = value.to_be_bytes();

    write(cpu, mem, address, low);
    write(cpu, mem, address + 1, high);
}

fn add_hl_r16(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let select = get_p(cpu.ir);

    let rp = unsafe { &mut cpu.gr.rp };

    let lhs = rp.hl;

    let rhs = match select {
        0 => rp.bc,
        1 => rp.de,
        2 => rp.hl,
        3 => cpu.sp,
        _ => unreachable!(),
    };

    let result = lhs + rhs;
    rp.hl = result;

    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, (lhs & 0x0fff) + (rhs & 0x0fff) > 0x0fff);
    set_flag(cpu, FLAG_C, (lhs as u32) + (rhs as u32) > 0xffff);
}

fn ld_a_dr16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = get_addr_indirect(cpu);

    let value = read(cpu, mem, address);
    unsafe {
        cpu.gr.sr.a = value;
    }
}

fn dec_r16(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let opcode = cpu.ir;
    let select = get_p(opcode);

    let rp = unsafe { &mut cpu.gr.rp };

    match select {
        0 => rp.bc -= 1,
        1 => rp.de -= 1,
        2 => rp.hl -= 1,
        3 => cpu.sp -= 1,
        _ => unreachable!(),
    }
}

//noinspection SpellCheckingInspection
fn rrca(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let sr = unsafe { &mut cpu.gr.sr };

    let value = sr.a;
    let result = value.rotate_right(1);
    sr.a = result;

    set_flag(cpu, FLAG_N | FLAG_H | FLAG_Z, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);
}

fn rla(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let carry_in = get_flag(cpu, FLAG_C) as u8;

    let sr = unsafe { &mut cpu.gr.sr };

    let value = sr.a;
    let result = (value << 1) | carry_in;
    sr.a = result;

    set_flag(cpu, FLAG_N | FLAG_H | FLAG_Z, false);
    set_flag(cpu, FLAG_C, value & 0x80 != 0);
}

fn jr_o8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let offset = read_pc(cpu, mem) as i8 as i16 as u16;
    let base = cpu.pc;
    let address = base + offset;

    cpu.pc = address;

    cycle(cpu, mem);
}

fn rra(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let carry_in = (get_flag(cpu, FLAG_C) as u8) << 7;

    let sr = unsafe { &mut cpu.gr.sr };

    let value = sr.a;
    let result = (value >> 1) | carry_in;
    sr.a = result;

    set_flag(cpu, FLAG_N | FLAG_H | FLAG_Z, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);
}

fn jr_cc_o8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let offset = read_pc(cpu, mem) as i8 as i16 as u16;
    let base = cpu.pc;
    let address = base + offset;

    let condition = get_cc(cpu, get_y(cpu.ir) - 4);

    if condition {
        cpu.pc = address;

        cycle(cpu, mem);
    }
}

/// DAA implementation adapted from [SameBoy](https://github.com/LIJI32/SameBoy/blob/06c6ce7d657a8576e556061d9a27deabf029f6cd/Core/sm83_cpu.c#L706C1-L741C2)
// fn daa(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
//     let mut result = unsafe { cpu.gr.sr.a } as u16;
//
//     if get_flag(cpu, FLAG_N) {
//         if get_flag(cpu, FLAG_H) {
//             result = (result - 0x6) & 0xff;
//         }
//
//         if get_flag(cpu, FLAG_C) {
//             result -= 0x60;
//         }
//     } else {
//         if get_flag(cpu, FLAG_H) || (result & 0xf) > 0x9 {
//             result += 0x6;
//         }
//
//         if get_flag(cpu, FLAG_C) || result > 0x9f {
//             result += 0x60;
//         }
//     }
//
//     set_flag(cpu, FLAG_Z, result & 0xff == 0);
//     set_flag(cpu, FLAG_C, result > 0xff);
//     set_flag(cpu, FLAG_H, false);
//
//     unsafe {
//         cpu.gr.sr.a = result as u8;
//     }
// }
fn daa(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let mut carry = false;
    let mut a = unsafe { cpu.gr.sr.a };
    if !get_flag(cpu, FLAG_N) {
        if get_flag(cpu, FLAG_C) || a > 0x99 {
            a += 0x60;
            carry = true;
        }
        if get_flag(cpu, FLAG_H) || a & 0xf > 0x9 {
            a += 0x6;
        }
    } else if get_flag(cpu, FLAG_C) {
        carry = true;
        a += if get_flag(cpu, FLAG_H) { 0x9a } else { 0xa0 };
    } else if get_flag(cpu, FLAG_H) {
        a += 0xfa;
    }

    set_flag(cpu, FLAG_Z, a == 0);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, carry);

    unsafe { cpu.gr.sr.a = a }
}

fn cpl(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    let sr = unsafe { &mut cpu.gr.sr };
    sr.a = !sr.a;

    set_flag(cpu, FLAG_N | FLAG_H, true);
}

fn scf(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    set_flag(cpu, FLAG_N | FLAG_H, false);
    set_flag(cpu, FLAG_C, true);
}

fn ccf(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    set_flag(cpu, FLAG_N | FLAG_H, false);
    set_flag(cpu, FLAG_C, !get_flag(cpu, FLAG_C));
}

fn ld_r8_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let dst = get_y(cpu.ir);
    let src = get_z(cpu.ir);

    let value = get_r8(cpu, mem, src);

    set_r8(cpu, mem, dst, value);
}

fn halt(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    if cpu.ime {
        cpu.halted = true;
    } else {
        cpu.halt_bug = true;
        if cpu.ie & cpu.if_ & 0x1f != 0 {
            cpu.halted = true;
        }
    }
}

static ALU_OPS: [fn(&mut Cpu, u8, u8) -> u8; 8] = [add, adc, sub, sbc, and, xor, or, cp];

fn add(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let result = (lhs as u16) + (rhs as u16);

    set_flag(cpu, FLAG_Z, result & 0xff == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, (lhs & 0xf) + (rhs & 0xf) > 0xf);
    set_flag(cpu, FLAG_C, result > 0xff);

    result as u8
}

fn adc(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let carry_in = get_flag(cpu, FLAG_C);
    let result = (lhs as u16) + (rhs as u16) + (carry_in as u16);

    set_flag(cpu, FLAG_Z, result & 0xff == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(
        cpu,
        FLAG_H,
        (lhs & 0xf) + (rhs & 0xf) + (carry_in as u8) > 0xf,
    );
    set_flag(cpu, FLAG_C, result > 0xff);

    result as u8
}

fn sub(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let result = (lhs as u16) - (rhs as u16);

    set_flag(cpu, FLAG_Z, result & 0xff == 0);
    set_flag(cpu, FLAG_N, true);
    set_flag(cpu, FLAG_H, (lhs & 0xf) < (rhs & 0xf));
    set_flag(cpu, FLAG_C, lhs < rhs);

    result as u8
}

fn sbc(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let carry_in = get_flag(cpu, FLAG_C);
    let result = (lhs as u16) - (rhs as u16) - (carry_in as u16);

    set_flag(cpu, FLAG_Z, result & 0xff == 0);
    set_flag(cpu, FLAG_N, true);
    set_flag(cpu, FLAG_H, ((lhs & 0xf) - (rhs & 0xf) - (carry_in as u8)) & 0xf + 1 != 0);
    set_flag(cpu, FLAG_C, (lhs as u16) < ((rhs as u16) + (carry_in as u16)));

    result as u8
}

fn and(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let result = lhs & rhs;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, true);
    set_flag(cpu, FLAG_C, false);

    result
}

fn xor(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let result = lhs ^ rhs;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, false);

    result
}

fn or(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    let result = lhs | rhs;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, false);

    result
}

fn cp(cpu: &mut Cpu, lhs: u8, rhs: u8) -> u8 {
    sub(cpu, lhs, rhs);
    lhs
}

fn alu_a_r8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let operation = ALU_OPS[get_y(cpu.ir) as usize];

    let lhs = unsafe { cpu.gr.sr.a };
    let rhs = get_r8(cpu, mem, get_z(cpu.ir));

    unsafe { cpu.gr.sr.a = operation(cpu, lhs, rhs) }
}

fn alu_a_n8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let operation = ALU_OPS[get_y(cpu.ir) as usize];

    let lhs = unsafe { cpu.gr.sr.a };
    let rhs = read_pc(cpu, mem);

    unsafe { cpu.gr.sr.a = operation(cpu, lhs, rhs) }
}

fn ret(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    cpu.pc = pop(cpu, mem);
    cycle(cpu, mem);
}

//noinspection SpellCheckingInspection
fn reti(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    cpu.ime = true;
    ret(cpu, mem);
}

fn ret_cc(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let condition = get_cc(cpu, get_y(cpu.ir));
    cycle(cpu, mem);

    if condition {
        ret(cpu, mem);
    }
}

fn pop_r16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = pop(cpu, mem);

    let select = get_p(cpu.ir);

    let rp = unsafe { &mut cpu.gr.rp };

    match select {
        0 => rp.bc = value,
        1 => rp.de = value,
        2 => rp.hl = value,
        3 => rp.af = value & 0xfff0,
        _ => unreachable!(),
    }
}

fn push_r16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let select = get_p(cpu.ir);

    let rp = unsafe { &cpu.gr.rp };

    let value = match select {
        0 => rp.bc,
        1 => rp.de,
        2 => rp.hl,
        3 => rp.af & 0xfff0,
        _ => unreachable!(),
    };

    cycle(cpu, mem);
    push(cpu, mem, value);
}

fn jp_n16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = read_n16_pc(cpu, mem);
    cpu.pc = value;
    cycle(cpu, mem);
}

fn jp_cc_n16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = read_n16_pc(cpu, mem);
    let condition = get_cc(cpu, get_y(cpu.ir));

    if condition {
        cpu.pc = value;
        cycle(cpu, mem);
    }
}

fn call_n16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = read_n16_pc(cpu, mem);
    cycle(cpu, mem);
    push(cpu, mem, cpu.pc);
    cpu.pc = value;
}

fn call_cc_n16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let value = read_n16_pc(cpu, mem);
    let condition = get_cc(cpu, get_y(cpu.ir));

    if condition {
        cycle(cpu, mem);
        push(cpu, mem, cpu.pc);
        cpu.pc = value;
    }
}

fn rst(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = get_y(cpu.ir) as u16 * 8;
    cycle(cpu, mem);
    push(cpu, mem, cpu.pc);
    cpu.pc = address;
}

static ROT_OPS: [fn(&mut Cpu, u8) -> u8; 8] = [rlc, rrc, rl, rr, sla, sra, swap, srl];

fn rlc(cpu: &mut Cpu, value: u8) -> u8 {
    let result = value.rotate_left(1);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 0x80 != 0);

    result
}

fn rrc(cpu: &mut Cpu, value: u8) -> u8 {
    let result = value.rotate_right(1);

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);

    result
}

fn rl(cpu: &mut Cpu, value: u8) -> u8 {
    let carry_in = get_flag(cpu, FLAG_C) as u8;
    let result = (value << 1) | carry_in;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 0x80 != 0);

    result
}

fn rr(cpu: &mut Cpu, value: u8) -> u8 {
    let carry_in = (get_flag(cpu, FLAG_C) as u8) << 7;
    let result = (value >> 1) | carry_in;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);

    result
}

fn sla(cpu: &mut Cpu, value: u8) -> u8 {
    let result = value << 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 0x80 != 0);

    result
}

fn sra(cpu: &mut Cpu, value: u8) -> u8 {
    let result = ((value as i8) >> 1) as u8;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);

    result
}

fn swap(cpu: &mut Cpu, value: u8) -> u8 {
    let low = value & 0xf;
    let high = (value >> 4) & 0xf;

    let result = (low << 4) | high;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N | FLAG_H | FLAG_C, false);

    result
}

fn srl(cpu: &mut Cpu, value: u8) -> u8 {
    let result = value >> 1;

    set_flag(cpu, FLAG_Z, result == 0);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, false);
    set_flag(cpu, FLAG_C, value & 1 != 0);

    result
}

fn bit(cpu: &mut Cpu, bit_select: u8, value: u8) {
    let result = value & (1 << bit_select) == 0;

    set_flag(cpu, FLAG_Z, result);
    set_flag(cpu, FLAG_N, false);
    set_flag(cpu, FLAG_H, true);
}

fn res(bit_select: u8, value: u8) -> u8 {
    value & !(1 << bit_select)
}

fn set(bit_select: u8, value: u8) -> u8 {
    value | (1 << bit_select)
}

fn prefix(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let opcode = read_pc(cpu, mem);

    let op_select = get_x(opcode);
    let y = get_y(opcode);
    let reg_select = get_z(opcode);

    let value = get_r8(cpu, mem, reg_select);

    match op_select {
        0 => {
            let op = ROT_OPS[y as usize];

            let result = op(cpu, value);

            set_r8(cpu, mem, reg_select, result);
        }
        1 => bit(cpu, y, value),
        2 => set_r8(cpu, mem, reg_select, res(y, value)),
        3 => set_r8(cpu, mem, reg_select, set(y, value)),
        _ => unreachable!(),
    }
}

fn ld_dn8_a(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = 0xff00 | (read_pc(cpu, mem) as u16);
    write(cpu, mem, address, unsafe { cpu.gr.sr.a });
}

fn ld_dc_a(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = 0xff00 | (unsafe { cpu.gr.sr.c } as u16);
    write(cpu, mem, address, unsafe { cpu.gr.sr.a });
}

fn ld_dn16_a(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = read_n16_pc(cpu, mem);
    write(cpu, mem, address, unsafe { cpu.gr.sr.a });
}

fn ld_a_dn8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = 0xff00 | (read_pc(cpu, mem) as u16);
    let value = read(cpu, mem, address);
    unsafe {
        cpu.gr.sr.a = value;
    }
}

fn ld_a_dc(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = 0xff00 | (unsafe { cpu.gr.sr.c } as u16);
    let value = read(cpu, mem, address);
    unsafe {
        cpu.gr.sr.a = value;
    }
}

fn ld_a_dn16(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let address = read_n16_pc(cpu, mem);
    let value = read(cpu, mem, address);
    unsafe {
        cpu.gr.sr.a = value;
    }
}

fn add_sp_o8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let offset = read_pc(cpu, mem) as i8 as i16 as u16;
    let base = cpu.sp;
    let result = base + offset;

    set_flag(cpu, FLAG_Z | FLAG_N, false);
    set_flag(cpu, FLAG_H, (base & 0xf) + (offset & 0xf) > 0xf);
    set_flag(cpu, FLAG_C, (base & 0xff) + (offset & 0xff) > 0xff);

    cpu.sp = result;
    cycle(cpu, mem);
}

fn di(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    cpu.ime = false;
    cpu.ime_next = false;
}

fn ei(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    cpu.ime_next = true;
}

fn jp_hl(cpu: &mut Cpu, _: &mut dyn MemoryBus) {
    cpu.pc = unsafe { cpu.gr.rp.hl };
}

fn ld_hl_sp_o8(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    let offset = read_pc(cpu, mem) as i8 as i16 as u16;
    let base = cpu.sp;
    let result = base + offset;

    set_flag(cpu, FLAG_Z | FLAG_N, false);
    set_flag(cpu, FLAG_H, (base & 0xf) + (offset & 0xf) > 0xf);
    set_flag(cpu, FLAG_C, (base & 0xff) + (offset & 0xff) > 0xff);

    unsafe { cpu.gr.rp.hl = result }
}

fn ld_sp_hl(cpu: &mut Cpu, mem: &mut dyn MemoryBus) {
    cpu.sp = unsafe { cpu.gr.rp.hl };
    cycle(cpu, mem);
}

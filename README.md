# nes-cpu
School project for course Modelling of Digital Circuits

The NES’s CPU is a Ricoh 2A03, which is based on the popular 8-bit MOS Technology 6502 and runs at 1.79 MHz. [1]

## Registers
- Accumulator (A)
- Indexes
    - X
    - Y
- Program Counter (PC)
- Stack Pointer (SP)
- Status Register (P)
[2]

## Addressing Modes
- Zero page indexed
- Absolute indexed
- Indexed indirect
- Indirect indexed

[4]

## CPU Memory Map
| Address range | Size  | Device |
|---------------|-------|--------|
| $0000–$07FF   | $0800 | 2 KB internal RAM |
| $0800–$0FFF   | $0800 | Mirrors of $0000–$07FF |
| $1000–$17FF   | $0800 | Mirrors of $0000–$07FF |
| $1800–$1FFF   | $0800 | Mirrors of $0000–$07FF |
| $2000–$2007   | $0008 | NES PPU registers |
| $2008–$3FFF   | $1FF8 | Mirrors of $2000–$2007 (repeats every 8 bytes) |
| $4000–$4017   | $0018 | NES APU and I/O registers |
| $4018–$401F   | $0008 | APU and I/O functionality that is normally disabled. |
| $4020–$FFFF   | $BFE0 | Unmapped. Available for cartridge use. |
| $6000–$7FFF   | $BFE0 | Usually cartridge RAM, when present. |
| $8000–$FFFF   | $8000 | Usually cartridge ROM and mapper registers. |
[3]

## Sources
[1] https://www.copetti.org/writings/consoles/nes/

[2] https://www.nesdev.org/wiki/CPU_registers

[3] https://www.nesdev.org/wiki/CPU_memory_map

[4] https://www.nesdev.org/wiki/CPU_addressing_modes


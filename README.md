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
    - N negative flag
    - V overflow
    - unused
    - B break
    - D decimal
    - I interrupt
    - Z zero
    - C carry

[2]

## Addressing Modes
- Zero page indexed
- Absolute indexed
- Indexed indirect
- Indirect indexed

[4]

## CPU Memory Map
| Address range | Size  | Device                                               |
|---------------|-------|------------------------------------------------------|
| $0000–$07FF   | $0800 | 2 KB internal RAM                                    |
| $0800–$0FFF   | $0800 | Mirrors of $0000–$07FF                               |
| $1000–$17FF   | $0800 | Mirrors of $0000–$07FF                               |
| $1800–$1FFF   | $0800 | Mirrors of $0000–$07FF                               |
| $2000–$2007   | $0008 | NES PPU registers                                    |
| $2008–$3FFF   | $1FF8 | Mirrors of $2000–$2007 (repeats every 8 bytes)       |
| $4000–$4017   | $0018 | NES APU and I/O registers                            |
| $4018–$401F   | $0008 | APU and I/O functionality that is normally disabled. |
| $4020–$FFFF   | $BFE0 | Unmapped. Available for cartridge use.               |
| $6000–$7FFF   | $BFE0 | Usually cartridge RAM, when present.                 |
| $8000–$FFFF   | $8000 | Usually cartridge ROM and mapper registers.          |

[3]

## Instruction Set
| Instruction | Description                      |
|-------------|----------------------------------|
| ADC         | add with carry                   |
| AND         | and (with accumulator)           |
| ASL         | arithmetic shift left            |
| BCC         | branch on carry clear            |
| BCS         | branch on carry set              |
| BEQ         | branch on equal (zero set)       |
| BIT         | bit test                         |
| BMI         | branch on minus (negative set)   |
| BNE         | branch on not equal (zero clear) |
| BPL         | branch on plus (negative clear)  |
| BRK         | break / interrupt                |
| BVC         | branch on overflow clear         |
| BVS         | branch on overflow set           |
| CLC         | clear carry                      |
| CLD         | clear decimal                    |
| CLI         | clear interrupt disable          |
| CLV         | clear overflow                   |
| CMP         | compare (with accumulator)       |
| CPX         | compare with X                   |
| CPY         | compare with Y                   |
| DEC         | decrement                        |
| DEX         | decrement X                      |
| DEY         | decrement Y                      |
| EOR         | exclusive or (with accumulator)  |
| INC         | increment                        |
| INX         | increment X                      |
| INY         | increment Y                      |
| JMP         | jump                             |
| JSR         | jump subroutine                  |
| LDA         | load accumulator                 |
| LDX         | load X                           |
| LDY         | load Y                           |
| LSR         | logical shift right              |
| NOP         | no operation                     |
| ORA         | or with accumulator              |
| PHA         | push accumulator                 |
| PHP         | push processor status (SR)       |
| PLA         | pull accumulator                 |
| PLP         | pull processor status (SR)       |
| ROL         | rotate left                      |
| ROR         | rotate right                     |
| RTI         | return from interrupt            |
| RTS         | return from subroutine           |
| SBC         | subtract with carry              |
| SEC         | set carry                        |
| SED         | set decimal                      |
| SEI         | set interrupt disable            |
| STA         | store accumulator                |
| STX         | store X                          |
| STY         | store Y                          |
| TAX         | transfer accumulator to X        |
| TAY         | transfer accumulator to Y        |
| TSX         | transfer stack pointer to X      |
| TXA         | transfer X to accumulator        |
| TXS         | transfer X to stack pointer      |
| TYA         | transfer Y to accumulator        |

[5]

## Sources
[1] https://www.copetti.org/writings/consoles/nes/

[2] https://www.nesdev.org/wiki/CPU_registers

[3] https://www.nesdev.org/wiki/CPU_memory_map

[4] https://www.nesdev.org/wiki/CPU_addressing_modes

[5] https://www.masswerk.at/6502/6502_instruction_set.html
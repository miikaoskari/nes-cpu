library ieee;
use ieee.std_logic_1164.all;

entity CPU is
    port (
        clk_in : in std_logic; -- 100MHz clock
        reset_in : in std_logic; -- reset
        ready_in : in std_logic; -- ready signal 
        --interrupt signals
        nnmi_in : in std_logic; -- nmi interrupt signal
        nres_in : in std_logic; -- res signal
        nirq_in : in std_logic; -- irq signal
        -- memory bus
        d_in : in std_logic_vector(7 downto 0); -- data bus input
        d_out : out std_logic_vector(7 downto 0); -- data bus output
        a_out : out std_logic_vector(15 downto 0); -- address bus output
        r_nw_out : out std_logic; -- read/write signal output
    );
end entity CPU;

-- time gen cycle states
constant T0 : std_logic_vector(2 downto 0) := "000";
constant T1 : std_logic_vector(2 downto 0) := "001";
constant T2 : std_logic_vector(2 downto 0) := "010";
constant T3 : std_logic_vector(2 downto 0) := "011";
constant T4 : std_logic_vector(2 downto 0) := "100";
constant T5 : std_logic_vector(2 downto 0) := "101";
constant T6 : std_logic_vector(2 downto 0) := "110";

-- interrupts
constant INT_RST : integer := 0;
constant INT_NMI : integer := 1;
constant INT_IRQ : integer := 2;
constant INT_NONE : integer := 3;

-- registers
signal q_ac, d_ac : std_logic_vector(7 downto 0); -- accumulator
signal q_x, d_x : std_logic_vector(7 downto 0); -- index register x
signal q_y, d_y : std_logic_vector(7 downto 0); -- index register y

-- processor status register
signal p : std_logic_vector(7 downto 0);
signal q_c, d_c : std_logic; -- carry
signal q_d, d_d : std_logic; -- decimal mode
signal q_i, d_i : std_logic; -- interrupt disable
signal q_n, d_n : std_logic; -- negative
signal q_v, d_v : std_logic; -- overflow
signal q_z, d_z : std_logic; -- zero

-- internal registers
signal q_abh, d_abh : std_logic_vector(7 downto 0); -- address bus high
signal q_abl, d_abl : std_logic_vector(7 downto 0); -- address bus low
signal q_acr : std_logic; -- internal carry latch
signal q_add, d_add : std_logic_vector(7 downto 0); -- internal adder
signal q_ai, d_ai : std_logic_vector(7 downto 0); -- alu a reg
signal q_bi, d_bi : std_logic_vector(7 downto 0); -- alu b reg
signal q_dl, d_dl : std_logic_vector(7 downto 0); -- data latch
signal q_ir, d_ir : std_logic_vector(7 downto 0); -- instruction register
signal q_pch, d_pch : std_logic_vector(7 downto 0); -- program counter high
signal q_pcl, d_pcl : std_logic_vector(7 downto 0); -- program counter low
signal q_pchs, d_pchs : std_logic_vector(7 downto 0); -- program counter high
signal q_pcls, d_pcls : std_logic_vector(7 downto 0); -- program counter low
signal q_pd, d_pd : std_logic_vector(7 downto 0); -- pre decode
signal q_s, d_s : std_logic_vector(7 downto 0); -- stack pointer
signal q_t, d_t : std_logic_vector(7 downto 0); -- timing cycle

-- buses
signal adl : std_logic_vector(7 downto 0); -- address bus low
signal adh_in, adh_out : std_logic_vector(7 downto 0); -- address bus high
signal db_in, db_out : std_logic_vector(7 downto 0); -- data bus
signal sb_in, sb_out : std_logic_vector(7 downto 0); -- stack bus

-- internal signals
-- adl bus
signal add_adl : std_logic; -- adder hold register
signal dl_adl : std_logic; -- data latch hold register
signal pcl_adl : std_logic; -- program counter low hold register
signal s_adl : std_logic; -- stack pointer hold register

-- adh bus
signal dl_adh : std_logic; -- data latch hold register
signal pch_adh : std_logic; -- program counter high hold register
signal zero_adh0 : std_logic; -- zero adh0
signal zero_adh17 : std_logic; -- zero adh1

-- db bus
signal ac_db : std_logic; -- accumulator data bus
signal dl_db : std_logic; -- data latch data bus
signal p_db : std_logic; -- processor status data bus
signal pch_db : std_logic; -- program counter high data bus
signal pcl_db : std_logic; -- program counter low data bus

-- sb bus
signal ac_sb : std_logic; -- accumulator stack bus
signal add_sb : std_logic; -- adder stack bus
signal x_sb : std_logic; -- index register x stack bus
signal y_sb : std_logic; -- index register y stack bus
signal s_sb : std_logic; -- stack pointer stack bus

-- mosfet
signal sb_adh : std_logic; -- stack bus address high
signal sb_db : std_logic; -- stack bus data bus

-- load control
signal adh_abh : std_logic; -- address bus high to address bus high
signal adl_abl : std_logic; -- address bus low to address bus low
signal sb_ac : std_logic; -- stack bus to accumulator
signal adl_add : std_logic; -- address bus low to adder
signal db_add : std_logic; -- data bus to adder
signal invdb_add : std_logic; -- inverted data bus to adder
signal sb_add : std_logic; -- stack bus to adder
signal zero_add : std_logic; -- zero to adder
signal adh_pch : std_logic; -- address bus high to program counter high
signal adl_pcl : std_logic; -- address bus low to program counter low
signal sb_s : std_logic; -- stack bus to stack pointer
signal sb_x : std_logic; -- stack bus to index register x
signal sb_y : std_logic; -- stack bus to index register y

-- cpu status
signal acr_c : std_logic; -- latch acr carry
signal db0_c : std_logic; -- data bus 0 carry
signal ir5_c : std_logic; -- instruction register 5 carry
signal db3_d : std_logic; -- data bus 3 decimal
signal ir5_d : std_logic; -- instruction register 5 decimal
signal db2_i : std_logic; -- data bus 2 interrupt
signal ir5_i : std_logic; -- instruction register 5 interrupt
signal db7_n : std_logic; -- data bus 7 negative
signal avr_v : std_logic; -- v register
signal db6_v : std_logic; -- data bus 6 overflow
signal zero_v : std_logic; -- zero to v register
signal db1_z : std_logic; -- data bus 1 zero
signal dbz_z : std_logic; -- data bus zero zero

-- pc
signal i_pc : std_logic; -- increment program counter

-- alu
signal ands : std_logic; -- and
signal eors : std_logic; -- exclusive or
signal ors : std_logic; -- or
signal sums : std_logic; -- sum
signal srs : std_logic; -- shift right
signal addc : std_logic; -- carry in
signal acr : std_logic; -- carry out
signal avr : std_logic; -- overflow out

-- ready control 
signal rdy : std_logic; -- ready signal
signal q_ready : std_logic; -- ready signal

process (clk_in, reset_in)
begin
    if reset_in = '1' then
        q_ready <= '0';
    elsif rising_edge(clk_in) then
        q_ready <= ready_in;
    end if;
end process;

rdy <= ready_in and q_ready;

-- clock phase gen
signal q_clk_phase, d_clk_phase : std_logic_vector(5 downto 0);

process (clk_in, reset_in)
begin
    if reset_in = '1' then
        q_clk_phase <= "000001";
    elsif rising_edge(clk_in) then
        if rdy = '1' then
            q_clk_phase <= d_clk_phase;
        end if;
    end if;
end process;

d_clk_phase <= (others => '0') when q_clk_phase = "111111" else std_logic_vector(unsigned(q_clk_phase) + 1);

-- interrupt and reset control
signal q_irq_sel, d_irq_sel : std_logic_vector(1 downto 0);
signal q_rst, d_rst : std_logic;
signal q_nres : std_logic;
signal q_nmi, d_nmi : std_logic;
signal q_nnmi : std_logic;
signal clear_rst : std_logic;
signal clear_nmi : std_logic;
signal force_noinc_pc : std_logic;

process (clk_in, reset_in)
begin
    if reset_in = '1' then
        q_irq_sel <= INTERRUPT_RST;
        q_rst <= '0';
        q_nres <= '1';
        q_nmi <= '0';
        q_nnmi <= '1';
    elsif rising_edge(clk_in) and q_clk_phase = "000000" then
        q_irq_sel <= d_irq_sel;
        q_rst <= d_rst;
        q_nres <= nres_in;
        q_nmi <= d_nmi;
        q_nnmi <= nnmi_in;
    end if;
end process;

d_rst <= '0' when clear_rst = '1' else
        '1' when nres_in = '0' and q_nres = '1' else
        q_rst;

d_nmi <= '0' when clear_nmi = '1' else
        '1' when nnmi_in = '0' and q_nnmi = '1' else
        q_nmi;

-- phase 1 clocked registers
process (clk_in, reset_in)
begin
    if reset_in = '1' then
        q_ac <= x"00";
        q_x <= x"00";
        q_y <= x"00";
        q_c <= '0';
        q_d <= '0';
        q_i <= '0';
        q_n <= '0';
        q_v <= '0';
        q_z <= '0';
        q_abh <= x"80";
        q_abl <= x"00";
        q_acr <= '0';
        q_ai <= x"00";
        q_bi <= x"00";
        q_dor <= x"00";
        q_ir <= NOP;
        q_pchs <= x"80";
        q_pcls <= x"00";
        q_s <= x"FF";
        q_t <= T1;
    elsif rising_edge(clk_in) and rdy = '1' and q_clk_phase = "000000" then
        q_ac   <= d_ac;
        q_x    <= d_x;
        q_y    <= d_y;
        q_c    <= d_c;
        q_d    <= d_d;
        q_i    <= d_i;
        q_n    <= d_n;
        q_v    <= d_v;
        q_z    <= d_z;
        q_abh  <= d_abh;
        q_abl  <= d_abl;
        q_acr  <= acr;
        q_ai   <= d_ai;
        q_bi   <= d_bi;
        q_dor  <= d_dor;
        q_ir   <= d_ir;
        q_pchs <= d_pchs;
        q_pcls <= d_pcls;
        q_s    <= d_s;
        q_t    <= d_t;
end process;

-- phase 2 clocked registers
process (clk_in, reset_in)
begin
    if reset_in = '1' then
        q_pcl <= x"00";
        q_pch <= x"80";
        q_dl <= x"00";
        q_pd <= x"00";
        q_add <= x"00";
    elsif rising_edge(clk_in) and rdy = '1' and q_clk_phase = "011100" then
        q_pcl <= d_pcl;
        q_pch <= d_pch;
        q_dl  <= d_dl;
        q_pd  <= d_pd;
        q_add <= d_add;
    end if;
end process;

-- time gen
process (clk_in, reset_in)
begin
    d_t            <= T0;
    d_irq_sel      <= q_irq_sel;
    force_noinc_pc <= '0';

    case q_t is
        when T0 =>
            d_t <= T1;
        when T1 =>
            if ((q_ir = CLC) or (q_ir = CLD)     or (q_ir = CLI)     or (q_ir = CLV)     or
                (q_ir = HLT) or (q_ir = LDA_IMM) or (q_ir = LDX_IMM) or (q_ir = LDY_IMM) or
                (q_ir = NOP) or (q_ir = SEC)     or (q_ir = SED)     or (q_ir = SEI)     or
                (q_ir = TAX) or (q_ir = TAY)     or (q_ir = TSX)     or (q_ir = TXA)     or
                (q_ir = TXS) or (q_ir = TYA)) then
                d_t <= T0;
            elsif (((q_ir = BCC) and q_c = '1') or ((q_ir = BCS) and q_c = '0') or
                ((q_ir = BPL) and q_n = '1') or ((q_ir = BMI) and q_n = '0') or
                ((q_ir = BVC) and q_v = '1') or ((q_ir = BVS) and q_v = '0') or
                ((q_ir = BNE) and q_z = '1') or ((q_ir = BEQ) and q_z = '0')) then
                d_t <= T0;
            else
                d_t <= T2;
            end if;
        when T2 =>
            if ((q_ir = ADC_IMM) or (q_ir = AND_IMM) or (q_ir = ASL_ACC) or (q_ir = CMP_IMM) or
                (q_ir = CPX_IMM) or (q_ir = CPY_IMM) or (q_ir = DEX)     or (q_ir = DEY)     or
                (q_ir = EOR_IMM) or (q_ir = INX)     or (q_ir = INY)     or (q_ir = LSR_ACC) or
                (q_ir = ORA_IMM) or (q_ir = ROL_ACC) or (q_ir = ROR_ACC) or (q_ir = SBC_IMM)) then
                d_t <= T1;
            elsif ((q_ir = JMP_ABS) or (q_ir = LDA_ZP) or (q_ir = LDX_ZP) or (q_ir = LDY_ZP) or
                (q_ir = SAX_ZP)  or (q_ir = STA_ZP) or (q_ir = STX_ZP) or (q_ir = STY_ZP)) then
                d_t <= T0;
            elsif (acr = '0' and ((q_ir = ADC_ABSX) or (q_ir = ADC_ABSY) or (q_ir = AND_ABSX) or
                (q_ir = AND_ABSY) or (q_ir = CMP_ABSX) or (q_ir = CMP_ABSY) or
                (q_ir = EOR_ABSX) or (q_ir = EOR_ABSY) or (q_ir = LDA_ABSX) or
                (q_ir = LDA_ABSY) or (q_ir = ORA_ABSX) or (q_ir = ORA_ABSY) or
                (q_ir = SBC_ABSX) or (q_ir = SBC_ABSY))) then
                d_t <= T4;
            elsif ((acr = q_ai(7)) and ((q_ir = BCC) or (q_ir = BCS) or (q_ir = BEQ) or
                (q_ir = BMI) or (q_ir = BNE) or (q_ir = BPL) or
                (q_ir = BVC) or (q_ir = BVS))) then
                d_t <= T0;
            else
                d_t <= T3;
            end if;
        when T3 =>
            if ((q_ir = ADC_ZP) or (q_ir = AND_ZP) or (q_ir = BIT_ZP) or (q_ir = CMP_ZP) or
                (q_ir = CPX_ZP) or (q_ir = CPY_ZP) or (q_ir = EOR_ZP) or (q_ir = ORA_ZP) or
                (q_ir = PHA)    or (q_ir = PHP)    or (q_ir = SBC_ZP)) then
                d_t <= T1;
            elsif ((q_ir = BCC)     or (q_ir = BCS)     or (q_ir = BEQ)     or
                (q_ir = BMI)     or (q_ir = BNE)     or (q_ir = BPL)     or
                (q_ir = BVC)     or (q_ir = BVS)     or (q_ir = LDA_ABS) or
                (q_ir = LDA_ZPX) or (q_ir = LDX_ABS) or (q_ir = LDX_ZPY) or
                (q_ir = LDY_ABS) or (q_ir = LDY_ZPX) or (q_ir = PLA)     or
                (q_ir = PLP)     or (q_ir = SAX_ABS) or (q_ir = SAX_ZPY) or
                (q_ir = STA_ABS) or (q_ir = STA_ZPX) or (q_ir = STX_ABS) or
                (q_ir = STX_ZPY) or (q_ir = STY_ABS) or (q_ir = STY_ZPX)) then
                d_t <= T0;
            elsif (acr = '0' and ((q_ir = ADC_INDY) or (q_ir = AND_INDY) or (q_ir = CMP_INDY) or
                (q_ir = EOR_INDY) or (q_ir = LDA_INDY) or
                (q_ir = ORA_INDY) or (q_ir = SBC_INDY))) then
                d_t <= T5;
            else
                d_t <= T4;
            end if;
        when T4 =>
            if ((q_ir = ADC_ABS) or (q_ir = ADC_ZPX) or (q_ir = AND_ABS) or (q_ir = AND_ZPX) or
                (q_ir = BIT_ABS) or (q_ir = CMP_ABS) or (q_ir = CMP_ZPX) or (q_ir = CPX_ABS) or
                (q_ir = CPY_ABS) or (q_ir = EOR_ABS) or (q_ir = EOR_ZPX) or (q_ir = ORA_ABS) or
                (q_ir = ORA_ZPX) or (q_ir = SBC_ABS) or (q_ir = SBC_ZPX)) then
                d_t <= T1;
            elsif ((q_ir = ASL_ZP)   or (q_ir = DEC_ZP)   or (q_ir = INC_ZP)   or
                (q_ir = JMP_IND)  or (q_ir = LDA_ABSX) or (q_ir = LDA_ABSY) or
                (q_ir = LDX_ABSY) or (q_ir = LDY_ABSX) or (q_ir = LSR_ZP)   or
                (q_ir = ROL_ZP)   or (q_ir = ROR_ZP)   or (q_ir = STA_ABSX) or
                (q_ir = STA_ABSY)) then
                d_t <= T0;
            else
                d_t <= T5;
            end if;
        when T5 =>
            if ((q_ir = ADC_ABSX) or (q_ir = ADC_ABSY) or (q_ir = AND_ABSX) or
                (q_ir = AND_ABSY) or (q_ir = CMP_ABSX) or (q_ir = CMP_ABSY) or
                (q_ir = EOR_ABSX) or (q_ir = EOR_ABSY) or (q_ir = ORA_ABSX) or
                (q_ir = ORA_ABSY) or (q_ir = SBC_ABSX) or (q_ir = SBC_ABSY)) then
                d_t <= T1;
            elsif ((q_ir = ASL_ABS)  or (q_ir = ASL_ZPX)  or (q_ir = DEC_ABS)  or
                (q_ir = DEC_ZPX)  or (q_ir = INC_ABS)  or (q_ir = INC_ZPX)  or
                (q_ir = JSR)      or (q_ir = LDA_INDX) or (q_ir = LDA_INDY) or
                (q_ir = LSR_ABS)  or (q_ir = LSR_ZPX)  or (q_ir = ROL_ABS)  or
                (q_ir = ROL_ZPX)  or (q_ir = ROR_ABS)  or (q_ir = ROR_ZPX)  or
                (q_ir = RTI)      or (q_ir = RTS)      or (q_ir = SAX_INDX) or
                (q_ir = STA_INDX) or (q_ir = STA_INDY)) then
                d_t <= T0;
            else
                d_t <= T6;
            end if;
        when T6 =>
            if ((q_ir = ADC_INDX) or (q_ir = ADC_INDY) or (q_ir = AND_INDX) or
                (q_ir = AND_INDY) or (q_ir = CMP_INDX) or (q_ir = CMP_INDY) or
                (q_ir = EOR_INDX) or (q_ir = EOR_INDY) or (q_ir = ORA_INDX) or
                (q_ir = ORA_INDY) or (q_ir = SBC_INDX) or (q_ir = SBC_INDY)) then
                d_t <= T1;
            else
                d_t <= T0;
        when d_t = T1 =>
            if (q_rst or q_nmi or not nirq_in)
                d_ir = BRK;
                force_noinc_pc = '1';
                if q_rst = '1'
                    d_irq_sel = INT_RST;
                elsif q_nmi = '1' then
                    d_irq_sel = INT_NMI;
                else
                    d_irq_sel = INT_IRQ;
                end if;
            else
                d_ir = q_pd;
                d_irq_sel = INT_BRK;
            end if;
        else
            d_ir = q_ir;
    end case;
end process;

-- decode rom output signals
-- pc and program stream controls
signal load_prg_byte : std_logic;
signal load_prg_byte_noinc : std_logic;
signal incpc_noload : std_logic;
signal alusum_to_pch : std_logic;
signal dl_to_pch : std_logic;
signal alusum_to_pcl : std_logic;
signal s_to_pcl : std_logic;

-- instruction controls
signal adc_op : std_logic;
signal and_op : std_logic;
signal asl_acc_op : std_logic;
signal asl_mem_op : std_logic;
signal bit_op : std_logic;
signal cmp_op : std_logic;
signal clc_op : std_logic;
signal cld_op : std_logic;
signal cli_op : std_logic;
signal clv_op : std_logic;
signal dec_op : std_logic;
signal dex_op : std_logic;
signal dey_op : std_logic;
signal eor_op : std_logic;
signal inc_op : std_logic;
signal inx_op : std_logic;
signal iny_op : std_logic;
signal lda_op : std_logic;
signal ldx_op : std_logic;
signal ldy_op : std_logic;
signal lsr_acc_op : std_logic;
signal lsr_mem_op : std_logic;
signal ora_op : std_logic;
signal rol_acc_op : std_logic;
signal rol_mem_op : std_logic;
signal ror_acc_op : std_logic;
signal ror_mem_op : std_logic;
signal sec_op : std_logic;
signal sed_op : std_logic;
signal sei_op : std_logic;
signal tax_op : std_logic;
signal tay_op : std_logic;
signal tsx_op : std_logic;
signal txa_op : std_logic;
signal txs_op : std_logic;
signal tya_op : std_logic;

-- data output register controls
signal ac_to_dor : std_logic;
signal p_to_dor : std_logic;
signal pch_to_dor : std_logic;
signal pcl_to_dor : std_logic;
signal x_to_dor : std_logic;
signal y_to_dor : std_logic;

-- address bus controls
signal aluinc_to_abh : std_logic;
signal alusum_to_abh : std_logic;
signal dl_to_abh : std_logic;
signal ff_to_abh : std_logic;
signal one_to_abh : std_logic;
signal zero_to_abh : std_logic;
signal aluinc_to_abl : std_logic;
signal dl_to_abl : std_logic;
signal fa_to_abl : std_logic;
signal fb_to_abl : std_logic;
signal fc_to_abl : std_logic;
signal fd_to_abl : std_logic;
signal fe_to_abl : std_logic;
signal ff_to_abl : std_logic;
signal s_to_abl : std_logic;

-- alu controls
signal ac_to_ai : std_logic;
signal dl_to_ai : std_logic;
signal one_to_ai : std_logic;
signal neg1_to_ai : std_logic;
signal s_to_ai : std_logic;
signal x_to_ai : std_logic;
signal y_to_ai : std_logic;
signal zero_to_ai : std_logic;
signal ac_to_bi : std_logic;
signal aluinc_to_bi : std_logic;
signal alusum_to_bi : std_logic;
signal dl_to_bi : std_logic;
signal invdl_to_bi : std_logic;
signal neg1_to_bi : std_logic;
signal pch_to_bi : std_logic;
signal pcl_to_bi : std_logic;
signal s_to_bi : std_logic;
signal x_to_bi : std_logic;
signal y_to_bi : std_logic;

-- stack controls
signal aluinc_to_s : std_logic;
signal alusum_to_s : std_logic;
signal dl_to_s : std_logic;

-- process status controls
signal dl_bits67_to_p : std_logic;
signal dl_to_p : std_logic;
signal one_to_s : std_logic;

-- set all decode rom output to value

-- decode rom logic
process(all)
begin
    -- set all decode rom output to value
    r_nw_out <= '1';
    brk_out <= '0';
    clear_rst <= '0';
    clear_nmi <= '0';

    if q_t = T0 then
        load_prg_byte <= '1';
    elsif q_t = T1 then
        case q_ir is
            when ADC_ABS | AND_ABS | ASL_ABS | BIT_ABS | CMP_ABS | CPX_ABS | CPY_ABS | DEC_ABS | EOR_ABS |
                 INC_ABS | JMP_ABS | JMP_IND | LDA_ABS | LDX_ABS | LDY_ABS | LSR_ABS |
                 ORA_ABS | ROL_ABS | ROR_ABS | SAX_ABS | SBC_ABS | 
                 STA_ABS | STX_ABS | STY_ABS =>
                load_prg_byte <= '1';
                zero_to_ai    <= '1';
                dl_to_bi      <= '1';
            when ADC_ABSX | AND_ABSX | ASL_ABSX |  CMP_ABSX |  DEC_ABSX |  EOR_ABSX | INC_ABSX |
                 LDA_ABSX | LDY_ABSX |  LSR_ABSX |  ORA_ABSX |  ROL_ABSX |
                 ROR_ABSX | SBC_ABSX |  STA_ABSX =>
                load_prg_byte <= '1';
                x_to_ai       <= '1';
                dl_to_bi      <= '1';
            when ADC_ABSY | AND_ABSY | CMP_ABSY | EOR_ABSY | LDA_ABSY | LDX_ABSY |
                 ORA_ABSY | SBC_ABSY | STA_ABSY =>
                load_prg_byte <= '1';
                y_to_ai       <= '1';
                dl_to_bi      <= '1';
            when ADC_IMM | AND_IMM | EOR_IMM | ORA_IMM =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                dl_to_bi      <= '1';
            when ADC_INDX| AND_INDX | CMP_INDX | EOR_INDX | LDA_INDX | ORA_INDX |
                SAX_INDX | SBC_INDX | STA_INDX |
                ADC_ZPX |  AND_ZPX |  ASL_ZPX |  CMP_ZPX |  DEC_ZPX |
                EOR_ZPX |  INC_ZPX |  LDA_ZPX |  LDY_ZPX |
                LSR_ZPX |  ORA_ZPX |  ROL_ZPX |  ROR_ZPX |  SBC_ZPX |  
                STA_ZPX |  STY_ZPX =>
                x_to_ai <= '1';
                dl_to_bi <= '1';
            when ADC_INDY | AND_INDY | CMP_INDY | EOR_INDY | LDA_INDY | ORA_INDY |
                SBC_INDY | STA_INDY =>
                zero_to_abh <= '1';
                dl_to_abh <= '1';
                zero_to_ai <= '1';
                dl_to_bi <= '1';
            when ADC_ZP | AND_ZP |  ASL_ZP | BIT_ZP | CMP_ZP | CPX_ZP | CPY_ZP | DEC_ZP |
                EOR_ZP | INC_ZP |  LDA_ZP | LDX_ZP | LDY_ZP | LSR_ZP | ORA_ZP |
                ROL_ZP | ROR_ZP | SBC_ZP =>
                zero_to_abh <= '1';
                dl_to_abh <= '1';
            when ASL_ACC | LSR_ACC | ROL_ACC | ROR_ACC =>
                ac_to_ai <= '1';
                ac_to_bi <= '1';
            when BCC | BCS | BEQ | BMI | BNE | BPL | BVC | BVS =>
                load_prg_byte <= '1';
                dl_to_ai <= '1';
                pcl_to_ai <= '1';
            when BRK =>
                if (q_irl_sel = INT_BRK) then
                    incpc_noload <= '1';
                pch_to_dor <= '1';
                one_to_abh <= '1';
                s_to_abl <= '1';
                neg1_to_ai <= '1';
                s_to_bi <= '1';
            when CLC =>
                clc_op <= '1';
            when CLD =>
                cld_op <= '1';
            when CLI =>
                cli_op <= '1';
            when CLV =>
                clv_op <= '1';
            when CMP_IMM | SBC_IMM =>
                load_prg_byte <= '1';
                ac_to_ai <= '1';
                invdl_to_bi <= '1';
            when CPX_IMM =>
                load_prg_byte <= '1';
                x_to_ai <= '1';
                invdl_to_bi <= '1';
            when CPY_IMM =>
                load_prg_byte <= '1';
                y_to_ai <= '1';
                invdl_to_bi <= '1';
            when DEX =>
                x_to_ai <= '1';
                neg1_to_bi <= '1';
            when DEY =>
                y_to_ai <= '1';
                neg1_to_bi <= '1';
            when HLT =>
                brk_out <= '1' when q_clk_phase = "000001" and rdy = '1' else '0';
            when INX =>
                zero_to_ai <= '1';
                x_to_bi <= '1';
            when INY =>
                zero_to_ai <= '1';
                y_to_bi <= '1';
            when JSR =>
                incpc_noload <= '1';
                one_to_abh <= '1';
                s_to_abl <= '1';
                s_to_bi <= '1';
                dl_to_s <= '1';
            when LDX_ZPY | SAX_ZPY | STX_ZPY =>
                y_to_ai <= '1';
                dl_to_bi <= '1';
            when LDA_IMM =>
                load_prg_byte <= '1';
                lda_op <= '1';
            when LDX_IMM =>
                load_prg_byte <= '1';
                ldx_op <= '1';
            when LDY_IMM =>
                load_prg_byte <= '1';
                ldy_op <= '1';
            when PHA =>
                ac_to_dor <= '1';
                one_to_abh <= '1';
                s_to_abl <= '1';
            when PHP =>
                p_to_dor <= '1';
                one_to_abh <= '1';
                s_to_abl <= '1';
            when PLA | PLP | RTI | RTS =>
                zero_to_ai <= '1';
                s_to_bi <= '1';
            when SEC =>
                sec_op <= '1';
            when SED =>
                sed_op <= '1';
            when SEI =>
                sei_op <= '1';
            when SAX_ZP =>
                ac_to_dor <= '1';
                x_to_dor <= '1';
                zero_to_abh <= '1';
                dl_to_abl <= '1';
            when STA_ZP =>
                ac_to_dor <= '1';
                zero_to_abh <= '1';
                dl_to_abl <= '1';
            when STX_ZP =>
                x_to_dor <= '1';
                zero_to_abh <= '1';
                dl_to_abl <= '1';
            when STY_ZP =>
                y_to_dor <= '1';
                zero_to_abh <= '1';
                dl_to_abl <= '1';
            when TAX =>
                tax_op <= '1';
            when TAY =>
                tay_op <= '1';
            when TSX =>
                tsx_op <= '1';
            when TXA =>
                txa_op <= '1';
            when TXS =>
                txs_op <= '1';
            when TYA =>
                tya_op <= '1';
            when others =>
                null; -- do nothing
        end case;
    elsif q_t = T2 then
        case q_ir is
            when ADC_ABS | AND_ABS | ASL_ABS | BIT_ABS | CMP_ABS | CPX_ABS | CPY_ABS | DEC_ABS | EOR_ABS |
                INC_ABS | LDA_ABS | LDX_ABS | LDY_ABS | LSR_ABS | ORA_ABS | 
                ROL_ABS | ROR_ABS | SBC_ABS |
                JMP_IND =>
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when ADC_ABSX | AND_ABSX | ASL_ABSX | CMP_ABSX | DEC_ABSX | EOR_ABSX | INC_ABSX |
                LDA_ABSX | LDY_ABSX | LSR_ABSX | ORA_ABSX | ROL_ABSX |
                ROR_ABSX | SBC_ABSX | STA_ABSX |
                ADC_ABSY | AND_ABSY | CMP_ABSY | EOR_ABSY | LDA_ABSY |
                LDX_ABSY | ORA_ABSY | SBC_ABSY |  
                STA_ABSY =>
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
                    zero_to_ai <= '1';
                    dl_to_bi <= '1';
            when ADC_IMM | SBC_IMM =>
                    load_prg_byte <= '1';
                    adc_op <= '1';
            when ADC_INDX | AND_INDX | CMP_INDX | EOR_INDX | LDA_INDX | ORA_INDX |
                SAX_INDX | SBC_INDX | STA_INDX |
                ADC_ZPX | AND_ZPX | ASL_ZPX | CMP_ZPX | DEC_ZPX |
                EOR_ZPX | INC_ZPX | LDA_ZPX | LDY_ZPX |
                LSR_ZPX | ORA_ZPX | ROL_ZPX | ROR_ZPX | SBC_ZPX |  
                LDX_ZPY =>
                    zero_to_abh <= '1';
                    alusum_to_abl <= '1';
            when ADC_INDY | AND_INDY | CMP_INDY | EOR_INDY | LDA_INDY | ORA_INDY |
                SBC_INDY | STA_INDY =>
                    zero_to_abh <= '1';
                    aluinc_to_abl <= '1';
                    y_to_ai <= '1';
                    dl_to_bi <= '1';
            when ADC_ZP | AND_ZP | EOR_ZP | ORA_ZP =>
                    load_prg_byte <= '1';
                    ac_to_ai <= '1';
                    dl_to_bi <= '1';
            when AND_IMM =>
                    load_prg_byte <= '1';
                    and_op <= '1';
            when ASL_ACC =>
                    load_prg_byte <= '1';
                    asl_acc_op <= '1';
            when ASL_ZP | LSR_ZP | ROL_ZP | ROR_ZP =>
                    dl_to_ai <= '1';
                    dl_to_bi <= '1';
            when LSR_ACC =>
                    load_prg_byte <= '1';
                    lsr_acc_op <= '1';
            when BCC | BCS | BEQ | BMI | BNE | BPL | BVC | BVS =>
                    alusum_to_pcl <= '1';
                    alusum_to_abl <= '1';
                    if q_ai(7) = '1' then
                        neg1_to_ai <= '1';
                    else
                        one_to_ai <= '1';
                    end if;
                    pch_to_bi <= '1';
            when BIT_ZP =>
                    load_prg_byte <= '1';
                    ac_to_ai <= '1';
                    dl_to_bi <= '1';
                    dl_bits67_to_p <= '1';
            when BRK =>
                    pcl_to_dor <= '1';
                    alusum_to_abl <= '1';
                    alusum_to_bi <= '1';
                    r_nw_out <= '0';
            when CMP_IMM | CPX_IMM | CPY_IMM =>
                    load_prg_byte <= '1';
                    cmp_op <= '1';
            when CMP_ZP | SBC_ZP =>
                    load_prg_byte <= '1';
                    ac_to_ai <= '1';
                    invdl_to_bi <= '1';
            when CPX_ZP =>
                    load_prg_byte <= '1';
                    x_to_ai <= '1';
                    invdl_to_bi <= '1';
            when CPY_ZP =>
                    load_prg_byte <= '1';
                    y_to_ai <= '1';
                    invdl_to_bi <= '1';
            when DEC_ZP =>
                    neg1_to_ai <= '1';
                    dl_to_bi <= '1';
            when DEX =>
                    load_prg_byte <= '1';
                    dex_op <= '1';
            when DEY =>
                    load_prg_byte <= '1';
                    dey_op <= '1';
            when EOR_IMM =>
                    load_prg_byte <= '1';
                    eor_op <= '1';
            when INC_ZP =>
                    zero_to_ai <= '1';
                    dl_to_bi <= '1';
            when INX =>
                    load_prg_byte <= '1';
                    inx_op <= '1';
            when INY =>
                    load_prg_byte <= '1';
                    iny_op <= '1';
            when JMP_ABS =>
                    dl_to_pch <= '1';
                    alusum_to_pcl <= '1';
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when JSR =>
                    pch_to_dor <= '1';
                    neg1_to_ai <= '1';
            when LDA_ZP =>
                    load_prg_byte <= '1';
                    lda_op <= '1';
            when LDX_ZP =>
                    load_prg_byte <= '1';
                    ldx_op <= '1';
            when LDY_ZP =>
                    load_prg_byte <= '1';
                    ldy_op <= '1';
            when ORA_IMM =>
                    load_prg_byte <= '1';
                    ora_op <= '1';
            when PHA | PHP =>
                    load_prg_byte_noinc <= '1';
                    s_to_ai <= '1';
                    neg1_to_bi <= '1';
                    r_nw_out <= '0';
            when PLA | PLP =>
                    one_to_abh <= '1';
                    aluinc_to_abl <= '1';
                    aluinc_to_s <= '1';
            when ROL_ACC =>
                    load_prg_byte <= '1';
                    rol_acc_op <= '1';
            when ROR_ACC =>
                    load_prg_byte <= '1';
                    ror_acc_op <= '1';
            when RTI | RTS =>
                    one_to_abh <= '1';
                    aluinc_to_abl <= '1';
                    aluinc_to_bi <= '1';
            when SAX_ABS =>
                    ac_to_dor <= '1';
                    x_to_dor <= '1';
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when SAX_ZP | STA_ZP | STX_ZP | STY_ZP =>
                    load_prg_byte <= '1';
                    r_nw_out <= '0';
            when SAX_ZPY =>
                    ac_to_dor <= '1';
                    x_to_dor <= '1';
                    zero_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STA_ABS =>
                    ac_to_dor <= '1';
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STA_ZPX =>
                    ac_to_dor <= '1';
                    zero_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STX_ABS =>
                    x_to_dor <= '1';
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STX_ZPY =>
                    x_to_dor <= '1';
                    zero_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STY_ABS =>
                    y_to_dor <= '1';
                    dl_to_abh <= '1';
                    alusum_to_abl <= '1';
            when STY_ZPX =>
                    y_to_dor <= '1';
                    zero_to_abh <= '1';
                    alusum_to_abl <= '1';
        end case;
    elsif q_t = T3 then
        case q_ir is
            when ADC_ABS | AND_ABS | EOR_ABS | ORA_ABS |
                ADC_ZPX | AND_ZPX | EOR_ZPX | ORA_ZPX =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                dl_to_bi      <= '1';
            when ADC_ABSX | AND_ABSX | ASL_ABSX | CMP_ABSX | DEC_ABSX | EOR_ABSX | INC_ABSX |
                LDA_ABSX | LDY_ABSX | LSR_ABSX | ORA_ABSX | ROL_ABSX |
                ROR_ABSX | SBC_ABSX |
                ADC_ABSY | AND_ABSY | CMP_ABSY | EOR_ABSY | LDA_ABSY |
                LDX_ABSY | ORA_ABSY | SBC_ABSY =>
                aluinc_to_abh <= q_acr;
            when ADC_INDX | AND_INDX | CMP_INDX | EOR_INDX | LDA_INDX | ORA_INDX |
                SAX_INDX | STA_INDX | SBC_INDX =>
                zero_to_abh   <= '1';
                aluinc_to_abl <= '1';
                zero_to_ai    <= '1';
                dl_to_bi      <= '1';
            when ADC_INDY | AND_INDY | CMP_INDY | EOR_INDY | LDA_INDY | ORA_INDY |
                SBC_INDY | STA_INDY =>
                dl_to_abh     <= '1';
                alusum_to_abl <= '1';
                zero_to_ai    <= '1';
                dl_to_bi      <= '1';
            when ADC_ZP | SBC_ZP =>
                load_prg_byte <= '1';
                adc_op        <= '1';
            when AND_ZP =>
                load_prg_byte <= '1';
                and_op        <= '1';
            when ASL_ABS | LSR_ABS | ROL_ABS | ROR_ABS |
                ASL_ZPX | LSR_ZPX | ROL_ZPX | ROR_ZPX =>
                dl_to_ai <= '1';
                dl_to_bi <= '1';
            when ASL_ZP =>
                asl_mem_op <= '1';
            when BCC | BCS | BEQ | BMI | BNE | BPL | BVC | BVS =>
                alusum_to_pch <= '1';
                alusum_to_abh <= '1';
            when BIT_ABS =>
                load_prg_byte  <= '1';
                ac_to_ai       <= '1';
                dl_to_bi       <= '1';
                dl_bits67_to_p <= '1';
            when BIT_ZP =>
                load_prg_byte <= '1';
                bit_op        <= '1';
            when BRK =>
                p_to_dor      <= '1';
                alusum_to_abl <= '1';
                alusum_to_bi  <= '1';
                r_nw_out      <= '0';
            when CMP_ABS | SBC_ABS |
                CMP_ZPX | SBC_ZPX =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                invdl_to_bi   <= '1';
            when CMP_ZP | CPX_ZP | CPY_ZP =>
                load_prg_byte <= '1';
                cmp_op        <= '1';
            when CPX_ABS =>
                load_prg_byte <= '1';
                x_to_ai       <= '1';
                invdl_to_bi   <= '1';
            when CPY_ABS =>
                load_prg_byte <= '1';
                y_to_ai       <= '1';
                invdl_to_bi   <= '1';
            when DEC_ABS |
                DEC_ZPX =>
                neg1_to_ai <= '1';
                dl_to_bi   <= '1';
            when DEC_ZP =>
                dec_op <= '1';
            when EOR_ZP =>
                load_prg_byte <= '1';
                eor_op        <= '1';
            when INC_ABS |
                INC_ZPX =>
                zero_to_ai <= '1';
                dl_to_bi   <= '1';
            when INC_ZP =>
                inc_op <= '1';
            when JMP_IND =>
                aluinc_to_abl <= '1';
                zero_to_ai    <= '1';
                dl_to_bi      <= '1';
            when JSR =>
                pcl_to_dor    <= '1';
                alusum_to_abl <= '1';
                alusum_to_bi  <= '1';
                r_nw_out      <= '0';
            when LDA_ABS | LDA_ZPX =>
                load_prg_byte <= '1';
                lda_op        <= '1';
            when LDX_ABS | LDX_ZPY =>
                load_prg_byte <= '1';
                ldx_op        <= '1';
            when LDY_ABS | LDY_ZPX =>
                load_prg_byte <= '1';
                ldy_op        <= '1';
            when LSR_ZP =>
                lsr_mem_op <= '1';
            when ORA_ZP =>
                load_prg_byte <= '1';
                ora_op        <= '1';
            when PHA | PHP =>
                load_prg_byte <= '1';
                alusum_to_s   <= '1';
            when PLA =>
                load_prg_byte_noinc <= '1';
                lda_op              <= '1';
            when PLP =>
                load_prg_byte_noinc <= '1';
                dl_to_p             <= '1';
            when ROL_ZP =>
                rol_mem_op <= '1';
            when ROR_ZP =>
                ror_mem_op <= '1';
            when RTI =>
                aluinc_to_abl <= '1';
                aluinc_to_bi  <= '1';
                dl_to_p       <= '1';
            when RTS =>
                aluinc_to_abl <= '1';
                dl_to_s       <= '1';
            when SAX_ABS | STA_ABS | STX_ABS | STY_ABS |
                STA_ZPX | STY_ZPX |
                SAX_ZPY | STX_ZPY =>
                load_prg_byte <= '1';
                r_nw_out      <= '0';
            when STA_ABSX |
                STA_ABSY =>
                ac_to_dor     <= '1';
                aluinc_to_abh <= q_acr;
            when others =>
                -- Handle the default case here
        end case;
    elsif q_t = T4 then
        case q_ir is
            when ADC_ABS | SBC_ABS | ADC_ZPX | SBC_ZPX =>
                load_prg_byte <= '1';
                adc_op        <= '1';
            when ADC_ABSX | AND_ABSX | EOR_ABSX | ORA_ABSX | ADC_ABSY | AND_ABSY | EOR_ABSY | ORA_ABSY =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                dl_to_bi      <= '1';
            when ADC_INDX | AND_INDX | CMP_INDX | EOR_INDX | LDA_INDX | ORA_INDX | SBC_INDX =>
                dl_to_abh     <= '1';
                alusum_to_abl <= '1';
            when ADC_INDY | AND_INDY | CMP_INDY | EOR_INDY | LDA_INDY | ORA_INDY | SBC_INDY =>
                aluinc_to_abh <= q_acr;
            when AND_ABS | AND_ZPX =>
                load_prg_byte <= '1';
                and_op        <= '1';
            when ASL_ABS | ASL_ZPX =>
                asl_mem_op <= '1';
            when ASL_ZP | DEC_ZP | INC_ZP | LSR_ZP | ROL_ZP | ROR_ZP | STA_ABSX | STA_ABSY =>
                load_prg_byte <= '1';
                r_nw_out      <= '0';
            when ASL_ABSX | LSR_ABSX | ROL_ABSX | ROR_ABSX =>
                dl_to_ai <= '1';
                dl_to_bi <= '1';
            when BIT_ABS =>
                load_prg_byte <= '1';
                bit_op        <= '1';
            when BRK =>
                ff_to_abh <= '1';
                r_nw_out  <= '0';
                one_to_i  <= '1';
                case q_irq_sel is
                    when INTERRUPT_RST =>
                        fc_to_abl <= '1';
                    when INTERRUPT_NMI =>
                        fa_to_abl <= '1';
                    when INTERRUPT_IRQ | INTERRUPT_BRK =>
                        fe_to_abl <= '1';
                    when others =>
                        -- Handle the default case here
                end case;
            when CMP_ABS | CPX_ABS | CPY_ABS | CMP_ZPX =>
                load_prg_byte <= '1';
                cmp_op        <= '1';
            when CMP_ABSX | SBC_ABSX | CMP_ABSY | SBC_ABSY =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                invdl_to_bi   <= '1';
            when DEC_ABS | DEC_ZPX =>
                dec_op <= '1';
            when DEC_ABSX =>
                neg1_to_ai <= '1';
                dl_to_bi   <= '1';
            when EOR_ABS | EOR_ZPX =>
                load_prg_byte <= '1';
                eor_op        <= '1';
            when INC_ABS | INC_ZPX =>
                inc_op <= '1';
            when INC_ABSX =>
                zero_to_ai <= '1';
                dl_to_bi   <= '1';
            when JMP_IND =>
                dl_to_pch     <= '1';
                alusum_to_pcl <= '1';
                dl_to_abh     <= '1';
                alusum_to_abl <= '1';
            when JSR =>
                load_prg_byte_noinc <= '1';
                r_nw_out            <= '0';
            when LDA_ABSX | LDA_ABSY =>
                load_prg_byte <= '1';
                lda_op        <= '1';
            when LDX_ABSY =>
                load_prg_byte <= '1';
                ldx_op        <= '1';
            when LDY_ABSX =>
                load_prg_byte <= '1';
                ldy_op        <= '1';
            when LSR_ABS | LSR_ZPX =>
                lsr_mem_op <= '1';
            when ORA_ABS | ORA_ZPX =>
                load_prg_byte <= '1';
                ora_op        <= '1';
            when ROL_ABS | ROL_ZPX =>
                rol_mem_op <= '1';
            when ROR_ABS | ROR_ZPX =>
                ror_mem_op <= '1';
            when RTI =>
                aluinc_to_abl <= '1';
                dl_to_s       <= '1';
            when RTS =>
                dl_to_pch   <= '1';
                s_to_pcl    <= '1';
                aluinc_to_s <= '1';
            when SAX_INDX =>
                ac_to_dor     <= '1';
                x_to_dor      <= '1';
                dl_to_abh     <= '1';
                alusum_to_abl <= '1';
            when STA_INDX =>
                ac_to_dor     <= '1';
                dl_to_abh     <= '1';
                alusum_to_abl <= '1';
            when STA_INDY =>
                ac_to_dor     <= '1';
                aluinc_to_abh <= q_acr;
            when others =>
                -- Handle the default case here
        end case;
    elsif q_t = T5 then
        case q_ir is
            when ADC_ABSX | SBC_ABSX | ADC_ABSY | SBC_ABSY =>
                load_prg_byte <= '1';
                adc_op        <= '1';
            when ADC_INDX | AND_INDX | EOR_INDX | ORA_INDX | ADC_INDY | AND_INDY | EOR_INDY | ORA_INDY =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                dl_to_bi      <= '1';
            when AND_ABSX | AND_ABSY =>
                load_prg_byte <= '1';
                and_op        <= '1';
            when ASL_ABS | DEC_ABS | INC_ABS | LSR_ABS | ROL_ABS | ROR_ABS | ASL_ZPX | DEC_ZPX | INC_ZPX | LSR_ZPX | ROL_ZPX | ROR_ZPX | SAX_INDX | STA_INDX | STA_INDY =>
                load_prg_byte <= '1';
                r_nw_out      <= '0';
            when ASL_ABSX =>
                asl_mem_op <= '1';
            when BRK =>
                ff_to_abh <= '1';
                dl_to_s   <= '1';
                case q_irq_sel is
                    when INTERRUPT_RST =>
                        fd_to_abl <= '1';
                    when INTERRUPT_NMI =>
                        fb_to_abl <= '1';
                    when INTERRUPT_IRQ | INTERRUPT_BRK =>
                        ff_to_abl <= '1';
                    when others =>
                        -- Handle the default case here
                end case;
            when CMP_ABSX | CMP_ABSY =>
                load_prg_byte <= '1';
                cmp_op        <= '1';
            when CMP_INDX | SBC_INDX | CMP_INDY | SBC_INDY =>
                load_prg_byte <= '1';
                ac_to_ai      <= '1';
                invdl_to_bi   <= '1';
            when DEC_ABSX =>
                dec_op <= '1';
            when EOR_ABSX | EOR_ABSY =>
                load_prg_byte <= '1';
                eor_op        <= '1';
            when INC_ABSX =>
                inc_op <= '1';
            when JSR =>
                dl_to_pch    <= '1';
                s_to_pcl     <= '1';
                dl_to_abh    <= '1';
                s_to_abl     <= '1';
                alusum_to_s  <= '1';
            when LDA_INDX | LDA_INDY =>
                load_prg_byte <= '1';
                lda_op        <= '1';
            when LSR_ABSX =>
                lsr_mem_op <= '1';
            when ORA_ABSX | ORA_ABSY =>
                load_prg_byte <= '1';
                ora_op        <= '1';
            when ROL_ABSX =>
                rol_mem_op <= '1';
            when ROR_ABSX =>
                ror_mem_op <= '1';
            when RTI =>
                dl_to_pch   <= '1';
                s_to_pcl    <= '1';
                dl_to_abh   <= '1';
                s_to_abl    <= '1';
                aluinc_to_s <= '1';
            when RTS =>
                load_prg_byte <= '1';
            when others =>
                -- Handle the default case here
        end case;
    elsif q_t = T6 then
        case q_ir is
            when ADC_INDX | SBC_INDX | ADC_INDY | SBC_INDY =>
                load_prg_byte <= '1';
                adc_op        <= '1';
            when AND_INDX | AND_INDY =>
                load_prg_byte <= '1';
                and_op        <= '1';
            when ASL_ABSX | DEC_ABSX | INC_ABSX | LSR_ABSX | ROL_ABSX | ROR_ABSX =>
                load_prg_byte  <= '1';
                r_nw_out       <= '0';
            when BRK =>
                dl_to_pch   <= '1';
                s_to_pcl    <= '1';
                dl_to_abh   <= '1';
                s_to_abl    <= '1';
                alusum_to_s <= '1';
                case q_irq_sel is
                    when INTERRUPT_RST =>
                        clear_rst <= '1';
                    when INTERRUPT_NMI =>
                        clear_nmi <= '1';
                    when others =>
                        -- Handle the default case here
                end case;
            when CMP_INDX | CMP_INDY =>
                load_prg_byte <= '1';
                cmp_op        <= '1';
            when EOR_INDX | EOR_INDY =>
                load_prg_byte <= '1';
                eor_op        <= '1';
            when ORA_INDX | ORA_INDY =>
                load_prg_byte <= '1';
                ora_op        <= '1';
            when others =>
                -- Handle the default case here
        end case;
    end if;
end process;

-- alu
process(all)
begin
    acr <= '0';
    avr <= '0';
    if ands = '1' then
        d_add <= q_ai and q_bi;
    elsif eors = '1' then
        d_add <= q_ai xor q_bi;
    elsif ors = '1' then
        d_add <= q_ai or q_bi;
    elsif sums = '1' then
        d_add <= std_logic_vector(unsigned(q_ai) + unsigned(q_bi) + unsigned(addc));
        acr <= d_add(7) xor q_ai(7) xor q_bi(7);
        avr <= acr;
    elsif srs = '1' then
        d_add <= std_logic_vector(unsigned(addc) sll 7) or (q_bi(7 downto 1) & '0');
        acr <= q_bi(0);
    else
        d_add <= q_add;
    end if;
end process;

-- random control logic
add_adl <= aluinc_to_abl or aluinc_to_bi or alusum_to_abl or alusum_to_bi or alusum_to_pcl;
dl_adl <= dl_to_abl;
pcl_adl <= load_prg_byte or load_prg_byte_noinc or pcl_to_bi;
s_adl <= s_to_abl or s_to_bi or s_to_pcl;
zero_adl0 <= fa_to_abl or fc_to_abl or fe_to_abl;
zero_adl1 <= fc_to_abl or fd_to_abl;
zero_adl2 <= fa_to_abl or fb_to_abl;
dl_adh <= dl_to_abh or dl_to_pch;
pch_adh <= load_prg_byte or load_prg_byte_noinc;
zero_adh0 <= zero_to_abh;
zero_adh17 <= one_to_abh or one_to_ai or zero_to_abh;
ac_db <= ac_to_bi or ac_to_dor;
dl_db <= dl_to_ai or dl_to_bi or dl_to_p or dl_to_s or invdl_to_bi or lda_op or ldx_op or ldy_op;
p_db <= p_to_dor;
pch_db <= pch_to_bi or pch_to_dor;
pcl_db <= pcl_to_dor;
ac_sb <= ac_to_ai or tax_op or tay_op;
add_sb <= adc_op or aluinc_to_abh or aluinc_to_s or alusum_to_abh or alusum_to_pch or alusum_to_s or and_op or asl_acc_op or asl_mem_op or bit_op or cmp_op or dec_op or dex_op or dey_op or eor_op or inc_op or inx_op or iny_op or lsr_acc_op or lsr_mem_op or ora_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op;
x_sb <= txa_op or txs_op or x_to_ai or x_to_bi or x_to_dor;
y_sb <= tya_op or y_to_ai or y_to_bi or y_to_dor;
s_sb <= s_to_ai or tsx_op;
sb_adh <= aluinc_to_abh or alusum_to_abh or alusum_to_pch or one_to_ai or one_to_i;
sb_db <= adc_op or and_op or asl_acc_op or asl_mem_op or bit_op or cmp_op or dl_to_s or dec_op or dex_op or dey_op or dl_to_ai or eor_op or inc_op or inx_op or iny_op or lda_op or ldx_op or ldy_op or lsr_acc_op or lsr_mem_op or one_to_i or ora_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op or tax_op or tay_op or tsx_op or txa_op or tya_op or x_to_bi or x_to_dor or y_to_bi or y_to_dor;
adh_abh <= aluinc_to_abh or alusum_to_abh or dl_to_abh or ff_to_abh or load_prg_byte or load_prg_byte_noinc or one_to_abh or zero_to_abh;
adl_abl <= aluinc_to_abl or alusum_to_abl or dl_to_abl or fa_to_abl or fb_to_abl or fc_to_abl or fd_to_abl or fe_to_abl or ff_to_abl or load_prg_byte or load_prg_byte_noinc or s_to_abl;
adl_add <= aluinc_to_bi or alusum_to_bi or pcl_to_bi or s_to_bi;
db_add <= ac_to_bi or dl_to_bi or neg1_to_bi or pch_to_bi or x_to_bi or y_to_bi;
invdb_add <= invdl_to_bi;
sb_s <= aluinc_to_s or alusum_to_s or dl_to_s or txs_op;
zero_add <= zero_to_ai;
sb_ac <= adc_op or and_op or asl_acc_op or eor_op or lda_op or lsr_acc_op or ora_op or rol_acc_op or ror_acc_op or txa_op or tya_op;
sb_add <= ac_to_ai or dl_to_ai or neg1_to_ai or one_to_ai or s_to_ai or x_to_ai or y_to_ai;
adh_pch <= alusum_to_pch or dl_to_pch;
adl_pcl <= alusum_to_pcl or s_to_pcl;
sb_x <= dex_op or inx_op or ldx_op or tax_op or tsx_op;
sb_y <= dey_op or iny_op or ldy_op or tay_op;
acr_c <= adc_op or asl_acc_op or asl_mem_op or cmp_op or lsr_acc_op or lsr_mem_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op;
db0_c <= dl_to_p;
ir5_c <= clc_op or sec_op;
db3_d <= dl_to_p;
ir5_d <= cld_op or sed_op;
db2_i <= dl_to_p or one_to_i;
ir5_i <= cli_op or sei_op;
db7_n <= adc_op or and_op or asl_acc_op or asl_mem_op or cmp_op or dec_op or dex_op or dey_op or dl_bits67_to_p or dl_to_p or eor_op or inc_op or inx_op or iny_op or lda_op or ldx_op or ldy_op or lsr_acc_op or lsr_mem_op or ora_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op or tax_op or tay_op or tsx_op or txa_op or tya_op;
avr_v <= adc_op;
db6_v <= dl_bits67_to_p or dl_to_p;
zero_v <= clv_op;
db1_z <= dl_to_p;
dbz_z <= adc_op or and_op or asl_acc_op or asl_mem_op or bit_op or cmp_op or dec_op or dex_op or dey_op or eor_op or inc_op or inx_op or iny_op or lda_op or ldx_op or ldy_op or lsr_acc_op or lsr_mem_op or ora_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op or tax_op or tay_op or tsx_op or txa_op or tya_op;
ands <= and_op or bit_op;
eors <= eor_op;
ors <= ora_op;
sums <= adc_op or aluinc_to_abh or aluinc_to_abl or aluinc_to_bi or aluinc_to_s or alusum_to_abh or alusum_to_abl or alusum_to_bi or alusum_to_pch or alusum_to_pcl or alusum_to_s or asl_acc_op or asl_mem_op or cmp_op or dec_op or dex_op or dey_op or inc_op or inx_op or iny_op or rol_acc_op or rol_mem_op;
srs <= lsr_acc_op or lsr_mem_op or ror_acc_op or ror_mem_op;

if (adc_op or rol_acc_op or rol_mem_op or ror_acc_op or ror_mem_op) = '1' then
    addc <= q_c;
else
    addc <= aluinc_to_abh or aluinc_to_abl or aluinc_to_bi or aluinc_to_s or cmp_op or inc_op or inx_op or iny_op;
end if;

i_pc <= (incpc_noload or load_prg_byte) and not force_noinc_pc;

-- update internal buses
adh_in(7 downto 1) <= q_dl(7 downto 1) when dl_adh = '1' else
                      q_pch(7 downto 1) when pch_adh = '1' else
                      "0000000" when zero_adh17 = '1' else
                      "1111111";

adh_in(0) <= q_dl(0) when dl_adh = '1' else
             q_pch(0) when pch_adh = '1' else
             '0' when zero_adh0 = '1' else
             '1';

adl(7 downto 3) <= q_add(7 downto 3) when add_adl = '1' else
                   q_dl(7 downto 3) when dl_adl = '1' else
                   q_pcl(7 downto 3) when pcl_adl = '1' else
                   q_s(7 downto 3) when s_adl = '1' else
                   "11111";

adl(2) <= q_add(2) when add_adl = '1' else
          q_dl(2) when dl_adl = '1' else
          q_pcl(2) when pcl_adl = '1' else
          q_s(2) when s_adl = '1' else
          '0' when zero_adl2 = '1' else
          '1';

adl(1) <= q_add(1) when add_adl = '1' else
          q_dl(1) when dl_adl = '1' else
          q_pcl(1) when pcl_adl = '1' else
          q_s(1) when s_adl = '1' else
          '0' when zero_adl1 = '1' else
          '1';

adl(0) <= q_add(0) when add_adl = '1' else
          q_dl(0) when dl_adl = '1' else
          q_pcl(0) when pcl_adl = '1' else
          q_s(0) when s_adl = '1' else
          '0' when zero_adl0 = '1' else
          '1';

db_in <= "11111111" and ((not ac_db and q_ac) or
                         (not dl_db and q_dl) or
                         (not p_db and p) or
                         (not pch_db and q_pch) or
                         (not pcl_db and q_pcl));

sb_in <= "11111111" and ((not ac_sb and q_ac) or
                         (not add_sb and q_add) or
                         (not s_sb and q_s) or
                         (not x_sb and q_x) or
                         (not y_sb and q_y));

adh_out <= (adh_in and sb_in and db_in) when (sb_adh and sb_db) = '1' else
           (adh_in and sb_in) when sb_adh = '1' else
           adh_in;

db_out <= (db_in and sb_in and adh_in) when (sb_db and sb_adh) = '1' else
          (db_in and sb_in) when sb_db = '1' else
          db_in;

sb_out <= (sb_in and db_in and adh_in) when (sb_adh and sb_db) = '1' else
          (sb_in and db_in) when sb_db = '1' else
          (sb_in and adh_in) when sb_adh = '1' else
          sb_in;

-- assign next ff states
d_ac <= sb_out when sb_ac = '1' else q_ac;
d_x <= sb_out when sb_x = '1' else q_x;
d_y <= sb_out when sb_y = '1' else q_y;
d_c <= acr when acr_c = '1' else
       db_out(0) when db0_c = '1' else
       q_ir(5) when ir5_c = '1' else q_c;
d_d <= db_out(3) when db3_d = '1' else
       q_ir(5) when ir5_d = '1' else q_d;
d_i <= db_out(2) when db2_i = '1' else
       q_ir(5) when ir5_i = '1' else q_i;
d_n <= db_out(7) when db7_n = '1' else q_n;
d_v <= avr when avr_v = '1' else
       db_out(6) when db6_v = '1' else
       '0' when zero_v = '1' else q_v;
d_z <= db_out(1) when db1_z = '1' else
       not or_reduce(db_out) when dbz_z = '1' else q_z;
d_abh <= adh_out when adh_abh = '1' else q_abh;
d_abl <= adl when adl_abl = '1' else q_abl;
d_ai <= sb_out when sb_add = '1' else
        "00000000" when zero_add = '1' else q_ai;
d_bi <= adl when adl_add = '1' else
        db_out when db_add = '1' else
        not db_out when invdb_add = '1' else q_bi;
d_dl <= d_in when r_nw_out = '1' else q_dl;
d_dor <= db_out;
d_pd <= d_in when r_nw_out = '1' else q_pd;
d_s <= sb_out when sb_s = '1' else q_s;

d_pchs <= adh_out when adh_pch = '1' else q_pch;
d_pcls <= adl when adl_pcl = '1' else q_pcl;
d_pch <= std_logic_vector(unsigned(q_pchs & q_pcls) + 1) when i_pc = '1' else q_pchs;
d_pcl <= std_logic_vector(unsigned(q_pchs & q_pcls) + 1) when i_pc = '1' else q_pcls;

-- combine full processor status register
p <= q_n & q_v & '1' & (q_irq_sel = INTERRUPT_BRK) & q_d & q_i & q_z & q_c;

-- assign output signals
d_out <= q_dor;
a_out <= q_abh & q_abl;

process(dbgreg_sel_in)
begin
  case dbgreg_sel_in is
    when REGSEL_AC =>
      dbgreg_out <= q_ac;
    when REGSEL_X =>
      dbgreg_out <= q_x;
    when REGSEL_Y =>
      dbgreg_out <= q_y;
    when REGSEL_P =>
      dbgreg_out <= p;
    when REGSEL_PCH =>
      dbgreg_out <= q_pch;
    when REGSEL_PCL =>
      dbgreg_out <= q_pcl;
    when REGSEL_S =>
      dbgreg_out <= q_s;
    when others =>
      dbgreg_out <= (others => 'X');
  end case;
end process;

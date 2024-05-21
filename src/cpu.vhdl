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

-- alu

-- random control logic

-- update internal buses

-- assign next ff states

-- assign output signals
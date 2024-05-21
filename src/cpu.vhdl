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

library ieee;
use ieee.std_logic_1164.all;

entity CPU is
    port (
        clk : in std_logic;
        reset : in std_logic;
    );
end entity CPU;

architecture Behavioral of CPU is
    component ALU is
        port (
            A : in std_logic_vector(7 downto 0); -- 8-bit operands
            B : in std_logic_vector(7 downto 0); -- 8-bit operands
            opcode : in std_logic_vector(7 downto 0); -- 8-bit opcode
            result : out std_logic_vector(7 downto 0); -- 8-bit result
            zero : out std_logic; -- Z flag
            carry : out std_logic; -- C flag
            negative : out std_logic; -- V flag
            overflow : out std_logic -- N flag
        );
    end component ALU;

    component Bus is
        port (
            clk : in std_logic;
            reset : in std_logic;
            data : in std_logic_vector(7 downto 0);
            address : in std_logic_vector(7 downto 0);
            read : in std_logic;
            write : in std_logic;
            data_out : out std_logic_vector(7 downto 0)
        );
    end component Bus;

    component Control_Unit is
        port (
            clk : in std_logic; -- clock
            reset : in std_logic; -- reset
            opcode : in std_logic_vector(7 downto 0); -- 8-bit opcode
            control_signals : out std_logic_vector(5 downto 0) -- 6-bit control signals
        );
    end component Control_Unit;

    component Decoder is
        port (
            opcode : in std_logic_vector(7 downto 0);
            control : out std_logic_vector(15 downto 0)
        );
    end component Decoder;

    component Memory is
        port (
            clk : in std_logic;
            address : in std_logic_vector(15 downto 0);
            data_in : in std_logic_vector(7 downto 0);
            write_enable : in std_logic;
            data_out : out std_logic_vector(7 downto 0)
        );
    end component Memory;

    component Registers is
        port (
            clk : in std_logic;
            reset : in std_logic;
            -- ports for each register
            A : out std_logic_vector(7 downto 0); -- accumulator
            X : out std_logic_vector(7 downto 0); -- index
            Y : out std_logic_vector(7 downto 0); -- index
            SP : out std_logic_vector(7 downto 0); -- stack pointer
            PC : out std_logic_vector(15 downto 0); -- program counter
            P : out std_logic_vector(7 downto 0) -- processor status
        );
    end component Registers;

    -- internal signals
    signal A, B, result : std_logic_vector(7 downto 0);
    signal opcode : std_logic_vector(7 downto 0);
    signal zero, carry, negative, overflow : std_logic;
    signal control_signals : std_logic_vector(15 downto 0);
    signal address : std_logic_vector(15 downto 0);
    signal data_in, data_out : std_logic_vector(7 downto 0);
    signal read, write : std_logic;
    signal PC, SP, X, Y, P : std_logic_vector(15 downto 0);

begin
    -- alu
    ALU_1: ALU port map (
        A => A,
        B => B,
        opcode => opcode,
        result => result,
        zero => zero,
        carry => carry,
        negative => negative,
        overflow => overflow
    );

    -- bus
    Bus_1: Bus port map (
        clk => clk,
        reset => reset,
        data => data,
        address => address,
        read => read,
        write => write,
        data_out => data_out
    );

    -- control unit
    Control_Unit_1: Control_Unit port map (
        clk => clk,
        reset => reset,
        opcode => opcode,
        control_signals => control_signals
    );

    -- decoder
    Decoder_1: Decoder port map (
        opcode => opcode,
        control => control
    );

    -- memory
    Memory_1: Memory port map (
        clk => clk,
        address => address,
        data_in => data_in,
        write_enable => write_enable,
        data_out => data_out
    );

    -- registers
    Registers_1: Registers port map (
        clk => clk,
        reset => reset,
        A => A,
        X => X,
        Y => Y,
        SP => SP,
        PC => PC,
        P => P
    );

    process(clk, reset)
    begin
        if reset = '1' then
            -- reset logic here
        elsif rising_edge(clk) then
            -- fetch-decode-execute cycle
            -- fetch instruction
            opcode <= data_out; -- assume data_out holds the opcode after memory fetch
            -- decode instruction
            -- control signals are generated by control unit based on the opcode
            -- execute instruction
            -- use ALU, registers, etc., to perform the operation
        end if;
    end process;
end architecture Behavioral;

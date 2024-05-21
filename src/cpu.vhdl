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

architecture Behavioral of CPU is
    component ALU is
        port (
            A : in std_logic_vector(7 downto 0); -- 8-bit operands
            B : in std_logic_vector(7 downto 0); -- 8-bit operands
            opcode : in std_logic_vector(7 downto 0); -- 8-bit opcode
            result : out std_logic_vector(7 downto 0); -- 8-bit result
            zero : out std_logic; -- Z flag
            carry : out std_logic; -- C flag
            negative : out std_logic; -- N flag
            overflow : out std_logic -- V flag
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
            -- Reset logic here
            PC <= (others => '0'); -- Reset program counter
            opcode <= (others => '0'); -- Reset opcode
            control_signals <= (others => '0'); -- Reset control signals
            read_enable <= '0';
            write_enable <= '0';
        elsif rising_edge(clk) then
            -- Fetch instruction
            address <= PC; -- Set address to program counter
            read_enable <= '1'; -- Enable reading from memory
            wait until memory_ready = '1'; -- Wait until memory is ready
            opcode <= data_out; -- Read opcode from memory
            read_enable <= '0'; -- Disable reading from memory
            PC <= PC + 1; -- Increment program counter

            -- Decode instruction
            control_signals <= control_signals; -- Control signals are generated by decoder based on the opcode

            -- Execute instruction
            if control_signals(0) = '1' then
                -- Example control signal handling for ALU operation
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
                A <= result; -- Example result handling
            end if;
        end if;
    end process;
end architecture Behavioral;

entity Registers is
    port (
        clk : in std_logic;
        reset : in std_logic;
        -- ports for each register
        A : out std_logic_vector(7 downto 0); -- accumulator
        X : out std_logic_vector(7 downto 0); -- index
        Y : out std_logic_vector(7 downto 0); -- index
        SP : out std_logic_vector(7 downto 0); -- stack pointer
        PC : out std_logic_vector(15 downto 0); -- program counter
        P : out std_logic_vector(7 downto 0); -- processor status
    );
end entity Registers;

architecture Behavioral of Registers is
    signal regA : std_logic_vector(7 downto 0) := (others => '0');
    signal regX : std_logic_vector(7 downto 0) := (others => '0');
    signal regY : std_logic_vector(7 downto 0) := (others => '0');
    signal regSP : std_logic_vector(7 downto 0) := (others => '1');
    signal regPC : std_logic_vector(15 downto 0) := (others => '0');
    signal regP : std_logic_vector(7 downto 0) := (others => '0');
begin
    A <= regA;
    X <= regX;
    Y <= regY;
    SP <= regSP;
    PC <= regPC;
    P <= regP;
end Behavioral;

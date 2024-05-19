library ieee;
use ieee.std_logic_1164.all;

entity Control_Unit is
    port (
        clk : in std_logic; -- clock
        reset : in std_logic; -- reset
        opcode : in std_logic_vector(7 downto 0); -- 8-bit opcode
        control_signals : out std_logic_vector(5 downto 0) -- 6-bit control signals
    );
end entity Control_Unit;

architecture Behavioral of Control_Unit is
begin
    process(clk, reset)
    begin

end Behavioral;

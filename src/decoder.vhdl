library ieee;
use ieee.std_logic_1164.all;

entity Decoder is
    Port (
        clk : in std_logic;
        reset : in std_logic;
        opcode : in std_logic_vector(7 downto 0);
        control : out std_logic_vector(15 downto 0)
    );
end entity Decoder;

architecture Behavioral of Decoder is
begin
    process(opcode)
    begin
        case opcode is
            when x"69" | x"65" | x"75" | x"6D" | x"7D" | x"79" | x"61" | x"71" -- adc
                control
                
            when others => -- TODO: handle other cases
                control <= (others => '0');
        end case;
    end process;
end Behavioral;

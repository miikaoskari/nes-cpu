library ieee;
use ieee.std_logic_1164.all;

entity Decoder is
    Port (
        opcode : in std_logic_vector(7 downto 0);
        control : out std_logic_vector(15 downto 0)
    );
end entity Decoder;

architecture Behavioral of Decoder is
begin
    process(opcode)
    begin
        case opcode is
            when "00000001" =>
                control <= "0000000000000001";
            when others => -- TODO: handle other cases
                control <= (others => '0');
        end case;
    end process;
end Behavioral;

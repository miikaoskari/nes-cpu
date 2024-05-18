library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity ALU is
    Port(
        A : in std_logic_vector(7 downto 0);
        B : in std_logic_vector(7 downto 0);
        opcode : in std_logic_vector(3 downto 0);
        result : out std_logic_vector(7 downto 0);
        flags : out std_logic_vector(7 downto 0)
    );
end ALU;

architecture Behavioral of ALU is
begin
    process(A, B, opcode)
    begin
        case opcode is
            
            when others => -- invalid opcode
                result <= (others => 'X');
        end case;
    end process;
end Behavioral;

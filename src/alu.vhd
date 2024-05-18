library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ALU is
    Port(
        A : in std_logic_vector(7 downto 0); -- 8-bit operands
        B : in std_logic_vector(7 downto 0); -- 8-bit operands
        opcode : in std_logic_vector(7 downto 0); -- 8-bit opcode
        result : out std_logic_vector(7 downto 0); -- 8-bit result
        zero : out std_logic; -- Z flag
        carry : out std_logic; -- C flag
        negative : out std_logic; -- V flag
        overflow : out std_logic -- N flag
    );
end ALU;

architecture Behavioral of ALU is
begin
    process(A, B, opcode)
    begin
        case opcode is
            when x"69" | x"65" | x"75" | x"6D" | x"7D" | x"79" | x"61" | x"71" => -- adc
                temp := unsigned('0' & A) + unsigned('0' & B) + unsigned(carry);
                result <= std_logic_vector(temp(7 downto 0));
                carry <= temp(8);
                zero <= '1' when temp = 0 else '0';
                negative <= temp(7);
                overflow <= A(7) and B(7) and not temp(7) or not A(7) and not B(7) and temp(7);
            when others => -- invalid opcode
                result <= (others => 'X');
        end case;
    end process;
end Behavioral;

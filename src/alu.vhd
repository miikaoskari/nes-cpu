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
    signal temp : unsigned(8 downto 0);
    signal unsigned_A : unsigned(7 downto 0);
    signal unsigned_B : unsigned(7 downto 0);
    signal unsigned_carry : unsigned(7 downto 0);
begin
    process(A, B, opcode)
    begin
        unsigned_A <= unsigned(A);
        unsigned_B <= unsigned(B);
        -- TODO: fix carry
        unsigned_carry <= unresolved_unsigned("00000000" & std_logic_vector'(carry));
        case opcode is
            -- arithmetic operations
            when x"69" | x"65" | x"75" | x"6D" | x"7D" | x"79" | x"61" | x"71" => -- adc (adc with carry)
                temp <= unsigned_A + unsigned_B + unsigned_carry;
                result <= std_logic_vector(temp(7 downto 0));
                carry <= temp(8);
                zero <= '1' when temp(7 downto 0) = "00000000" else '0';
                negative <= temp(7);
                overflow <= (A(7) and B(7) and not temp(7)) or (not A(7) and not B(7) and temp(7));
            when x"E9" | x"E5" | x"F5" | x"ED" | x"FD" | x"F9" | x"E1" | x"F1" => -- sbc (subtract with carry)
                temp <= unsigned_A - unsigned_B - not unsigned_carry & "00000000";
                result <= std_logic_vector(temp(7 downto 0));
                carry <= not temp(8);
                zero <= '1' when temp(7 downto 0) = "00000000" else '0'; -- Added condition
                negative <= temp(7);
                overflow <= (A(7) and not B(7) and not temp(7)) or (not A(7) and B(7) and temp(7));
            -- logical operations
            when x"29" | x"25" | x"35" | x"2D" | x"3D" | x"39" | x"21" | x"31" => -- and (bitwise and with accumulator)
                result <= A and B;
                carry <= '0';
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= '0';
            when x"09" | x"05" | x"15" | x"0D" | x"1D" | x"19" | x"01" | x"11" => -- ora (bitwise or with accumulator)
                result <= A or B;
                carry <= '0';
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= '0';
            when x"49" | x"45" | x"55" | x"4D" | x"5D" | x"59" | x"41" | x"51" => -- eor (bitwise exclusive or with accumulator)
                result <= A xor B;
                carry <= '0';
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= '0';
            -- bit shifting
            when x"0A" | x"06" | x"16" | x"0E" | x"1E" => -- asl (arithmetic shift left)
                result <= std_logic_vector(unsigned(A) sll 1);
                carry <= A(7);
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= A(7) xor result(7);
            when x"4A" | x"46" | x"56" | x"4E" | x"5E" => -- lsr (logical shift right)
                result <= std_logic_vector(unsigned(A) srl 1);
                carry <= A(0);
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= '0';
                overflow <= A(0) xor result(7);
            -- bit rotation
            when x"2A" | x"26" | x"36" | x"2E" | x"3E" => -- rol (rotate left)
                result <= std_logic_vector(unsigned(A) sll 1) or carry;
                carry <= A(7);
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= A(7) xor result(7);
            when x"6A" | x"66" | x"76" | x"6E" | x"7E" => -- ror (rotate right)
                result <= std_logic_vector(unsigned(A) srl 1) or (carry & A(7));
                carry <= A(0);
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= A(0) xor result(7);
            -- bit testing
            when x"24" | x"2C" => -- bit (test bits)
                result <= A and B;
                carry <= '0';
                zero <= '1' when unsigned(result) = 0 else '0';
                negative <= result(7);
                overflow <= B(6);    
            when others => -- invalid opcode
                result <= (others => 'X');
        end case;
    end process;
end Behavioral;

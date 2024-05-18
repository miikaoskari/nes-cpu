entity Instruction_Register is
    Port (
        opcode : in std_logic_vector(7 downto 0);
        control : out std_logic_vector(15 downto 0);
    );
end entity Instruction_Register;

architecture Behavioral of Instruction_Register is
begin

end Behavioral;

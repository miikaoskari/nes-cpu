library ieee;
use ieee.std_logic_1164.all;

entity Memory is
    port (
       clk : in std_logic;
       address : in std_logic_vector(15 downto 0);
       data_in : in std_logic_vector(7 downto 0);
       write_enable : in std_logic;
       data_out : out std_logic_vector(7 downto 0)
    );
end entity Memory;

architecture Behavioral of Memory is
    type memory_array is array (0 to 65535) of std_logic_vector(7 downto 0);
    signal memory : memory_array;
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if write_enable = '1' then
                memory(to_integer(unsigned(address))) <= data_in;
            end if;
            data_out <= memory(to_integer(unsigned(address)));
        end if;
    end process;
end Behavioral;

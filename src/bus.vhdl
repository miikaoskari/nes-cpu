library ieee;
use ieee.std_logic_1164.all;

entity Bus is
    port (
        clk : in std_logic;
        reset : in std_logic;
        data : in std_logic_vector(7 downto 0);
        address : in std_logic_vector(7 downto 0);
        read : in std_logic;
        write : in std_logic;
        data_out : out std_logic_vector(7 downto 0)
    );
end entity Bus;

architecture Behavioral of Bus is
    type memory is array (0 to 255) of std_logic_vector(7 downto 0);
    signal mem : memory;
begin
    process(clk, reset)
    begin
        if reset = '1' then
            mem <= (others => (others => '0'));
        elsif rising_edge(clk) then
            if write = '1' then
                mem(to_integer(unsigned(address))) <= data;
            elsif read = '1' then
                data_out <= mem(to_integer(unsigned(address)));
            end if;
        end if;
    end process;
end Behavioral;

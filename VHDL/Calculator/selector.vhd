-- Selector
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- This entity decides whether the result of the input should be shown at the display

entity selector is
	port (rst_sel, clk_sel, button_pressed, enable_selector	:	in  std_logic;
			input_alu														: 	in  std_logic_vector(15 downto 0); 	-- Result from ALU
			input_keypad													:	in  std_logic_vector(3 downto 0); 	-- Keypad input
			output															:	out std_logic_vector(15 downto 0)); -- To display
end selector;


architecture Behavioral of selector is
signal temp_output	: std_logic_vector(15 downto 0);
signal shift_reg		: std_logic_vector(15 downto 0);

begin
process (clk_sel, rst_sel, button_pressed, input_keypad)
begin 
	if (rst_sel = '0') then
		shift_reg <= (others => '0');
		
	elsif rising_edge(clk_sel) then
		if (button_pressed = '1') then
			-- Save the keypad input in 16 bit register before sending to display
			if (input_keypad /= "1010" and input_keypad /= "1011" and input_keypad /= "1100" and input_keypad /= "1111") then
				shift_reg(7 downto 4) <= shift_reg(3 downto 0);
				shift_reg(11 downto 8) <= shift_reg(7 downto 4);
				shift_reg(15 downto 12) <= shift_reg(11 downto 8);
				shift_reg(3 downto 0) <= input_keypad;			
			else
				shift_reg <= (others => '0');
			end if;
		end if;
	end if;
end process;


process(rst_sel, input_alu, input_keypad, enable_selector)
begin
	if (rst_sel = '0') then
		temp_output <= (others => '0');
	else
		-- When operator is pressed
		if (enable_selector = '1') then
			temp_output <= input_alu;
		-- when character is pressed
		else 
			temp_output <= shift_reg;
		end if;
	end if;
end process;

output <= temp_output;
end Behavioral; 




	
			
			
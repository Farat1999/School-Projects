-- Controller
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity Controller is
	port(
	rst, clk_controller, pressed_button					: in  std_logic;
	inBCD 														: in  std_logic_vector(3 downto 0);
	x_alu															: in  std_logic_vector(15 downto 0);
	a																: out std_logic_vector(15 downto 0);
	b																: out std_logic_vector(15 downto 0);
	enable_ctrl1, enable_ctrl2								: out std_logic;
	outBCD														: out std_logic_vector(3 downto 0));
end Controller;


architecture Behavioral of Controller is
type state_type is (x, y, z); -- 3 states are needed to perform addition and substraction

signal tempBCD 	: std_logic_vector(3 downto 0) 	:= (others => '0');
signal tempX 		: std_logic_vector(15 downto 0)	:= (others => '0');
signal tempY		: std_logic_vector(15 downto 0)	:= (others => '0');
signal enable1		: std_logic 							:= '0';
signal enable2		: std_logic 							:= '1';

begin 
P0: process (clk_controller, rst, next_state)
begin
	if (rst = '0') then
		current_state <= x;
		
	elsif rising_edge(clk_controller) then
		current_state <= next_state;
	end if;
end process;


---------------------------------------------------------------------------------------------------
-- The aim of this process is to arrange jumps between the different states depending on which
-- button has been pressed. tempX and tempY are two registers for value 1 respective value 2 that
-- will be sent to alu, where the calculations occur.
---------------------------------------------------------------------------------------------------
	
P1: process (rst, clk_controller, current_state, pressed_button, inBCD, tempX, tempY)
begin
	if (rst = '0') then
		tempX <= (others => '0');
		tempY <= (others => '0');
		enable1 <= '0';
		enable2 <= '1';
		next_state <= x;
	elsif rising_edge(clk_controller) then
		case current_state is
			when x =>
				if (button_pressed = '1') then
					if (inBCD = "1010") then -- Addition
						enable1 <= '1';
						enable2 <= '0';
						next_state <= z; 
					elsif (inBCD = "1011") then -- Substraction
						enable1 <= '1';
						enable2 <= '0';
						next_state <= z;
					elsif (inBCD = "1100") then	-- clear
						tempX <= (others => '0'); 
						tempY <= (others => '0');
						enable1 <= '0';
						enabel2 <= '1';
						next_state <= x;
					elsif (inBCD = "1111") then
						next_state <= x;
					elsif (inBCD /= "1111") then
						next_state <= x;
						tempY <= (others => '0');
						enable1 <= '0';
						enabel2 <= '1';
						
						tempX(3 downto 0) <= inBCD;
						tempX(7 downto 4) <= tempX(3 downto 0);
						tempX(11 downto 8) <= tempX(7 downto 4);
						tempX(15 downto 12) <= tempX(11 downto 8);
					end if;
				end if;
			
		when y => 
			tempX <= x_alu;
			if (pressed_button = '1') then
				if (inBCD = "1010") then		-- Addition
					next_state <= z;
					enable1 <= '1';
				elsif (inBCD = "1011") then	-- Substraction
					next_state <= z;
					enable1 <= '1'; 
				elsif (inBCD = "1100") then 	-- Clear
					tempY <= (others => '0');
					tempX <= (others => '0');
					enable1 <= '0';
					next_state <= x;
				elsif (inBCD = "1111") then
					next_state <= y;
				elsif (inBCD /= "1111") then
					next_state <= y;
					enable1 <= '0';
					
					tempY(3 downto 0) <= inBCD;
					tempY(7 downto 4) <= tempY(3 downto 0);
					tempY(11 downto 8) <= tempY(7 downto 4);
					tempY(15 downto 12) <= tempY(11 downto 8);			
				end if;
			end if;
		
		when z =>
			tempY <= (others => '0');
			next_state <= y;
			
		end case;
	end if;
end process;
			
tempBCD <= inBCD;
outBCD <= tempBCD;
a <= tempX;
b <= tempY;
enable_ctrl1 <= enable1;
enable_ctrl2 <= enable2;

end Behavioral;			
			
			
			
			
			
			
			
			
			
			
										
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_bit_sync is
	generic (
		BAUD_HZ : positive := 10_000_000
	);
end entity tb_bit_sync;

architecture sim of tb_bit_sync is
	constant SYS_CLK_FREQ_HZ : natural := 200_000_000;
	constant CLK_PERIOD      : time := 5 ns;
	constant BIT_PERIOD      : time := 1 sec / BAUD_HZ;

	signal clk           : std_logic := '0';
	signal reset         : std_logic := '1';
	signal target_baud   : unsigned(31 downto 0) := to_unsigned(BAUD_HZ, 32);
	signal async_data    : std_logic := '1';
	signal sync_data     : std_logic;
	signal recovered_clk : std_logic;
	signal lock          : std_logic;
begin
	clk <= not clk after CLK_PERIOD / 2;

	dut : entity work.bit_sync
		generic map (
			SYS_CLK_FREQ_HZ => SYS_CLK_FREQ_HZ
		)
		port map (
			clk           => clk,
			reset         => reset,
			target_baud   => target_baud,
			async_data    => async_data,
			sync_data     => sync_data,
			recovered_clk => recovered_clk,
			lock          => lock
		);

	lock_monitor : process
	begin
		wait until reset = '0';
		wait until lock = '1';
		loop
			wait until rising_edge(clk);
			assert lock = '1'
				report "DUT lost lock after acquisition"
				severity failure;
		end loop;
	end process;

	stimulus : process
		variable bit_value : std_logic;
		variable pattern   : std_logic_vector(7 downto 0) := x"96";
	begin
		wait until falling_edge(clk);
		reset <= '0';

		-- Provide enough transitions for acquisition before checking outputs.
		for bit_index in 0 to 299 loop
			bit_value := pattern(7 - (bit_index mod 8));
			async_data <= bit_value;
			wait for BIT_PERIOD;
		end loop;

		assert lock = '1'
			report "DUT did not acquire lock for baud " & integer'image(BAUD_HZ)
			severity failure;

		-- Verify the recovered bit clock continues to produce complete cycles.
		for cycle_index in 1 to 4 loop
			wait until rising_edge(recovered_clk);
			wait until falling_edge(recovered_clk);
		end loop;

		if BAUD_HZ = 10_000_000 then
			-- Check after the registered midpoint sample has settled.
			for bit_index in 0 to 63 loop
				bit_value := pattern(7 - (bit_index mod 8));
				async_data <= bit_value;
				wait for BIT_PERIOD / 2 + 2 * CLK_PERIOD;
				wait for 1 ps;
				assert sync_data = bit_value
					report "Data mismatch at bit " & integer'image(bit_index) &
						": expected " & std_logic'image(bit_value) &
						", got " & std_logic'image(sync_data)
					severity failure;
				wait for BIT_PERIOD / 2 - 2 * CLK_PERIOD - 1 ps;
			end loop;
		end if;

		-- Keep exercising the lock loop for 1 ms with bounded edge jitter.
		for bit_index in 0 to 9_999 loop
			bit_value := pattern(7 - (bit_index mod 8));
			async_data <= bit_value;
			if bit_index mod 2 = 0 then
				wait for BIT_PERIOD + 1 ns;
			else
				wait for BIT_PERIOD - 1 ns;
			end if;
		end loop;

		report "tb_bit_sync passed" severity note;
		stop;
		wait;
	end process;
end architecture sim;

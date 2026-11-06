library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_bit_sync is
	generic (
		SYS_CLK_FREQ_HZ    : positive := 200_000_000;
		BAUD_HZ            : positive := 10_000_000;
		START_PHASE_PERCENT: natural range 0 to 99 := 37;
		TARGET_STEP_PERCENT: integer := 0;
		BAUD_OFFSET_PERCENT: integer := 0;
		TRACK_OFFSET_PERCENT: integer := 0;
		LINE_CODE_MODE      : natural range 0 to 3 := 0;
		PHASE_TAP          : natural range 0 to 3 := 0
	);
end entity tb_bit_sync;

architecture sim of tb_bit_sync is
	constant TRACK_TARGET_HZ : positive := BAUD_HZ + BAUD_HZ * TARGET_STEP_PERCENT / 100;
	constant ACTUAL_BAUD_HZ : positive := BAUD_HZ + BAUD_HZ * BAUD_OFFSET_PERCENT / 100;
	constant TRACK_BAUD_HZ  : positive := TRACK_TARGET_HZ + TRACK_TARGET_HZ * TRACK_OFFSET_PERCENT / 100;
	constant CLK_PERIOD    : time := 1 sec / SYS_CLK_FREQ_HZ;
	constant BIT_PERIOD    : time := 1 sec / ACTUAL_BAUD_HZ;
	constant TRACK_PERIOD  : time := 1 sec / TRACK_BAUD_HZ;
	constant START_PHASE   : time := BIT_PERIOD * START_PHASE_PERCENT / 100;

	signal clk           : std_logic := '0';
	signal reset         : std_logic := '1';
	signal target_baud   : unsigned(31 downto 0) := to_unsigned(BAUD_HZ, 32);
	signal line_code     : unsigned(1 downto 0) := to_unsigned(LINE_CODE_MODE, 2);
	signal async_data    : std_logic := '1';
	signal sync_data_taps: std_logic_vector(3 downto 0);
	signal recovered_clks: std_logic_vector(3 downto 0);
	signal lock_taps     : std_logic_vector(3 downto 0);
	signal sync_data     : std_logic;
	signal recovered_clk : std_logic;
	signal lock          : std_logic;
begin
	clk <= not clk after CLK_PERIOD / 2;

	tap_duts : for tap_index in 0 to 3 generate
		dut : entity work.bit_sync
			generic map (
				SYS_CLK_FREQ_HZ => SYS_CLK_FREQ_HZ
			)
			port map (
				clk           => clk,
				reset         => reset,
				target_baud   => target_baud,
				line_code     => line_code,
				clock_phase   => to_unsigned(tap_index, 2),
				async_data    => async_data,
				sync_data     => sync_data_taps(tap_index),
				recovered_clk => recovered_clks(tap_index),
				lock          => lock_taps(tap_index)
			);
	end generate;

	sync_data <= sync_data_taps(PHASE_TAP);
	recovered_clk <= recovered_clks(PHASE_TAP);
	lock <= lock_taps(PHASE_TAP);

	lock_monitor : process
	begin
		wait until reset = '0';
		wait until lock_taps = "1111";
		loop
			wait until rising_edge(clk);
			assert lock_taps = "1111"
				report "DUT lost lock after acquisition"
				severity failure;
		end loop;
	end process;

	phase_tap_monitor : process
		variable reference_time : time;
		constant QUARTER_UI : time := BIT_PERIOD / 4;
		constant PHASE_TOL  : time := 2 * CLK_PERIOD;
	begin
		wait until reset = '0';
		wait until lock_taps = "1111";
		wait until rising_edge(recovered_clks(0));
		reference_time := now;
		wait until rising_edge(recovered_clks(3));
		assert abs((now - reference_time) - QUARTER_UI) <= PHASE_TOL
			report "90-degree clock tap is not one quarter UI from 0 degrees"
			severity failure;
		reference_time := now;
		wait until rising_edge(recovered_clks(2));
		assert abs((now - reference_time) - QUARTER_UI) <= PHASE_TOL
			report "180-degree clock tap is not one quarter UI from 90 degrees"
			severity failure;
		reference_time := now;
		wait until rising_edge(recovered_clks(1));
		assert abs((now - reference_time) - QUARTER_UI) <= PHASE_TOL
			report "270-degree clock tap is not one quarter UI from 180 degrees"
			severity failure;
		wait;
	end process;

	stimulus : process
		variable bit_value : std_logic;
		variable encoded   : std_logic;
		variable feedback  : std_logic;
		variable lfsr      : std_logic_vector(14 downto 0) := (others => '1');
		variable pattern   : std_logic_vector(7 downto 0) := x"96";

		procedure send_bit(
			constant value : in std_logic;
			constant code  : in natural;
			constant period: in time;
			signal serial  : out std_logic;
			variable state : inout std_logic_vector(14 downto 0)
		) is
		begin
			if code = 1 then
				serial <= not value;
				wait for period / 2;
				serial <= value;
				wait for period / 2;
			elsif code = 2 then
				encoded := value xor state(10);
				serial <= encoded;
				feedback := state(10) xor state(8);
				state(10 downto 0) := state(9 downto 0) & feedback;
				state(14 downto 11) := (others => '0');
				wait for period;
			elsif code = 3 then
				encoded := value xor state(14);
				serial <= encoded;
				feedback := state(14) xor state(0);
				state := state(13 downto 0) & feedback;
				wait for period;
			else
				serial <= value;
				wait for period;
			end if;
		end procedure;
	begin
		wait until falling_edge(clk);
		reset <= '0';
		wait for START_PHASE;

		for bit_index in 0 to 127 loop
			bit_value := pattern(7 - (bit_index mod 8));
			send_bit(bit_value, LINE_CODE_MODE, BIT_PERIOD, async_data, lfsr);
		end loop;

		assert lock = '1'
			report "DUT did not acquire lock for baud " & integer'image(BAUD_HZ) &
				" with offset " & integer'image(BAUD_OFFSET_PERCENT) & "%"
			severity failure;
		target_baud <= to_unsigned(TRACK_TARGET_HZ, 32);

		for bit_index in 0 to 31 loop
			bit_value := pattern(7 - (bit_index mod 8));
			send_bit(bit_value, LINE_CODE_MODE, TRACK_PERIOD, async_data, lfsr);
			assert sync_data = bit_value
				report "Data mismatch at bit " & integer'image(bit_index) &
					": expected " & std_logic'image(bit_value) &
					", got " & std_logic'image(sync_data)
				severity failure;
		end loop;

		for bit_index in 0 to 999 loop
			bit_value := pattern(7 - (bit_index mod 8));
			send_bit(bit_value, LINE_CODE_MODE, TRACK_PERIOD, async_data, lfsr);
		end loop;

		report "tb_bit_sync passed" severity note;
		stop;
		wait;
	end process;
end architecture sim;
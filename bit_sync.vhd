library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bit_sync is
	generic (
		SYS_CLK_FREQ_HZ : natural := 200_000_000;
		LOCK_THRESHOLD : natural := 16
	);
	port (
		clk           : in  std_logic;
		reset         : in  std_logic;
		target_baud   : in  unsigned(31 downto 0);
		async_data    : in  std_logic;
		sync_data     : out std_logic;
		recovered_clk : out std_logic;
		lock          : out std_logic
	);
end entity bit_sync;

architecture rtl of bit_sync is
	subtype phase_word is unsigned(31 downto 0);
	subtype signed_phase is signed(32 downto 0);

	constant PHASE_PI               : phase_word := x"80000000";
	constant MIN_PHASE_TOLERANCE    : unsigned(32 downto 0) := to_unsigned(536_870_912, 33);
	constant MAX_PHASE_TOLERANCE    : unsigned(32 downto 0) := to_unsigned(1_073_741_824, 33);

	function calculate_step(baud : unsigned(31 downto 0); clock_frequency : natural) return phase_word is
		variable numerator : unsigned(63 downto 0);
		variable divisor   : unsigned(63 downto 0);
		variable quotient  : unsigned(63 downto 0);
	begin
		if clock_frequency = 0 or baud > to_unsigned(clock_frequency / 2, 32) then
			return to_unsigned(0, 32);
		end if;

		-- The 64-bit shift preserves the full baud * 2^32 numerator.
		numerator := shift_left(resize(baud, 64), 32);
		divisor := to_unsigned(clock_frequency, 64);
		quotient := numerator / divisor;
		return resize(quotient, 32);
	end function;

	function calculate_tolerance(step : phase_word) return unsigned is
		variable clock_quantization : unsigned(32 downto 0);
	begin
		-- Allow at least two system-clock phase steps, with a bounded UI fraction.
		clock_quantization := shift_left(resize(step, 33), 1);
		if clock_quantization < MIN_PHASE_TOLERANCE then
			return MIN_PHASE_TOLERANCE;
		elsif clock_quantization > MAX_PHASE_TOLERANCE then
			return MAX_PHASE_TOLERANCE;
		end if;
		return clock_quantization;
	end function;

	function magnitude(value : signed_phase) return unsigned is
	begin
		if value(value'high) = '1' then
			return unsigned(-value);
		end if;
		return unsigned(value);
	end function;

	signal data_pipe      : std_logic_vector(2 downto 0) := (others => '1');
	signal edge_detected  : std_logic;
	signal phase_accum    : phase_word := (others => '0');
	signal step_size      : phase_word := (others => '0');
	signal phase_tolerance: unsigned(32 downto 0) := MIN_PHASE_TOLERANCE;
	signal lock_count     : natural range 0 to LOCK_THRESHOLD := 0;
	signal lock_reg       : std_logic := '0';
	signal clock_active   : std_logic := '0';
	signal data_reg       : std_logic := '1';
begin
	assert SYS_CLK_FREQ_HZ > 0
		report "SYS_CLK_FREQ_HZ must be non-zero"
		severity failure;
	assert LOCK_THRESHOLD > 0
		report "LOCK_THRESHOLD must be non-zero"
		severity failure;
	assert target_baud <= to_unsigned(SYS_CLK_FREQ_HZ / 2, 32)
		report "target_baud must not exceed the system clock Nyquist limit"
		severity warning;

	step_size <= calculate_step(target_baud, SYS_CLK_FREQ_HZ);
	phase_tolerance <= calculate_tolerance(step_size);
	edge_detected <= data_pipe(2) xor data_pipe(1);
	sync_data <= data_reg;
	lock <= lock_reg;

	-- Start the gated output only while the NCO clock is low, so lock acquisition
	-- cannot create a partial high pulse. The output follows the NCO MSB thereafter.
	recovered_clk <= phase_accum(31) when lock_reg = '1' and clock_active = '1' else '0';

	process (clk, reset)
		variable dist_0       : signed_phase;
		variable dist_pi      : signed_phase;
		variable phase_error  : signed_phase;
		variable abs_0        : unsigned(32 downto 0);
		variable abs_pi       : unsigned(32 downto 0);
		variable abs_error    : unsigned(32 downto 0);
		variable correction   : phase_word;
		variable phase_next   : phase_word;
	begin
		if reset = '1' then
			data_pipe    <= (others => '1');
			phase_accum  <= (others => '0');
			lock_count   <= 0;
			lock_reg     <= '0';
			clock_active <= '0';
			data_reg     <= '1';
		elsif rising_edge(clk) then
			data_pipe <= data_pipe(1 downto 0) & async_data;

			-- The phase accumulator wraps once per bit interval. Its MSB rises at
			-- 180 degrees, the nominal middle of the bit, where data is sampled.
			phase_next := phase_accum + step_size;

			if edge_detected = '1' then
				-- Measure circular phase distance to either transition boundary.
				dist_0 := resize(signed(phase_accum), signed_phase'length);
				dist_pi := resize(signed(phase_accum - PHASE_PI), signed_phase'length);
				abs_0 := magnitude(dist_0);
				abs_pi := magnitude(dist_pi);

				if abs_0 <= abs_pi then
					phase_error := dist_0;
					abs_error := abs_0;
				else
					phase_error := dist_pi;
					abs_error := abs_pi;
				end if;

				-- Apply a conservative quarter-strength correction. Explicit
				-- unsigned add/subtract avoids signed-to-unsigned reinterpretation.
				correction := resize(shift_right(abs_error, 2), 32);
				if phase_error(phase_error'high) = '1' then
					phase_next := phase_next + correction;
				elsif abs_error /= 0 then
					phase_next := phase_next - correction;
				end if;

				-- Accept phase errors within one eighth of a bit interval. The
				-- saturating counter acquires lock after good edges and releases it
				-- only after repeated out-of-tolerance edges.
				if abs_error < phase_tolerance then
					if lock_count < LOCK_THRESHOLD then
						lock_count <= lock_count + 1;
						if lock_count + 1 >= LOCK_THRESHOLD then
							lock_reg <= '1';
						end if;
					end if;
				elsif lock_count > 0 then
					lock_count <= lock_count - 1;
					if lock_count = 1 then
						lock_reg <= '0';
					end if;
				end if;
			end if;

			phase_accum <= phase_next;

			-- Sampling on the NCO's 0-to-180-degree crossing places the data
			-- register at the center of each nominal bit interval.
			if phase_accum(31) = '0' and phase_next(31) = '1' then
				data_reg <= data_pipe(2);
			end if;

			if lock_reg = '0' then
				clock_active <= '0';
			elsif phase_accum(31) = '0' then
				clock_active <= '1';
			end if;
		end if;
	end process;
end architecture rtl;

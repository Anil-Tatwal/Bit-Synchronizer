library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bit_sync is
	generic (
		SYS_CLK_FREQ_HZ : positive := 200_000_000;
		LOCK_THRESHOLD  : positive := 16
	);
	port (
		clk           : in  std_logic;
		reset         : in  std_logic;
		target_baud   : in  unsigned(31 downto 0);
		line_code     : in  unsigned(1 downto 0);
		clock_phase   : in  unsigned(1 downto 0);
		async_data    : in  std_logic;
		sync_data     : out std_logic;
		recovered_clk : out std_logic;
		lock          : out std_logic
	);
end entity bit_sync;

architecture rtl of bit_sync is
	subtype phase_word is unsigned(47 downto 0);
	subtype signed_phase is signed(47 downto 0);
	subtype frequency_adjustment is signed(48 downto 0);

	constant PHASE_PI            : phase_word := x"800000000000";
	constant MIN_LOCK_TOLERANCE  : phase_word := x"004000000000";
	constant MAX_LOCK_TOLERANCE  : phase_word := x"400000000000";
	constant ACQUIRE_MIN_BAUD    : unsigned(31 downto 0) := to_unsigned(100, 32);
	constant ACQUIRE_MAX_BAUD    : unsigned(31 downto 0) := to_unsigned(20_000_000, 32);

	function calculate_step(baud : unsigned(31 downto 0); clock_frequency : positive) return phase_word is
		variable numerator : unsigned(79 downto 0);
		variable divisor   : unsigned(79 downto 0);
		variable quotient  : unsigned(79 downto 0);
	begin
		numerator := shift_left(resize(baud, 80), 48);
		divisor := to_unsigned(clock_frequency, 80);
		quotient := numerator / divisor;
		return resize(quotient, 48);
	end function;

	function phase_magnitude(value : signed_phase) return phase_word is
	begin
		if value(value'high) = '1' then
			return unsigned(-value);
		end if;
		return unsigned(value);
	end function;

	function calculate_tolerance(step : phase_word) return phase_word is
		variable clock_quantization : unsigned(48 downto 0);
	begin
		clock_quantization := shift_left(resize(step, 49), 1);
		if clock_quantization < resize(MIN_LOCK_TOLERANCE, 49) then
			return MIN_LOCK_TOLERANCE;
		elsif clock_quantization > resize(MAX_LOCK_TOLERANCE, 49) then
			return MAX_LOCK_TOLERANCE;
		end if;
		return resize(clock_quantization, 48);
	end function;

	signal data_pipe        : std_logic_vector(2 downto 0) := (others => '1');
	signal edge_detected    : std_logic;
	signal phase_accum      : phase_word := (others => '0');
	signal nominal_step     : phase_word := (others => '0');
	signal phase_tolerance  : phase_word := MIN_LOCK_TOLERANCE;
	signal freq_adjust      : frequency_adjustment := (others => '0');
	signal lock_count       : natural range 0 to LOCK_THRESHOLD := 0;
	signal unlock_count     : natural range 0 to LOCK_THRESHOLD := 0;
	signal lock_reg         : std_logic := '0';
	signal tracking_enabled : std_logic := '0';
	signal clock_active     : std_logic := '0';
	signal data_reg         : std_logic := '1';
	signal previous_edge_phase : std_logic := '0';
	signal previous_edge_valid : std_logic := '0';
	signal previous_interval_short : std_logic := '0';
	signal scrambler_state  : std_logic_vector(14 downto 0) := (others => '1');
	signal phase_rotated    : phase_word;
	signal phase_clock      : std_logic;
	attribute ASYNC_REG : string;
	attribute ASYNC_REG of data_pipe : signal is "TRUE";
begin
	assert target_baud >= ACQUIRE_MIN_BAUD and target_baud <= ACQUIRE_MAX_BAUD
		report "target_baud must be between 100 bps and 20 Mbps"
		severity warning;
	assert target_baud <= to_unsigned(SYS_CLK_FREQ_HZ / 2, 32)
		report "system clock must be at least twice target_baud"
		severity failure;

	nominal_step <= calculate_step(target_baud, SYS_CLK_FREQ_HZ);
	phase_tolerance <= calculate_tolerance(nominal_step);
	edge_detected <= data_pipe(2) xor data_pipe(1);
	sync_data <= data_reg;
	lock <= lock_reg;

	phase_rotated <= phase_accum + shift_left(resize(clock_phase, 48), 46);
	phase_clock <= phase_rotated(47);
	recovered_clk <= phase_clock when lock_reg = '1' and clock_active = '1' else '0';

	process (clk, reset)
		variable phase_error       : signed_phase;
		variable dist_pi           : signed_phase;
		variable phase_measure     : phase_word;
		variable abs_pi            : phase_word;
		variable phase_next        : phase_word;
		variable effective_step    : phase_word;
		variable phase_rotation    : phase_word;
		variable max_adjust        : frequency_adjustment;
		variable min_step          : phase_word;
		variable max_step          : phase_word;
		variable loop_product      : unsigned(95 downto 0);
		variable frequency_delta   : phase_word;
		variable proposed_adjust   : frequency_adjustment;
		variable sampled_bit       : std_logic;
		variable lfsr_feedback     : std_logic;
		variable lfsr_next         : std_logic_vector(14 downto 0);
		variable edge_phase_error  : signed_phase;
		variable edge_phase_abs    : phase_word;
		variable edge_phase_slot   : std_logic;
		variable is_center_edge    : boolean;
		variable short_interval    : boolean;
	begin
		if reset = '1' then
			data_pipe        <= (others => '1');
			phase_accum      <= (others => '0');
			freq_adjust      <= (others => '0');
			lock_count       <= 0;
			unlock_count     <= 0;
			lock_reg         <= '0';
			tracking_enabled <= '0';
			clock_active     <= '0';
			data_reg         <= '1';
			previous_edge_phase <= '0';
			previous_edge_valid <= '0';
			previous_interval_short <= '0';
			scrambler_state  <= (others => '1');
		elsif rising_edge(clk) then
			data_pipe <= data_pipe(1 downto 0) & async_data;

			-- Keep acquisition within +/-10%; expand the tracking clamp after lock.
			max_adjust := signed(resize(nominal_step / 5, 49));
			if tracking_enabled = '0' then
				max_adjust := signed(resize(nominal_step / 10, 49));
			end if;
			min_step := nominal_step - unsigned(max_adjust(47 downto 0));
			max_step := nominal_step + unsigned(max_adjust(47 downto 0));
			proposed_adjust := freq_adjust;
			effective_step := nominal_step + unsigned(freq_adjust(47 downto 0));
			if freq_adjust(48) = '1' then
				effective_step := nominal_step - unsigned(-freq_adjust(47 downto 0));
			end if;
			if effective_step < min_step then
				effective_step := min_step;
			elsif effective_step > max_step then
				effective_step := max_step;
			end if;

			phase_next := phase_accum + effective_step;
			if edge_detected = '1' then
				-- The stage-2/stage-1 XOR is observed two clocks after input sampling.
				phase_measure := phase_accum - effective_step - effective_step;
				phase_error := signed(phase_measure);
				edge_phase_slot := '0';
				edge_phase_abs := phase_magnitude(phase_error);
				edge_phase_error := phase_error;
				is_center_edge := true;

				if line_code = "01" then
					dist_pi := signed(phase_measure - PHASE_PI);
				abs_pi := phase_magnitude(dist_pi);
				if abs_pi < edge_phase_abs then
					edge_phase_slot := '1';
					edge_phase_error := dist_pi;
					edge_phase_abs := abs_pi;
				end if;
				if previous_edge_valid = '1' then
					short_interval := edge_phase_slot /= previous_edge_phase;
					is_center_edge := not short_interval or previous_interval_short = '1';
					if short_interval then
						previous_interval_short <= '1';
					else
						previous_interval_short <= '0';
					end if;
				else
					previous_edge_valid <= '1';
					previous_interval_short <= '0';
				end if;
				previous_edge_phase <= edge_phase_slot;
				end if;

				-- Proportional phase correction plus an integral frequency correction.
				if edge_phase_error(47) = '1' then
					phase_next := phase_next + shift_right(edge_phase_abs, 2);
				else
					phase_next := phase_next - shift_right(edge_phase_abs, 2);
				end if;
				loop_product := edge_phase_abs * nominal_step;
				frequency_delta := resize(shift_right(loop_product, 50), 48);
				if edge_phase_error(47) = '1' then
					proposed_adjust := freq_adjust + signed(resize(frequency_delta, 49));
				else
					proposed_adjust := freq_adjust - signed(resize(frequency_delta, 49));
				end if;
				if proposed_adjust > max_adjust then
					proposed_adjust := max_adjust;
				elsif proposed_adjust < -max_adjust then
					proposed_adjust := -max_adjust;
				end if;
				freq_adjust <= proposed_adjust;

				if is_center_edge and edge_phase_abs <= phase_tolerance then
					unlock_count <= 0;
					if lock_count < LOCK_THRESHOLD then
						lock_count <= lock_count + 1;
						if lock_count + 1 >= LOCK_THRESHOLD then
							lock_reg <= '1';
							tracking_enabled <= '1';
						end if;
					end if;
				elsif is_center_edge then
					lock_count <= 0;
					if lock_reg = '1' then
						if unlock_count < LOCK_THRESHOLD then
							unlock_count <= unlock_count + 1;
							if unlock_count + 1 >= LOCK_THRESHOLD then
								lock_reg <= '0';
							end if;
						end if;
					else
						unlock_count <= 0;
					end if;
				end if;
				if line_code = "01" and is_center_edge then
					data_reg <= data_pipe(1);
				end if;
			end if;

			phase_accum <= phase_next;
			phase_rotation := phase_accum + shift_left(resize(clock_phase, 48), 46);
			if lock_reg = '0' then
				clock_active <= '0';
			elsif phase_rotation(47) = '0' then
				clock_active <= '1';
			end if;

			-- The NCO half-cycle crossing is the nominal center of each NRZ bit.
			if line_code /= "01" and phase_accum(47) = '0' and phase_next(47) = '1' then
				sampled_bit := data_pipe(2);
				if line_code = "10" then
					sampled_bit := data_pipe(2) xor scrambler_state(10);
					lfsr_feedback := scrambler_state(10) xor scrambler_state(8);
					lfsr_next := (others => '0');
					lfsr_next(10 downto 0) := scrambler_state(9 downto 0) & lfsr_feedback;
					scrambler_state <= lfsr_next;
				elsif line_code = "11" then
					sampled_bit := data_pipe(2) xor scrambler_state(14);
					lfsr_feedback := scrambler_state(14) xor scrambler_state(0);
					lfsr_next := scrambler_state(13 downto 0) & lfsr_feedback;
					scrambler_state <= lfsr_next;
				end if;
				data_reg <= sampled_bit;
			end if;
		end if;
	end process;
end architecture rtl;
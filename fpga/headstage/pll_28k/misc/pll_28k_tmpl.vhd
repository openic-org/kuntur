component pll_28k is
    port(
        clki_i: in std_logic;
        phasedir_i: in std_logic;
        phasestep_i: in std_logic;
        phaseloadreg_i: in std_logic;
        phasesel_i: in std_logic_vector(2 downto 0);
        clkop_o: out std_logic;
        clkos_o: out std_logic;
        lock_o: out std_logic
    );
end component;

__: pll_28k port map(
    clki_i=>,
    phasedir_i=>,
    phasestep_i=>,
    phaseloadreg_i=>,
    phasesel_i=>,
    clkop_o=>,
    clkos_o=>,
    lock_o=>
);

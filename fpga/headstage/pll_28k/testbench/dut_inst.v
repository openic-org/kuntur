    pll_28k u_pll_28k(.clki_i(clki_i),
        .phasedir_i(phasedir_i),
        .phasestep_i(phasestep_i),
        .phaseloadreg_i(phaseloadreg_i),
        .phasesel_i(phasesel_i),
        .clkop_o(clkop_o),
        .clkos_o(clkos_o),
        .lock_o(lock_o));

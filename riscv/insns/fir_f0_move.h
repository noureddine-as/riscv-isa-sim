#define N_COEFFS   32

if (RS2 > (N_COEFFS + 2)){
	fprintf(stderr, "[SPIKE / FIR RoCC] RS2 (regfile[ %d) ]) is greater than %d, this register doesn't exist! \n", N_COEFFS + 2);
	throw trap_illegal_instruction(0);
}
else if (RS2 == (N_COEFFS + 2)){
	fprintf(stderr, "[SPIKE / FIR RoCC] Regfile[ %d ] is READ-ONLY Result register.\n", N_COEFFS + 2);
	throw trap_illegal_instruction(0);
}
else if(RS2 == (N_COEFFS + 1)){
	fprintf(stderr, "[SPIKE / FIR RoCC] Writing 0x%08x to Status Register (reg. %d) \n", RS1, N_COEFFS + 1);

	if(RS1 == 0x01){
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  FIR enabled. \n");

		// TO DO
		// Begin the calculations here.

        uint64_t fir_res = 0;

        int j ;
        for(j=0; j < N_COEFFS; j++)
        {
        	fir_res = (uint64_t)(fir_res + MMU.fir_rocc_regfile[j]*MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j));
        }

        for(j=0; j < MMU.fir_rocc_fifo.size(); j++)
        {
        	fprintf(stderr, "FIFO[ %d ] = 0x%08x \n", MMU.fir_rocc_fifo.size() - 1 - j, MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j));
        }

        // Output on 8 bits
        fir_res = (fir_res >> 12); // Result register = Result
        MMU.fir_rocc_regfile[N_COEFFS + 2] = fir_res;
        MMU.fir_rocc_regfile[N_COEFFS + 1] &= 0xFFFFFFFFFFFFFFFE; // ENABLE_BIT = 0
		MMU.fir_rocc_regfile[N_COEFFS + 1] |= (1 << 31); // DONE_BIT = 1

		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Calculated Yn = 0x%08x \n", fir_res);

	}
	else if(RS1 == 0x02){
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  INT enabled. \n");

		// TO DO
		// Here you should calculate and send an interrupt !!!

	}
	else if(RS1 == (0x01 << 31)){
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Done bit written, this is ILLEGAL !!\n");
		throw trap_illegal_instruction(0);
	}else
	{
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Other fields of the FIR RoCC aren't supported !!\n");
		throw trap_illegal_instruction(0);
	}

}
else if(RS2 == (N_COEFFS + 1)){
	// TO DO
	// Rewriting precesion
}
else
{
	MMU.fir_rocc_regfile[RS2] = RS1;
	fprintf(stderr, "[SPIKE / FIR RoCC] Moved 0x%08x  to register  reg_file[%d] \n", RS1, RS2);
}




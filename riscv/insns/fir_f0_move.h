#define N_COEFFS   32

#define VERBOSITY  0
	#if(VERBOSITY)
	#endif
static int samples_counter = 0;
if (RS2 > (N_COEFFS + 2)){
	#if(VERBOSITY)
	fprintf(stderr, "[SPIKE / FIR RoCC] RS2 (regfile[ %d) ]) is greater than %d, this register doesn't exist! \n", N_COEFFS + 2);
	#endif

	throw trap_illegal_instruction(0);
}
else if (RS2 == (N_COEFFS + 2)){
	#if(VERBOSITY)
	fprintf(stderr, "[SPIKE / FIR RoCC] Regfile[ %d ] is READ-ONLY Result register.\n", N_COEFFS + 2);
	#endif

	throw trap_illegal_instruction(0);
}
else if(RS2 == (N_COEFFS + 1)){
	#if(VERBOSITY)
	fprintf(stderr, "[SPIKE / FIR RoCC] Writing 0x%08x to Status Register (reg. %d) \n", RS1, N_COEFFS + 1);
	#endif

	if(RS1 == 0x01){
		#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  FIR enabled. \n");
		#endif

		// TO DO
		// Begin the calculations here.

        uint64_t fir_res = 0;
        

        int j, approx_counter=0;
        for(j=0; j < N_COEFFS; j++)
        {

        	// FIFO[ 0 ] = t-n
        	// FIFO[ 1 ] = t - (n-1)
        	// ...
        	// FIFO[ N_COEFFS - 2 ] = t - 1
        	// FIFO[ N_COEFFS - 1 ] = t

        	if((/* j > 0 && */ j < N_COEFFS-1) && 
        	   ((MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j - 1) & (~MMU.fir_rocc_regfile[N_COEFFS]))
        	    == (MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j) & (~MMU.fir_rocc_regfile[N_COEFFS])))){

        		fir_res = (uint64_t)(fir_res + (MMU.fir_rocc_regfile[j] + MMU.fir_rocc_regfile[j+1])*MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j));
				// fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  APPROXIMATION at Sample= %d \t- j = %d \t and j+1 = %d \n", samples_counter, j, j+1);
    //     		fprintf(stderr, "[SPIKE / FIR RoCC]               sple[j]     = %d \t sple[j+1]     = %d \n", MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j),
    //     																							  MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j - 1));
    //     		fprintf(stderr, "[SPIKE / FIR RoCC]               AxC_sple[j] = %d \t AxC_sple[j+1] = %d \n",
    //     													(MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j - 1) & (~(MMU.fir_rocc_regfile[N_COEFFS]))),
    //     													(MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j)     & (~(MMU.fir_rocc_regfile[N_COEFFS]))));

    //     		fprintf(stderr, "[SPIKE / FIR RoCC]               Applied precision mask = 0x%x \n", ~MMU.fir_rocc_regfile[N_COEFFS]);

        		j++; // Advance the pointer, since we used two coeffs
        		approx_counter++;

        	}else{
    	    	fir_res = (uint64_t)(fir_res + MMU.fir_rocc_regfile[j]*MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j));
           	}


       
        }
       	// if(approx_counter)
    		fprintf(stderr, "[SPIKE / FIR RoCC]               [Sample %d ] Number of approximations (1 * instead of 2) is %d\n", samples_counter, approx_counter);
        samples_counter++;


        // int j , prev_sample;
        // for(j= N_COEFFS-1; j > 0; j--)
        // {
        // 	// if((j != 0) && 
        // 	//    ((MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j - 1) & MMU.fir_rocc_regfile[N_COEFFS + 1])
        // 	//     == (MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j) & MMU.fir_rocc_regfile[N_COEFFS + 1])) )

        // 	fir_res = (uint64_t)(fir_res + MMU.fir_rocc_regfile[N_COEFFS-1 - j]*MMU.fir_rocc_fifo.at(j));
       
        // }


	#if(VERBOSITY)
        for(j=0; j < MMU.fir_rocc_fifo.size(); j++)
        {
        	fprintf(stderr, "FIFO[ %d ] = 0x%08x \n", MMU.fir_rocc_fifo.size() - 1 - j, MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j));
        }
	#endif

        // Output on 8 bits
        // TO DO
        // fir_res = (fir_res >> 12); // Result register = Result
        MMU.fir_rocc_regfile[N_COEFFS + 2] = fir_res;
        MMU.fir_rocc_regfile[N_COEFFS + 1] &= 0xFFFFFFFFFFFFFFFE; // ENABLE_BIT = 0
		MMU.fir_rocc_regfile[N_COEFFS + 1] |= (1 << 31); // DONE_BIT = 1
	#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Calculated Yn = 0x%08x \n", fir_res);
	#endif

	}
	else if(RS1 == 0x02){
	#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  INT enabled. \n");
	#endif

		// TO DO
		// Here you should calculate and send an interrupt !!!

	}
	else if(RS1 == (0x01 << 31)){
	#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Done bit written, this is ILLEGAL !!\n");
	#endif
		throw trap_illegal_instruction(0);
	}else
	{
	#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]        --->>  Other fields of the FIR RoCC aren't supported !!\n");
	#endif
		throw trap_illegal_instruction(0);
	}

}
else if(RS2 == N_COEFFS){
	MMU.fir_rocc_regfile[RS2] = RS1;
	#if(VERBOSITY)
	fprintf(stderr, "[SPIKE / FIR RoCC] Modified precision to 0x%08x \n", RS1);
	#endif
}
else
{
	MMU.fir_rocc_regfile[RS2] = RS1;
	#if(VERBOSITY)
	fprintf(stderr, "[SPIKE / FIR RoCC] Moved 0x%08x  to register  reg_file[%d] \n", RS1, RS2);
	#endif
}




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

        uint64_t fir_res = 0;
        int j, approx_counter=0;
        for(j=0; j < N_COEFFS; j++)
        {

        	// FIFO[ 0 ] = t-n
        	// FIFO[ 1 ] = t - (n-1)
        	// ...
        	// FIFO[ N_COEFFS - 2 ] = t - 1
        	// FIFO[ N_COEFFS - 1 ] = t
	        
	        uint16_t H_now, H_last, X_now, X_last, Precision_Mask;

        	X_now = (uint16_t)(((MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j) >> (16 * 3))) & 0xFFFF);
        	H_now = (uint16_t)((MMU.fir_rocc_regfile[j] >> (16 * 3)) & 0xFFFF);

        	if(j < N_COEFFS-1)
        	{ 
	        	H_last = (uint16_t)((MMU.fir_rocc_regfile[j+1] >> (16 * 3)) & 0xFFFF);
	        	X_last = (uint16_t)(((MMU.fir_rocc_fifo.at(MMU.fir_rocc_fifo.size() - 1 - j - 1) >> (16 * 3))) & 0xFFFF);

	        	Precision_Mask = (uint16_t)((~MMU.fir_rocc_regfile[N_COEFFS]) & 0xFFFF); 
    	
    	   	    if( (Precision_Mask & X_now) == (Precision_Mask & X_last)){
					fir_res +=  (uint64_t)(X_now*(H_now + H_last));

	        		j++; // Advance the pointer, since we used two coeffs
	        		approx_counter++;
    	   	    }else{
					fir_res +=  (uint64_t)(X_now * H_now);
    	   	    }

        	}else{
				
				fir_res +=  (uint64_t)(X_now * H_now);
           	}
        }
		// if(approx_counter)
		samples_counter++;
	#if(VERBOSITY)
		fprintf(stderr, "[SPIKE / FIR RoCC]               [Sample %d ] Number of approximations (1 * instead of 2) is %d\n", samples_counter, approx_counter);

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




MMU.store_uint64(RS1, MMU.fir_rocc_regfile[RS2]);

#define VERBOSITY  0
	#if(VERBOSITY)
fprintf(stderr, "[SPIKE / FIR RoCC] Stored regfile[ %d ] = 0x%08x at addr %08x", RS2, MMU.fir_rocc_regfile[RS2], RS1);
	#endif

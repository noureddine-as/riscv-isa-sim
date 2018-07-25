#define N_COEFFS   32

if(MMU.fir_rocc_fifo.size() < N_COEFFS)
	MMU.fir_rocc_fifo.push_back(RS1);
else
{
	MMU.fir_rocc_fifo.erase(MMU.fir_rocc_fifo.begin()); // - 1);
	// Is it really a FIFO ?
	//MMU.fir_rocc_fifo.erase(MMU.fir_rocc_fifo.begin());
	MMU.fir_rocc_fifo.push_back(RS1);  
	// New values are those in the END. You may have to invert your indexes in the original FIR algo

}

fprintf(stderr, "[SPIKE / FIR RoCC]     RS1 (0x%08x) pushed in the FIR fifo. \n", RS1);

// Print all fifo elements
/*
for(auto i = MMU.fir_rocc_fifo.begin(); i != MMU.fir_rocc_fifo.end(); i++)
    printf("[SPIKE / FIR RoCC]     fir_rocc_fifo[ %d ] = %d \n", i - MMU.fir_rocc_fifo.begin(), MMU.fir_rocc_fifo.at(i - MMU.fir_rocc_fifo.begin()));
*/

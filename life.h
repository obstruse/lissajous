
uint lifeNext = 1;
uint lifeCounter = 0;
uint lifeBit = 0;
uint lifeByte = 0;
uint64_t emptyGlob;

    for ( int i=0; i<sizeof(emptyGlob); i++ ) {
	emptyGlob = (emptyGlob<<2)|BLACK;
	emptyGlob = (emptyGlob<<2)|BLACK;
	emptyGlob = (emptyGlob<<2)|BLACK;
	emptyGlob = (emptyGlob<<2)|BLACK;
    }

// seed
//#include "gosperglidergun.h"
//#include "bunnies.h"
//#include "blom.h"
//#include "homer.h"
#include "multuminparvo.h"

    int lastCol = 0;
    int thisCol = 0;
    int nextCol = 0;

    while (true) {
	    // work on the next generation
	    lifeNext = (lifeGen + 1) & 0x01;

	    // for every cell, count neighbors in current generation
	    // ...for each row
	    for ( int row = 1; row < height-1; row++ ) {

		// get the globs ready
		uint64_t * glob1 = (uint64_t *)(life[lifeGen]+((row-1)*width/4));
		uint64_t * glob2 = (uint64_t *)(life[lifeGen]+((row)*width/4));
		uint64_t * glob3 = (uint64_t *)(life[lifeGen]+((row+1)*width/4));
		uint64_t * glob4 = (uint64_t *)(life[lifeNext]+((row)*width/4));

		// sum the cols...
		lastCol = getLifeCell(lifeGen,-1,row-1) + getLifeCell(lifeGen,-1,row) + getLifeCell(lifeGen,-1,row+1);

		// do each glob on row
		int col = 0;
		for ( int g = 0; g < width/32; g++ ) {
		    int c = 0;

		    if ( lastCol == 0 && *(glob1+g) == emptyGlob && *(glob2+g) == emptyGlob && *(glob3+g) == emptyGlob ) {
			// there's nothing to process, skip to end of glob
			*(glob4+g) = emptyGlob;
			c = 31;
			col += c;
		    }

    		    thisCol = getLifeCell(lifeGen,col,row-1) + getLifeCell(lifeGen,col,row) + getLifeCell(lifeGen,col,row+1);
	    	    for ( ; c < 32; c++ ) {
			
		        nextCol = getLifeCell(lifeGen,col+1,row-1) + getLifeCell(lifeGen,col+1,row) + getLifeCell(lifeGen,col+1,row+1);
		        int currentCell = getLifeCell(lifeGen, col, row);
		        int liveNeighbors = lastCol + thisCol + nextCol - currentCell;

		        setLifeCell(lifeNext, col,  row, currentCell);
		        if ( liveNeighbors < 2 ) setLifeCell(lifeNext, col, row, 0);
		        if ( liveNeighbors > 3 ) setLifeCell(lifeNext, col, row, 0);
		        if ( liveNeighbors == 3 ) setLifeCell(lifeNext, col, row, 1);

                        lastCol = thisCol;
                        thisCol = nextCol;
			col++;
		    }

		}
	    }


	    lifeGen = lifeNext;

	    //sleep_ms(1000);		// sleep_ms() causes a visible glitch
	    //busy_wait_us(1000000);
    }
}

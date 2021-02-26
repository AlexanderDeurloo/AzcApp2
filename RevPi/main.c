/*=======================================================================================
 *
 *	       KK    KK   UU    UU   NN    NN   BBBBBB    UU    UU    SSSSSS
 *	       KK   KK    UU    UU   NNN   NN   BB   BB   UU    UU   SS
 *	       KK  KK     UU    UU   NNNN  NN   BB   BB   UU    UU   SS
 *	+----- KKKKK      UU    UU   NN NN NN   BBBBB     UU    UU    SSSSS
 *	|      KK  KK     UU    UU   NN  NNNN   BB   BB   UU    UU        SS
 *	|      KK   KK    UU    UU   NN   NNN   BB   BB   UU    UU        SS
 *	|      KK    KKK   UUUUUU    NN    NN   BBBBBB     UUUUUU    SSSSSS     GmbH
 *	|
 *	|            [#]  I N D U S T R I A L   C O M M U N I C A T I O N
 *	|             |
 *	+-------------+
 *
 *---------------------------------------------------------------------------------------
 *
 * (C) KUNBUS GmbH, Heerweg 15C, 73770 Denkendorf, Germany
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License V2 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  For licencing details see COPYING
 *
 *=======================================================================================
 */

#include "piControlIf.h"
#include "piControl.h"
#include <string.h>
#include <stdio.h>

#include <time.h>


#include "CustomFunctions.h"
int main(int argc, char ** argv)
{

    blinkOnInput();
// 	int  i = 0;
// 	int  iLastInputValue = 0;
// 	char *pchInput  = NULL;
// 	char *pchOutput = NULL;

// 	// structures containing variable information: Name, Offset, Bit, Length
// 	SPIVariable spiVariableOut = {"", 0, 0, 0};
//     SPIVariable spiVariableIn = {"", 0, 0, 0};

// 	// structures containing variable value: Offset, Bit, Value
// 	SPIValue sValueOut  = {0, 0, 0};
//     SPIValue sValueIn  = {0, 0, 0};


//     pchOutput = "O_1_i03";
//     pchInput = "I_1";

//     //Copy pin name into struct
// 	strncpy(spiVariableOut.strVarName, pchOutput, sizeof(spiVariableOut.strVarName));
//     strncpy(spiVariableIn.strVarName, pchInput, sizeof(spiVariableIn.strVarName));

//     spiVariableOut.i8uBit = 9;

//     i = piControlGetVariableInfo(&spiVariableOut);		// PiBridge - get variable info
// 	if(0 != i)											// handle error
// 	{
// 		fprintf(stderr, "Error: piControlGetVariableInfo() returned %d for variable '%s' \n",
// 			i, spiVariableOut.strVarName);
// 		return -1;
// 	}

//     i = piControlGetVariableInfo(&spiVariableIn);		// PiBridge - get variable info
// 	if(0 != i)											// handle error
// 	{
// 		fprintf(stderr, "Error: piControlGetVariableInfo() returned %d for variable '%s' \n",
// 			i, spiVariableOut.strVarName);
// 		return -1;
// 	}



//     printf("Name: %s Adress: %d Bit %d Length: %d \n", spiVariableOut.strVarName, spiVariableOut.i16uAddress, spiVariableOut.i8uBit, spiVariableOut.i16uLength);
// 	sValueIn.i16uAddress  = spiVariableIn.i16uAddress;
// 	sValueIn.i8uBit       = spiVariableIn.i8uBit;
// 	sValueIn.i8uValue     = 0;

// 	sValueOut.i16uAddress = spiVariableOut.i16uAddress;
// 	sValueOut.i8uBit      = spiVariableOut.i8uBit;
// 	sValueOut.i8uValue    = 0;

// 	//printf("%s is running waiting for switch '%s' \n", argv[0], pchInput);
	
//     int counter = 0;
//     int counter2 = 0;

//     uint8_t firstLeds = 0;
//    // piControlWrite(232,8,0);


//     struct timespec ts;

//     while(1)
//     {

//         piControlGetBitValue(&sValueIn);
//         if(sValueIn.i8uValue==1)
//         {
//             firstLeds = 0b00000001;
//             // uint8_t secondLeds =0b00000010;


//             for(counter2 = 0; counter2<14; counter2++)
//             {
//                 piControlWrite(70,8,&firstLeds);

//                 if(counter2 > 6)
//                 {   
//                     printf("Down %d \n",firstLeds);
//                     firstLeds = firstLeds>>1;
//                 }
//                 else
//                 {
//                     printf("Up %d\n",firstLeds);
//                     firstLeds = firstLeds<<1;
//                 }
                
                
//                 ts.tv_sec = 50/1000;
//                 ts.tv_nsec = (50%1000)*1000000;

//                 nanosleep(&ts,&ts);
//             }
//             firstLeds = 0b00000000;
//             piControlWrite(70,8,&firstLeds);
//         }

    
        
//         // sValueOut.i8uValue = 1;
//         // piControlSetBitValue(&sValueOut);
//         // sleep(1);
//         // sValueOut.i8uValue = 0;
//         // piControlSetBitValue(&sValueOut);
// 		// sleep(1);
// 		//iLastInputValue = sValueIn.i8uValue;			// remember last input value
	
//     }


	return 0;
}

#include <stdio.h>

volatile uint8_t inBuffer[32];
volatile uint8_t payloadBytes[32];

#define N 192

void setcolor(uint8_t* image, uint8_t x, uint8_t yline, float r, float g, float b) {
	uint8_t rval = (uint8_t) 31.9*r;
	uint8_t gval = (uint8_t) 63.9*g;
	uint8_t bval = (uint8_t) 31.9*b;

	image[2*(x + 96*yline)] = (uint8_t) (rval << 3) + (gval >> 3);
	image[2*(x + 96*yline) + 1] = (uint8_t) (gval << 5) + bval;
}


int round_to_int(float x)
{
    if (x < 0.0)
        return (int)(x - 0.5);
    else
        return (int)(x + 0.5);
}

float triangle_interpolate(float f1,float f2,float f3, float v1_1, float v1_2, float v2_1, float v2_2, 
							float v3_1, float v3_2, float p_1, float p_2) {
    // three values f1-f3 linearly interpolated at a point p between three vertices v1,v2,v3
	
	float Wv1 = ( (v2_2-v3_2)*(p_1-v3_1) + (v3_1-v2_1)*(p_2-v3_2) ) 
		/ ( (v2_2-v3_2)*(v1_1-v3_1)	+ (v3_1-v2_1)*(v1_2-v3_2) );
	float Wv2 = ( (v3_2-v1_2)*(p_1-v3_1) + (v1_1-v3_1)*(p_2-v3_2) )
		/ ( (v2_2-v3_2)*(v1_1-v3_1)	+ (v3_1-v2_1)*(v1_2-v3_2) );
	float Wv3 = 1.0 - Wv1 - Wv2;

	return (f1*Wv1 + f2*Wv2 + f3*Wv3);   

} 

void
displaywrite(uint8_t* image)
{
	spi_status_t status;

	/*
	 *	/CS (PTA12) low
	 */
	GPIO_DRV_ClearPinOutput(kGpioOC);
    GPIO_DRV_ClearPinOutput(kGpioRST);
	GPIO_DRV_SetPinOutput(kGpioDC);
	
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&image[0],
					(uint8_t * restrict)&image[0],
					N		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);					

	/*
	 *	/CS (PTA12) high
	 */
	GPIO_DRV_SetPinOutput(kGpioOC);
	GPIO_DRV_SetPinOutput(kGpioDC);
	GPIO_DRV_ClearPinOutput(kGpioRST);

	return;
}

void
writedisplay(uint8_t option)
{
	spi_status_t status;

    // #define N 768

    uint8_t image[N];
	int i,j;
	

	GPIO_DRV_ClearPinOutput(kGpioOC);
	GPIO_DRV_ClearPinOutput(kGpioRST);
	GPIO_DRV_SetPinOutput(kGpioDC);


	int xarr[] = {3,4,2,5};

	// geometry
	int Ngeom = 8;

	uint8_t tx1[8],tx2[8],tx3[8],ty1[8],ty2[8],ty3[8],tz1[8],tz2[8],tz3[8];
	float rcolor[8],gcolor[8],bcolor[8];

	
	tx1[0]=120;ty1[0]=78;tz1[0]=149; tx2[0]=120;ty2[0]=138;tz2[0]=119; tx3[0]=94;ty3[0]=78;tz3[0]=104; 
	tx1[1]=94;ty1[1]=78;tz1[1]=104; tx2[1]=120;ty2[1]=138;tz2[1]=119; tx3[1]=146;ty3[1]=78;tz3[1]=104; 
	tx1[2]=146;ty1[2]=78;tz1[2]=104; tx2[2]=120;ty2[2]=138;tz2[2]=119; tx3[2]=120;ty3[2]=78;tz3[2]=149; 
	tx1[3]=120;ty1[3]=78;tz1[3]=149; tx2[3]=94;ty2[3]=78;tz2[3]=104; tx3[3]=146;ty3[3]=78;tz3[3]=104; 
	tx1[4]=105;ty1[4]=127;tz1[4]=102; tx2[4]=114;ty2[4]=79;tz2[4]=55; tx3[4]=126;ty3[4]=145;tz3[4]=57; 
	tx1[5]=126;ty1[5]=145;tz1[5]=57; tx2[5]=114;ty2[5]=79;tz2[5]=55; tx3[5]=75;ty3[5]=133;tz3[5]=60; 
	tx1[6]=75;ty1[6]=133;tz1[6]=60; tx2[6]=114;ty2[6]=79;tz2[6]=55; tx3[6]=105;ty3[6]=127;tz3[6]=102; 
	tx1[7]=105;ty1[7]=127;tz1[7]=102; tx2[7]=126;ty2[7]=145;tz2[7]=57; tx3[7]=75;ty3[7]=133;tz3[7]=60; 

	rcolor[0] = 1.0;
	rcolor[1] = 0.7;
	rcolor[2] = 0.4;
	rcolor[3] = 0.2;
	rcolor[4] = 0.0;
	rcolor[5] = 0.0;
	rcolor[6] = 0.0;
	rcolor[7] = 0.0;
	gcolor[0] = 0.0;
	gcolor[1] = 0.0;
	gcolor[2] = 0.0;
	gcolor[3] = 0.0;
	gcolor[4] = 0.2;
	gcolor[5] = 0.4;
	gcolor[6] = 0.7;
	gcolor[7] = 0.9;
	bcolor[0] = 0.0;
	bcolor[1] = 0.0;
	bcolor[2] = 0.0;
	bcolor[3] = 0.0;
	bcolor[4] = 0.2;
	bcolor[5] = 0.4;
	bcolor[6] = 0.7;
	bcolor[7] = 0.9;


	// Vertex shader  ---------------------------------------------------------

	// perspective matrix, row major
	float PV[16], R[9];
	PV[0]=0.385;PV[1]=0.000;PV[2]=0.000;PV[3]=0.000;
	PV[4]=0.000;PV[5]=0.577;PV[6]=0.000;PV[7]=-0.173;
	PV[8]=0.000;PV[9]=0.000;PV[10]=-1.002;PV[11]=2.607;
	PV[12]=0.000;PV[13]=0.000;PV[14]=-0.200;PV[15]=0.721;

	R[0]=0.985;R[1]=0.000;R[2]=0.174;
	R[3]=0.000;R[4]=1.000;R[5]=0.000;
	R[6]=-0.174;R[7]=0.000;R[8]=0.985;


	float vx,vy,vz;
	float tempx,tempy,tempz,tempw;

	for(i = 0; i < Ngeom; i++) {

		// First vertex
		vx = ((float)tx1[i] - 100.0) / 50.0;
		vy = ((float)ty1[i] - 100.0) / 50.0;
		vz = ((float)tz1[i] - 100.0) / 50.0;

		for(j = 0; j < option; j++) {
			tempx = R[0]*vx + R[1]*vy + R[2]*vz;
			tempy = R[3]*vx + R[4]*vy + R[5]*vz;
			tempz = R[6]*vx + R[7]*vy + R[8]*vz;
			vx = tempx;
			vy = tempy;
			vz = tempz;
		}

		tempx = PV[0]*vx + PV[1]*vy + PV[2]*vz + PV[3];
		tempy = PV[4]*vx + PV[5]*vy + PV[6]*vz + PV[7];
		tempz = PV[8]*vx + PV[9]*vy + PV[10]*vz + PV[11];
		tempw = PV[12]*vx + PV[13]*vy + PV[14]*vz + PV[15];
		tx1[i] = (uint8_t) ((tempx/tempw+1.0)*1.0/2.0*96.0 + 1.0 + 0.5);
		ty1[i] = (uint8_t) ((tempy/tempw+1.0)*1.0/2.0*64.0 + 1.0 + 0.5);
		tz1[i] = (uint8_t) ((tempz/tempw+1.0)*1.0/2.0*80.0 + 1.0 + 0.5);


		// Second vertex
		vx = ((float)tx2[i] - 100.0) / 50.0;
		vy = ((float)ty2[i] - 100.0) / 50.0;
		vz = ((float)tz2[i] - 100.0) / 50.0;

		for(j = 0; j < option; j++) {
			tempx = R[0]*vx + R[1]*vy + R[2]*vz;
			tempy = R[3]*vx + R[4]*vy + R[5]*vz;
			tempz = R[6]*vx + R[7]*vy + R[8]*vz;
			vx = tempx;
			vy = tempy;
			vz = tempz;
		}

		tempx = PV[0]*vx + PV[1]*vy + PV[2]*vz + PV[3];
		tempy = PV[4]*vx + PV[5]*vy + PV[6]*vz + PV[7];
		tempz = PV[8]*vx + PV[9]*vy + PV[10]*vz + PV[11];
		tempw = PV[12]*vx + PV[13]*vy + PV[14]*vz + PV[15];
		tx2[i] = (uint8_t) ((tempx/tempw+1.0)*1.0/2.0*96.0 + 1.0 + 0.5);
		ty2[i] = (uint8_t) ((tempy/tempw+1.0)*1.0/2.0*64.0 + 1.0 + 0.5);
		tz2[i] = (uint8_t) ((tempz/tempw+1.0)*1.0/2.0*80.0 + 1.0 + 0.5);

		// Third vertex
		vx = ((float)tx3[i] - 100.0) / 50.0;
		vy = ((float)ty3[i] - 100.0) / 50.0;
		vz = ((float)tz3[i] - 100.0) / 50.0;

		for(j = 0; j < option; j++) {
			tempx = R[0]*vx + R[1]*vy + R[2]*vz;
			tempy = R[3]*vx + R[4]*vy + R[5]*vz;
			tempz = R[6]*vx + R[7]*vy + R[8]*vz;
			vx = tempx;
			vy = tempy;
			vz = tempz;
		}

		tempx = PV[0]*vx + PV[1]*vy + PV[2]*vz + PV[3];
		tempy = PV[4]*vx + PV[5]*vy + PV[6]*vz + PV[7];
		tempz = PV[8]*vx + PV[9]*vy + PV[10]*vz + PV[11];
		tempw = PV[12]*vx + PV[13]*vy + PV[14]*vz + PV[15];
		tx3[i] = (uint8_t) ((tempx/tempw+1.0)*1.0/2.0*96.0 + 1.0 + 0.5);
		ty3[i] = (uint8_t) ((tempy/tempw+1.0)*1.0/2.0*64.0 + 1.0 + 0.5);
		tz3[i] = (uint8_t) ((tempz/tempw+1.0)*1.0/2.0*80.0 + 1.0 + 0.5);
		// printf("tx ty tz %d %d %d, tempz %d, tempw %d\r\n",tx3[i],ty3[i],tz3[i],(int) (100.0*tempz),(int) (100.0*tempw));
	}

	// ---------------------------------------------
	// Generate edgebucket
	// ---------------------------------------------
	uint8_t ebdx[24];
	uint8_t ebdy[24];
	float ebx[24];
	uint8_t ebymax[24];
	uint8_t ebymin[24];
	uint8_t ebs[24];
	uint8_t ebt[24];
	uint8_t AL[24];
	uint8_t tarr[24];

	uint8_t edge_counter = 0;

	uint8_t dx, dy, x1,x2,y1,y2;

	for(i = 0; i < Ngeom; i++) {
		for(j = 0; j < 3; j++) {
			if(j == 0) {
				x1 = tx1[i];
				x2 = tx2[i];
				y1 = ty1[i];
				y2 = ty2[i];
			} else if(j == 1) {
				x1 = tx2[i];
				x2 = tx3[i];
				y1 = ty2[i];
				y2 = ty3[i];
			} else {
				x1 = tx3[i];
				x2 = tx1[i];
				y1 = ty3[i];
				y2 = ty1[i];
			}
			if(x2 - x1 < 0)
				dx = x1 - x2;
			else
				dx = x2 - x1;
			if(y2 - y1 < 0)
				dy = y1 - y2;
			else
				dy = y2 - y1;

			if(dy < 1) continue; // horizontal line


			if(y1 < y2) {
				ebymax[edge_counter] = y2;
				ebymin[edge_counter] = y1;
				ebs[edge_counter] = (x2 - x1) > 0 ? 1 : 0;
				ebx[edge_counter] = x2;
			} else {
				ebymax[edge_counter] = y1;
				ebymin[edge_counter] = y2;
				ebs[edge_counter] = (x1 - x2) > 0 ? 1 : 0;
				ebx[edge_counter] = x1;
			}

			ebt[edge_counter] = i;
			ebdx[edge_counter] = dx;
			ebdy[edge_counter] = dy;
			edge_counter++;
		}
	}
	

	// ---------------------------------------------
	// Scanline loop
	// ---------------------------------------------

    uint8_t yline,y,y0,x,renderIndex, AL_counter;
	AL_counter = 0;

	for(renderIndex = 0;  renderIndex < 64; renderIndex++) {

		// clear image and zbuffer
		for(i = 0; i < N; i++) 
			image[i] = (uint8_t) 0;
	
		y0 = 63 - renderIndex;


		for(yline = 0; yline < 1; yline++) {
			y = y0 - yline;

			// Generate ActiveList -------------------------------------------------
			for(i = 0; i < edge_counter; i++) {
				if(ebymax[i] == y) {
					// printf("MATCH %d\r\n",y);
					tarr[AL_counter] = ebt[i];
					AL[AL_counter] = i;
					AL_counter++;
					// printf("incrementing al counter\n");
				}
			}

			// Proceed?
			if(AL_counter == 0) continue;

			// Sort ActiveList -----------------------------------------------------
			uint8_t u, v, min, temp;
		
			for (u = 0; u < AL_counter - 1; u++)
			{
				min = u;
				for (v = u + 1; v < AL_counter; v++)
				{
					if (tarr[v] < tarr[min])
					{
						min = v;
					}
				}
				
				temp = AL[u];
				AL[u] = AL[min];
				AL[min] = temp;

				temp = tarr[u];
				tarr[u] = tarr[min];
				tarr[min] = temp;
			}


			// iterate ActiveList ------------------------------------------------------
			for(x = 0; x < 96; x++) {

				float minz = 255;
				for(i = 0; i < AL_counter - 1; i++) {
					
					uint8_t e1 = AL[i];
					uint8_t e2 = AL[i+1];

					if(ebt[e1] != ebt[e2]) 
						continue; // Edges do not belong to the same polygon 
				
					float x1,x2;
					if (ebx[e2] > ebx[e1]) {
						x1 = ebx[e1];
						x2 = ebx[e2];
					} else {
						x1 = ebx[e2];
						x2 = ebx[e1];
					}

					if(x < x1 || x > x2) 
						continue; 

					uint8_t t = ebt[e1];
					float z = triangle_interpolate(tz1[t],tz2[t],tz3[t],tx1[t],ty1[t],tx2[t],ty2[t],tx3[t],ty3[t],x,y);
					
				
					if(z < minz) {
						minz = z;
						setcolor(image,x,yline,rcolor[ebt[e1]],gcolor[ebt[e1]],bcolor[ebt[e1]]);
					}
				}	
			}		

			// update AL and x ----------------------------------------------------------
			uint8_t AL_counter_temp = AL_counter;
			AL_counter = 0;
			for(i = 0; i < AL_counter_temp; i++) {
				uint8_t eidx = AL[i];
				if (y-1 >= ebymin[eidx]) {
					tarr[AL_counter] = ebt[eidx];
					AL[AL_counter++] = eidx;
					float slope = ((float)ebdx[eidx]) / ((float) ebdy[eidx]);
					ebx[eidx] -= slope * (-1.0 + 2.0 * (float) ebs[eidx]);
				}
			}

		}
		
		displaywrite(image);

	}
    
	return;
}


void
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	/CS (PTA12) low
	 */
	GPIO_DRV_ClearPinOutput(kGpioOC);
	GPIO_DRV_TogglePinOutput(kGpioDC);


	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);					

	/*
	 *	/CS (PTA12) high
	 */
	GPIO_DRV_SetPinOutput(kGpioOC);
	GPIO_DRV_TogglePinOutput(kGpioDC);

	return;
}



int main(void)
{
	printf("starting...");

	uint8_t i;
	while(1)
		for(i = 0; i < 36; i++) {
			printf("now rotation %d deg \r\n",i*10);
			writedisplay(i);
			printf("returned\n");
		}
   
	printf("finished\r\n");

}

/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include "mex.h" 
#include <cstdio>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{ 
  char *name = mxArrayToString(prhs[0]);
  float sigma = mxGetScalar(prhs[1]);
  float k = mxGetScalar(prhs[2]);
  int min_size = mxGetScalar(prhs[3]);
	
  printf("loading input image.\n");
  image<rgb> *input = loadPPM(name);
	
  printf("processing\n");
  int num_ccs; 
  int *seg = segment_image(input, sigma, k, min_size, &num_ccs); 
  //savePPM(seg, argv[5]);
  
  int width = input->width();
  int height = input->height();
  plhs[0] = mxCreateDoubleMatrix(height,width,mxREAL);
  double *outdata = mxGetPr(plhs[0]);
  for(int i=0; i<height; ++i)
  {
	  for(int j=0; j<width; ++j)
	  {
		  outdata[j*height+i] = seg[i*width+j];
	  }
  }
}


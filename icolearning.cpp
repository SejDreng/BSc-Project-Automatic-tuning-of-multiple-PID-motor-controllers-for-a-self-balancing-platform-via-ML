#include "icolearning.h"

#include <complex>
#include <stdio.h>

/**
 * get a pointer to a filter
 **/
Trace* Icolearning::getFilter(int channel, int filterindex) {
	return iir[channel][filterindex];
}


float Icolearning::getProd(int channel, int index) {
	return delta*getFilter(channel,index)->getActualOutput();
}


void Icolearning::setInput(int i,float f) {
	lastInputs[i]=inputs[i];
	inputs[i]=f;
}



/**
 * init the bandpass filters and set all weights to zero
 **/
Icolearning::Icolearning(int nInputs, 
			 int nFi) {
	logging("ICO init: # of channels = %d, # of filters = %d\n",
		nInputs,nFi);
	nChannels=nInputs;
	inputs=new float[nInputs];
	lastInputs=new float[nInputs];
	for(int i=0;i<nInputs;i++) {
		inputs[i]=0.0;
		lastInputs[i]=0.0;
	}
	nFilters=nFi;
	learningRate=0.00001f; // this should work initially
	delta=0.0F;
	actualActivity=0.0F;
	lastActivity=0.0F;
	logging("Init filters\n");
	iir=new Trace**[nChannels];
	for(int j=0;j<nChannels;j++) {
		iir[j]=new Trace*[nFilters];
		for(int i=0;i<nFilters;i++) {
			iir[j][i]=new Trace();
		}
	}
	logging("Init weights\n");
	weight=new float*[nChannels];
	for(int j=0;j<nChannels;j++) {
		weight[j]=new float[nFilters];
		for(int i=0;i<nFilters;i++) {
			weight[j][i]=0.0;
		}
	}
	logging("Init the last corr\n");
	lastCorrel=new float*[nChannels];
	for(int j=0;j<nChannels;j++) {
		lastCorrel[j]=new float[nFilters];
		for(int i=0;i<nFilters;i++) {
			lastCorrel[j][i]=0.0;
		}
	}
	
	/**
	 * setting the weight of the reflex to a initial level (as a reference)
	 **/
	weight[0][0]=1.0;
	fOutput=NULL;
	fWeights=NULL;
	fFilter=NULL;
	fweightFilter=NULL;
	fProd=NULL;
	doDocu=0;
	onlyChange=0;
}


Icolearning::~Icolearning() {
	if (doDocu) {
		fclose(fOutput);
		fclose(fWeights);
		fclose(fFilter);
		fclose(fweightFilter);
		fclose(fProd);
	}
	delete inputs;
	for(int j=0;j<nChannels;j++) {
		for(int i=0;i<nFilters;i++) {
			delete iir[j][i];
		}
		delete[] iir[j];
	}
	delete[] iir;
	for(int j=0;j<nChannels;j++) {
		delete[] weight[j];
	}
	delete[] weight;
	for(int j=0;j<nChannels;j++) {
		delete[] lastCorrel[j];
	}
	delete lastCorrel;
}



void Icolearning::openDocu(const char* ndocu) {
	char tmp[128];
	sprintf(tmp,"%s.log",ndocu);
	char nWeights[128];
	sprintf(nWeights,"%s_weights.dat",ndocu);
	logging("Opening %s for the weights\n",nWeights);
	fWeights=fopen(nWeights,"wt");
	if (!fWeights) {
		logging("Could not open docu %s\n",nWeights);
		return;
	}

	char nProd[128];
	sprintf(nProd,"%s_prod.dat",ndocu);
	logging("Opening %s for the prod uv'\n",nProd);
	fProd=fopen(nProd,"wt");
	if (!fProd) {
		logging("Could not open docu %s\n",nProd);
		return;
	}

	char nFilter[128];
	sprintf(nFilter,"%s_filter.dat",ndocu);
	logging("Opening %s for the Filters\n",nFilter);
	fFilter=fopen(nFilter,"wt");
	if (!fFilter) {
		logging("Could not open docu %s\n",nFilter);
		return;
	}

	char weightFilter[128];
	sprintf(weightFilter,"%s_filters_weighted.dat",ndocu);
	logging("Opening %s for the Filters\n",nFilter);
	fweightFilter=fopen(weightFilter,"wt");
	if (!fweightFilter) {
		logging("Could not open docu %s\n",weightFilter);
		return;
	}



	char nOutput[128];
	sprintf(nOutput,"%s_output.dat",ndocu);
	logging("Opening %s for the output\n",nOutput);
	fOutput=fopen(nOutput,"wt");
	if (!fOutput) {
		logging("Cound not open docu %s\n",nOutput);
		return;
	}
	doDocu=1;
}




float Icolearning::calcFrequ(float f0,int i) {
	return f0/((float)i+1.0f);
}



void Icolearning::setReflex(float f0, // frequency
			    float q0  // quality
			    ) {
	getFilter(0,0)->calcCoeffBandp(f0,q0);
	if (doDocu)
	{
		getFilter(0, 0)->impulse("h0.dat");
	}
}

void Icolearning::setPredictorsAsBandp(float f1,
				       float q1
				       ) {
	for(int i=1;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			float fi=calcFrequ(f1,j);
			logging("Bandp channel %d,filter #%d: f=%f, Q=%f\n",
				i,j,fi,q1);
			getFilter(i, j)->calcCoeffBandp(fi, q1);
			if (doDocu)
			{
				char tmp[256];
				sprintf(tmp, "h_t_bandpass_%d_%02d.dat", i, j);
				getFilter(i, j)->impulse(tmp);
			}
		}
	}
}


void Icolearning::setPredictorsAsTraces(int nTaps,
					float tau
					) {
	for(int i=1;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			getFilter(i,j)->calcCoeffTrace((int)(((float)nTaps)/((float)(j+1))),
						       tau/((float)(j+1)));
			if (doDocu)
			{
				char tmp[256];
				sprintf(tmp, "h(t),trace: h%d_%02d.dat", i, j);
				getFilter(i, j)->impulse(tmp);
			}
		}
	}
}



void Icolearning::prediction(int) {
	/**
	 * Feed the inputs into the filter bank
	 **/

	// channel 0 is no filter bank, it's just one filter
	getFilter(0,0)->filter(inputs[0]);

	// feed the input-channels into the filters
	for(int j=1;j<nChannels;j++) {
		// look throught the filter bank
		for(int i=0;i<nFilters;i++) {
			// get an input
			float dd=inputs[j];
			if (onlyChange) {
				// filter out the DC
				dd=inputs[j]-lastInputs[j];
			}
			// feed it into the filter
			getFilter(j,i)->filter(dd);
		}
	}


	/**
	 * calculate the new activity: weighted sum over the filters
	 **/
	// get the reflex
	actualActivity=getFilter(0,0)->getActualOutput();
	// loop through the predictive inputs
	for(int j=1;j<nChannels;j++) {
		for(int i=0;i<nFilters;i++) {
			actualActivity=actualActivity+
				getWeight(j,i)*getFilter(j,i)->getActualOutput();
		}
	}

	/**
	 * calc the difference, i.e. the derivative
	 **/
	delta=getFilter(0,0)->getDeriv();
	lastActivity=actualActivity;
	
	/**
	 * change the weights
	 **/
	// run through different predictive input channels
	for(int j=1;j<nChannels;j++) {
		// run through the filter bank
		for(int i=0;i<nFilters;i++) {
			// ICO learning
			float correl=delta*getFilter(j,i)->getActualOutput();
			// calculate the integral by a linear approximation
			float integral=correl-(correl-lastCorrel[j][i])/2.0f;
			// set the weight
			setWeight(j,i,
				  getWeight(j,i)+learningRate*integral);
			// save the last correlation result. Needed for the
			// integral.
			lastCorrel[j][i]=correl;
		}
	}
}



/**
 * Write gnuplot friendly output
 **/
void Icolearning::writeDocu(int step) {
	if (!doDocu) return;

	fprintf(fOutput,"%d %f %f\n",step,actualActivity,delta);
	
	// weights
	fprintf(fWeights,"%d",step);
	for(int i=0;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			fprintf(fWeights," %f",getWeight(i,j));
		}
	}
	fprintf(fWeights,"\n");
	fflush(fWeights);

	// weights
	fprintf(fProd,"%d",step);
	for(int i=0;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			fprintf(fProd," %f",getProd(i,j));
		}
	}
	fprintf(fProd,"\n");
	fflush(fProd);

	// Filter outputs
	fprintf(fFilter,"%d",step);
	for(int i=0;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			fprintf(fFilter," %f",getFilter(i,j)->getActualOutput());
		}
	}
	fprintf(fFilter,"\n");
	fflush(fFilter);

	// weighted Filter outputs
	fprintf(fweightFilter,"%d",step);
	for(int i=0;i<nChannels;i++) {
		for(int j=0;j<nFilters;j++) {
			fprintf(fweightFilter," %f",
				(getFilter(i,j)->getActualOutput())*getWeight(i,j)
				);
		}
	}
	fprintf(fweightFilter,"\n");
	fflush(fweightFilter);
}

#include "icolearning.h"
#include "trace.h"
#include <cstring>

int trace=0;

void icoLearningWithOneFilter() {
	// Two inputs: reflex and predictor
	// Just one filter in the filterbank
	Icolearning icolearning(2,1);
	icolearning.setDebugMessages(true);
	icolearning.setLearningRate(0.000001f);
	icolearning.setReflex(0.01f,0.6f);
	if (trace)
		icolearning.setPredictorsAsTraces(50);
	else
		icolearning.setPredictorsAsBandp(0.01f,0.6f);
	icolearning.openDocu("onefilter");
	for(int step=0;step<150000;step++) {
		if (step%1000==500) {
			icolearning.setInput(1,1);
		} else {
			icolearning.setInput(1,0);
		}
		if ((step%1000==525)&&(step<100000)) {
			icolearning.setInput(0,1);
		} else {
			icolearning.setInput(0,0);
		}
		icolearning.prediction(step);
		icolearning.writeDocu(step);
	}
}


void icoLearningWith10Filters() {
	// Two inputs: reflex and predictor
	// Just one filter in the filterbank
	Icolearning icolearning(2,10);
	icolearning.setLearningRate(0.0001f);
	icolearning.setDebugMessages(true);

	icolearning.setReflex(0.01f,0.6f);
	if (trace)
		icolearning.setPredictorsAsTraces(50);
	else
		icolearning.setPredictorsAsBandp(0.1f,0.6f);
	icolearning.openDocu("ten_filters");
	for(int step=0;step<150000;step++) {
		if (step%1000==500) {
			icolearning.setInput(1,1);
		} else {
			icolearning.setInput(1,0);
		}
		if ((step%1000==525)&&(step<100000)) {
			icolearning.setInput(0,1);
		} else {
			icolearning.setInput(0,0);
		}
		icolearning.prediction(step);
		if (step%10==0) {
			icolearning.writeDocu(step);
		}
	}
}




void stdpWithOneFilter() {
	FILE* f=fopen("stdp.dat","wt");
	for(int t=-200;t<=200;t++) {
		// Two inputs: reflex and predictor
		// Just one filter in the filterbank
		Icolearning icolearning(2,1);
		icolearning.setLearningRate(0.000001f);
		icolearning.setReflex(0.01f,0.6f);
		if (trace)
			icolearning.setPredictorsAsTraces(50);
		else
			icolearning.setPredictorsAsBandp(0.01f,0.6f);
		for(int step=0;step<150000;step++) {
			if (step%1000==500) {
				icolearning.setInput(1,1);
			} else {
				icolearning.setInput(1,0);
			}
			if ((step%1000==(500+t))&&(step<100000)) {
				icolearning.setInput(0,1);
			} else {
				icolearning.setInput(0,0);
			}
			icolearning.prediction(step);
		}
		fprintf(f,"%d",t);
		for(int i=0;i<icolearning.getNchannels();i++) {
			for(int j=0;j<icolearning.getNfilters();j++) {
				fprintf(f," %e",
					icolearning.getWeight(i,j));
			}
		}
		fprintf(f,"\n");
	}
	fclose(f);
	fprintf(stderr,"\n");
}



int main() {
	int demoNumber=10;
	fprintf(stderr, "Demo #%d\n", demoNumber);
	switch (demoNumber) {
	case 0:
		fprintf(stderr,"ICO learning with one filter in the predictive pathway.\n");
		icoLearningWithOneFilter();
		break;

	case 1:
		fprintf(stderr,"STDP curve calc for ICO learning with one filter in the predictive pathway.\n");
		stdpWithOneFilter();
		break;

	case 10:
		fprintf(stderr,"ICO learning with ten filters in the predictive pathway.\n");
		icoLearningWith10Filters();
		break;

	}

}

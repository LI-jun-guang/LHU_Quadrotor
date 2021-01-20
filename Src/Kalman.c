#include "Kalman.h"
void Init(struct Kalman* klm){
     
     klm->Q_angle = 0.000008; 
     klm->Q_bias =  0.0000000003; //0.0000000003
	
     klm->R_measure = 0.03; //0.03

     klm->angle = 0; // Reset the angle
     klm->bias = 0; // Reset bias
 
     klm->P[0][0] = 0; 
     klm->P[0][1] = 0;
     klm->P[1][0] = 0;
     klm->P[1][1] = 0;
  }

  double getAngle(struct Kalman * klm, double newAngle, double newRate, double dt) {
      
      klm->rate = newRate - klm->bias;
      klm->angle += dt * klm->rate;
	  
	    klm->P[0][0] += dt * klm->Q_angle + (dt*(klm->P[0][1] + klm->P[1][0] )); 
      klm->P[0][1] -= dt * klm->P[1][1];
      klm->P[1][0] -= dt * klm->P[1][1];
      klm->P[1][1] += klm->Q_bias * dt;
     
      klm->S = klm->P[0][0] + klm->R_measure;
     
      klm->K[0] = klm->P[0][0] / klm->S;
      klm->K[1] = klm->P[1][0] / klm->S;
      
      klm->y = newAngle - klm->angle;
      
      klm->angle += klm->K[0] * klm->y;
      klm->bias += klm->K[1] * klm->y;
      
      klm->P[0][0] -= klm->K[0] * klm->P[0][0];
      klm->P[0][1] -= klm->K[0] * klm->P[0][1];
      klm->P[1][0] -= klm->K[1] * klm->P[0][0];
      klm->P[1][1] -= klm->K[1] * klm->P[0][1];
     
     return klm->angle;
  }
  
  void setAngle(struct Kalman* klm, double newAngle) { klm->angle = newAngle; } // Used to set angle, this should be set as the starting angle
  double getRate(struct Kalman* klm) { return klm->rate; } // Return the unbiased rate
  
  void setQangle(struct Kalman* klm, double newQ_angle) { klm->Q_angle = newQ_angle; }
  void setQbias(struct Kalman* klm, double newQ_bias) { klm->Q_bias = newQ_bias; }
  void setRmeasure(struct Kalman* klm, double newR_measure) { klm->R_measure = newR_measure; }
 
  double getQangle(struct Kalman* klm) { return klm->Q_angle; }
  double getQbias(struct Kalman* klm) { return klm->Q_bias; }
  double getRmeasure(struct Kalman* klm) { return klm->R_measure; }
  

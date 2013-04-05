#include "navigator.h"
#include <cmath>
const int TURN_LEFT = 0, 
		  TURN_RIGHT = 1,
		  TURN_AROUND = 2;

Navigator::Navigator(){
	vectorCount = 0;
	returningHome = false;
	addVector();
}

Navigator::addVector(){
	vectorCount ++;
	vector[vectorCount] = new Vector;
	vector[vectorCount].distance = 0;
}

int Navigator::recordDistance(int distance){
	if(!returningHome){
		vector[vectorCount].distance += distance;

	}else{
		vector[vectorCount].distance -= distance;
		if(vector[vectorCount].distance <= 0){
			nextVector();
		}
	}
}

int Navigator::recordTurn(int angle){
	if(!returningHome){
		if(cmath.fabs(angle) > 45){
			if( angle > 0){
				vector[vectorCount] = TURN_LEFT;
			}else{
				vector[vectorCount] = TURN_RIGHT;
			}
			//TODO: Record turn-arounds 
			//ALSO: check turn right/left for accuracy 
			addVector();
		}else{
			//If we want to do angle math for the vectors
		}
	}else{

	}
}

Navigator::returnHome(){
	returningHome = true;
}

Navigator::nextVector(){
	if(returningHome && vectorCount > 0){
		vectorCount --;
	}
}

#ifndef Navigator_H
#define Navigator_H


struct Vector {
	int distance;
	int endTurn;
	int averageAngle;
};

class Navigation {
	
public:
	Vector vectors[30];
	Navigation();
	int recordDistance(int distance);
	int recordTurn(int angle);
	void returnHome(); //Switch to ReturnHome mode 
private:
	bool returningHome;
	int vectorCount;
	void addVector();
	void nextVector();
};

#endif
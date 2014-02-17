
#include "MatrixDefs.h"

#ifndef _PI
#define _PI

const numeric Pi = numeric(std::acos(-1.0));

const numeric RadiansToDegrees = numeric(180.0/Pi);
const numeric DegreesToRadians = numeric(Pi/180);

#define RADIAN(X)	((X)*DegreesToRadians)

static numeric GetAngle(numeric cosinus, numeric sinus)
{	
	int sign = (sinus > 0) ? 1 : -1;
	return sign * acos(cosinus);
}

#endif // _PI

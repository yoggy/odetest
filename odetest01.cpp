//
// odetest01.cpp - simple free fall test
//
#include <ode/ode.h>

int main (int argc, char **argv)
{
	dInitODE();

	// setup world
	dWorldID world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);

	// body
	dBodyID ball = dBodyCreate(world);

	// mass
	dMass m;
	dMassSetZero(&m);

	const dReal radius = 0.2;  // 20cm
	const dReal mass   = 1.0;  // 1kg

	dMassSetSphereTotal(&m, mass, radius);
	dBodySetMass(ball, &m);
	dBodySetPosition(ball, 0.0, 0.0, 10); // x=0m, y=0m, z=10m

	// simulation loop (1000 step)
	dReal stepsize = 0.01; // 0.01ms
	for (int i = 0; i < 1000; ++i) {
		dWorldStep(world, 0.01);
		const dReal *pos = dBodyGetPosition(ball);
		const dReal *R   = dBodyGetRotation(ball);

		printf("t=%f, pos=(%f, %f, %f), \n", stepsize * i, pos[0], pos[1], pos[2]);
	}

	// cleanup
	dWorldDestroy(world);
	dCloseODE();

	return 0;
}

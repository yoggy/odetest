//
// odetest02.cpp - simple free fall & bound test
//
#include <ode/ode.h>
#include <ncurses.h>
#include <unistd.h>

dWorldID world;
dJointGroupID contactgroup;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	const int N = 10;
	dContact contact[N];

	int n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

	for (int i = 0; i < n; i++) {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu         = 0.0;
		contact[i].surface.bounce     = 0.7;
		contact[i].surface.bounce_vel = 0.01;
		dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
		dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
	}
}

int main (int argc, char **argv)
{
	initscr();

	dInitODE();

	// setup world
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);

	dSpaceID space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);

	// grand plane (for collision)
	dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);

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

	dGeomID geom = dCreateSphere(space, radius);
	dGeomSetBody(geom, ball);

	// simulation loop (1000 step)
	dReal stepsize = 0.01; // 0.01ms
	for (int i = 0; i < 1000; ++i) {
		dSpaceCollide(space, 0, &nearCallback);

		dWorldStep(world, 0.01);

		dJointGroupEmpty(contactgroup);

		// draw
		erase();

		const dReal *pos = dBodyGetPosition(ball);
		const dReal *R   = dBodyGetRotation(ball);
		mvprintw((int)(12-pos[2]), 5, "*");  // ball

		mvprintw(12, 0, "============================");  // ground

		// draw 
		move(0, 0);
		printw("t=%f, pos=(%f, %f, %f) \n", stepsize * i, pos[0], pos[1], pos[2]);
		refresh();

		usleep(10 * 1000);
	}

	// cleanup
	dWorldDestroy(world);
	dCloseODE();

	endwin();

	return 0;
}

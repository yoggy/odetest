//
// odetest03.cpp - simple joint test
//
#include <ode/ode.h>
#include <ncurses.h>
#include <unistd.h>

dWorldID world;
dJointGroupID contactgroup;

#define BALL_NUM	3

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	const int N = 10;
	dContact contact[N];

	int n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

	for (int i = 0; i < n; i++) {
		contact[i].surface.mode       = dContactBounce;
		contact[i].surface.mu         = 0.0;
		contact[i].surface.bounce     = 0.7;
		contact[i].surface.bounce_vel = 0.01;
		dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
		dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
	}
}

typedef struct Ball_ {
	dBodyID body;
	dGeomID geom;
} Ball;

int main (int argc, char **argv)
{
	initscr();

	dInitODE2(0);

	// setup world
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);
	dWorldSetDamping(world, 1e-4, 1e-5);

	dSpaceID space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(10000);

	// grand plane (for collision)
	dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);

	// mass
	dMass m;
	dMassSetZero(&m);

	const dReal radius = 0.2;  // 20cm
	const dReal mass   = 1.0;  // 1kg

	// balls
	Ball balls[BALL_NUM];

	for (int i = 0; i < BALL_NUM; ++i) {
		balls[i].body = dBodyCreate(world);
		dMassSetSphereTotal(&m, mass, radius);
		dBodySetMass(balls[i].body, &m);
		dBodySetPosition(balls[i].body, 5.0 * i, 0.0, 10); // x=0m, y=0m, z=10m
	
		balls[i].geom = dCreateSphere(space, radius);
		dGeomSetBody(balls[i].geom, balls[i].body);
	}

	// create joint
	dJointID hinges[BALL_NUM];
	for (int i = 0; i < BALL_NUM; ++i) {
		hinges[i] = dJointCreateHinge(world, 0);
	}

	// attach world->balls[0]
	dJointAttach(hinges[0], 0, balls[0].body);
	dJointSetHingeAnchor(hinges[0], 0.0, 0.0, 10.0);
	dJointSetHingeAxis(hinges[0], 0, 1, 0);

	dJointAttach(hinges[1], balls[0].body, balls[1].body);
	dJointSetHingeAnchor(hinges[1], 5.0, 0.0, 10.0);
	dJointSetHingeAxis(hinges[1], 0, 1, 0);

	dJointAttach(hinges[2], balls[1].body, balls[2].body);
	dJointSetHingeAnchor(hinges[2], 10.0, 0.0, 10.0);
	dJointSetHingeAxis(hinges[2], 0, 1, 0);

	// simulation loop (1000 step)
	dReal stepsize = 0.01; // 0.01ms
	for (int count = 0; count < 1000; ++count) {
		dSpaceCollide(space, 0, &nearCallback);

		dWorldQuickStep(world, 0.01);

		dJointGroupEmpty(contactgroup);

		// draw
		erase();

		for (int i = 0; i < BALL_NUM; ++i) {
			const dReal *pos = dBodyGetPosition(balls[i].body);
			const dReal *R   = dBodyGetRotation(balls[i].body);
			mvprintw((int)(20-pos[2]), (10 + pos[0]), "*");  // ball
		
			move(1 + i, 4);
			printw("i=%d, pos=(%f, %f, %f)", i, pos[0], pos[1], pos[2]);
		}

		mvprintw(20, 0, "====================================");  // ground

		// draw 
		move(0, 0);
		printw("t=%f", stepsize * count);
		refresh();

		usleep(10 * 1000);
	}

	// cleanup
	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	endwin();

	return 0;
}

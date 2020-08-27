#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	const char * getIntegrationMethod();
	const char * getGravityState();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void notifyMethodChanged(int integartor);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void drawSpheresLines();

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force, float timeStep);
	void integrateEuler(float timestep);
	void integrateMidpoint(float timestep);
	void integrateLeapFrog(float timeStep);
	void initSystem(int mp, int springs);
	void notifyGravityChanged(bool enabled);


	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	class massPoint {
	public:
		Vec3 position;
		Vec3 newPosition;
		Vec3 velocity;
		Vec3 newVelocity;
		bool isFixed;
		bool isOpen;
	};

	class spring {
	public:
		int p1;
		int p2;
		float length;
		bool isOpen;
	};

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	bool debug;
	bool gravity;

	std::vector<massPoint> massPoints;
	std::vector<spring> springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

};
#endif
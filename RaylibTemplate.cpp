#include "raylib.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>

int main(int argc, char* argv[])
{
	const int screenWidth = 1280;
	const int screenHeight = 800;

	InitWindow(screenWidth, screenHeight, "raylib bullet physics example");
	Camera camera = { 0 };
	camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
	camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
	camera.up = Vector3{ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;

	Vector3 cubePosition0 = { 0.0f, 0.0f, 0.0f };
	Vector3 cubePosition1 = { 0.0f, 0.0f, 0.0f };
	Vector3 cubePosition2 = { 0.0f, 0.0f, 0.0f };
	Vector2 cubeScreenPosition = { 0.0f, 0.0f };

	SetCameraMode(camera, CAMERA_FREE);
	SetTargetFPS(60);              

	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

	/////////////////////////////SUELO? es ESTATICO//////////////////////////////////////////////////////////////
	btCollisionShape* colShape0 = new btBoxShape(btVector3(15.0f, 0.1f, 15.f)); 	// cube1
	btTransform startTransform0;
	startTransform0.setIdentity();
	btScalar mass0(0.f);
	bool isDynamic0 = (mass0 != 0.f);
	btVector3 localInertia0(0, 0, 0);
	if (isDynamic0) colShape0->calculateLocalInertia(mass0, localInertia0);
	startTransform0.setOrigin(btVector3(0, 0, 0));
	btDefaultMotionState* motionstate0 = new btDefaultMotionState(startTransform0);
	// mass, in kg. 0 -> Static object, will never move.
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI0(mass0, motionstate0, colShape0, btVector3(0, 0, 0));
	btRigidBody* body0 = new btRigidBody(rigidBodyCI0);
	dynamicsWorld->addRigidBody(body0);

	/////////////////////////////////ES DINAMICA PORQUE MASA !=0//////////////////////////////////////////////////
	btCollisionShape* colShape1 = new btSphereShape(btScalar(1));
	btTransform startTransform1;
	startTransform1.setIdentity();
	btScalar mass1(1.5f);
	bool isDynamic1 = (mass1 != 0.f); //rigidbody is dynamic if and only if mass is non zero, otherwise static
	btVector3 localInertia1(0, 0, 0);
	if (isDynamic1)
		colShape1->calculateLocalInertia(mass1, localInertia1);
	startTransform1.setOrigin(btVector3(0, 4, 0));
	btDefaultMotionState* motionstate1 = new btDefaultMotionState(startTransform1);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI1(mass1, motionstate1, colShape1, btVector3(0, 0, 0) );
	btRigidBody* body1 = new btRigidBody(rigidBodyCI1);
	dynamicsWorld->addRigidBody(body1);

	//////////////////////ULTIMO RB ////////////////////////////////////////////////////////////////////////////
	btCollisionShape* colShape2 = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f)); 	// cube1
	btTransform startTransform2;
	startTransform2.setIdentity();
	btScalar mass2(1.9f);
	bool isDynamic2 = (mass2 != 0.f); //rigidbody is dynamic if and only if mass is non zero, otherwise static
	btVector3 localInertia2(0, 0, 0);
	if (isDynamic2)
		colShape2->calculateLocalInertia(mass2, localInertia2);
	startTransform2.setOrigin(btVector3(0, 8, 0));
	btDefaultMotionState* motionstate2 = new btDefaultMotionState(startTransform2);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI2(mass2, motionstate2, colShape2, btVector3(0, 0, 0));
	btRigidBody* body2 = new btRigidBody(rigidBodyCI2);
	dynamicsWorld->addRigidBody(body2);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	while (!WindowShouldClose())
	{
		UpdateCamera(&camera);
		dynamicsWorld->stepSimulation(1.f / 60.f, 10); // update physics

		// Calculate cube screen space position (with a little offset to be in top)
		cubeScreenPosition = GetWorldToScreen(Vector3{ cubePosition0.x, cubePosition0.y + 1000.5f, cubePosition0.z }, camera);

		//update physics
		body0->getMotionState()->getWorldTransform(startTransform0);
		cubePosition0 = { body0->getWorldTransform().getOrigin().x(), body0->getWorldTransform().getOrigin().y(), body0->getWorldTransform().getOrigin().z() };
		body1->getMotionState()->getWorldTransform(startTransform1);
		cubePosition1 = { startTransform1.getOrigin().x(), startTransform1.getOrigin().y(), startTransform1.getOrigin().z() };
		body2->getMotionState()->getWorldTransform(startTransform2);
		cubePosition2 = { startTransform2.getOrigin().x(), startTransform2.getOrigin().y(), startTransform2.getOrigin().z() };
		printf("%f\n", float( body2->getWorldTransform().getOrigin().y() ));
		////////////////////////////////////////////////////////////////////////////////
		if (IsKeyDown(KEY_RIGHT)) {

			body1->applyForce(btVector3(-80, 0, 0), btVector3(11, 0, 0));
			printf("Aplica fuerza\n");
		}

		///////////////////DRAWING////////////////////////////////////////////////////////
		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);
		DrawCube(cubePosition0, 30.0f, 0.1f, 30.0f, BLUE);
		DrawCube(cubePosition1, 2.0f, 2.0f, 2.0f, RED);
		DrawCube(cubePosition2, 2.0f, 2.0f, 2.0f, GREEN);
		DrawCubeWires(cubePosition0, 2.0f, 2.0f, 2.0f, MAROON);
		DrawGrid(10, 1.0f);
		EndMode3D();
		DrawText("Physics example using Bullet Physics", (screenWidth - MeasureText("Physics example using Bullet Physics", 20)) / 2, 25, 20, GRAY);
		EndDrawing();
	}

	CloseWindow();  

	return 0;
	///-----stepsimulation_end-----

	//cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	int i = 0;
	for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	delete colShape0;
	delete colShape1;
	delete colShape2;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;

	CloseWindow();      
	return 0;
}
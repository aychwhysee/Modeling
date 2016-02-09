/*--------------------------------------------------------*/
/*  CS-378           Computer Graphics         Harry Cho  */
/*--------------------------------------------------------*/
/*  Assignment 3: Sisyphus - Robot Modeling Program       */
/*--------------------------------------------------------*/

#include <cstdlib>
#include <GL/glut.h>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <ctime>

using namespace std;

// Constants defining the sizes, orbits and angular
// velocities of the heavenly bodies.
#define ORBIT_SLICES 360
#define ORBIT_RINGS 10
#define SUN_RADIUS 0.5
#define CART_RADIUS 0.1
#define WHEEL_INNER_RADIUS 0.015
#define WHEEL_OUTER_RADIUS 0.075
#define CART_ORBIT_RADIUS 2.0
#define WHEEL1_ORBIT_RADIUS 0.2
#define WHEEL2_ORBIT_RADIUS 0.2
#define WHEEL3_ORBIT_RADIUS 0.2
#define WHEEL4_ORBIT_RADIUS 0.2
#define SUN_ROTATION_STEP 90/44
#define CART_REVOLUTION_STEP 90/18
#define CART_ROTATION_STEP 90/18
#define WHEEL1_REVOLUTION_STEP 90/18
#define WHEEL1_ROTATION_STEP 90/18
#define WHEEL2_REVOLUTION_STEP 90/18
#define WHEEL2_ROTATION_STEP 90/18
#define WHEEL3_REVOLUTION_STEP 90/18
#define WHEEL3_ROTATION_STEP 90/18
#define WHEEL4_REVOLUTION_STEP 90/18
#define WHEEL4_ROTATION_STEP 90/18
#define BALL_RADIUS 0.2
#define BALL_ORBIT_RADIUS 0.6
#define BALL_REVOLUTION_STEP 90/18
#define BALL_REVOLUTION_STEP2 45/34
#define PI 3.14159
#define DELAY 0.1

//Constants defining the step increments for changing
//the location and the aim of the camera.
#define EYE_STEP 0.1
#define CEN_STEP 0.1
#define ZOOM_FACTOR 8.0

//Global variables that represent the state of the planetary
//system. Rotation variables change as the object spins on its
//axis. Revolution variables change as the object moves in its
//circular orbit.
int sunRotation = 0;
int cartRevolution = 0, cartRotation = 0;
int wheel1Revolution = 0, wheel1Rotation = 0;
int wheel2Revolution = 0, wheel2Rotation = 0;
int wheel3Revolution = 0, wheel3Rotation = 0;
int wheel4Revolution = 0, wheel4Rotation = 0;
float ballRevolution = 0.0, ballRotation = 0.0;
int distance = ballRevolution - cartRevolution;
int ballRevolution2 = 0;

//Global variables that represent the position of each body
//in cartesian coordinates.
GLdouble sunX, sunY, sunZ;
GLdouble cartX, cartY, cartZ;
GLdouble wheel1X, wheel1Y, wheel1Z;
GLdouble wheel2X, wheel2Y, wheel2Z;
GLdouble wheel3X, wheel3Y, wheel3Z;
GLdouble wheel4X, wheel4Y, wheel4Z;
GLfloat shoulderX = 0, elbowX = 0, wristX = 0;
GLfloat shoulderY = 0, elbowY = 0, wristY = 0;
GLfloat shoulderZ = 0, elbowZ = 0, wristZ = 0;
GLdouble ballX, ballY;

//Global variables for keeping track of the size and shape
//of the window.
int windowHeight, windowWidth;

//Enumerated type and global variable for keeping track
//of the selected operation.
typedef enum {
	POSITION, AIM, ORIENTATION, ZOOM, HOME, TIME, PROJECTION,
	ANIMATE, NOTHING
} operationType;
operationType operation = POSITION;

//Enumerated type and global variable for keeping track
//of the object the user has selected to track with the camera
//and the object the user has selected for the camera to ride.
typedef enum {
	SUN, CART, WHEEL1, WHEEL2, WHEEL3, WHEEL4,
	UPPERARM, FOREARM, HAND, BALL, NONE
} trackingType;
trackingType tracking = NONE;
trackingType riding = NONE;

//Enumerated type and global variable for talking about axies.
typedef enum { X, Y, Z } axisType;
axisType axis = Z;

//Enumerated type and global variable for talking about
//direction of changes in camera position, and aim
//and the direction of time steps and animation.
typedef enum { DOWN, UP } directionType;
directionType direction = UP;

//Enumerated type and global variable for keeping track of
//the type of projection being used.
typedef enum { ORTHOGRAPHIC, PERSPECTIVE } projectionType;
projectionType  projection = PERSPECTIVE;

//Enumerated type and global variable for deciding how to
//display bodies.
typedef enum { WIRE, SOLID } sphereType;
sphereType sphere = WIRE;

typedef enum { SHOULDER, ELBOW, WRIST } jointType;
jointType joint = SHOULDER;

GLUquadricObj *upperArm, *foreArm, *hand;


//Global variables for keeping track of the camera position.
GLdouble xEye = 0.0;
GLdouble yEye = 0.0;
GLdouble zEye = 5.0;

//Global variables for keeping track of the camera aim.
GLdouble xCen = 0.0;
GLdouble yCen = 0.0;
GLdouble zCen = 0.0;

//Global variables for keeping track of the camera orientation.
GLdouble xUp = 0.0;
GLdouble yUp = 1.0;
GLdouble zUp = 0.0;

bool inrange = false;

void multiplyMatrixVector(GLdouble* m, GLdouble* v, GLdouble* w, int size)
//Multiplying a vector by a matrix.
{
	int i, j;
	for (i = 0; i < size; i++)
	{
		GLdouble temp = 0.0;
		for (j = 0; j < size; j++)
			temp += (*(m + i + j*size)) * (*(v + j));
		*(w + i) = temp;
	}
}

void getCurrentLocation(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the location in world coordinates to which the current
//modelview matrix maps the origin.
{
	GLdouble modelviewMatrix[16];
	GLdouble v[4] = { 0.0,0.0,0.0,1.0 };
	GLdouble w[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
	multiplyMatrixVector(modelviewMatrix, v, w, 4);
	*x = w[0] / w[3];
	*y = w[1] / w[3];
	*z = w[2] / w[3];
}


//Initialize the display window.
void init()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
}


//Draw a circle in the z=0 plane the slow but easy way.
void drawCircle(GLdouble radius)
{
	GLdouble theta, delta;
	delta = 2 * PI / ORBIT_SLICES;
	glBegin(GL_LINE_LOOP);
	for (theta = 0; theta < 2 * PI; theta += delta)
		glVertex3f(radius*cos(theta), radius*sin(theta), 0.0);
	glEnd();
}

void gotoCartCoordinates()
{
	glRotatef((GLdouble)cartRevolution, 0.0, 0.0, 1.0);
	glTranslatef(CART_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-cartRevolution, 0.0, 0.0, 1.0);
}

void gotoShoulderCoordinates()
{
	glRotatef((GLfloat)shoulderX, 1.0, 0.0, 0.0);
	glRotatef((GLfloat)shoulderY, 0.0, 1.0, 0.0);
	glRotatef((GLfloat)shoulderZ, 0.0, 0.0, 1.0);
}

void drawUpperArm()
{
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glRotatef((GLfloat)90, 0.0, 1.0, 0.0);
	gluCylinder(upperArm, 0.1, 0.1, 1.0, 8, 1);
	glPopMatrix();
}

void gotoElbowCoordinates()
{
	glTranslatef(1.0, 0.0, 0.0);
	glRotatef((GLfloat)elbowX, 1.0, 0.0, 0.0);
	glRotatef((GLfloat)elbowY, 0.0, 1.0, 0.0);
	glRotatef((GLfloat)elbowZ, 0.0, 0.0, 1.0);
}

void drawForeArm()
{
	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0);
	glRotatef((GLfloat)90, 0.0, 1.0, 0.0);
	gluCylinder(foreArm, 0.1, 0.1, 1.0, 8, 1);
	glPopMatrix();
}

void gotoWristCoordinates()
{
	glTranslatef(1.0, 0.0, 0.0);
	glRotatef((GLfloat)wristX, 1.0, 0.0, 0.0);
	glRotatef((GLfloat)wristY, 0.0, 1.0, 0.0);
	glRotatef((GLfloat)wristZ, 0.0, 0.0, 1.0);
}

void drawHand()
{
	glPushMatrix();
	glColor3f(1.0, 1.0, 0.0);
	glRotatef((GLfloat)90, 0.0, 1.0, 0.0);
	gluCylinder(hand, 0.1, 0.1, 1.0, 8, 1);
	glPopMatrix();
}

void drawRobotArm()
{
	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glScaled(0.2, 0.4, 0.2);
	gotoShoulderCoordinates();
	drawUpperArm();
	gotoElbowCoordinates();
	drawForeArm();
	gotoWristCoordinates();
	drawHand();
	glPopMatrix();
}

void drawWheel1()
{
	glRotatef((GLdouble) 90.0, 0.0, 1.0, 0.0);
	glTranslatef((GLdouble) 0.1, 0.12, -0.1);
	if (sphere == WIRE)
		glutWireTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
	else glutSolidTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
}

void gotoWheel1Coordinates()
{
	glRotatef((GLdouble)wheel1Revolution, 0.0, 0.0, 1.0);
	glTranslatef(WHEEL1_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-wheel1Revolution, 0.0, 0.0, 1.0);
	//gotoCartCoordinates();
	//glRotatef((GLdouble) 90.0, 0.0, 1.0, 0.0);
	//glTranslatef((GLdouble) 0.1, 0.12, -0.1);
}

void drawWheel1System()
{
	glPushMatrix();
	gotoWheel1Coordinates();
	drawWheel1();
	glPopMatrix();
}

//void drawWheel1Orbit()
//{
//   glColor3f (0.0, 1.0, 0.0);
//   drawCircle(WHEEL1_ORBIT_RADIUS);
//}

void drawWheel2()
{
	glRotatef((GLdouble) 90.0, 0.0, 0.0, 1.0);
	glTranslatef(0.01, 0.01, 0.2);
	if (sphere == WIRE)
		glutWireTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
	else glutSolidTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
}

void gotoWheel2Coordinates()
{
	glRotatef((GLdouble)wheel2Revolution, 0.0, 0.0, 1.0);
	glTranslatef(WHEEL2_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-wheel2Revolution, 0.0, 0.0, 1.0);
}

void drawWheel2System()
{
	glPushMatrix();
	gotoWheel2Coordinates();
	drawWheel2();
	glPopMatrix();
}

//void drawWheel2Orbit()
//{
//   glColor3f (1.0, 0.0, 0.0);
//   drawCircle(WHEEL2_ORBIT_RADIUS);
//}

void drawWheel3()
{
	glRotatef((GLdouble) 90.0, 0.0, 0.0, 1.0);
	glTranslatef(0.001, 0.2, -0.2);
	if (sphere == WIRE)
		glutWireTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
	else glutSolidTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
}

void gotoWheel3Coordinates()
{
	glRotatef((GLdouble)wheel3Revolution, 0.0, 0.0, 1.0);
	glTranslatef(WHEEL3_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-wheel3Revolution, 0.0, 0.0, 1.0);
}

void drawWheel3System()
{
	glPushMatrix();
	gotoWheel3Coordinates();
	drawWheel3();
	glPopMatrix();
}

//void drawWheel3Orbit()
//{
//   glColor3f (0.0, 1.0, 0.0);
//   drawCircle(WHEEL1_ORBIT_RADIUS);
//}

void drawWheel4()
{
	glRotatef((GLdouble) 90.0, 0.0, 0.0, 1.0);
	glTranslatef(0.01, -0.01, 0.2);
	if (sphere == WIRE)
		glutWireTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
	else glutSolidTorus(WHEEL_INNER_RADIUS, WHEEL_OUTER_RADIUS, 50, 300);
}

void gotoWheel4Coordinates()
{
	glRotatef((GLdouble)wheel4Revolution, 0.0, 0.0, 1.0);
	glTranslatef(WHEEL4_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-wheel4Revolution, 0.0, 0.0, 1.0);
}

void drawWheel4System()
{
	glPushMatrix();
	gotoWheel4Coordinates();
	drawWheel4();
	glPopMatrix();
}

//void drawWheel4Orbit()
//{
//   glColor3f (0.0, 1.0, 0.0);
//   drawCircle(WHEEL1_ORBIT_RADIUS);
//}

void drawCart()
{
	glPushMatrix();
	glRotatef((GLdouble)cartRotation, 0.0, 0.0, 1.0);
	glTranslatef((GLdouble) 0.0, 0.0, 0.2);
	if (sphere == WIRE)
		glutWireCube(CART_RADIUS * 2);
	else glutSolidCube(CART_RADIUS * 2);
	drawRobotArm();
	drawWheel1();
	drawWheel2();
	drawWheel3();
	drawWheel4();
	glPopMatrix();
}

void drawCart2()
{
	glPushMatrix();
	glRotatef((GLdouble)cartRotation, 1.0, 0.0, 0.0);
	if (sphere == WIRE)
		glutWireCube(CART_RADIUS * 2);
	else glutSolidCube(CART_RADIUS * 2);
	drawRobotArm();
	drawWheel1();
	drawWheel2();
	drawWheel3();
	drawWheel4();
	glPopMatrix();
}

void drawBall()
{
	glTranslatef((GLdouble) 0.0, 0.0, 0.2);
	if (sphere == WIRE)
		glutWireSphere(BALL_RADIUS, 10, 8);
	else glutSolidSphere(BALL_RADIUS, 10, 8);
}

void gotoBallCoordinates()
{
	glRotatef((GLdouble)ballRevolution, 0.0, 0.0, 1.0);
	glTranslatef(CART_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-ballRevolution, 0.0, 0.0, 1.0);
}

void gotoBallCoordinates2()
{
	glRotatef((GLdouble)ballRevolution, 0.0, 0.0, 1.0);
	glTranslatef(BALL_ORBIT_RADIUS, 0.0, 0.0);
	glRotatef((GLdouble)-ballRevolution, 0.0, 0.0, 1.0);
}

void someFuctionTob()
{
	GLdouble modelViewMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
	glLoadIdentity();
	gotoBallCoordinates();
	GLdouble Tob = *modelViewMatrix;
}

void drawCartSystem()
{
	glPushMatrix();
	gotoCartCoordinates();
	//drawRobotArm();
	drawCart();
	//drawWheel1System();
	//drawWheel2System();
	//drawWheel3System();
	//drawWheel4System();
	//drawMoon1Orbit();
	//drawMoon2Orbit();
	glPopMatrix();
}

void drawBallSystem()
{
	glPushMatrix();
	gotoBallCoordinates();
	drawBall();
	glPopMatrix();
}

void drawBallSystem2()
{
	glPushMatrix();
	gotoBallCoordinates2();
	drawBall();
	glPopMatrix();
}

void drawCartSystem2()
{
	glPushMatrix();
	gotoCartCoordinates();
	drawCart();
	drawBallSystem2();
	glPopMatrix();
}


void drawCartOrbit()
{
	glColor3f(0.0, 0.0, 1.0);
	drawCircle(CART_ORBIT_RADIUS);
}


void drawSun()
{
	glPushMatrix();
	glColor3f(1.0, 1.0, 0.0);
	glRotatef((GLdouble)sunRotation, 0.0, 0.0, 1.0);
	if (sphere == WIRE)
		glutWireSphere(SUN_RADIUS, 20, 16);
	else glutSolidSphere(SUN_RADIUS, 20, 16);
	glPopMatrix();
}

void getWheel1Position(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the moon1.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoWheel1Coordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getWheel2Position(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the moon2.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoWheel2Coordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getWheel3Position(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the moon1.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoWheel3Coordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getWheel4Position(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the moon1.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoWheel4Coordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getCartPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the cart.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getUpperArmPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the cart.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoShoulderCoordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getForearmPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the cart.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoElbowCoordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getHandPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the cart.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoCartCoordinates();
	gotoWristCoordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getBallPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the cart.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gotoBallCoordinates();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void getSunPosition(GLdouble* x, GLdouble* y, GLdouble* z)
//Finding the current position of the sun.
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	getCurrentLocation(x, y, z);
	glPopMatrix();
}

void drawSolarSystem()
{
	//drawSun();
	drawCartOrbit();
	drawCartSystem();
	drawBallSystem();
}

void drawSolarSystem2()
{
	drawCartOrbit();
	drawCartSystem2();
}


void display()
//Callback for redisplaying the image.
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Set the camera position, aim and orientation.
	glLoadIdentity();
	gluLookAt(xEye, yEye, zEye, xCen, yCen, zCen, xUp, yUp, zUp);

	//Set the projection type and clipping planes.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection == ORTHOGRAPHIC)
		glOrtho(-3.0, 3.0, -3.0, 3.0, -50.0, 50.0);
	else  gluPerspective(60.0, (GLdouble)windowWidth / (GLdouble)windowHeight,
		0.1, 200.0);
	glMatrixMode(GL_MODELVIEW);

	//Draw the solar system.
	drawSolarSystem();

	glutSwapBuffers();

	glFlush();
}

void reshape(int w, int h)
//Callback for responding to reshaping of the display window.
{
	windowWidth = w;
	windowHeight = h;

	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection == ORTHOGRAPHIC)
		glOrtho(-3.0, 3.0, -3.0, 3.0, -3.0, 3.0);
	else  gluPerspective(60.0, (GLdouble)w / (GLdouble)h, 1.0, 200.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void homePosition()
//Putting the solar system and all viewing varibles back to their
//original states.
{
	axis = Z;
	direction = UP;
	operation = POSITION;
	tracking = NONE;
	riding = NONE;
	sunRotation = 0;
	cartRevolution = 0;
	cartRotation = 0;
	wheel1Revolution = 0;
	wheel1Rotation = 0;
	wheel2Revolution = 0;
	wheel2Rotation = 0;
	wheel3Revolution = 0;
	wheel3Rotation = 0;
	wheel4Revolution = 0;
	wheel4Rotation = 0;
	ballRevolution = -22.5;
	//ballRevolution2 = 45;
	ballRotation = 0;
	xEye = 0.0;
	yEye = 0.0;
	zEye = 5.0;
	xCen = 0.0;
	yCen = 0.0;
	zCen = 0.0;
	xUp = 0.0;
	yUp = 1.0;
	zUp = 0.0;
	sunX = sunY = 0.0;
	sunZ = 0.0;
	cartX = CART_ORBIT_RADIUS;
	cartY = 0.0;
	cartZ = WHEEL_OUTER_RADIUS;
	wheel1X = CART_ORBIT_RADIUS - (CART_RADIUS / 2);
	wheel1Y = CART_RADIUS / 2;
	wheel1Z = 0.0;
	wheel2X = CART_ORBIT_RADIUS - (CART_RADIUS / 2);
	wheel2Y = -CART_RADIUS / 2;
	wheel2Z = 0.0;
	wheel3X = CART_ORBIT_RADIUS + (CART_RADIUS / 2);
	wheel3Y = CART_RADIUS / 2;
	wheel3Z = 0.0;
	wheel4X = CART_ORBIT_RADIUS + (CART_RADIUS / 2);
	wheel4Y = -CART_RADIUS / 2;
	wheel4Z = 0.0;
	joint = SHOULDER;
	shoulderX = 0; elbowX = 0; wristX = 0;
	shoulderY = 0; elbowY = 0; wristY = 0;
	shoulderZ = 0; elbowZ = 0; wristZ = 0;
	ballX = ballY = CART_ORBIT_RADIUS;
	distance = ballRevolution - cartRevolution;
	glutPostRedisplay();
}

void track()
//Setting global variables that define an point at which the
//camera is aimed.
{
	switch (tracking)
	{
	case SUN: getSunPosition(&xCen, &yCen, &zCen); break;
	case CART: getCartPosition(&xCen, &yCen, &zCen); break;
	case WHEEL1: getWheel1Position(&xCen, &yCen, &zCen); break;
	case WHEEL2: getWheel2Position(&xCen, &yCen, &zCen); break;
	case WHEEL3: getWheel3Position(&xCen, &yCen, &zCen); break;
	case WHEEL4: getWheel4Position(&xCen, &yCen, &zCen); break;
	case UPPERARM: getUpperArmPosition(&xCen, &yCen, &zCen); break;
	case FOREARM: getForearmPosition(&xCen, &yCen, &zCen); break;
	case HAND: getHandPosition(&xCen, &yCen, &zCen); break;
	case BALL: getBallPosition(&xCen, &yCen, &zCen); break;
	default: break;
	}
}


void ride()
//Setting global variables that define the location of the camera.
{
	switch (riding)
	{
	case SUN: getSunPosition(&xEye, &yEye, &zEye); break;
	case CART: getCartPosition(&xEye, &yEye, &zEye); break;
	case WHEEL1: getWheel1Position(&xEye, &yEye, &zEye); break;
	case WHEEL2: getWheel2Position(&xEye, &yEye, &zEye); break;
	case WHEEL3: getWheel3Position(&xEye, &yEye, &zEye); break;
	case WHEEL4: getWheel4Position(&xEye, &yEye, &zEye); break;
	case UPPERARM: getUpperArmPosition(&xEye, &yEye, &zEye); break;
	case FOREARM: getForearmPosition(&xEye, &yEye, &zEye); break;
	case HAND: getHandPosition(&xEye, &yEye, &zEye); break;
	case BALL: getBallPosition(&xEye, &yEye, &zEye); break;
	default: break;
	}
}

void inRange()
{
	distance = ballRevolution - cartRevolution;
	if (distance = 15)
	{
		inrange = true;
	}
	else inrange = false;

}

int numSteps = 1;

void timeStep()
//Move forward or backward in time and change state variables
//accordingly.
{
	clock_t t = clock();
	do {} while ((float)(clock() - t) / CLOCKS_PER_SEC < DELAY);

	//inRange();
	//if (!inrange)
	if (numSteps < 66)
	{
		if (direction == UP)
		{
			sunRotation += SUN_ROTATION_STEP;
			cartRevolution += CART_REVOLUTION_STEP;
			cartRotation += CART_ROTATION_STEP;
			wheel1Revolution += WHEEL1_REVOLUTION_STEP;
			wheel1Rotation += WHEEL1_ROTATION_STEP;
			wheel2Revolution += WHEEL2_REVOLUTION_STEP;
			wheel2Rotation += WHEEL2_ROTATION_STEP;
			wheel3Revolution += WHEEL3_REVOLUTION_STEP;
			wheel3Rotation += WHEEL3_ROTATION_STEP;
			wheel4Revolution += WHEEL4_REVOLUTION_STEP;
			wheel4Rotation += WHEEL4_ROTATION_STEP;
		}
		else
		{
			sunRotation -= SUN_ROTATION_STEP;
			cartRevolution -= CART_REVOLUTION_STEP;
			cartRotation -= CART_ROTATION_STEP;
			wheel1Revolution -= WHEEL1_REVOLUTION_STEP;
			wheel1Rotation -= WHEEL1_ROTATION_STEP;
			wheel2Revolution -= WHEEL2_REVOLUTION_STEP;
			wheel2Rotation -= WHEEL2_ROTATION_STEP;
			wheel3Revolution -= WHEEL3_REVOLUTION_STEP;
			wheel3Rotation -= WHEEL3_ROTATION_STEP;
			wheel4Revolution -= WHEEL4_REVOLUTION_STEP;
			wheel4Rotation -= WHEEL4_ROTATION_STEP;
		}
		track();
		ride();
		numSteps++;
	}
	else if (numSteps >= 66 && numSteps < 100)
	{
		cartRotation += CART_ROTATION_STEP;
		gotoBallCoordinates2();
		ballRevolution -= BALL_REVOLUTION_STEP2;
		track();
		ride();
		numSteps++;
	}
	else if (numSteps >= 100 && numSteps < 134)
	{
		cartRotation -= CART_ROTATION_STEP;
		track();
		ride();
		numSteps++;
	}
	else if (numSteps >= 134)
	{
		numSteps = 1;
		track();
		ride();
	}
}

void operate()
//Process the operation that the user has selected.
{
	if (operation == TIME) timeStep();
	else if (operation == POSITION)
		switch (axis)
		{
		case X:
			if (direction == UP) xEye += EYE_STEP;
			else xEye -= EYE_STEP;
			break;
		case Y:
			if (direction == UP) yEye += EYE_STEP;
			else yEye -= EYE_STEP;
			break;
		case Z:
			if (direction == UP) zEye += EYE_STEP;
			else zEye -= EYE_STEP;
			break;
		}
	else if (operation == AIM)
		switch (axis)
		{
		case X:
			if (direction == UP) xCen += CEN_STEP;
			else xCen -= CEN_STEP;
			break;
		case Y:
			if (direction == UP) yCen += CEN_STEP;
			else yCen -= CEN_STEP;
			break;
		case Z:
			if (direction == UP) zCen += CEN_STEP;
			else zCen -= CEN_STEP;
			break;
		}
	else if (operation == ZOOM)
	{
		int sign;
		if (direction == UP) sign = 1; else sign = -1;
		xEye += sign * (xCen - xEye) / ZOOM_FACTOR;
		yEye += sign * (yCen - yEye) / ZOOM_FACTOR;
		zEye += sign * (zCen - zEye) / ZOOM_FACTOR;
	}
	else if (operation == PROJECTION)
		if (direction == UP) projection = ORTHOGRAPHIC;
		else projection = PERSPECTIVE;
		glutPostRedisplay();
}

void stepDisplay()
//Glut idle callback for animation.
{
	timeStep();
	glutPostRedisplay();
}


void keyboard(unsigned char key, int, int)
//Function to support keyboard control of some operations.
{
	switch (key) {
	case 't': direction = DOWN; timeStep(); break;
	case 'T': direction = UP; timeStep(); break;
	case 'x':
		xEye -= EYE_STEP;
		glutPostRedisplay();
		break;
	case 'X':
		xEye += EYE_STEP;
		glutPostRedisplay();
		break;
	case 'y':
		yEye -= EYE_STEP;
		glutPostRedisplay();
		break;
	case 'Y':
		yEye += EYE_STEP;
		glutPostRedisplay();
		break;
	case 'z':
		zEye -= EYE_STEP;
		glutPostRedisplay();
		break;
	case 'Z':
		zEye += EYE_STEP;
		glutPostRedisplay();
		break;
	case 'a':
		xCen -= CEN_STEP;
		glutPostRedisplay();
		break;
	case 'A':
		xCen += CEN_STEP;
		glutPostRedisplay();
		break;
	case 'b':
		yCen -= CEN_STEP;
		glutPostRedisplay();
		break;
	case 'B':
		yCen += CEN_STEP;
		glutPostRedisplay();
		break;
	case 'c':
		zCen -= CEN_STEP;
		glutPostRedisplay();
		break;
	case 'C':
		zCen += CEN_STEP;
		glutPostRedisplay();
		break;
	case 'P':
		projection = PERSPECTIVE;
		glutPostRedisplay();
		break;
	case 'p':
		projection = PERSPECTIVE;
		glutPostRedisplay();
		break;
	case 'O':
		projection = ORTHOGRAPHIC;
		glutPostRedisplay();
		break;
	case 'o':
		projection = ORTHOGRAPHIC;
		glutPostRedisplay();
		break;
	case 'W':
		sphere = WIRE;
		glutPostRedisplay();
		break;
	case 'w':
		sphere = WIRE;
		glutPostRedisplay();
		break;
	case 'S':
		sphere = SOLID;
		glutPostRedisplay();
		break;
	case 's':
		sphere = SOLID;
		glutPostRedisplay();
		break;
	case 27:
		std::exit(0);
		break;
	default:
		break;
	}
}

void mainMenu(int item)
// Callback for processing main menu.
{
	switch (item)
	{
	case 0: operation = ZOOM; break;
	case 1: operation = TIME; break;
	case 2: operation = PROJECTION; break;
	case 3: homePosition(); glutPostRedisplay(); break;
	case 4: std::exit(0);
	}
}


void trackSubMenu(int item)
// Callback for processing camera set aim submenu.
{
	switch (item)
	{
	case 1: tracking = SUN; track(); break;
	case 2: tracking = CART; track(); break;
	case 3: tracking = WHEEL1; track(); break;
	case 4: tracking = WHEEL2; track(); break;
	case 5: tracking = WHEEL3; track(); break;
	case 6: tracking = WHEEL4; track(); break;
	case 7: tracking = UPPERARM; track(); break;
	case 8: tracking = FOREARM; track(); break;
	case 9: tracking = HAND; track(); break;
	case 10: tracking = BALL; track(); break;
	}
	glutPostRedisplay();
}

void rideSubMenu(int item)
// Callback for processing camera set ride submenu.
{
	switch (item)
	{
	case 1: riding = SUN; ride(); break;
	case 2: riding = CART; ride(); break;
	case 3: riding = WHEEL1; ride(); break;
	case 4: riding = WHEEL2; ride(); break;
	case 5: riding = WHEEL3; ride(); break;
	case 6: riding = WHEEL4; ride(); break;
	case 7: riding = UPPERARM; ride(); break;
	case 8: riding = FOREARM; ride(); break;
	case 9: riding = HAND; ride(); break;
	case 10: riding = BALL; ride(); break;
	}
	glutPostRedisplay();
}

void aimSubMenu(int item)
// Callback for processing camera change aim submenu.
{
	operation = AIM;
	tracking = NONE;
	switch (item)
	{
	case 1: axis = X; break;
	case 2: axis = Y; break;
	case 3: axis = Z; break;
	}
}

void positionSubMenu(int item)
// Callback for processing camera position submenu.
{
	operation = POSITION;
	riding = NONE;
	switch (item)
	{
	case 1: axis = X; break;
	case 2: axis = Y; break;
	case 3: axis = Z; break;
	}
}


void orientationSubMenu(int item)
// Callback for processing camera orientation submenu.
{
	switch (item)
	{
	case 1: {xUp = 1.0; yUp = 0.0; zUp = 0.0; break; }
	case 2: {xUp = 0.0; yUp = 1.0; zUp = 0.0; break; }
	case 3: {xUp = 0.0; yUp = 0.0; zUp = 1.0; break; }
	}
	glutPostRedisplay();
}


void animateSubMenu(int item)
// Callback for processing animate submenu.
{
	operation = ANIMATE;
	glutIdleFunc(stepDisplay);
	switch (item)
	{
	case 1: direction = UP; break;
	case 2: direction = DOWN; break;
	}
}


void setMenus()
// Routine for creating menus.
{
	int trackSubMenuCode, rideSubMenuCode;
	int aimSubMenuCode, positionSubMenuCode, orientationSubMenuCode;
	int animateSubMenuCode;

	trackSubMenuCode = glutCreateMenu(trackSubMenu);
	glutAddMenuEntry("Sun", 1);
	glutAddMenuEntry("Cart", 2);
	glutAddMenuEntry("Wheel1", 3);
	glutAddMenuEntry("Wheel2", 4);
	glutAddMenuEntry("Wheel3", 5);
	glutAddMenuEntry("Wheel4", 6);
	glutAddMenuEntry("Upper Arm", 7);
	glutAddMenuEntry("Forearm", 8);
	glutAddMenuEntry("Hand", 9);
	glutAddMenuEntry("Ball", 10);

	rideSubMenuCode = glutCreateMenu(rideSubMenu);
	glutAddMenuEntry("Sun", 1);
	glutAddMenuEntry("Cart", 2);
	glutAddMenuEntry("Wheel1", 3);
	glutAddMenuEntry("Wheel2", 4);
	glutAddMenuEntry("Wheel3", 5);
	glutAddMenuEntry("Wheel4", 6);
	glutAddMenuEntry("Upper Arm", 7);
	glutAddMenuEntry("Forearm", 8);
	glutAddMenuEntry("Hand", 9);
	glutAddMenuEntry("Ball", 10);

	aimSubMenuCode = glutCreateMenu(aimSubMenu);
	glutAddMenuEntry("X Axis", 1);
	glutAddMenuEntry("Y Axis", 2);
	glutAddMenuEntry("Z Axis", 3);

	positionSubMenuCode = glutCreateMenu(positionSubMenu);
	glutAddMenuEntry("X Axis", 1);
	glutAddMenuEntry("Y Axis", 2);
	glutAddMenuEntry("Z Axis", 3);

	orientationSubMenuCode = glutCreateMenu(orientationSubMenu);
	glutAddMenuEntry("X Axis", 1);
	glutAddMenuEntry("Y Axis", 2);
	glutAddMenuEntry("Z Axis", 3);

	animateSubMenuCode = glutCreateMenu(animateSubMenu);
	glutAddMenuEntry("Forward", 1);
	glutAddMenuEntry("Backward", 2);

	glutCreateMenu(mainMenu);
	glutAddSubMenu("Set Body Tracking  ...", trackSubMenuCode);
	glutAddSubMenu("Set Body Riding  ...", rideSubMenuCode);
	glutAddSubMenu("Change Camera Aim ...", aimSubMenuCode);
	glutAddSubMenu("Change Camera Position ...", positionSubMenuCode);
	glutAddSubMenu("Change Camera Orientation ...", orientationSubMenuCode);
	glutAddSubMenu("Animate ...", animateSubMenuCode);
	glutAddMenuEntry("Zoom", 0);
	glutAddMenuEntry("Step", 1);
	glutAddMenuEntry("Change Projection", 2);
	glutAddMenuEntry("Home Position", 3);
	glutAddMenuEntry("Exit", 4);
	glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

void mouse(int button, int state, int, int)
// Routine for processing mouse events.
{
	if (operation == ANIMATE)
	{
		glutIdleFunc(NULL); operation = NOTHING; return;
	}
	if (button == GLUT_LEFT_BUTTON)
		switch (state)
		{
		case GLUT_DOWN: direction = DOWN; operate(); break;
		case GLUT_UP: break;
		}
	else if (button == GLUT_RIGHT_BUTTON)
		switch (state)
		{
		case GLUT_DOWN: direction = UP; operate(); break;
		case GLUT_UP: break;
		}
}




int main(int argc, char** argv)
{
	// Mask floating point exceptions.
	_control87(MCW_EM, MCW_EM);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Sisyphus");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glEnable(GL_DEPTH_TEST);

	upperArm = gluNewQuadric();
	gluQuadricDrawStyle(upperArm, (GLenum)GLU_LINE);
	foreArm = gluNewQuadric();
	gluQuadricDrawStyle(foreArm, (GLenum)GLU_LINE);
	hand = gluNewQuadric();
	gluQuadricDrawStyle(hand, (GLenum)GLU_LINE);

	setMenus();
	homePosition();

	glutMainLoop();
	return 0;
}

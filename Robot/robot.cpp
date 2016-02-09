/*-------------------------------------------------*/
/*  CS-378       Computer Graphics     Tom Ellman  */
/*-------------------------------------------------*/
/*  robot.cpp : Rotating Robot Arm  IN 3-D         */
/*-------------------------------------------------*/

#include <cstdlib>
#include <GL/glut.h>
#include <cfloat>
#include <cstdlib>
using namespace std;

#define RSTEP     5.00

typedef enum { SHOULDER, ELBOW, WRIST } jointType;
typedef enum { X, Y, Z } axisType;
typedef enum { DOWN, UP } directionType;

axisType axis = Z;
directionType direction = UP;
jointType joint = SHOULDER;

GLfloat shoulderX = 0, elbowX = 0, wristX = 0;
GLfloat shoulderY = 0, elbowY = 0, wristY = 0;
GLfloat shoulderZ = 0, elbowZ = 0, wristZ = 0;


GLUquadricObj *upperArm, *foreArm, *hand;


void init()
{
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);
}

void gotoShoulderCoordinates()
{
   glRotatef ((GLfloat) shoulderX, 1.0, 0.0, 0.0);
   glRotatef ((GLfloat) shoulderY, 0.0, 1.0, 0.0);
   glRotatef ((GLfloat) shoulderZ, 0.0, 0.0, 1.0);
}

void drawUpperArm()
{
   glPushMatrix();
   glColor3f (1.0, 0.0, 0.0);
   glRotatef((GLfloat) 90, 0.0, 1.0, 0.0);
   gluCylinder(upperArm,0.1,0.1,1.0,8,1);
   glPopMatrix();
}

void gotoElbowCoordinates()
{
   glTranslatef (1.0, 0.0, 0.0);
   glRotatef ((GLfloat) elbowX, 1.0, 0.0, 0.0);
   glRotatef ((GLfloat) elbowY, 0.0, 1.0, 0.0);
   glRotatef ((GLfloat) elbowZ, 0.0, 0.0, 1.0);
}

void drawForeArm()
{
   glPushMatrix();
   glColor3f (0.0, 1.0, 0.0);
   glRotatef((GLfloat) 90, 0.0, 1.0, 0.0);
   gluCylinder(foreArm,0.1,0.1,1.0,8,1);
   glPopMatrix();
}

void gotoWristCoordinates()
{
   glTranslatef (1.0, 0.0, 0.0);
   glRotatef ((GLfloat) wristX, 1.0, 0.0, 0.0);
   glRotatef ((GLfloat) wristY, 0.0, 1.0, 0.0);
   glRotatef ((GLfloat) wristZ, 0.0, 0.0, 1.0);
}

void drawHand()
{
   glPushMatrix();
   glColor3f (1.0, 1.0, 0.0);
   glRotatef((GLfloat) 90, 0.0, 1.0, 0.0);
   gluCylinder(hand,0.1,0.1,1.0,8,1);
   glPopMatrix();
}

void drawRobotArm()
{
   glPushMatrix();
   gotoShoulderCoordinates();
   drawUpperArm();
   gotoElbowCoordinates();
   drawForeArm();
   gotoWristCoordinates();
   drawHand();
   glPopMatrix();
}


void display()
{
   glClear (GL_COLOR_BUFFER_BIT);
   drawRobotArm();
   glutSwapBuffers();
   glFlush();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef (0.0, 0.0, -5.0);
}

void operate ()
{
 if (joint==SHOULDER)
   switch (axis)
      {
    case X :
      if (direction == UP) shoulderX += RSTEP;
      else shoulderX -= RSTEP;
      break;
    case Y :
      if (direction == UP) shoulderY += RSTEP;
      else shoulderY -= RSTEP;
      break;
    case Z :
      if (direction == UP) shoulderZ += RSTEP;
      else shoulderZ -= RSTEP;
      break;
      }
 if (joint==ELBOW)
   switch (axis)
      {
    case X :
      if (direction == UP) elbowX += RSTEP;
      else elbowX -= RSTEP;
      break;
    case Y :
      if (direction == UP) elbowY += RSTEP;
      else elbowY -= RSTEP;
      break;
    case Z :
      if (direction == UP) elbowZ += RSTEP;
      else elbowZ -= RSTEP;
      break;
      }
 if (joint==WRIST)
   switch (axis)
      {
    case X :
      if (direction == UP) wristX += RSTEP;
      else wristX -= RSTEP;
      break;
    case Y :
      if (direction == UP) wristY += RSTEP;
      else wristY -= RSTEP;
      break;
    case Z :
      if (direction == UP) wristZ += RSTEP;
      else wristZ -= RSTEP;
      break;
      }
  glutPostRedisplay( );
}


void keyboard (unsigned char key, int, int)
{
   switch (key) {
      case 'S':  joint = SHOULDER;    break;
      case 's':  joint = SHOULDER;    break;
      case 'E':	 joint = ELBOW;       break;
      case 'e':	 joint = ELBOW;       break;
      case 'W':	 joint = WRIST;       break;
      case 'w':	 joint = WRIST;       break;
      case 'X':  axis = X; direction = UP; operate();break;
      case 'x':  axis = X; direction = DOWN; operate();break;
      case 'Y':  axis = Y; direction = UP; operate();break;
      case 'y':  axis = Y; direction = DOWN; operate();break;
      case 'Z':  axis = Z; direction = UP; operate();break;
      case 'z':  axis = Z; direction = DOWN; operate();break;
      case 27 :  std::exit(0);  break;
      default :  break;
   }
}

void homePosition()
{
 axis = Z;
 direction = UP;
 joint = SHOULDER;
 shoulderX = 0; elbowX = 0; wristX = 0;
 shoulderY = 0; elbowY = 0; wristY = 0;
 shoulderZ = 0; elbowZ = 0; wristZ = 0;
}


void mainMenu( int item )
// Callback for processing main menu.
{
 switch ( item )
 {
 case 1 : homePosition();glutPostRedisplay(); break;
 case 2 : std::exit( 0 );
 }
}


void axisSubMenu( int item )
// Callback for processing axis submenu.
{
  switch ( item )
  {
    case 1 : axis = X; break;
    case 2 : axis = Y; break;
    case 3 : axis = Z; break;
  }
}

void jointSubMenu( int item )
// Callback for processing rotation submenu.
{
  switch ( item )
  {
    case 1 : joint = SHOULDER; break;
    case 2 : joint = ELBOW; break;
    case 3 : joint = WRIST; break;
  }
}


void setMenus( )
// Routine for creating menus.
{
  int axisSubMenuCode, jointSubMenuCode;

  axisSubMenuCode = glutCreateMenu( axisSubMenu );
  glutAddMenuEntry( "X Axis",      1 );
  glutAddMenuEntry( "Y Axis",      2 );
  glutAddMenuEntry( "Z Axis",      3 );

  jointSubMenuCode = glutCreateMenu( jointSubMenu );
  glutAddMenuEntry( "Shoulder",      1 );
  glutAddMenuEntry( "Elbow",      2 );
  glutAddMenuEntry( "Wrist",      3 );

  glutCreateMenu( mainMenu );
  glutAddSubMenu( "Select Axis ...",  axisSubMenuCode );
  glutAddSubMenu( "Select Joint ...", jointSubMenuCode );
  glutAddMenuEntry( "Home Position",  1 );
  glutAddMenuEntry( "Exit",  2 );
  glutAttachMenu( GLUT_MIDDLE_BUTTON );
}

void mouse( int button, int state, int, int )
// Routine for processing mouse events.
{
   if ( button == GLUT_LEFT_BUTTON )
    switch ( state )
    {
    case GLUT_DOWN : direction = DOWN; operate(); break;
    case GLUT_UP   : break;
    }
   else if ( button == GLUT_RIGHT_BUTTON )
    switch ( state )
    {
    case GLUT_DOWN : direction = UP; operate(); break;
    case GLUT_UP   : break;
    }
}


int main(int argc, char** argv)
{
   // Mask floating point exceptions.
   _control87( MCW_EM, MCW_EM );
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (500, 500);
   glutInitWindowPosition (100, 100);
   glutCreateWindow ("Robot");
   init ();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMouseFunc( mouse );

   upperArm = gluNewQuadric();
   gluQuadricDrawStyle(upperArm,(GLenum)GLU_LINE);
   foreArm = gluNewQuadric();
   gluQuadricDrawStyle(foreArm,(GLenum)GLU_LINE);
   hand = gluNewQuadric();
   gluQuadricDrawStyle(hand,(GLenum)GLU_LINE);

   setMenus();

   glutMainLoop();
   return 0;
}

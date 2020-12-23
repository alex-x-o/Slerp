#define TIMER_INTERVAL 20
#define TIMER_ID 0

#include "transform.hpp"
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

using namespace glm; 

static float animPar;
static float animParSaved;
static bool animAct;

static void onDisplay(void);
static void onReshape(int width, int height);
static void onKeyBoard(unsigned char key, int x, int y);
static void onTimer(int value);

static void drawAxis(void);
static void drawObject(void);
static void drawFirstAndLast(void);

static void initCalculation(void);

typedef struct object
{
	double x, y, z;
	double phi, theta, psi;
	Quaterniond q;
} obj;

static obj first, last, curr;
static std::pair<RowVector3d, double> axisAngle;
static Matrix3d matrixA;
static double tm;

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    glutInitWindowSize(600, 600);
    glutInitWindowPosition(300, 100);
    glutCreateWindow(argv[0]);

    glutDisplayFunc(onDisplay);
    glutReshapeFunc(onReshape);
    glutKeyboardFunc(onKeyBoard);

    animPar = 0;
    animAct = false;
	tm = 7;
	initCalculation();

    glClearColor(.05, .05, .05, 0);
    glutMainLoop();

	/* checking if slerp function is working correctly
	Quaterniond q1(1, 2, 3, 4);
	Quaterniond q2(10, 7, 8, 9);
	Quaterniond q;
	q = q1.slerp(0.5, q2);
    std::cout << "q={" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "}" << std::endl;
	q = slerp(q1, q2, 1, 0.5);
	std::cout << "q={" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "}" << std::endl;
	*/

    return 0;
}

static void onDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(5, 5, 5, 
              0, 0, -5, 
              0, 1, 0);

    glPushMatrix();
		glScalef(5, 5, 5);
        drawAxis();
    glPopMatrix();

    glPushMatrix();
        drawObject();
    glPopMatrix();

    glPushMatrix();
		drawFirstAndLast();
    glPopMatrix();

    glutSwapBuffers();
}

static void onReshape(int width, int height)
{
    glViewport(0, 0, 2*width, 2*height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (float) width / height, 1, 200);
}

static void onKeyBoard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27:
			exit(0);
			break;
		case 'g':
		case 'G':
			if(!animAct)
			{
				animPar = 0;
				animAct = true;
				glutTimerFunc(TIMER_INTERVAL, onTimer, TIMER_ID);
			}
			break;
		case 'c':
		case 'C':
			if(!animAct)
			{
				animPar = animParSaved;
				animAct = true;
				glutTimerFunc(TIMER_INTERVAL, onTimer, TIMER_ID);
			}
			break;
		case 's':
		case 'S':
			animAct = false;
			animParSaved = animPar;
			break;
		case 'r':
		case 'R':
			animAct = false;
			animPar = 0;
			glutPostRedisplay();
			break;
	}
}

static void onTimer(int value)
{
    if (value != TIMER_ID)
        return;

    if (animAct) {
        if(animPar >= tm-0.08){    
            animAct = false;
            glutPostRedisplay();
        }

        animPar += 0.05;
        glutPostRedisplay();
        
        glutTimerFunc(TIMER_INTERVAL, onTimer, TIMER_ID);
    }
}

static void drawAxis(void)
{
	glPushMatrix();
		glBegin(GL_LINES);
			glColor3f(1, .3, .3);
			glVertex3f(0, 0, 0);
			glVertex3f(1, 0, 0);

			glColor3f(.3, 1, .3);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 1, 0);

			glColor3f(.3, .3, 1);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 0, 1);
		glEnd();
	glPopMatrix();
}

static void drawObject(void)
{
	glPushMatrix();
		curr.q = slerp(first.q, last.q, tm, animPar);
		axisAngle = Q2AxisAngle(curr.q);
		matrixA = Rodrigez(axisAngle.first, axisAngle.second);

		// linear center of gravity interpolation
		curr.x = (1 - animPar/tm)*first.x + animPar/tm*last.x;
		curr.y = (1 - animPar/tm)*first.y + animPar/tm*last.y;
		curr.z = (1 - animPar/tm)*first.z + animPar/tm*last.z;

		GLdouble transformMatrix[16] = 
			{ matrixA(0, 0), matrixA(1, 0), matrixA(2, 0), 0,
			  matrixA(0, 1), matrixA(1, 1), matrixA(2, 1), 0,
			  matrixA(0, 2), matrixA(1, 2), matrixA(2, 2), 0,
			  curr.x, curr.y, curr.z, 1 };

		glMultMatrixd(transformMatrix);
		glPushMatrix();
			// changing color depending on the animation parameter
			if(animPar/tm < 0.499)
			{
				glColor3f(1,
					  	  1 - 2 * animPar/tm,
					  	  0 + 2 * animPar/tm);
			}
			else
			{
				glColor3f(1,
					  	  0 + (2 * animPar/tm - 1),
					  	  1 - (2 * animPar/tm - 1));
			}
			glLineWidth(2);
			glScalef(0.3, 0.3, 0.3);
			glutWireTorus(1, 2, 100, 100);
		glPopMatrix();
		drawAxis();
	glPopMatrix();
}

static void drawFirstAndLast(void)
{
	glPushMatrix();
		axisAngle = Q2AxisAngle(first.q);
		matrixA = Rodrigez(axisAngle.first, axisAngle.second);
		GLdouble transformMatrix[16] = 
			{ matrixA(0, 0), matrixA(1, 0), matrixA(2, 0), 0,
			  matrixA(0, 1), matrixA(1, 1), matrixA(2, 1), 0,
			  matrixA(0, 2), matrixA(1, 2), matrixA(2, 2), 0,
			  first.x, first.y, first.z, 1 };
		
		glMultMatrixd(transformMatrix);
		glPushMatrix();
			glColor3f(0.8, 1, 0.4);
			glLineWidth(2);
			glScalef(0.3, 0.3, 0.3);
			glutWireTorus(1, 2, 100, 100);
		glPopMatrix();
		drawAxis();
	glPopMatrix();

	glPushMatrix();
		axisAngle = Q2AxisAngle(last.q);
		matrixA = Rodrigez(axisAngle.first, axisAngle.second);
		GLdouble transformMatrix2[16] = 
			{ matrixA(0, 0), matrixA(1, 0), matrixA(2, 0), 0,
			  matrixA(0, 1), matrixA(1, 1), matrixA(2, 1), 0,
			  matrixA(0, 2), matrixA(1, 2), matrixA(2, 2), 0,
			  last.x, last.y, last.z, 1 };
		
		glMultMatrixd(transformMatrix2);
		glPushMatrix();
			glColor3f(0.8, 1, 0.4);
			glLineWidth(2);
			glScalef(0.3, 0.3, 0.3);
			glutWireTorus(1, 2, 100, 100);
		glPopMatrix();
		drawAxis();
	glPopMatrix();
}

static void initCalculation(void)
{
	// initialization of center of gravity and orientation for the first and last position
	first.x = -1;
	first.y = 0;
	first.z = 1;
	first.phi = radians(60.0);
    first.theta = radians(90.0);
    first.psi = radians(0.0);
	last.x = 1.5;
	last.y = 1;
	last.z = 0;
	last.phi = radians(180.0);
    last.theta = radians(45.0);
    last.psi = radians(90.0);

	// find quaternions of the first and last position based on their center of gravity and orientation
	matrixA = Euler2A(first.phi, first.theta, first.psi);
	axisAngle = AxisAngle(matrixA);
	first.q = AxisAngle2Q(axisAngle.first, axisAngle.second);

	matrixA = Euler2A(last.phi, last.theta, last.psi);
	axisAngle = AxisAngle(matrixA);
	last.q = AxisAngle2Q(axisAngle.first, axisAngle.second);
}

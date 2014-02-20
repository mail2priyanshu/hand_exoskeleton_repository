#include "glwidget.h"
#include <GL/glut.h>
#include <Eigen/Dense>
#include <QDebug>

using namespace Eigen;


GLWidget::GLWidget(QWidget *parent) :
        QGLWidget(parent)
{

    //IndxFinger.Read_Finger_Param_From_File();
    IndxFinger.Update_Joint_Points();
}



void GLWidget::initializeGL(){
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
}

void GLWidget::paintGL(){

    glClear (GL_COLOR_BUFFER_BIT);

    glLoadIdentity ();             /* clear the matrix */

    /* viewing transformation  */
//    gluLookAt (-10.0, 12.0, 1.5,    0.0, 0.0, 0.0,    0, 0.0, 1.0);
    gluLookAt (0.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    glScalef (0.08, 0.08, 0.08);      /* modeling transformation */


    glTranslatef(0.0,0.0,0.0);
    glLineWidth(5);

    float Coordinate_Scale = 20;
    glBegin(GL_LINES);

    //plot XYZ coordinate
    glColor3f(1.0,0.0,0.0);
    glVertex3f(Coordinate_Scale ,0,0);
    glVertex3f(0,0,0);

    glColor3f(0.0,1.0,0.0);
    glVertex3f(0,Coordinate_Scale ,0);
    glVertex3f(0,0,0);

    glColor3f(0.0,0.0,1.0);
    glVertex3f(0 ,0,Coordinate_Scale );
    glVertex3f(0,0,0);

    //    // Draw Metacarpal
    //    glColor3f(0.5,0.5,0.5);
    //    glVertex3fv(IndxFinger.Metacarpal_Point);
    //    glVertex3fv(IndxFinger.MCP_Point);
    //    // Draw Prox Phalx
    //    glColor3f(0.5,0.3,0.0);
    //    glVertex3fv(IndxFinger.MCP_Point);
    //    glVertex3fv(IndxFinger.PIP_Point);
    //    // Draw Mid Phalx
    //    glColor3f(0.0,0.7,0.3);
    //    glVertex3fv(IndxFinger.DIP_Point);
    //    glVertex3fv(IndxFinger.PIP_Point);
    //    // Draw Dis Phalx
    //    glColor3f(0.6,0.1,0.3);
    //    glVertex3fv(IndxFinger.DIP_Point);
    //    glVertex3fv(IndxFinger.Tip_Point);

    glEnd();

    // Draw Metacarpal
    glColor3f(0.37,0.37,0.37);
    renderCylinder_convenient(IndxFinger.Metacarpal_Point[0], IndxFinger.Metacarpal_Point[1], IndxFinger.Metacarpal_Point[2],
                              IndxFinger.MCP_Point[0], IndxFinger.MCP_Point[1], IndxFinger.MCP_Point[2], 2,10);

    // Draw Proximal Phalange
    glColor3f(0.5,0.3,0.0);
    renderCylinder_convenient(IndxFinger.MCP_Point[0], IndxFinger.MCP_Point[1], IndxFinger.MCP_Point[2],
                              IndxFinger.PIP_Point[0], IndxFinger.PIP_Point[1], IndxFinger.PIP_Point[2], 2,10);

    // Draw Middle Phalange
    glColor3f(0.0,0.7,0.3);
    renderCylinder_convenient(IndxFinger.PIP_Point[0], IndxFinger.PIP_Point[1], IndxFinger.PIP_Point[2],
                              IndxFinger.DIP_Point[0], IndxFinger.DIP_Point[1], IndxFinger.DIP_Point[2], 2,10);

    // Draw Distal Phalange
    glColor3f(0.6,0.1,0.3);
    renderCylinder_convenient(IndxFinger.DIP_Point[0], IndxFinger.DIP_Point[1], IndxFinger.DIP_Point[2],
                              IndxFinger.Tip_Point[0], IndxFinger.Tip_Point[1], IndxFinger.Tip_Point[2], 2,10);

    // Draw joints
    double radius_for_MCP_Sphere = 2*log(IndxFinger.Cov_MCP_Flex)+25;
    glColor3f (1.0, 0.0, 1.0);
    glutWireSphere(radius_for_MCP_Sphere,5,5);

    double radius_for_PIP_Sphere = 2*log(IndxFinger.Cov_PIP_Flex)+25;
    glTranslatef(IndxFinger.PIP_Point[0],IndxFinger.PIP_Point[1],IndxFinger.PIP_Point[2]);
    glColor3f (0.3, 0.2, 0.9);
    glutWireSphere(radius_for_PIP_Sphere,5,5);

    double radius_for_DIP_Sphere = 2*log(IndxFinger.Cov_DIP_Flex)+25;
    glTranslatef(-IndxFinger.PIP_Point[0],-IndxFinger.PIP_Point[1],-IndxFinger.PIP_Point[2]);
    glTranslatef(IndxFinger.DIP_Point[0],IndxFinger.DIP_Point[1],IndxFinger.DIP_Point[2]);
    glColor3f (0.8, 0.1, 0.2);
    glutWireSphere(6 ,5,5);
//        glutWireSphere(radius_for_DIP_Sphere ,5,5);


    // Create light components
    float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    float diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
    float specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    float position[] = { -1.5f, 1.0f, -4.0f, 1.0f };

    // Assign created components to GL_LIGHT0
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glFlush ();

}

void GLWidget::resizeGL(int w, int h){
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glFrustum (-1.0, 1.0, -1.0, 1.0, 1.5, 20.0);
    glMatrixMode (GL_MODELVIEW);
    gluPerspective(45, (float)w/h, 0.01, 100.0);
}

void GLWidget::renderCylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions,GLUquadricObj *quadric)
{
    float vx = x2-x1;
    float vy = y2-y1;
    float vz = z2-z1;

    //handle the degenerate case of z1 == z2 with an approximation
    if(vz == 0)
        vz = .0001;

    float v = sqrt( vx*vx + vy*vy + vz*vz );
    float ax = 57.2957795*acos( vz/v );
    if ( vz < 0.0 )
        ax = -ax;
    float rx = -vy*vz;
    float ry = vx*vz;
    glPushMatrix();

    //draw the cylinder body
    glTranslatef( x1,y1,z1 );
    glRotatef(ax, rx, ry, 0.0);
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluCylinder(quadric, radius, radius, v, subdivisions, 1);

    //draw the first cap
    gluQuadricOrientation(quadric,GLU_INSIDE);
    gluDisk( quadric, 0.0, radius, subdivisions, 1);
    glTranslatef( 0,0,v );

    //draw the second cap
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluDisk( quadric, 0.0, radius, subdivisions, 1);
    glPopMatrix();
}

void GLWidget::renderCylinder_convenient(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions)
{
    //the same quadric can be re-used for drawing many cylinders
    GLUquadricObj *quadric=gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    renderCylinder(x1,y1,z1,x2,y2,z2,radius,subdivisions,quadric);
    gluDeleteQuadric(quadric);
}

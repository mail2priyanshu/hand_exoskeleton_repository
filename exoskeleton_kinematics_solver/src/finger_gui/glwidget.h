#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <cfinger.h>


class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0);
    
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void renderCylinder_convenient(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions);
    void renderCylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions,GLUquadricObj *quadric);
    CFinger IndxFinger;

private:

private slots:
//    void get_Finger_Angles();

};

#endif // GLWIDGET_H

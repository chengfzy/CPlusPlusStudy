#include <pangolin/pangolin.h>

using namespace std;
using namespace pangolin;

int main(int argc, char* argv[]) {
    CreateWindowAndBind("Hello Pangolin", 640, 480);
    glEnable(GL_DEPTH_TEST);

    // define projection and initial ModelView matrix
    OpenGlRenderState sCam(ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
                           ModelViewLookAt(0, 0, 5, 0, 0, 0, pangolin::AxisY));  // loop at XY plane
    // ModelViewLookAt(2, -2, 2, 0, 0, 0, pangolin::AxisZ));

    // create interactive view in window
    Handler3D handler(sCam);
    View& dCam = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0 / 480.0).SetHandler(&handler);

    while (!ShouldQuit()) {
        // clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        dCam.Activate(sCam);

        // render OpenGL cube
        glDrawColouredCube();

        // draw origin point
        glPointSize(5);
        glColor3f(0, 0, 0);
        glBegin(GL_POINTS);
        glVertex3f(0, 0, 0);
        glEnd();

        // draw line each axis
        glBegin(GL_LINE);
        glLineWidth(10);
        // red, X
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3d(10, 0, 0);
        // green, Y
        glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 10, 0);
        // blue, Z
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 10);
        glEnd();

        // swap frames and process events
        FinishFrame();
    }

    return 0;
}
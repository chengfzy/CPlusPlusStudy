#include "pangolin/pangolin.h"

using namespace std;
using namespace pangolin;

int main(int argc, char* argv[]) {
    CreateWindowAndBind("Hello Pangolin", 640, 480);
    glEnable(GL_DEPTH_TEST);

    // define projection and initial ModelView matrix
    OpenGlRenderState sCam(ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
                           ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

    // create interactive view in windows
    Handler3D handler(sCam);
    View& dCam = CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0 / 480.0).SetHandler(&handler);

    while (!ShouldQuit()) {
        // clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        dCam.Activate(sCam);

        // render OpenGL cube
        glDrawColouredCube();

        // swap frames and process events
        FinishFrame();
    }

    return 0;
}
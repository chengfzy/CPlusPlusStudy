#include <pangolin/pangolin.h>
#include <iostream>

using namespace std;

void setImageData(unsigned char* imageArray, int size) {
    for (size_t i = 0; i < size; ++i) {
        imageArray[i] = static_cast<unsigned char>(rand() / (RAND_MAX / 255.0));
    }
}

int main(int argc, char* argv[]) {
    // create OpenGL window
    pangolin::CreateWindowAndBind("Multiple Display", 640, 480);

    // 3D mouse handler enabled
    glEnable(GL_DEPTH_TEST);

    // issue specific OpenGL we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // define camera render object(for view / scene browsing)
    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000);
    pangolin::OpenGlRenderState sCam(proj, pangolin::ModelViewLookAt(1, 0.5, -2, 0, 0, 0, pangolin::AxisY));
    pangolin::OpenGlRenderState sCam2(proj, pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 0, pangolin::AxisY));

    // add named OpenGL viewport to window and provide 3D handler
    pangolin::View& dCam1 =
        pangolin::Display("cam1").SetAspect(640.f / 480.f).SetHandler(new pangolin::Handler3D(sCam));
    pangolin::View& dCam2 =
        pangolin::Display("cam2").SetAspect(640.f / 480.f).SetHandler(new pangolin::Handler3D(sCam2));
    pangolin::View& dCam3 =
        pangolin::Display("cam3").SetAspect(640.f / 480.f).SetHandler(new pangolin::Handler3D(sCam));
    pangolin::View& dCam4 =
        pangolin::Display("cam4").SetAspect(640.f / 480.f).SetHandler(new pangolin::Handler3D(sCam2));
    pangolin::View& dImg1 = pangolin::Display("img1").SetAspect(640.f / 480.f);
    pangolin::View& dImg2 = pangolin::Display("img2").SetAspect(640.f / 480.f);

    // LayoutEqual is an EXPERIMENTAL feature - it requires that all sub-displays share the same aspect ratio, placing
    // them in a raster fasion in the viewport so as to maximize display size
    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqual)
        .AddDisplay(dCam1)
        .AddDisplay(dImg1)
        .AddDisplay(dCam2)
        .AddDisplay(dImg2)
        .AddDisplay(dCam3)
        .AddDisplay(dCam4);

    const int width{64};
    const int height{48};
    unsigned char* imageArray = new unsigned char[3 * width * height];
    pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_UNSIGNED_BYTE);

    // default hooks for exiting(ESC) and fullscreen(TAB)
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // generate random image and place in texture memory for display
        setImageData(imageArray, 3 * width * height);
        imageTexture.Upload(imageArray, GL_RGB, GL_UNSIGNED_BYTE);

        glColor3f(1.0, 1.0, 1.0);

        dCam1.Activate(sCam);
        pangolin::glDrawColouredCube();

        dCam2.Activate(sCam2);
        pangolin::glDrawColouredCube();

        dCam3.Activate(sCam);
        pangolin::glDrawColouredCube();

        dCam4.Activate(sCam2);
        pangolin::glDrawColouredCube();

        dImg1.Activate();
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        imageTexture.RenderToViewport();

        dImg2.Activate();
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        imageTexture.RenderToViewport();

        // swap frames and process events
        pangolin::FinishFrame();
    }

    // delete image
    delete[] imageArray;

    return 0;
}
#include <pangolin/pangolin.h>
#include <iostream>

using namespace std;
using namespace pangolin;

struct CustomType {
    CustomType() : x(0), y(0.0f) {}

    CustomType(int x, const float& y, const string& z) : x(x), y(y), z(z) {}

    int x;
    float y;
    string z;
};

std::ostream& operator<<(std::ostream& os, const CustomType& c) {
    os << c.x << " " << c.y << " " << c.z;
    return os;
}

std::istream& operator>>(std::istream& is, CustomType& c) {
    is >> c.x;
    is >> c.y;
    is >> c.z;
    return is;
}

void SampleMethod() { cout << "You typed ctrl-r or pushed reset" << endl; }

int main(int argc, char* argv[]) {
    // load configuration data
    // ParseVarsFile("../../Pangolin03_SimpleDisplay/app.cfg");

    // create OpenGL window in single line
    CreateWindowAndBind("Simple Display", 640, 480);
    // 3D mouse handler requires depth testing to be enables
    glEnable(GL_DEPTH_TEST);

    // define camera render object for view/scene browsing
    OpenGlRenderState sCam(ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                           ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY));

    const int uiWidth{180};
    // add named OpenGL viewport to window and provide 3D handler
    View& dCam = CreateDisplay()
                     .SetBounds(0.0, 1.0, Attach::Pix(uiWidth), 1.0, -640.0f / 480.0f)
                     .SetHandler(new Handler3D(sCam));

    // add named panel and bind to variables beginning 'ui', A panel is just a view with a default layout and input
    // handling
    CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, Attach::Pix(uiWidth));

    // safe and efficient binding of named variables, specialisations mean no conversions take place for exact types and
    // conversions between scalar types are cheap
    Var<bool> aButton("ui.AButton", false, false);
    Var<double> aDouble("ui.ADouble", 3, 0, 5);
    Var<int> anInt("ui.AnInt", 2, 0, 5);
    Var<double> aDoubleLog("ui.logScaleVar", 3, 1, 1e4, true);
    Var<bool> aCheckbox("ui.ACheckbox", false, true);
    Var<int> anIntNoInput("ui.AnIntNoInput", 2);
    Var<CustomType> anyType("ui.SomeType", CustomType(0, 1.2f, "Hello"));
    Var<bool> saveWindow("ui.SaveWindow", false, false);
    Var<bool> saveCube("ui.SaveCube", false, false);
    Var<bool> recordCube("ui.RecordCube", false, false);
    // std::function objects can be used for Var's too, These work great with C++11 closures
    Var<std::function<void()>> reset("ui.reset", SampleMethod);

    // demonstration of how we can register a keyboard hook to alter a Var
    RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', SetVarFunctor<double>("ui.ADouble", 3.5));
    // demonstration of how we can register a keyboard hook to trigger a method
    RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', SampleMethod);

    // default hooks for exiting(ESC) and fullscreen(tab)
    while (!ShouldQuit()) {
        // clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (Pushed(aButton)) {
            cout << "You pushed a button!" << endl;
        }

        // overloading of Var<T> operators allows us to treat them like their wrapped types, e.g.:
        if (aCheckbox) {
            anInt = static_cast<int>(aDouble);
        }
        if (!anyType->z.compare("robot")) {
            anyType = CustomType(1, 2.3f, "Boogie");
        }
        anIntNoInput = anInt;

        if (Pushed(saveWindow)) {
            SaveWindowOnRender("window");
        }
        if (Pushed(saveCube)) {
            SaveWindowOnRender("cube", dCam.v);
        }
        if (Pushed(recordCube)) {
            // DisplayBase().RecordOnRender("ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");
        }

        // activate efficiently by object
        dCam.Activate(sCam);

        // render some stuff
        // glColor3f(1.0, 1.0, 1.0);
        glDrawColouredCube();

        // swamp frames and process events
        FinishFrame();
    }

    return 0;
}
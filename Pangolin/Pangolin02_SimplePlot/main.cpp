#include <pangolin/pangolin.h>
#include <iostream>

using namespace std;
using namespace pangolin;

int main(int argc, char* argv[]) {
    // create OpenGL window in single line
    CreateWindowAndBind("Simple Plot", 640, 480);

    // Data logger object
    DataLog log;

    // optionally and named labels
    vector<string> labels;
    labels.emplace_back("sin(t)");
    labels.emplace_back("cos(t)");
    labels.emplace_back("sin(t) + cos(t)");
    log.SetLabels(labels);

    const float tinc{0.001f};

    // OpenGL view of data, we might have many views of the same data
    Plotter plotter(&log, 0.0f, 4.0f * static_cast<float>(M_PI) / tinc, -2.0f, 2.0f,
                    static_cast<float>(M_PI) / (4.0f * tinc), 0.5f);
    plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
    plotter.Track("$i");

    // add some sample annotations to the plot
    plotter.AddMarker(Marker::Direction::Vertical, -0.5, Marker::Equality::LessThan, Colour::Blue().WithAlpha(0.2f));
    plotter.AddMarker(Marker::Direction::Horizontal, 0.3, Marker::GreaterThan, Colour::Red().WithAlpha(0.2f));
    plotter.AddMarker(Marker::Direction::Horizontal, 0.5, Marker::Equal, Colour::Green().WithAlpha(0.2f));

    DisplayBase().AddDisplay(plotter);

    float t{0};
    // default hooks for exiting(Esc) and fullscreen(tab)
    while (!ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        log.Log(sin(t), cos(t), sin(t) + cos(t));
        t += tinc;

        // render graph, swap frames and process events
        FinishFrame();
    }

    return 0;
}
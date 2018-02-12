#ifndef X11_WINDOW_STATE_MONITOR_H
#define X11_WINDOW_STATE_MONITOR_H

#include <memory>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <mgl2/mgl.h>

class X11Window {
 public:
  X11Window(const size_t graph_quality = 0, const size_t inital_width = 1000,
            const size_t inital_height = 500);

  void render();

  int getKeypress();

  std::shared_ptr<mglGraph> getMGLGraph() const;

  void resizeAndClear();

 private:
  Display *display_;
  Window window_;
  Visual *visual_;
  int screen_;
  GC gc_;

  std::shared_ptr<mglGraph> gr_;

  static constexpr int padding_ = 10;
};

#endif  // X11_WINDOW_STATE_MONITOR_H
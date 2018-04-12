#ifndef X11_WINDOW_STATE_MONITOR_H
#define X11_WINDOW_STATE_MONITOR_H

#include <memory>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/XKBlib.h>
#include <mgl2/mgl.h>
#include <mgl2/fltk.h>

class X11Window {
 public:
  X11Window(const size_t graph_quality = 4);

  void render();

  KeySym getKeypress();

  std::shared_ptr<mglGraph> getMGLGraph() const;

  void resizeAndClear();

 private:
  Display *display_;
  Window window_;
  Visual *visual_;
  GC gc_;

  std::shared_ptr<mglGraph> gr_;

  static constexpr int kPadding = 10;
  static constexpr double kStartingScreenRatio = 0.7;
};

#endif  // X11_WINDOW_STATE_MONITOR_H
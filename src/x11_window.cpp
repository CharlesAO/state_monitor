#include <X11/Xutil.h>

#include "state_monitor/x11_window.h"

X11Window::X11Window(const size_t graph_quality) {

  gr_ = std::make_shared<mglGraph>();
  // a value of 0 enforces a black background but halves cpu usage
  gr_->SetQuality(graph_quality);
  gr_->Alpha(false);
  gr_->Light(false);
  gr_->LoadFont("chorus");

  display_ = XOpenDisplay(NULL);

  const int screen_num = DefaultScreen(display_);
  Screen *screen = XDefaultScreenOfDisplay(display_);

  window_ = XCreateSimpleWindow(display_, DefaultRootWindow(display_), 0, 0,
                                kStartingScreenRatio * XWidthOfScreen(screen),
                                kStartingScreenRatio * XHeightOfScreen(screen),
                                0, BlackPixel(display_, screen_num),
                                BlackPixel(display_, screen_num));

  constexpr unsigned long valuemask = 0;
  XGCValues values;
  gc_ = XCreateGC(display_, window_, valuemask, &values);
  XSetForeground(display_, gc_, BlackPixel(display_, screen_num));
  XSetBackground(display_, gc_, BlackPixel(display_, screen_num));

  visual_ = DefaultVisual(display_, screen_num);

  XSelectInput(display_, window_, KeyPressMask);

  XMapWindow(display_, window_);
  XFlush(display_);

  XClassHint class_hint;
  class_hint.res_name = "State Monitor";
  class_hint.res_class = "State Monitor";

  XSetClassHint(display_, window_, &class_hint);
}

void X11Window::render() {
  constexpr int kBitDepth = 24;  // RGB (don't include A here for some reason)
  constexpr int kPixelBitSize = 32;  // RGBA

  // cast away the const because that never goes wrong (also note that colors
  // come out inverted)
  XImage *ximage = XCreateImage(
      display_, visual_, kBitDepth, ZPixmap, 0,
      reinterpret_cast<char *>(const_cast<unsigned char *>(gr_->GetRGBA())),
      gr_->GetWidth(), gr_->GetHeight(), kPixelBitSize, 0);

  XPutImage(display_, window_, gc_, ximage, 0, 0, kPadding, kPadding,
            gr_->GetWidth(), gr_->GetHeight());

  XFlush(display_);
}

KeySym X11Window::getKeypress() {
  XEvent event;

  if(!XCheckMaskEvent(display_, KeyPressMask, &event)){
    return XK_VoidSymbol;
  }
  while (XCheckMaskEvent(display_, KeyPressMask, &event))
    ;

  KeySym key_pressed = XkbKeycodeToKeysym( display_, event.xkey.keycode, 
                                0, event.xkey.state & ShiftMask ? 1 : 0);

  return key_pressed;
}

std::shared_ptr<mglGraph> X11Window::getMGLGraph() const { return gr_; }

void X11Window::resizeAndClear() {
  XWindowAttributes win_attr;

  Status rc = XGetWindowAttributes(display_, window_, &win_attr);
  gr_->SetSize(std::max(win_attr.width - 2 * kPadding, 1),
               std::max(win_attr.height - 2 * kPadding, 1));
  gr_->Clf();
}

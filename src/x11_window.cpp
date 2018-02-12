#include "state_monitor/x11_window.h"

X11Window::X11Window(const size_t graph_quality, const size_t inital_width,
                     const size_t inital_height) {
  gr_ = std::make_shared<mglGraph>(0);
  // a value of 0 enforces a black background but halves cpu usage
  gr_->SetQuality(graph_quality);
  gr_->Alpha(false);
  gr_->Light(false);
  gr_->SetFontSizePT(5);
  gr_->LoadFont("chorus");

  display_ = XOpenDisplay(NULL);

  screen_ = DefaultScreen(display_);
  window_ = XCreateSimpleWindow(
      display_, DefaultRootWindow(display_), 0, 0, inital_width, inital_height,
      0, BlackPixel(display_, screen_), BlackPixel(display_, screen_));

  constexpr unsigned long valuemask = 0;
  XGCValues values;
  gc_ = XCreateGC(display_, window_, valuemask, &values);
  XSetForeground(display_, gc_, BlackPixel(display_, screen_));
  XSetBackground(display_, gc_, BlackPixel(display_, screen_));

  visual_ = DefaultVisual(display_, screen_);

  XSelectInput(display_, window_, KeyPressMask);

  XMapWindow(display_, window_);
  XFlush(display_);
}

void X11Window::render() {
  constexpr int bit_depth = 24;  // RGB (don't include A here for some reason)
  constexpr int pixel_bit_size = 32;  // RGBA

  // cast away the const because that never goes wrong
  XImage *ximage = XCreateImage(
      display_, visual_, bit_depth, ZPixmap, 0,
      reinterpret_cast<char *>(const_cast<unsigned char *>(gr_->GetRGBA())),
      gr_->GetWidth(), gr_->GetHeight(), pixel_bit_size, 0);

  XPutImage(display_, window_, gc_, ximage, 0, 0, padding_, padding_,
            gr_->GetWidth(), gr_->GetHeight());

  XFlush(display_);
}

int X11Window::getKeypress() {
  int key_pressed = 0;
  XEvent event;
  while (XCheckMaskEvent(display_, KeyPressMask, &event))
    ;
  return event.xkey.keycode;
}

std::shared_ptr<mglGraph> X11Window::getMGLGraph() const { return gr_; }

void X11Window::resizeAndClear() {
  XWindowAttributes win_attr;

  Status rc = XGetWindowAttributes(display_, window_, &win_attr);
  gr_->SetSize(std::max(win_attr.width - 2 * padding_, 1),
               std::max(win_attr.height - 2 * padding_, 1));
}

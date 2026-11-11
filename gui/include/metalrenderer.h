#pragma once

// Native Metal video renderer for macOS (see metalrenderer.mm). The interface is plain C++ so
// qmlmainwindow.cpp can use it; Objective-C types cross it as void pointers.

#include <QSize>
#include <QString>

#include <functional>

struct AVFrame;

class MetalRenderer
{
public:
    enum class VideoMode {
        Normal,
        Stretch,
        Zoom
    };

    struct RenderParams {
        VideoMode video_mode = VideoMode::Normal;
        // log2 zoom like the libplacebo path; -1 fills the drawable
        float zoom_factor = 0.0f;
    };

    struct RenderResult {
        // A drawable was presented
        bool presented = false;
        // ...and it contained a decoded video frame
        bool video_presented = false;
        // Nothing was presented because of back-pressure (no drawable, GPU busy); try again soon
        bool retry = false;
    };

    // Runs fn inside an Objective-C autorelease pool. The render thread is a plain QThread with no
    // pool of its own, so Metal objects Qt Quick autoreleases there are drained per call.
    static void runInAutoreleasePool(const std::function<void()> &fn);

    // Creates the device, queue, pipelines and texture cache. Never waits on the GPU.
    // Returns nullptr and fills error if Metal can't be used.
    static MetalRenderer *create(QString *error);
    // Doesn't wait for the GPU: in-flight command buffers own everything they still use
    ~MetalRenderer();

    MetalRenderer(const MetalRenderer &) = delete;
    MetalRenderer &operator=(const MetalRenderer &) = delete;

    // id<MTLDevice> / id<MTLCommandQueue>, shared with Qt Quick
    void *device() const;
    void *commandQueue() const;

    // GUI thread. view is the window's NSView (QWindow::winId()).
    bool attachToView(void *view, bool vsync, QString *error);
    bool attached() const;
    void setDrawableGeometry(const QSize &pixel_size, qreal scale);
    void setVSync(bool enabled);

    // Any thread. Takes ownership of frame. The newest frame wins: returns true when an older
    // frame that was never rendered got dropped for it.
    bool submitFrame(AVFrame *frame);
    // Any thread. Drops the pending and the displayed frame; the next render shows black.
    void clearFrames();
    bool hasPendingFrame() const;

    // Called on the render thread when a VideoToolbox frame can't be used without a copy
    std::function<void()> on_zero_copy_failure;

    // Render thread. Returns the id<MTLTexture> (RGBA8Unorm) Qt Quick renders the UI into,
    // recreating it for a new size; nullptr on failure.
    void *ensureOverlayTexture(const QSize &pixel_size);
    // Render thread. The UI was rendered into the overlay texture again.
    void noteOverlayRendered();
    // Render thread. Draws the newest frame (or the last one again) plus the UI and presents.
    RenderResult render(const RenderParams &params);

private:
    MetalRenderer();
    struct Private;
    Private *d;
};

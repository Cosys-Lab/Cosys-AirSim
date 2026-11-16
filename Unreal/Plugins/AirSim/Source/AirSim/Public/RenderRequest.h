// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include "RHIGPUReadback.h"
#include <memory>
#include "common/Common.hpp"


class AIRSIM_API RenderRequest
{
public:
    struct RenderParams {
        USceneCaptureComponent2D * const render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;
        bool disable_gamma;

        RenderParams(USceneCaptureComponent2D * render_component_val, UTextureRenderTarget2D* render_target_val, bool pixels_as_float_val, bool compress_val, bool disable_gamma_val)
            : render_component(render_component_val), render_target(render_target_val), pixels_as_float(pixels_as_float_val), compress(compress_val), disable_gamma(disable_gamma_val)
        {
        }
    };
    struct RenderResult {
        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        int width;
        int height;

        msr::airlib::TTimePoint time_stamp;
    };

private:
    // One in-flight GPU->CPU copy of a capture (asynchronous readback, see airsim.AsyncCaptureReadback).
    struct AsyncReadbackItem {
        std::shared_ptr<RenderParams> params;
        std::shared_ptr<RenderResult> result;
        TUniquePtr<FRHIGPUTextureReadback> readback;
        bool done = false;
    };

    static FReadSurfaceDataFlags setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size);
    static void copyReadbackToResult(AsyncReadbackItem& item);
    void pollAsyncReadbacks_RenderThread();

    // render thread only: readbacks of the current request still in flight, and the end-of-frame hook
    // that polls them (registered only while readbacks are pending, removed before signalling)
    TArray<AsyncReadbackItem> pending_readbacks_;
    uint32 readback_frames_waited_ = 0;
    FDelegateHandle readback_poll_handle_;

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

public:
    RenderRequest(UGameViewportClient * game_viewport, std::function<void()>&& query_camera_pose_cb);
    ~RenderRequest();
    // not copyable: owns in-flight GPU readbacks and a viewport hook (and the exported class would
    // otherwise get an implicitly generated copy constructor)
    RenderRequest(const RenderRequest&) = delete;
    RenderRequest& operator=(const RenderRequest&) = delete;

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method);

    void ExecuteTask();
};

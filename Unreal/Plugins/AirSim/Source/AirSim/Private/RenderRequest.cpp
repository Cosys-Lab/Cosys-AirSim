// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CoreDelegates.h"
#include "RHICommandList.h"

// Asynchronous GPU readback for camera captures: instead of a blocking ReadSurfaceData per image (a
// full GPU sync on the render thread), the copy is enqueued and picked up on a later frame, so the
// render thread no longer stalls. simGetImages then returns about a frame later. Off by default.
// Based on a contribution by minhquyminhquy.
static TAutoConsoleVariable<int32> CVarAirSimAsyncCaptureReadback(
    TEXT("airsim.AsyncCaptureReadback"),
    0,
    TEXT("Camera captures: 0 = blocking readback (default), 1 = asynchronous GPU readback."),
    ECVF_RenderThreadSafe);

// frames to wait for an in-flight readback before giving up and returning empty images, instead of
// blocking the API call forever (e.g. after a lost GPU device)
static constexpr uint32 kAsyncReadbackTimeoutFrames = 600;

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), wait_signal_(new msr::airlib::WorkerThreadSignal), game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method)
{
    //TODO: is below really needed?
    for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (!params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->time_stamp = 0;
    }

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (use_safe_method) {
        for (unsigned int i = 0; i < req_size; ++i) {
            if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
                //TODO: below doesn't work right now because it must be running in game thread
                FIntPoint img_size;
                if (!params[i]->pixels_as_float) {
                    //below is documented method but more expensive because it forces flush
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GameThread_GetRenderTargetResource();
                    auto flags = setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    if (params[i]->disable_gamma)flags.SetLinearToGamma(false);
                    rt_resource->ReadPixels(results[i]->bmp, flags);
                }
                else {
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GetRenderTargetResource();
                    setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    rt_resource->ReadFloat16Pixels(results[i]->bmp_float);
                }
            }
        }
    }
    else {
        //wait for render thread to pick up our task
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // capture CameraPose for this frame
                query_camera_pose_cb_();

                // The completion is called immeidately after GameThread sends the
                // rendering commands to RenderThread. Hence, our ExecuteTask will
                // execute *immediately* after RenderThread renders the scene!
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        This->ExecuteTask();
                    });

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                assert(end_draw_handle_.IsValid());
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);
            });

            // while we're still on GameThread, enqueue request for capture the scene!
            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                    params_[i]->render_component->CaptureSceneDeferred();
                }
            }
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
            if (!params[i]->pixels_as_float) {
                if (results[i]->width != 0 && results[i]->height != 0) {
                    results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 3, EAllowShrinking::No);
                    if (params[i]->compress)
                        UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                    else {
                        uint8* ptr = results[i]->image_data_uint8.GetData();
                        for (const auto& item : results[i]->bmp) {
                            *ptr++ = item.R;
                            *ptr++ = item.G;
                            *ptr++ = item.B;
                        }
                    }
                }
            }
            else {
                results[i]->image_data_float.SetNumUninitialized(results[i]->bmp_float.Num());
                float* ptr = results[i]->image_data_float.GetData();
                for (const auto& item : results[i]->bmp_float) {
                    *ptr++ = item.R.GetFloat();
                }
            }
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr && req_size_ > 0) {
        FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
        const bool use_async_readback = CVarAirSimAsyncCaptureReadback.GetValueOnRenderThread() != 0;

        for (unsigned int i = 0; i < req_size_; ++i) {
            if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
                if (rt_resource != nullptr) {
                    const FTextureRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                    FIntPoint size;
                    auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

                    // The readback copies raw texel memory into bmp/bmp_float, so only formats whose layout
                    // matches FColor / FFloat16Color can go async. Since the blocking path reads with
                    // RCM_UNorm and no gamma conversion, the bytes are identical. Other formats fall through.
                    const EPixelFormat pixel_format = rhi_texture->GetFormat();
                    const bool format_supports_async =
                        (!params_[i]->pixels_as_float && pixel_format == PF_B8G8R8A8) ||
                        (params_[i]->pixels_as_float && pixel_format == PF_FloatRGBA);

                    if (use_async_readback && format_supports_async) {
                        AsyncReadbackItem item;
                        item.params = params_[i];
                        item.result = results_[i];
                        item.readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("AirSimCaptureReadback"));

                        RHICmdList.Transition(FRHITransitionInfo(rhi_texture.GetReference(), ERHIAccess::Unknown, ERHIAccess::CopySrc));
                        item.readback->EnqueueCopy(RHICmdList, rhi_texture.GetReference());
                        RHICmdList.Transition(FRHITransitionInfo(rhi_texture.GetReference(), ERHIAccess::CopySrc, ERHIAccess::SRVMask));

                        results_[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
                        pending_readbacks_.Add(MoveTemp(item));
                        continue;
                    }

                    //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
                    //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
                    if (!params_[i]->pixels_as_float) {
                        //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                        RHICmdList.ReadSurfaceData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp,
                            flags);
                    }
                    else {
                        RHICmdList.ReadSurfaceFloatData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp_float,
                            CubeFace_PosX,
                            0,
                            0);
                    }
                }
            }
            results_[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        }

        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;

        if (pending_readbacks_.Num() > 0) {
            // the poller signals once every readback has landed
            readback_frames_waited_ = 0;
            readback_poll_handle_ = FCoreDelegates::OnEndFrameRT.AddRaw(this, &RenderRequest::pollAsyncReadbacks_RenderThread);
        }
        else {
            wait_signal_->signal();
        }
    }
}

void RenderRequest::pollAsyncReadbacks_RenderThread()
{
    bool all_done = true;
    for (AsyncReadbackItem& item : pending_readbacks_) {
        if (item.done)
            continue;
        if (item.readback->IsReady()) {
            copyReadbackToResult(item);
            item.done = true;
        }
        else {
            all_done = false;
        }
    }

    if (!all_done && ++readback_frames_waited_ > kAsyncReadbackTimeoutFrames) {
        UE_LOG(LogTemp, Warning, TEXT("AirSim: camera capture readback timed out after %u frames, returning empty images"), readback_frames_waited_);
        for (AsyncReadbackItem& item : pending_readbacks_) {
            if (!item.done) {
                item.result->width = 0;
                item.result->height = 0;
                item.done = true;
            }
        }
        all_done = true;
    }

    if (all_done) {
        // unhook before signalling: once signalled, the waiting API thread may destroy this request
        FCoreDelegates::OnEndFrameRT.Remove(readback_poll_handle_);
        readback_poll_handle_.Reset();
        pending_readbacks_.Empty();
        wait_signal_->signal();
    }
}

void RenderRequest::copyReadbackToResult(AsyncReadbackItem& item)
{
    int32 row_pitch_pixels = 0;
    const void* src_data = item.readback->Lock(row_pitch_pixels);
    if (src_data == nullptr) {
        item.result->width = 0;
        item.result->height = 0;
        return;
    }

    const int32 width = item.result->width;
    const int32 height = item.result->height;
    const uint8* src = static_cast<const uint8*>(src_data);

    if (!item.params->pixels_as_float) {
        // PF_B8G8R8A8 rows have the memory layout of FColor
        item.result->bmp.SetNumUninitialized(width * height);
        uint8* dst = reinterpret_cast<uint8*>(item.result->bmp.GetData());
        const size_t src_stride = static_cast<size_t>(row_pitch_pixels) * sizeof(FColor);
        const size_t dst_stride = static_cast<size_t>(width) * sizeof(FColor);
        for (int32 row = 0; row < height; ++row)
            FMemory::Memcpy(dst + row * dst_stride, src + row * src_stride, dst_stride);
    }
    else {
        // PF_FloatRGBA rows have the memory layout of FFloat16Color
        item.result->bmp_float.SetNumUninitialized(width * height);
        uint8* dst = reinterpret_cast<uint8*>(item.result->bmp_float.GetData());
        const size_t src_stride = static_cast<size_t>(row_pitch_pixels) * sizeof(FFloat16Color);
        const size_t dst_stride = static_cast<size_t>(width) * sizeof(FFloat16Color);
        for (int32 row = 0; row < height; ++row)
            FMemory::Memcpy(dst + row * dst_stride, src + row * src_stride, dst_stride);
    }

    item.readback->Unlock();
}

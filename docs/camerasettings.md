# Cosys-AirSim Camera settings

The `CaptureSettings` in the settings.json file for either the `CameraDefaults` or specific camera settings determines how different image types such as scene, depth, disparity, surface normals and segmentation views are rendered. 

The Width, Height and FOV settings should be self explanatory. 
 The `ProjectionMode` decides the projection used by the capture camera and can take value "perspective" (default) or "orthographic". If projection mode is "orthographic" then `OrthoWidth` determines width of projected area captured in meters.
To disable the rendering of certain objects on specific cameras or all, use the `IgnoreMarked` boolean setting. This requires to mark individual objects that have to be ignore using an Unreal Tag called _MarkedIgnore_.

Unreal 5 introduces Lumen lightning. Due to the cameras using scene capture components enabling Lumen for them can be costly on performance. Settings have been added specfically for the scene camera to customize the usage of Lumen for Global Illumination and Reflections. 
The `LumenGIEnable` and `LumenReflectionEnable` settings enable or disable Lumen for the camera. The `LumenFinalQuality`(0.25-2) setting determines the quality of the final image. The `LumenSceneDetail`(0.25-4) setting determines the quality of the scene. The `LumenSceneLightningDetail`(0.25-2) setting determines the quality of the lightning in the scene.

Below you can find a list of all available settings and their purpose.
They are settings that are directly transferred to the post processing settings of cameras of which more documentation can be found [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/post-process-effects-in-unreal-engine). 

## General
* **Width**: The width of the captured image in pixels. (Default: 256)
* **Height**: The height of the captured image in pixels. (Default: 144)
* **FOV_Degrees**: The horizontal field of view of the camera in degrees. 
* **ImageType**: The type of image being captured (e.g., scene, depth, etc.). (Default: 0)
* **TargetGamma**: The gamma value applied to the captured image.
* **IgnoreMarked**: Whether to ignore objects marked for a specific purpose (e.g., segmentation). (Default: false)
* **ProjectionMode**: The camera's projection mode ("Perspective" or "Orthographic"). (Default: "Perspective")
* **OrthoWidth**: The width of the orthographic view frustum. 

## Lumen Global Illumination and Reflections
* **LumenGIEnable**: Whether Lumen Global Illumination is enabled. (Default: false)
* **LumenReflectionEnable**: Whether Lumen Reflections are enabled. (Default: false)
* **LumenFinalQuality**: The quality of Lumen's final gather. 
* **LumenSceneDetail**: Controls the size of instances that can be represented in the Lumen Scene.
* **LumenSceneLightningDetail**: The quality of Lumen Scene lighting.

## Camera Settings
* **CameraShutterSpeed**: The camera's shutter speed in seconds.
* **CameraISO**: The camera's sensor sensitivity (ISO).
* **CameraAperture**: The camera's aperture value (f-stop).
* **CameraMaxAperture**: The camera's maximum aperture value (minimum f-stop).
* **CameraNumBlades**: The number of blades in the camera's aperture diaphragm.

## Depth of Field
* **DepthOfFieldSensorWidth**: Width of the camera sensor to assume, in millimeters.
* **DepthOfFieldSqueezeFactor**: Squeeze factor for the depth of field, emulating anamorphic lenses.
* **DepthOfFieldFocalDistance**: Distance at which the depth of field effect should be sharp, in centimeters.
* **DepthOfFieldDepthBlurAmount**: Depth blur in kilometers for 50% (CircleDOF only).
* **DepthOfFieldDepthBlurRadius**: Depth blur radius in pixels at 1920x resolution (CircleDOF only).
* **DepthOfFieldUseHairDepth**: Whether to use hair depth for computing the circle of confusion size.

## Exposure
* **AutoExposureMethod**: Luminance computation method. (0: Histogram, 1: Basic, 2: Manual)
* **AutoExposureCompensation**: Logarithmic adjustment for the exposure. 0: no adjustment, -1: 2x darker, -2: 4x darker, 1: 2x brighter, 2: 4x brighter, ...
* **AutoExposureApplyPhysicalCameraExposure**: Enables physical camera exposure using Shutter Speed, ISO, and Aperture. (Only affects Manual exposure mode)
* **AutoExposureMinBrightness**: Minimum brightness for auto exposure adaptation.
* **AutoExposureMaxBrightness**: Maximum brightness for auto exposure adaptation.
* **AutoExposureSpeedUp**: Speed of exposure adaptation upwards (in f-stops per second).
* **AutoExposureSpeedDown**: Speed of exposure adaptation downwards (in f-stops per second).
* **AutoExposureLowPercent**: The lower percentage for the luminance histogram used in auto exposure.
* **AutoExposureHighPercent**: The higher percentage for the luminance histogram used in auto exposure.
* **AutoExposureHistogramLogMin**: Minimum value for the auto exposure histogram (expressed in Log2(Luminance) or EV100).
* **AutoExposureHistogramLogMax**: Maximum value for the auto exposure histogram (expressed in Log2(Luminance) or EV100).

## Motion Blur
* **MotionBlurAmount**: The strength of motion blur applied to the image. 0: off.
* **MotionBlurMax**: The maximum distortion caused by motion blur, in percent of the screen width. 0: off.
* **MotionBlurTargetFPS**: Defines the target FPS for motion blur. Makes motion blur independent of actual frame rate.

## Bloom
* **BloomIntensity**: The intensity of the bloom effect.
* **BloomThreshold**: The minimum brightness for pixels to contribute to the bloom effect.

## Chromatic Aberration
* **ChromaticAberrationIntensity**: The intensity of chromatic aberration.
* **ChromaticAberrationStartOffset**: A normalized distance to the center of the framebuffer where the chromatic aberration effect takes place.

## Lens Flare
* **LensFlareIntensity**: Brightness scale of the image-based lens flares.
* **LensFlareBokehSize**: Size of the lens blur (Bokeh) used for lens flares, as a percentage of the view width.
* **LensFlareThreshold**: Minimum brightness for lens flares to take effect.

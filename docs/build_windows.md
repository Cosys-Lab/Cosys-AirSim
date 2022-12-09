# Build AirSim on Windows

## Install Unreal Engine
This branch uses a custom version of the Unreal Engine!
- Make sure you are [registered with Epic Games](https://www.unrealengine.com/en-US/ue-on-github). This is required to get source code access for Unreal Engine.

- Clone Unreal in your favorite folder and build it (this may take a while!). **Note**: We only support Unreal 4.24.4 ([Cosys-Lab fork](https://github.com/Cosys-Lab/UnrealEngine.git)) at present.
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://github.com/Cosys-Lab/UnrealEngine.git
   cd UnrealEngine
   ```
 -Visual Studio 2019 is required for building. 
- To install the correct components for UE4 development, check the "Game Development with C++" workload and the “.net 4.6.2”, "Unreal Engine Installer" and "Nuget Package Manager" optional individual components.
- run `Setup.bat`
- run `GenerateProjectFiles.bat ` as administrator 

## Build UnrealEngine
- Open the generated Visual Studio project `UE4.sln` in the root of the repository. Once it is open you need to set it to build the _Development Editor_ configuration for _Win64_. 
- Once that is set, you can right click the UE4 target in the Solution Explorer on the right side of the window and press build. This will take a while. 
- please run `Engine\Binaries\Win64\UnrealVersionSelector-Win64-Shipping.exe` once so it is detectable by your system. 

## Build AirSim
* Start `x64 Native Tools Command Prompt for VS 2019`. 
* Clone the repo: `git clone https://cosysgit.uantwerpen.be/sensorsimulation/airsim.git`, and go the AirSim directory by `cd AirSim`. 
* Run `build.cmd` from the command line. This will create ready to use plugin bits in the `Unreal\Plugins` folder that can be dropped into any Unreal project.

## Build Unreal Project

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md).

## Setup Remote Control (Multirotor only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.

## How to Use AirSim

Once AirSim is set up by following above steps, you can,

1. Double click on .sln file to load for example the Blocks project in `Unreal\Environments\Blocks` (or .sln file in your own [custom](unreal_custenv.md) Unreal project). If you don't see .sln file then you probably haven't completed steps in Build Unreal Project section above.
2. Select your Unreal project as Start Up project (for example, Blocks project) and make sure Build config is set to "Develop Editor" and x64.
3. After Unreal Editor loads, press Play button. Tip: go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available.

The other environments available often need additional asset packs to be downloaded first, read [here](environments.md) for more information.

# AirSim on Unity (Experimental)
[Unity](https://unity3d.com/) is another great game engine platform and we have an [experimental release](https://github.com/Microsoft/AirSim/tree/master/Unity) of AirSim on Unity. Please note that this is work in progress and all features may not work yet. 

# FAQ
#### I get `error C100 : An internal error has occurred in the compiler` when running build.cmd
We have noticed this happening with VS version `15.9.0` and have checked-in a workaround in AirSim code. If you have this VS version, please make sure to pull the latest AirSim code.

#### I get error "'corecrt.h': No such file or directory" or "Windows SDK version 8.1 not found"
Very likely you don't have [Windows SDK](https://developercommunity.visualstudio.com/content/problem/3754/cant-compile-c-program-because-of-sdk-81cant-add-a.html) installed with Visual Studio. 

#### How do I use PX4 firmware with AirSim?
By default, AirSim uses its own built-in firmware called [simple_flight](simple_flight.md). There is no additional setup if you just want to go with it. If you want to switch to using PX4 instead then please see [this guide](px4_setup.md).

#### I made changes in Visual Studio but there is no effect

Sometimes the Unreal + VS build system doesn't recompile if you make changes to only header files. To ensure a recompile, make some Unreal based cpp file "dirty" like AirSimGameMode.cpp.

#### Unreal still uses VS2015 or I'm getting some link error
Running several versions of VS can lead to issues when compiling UE projects. One problem that may arise is that UE will try to compile with an older version of VS which may or may not work. There are two settings in Unreal, one for for the engine and one for the project, to adjust the version of VS to be used.
1. Edit -> Editor preferences -> General -> Source code
2. Edit -> Project Settings -> Platforms -> Windows -> Toolchain ->CompilerVersion

In some cases, these settings will still not lead to the desired result and errors such as the following might be produced: LINK : fatal error LNK1181: cannot open input file 'ws2_32.lib'

To resolve such issues the following procedure can be applied:
1. Uninstall all old versions of VS using the [VisualStudioUninstaller](https://github.com/Microsoft/VisualStudioUninstaller/releases)
2. Repair/Install VS2017
3. Restart machine and install Epic launcher and desired version of the engine

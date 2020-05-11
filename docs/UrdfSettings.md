# AirSim Settings for URDF simulation

# Overview

Once the URDF file is constructed, there is some modification that needs to be done to the AirSim settings file in order to use the bot. First, read through and understand the global settings that apply to every robot [here](settings.md). The following modifications need to be made:

* The pawn type needs to be set to "urdfbot"
* Under "PawnPaths", there needs to be a key "UrdfFile", which contains the full filepath of the URDF XML to be used to simulate the bot.
* Under vehicles, there must be a single entry called "UrdfBot".
* There are no sensors added by default. Every sensor needs to be explicitly specified in the Sensor element. This includes cameras - there are no cameras attached by default. At least one camera must be added to the bot. During gameplay, the 'n' key can be used to cycle through the active camera views.
* Each sensor needs an "attach_link" member, which specifies the link to which the sensor will be attached. The sensor will then follow the motion of that particular link. Offsets will be defined from the reference frame of this link.
* Inside the UrdfBot vehicle member, there are two special members that may be used during development that do not apply to other pawn types:
    * DebugSymbolScale: This integer value specifies the size of debug symbols to draw on the screen. This is useful when developing the XML file, but should be turned off once the development of the XML is complete. Omitting or setting this value to 0 disables the debug mode. For more information, see [Using Debug mode for URDF bots](UrdfDebugMode.md).
    * CollisionBlacklist: For many bots, there will be some portion of the bot that is expected to be in constant contact with the ground (wheels, treads, legs, etc). Currently, this will create a collision event every frame, which is extremely noisy. Using this array, we can ignore collisions between the bot and other external meshes. Each entry in this array has two members: "BotMesh", which must exactly match the name of a link within the bot, and "ExternalActorRegex", which is a regular expression which can be used to specify a range of meshes to ignore. If a collision event is observed between a link specified in "BotMesh" and a mesh matching the "ExternalActorRegex", then the collision event will not be emitted.
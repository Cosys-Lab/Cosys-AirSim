// Copyright (c) Cosys-Lab, University of Antwerp. 2026

#pragma once

// This header is included by almost every file of the module, so it only includes what its declarations need.
// The implementation includes are in AirBlueprintLib.cpp.
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerInput.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AssetRegistry/AssetData.h"
#include "common/CommonStructs.hpp"
#include "UObject/ObjectMacros.h"
#include "UObject/ScriptMacros.h"
#include <string>
#include <utility>
#include <vector>
#include "AirBlueprintLib.generated.h"

class ALandscapeProxy;
class IImageWrapperModule;
class UInputComponent;

UENUM(BlueprintType)
enum class LogDebugLevel : uint8
{
    Informational UMETA(DisplayName = "Informational"),
    Success UMETA(DisplayName = "Success"),
    Failure UMETA(DisplayName = "Failure"),
    Unimportant UMETA(DisplayName = "Unimportant")
};

/**
*
*/
UCLASS()
class AIRSIM_API UAirBlueprintLib : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    static void OnBeginPlay();
    static void OnEndPlay();
    static void LogMessageString(const std::string& prefix, const std::string& suffix, LogDebugLevel level, float persist_sec = 60);
    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    static void LogMessage(const FString& prefix, const FString& suffix, LogDebugLevel level, float persist_sec = 60);
    static float GetWorldToMetersScale(const AActor* context);
    template <typename T>
    static T* GetActorComponent(AActor* actor, FString name);

    template <typename T>
    static T* FindActor(const UObject* context, FString name)
    {
        return static_cast<T*>(FindActorOfClass(context, T::StaticClass(), name));
    }

    template <typename T>
    static void FindAllActor(const UObject* context, TArray<AActor*>& foundActors)
    {
        FindAllActorsOfClass(context, T::StaticClass(), foundActors);
    }

    // first actor of this class with this tag or name (the non-template part of FindActor)
    static AActor* FindActorOfClass(const UObject* context, UClass* actor_class, const FString& name);
    static void FindAllActorsOfClass(const UObject* context, UClass* actor_class, TArray<AActor*>& found_actors);

    static std::vector<std::string> ListMatchingActorsOriginal(const UObject* context, const std::string& name_regex);
    static std::vector<std::pair<std::string, std::string>> ListMatchingActorsTags(const UObject* context, const std::string& name_regex);
    static std::vector<std::string> ListMatchingActors(const UObject *context, const std::string& name_regex);
    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    static bool loadLevel(UObject* context, const FString& level_name);
    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    static bool spawnPlayer(UWorld* context);
    UFUNCTION(BlueprintPure, Category = "airsim | Utils")
    static TArray<FName> ListWorldsInRegistry();
    static UObject* GetMeshFromRegistry(const std::string& load_object);
    static void GenerateAssetRegistryMap(const UObject* context, TMap<FString, FAssetData>& asset_map);
    static void GenerateActorMap(const UObject* context, TMap<FString, AActor*>& scene_object_map);

    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    static bool RunConsoleCommand(const AActor* context, const FString& command);

    static bool HasObstacle(const AActor* actor, const FVector& start, const FVector& end,
                            const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static bool GetObstacle(const AActor* actor, const FVector& start, const FVector& end,
                            FHitResult& hit, const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static bool GetObstacleAdv(const AActor* actor, const FVector& start, const FVector& end,
		                       FHitResult& hit, TArray<AActor*>& ignore_actors, ECollisionChannel collision_channel = ECC_Visibility, bool trace_complex = false, bool get_material = false);
    static bool GetLastObstaclePosition(const AActor* actor, const FVector& start, const FVector& end,
                                        FHitResult& hit, const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static void FollowActor(AActor* follower, const AActor* followee, const FVector& offset, bool fixed_z = false, float fixed_z_val = 2.0f);

    static bool SetMeshStencilID(const std::string& mesh_name, int object_id,
                                 bool is_name_regex = false);
    static int GetMeshStencilID(const std::string& mesh_name);
    static void InitializeMeshStencilIDs(bool ignore_existing, FString material_list);

    static bool IsInGameThread();

	template<class T>
	static std::string GetMeshName(T* mesh)
	{
		if (mesh->GetOwner())
		{
			// For foliage actors, need to use mesh's name in order to differentiate different meshes from each other.
			// Otherwise, all items placed with the foliage actor will get the same ID.
			auto owner = mesh->GetOwner();
			if (owner->GetName().StartsWith("InstancedFoliageActor"))
			{
				FString meshName = mesh->GetName();

				// It is expected that the user will create a blueprint class that starts with "IFA_" for each of the classes they want to differentiate.
				// Unreal adds a _C_ into the name for blueprint created classes, which messes up the hash. Remove it as well.
				if (meshName.StartsWith(TEXT("IFA_")))
				{
					meshName = meshName.Replace(TEXT("IFA_"), TEXT("")).Replace(TEXT("_C_"), TEXT(""));
				}

				return std::string(TCHAR_TO_UTF8(*(meshName)));
			}

			return std::string(TCHAR_TO_UTF8(*(owner->GetName())));
		}
		else
			return ""; //std::string(TCHAR_TO_UTF8(*(UKismetSystemLibrary::GetDisplayName(mesh))));
	}

    static std::string GetMeshName(ALandscapeProxy* mesh);

    template <class UserClass>
    static FInputActionBinding& BindActionToKey(const FName action_name, const FKey in_key, UserClass* actor,
                                                typename FInputActionHandlerSignature::TMethodPtr<UserClass> func, bool on_press_or_release = false,
                                                bool shift_key = false, bool control_key = false, bool alt_key = false, bool command_key = false)
    {
        FInputActionKeyMapping action(action_name, in_key, shift_key, control_key, alt_key, command_key);

        AddActionMapping(actor, action);
        return GetPlayerInputComponent(actor)->BindAction(action_name, on_press_or_release ? IE_Pressed : IE_Released, actor, func);
    }

    template <class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FName axis_name, const FKey in_key, AActor* actor, UserClass* obj,
                                            typename FInputAxisHandlerSignature::TMethodPtr<UserClass> func)
    {
        FInputAxisKeyMapping axis(axis_name, in_key);

        return UAirBlueprintLib::BindAxisToKey(axis, actor, obj, func);
    }

    template <class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FInputAxisKeyMapping& axis, AActor* actor, UserClass* obj,
                                            typename FInputAxisHandlerSignature::TMethodPtr<UserClass> func)
    {
        AddAxisMapping(actor, axis);
        return GetPlayerInputComponent(actor)->BindAxis(axis.AxisName, obj, func);
    }

    // the player controller parts of the Bind*ToKey templates
    static void AddActionMapping(AActor* actor, const FInputActionKeyMapping& action);
    static void AddAxisMapping(AActor* actor, const FInputAxisKeyMapping& axis);
    static UInputComponent* GetPlayerInputComponent(AActor* actor);

    static int RemoveAxisBinding(const FInputAxisKeyMapping& axis, FInputAxisBinding* axis_binding, AActor* actor);

    static void EnableInput(AActor* actor);

    static void RunCommandOnGameThread(TFunction<void()> InFunction, bool wait = false, const TStatId InStatId = TStatId());

    static float GetDisplayGamma();

    static EAppReturnType::Type ShowMessage(EAppMsgType::Type MessageType, const std::string& message, const std::string& title);

    static bool getLogMessagesHidden()
    {
        return log_messages_hidden_;
    }
    static void setLogMessagesVisibility(bool is_visible);

    static void enableWorldRendering(AActor* context, bool enable);
    static void DrawCoordinateSystem(const UWorld* InWorld, FVector const& AxisLoc, FRotator const& AxisRot, float Scale, bool bPersistentLines = false, float LifeTime = -1.f, uint8 DepthPriority = 0, float Thickness = 0.f);
	static void DrawLine(const UWorld* InWorld, FVector const& LineStart, FVector const& LineEnd, FColor const& Color, bool bPersistentLines = false, float LifeTime = -1.f, uint8 DepthPriority = 0, float Thickness = 0.f);
	static void DrawPoint(const UWorld* InWorld, FVector const& Position, float Size, FColor const& Color, bool bPersistentLines = false, float LifeTime = -1.f, uint8 DepthPriority = 0);
	static void DrawCircle(const UWorld* InWorld, const FMatrix& TransformMatrix, float Radius, int32 Segments, const FColor& Color, bool bPersistentLines = false, float LifeTime = -1.f, uint8 DepthPriority = 0, float Thickness = 0.f, bool bDrawAxis = true);
	static void DrawCircle(const UWorld* InWorld, FVector Center, float Radius, int32 Segments, const FColor& Color, bool bPersistentLines = false, float LifeTime = -1.f, uint8 DepthPriority = 0, float Thickness = 0.f, FVector YAxis = FVector(0.f, 1.f, 0.f), FVector ZAxis = FVector(0.f, 0.f, 1.f), bool bDrawAxis = true);
    static void enableViewportRendering(AActor* context, bool enable);
    static void setSimulatePhysics(AActor* actor, bool simulate_physics);
    static void resetSimulatePhysics(AActor* actor);
    static std::vector<UPrimitiveComponent*> getPhysicsComponents(AActor* actor);

    static UObject* LoadObject(const std::string& name);
    static UClass* LoadClass(const std::string& name);

    static void setUnrealClockSpeed(const AActor* context, float clock_speed);
	static float getUnrealClockSpeed(const AActor* context);
    static IImageWrapperModule* getImageWrapperModule();
    static void CompressImageArray(int32 width, int32 height, const TArray<FColor>& src, TArray<uint8>& dest);
    static std::vector<msr::airlib::MeshPositionVertexBuffersResponse> GetStaticMeshComponents();

private:
    static bool CompressUsingImageWrapper(const TArray<uint8>& uncompressed, const int32 width, const int32 height, TArray<uint8>& compressed);

private:
    static bool log_messages_hidden_;
    //FViewPort doesn't expose this field so we are doing dirty work around by maintaining count by ourselves
    static uint32_t flush_on_draw_count_;

    static IImageWrapperModule* image_wrapper_module_;
};

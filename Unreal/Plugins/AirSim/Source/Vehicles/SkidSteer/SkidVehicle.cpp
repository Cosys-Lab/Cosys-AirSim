// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehicle.h"
#include "SkidVehicleMovementComponent.h"

ASkidVehicle::ASkidVehicle(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer.SetDefaultSubobjectClass<USkidVehicleMovementComponent>(AWheeledVehicle::VehicleMovementComponentName))
{


}

// Called when the game starts or when spawned
void ASkidVehicle::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void ASkidVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

// Called to bind functionality to input
void ASkidVehicle::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{

	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

void ASkidVehicle::PostInitializeComponents()
{
	Super::PostInitializeComponents();
}

class USkidVehicleMovementComponent* ASkidVehicle::GetSkidVehicleMovement() const
{
	class USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());
	return SkidMovmentComponent;
}



void ASkidVehicle::SetXJoy(float nJoyX)
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetXJoy(nJoyX);
	}
}

void ASkidVehicle::SetYJoy(float nJoyY)
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetYJoy(nJoyY);
	}
}

void ASkidVehicle::SetBreaksOn()
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetBreaksOn();
	}
}

void ASkidVehicle::SetBreaksOff()
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetBreaksOff();
	}
}

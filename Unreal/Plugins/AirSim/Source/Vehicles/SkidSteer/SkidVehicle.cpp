// Fill out your copyright notice in the Description page of Project Settings.


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

void ASkidVehicle::SetRightThrust(float RightThrust)
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetRightThrust(RightThrust);
	}
}

void ASkidVehicle::SetLeftThrust(float LeftThrust)
{
	USkidVehicleMovementComponent* SkidMovmentComponent = Cast<USkidVehicleMovementComponent>(GetVehicleMovement());

	if (SkidMovmentComponent)
	{
		SkidMovmentComponent->SetLeftThrust(LeftThrust);
	}
}

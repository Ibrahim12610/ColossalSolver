// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UColossalBracingComponent.generated.h"

// Packaging our data together makes it incredibly easy to read inside the AnimBP and Control Rig
USTRUCT(BlueprintType)
struct FColossalBraceData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Colossal Bracing")
    bool bIsBracing = false;

    UPROPERTY(BlueprintReadOnly, Category = "Colossal Bracing")
    FVector TargetHandLocation = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Colossal Bracing")
    FVector TargetHandNormal = FVector::UpVector;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class COLOSSALSOLVER_API UUColossalBracingComponent : public UActorComponent
{
    GENERATED_BODY()

public: 
    UUColossalBracingComponent();

protected:
    virtual void BeginPlay() override;

public: 
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // --- Configuration Parameters ---
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bracing Config")
    float TraceDistance = 400.0f; // How far ahead the giant reaches out

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bracing Config")
    float SweepRadius = 50.0f; // Thickness of the sweeping sphere capsule

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bracing Config")
    float SmoothSpeed = 7.0f; // Interp speed to stop hands from popping/snapping onto walls

    // --- Output Structs for the Rig Bridge ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bracing Data")
    FColossalBraceData LeftArmBrace;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Bracing Data")
    FColossalBraceData RightArmBrace;

private:
    UPROPERTY()
    class USkeletalMeshComponent* OwnerMesh;

    // Extracted processing method to keep Tick clean and human-readable
    void ProcessArmBrace(FName ShoulderBoneName, FColossalBraceData& OutBraceData, float DeltaTime);
};
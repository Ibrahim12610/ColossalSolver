#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UColossalDetectorComponent.generated.h"

class USkeletalMeshComponent;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class COLOSSALSOLVER_API UUColossalDetectorComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UUColossalDetectorComponent();

protected:
    virtual void BeginPlay() override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // === SETUP ===

    // The IK foot bone names on your skeleton — same pattern as ik_foot_l/ik_foot_r on Mannequin
    // On Grux check your skeleton for IK foot bones, or create virtual bones
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    FName LeftIKFootBoneName = FName("ik_foot_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    FName RightIKFootBoneName = FName("ik_foot_r");

    // How far above the IK foot bone to start the trace
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float TraceStartOffset = 600.0f;

    // How far down to trace for terrain
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float TraceLength = 2000.0f;

    // Sphere sweep radius — larger catches more terrain geometry
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float SweepSphereRadius = 150.0f;

    // Scale the final offset — tune in editor to prevent over/under correction
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float CorrectionScale = 1.0f;

    // Interpolation speed toward target — matches Mannequin's 15.0
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float InterpSpeedIncreasing = 15.0f;

    // Interpolation speed returning to zero
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float InterpSpeedDecreasing = 15.0f;

    // Master switch — when false resets all offsets to 0 (control condition)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    bool bShouldDoIKTrace = true;

    // === STEP 1 OUTPUTS: Raw Z offset targets from trace ===
    // These are what the trace found — before interpolation
    // Matches ZOffset_L_Target and ZOffset_R_Target in Mannequin IK

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Step 1 Targets")
    float ZOffsetL_Target = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Step 1 Targets")
    float ZOffsetR_Target = 0.0f;

    // === STEP 2 OUTPUTS: Smoothed/interpolated Z offsets ===
    // These are what you feed into Modify Transforms in Control Rig
    // Matches ZOffset_L and ZOffset_R after Alpha Interpolate in Mannequin IK

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Step 2 Smoothed")
    float ZOffsetL = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Step 2 Smoothed")
    float ZOffsetR = 0.0f;

    // === STEP 3 OUTPUT: Pelvis offset ===
    // Lowest of the two foot offsets — prevents overextension
    // Matches ZOffset_Pelvis in Mannequin IK

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Step 3 Pelvis")
    float ZOffsetPelvis = 0.0f;

    // === SURFACE NORMAL OUTPUTS: For foot rotation after FBIK ===
    // Feed into Set Rotation on foot bones after FBIK solves

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Rotation")
    FRotator LeftFootRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Rotation")
    FRotator RightFootRotation = FRotator::ZeroRotator;

    // === TELEMETRY ===
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Telemetry")
    FVector CachedCenterOfMass;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Telemetry")
    bool bLeftFootHit = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Telemetry")
    bool bRightFootHit = false;

    FVector CalculateCenterOfMass();

private:
    UPROPERTY()
    USkeletalMeshComponent* OwnerMesh = nullptr;

    int32 CachedBoneCount = 0;

    // Internal smoothed normal state
    FVector SmoothedNormalL = FVector::UpVector;
    FVector SmoothedNormalR = FVector::UpVector;

    bool DoFootTrace(FName IKBoneName, float& OutZTarget, FRotator& OutRotation,
        FVector& InOutSmoothedNormal, bool& OutHit, float DeltaTime);

    void LogTelemetryMessage(int32 Key, const FString& Message, FColor Color);
};
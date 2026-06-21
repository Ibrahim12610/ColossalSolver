#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UColossalDetectorComponent.generated.h"

class USkeletalMeshComponent;

UENUM(BlueprintType)
enum class EColossalLimbType : uint8
{
    Foot    UMETA(DisplayName = "Foot (Downward Sense)"),
    HandArm UMETA(DisplayName = "Hand/Arm (Forward/Lateral Sense)")
};

USTRUCT(BlueprintType)
struct FColossalEffectorTarget
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FName BoneName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    EColossalLimbType LimbType;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    float TraceLength = 150.0f;

    // Radius of the sphere sweep — tune per effector in editor without recompiling
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    float SweepSphereRadius = 150.0f;

    // Multiplier applied to CorrectionDelta. 1.0 = full correction.
    // Tune live in editor to fix over/under-correction.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    float CorrectionScale = 1.0f;

    // Direction override for hand/arm traces in bone local space
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector TraceDirectionOverride = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    bool bUseDirectionOverride = false;

    // Runtime outputs — VisibleAnywhere prevents editor refresh instability
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FRotator CalculatedDeltaRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector CalculatedImpactPoint = FVector::ZeroVector;

    // Correction delta — relative offset from stable origin to impact point.
    // Use this in Control Rig instead of CalculatedImpactPoint directly.
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector CorrectionDelta = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    bool bIsColliding = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FRotator CurrentSmoothedRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector SmoothedNormal = FVector::UpVector;

    // Stable trace origin — set once in BeginPlay from reference pose,
    // tracked via actor movement delta to break the IK feedback loop
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Internal")
    FVector StableTraceOrigin = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Internal")
    bool bStableOriginInitialized = false;
};

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

    FRotator CalculateAnkleRotation(const FVector& FootForward, const FVector& SurfaceNormal);
    FVector CalculateCenterOfMass();

    // Calculates a pole vector world position offset from a given bone along its
    // local bend axis. Uses GetUnitAxis (always a proper unit vector) and a single
    // scalar multiply — avoids the component-wise vector*vector bug that produced
    // near-zero magnitude offsets when done in the Control Rig graph.
    UFUNCTION(BlueprintCallable, Category = "Colossal Solver")
    FVector CalculatePoleVector(FName BendBoneName, float Distance, bool bInvertDirection);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    TArray<FColossalEffectorTarget> EffectorTargets;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float AirTimeThreshold = 2000.0f;

    // --- Pole vector setup ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Pole Vectors")
    FName LeftCalfBoneName = FName("calf_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Pole Vectors")
    FName RightCalfBoneName = FName("calf_r");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Pole Vectors")
    float PoleVectorDistance = 1000.0f;

    // Independent per-leg inversion toggles — fixes mirrored skeleton axis issues
    // without needing to touch Control Rig graph wiring
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Pole Vectors")
    bool bInvertLeftPoleDirection = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Pole Vectors")
    bool bInvertRightPoleDirection = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Telemetry")
    FVector CachedCenterOfMass;

    // --- Pole vector outputs — read these directly in Control Rig ---
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector LeftPoleVectorPosition;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector RightPoleVectorPosition;

    // --- Named correction delta / rotation outputs ---
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector LeftFootCorrectionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector RightFootCorrectionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector LeftHandCorrectionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FVector RightHandCorrectionDelta;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FRotator LeftAnkleTargetRotation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FRotator RightAnkleTargetRotation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FRotator LeftWristTargetRotation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    FRotator RightWristTargetRotation;

    // Single map of correction deltas keyed by bone name — alternative to named
    // outputs above. In Control Rig: Get this map, Find with bone name as key.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    TMap<FName, FVector> CorrectionTranslations;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Output")
    TMap<FName, FRotator> CorrectionRotations;

private:
    UPROPERTY()
    USkeletalMeshComponent* OwnerMesh = nullptr;

    int32 CachedBoneCount = 0;
    bool bIsGrounded = false;
    int32 AirFrameCount = 0;

    UPROPERTY()
    FVector InitialActorLocation = FVector::ZeroVector;

    void LogTelemetryMessage(int32 Key, const FString& Message, FColor Color);
};
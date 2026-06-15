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

    // Direction override for hand/arm traces in bone local space
    // X=forward/back, Y=left/right, Z=up/down relative to bone
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector TraceDirectionOverride = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    bool bUseDirectionOverride = false;

    // Runtime outputs — VisibleAnywhere prevents editor refresh instability
    // that EditAnywhere causes on computed values
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FRotator CalculatedDeltaRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector CalculatedImpactPoint = FVector::ZeroVector;

    // Correction delta — use this in Control Rig instead of CalculatedImpactPoint
    // This is the Z offset the foot/hand needs to move, not an absolute world position
    // Apply as: CurrentBoneGlobalTranslation + CorrectionDelta in Control Rig
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector CorrectionDelta = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    bool bIsColliding = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FRotator CurrentSmoothedRotation = FRotator::ZeroRotator;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Colossal Solver")
    FVector SmoothedNormal = FVector::UpVector;

    // Stable trace origin — set once in BeginPlay from reference pose
    // tracked via actor movement delta to break IK feedback loop
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    TArray<FColossalEffectorTarget> EffectorTargets;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Colossal Solver | Setup")
    float AirTimeThreshold = 2000.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Colossal Solver | Telemetry")
    FVector CachedCenterOfMass;

    // Named correction delta outputs for easy Control Rig access
    // Use these instead of indexing the array at runtime
    // Apply in Control Rig as: CurrentBoneGlobalTranslation + Delta
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

private:
    UPROPERTY()
    USkeletalMeshComponent* OwnerMesh = nullptr;

    int32 CachedBoneCount = 0;
    bool bIsGrounded = false;
    int32 AirFrameCount = 0;

    // Captured at BeginPlay — base for computing actor movement delta
    UPROPERTY()
    FVector InitialActorLocation = FVector::ZeroVector;

    void LogTelemetryMessage(int32 Key, const FString& Message, FColor Color);
};
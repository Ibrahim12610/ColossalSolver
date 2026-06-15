
#include "UColossalDetectorComponent.h"
#include "DrawDebugHelpers.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"

UUColossalDetectorComponent::UUColossalDetectorComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UUColossalDetectorComponent::BeginPlay()
{
    Super::BeginPlay();

    AActor* Owner = GetOwner();
    if (Owner)
    {
        OwnerMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
        InitialActorLocation = Owner->GetActorLocation();
    }

    if (OwnerMesh)
    {
        CachedBoneCount = OwnerMesh->GetNumBones();

        // Initialize stable trace origins from the reference pose once at start.
        // Traces for all effectors (feet AND hands) are anchored to this position
        // tracked via actor movement delta — never the live IK-modified bone position.
        // This breaks the feedback loop where IK output feeds back into trace input
        // causing the flicker that was previously only fixed for feet.
        for (FColossalEffectorTarget& Target : EffectorTargets)
        {
            if (OwnerMesh->GetBoneIndex(Target.BoneName) != INDEX_NONE)
            {
                Target.StableTraceOrigin = OwnerMesh->GetBoneTransform(
                    Target.BoneName, ERelativeTransformSpace::RTS_World).GetLocation();
                Target.bStableOriginInitialized = true;
            }
        }
    }
}

void UUColossalDetectorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!OwnerMesh || EffectorTargets.Num() == 0) return;
    UWorld* World = GetWorld();
    if (!World) return;

    FCollisionQueryParams TraceParams;
    TraceParams.AddIgnoredActor(GetOwner());
    TraceParams.AddIgnoredComponent(OwnerMesh);
    TraceParams.bTraceComplex = true;

    CachedCenterOfMass = CalculateCenterOfMass();
    DrawDebugSphere(World, CachedCenterOfMass, 25.0f, 12, FColor::Yellow, false, 0.5f, 0, 3.0f);
    DrawDebugLine(World, CachedCenterOfMass, CachedCenterOfMass + (FVector::DownVector * 500.0f), FColor::Orange, false, 0.5f, 0, 1.5f);
    LogTelemetryMessage(99, FString::Printf(TEXT("Center of Mass: %s"), *CachedCenterOfMass.ToString()), FColor::Yellow);

    // Actor movement delta — how far the actor has moved since BeginPlay.
    // Applied to each effector's StableTraceOrigin so traces track overall movement
    // (walking, falling) while remaining independent of IK-modified bone positions.
    // Applied to BOTH feet and hands to fix flickering on arms too.
    FVector ActorDelta = GetOwner()->GetActorLocation() - InitialActorLocation;

    for (FColossalEffectorTarget& Target : EffectorTargets)
    {
        int32 BoneIndex = OwnerMesh->GetBoneIndex(Target.BoneName);
        if (BoneIndex == INDEX_NONE) continue;

        FTransform BoneTransform = OwnerMesh->GetBoneTransform(
            Target.BoneName, ERelativeTransformSpace::RTS_World);
        FVector BoneWorldPos = BoneTransform.GetLocation();

        if (Target.CalculatedImpactPoint.IsNearlyZero())
            Target.CalculatedImpactPoint = BoneWorldPos;

        // Fallback if stable origin was never initialized
        if (!Target.bStableOriginInitialized)
        {
            Target.StableTraceOrigin = BoneWorldPos - ActorDelta;
            Target.bStableOriginInitialized = true;
        }

        // Stable origin for this frame — reference pose position plus actor movement.
        // Same logic for feet and hands — breaks feedback loop for both.
        FVector StableOrigin = Target.StableTraceOrigin + ActorDelta;

        FVector TraceStart;
        FVector TraceEnd;
        FHitResult HitResult;
        bool bHitOccurred = false;

        if (Target.LimbType == EColossalLimbType::Foot)
        {
            // Start 600 above stable origin — guarantees trace starts above terrain
            // even when the capsule sinks slightly into uneven rocky surfaces
            TraceStart = FVector(StableOrigin.X, StableOrigin.Y, StableOrigin.Z + 600.0f);
            TraceEnd = TraceStart + (FVector::DownVector * Target.TraceLength);

            FCollisionShape SweepSphere = FCollisionShape::MakeSphere(Target.SweepSphereRadius);
            bHitOccurred = World->SweepSingleByChannel(
                HitResult, TraceStart, TraceEnd,
                FQuat::Identity, ECC_Visibility, SweepSphere, TraceParams);

            // Debug visualization
            DrawDebugLine(World, TraceStart, TraceEnd, FColor::White, false, 0.5f, 0, 1.0f);
            DrawDebugSphere(World, TraceStart, Target.SweepSphereRadius, 8, FColor::White, false, 0.5f, 0, 1.0f);

            if (bHitOccurred)
            {
                DrawDebugSphere(World, HitResult.ImpactPoint, Target.SweepSphereRadius, 8, FColor::Red, false, 0.5f, 0, 2.0f);
                DrawDebugLine(World, TraceStart, HitResult.ImpactPoint, FColor::Red, false, 0.5f, 0, 2.0f);
            }
            else
            {
                DrawDebugSphere(World, TraceEnd, Target.SweepSphereRadius, 8, FColor::Green, false, 0.5f, 0, 1.0f);
            }

            // Diagnostic log — ImpactZ vs BoneZ vs StableZ
            LogTelemetryMessage(
                60 + BoneIndex,
                FString::Printf(TEXT("%s | ImpactZ: %.0f | BoneZ: %.0f | StableZ: %.0f"),
                    *Target.BoneName.ToString(),
                    Target.CalculatedImpactPoint.Z,
                    BoneWorldPos.Z,
                    StableOrigin.Z),
                FColor::Cyan
            );
        }
        else if (Target.LimbType == EColossalLimbType::HandArm)
        {
            // Hands use stable origin for trace start — same fix as feet.
            // Stable origin tracks actor movement without being affected by
            // whatever Control Rig does to the hand/arm bones each frame.
            TraceStart = StableOrigin;

            FVector SenseDirection;

            if (Target.bUseDirectionOverride && !Target.TraceDirectionOverride.IsNearlyZero())
            {
                // Use the bone's current orientation to transform the local-space override
                // into world space. Bone orientation from BoneTransform is fine here —
                // we only use it for direction, not for the trace start position.
                SenseDirection = BoneTransform.TransformVector(
                    Target.TraceDirectionOverride.GetSafeNormal());
            }
            else
            {
                FVector ForwardDirection = BoneTransform.GetUnitAxis(EAxis::X);
                FVector OutwardDirection = BoneTransform.GetUnitAxis(EAxis::Y);
                SenseDirection = (ForwardDirection * 0.7f + OutwardDirection * 0.3f).GetSafeNormal();
            }

            if (SenseDirection.IsNearlyZero()) SenseDirection = FVector::ForwardVector;
            TraceEnd = TraceStart + (SenseDirection * Target.TraceLength);

            FCollisionShape ArmSphere = FCollisionShape::MakeSphere(Target.SweepSphereRadius);
            bHitOccurred = World->SweepSingleByChannel(
                HitResult, TraceStart, TraceEnd,
                FQuat::Identity, ECC_Visibility, ArmSphere, TraceParams);

            DrawDebugLine(World, TraceStart, TraceEnd, FColor::White, false, 0.5f, 0, 1.0f);
            DrawDebugSphere(World, TraceStart, Target.SweepSphereRadius, 8, FColor::White, false, 0.5f, 0, 1.0f);

            if (bHitOccurred)
            {
                DrawDebugSphere(World, HitResult.ImpactPoint, Target.SweepSphereRadius, 8, FColor::Red, false, 0.5f, 0, 2.0f);
                DrawDebugLine(World, TraceStart, HitResult.ImpactPoint, FColor::Red, false, 0.5f, 0, 2.0f);
            }
            else
            {
                DrawDebugSphere(World, TraceEnd, Target.SweepSphereRadius, 8, FColor::Green, false, 0.5f, 0, 1.0f);
            }

            // Diagnostic log for hands
            LogTelemetryMessage(
                60 + BoneIndex,
                FString::Printf(TEXT("%s | ImpactZ: %.0f | BoneZ: %.0f | StableZ: %.0f"),
                    *Target.BoneName.ToString(),
                    Target.CalculatedImpactPoint.Z,
                    BoneWorldPos.Z,
                    StableOrigin.Z),
                FColor::Magenta
            );
        }
        else
        {
            continue;
        }

        LogTelemetryMessage(
            30 + BoneIndex,
            FString::Printf(TEXT("%s | Hit:%s | StartZ:%.0f | EndZ:%.0f | Len:%.0f"),
                *Target.BoneName.ToString(),
                bHitOccurred ? TEXT("YES") : TEXT("NO"),
                TraceStart.Z, TraceEnd.Z, Target.TraceLength),
            bHitOccurred ? FColor::Green : FColor::Red
        );

        Target.bIsColliding = bHitOccurred;

        if (bHitOccurred)
        {
            Target.SmoothedNormal = FMath::VInterpTo(
                Target.SmoothedNormal, HitResult.ImpactNormal, DeltaTime, 12.0f);

            FVector LimbForward = BoneTransform.GetUnitAxis(EAxis::X);
            Target.CalculatedDeltaRotation = FMath::RInterpTo(
                Target.CalculatedDeltaRotation,
                CalculateAnkleRotation(LimbForward, Target.SmoothedNormal),
                DeltaTime, 12.0f);

            Target.CalculatedImpactPoint = HitResult.ImpactPoint;

            // Correction delta — the offset needed from stable origin to impact point.
            // Use this in Control Rig instead of CalculatedImpactPoint directly.
            // Apply as: Get Bone Transform (Global) Translation + CorrectionDelta
            // This works correctly regardless of mesh scale because it is a relative
            // offset, not an absolute world position.
            Target.CorrectionDelta = FVector(
                HitResult.ImpactPoint.X - StableOrigin.X,
                HitResult.ImpactPoint.Y - StableOrigin.Y,
                HitResult.ImpactPoint.Z - StableOrigin.Z
            );

            // Route correction deltas to named output variables for clean Control Rig access
            if (Target.BoneName == FName("foot_l"))
            {
                LeftFootCorrectionDelta = Target.CorrectionDelta;
                LeftAnkleTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("foot_r"))
            {
                RightFootCorrectionDelta = Target.CorrectionDelta;
                RightAnkleTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("hand_l"))
            {
                LeftHandCorrectionDelta = Target.CorrectionDelta;
                LeftWristTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("hand_r"))
            {
                RightHandCorrectionDelta = Target.CorrectionDelta;
                RightWristTargetRotation = Target.CalculatedDeltaRotation;
            }
        }
        else
        {
            Target.SmoothedNormal = FMath::VInterpTo(
                Target.SmoothedNormal, FVector::UpVector, DeltaTime, 8.0f);

            Target.CalculatedDeltaRotation = FMath::RInterpTo(
                Target.CalculatedDeltaRotation, FRotator::ZeroRotator, DeltaTime, 8.0f);

            // Smoothly return correction delta to zero when no surface detected
            Target.CorrectionDelta = FMath::VInterpTo(
                Target.CorrectionDelta, FVector::ZeroVector, DeltaTime, 8.0f);

            // Route zeroing deltas to named outputs too
            if (Target.BoneName == FName("foot_l"))
            {
                LeftFootCorrectionDelta = Target.CorrectionDelta;
                LeftAnkleTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("foot_r"))
            {
                RightFootCorrectionDelta = Target.CorrectionDelta;
                RightAnkleTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("hand_l"))
            {
                LeftHandCorrectionDelta = Target.CorrectionDelta;
                LeftWristTargetRotation = Target.CalculatedDeltaRotation;
            }
            else if (Target.BoneName == FName("hand_r"))
            {
                RightHandCorrectionDelta = Target.CorrectionDelta;
                RightWristTargetRotation = Target.CalculatedDeltaRotation;
            }

            // CalculatedImpactPoint holds last valid position intentionally
        }
    }
}

FRotator UUColossalDetectorComponent::CalculateAnkleRotation(const FVector& FootForward, const FVector& SurfaceNormal)
{
    FVector WorldUp = FVector::UpVector;
    FVector SurfaceUp = SurfaceNormal.GetSafeNormal();

    FVector RotationAxis = FVector::CrossProduct(WorldUp, SurfaceUp).GetSafeNormal();
    float RotationAngle = FMath::RadiansToDegrees(
        FMath::Acos(FMath::Clamp(FVector::DotProduct(WorldUp, SurfaceUp), -1.0f, 1.0f))
    );

    if (RotationAxis.IsNearlyZero())
        return FRotator::ZeroRotator;

    FQuat TiltQuat = FQuat(RotationAxis, FMath::DegreesToRadians(RotationAngle));
    return TiltQuat.Rotator();
}

FVector UUColossalDetectorComponent::CalculateCenterOfMass()
{
    if (!OwnerMesh) return FVector::ZeroVector;

    if (OwnerMesh->IsAnySimulatingPhysics())
    {
        FVector WeightedPositionSum = FVector::ZeroVector;
        float TotalMassSum = 0.0f;

        for (FBodyInstance* BodyInstance : OwnerMesh->Bodies)
        {
            if (BodyInstance && BodyInstance->IsValidBodyInstance())
            {
                float BodyMass = BodyInstance->GetBodyMass();
                if (BodyMass > 0.01f)
                {
                    WeightedPositionSum += (BodyInstance->GetCOMPosition() * BodyMass);
                    TotalMassSum += BodyMass;
                }
            }
        }

        if (TotalMassSum > 0.0f)
            return WeightedPositionSum / TotalMassSum;
    }

    if (CachedBoneCount <= 0) return OwnerMesh->GetComponentLocation();

    FVector PositionSum = FVector::ZeroVector;
    int32 ValidBoneCount = 0;

    for (int32 i = 0; i < CachedBoneCount; i++)
    {
        FMatrix BoneMatrix = OwnerMesh->GetBoneMatrix(i);
        FVector BoneWorldPos = BoneMatrix.GetOrigin();

        if (!BoneWorldPos.IsNearlyZero(1.0f))
        {
            PositionSum += BoneWorldPos;
            ValidBoneCount++;
        }
    }

    if (ValidBoneCount > 0)
        return PositionSum / ValidBoneCount;

    return OwnerMesh->GetComponentLocation();
}

void UUColossalDetectorComponent::LogTelemetryMessage(int32 Key, const FString& Message, FColor Color)
{
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(Key, 0.03f, Color, Message);
    }
}
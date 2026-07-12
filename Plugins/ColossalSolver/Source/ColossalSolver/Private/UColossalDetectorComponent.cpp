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
    }

    if (OwnerMesh)
    {
        CachedBoneCount = OwnerMesh->GetNumBones();
    }
}

bool UUColossalDetectorComponent::DoFootTrace(
    FName IKBoneName,
    float& OutZTarget,
    FRotator& OutRotation,
    FVector& InOutSmoothedNormal,
    bool& OutHit,
    float DeltaTime)
{
    if (!OwnerMesh) return false;

    int32 BoneIndex = OwnerMesh->GetBoneIndex(IKBoneName);
    if (BoneIndex == INDEX_NONE)
    {
        LogTelemetryMessage(10, FString::Printf(TEXT("Bone not found: %s"), *IKBoneName.ToString()), FColor::Red);
        return false;
    }

    // Get the current IK foot bone world position
    FVector BoneWorldPos = OwnerMesh->GetBoneTransform(
        IKBoneName, ERelativeTransformSpace::RTS_World).GetLocation();

    // Trace starts above the bone to ensure we're above terrain even if bone sinks
    FVector TraceStart = FVector(BoneWorldPos.X, BoneWorldPos.Y, BoneWorldPos.Z + TraceStartOffset);
    FVector TraceEnd = TraceStart + (FVector::DownVector * TraceLength);

    FCollisionQueryParams TraceParams;
    TraceParams.AddIgnoredActor(GetOwner());
    TraceParams.AddIgnoredComponent(OwnerMesh);
    TraceParams.bTraceComplex = true;

    FHitResult HitResult;
    FCollisionShape SweepSphere = FCollisionShape::MakeSphere(SweepSphereRadius);

    UWorld* World = GetWorld();
    if (!World) return false;

    bool bHit = World->SweepSingleByChannel(
        HitResult, TraceStart, TraceEnd,
        FQuat::Identity, ECC_Visibility, SweepSphere, TraceParams);

    // Debug visualization
    DrawDebugLine(World, TraceStart, bHit ? HitResult.ImpactPoint : TraceEnd,
        bHit ? FColor::Red : FColor::Green, false, 0.05f, 0, 2.0f);
    if (bHit)
    {
        DrawDebugSphere(World, HitResult.ImpactPoint, SweepSphereRadius,
            8, FColor::Red, false, 0.05f, 0, 2.0f);
    }

    OutHit = bHit;

    if (bHit)
    {
        // STEP 1 CORE:
        // Z offset = how much to move the IK foot bone up or down
        // from its current animated position to sit on the terrain surface
        // This is purely a Z value — same as Mannequin IK ZOffset_L/R_Target
        float ZDiff = HitResult.ImpactPoint.Z - BoneWorldPos.Z;
        OutZTarget = ZDiff * CorrectionScale;

        // Surface normal for foot rotation after FBIK solves
        FVector SurfaceNormal = HitResult.ImpactNormal.GetSafeNormal();
        InOutSmoothedNormal = FMath::VInterpTo(
            InOutSmoothedNormal, SurfaceNormal, DeltaTime, InterpSpeedIncreasing);

        FVector RotAxis = FVector::CrossProduct(FVector::UpVector, InOutSmoothedNormal).GetSafeNormal();
        float RotAngle = FMath::RadiansToDegrees(
            FMath::Acos(FMath::Clamp(
                FVector::DotProduct(FVector::UpVector, InOutSmoothedNormal), -1.0f, 1.0f)));

        if (!RotAxis.IsNearlyZero())
        {
            FQuat TiltQuat = FQuat(RotAxis, FMath::DegreesToRadians(RotAngle));
            OutRotation = TiltQuat.Rotator();
        }
        else
        {
            OutRotation = FRotator::ZeroRotator;
        }

        LogTelemetryMessage(
            BoneIndex + 30,
            FString::Printf(TEXT("%s | Hit:YES | BoneZ:%.0f | SurfZ:%.0f | ZOffset:%.1f"),
                *IKBoneName.ToString(), BoneWorldPos.Z, HitResult.ImpactPoint.Z, OutZTarget),
            FColor::Green);

        return true;
    }
    else
    {
        // No hit — target returns to zero (foot follows base animation)
        OutZTarget = 0.0f;
        InOutSmoothedNormal = FMath::VInterpTo(
            InOutSmoothedNormal, FVector::UpVector, DeltaTime, InterpSpeedDecreasing);
        OutRotation = FRotator::ZeroRotator;

        LogTelemetryMessage(
            BoneIndex + 30,
            FString::Printf(TEXT("%s | Hit:NO | BoneZ:%.0f"), *IKBoneName.ToString(), BoneWorldPos.Z),
            FColor::Red);

        return false;
    }
}

void UUColossalDetectorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!OwnerMesh) return;
    UWorld* World = GetWorld();
    if (!World) return;

    // Update COM
    CachedCenterOfMass = CalculateCenterOfMass();

    // =============================================
    // STEP 1: Trace both feet and set Z offset targets
    // Matches: Branch(bShouldDoIKTrace) → FootTrace → Set ZOffset_L/R_Target
    // =============================================
    if (bShouldDoIKTrace)
    {
        DoFootTrace(LeftIKFootBoneName, ZOffsetL_Target, LeftFootRotation,
            SmoothedNormalL, bLeftFootHit, DeltaTime);

        DoFootTrace(RightIKFootBoneName, ZOffsetR_Target, RightFootRotation,
            SmoothedNormalR, bRightFootHit, DeltaTime);
    }
    else
    {
        // bShouldDoIKTrace is false — reset all targets to zero
        // Matches the False branch in Mannequin IK
        ZOffsetL_Target = 0.0f;
        ZOffsetR_Target = 0.0f;
        bLeftFootHit = false;
        bRightFootHit = false;
    }

    // =============================================
    // STEP 2: Interpolate Z offsets toward their target values
    // Matches: Alpha Interpolate nodes with InterpSpeedIncreasing/Decreasing
    // Use increasing speed when offset is growing, decreasing when shrinking
    // =============================================
    float SpeedL = (ZOffsetL_Target > ZOffsetL) ? InterpSpeedIncreasing : InterpSpeedDecreasing;
    float SpeedR = (ZOffsetR_Target > ZOffsetR) ? InterpSpeedIncreasing : InterpSpeedDecreasing;

    ZOffsetL = FMath::FInterpTo(ZOffsetL, ZOffsetL_Target, DeltaTime, SpeedL);
    ZOffsetR = FMath::FInterpTo(ZOffsetR, ZOffsetR_Target, DeltaTime, SpeedR);

    // =============================================
    // STEP 3: Use the lowest foot offset for the pelvis
    // Prevents overextension when one foot is much higher than the other
    // Matches: Less comparison → Select → Set ZOffset_Pelvis
    // =============================================
    ZOffsetPelvis = FMath::Min(ZOffsetL, ZOffsetR);

    // Debug
    LogTelemetryMessage(99, FString::Printf(
        TEXT("L_Offset:%.1f | R_Offset:%.1f | Pelvis:%.1f | ShouldTrace:%s"),
        ZOffsetL, ZOffsetR, ZOffsetPelvis,
        bShouldDoIKTrace ? TEXT("YES") : TEXT("NO")),
        FColor::Yellow);

    DrawDebugSphere(World, CachedCenterOfMass, 25.0f, 12, FColor::Yellow, false, 0.05f, 0, 3.0f);
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
        GEngine->AddOnScreenDebugMessage(Key, 0.05f, Color, Message);
    }
}
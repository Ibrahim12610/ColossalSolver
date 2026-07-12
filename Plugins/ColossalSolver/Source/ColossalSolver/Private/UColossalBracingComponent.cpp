#include "UColossalBracingComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "CollisionQueryParams.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"

UUColossalBracingComponent::UUColossalBracingComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UUColossalBracingComponent::BeginPlay()
{
    Super::BeginPlay();

    AActor* Owner = GetOwner();
    if (Owner)
    {
        OwnerMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
    }
}

void UUColossalBracingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!OwnerMesh) return;

    // Process both arms independently using the stable upperarm bone structures
    ProcessArmBrace(FName("upperarm_l"), LeftArmBrace, DeltaTime);
    ProcessArmBrace(FName("upperarm_r"), RightArmBrace, DeltaTime);
}

void UUColossalBracingComponent::ProcessArmBrace(FName UpperArmBoneName, FColossalBraceData& OutBraceData, float DeltaTime)
{
    UWorld* World = GetWorld();
    if (!World || !GetOwner() || !OwnerMesh) return;

    // 1. STABLE UPPER ARM ORIGIN SEARCH:
    // Pull the transform data from the stable component-space reference of the upperarm bone.
    // This preserves width configurations while isolating sensors from runtime Full Body IK displacement.
    FTransform StableComponentTransform = OwnerMesh->GetSocketTransform(UpperArmBoneName, RTS_Component);
    FTransform MeshWorldTransform = OwnerMesh->GetComponentTransform();
    FVector TraceStart = MeshWorldTransform.TransformPosition(StableComponentTransform.GetLocation());
    
    FVector ForwardVector = GetOwner()->GetActorForwardVector();
    FVector RightVector = GetOwner()->GetActorRightVector();
    
    // Determine lateral orientation strings cleanly
    FString BoneNameString = UpperArmBoneName.ToString().ToLower();
    bool bIsLeft = BoneNameString.Contains(TEXT("_l")) || BoneNameString.Contains(TEXT("left"));
    FVector OutwardVector = bIsLeft ? -RightVector : RightVector;
    
    // 2. HEAVY BRACING SLOPE CALCULATION:
    // Blend a fanned horizontal direction vector with a distinct downward pitch angle (-0.4f).
    // This shifts the physical sphere sweep trajectory downward to target surfaces at a natural leaning level.
    FVector FlatDirection = (ForwardVector * 0.70f + OutwardVector * 0.30f).GetSafeNormal();
    FVector DownwardTraceDir = (FlatDirection + FVector(0.0f, 0.0f, -0.40f)).GetSafeNormal();
    
    FVector TraceEnd = TraceStart + (DownwardTraceDir * TraceDistance);

    FHitResult HitResult;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(GetOwner());
    Params.bTraceComplex = true;

    FCollisionShape SweepSphere = FCollisionShape::MakeSphere(SweepRadius);
    bool bHit = World->SweepSingleByChannel(HitResult, TraceStart, TraceEnd, FQuat::Identity, ECC_WorldStatic, SweepSphere, Params);

    if (bHit)
    {
        OutBraceData.bIsBracing = true;
        
        // Convert raw world hit info into Local Component space for Control Rig processing
        FVector LocalSpacePos = MeshWorldTransform.InverseTransformPosition(HitResult.ImpactPoint);
        FVector LocalSpaceNormal = MeshWorldTransform.InverseTransformVector(HitResult.ImpactNormal);
        
        // 3. LOW REST HEIGHT MODIFIERS:
        // Lower the incoming collision data out of the upper chest space and clamp it hard
        // to a grounded vertical band. The hand will never target heights exceeding your ceiling.
        LocalSpacePos.Z -= 100.0f; 
        float MaxBraceHeightRelativetoFloor = 220.0f; 
        LocalSpacePos.Z = FMath::Clamp(LocalSpacePos.Z, 40.0f, MaxBraceHeightRelativetoFloor);

        // 4. SMOOTH INTERPOLATION RUNTIME:
        // Drive coordinates independently to avoid frame-snapping when targets change
        OutBraceData.TargetHandLocation = FMath::VInterpTo(OutBraceData.TargetHandLocation, LocalSpacePos, DeltaTime, SmoothSpeed);
        OutBraceData.TargetHandNormal = FMath::VInterpTo(OutBraceData.TargetHandNormal, LocalSpaceNormal, DeltaTime, SmoothSpeed);

        // --- Live Hand Viewport Debugging System ---
        if (World)
        {
            FName HandBoneName = bIsLeft ? FName("hand_l") : FName("hand_r");
            FVector CurrentHandWorldPos = OwnerMesh->GetBoneLocation(HandBoneName);
            FVector TargetHandWorldPos = MeshWorldTransform.TransformPosition(OutBraceData.TargetHandLocation);

            // Bright Yellow line trailing from current hand to target
            DrawDebugLine(World, CurrentHandWorldPos, TargetHandWorldPos, FColor::Yellow, false, -1.0f, 0, 5.0f);
            
            // Cyan target sphere resting low against the structure skin
            DrawDebugSphere(World, TargetHandWorldPos, 15.0f, 8, FColor::Cyan, false, -1.0f, 0, 2.0f);
        }
    }
    else
    {
        OutBraceData.bIsBracing = false;
        
        // Return values back to base positions smoothly so walking cycles blend cleanly
        OutBraceData.TargetHandLocation = FMath::VInterpTo(OutBraceData.TargetHandLocation, FVector::ZeroVector, DeltaTime, SmoothSpeed);
        OutBraceData.TargetHandNormal = FMath::VInterpTo(OutBraceData.TargetHandNormal, FVector::UpVector, DeltaTime, SmoothSpeed);
    }

    // Secondary Sweep System Visualizers (Fires every frame regardless of hit status)
    if (World)
    {
        FColor DebugColor = bHit ? FColor::Green : FColor::Red;
        DrawDebugLine(World, TraceStart, TraceEnd, DebugColor, false, -1.0f, 0, 4.0f);
        DrawDebugSphere(World, TraceEnd, SweepRadius, 12, DebugColor, false, -1.0f, 0, 2.0f);
        if (bHit)
        {
            DrawDebugSphere(World, HitResult.ImpactPoint, 25.0f, 8, FColor::Magenta, false, -1.0f, 0, 3.0f);
        }
    }
}
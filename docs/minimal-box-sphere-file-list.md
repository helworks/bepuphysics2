# Minimal BEPU Box/Sphere Slice Inventory

## Intent

This file records the intended active vendored BEPU source surface for the immediate Helengine box/sphere integration milestone.

The immediate supported runtime scope is:

- `box-box`
- `sphere-sphere`
- `box-sphere`

The goal is to keep real upstream BEPU source while excluding unrelated families from the active project graph seen by `helengine.bepu`, `helengine.physics3d`, and native codegen.

## Core runtime entrypoints that must remain

- `BepuPhysics/Simulation.cs`
- `BepuPhysics/Bodies*.cs`
- `BepuPhysics/Statics.cs`
- `BepuPhysics/BodyDescription.cs`
- `BepuPhysics/RigidPose.cs`
- `BepuPhysics/BodyVelocity.cs`
- `BepuPhysics/Solver*.cs`
- `BepuPhysics/Island*.cs`
- `BepuPhysics/BatchCompressor.cs`
- `BepuPhysics/BoundingBoxHelpers.cs`
- `BepuPhysics/SimulationAllocationSizes.cs`
- `BepuPhysics/Collidables/Shapes.cs`
- `BepuPhysics/Collidables/TypedIndex.cs`
- `BepuPhysics/Collidables/Collidable*.cs`
- `BepuPhysics/Collidables/IShape.cs`
- `BepuPhysics/Collidables/Box.cs`
- `BepuPhysics/Collidables/Sphere.cs`
- `BepuPhysics/CollisionDetection/NarrowPhase*.cs`
- `BepuPhysics/CollisionDetection/CollisionBatcher*.cs`
- `BepuPhysics/CollisionDetection/CollisionTaskRegistry.cs`
- `BepuPhysics/CollisionDetection/ContactConstraintAccessor.cs`
- `BepuPhysics/CollisionDetection/ConvexContactManifoldWide.cs`
- `BepuPhysics/CollisionDetection/ContactManifold.cs`
- `BepuPhysics/CollisionDetection/CollidableOverlapFinder.cs`
- `BepuPhysics/CollisionDetection/BroadPhase*.cs`
- `BepuPhysics/CollisionDetection/DepthRefiner.cs`
- `BepuPhysics/CollisionDetection/FreshnessChecker.cs`
- `BepuPhysics/CollisionDetection/INarrowPhaseCallbacks.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/ConvexCollisionTask.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/PairTypes.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/ManifoldCandidateHelper.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/BoxPairTester.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/SpherePairTester.cs`
- `BepuPhysics/CollisionDetection/CollisionTasks/SphereBoxTester.cs`
- `BepuPhysics/Constraints/TypeProcessor.cs`
- `BepuPhysics/Constraints/TypeBatch.cs`
- `BepuPhysics/Constraints/OneBodyTypeProcessor.cs`
- `BepuPhysics/Constraints/TwoBodyTypeProcessor.cs`
- `BepuPhysics/Constraints/IConstraintDescription.cs`
- `BepuPhysics/Constraints/IBodyAccessFilter.cs`
- `BepuPhysics/Constraints/IBatchIntegrationMode.cs`
- `BepuPhysics/Constraints/SpringSettings.cs`
- `BepuPhysics/Constraints/Contact/*` limited to convex-contact support
- `BepuPhysics/Trees/*` only as directly required by active broad-phase maintenance

## Out-of-scope families that should be excluded from the minimal project

- mesh collidables and mesh reduction
- compounds and big compounds
- convex hulls
- capsules
- cylinders
- triangles as authored shape families
- broad sweep task implementations
- unused concrete constraint descriptions unrelated to contact solving
- nonconvex contact registrations and nonconvex contact type families
- ray/query convenience surfaces not required by the current rigid-body stepping path

## Immediate pruning mechanism

The first pruning pass uses a parallel `BepuPhysics.Minimal.csproj` and `BepuUtilities.Minimal.csproj`.

The minimal physics project:

- excludes the full upstream `DefaultTypes.cs`
- replaces it with a reduced `DefaultTypes` that only registers:
  - convex contact constraints
  - `SpherePairTester`
  - `SphereBoxTester`
  - `BoxPairTester`
- excludes the obvious out-of-scope shape, sweep, reduction, and unrelated constraint files at the project layer

This inventory is expected to widen slightly if compile-time evidence proves additional direct transitive dependencies are required by the supported box/sphere path.

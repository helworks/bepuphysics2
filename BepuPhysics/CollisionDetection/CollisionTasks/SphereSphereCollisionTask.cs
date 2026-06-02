using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks {
    /// <summary>
    /// Executes sphere-sphere collision batches using a concrete task type so native code generation does not depend on open generic task dispatch.
    /// </summary>
    public sealed class SphereSphereCollisionTask : CollisionTask {
        /// <summary>
        /// Creates a concrete sphere-sphere collision task for the reduced box and sphere runtime.
        /// </summary>
        public SphereSphereCollisionTask() {
            BatchSize = SpherePairTester.BatchSize;
            ShapeTypeIndexA = Sphere.TypeId;
            ShapeTypeIndexB = Sphere.TypeId;
            PairType = SpherePair.PairType;
        }

        /// <summary>
        /// Executes a sphere-sphere batch and reports every resulting convex manifold back through the batcher callbacks.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the collision callback sink receiving manifold results.</typeparam>
        /// <param name="batch">Untyped pair batch containing packed <see cref="SpherePair"/> entries.</param>
        /// <param name="batcher">Owning batcher that receives finished contact manifolds.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher) {
            ref SpherePair start = ref Unsafe.As<byte, SpherePair>(ref batch.Buffer[0]);
            SpherePairWide pairWide = default;
            ref SphereWide aWide = ref SpherePairWide.GetShapeA(ref pairWide);
            ref SphereWide bWide = ref SpherePairWide.GetShapeB(ref pairWide);
            Convex1ContactManifoldWide manifoldWide;
            ConvexContactManifold manifold = default;

            for (int i = 0; i < batch.Count; i += Vector<float>.Count) {
                ref SpherePair bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count) {
                    countInBundle = Vector<float>.Count;
                }

                for (int j = 0; j < countInBundle; ++j) {
                    pairWide.WriteSlot(j, Unsafe.Add(ref bundleStart, j));
                }

                SpherePairTester.Test(
                    ref aWide,
                    ref bWide,
                    ref SpherePairWide.GetSpeculativeMargin(ref pairWide),
                    ref SpherePairWide.GetOffsetB(ref pairWide),
                    countInBundle,
                    out manifoldWide);

                for (int j = 0; j < countInBundle; ++j) {
                    ref var manifoldSource = ref GetOffsetInstance(ref manifoldWide, j);
                    ref var offsetSource = ref GetOffsetInstance(ref SpherePairWide.GetOffsetB(ref pairWide), j);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref SpherePair pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(ref manifold, ref SpherePair.GetContinuation(ref pair));
                }
            }
        }
    }
}

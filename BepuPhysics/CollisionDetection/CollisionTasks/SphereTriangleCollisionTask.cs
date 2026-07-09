using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks {
    /// <summary>
    /// Executes sphere-triangle collision batches using a concrete task type so native code generation can dispatch without open generic task casts.
    /// </summary>
    public sealed class SphereTriangleCollisionTask : CollisionTask {
        /// <summary>
        /// Creates a concrete sphere-triangle collision task for the reduced Helengine runtime.
        /// </summary>
        public SphereTriangleCollisionTask() {
            BatchSize = SphereTriangleTester.BatchSize;
            ShapeTypeIndexA = Sphere.TypeId;
            ShapeTypeIndexB = Triangle.TypeId;
            PairType = SphereIncludingPair.PairType;
        }

        /// <summary>
        /// Executes a sphere-triangle batch and reports every resulting convex manifold back through the batcher callbacks.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the collision callback sink receiving manifold results.</typeparam>
        /// <param name="batch">Untyped pair batch containing packed <see cref="SphereIncludingPair"/> entries.</param>
        /// <param name="batcher">Owning batcher that receives finished contact manifolds.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher) {
            ref SphereIncludingPair start = ref Unsafe.As<byte, SphereIncludingPair>(ref batch.Buffer[0]);
            SphereIncludingPairWide<Triangle, TriangleWide> pairWide = default;
            ref SphereWide aWide = ref SphereIncludingPairWide<Triangle, TriangleWide>.GetShapeA(ref pairWide);
            ref TriangleWide bWide = ref SphereIncludingPairWide<Triangle, TriangleWide>.GetShapeB(ref pairWide);
            if (aWide.InternalAllocationSize > 0) {
                byte* memory = stackalloc byte[aWide.InternalAllocationSize];
                aWide.Initialize(new Buffer<byte>(memory, aWide.InternalAllocationSize));
            }
            if (bWide.InternalAllocationSize > 0) {
                byte* memory = stackalloc byte[bWide.InternalAllocationSize];
                bWide.Initialize(new Buffer<byte>(memory, bWide.InternalAllocationSize));
            }

            Convex1ContactManifoldWide manifoldWide;
            ConvexContactManifold manifold = default;

            for (int i = 0; i < batch.Count; i += Vector<float>.Count) {
                ref SphereIncludingPair bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count) {
                    countInBundle = Vector<float>.Count;
                }

                for (int j = 0; j < countInBundle; ++j) {
                    pairWide.WriteSlot(j, Unsafe.Add(ref bundleStart, j));
                }

                SphereTriangleTester.Test(
                    ref aWide,
                    ref bWide,
                    ref SphereIncludingPairWide<Triangle, TriangleWide>.GetSpeculativeMargin(ref pairWide),
                    ref SphereIncludingPairWide<Triangle, TriangleWide>.GetOffsetB(ref pairWide),
                    ref SphereIncludingPairWide<Triangle, TriangleWide>.GetOrientationB(ref pairWide),
                    countInBundle,
                    out manifoldWide);

                if (SphereIncludingPairWide<Triangle, TriangleWide>.HasFlipMask) {
                    manifoldWide.ApplyFlipMask(ref SphereIncludingPairWide<Triangle, TriangleWide>.GetOffsetB(ref pairWide), SphereIncludingPairWide<Triangle, TriangleWide>.GetFlipMask(ref pairWide));
                }

                for (int j = 0; j < countInBundle; ++j) {
                    ref var manifoldSource = ref GetOffsetInstance(ref manifoldWide, j);
                    ref var offsetSource = ref GetOffsetInstance(ref SphereIncludingPairWide<Triangle, TriangleWide>.GetOffsetB(ref pairWide), j);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref SphereIncludingPair pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(ref manifold, ref SphereIncludingPair.GetContinuation(ref pair));
                }
            }
        }
    }
}

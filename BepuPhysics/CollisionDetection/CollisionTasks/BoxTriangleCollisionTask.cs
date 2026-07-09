using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks {
    /// <summary>
    /// Executes box-triangle collision batches using a concrete task type so native code generation can dispatch without open generic task casts.
    /// </summary>
    public sealed class BoxTriangleCollisionTask : CollisionTask {
        /// <summary>
        /// Creates a concrete box-triangle collision task for the reduced Helengine runtime.
        /// </summary>
        public BoxTriangleCollisionTask() {
            BatchSize = BoxTriangleTester.BatchSize;
            ShapeTypeIndexA = Box.TypeId;
            ShapeTypeIndexB = Triangle.TypeId;
            PairType = CollisionPair.PairType;
        }

        /// <summary>
        /// Executes a box-triangle batch and reports every resulting convex manifold back through the batcher callbacks.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the collision callback sink receiving manifold results.</typeparam>
        /// <param name="batch">Untyped pair batch containing packed <see cref="CollisionPair"/> entries.</param>
        /// <param name="batcher">Owning batcher that receives finished contact manifolds.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher) {
            ref CollisionPair start = ref Unsafe.As<byte, CollisionPair>(ref batch.Buffer[0]);
            ConvexPairWide<Box, BoxWide, Triangle, TriangleWide> pairWide = default;
            ref BoxWide aWide = ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetShapeA(ref pairWide);
            ref TriangleWide bWide = ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetShapeB(ref pairWide);
            if (aWide.InternalAllocationSize > 0) {
                byte* memory = stackalloc byte[aWide.InternalAllocationSize];
                aWide.Initialize(new Buffer<byte>(memory, aWide.InternalAllocationSize));
            }
            if (bWide.InternalAllocationSize > 0) {
                byte* memory = stackalloc byte[bWide.InternalAllocationSize];
                bWide.Initialize(new Buffer<byte>(memory, bWide.InternalAllocationSize));
            }

            Convex4ContactManifoldWide manifoldWide;
            ConvexContactManifold manifold = default;

            for (int i = 0; i < batch.Count; i += Vector<float>.Count) {
                ref CollisionPair bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count) {
                    countInBundle = Vector<float>.Count;
                }

                for (int j = 0; j < countInBundle; ++j) {
                    pairWide.WriteSlot(j, Unsafe.Add(ref bundleStart, j));
                }

                BoxTriangleTester.Test(
                    ref aWide,
                    ref bWide,
                    ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetSpeculativeMargin(ref pairWide),
                    ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetOffsetB(ref pairWide),
                    ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetOrientationA(ref pairWide),
                    ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetOrientationB(ref pairWide),
                    countInBundle,
                    out manifoldWide);

                for (int j = 0; j < countInBundle; ++j) {
                    ref var manifoldSource = ref GetOffsetInstance(ref manifoldWide, j);
                    ref var offsetSource = ref GetOffsetInstance(ref ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>.GetOffsetB(ref pairWide), j);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref CollisionPair pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(ref manifold, ref CollisionPair.GetContinuation(ref pair));
                }
            }
        }
    }
}

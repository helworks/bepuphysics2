using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks {
    /// <summary>
    /// Executes box-box collision batches using a concrete task type so native code generation can dispatch without open generic task casts.
    /// </summary>
    public sealed class BoxBoxCollisionTask : CollisionTask {
        /// <summary>
        /// Creates a concrete box-box collision task for the reduced box and sphere runtime.
        /// </summary>
        public BoxBoxCollisionTask() {
            BatchSize = BoxPairTester.BatchSize;
            ShapeTypeIndexA = Box.TypeId;
            ShapeTypeIndexB = Box.TypeId;
            PairType = FliplessPair.PairType;
        }

        /// <summary>
        /// Executes a box-box batch and reports every resulting convex manifold back through the batcher callbacks.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the collision callback sink receiving manifold results.</typeparam>
        /// <param name="batch">Untyped pair batch containing packed <see cref="FliplessPair"/> entries.</param>
        /// <param name="batcher">Owning batcher that receives finished contact manifolds.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher) {
            ref FliplessPair start = ref Unsafe.As<byte, FliplessPair>(ref batch.Buffer[0]);
            FliplessPairWide<Box, BoxWide> pairWide = default;
            ref BoxWide aWide = ref FliplessPairWide<Box, BoxWide>.GetShapeA(ref pairWide);
            ref BoxWide bWide = ref FliplessPairWide<Box, BoxWide>.GetShapeB(ref pairWide);
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
                ref FliplessPair bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count) {
                    countInBundle = Vector<float>.Count;
                }

                for (int j = 0; j < countInBundle; ++j) {
                    pairWide.WriteSlot(j, Unsafe.Add(ref bundleStart, j));
                }

                BoxPairTester.Test(
                    ref aWide,
                    ref bWide,
                    ref FliplessPairWide<Box, BoxWide>.GetSpeculativeMargin(ref pairWide),
                    ref FliplessPairWide<Box, BoxWide>.GetOffsetB(ref pairWide),
                    ref FliplessPairWide<Box, BoxWide>.GetOrientationA(ref pairWide),
                    ref FliplessPairWide<Box, BoxWide>.GetOrientationB(ref pairWide),
                    countInBundle,
                    out manifoldWide);

                for (int j = 0; j < countInBundle; ++j) {
                    ref var manifoldSource = ref GetOffsetInstance(ref manifoldWide, j);
                    ref var offsetSource = ref GetOffsetInstance(ref FliplessPairWide<Box, BoxWide>.GetOffsetB(ref pairWide), j);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref FliplessPair pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(ref manifold, ref FliplessPair.GetContinuation(ref pair));
                }
            }
        }
    }
}

using BepuUtilities.Memory;
using System;

namespace BepuPhysics.CollisionDetection
{
    public struct CompoundMeshReduction : ICollisionTestContinuation
    {
        static Exception CreateUnsupportedException()
        {
            return new NotSupportedException("Compound-mesh reduction is not available in the minimal BEPU box/sphere slice.");
        }

        public void Create(int slots, BufferPool pool)
        {
        }

        public void OnChildCompleted<TCallbacks>(ref PairContinuation report, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            throw CreateUnsupportedException();
        }

        public void OnUntestedChildCompleted<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            throw CreateUnsupportedException();
        }

        public bool TryFlush<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            throw CreateUnsupportedException();
        }
    }
}

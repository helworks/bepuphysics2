using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    public abstract class ShapeBatch
    {
        protected Buffer<byte> shapesData;
        protected int shapeDataSize;
        public int Capacity { get { return shapesData.Length / shapeDataSize; } }
        protected BufferPool pool;
        protected IdPool idPool;
        public int TypeId { get; protected set; }
        public bool Compound { get; protected set; }
        public int ShapeDataSize { get { return shapeDataSize; } }

        protected abstract void Dispose(int index, BufferPool pool);
        protected abstract void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool);

        public void Remove(int index)
        {
            idPool.Return(index, pool);
        }

        public void RemoveAndDispose(int index, BufferPool pool)
        {
            Dispose(index, pool);
            Remove(index);
        }

        public void RecursivelyRemoveAndDispose(int index, Shapes shapes, BufferPool pool)
        {
            RemoveAndDisposeChildren(index, shapes, pool);
            RemoveAndDispose(index, pool);
        }

        public abstract void ComputeBounds(ref BoundingBoxBatcher batcher);
        public abstract void ComputeBounds(int shapeIndex, Quaternion orientation, out Vector3 min, out Vector3 max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeBounds(int shapeIndex, Vector3 position, Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            ComputeBounds(shapeIndex, orientation, out min, out max);
            min += position;
            max += position;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeBounds(int shapeIndex, RigidPose pose, out Vector3 min, out Vector3 max)
        {
            ComputeBounds(shapeIndex, pose.Orientation, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }

        internal virtual void ComputeBounds(int shapeIndex, Quaternion orientation, out float maximumRadius, out float maximumAngularExpansion, out Vector3 min, out Vector3 max)
        {
            throw new InvalidOperationException("Nonconvex shapes are not required to have a maximum radius or angular expansion implementation. This should only ever be called on convexes.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetShapeData(int shapeIndex, out void* shapePointer, out int shapeSize)
        {
            Debug.Assert(shapeIndex >= 0 && shapeIndex < Capacity);
            shapePointer = shapesData.Memory + shapeDataSize * shapeIndex;
            shapeSize = shapeDataSize;
        }

        public abstract void Clear();
        public abstract void EnsureCapacity(int shapeCapacity);
        public abstract void Resize(int shapeCapacity);
        public abstract void Dispose();

        public void ResizeIdPool(int targetIdCapacity)
        {
            idPool.Resize(targetIdCapacity, pool);
        }
    }

    public abstract class ShapeBatch<TShape> : ShapeBatch where TShape : unmanaged, IShape
    {
        internal Buffer<TShape> shapes;

        public ref TShape this[int shapeIndex] { get { return ref shapes[shapeIndex]; } }

        protected ShapeBatch(BufferPool pool, int initialShapeCount)
        {
            this.pool = pool;
            TypeId = TShape.TypeId;
            InternalResize(initialShapeCount, 0);
            idPool = new IdPool(initialShapeCount, pool);
        }

        public int Add(in TShape shape)
        {
            var shapeIndex = idPool.Take();
            if (shapes.Length <= shapeIndex)
            {
                InternalResize(shapeIndex + 1, shapes.Length);
            }
            shapes[shapeIndex] = shape;
            return shapeIndex;
        }

        void InternalResize(int shapeCount, int oldCopyLength)
        {
            shapeDataSize = Unsafe.SizeOf<TShape>();
            var requiredSizeInBytes = shapeCount * Unsafe.SizeOf<TShape>();
            pool.TakeAtLeast<byte>(requiredSizeInBytes, out var newShapesData);
            var newShapes = newShapesData.As<TShape>();
#if DEBUG
            if (newShapes.Length > shapes.Length)
                newShapes.Clear(shapes.Length, newShapes.Length - shapes.Length);
#endif
            if (shapesData.Allocated)
            {
                shapes.CopyTo(0, newShapes, 0, oldCopyLength);
                pool.Return(ref shapesData);
            }
            else
            {
                Debug.Assert(oldCopyLength == 0);
            }
            shapes = newShapes;
            shapesData = newShapesData;
        }

        public override void Clear()
        {
#if DEBUG
            shapes.Clear(0, idPool.HighestPossiblyClaimedId + 1);
#endif
            idPool.Clear();
        }

        public override void EnsureCapacity(int shapeCapacity)
        {
            if (shapes.Length < shapeCapacity)
            {
                InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
            }
        }

        public override void Resize(int shapeCapacity)
        {
            shapeCapacity = BufferPool.GetCapacityForCount<TShape>(Math.Max(idPool.HighestPossiblyClaimedId + 1, shapeCapacity));
            if (shapeCapacity != shapes.Length)
            {
                InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
            }
        }

        public override void Dispose()
        {
            Debug.Assert(shapesData.Id == shapes.Id, "If the buffer ids don't match, there was some form of failed resize.");
            pool.Return(ref shapesData);
            idPool.Dispose(pool);
        }
    }

    public interface IConvexShapeBatch
    {
        BodyInertia ComputeInertia(int shapeIndex, float mass);
    }

    public class ConvexShapeBatch<TShape, TShapeWide> : ShapeBatch<TShape>, IConvexShapeBatch
        where TShape : unmanaged, IConvexShape
        where TShapeWide : unmanaged, IShapeWide<TShape>
    {
        public ConvexShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
        }

        protected override void Dispose(int index, BufferPool pool)
        {
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
        }

        public BodyInertia ComputeInertia(int shapeIndex, float mass)
        {
            return shapes[shapeIndex].ComputeInertia(mass);
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteConvexBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].ComputeBounds(orientation, out min, out max);
        }

        internal override void ComputeBounds(int shapeIndex, Quaternion orientation, out float maximumRadius, out float angularExpansion, out Vector3 min, out Vector3 max)
        {
            ref var shape = ref shapes[shapeIndex];
            shape.ComputeBounds(orientation, out min, out max);
            shape.ComputeAngularExpansionData(out maximumRadius, out angularExpansion);
        }
    }

    public class HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> : ShapeBatch<TShape>
        where TShape : unmanaged, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
        where TChildShape : unmanaged, IConvexShape
        where TChildShapeWide : unmanaged, IShapeWide<TChildShape>
    {
        public HomogeneousCompoundShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
            Compound = true;
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            throw CreateUnsupportedException();
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
            throw CreateUnsupportedException();
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            throw CreateUnsupportedException();
        }

        public override void ComputeBounds(int shapeIndex, Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            throw CreateUnsupportedException();
        }

        static Exception CreateUnsupportedException()
        {
            return new NotSupportedException("Homogeneous compounds are not available in the minimal BEPU box/sphere slice.");
        }
    }

    public class CompoundShapeBatch<TShape> : ShapeBatch<TShape> where TShape : unmanaged, ICompoundShape
    {
        public CompoundShapeBatch(BufferPool pool, int initialShapeCount, Shapes shapeBatches) : base(pool, initialShapeCount)
        {
            Compound = true;
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            throw CreateUnsupportedException();
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
            throw CreateUnsupportedException();
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            throw CreateUnsupportedException();
        }

        public override void ComputeBounds(int shapeIndex, Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            throw CreateUnsupportedException();
        }

        static Exception CreateUnsupportedException()
        {
            return new NotSupportedException("Compound shapes are not available in the minimal BEPU box/sphere slice.");
        }
    }

    public class Shapes
    {
        ShapeBatch[] batches;
        int registeredTypeSpan;
        public int RegisteredTypeSpan => registeredTypeSpan;
        public int InitialCapacityPerTypeBatch { get; set; }
        public ShapeBatch this[int typeIndex] => batches[typeIndex];
        BufferPool pool;

        public Shapes(BufferPool pool, int initialCapacityPerTypeBatch)
        {
            InitialCapacityPerTypeBatch = initialCapacityPerTypeBatch;
            batches = new ShapeBatch[16];
            this.pool = pool;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateBounds(RigidPose pose, TypedIndex shapeIndex, out BoundingBox bounds)
        {
            batches[shapeIndex.Type].ComputeBounds(shapeIndex.Index, pose, out bounds.Min, out bounds.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateBounds(Vector3 position, Quaternion orientation, TypedIndex shapeIndex, out BoundingBox bounds)
        {
            batches[shapeIndex.Type].ComputeBounds(shapeIndex.Index, position, orientation, out bounds.Min, out bounds.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShape GetShape<TShape>(int shapeIndex) where TShape : unmanaged, IShape
        {
            var typeId = TShape.TypeId;
            return ref ((ShapeBatch<TShape>)batches[typeId])[shapeIndex];
        }

        public TypedIndex Add<TShape>(in TShape shape) where TShape : unmanaged, IShape
        {
            var typeId = TShape.TypeId;
            if (RegisteredTypeSpan <= typeId)
            {
                registeredTypeSpan = typeId + 1;
                if (batches.Length <= typeId)
                {
                    Array.Resize(ref batches, typeId + 1);
                }
            }
            if (batches[typeId] == null)
            {
                batches[typeId] = TShape.CreateShapeBatch(pool, InitialCapacityPerTypeBatch, this);
            }

            Debug.Assert(batches[typeId] is ShapeBatch<TShape>);
            var batch = (ShapeBatch<TShape>)batches[typeId];
            var index = batch.Add(shape);
            return new TypedIndex(typeId, index);
        }

        public void RecursivelyRemoveAndDispose(TypedIndex shapeIndex, BufferPool pool)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].RecursivelyRemoveAndDispose(shapeIndex.Index, this, pool);
            }
        }

        public void RemoveAndDispose(TypedIndex shapeIndex, BufferPool pool)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].RemoveAndDispose(shapeIndex.Index, pool);
            }
        }

        public void Remove(TypedIndex shapeIndex)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].Remove(shapeIndex.Index);
            }
        }

        public void Clear()
        {
            for (int i = 0; i < registeredTypeSpan; ++i)
            {
                if (batches[i] != null)
                    batches[i].Clear();
            }
        }

        public void EnsureBatchCapacities(int shapeCapacity)
        {
            for (int i = 0; i < registeredTypeSpan; ++i)
            {
                if (batches[i] != null)
                    batches[i].EnsureCapacity(shapeCapacity);
            }
        }

        public void ResizeBatches(int shapeCapacity)
        {
            for (int i = 0; i < registeredTypeSpan; ++i)
            {
                if (batches[i] != null)
                    batches[i].Resize(shapeCapacity);
            }
        }

        public void Dispose()
        {
            for (int i = 0; i < registeredTypeSpan; ++i)
            {
                if (batches[i] != null)
                    batches[i].Dispose();
            }
        }
    }
}

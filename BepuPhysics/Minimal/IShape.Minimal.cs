using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape
    {
        /// <summary>
        /// Unique type id for this shape type.
        /// </summary>
        static abstract int TypeId { get; }
        /// <summary>
        /// Creates a shape batch for this type of shape.
        /// </summary>
        /// <param name="pool">Buffer pool used to create the batch.</param>
        /// <param name="initialCapacity">Initial capacity to allocate within the batch.</param>
        /// <param name="shapeBatches">The set of shapes to contain this batch.</param>
        /// <returns>Shape batch for the shape type.</returns>
        static abstract ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches);
    }

    /// <summary>
    /// Defines functions available on all convex shapes.
    /// </summary>
    public interface IConvexShape : IShape
    {
        /// <summary>
        /// Computes the bounding box of a shape given an orientation.
        /// </summary>
        /// <param name="orientation">Orientation of the shape to use when computing the bounding box.</param>
        /// <param name="min">Minimum corner of the bounding box.</param>
        /// <param name="max">Maximum corner of the bounding box.</param>
        void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max);

        /// <summary>
        /// Computes information about how the bounding box should be expanded in response to angular velocity.
        /// </summary>
        /// <param name="maximumRadius">Maximum radius from the center of mass to any point on the shape.</param>
        /// <param name="maximumAngularExpansion">Maximum expansion caused by angular velocity.</param>
        void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion);

        /// <summary>
        /// Computes the inertia for a body given a mass.
        /// </summary>
        /// <param name="mass">Mass to use to compute the body's inertia.</param>
        /// <returns>Inertia for the body.</returns>
        BodyInertia ComputeInertia(float mass);
    }

    /// <summary>
    /// Defines the minimal compound API surface preserved for shared batching code.
    /// </summary>
    public interface ICompoundShape : IDisposableShape
    {
        /// <summary>
        /// Computes the bounding box of the compound shape.
        /// </summary>
        /// <param name="orientation">Orientation of the compound.</param>
        /// <param name="shapeBatches">Shape batches used for any child lookups.</param>
        /// <param name="min">Minimum corner of the bounds.</param>
        /// <param name="max">Maximum corner of the bounds.</param>
        void ComputeBounds(Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max);
        /// <summary>
        /// Adds any child bounds required by the compound to the bounding box batcher.
        /// </summary>
        /// <param name="batcher">Batcher to populate.</param>
        /// <param name="pose">Compound pose.</param>
        /// <param name="velocity">Compound velocity.</param>
        /// <param name="bodyIndex">Body index associated with the compound.</param>
        void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, in RigidPose pose, in BodyVelocity velocity, int bodyIndex);
    }

    /// <summary>
    /// Defines the minimal homogeneous compound API surface preserved for shared batching code.
    /// </summary>
    /// <typeparam name="TChildShape">Child shape type.</typeparam>
    /// <typeparam name="TChildShapeWide">Wide child shape type.</typeparam>
    public interface IHomogeneousCompoundShape<TChildShape, TChildShapeWide> : IDisposableShape
        where TChildShape : unmanaged, IConvexShape
        where TChildShapeWide : unmanaged, IShapeWide<TChildShape>
    {
        /// <summary>
        /// Computes the bounding box of the compound shape.
        /// </summary>
        /// <param name="orientation">Orientation of the compound.</param>
        /// <param name="min">Minimum corner of the bounds.</param>
        /// <param name="max">Maximum corner of the bounds.</param>
        void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max);
    }

    /// <summary>
    /// Defines a widely vectorized bundle representation of a shape.
    /// </summary>
    /// <typeparam name="TShape">Scalar type of the shape.</typeparam>
    public interface IShapeWide<TShape> where TShape : IShape
    {
        /// <summary>
        /// Gets whether this type supports accessing its memory by lane offsets.
        /// </summary>
        bool AllowOffsetMemoryAccess { get; }
        /// <summary>
        /// Gets the number of bytes required for allocations within the wide shape.
        /// </summary>
        int InternalAllocationSize { get; }
        /// <summary>
        /// Provides memory to the shape for internal allocations.
        /// </summary>
        /// <param name="memory">Memory to use for internal allocations in the wide shape.</param>
        void Initialize(in Buffer<byte> memory);
        /// <summary>
        /// Places the specified AOS-formatted shape into the first lane of the wide reference.
        /// </summary>
        /// <param name="source">AOS-formatted shape to gather from.</param>
        void WriteFirst(in TShape source);
        /// <summary>
        /// Places the specified AOS-formatted shape into the selected slot of the wide reference.
        /// </summary>
        /// <param name="index">Index of the slot to put the data into.</param>
        /// <param name="source">Source of the data to insert.</param>
        void WriteSlot(int index, in TShape source);
        /// <summary>
        /// Broadcasts a scalar shape into a bundle containing the same shape in every lane.
        /// </summary>
        /// <param name="shape">Scalar shape to broadcast.</param>
        void Broadcast(in TShape shape);
        /// <summary>
        /// Computes the bounds of all shapes in the bundle.
        /// </summary>
        /// <param name="orientations">Orientations of the shapes in the bundle.</param>
        /// <param name="countInBundle">Number of lanes filled in the bundle.</param>
        /// <param name="maximumRadius">Computed maximum radius of the shapes in the bundle.</param>
        /// <param name="maximumAngularExpansion">Computed maximum bounds expansion that can be caused by angular motion.</param>
        /// <param name="min">Minimum bounds of the shapes.</param>
        /// <param name="max">Maximum bounds of the shapes.</param>
        void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max);
    }
}

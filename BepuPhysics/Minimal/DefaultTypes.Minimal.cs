using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics {
    /// <summary>
    /// Registers only the real upstream BEPU contact and pair-task types required by the reduced Helengine box and sphere slice.
    /// </summary>
    public static class DefaultTypes {
        /// <summary>
        /// Registers the supported convex contact constraint types used by the reduced box and sphere runtime.
        /// </summary>
        /// <param name="solver">Solver that receives the supported contact constraint descriptions.</param>
        /// <param name="narrowPhase">Narrow phase that receives the contact constraint accessors.</param>
        public static void RegisterDefaults(Solver solver, NarrowPhase narrowPhase) {
            solver.Register<Contact1OneBody>();
            solver.Register<Contact2OneBody>();
            solver.Register<Contact3OneBody>();
            solver.Register<Contact4OneBody>();
            solver.Register<Contact1>();
            solver.Register<Contact2>();
            solver.Register<Contact3>();
            solver.Register<Contact4>();
            solver.Register<Contact2NonconvexOneBody>();
            solver.Register<Contact3NonconvexOneBody>();
            solver.Register<Contact4NonconvexOneBody>();
            solver.Register<Contact2Nonconvex>();
            solver.Register<Contact3Nonconvex>();
            solver.Register<Contact4Nonconvex>();

            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact4Nonconvex, Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact3Nonconvex, Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact2Nonconvex, Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses, ContactImpulses2>());

            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses, ContactImpulses2>());

            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact4, Contact4PrestepData, Contact4AccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact3, Contact3PrestepData, Contact3AccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact2, Contact2PrestepData, Contact2AccumulatedImpulses, ContactImpulses2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact1, Contact1PrestepData, Contact1AccumulatedImpulses, ContactImpulses1>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact4OneBody, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact3OneBody, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact2OneBody, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses, ContactImpulses2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact1OneBody, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses, ContactImpulses1>());
        }

        /// <summary>
        /// Creates a collision task registry containing only the supported box and sphere pair handlers.
        /// </summary>
        /// <returns>Collision task registry for the reduced box and sphere runtime.</returns>
        public static CollisionTaskRegistry CreateDefaultCollisionTaskRegistry() {
            CollisionTaskRegistry defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new SphereSphereCollisionTask());
            defaultTaskRegistry.Register(new SphereBoxCollisionTask());
            defaultTaskRegistry.Register(new SphereTriangleCollisionTask());
            defaultTaskRegistry.Register(new BoxBoxCollisionTask());
            defaultTaskRegistry.Register(new BoxTriangleCollisionTask());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, Mesh, ConvexCompoundOverlapFinder<Sphere, SphereWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Box, Mesh, ConvexCompoundOverlapFinder<Box, BoxWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());
            return defaultTaskRegistry;
        }

        /// <summary>
        /// Creates an empty sweep task registry for the reduced box and sphere runtime.
        /// </summary>
        /// <returns>Empty sweep task registry.</returns>
        public static SweepTaskRegistry CreateDefaultSweepTaskRegistry() {
            return new SweepTaskRegistry();
        }
    }
}

using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    public static class Compound
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPose localPose, Quaternion orientation, out RigidPose rotatedChildPose)
        {
            GetRotatedChildPose(localPose.Position, localPose.Orientation, orientation, out rotatedChildPose.Position, out rotatedChildPose.Orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(Vector3 localPosition, Quaternion localOrientation, Quaternion parentOrientation, out Vector3 rotatedPosition, out Quaternion rotatedOrientation)
        {
            QuaternionEx.TransformWithoutOverlap(localPosition, parentOrientation, out rotatedPosition);
            QuaternionEx.ConcatenateWithoutOverlap(localOrientation, parentOrientation, out rotatedOrientation);
        }
    }
}

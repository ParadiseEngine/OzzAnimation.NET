using System.Numerics;

namespace OzzAnimation;

/// <summary>
/// Aims one joint's forward axis at a target, rolling it about that axis so its up axis leans
/// toward a pole vector. ozz's <c>IKAimJob</c> — the look-at half of the IK pair, used for heads,
/// eyes and gun barrels.
/// </summary>
/// <remarks>
/// Returns a correction for the joint's local rotation; nothing is written to the skeleton, and
/// the chain must be re-run through <see cref="LocalToModel"/> afterwards. Chaining the job up a
/// hierarchy (eyes, then head, then chest) with decreasing weights is how ozz spreads a look-at
/// over several joints. Allocation-free.
/// </remarks>
public struct IKAimJob()
{
    /// <summary>Position to aim at, in model space.</summary>
    public Vector3 Target = Vector3.Zero;

    /// <summary>The joint-local axis that is aimed at the target. Must be unit length.</summary>
    public Vector3 Forward = Vector3.UnitX;

    /// <summary>Joint-local position that is actually aimed — an eye offset from the head's pivot, say. Zero aims the joint's own origin.</summary>
    public Vector3 Offset = Vector3.Zero;

    /// <summary>The joint-local axis rolled toward <see cref="PoleVector"/>, which fixes the rotation left about the forward axis.</summary>
    public Vector3 Up = Vector3.UnitY;

    /// <summary>Model-space direction <see cref="Up"/> is rolled toward — "keep the head upright".</summary>
    public Vector3 PoleVector = Vector3.UnitY;

    /// <summary>Extra rotation about the joint-to-target axis, radians, applied on top of the pole vector.</summary>
    public float TwistAngle = 0f;

    /// <summary>How much of the correction to apply, 0..1. At 0 the correction is identity; below 1 it lerps from identity, which is how a look-at is spread across a chain.</summary>
    public float Weight = 1f;

    /// <summary>Model-space matrix of the joint being aimed.</summary>
    public Matrix4x4 Joint = Matrix4x4.Identity;

    /// <summary>Whether these arguments would run: <see cref="Forward"/> must be normalized.</summary>
    public readonly bool Validate() => MathF.Abs(Forward.LengthSquared() - 1f) < 2e-3f;

    /// <summary>Aims the joint.</summary>
    /// <param name="correction">Rotation to apply to the joint's local rotation.</param>
    /// <param name="reached">False when <see cref="Offset"/> puts the aimed point outside the sphere of radius |target|, so no direction can hit the target.</param>
    /// <returns>False when <see cref="Validate"/> would fail; the correction is identity then.</returns>
    public readonly bool Run(out Quaternion correction, out bool reached)
    {
        correction = Quaternion.Identity;
        reached = false;
        if (!Validate()) return false;

        var inverseJoint = SoaMath.InvertOrZero(Joint);
        var jointToTargetJs = Vector3.Transform(Target, inverseJoint);
        var jointToTargetJsLengthSquared = jointToTargetJs.LengthSquared();

        reached = ComputeOffsetForward(Forward, Offset, jointToTargetJs, out var offsetForward);
        if (!reached || jointToTargetJsLengthSquared == 0f) return true;

        var jointToTargetRotationJs = SoaMath.QuaternionFromVectors(offsetForward, jointToTargetJs);

        // With the aim solved, one degree of freedom is left — the roll about the aim axis. It is
        // fixed by rotating the plane through the joint's up axis onto the plane through the pole.
        var correctedUpJs = Vector3.Transform(Up, jointToTargetRotationJs);
        var poleVectorJs = Vector3.TransformNormal(PoleVector, inverseJoint);
        var referenceNormalJs = Vector3.Cross(poleVectorJs, jointToTargetJs);
        var jointNormalJs = Vector3.Cross(correctedUpJs, jointToTargetJs);
        var referenceNormalLengthSquared = referenceNormalJs.LengthSquared();
        var jointNormalLengthSquared = jointNormalJs.LengthSquared();

        Vector3 rotatePlaneAxisJs;
        Quaternion rotatePlaneJs;
        if (jointToTargetJsLengthSquared != 0f && jointNormalLengthSquared != 0f && referenceNormalLengthSquared != 0f)
        {
            rotatePlaneAxisJs = jointToTargetJs / MathF.Sqrt(jointToTargetJsLengthSquared);
            var rotatePlaneCos = Math.Clamp(
                Vector3.Dot(jointNormalJs / MathF.Sqrt(jointNormalLengthSquared), referenceNormalJs / MathF.Sqrt(referenceNormalLengthSquared)),
                -1f, 1f);
            var flipped = Vector3.Dot(referenceNormalJs, correctedUpJs) < 0f ? -rotatePlaneAxisJs : rotatePlaneAxisJs;
            rotatePlaneJs = SoaMath.QuaternionFromAxisCosAngle(flipped, rotatePlaneCos);
        }
        else
        {
            // Up is parallel to the aim direction, or the pole is: no roll is defined.
            rotatePlaneAxisJs = jointToTargetJs / MathF.Sqrt(jointToTargetJsLengthSquared);
            rotatePlaneJs = Quaternion.Identity;
        }

        var twisted = TwistAngle == 0f
            ? rotatePlaneJs * jointToTargetRotationJs
            : Quaternion.CreateFromAxisAngle(rotatePlaneAxisJs, TwistAngle) * rotatePlaneJs * jointToTargetRotationJs;

        correction = SoaMath.WeightTowardIdentity(twisted, Weight);
        return true;
    }

    /// <summary>
    /// With an offset, the vector to aim is no longer <see cref="Forward"/> from the joint's
    /// origin: it runs from the origin to where a line from the offset along forward meets the
    /// sphere of radius |target| centred on the joint. Returns false when the offset lies outside
    /// that sphere, where no such intersection exists.
    /// </summary>
    private static bool ComputeOffsetForward(Vector3 forward, Vector3 offset, Vector3 target, out Vector3 offsetForward)
    {
        var alongForward = Vector3.Dot(forward, offset);
        var perpendicularLengthSquared = offset.LengthSquared() - alongForward * alongForward;
        var radiusSquared = target.LengthSquared();
        if (perpendicularLengthSquared > radiusSquared)
        {
            offsetForward = forward;
            return false;
        }

        var toIntersection = MathF.Sqrt(radiusSquared - perpendicularLengthSquared);
        offsetForward = offset + forward * (toIntersection - alongForward);
        return true;
    }
}

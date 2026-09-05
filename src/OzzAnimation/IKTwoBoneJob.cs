using System.Numerics;

namespace OzzAnimation;

/// <summary>
/// Analytic two-bone IK: given the model-space matrices of a chain's start, mid and end joints,
/// finds the two local rotation corrections that put the end joint on a target. ozz's
/// <c>IKTwoBoneJob</c>.
/// </summary>
/// <remarks>
/// Nothing is written to the skeleton: the job returns corrections to be applied to the start and
/// mid joints' local rotations, after which the chain must be re-run through
/// <see cref="LocalToModel"/>. Allocation-free.
/// </remarks>
public struct IKTwoBoneJob()
{
    /// <summary>Target position for the end joint, in model space.</summary>
    public Vector3 Target = Vector3.Zero;

    /// <summary>The axis the mid joint bends around, in mid-joint local space. Must be unit length; its direction picks which way an elbow or knee folds.</summary>
    public Vector3 MidAxis = Vector3.UnitZ;

    /// <summary>Model-space direction the chain's plane is rotated toward — the "elbow points here" hint.</summary>
    public Vector3 PoleVector = Vector3.UnitY;

    /// <summary>Rotation of the chain plane about the start-to-target axis, radians, applied on top of the pole vector.</summary>
    public float TwistAngle = 0f;

    /// <summary>Fraction of the chain's reach past which the target is eased rather than snapped to, 0..1. 1 lets the chain extend fully straight; lower values keep a bend and avoid the pop at full extension.</summary>
    public float Soften = 1f;

    /// <summary>How much of the correction to apply, 0..1. At 0 both corrections are identity.</summary>
    public float Weight = 1f;

    /// <summary>Model-space matrix of the chain's first joint (the shoulder or hip).</summary>
    public Matrix4x4 StartJoint = Matrix4x4.Identity;

    /// <summary>Model-space matrix of the middle joint (the elbow or knee).</summary>
    public Matrix4x4 MidJoint = Matrix4x4.Identity;

    /// <summary>Model-space matrix of the end joint (the wrist or ankle).</summary>
    public Matrix4x4 EndJoint = Matrix4x4.Identity;

    /// <summary>Whether these arguments would run: <see cref="MidAxis"/> must be normalized.</summary>
    public readonly bool Validate() => MathF.Abs(MidAxis.LengthSquared() - 1f) < 2e-3f;

    /// <summary>Solves the chain.</summary>
    /// <param name="startCorrection">Rotation to apply to the start joint's local rotation.</param>
    /// <param name="midCorrection">Rotation to apply to the mid joint's local rotation.</param>
    /// <param name="reached">Whether the target is inside the chain's (softened) reach and the correction was applied in full.</param>
    /// <returns>False when <see cref="Validate"/> would fail; both corrections are identity then.</returns>
    public readonly bool Run(out Quaternion startCorrection, out Quaternion midCorrection, out bool reached)
    {
        startCorrection = midCorrection = Quaternion.Identity;
        reached = false;
        if (!Validate()) return false;
        if (Weight <= 0f) return true;

        var setup = new Setup(in this);
        var localReached = SoftenTarget(in setup, out var startTargetSs, out var startTargetSsLengthSquared);
        reached = localReached && Weight >= 1f;

        var midRotationMs = ComputeMidJoint(in setup, startTargetSsLengthSquared);
        var startRotationSs = ComputeStartJoint(in setup, midRotationMs, startTargetSs, startTargetSsLengthSquared);

        startCorrection = SoaMath.WeightTowardIdentity(startRotationSs, Weight);
        midCorrection = SoaMath.WeightTowardIdentity(midRotationMs, Weight);
        return true;
    }

    /// <summary>The bone vectors and squared lengths, in start-joint space (<c>ss</c>) and mid-joint space (<c>ms</c>), that every stage below shares.</summary>
    private readonly struct Setup
    {
        public readonly Matrix4x4 InverseStartJoint;
        public readonly Vector3 StartMidMs;
        public readonly Vector3 MidEndMs;
        public readonly Vector3 StartMidSs;
        public readonly float StartMidSsLengthSquared;
        public readonly float MidEndSsLengthSquared;
        public readonly float StartEndSsLengthSquared;
        public readonly Matrix4x4 MidJoint;
        public readonly Vector3 MidAxis;
        public readonly Vector3 PoleVector;
        public readonly Vector3 Target;
        public readonly float Soften;
        public readonly float TwistAngle;

        public Setup(in IKTwoBoneJob job)
        {
            InverseStartJoint = SoaMath.InvertOrZero(job.StartJoint);
            var inverseMidJoint = SoaMath.InvertOrZero(job.MidJoint);

            var startMs = Vector3.Transform(job.StartJoint.Translation, inverseMidJoint);
            var endMs = Vector3.Transform(job.EndJoint.Translation, inverseMidJoint);
            var midSs = Vector3.Transform(job.MidJoint.Translation, InverseStartJoint);
            var endSs = Vector3.Transform(job.EndJoint.Translation, InverseStartJoint);

            // Everything is expressed relative to the start joint, so its own position is 0.
            StartMidMs = -startMs;
            MidEndMs = endMs;
            StartMidSs = midSs;
            StartMidSsLengthSquared = midSs.LengthSquared();
            MidEndSsLengthSquared = (endSs - midSs).LengthSquared();
            StartEndSsLengthSquared = endSs.LengthSquared();

            MidJoint = job.MidJoint;
            MidAxis = job.MidAxis;
            PoleVector = job.PoleVector;
            Target = job.Target;
            Soften = job.Soften;
            TwistAngle = job.TwistAngle;
        }
    }

    /// <summary>
    /// Pulls the target in when it sits further than <see cref="Soften"/> of the chain's reach,
    /// easing it along <c>1 − 3⁴/(α+3)⁴</c> — a curve whose derivative is 1 at the limit, so the
    /// chain approaches full extension smoothly instead of snapping straight.
    /// </summary>
    /// <returns>Whether the (unsoftened) target lies within reach.</returns>
    private static bool SoftenTarget(in Setup setup, out Vector3 startTargetSs, out float startTargetSsLengthSquared)
    {
        var startTargetOriginalSs = Vector3.Transform(setup.Target, setup.InverseStartJoint);
        var startTargetOriginalSsLengthSquared = startTargetOriginalSs.LengthSquared();

        var startMidSsLength = MathF.Sqrt(setup.StartMidSsLengthSquared);
        var midEndSsLength = MathF.Sqrt(setup.MidEndSsLengthSquared);
        var startTargetOriginalSsLength = MathF.Sqrt(startTargetOriginalSsLengthSquared);
        var boneLengthDifference = MathF.Abs(startMidSsLength - midEndSsLength);
        var bonesChainLength = startMidSsLength + midEndSsLength;
        var da = bonesChainLength * Math.Clamp(setup.Soften, 0f, 1f);
        var ds = bonesChainLength - da;

        if (startTargetOriginalSsLength > da && startTargetOriginalSsLength > 0f && ds > 0f)
        {
            var alpha = (startTargetOriginalSsLength - da) / ds;
            var op = alpha + 3f;
            var op2 = op * op;
            var op4 = op2 * op2;
            var ratio = 81f / op4;
            var startTargetSsLength = da + ds - ds * ratio;
            startTargetSsLengthSquared = startTargetSsLength * startTargetSsLength;
            startTargetSs = startTargetOriginalSs * (startTargetSsLength / startTargetOriginalSsLength);
        }
        else
        {
            startTargetSs = startTargetOriginalSs;
            startTargetSsLengthSquared = startTargetOriginalSsLengthSquared;
        }

        // Reachable when the target is no further than the softened chain length and no closer
        // than the difference of the two bone lengths.
        return startTargetOriginalSsLength <= da && startTargetOriginalSsLength > boneLengthDifference;
    }

    /// <summary>The mid joint's angle by the law of cosines, minus the angle it already has; the difference becomes a rotation about <see cref="MidAxis"/>.</summary>
    private static Quaternion ComputeMidJoint(in Setup setup, float startTargetSsLengthSquared)
    {
        var sumLengthSquared = setup.StartMidSsLengthSquared + setup.MidEndSsLengthSquared;
        var halfReciprocalLength = 0.5f / MathF.Sqrt(setup.StartMidSsLengthSquared * setup.MidEndSsLengthSquared);

        // Clamped because a target beyond the triangle's reach drives the cosine out of range.
        var correctedCos = Math.Clamp((sumLengthSquared - startTargetSsLengthSquared) * halfReciprocalLength, -1f, 1f);
        var initialCos = Math.Clamp((sumLengthSquared - setup.StartEndSsLengthSquared) * halfReciprocalLength, -1f, 1f);

        var correctedAngle = MathF.Acos(correctedCos);

        // The existing angle is signed: negative when the chain is already bent against MidAxis.
        var bentSideReference = Vector3.Cross(setup.StartMidMs, setup.MidAxis);
        var initialAngle = MathF.Acos(initialCos);
        if (Vector3.Dot(bentSideReference, setup.MidEndMs) < 0f) initialAngle = -initialAngle;

        return Quaternion.CreateFromAxisAngle(setup.MidAxis, correctedAngle - initialAngle);
    }

    /// <summary>Swings the (already bent) chain onto the target, then rolls its plane onto the pole vector and applies the twist.</summary>
    private static Quaternion ComputeStartJoint(in Setup setup, Quaternion midRotationMs, Vector3 startTargetSs, float startTargetSsLengthSquared)
    {
        var poleSs = Vector3.TransformNormal(setup.PoleVector, setup.InverseStartJoint);

        var midEndSsFinal = Vector3.TransformNormal(
            Vector3.TransformNormal(Vector3.Transform(setup.MidEndMs, midRotationMs), setup.MidJoint),
            setup.InverseStartJoint);
        var startEndSsFinal = setup.StartMidSs + midEndSsFinal;

        var endToTargetRotationSs = SoaMath.QuaternionFromVectors(startEndSsFinal, startTargetSs);
        if (startTargetSsLengthSquared <= 0f) return endToTargetRotationSs;

        // The chain's plane normal is the mid axis (same triangle); rolling it onto the plane
        // through the pole vector is what decides where the elbow points.
        var referencePlaneNormalSs = Vector3.Cross(startTargetSs, poleSs);
        var midAxisSs = Vector3.TransformNormal(Vector3.TransformNormal(setup.MidAxis, setup.MidJoint), setup.InverseStartJoint);
        var jointPlaneNormalSs = Vector3.Transform(midAxisSs, endToTargetRotationSs);

        var referenceLengthSquared = referencePlaneNormalSs.LengthSquared();
        var jointLengthSquared = jointPlaneNormalSs.LengthSquared();
        if (referenceLengthSquared <= 0f || jointLengthSquared <= 0f) return endToTargetRotationSs;

        var rotatePlaneCos = Math.Clamp(
            Vector3.Dot(referencePlaneNormalSs / MathF.Sqrt(referenceLengthSquared), jointPlaneNormalSs / MathF.Sqrt(jointLengthSquared)),
            -1f, 1f);

        var rotatePlaneAxisSs = startTargetSs / MathF.Sqrt(startTargetSsLengthSquared);
        var flipped = Vector3.Dot(jointPlaneNormalSs, poleSs) < 0f ? -rotatePlaneAxisSs : rotatePlaneAxisSs;
        var rotatePlaneSs = SoaMath.QuaternionFromAxisCosAngle(flipped, rotatePlaneCos);

        if (setup.TwistAngle == 0f) return rotatePlaneSs * endToTargetRotationSs;

        var twist = Quaternion.CreateFromAxisAngle(rotatePlaneAxisSs, setup.TwistAngle);
        return twist * rotatePlaneSs * endToTargetRotationSs;
    }
}

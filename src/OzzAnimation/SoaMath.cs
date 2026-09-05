using System.Numerics;
using System.Runtime.Intrinsics;

namespace OzzAnimation;

/// <summary>
/// The pieces of ozz's <c>simd_math</c> and <c>soa_math</c> the jobs need: lane-wise quaternion
/// algebra over <see cref="SoaQuaternion"/>, and the two quaternion constructions
/// (<c>FromVectors</c>, <c>FromAxisCosAngle</c>) the IK jobs are written in terms of.
/// </summary>
/// <remarks>
/// Where ozz uses estimated reciprocal square roots (<c>NormalizeEst</c>, <c>RSqrtEstNR</c>) this
/// uses exact square roots and division — the same choice the sampler makes, so results differ
/// from native ozz around the seventh decimal and never structurally.
/// </remarks>
internal static class SoaMath
{
    public static readonly Vector128<float> SignMask = Vector128.Create(unchecked((int)0x80000000)).AsSingle();

    /// <summary>ozz's <c>Sign</c>: the sign bit of each lane, as a mask to <c>Xor</c> with.</summary>
    public static Vector128<float> Sign(Vector128<float> value) => value & SignMask;

    public static Vector128<float> Max0(Vector128<float> value) => Vector128.Max(value, Vector128<float>.Zero);

    public static Vector128<float> Dot(in SoaQuaternion a, in SoaQuaternion b) =>
        a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;

    public static SoaQuaternion Normalize(in SoaQuaternion q)
    {
        var inverseLength = Vector128<float>.One / Vector128.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        return new SoaQuaternion { X = q.X * inverseLength, Y = q.Y * inverseLength, Z = q.Z * inverseLength, W = q.W * inverseLength };
    }

    public static SoaQuaternion Conjugate(in SoaQuaternion q) =>
        new() { X = q.X ^ SignMask, Y = q.Y ^ SignMask, Z = q.Z ^ SignMask, W = q.W };

    /// <summary>Hamilton product per lane, the same operand order as ozz's <c>SimdQuaternion operator*</c>.</summary>
    public static SoaQuaternion Multiply(in SoaQuaternion a, in SoaQuaternion b) =>
        new()
        {
            X = a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            Y = a.W * b.Y + a.Y * b.W + a.Z * b.X - a.X * b.Z,
            Z = a.W * b.Z + a.Z * b.W + a.X * b.Y - a.Y * b.X,
            W = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
        };

    /// <summary>
    /// ozz's <c>SimdQuaternion::FromVectors</c>: the rotation taking <paramref name="from"/> onto
    /// <paramref name="to"/> about their common perpendicular. Neither needs to be normalized, and
    /// either may be zero. Exactly opposed vectors rotate 180° about an arbitrary orthogonal axis.
    /// </summary>
    public static Quaternion QuaternionFromVectors(Vector3 from, Vector3 to)
    {
        var normFromNormTo = MathF.Sqrt(from.LengthSquared() * to.LengthSquared());
        if (normFromNormTo < 1e-6f) return Quaternion.Identity;

        var realPart = normFromNormTo + Vector3.Dot(from, to);
        Quaternion quaternion;
        if (realPart < 1e-6f * normFromNormTo)
        {
            quaternion = MathF.Abs(from.X) > MathF.Abs(from.Z)
                ? new Quaternion(-from.Y, from.X, 0f, 0f)
                : new Quaternion(0f, -from.Z, from.Y, 0f);
        }
        else
        {
            var axis = Vector3.Cross(from, to);
            quaternion = new Quaternion(axis, realPart);
        }

        return Quaternion.Normalize(quaternion);
    }

    /// <summary>ozz's <c>SimdQuaternion::FromAxisCosAngle</c>: the rotation about a unit <paramref name="axis"/> by the angle whose cosine is <paramref name="cos"/>, built from half-angle identities so no arc-cosine is taken.</summary>
    public static Quaternion QuaternionFromAxisCosAngle(Vector3 axis, float cos)
    {
        var halfCosSquared = (1f + cos) * 0.5f;
        var halfSin = MathF.Sqrt(1f - halfCosSquared);
        return new Quaternion(axis * halfSin, MathF.Sqrt(halfCosSquared));
    }

    /// <summary>ozz inverts with a routine that yields an all-zero matrix for a singular input, so a degenerate joint produces a zero vector and then an identity correction rather than a NaN. <see cref="Matrix4x4.Invert"/> leaves its output undefined instead, so it is zeroed here.</summary>
    public static Matrix4x4 InvertOrZero(in Matrix4x4 matrix) =>
        Matrix4x4.Invert(matrix, out var inverse) ? inverse : default;

    /// <summary>Normalized lerp toward identity, ozz's weighting of an IK correction; the input is first flipped onto the positive-w hemisphere so the lerp takes the short arc.</summary>
    public static Quaternion WeightTowardIdentity(Quaternion rotation, float weight)
    {
        var fixedUp = rotation.W < 0f ? -rotation : rotation;
        if (weight >= 1f) return fixedUp;

        var w = MathF.Max(0f, weight);
        var lerp = new Quaternion(fixedUp.X * w, fixedUp.Y * w, fixedUp.Z * w, (fixedUp.W - 1f) * w + 1f);
        return Quaternion.Normalize(lerp);
    }
}

using System.Numerics;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

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

    /// <summary>
    /// A 4×4 transpose of four rows into four columns — ozz's <c>Transpose4x4</c>, and the whole of
    /// what turns structure-of-arrays lanes back into per-joint values and vice versa. Two
    /// interleaves and four half-vector joins; no value ever passes through a scalar register.
    /// </summary>
    public static void Transpose(
        Vector128<float> row0, Vector128<float> row1, Vector128<float> row2, Vector128<float> row3,
        out Vector128<float> column0, out Vector128<float> column1, out Vector128<float> column2, out Vector128<float> column3)
    {
        var low01 = InterleaveLower(row0, row1);     // x0 x1 y0 y1
        var low23 = InterleaveLower(row2, row3);     // x2 x3 y2 y3
        var high01 = InterleaveUpper(row0, row1);    // z0 z1 w0 w1
        var high23 = InterleaveUpper(row2, row3);    // z2 z3 w2 w3
        column0 = Vector128.Create(low01.GetLower(), low23.GetLower());
        column1 = Vector128.Create(low01.GetUpper(), low23.GetUpper());
        column2 = Vector128.Create(high01.GetLower(), high23.GetLower());
        column3 = Vector128.Create(high01.GetUpper(), high23.GetUpper());
    }

    // There is no cross-platform Vector128 interleave, so the one instruction that needs naming is
    // named per architecture; the JIT folds these checks away. The last branch is reached only on a
    // target with neither instruction set, where the surrounding code is scalar anyway.
    private static Vector128<float> InterleaveLower(Vector128<float> a, Vector128<float> b) =>
        Sse.IsSupported ? Sse.UnpackLow(a, b)
        : AdvSimd.Arm64.IsSupported ? AdvSimd.Arm64.ZipLow(a, b)
        : Vector128.Create(a[0], b[0], a[1], b[1]);

    private static Vector128<float> InterleaveUpper(Vector128<float> a, Vector128<float> b) =>
        Sse.IsSupported ? Sse.UnpackHigh(a, b)
        : AdvSimd.Arm64.IsSupported ? AdvSimd.Arm64.ZipHigh(a, b)
        : Vector128.Create(a[2], b[2], a[3], b[3]);
}

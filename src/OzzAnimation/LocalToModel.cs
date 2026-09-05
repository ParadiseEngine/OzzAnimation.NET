using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace OzzAnimation;

/// <summary>The blob runtime's vectorized <c>LocalToModelJob</c> on managed arrays: the four local matrices of a group are built in lanes, then each multiplies its parent.</summary>
public static class LocalToModel
{
    /// <summary>Row-vector convention: <c>model[i] = local[i] × model[parent]</c>, roots multiplied by <paramref name="root"/> (identity when null).</summary>
    /// <exception cref="ArgumentException">Fewer poses or outputs than joints.</exception>
    public static void Compute(Skeleton skeleton, SoaTransforms locals, Span<Matrix4x4> models, in Matrix4x4? root = null) =>
        Compute(skeleton, locals, models, Skeleton.NoParent, Skeleton.MaxJoints, false, root);

    /// <summary>
    /// The same walk restricted to part of the hierarchy — ozz's <c>from</c>, <c>to</c> and
    /// <c>from_excluded</c>. Updating only the arm below a shoulder an IK job just moved costs a
    /// few joints instead of the whole skeleton.
    /// </summary>
    /// <param name="from">Joint the walk starts at; <see cref="Skeleton.NoParent"/> for the whole hierarchy. Its parent's model matrix must already be valid.</param>
    /// <param name="to">Last joint updated, inclusive. The walk stops earlier if it leaves <paramref name="from"/>'s subtree.</param>
    /// <param name="fromExcluded">Skip <paramref name="from"/> itself and update only its descendants — for propagating a model-space matrix that was set directly rather than derived from a local pose.</param>
    /// <exception cref="ArgumentException">Fewer poses or outputs than joints.</exception>
    public static void Compute(Skeleton skeleton, SoaTransforms locals, Span<Matrix4x4> models, int from, int to, bool fromExcluded, in Matrix4x4? root = null)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        ArgumentNullException.ThrowIfNull(locals);
        var count = skeleton.JointCount;
        if (locals.JointCount < count) throw new ArgumentException($"{locals.JointCount} poses for {count} joints.", nameof(locals));
        if (models.Length < count) throw new ArgumentException($"{models.Length} outputs for {count} joints.", nameof(models));

        var rootMatrix = root ?? Matrix4x4.Identity;
        var parents = skeleton.Parents;
        var translations = locals.Translations.AsSpan();
        var rotations = locals.Rotations.AsSpan();
        var scales = locals.Scales.AsSpan();
        Span<Matrix4x4> group = stackalloc Matrix4x4[4];

        var end = Math.Min(to + 1, count);
        var joint = Math.Max(from + (fromExcluded ? 1 : 0), 0);
        // parents[joint] >= from holds exactly as long as joint is still inside from's subtree.
        var process = joint < end && (!fromExcluded || parents[joint] >= from);
        while (process)
        {
            // A whole group of four is built at once even when only some of its lanes are wanted;
            // the lane loop below is what decides which of them are written.
            var g = joint / 4;
            AffineMatrices(in translations[g], in rotations[g], in scales[g], group);
            for (var groupEnd = (joint + 4) & ~3; joint < groupEnd && process; joint++, process = joint < end && parents[joint] >= from)
            {
                var parent = parents[joint];
                models[joint] = group[joint & 3] * (parent == Skeleton.NoParent ? rootMatrix : models[parent]);
            }
        }
    }

    /// <summary>
    /// The affine matrices of four joints at once — the rotation's rows scaled per axis, then the
    /// translation — built in lanes and transposed straight into four <see cref="Matrix4x4"/>.
    /// ozz's <c>SoaFloat4x4::FromAffine</c> followed by its <c>Transpose16x16</c>.
    /// </summary>
    private static void AffineMatrices(in SoaVector3 t, in SoaQuaternion q, in SoaVector3 s, Span<Matrix4x4> group)
    {
        var two = Vector128.Create(2f);
        var one = Vector128<float>.One;
        var zero = Vector128<float>.Zero;
        var xx = q.X * q.X; var yy = q.Y * q.Y; var zz = q.Z * q.Z;
        var xy = q.X * q.Y; var wz = q.Z * q.W; var xz = q.Z * q.X; var wy = q.Y * q.W; var yz = q.Y * q.Z; var wx = q.X * q.W;

        // Each transpose turns one row of the four joints' matrices into that row for each joint,
        // so the four results land as whole rows and nothing round-trips through the stack.
        SoaMath.Transpose((one - two * (yy + zz)) * s.X, two * (xy + wz) * s.X, two * (xz - wy) * s.X, zero, out var a0, out var b0, out var c0, out var d0);
        SoaMath.Transpose(two * (xy - wz) * s.Y, (one - two * (zz + xx)) * s.Y, two * (yz + wx) * s.Y, zero, out var a1, out var b1, out var c1, out var d1);
        SoaMath.Transpose(two * (xz + wy) * s.Z, two * (yz - wx) * s.Z, (one - two * (yy + xx)) * s.Z, zero, out var a2, out var b2, out var c2, out var d2);
        SoaMath.Transpose(t.X, t.Y, t.Z, one, out var a3, out var b3, out var c3, out var d3);

        var rows = MemoryMarshal.Cast<Matrix4x4, Vector128<float>>(group);
        rows[0] = a0; rows[1] = a1; rows[2] = a2; rows[3] = a3;
        rows[4] = b0; rows[5] = b1; rows[6] = b2; rows[7] = b3;
        rows[8] = c0; rows[9] = c1; rows[10] = c2; rows[11] = c3;
        rows[12] = d0; rows[13] = d1; rows[14] = d2; rows[15] = d3;
    }
}

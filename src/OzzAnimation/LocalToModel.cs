using System.Numerics;
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
        Span<float> rows = stackalloc float[48];

        var end = Math.Min(to + 1, count);
        var joint = Math.Max(from + (fromExcluded ? 1 : 0), 0);
        // parents[joint] >= from holds exactly as long as joint is still inside from's subtree.
        var process = joint < end && (!fromExcluded || parents[joint] >= from);
        while (process)
        {
            // A whole group of four is built at once even when only some of its lanes are wanted;
            // the lane loop below is what decides which of them are written.
            var g = joint / 4;
            AffineRows(in translations[g], in rotations[g], in scales[g], rows);
            for (var groupEnd = (joint + 4) & ~3; joint < groupEnd && process; joint++, process = joint < end && parents[joint] >= from)
            {
                var lane = joint & 3;
                var local = new Matrix4x4(
                    rows[lane], rows[4 + lane], rows[8 + lane], 0f,
                    rows[12 + lane], rows[16 + lane], rows[20 + lane], 0f,
                    rows[24 + lane], rows[28 + lane], rows[32 + lane], 0f,
                    rows[36 + lane], rows[40 + lane], rows[44 + lane], 1f);
                var parent = parents[joint];
                models[joint] = local * (parent == Skeleton.NoParent ? rootMatrix : models[parent]);
            }
        }
    }

    /// <summary>The twelve affine entries of four joints at once — rotation rows scaled per axis, then the translation — stored lane-major so a joint's matrix is one column of the buffer.</summary>
    private static void AffineRows(in SoaVector3 t, in SoaQuaternion q, in SoaVector3 s, Span<float> rows)
    {
        var two = Vector128.Create(2f);
        var one = Vector128<float>.One;
        var xx = q.X * q.X; var yy = q.Y * q.Y; var zz = q.Z * q.Z;
        var xy = q.X * q.Y; var wz = q.Z * q.W; var xz = q.Z * q.X; var wy = q.Y * q.W; var yz = q.Y * q.Z; var wx = q.X * q.W;
        ((one - two * (yy + zz)) * s.X).CopyTo(rows[..4]);
        (two * (xy + wz) * s.X).CopyTo(rows[4..8]);
        (two * (xz - wy) * s.X).CopyTo(rows[8..12]);
        (two * (xy - wz) * s.Y).CopyTo(rows[12..16]);
        ((one - two * (zz + xx)) * s.Y).CopyTo(rows[16..20]);
        (two * (yz + wx) * s.Y).CopyTo(rows[20..24]);
        (two * (xz + wy) * s.Z).CopyTo(rows[24..28]);
        (two * (yz - wx) * s.Z).CopyTo(rows[28..32]);
        ((one - two * (yy + xx)) * s.Z).CopyTo(rows[32..36]);
        t.X.CopyTo(rows[36..40]);
        t.Y.CopyTo(rows[40..44]);
        t.Z.CopyTo(rows[44..48]);
    }
}

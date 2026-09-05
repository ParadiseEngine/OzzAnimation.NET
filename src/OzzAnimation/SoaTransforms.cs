using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace OzzAnimation;

/// <summary>Three components of four joints, one joint per lane.</summary>
public struct SoaVector3
{
    public Vector128<float> X, Y, Z;
}

/// <summary>Four components of four joints' rotations, one joint per lane.</summary>
public struct SoaQuaternion
{
    public Vector128<float> X, Y, Z, W;
}

/// <summary>
/// A skeleton's worth of local poses in ozz's structure-of-arrays layout: joints in groups of
/// four, each component of a group one <see cref="Vector128{T}"/> with a joint per lane. The
/// sampler writes it without a transpose, and a blend or a hierarchy walk handles four joints per
/// instruction. The last group's spare lanes hold identity.
/// </summary>
/// <remarks>
/// The indexer gathers or scatters one joint as a <see cref="JointPose"/> for code that thinks per
/// joint — an attachment, a test; it is not the hot path.
/// </remarks>
public sealed class SoaTransforms
{
    /// <summary>Room for <paramref name="jointCount"/> joints, every lane at identity.</summary>
    public SoaTransforms(int jointCount)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(jointCount);
        var groups = AnimationClip.PaddedTrackCount(jointCount) / 4;
        JointCount = jointCount;
        Translations = new SoaVector3[groups];
        Rotations = new SoaQuaternion[groups];
        Scales = new SoaVector3[groups];
        for (var g = 0; g < groups; g++)
        {
            Rotations[g].W = Vector128<float>.One;
            Scales[g].X = Scales[g].Y = Scales[g].Z = Vector128<float>.One;
        }
    }

    public int JointCount { get; }

    public SoaVector3[] Translations { get; }

    public SoaQuaternion[] Rotations { get; }

    public SoaVector3[] Scales { get; }

    public int GroupCount => Translations.Length;

    public JointPose this[int joint]
    {
        get
        {
            Check(joint);
            var g = joint >> 2;
            var lane = joint & 3;
            ref var t = ref Translations[g];
            ref var r = ref Rotations[g];
            ref var s = ref Scales[g];
            return new JointPose(
                new Vector3(Lane(ref t.X, lane), Lane(ref t.Y, lane), Lane(ref t.Z, lane)),
                new Quaternion(Lane(ref r.X, lane), Lane(ref r.Y, lane), Lane(ref r.Z, lane), Lane(ref r.W, lane)),
                new Vector3(Lane(ref s.X, lane), Lane(ref s.Y, lane), Lane(ref s.Z, lane)));
        }
        set
        {
            Check(joint);
            var g = joint >> 2;
            var lane = joint & 3;
            ref var t = ref Translations[g];
            ref var r = ref Rotations[g];
            ref var s = ref Scales[g];
            Lane(ref t.X, lane) = value.Translation.X; Lane(ref t.Y, lane) = value.Translation.Y; Lane(ref t.Z, lane) = value.Translation.Z;
            Lane(ref r.X, lane) = value.Rotation.X; Lane(ref r.Y, lane) = value.Rotation.Y; Lane(ref r.Z, lane) = value.Rotation.Z; Lane(ref r.W, lane) = value.Rotation.W;
            Lane(ref s.X, lane) = value.Scale.X; Lane(ref s.Y, lane) = value.Scale.Y; Lane(ref s.Z, lane) = value.Scale.Z;
        }
    }

    /// <summary>
    /// Fills this pose set from one transform per joint, four joints at a time. A
    /// <see cref="JointPose"/> is three 16-byte lanes — translation, rotation, scale — so a group
    /// is three 4×4 transposes with no scalar shuffling. The spare lanes of the last group are left
    /// holding identity.
    /// </summary>
    /// <exception cref="ArgumentException">Fewer poses than joints.</exception>
    public void CopyFrom(ReadOnlySpan<JointPose> poses)
    {
        if (poses.Length < JointCount) throw new ArgumentException($"{poses.Length} poses for {JointCount} joints.", nameof(poses));

        var lanes = MemoryMarshal.Cast<JointPose, Vector128<float>>(poses);
        var identityRotation = Vector128.Create(0f, 0f, 0f, 1f);
        for (var g = 0; g < GroupCount; g++)
        {
            Vector128<float> t0 = default, t1 = default, t2 = default, t3 = default;
            var r0 = identityRotation; var r1 = identityRotation; var r2 = identityRotation; var r3 = identityRotation;
            var s0 = Vector128<float>.One; var s1 = Vector128<float>.One; var s2 = Vector128<float>.One; var s3 = Vector128<float>.One;
            var present = Math.Min(4, JointCount - g * 4);
            if (present > 0) Read(lanes, g * 4, out t0, out r0, out s0);
            if (present > 1) Read(lanes, g * 4 + 1, out t1, out r1, out s1);
            if (present > 2) Read(lanes, g * 4 + 2, out t2, out r2, out s2);
            if (present > 3) Read(lanes, g * 4 + 3, out t3, out r3, out s3);

            ref var translation = ref Translations[g];
            SoaMath.Transpose(t0, t1, t2, t3, out translation.X, out translation.Y, out translation.Z, out _);
            ref var rotation = ref Rotations[g];
            SoaMath.Transpose(r0, r1, r2, r3, out rotation.X, out rotation.Y, out rotation.Z, out rotation.W);
            ref var scale = ref Scales[g];
            SoaMath.Transpose(s0, s1, s2, s3, out scale.X, out scale.Y, out scale.Z, out _);
        }

        static void Read(ReadOnlySpan<Vector128<float>> lanes, int joint, out Vector128<float> t, out Vector128<float> r, out Vector128<float> s)
        {
            t = lanes[joint * 3];
            r = lanes[joint * 3 + 1];
            s = lanes[joint * 3 + 2];
        }
    }

    /// <summary>The inverse of <see cref="CopyFrom(ReadOnlySpan{JointPose})"/> — the same three transposes per group, the other way round.</summary>
    /// <exception cref="ArgumentException">Fewer slots than joints.</exception>
    public void CopyTo(Span<JointPose> poses)
    {
        if (poses.Length < JointCount) throw new ArgumentException($"{poses.Length} slots for {JointCount} joints.", nameof(poses));

        var lanes = MemoryMarshal.Cast<JointPose, Vector128<float>>(poses);
        for (var g = 0; g < GroupCount; g++)
        {
            ref readonly var translation = ref Translations[g];
            ref readonly var rotation = ref Rotations[g];
            ref readonly var scale = ref Scales[g];
            // A JointPose's fourth lane is padding; feeding the two vector transposes a zero row is
            // what puts a zero there rather than whatever the lane last held.
            SoaMath.Transpose(translation.X, translation.Y, translation.Z, Vector128<float>.Zero, out var t0, out var t1, out var t2, out var t3);
            SoaMath.Transpose(rotation.X, rotation.Y, rotation.Z, rotation.W, out var r0, out var r1, out var r2, out var r3);
            SoaMath.Transpose(scale.X, scale.Y, scale.Z, Vector128<float>.Zero, out var s0, out var s1, out var s2, out var s3);

            var present = Math.Min(4, JointCount - g * 4);
            if (present > 0) Write(lanes, g * 4, t0, r0, s0);
            if (present > 1) Write(lanes, g * 4 + 1, t1, r1, s1);
            if (present > 2) Write(lanes, g * 4 + 2, t2, r2, s2);
            if (present > 3) Write(lanes, g * 4 + 3, t3, r3, s3);
        }

        static void Write(Span<Vector128<float>> lanes, int joint, Vector128<float> t, Vector128<float> r, Vector128<float> s)
        {
            lanes[joint * 3] = t;
            lanes[joint * 3 + 1] = r;
            lanes[joint * 3 + 2] = s;
        }
    }

    /// <summary>For tests and tools, not the hot path.</summary>
    public JointPose[] ToArray()
    {
        var poses = new JointPose[JointCount];
        CopyTo(poses);
        return poses;
    }

    /// <summary>Copies every lane of <paramref name="source"/>; both must be sized for the same joint count.</summary>
    public void CopyFrom(SoaTransforms source)
    {
        ArgumentNullException.ThrowIfNull(source);
        if (source.GroupCount != GroupCount) throw new ArgumentException($"{source.JointCount} joints into {JointCount}.", nameof(source));
        source.Translations.CopyTo(Translations, 0);
        source.Rotations.CopyTo(Rotations, 0);
        source.Scales.CopyTo(Scales, 0);
    }

    private void Check(int joint)
    {
        if (joint < 0 || joint >= JointCount) throw new ArgumentOutOfRangeException(nameof(joint), $"Joint {joint} of {JointCount}.");
    }

    private static ref float Lane(ref Vector128<float> vector, int lane) => ref Unsafe.Add(ref Unsafe.As<Vector128<float>, float>(ref vector), lane);
}

using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>
/// The data the tests share. This is a runtime-only library — ozz keeps its builders in a separate
/// offline module and so does this repository — so skeletons are built through the runtime
/// constructor and clips come from archives committed under <c>Fixtures/</c>.
/// </summary>
internal static class TestRigs
{
    public static readonly Quaternion QuarterTurnZ = new(0f, 0f, 0.7071068f, 0.7071068f);

    /// <summary>hip at (0,1,0) with child knee turned a quarter turn about Z, plus an unparented "prop" node.</summary>
    public static Skeleton Chain() => new(
        ["hip", "knee", "prop"],
        [Skeleton.NoParent, 0, Skeleton.NoParent],
        Pose(
            new JointPose(new Vector3(0, 1, 0), Quaternion.Identity, Vector3.One),
            new JointPose(Vector3.Zero, QuarterTurnZ, Vector3.One),
            JointPose.Identity));

    /// <summary>A three-joint chain lying along +X, each bone one unit long — the shape the two-bone IK tests solve.</summary>
    public static Skeleton Arm() => new(
        ["shoulder", "elbow", "wrist"],
        [Skeleton.NoParent, 0, 1],
        Pose(
            JointPose.Identity,
            new JointPose(new Vector3(1, 0, 0), Quaternion.Identity, Vector3.One),
            new JointPose(new Vector3(1, 0, 0), Quaternion.Identity, Vector3.One)));

    /// <summary>The pose set holding these joints, in order.</summary>
    public static SoaTransforms Pose(params JointPose[] poses)
    {
        var transforms = new SoaTransforms(poses.Length);
        transforms.CopyFrom(poses);
        return transforms;
    }

    public static Skeleton BenchmarkSkeleton() => Skeleton.Load(Fixture("bench-skeleton.ozz"));

    public static AnimationClip BenchmarkClip() => AnimationClip.Load(Fixture("bench-animation.ozz"));

    /// <summary>
    /// The closed form <c>bench-animation.ozz</c> was baked from, so a sampled pose can be checked
    /// against ground truth rather than against another run of the same code. Keys were laid at
    /// 30 Hz over the clip's 1.333 s; sampling exactly on one isolates quantization error from
    /// interpolation error.
    /// </summary>
    public static JointPose BenchmarkSource(int track, float seconds)
    {
        var a = track * 0.37f + seconds * 3.1f;
        return new JointPose(
            new Vector3(MathF.Sin(a) * 0.05f, 0.2f + MathF.Cos(a) * 0.02f, MathF.Sin(a * 0.5f) * 0.05f),
            Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1f, 0.2f)), MathF.Sin(a) * 0.6f),
            new Vector3(1f + MathF.Sin(a) * 0.03f));
    }

    public const int BenchmarkKeyCount = 40;

    public static byte[] Fixture(string name)
    {
        using var stream = typeof(TestRigs).Assembly.GetManifestResourceStream($"OzzAnimation.Tests.Fixtures.{name}")
            ?? throw new FileNotFoundException($"Embedded fixture '{name}' is missing.");
        using var memory = new MemoryStream();
        stream.CopyTo(memory);
        return memory.ToArray();
    }

    public static float MaxAbs(Matrix4x4 m)
    {
        var max = 0f;
        foreach (var v in new[] { m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24, m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44 }) max = MathF.Max(max, MathF.Abs(v));
        return max;
    }

    /// <summary>Angle between two rotations, radians — the comparison that ignores the double cover.</summary>
    public static float AngleBetween(Quaternion a, Quaternion b) =>
        2f * MathF.Acos(Math.Clamp(MathF.Abs(Quaternion.Dot(Quaternion.Normalize(a), Quaternion.Normalize(b))), 0f, 1f));

    /// <summary>A pose set whose every joint holds <paramref name="pose"/>.</summary>
    public static SoaTransforms Uniform(int joints, JointPose pose)
    {
        var transforms = new SoaTransforms(joints);
        for (var i = 0; i < joints; i++) transforms[i] = pose;
        return transforms;
    }
}

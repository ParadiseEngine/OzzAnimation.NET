using System.Numerics;
using System.Runtime.InteropServices;

namespace OzzAnimation;

/// <summary>One joint's local transform: what a clip samples to and a rest pose holds.</summary>
/// <remarks>
/// 48 bytes, not the 40 the fields need: translation, rotation and scale each start on a 16-byte
/// boundary so a blend or a matrix build loads each as one <see cref="System.Runtime.Intrinsics.Vector128{T}"/>.
/// The padding lanes are zero and stay zero; nothing reads them.
/// </remarks>
[StructLayout(LayoutKind.Explicit, Size = 48)]
public readonly struct JointPose : IEquatable<JointPose>
{
    [FieldOffset(0)] public readonly Vector3 Translation;
    [FieldOffset(16)] public readonly Quaternion Rotation;
    [FieldOffset(32)] public readonly Vector3 Scale;

    public JointPose(Vector3 translation, Quaternion rotation, Vector3 scale)
    {
        Translation = translation;
        Rotation = rotation;
        Scale = scale;
    }

    public static JointPose Identity { get; } = new(Vector3.Zero, Quaternion.Identity, Vector3.One);

    /// <summary>Row-vector convention: scale, then rotate, then translate.</summary>
    public Matrix4x4 ToMatrix() =>
        Matrix4x4.CreateScale(Scale) * Matrix4x4.CreateFromQuaternion(Rotation) * Matrix4x4.CreateTranslation(Translation);

    /// <summary>The same pose with a unit rotation, a zero one becoming identity — ozz's <c>NormalizeSafe</c> in its operation order, so a built skeleton's bytes still match ozz's. Every authoring path applies it; a non-unit rest rotation would otherwise scale every unanimated joint below it.</summary>
    public JointPose WithNormalizedRotation()
    {
        var q = Rotation;
        var lengthSquared = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        if (lengthSquared == 0f) return new JointPose(Translation, Quaternion.Identity, Scale);
        var inverse = 1f / MathF.Sqrt(lengthSquared);
        return new JointPose(Translation, new Quaternion(q.X * inverse, q.Y * inverse, q.Z * inverse, q.W * inverse), Scale);
    }

    public bool Equals(JointPose other) => Translation == other.Translation && Rotation == other.Rotation && Scale == other.Scale;

    public override bool Equals(object? obj) => obj is JointPose other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(Translation, Rotation, Scale);

    public override string ToString() => $"JointPose {{ Translation = {Translation}, Rotation = {Rotation}, Scale = {Scale} }}";

    public static bool operator ==(JointPose left, JointPose right) => left.Equals(right);

    public static bool operator !=(JointPose left, JointPose right) => !left.Equals(right);
}

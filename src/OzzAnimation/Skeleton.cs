using System.Runtime.InteropServices;
using System.Text;

namespace OzzAnimation;

/// <summary>
/// An ozz-animation runtime skeleton: joints in depth-first order, each with its parent index, its
/// name and its rest-pose local transform. A parent always precedes its children, so a single
/// forward pass computes model-space poses.
/// </summary>
/// <remarks>
/// Reads and writes the <c>ozz-skeleton</c> archive, version 2 (ozz-animation 0.17). The rest pose
/// is held in the structure-of-arrays layout the archive stores and every job consumes, so loading
/// is a bulk copy rather than a transpose, and there is one representation of it rather than two.
/// Reach a single joint through <see cref="SoaTransforms"/>'s indexer or
/// <see cref="SkeletonUtils.JointRestPoseLocalSpace"/>.
/// </remarks>
public sealed class Skeleton
{
    public const string Tag = "ozz-skeleton";

    public const uint Version = 2;

    /// <summary>ozz's limit; a clip's track index and the sampler's cache are 16-bit.</summary>
    public const int MaxJoints = 1024;

    public const short NoParent = -1;

    // The archive stores a group of four joints as translation xyz, rotation xyzw, scale xyz —
    // ten SIMD lanes of four floats, which is exactly how SoaTransforms holds them. A SoaVector3
    // is twelve contiguous floats and a SoaQuaternion sixteen, so loading and saving are bulk
    // copies over the pose set's own arrays rather than a per-lane transpose.
    private const int FloatsPerSoaVector3 = 12;
    private const int FloatsPerSoaQuaternion = 16;

    private readonly string[] _names;
    private readonly short[] _parents;

    /// <exception cref="ArgumentException">Mismatched lengths, too many joints, or a parent that does not precede its child.</exception>
    public Skeleton(string[] names, short[] parents, SoaTransforms restPose)
    {
        ArgumentNullException.ThrowIfNull(names);
        ArgumentNullException.ThrowIfNull(parents);
        ArgumentNullException.ThrowIfNull(restPose);
        if (parents.Length != names.Length || restPose.JointCount != names.Length)
        {
            throw new ArgumentException($"{names.Length} names, {parents.Length} parents and {restPose.JointCount} rest poses do not describe one skeleton.");
        }

        if (names.Length > MaxJoints) throw new ArgumentException($"{names.Length} joints exceed ozz's limit of {MaxJoints}.");
        for (var i = 0; i < parents.Length; i++)
        {
            if (parents[i] != NoParent && (parents[i] < 0 || parents[i] >= i))
            {
                throw new ArgumentException($"Joint {i} has parent {parents[i]}; a parent precedes its child in depth-first order.");
            }
        }

        _names = names;
        _parents = parents;
        RestPose = restPose;
    }

    /// <summary>The rest pose — ozz's <c>joint_rest_poses()</c>. What <see cref="BlendingJob"/> takes as its reference pose and what <see cref="SkeletonUtils.RestPoseModelSpace"/> walks; the spare lanes of the last group hold identity.</summary>
    public SoaTransforms RestPose { get; }

    public int JointCount => _names.Length;

    public int SoaJointCount => (JointCount + 3) / 4;

    public ReadOnlySpan<string> Names => _names;

    public ReadOnlySpan<short> Parents => _parents;

    /// <summary>The skeleton with no joints — ozz's default-constructed <c>Skeleton</c>, valid anywhere one is asked for.</summary>
    public static Skeleton Empty { get; } = new([], [], new SoaTransforms(0));

    /// <summary>The first joint of that exact name, or −1. ozz's <c>FindJoint</c>.</summary>
    public int FindJoint(string name) => Array.IndexOf(_names, name);

    public bool IsLeaf(int joint)
    {
        for (var i = joint + 1; i < _parents.Length; i++)
        {
            if (_parents[i] == joint) return false;
        }

        return true;
    }

    public static bool IsSkeleton(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    /// <exception cref="InvalidDataException">Not a version-2 ozz skeleton archive, or one whose joints do not form a depth-first tree.</exception>
    public static Skeleton Load(ReadOnlySpan<byte> bytes)
    {
        var reader = OzzReader.Open(bytes, Tag, Version);
        var count = reader.ReadInt32();
        if (count == 0)
        {
            reader.ExpectEnd("skeleton");
            return new Skeleton([], [], new SoaTransforms(0));
        }

        if (count < 0 || count > MaxJoints) throw new InvalidDataException($"The ozz skeleton names {count} joints; the limit is {MaxJoints}.");
        var charCount = reader.ReadInt32();
        if (charCount < count) throw new InvalidDataException("The ozz skeleton's name block is shorter than one terminator per joint.");
        var chars = reader.ReadBytes(charCount);
        var names = new string[count];
        var at = 0;
        for (var i = 0; i < count; i++)
        {
            var end = chars[at..].IndexOf((byte)0);
            if (end < 0) throw new InvalidDataException($"The ozz skeleton's name {i} is not terminated.");
            names[i] = Encoding.UTF8.GetString(chars.Slice(at, end));
            at += end + 1;
        }

        var parents = new short[count];
        for (var i = 0; i < count; i++)
        {
            parents[i] = reader.ReadInt16();
            if (parents[i] != NoParent && (parents[i] < 0 || parents[i] >= i))
            {
                throw new InvalidDataException($"The ozz skeleton's joint {i} has parent {parents[i]}, which does not precede it.");
            }
        }

        // The archive's rest-pose block is already grouped the way SoaTransforms holds it, so each
        // group's three runs are read straight into place — the spare lanes of the last group
        // included, which is what lets Save reproduce the file byte for byte.
        var restPose = new SoaTransforms(count);
        var translations = MemoryMarshal.Cast<SoaVector3, float>(restPose.Translations.AsSpan());
        var rotations = MemoryMarshal.Cast<SoaQuaternion, float>(restPose.Rotations.AsSpan());
        var scales = MemoryMarshal.Cast<SoaVector3, float>(restPose.Scales.AsSpan());
        for (var g = 0; g < restPose.GroupCount; g++)
        {
            reader.ReadSingles(translations.Slice(g * FloatsPerSoaVector3, FloatsPerSoaVector3));
            reader.ReadSingles(rotations.Slice(g * FloatsPerSoaQuaternion, FloatsPerSoaQuaternion));
            reader.ReadSingles(scales.Slice(g * FloatsPerSoaVector3, FloatsPerSoaVector3));
        }

        reader.ExpectEnd("skeleton");
        return new Skeleton(names, parents, restPose);
    }

    public byte[] Save()
    {
        var writer = new OzzWriter(Tag, Version);
        writer.Write(JointCount);
        if (JointCount == 0) return writer.ToArray();

        var chars = new MemoryStream();
        foreach (var name in _names)
        {
            chars.Write(Encoding.UTF8.GetBytes(name));
            chars.WriteByte(0);
        }

        writer.Write((int)chars.Length);
        writer.Write(chars.ToArray());
        writer.Write(_parents);
        var translations = MemoryMarshal.Cast<SoaVector3, float>(RestPose.Translations);
        var rotations = MemoryMarshal.Cast<SoaQuaternion, float>(RestPose.Rotations);
        var scales = MemoryMarshal.Cast<SoaVector3, float>(RestPose.Scales);
        for (var g = 0; g < RestPose.GroupCount; g++)
        {
            writer.Write(translations.Slice(g * FloatsPerSoaVector3, FloatsPerSoaVector3));
            writer.Write(rotations.Slice(g * FloatsPerSoaQuaternion, FloatsPerSoaQuaternion));
            writer.Write(scales.Slice(g * FloatsPerSoaVector3, FloatsPerSoaVector3));
        }

        return writer.ToArray();
    }

}

using System.Numerics;

namespace OzzAnimation;

/// <summary>Walks and queries a <see cref="Skeleton"/>. ozz's <c>skeleton_utils.h</c>.</summary>
public static class SkeletonUtils
{
    /// <summary>One joint's rest transform, unpacked from the skeleton's structure-of-arrays storage. ozz's <c>GetJointRestPoseLocalSpace</c>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The joint is outside the skeleton.</exception>
    public static JointPose JointRestPoseLocalSpace(Skeleton skeleton, int joint)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        if (joint < 0 || joint >= skeleton.JointCount) throw new ArgumentOutOfRangeException(nameof(joint), $"Joint {joint} of {skeleton.JointCount}.");
        return skeleton.RestPose[joint];
    }

    /// <summary>The rest pose as model-space matrices, the reference a bind pose or an IK setup is measured against. ozz's <c>GetRestPoseModelSpace</c>.</summary>
    public static Matrix4x4[] RestPoseModelSpace(Skeleton skeleton)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        var models = new Matrix4x4[skeleton.JointCount];
        LocalToModel.Compute(skeleton, skeleton.RestPose, models);
        return models;
    }

    /// <summary>
    /// Visits <paramref name="root"/> and everything below it, parents before children, siblings in
    /// order — the order joints are stored in, so this is a plain forward scan. ozz's
    /// <c>IterateJointsDF</c>. A negative <paramref name="root"/> visits the whole skeleton.
    /// </summary>
    public static void IterateJointsDepthFirst(Skeleton skeleton, Action<int, int> visit, int root = Skeleton.NoParent)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        ArgumentNullException.ThrowIfNull(visit);
        var parents = skeleton.Parents;
        var count = skeleton.JointCount;
        // parents[i] >= root holds exactly as long as i is still under root, which is what makes
        // a depth-first subtree walk a contiguous forward scan.
        var i = root < 0 ? 0 : root;
        var process = i < count;
        while (process)
        {
            visit(i, parents[i]);
            i++;
            process = i < count && parents[i] >= root;
        }
    }

    /// <summary>The same traversal in reverse, children before parents — what an accumulation up the hierarchy needs. ozz's <c>IterateJointsDFReverse</c>.</summary>
    public static void IterateJointsDepthFirstReverse(Skeleton skeleton, Action<int, int> visit)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        ArgumentNullException.ThrowIfNull(visit);
        var parents = skeleton.Parents;
        for (var i = skeleton.JointCount - 1; i >= 0; i--) visit(i, parents[i]);
    }
}

/// <summary>Queries over an <see cref="AnimationClip"/>. ozz's <c>animation_utils.h</c>.</summary>
public static class AnimationUtils
{
    /// <summary>Keys in the translation stream — of one track, or of every track when <paramref name="track"/> is negative. ozz's <c>CountTranslationKeyframes</c>.</summary>
    public static int CountTranslationKeyframes(AnimationClip clip, int track = -1) => Count(clip, c => c.Translations, track);

    /// <summary>ozz's <c>CountRotationKeyframes</c>.</summary>
    public static int CountRotationKeyframes(AnimationClip clip, int track = -1) => Count(clip, c => c.Rotations, track);

    /// <summary>ozz's <c>CountScaleKeyframes</c>.</summary>
    public static int CountScaleKeyframes(AnimationClip clip, int track = -1) => Count(clip, c => c.Scales, track);

    /// <summary>Follows a track's back-links forward through the shared stream, counting the keys that belong to it.</summary>
    private static int Count(AnimationClip clip, Func<AnimationClip, KeyframeStream> select, int track)
    {
        ArgumentNullException.ThrowIfNull(clip);
        var previouses = select(clip).Previouses;
        if (track < 0) return previouses.Length;

        var count = 1;
        var previous = track;
        for (var i = previous + 1; i < previouses.Length; i++)
        {
            if (i - previouses[i] == previous)
            {
                count++;
                previous = i;
            }
        }

        return count;
    }
}

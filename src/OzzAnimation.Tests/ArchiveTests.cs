using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>Archives round-trip every field, and a foreign, big-endian or newer archive is refused by name rather than misread.</summary>
public class ArchiveTests
{
    [Test]
    public async Task a_skeleton_round_trips_in_depth_first_order()
    {
        var archive = TestRigs.Chain().Save();

        var read = Skeleton.Load(archive);

        await Assert.That(read.JointCount).IsEqualTo(3);
        await Assert.That(read.SoaJointCount).IsEqualTo(1);
        await Assert.That(read.Names[0]).IsEqualTo("hip");
        await Assert.That(read.Names[2]).IsEqualTo("prop");
        await Assert.That(read.Parents.ToArray()).IsEquivalentTo(new short[] { -1, 0, -1 });
        await Assert.That(read.RestPose[0].Translation).IsEqualTo(new Vector3(0, 1, 0));
        await Assert.That(read.RestPose[1].Rotation).IsEqualTo(TestRigs.QuarterTurnZ);
        await Assert.That(read.FindJoint("knee")).IsEqualTo(1);
        await Assert.That(read.FindJoint("toe")).IsEqualTo(-1);
        await Assert.That(read.IsLeaf(0)).IsFalse();
        await Assert.That(read.IsLeaf(1)).IsTrue();
        await Assert.That(Skeleton.IsSkeleton(archive)).IsTrue();
        await Assert.That(AnimationClip.IsAnimation(archive)).IsFalse();
    }

    [Test]
    public async Task the_spare_lanes_of_the_last_group_hold_identity()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();

        // Which is what keeps a blend or a hierarchy walk from reading rubbish out of the padding.
        var lastGroup = skeleton.SoaJointCount - 1;
        for (var lane = skeleton.JointCount - lastGroup * 4; lane < 4; lane++)
        {
            await Assert.That(skeleton.RestPose.Rotations[lastGroup].W[lane]).IsEqualTo(1f);
            await Assert.That(skeleton.RestPose.Scales[lastGroup].X[lane]).IsEqualTo(1f);
        }
    }

    [Test]
    public async Task an_empty_skeleton_is_a_valid_archive()
    {
        await Assert.That(Skeleton.Load(Skeleton.Empty.Save()).JointCount).IsEqualTo(0);
    }

    [Test]
    public async Task foreign_big_endian_and_newer_archives_are_refused_by_name()
    {
        var skeleton = TestRigs.Chain().Save();
        var bigEndian = (byte[])skeleton.Clone();
        bigEndian[0] = 0;
        var newer = (byte[])skeleton.Clone();
        BitConverter.TryWriteBytes(newer.AsSpan(1 + Skeleton.Tag.Length + 1), Skeleton.Version + 1);

        var foreign = await Assert.That(() => Skeleton.Load("not an archive"u8.ToArray())).Throws<InvalidDataException>();
        await Assert.That(foreign!.Message).Contains("ozz-skeleton");
        var wrongKind = await Assert.That(() => AnimationClip.Load(skeleton)).Throws<InvalidDataException>();
        await Assert.That(wrongKind!.Message).Contains("ozz-animation");
        var endian = await Assert.That(() => Skeleton.Load(bigEndian)).Throws<InvalidDataException>();
        await Assert.That(endian!.Message).Contains("big-endian");
        var version = await Assert.That(() => Skeleton.Load(newer)).Throws<InvalidDataException>();
        await Assert.That(version!.Message).Contains("version 3");
        var truncated = await Assert.That(() => Skeleton.Load(skeleton.AsSpan(0, skeleton.Length - 8).ToArray())).Throws<InvalidDataException>();
        await Assert.That(truncated!.Message).Contains("ends inside");
    }

    [Test]
    public async Task a_skeleton_whose_parent_follows_its_child_is_refused()
    {
        var error = await Assert.That(() => new Skeleton(["a", "b"], [1, -1], TestRigs.Pose(JointPose.Identity, JointPose.Identity))).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("depth-first");
    }

    [Test]
    public async Task keyframes_can_be_counted_per_track_and_in_total()
    {
        var clip = TestRigs.BenchmarkClip();

        var total = AnimationUtils.CountTranslationKeyframes(clip);
        var first = AnimationUtils.CountTranslationKeyframes(clip, 0);

        await Assert.That(total).IsEqualTo(clip.Translations.KeyCount);
        // Every track was baked with the same key count, and the builder adds none here.
        await Assert.That(first).IsEqualTo(TestRigs.BenchmarkKeyCount + 1);
        await Assert.That(AnimationUtils.CountRotationKeyframes(clip, 3)).IsEqualTo(TestRigs.BenchmarkKeyCount + 1);
        await Assert.That(AnimationUtils.CountScaleKeyframes(clip)).IsEqualTo(clip.Scales.KeyCount);
    }
}

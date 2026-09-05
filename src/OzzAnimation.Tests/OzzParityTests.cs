using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>
/// The format contract with ozz-animation 0.17, held as bytes. <c>ozz-skeleton.ozz</c> and
/// <c>ozz-animation.ozz</c> were written by ozz's own C++ builders from a procedural rig; loading
/// one here and saving it again must reproduce the file exactly, which pins every field's size,
/// order and encoding — the header counts, the SoA rest-pose groups, the keyframe streams, the
/// group-varint i-frames. A reader that got any of them wrong would round-trip differently.
/// </summary>
public class OzzParityTests
{
    [Test]
    public async Task a_skeleton_ozz_wrote_round_trips_byte_for_byte()
    {
        var bytes = TestRigs.Fixture("ozz-skeleton.ozz");

        var skeleton = Skeleton.Load(bytes);

        await Assert.That(skeleton.JointCount).IsEqualTo(37);
        await Assert.That(skeleton.Names[0]).IsEqualTo("j0");
        await Assert.That(skeleton.Parents[0]).IsEqualTo(Skeleton.NoParent);
        await Assert.That(skeleton.Save()).IsEquivalentTo(bytes);
    }

    [Test]
    public async Task a_clip_ozz_wrote_round_trips_byte_for_byte()
    {
        var bytes = TestRigs.Fixture("ozz-animation.ozz");

        var clip = AnimationClip.Load(bytes);

        await Assert.That(clip.Name).IsEqualTo("parity");
        await Assert.That(clip.Duration).IsEqualTo(2.5f);
        await Assert.That(clip.TrackCount).IsEqualTo(37);
        await Assert.That(clip.Rotations.IframeDesc.Length).IsEqualTo(2 * 5);
        await Assert.That(clip.Save()).IsEquivalentTo(bytes);
    }

    [Test]
    public async Task an_optimized_clip_ozz_wrote_round_trips_byte_for_byte()
    {
        var bytes = TestRigs.Fixture("ozz-animation-optimized.ozz");

        var clip = AnimationClip.Load(bytes);

        // The optimizer dropped keys, so this file exercises a different stream shape than the
        // unoptimized one: fewer keys, different back-link distances, a different ratio width.
        await Assert.That(clip.TrackCount).IsEqualTo(37);
        await Assert.That(clip.Translations.KeyCount).IsLessThan(AnimationClip.Load(TestRigs.Fixture("ozz-animation.ozz")).Translations.KeyCount);
        await Assert.That(clip.Save()).IsEquivalentTo(bytes);
    }

    [Test]
    public async Task an_archive_ozz_wrote_samples_to_finite_normalized_poses()
    {
        var skeleton = Skeleton.Load(TestRigs.Fixture("ozz-skeleton.ozz"));
        var clip = AnimationClip.Load(TestRigs.Fixture("ozz-animation.ozz"));
        var context = new SamplingContext(clip.TrackCount);
        var poses = new SoaTransforms(clip.TrackCount);

        await Assert.That(clip.TrackCount).IsEqualTo(skeleton.JointCount);
        foreach (var ratio in new[] { 0f, 0.3f, 0.9f, 0.1f, 1f })
        {
            context.Sample(clip, ratio, poses);
            foreach (var pose in poses.ToArray())
            {
                await Assert.That(MathF.Abs(pose.Rotation.Length() - 1f)).IsLessThan(1e-4f);
                await Assert.That(float.IsFinite(pose.Translation.X + pose.Translation.Y + pose.Translation.Z)).IsTrue();
            }
        }
    }

    [Test]
    public async Task the_sampler_reproduces_the_curve_the_clip_was_baked_from()
    {
        // Ground truth, not a second run of the same code: bench-animation.ozz was baked from a
        // closed-form curve, so this checks the whole decode path — timepoints, back-links,
        // half-float and 15-bit quaternion unpacking — against what it should be.
        var clip = TestRigs.BenchmarkClip();
        var context = new SamplingContext(clip.TrackCount);
        var poses = new SoaTransforms(clip.TrackCount);

        foreach (var key in new[] { 0, 7, 20, TestRigs.BenchmarkKeyCount })
        {
            var ratio = (float)key / TestRigs.BenchmarkKeyCount;
            context.Sample(clip, ratio, poses);
            for (var track = 0; track < clip.TrackCount; track++)
            {
                var expected = TestRigs.BenchmarkSource(track, ratio * clip.Duration);
                var actual = poses[track];
                await Assert.That(Vector3.Distance(actual.Translation, expected.Translation)).IsLessThan(1e-3f);
                await Assert.That(Vector3.Distance(actual.Scale, expected.Scale)).IsLessThan(1e-3f);
                await Assert.That(TestRigs.AngleBetween(actual.Rotation, expected.Rotation)).IsLessThan(1e-3f);
            }
        }
    }
}

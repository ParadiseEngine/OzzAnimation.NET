using System.Numerics;

using OzzAnimation.Offline;

namespace OzzAnimation.Tests;

/// <summary>The optimizer drops what a lerp reproduces and refuses what the skeleton cannot take; the builder splits tracks whose back-link would overflow.</summary>
public class OfflineTests
{
    [Test]
    public async Task the_optimizer_drops_keys_a_lerp_reproduces_and_keeps_the_rest()
    {
        var skeleton = TestRigs.Chain();
        var raw = TestRigs.Clip("walk", 1f, skeleton.JointCount, tracks =>
        {
            for (var k = 0; k <= 10; k++)
            {
                var t = k / 10f;
                tracks[0].Translations.Add(new TranslationKey(t, new Vector3(t, 0, 0)));
                tracks[0].Rotations.Add(new RotationKey(t, Quaternion.Identity));
                tracks[1].Rotations.Add(new RotationKey(t, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.05f * t * t)));
            }
        });

        var optimized = AnimationOptimizer.Optimize(raw, skeleton, AnimationOptimizer.Setting.Default);
        var strict = AnimationOptimizer.Optimize(raw, skeleton, AnimationOptimizer.Setting.Default, new Dictionary<int, AnimationOptimizer.Setting> { [1] = new(1e-6f, 1f) });

        await Assert.That(optimized.Tracks[0].Translations.Count).IsEqualTo(2);
        await Assert.That(optimized.Tracks[0].Rotations).IsEmpty();
        await Assert.That(optimized.Tracks[1].Rotations.Count).IsGreaterThan(2);
        await Assert.That(optimized.Tracks[1].Rotations.Count).IsLessThan(11);
        // Not all 11: below ~1e-3 radians the dot product of two rotations rounds to 1 in float, and
        // ozz's angular distance is zero there too.
        await Assert.That(strict.Tracks[1].Rotations.Count).IsGreaterThan(optimized.Tracks[1].Rotations.Count);
        await Assert.That(optimized.IsValid).IsTrue();
    }

    [Test]
    public async Task the_optimizer_refuses_a_clip_that_does_not_fit_the_skeleton()
    {
        var skeleton = TestRigs.Chain();

        await Assert.That(() => AnimationOptimizer.Optimize(new RawAnimation { Duration = 1f }, skeleton, AnimationOptimizer.Setting.Default)).Throws<ArgumentException>();
        await Assert.That(() => AnimationBuilder.Build(new RawAnimation { Duration = 0f })).Throws<ArgumentException>();
    }

    [Test]
    public async Task a_track_with_keys_far_apart_in_the_stream_is_split_so_the_back_link_fits()
    {
        // Keys sort by the time of the key before them, so between track 0's keys at 0.5 and 1
        // sit every other track's keys whose predecessor lies in that half: 1023 × 70 of them,
        // past the 16-bit back-link — the builder must insert a midpoint key on track 0.
        var raw = TestRigs.Clip("wide", 1f, Skeleton.MaxJoints, tracks =>
        {
            tracks[0].Translations.Add(new TranslationKey(0f, Vector3.Zero));
            tracks[0].Translations.Add(new TranslationKey(0.5f, new Vector3(1, 0, 0)));
            tracks[0].Translations.Add(new TranslationKey(1f, new Vector3(3, 0, 0)));
            for (var i = 1; i < tracks.Length; i++)
            {
                for (var k = 0; k < 140; k++) tracks[i].Translations.Add(new TranslationKey(k / 139f, new Vector3(k, 0, 0)));
            }
        });

        var read = AnimationClip.Load(AnimationBuilder.Build(raw).Save());
        var context = new SamplingContext(read.TrackCount);
        var poses = new SoaTransforms(read.TrackCount);
        context.Sample(read, 0.75f, poses);

        await Assert.That(read.Translations.KeyCount).IsGreaterThan(1023 * 140 + 3);
        await Assert.That(MathF.Abs(poses[0].Translation.X - 2f)).IsLessThan(0.01f);
        await Assert.That(MathF.Abs(poses[5].Translation.X - 104.25f)).IsLessThan(0.1f);
    }
}

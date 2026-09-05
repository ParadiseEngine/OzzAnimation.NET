using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>The sampler's cursor cache: walking forward, seeking backward and jumping to an i-frame must all land on the same pose a fresh context lands on.</summary>
public class SamplingTests
{
    [Test]
    public async Task seeking_in_any_order_gives_the_pose_a_fresh_context_gives()
    {
        // The whole point of the cursor cache is that it is a pure optimization: a context that
        // walked, rewound and jumped must agree exactly with one that started cold.
        var clip = TestRigs.BenchmarkClip();
        var walked = new SamplingContext(clip.TrackCount);
        var poses = new SoaTransforms(clip.TrackCount);
        var fresh = new SoaTransforms(clip.TrackCount);

        foreach (var ratio in new[] { 0f, 0.3f, 0.6f, 0.9f, 0.4f, 0.1f, 0.95f, 0.05f, 1f, 0.5f })
        {
            walked.Sample(clip, ratio, poses);
            new SamplingContext(clip.TrackCount).Sample(clip, ratio, fresh);
            await Assert.That(poses.ToArray()).IsEquivalentTo(fresh.ToArray());
        }
    }

    [Test]
    public async Task stepping_forward_frame_by_frame_agrees_with_seeking()
    {
        var clip = TestRigs.BenchmarkClip();
        var stepped = new SamplingContext(clip.TrackCount);
        var poses = new SoaTransforms(clip.TrackCount);
        var fresh = new SoaTransforms(clip.TrackCount);

        for (var frame = 0; frame <= 60; frame++)
        {
            var ratio = frame / 60f;
            stepped.Sample(clip, ratio, poses);
            new SamplingContext(clip.TrackCount).Sample(clip, ratio, fresh);
            await Assert.That(poses.ToArray()).IsEquivalentTo(fresh.ToArray());
        }
    }

    [Test]
    public async Task the_ratio_is_clamped_and_a_context_too_small_is_refused()
    {
        var clip = TestRigs.BenchmarkClip();
        var context = new SamplingContext(clip.TrackCount);
        var small = new SamplingContext(0);
        var poses = new SoaTransforms(clip.TrackCount);
        var end = new SoaTransforms(clip.TrackCount);

        context.Sample(clip, 1f, end);
        context.Sample(clip, 7f, poses);

        await Assert.That(poses.ToArray()).IsEquivalentTo(end.ToArray());
        var error = await Assert.That(() => small.Sample(clip, 0f, poses)).Throws<ArgumentException>();
        await Assert.That(error!.Message).Contains("at most 0");
    }

    [Test]
    public async Task a_context_reused_for_another_clip_restarts_its_cursor()
    {
        var first = TestRigs.BenchmarkClip();
        var second = AnimationClip.Load(TestRigs.Fixture("bench-animation.ozz"));
        var context = new SamplingContext(first.TrackCount);
        var poses = new SoaTransforms(first.TrackCount);
        var expected = new SoaTransforms(first.TrackCount);

        context.Sample(first, 0.9f, poses);
        context.Sample(second, 0.3f, poses);
        new SamplingContext(first.TrackCount).Sample(second, 0.3f, expected);

        await Assert.That(poses.ToArray()).IsEquivalentTo(expected.ToArray());
    }

    [Test]
    public async Task sampling_allocates_nothing()
    {
        var clip = TestRigs.BenchmarkClip();
        var skeleton = TestRigs.BenchmarkSkeleton();
        var context = new SamplingContext(clip.TrackCount);
        var poses = new SoaTransforms(clip.TrackCount);
        var models = new Matrix4x4[skeleton.JointCount];
        context.Sample(clip, 0f, poses);
        LocalToModel.Compute(skeleton, poses, models);

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var frame = 0; frame < 100; frame++)
        {
            context.Sample(clip, frame / 100f, poses);
            LocalToModel.Compute(skeleton, poses, models);
        }

        await Assert.That(GC.GetAllocatedBytesForCurrentThread() - before).IsEqualTo(0L);
    }
}

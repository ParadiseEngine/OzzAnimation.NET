using System.Numerics;

using OzzAnimation.Offline;

namespace OzzAnimation.Tests;

/// <summary>The sampler interpolates the way the builder laid keys out, seeks in both directions to the same pose, and pads unanimated joints with rest.</summary>
public class SamplingTests
{
    private const float Quantization = 2e-3f;

    /// <summary>The chain with the hip rising 1→3 on Y, the knee turning a quarter turn, and the prop doubling in scale half-way.</summary>
    private static (Skeleton Skeleton, AnimationClip Clip) Bend(float iframeInterval = 0f)
    {
        var skeleton = TestRigs.Chain();
        var raw = TestRigs.Clip("bend", 1f, skeleton.JointCount, tracks =>
        {
            tracks[0].Translations.Add(new TranslationKey(0f, new Vector3(0, 1, 0)));
            tracks[0].Translations.Add(new TranslationKey(0.5f, new Vector3(0, 2, 0)));
            tracks[0].Translations.Add(new TranslationKey(1f, new Vector3(0, 3, 0)));
            tracks[1].Rotations.Add(new RotationKey(0f, Quaternion.Identity));
            tracks[1].Rotations.Add(new RotationKey(1f, TestRigs.QuarterTurnZ));
            // A step hold, baked the way a STEP channel is: hold the old value up to the next key.
            tracks[2].Scales.Add(new ScaleKey(0f, Vector3.One));
            tracks[2].Scales.Add(new ScaleKey(0.5f - 1e-4f, Vector3.One));
            tracks[2].Scales.Add(new ScaleKey(0.5f, new Vector3(2, 2, 2)));
        });
        return (skeleton, AnimationBuilder.Build(raw, iframeInterval));
    }

    [Test]
    public async Task keys_interpolate_linearly_and_unanimated_components_hold_identity()
    {
        var (skeleton, clip) = Bend();
        var context = new SamplingContext(skeleton.JointCount);
        var poses = new SoaTransforms(skeleton.JointCount);

        context.Sample(clip, 0.25f, poses);

        await Assert.That(Vector3.Distance(poses[0].Translation, new Vector3(0, 1.5f, 0))).IsLessThan(Quantization);
        await Assert.That(Quaternion.Dot(poses[0].Rotation, Quaternion.Identity)).IsGreaterThan(1f - Quantization);
        var expected = Quaternion.Normalize(Quaternion.Lerp(Quaternion.Identity, TestRigs.QuarterTurnZ, 0.25f));
        await Assert.That(Quaternion.Dot(poses[1].Rotation, expected)).IsGreaterThan(1f - Quantization);
        await Assert.That(poses[1].Translation).IsEqualTo(Vector3.Zero);
        await Assert.That(Vector3.Distance(poses[2].Scale, Vector3.One)).IsLessThan(Quantization);
        await Assert.That(poses[2].Translation).IsEqualTo(Vector3.Zero);
    }

    [Test]
    public async Task a_step_channel_holds_its_value_until_the_next_key()
    {
        var (skeleton, clip) = Bend();
        var context = new SamplingContext(skeleton.JointCount);
        var poses = new SoaTransforms(skeleton.JointCount);

        context.Sample(clip, 0.49f, poses);
        var before = poses[2].Scale;
        context.Sample(clip, 0.5f, poses);
        var at = poses[2].Scale;

        await Assert.That(Vector3.Distance(before, Vector3.One)).IsLessThan(Quantization);
        await Assert.That(Vector3.Distance(at, new Vector3(2, 2, 2))).IsLessThan(Quantization);
    }

    [Test]
    public async Task seeking_backwards_gives_the_pose_a_fresh_context_gives()
    {
        foreach (var interval in new[] { 0f, 0.25f })
        {
            var (skeleton, clip) = Bend(interval);
            var walked = new SamplingContext(skeleton.JointCount);
            var poses = new SoaTransforms(skeleton.JointCount);
            foreach (var ratio in new[] { 0f, 0.3f, 0.6f, 0.9f, 0.4f, 0.1f, 0.95f, 0.05f })
            {
                walked.Sample(clip, ratio, poses);
                var fresh = new SoaTransforms(skeleton.JointCount);
                new SamplingContext(skeleton.JointCount).Sample(clip, ratio, fresh);
                await Assert.That(poses.ToArray()).IsEquivalentTo(fresh.ToArray());
            }
        }
    }

    [Test]
    public async Task the_ratio_is_clamped_and_a_context_too_small_is_refused()
    {
        var (skeleton, clip) = Bend();
        var context = new SamplingContext(skeleton.JointCount);
        var small = new SamplingContext(0);
        var poses = new SoaTransforms(skeleton.JointCount);
        var end = new SoaTransforms(skeleton.JointCount);

        context.Sample(clip, 1f, end);
        context.Sample(clip, 7f, poses);

        await Assert.That(poses.ToArray()).IsEquivalentTo(end.ToArray());
        var error = await Assert.That(() => small.Sample(clip, 0f, poses)).Throws<ArgumentException>();
        await Assert.That(error!.Message).Contains("at most 0");
    }

    [Test]
    public async Task a_context_reused_for_another_clip_restarts_its_cursor()
    {
        var (skeleton, first) = Bend();
        var (_, second) = Bend(0.25f);
        var context = new SamplingContext(skeleton.JointCount);
        var poses = new SoaTransforms(skeleton.JointCount);
        var expected = new SoaTransforms(skeleton.JointCount);

        context.Sample(first, 0.9f, poses);
        context.Sample(second, 0.3f, poses);
        new SamplingContext(skeleton.JointCount).Sample(second, 0.3f, expected);

        await Assert.That(poses.ToArray()).IsEquivalentTo(expected.ToArray());
    }

    [Test]
    public async Task local_to_model_walks_the_parents_in_the_row_vector_convention()
    {
        var skeleton = TestRigs.Chain();
        var locals = new SoaTransforms(skeleton.JointCount);
        locals.CopyFrom(skeleton.RestPoses);
        var models = new Matrix4x4[skeleton.JointCount];

        LocalToModel.Compute(skeleton, locals, models, Matrix4x4.CreateTranslation(10, 0, 0));

        await Assert.That(models[0].Translation).IsEqualTo(new Vector3(10, 1, 0));
        await Assert.That(TestRigs.MaxAbs(models[1] - Matrix4x4.CreateFromQuaternion(TestRigs.QuarterTurnZ) * Matrix4x4.CreateTranslation(10, 1, 0))).IsLessThan(1e-6f);
        await Assert.That(models[2].Translation).IsEqualTo(new Vector3(10, 0, 0));
    }
}

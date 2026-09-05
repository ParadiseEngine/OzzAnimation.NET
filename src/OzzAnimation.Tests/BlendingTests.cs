using System.Numerics;
using System.Runtime.Intrinsics;

namespace OzzAnimation.Tests;

/// <summary>Weighted blending: the normalization, the rest-pose fallback below the threshold, per-joint weights, and the additive pass.</summary>
public class BlendingTests
{
    private const int Joints = 4;

    private static readonly JointPose PoseA = new(Vector3.Zero, Quaternion.Identity, Vector3.One);
    private static readonly JointPose PoseB = new(new Vector3(2, 0, 0), TestRigs.QuarterTurnZ, new Vector3(3, 3, 3));
    private static readonly JointPose Rest = new(new Vector3(0, 10, 0), Quaternion.Identity, Vector3.One);

    [Test]
    public async Task two_equal_layers_give_the_midpoint()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers =
        [
            new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, PoseA) },
            new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, PoseB) },
        ];

        var ran = job.Run(layers, [], TestRigs.Uniform(Joints, Rest), output);

        await Assert.That(ran).IsTrue();
        var blended = output[0];
        await Assert.That(Vector3.Distance(blended.Translation, new Vector3(1, 0, 0))).IsLessThan(1e-6f);
        await Assert.That(Vector3.Distance(blended.Scale, new Vector3(2, 2, 2))).IsLessThan(1e-6f);
        var expected = Quaternion.Normalize(Quaternion.Lerp(Quaternion.Identity, TestRigs.QuarterTurnZ, 0.5f));
        await Assert.That(TestRigs.AngleBetween(blended.Rotation, expected)).IsLessThan(1e-5f);
    }

    [Test]
    public async Task weights_need_not_sum_to_one_because_the_blend_normalizes()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers =
        [
            new() { Weight = 30f, Transform = TestRigs.Uniform(Joints, PoseA) },
            new() { Weight = 10f, Transform = TestRigs.Uniform(Joints, PoseB) },
        ];

        job.Run(layers, [], TestRigs.Uniform(Joints, Rest), output);

        // 3:1 in favour of A.
        await Assert.That(Vector3.Distance(output[0].Translation, new Vector3(0.5f, 0, 0))).IsLessThan(1e-6f);
    }

    [Test]
    public async Task no_layers_at_all_gives_the_rest_pose()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);

        var ran = job.Run([], [], TestRigs.Uniform(Joints, Rest), output);

        await Assert.That(ran).IsTrue();
        await Assert.That(Vector3.Distance(output[0].Translation, Rest.Translation)).IsLessThan(1e-6f);
    }

    [Test]
    public async Task a_layer_below_the_threshold_is_topped_up_with_the_rest_pose()
    {
        // Without the fallback a nearly-zero total weight would divide the pose into nonsense;
        // instead the rest pose makes up the missing weight.
        var job = new BlendingJob(1) { Threshold = 0.1f };
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers = [new() { Weight = 0.05f, Transform = TestRigs.Uniform(Joints, PoseB) }];

        job.Run(layers, [], TestRigs.Uniform(Joints, Rest), output);

        // Half the layer, half the rest pose: 0.05 of each, normalized by the 0.1 threshold.
        var expected = (PoseB.Translation + Rest.Translation) * 0.5f;
        await Assert.That(Vector3.Distance(output[0].Translation, expected)).IsLessThan(1e-6f);
    }

    [Test]
    public async Task per_joint_weights_split_the_skeleton_between_layers()
    {
        // Partial blending: the upper body from one clip, the lower from another, in one pass.
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers =
        [
            new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, PoseA), JointWeights = [Vector128.Create(1f, 1f, 0f, 0f)] },
            new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, PoseB), JointWeights = [Vector128.Create(0f, 0f, 1f, 1f)] },
        ];

        job.Run(layers, [], TestRigs.Uniform(Joints, Rest), output);

        await Assert.That(Vector3.Distance(output[0].Translation, PoseA.Translation)).IsLessThan(1e-6f);
        await Assert.That(Vector3.Distance(output[1].Translation, PoseA.Translation)).IsLessThan(1e-6f);
        await Assert.That(Vector3.Distance(output[2].Translation, PoseB.Translation)).IsLessThan(1e-6f);
        await Assert.That(Vector3.Distance(output[3].Translation, PoseB.Translation)).IsLessThan(1e-6f);
    }

    [Test]
    public async Task an_additive_layer_is_applied_on_top_of_the_blend()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers = [new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, PoseA) }];
        var additive = new JointPose(new Vector3(1, 0, 0), TestRigs.QuarterTurnZ, new Vector3(2, 2, 2));
        BlendingLayer[] additiveLayers = [new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, additive) }];

        job.Run(layers, additiveLayers, TestRigs.Uniform(Joints, Rest), output);

        var result = output[0];
        await Assert.That(Vector3.Distance(result.Translation, PoseA.Translation + additive.Translation)).IsLessThan(1e-6f);
        await Assert.That(Vector3.Distance(result.Scale, PoseA.Scale * additive.Scale)).IsLessThan(1e-5f);
        await Assert.That(TestRigs.AngleBetween(result.Rotation, PoseA.Rotation * additive.Rotation)).IsLessThan(1e-5f);
    }

    [Test]
    public async Task an_additive_layer_at_half_weight_applies_half_of_it()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers = [new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, PoseA) }];
        var additive = new JointPose(new Vector3(1, 0, 0), TestRigs.QuarterTurnZ, Vector3.One);
        BlendingLayer[] additiveLayers = [new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, additive) }];

        job.Run(layers, additiveLayers, TestRigs.Uniform(Joints, Rest), output);

        var result = output[0];
        await Assert.That(Vector3.Distance(result.Translation, new Vector3(0.5f, 0, 0))).IsLessThan(1e-6f);
        var half = Quaternion.Normalize(Quaternion.Lerp(Quaternion.Identity, TestRigs.QuarterTurnZ, 0.5f));
        await Assert.That(TestRigs.AngleBetween(result.Rotation, half)).IsLessThan(1e-5f);
    }

    [Test]
    public async Task a_negative_additive_weight_subtracts_what_a_positive_one_added()
    {
        // The subtractive pass is the exact inverse of the additive one, which is what makes an
        // additive layer removable without rebuilding the blend.
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        BlendingLayer[] layers = [new() { Weight = 1f, Transform = TestRigs.Uniform(Joints, PoseB) }];
        var additive = TestRigs.Uniform(Joints, new JointPose(new Vector3(1, 2, 3), TestRigs.QuarterTurnZ, new Vector3(2, 2, 2)));
        BlendingLayer[] additiveLayers =
        [
            new() { Weight = 1f, Transform = additive },
            new() { Weight = -1f, Transform = additive },
        ];

        job.Run(layers, additiveLayers, TestRigs.Uniform(Joints, Rest), output);

        var result = output[0];
        await Assert.That(Vector3.Distance(result.Translation, PoseB.Translation)).IsLessThan(1e-5f);
        await Assert.That(Vector3.Distance(result.Scale, PoseB.Scale)).IsLessThan(1e-5f);
        await Assert.That(TestRigs.AngleBetween(result.Rotation, PoseB.Rotation)).IsLessThan(1e-5f);
    }

    [Test]
    public async Task bad_arguments_are_refused_rather_than_thrown()
    {
        // Sizes are compared in groups of four, as in ozz, so "too small" means a whole group
        // short: an 8-joint rest pose against a 4-joint layer.
        var job = new BlendingJob(2);
        var rest = TestRigs.Uniform(8, Rest);
        var output = new SoaTransforms(8);
        BlendingLayer[] tooSmall = [new() { Weight = 1f, Transform = TestRigs.Uniform(4, PoseA) }];
        BlendingLayer[] noTransform = [new() { Weight = 1f }];
        BlendingLayer[] shortJointWeights = [new() { Weight = 1f, Transform = TestRigs.Uniform(8, PoseA), JointWeights = [Vector128<float>.One] }];

        await Assert.That(job.Run(tooSmall, [], rest, output)).IsFalse();
        await Assert.That(job.Run([], tooSmall, rest, output)).IsFalse();
        await Assert.That(job.Run(noTransform, [], rest, output)).IsFalse();
        await Assert.That(job.Run(shortJointWeights, [], rest, output)).IsFalse();
        await Assert.That(job.Run([], [], rest, new SoaTransforms(4))).IsFalse();
        // A job sized for fewer groups than the rest pose cannot accumulate its weights.
        await Assert.That(new BlendingJob(1).Run([], [], rest, output)).IsFalse();
        job.Threshold = 0f;
        await Assert.That(job.Run([], [], rest, output)).IsFalse();
    }

    [Test]
    public async Task blending_allocates_nothing()
    {
        var job = new BlendingJob(1);
        var output = new SoaTransforms(Joints);
        var rest = TestRigs.Uniform(Joints, Rest);
        BlendingLayer[] layers =
        [
            new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, PoseA) },
            new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, PoseB) },
        ];
        BlendingLayer[] additiveLayers = [new() { Weight = 0.5f, Transform = TestRigs.Uniform(Joints, PoseB) }];
        job.Run(layers, additiveLayers, rest, output);

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 100; i++) job.Run(layers, additiveLayers, rest, output);

        await Assert.That(GC.GetAllocatedBytesForCurrentThread() - before).IsEqualTo(0L);
    }
}

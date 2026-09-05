using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>Hierarchy traversal, and blending of root motion.</summary>
public class SkeletonUtilsTests
{
    [Test]
    public async Task a_depth_first_walk_visits_parents_before_children()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var seen = new List<int>();
        var visited = new bool[skeleton.JointCount];
        var outOfOrder = new List<int>();

        SkeletonUtils.IterateJointsDepthFirst(skeleton, (joint, parent) =>
        {
            if (parent != Skeleton.NoParent && !visited[parent]) outOfOrder.Add(joint);
            visited[joint] = true;
            seen.Add(joint);
        }, root: Skeleton.NoParent);

        await Assert.That(outOfOrder).IsEmpty();
        await Assert.That(seen.Count).IsEqualTo(skeleton.JointCount);
        await Assert.That(seen).IsEquivalentTo(Enumerable.Range(0, skeleton.JointCount).ToList());
    }

    [Test]
    public async Task a_walk_from_a_joint_covers_exactly_its_subtree()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var seen = new List<int>();

        SkeletonUtils.IterateJointsDepthFirst(skeleton, (joint, _) => seen.Add(joint), root: 2);

        var expected = Enumerable.Range(0, skeleton.JointCount).Where(j => IsUnder(skeleton, j, 2)).ToList();
        await Assert.That(seen).IsEquivalentTo(expected);
        await Assert.That(seen[0]).IsEqualTo(2);
        await Assert.That(seen.Count).IsGreaterThan(1);
    }

    [Test]
    public async Task the_reverse_walk_visits_children_before_parents()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var visited = new bool[skeleton.JointCount];

        SkeletonUtils.IterateJointsDepthFirstReverse(skeleton, (joint, parent) =>
        {
            if (parent != Skeleton.NoParent && visited[parent]) Assert.Fail($"Parent {parent} was visited before its child {joint}.");
            visited[joint] = true;
        });

        await Assert.That(visited.All(v => v)).IsTrue();
    }

    [Test]
    public async Task a_joints_rest_pose_can_be_read_back_one_at_a_time()
    {
        var skeleton = TestRigs.Chain();

        await Assert.That(SkeletonUtils.JointRestPoseLocalSpace(skeleton, 0).Translation).IsEqualTo(new Vector3(0, 1, 0));
        await Assert.That(SkeletonUtils.JointRestPoseLocalSpace(skeleton, 1).Rotation).IsEqualTo(TestRigs.QuarterTurnZ);
        await Assert.That(() => SkeletonUtils.JointRestPoseLocalSpace(skeleton, 9)).Throws<ArgumentOutOfRangeException>();
    }

    private static bool IsUnder(Skeleton skeleton, int joint, int root)
    {
        for (var i = joint; i != Skeleton.NoParent; i = skeleton.Parents[i])
        {
            if (i == root) return true;
        }

        return false;
    }
}

/// <summary>Blending root motion, where direction and distance are interpolated apart.</summary>
public class MotionBlendingTests
{
    [Test]
    public async Task one_layer_passes_its_delta_through()
    {
        var delta = new JointPose(new Vector3(0, 0, 2), TestRigs.QuarterTurnZ, Vector3.One);

        MotionBlending.Blend([new MotionLayer(1f, delta)], out var output);

        await Assert.That(Vector3.Distance(output.Translation, delta.Translation)).IsLessThan(1e-5f);
        await Assert.That(TestRigs.AngleBetween(output.Rotation, delta.Rotation)).IsLessThan(1e-5f);
        await Assert.That(output.Scale).IsEqualTo(Vector3.One);
    }

    [Test]
    public async Task two_directions_blend_without_losing_distance()
    {
        // This is the reason translation is split into direction and length: lerping the vectors
        // directly would give |(1,0,0)+(0,0,1)|/2 ≈ 0.707, and the character would creep.
        var forward = new JointPose(new Vector3(2, 0, 0), Quaternion.Identity, Vector3.One);
        var sideways = new JointPose(new Vector3(0, 0, 2), Quaternion.Identity, Vector3.One);

        MotionBlending.Blend([new MotionLayer(0.5f, forward), new MotionLayer(0.5f, sideways)], out var output);

        await Assert.That(output.Translation.Length()).IsEqualTo(2f).Within(1e-5f);
        await Assert.That(Vector3.Normalize(output.Translation).X).IsEqualTo(Vector3.Normalize(output.Translation).Z).Within(1e-5f);
    }

    [Test]
    public async Task weights_are_normalized_and_a_zero_weight_layer_is_ignored()
    {
        var moving = new JointPose(new Vector3(4, 0, 0), Quaternion.Identity, Vector3.One);
        var still = new JointPose(Vector3.Zero, Quaternion.Identity, Vector3.One);

        MotionBlending.Blend([new MotionLayer(3f, moving), new MotionLayer(1f, still)], out var output);

        // Three quarters of the way toward the moving layer's distance.
        await Assert.That(output.Translation.X).IsEqualTo(3f).Within(1e-5f);
    }

    [Test]
    public async Task no_layers_gives_an_identity_delta_and_reports_it()
    {
        var ran = MotionBlending.Blend([], out var output);

        await Assert.That(ran).IsFalse();
        await Assert.That(output.Translation).IsEqualTo(Vector3.Zero);
        await Assert.That(output.Rotation).IsEqualTo(Quaternion.Identity);
        await Assert.That(output.Scale).IsEqualTo(Vector3.One);
    }
}

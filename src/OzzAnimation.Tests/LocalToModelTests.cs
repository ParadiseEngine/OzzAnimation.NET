using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>The hierarchy walk, whole and partial.</summary>
public class LocalToModelTests
{
    [Test]
    public async Task it_walks_the_parents_in_the_row_vector_convention()
    {
        var skeleton = TestRigs.Chain();
        var models = new Matrix4x4[skeleton.JointCount];

        LocalToModel.Compute(skeleton, skeleton.RestPose, models, Matrix4x4.CreateTranslation(10, 0, 0));

        await Assert.That(models[0].Translation).IsEqualTo(new Vector3(10, 1, 0));
        await Assert.That(TestRigs.MaxAbs(models[1] - Matrix4x4.CreateFromQuaternion(TestRigs.QuarterTurnZ) * Matrix4x4.CreateTranslation(10, 1, 0))).IsLessThan(1e-6f);
        await Assert.That(models[2].Translation).IsEqualTo(new Vector3(10, 0, 0));
    }

    [Test]
    public async Task the_rest_pose_in_model_space_matches_a_full_walk()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var expected = new Matrix4x4[skeleton.JointCount];
        LocalToModel.Compute(skeleton, skeleton.RestPose, expected);

        var models = SkeletonUtils.RestPoseModelSpace(skeleton);

        await Assert.That(models).IsEquivalentTo(expected);
    }

    [Test]
    public async Task a_partial_walk_updates_only_the_subtree_it_names()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var full = new Matrix4x4[skeleton.JointCount];
        LocalToModel.Compute(skeleton, skeleton.RestPose, full);

        // Start from a marker everywhere, then update only joint 2's subtree; anything outside it
        // must still hold the marker, and anything inside must match the full walk.
        var partial = new Matrix4x4[skeleton.JointCount];
        Array.Fill(partial, Matrix4x4.CreateScale(7f));
        partial[skeleton.Parents[2]] = full[skeleton.Parents[2]];
        LocalToModel.Compute(skeleton, skeleton.RestPose, partial, from: 2, to: Skeleton.MaxJoints, fromExcluded: false);

        var updated = 0;
        for (var i = 0; i < skeleton.JointCount; i++)
        {
            if (IsUnder(skeleton, i, 2))
            {
                updated++;
                await Assert.That(TestRigs.MaxAbs(partial[i] - full[i])).IsLessThan(1e-6f);
            }
            else if (i != skeleton.Parents[2])
            {
                await Assert.That(partial[i]).IsEqualTo(Matrix4x4.CreateScale(7f));
            }
        }

        await Assert.That(updated).IsGreaterThan(1);
    }

    [Test]
    public async Task from_excluded_propagates_a_model_matrix_set_by_hand()
    {
        // This is how an IK result is pushed down a chain: the solved joint's model matrix is
        // written directly, and only its children are recomputed from it.
        var skeleton = TestRigs.BenchmarkSkeleton();
        var models = new Matrix4x4[skeleton.JointCount];
        LocalToModel.Compute(skeleton, skeleton.RestPose, models);
        var before = (Matrix4x4[])models.Clone();

        var moved = Matrix4x4.CreateTranslation(0, 5, 0);
        models[1] = moved;
        LocalToModel.Compute(skeleton, skeleton.RestPose, models, from: 1, to: Skeleton.MaxJoints, fromExcluded: true);

        await Assert.That(models[1]).IsEqualTo(moved);
        await Assert.That(models[0]).IsEqualTo(before[0]);
        for (var i = 0; i < skeleton.JointCount; i++)
        {
            if (skeleton.Parents[i] == 1)
            {
                await Assert.That(TestRigs.MaxAbs(models[i] - before[i])).IsGreaterThan(1e-3f);
            }
        }
    }

    [Test]
    public async Task the_walk_stops_at_the_to_joint()
    {
        var skeleton = TestRigs.BenchmarkSkeleton();
        var models = new Matrix4x4[skeleton.JointCount];
        Array.Fill(models, Matrix4x4.CreateScale(7f));

        LocalToModel.Compute(skeleton, skeleton.RestPose, models, from: Skeleton.NoParent, to: 2, fromExcluded: false);

        await Assert.That(models[2]).IsNotEqualTo(Matrix4x4.CreateScale(7f));
        await Assert.That(models[3]).IsEqualTo(Matrix4x4.CreateScale(7f));
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

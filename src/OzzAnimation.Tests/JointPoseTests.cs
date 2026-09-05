using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>The per-joint transform: its matrix form, the normalization every authoring path applies, and value equality.</summary>
public class JointPoseTests
{
    private static readonly JointPose Pose = new(new Vector3(1, 2, 3), TestRigs.QuarterTurnZ, new Vector3(2, 2, 2));

    [Test]
    public async Task to_matrix_scales_then_rotates_then_translates()
    {
        var matrix = Pose.ToMatrix();

        // Row-vector convention, so a point runs through scale, then rotation, then translation.
        var point = Vector3.Transform(new Vector3(1, 0, 0), matrix);
        var expected = Vector3.Transform(new Vector3(1, 0, 0) * 2f, TestRigs.QuarterTurnZ) + new Vector3(1, 2, 3);
        await Assert.That(Vector3.Distance(point, expected)).IsLessThan(1e-5f);
        await Assert.That(matrix.Translation).IsEqualTo(new Vector3(1, 2, 3));
    }

    [Test]
    public async Task an_identity_pose_is_the_identity_matrix()
    {
        await Assert.That(TestRigs.MaxAbs(JointPose.Identity.ToMatrix() - Matrix4x4.Identity)).IsLessThan(1e-6f);
        await Assert.That(JointPose.Identity.Translation).IsEqualTo(Vector3.Zero);
        await Assert.That(JointPose.Identity.Rotation).IsEqualTo(Quaternion.Identity);
        await Assert.That(JointPose.Identity.Scale).IsEqualTo(Vector3.One);
    }

    [Test]
    public async Task normalizing_makes_the_rotation_unit_and_leaves_the_rest_alone()
    {
        var scaled = new JointPose(new Vector3(1, 2, 3), new Quaternion(0, 0, 3, 3), new Vector3(4, 5, 6));

        var normalized = scaled.WithNormalizedRotation();

        await Assert.That(MathF.Abs(normalized.Rotation.Length() - 1f)).IsLessThan(1e-6f);
        await Assert.That(TestRigs.AngleBetween(normalized.Rotation, TestRigs.QuarterTurnZ)).IsLessThan(1e-5f);
        await Assert.That(normalized.Translation).IsEqualTo(scaled.Translation);
        await Assert.That(normalized.Scale).IsEqualTo(scaled.Scale);
    }

    [Test]
    public async Task a_zero_rotation_normalizes_to_identity_rather_than_a_nan()
    {
        // ozz's NormalizeSafe. Without the guard this divides by zero and every joint below it in
        // the hierarchy inherits the NaN.
        var degenerate = new JointPose(Vector3.One, new Quaternion(0, 0, 0, 0), Vector3.One);

        var normalized = degenerate.WithNormalizedRotation();

        await Assert.That(normalized.Rotation).IsEqualTo(Quaternion.Identity);
        await Assert.That(normalized.Translation).IsEqualTo(Vector3.One);
    }

    [Test]
    public async Task poses_compare_and_hash_by_value()
    {
        var same = new JointPose(new Vector3(1, 2, 3), TestRigs.QuarterTurnZ, new Vector3(2, 2, 2));
        var different = new JointPose(new Vector3(9, 2, 3), TestRigs.QuarterTurnZ, new Vector3(2, 2, 2));

        await Assert.That(Pose.Equals(same)).IsTrue();
        await Assert.That(Pose.Equals(different)).IsFalse();
        await Assert.That(Pose.Equals((object)same)).IsTrue();
        await Assert.That(Pose.Equals((object?)null)).IsFalse();
        await Assert.That(Pose.Equals("not a pose")).IsFalse();
        await Assert.That(Pose.GetHashCode()).IsEqualTo(same.GetHashCode());
        await Assert.That(Pose == same).IsTrue();
        await Assert.That(Pose != same).IsFalse();
        await Assert.That(Pose == different).IsFalse();
        await Assert.That(Pose != different).IsTrue();
    }

    [Test]
    public async Task the_string_form_names_all_three_components()
    {
        var text = Pose.ToString();

        await Assert.That(text).Contains("Translation");
        await Assert.That(text).Contains("Rotation");
        await Assert.That(text).Contains("Scale");
    }
}

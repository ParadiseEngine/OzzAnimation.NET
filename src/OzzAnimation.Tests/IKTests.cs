using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>
/// The two IK jobs, checked by their contract rather than their intermediates: after the returned
/// corrections are applied to the local pose and the hierarchy is re-walked, the end joint sits on
/// the target (two-bone) or the forward axis points at it (aim).
/// </summary>
public class IKTests
{
    /// <summary>Applies a correction the way ozz's samples do — in the joint's own space, so it post-multiplies the local rotation.</summary>
    private static void ApplyCorrection(SoaTransforms locals, int joint, Quaternion correction)
    {
        var pose = locals[joint];
        locals[joint] = new JointPose(pose.Translation, pose.Rotation * correction, pose.Scale);
    }

    private static (Skeleton Skeleton, SoaTransforms Locals, Matrix4x4[] Models) Arm()
    {
        var skeleton = TestRigs.Arm();
        var locals = new SoaTransforms(skeleton.JointCount);
        locals.CopyFrom(skeleton.RestPoses);
        var models = new Matrix4x4[skeleton.JointCount];
        LocalToModel.Compute(skeleton, locals, models);
        return (skeleton, locals, models);
    }

    [Test]
    public async Task two_bone_ik_puts_the_end_joint_on_a_reachable_target()
    {
        var (skeleton, locals, models) = Arm();
        var target = new Vector3(1f, 1f, 0f);   // 1.414 away, chain reaches 2
        var job = new IKTwoBoneJob
        {
            Target = target,
            MidAxis = Vector3.UnitZ,
            PoleVector = Vector3.UnitY,
            StartJoint = models[0],
            MidJoint = models[1],
            EndJoint = models[2],
        };

        var ran = job.Run(out var start, out var mid, out var reached);

        await Assert.That(ran).IsTrue();
        await Assert.That(reached).IsTrue();
        ApplyCorrection(locals, 0, start);
        ApplyCorrection(locals, 1, mid);
        LocalToModel.Compute(skeleton, locals, models);
        await Assert.That(Vector3.Distance(models[2].Translation, target)).IsLessThan(1e-4f);
        // The elbow had to bend to get there.
        await Assert.That(TestRigs.AngleBetween(mid, Quaternion.Identity)).IsGreaterThan(0.1f);
    }

    [Test]
    public async Task two_bone_ik_reaches_targets_all_around_the_chain()
    {
        foreach (var target in new[]
                 {
                     new Vector3(1.5f, 0.5f, 0f), new Vector3(0.5f, 1.2f, 0f), new Vector3(1f, -1f, 0f),
                     new Vector3(0.8f, 0.3f, 0.9f), new Vector3(-0.5f, 1f, 0.2f), new Vector3(0f, 0f, 1.7f),
                 })
        {
            var (skeleton, locals, models) = Arm();
            var job = new IKTwoBoneJob
            {
                Target = target,
                MidAxis = Vector3.UnitZ,
                PoleVector = Vector3.UnitY,
                StartJoint = models[0],
                MidJoint = models[1],
                EndJoint = models[2],
            };

            job.Run(out var start, out var mid, out var reached);

            await Assert.That(reached).IsTrue();
            ApplyCorrection(locals, 0, start);
            ApplyCorrection(locals, 1, mid);
            LocalToModel.Compute(skeleton, locals, models);
            await Assert.That(Vector3.Distance(models[2].Translation, target)).IsLessThan(1e-4f);
        }
    }

    [Test]
    public async Task an_unreachable_target_straightens_the_chain_and_reports_it()
    {
        var (skeleton, locals, models) = Arm();
        var target = new Vector3(5f, 0f, 0f);   // well past the chain's reach of 2

        var job = new IKTwoBoneJob
        {
            Target = target,
            MidAxis = Vector3.UnitZ,
            PoleVector = Vector3.UnitY,
            StartJoint = models[0],
            MidJoint = models[1],
            EndJoint = models[2],
        };
        job.Run(out var start, out var mid, out var reached);

        await Assert.That(reached).IsFalse();
        ApplyCorrection(locals, 0, start);
        ApplyCorrection(locals, 1, mid);
        LocalToModel.Compute(skeleton, locals, models);
        // Fully extended along the target direction, stopping at the chain's length.
        await Assert.That(Vector3.Distance(models[2].Translation, new Vector3(2, 0, 0))).IsLessThan(1e-4f);
    }

    [Test]
    public async Task softening_keeps_a_bend_as_the_chain_approaches_full_extension()
    {
        // With soften below 1 the chain eases toward full extension instead of snapping straight,
        // which is what removes the pop at the end of a reach.
        var target = new Vector3(1.99f, 0f, 0f);
        var reach = new Func<float, float>(soften =>
        {
            var (skeleton, locals, models) = Arm();
            var job = new IKTwoBoneJob
            {
                Target = target,
                MidAxis = Vector3.UnitZ,
                PoleVector = Vector3.UnitY,
                Soften = soften,
                StartJoint = models[0],
                MidJoint = models[1],
                EndJoint = models[2],
            };
            job.Run(out var start, out var mid, out _);
            ApplyCorrection(locals, 0, start);
            ApplyCorrection(locals, 1, mid);
            LocalToModel.Compute(skeleton, locals, models);
            return models[2].Translation.Length();
        });

        await Assert.That(reach(1f)).IsGreaterThan(reach(0.5f));
        await Assert.That(reach(0.5f)).IsLessThan(1.99f);
    }

    [Test]
    public async Task a_zero_weight_two_bone_correction_is_identity_and_a_half_weight_is_partial()
    {
        var (_, _, models) = Arm();
        var full = new IKTwoBoneJob { Target = new Vector3(1, 1, 0), MidAxis = Vector3.UnitZ, PoleVector = Vector3.UnitY, StartJoint = models[0], MidJoint = models[1], EndJoint = models[2] };
        var none = full with { Weight = 0f };
        var half = full with { Weight = 0.5f };

        none.Run(out var noneStart, out var noneMid, out var noneReached);
        full.Run(out _, out var fullMid, out _);
        half.Run(out _, out var halfMid, out var halfReached);

        await Assert.That(noneStart).IsEqualTo(Quaternion.Identity);
        await Assert.That(noneMid).IsEqualTo(Quaternion.Identity);
        await Assert.That(noneReached).IsFalse();
        // Reached is only reported when the correction was applied in full.
        await Assert.That(halfReached).IsFalse();
        await Assert.That(TestRigs.AngleBetween(halfMid, Quaternion.Identity)).IsLessThan(TestRigs.AngleBetween(fullMid, Quaternion.Identity));
    }

    [Test]
    public async Task a_two_bone_job_with_an_unnormalized_mid_axis_is_refused()
    {
        var (_, _, models) = Arm();
        var job = new IKTwoBoneJob { MidAxis = new Vector3(0, 0, 2), StartJoint = models[0], MidJoint = models[1], EndJoint = models[2] };

        await Assert.That(job.Validate()).IsFalse();
        await Assert.That(job.Run(out var start, out _, out _)).IsFalse();
        await Assert.That(start).IsEqualTo(Quaternion.Identity);
    }

    [Test]
    public async Task aim_ik_points_the_forward_axis_at_the_target()
    {
        foreach (var target in new[] { new Vector3(0, 5, 0), new Vector3(0, 0, 3), new Vector3(-2, 1, 4), new Vector3(1, 1, 1) })
        {
            var job = new IKAimJob
            {
                Target = target,
                Forward = Vector3.UnitX,
                Up = Vector3.UnitY,
                PoleVector = Vector3.UnitY,
                Joint = Matrix4x4.Identity,
            };

            var ran = job.Run(out var correction, out var reached);

            await Assert.That(ran).IsTrue();
            await Assert.That(reached).IsTrue();
            var aimed = Vector3.Transform(Vector3.UnitX, correction);
            await Assert.That(Vector3.Distance(aimed, Vector3.Normalize(target))).IsLessThan(1e-5f);
        }
    }

    [Test]
    public async Task aim_ik_rolls_the_up_axis_toward_the_pole_vector()
    {
        // Aiming leaves one degree of freedom — the roll about the aim axis — and the pole vector
        // is what fixes it. Aiming +X at +Z with the pole on +Y should keep up near +Y.
        var job = new IKAimJob
        {
            Target = new Vector3(0, 0, 4),
            Forward = Vector3.UnitX,
            Up = Vector3.UnitY,
            PoleVector = Vector3.UnitY,
            Joint = Matrix4x4.Identity,
        };

        job.Run(out var correction, out _);

        var up = Vector3.Transform(Vector3.UnitY, correction);
        await Assert.That(Vector3.Distance(up, Vector3.UnitY)).IsLessThan(1e-5f);
    }

    [Test]
    public async Task a_twist_angle_spins_the_aim_about_its_own_axis()
    {
        var job = new IKAimJob { Target = new Vector3(0, 0, 4), Forward = Vector3.UnitX, Up = Vector3.UnitY, PoleVector = Vector3.UnitY, Joint = Matrix4x4.Identity };
        var twisted = job with { TwistAngle = MathF.PI / 2f };

        job.Run(out var plain, out _);
        twisted.Run(out var rolled, out _);

        // Still aimed at the target...
        var aimed = Vector3.Transform(Vector3.UnitX, rolled);
        await Assert.That(Vector3.Distance(aimed, Vector3.UnitZ)).IsLessThan(1e-5f);
        // ...but rolled a quarter turn about it.
        await Assert.That(TestRigs.AngleBetween(plain, rolled)).IsGreaterThan(1f);
    }

    [Test]
    public async Task an_offset_beyond_the_targets_distance_cannot_be_aimed()
    {
        var job = new IKAimJob
        {
            Target = new Vector3(1, 0, 0),
            Forward = Vector3.UnitX,
            Offset = new Vector3(0, 2, 0),   // further from the joint than the target is
            Up = Vector3.UnitY,
            PoleVector = Vector3.UnitY,
            Joint = Matrix4x4.Identity,
        };

        var ran = job.Run(out var correction, out var reached);

        await Assert.That(ran).IsTrue();
        await Assert.That(reached).IsFalse();
        await Assert.That(correction).IsEqualTo(Quaternion.Identity);
    }

    [Test]
    public async Task an_aim_job_with_an_unnormalized_forward_is_refused()
    {
        var job = new IKAimJob { Forward = new Vector3(3, 0, 0), Joint = Matrix4x4.Identity };

        await Assert.That(job.Validate()).IsFalse();
        await Assert.That(job.Run(out _, out _)).IsFalse();
    }

    [Test]
    public async Task ik_allocates_nothing()
    {
        var (_, _, models) = Arm();
        var twoBone = new IKTwoBoneJob { Target = new Vector3(1, 1, 0), MidAxis = Vector3.UnitZ, PoleVector = Vector3.UnitY, StartJoint = models[0], MidJoint = models[1], EndJoint = models[2] };
        var aim = new IKAimJob { Target = new Vector3(0, 0, 4), Forward = Vector3.UnitX, Up = Vector3.UnitY, PoleVector = Vector3.UnitY, Joint = models[0] };
        twoBone.Run(out _, out _, out _);
        aim.Run(out _, out _);

        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 100; i++)
        {
            twoBone.Run(out _, out _, out _);
            aim.Run(out _, out _);
        }

        await Assert.That(GC.GetAllocatedBytesForCurrentThread() - before).IsEqualTo(0L);
    }
}

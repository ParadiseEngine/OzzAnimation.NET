using System.Numerics;

namespace OzzAnimation.Tests;

/// <summary>User channels: sampling, the step flag, the five value types and their archives.</summary>
public class TrackTests
{
    private static FloatTrack Ramp() => new([0f, 0.5f, 1f], [0f, 10f, 20f], [0]);

    [Test]
    public async Task a_track_interpolates_between_its_keys_and_clamps_outside_them()
    {
        var track = Ramp();

        await Assert.That(track.Sample(0f)).IsEqualTo(0f);
        await Assert.That(track.Sample(0.25f)).IsEqualTo(5f).Within(1e-5f);
        await Assert.That(track.Sample(0.5f)).IsEqualTo(10f).Within(1e-5f);
        await Assert.That(track.Sample(0.75f)).IsEqualTo(15f).Within(1e-5f);
        await Assert.That(track.Sample(1f)).IsEqualTo(20f);
        await Assert.That(track.Sample(-3f)).IsEqualTo(0f);
        await Assert.That(track.Sample(7f)).IsEqualTo(20f);
    }

    [Test]
    public async Task a_stepped_key_holds_its_value_until_the_next_one()
    {
        // Bit 0 set: key 0 steps, key 1 interpolates.
        var track = new FloatTrack([0f, 0.5f, 1f], [0f, 10f, 20f], [0b001]);

        await Assert.That(track.Sample(0.25f)).IsEqualTo(0f);
        await Assert.That(track.Sample(0.49f)).IsEqualTo(0f);
        await Assert.That(track.Sample(0.5f)).IsEqualTo(10f);
        await Assert.That(track.Sample(0.75f)).IsEqualTo(15f).Within(1e-5f);
    }

    [Test]
    public async Task an_empty_track_samples_to_identity_and_a_single_key_holds()
    {
        await Assert.That(new FloatTrack([], [], []).Sample(0.5f)).IsEqualTo(0f);
        await Assert.That(new QuaternionTrack([], [], []).Sample(0.5f)).IsEqualTo(Quaternion.Identity);
        await Assert.That(new Float3Track([], [], []).Sample(0.5f)).IsEqualTo(Vector3.Zero);
        await Assert.That(new FloatTrack([0.25f], [7f], [0]).Sample(0.9f)).IsEqualTo(7f);
    }

    [Test]
    public async Task a_quaternion_track_interpolates_by_normalized_lerp()
    {
        var track = new QuaternionTrack([0f, 1f], [Quaternion.Identity, TestRigs.QuarterTurnZ], [0]);

        var half = track.Sample(0.5f);

        await Assert.That(MathF.Abs(half.Length() - 1f)).IsLessThan(1e-6f);
        await Assert.That(TestRigs.AngleBetween(half, Quaternion.Normalize(Quaternion.Lerp(Quaternion.Identity, TestRigs.QuarterTurnZ, 0.5f)))).IsLessThan(1e-5f);
    }

    [Test]
    public async Task every_track_type_interpolates_and_reports_its_own_identity()
    {
        var float2 = new Float2Track([0f, 1f], [Vector2.Zero, new Vector2(10, 20)], [0]);
        var float3 = new Float3Track([0f, 1f], [Vector3.Zero, new Vector3(10, 20, 30)], [0]);
        var float4 = new Float4Track([0f, 1f], [Vector4.Zero, new Vector4(10, 20, 30, 40)], [0]);

        await Assert.That(float2.Sample(0.5f)).IsEqualTo(new Vector2(5, 10));
        await Assert.That(float3.Sample(0.5f)).IsEqualTo(new Vector3(5, 10, 15));
        await Assert.That(float4.Sample(0.5f)).IsEqualTo(new Vector4(5, 10, 15, 20));
        await Assert.That(float2.Identity).IsEqualTo(Vector2.Zero);
        await Assert.That(float4.Identity).IsEqualTo(Vector4.Zero);
        await Assert.That(new Float2Track([], [], []).Sample(0.5f)).IsEqualTo(Vector2.Zero);
        await Assert.That(new Float4Track([], [], []).Sample(0.5f)).IsEqualTo(Vector4.Zero);
    }

    [Test]
    public async Task each_track_type_recognizes_only_its_own_archive()
    {
        var floats = new FloatTrack([0f, 1f], [1f, 2f], [0]).Save();
        var float2 = new Float2Track([0f, 1f], [Vector2.Zero, Vector2.One], [0]).Save();
        var float3 = new Float3Track([0f, 1f], [Vector3.Zero, Vector3.One], [0]).Save();
        var float4 = new Float4Track([0f, 1f], [Vector4.Zero, Vector4.One], [0]).Save();
        var quaternions = new QuaternionTrack([0f, 1f], [Quaternion.Identity, TestRigs.QuarterTurnZ], [0]).Save();

        await Assert.That(Float2Track.IsTrack(float2)).IsTrue();
        await Assert.That(Float3Track.IsTrack(float3)).IsTrue();
        await Assert.That(Float4Track.IsTrack(float4)).IsTrue();
        await Assert.That(QuaternionTrack.IsTrack(quaternions)).IsTrue();
        await Assert.That(Float2Track.IsTrack(floats)).IsFalse();
        await Assert.That(Float4Track.IsTrack(float3)).IsFalse();
        await Assert.That(QuaternionTrack.IsTrack(float4)).IsFalse();
    }

    [Test]
    public async Task every_track_type_round_trips_through_its_archive()
    {
        var floats = new FloatTrack([0f, 1f], [1f, 2f], [0b01], "intensity");
        var float2 = new Float2Track([0f, 1f], [new Vector2(1, 2), new Vector2(3, 4)], [0], "uv");
        var float3 = new Float3Track([0f, 1f], [new Vector3(1, 2, 3), new Vector3(4, 5, 6)], [0], "position");
        var float4 = new Float4Track([0f, 1f], [new Vector4(1, 2, 3, 4), new Vector4(5, 6, 7, 8)], [0], "colour");
        var quaternions = new QuaternionTrack([0f, 1f], [Quaternion.Identity, TestRigs.QuarterTurnZ], [0], "spin");

        await Assert.That(FloatTrack.Load(floats.Save()).Save()).IsEquivalentTo(floats.Save());
        await Assert.That(FloatTrack.Load(floats.Save()).Name).IsEqualTo("intensity");
        await Assert.That(FloatTrack.Load(floats.Save()).IsStep(0)).IsTrue();
        await Assert.That(Float2Track.Load(float2.Save()).Values).IsEquivalentTo(float2.Values);
        await Assert.That(Float3Track.Load(float3.Save()).Values).IsEquivalentTo(float3.Values);
        await Assert.That(Float4Track.Load(float4.Save()).Values).IsEquivalentTo(float4.Values);
        await Assert.That(QuaternionTrack.Load(quaternions.Save()).Values).IsEquivalentTo(quaternions.Values);
        await Assert.That(Float3Track.Load(float3.Save()).Ratios).IsEquivalentTo(float3.Ratios);
    }

    [Test]
    public async Task a_track_archive_of_the_wrong_kind_is_refused_by_name()
    {
        var floats = new FloatTrack([0f, 1f], [1f, 2f], [0]).Save();

        await Assert.That(FloatTrack.IsTrack(floats)).IsTrue();
        await Assert.That(Float3Track.IsTrack(floats)).IsFalse();
        var error = await Assert.That(() => Float3Track.Load(floats)).Throws<InvalidDataException>();
        await Assert.That(error!.Message).Contains("ozz-float3_track");
    }

    [Test]
    public async Task a_track_with_mismatched_or_disordered_keys_is_refused()
    {
        await Assert.That(() => new FloatTrack([0f, 1f], [1f], [0])).Throws<ArgumentException>();
        await Assert.That(() => new FloatTrack([0f, 1f], [1f, 2f], [])).Throws<ArgumentException>();
        await Assert.That(() => new FloatTrack([1f, 0f], [1f, 2f], [0])).Throws<ArgumentException>();
        await Assert.That(() => new FloatTrack([0f, 2f], [1f, 2f], [0])).Throws<ArgumentException>();
    }

    [Test]
    public async Task sampling_a_track_allocates_nothing()
    {
        var track = Ramp();
        track.Sample(0.3f);

        var before = GC.GetAllocatedBytesForCurrentThread();
        var total = 0f;
        for (var i = 0; i < 1000; i++) total += track.Sample(i / 1000f);

        await Assert.That(GC.GetAllocatedBytesForCurrentThread() - before).IsEqualTo(0L);
        await Assert.That(total).IsGreaterThan(0f);
    }
}

/// <summary>Turning a curve into discrete events, forwards and backwards over loop boundaries.</summary>
public class TrackTriggeringTests
{
    /// <summary>Crosses zero once going up, at ratio 0.25; the wrap from the last key back to the first crosses it going down, at the loop point.</summary>
    private static FloatTrack Pulse() => new([0f, 0.5f], [-1f, 1f], [0]);

    [Test]
    public async Task a_forward_pass_reports_each_crossing_once_in_order()
    {
        var edges = TrackTriggering.Edges(Pulse(), 0f, 1f).ToArray();

        await Assert.That(edges.Length).IsEqualTo(2);
        // The wrap-around pair crosses downward at the loop point itself...
        await Assert.That(edges[0].Ratio).IsEqualTo(0f).Within(1e-6f);
        await Assert.That(edges[0].Rising).IsFalse();
        // ...then the curve rises through zero half-way between its two keys.
        await Assert.That(edges[1].Ratio).IsEqualTo(0.25f).Within(1e-6f);
        await Assert.That(edges[1].Rising).IsTrue();
    }

    [Test]
    public async Task a_backward_pass_reports_the_same_crossings_reversed_and_inverted()
    {
        // Scrubbing back over a clip must fire the same events at the same ratios, with rising
        // and falling swapped — otherwise a rewind would leave state inconsistent.
        var forward = TrackTriggering.Edges(Pulse(), 0f, 1f).ToArray();
        var backward = TrackTriggering.Edges(Pulse(), 1f, 0f).ToArray();

        await Assert.That(backward.Length).IsEqualTo(forward.Length);
        for (var i = 0; i < forward.Length; i++)
        {
            var mirrored = forward[^(i + 1)];
            await Assert.That(backward[i].Ratio).IsEqualTo(mirrored.Ratio).Within(1e-6f);
            await Assert.That(backward[i].Rising).IsEqualTo(!mirrored.Rising);
        }
    }

    [Test]
    public async Task a_query_spanning_a_loop_reports_both_loops_in_a_continuous_ratio_space()
    {
        var edges = TrackTriggering.Edges(Pulse(), 0.5f, 2.5f).ToArray();

        // One full loop's worth of crossings per loop, at ratios past 1 for the second.
        await Assert.That(edges.Select(e => e.Ratio).ToArray()).IsEquivalentTo(new[] { 1f, 1.25f, 2f, 2.25f });
    }

    [Test]
    public async Task the_threshold_moves_where_the_crossing_is_reported()
    {
        var edges = TrackTriggering.Edges(Pulse(), 0f, 1f, threshold: 0.5f).ToArray();

        // -1 → 1 over ratios 0 → 0.5 reaches 0.5 at three quarters of the way.
        await Assert.That(edges.Single(e => e.Rising).Ratio).IsEqualTo(0.375f).Within(1e-6f);
    }

    [Test]
    public async Task a_stepped_key_puts_the_crossing_at_the_later_key()
    {
        // A stepped key does not ramp, so the value jumps at the next key's ratio rather than
        // crossing somewhere between the two.
        var stepped = new FloatTrack([0f, 0.5f], [-1f, 1f], [0b01]);

        var rising = TrackTriggering.Edges(stepped, 0f, 1f).Single(e => e.Rising);

        await Assert.That(rising.Ratio).IsEqualTo(0.5f).Within(1e-6f);
    }

    [Test]
    public async Task a_track_that_never_crosses_and_an_empty_range_report_nothing()
    {
        var flat = new FloatTrack([0f, 1f], [5f, 7f], [0]);

        await Assert.That(TrackTriggering.Edges(flat, 0f, 1f)).IsEmpty();
        await Assert.That(TrackTriggering.Edges(Pulse(), 0.4f, 0.4f)).IsEmpty();
        await Assert.That(TrackTriggering.Edges(new FloatTrack([], [], []), 0f, 1f)).IsEmpty();
    }
}

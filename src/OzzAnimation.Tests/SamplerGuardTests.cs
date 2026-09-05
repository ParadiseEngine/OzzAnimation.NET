namespace OzzAnimation.Tests;

/// <summary>
/// The sampler's behaviour on clips that are structurally valid but that its cursor walk cannot
/// make sense of, and on clips with no i-frames at all. These paths are unreachable from any
/// archive a builder would write, which is exactly why they need pinning: a stream whose back-links
/// disagree with the cursors must raise, not loop forever or read a joint's pose from nowhere.
/// </summary>
public class SamplerGuardTests
{
    private static (SamplingContext Context, SoaTransforms Poses) Sampler(AnimationClip clip) =>
        (new SamplingContext(clip.TrackCount), new SoaTransforms(clip.TrackCount));

    [Test]
    public async Task a_clip_with_no_iframes_samples_by_walking_from_the_start()
    {
        // Nothing forces a builder to emit i-frames, and without them a seek rewinds to key 0 and
        // walks. The fixtures all have them, so this is the only cover for that branch.
        var clip = TestRigs.ClipOf(TestRigs.Stream());
        var (context, poses) = Sampler(clip);

        context.Sample(clip, 0.75f, poses);
        var forward = poses.ToArray();
        // Backwards, which re-enters the same rewind-to-the-start path with a negative delta.
        context.Sample(clip, 0.25f, poses);
        var backward = poses.ToArray();

        var fresh = new SoaTransforms(clip.TrackCount);
        new SamplingContext(clip.TrackCount).Sample(clip, 0.25f, fresh);
        await Assert.That(clip.Translations.IframeDesc).IsEmpty();
        await Assert.That(forward.Length).IsEqualTo(4);
        await Assert.That(backward).IsEquivalentTo(fresh.ToArray());
    }

    [Test]
    public async Task a_key_linked_to_a_key_no_track_holds_is_refused()
    {
        // Key 8's back-link points at itself, so the forward walk looks for a cursor holding key 8
        // and no track has one. Walking on would assign the key to an arbitrary track.
        var clip = TestRigs.ClipOf(TestRigs.Stream(
            ratios: [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
            previouses: [0, 0, 0, 0, 4, 4, 4, 4, 0, 4, 4, 4]));
        var (context, poses) = Sampler(clip);

        var error = await Assert.That(() => context.Sample(clip, 1f, poses)).Throws<InvalidDataException>();

        await Assert.That(error!.Message).Contains("which no track's cursor holds");
    }

    [Test]
    public async Task an_iframe_whose_cursors_do_not_cover_the_rewind_is_refused()
    {
        // A hand-built i-frame snapshot claiming cursors {0,1,2,3} while covering up to key 8: the
        // backward walk then looks for key 8 among them and finds nothing.
        // Group-varint: one prefix byte of four 1-byte lengths, then the four values.
        byte[] snapshot = [0b0000_0000, 0, 1, 2, 3];
        var clip = TestRigs.ClipOf(TestRigs.Stream(
            ratios: [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
            previouses: [0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4],
            iframeEntries: snapshot,
            iframeDesc: [0, 8],
            iframeInterval: 0.5f));
        var (context, poses) = Sampler(clip);

        var error = await Assert.That(() => context.Sample(clip, 0.5f, poses)).Throws<InvalidDataException>();

        await Assert.That(error!.Message).Contains("is not the cursor of any track");
    }

    [Test]
    public async Task a_seek_past_the_last_iframe_is_refused()
    {
        byte[] snapshot = [0b0000_0000, 4, 5, 6, 7];
        var clip = TestRigs.ClipOf(TestRigs.Stream(iframeEntries: snapshot, iframeDesc: [0, 6], iframeInterval: 0.5f));
        var (context, poses) = Sampler(clip);

        var error = await Assert.That(() => context.Sample(clip, 1f, poses)).Throws<InvalidDataException>();

        await Assert.That(error!.Message).Contains("no i-frame");
    }

    [Test]
    public async Task a_clip_with_no_tracks_samples_to_nothing()
    {
        var empty = new AnimationClip("empty", 1f, 0, [0f], new KeyframeStream([], [], [], [], [], 1f), new KeyframeStream([], [], [], [], [], 1f), new KeyframeStream([], [], [], [], [], 1f));
        var context = new SamplingContext(0);
        var poses = new SoaTransforms(0);

        context.Sample(empty, 0.5f, poses);

        await Assert.That(AnimationClip.PaddedTrackCount(empty.TrackCount)).IsEqualTo(0);
        await Assert.That(poses.ToArray()).IsEmpty();
    }
}

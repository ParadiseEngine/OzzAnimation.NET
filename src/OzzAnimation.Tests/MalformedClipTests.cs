namespace OzzAnimation.Tests;

/// <summary>
/// The validation a malformed clip must hit on the way in. This is not politeness: the sampler's
/// cursor walk indexes these streams without bounds checks, on the strength of these very checks
/// having run, so a stream that slipped through would be an out-of-bounds read rather than an
/// exception. Each case here is one such invariant.
/// </summary>
public class MalformedClipTests
{
    private const int Tracks = 4;

    private static readonly float[] Timepoints = [0f, 1f];

    /// <summary>Eight keys over four tracks: a first key at ratio 0 and a last at ratio 1 for each, which is the minimum a stream may hold.</summary>
    private static KeyframeStream Stream(byte[]? ratios = null, ushort[]? previouses = null, byte[]? iframeEntries = null, uint[]? iframeDesc = null, float iframeInterval = 1f) =>
        new(ratios ?? [0, 0, 0, 0, 1, 1, 1, 1],
            previouses ?? [0, 0, 0, 0, 4, 4, 4, 4],
            new ushort[8 * 3],
            iframeEntries ?? [],
            iframeDesc ?? [],
            iframeInterval);

    private static AnimationClip Clip(KeyframeStream translations) =>
        new("clip", 1f, Tracks, Timepoints, translations, Stream(), Stream());

    [Test]
    public async Task a_well_formed_clip_is_accepted()
    {
        var clip = Clip(Stream());

        await Assert.That(clip.TrackCount).IsEqualTo(Tracks);
        await Assert.That(clip.Translations.KeyCount).IsEqualTo(8);
    }

    [Test]
    public async Task a_stream_whose_first_key_is_not_at_the_clips_start_is_refused()
    {
        // Without this the backward walk has nothing to stop it and runs off the front of the stream.
        var error = await Assert.That(() => Clip(Stream(ratios: [1, 0, 0, 0, 1, 1, 1, 1]))).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("not at the clip's start");
    }

    [Test]
    public async Task a_key_naming_a_timepoint_past_the_end_is_refused()
    {
        var error = await Assert.That(() => Clip(Stream(ratios: [0, 0, 0, 0, 9, 1, 1, 1]))).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("names timepoint 9 of 2");
    }

    [Test]
    public async Task a_back_link_reaching_before_the_streams_start_is_refused()
    {
        var error = await Assert.That(() => Clip(Stream(previouses: [0, 0, 0, 0, 4, 4, 4, 99]))).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("keys back, before the stream's start");
    }

    [Test]
    public async Task an_iframe_table_pointing_outside_its_buffers_is_refused()
    {
        var pastEntries = await Assert.That(() => Clip(Stream(iframeEntries: [0], iframeDesc: [1000, 7]))).Throws<ArgumentException>();
        await Assert.That(pastEntries!.Message).Contains("starts at byte 1000 of 1");

        var pastKeys = await Assert.That(() => Clip(Stream(iframeEntries: [0], iframeDesc: [0, 1000]))).Throws<ArgumentException>();
        await Assert.That(pastKeys!.Message).Contains("covers key 1000 of 8");

        var oddTable = await Assert.That(() => Clip(Stream(iframeEntries: [0], iframeDesc: [0]))).Throws<ArgumentException>();
        await Assert.That(oddTable!.Message).Contains("odd length");
    }

    [Test]
    public async Task iframes_at_a_non_positive_interval_are_refused()
    {
        // The interval is a divisor when the sampler decides which i-frame to jump to.
        var error = await Assert.That(() => Clip(Stream(iframeEntries: [0], iframeDesc: [0, 7], iframeInterval: 0f))).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("interval of 0");
    }

    [Test]
    public async Task a_stream_too_short_for_its_tracks_is_refused()
    {
        var short_ = new KeyframeStream([0, 0], [0, 0], new ushort[6], [], [], 1f);

        var error = await Assert.That(() => Clip(short_)).Throws<ArgumentException>();

        await Assert.That(error!.Message).Contains("needs a first and a last key");
    }

    [Test]
    public async Task a_malformed_archive_is_refused_on_read_as_invalid_data()
    {
        // Load wraps the same validation, so a bad file is InvalidDataException rather than
        // ArgumentException — a caller reading untrusted archives catches one thing.
        var bytes = Clip(Stream()).Save();
        var corrupted = (byte[])bytes.Clone();
        // The first stream's first ratio byte sits just past the header, name and timepoints.
        var firstRatio = 1 + AnimationClip.Tag.Length + 1 + 4 + (13 * 4) + "clip".Length + (Timepoints.Length * 4);
        corrupted[firstRatio] = 1;

        var error = await Assert.That(() => AnimationClip.Load(corrupted)).Throws<InvalidDataException>();

        await Assert.That(error!.Message).Contains("not at the clip's start");
    }
}

namespace OzzAnimation;

/// <summary>A crossing of the threshold by a <see cref="FloatTrack"/>. ozz's <c>TrackTriggeringJob::Edge</c>.</summary>
/// <param name="Ratio">Where the crossing happens, in the same space the query was made in — so past 1 when the query spans a loop.</param>
/// <param name="Rising">True when the value crossed upward through the threshold.</param>
public readonly record struct TrackEdge(float Ratio, bool Rising);

/// <summary>
/// Finds where a <see cref="FloatTrack"/> crosses a threshold over an interval of playback, so a
/// curve can drive discrete events — a footstep, a muzzle flash — without the caller polling the
/// value every frame and missing a crossing between two of them. ozz's <c>TrackTriggeringJob</c>.
/// </summary>
/// <remarks>
/// The interval is in loop space rather than clamped to 0..1: a query from 0.9 to 1.3 covers the
/// end of one loop and the start of the next, and the ratios reported stay in that space. A query
/// whose <c>to</c> precedes its <c>from</c> walks backwards and reports
/// edges in reverse with rising and falling swapped, so scrubbing back over a clip fires the same
/// events at the same places. Edges are produced lazily — nothing is allocated per edge.
/// </remarks>
public static class TrackTriggering
{
    /// <summary>Every crossing of <paramref name="threshold"/> between <paramref name="from"/> and <paramref name="to"/>, in the order playback meets them.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="track"/> is null.</exception>
    public static IEnumerable<TrackEdge> Edges(FloatTrack track, float from, float to, float threshold = 0f)
    {
        ArgumentNullException.ThrowIfNull(track);
        if (from == to || track.KeyCount == 0) return [];
        return to > from ? Forward(track, from, to, threshold) : Backward(track, from, to, threshold);
    }

    private static IEnumerable<TrackEdge> Forward(FloatTrack track, float from, float to, float threshold)
    {
        var keyCount = track.KeyCount;
        var inner = 0;
        for (var outer = MathF.Floor(from); outer < to; outer += 1f)
        {
            while (inner < keyCount)
            {
                var previous = inner == 0 ? keyCount - 1 : inner - 1;
                if (Detect(track, previous, inner, forward: true, threshold, out var edge))
                {
                    var ratio = edge.Ratio + outer;
                    if (ratio >= from && (ratio < to || to >= 1f + outer))
                    {
                        inner++;
                        yield return edge with { Ratio = ratio };
                        continue;
                    }

                    // Past the end of the query: no later key can produce an edge inside it.
                    if (track.Ratios[inner] + outer >= to) break;
                }

                inner++;
            }

            inner = 0;
        }
    }

    private static IEnumerable<TrackEdge> Backward(FloatTrack track, float from, float to, float threshold)
    {
        var keyCount = track.KeyCount;
        var inner = keyCount - 1;
        for (var outer = MathF.Floor(from); outer + 1f > to; outer -= 1f)
        {
            while (inner >= 0)
            {
                var previous = inner == 0 ? keyCount - 1 : inner - 1;
                if (Detect(track, previous, inner, forward: false, threshold, out var edge))
                {
                    var ratio = edge.Ratio + outer;
                    if (ratio >= to && (ratio < from || from >= 1f + outer))
                    {
                        inner--;
                        yield return edge with { Ratio = ratio };
                        continue;
                    }
                }

                if (track.Ratios[inner] + outer <= to) break;
                inner--;
            }

            inner = keyCount - 1;
        }
    }

    /// <summary>Whether the value crosses the threshold between two consecutive keys, and where. A stepped key jumps at the later key's ratio; an interpolated one crosses where the lerp reaches the threshold.</summary>
    private static bool Detect(FloatTrack track, int previous, int next, bool forward, float threshold, out TrackEdge edge)
    {
        var values = track.Values;
        var before = values[previous];
        var after = values[next];

        bool rising;
        if (before <= threshold && after > threshold) rising = forward;
        else if (before > threshold && after <= threshold) rising = !forward;
        else
        {
            edge = default;
            return false;
        }

        float ratio;
        if (track.IsStep(previous))
        {
            ratio = track.Ratios[next];
        }
        else if (next == 0)
        {
            // The wrap-around pair: the crossing sits at the loop point itself.
            ratio = 0f;
        }
        else
        {
            var alpha = (threshold - before) / (after - before);
            var from = track.Ratios[previous];
            var to = track.Ratios[next];
            ratio = (to - from) * alpha + from;
        }

        edge = new TrackEdge(ratio, rising);
        return true;
    }
}

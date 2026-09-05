using System.Numerics;
using System.Text;

namespace OzzAnimation;

/// <summary>
/// A user channel: one value animated over the unit ratio of a clip, independent of the skeleton —
/// a curve driving an effect's intensity, a camera's field of view, an attachment's position.
/// ozz's <c>internal::Track&lt;T&gt;</c>, and the base of the five concrete track types.
/// </summary>
/// <remarks>
/// Keys are stored as parallel ratio and value arrays, plus one bit per key saying whether it
/// steps (holds its value to the next key) or interpolates. Sampling is a binary search; unlike
/// the skeleton sampler there is no cursor cache, because a track is one channel, not hundreds.
/// </remarks>
public abstract class Track<T> where T : struct
{
    /// <exception cref="ArgumentException">Ratios and values of different lengths, ratios out of order or outside 0..1, or a step bitset too short.</exception>
    protected Track(float[] ratios, T[] values, byte[] steps, string name)
    {
        ArgumentNullException.ThrowIfNull(ratios);
        ArgumentNullException.ThrowIfNull(values);
        ArgumentNullException.ThrowIfNull(steps);
        ArgumentNullException.ThrowIfNull(name);
        if (ratios.Length != values.Length) throw new ArgumentException($"{ratios.Length} ratios and {values.Length} values.", nameof(values));
        if (steps.Length < (ratios.Length + 7) / 8) throw new ArgumentException($"{steps.Length} step bytes for {ratios.Length} keys.", nameof(steps));
        for (var i = 0; i < ratios.Length; i++)
        {
            if (!(ratios[i] >= 0f && ratios[i] <= 1f)) throw new ArgumentException($"Key {i} sits at ratio {ratios[i]}, outside 0..1.", nameof(ratios));
            if (i > 0 && ratios[i] < ratios[i - 1]) throw new ArgumentException($"Key {i} sits at ratio {ratios[i]}, before key {i - 1} at {ratios[i - 1]}.", nameof(ratios));
        }

        Ratios = ratios;
        Values = values;
        Steps = steps;
        Name = name;
    }

    /// <summary>Each key's position as a ratio of the clip, 0..1, ascending.</summary>
    public float[] Ratios { get; }

    public T[] Values { get; }

    /// <summary>One bit per key, least significant first: set means the key holds its value until the next one instead of interpolating.</summary>
    public byte[] Steps { get; }

    public string Name { get; }

    public int KeyCount => Ratios.Length;

    /// <summary>Whether key <paramref name="key"/> holds its value to the next key.</summary>
    public bool IsStep(int key) => (Steps[key / 8] & (1 << (key & 7))) != 0;

    /// <summary>The value an empty track samples to, and the identity of the type: 0, or the identity rotation.</summary>
    public abstract T Identity { get; }

    /// <summary>How this type interpolates — a plain lerp, or a normalized lerp for rotations.</summary>
    protected abstract T Lerp(T a, T b, float alpha);

    /// <summary>
    /// Samples at <paramref name="ratio"/> of the clip. ozz's <c>TrackSamplingJob</c>: an empty
    /// track gives <see cref="Identity"/>, a ratio outside 0..1 clamps to the end keys, and a
    /// stepped key holds its value. Allocation-free.
    /// </summary>
    public T Sample(float ratio)
    {
        if (KeyCount == 0) return Identity;
        if (KeyCount == 1 || ratio <= 0f) return Values[0];
        if (ratio >= 1f) return Values[^1];

        var next = UpperBound(Ratios, ratio);
        if (next == KeyCount) return Values[^1];
        var previous = next - 1;
        if (IsStep(previous)) return Values[previous];

        var from = Ratios[previous];
        var to = Ratios[next];
        if (to == from) return Values[previous];
        return Lerp(Values[previous], Values[next], (ratio - from) / (to - from));
    }

    /// <summary>Index of the first ratio strictly greater than <paramref name="ratio"/>, matching C++'s <c>std::upper_bound</c>.</summary>
    private static int UpperBound(ReadOnlySpan<float> ratios, float ratio)
    {
        int low = 0, high = ratios.Length;
        while (low < high)
        {
            var middle = (int)(((uint)low + (uint)high) >> 1);
            if (ratios[middle] <= ratio) low = middle + 1;
            else high = middle;
        }

        return low;
    }

    internal static (float[] Ratios, TValue[] Values, byte[] Steps, string Name) ReadArchive<TValue>(ReadOnlySpan<byte> bytes, string tag, uint version, int floatsPerValue, Func<ReadOnlySpan<float>, TValue> read)
    {
        var reader = OzzReader.Open(bytes, tag, version);
        var keyCount = reader.ReadInt32();
        var nameLength = reader.ReadInt32();
        if (keyCount < 0 || nameLength < 0) throw new InvalidDataException($"The ozz '{tag}' archive carries a negative count.");

        var ratios = reader.ReadSingles(keyCount);
        var floats = reader.ReadSingles(keyCount * floatsPerValue);
        var values = new TValue[keyCount];
        for (var i = 0; i < keyCount; i++) values[i] = read(floats.AsSpan(i * floatsPerValue, floatsPerValue));
        var steps = reader.ReadBytes((keyCount + 7) / 8).ToArray();
        var name = Encoding.UTF8.GetString(reader.ReadBytes(nameLength));
        reader.ExpectEnd(tag);
        return (ratios, values, steps, name);
    }

    internal byte[] WriteArchive(string tag, uint version, int floatsPerValue, Action<T, Span<float>> write)
    {
        var writer = new OzzWriter(tag, version);
        var name = Encoding.UTF8.GetBytes(Name);
        writer.Write(KeyCount);
        writer.Write(name.Length);
        writer.Write(Ratios);
        var floats = new float[KeyCount * floatsPerValue];
        for (var i = 0; i < KeyCount; i++) write(Values[i], floats.AsSpan(i * floatsPerValue, floatsPerValue));
        writer.Write(floats);
        writer.Write(Steps.AsSpan(0, (KeyCount + 7) / 8));
        writer.Write(name);
        return writer.ToArray();
    }
}

/// <summary>A track of scalars — ozz's <c>FloatTrack</c>, and the only kind <see cref="TrackTriggering"/> reads.</summary>
public sealed class FloatTrack(float[] ratios, float[] values, byte[] steps, string name = "") : Track<float>(ratios, values, steps, name)
{
    public const string Tag = "ozz-float_track";

    public const uint Version = 1;

    public override float Identity => 0f;

    protected override float Lerp(float a, float b, float alpha) => (b - a) * alpha + a;

    public static bool IsTrack(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    /// <exception cref="InvalidDataException">Not a version-1 ozz float track archive.</exception>
    public static FloatTrack Load(ReadOnlySpan<byte> bytes)
    {
        var (ratios, values, steps, name) = ReadArchive(bytes, Tag, Version, 1, f => f[0]);
        return new FloatTrack(ratios, values, steps, name);
    }

    public byte[] Save() => WriteArchive(Tag, Version, 1, (v, f) => f[0] = v);
}

/// <summary>A track of 2D vectors — ozz's <c>Float2Track</c>.</summary>
public sealed class Float2Track(float[] ratios, Vector2[] values, byte[] steps, string name = "") : Track<Vector2>(ratios, values, steps, name)
{
    public const string Tag = "ozz-float2_track";

    public const uint Version = 1;

    public override Vector2 Identity => Vector2.Zero;

    protected override Vector2 Lerp(Vector2 a, Vector2 b, float alpha) => Vector2.Lerp(a, b, alpha);

    public static bool IsTrack(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    public static Float2Track Load(ReadOnlySpan<byte> bytes)
    {
        var (ratios, values, steps, name) = ReadArchive(bytes, Tag, Version, 2, f => new Vector2(f[0], f[1]));
        return new Float2Track(ratios, values, steps, name);
    }

    public byte[] Save() => WriteArchive(Tag, Version, 2, (v, f) => { f[0] = v.X; f[1] = v.Y; });
}

/// <summary>A track of 3D vectors — ozz's <c>Float3Track</c>.</summary>
public sealed class Float3Track(float[] ratios, Vector3[] values, byte[] steps, string name = "") : Track<Vector3>(ratios, values, steps, name)
{
    public const string Tag = "ozz-float3_track";

    public const uint Version = 1;

    public override Vector3 Identity => Vector3.Zero;

    protected override Vector3 Lerp(Vector3 a, Vector3 b, float alpha) => Vector3.Lerp(a, b, alpha);

    public static bool IsTrack(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    public static Float3Track Load(ReadOnlySpan<byte> bytes)
    {
        var (ratios, values, steps, name) = ReadArchive(bytes, Tag, Version, 3, f => new Vector3(f[0], f[1], f[2]));
        return new Float3Track(ratios, values, steps, name);
    }

    public byte[] Save() => WriteArchive(Tag, Version, 3, (v, f) => { f[0] = v.X; f[1] = v.Y; f[2] = v.Z; });
}

/// <summary>A track of 4D vectors — ozz's <c>Float4Track</c>.</summary>
public sealed class Float4Track(float[] ratios, Vector4[] values, byte[] steps, string name = "") : Track<Vector4>(ratios, values, steps, name)
{
    public const string Tag = "ozz-float4_track";

    public const uint Version = 1;

    public override Vector4 Identity => Vector4.Zero;

    protected override Vector4 Lerp(Vector4 a, Vector4 b, float alpha) => Vector4.Lerp(a, b, alpha);

    public static bool IsTrack(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    public static Float4Track Load(ReadOnlySpan<byte> bytes)
    {
        var (ratios, values, steps, name) = ReadArchive(bytes, Tag, Version, 4, f => new Vector4(f[0], f[1], f[2], f[3]));
        return new Float4Track(ratios, values, steps, name);
    }

    public byte[] Save() => WriteArchive(Tag, Version, 4, (v, f) => { f[0] = v.X; f[1] = v.Y; f[2] = v.Z; f[3] = v.W; });
}

/// <summary>A track of rotations — ozz's <c>QuaternionTrack</c>; interpolates by normalized lerp, not a plain one.</summary>
public sealed class QuaternionTrack(float[] ratios, Quaternion[] values, byte[] steps, string name = "") : Track<Quaternion>(ratios, values, steps, name)
{
    public const string Tag = "ozz-quat_track";

    public const uint Version = 1;

    public override Quaternion Identity => Quaternion.Identity;

    /// <summary>ozz's <c>NLerp</c>: a lerp of the components, then a normalize. The short arc is not chosen here — the builder already flipped keys onto one hemisphere.</summary>
    protected override Quaternion Lerp(Quaternion a, Quaternion b, float alpha) =>
        Quaternion.Normalize(new Quaternion(
            (b.X - a.X) * alpha + a.X,
            (b.Y - a.Y) * alpha + a.Y,
            (b.Z - a.Z) * alpha + a.Z,
            (b.W - a.W) * alpha + a.W));

    public static bool IsTrack(ReadOnlySpan<byte> bytes) => OzzReader.HasTag(bytes, Tag);

    public static QuaternionTrack Load(ReadOnlySpan<byte> bytes)
    {
        var (ratios, values, steps, name) = ReadArchive(bytes, Tag, Version, 4, f => new Quaternion(f[0], f[1], f[2], f[3]));
        return new QuaternionTrack(ratios, values, steps, name);
    }

    public byte[] Save() => WriteArchive(Tag, Version, 4, (v, f) => { f[0] = v.X; f[1] = v.Y; f[2] = v.Z; f[3] = v.W; });
}

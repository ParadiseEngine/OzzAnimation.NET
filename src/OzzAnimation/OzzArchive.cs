using System.Buffers.Binary;
using System.Runtime.InteropServices;
using System.Text;

namespace OzzAnimation;

/// <summary>
/// The byte layout ozz-animation archives use: one endianness byte, a null-terminated type tag, a
/// uint32 version, then the payload. Little-endian only — the archives this engine cooks and reads
/// are its own, and a big-endian file is refused rather than byte-swapped.
/// </summary>
/// <remarks>
/// Cross-language contract with ozz-animation 0.17 (<c>ozz/base/io/archive.h</c>): a file written
/// by <c>gltf2ozz</c> loads here, and a file written here loads in ozz's C++ runtime. Keep the
/// tags and versions in <see cref="Skeleton"/> and <see cref="AnimationClip"/> pinned to that release.
/// </remarks>
internal ref struct OzzReader
{
    private const byte LittleEndian = 1;

    private readonly ReadOnlySpan<byte> _bytes;
    private int _at;

    private OzzReader(ReadOnlySpan<byte> bytes)
    {
        _bytes = bytes;
        _at = 0;
    }

    /// <summary>Opens the archive and checks the tag and version; the returned reader sits at the payload.</summary>
    /// <exception cref="InvalidDataException">Not an ozz archive of that tag, big-endian, or another version.</exception>
    public static OzzReader Open(ReadOnlySpan<byte> bytes, string tag, uint version)
    {
        var reader = new OzzReader(bytes);
        var tagBytes = Encoding.ASCII.GetBytes(tag + '\0');
        if (bytes.Length < 1 + tagBytes.Length + 4 || !bytes.Slice(1, tagBytes.Length).SequenceEqual(tagBytes))
        {
            throw new InvalidDataException($"Not an ozz '{tag}' archive.");
        }

        if (bytes[0] != LittleEndian) throw new InvalidDataException($"The ozz '{tag}' archive is big-endian; only little-endian archives are read.");
        reader._at = 1 + tagBytes.Length;
        var found = reader.ReadUInt32();
        if (found != version) throw new InvalidDataException($"The ozz '{tag}' archive is version {found}; this build reads version {version}.");
        return reader;
    }

    /// <summary>Whether the bytes begin as an archive of that tag, without reading further.</summary>
    public static bool HasTag(ReadOnlySpan<byte> bytes, string tag)
    {
        var tagBytes = Encoding.ASCII.GetBytes(tag + '\0');
        return bytes.Length >= 1 + tagBytes.Length && bytes.Slice(1, tagBytes.Length).SequenceEqual(tagBytes);
    }

    public readonly int Remaining => _bytes.Length - _at;

    public float ReadSingle() => BitConverter.Int32BitsToSingle((int)ReadUInt32());

    public int ReadInt32() => (int)ReadUInt32();

    public uint ReadUInt32()
    {
        var value = BinaryPrimitives.ReadUInt32LittleEndian(Take(4));
        return value;
    }

    public ushort ReadUInt16() => BinaryPrimitives.ReadUInt16LittleEndian(Take(2));

    public short ReadInt16() => BinaryPrimitives.ReadInt16LittleEndian(Take(2));

    public ReadOnlySpan<byte> ReadBytes(int count) => Take(count);

    public float[] ReadSingles(int count)
    {
        var values = new float[count];
        ReadSingles(values);
        return values;
    }

    /// <summary>Reads straight into <paramref name="destination"/>. Archives are little-endian, so on a little-endian host this is one vectorized copy rather than a loop.</summary>
    public void ReadSingles(Span<float> destination)
    {
        var bytes = Take(destination.Length * sizeof(float));
        if (BitConverter.IsLittleEndian)
        {
            MemoryMarshal.Cast<byte, float>(bytes).CopyTo(destination);
            return;
        }

        // Unreachable on every platform .NET currently supports, and so uncovered by the tests.
        // It stays because deleting it would not make the code faster, only wrong on a host where
        // the reinterpret above does not hold.
        for (var i = 0; i < destination.Length; i++) destination[i] = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(i * sizeof(float), sizeof(float))));
    }

    public ushort[] ReadUInt16s(int count)
    {
        var values = new ushort[count];
        for (var i = 0; i < count; i++) values[i] = ReadUInt16();
        return values;
    }

    public uint[] ReadUInt32s(int count)
    {
        var values = new uint[count];
        for (var i = 0; i < count; i++) values[i] = ReadUInt32();
        return values;
    }

    public void ExpectEnd(string what)
    {
        if (Remaining != 0) throw new InvalidDataException($"The ozz {what} archive has {Remaining} bytes past its payload.");
    }

    private ReadOnlySpan<byte> Take(int count)
    {
        if (count < 0 || _at + count > _bytes.Length) throw new InvalidDataException("The ozz archive ends inside a field.");
        var slice = _bytes.Slice(_at, count);
        _at += count;
        return slice;
    }
}

/// <summary>Writes the archive layout <see cref="OzzReader"/> reads; little-endian, one type per file.</summary>
internal sealed class OzzWriter
{
    private const byte LittleEndian = 1;

    private readonly MemoryStream _stream = new();

    public OzzWriter(string tag, uint version)
    {
        _stream.WriteByte(LittleEndian);
        var tagBytes = Encoding.ASCII.GetBytes(tag + '\0');
        _stream.Write(tagBytes);
        Write(version);
    }

    public void Write(uint value)
    {
        Span<byte> buffer = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32LittleEndian(buffer, value);
        _stream.Write(buffer);
    }

    public void Write(int value) => Write((uint)value);

    public void Write(float value) => Write((uint)BitConverter.SingleToInt32Bits(value));

    public void Write(ushort value)
    {
        Span<byte> buffer = stackalloc byte[2];
        BinaryPrimitives.WriteUInt16LittleEndian(buffer, value);
        _stream.Write(buffer);
    }

    public void Write(short value) => Write((ushort)value);

    public void Write(ReadOnlySpan<byte> bytes) => _stream.Write(bytes);

    public void Write(ReadOnlySpan<float> values)
    {
        if (BitConverter.IsLittleEndian)
        {
            _stream.Write(MemoryMarshal.AsBytes(values));
            return;
        }

        // As in ReadSingles: a big-endian host cannot reinterpret, and no test can reach this.
        foreach (var value in values) Write(value);
    }

    public void Write(ReadOnlySpan<ushort> values)
    {
        foreach (var value in values) Write(value);
    }

    public void Write(ReadOnlySpan<short> values)
    {
        foreach (var value in values) Write(value);
    }

    public void Write(ReadOnlySpan<uint> values)
    {
        foreach (var value in values) Write(value);
    }

    public byte[] ToArray() => _stream.ToArray();
}

/// <summary>
/// ozz's group-varint coding of uint32 quadruples (<c>ozz/base/encode/group_varint.h</c>): one
/// prefix byte holding four 2-bit lengths, then 1–4 bytes per value. Only decoding is here — an
/// i-frame is read by the sampler and written by the offline builder, which this runtime is not.
/// </summary>
internal static class GroupVarint
{
    /// <summary>Decodes <paramref name="output"/>.Length values (a multiple of four) starting at <paramref name="offset"/>.</summary>
    public static void Decode(ReadOnlySpan<byte> encoded, int offset, Span<uint> output)
    {
        if (output.Length % 4 != 0) throw new ArgumentException("A group-varint stream holds a multiple of four values.", nameof(output));
        var at = offset;
        for (var i = 0; i < output.Length; i += 4)
        {
            if (at >= encoded.Length) throw new InvalidDataException("The group-varint stream ends inside a group.");
            var prefix = encoded[at++];
            for (var k = 0; k < 4; k++)
            {
                var length = ((prefix >> (2 * k)) & 0x3) + 1;
                if (at + length > encoded.Length) throw new InvalidDataException("The group-varint stream ends inside a value.");
                uint value = 0;
                for (var b = 0; b < length; b++) value |= (uint)encoded[at + b] << (8 * b);
                output[i + k] = value;
                at += length;
            }
        }
    }

}

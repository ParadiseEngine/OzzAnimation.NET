using System.Runtime.InteropServices;

namespace OzzAnimation.Benchmarks;

/// <summary>
/// ozz-animation's own C++ runtime, behind the shim in <c>bench/native</c>. There is no native
/// code in this repository, so the library is found only through <c>OZZ_NATIVE_LIB</c> — the path
/// to the built <c>libozz_shim</c> — and the native rows are simply absent from a run without it.
/// </summary>
internal static unsafe partial class NativeOzz
{
    public const string EnvironmentVariable = "OZZ_NATIVE_LIB";

    private const string Library = "ozz_shim";

    public static bool IsAvailable { get; } = Probe();

    [LibraryImport(Library, EntryPoint = "Ozz_LoadSkeleton")] public static partial nint LoadSkeleton(byte* bytes, nuint length);
    [LibraryImport(Library, EntryPoint = "Ozz_LoadAnimation")] public static partial nint LoadAnimation(byte* bytes, nuint length);
    [LibraryImport(Library, EntryPoint = "Ozz_FreeSkeleton")] public static partial void FreeSkeleton(nint skeleton);
    [LibraryImport(Library, EntryPoint = "Ozz_FreeAnimation")] public static partial void FreeAnimation(nint animation);
    [LibraryImport(Library, EntryPoint = "Ozz_JointCount")] public static partial int JointCount(nint skeleton);
    [LibraryImport(Library, EntryPoint = "Ozz_TrackCount")] public static partial int TrackCount(nint animation);
    [LibraryImport(Library, EntryPoint = "Ozz_CreateContext")] public static partial nint CreateContext(nint skeleton, int maxTracks);
    [LibraryImport(Library, EntryPoint = "Ozz_FreeContext")] public static partial void FreeContext(nint context);
    [LibraryImport(Library, EntryPoint = "Ozz_SampleModelSpace")] public static partial int SampleModelSpace(nint skeleton, nint animation, nint context, float ratio, float* models);

    private static bool Probe()
    {
        var path = Environment.GetEnvironmentVariable(EnvironmentVariable);
        if (string.IsNullOrEmpty(path) || !File.Exists(path)) return false;
        NativeLibrary.SetDllImportResolver(typeof(NativeOzz).Assembly, (name, _, _) => name == Library ? NativeLibrary.Load(path) : nint.Zero);
        try
        {
            return NativeLibrary.TryLoad(path, out _);
        }
        catch (Exception exception) when (exception is DllNotFoundException or BadImageFormatException)
        {
            return false;
        }
    }
}

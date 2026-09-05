using System.Numerics;

using BenchmarkDotNet.Attributes;

namespace OzzAnimation.Benchmarks;

/// <summary>
/// One frame of one character: sample a clip and walk the hierarchy to model space, this library
/// against ozz-animation's own C++ runtime when <c>OZZ_NATIVE_LIB</c> names the shim from
/// <c>bench/native</c>. Two access patterns, because they exercise different machinery:
/// <c>Advance</c> steps 1/100 of the clip per frame — playback, where the per-track cursor cache
/// pays — and <c>Seek</c> jumps to a random ratio every frame — scrubbing, where the i-frames pay.
/// </summary>
/// <remarks>
/// Both sides are allocation-free per frame: the shim keeps its scratch buffers in its context for
/// the same reason this library sizes everything up front, so what is timed is the sampling and the
/// hierarchy walk rather than an allocator.
/// </remarks>
[MemoryDiagnoser]
[SimpleJob(warmupCount: 5, iterationCount: 15)]
public unsafe class AnimationBenchmarks
{
    private Skeleton _skeleton = null!;
    private AnimationClip _clip = null!;
    private SamplingContext _context = null!;
    private SoaTransforms _poses = null!;
    private Matrix4x4[] _models = null!;
    private float[] _seeks = null!;
    private int _frame;

    private nint _nativeSkeleton;
    private nint _nativeClip;
    private nint _nativeContext;
    private float[] _nativeModels = null!;

    [GlobalSetup]
    public void Setup()
    {
        var skeletonArchive = Fixtures.Read("bench-skeleton.ozz");
        var clipArchive = Fixtures.Read("bench-animation.ozz");

        _skeleton = Skeleton.Load(skeletonArchive);
        _clip = AnimationClip.Load(clipArchive);
        _context = new SamplingContext(_clip.TrackCount);
        _poses = new SoaTransforms(_clip.TrackCount);
        _models = new Matrix4x4[_skeleton.JointCount];

        var random = new Random(1);
        _seeks = Enumerable.Range(0, 1024).Select(_ => random.NextSingle()).ToArray();

        if (!NativeOzz.IsAvailable) return;
        fixed (byte* s = skeletonArchive) _nativeSkeleton = NativeOzz.LoadSkeleton(s, (nuint)skeletonArchive.Length);
        fixed (byte* a = clipArchive) _nativeClip = NativeOzz.LoadAnimation(a, (nuint)clipArchive.Length);
        _nativeContext = NativeOzz.CreateContext(_nativeSkeleton, NativeOzz.TrackCount(_nativeClip));
        _nativeModels = new float[_skeleton.JointCount * 16];
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        if (_nativeContext != 0) NativeOzz.FreeContext(_nativeContext);
        if (_nativeClip != 0) NativeOzz.FreeAnimation(_nativeClip);
        if (_nativeSkeleton != 0) NativeOzz.FreeSkeleton(_nativeSkeleton);
    }

    private float Advance() => (_frame++ % 100) / 100f;

    private float Seek() => _seeks[_frame++ & 1023];

    [Benchmark(Baseline = true), BenchmarkCategory("Advance")]
    public void Managed_Advance()
    {
        _context.Sample(_clip, Advance(), _poses);
        LocalToModel.Compute(_skeleton, _poses, _models);
    }

    [Benchmark, BenchmarkCategory("Advance")]
    public void NativeOzz_Advance()
    {
        if (!NativeOzz.IsAvailable) throw new NotSupportedException($"Set {NativeOzz.EnvironmentVariable} to the shim library to measure native ozz.");
        fixed (float* m = _nativeModels) NativeOzz.SampleModelSpace(_nativeSkeleton, _nativeClip, _nativeContext, Advance(), m);
    }

    [Benchmark, BenchmarkCategory("Seek")]
    public void Managed_Seek()
    {
        _context.Sample(_clip, Seek(), _poses);
        LocalToModel.Compute(_skeleton, _poses, _models);
    }

    [Benchmark, BenchmarkCategory("Seek")]
    public void NativeOzz_Seek()
    {
        if (!NativeOzz.IsAvailable) throw new NotSupportedException($"Set {NativeOzz.EnvironmentVariable} to the shim library to measure native ozz.");
        fixed (float* m = _nativeModels) NativeOzz.SampleModelSpace(_nativeSkeleton, _nativeClip, _nativeContext, Seek(), m);
    }
}

/// <summary>The other runtime jobs, measured on the same 64-joint rig. ozz's C++ is not wired up for these — the shim covers sampling only — so these are absolute numbers, not a comparison.</summary>
[MemoryDiagnoser]
[SimpleJob(warmupCount: 5, iterationCount: 15)]
public class JobBenchmarks
{
    private Skeleton _skeleton = null!;
    private BlendingJob _blending = null!;
    private BlendingLayer[] _layers = null!;
    private BlendingLayer[] _additive = null!;
    private SoaTransforms _output = null!;
    private Matrix4x4[] _models = null!;
    private IKTwoBoneJob _twoBone;
    private IKAimJob _aim;

    [GlobalSetup]
    public void Setup()
    {
        _skeleton = Skeleton.Load(Fixtures.Read("bench-skeleton.ozz"));
        var clip = AnimationClip.Load(Fixtures.Read("bench-animation.ozz"));

        var first = new SoaTransforms(_skeleton.JointCount);
        var second = new SoaTransforms(_skeleton.JointCount);
        new SamplingContext(clip.TrackCount).Sample(clip, 0.2f, first);
        new SamplingContext(clip.TrackCount).Sample(clip, 0.7f, second);

        _blending = new BlendingJob(_skeleton.SoaJointCount);
        _layers = [new() { Weight = 0.6f, Transform = first }, new() { Weight = 0.4f, Transform = second }];
        _additive = [new() { Weight = 0.5f, Transform = second }];
        _output = new SoaTransforms(_skeleton.JointCount);

        _models = new Matrix4x4[_skeleton.JointCount];
        LocalToModel.Compute(_skeleton, _skeleton.RestPose, _models);
        _twoBone = new IKTwoBoneJob
        {
            Target = new Vector3(0.3f, 0.5f, 0.1f),
            MidAxis = Vector3.UnitZ,
            PoleVector = Vector3.UnitY,
            StartJoint = _models[0],
            MidJoint = _models[1],
            EndJoint = _models[4],
        };
        _aim = new IKAimJob { Target = new Vector3(1, 2, 3), Forward = Vector3.UnitX, Up = Vector3.UnitY, PoleVector = Vector3.UnitY, Joint = _models[1] };
    }

    [Benchmark]
    public void Blend_TwoLayers() => _blending.Run(_layers, [], _skeleton.RestPose, _output);

    [Benchmark]
    public void Blend_TwoLayersPlusAdditive() => _blending.Run(_layers, _additive, _skeleton.RestPose, _output);

    [Benchmark]
    public void LocalToModel_WholeSkeleton() => LocalToModel.Compute(_skeleton, _skeleton.RestPose, _models);

    [Benchmark]
    public void IK_TwoBone() => _twoBone.Run(out _, out _, out _);

    [Benchmark]
    public void IK_Aim() => _aim.Run(out _, out _);
}

internal static class Fixtures
{
    public static byte[] Read(string name)
    {
        using var stream = typeof(Fixtures).Assembly.GetManifestResourceStream($"bench.{name}")
            ?? throw new FileNotFoundException($"Embedded fixture '{name}' is missing.");
        using var memory = new MemoryStream();
        stream.CopyTo(memory);
        return memory.ToArray();
    }
}

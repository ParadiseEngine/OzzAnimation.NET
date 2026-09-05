# OzzAnimation.NET

A managed C# port of the **[ozz-animation](https://github.com/guillaumeblanc/ozz-animation) runtime** —
Guillaume Blanc's open-source skeletal animation library. Same data layout, same compression, same
archive format, byte for byte; no native code, no P/Invoke, no marshalling.

```csharp
var skeleton = Skeleton.Load(File.ReadAllBytes("skeleton.ozz"));
var clip     = AnimationClip.Load(File.ReadAllBytes("walk.ozz"));

var context = new SamplingContext(clip.TrackCount);
var locals  = new SoaTransforms(skeleton.JointCount);
var models  = new Matrix4x4[skeleton.JointCount];

// per frame — allocates nothing
context.Sample(clip, ratio, locals);
LocalToModel.Compute(skeleton, locals, models);
```

## Thanks

This library exists because of **ozz-animation** and its author, **Guillaume Blanc**. The runtime
data layout, the archive format, the keyframe compression, the cursor-cache sampler, the i-frame
seeking, the blending model, the analytic IK solvers — all of it is *his* design. This repository
reimplements them in C#; it invents nothing. If you find this useful, the credit belongs upstream:

> **ozz-animation** — https://github.com/guillaumeblanc/ozz-animation
> © Guillaume Blanc, MIT licensed.

Thank you for building it, and for releasing it under a licence that made this port possible.

## Compatibility

Byte-compatible with **ozz-animation 0.17**:

| Archive | Tag | Version |
|---|---|---|
| Skeleton | `ozz-skeleton` | 2 |
| Animation | `ozz-animation` | 7 |
| Tracks | `ozz-float_track` … `ozz-quat_track` | 1 |

- A file written by `gltf2ozz` **loads here**.
- A file written here **loads in ozz's C++ runtime**.

This is enforced, not asserted. The test suite embeds archives produced by ozz-animation 0.17's own
C++ builders; loading one and saving it again reproduces the file **byte for byte**, which pins
every header count, the SoA rest-pose groups, the keyframe streams and the group-varint i-frames.
A separate test samples a clip baked from a closed-form curve and checks the decoded poses against
that curve, so the whole decode path is measured against ground truth rather than against itself.

Little-endian only. A big-endian archive is refused rather than byte-swapped.

## What is here

This is the **runtime** half of ozz. ozz ships its authoring code as a separate `ozz_animation_offline`
library, and this repository draws the same line — see [Not ported](#not-ported).

| This library | ozz |
|---|---|
| `Skeleton` | `ozz::animation::Skeleton` |
| `AnimationClip` | `ozz::animation::Animation` |
| `SamplingContext.Sample` | `SamplingJob` + its `Context` |
| `SoaTransforms` | `span<ozz::math::SoaTransform>` |
| `LocalToModel.Compute` | `LocalToModelJob` (including `from` / `to` / `from_excluded`) |
| `BlendingJob` | `BlendingJob` (weighted, per-joint, additive and subtractive layers) |
| `IKTwoBoneJob` | `IKTwoBoneJob` |
| `IKAimJob` | `IKAimJob` |
| `FloatTrack` … `QuaternionTrack` | `FloatTrack` … `QuaternionTrack` |
| `Track<T>.Sample` | `TrackSamplingJob` |
| `TrackTriggering.Edges` | `TrackTriggeringJob` and its iterator |
| `MotionBlending.Blend` | `MotionBlendingJob` |
| `SkeletonUtils` | `skeleton_utils.h` |
| `AnimationUtils` | `animation_utils.h` |

### Two deliberate differences

**Row-vector matrices.** ozz composes `model[i] = parent × local[i]` in a column-vector convention.
`System.Numerics.Matrix4x4` is row-vector, so this library composes `model[i] = local[i] × parent`
and stores the transpose. The transforms are the same; only the convention differs, and it is the
one every other .NET library expects.

**Exact instead of estimated math.** Where ozz uses estimated reciprocal and inverse-square-root
instructions (`RcpEst`, `RSqrtEstNR`, `NormalizeEst`), this uses exact division and square root.
Poses differ from native ozz around the seventh decimal, never structurally.

Jobs are C# methods rather than structs with `Validate()`/`Run()` fields, but every ozz parameter is
present, under its ozz name.

### Not ported

- **The offline module** — `RawSkeleton`/`RawAnimation`, `SkeletonBuilder`, `AnimationBuilder`,
  `AnimationOptimizer`, `TrackBuilder`, `AdditiveAnimationBuilder`, `MotionExtractor`. Build your
  archives with ozz's own tools (`gltf2ozz`) and load them here.
- **The glTF importer** and the Fbx toolchain.
- `ozz::geometry` skinning jobs, and the sample framework.

Nothing above changes the archive format, so any of it could be added later without breaking files
already written.

### One behavioural note

A track with **no keys** samples to **identity**, not to the joint's rest pose — same as ozz. If you
want unanimated joints to hold their rest transform, that is the exporter's job, and `gltf2ozz`
does it.

## Performance

Sampling is a port of ozz's SIMD `SamplingJob`: keys are decoded and interpolated four tracks at a
time in `Vector128<float>` lanes, straight into a structure-of-arrays pose set with no transpose.
The cursor walk stays scalar, as in ozz.

One frame of one 64-joint character — sample plus local-to-model — on an Apple M3 Max, .NET 10:

| | This library | ozz 0.17 (C++) |
|---|---|---|
| Playback (steps 1/100 of the clip per frame) | 1.128 µs | 1.197 µs |
| Scrubbing (jumps to a random ratio per frame) | 3.607 µs | **3.201 µs** |
| Allocated per frame | 0 B | 0 B |

Playback is a dead heat; this run has it slightly ahead, but run-to-run drift on this machine is a
few percent either way, so read the two as equal rather than as a win. Scrubbing is about 12%
behind, and that gap is the deliberate trade named above: ozz seeks with estimated reciprocal and
inverse-square-root instructions, this library uses exact ones.

Splitting the frame shows where the work is, and where it is worth looking:

| | Sampling | Local-to-model | Frame |
|---|---|---|---|
| Playback | 690 ns | 438 ns | 1128 ns |
| Scrubbing | 3175 ns | 432 ns | 3607 ns |

Scrubbing is 88% sampler. Both runtimes walk the same keys from the same i-frames — the cost is
per-key, so the lever that actually moves it is i-frame density at build time (`gltf2ozz`'s
sampling rate), not anything the runtime can do.

The other runtime jobs on the same rig, for scale (ozz's C++ is not wired up for these — the shim
covers sampling only — so these are absolute numbers, not a comparison):

| Job | Mean |
|---|---|
| `BlendingJob`, two layers | 175 ns |
| `BlendingJob`, two layers plus an additive one | 397 ns |
| `LocalToModel`, whole skeleton | 435 ns |
| `IKTwoBoneJob` | 155 ns |
| `IKAimJob` | 92 ns |

Nothing allocates after construction, so a frame produces no GC pressure. The library is trimmable
and NativeAOT-compatible.

Reproduce it:

```bash
dotnet run --project src/OzzAnimation.Benchmarks -c Release -- --filter '*'
```

The native rows need ozz's own C++ runtime, through the shim in [`bench/native`](bench/native):

```bash
cmake -B bench/native/build bench/native -DOZZ_ROOT=/path/to/ozz-animation -DCMAKE_BUILD_TYPE=Release
cmake --build bench/native/build --config Release
OZZ_NATIVE_LIB=$PWD/bench/native/build/libozz_shim.dylib \
  dotnet run --project src/OzzAnimation.Benchmarks -c Release -- --filter '*'
```

Both sides are allocation-free per frame — the shim keeps its scratch buffers in its context for the
same reason this library sizes everything up front, so what is timed is the sampling and the
hierarchy walk rather than an allocator.

## Building

```bash
dotnet build OzzAnimation.NET.slnx
dotnet run --project src/OzzAnimation.Tests -c Release
```

Targets `net10.0`.

## Licence

MIT — see [LICENSE](LICENSE), which also carries ozz-animation's own MIT notice.

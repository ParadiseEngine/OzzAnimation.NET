# OzzAnimation.NET

A managed C# port of **[ozz-animation](https://github.com/guillaumeblanc/ozz-animation)** — Guillaume
Blanc's open-source skeletal animation runtime. Same data layout, same compression, same archive
format, byte for byte; no native code, no P/Invoke, no marshalling.

Load an ozz skeleton and its clips, sample a clip into local poses and model-space matrices, and —
offline — build, optimize and save the very same archives from raw keyframes.

```csharp
var skeleton = Skeleton.Load(File.ReadAllBytes("skeleton.ozz"));
var clip     = AnimationClip.Load(File.ReadAllBytes("walk.ozz"));

var player = new AnimationPlayer(skeleton);
player.Play(clip, loop: true);

// per frame — allocates nothing
player.Advance(deltaSeconds);
player.Evaluate();
ReadOnlySpan<Matrix4x4> models = player.ModelMatrices;
```

## Thanks

This library exists because of **ozz-animation** and its author, **Guillaume Blanc**. The runtime
data layout, the archive format, the keyframe compression, the cursor-cache sampler, the i-frame
seeking, the offline builder and the hierarchy-aware optimizer are all *his* design — this
repository is a reimplementation of them in C#, not an independent invention. If you find this
useful, the credit belongs upstream:

> **ozz-animation** — https://github.com/guillaumeblanc/ozz-animation
> © Guillaume Blanc, MIT licensed.

Thank you for building it, and for releasing it under a licence that made this port possible.

## Compatibility

Byte-compatible with **ozz-animation 0.17**:

| Archive | Tag | Version |
|---|---|---|
| Skeleton | `ozz-skeleton` | 2 |
| Animation | `ozz-animation` | 7 |

- A file written by `gltf2ozz` **loads here**.
- A file written here **loads in ozz's C++ runtime**.

This is enforced, not asserted: the test suite embeds archives produced by ozz-animation 0.17's own
C++ `SkeletonBuilder` and `AnimationBuilder` from a procedural rig, and checks that this library
reproduces them **byte for byte** from the same raw input — including after the optimizer runs.

Little-endian only. A big-endian archive is refused rather than byte-swapped.

## Performance

Sampling is a port of ozz's SIMD `SamplingJob`: keys are decoded and interpolated four tracks at a
time in `Vector128<float>` lanes, straight into a structure-of-arrays pose set with no transpose.
The cursor walk stays scalar, as in ozz. Where ozz uses estimated reciprocal and inverse-square-root
instructions, this uses exact division and square root — poses differ from native ozz around the
seventh decimal, and from the source clip by the quantization alone.

One frame of one 64-joint character (sample + local-to-model), Apple silicon, .NET 10, BenchmarkDotNet:

| Runtime | Playback | Scrubbing | Allocated |
|---|---|---|---|
| **OzzAnimation.NET** | **1.21 µs** | 5.36 µs | **0 B** |
| ozz-animation 0.17 (C++, native) | 1.29 µs | **4.48 µs** | 0 B |
| A scalar managed port, for reference | 2.25 µs | 7.77 µs | 0 B |

*Playback* steps 1/100 of the clip per frame — where ozz's per-track cursor cache pays off.
*Scrubbing* jumps to a random ratio every frame — where the i-frames pay off, and where native
ozz's estimated-reciprocal instructions still win by about 20%.

These numbers were measured in the harness this port was extracted from; the benchmark project is
not (yet) part of this repository.

Nothing allocates after construction, so a frame produces no GC pressure. The library is trimmable
and NativeAOT-compatible.

## What is here

**Runtime**

| Type | ozz equivalent |
|---|---|
| `Skeleton` | `ozz::animation::Skeleton` |
| `AnimationClip` | `ozz::animation::Animation` |
| `SamplingContext` | `SamplingJob` + its context |
| `SoaTransforms` | `ozz::math::SoaTransform` span |
| `LocalToModel` | `LocalToModelJob` |
| `AnimationPlayer` | — (a small playback/cross-fade helper, not from ozz) |
| `SkinningPalette` | the skinning matrix product |

**Offline** (`OzzAnimation.Offline`)

| Type | ozz equivalent |
|---|---|
| `RawSkeleton` / `RawAnimation` | `offline::RawSkeleton` / `RawAnimation` |
| `SkeletonBuilder` | `offline::SkeletonBuilder` |
| `AnimationBuilder` | `offline::AnimationBuilder` |
| `AnimationOptimizer` | `offline::AnimationOptimizer` |

### Not ported

Blending jobs beyond the two-clip cross-fade in `AnimationPlayer`, IK (`TwoBoneIKJob`,
`AimIKJob`), user-channel tracks (`Track`, `TrackSamplingJob`, `TrackTriggeringJob`),
additive blending, and the glTF importer. The archive format for those is untouched, so adding
them later would not break anything already written.

### One behavioural note

A `RawTrack` with **no keys** builds to **identity**, not to the joint's rest pose — same as ozz.
If you want unanimated joints to hold their rest transform, fill those tracks from the skeleton
before calling `AnimationBuilder.Build`.

## Building

```bash
dotnet build OzzAnimation.NET.slnx
dotnet run --project src/OzzAnimation.Tests -c Release   # 24 tests
```

Targets `net10.0`.

## Licence

MIT — see [LICENSE](LICENSE), which also carries ozz-animation's own MIT notice.

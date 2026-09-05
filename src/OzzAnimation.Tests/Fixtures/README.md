# Test fixtures

Binary ozz archives the tests and the benchmark read. This repository is runtime-only and ships no
builders, so nothing here can be regenerated from it — the provenance is recorded instead.

## `ozz-skeleton.ozz`, `ozz-animation.ozz`, `ozz-animation-optimized.ozz`

Written by **ozz-animation 0.17's own C++ builders** (`offline::SkeletonBuilder`,
`offline::AnimationBuilder`, `offline::AnimationOptimizer`) from a procedural 37-joint rig: a
branching tree seeded by an LCG, 12 keys per track over 2.5 s, i-frames every 0.5 s, and for the
optimized file the default tolerance of 1 mm measured 10 cm from the joint.

These are the format contract. `OzzParityTests` loads each one and saves it again, and the bytes
must come back identical — which pins every header count, the SoA rest-pose groups, the keyframe
streams and the group-varint i-frame tables against a file this codebase did not produce.

**Do not regenerate these from this library.** Their whole value is that ozz's C++ wrote them. If
they ever need replacing, build them with ozz's own tools and keep this note accurate.

## `bench-skeleton.ozz`, `bench-animation.ozz`

A 64-joint branching rig, every joint animated on all three channels at 30 Hz over 1.333 s, with
i-frames every 0.5 s — the shape a DCC export has, and the size the benchmark reports.

Baked from a closed-form curve, which is what lets `OzzParityTests` check decoded poses against
ground truth rather than against another run of the same code. The curve is mirrored in
`TestRigs.BenchmarkSource`; the two must stay in step, so treat these files as fixed. Per track `j`
at time `t`, with `a = j × 0.37 + t × 3.1`:

    translation = (sin a × 0.05,  0.2 + cos a × 0.02,  sin(a / 2) × 0.05)
    rotation    = axis-angle(normalize(0.3, 1, 0.2), sin a × 0.6)
    scale       = 1 + sin a × 0.03

The benchmark loads these same files into both this library and ozz's C++ runtime, so a run with
`OZZ_NATIVE_LIB` set is also a live check that one archive satisfies both.

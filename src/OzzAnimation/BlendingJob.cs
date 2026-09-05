using System.Runtime.Intrinsics;

namespace OzzAnimation;

/// <summary>
/// One input pose of a <see cref="BlendingJob"/>, with its weight and optional per-joint weights.
/// ozz's <c>BlendingJob::Layer</c>.
/// </summary>
public struct BlendingLayer
{
    /// <summary>Negative values count as zero. Weights are normalized across layers, so any range works; 0..1 is the natural one. On an additive layer a negative weight subtracts the pose instead of adding it.</summary>
    public float Weight;

    /// <summary>Local-space transforms, usually a sampler's output. Must hold at least as many joints as the rest pose.</summary>
    public SoaTransforms? Transform;

    /// <summary>
    /// Optional weight per joint, one <see cref="Vector128{T}"/> per group of four joints — this is
    /// what makes partial blending (upper body from one clip, lower from another) possible. Null
    /// means every joint weighs 1. Negative lanes count as zero; lanes above 1 are not clamped.
    /// </summary>
    public Vector128<float>[]? JointWeights;
}

/// <summary>
/// Blends any number of local-space poses into one, by weight, with optional per-joint weights and
/// optional additive layers. ozz's <c>BlendingJob</c>.
/// </summary>
/// <remarks>
/// The number of joints processed is the rest pose's; every other buffer must be at least that big.
/// Where the accumulated weight of a joint falls below <see cref="Threshold"/> the rest pose makes
/// up the difference, so a blend of nothing is the rest pose rather than a collapsed one.
/// Allocates only the per-joint weight accumulator, sized to the rest pose, in the constructor.
/// </remarks>
public sealed class BlendingJob
{
    private readonly Vector128<float>[] _accumulatedWeights;

    /// <param name="soaJointCount">Groups of four joints this job will be run on — <see cref="Skeleton.SoaJointCount"/>.</param>
    public BlendingJob(int soaJointCount)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(soaJointCount);
        _accumulatedWeights = new Vector128<float>[soaJointCount];
    }

    /// <summary>The rest pose fills in below this accumulated weight. Must be positive; ozz's default is 0.1.</summary>
    public float Threshold { get; set; } = 0.1f;

    /// <summary>Whether these arguments would run. Mirrors ozz's <c>Validate</c>, so a caller can check without catching.</summary>
    public bool Validate(ReadOnlySpan<BlendingLayer> layers, ReadOnlySpan<BlendingLayer> additiveLayers, SoaTransforms? restPose, SoaTransforms? output)
    {
        if (restPose is null || output is null) return false;
        var groups = restPose.GroupCount;
        if (groups == 0 || groups > _accumulatedWeights.Length) return false;
        if (output.GroupCount < groups) return false;
        foreach (var layer in layers)
        {
            if (!ValidateLayer(layer, groups)) return false;
        }

        foreach (var layer in additiveLayers)
        {
            if (!ValidateLayer(layer, groups)) return false;
        }

        return Threshold > 0f;
    }

    /// <summary>Blends <paramref name="layers"/>, falls back on <paramref name="restPose"/> below the threshold, normalizes, then applies <paramref name="additiveLayers"/> — ozz's four stages, in that order.</summary>
    /// <returns>False when <see cref="Validate"/> would fail, leaving the output untouched.</returns>
    public bool Run(ReadOnlySpan<BlendingLayer> layers, ReadOnlySpan<BlendingLayer> additiveLayers, SoaTransforms restPose, SoaTransforms output)
    {
        if (!Validate(layers, additiveLayers, restPose, output)) return false;

        var groups = restPose.GroupCount;
        var passes = 0;
        var partialPasses = 0;
        var accumulatedWeight = 0f;

        BlendLayers(layers, groups, output, ref passes, ref partialPasses, ref accumulatedWeight);
        BlendRestPose(restPose, groups, output, passes, partialPasses, ref accumulatedWeight);
        Normalize(groups, output, partialPasses, accumulatedWeight);
        AddLayers(additiveLayers, groups, output);
        return true;
    }

    private static bool ValidateLayer(in BlendingLayer layer, int groups) =>
        layer.Transform is not null
        && layer.Transform.GroupCount >= groups
        && (layer.JointWeights is null || layer.JointWeights.Length >= groups);

    private void BlendLayers(ReadOnlySpan<BlendingLayer> layers, int groups, SoaTransforms output, ref int passes, ref int partialPasses, ref float accumulatedWeight)
    {
        foreach (var layer in layers)
        {
            if (layer.Weight <= 0f) continue;
            accumulatedWeight += layer.Weight;
            var layerWeight = Vector128.Create(layer.Weight);
            var source = layer.Transform!;
            var weights = layer.JointWeights;
            if (weights is not null) partialPasses++;

            for (var g = 0; g < groups; g++)
            {
                var weight = weights is null ? layerWeight : layerWeight * SoaMath.Max0(weights[g]);
                if (passes == 0)
                {
                    _accumulatedWeights[g] = weight;
                    BlendFirstPass(source, g, weight, output);
                }
                else
                {
                    _accumulatedWeights[g] += weight;
                    BlendNextPass(source, g, weight, output);
                }
            }

            passes++;
        }
    }

    private void BlendRestPose(SoaTransforms restPose, int groups, SoaTransforms output, int passes, int partialPasses, ref float accumulatedWeight)
    {
        if (partialPasses == 0)
        {
            // No per-joint weights anywhere, so one global comparison settles it.
            var restWeight = Threshold - accumulatedWeight;
            if (restWeight <= 0f) return;

            if (passes == 0)
            {
                accumulatedWeight = 1f;
                for (var g = 0; g < groups; g++)
                {
                    output.Translations[g] = restPose.Translations[g];
                    output.Rotations[g] = restPose.Rotations[g];
                    output.Scales[g] = restPose.Scales[g];
                }

                return;
            }

            accumulatedWeight = Threshold;
            var weight = Vector128.Create(restWeight);
            for (var g = 0; g < groups; g++) BlendNextPass(restPose, g, weight, output);
            return;
        }

        // A partial pass ran, so the threshold has to be tested per joint.
        var threshold = Vector128.Create(Threshold);
        for (var g = 0; g < groups; g++)
        {
            var restWeight = SoaMath.Max0(threshold - _accumulatedWeights[g]);
            _accumulatedWeights[g] = Vector128.Max(threshold, _accumulatedWeights[g]);
            BlendNextPass(restPose, g, restWeight, output);
        }
    }

    private void Normalize(int groups, SoaTransforms output, int partialPasses, float accumulatedWeight)
    {
        // Translations and scales were pre-multiplied by their weights, so they only need the
        // divide; rotations were summed and need a normalize, which subsumes it. Without a partial
        // pass every joint shares one divisor, so it is computed once rather than per group.
        var shared = partialPasses == 0 ? Vector128.Create(1f / accumulatedWeight) : default;
        for (var g = 0; g < groups; g++)
        {
            var ratio = partialPasses == 0 ? shared : Vector128<float>.One / _accumulatedWeights[g];
            output.Rotations[g] = SoaMath.Normalize(output.Rotations[g]);
            ref var t = ref output.Translations[g];
            t.X *= ratio; t.Y *= ratio; t.Z *= ratio;
            ref var s = ref output.Scales[g];
            s.X *= ratio; s.Y *= ratio; s.Z *= ratio;
        }
    }

    private static void AddLayers(ReadOnlySpan<BlendingLayer> additiveLayers, int groups, SoaTransforms output)
    {
        foreach (var layer in additiveLayers)
        {
            if (layer.Weight == 0f) continue;
            var subtract = layer.Weight < 0f;
            var layerWeight = Vector128.Create(MathF.Abs(layer.Weight));
            var source = layer.Transform!;
            var weights = layer.JointWeights;

            for (var g = 0; g < groups; g++)
            {
                var weight = weights is null ? layerWeight : layerWeight * SoaMath.Max0(weights[g]);
                if (subtract) SubtractPass(source, g, weight, output);
                else AddPass(source, g, weight, output);
            }
        }
    }

    private static void BlendFirstPass(SoaTransforms source, int g, Vector128<float> weight, SoaTransforms output)
    {
        ref readonly var st = ref source.Translations[g];
        ref var ot = ref output.Translations[g];
        ot.X = st.X * weight; ot.Y = st.Y * weight; ot.Z = st.Z * weight;

        ref readonly var sr = ref source.Rotations[g];
        ref var or = ref output.Rotations[g];
        or.X = sr.X * weight; or.Y = sr.Y * weight; or.Z = sr.Z * weight; or.W = sr.W * weight;

        ref readonly var ss = ref source.Scales[g];
        ref var os = ref output.Scales[g];
        os.X = ss.X * weight; os.Y = ss.Y * weight; os.Z = ss.Z * weight;
    }

    private static void BlendNextPass(SoaTransforms source, int g, Vector128<float> weight, SoaTransforms output)
    {
        ref readonly var st = ref source.Translations[g];
        ref var ot = ref output.Translations[g];
        ot.X += st.X * weight; ot.Y += st.Y * weight; ot.Z += st.Z * weight;

        // Opposed quaternions are negated so the accumulation stays on one hemisphere; without
        // this a sum of two equivalent rotations can cancel to zero length.
        ref readonly var sr = ref source.Rotations[g];
        ref var or = ref output.Rotations[g];
        var sign = SoaMath.Sign(SoaMath.Dot(or, sr));
        or.X += (sr.X ^ sign) * weight;
        or.Y += (sr.Y ^ sign) * weight;
        or.Z += (sr.Z ^ sign) * weight;
        or.W += (sr.W ^ sign) * weight;

        ref readonly var ss = ref source.Scales[g];
        ref var os = ref output.Scales[g];
        os.X += ss.X * weight; os.Y += ss.Y * weight; os.Z += ss.Z * weight;
    }

    private static void AddPass(SoaTransforms source, int g, Vector128<float> weight, SoaTransforms output)
    {
        var one = Vector128<float>.One;
        var oneMinusWeight = one - weight;

        ref readonly var st = ref source.Translations[g];
        ref var ot = ref output.Translations[g];
        ot.X += st.X * weight; ot.Y += st.Y * weight; ot.Z += st.Z * weight;

        ref var or = ref output.Rotations[g];
        or = SoaMath.Multiply(or, SoaMath.Normalize(InterpolateFromIdentity(source.Rotations[g], weight)));

        ref readonly var ss = ref source.Scales[g];
        ref var os = ref output.Scales[g];
        os.X *= ss.X * weight + oneMinusWeight;
        os.Y *= ss.Y * weight + oneMinusWeight;
        os.Z *= ss.Z * weight + oneMinusWeight;
    }

    private static void SubtractPass(SoaTransforms source, int g, Vector128<float> weight, SoaTransforms output)
    {
        var one = Vector128<float>.One;
        var oneMinusWeight = one - weight;

        ref readonly var st = ref source.Translations[g];
        ref var ot = ref output.Translations[g];
        ot.X -= st.X * weight; ot.Y -= st.Y * weight; ot.Z -= st.Z * weight;

        ref var or = ref output.Rotations[g];
        or = SoaMath.Multiply(or, SoaMath.Conjugate(SoaMath.Normalize(InterpolateFromIdentity(source.Rotations[g], weight))));

        ref readonly var ss = ref source.Scales[g];
        ref var os = ref output.Scales[g];
        os.X /= ss.X * weight + oneMinusWeight;
        os.Y /= ss.Y * weight + oneMinusWeight;
        os.Z /= ss.Z * weight + oneMinusWeight;
    }

    /// <summary>The additive layer's rotation scaled back toward identity by its weight, flipped onto the positive-w hemisphere first so the lerp takes the short arc.</summary>
    private static SoaQuaternion InterpolateFromIdentity(in SoaQuaternion rotation, Vector128<float> weight)
    {
        var one = Vector128<float>.One;
        var sign = SoaMath.Sign(rotation.W);
        return new SoaQuaternion
        {
            X = (rotation.X ^ sign) * weight,
            Y = (rotation.Y ^ sign) * weight,
            Z = (rotation.Z ^ sign) * weight,
            W = ((rotation.W ^ sign) - one) * weight + one,
        };
    }
}

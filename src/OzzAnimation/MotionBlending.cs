using System.Numerics;

namespace OzzAnimation;

/// <summary>One root-motion delta and its weight. ozz's <c>MotionBlendingJob::Layer</c>.</summary>
/// <param name="Weight">Negative values count as zero.</param>
/// <param name="Delta">The motion accumulated over this frame by one clip.</param>
public readonly record struct MotionLayer(float Weight, JointPose Delta);

/// <summary>
/// Blends the root-motion deltas of clips being blended together, so the character travels at a
/// rate consistent with the pose it is actually in. ozz's <c>MotionBlendingJob</c>.
/// </summary>
/// <remarks>
/// Translation is blended as direction and length separately, then recombined: lerping the vectors
/// directly would shorten the result whenever two clips point different ways, and a walk blended
/// with a turn would creep slower than either. Scale is not blended — root motion has none, and
/// the output always carries unit scale.
/// </remarks>
public static class MotionBlending
{
    /// <summary>Blends the layers by weight into one delta.</summary>
    /// <returns>False when <paramref name="layers"/> is empty or every weight is zero; the output is identity then.</returns>
    public static bool Blend(ReadOnlySpan<MotionLayer> layers, out JointPose output)
    {
        var accumulatedWeight = 0f;
        var weightedLength = 0f;
        var direction = Vector3.Zero;
        var rotation = new Quaternion(0f, 0f, 0f, 0f);

        foreach (var layer in layers)
        {
            if (layer.Weight <= 0f) continue;
            accumulatedWeight += layer.Weight;

            // Direction and length are accumulated apart so that interpolating between two clips
            // that move the same distance in different directions keeps that distance.
            var length = layer.Delta.Translation.Length();
            weightedLength += length * layer.Weight;
            if (length != 0f) direction += layer.Delta.Translation * (layer.Weight / length);

            // Onto one hemisphere, so the accumulation cannot cancel itself out.
            var signedWeight = Vector4.Dot(AsVector4(rotation), AsVector4(layer.Delta.Rotation)) < 0f ? -layer.Weight : layer.Weight;
            rotation = new Quaternion(
                rotation.X + layer.Delta.Rotation.X * signedWeight,
                rotation.Y + layer.Delta.Rotation.Y * signedWeight,
                rotation.Z + layer.Delta.Rotation.Z * signedWeight,
                rotation.W + layer.Delta.Rotation.W * signedWeight);
        }

        var denominator = direction.Length() * accumulatedWeight;
        var translation = denominator == 0f ? Vector3.Zero : direction * (weightedLength / denominator);
        var lengthSquared = AsVector4(rotation).LengthSquared();
        var normalized = lengthSquared == 0f ? Quaternion.Identity : Quaternion.Normalize(rotation);

        output = new JointPose(translation, normalized, Vector3.One);
        return accumulatedWeight > 0f;
    }

    private static Vector4 AsVector4(Quaternion q) => new(q.X, q.Y, q.Z, q.W);
}

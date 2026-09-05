using System.Numerics;

namespace OzzAnimation;

/// <summary>
/// One character's playback: the clip it is playing, where in it, at what rate, whether it loops,
/// and the clip it is fading out of. <see cref="Advance"/> moves time, <see cref="Evaluate"/>
/// samples (blending the two clips while a fade runs) into local poses and model-space matrices
/// the caller reads from <see cref="LocalPose"/> and <see cref="ModelMatrices"/>. Allocates only in
/// the constructor: both sampling contexts, both pose sets and the matrices are sized once.
/// </summary>
/// <remarks>
/// The skeleton is fixed at construction because the buffers are sized to it, and because a clip's
/// tracks index that skeleton's joints — playing a clip cooked for another skeleton is refused.
/// </remarks>
public sealed class AnimationPlayer
{
    private readonly Skeleton _skeleton;
    private readonly SamplingContext _currentContext;
    private readonly SamplingContext _outgoingContext;
    private readonly SoaTransforms _pose;
    private readonly SoaTransforms _outgoingPose;
    private readonly Matrix4x4[] _models;
    private Slot _current;
    private Slot _outgoing;
    private float _fadeDuration;
    private float _fadeRemaining;

    public AnimationPlayer(Skeleton skeleton)
    {
        ArgumentNullException.ThrowIfNull(skeleton);
        _skeleton = skeleton;
        _currentContext = new SamplingContext(skeleton.JointCount);
        _outgoingContext = new SamplingContext(skeleton.JointCount);
        _pose = new SoaTransforms(skeleton.JointCount);
        _outgoingPose = new SoaTransforms(skeleton.JointCount);
        _models = new Matrix4x4[skeleton.JointCount];
        _current = Slot.Rest;
        _outgoing = Slot.Rest;
    }

    public int JointCount => _models.Length;

    /// <summary>The clip playing, or null for the rest pose.</summary>
    public AnimationClip? Current => _current.Clip;

    /// <summary>Seconds into the current clip.</summary>
    public float Time => _current.Time;

    public float Rate
    {
        get => _current.Rate;
        set => _current.Rate = value;
    }

    public bool IsLooping => _current.Loop;

    /// <summary>A non-looping clip that has reached its end (or its start when playing backwards).</summary>
    public bool IsFinished => _current.Clip is not null && !_current.Loop && (_current.Rate >= 0f ? _current.Time >= _current.Duration : _current.Time <= 0f);

    public bool IsFading => _fadeRemaining > 0f;

    /// <summary>The blend toward the current clip, 0 at the start of a fade, 1 when none runs.</summary>
    public float FadeProgress => _fadeRemaining > 0f ? 1f - _fadeRemaining / _fadeDuration : 1f;

    /// <summary>What the last <see cref="Evaluate"/> produced, one local pose per joint.</summary>
    public SoaTransforms LocalPose => _pose;

    /// <summary>What the last <see cref="Evaluate"/> produced, one model-space matrix per joint (row-vector convention).</summary>
    public ReadOnlySpan<Matrix4x4> ModelMatrices => _models;

    /// <summary>Starts <paramref name="clip"/>; with a positive <paramref name="fadeSeconds"/> the clip playing until now keeps advancing and blends out over that time.</summary>
    /// <exception cref="ArgumentException">The clip has a different track count than the skeleton has joints.</exception>
    public void Play(AnimationClip clip, float fadeSeconds = 0f, bool loop = true, float rate = 1f, float startTime = 0f)
    {
        ArgumentNullException.ThrowIfNull(clip);
        if (clip.TrackCount != JointCount)
        {
            throw new ArgumentException($"The clip '{clip.Name}' has {clip.TrackCount} tracks; the skeleton has {JointCount} joints.", nameof(clip));
        }

        StartFade(fadeSeconds);
        _current = new Slot(clip, loop, rate, Math.Clamp(startTime, 0f, clip.Duration));
        _currentContext.Invalidate();
    }

    /// <summary>Back to the rest pose, fading the playing clip out over <paramref name="fadeSeconds"/> when positive.</summary>
    public void Stop(float fadeSeconds = 0f)
    {
        StartFade(fadeSeconds);
        _current = Slot.Rest;
    }

    /// <summary>Moves both clips by <paramref name="deltaSeconds"/> at their rates — a looping clip wraps, a one-shot clamps — and runs the fade down; the fade's clock is unscaled.</summary>
    public void Advance(float deltaSeconds)
    {
        _current.Advance(deltaSeconds);
        if (_fadeRemaining <= 0f) return;

        _outgoing.Advance(deltaSeconds);
        _fadeRemaining -= deltaSeconds;
        if (_fadeRemaining <= 0f)
        {
            _fadeRemaining = 0f;
            _outgoing = Slot.Rest;
        }
    }

    /// <summary>Samples the clips at their current times, blends them by the fade, and walks the hierarchy into <see cref="ModelMatrices"/>.</summary>
    public void Evaluate()
    {
        Sample(in _current, _currentContext, _pose);
        if (_fadeRemaining > 0f)
        {
            Sample(in _outgoing, _outgoingContext, _outgoingPose);
            SoaTransforms.Blend(_outgoingPose, _pose, FadeProgress, _pose);
        }

        LocalToModel.Compute(_skeleton, _pose, _models);
    }

    private void StartFade(float fadeSeconds)
    {
        if (fadeSeconds > 0f && _current.Clip is not null)
        {
            _outgoing = _current;
            _fadeDuration = fadeSeconds;
            _fadeRemaining = fadeSeconds;
        }
        else
        {
            _outgoing = Slot.Rest;
            _fadeRemaining = 0f;
        }
    }

    private void Sample(in Slot slot, SamplingContext context, SoaTransforms pose)
    {
        if (slot.Clip is null)
        {
            pose.CopyFrom(_skeleton.RestPoses);
            return;
        }

        context.Sample(slot.Clip, slot.Duration > 0f ? slot.Time / slot.Duration : 0f, pose);
    }

    /// <summary>One playing clip: which, where, how fast, and whether it wraps.</summary>
    private struct Slot(AnimationClip? clip, bool loop, float rate, float time)
    {
        public static Slot Rest => new(null, false, 1f, 0f);

        public readonly AnimationClip? Clip = clip;
        public readonly bool Loop = loop;
        public float Rate = rate;
        public float Time = time;

        public float Duration => Clip?.Duration ?? 0f;

        public void Advance(float deltaSeconds)
        {
            if (Clip is null) return;
            var duration = Duration;
            Time += deltaSeconds * Rate;
            if (Loop)
            {
                Time -= MathF.Floor(Time / duration) * duration;
                if (Time < 0f || Time >= duration) Time = 0f;
            }
            else
            {
                Time = Math.Clamp(Time, 0f, duration);
            }
        }
    }
}

/// <summary>The joint palette a skinned mesh's vertex shader consumes, from model-space matrices and the mesh's skin — handed over as spans so this assembly need not know any mesh format.</summary>
public static class SkinningPalette
{
    /// <summary>Row-vector convention: <c>palette[i] = inverseBind[i] × model[joints[i]] × inverse(model[meshJoint])</c>; a negative <paramref name="meshJoint"/> means the mesh sits at the model's origin.</summary>
    /// <exception cref="ArgumentException">Mismatched skin arrays, or a joint outside the skeleton.</exception>
    public static void Compute(ReadOnlySpan<Matrix4x4> models, ReadOnlySpan<int> joints, ReadOnlySpan<Matrix4x4> inverseBinds, int meshJoint, Span<Matrix4x4> palette)
    {
        if (inverseBinds.Length != joints.Length) throw new ArgumentException($"{joints.Length} joints and {inverseBinds.Length} inverse-bind matrices.", nameof(inverseBinds));
        if (palette.Length < joints.Length) throw new ArgumentException($"The palette holds {palette.Length} of {joints.Length} slots.", nameof(palette));
        if (meshJoint >= models.Length) throw new ArgumentException($"Mesh joint {meshJoint} is outside the {models.Length}-joint skeleton.", nameof(meshJoint));

        var inverseMeshWorld = Matrix4x4.Identity;
        if (meshJoint >= 0 && !Matrix4x4.Invert(models[meshJoint], out inverseMeshWorld)) inverseMeshWorld = Matrix4x4.Identity;
        for (var i = 0; i < joints.Length; i++)
        {
            var joint = joints[i];
            if (joint < 0 || joint >= models.Length) throw new ArgumentException($"Palette slot {i} names joint {joint} of {models.Length}.", nameof(joints));
            palette[i] = inverseBinds[i] * models[joint] * inverseMeshWorld;
        }
    }
}

// The C surface the benchmark measures native ozz-animation through: load archives from memory,
// then sample a clip to model-space joint matrices. Buffers live in the context and are allocated
// once, so what is timed is ozz's sampling and hierarchy walk and not an allocator — the managed
// side under comparison allocates nothing per frame either.
#include <cstdint>
#include <vector>

#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_transform.h"

#if defined(_WIN32)
#define OZZ_SHIM_EXPORT extern "C" __declspec(dllexport)
#else
#define OZZ_SHIM_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {
template <typename T>
T* LoadArchive(const uint8_t* bytes, size_t length) {
  ozz::io::MemoryStream stream;
  stream.Write(bytes, length);
  stream.Seek(0, ozz::io::Stream::kSet);
  ozz::io::IArchive archive(&stream);
  if (!archive.TestTag<T>()) return nullptr;
  T* loaded = new T();
  archive >> *loaded;
  return loaded;
}

// The sampling cursor plus the scratch both jobs need, so a frame allocates nothing.
struct Context {
  Context(int max_tracks, int num_soa_joints, int num_joints)
      : sampling(max_tracks), locals(num_soa_joints), models(num_joints) {}
  ozz::animation::SamplingJob::Context sampling;
  std::vector<ozz::math::SoaTransform> locals;
  std::vector<ozz::math::Float4x4> models;
};
}  // namespace

OZZ_SHIM_EXPORT void* Ozz_LoadSkeleton(const uint8_t* bytes, size_t length) {
  return LoadArchive<ozz::animation::Skeleton>(bytes, length);
}

OZZ_SHIM_EXPORT void* Ozz_LoadAnimation(const uint8_t* bytes, size_t length) {
  return LoadArchive<ozz::animation::Animation>(bytes, length);
}

OZZ_SHIM_EXPORT void Ozz_FreeSkeleton(void* skeleton) { delete static_cast<ozz::animation::Skeleton*>(skeleton); }
OZZ_SHIM_EXPORT void Ozz_FreeAnimation(void* animation) { delete static_cast<ozz::animation::Animation*>(animation); }

OZZ_SHIM_EXPORT int Ozz_JointCount(const void* skeleton) {
  return static_cast<const ozz::animation::Skeleton*>(skeleton)->num_joints();
}

OZZ_SHIM_EXPORT int Ozz_TrackCount(const void* animation) {
  return static_cast<const ozz::animation::Animation*>(animation)->num_tracks();
}

OZZ_SHIM_EXPORT void* Ozz_CreateContext(const void* skeletonHandle, int maxTracks) {
  const auto* skeleton = static_cast<const ozz::animation::Skeleton*>(skeletonHandle);
  return new Context(maxTracks, skeleton->num_soa_joints(), skeleton->num_joints());
}

OZZ_SHIM_EXPORT void Ozz_FreeContext(void* context) { delete static_cast<Context*>(context); }

// Model-space joint matrices for one clip at one ratio, 16 floats per joint, in ozz's column-major
// memory order: read straight into a row-major row-vector matrix, that IS the transpose the
// row-vector convention wants. Returns 0 on a job failure.
OZZ_SHIM_EXPORT int Ozz_SampleModelSpace(const void* skeletonHandle, const void* animationHandle, void* contextHandle,
                                         float ratio, float* out) {
  const auto* skeleton = static_cast<const ozz::animation::Skeleton*>(skeletonHandle);
  const auto* animation = static_cast<const ozz::animation::Animation*>(animationHandle);
  auto* context = static_cast<Context*>(contextHandle);

  ozz::animation::SamplingJob sampling;
  sampling.animation = animation;
  sampling.context = &context->sampling;
  sampling.ratio = ratio;
  sampling.output = ozz::make_span(context->locals);
  if (!sampling.Run()) return 0;

  ozz::animation::LocalToModelJob ltm;
  ltm.skeleton = skeleton;
  ltm.input = ozz::make_span(context->locals);
  ltm.output = ozz::make_span(context->models);
  if (!ltm.Run()) return 0;

  for (int j = 0; j < skeleton->num_joints(); ++j) {
    for (int c = 0; c < 4; ++c) ozz::math::StorePtrU(context->models[j].cols[c], out + j * 16 + c * 4);
  }
  return 1;
}

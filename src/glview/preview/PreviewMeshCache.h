#pragma once

#include <memory>
#include <cstddef>

#include "geometry/linalg.h"

class CSGProducts;

namespace PreviewMeshCache {

inline constexpr size_t minGroupSize = 12;

// Construct an equivalent mesh for repeated, opaque CSG products. This runs on
// an immutable preview snapshot, independently of the evaluator and OpenGL.
std::shared_ptr<CSGProducts> bake(const CSGProducts& source, const Color4f& material,
                                  const Color4f& cutout, const std::weak_ptr<void>& lifetime);

}  // namespace PreviewMeshCache

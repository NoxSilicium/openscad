#include "glview/preview/PreviewMeshCache.h"

#if defined(ENABLE_OPENCSG) && defined(ENABLE_MANIFOLD)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "core/CSGNode.h"
#include "geometry/PolySet.h"
#include "geometry/manifold/manifoldutils.h"

namespace {

using Manifold = manifold::Manifold;
using Operation = manifold::OpType;

constexpr size_t minGroupSize = 12;
constexpr size_t maxGroupSize = 256;
constexpr size_t maxInputElements = 200000;
constexpr size_t maxOutputFaces = 120000;
constexpr size_t maxPlaneComparisons = 20000000;

bool sameOperand(const CSGChainObject& a, const CSGChainObject& b)
{
  return a.leaf == b.leaf && a.flags == b.flags;
}

std::optional<size_t> varyingIntersection(const CSGProduct& a, const CSGProduct& b)
{
  if (a.intersections.size() != b.intersections.size() ||
      a.subtractions.size() != b.subtractions.size() ||
      a.intersections.size() + a.subtractions.size() < 2 ||
      !std::equal(a.subtractions.begin(), a.subtractions.end(), b.subtractions.begin(), sameOperand)) {
    return std::nullopt;
  }

  std::optional<size_t> varying;
  for (size_t i = 0; i < a.intersections.size(); ++i) {
    if (sameOperand(a.intersections[i], b.intersections[i])) continue;
    if (varying) return std::nullopt;
    varying = i;
  }
  return varying;
}

struct FaceAppearance {
  Color4f color;
  bool backFace;
};

struct FacePlane {
  Vector3d anchor;
  Vector3d normal;
  BoundingBox bounds;
};

struct OperandSurfaces {
  FaceAppearance appearance;
  BoundingBox bounds;
  double padding = 0.0;
  std::vector<FacePlane> faces;
};

std::optional<OperandSurfaces> collectSurfaces(const PolySet& mesh, const Transform3d& transform,
                                               FaceAppearance appearance, size_t& remainingInput)
{
  if (mesh.vertices.size() > remainingInput) return std::nullopt;
  remainingInput -= mesh.vertices.size();
  if (mesh.indices.size() > remainingInput) return std::nullopt;

  OperandSurfaces result{appearance, {}, 0.0, {}};
  std::vector<Vector3d> vertices;
  vertices.reserve(mesh.vertices.size());
  for (const auto& vertex : mesh.vertices) {
    const Vector3d world = transform * vertex;
    if (!world.cast<float>().allFinite()) return std::nullopt;
    vertices.push_back(world);
    result.bounds.extend(world);
  }
  const auto magnitude = std::max(result.bounds.min().cwiseAbs().maxCoeff(),
                                  result.bounds.max().cwiseAbs().maxCoeff());
  result.padding = 16 * std::numeric_limits<float>::epsilon() * std::max(1.0, magnitude);
  result.faces.reserve(mesh.indices.size());
  for (const auto& face : mesh.indices) {
    if (face.size() < 3 || face.size() > remainingInput) return std::nullopt;
    remainingInput -= face.size();
    BoundingBox bounds;
    for (const auto vertex : face) {
      if (vertex < 0 || static_cast<size_t>(vertex) >= vertices.size()) return std::nullopt;
      bounds.extend(vertices[vertex]);
    }

    const auto& anchor = vertices[face.front()];
    Vector3d normal = Vector3d::Zero();
    for (size_t i = 2; i < face.size(); ++i) {
      const Vector3d cross = (vertices[face[i - 1]] - anchor).cross(vertices[face[i]] - anchor);
      if (cross.squaredNorm() > normal.squaredNorm()) normal = cross;
    }
    const double length = normal.norm();
    if (!std::isfinite(length)) return std::nullopt;
    if (length == 0.0) continue;
    normal /= length;
    for (const auto vertex : face) {
      if (std::abs(normal.dot(vertices[vertex] - anchor)) > result.padding) return std::nullopt;
    }
    result.faces.push_back({anchor, normal, bounds});
  }
  return result;
}

bool overlaps(const BoundingBox& a, const BoundingBox& b, double padding)
{
  return ((a.min() - b.max()).array() <= padding).all() &&
         ((b.min() - a.max()).array() <= padding).all();
}

bool hasAmbiguousFaces(const OperandSurfaces& a, const OperandSurfaces& b, size_t& remaining)
{
  if (a.appearance.color == b.appearance.color && a.appearance.backFace == b.appearance.backFace) {
    return false;
  }
  const double padding = a.padding + b.padding;
  if (!overlaps(a.bounds, b.bounds, padding)) return false;
  const size_t comparisons = a.faces.size() * b.faces.size();
  if (comparisons > remaining) return true;
  remaining -= comparisons;
  // A Boolean mesh picks one provenance for an overlapping coplanar surface.
  // OpenCSG uses draw order there, so keep its path for competing appearances.
  for (const auto& x : a.faces) {
    for (const auto& y : b.faces) {
      if (std::abs(x.normal.dot(y.normal)) > 1 - 1e-8 && overlaps(x.bounds, y.bounds, padding) &&
          std::abs(x.normal.dot(y.anchor - x.anchor)) <= padding) return true;
    }
  }
  return false;
}

std::optional<Manifold> convert(const CSGChainObject& operand, const Color4f& defaultColor,
                                bool cutout, size_t& remainingInput, size_t& remainingComparisons,
                                std::vector<OperandSurfaces>& surfaces,
                                std::map<uint32_t, FaceAppearance>& appearance)
{
  const auto& leaf = *operand.leaf;
  const auto& mesh = leaf.polyset;
  if (operand.flags != CSGNode::FLAG_NONE || !mesh || mesh->getDimension() != 3 ||
      (!cutout && !leaf.color.isValid() && !mesh->color_indices.empty()) ||
      !leaf.matrix.matrix().allFinite()) return std::nullopt;

  Color4f color = leaf.color;
  if (!color.hasRgb()) color.setRgb(defaultColor.r(), defaultColor.g(), defaultColor.b());
  if (!color.hasAlpha()) color.setAlpha(defaultColor.a());
  if (!color.isValid() || !color.toVector4f().allFinite() || color.a() != 1.0f) return std::nullopt;

  auto faces = collectSurfaces(*mesh, leaf.matrix, {color, cutout}, remainingInput);
  if (!faces) return std::nullopt;
  for (const auto& previous : surfaces) {
    if (hasAmbiguousFaces(previous, *faces, remainingComparisons)) return std::nullopt;
  }
  surfaces.push_back(std::move(*faces));

  auto geometry = ManifoldUtils::createManifoldFromPolySet(*mesh, false);
  if (!geometry || geometry->getManifold().IsEmpty()) return std::nullopt;
  geometry->toOriginal();
  appearance.emplace(geometry->getManifold().OriginalID(), FaceAppearance{color, cutout});
  geometry->transform(leaf.matrix);
  return geometry->getManifold();
}

std::shared_ptr<PolySet> bakeGroup(const std::vector<CSGProduct>& products, size_t first,
                                  size_t end, size_t varying, const Color4f& material,
                                  const Color4f& cutout, size_t remainingOutputFaces)
{
  const auto& reference = products[first];
  size_t remainingInput = maxInputElements;
  size_t remainingComparisons = maxPlaneComparisons;
  std::vector<OperandSurfaces> surfaces;
  std::map<uint32_t, FaceAppearance> appearance;
  std::vector<Manifold> varyingSolids, intersections, subtractions;
  const auto load = [&](const CSGChainObject& operand, bool subtract) {
    return convert(operand, subtract ? cutout : material, subtract, remainingInput,
                   remainingComparisons, surfaces, appearance);
  };
  for (size_t i = first; i < end; ++i) {
    auto solid = load(products[i].intersections[varying], false);
    if (!solid) return {};
    varyingSolids.push_back(std::move(*solid));
  }
  for (size_t i = 0; i < reference.intersections.size(); ++i) {
    if (i == varying) continue;
    auto solid = load(reference.intersections[i], false);
    if (!solid) return {};
    intersections.push_back(std::move(*solid));
  }
  for (const auto& operand : reference.subtractions) {
    auto solid = load(operand, true);
    if (!solid) return {};
    subtractions.push_back(std::move(*solid));
  }

  // Union distributes over the common intersections and subtractions, so the
  // shared operands only need to be applied once.
  intersections.push_back(Manifold::BatchBoolean(varyingSolids, Operation::Add));
  auto result = Manifold::BatchBoolean(intersections, Operation::Intersect);
  if (!subtractions.empty()) result -= Manifold::BatchBoolean(subtractions, Operation::Add);
  if (result.Status() != Manifold::Error::NoError || result.NumTri() > remainingOutputFaces) return {};
  auto polyset = std::make_shared<PolySet>(3);
  polyset->setTriangular(true);
  if (result.IsEmpty()) return polyset;

  const auto mesh = result.GetMeshGL64();
  polyset->vertices.reserve(mesh.NumVert());
  polyset->indices.reserve(mesh.NumTri());
  polyset->color_indices.reserve(mesh.NumTri());
  for (size_t i = 0; i < mesh.vertProperties.size(); i += mesh.numProp) {
    polyset->vertices.emplace_back(mesh.vertProperties[i], mesh.vertProperties[i + 1],
                                  mesh.vertProperties[i + 2]);
  }
  std::map<Color4f, int32_t> colors;
  for (size_t run = 0; run < mesh.runOriginalID.size(); ++run) {
    const size_t start = mesh.runIndex[run];
    const size_t stop = mesh.runIndex[run + 1];
    if (start == stop) continue;
    const auto source = appearance.find(mesh.runOriginalID[run]);
    if (source == appearance.end()) return {};
    const auto [color, backFace] = source->second;
    const auto [index, inserted] = colors.emplace(color, polyset->colors.size());
    if (inserted) polyset->colors.push_back(color);
    for (size_t i = start; i < stop; i += 3) {
      // Match OpenCSG's lighting of the original cutter's back faces.
      polyset->indices.push_back({static_cast<int>(mesh.triVerts[i]),
                                  static_cast<int>(mesh.triVerts[i + (backFace ? 2 : 1)]),
                                  static_cast<int>(mesh.triVerts[i + (backFace ? 1 : 2)])});
      polyset->color_indices.push_back(index->second);
    }
  }
  return polyset;
}

}  // namespace

std::shared_ptr<CSGProducts> PreviewMeshCache::bake(const CSGProducts& source, const Color4f& material,
                                                    const Color4f& cutout,
                                                    const std::weak_ptr<void>& lifetime)
{
  const auto& products = source.products;
  auto cached = std::make_shared<CSGProducts>();
  cached->products.clear();
  size_t attempts = 0;
  size_t remainingOutputFaces = maxOutputFaces;
  bool changed = false;
  for (size_t i = 0; i < products.size();) {
    if (lifetime.expired()) return {};
    const auto varying = i + 1 < products.size() ? varyingIntersection(products[i], products[i + 1])
                                                  : std::nullopt;
    if (!varying || attempts >= 4) {
      cached->products.push_back(products[i++]);
      continue;
    }

    size_t end = i + 2;
    while (end < products.size() && end - i < maxGroupSize &&
           varyingIntersection(products[i], products[end]) == varying) ++end;

    if (end - i >= minGroupSize) {
      ++attempts;
      if (auto mesh = bakeGroup(products, i, end, *varying, material, cutout, remainingOutputFaces)) {
        changed = true;
        remainingOutputFaces -= mesh->numFacets();
        if (!mesh->isEmpty()) {
          CSGProduct product;
          product.intersections.emplace_back(std::make_shared<CSGLeaf>(
            mesh, Transform3d::Identity(), Color4f(), "Cached preview", 0));
          cached->products.push_back(std::move(product));
        }
        i = end;
        continue;
      }
    }
    cached->products.insert(cached->products.end(), products.begin() + i, products.begin() + end);
    i = end;
  }
  return changed ? cached : nullptr;
}

#endif

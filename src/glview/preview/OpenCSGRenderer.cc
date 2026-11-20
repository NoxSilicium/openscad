/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "glview/preview/OpenCSGRenderer.h"
#include "glview/preview/PreviewMeshCache.h"
#include "glview/Renderer.h"
#include "glview/ShaderUtils.h"
#include "glview/VertexState.h"
#include "geometry/linalg.h"
#include "glview/system-gl.h"

#include "Feature.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <memory.h>
#include <cstddef>
#include <utility>
#include <vector>

#ifdef ENABLE_OPENCSG

namespace {

// A solid cube is a box in its own coordinates even after rotation, scaling or shear.
bool fullySubtractedByBox(const CSGProduct& product)
{
  for (const auto& [cutter, flags] : product.subtractions) {
    if (!cutter->axisAlignedBox || !cutter->polyset || flags != CSGNode::FLAG_NONE) continue;

    const Transform3d toCutter = cutter->matrix.inverse();
    if (!toCutter.matrix().allFinite()) continue;
    const auto cutterBounds = cutter->polyset->getBoundingBox();

    for (const auto& [operand, operandFlags] : product.intersections) {
      if (!operand->polyset || operand->polyset->getDimension() != 3 ||
          operandFlags != CSGNode::FLAG_NONE) continue;

      const auto& worldBounds = operand->getBoundingBox();
      if (!worldBounds.min().allFinite() || !worldBounds.max().allFinite()) continue;
      const Vector3d magnitude = worldBounds.min().cwiseAbs().cwiseMax(worldBounds.max());
      const Vector3d margin = (32 * std::numeric_limits<float>::epsilon()) *
                              (toCutter.linear().cwiseAbs() * magnitude +
                               toCutter.translation().cwiseAbs());
      const auto& operandMatrix = operand->matrix;
      const Transform3d relative = toCutter * operandMatrix;
      const auto operandBounds = relative * operand->polyset->getBoundingBox();
      // Strict containment avoids removing coincident faces or near-tangencies.
      if (!operandBounds.isEmpty() &&
          ((operandBounds.min() - margin).array() > cutterBounds.min().array()).all() &&
          ((operandBounds.max() + margin).array() < cutterBounds.max().array()).all()) return true;

      // A rotated bounding box can overestimate a small mesh substantially.
      const auto& vertices = operand->polyset->vertices;
      if (vertices.size() > 128 || !operandBounds.intersects(cutterBounds)) continue;
      std::array<int, 3> touches{};
      const bool enclosed = std::all_of(vertices.begin(), vertices.end(), [&](const Vector3d& vertex) {
        const Vector3d world = operandMatrix * vertex;
        const Vector3d point = toCutter * world;
        if (!point.allFinite()) return false;
        for (int axis = 0; axis < 3; ++axis) {
          const bool above = point[axis] > cutterBounds.min()[axis] + margin[axis];
          const bool below = point[axis] < cutterBounds.max()[axis] - margin[axis];
          if (above && below) continue;
          // Allow an exact vertex or edge on a face through the cutter's origin.
          // Accept zeros only when each contribution to the plane equation is zero.
          const bool onOriginPlane =
            toCutter.translation()[axis] == 0.0 &&
            ((toCutter.linear().row(axis).array() == 0.0) || (world.transpose().array() == 0.0)).all();
          if (!onOriginPlane || ++touches[axis] > 2 ||
              (!above && cutterBounds.min()[axis] != 0.0) ||
              (!below && cutterBounds.max()[axis] != 0.0)) return false;
        }
        return true;
      });
      if (enclosed) return true;
    }
  }
  return false;
}

class OpaqueDepth
{
public:
  [[nodiscard]] bool available() const { return !tiles.empty(); }

  void capture(int width, int height, int depthBits)
  {
    this->width = width;
    this->height = height;
    depthPadding = std::max(1e-6, std::ldexp(2.0, -depthBits));
    const int columns = (width + tileSize - 1) / tileSize;
    const int rows = (height + tileSize - 1) / tileSize;

    pixels.resize(static_cast<size_t>(width) * height);
    glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_PACK_SWAP_BYTES, GL_FALSE);
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, pixels.data());
    const bool valid = glCheckd("Reading viewport depth", __FILE__, __LINE__);
    glPopClientAttrib();
    if (!valid) {
      tiles.clear();
      return;
    }

    tiles.resize(static_cast<size_t>(columns) * rows);
    for (int y = 0; y < height; y += tileSize) {
      for (int x = 0; x < width; x += tileSize) {
        float farthest = 0.0f;
        for (int row = y; row < std::min(y + tileSize, height); ++row) {
          const auto *first = pixels.data() + static_cast<size_t>(row) * width + x;
          farthest = std::max(farthest, *std::max_element(first, first + std::min(tileSize, width - x)));
        }
        tiles[static_cast<size_t>(y / tileSize) * columns + x / tileSize] = farthest;
      }
    }
  }

  [[nodiscard]] bool hides(const BoundingBox& bounds) const
  {
    if (tiles.empty() || bounds.isEmpty()) return false;
    const auto pixel = [](double ndc, int extent) {
      return std::clamp(static_cast<int>((std::clamp(ndc, -1.0, 1.0) + 1.0) * 0.5 * extent),
                        0, extent - 1);
    };
    const int x0 = pixel(bounds.min().x(), width) / tileSize;
    const int x1 = pixel(bounds.max().x(), width) / tileSize;
    const int y0 = pixel(bounds.min().y(), height) / tileSize;
    const int y1 = pixel(bounds.max().y(), height) / tileSize;
    const int columns = (width + tileSize - 1) / tileSize;
    // Ties can contribute color through GL_EQUAL. Allow for depth quantization.
    const double nearest = (bounds.min().z() + 1.0) * 0.5 - depthPadding;
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (!(tiles[static_cast<size_t>(y) * columns + x] < nearest)) return false;
      }
    }
    return true;
  }

private:
  static constexpr int tileSize = 8;
  int width = 0;
  int height = 0;
  double depthPadding = 0.0;
  std::vector<float> pixels;
  std::vector<float> tiles;
};

class OpenCSGVBOPrim : public OpenCSG::Primitive
{
public:
  OpenCSGVBOPrim(OpenCSG::Operation operation, unsigned int convexity,
                 std::unique_ptr<VertexState> vertex_state, const BoundingBox& bounds)
    : OpenCSG::Primitive(operation, convexity), vertex_state(std::move(vertex_state)), bounds(bounds)
  {
  }

  BoundingBox updateScreenBounds(const Eigen::Matrix4d& viewProjection, const Vector2d& padding)
  {
    // OpenCSG uses these bounds to restrict its stencil and depth passes. Keep the
    // full depth range and fall back to the full viewport around the eye plane.
    if (bounds.isEmpty() || !bounds.min().allFinite() || !bounds.max().allFinite()) {
      setBoundingBox(-1, -1, -1, 1, 1, 1);
      return {Vector3d::Constant(-1), Vector3d::Constant(1)};
    }

    // VBO positions and OpenGL matrix operations use single precision. Bound
    // their roundoff, including loss of precision for models far from the origin.
    const Eigen::Vector4d extent(bounds.min().cwiseAbs().cwiseMax(bounds.max()).homogeneous());
    const Eigen::Vector4d error =
      (8 * std::numeric_limits<float>::epsilon()) * (viewProjection.cwiseAbs() * extent);
    BoundingBox projected;
    for (unsigned int i = 0; i < 8; ++i) {
      const Vector3d corner = bounds.corner(static_cast<BoundingBox::CornerType>(i));
      const Eigen::Vector4d clip = viewProjection * corner.homogeneous();
      if (!clip.allFinite() || !(clip.w() > error.w())) {
        setBoundingBox(-1, -1, -1, 1, 1, 1);
        return {Vector3d::Constant(-1), Vector3d::Constant(1)};
      }

      const Vector3d point = clip.head<3>() / clip.w();
      const Vector3d margin =
        (error.head<3>() + point.cwiseAbs() * error.w()) / (clip.w() - error.w());
      projected.extend(point - margin);
      projected.extend(point + margin);
    }

    if (!projected.min().allFinite() || !projected.max().allFinite()) {
      setBoundingBox(-1, -1, -1, 1, 1, 1);
      return {Vector3d::Constant(-1), Vector3d::Constant(1)};
    }
    projected.min().head<2>() -= padding;
    projected.max().head<2>() += padding;
    const Vector2d lo = projected.min().head<2>().cwiseMax(-1).cwiseMin(1);
    const Vector2d hi = projected.max().head<2>().cwiseMax(-1).cwiseMin(1);
    setBoundingBox(lo.x(), lo.y(), -1, hi.x(), hi.y(), 1);
    return projected;
  }

  void render() override
  {
    if (vertex_state != nullptr) {
      vertex_state->draw();
    } else {
      if (OpenSCAD::debug != "") PRINTD("OpenCSGVBOPrim vertex_state was null");
    }
  }

private:
  const std::unique_ptr<VertexState> vertex_state;
  const BoundingBox bounds;
};

// Primitive for drawing using OpenCSG
// Makes a copy of the given VertexState enabling just unlit/uncolored vertex
// rendering
std::unique_ptr<OpenCSGVBOPrim> createVBOPrimitive(
  const std::shared_ptr<OpenCSGVertexState>& vertex_state, const OpenCSG::Operation operation,
  const unsigned int convexity, const BoundingBox& bounds)
{
  std::unique_ptr<VertexState> opencsg_vs = std::make_unique<VertexState>(
    vertex_state->drawMode(), vertex_state->drawSize(), vertex_state->drawType(),
    vertex_state->drawOffset(), vertex_state->elementOffset(), vertex_state->verticesVBO(),
    vertex_state->elementsVBO());
  // First two glBegin entries are the vertex position calls
  opencsg_vs->glBegin().insert(opencsg_vs->glBegin().begin(), vertex_state->glBegin().begin(),
                               vertex_state->glBegin().begin() + 2);
  // First glEnd entry is the disable vertex position call
  opencsg_vs->glEnd().insert(opencsg_vs->glEnd().begin(), vertex_state->glEnd().begin(),
                             vertex_state->glEnd().begin() + 1);

  return std::make_unique<OpenCSGVBOPrim>(operation, convexity, std::move(opencsg_vs), bounds);
}

}  // namespace

#endif  // ENABLE_OPENCSG

OpenCSGRenderer::OpenCSGRenderer(std::shared_ptr<CSGProducts> root_products,
                                 std::shared_ptr<CSGProducts> highlights_products,
                                 std::shared_ptr<CSGProducts> background_products)
  : root_products_(std::move(root_products)),
    highlights_products_(std::move(highlights_products)),
    background_products_(std::move(background_products))
{
  opencsg_vertex_shader_code_ = ShaderUtils::loadShaderSource("OpenCSG.vert");
}

void OpenCSGRenderer::prepare(const ShaderUtils::ShaderInfo *shaderinfo)
{
  if (!prepared_) {
    if (root_products_) {
      createCSGVBOProducts(*root_products_, false, false, shaderinfo);
    }
    if (background_products_) {
      createCSGVBOProducts(*background_products_, false, true, shaderinfo);
    }
    if (highlights_products_) {
      createCSGVBOProducts(*highlights_products_, true, false, shaderinfo);
    }
    prepared_ = true;
  }
#ifdef ENABLE_MANIFOLD
  if (cachedRenderer_ && colorscheme_ == previewMeshColorScheme_) cachedRenderer_->prepare(shaderinfo);
#endif
}

#if defined(ENABLE_OPENCSG) && defined(ENABLE_MANIFOLD)
std::function<std::shared_ptr<CSGProducts>()> OpenCSGRenderer::previewMeshJob()
{
  if (previewMeshLifetime_) return {};
  previewMeshLifetime_ = std::make_shared<int>(0);
  if (!root_products_ || root_products_->products.size() < PreviewMeshCache::minGroupSize || highlights_products_ ||
      background_products_) return {};

  Color4f material, cutout;
  if (!getColorSchemeColor(ColorMode::MATERIAL, material) || material.a() != 1.0f ||
      !getColorSchemeColor(ColorMode::CUTOUT, cutout)) return {};
  previewMeshColorScheme_ = colorscheme_;
  return [source = root_products_, material, cutout,
          lifetime = std::weak_ptr<void>(previewMeshLifetime_)] {
    try {
      return PreviewMeshCache::bake(*source, material, cutout, lifetime);
    } catch (...) {
      // A speculative cache failure must leave the regular preview available.
      return std::shared_ptr<CSGProducts>{};
    }
  };
}

void OpenCSGRenderer::installPreviewMesh(std::shared_ptr<CSGProducts> products)
{
  if (!products || colorscheme_ != previewMeshColorScheme_) return;
  PRINTDB("Cached repeated preview geometry: %1$d products instead of %2$d",
          products->products.size() % root_products_->products.size());
  cachedRenderer_ = std::make_unique<OpenCSGRenderer>(std::move(products), nullptr, nullptr);
  cachedRenderer_->setColorScheme(*colorscheme_);
}
#endif

void OpenCSGRenderer::draw(bool showedges, const ShaderUtils::ShaderInfo *shaderinfo) const
{
#ifdef ENABLE_OPENCSG
#ifdef ENABLE_MANIFOLD
  if (cachedRenderer_ && colorscheme_ == previewMeshColorScheme_ && !showedges &&
      (!shaderinfo || shaderinfo->type != ShaderUtils::ShaderType::SELECT_RENDERING)) {
    cachedRenderer_->draw(false, shaderinfo);
    return;
  }
#endif
  // Only use shader if select rendering or showedges
  bool enable_shader =
    shaderinfo && ((shaderinfo->type == ShaderUtils::ShaderType::EDGE_RENDERING && showedges) ||
                   shaderinfo->type == ShaderUtils::ShaderType::SELECT_RENDERING);

  Eigen::Matrix4d viewProjection;
  Vector2d padding;
  bool haveViewProjection = false;
  std::array<GLint, 4> viewport{};
  // A readback synchronizes the GPU. Only consider it for a large batch and a
  // broad opaque surface that was drawn before any translucent products.
  bool canCaptureDepth = !enable_shader && occlusionCandidates_ >= 48;
  OpaqueDepth opaqueDepth;
  size_t remainingCandidates = occlusionCandidates_;
  GLint depthBits = 0;

  if (skippedFirstProduct_) glDepthFunc(GL_LEQUAL);

  for (const auto& product : vertex_state_containers_) {
    canCaptureDepth = canCaptureDepth && product->opaque;
    const bool usesOpenCSG = product->primitives().size() > 1;
    if (usesOpenCSG && product->opaque) --remainingCandidates;
    const bool depthSource = canCaptureDepth && product->primitives().size() == 1 &&
                             remainingCandidates >= 32;
    BoundingBox drawBounds;
    if (usesOpenCSG || depthSource) {
      if (!haveViewProjection) {
        Eigen::Matrix4d modelView, projection;
        glGetDoublev(GL_MODELVIEW_MATRIX, modelView.data());
        glGetDoublev(GL_PROJECTION_MATRIX, projection.data());
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        viewProjection = projection * modelView;
        // Round outwards through OpenCSG's integer scissor conversion.
        padding = Vector2d(4.0 / std::max(1, viewport[2]), 4.0 / std::max(1, viewport[3]));
        haveViewProjection = true;
        if (canCaptureDepth) {
          GLint samples = -1;
          GLint packBuffer = 0;
          std::array<GLdouble, 2> depthRange{};
          glGetIntegerv(GL_SAMPLES, &samples);
          glGetIntegerv(GL_DEPTH_BITS, &depthBits);
#ifdef USE_GLEW
          if (GLEW_VERSION_2_1) glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &packBuffer);
#else
          if (GLAD_GL_VERSION_2_1) glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &packBuffer);
#endif
          glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &packBuffer);
          glGetDoublev(GL_DEPTH_RANGE, depthRange.data());
          canCaptureDepth = samples == 0 && depthBits >= 16 && packBuffer == 0 && depthRange[0] == 0.0 &&
                            depthRange[1] == 1.0 && viewport[0] == 0 && viewport[1] == 0 &&
                            viewport[2] > 0 && viewport[3] > 0 &&
                            static_cast<size_t>(viewport[2]) * viewport[3] <= 2200000;
        }
      }
      for (auto *primitive : product->primitives()) {
        const auto bounds =
          static_cast<OpenCSGVBOPrim *>(primitive)->updateScreenBounds(viewProjection, padding);
        if (depthSource || opaqueDepth.available()) drawBounds.extend(bounds);
      }
    }
    if (usesOpenCSG) {
      // Include cutters in drawBounds: their color passes can contribute at
      // coincident depths, even outside the mathematical CSG result.
      if (product->opaque && opaqueDepth.hides(drawBounds)) continue;
#if OPENCSG_VERSION >= 0x0180
      if (enable_shader) OpenCSG::setVertexShader(opencsg_vertex_shader_code_);
      else OpenCSG::setVertexShader({});
#endif
      GL_CHECKD(OpenCSG::render(product->primitives()));
      GL_TRACE0("glDepthFunc(GL_EQUAL)");
      GL_CHECKD(glDepthFunc(GL_EQUAL));
    }

    if (enable_shader) {
      GL_TRACE("glUseProgram(%d)", shaderinfo->resource.shader_program);
      GL_CHECKD(glUseProgram(shaderinfo->resource.shader_program));
      VBOUtils::shader_attribs_enable(*shaderinfo);
    }

    for (const auto& vertex_state : product->states()) {
      // Specify ID color if we're using select rendering
      if (shaderinfo && shaderinfo->type == ShaderUtils::ShaderType::SELECT_RENDERING) {
        if (const auto csg_vs = std::dynamic_pointer_cast<OpenCSGVertexState>(vertex_state)) {
          GL_TRACE("glUniform3f(%d, %f, %f, %f)",
                   shaderinfo->uniforms.at("frag_idcolor") %
                     (((csg_vs->csgObjectIndex() >> 0) & 0xff) / 255.0f) %
                     (((csg_vs->csgObjectIndex() >> 8) & 0xff) / 255.0f) %
                     (((csg_vs->csgObjectIndex() >> 16) & 0xff) / 255.0f));
          GL_CHECKD(glUniform3f(shaderinfo->uniforms.at("frag_idcolor"),
                                ((csg_vs->csgObjectIndex() >> 0) & 0xff) / 255.0f,
                                ((csg_vs->csgObjectIndex() >> 8) & 0xff) / 255.0f,
                                ((csg_vs->csgObjectIndex() >> 16) & 0xff) / 255.0f));
        }
      }
      const auto shader_vs = std::dynamic_pointer_cast<VBOShaderVertexState>(vertex_state);
      if (!shader_vs || (showedges && shader_vs)) {
        vertex_state->draw();
      }
    }

    if (enable_shader) {
      GL_TRACE0("glUseProgram(0)");
      GL_CHECKD(glUseProgram(0));
      VBOUtils::shader_attribs_disable(*shaderinfo);
    }
    GL_TRACE0("glDepthFunc(GL_LEQUAL)");
    GL_CHECKD(glDepthFunc(GL_LEQUAL));

    if (depthSource && canCaptureDepth) {
      drawBounds.clamp({Vector3d::Constant(-1), Vector3d::Constant(1)});
      // Require at least one eighth of the viewport (whose NDC area is 4).
      if (!drawBounds.isEmpty() && drawBounds.sizes().head<2>().prod() >= 0.5) {
        opaqueDepth.capture(viewport[2], viewport[3], depthBits);
        canCaptureDepth = false;
      }
    }
  }
#endif  // ENABLE_OPENCSG
}

// Turn the CSGProducts into VBOs
// Will create one (temporary) VertexArray and one VBO(+EBO) per product
// The VBO will be utilized to render multiple objects with correct state
// management. In the future, we could use a VBO per primitive and potentially
// reuse VBOs, but that requires some more careful state management.
// Note: This function can be called multiple times for different products.
// Each call will add to vbo_vertex_products_.
void OpenCSGRenderer::createCSGVBOProducts(const CSGProducts& products, bool highlight_mode,
                                           bool background_mode,
                                           const ShaderUtils::ShaderInfo *shaderinfo)
{
#ifdef ENABLE_OPENCSG
  bool enable_barycentric = true;
  const bool mainScene = !highlight_mode && !background_mode;
  for (const auto& product : products.products) {
    if (mainScene && fullySubtractedByBox(product)) {
      if (vertex_state_containers_.empty()) skippedFirstProduct_ = true;
      continue;
    }
    std::unique_ptr<OpenCSGVBOProduct> vertex_state_container = std::make_unique<OpenCSGVBOProduct>();
    vertex_state_container->opaque = mainScene;

    Color4f last_color;
    auto& vertex_states = vertex_state_container->states();
    VBOBuilder vbo_builder(std::make_unique<OpenCSGVertexStateFactory>(), *vertex_state_container.get());
    vbo_builder.addSurfaceData();
    vbo_builder.writeSurface();
    vbo_builder.addShaderData();  // Always enable barycentric coordinates

    size_t num_vertices = 0;
    for (const auto& csgobj : product.intersections) {
      if (csgobj.leaf->polyset) {
        num_vertices += calcNumVertices(csgobj);
      }
    }
    for (const auto& csgobj : product.subtractions) {
      if (csgobj.leaf->polyset) {
        num_vertices += calcNumVertices(csgobj);
      }
    }

    vbo_builder.allocateBuffers(num_vertices);

    for (const auto& csgobj : product.intersections) {
      if (csgobj.leaf->polyset) {
        const Color4f& c = csgobj.leaf->color;

        ColorMode colormode = ColorMode::NONE;
        bool override_color;
        if (highlight_mode) {
          colormode = ColorMode::HIGHLIGHT;
          override_color = true;
        } else if (background_mode) {
          colormode = ColorMode::BACKGROUND;
          override_color = true;
        } else {
          colormode = ColorMode::MATERIAL;
          override_color = c.isValid();
        }

        Color4f color;
        if (getShaderColor(colormode, c, color)) {
          last_color = color;
        }

        if (last_color.a() != 1.0f ||
            (!override_color && !csgobj.leaf->polyset->color_indices.empty() &&
             std::any_of(csgobj.leaf->polyset->colors.begin(), csgobj.leaf->polyset->colors.end(),
                         [](const Color4f& faceColor) { return faceColor.isValid() && faceColor.a() != 1.0f; }))) {
          vertex_state_container->opaque = false;
        }

        add_shader_pointers(vbo_builder, shaderinfo);

        if (color.a() == 1.0f) {
          // object is opaque, draw normally
          vbo_builder.create_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, last_color,
                                     enable_barycentric, override_color);
          if (const auto csg_vs = std::dynamic_pointer_cast<OpenCSGVertexState>(vertex_states.back())) {
            csg_vs->setCsgObjectIndex(csgobj.leaf->index);
            vertex_state_container->addPrimitive(
              createVBOPrimitive(csg_vs, OpenCSG::Intersection, csgobj.leaf->polyset->getConvexity(),
                                 csgobj.leaf->getBoundingBox()));
          }
        } else {
          // object is transparent, so draw rear faces first.  Issue #1496
          std::shared_ptr<VertexState> cull = std::make_shared<VertexState>();
          cull->glBegin().emplace_back([]() {
            GL_TRACE0("glEnable(GL_CULL_FACE)");
            glEnable(GL_CULL_FACE);
            GL_TRACE0("glCullFace(GL_FRONT)");
            glCullFace(GL_FRONT);
          });
          vertex_states.emplace_back(std::move(cull));

          vbo_builder.create_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, last_color,
                                     enable_barycentric, override_color);
          if (const auto csg_vs = std::dynamic_pointer_cast<OpenCSGVertexState>(vertex_states.back())) {
            csg_vs->setCsgObjectIndex(csgobj.leaf->index);

            vertex_state_container->addPrimitive(
              createVBOPrimitive(csg_vs, OpenCSG::Intersection, csgobj.leaf->polyset->getConvexity(),
                                 csgobj.leaf->getBoundingBox()));

            cull = std::make_shared<VertexState>();
            cull->glBegin().emplace_back([]() {
              GL_TRACE0("glCullFace(GL_BACK)");
              glCullFace(GL_BACK);
            });
            vertex_states.emplace_back(std::move(cull));

            vertex_states.emplace_back(csg_vs);

            cull = std::make_shared<VertexState>();
            cull->glEnd().emplace_back([]() {
              GL_TRACE0("glDisable(GL_CULL_FACE)");
              glDisable(GL_CULL_FACE);
            });
            vertex_states.emplace_back(std::move(cull));
          } else {
            assert(false && "Intersection surface state was nullptr");
          }
        }
      }
    }

    for (const auto& csgobj : product.subtractions) {
      if (csgobj.leaf->polyset) {
        const Color4f& c = csgobj.leaf->color;
        ColorMode colormode = ColorMode::NONE;
        bool override_color;
        if (highlight_mode) {
          colormode = ColorMode::HIGHLIGHT;
          override_color = true;
        } else if (background_mode) {
          colormode = ColorMode::BACKGROUND;
          override_color = true;
        } else {
          colormode = ColorMode::CUTOUT;
          override_color = true;
        }

        Color4f color;
        if (getShaderColor(colormode, c, color)) {
          last_color = color;
        }
        if (last_color.a() != 1.0f) vertex_state_container->opaque = false;

        add_shader_pointers(vbo_builder, shaderinfo);

        // negative objects should only render rear faces
        std::shared_ptr<VertexState> cull = std::make_shared<VertexState>();
        cull->glBegin().emplace_back([]() {
          GL_TRACE0("glEnable(GL_CULL_FACE)");
          GL_CHECKD(glEnable(GL_CULL_FACE));
        });
        cull->glBegin().emplace_back([]() {
          GL_TRACE0("glCullFace(GL_FRONT)");
          GL_CHECKD(glCullFace(GL_FRONT));
        });
        vertex_states.emplace_back(std::move(cull));
        Transform3d tmp = csgobj.leaf->matrix;
        if (csgobj.leaf->polyset->getDimension() == 2) {
          // Scale 2D negative objects 10% in the Z direction to avoid z fighting
          tmp *= Eigen::Scaling(1.0, 1.0, 1.1);
        }
        vbo_builder.create_surface(*csgobj.leaf->polyset, tmp, last_color, enable_barycentric,
                                   override_color);
        if (const auto csg_vs = std::dynamic_pointer_cast<OpenCSGVertexState>(vertex_states.back())) {
          csg_vs->setCsgObjectIndex(csgobj.leaf->index);
          vertex_state_container->addPrimitive(
            createVBOPrimitive(csg_vs, OpenCSG::Subtraction, csgobj.leaf->polyset->getConvexity(),
                               tmp * csgobj.leaf->polyset->getBoundingBox()));
        } else {
          assert(false && "Subtraction surface state was nullptr");
        }

        cull = std::make_shared<VertexState>();
        cull->glEnd().emplace_back([]() {
          GL_TRACE0("glDisable(GL_CULL_FACE)");
          GL_CHECKD(glDisable(GL_CULL_FACE));
        });
        vertex_states.emplace_back(std::move(cull));
      }
    }

    if (Feature::ExperimentalVxORenderersIndexing.is_enabled()) {
      GL_TRACE0("glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)");
      GL_CHECKD(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    }
    GL_TRACE0("glBindBuffer(GL_ARRAY_BUFFER, 0)");
    GL_CHECKD(glBindBuffer(GL_ARRAY_BUFFER, 0));

    vbo_builder.createInterleavedVBOs();
    if (vertex_state_container->opaque && vertex_state_container->primitives().size() > 1) {
      ++occlusionCandidates_;
    }
    vertex_state_containers_.push_back(std::move(vertex_state_container));
  }
#endif  // ENABLE_OPENCSG
}

BoundingBox OpenCSGRenderer::getBoundingBox() const
{
  BoundingBox bbox;
  if (root_products_) {
    bbox = root_products_->getBoundingBox();
  }
  if (highlights_products_) {
    bbox.extend(highlights_products_->getBoundingBox());
  }
  if (background_products_) {
    bbox.extend(background_products_->getBoundingBox());
  }

  return bbox;
}

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

#include <double-conversion/double-conversion.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ios>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry/Geometry.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetUtils.h"
#include "geometry/linalg.h"
#include "io/export.h"
#include "utils/parallel.h"
#include "utils/printutils.h"

#ifdef ENABLE_MANIFOLD
#include "geometry/manifold/ManifoldGeometry.h"
#endif
#ifdef ENABLE_CGAL
#include "geometry/cgal/CGALNefGeometry.h"
#include "geometry/cgal/cgalutils.h"
#endif

namespace {
/* Define values for double-conversion library. */
#define DC_BUFFER_SIZE (128)
#define DC_FLAGS (double_conversion::DoubleToStringConverter::UNIQUE_ZERO)
#define DC_INF NULL  // Only finite values in STL outputs!
#define DC_NAN NULL
#define DC_EXP 'e'
#define DC_DECIMAL_LOW_EXP (-6)
#define DC_DECIMAL_HIGH_EXP (21)
#define DC_MAX_LEADING_ZEROES (5)
#define DC_MAX_TRAILING_ZEROES (0)

std::string toString(const Vector3d& v)
{
  const double_conversion::DoubleToStringConverter dc(DC_FLAGS, DC_INF, DC_NAN, DC_EXP,
                                                      DC_DECIMAL_LOW_EXP, DC_DECIMAL_HIGH_EXP,
                                                      DC_MAX_LEADING_ZEROES, DC_MAX_TRAILING_ZEROES);

  char buffer[DC_BUFFER_SIZE];

  double_conversion::StringBuilder builder(buffer, DC_BUFFER_SIZE);
  dc.ToShortest(v[0], &builder);
  builder.AddCharacter(' ');
  dc.ToShortest(v[1], &builder);
  builder.AddCharacter(' ');
  dc.ToShortest(v[2], &builder);
  builder.Finalize();

  return buffer;
}

Vector3d facet_normal(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2)
{
  // Tessellation already eliminated these cases.
  assert(p0 != p1 && p0 != p2 && p1 != p2);
  auto normal = (p1 - p0).cross(p2 - p0);
  if (!normal.isZero(0)) normal.normalize();
  return normal;
}

constexpr size_t facetsPerChunk = 4096;
constexpr size_t facetsPerBatch = 16 * facetsPerChunk;
using BinaryFacet = std::array<char, 50>;

BinaryFacet encode_binary_facet(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2)
{
  static_assert(sizeof(float) == 4, "Need 32 bit float");
  static constexpr uint16_t test = 0x0001;
  static const bool isLittleEndian = *reinterpret_cast<const char *>(&test) == 1;
  const auto normal = facet_normal(p0, p1, p2);
  std::array<float, 12> coords;
  size_t offset = 0;
  for (const auto *point : {&normal, &p0, &p1, &p2}) {
    for (const auto value : {(*point)[0], (*point)[1], (*point)[2]}) coords[offset++] = value;
  }
  BinaryFacet facet{};
  std::memcpy(facet.data(), coords.data(), coords.size() * sizeof(float));
  if (!isLittleEndian) {
    for (size_t i = 0; i < coords.size(); ++i) {
      const auto first = facet.begin() + i * sizeof(float);
      std::reverse(first, first + sizeof(float));
    }
  }
  return facet;
}

void append_binary_facets(const PolySet& ps, std::ostream& output)
{
  static_assert(sizeof(BinaryFacet) == 50, "STL facets are exactly 50 bytes");
  const size_t total = ps.indices.size();
  // For small exports, a single buffered write costs less than starting a team.
  const size_t chunkSize = total >= 4 * facetsPerBatch ? facetsPerChunk : facetsPerBatch;
  std::vector<BinaryFacet> buffer(std::min(facetsPerBatch, total));
  for (size_t offset = 0; offset < total; offset += facetsPerBatch) {
    const size_t count = std::min(facetsPerBatch, total - offset);
    OpenSCADParallel::forEachChunk(count, chunkSize, [&](size_t first, size_t last) {
      for (size_t i = first; i < last; ++i) {
        const auto& face = ps.indices[offset + i];
        buffer[i] = encode_binary_facet(ps.vertices[face[0]], ps.vertices[face[1]], ps.vertices[face[2]]);
      }
    });
    output.write(reinterpret_cast<const char *>(buffer.data()), count * sizeof(BinaryFacet));
  }
}

void append_ascii_facets(const PolySet& ps, std::ostream& output)
{
  std::vector<std::string> vertexStrings(ps.vertices.size());
  OpenSCADParallel::forEachChunk(ps.vertices.size(), facetsPerChunk, [&](size_t first, size_t last) {
    for (size_t i = first; i < last; ++i) vertexStrings[i] = toString(ps.vertices[i]);
  });

  std::array<std::string, facetsPerBatch / facetsPerChunk> buffers;
  const size_t total = ps.indices.size();
  for (size_t offset = 0; offset < total; offset += facetsPerBatch) {
    const size_t count = std::min(facetsPerBatch, total - offset);
    OpenSCADParallel::forEachChunk(count, facetsPerChunk, [&](size_t first, size_t last) {
      auto& buffer = buffers[first / facetsPerChunk];
      buffer.clear();
      buffer.reserve(256 * (last - first));
      for (size_t i = first; i < last; ++i) {
        const auto& t = ps.indices[offset + i];
        const auto normal = facet_normal(ps.vertices[t[0]], ps.vertices[t[1]], ps.vertices[t[2]]);
        assert(vertexStrings[t[0]] != vertexStrings[t[1]] && vertexStrings[t[0]] != vertexStrings[t[2]] &&
               vertexStrings[t[1]] != vertexStrings[t[2]]);
        buffer += "  facet normal ";
        buffer += toString(normal);
        buffer += "\n    outer loop\n";
        for (const auto index : {t[0], t[1], t[2]}) {
          buffer += "      vertex ";
          buffer += vertexStrings[index];
          buffer += '\n';
        }
        buffer += "    endloop\n  endfacet\n";
      }
    });
    for (size_t first = 0; first < count; first += facetsPerChunk) {
      const auto& buffer = buffers[first / facetsPerChunk];
      output.write(buffer.data(), buffer.size());
    }
  }
}

uint64_t append_stl(const std::shared_ptr<const PolySet>& polyset, std::ostream& output, bool binary)
{
  std::shared_ptr<const PolySet> ps = polyset;
  if (!ps->isTriangular()) ps = PolySetUtils::tessellate_faces(*ps);
  ps = createSortedPolySet(*ps);

  if (binary) append_binary_facets(*ps, output);
  else append_ascii_facets(*ps, output);

  return ps->indices.size();
}

#ifdef ENABLE_CGAL
/*!
    Saves the current 3D CGAL Nef polyhedron as STL to the given file.
    The file must be open.
 */
uint64_t append_stl(const CGALNefGeometry& root_N, std::ostream& output, bool binary)
{
  uint64_t triangle_count = 0;
  if (!root_N.p3->is_simple()) {
    LOG(message_group::Export_Warning,
        "Exported object may not be a valid 2-manifold and may need repair");
  }

  if (const std::shared_ptr<PolySet> ps = CGALUtils::createPolySetFromNefPolyhedron3(*(root_N.p3))) {
    triangle_count += append_stl(ps, output, binary);
  } else {
    LOG(message_group::Export_Error, "Nef->PolySet failed");
  }

  return triangle_count;
}

#endif  // ENABLE_CGAL

#ifdef ENABLE_MANIFOLD
/*!
   Saves the current 3D Manifold geometry as STL to the given file.
   The file must be open.
 */
uint64_t append_stl(const ManifoldGeometry& mani, std::ostream& output, bool binary)
{
  uint64_t triangle_count = 0;
  if (!mani.isManifold()) {
    LOG(message_group::Export_Warning,
        "Exported object may not be a valid 2-manifold and may need repair");
  }

  const auto ps = mani.toPolySet();
  if (ps) {
    triangle_count += append_stl(ps, output, binary);
  } else {
    LOG(message_group::Export_Error, "Manifold->PolySet failed");
  }

  return triangle_count;
}
#endif  // ENABLE_MANIFOLD

uint64_t append_stl(const std::shared_ptr<const Geometry>& geom, std::ostream& output, bool binary)
{
  uint64_t triangle_count = 0;
  if (const auto geomlist = std::dynamic_pointer_cast<const GeometryList>(geom)) {
    for (const Geometry::GeometryItem& item : geomlist->getChildren()) {
      triangle_count += append_stl(item.second, output, binary);
    }
  } else if (const auto ps = std::dynamic_pointer_cast<const PolySet>(geom)) {
    triangle_count += append_stl(ps, output, binary);
#ifdef ENABLE_CGAL
  } else if (const auto N = std::dynamic_pointer_cast<const CGALNefGeometry>(geom)) {
    triangle_count += append_stl(*N, output, binary);
#endif
#ifdef ENABLE_MANIFOLD
  } else if (const auto mani = std::dynamic_pointer_cast<const ManifoldGeometry>(geom)) {
    triangle_count += append_stl(*mani, output, binary);
#endif
  } else if (std::dynamic_pointer_cast<const Polygon2d>(geom)) {  // NOLINT(bugprone-branch-clone)
    assert(false && "Unsupported file format");
  } else {  // NOLINT(bugprone-branch-clone)
    assert(false && "Not implemented");
  }

  return triangle_count;
}

}  // namespace

void export_stl(const std::shared_ptr<const Geometry>& geom, std::ostream& output, bool binary)
{
  // FIXME: In lazy union mode, should we export multiple solids?
  if (binary) {
    std::ostringstream buffer;  // Using a memory buffer
    char header[80] = "OpenSCAD Model\n";
    buffer.write(header, sizeof(header));

    // Placeholder for triangle count
    uint32_t triangle_count = 0;
    char tmp_triangle_count[4] = {0, 0, 0, 0};
    buffer.write(tmp_triangle_count, 4);

    // Writing triangles and counting them
    triangle_count = append_stl(geom, buffer, binary);

    if (triangle_count > 4294967295) {
      LOG(message_group::Export_Error,
          "Triangle count exceeded 4294967295, so the STL file is not valid");
    }

    // Updating the triangle count in the buffer
    char triangle_count_bytes[4] = {static_cast<char>(triangle_count & 0xff),
                                    static_cast<char>((triangle_count >> 8) & 0xff),
                                    static_cast<char>((triangle_count >> 16) & 0xff),
                                    static_cast<char>((triangle_count >> 24) & 0xff)};
    buffer.seekp(80, std::ios_base::beg);
    buffer.write(triangle_count_bytes, 4);

    // Flushing the buffer to the output stream
    output << buffer.str();

  } else {
    // ASCII mode: Write directly to the output stream
    output << "solid OpenSCAD_Model\n";
    append_stl(geom, output, binary);
    output << "endsolid OpenSCAD_Model\n";
  }
}

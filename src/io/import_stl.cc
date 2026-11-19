#include "io/import.h"

#include <boost/predef.h>

#include <algorithm>
#include <array>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/AST.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetBuilder.h"
#include "io/ascii_tokens.h"
#include "utils/printutils.h"

#if !defined(BOOST_ENDIAN_BIG_BYTE_AVAILABLE) && !defined(BOOST_ENDIAN_LITTLE_BYTE_AVAILABLE)
#error Byte order undefined or unknown. Currently only BOOST_ENDIAN_BIG_BYTE and BOOST_ENDIAN_LITTLE_BYTE are supported.
#endif

namespace {
constexpr size_t facetSize = 50;
constexpr size_t normalSize = 12;
constexpr std::string_view whitespace = " \t\n\r\f\v";

void appendBinaryFacet(PolySetBuilder& builder, const char *facet)
{
  static_assert(sizeof(float) == 4, "STL coordinates require 32-bit floats");
  std::array<float, 9> coordinates;
  std::memcpy(coordinates.data(), facet + normalSize, coordinates.size() * sizeof(float));
#if BOOST_ENDIAN_BIG_BYTE
  for (auto& coordinate : coordinates) {
    auto *bytes = reinterpret_cast<char *>(&coordinate);
    std::reverse(bytes, bytes + sizeof(coordinate));
  }
#endif
  builder.appendPolygon({Vector3d(coordinates[0], coordinates[1], coordinates[2]),
                         Vector3d(coordinates[3], coordinates[4], coordinates[5]),
                         Vector3d(coordinates[6], coordinates[7], coordinates[8])});
}

bool isVertexLine(std::string_view line, std::array<std::string_view, 3>& coordinates)
{
  if (takeAsciiToken(line, whitespace) != "vertex") return false;
  for (auto& coordinate : coordinates) {
    coordinate = takeAsciiToken(line, whitespace);
    if (coordinate.empty()) return false;
  }
  return takeAsciiToken(line, whitespace).empty();
}
}  // namespace

std::unique_ptr<PolySet> import_stl(const std::string& filename, const Location& loc)
{
  // Open file and position at the end
  std::ifstream f(std::filesystem::u8path(filename), std::ios::in | std::ios::binary | std::ios::ate);
  if (!f.good()) {
    LOG(message_group::Warning, "Can't open import file '%1$s', import() at line %2$d", filename,
        loc.firstLine());
    return PolySet::createEmpty();
  }

  uint32_t facenum = 0;
  bool binary = false;
  const std::streampos file_size = f.tellg();
  f.seekg(80);
  if (f.good() && !f.eof()) {
    std::array<unsigned char, 4> count{};
    f.read(reinterpret_cast<char *>(count.data()), count.size());
    for (size_t i = 0; i < count.size(); ++i) facenum |= uint32_t(count[i]) << (8 * i);
    if (file_size == static_cast<std::streamoff>(84 + uint64_t(facetSize) * facenum)) {
      binary = true;
    }
  }
  if (!binary) facenum = 0;
  PolySetBuilder builder(0, facenum);
  f.seekg(0);

  char data[5];
  f.read(data, 5);
  if (!binary && !f.eof() && f.good() && !memcmp(data, "solid", 5)) {
    int i = 0;
    int lineno = 1;
    std::array<std::array<double, 3>, 3> vdata;
    std::string line;

    auto AsciiError = [&](const auto& errstr) {
      LOG(message_group::Error, loc, "", "STL line %1$s, %2$s line '%3$s' importing file '%4$s'", lineno,
          errstr, line, filename);
    };

    std::getline(f, line);
    bool reached_end = false;
    while (!f.eof()) {
      lineno++;
      std::getline(f, line);
      boost::trim(line);
      std::array<std::string_view, 3> coordinates;
      if (line.empty() || line.compare(0, 5, "solid") == 0 || line.compare(0, 5, "facet") == 0 ||
          line.compare(0, 8, "endfacet") == 0) {
        continue;
      } else if (line == "outer loop") {
        i = 0;
        continue;
      } else if (line == "endloop") {
        if (i < 3) {
          AsciiError("missing vertex");
        }
        continue;
      } else if (line.compare(0, 8, "endsolid") == 0) {
        reached_end = true;
        break;
      } else if (i >= 3) {
        AsciiError("extra vertex");
        return PolySet::createEmpty();
      } else if (isVertexLine(line, coordinates)) {
        try {
          for (int v = 0; v < 3; ++v) {
            vdata.at(i).at(v) = boost::lexical_cast<double>(coordinates[v].data(), coordinates[v].size());
          }
          if (++i == 3) {
            builder.beginPolygon(3);
            for (int j = 0; j < 3; j++) {
              builder.addVertex(Vector3d(vdata[j][0], vdata[j][1], vdata[j][2]));
            }
          }
        } catch (const boost::bad_lexical_cast& blc) {
          AsciiError("can't parse vertex");
          return PolySet::createEmpty();
        }
      }
    }
    if (!reached_end) {
      AsciiError("file incomplete");
    }
  } else if (binary && !f.eof() && f.good()) {
    try {
      f.ignore(80 - 5 + 4);
      std::vector<char> facets(4096 * facetSize);
      while (f) {
        f.read(facets.data(), facets.size());
        const size_t count = static_cast<size_t>(f.gcount()) / facetSize;
        for (size_t i = 0; i < count; ++i) {
          appendBinaryFacet(builder, facets.data() + facetSize * i);
        }
      }
      if (f.bad() || !f.eof()) throw std::ios_base::failure("facet data truncated");
    } catch (const std::ios_base::failure& ex) {
      int64_t offset = -1;
      try {
        offset = f.tellg();
      } catch (...) {
      }
      if (offset < 0) {
        LOG(message_group::Error, loc, "", "Binary STL '%1$s' error: %3$s", filename, ex.what());
      } else {
        LOG(message_group::Error, loc, "", "Binary STL '%1$s' error at byte %2$s: %3$s", filename,
            offset, ex.what());
      }
      return PolySet::createEmpty();
    }
  } else {
    LOG(message_group::Error, loc, "", "STL format not recognized in '%1$s'.", filename);
    return PolySet::createEmpty();
  }
  return builder.build();
}

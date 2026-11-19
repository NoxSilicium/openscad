#pragma once

#include <string_view>

inline std::string_view takeAsciiToken(std::string_view& text, std::string_view separators)
{
  const auto first = text.find_first_not_of(separators);
  if (first == std::string_view::npos) {
    text = {};
    return {};
  }
  text.remove_prefix(first);
  const auto end = text.find_first_of(separators);
  const auto token = text.substr(0, end);
  text.remove_prefix(token.size());
  return token;
}

/** @file
 * @ingroup utils
 */

#pragma once

#include <boost/none_t.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

namespace beam {

template <class T>
class optional : public boost::optional<T> {
public:
  optional() : boost::optional<T>() {}

  optional(T const& v) : boost::optional<T>(v) {}

  optional(T&& v) : boost::optional<T>(v) {}

  optional(bool condition, T const& v) : boost::optional<T>(condition, v) {}

  constexpr bool has_value() const {
    bool val = boost::optional<T>::operator bool();
    if (val) {
      return true;
    } else {
      return false;
    }
  }
};

template<class T>
using opt = optional<T>;

} // namespace beam

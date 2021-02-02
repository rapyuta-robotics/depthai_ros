#pragma once

/**
 * SPDX-License-Identifier: MPLv2
 *
 * Copyright (c) 2020- Rapyuta Robotics Inc.
 * Copyright (c) 2019-2020 Rakuten Inc. (MPL v2)
 */

#include <type_traits>

namespace rr {
namespace detail {
#if defined(__cpp_lib_void_t) && (__cpp_lib_void_t >= 201411)
using std::void_t;
#else
template <class...>
using void_t = void;
#endif
}  // namespace detail

namespace macro_helpers {
#define ADD_HELPER_VALUE_TEMPLATE(Name, Query) \
    template <class Query>                     \
    inline constexpr auto Name##_v = Name<Query>::value

#define ADD_HELPER_VALUE_SFINAE(Name, Variable, Query) \
    template <class Query>                             \
    using Name = std::enable_if_t<Variable<Query>, bool>

#define SETUP_HAS_PUBLIC_TYPE_ALIAS(Name)                                                                        \
    template <class Query, class = void>                                                                         \
    struct has_public_type_alias_##Name : public std::false_type {};                                             \
    template <class Query>                                                                                       \
    struct has_public_type_alias_##Name<Query, detail::void_t<typename Query::Name>> : public std::true_type {}; \
    ADD_HELPER_VALUE_TEMPLATE(has_public_type_alias_##Name, Query);                                              \
    ADD_HELPER_VALUE_SFINAE(HasPublicTypeAlias_##Name, has_public_type_alias_##Name##_v, Query)
}  // namespace macro_helpers
}  // namespace rr

/**
 * @file APSCore.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Core utils for the APS libraries.
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (C) 2023 Apogeo Space srl
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Refer to the attached LICENSE.md file for more information.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef AS_CORE_ARDUINO_HPP
#define AS_CORE_ARDUINO_HPP

namespace ApogeoSpace
{
    namespace Core
    {
        /**
         * @brief Objects derived from this class cannot be copied or moved.
         * Used mainly for hw drivers and the like.
         */
        class Singleton
        {
            public:
            Singleton() = default;
            Singleton(const Singleton& rhs) = delete;
            Singleton& operator=(const Singleton& rhs) = delete;
            Singleton(Singleton&& rhs) = delete;
            Singleton& operator=(Singleton&& rhs) = delete;
        };

        /**
         * @brief Minimal implementation of some std functionality used throughout the library.
         * This addition was made necessary by the lack of such constructs in the libraries
         * shipped with the AVR core.
         * 
         */
        namespace ustd
        {
            namespace type_traits
            {
                template <class T, T v>
                struct integral_constant
                {
                    static constexpr T value = v;
                    using value_type = T;
                    using type = integral_constant;
                    constexpr operator value_type() const noexcept { return value; }
                    constexpr value_type operator()() const noexcept { return value; }
                };
                using true_type = integral_constant<bool, true>;
                using false_type = integral_constant<bool, false>;
                template <typename T, typename U>
                struct is_same : false_type
                {
                };
                template <typename T>
                struct is_same<T, T> : true_type
                {
                };

                template <typename T>
                struct remove_const
                {
                    typedef T type;
                };

                template <typename T>
                struct remove_const<const T>
                {
                    typedef T type;
                };

                template <typename T>
                struct remove_volatile
                {
                    typedef T type;
                };

                template <typename T>
                struct remove_volatile<volatile T>
                {
                    typedef T type;
                };

                template <typename T>
                struct remove_cv : remove_const<typename remove_volatile<T>::type>
                {
                };

                template <typename T>
                struct is_unqualified_pointer
                {
                    enum
                    {
                        value = false
                    };
                };

                template <typename T>
                struct is_unqualified_pointer<T *>
                {
                    enum
                    {
                        value = true
                    };
                };

                template <typename T>
                struct is_pointer_type : is_unqualified_pointer<typename remove_cv<T>::type>
                {
                };

                template <typename T>
                constexpr bool is_pointer(const T &)
                {
                    return is_pointer_type<T>::value;
                }

                template <bool... b> struct static_all_of;

                // do recursion if the first argument is true
                template <bool... tail>
                struct static_all_of<true, tail...> : static_all_of<tail...> {};

                // end recursion if first argument is false
                template <bool... tail>
                struct static_all_of<false, tail...> : false_type {};

                // end recursion if no more arguments need to be processed
                template <> struct static_all_of<> : true_type {};
            }
        }
    }
}

#endif

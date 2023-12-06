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
    }
}

#endif

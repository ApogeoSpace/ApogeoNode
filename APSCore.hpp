/**
 * @file APSCore.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Core utils for the APS libraries.
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (c) 2023
 * 
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
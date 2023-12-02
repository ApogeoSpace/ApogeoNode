/**
 * @file GNSS.hpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Simple GNSS library to parse GGA data and extract basic geopositioning information
 * @version 0.1
 * @date 2023-12-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef AS_GNSS_ARDUINO_HPP
#define AS_GNSS_ARDUINO_HPP
#include <stdint.h>
#include "Arduino.h"
#include "APSCrypto.hpp"

#define TEST_GGA 0
#define DEBUG_PRINT 1

#if DEBUG_PRINT
#define DPRINT(x, ...) Serial.print(x, ##__VA_ARGS__)
#define DPRINTLN(x, ...) Serial.println(x, ##__VA_ARGS__)
#else
#define DPRINT(x, ...) (void)(x)
#define DPRINTLN(x, ...) (void)(x)
#endif

#ifdef SE
#undef SE
#endif

namespace ApogeoSpace
{
    namespace GNSS
    {
        /**
         * @brief Representation of cardinal points
         *
         */
        enum class Cardinal : uint8_t
        {
            N = 0,
            NE,
            E,
            SE,
            S,
            SW,
            W,
            NW,
            Invalid
        };

        /**
         * @brief Type of fix that the receiver could acquire
         *
         */
        enum class FixMode : uint8_t
        {
            NA = 0,   /** Not available */
            GPS = 1,  /** GPS SPS Mode */
            DGPS = 2, /** Differential GPS SPS Mode */
            DRM = 6   /** Dead reckoning mode */
        };

        /**
         * @brief Representation and utilities for parsing NMEA GGA strings
         *
         */
        struct GGAData
        {

            enum class Fields : uint8_t
            {
                UTC = 0,
                Latitude,
                N_S_Ind,
                Longitude,
                E_W_Ind,
                Fix,
                Satellites,
                HDOP,
                Altitude,
                Altitude_Units,
                GeoidSep,
                GSE_Units,
                AgeDiffRef,
                DiffReffID,
                Checksum,
                Invalid
            };

            GGAData();
            void Init();

            /**
             * @brief Get the integer part of a fractional number.
             * Given the restricted width of the output number, it's mainly used for
             * @param val
             * @return int16_t
             */
            int16_t GetIntPart(float val) const;
            int16_t GetFracPart(float val) const;

            void Print() const;

            void FeedString(char *buffer, const size_t len);

            bool IsValid() const
            {
                return Valid;
            }

            uint32_t UTC;
            float Latitude;
            Cardinal LatDir;
            float Longitude;
            Cardinal LonDir;
            FixMode Fix;
            uint8_t Satellites;
            float Altitude;

        private:
            bool Valid;
        };

        struct RMCData
        {

            enum class Fields : uint8_t
            {
                UTC = 0,
                Status,
                Latitude,
                N_S_Ind,
                Longitude,
                E_W_Ind,
                Speed,
                Course,
                Date,
                MagVar,
                MagVarDir,
                Mode,
                Checksum,
                Invalid
            };

            RMCData();
            void Init();

            void FeedString(char *buffer, const size_t len);

            bool IsValid() const
            {
                return Valid;
            }

            bool IsFixAvailable() const
            {
                return DataStatus;
            }

            uint32_t UTC;
            bool DataStatus;
            float Latitude;
            Cardinal LatDir;
            float Longitude;
            Cardinal LonDir;

        private:
            bool Valid;
        };

        class GNSS
        {
        public:
            GNSS(Stream &port) : data_provider{port},
                                 Buffering{false}
            {
            }
            GNSS(const GNSS &rhs) = delete;
            GNSS &operator=(const GNSS &rhs) = delete;
            GNSS(GNSS &&rhs) = delete;
            GNSS &operator=(GNSS &&rhs) = delete;

            inline float GetLatitude() const
            {
                return RMCData.Latitude;
            }
            inline Cardinal GetLatitudeDir() const
            {
                return RMCData.LatDir;
            }
            inline float GetLongitude() const
            {
                return RMCData.Longitude;
            }
            inline Cardinal GetLongitudeDir() const
            {
                return RMCData.LonDir;
            }
            inline float GetAltitude() const
            {
                return GGAData.Altitude;
            }
            inline uint8_t GetSatCount() const
            {
                return GGAData.Satellites;
            }

            void ParseBuffer();

            void Process(void);
            bool IsFixAvailable(void) const
            {
                return RMCData.IsFixAvailable();
            }

            void FillPayload(Crypto::Payload_t &payload, Crypto::Timestamp_t &ts) const;

        private:
            struct RMCData RMCData;
            struct GGAData GGAData;

            static constexpr uint16_t kBufferSize{128};
            uint16_t BufferingIdx;
            bool Buffering;
            char Buffer[kBufferSize];
            uint8_t PreviousChar;
            Stream &data_provider;
        };
    }
}

using APS_GNSS = ApogeoSpace::GNSS::GNSS;

#endif //AS_GNSS_ARDUINO_HPP
/**
 * @file GNSS.cpp
 * @author Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Simple GNSS library to parse GGA data and extract basic geopositioning information
 * @version 0.1
 * @date 2023-12-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "GNSS.hpp"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
using namespace ApogeoSpace::GNSS;

GGAData::GGAData()
{
  Init();
  ;
}

void GGAData::Init()
{
  Valid = false;
  UTC = 0U;
  Latitude = 0.0f;
  LatDir = Cardinal::Invalid;
  Longitude = 0.0f;
  LonDir = Cardinal::Invalid;
  Fix = FixMode::NA;
  Satellites = 0U;
  Altitude = 0.0f;
}

int16_t GGAData::GetIntPart(float val) const
{
  return static_cast<int16_t>(val);
}
int16_t GGAData::GetFracPart(float val) const
{
  auto frac = GetIntPart(1000 * (val - GetIntPart(val)));
}

void GGAData::Print() const
{
  char buf[30];
  sprintf(buf, "UTC: %ld\n", UTC);
  DPRINT(buf);

  sprintf(buf, "Latitude: %d.%d ° %s\n", GetIntPart(Latitude), GetFracPart(Latitude),
          LatDir == Cardinal::Invalid ? "" : LatDir == Cardinal::N ? "N"
                                                                   : "S");
  DPRINT(buf);

  sprintf(buf, "Longitude: %d.%d ° %s\n", GetIntPart(Longitude), GetFracPart(Longitude),
          LonDir == Cardinal::Invalid ? "" : LonDir == Cardinal::E ? "E"
                                                                   : "W");
  DPRINT(buf);

  const char *fix = "N/A";
  switch (Fix)
  {
  case FixMode::GPS:
    fix = "GPS";
    break;
  case FixMode::DGPS:
    fix = "DGPS";
    break;
  case FixMode::DRM:
    fix = "DRM";
    break;
  default:
    break;
  }

  sprintf(buf, "Fix: %s\n", fix);
  DPRINT(buf);

  sprintf(buf, "Sats: %d\n", Satellites);
  DPRINT(buf);

  sprintf(buf, "Altitude: %d.%d m\n", GetIntPart(Altitude), GetFracPart(Altitude));
  DPRINT(buf);
}

void GGAData::FeedString(char *buffer, const size_t len)
{

  if (not buffer or not len)
  {
    return;
  }

  // Parse the string
  // Check type of string
  auto token = strchr(buffer, ',');
  if (not token)
  {
    DPRINTLN("Error in parsing");
    return;
  }
  if (not strstr(buffer, "GGA"))
  {
    // Not a GGA string
    return;
  }

  Init();

#if 0 and DEBUG_PRINT
  for(size_t idx{0}; idx <len; idx++)
  {
    Serial.print(buffer[idx]);
  }
  Serial.println("");
#endif

  Fields state = Fields::UTC;
  auto prev_token = token;
  while (token = strchr(prev_token + 1, ','))
  {
    if (state == Fields::Invalid)
    {
      // Somehow more fields were found than those that are actually supported. The buffer must be corrupted.
      // DPRINTLN("Critical error! Fields in excess were found");
      return;
    }
    if (token - prev_token == 1)
    {
      // Empty field
      // DPRINTLN("Empty token");
    }
    else
    {
      char *internal_token = prev_token + 1;
      switch (state)
      {

      case Fields::UTC:
      {
        auto dot_pos = strchr(internal_token, '.');
        UTC = strtol(internal_token, nullptr, 10);
        UTC += 1000 * strtol(dot_pos + 1, nullptr, 10);
      }
      break;
      case Fields::Latitude:
        // TODO: consider 2xstrtol + strchr
        Latitude = atof(internal_token) / 100;
        break;
      case Fields::N_S_Ind:
        if (*internal_token == 'N')
        {
          LatDir = Cardinal::N;
        }
        else if (*internal_token == 'S')
        {
          LatDir = Cardinal::S;
        }
        else
        {
          LatDir = Cardinal::Invalid;
        }
        break;
      case Fields::Longitude:
        // TODO: consider 2xstrtol + strchr
        Longitude = atof(internal_token) / 100;
        break;
      case Fields::E_W_Ind:
        if (*internal_token == 'E')
        {
          LonDir = Cardinal::E;
        }
        else if (*internal_token == 'W')
        {
          LonDir = Cardinal::W;
        }
        else
        {
          LonDir = Cardinal::Invalid;
        }
        break;
      case Fields::Fix:

        switch (*internal_token)
        {
        case '1':
          Fix = FixMode::GPS;
          break;
        case '2':
          Fix = FixMode::DGPS;
          break;
        case '6':
          Fix = FixMode::DRM;
          break;
        default:
          Fix = FixMode::NA;
          break;
        }
        break;

      case Fields::Altitude:
        // TODO: consider 2xstrtol + strchr

        Altitude = atof(internal_token);

        break;

      case Fields::Satellites:
        Satellites = static_cast<uint8_t>(strtol(internal_token, nullptr, 10));
        break;
      default:
        break;
      }
    }

    state = static_cast<Fields>(static_cast<uint8_t>(state) + 1);
    prev_token = token;
  }
  Valid = true;
}

RMCData::RMCData()
{
  Init();
}

void RMCData::Init()
{
  Valid = false;
  UTC = 0U;
  DataStatus = false;
  Latitude = 0.0f;
  LatDir = Cardinal::Invalid;
  Longitude = 0.0f;
  LonDir = Cardinal::Invalid;
}

void RMCData::FeedString(char *buffer, const size_t len)
{

  if (not buffer or not len)
  {
    return;
  }

  // Parse the string
  // Check type of string
  auto token = strchr(buffer, ',');
  if (not token)
  {
    DPRINTLN("Error in parsing");
    return;
  }
  if (not strstr(buffer, "RMC"))
  {
    // Not an RMC string
    return;
  }

  Init();

  Fields state = Fields::UTC;
  auto prev_token = token;
  while (token = strchr(prev_token + 1, ','))
  {
    if (state == Fields::Invalid)
    {
      // Somehow more fields were found than those that are actually supported. The buffer must be corrupted.
      // DPRINTLN("Critical error! Fields in excess were found");
      return;
    }
    if (token - prev_token == 1)
    {
      // Empty field
      // DPRINTLN("Empty token");
    }
    else
    {
      char *internal_token = prev_token + 1;
      switch (state)
      {

      case Fields::UTC:
      {
        auto dot_pos = strchr(internal_token, '.');
        UTC = 1000 * strtol(internal_token, nullptr, 10);
        UTC += strtol(dot_pos + 1, nullptr, 10);
      }

      case Fields::Status:
      {
        DataStatus = *internal_token == 'A';
      }

      break;
      case Fields::Latitude:
        // TODO: consider 2xstrtol + strchr
        Latitude = atof(internal_token) / 100;
        break;
      case Fields::N_S_Ind:
        if (*internal_token == 'N')
        {
          LatDir = Cardinal::N;
        }
        else if (*internal_token == 'S')
        {
          LatDir = Cardinal::S;
        }
        else
        {
          LatDir = Cardinal::Invalid;
        }
        break;
      case Fields::Longitude:
        // TODO: consider 2xstrtol + strchr
        Longitude = atof(internal_token) / 100;
        break;
      case Fields::E_W_Ind:
        if (*internal_token == 'E')
        {
          LonDir = Cardinal::E;
        }
        else if (*internal_token == 'W')
        {
          LonDir = Cardinal::W;
        }
        else
        {
          LonDir = Cardinal::Invalid;
        }
        break;
      default:
        break;
      }
    }

    state = static_cast<Fields>(static_cast<uint8_t>(state) + 1);
    prev_token = token;
  }
  Valid = true;
}

void GNSS::ParseBuffer()
{
#if TEST_GGA
  char Buffer[]{"$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5E"};
  size_t BufferingIdx{strlen(Buffer)};
#endif
  GGAData.FeedString(Buffer, BufferingIdx);
  RMCData.FeedString(Buffer, BufferingIdx);
#if TEST_GGA
  if (not GGAData.IsValid())
  {
    DPRINTLN("GGA Data could not be parsed");
  }
  else
  {
    DPRINTLN("Correctly parsed GGA data");
    if (GGAData.Fix != FixMode::NA)
    {
      GGAData.Print();
    }
    else
    {
      DPRINTLN("No fix.");
    }
  }
#endif
}

void GNSS::Process(void)
{
  while (data_provider.available() > 0)
  {
    char c = data_provider.read();
    // DPRINT(c);

    if (not Buffering and c == '$')
    {
      PreviousChar = 0;
      Buffering = true;
      BufferingIdx = 0;
    }
    else
    {

      Buffer[BufferingIdx] = c;
      // Check for termination
      if (c == '\n' and PreviousChar == '\r')
      {
        Buffering = false;
        PreviousChar = 0;
        ParseBuffer();
      }
      else
      {
        if (BufferingIdx == kBufferSize - 1)
        {
          BufferingIdx = 0;
          Buffering = false;
        }
        else
        {
          BufferingIdx++;
          PreviousChar = c;
        }
      }
    }
  }
}

void GNSS::FillPayload(Crypto::Payload_t &payload, Crypto::Timestamp_t &ts) const
{
  struct
  {
    uint8_t fix : 1;    // 0: n/a, 1: fix
    uint8_t latdir : 1; // 0: N, 1: S
    uint8_t londir : 1; // 0: E, 1: W
    uint8_t sats : 5;   // Up to 31 sats
    uint32_t lat : 32;
    uint32_t lon : 32;
  } status;

  status.fix = IsFixAvailable();
  status.latdir = GetLatitudeDir() == Cardinal::S;
  status.londir = GetLongitudeDir() == Cardinal::W;
  status.sats = GetSatCount() > 31 ? 31 : GetSatCount();
  status.lat = *reinterpret_cast<const uint32_t *>(&RMCData.Latitude);
  status.lon = *reinterpret_cast<const uint32_t *>(&RMCData.Longitude);
  memcpy(payload, &status, sizeof(status));
  memcpy(ts, &RMCData.UTC, sizeof(ts));
}
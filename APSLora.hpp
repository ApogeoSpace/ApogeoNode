/**
 * @file APSLora.hpp
 * @author Gianfranco Manfredini (g.manfredini@apogeo.space), Donato Brusamento (d.brusamento@apogeo.space)
 * @brief Radio library for transmitting data within the Apogeo Space network.
 * This library also includes a prototype class for implementing radio transmitters other that the RFM98.
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) Apogeo Space 2023 TODO:put license details
 */

#include <stdint.h>
#include "APSCore.hpp"

#ifndef AS_LORA_ARDUINO_HPP
#define AS_LORA_ARDUINO_HPP

// Set this to 1 if you wish to add the preamble used in the RadioHead library
#ifndef RH_COMPATIBILITY_ENABLED
#define RH_COMPATIBILITY_ENABLED 0
#endif

namespace ApogeoSpace
{
	namespace LoRa
	{
		/**
		 * @brief Default carrier frequency for LoRa transmissions, in Hz.
		 * 
		 */
		constexpr uint32_t kDefaultFrequency{169425000};	// 169.425 Hz

		/**
		 * @brief Prototype class for a generic LoRa transmitter.
		 * If you need to implement a driver for a device other than those already supported by APS, you can
		 * derive it from this class and override the necessary functionalities.
		 * Implementing the basic functions in a virtual way will then allow for "plugging in" these new drivers
		 * directly in the APS applets
		 */
		class Transmitter : public Core::Singleton
		{
		public:
			/**
			 * @brief Construct a new Transmitter object.
			 */
			Transmitter() = default;

			/**
			 * @brief Initialize the transmitter.
			 *
			 * @param frequency frequency of operation in Hz
			 * @return true Initialization completed successfully.
			 * @return false An error occured during initialization.
			 */
			virtual bool Init(uint32_t frequency = 169425000) = 0;

			/**
			 * @brief Transmit a packet.
			 *
			 * @param src [in] input data to transmit
			 * @param length number of bytes to send
			 * @return true Transmission completed successfully.
			 * @return false Invalid data or error during transmission.
			 */
			virtual bool Transmit(const uint8_t *src, uint8_t length);

			/**
			 * @brief Check if transmission has completed
			 *
			 * @return true transmission process has completed
			 * @return false transmission process is still ongoing
			 */
			virtual bool IsTxDone() = 0;
		};

		/**
		 * @brief Driver for the HopeRF RFM98 LoRa transceiver
		 *
		 */
		class RFM98 : public Transmitter
		{

		public:
			/**
			 * @brief Construct a new RFM98 object
			 *
			 * @param rst pin for resetting the board
			 * @param ss slave select pin for the SPI bus
			 * @param d0 pin connected to DIO0 signal. Used for TX_READY (Cleared when leaving Tx)
			 */
			RFM98(uint8_t rst = kDefault_RST_Pin, uint8_t ss = kDefault_SS_Pin, uint8_t d0 = kDefault_D0_Pin);
			virtual bool Init(uint32_t frequency = kDefaultFrequency) override;
			virtual bool Transmit(const uint8_t *src, uint8_t length) override;
			virtual bool IsTxDone() override;

			static constexpr uint8_t kDefault_RST_Pin{5U};
			static constexpr uint8_t kDefault_SS_Pin{7U};
			static constexpr uint8_t kDefault_D0_Pin{2U};

		private:
			/**
			 * @brief Register address for SPI communication with the module
			 *
			 */
			enum class Register : uint8_t
			{
				RW = 0x00,				/** FIFO read/write access register */
				Mode = 0x01,			/** Mode register */
				FrequencyMsb = 0x06,	/** Frequency MSB register */
				FrequencyMid = 0x07,	/** Frequency middle byte register */
				FrequencyLsb = 0x08,	/** Frequency LSB register */
				PAConfiguration = 0x09, /** PA Configuration register */
				SPIFifoAddress = 0x0D,	/** SPI FIFO buffer pointer register */
				TxFifoAddress = 0x0E,	/** Tx FIFO pointer register */
				RxFifoAddress = 0x0F,	/** Rx FIFO pointer register */
				ModemCfg1 = 0x1D,		/** Modem configuration 1 register */
				ModemCfg2 = 0x1E,		/** Modem configuration 2 register */
				PreambleLenMSB = 0x20,	/** Preamble length MSB register */
				PreambleLenLSB = 0x21,	/** Preamble length LSB register */
				PayloadLen = 0x22,		/** Payload length register */
				ModemCfg3 = 0x26,		/** Modem configuration 3 register */
				DI0_1 = 0x40,			/** DI0 mapping configuration 1 register */
				DI0_2 = 0x41,			/** DI0 mapping configuration 2 register */
				PADac = 0x4D,			/** PA boost register */
				SimpleWrite = 0x80,
			};

			enum class OperatingMode : uint8_t
			{
				kSleep = 0x00,		  /** Sleep mode */
				kStandby = 0x01,	  /** Standby mode */
				kTx = 0x03,			  /** Transmit mode */
				kRxContinuous = 0x05, /** Continuous receive mode */
				kLora = 0x80		  /** Lora mode */
			};

			enum class BW : uint8_t
			{
				k7_8kHz = 0x00,
				k10_4kHz = 0x10,
				k15_6kHz = 0x20,
				k20_8kHz = 0x30,
				k31_25kHz = 0x40,
				k41_7kHz = 0x50,
				k62_5kHz = 0x60,
				k125kHz = 0x70,
				k250kHz = 0x80,
				k500kHz = 0x90,
			};

			enum class CR : uint8_t
			{
				k5 = 0x02,
				k6 = 0x04,
				k7 = 0x06,
				k8 = 0x08,
			};

			enum class SF : uint8_t
			{
				k64 = 0x60,
				k128 = 0x70,
				k256 = 0x80,
				k512 = 0x90,
				k1024 = 0xA0,
				k2048 = 0xB0,
				k4096 = 0xC0,
			};

			enum class HeaderMode : uint8_t
			{
				kExplicit = 0x00,
				kImplicit = 0x01,
			};

			enum class TransmitMode : uint8_t
			{
				kNormal = 0x00,
				kContinuous = 0x80
			};

			enum class CRCMode : uint8_t
			{
				kOff = 0x00,
				kOn = 0x04,
			};

			/**
			 * @brief Write a value to a register.
			 *
			 * @param address
			 * @param value
			 * @param verify if true, the value just sent will be read back to check that it's been written successfully
			 * @return true always if verify is not set, otherwise if value was written successfully
			 * @return false if verify is set, the value read from the device does not correspond to the one set
			 */
			bool WriteRegister(const Register address, const uint8_t value, const bool verify = false) const;

			/**
			 * @brief Read from a register
			 *
			 * @param address
			 * @return uint8_t value read
			 */
			uint8_t ReadRegister(const Register address) const;

			uint8_t RST_Pin;
			uint8_t SS_Pin;
			uint8_t D0_Pin;
			bool Initialized;
		};
	}
}

#endif // AS_LORA_ARDUINO_HPP
#ifndef PICO_INC_NVM_HPP_
#define PICO_INC_NVM_HPP_

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

/*! @brief Non-volatile memory (NVM) management class.
 * This class provides an interface for managing non-volatile memory on the Raspberry Pi Pico. It allows you to save and load user data to and from flash memory. The class handles erasing and programming the flash memory.
 * @warning The caller is responsible for ensuring that only one core accesses the flash at a time to prevent data corruption.
 */
class NVM
{
    size_t totalSize;
    size_t userDataSize;
    void* userData;

public:
    /*! @brief Construct a new NVM object.
     * @param nvmSize The size of the non-volatile memory.
     * @param userData Pointer to the user data.
     * @param userDataSize The size of the user data.
     */
    NVM(size_t nvmSize, void* userData, size_t userDataSize) : totalSize(nvmSize), userData(userData), userDataSize(userDataSize)
    {
    }

    /*! @brief Erase the entire non-volatile memory.
     * @warning Only one core should call this function to prevent data corruption. The calling core must lock out the other core before calling this function because flash cannot be read while it's being programmed or erased.
     */
    void erase(void)
    {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(PICO_FLASH_SIZE_BYTES - totalSize, totalSize);
        restore_interrupts(ints);
    }

    /*! @brief Load user data from non-volatile memory.
     */
    void load(void)
    {
        const uint8_t* flashData = reinterpret_cast<const uint8_t*>(XIP_BASE + PICO_FLASH_SIZE_BYTES - totalSize);
        std::memcpy(userData, flashData, userDataSize);
    }

    /*! @brief Save user data to non-volatile memory.
     * @warning Only one core should call this function to prevent data corruption. The calling core must lock out the other core before calling this function because flash cannot be read while it's being programmed or erased.
     */
    void save(void)
    {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(PICO_FLASH_SIZE_BYTES - totalSize, totalSize);
        flash_range_program(PICO_FLASH_SIZE_BYTES - totalSize, reinterpret_cast<const uint8_t*>(userData), userDataSize);
        restore_interrupts(ints);
        load();
    }
};

#endif // PICO_INC_NVM_HPP_

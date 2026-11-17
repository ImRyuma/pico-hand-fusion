// Minimal GOT (PLT) patching for an already loaded library of the current process.
#pragma once

#include <cstdint>
#include <functional>

namespace hf {

struct LibMapping {
    bool found = false;
    uintptr_t base = 0;   // address of the ELF header (mapping with file offset 0)
    uintptr_t start = 0;  // lowest mapped address of the file
    uintptr_t end = 0;    // highest mapped address of the file
};

// Looks up a library in /proc/self/maps by file name suffix (e.g. "/libfoo.so").
LibMapping findLibrary(const char* fileSuffix);

// Calls `fn` for every R_AARCH64_JUMP_SLOT relocation of the library loaded at `base`.
bool forEachJumpSlot(uintptr_t base, const std::function<void(const char* symbol, void** slot)>& fn);

// Writes a GOT slot, temporarily lifting the RELRO write protection.
bool writeSlot(void** slot, void* value);

}  // namespace hf

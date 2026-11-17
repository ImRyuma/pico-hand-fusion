#include "plthook.h"

#include <elf.h>
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace hf {

LibMapping findLibrary(const char* fileSuffix) {
    LibMapping m;
    FILE* f = fopen("/proc/self/maps", "re");
    if (!f) return m;
    char line[512];
    const size_t suffixLen = strlen(fileSuffix);
    while (fgets(line, sizeof(line), f)) {
        size_t len = strlen(line);
        while (len && (line[len - 1] == '\n' || line[len - 1] == ' ')) line[--len] = 0;
        if (len < suffixLen || strcmp(line + len - suffixLen, fileSuffix) != 0) continue;
        uintptr_t start = 0, end = 0, offset = 0;
        char perms[8] = {0};
        if (sscanf(line, "%" SCNxPTR "-%" SCNxPTR " %7s %" SCNxPTR, &start, &end, perms, &offset) != 4) continue;
        if (!m.found || start < m.start) m.start = start;
        if (end > m.end) m.end = end;
        if (offset == 0 && (!m.base || start < m.base)) m.base = start;
        m.found = true;
    }
    fclose(f);
    if (!m.base) m.found = false;
    return m;
}

bool forEachJumpSlot(uintptr_t base, const std::function<void(const char*, void**)>& fn) {
    auto* eh = reinterpret_cast<const Elf64_Ehdr*>(base);
    if (memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0 || eh->e_ident[EI_CLASS] != ELFCLASS64) return false;
    auto* ph = reinterpret_cast<const Elf64_Phdr*>(base + eh->e_phoff);

    // Load bias: the first PT_LOAD segment is mapped at `base`.
    uintptr_t minVaddr = UINTPTR_MAX;
    const Elf64_Phdr* dynPh = nullptr;
    for (int i = 0; i < eh->e_phnum; i++) {
        if (ph[i].p_type == PT_LOAD && ph[i].p_vaddr < minVaddr) minVaddr = ph[i].p_vaddr & ~(uintptr_t)(PAGE_SIZE - 1);
        if (ph[i].p_type == PT_DYNAMIC) dynPh = &ph[i];
    }
    if (!dynPh || minVaddr == UINTPTR_MAX) return false;
    const uintptr_t bias = base - minVaddr;

    // Bionic does not rewrite d_ptr values in memory: they stay link-time addresses.
    const Elf64_Rela* jmprel = nullptr;
    size_t pltrelsz = 0;
    const Elf64_Sym* symtab = nullptr;
    const char* strtab = nullptr;
    for (auto* d = reinterpret_cast<const Elf64_Dyn*>(bias + dynPh->p_vaddr); d->d_tag != DT_NULL; d++) {
        switch (d->d_tag) {
            case DT_JMPREL: jmprel = reinterpret_cast<const Elf64_Rela*>(bias + d->d_un.d_ptr); break;
            case DT_PLTRELSZ: pltrelsz = d->d_un.d_val; break;
            case DT_SYMTAB: symtab = reinterpret_cast<const Elf64_Sym*>(bias + d->d_un.d_ptr); break;
            case DT_STRTAB: strtab = reinterpret_cast<const char*>(bias + d->d_un.d_ptr); break;
            default: break;
        }
    }
    if (!jmprel || !symtab || !strtab) return false;

    for (size_t i = 0; i < pltrelsz / sizeof(Elf64_Rela); i++) {
        const Elf64_Rela& r = jmprel[i];
        if (ELF64_R_TYPE(r.r_info) != R_AARCH64_JUMP_SLOT) continue;
        const char* name = strtab + symtab[ELF64_R_SYM(r.r_info)].st_name;
        fn(name, reinterpret_cast<void**>(bias + r.r_offset));
    }
    return true;
}

// Current protection of the mapping containing `addr`, or -1.
static int protectionOf(uintptr_t addr) {
    FILE* f = fopen("/proc/self/maps", "re");
    if (!f) return -1;
    char line[512];
    int prot = -1;
    while (fgets(line, sizeof(line), f)) {
        uintptr_t start = 0, end = 0;
        char perms[8] = {0};
        if (sscanf(line, "%" SCNxPTR "-%" SCNxPTR " %7s", &start, &end, perms) != 3) continue;
        if (addr < start || addr >= end) continue;
        prot = (perms[0] == 'r' ? PROT_READ : 0) | (perms[1] == 'w' ? PROT_WRITE : 0) | (perms[2] == 'x' ? PROT_EXEC : 0);
        break;
    }
    fclose(f);
    return prot;
}

bool writeSlot(void** slot, void* value) {
    const uintptr_t page = reinterpret_cast<uintptr_t>(slot) & ~(uintptr_t)(PAGE_SIZE - 1);
    const int prot = protectionOf(page);
    if (prot < 0) return false;
    if (!(prot & PROT_WRITE) && mprotect(reinterpret_cast<void*>(page), PAGE_SIZE, prot | PROT_WRITE) != 0) return false;
    __atomic_store_n(slot, value, __ATOMIC_SEQ_CST);
    if (!(prot & PROT_WRITE)) mprotect(reinterpret_cast<void*>(page), PAGE_SIZE, prot);
    return true;
}

}  // namespace hf

/*
 * Experimental Qualcomm ARMv8 SCM and QSEE service transport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is a limited software monitor, not execution of vendor TrustZone code.
 * Unknown services and application operations fail explicitly. The variable
 * provider is an empty, read-only namespace, and supplies no authentication.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/main-loop.h"
#include "qemu/units.h"
#include "hw/core/cpu.h"
#include "hw/arm/qcom8998-scm.h"
#include "system/memory.h"
#include "cpu.h"
#include "trace.h"

/* Minimal SCM services for the experimental MSM8998 virt configuration. */
#define QCOM_SCM_OWNER_SIP 2
#define QCOM_SCM_OWNER_TZ_APPS 48
#define QCOM_SCM_OWNER_QSEE_OS 50
#define QCOM_SCM_INFO_IS_CALL_AVAIL 0x02000601
#define QCOM_SCM_INFO_GET_FEATURE_VERSION 0x02000603
#define QCOM_SCM_QSEE_REGISTER_LOG_BUFFER 0x32000106
#define QCOM_SCM_QSEE_APP_LOOKUP 0x32000103
#define QCOM_SCM_QSEE_APP_SEND 0x30000001
#define QCOM_SCM_RESTORE_SEC_CFG 0x02000c02
#define QCOM_SCM_DEVICE_MDSS 1
/* Exclude SMCCC fast-call and width bits. */
#define QCOM_SCM_FNID_MASK 0x3fffffff
#define QCOM_SCM_CALL_64BIT BIT(30)
#define QCOM_SCM_ARGS_ONE_VALUE 1
#define QCOM_SCM_ARGS_TWO_VALUES 2
#define QCOM_SCM_ARGS_RW_BUFFER_LENGTH 0x22 /* Two args; first is read/write. */
#define QCOM_SCM_ARGS_APP_SEND 0x885 /* app, request, size, response, size */
#define QCOM_SCM_MAX_ARGS 10
#define QCOM_QSEE_TRACE_BYTES 256
#define QCOM_SCM_EOPNOTSUPP (-4)
#define QCOM_SCM_EINVAL (-2)
#define QCOM_SCM_ENOMEM (-5)
#define QCOM_QSEE_APP_NAME_SIZE 32
#define QCOM_QSEE_RESULT_FAILURE UINT32_MAX
#define QCOM_QSEE_RESPONSE_APP_ID 0xee01
#define QCOM_QSEE_APP_WINSEC 1
#define QCOM_QSEE_APP_UEFISEC 2
#define QCOM_UEFI_GET_VARIABLE 0x8000
#define QCOM_UEFI_SET_VARIABLE 0x8001
#define QCOM_UEFI_NEXT_VARIABLE 0x8002
#define QCOM_UEFI_QUERY_INFO 0x8003
#define QCOM_UEFI_MAX_REQUEST (64 * KiB)
#define QCOM_UEFI_MAX_NAME_BYTES 2048
#define QCOM_UEFI_GUID_BYTES 16
#define QCOM_EFI_INVALID_PARAMETER 0x80000002U
#define QCOM_EFI_UNSUPPORTED 0x80000003U
#define QCOM_EFI_WRITE_PROTECTED 0x80000008U
#define QCOM_EFI_NOT_FOUND 0x8000000eU

/*
 * Little-endian UEFI secure-app wire layout, shared by Get/SetVariable.
 * The native Windows caller can exclude the appended name from fixed_bytes;
 * buffer bounds must therefore use the SMC request size, not that field.
 */
typedef struct QcomUefiVariableRequest {
    uint32_t command;
    uint32_t fixed_bytes;
    uint32_t name_offset;
    uint32_t name_bytes;
    uint32_t vendor_offset;
    uint32_t vendor_bytes;
} QcomUefiVariableRequest;

enum {
    QCOM_UEFI_COMMON_REPLY = 12,
    QCOM_UEFI_GET_REQUEST = 28,
    QCOM_UEFI_SET_REQUEST = 36,
    QCOM_UEFI_NEXT_REQUEST = 24,
    QCOM_UEFI_GET_REPLY = 24,
    QCOM_UEFI_SET_REPLY = 20,
    QCOM_UEFI_NEXT_REPLY = 28,
    QCOM_UEFI_INFO_REPLY = 40,
};

/* The caller holds the BQL while validating or accessing an SCM RAM buffer. */
static bool qcom8998_scm_ram_buffer(ARMCPU *cpu, hwaddr address,
                                  hwaddr size, bool write)
{
    hwaddr length = size;
    hwaddr offset;
    MemoryRegion *mr;

    if (!address || !size) {
        return false;
    }
    mr = address_space_translate(cpu_get_address_space(CPU(cpu), 0),
                                 address, &offset, &length, write,
                                 MEMTXATTRS_UNSPECIFIED);
    return memory_region_is_ram(mr) &&
           (!write || !memory_region_is_rom(mr)) && length == size;
}

static bool qcom8998_scm_arguments(ARMCPU *cpu, uint64_t *arguments)
{
    CPUARMState *env = &cpu->env;
    unsigned count = env->xregs[1] & 0xf;
    bool wide = env->xregs[0] & QCOM_SCM_CALL_64BIT;
    unsigned word_bytes = wide ? 8 : 4;
    unsigned direct = count > 4 ? 3 : count;
    uint8_t extra[(QCOM_SCM_MAX_ARGS - 3) * sizeof(uint64_t)];

    if (count > QCOM_SCM_MAX_ARGS) {
        return false;
    }
    for (unsigned i = 0; i < direct; i++) {
        arguments[i] = wide ? env->xregs[i + 2] :
                             (uint32_t)env->xregs[i + 2];
    }
    if (count > direct) {
        uint64_t address = wide ? env->xregs[5] : (uint32_t)env->xregs[5];
        unsigned bytes = (count - direct) * word_bytes;

        if (!qcom8998_scm_ram_buffer(cpu, address, bytes, false) ||
            address_space_read(cpu_get_address_space(CPU(cpu), 0), address,
                               MEMTXATTRS_UNSPECIFIED, extra, bytes) !=
                               MEMTX_OK) {
            return false;
        }
        for (unsigned i = direct; i < count; i++) {
            unsigned offset = (i - direct) * word_bytes;
            arguments[i] = wide ? ldq_le_p(extra + offset) :
                                  (uint32_t)ldl_le_p(extra + offset);
        }
    }
    return true;
}

static bool qcom8998_uefi_name_valid(const uint8_t *request, size_t bytes,
                                    uint32_t name_at, uint32_t name_bytes,
                                    uint32_t vendor_at, uint32_t vendor_bytes,
                                    size_t minimum)
{
    return name_bytes >= 2 && !(name_bytes & 1) &&
           name_bytes <= QCOM_UEFI_MAX_NAME_BYTES && name_at >= minimum &&
           name_at <= bytes && vendor_at >= minimum &&
           name_bytes <= bytes - name_at &&
           vendor_bytes == QCOM_UEFI_GUID_BYTES && vendor_at <= bytes &&
           vendor_bytes <= bytes - vendor_at &&
           lduw_le_p(request + name_at + name_bytes - 2) == 0;
}

static int32_t qcom8998_qsee_uefi(ARMCPU *cpu, const uint64_t *args)
{
    g_autofree uint8_t *request = NULL;
    uint8_t reply[QCOM_UEFI_INFO_REPLY] = { 0 };
    uint32_t command, fixed_bytes;
    uint32_t efi_status = QCOM_EFI_UNSUPPORTED;
    size_t reply_bytes = QCOM_UEFI_COMMON_REPLY;
    bool valid = true;
    AddressSpace *as = cpu_get_address_space(CPU(cpu), 0);

    if (args[2] < 8 || args[2] > QCOM_UEFI_MAX_REQUEST) {
        return QCOM_SCM_EINVAL;
    }
    request = g_try_malloc(args[2]);
    if (!request) {
        return QCOM_SCM_ENOMEM;
    }
    if (address_space_read(as, args[1], MEMTXATTRS_UNSPECIFIED,
                           request, args[2]) != MEMTX_OK) {
        return QCOM_SCM_EINVAL;
    }
    command = ldl_le_p(request);
    fixed_bytes = ldl_le_p(request + sizeof(uint32_t));

    switch (command) {
    case QCOM_UEFI_GET_VARIABLE:
    case QCOM_UEFI_SET_VARIABLE:
        {
            const QcomUefiVariableRequest *header = (const void *)request;
            size_t minimum = command == QCOM_UEFI_GET_VARIABLE ?
                             QCOM_UEFI_GET_REQUEST : QCOM_UEFI_SET_REQUEST;

            reply_bytes = command == QCOM_UEFI_GET_VARIABLE ?
                          QCOM_UEFI_GET_REPLY : QCOM_UEFI_SET_REPLY;
            valid = args[2] >= minimum && fixed_bytes >= minimum &&
                    fixed_bytes <= args[2];
            if (valid) {
                valid = qcom8998_uefi_name_valid(request, args[2],
                    ldl_le_p(&header->name_offset),
                    ldl_le_p(&header->name_bytes),
                    ldl_le_p(&header->vendor_offset),
                    ldl_le_p(&header->vendor_bytes), minimum);
            }
            if (valid && command == QCOM_UEFI_SET_VARIABLE) {
                uint32_t data_at = ldl_le_p(request + 7 * sizeof(uint32_t));
                uint32_t data_bytes = ldl_le_p(request + 8 * sizeof(uint32_t));

                valid = data_at <= args[2] && data_bytes <= args[2] - data_at &&
                        (!data_bytes || data_at >= minimum);
            }
            /*
             * This experimental provider has an empty, read-only variable
             * namespace. Report variable errors inside a valid QSEE reply,
             * rather than confusing them with an unsupported transport.
             */
            efi_status = command == QCOM_UEFI_GET_VARIABLE ?
                         QCOM_EFI_NOT_FOUND : QCOM_EFI_WRITE_PROTECTED;
        }
        break;
    case QCOM_UEFI_NEXT_VARIABLE:
        reply_bytes = QCOM_UEFI_NEXT_REPLY;
        valid = args[2] >= QCOM_UEFI_NEXT_REQUEST &&
                fixed_bytes >= QCOM_UEFI_NEXT_REQUEST && fixed_bytes <= args[2];
        if (valid) {
            /* GetNextVariable places GUID fields before the name fields. */
            const QcomUefiVariableRequest *header = (const void *)request;
            valid = qcom8998_uefi_name_valid(request, args[2],
                ldl_le_p(&header->vendor_offset),
                ldl_le_p(&header->vendor_bytes),
                ldl_le_p(&header->name_offset),
                ldl_le_p(&header->name_bytes), QCOM_UEFI_NEXT_REQUEST);
        }
        efi_status = QCOM_EFI_NOT_FOUND;
        break;
    case QCOM_UEFI_QUERY_INFO:
        reply_bytes = QCOM_UEFI_INFO_REPLY;
        valid = args[2] >= QCOM_UEFI_COMMON_REPLY &&
                fixed_bytes >= QCOM_UEFI_COMMON_REPLY && fixed_bytes <= args[2];
        break;
    }
    if (!valid) {
        efi_status = QCOM_EFI_INVALID_PARAMETER;
    }
    if (args[4] < reply_bytes) {
        return QCOM_SCM_EINVAL;
    }
    stl_le_p(reply, command);
    stl_le_p(reply + sizeof(uint32_t), reply_bytes);
    stl_le_p(reply + 2 * sizeof(uint32_t), efi_status);
    if (command == QCOM_UEFI_GET_VARIABLE) {
        /* Even an error response identifies where variable data would start. */
        stl_le_p(reply + 4 * sizeof(uint32_t), QCOM_UEFI_GET_REPLY);
    }
    trace_qcom8998_uefi_reply(command, efi_status, reply_bytes);
    return address_space_write(as, args[3], MEMTXATTRS_UNSPECIFIED,
                               reply, reply_bytes) == MEMTX_OK ?
           0 : QCOM_SCM_EINVAL;
}

bool qcom8998_smc(ARMCPU *cpu)
{
    CPUARMState *env = &cpu->env;
    uint32_t function = env->xregs[0] & QCOM_SCM_FNID_MASK;
    unsigned owner = function >> 24;
    uint64_t arginfo = env->xregs[1];
    uint64_t argument = env->xregs[2];
    uint64_t result[3] = { 0 };
    int32_t status = QCOM_SCM_EOPNOTSUPP;

    if (!is_a64(env)) {
        return false;
    }

    if (owner != QCOM_SCM_OWNER_SIP && owner != QCOM_SCM_OWNER_TZ_APPS &&
        owner != QCOM_SCM_OWNER_QSEE_OS) {
        /* Leave PSCI and architectural SMCCC calls to the normal handler. */
        return false;
    }

    BQL_LOCK_GUARD();

    switch (function) {
    case QCOM_SCM_INFO_IS_CALL_AVAIL:
        if (arginfo != QCOM_SCM_ARGS_ONE_VALUE) {
            status = QCOM_SCM_EINVAL;
            break;
        }
        status = 0;
        argument &= QCOM_SCM_FNID_MASK;
        result[0] = argument == QCOM_SCM_INFO_IS_CALL_AVAIL ||
                    argument == QCOM_SCM_INFO_GET_FEATURE_VERSION ||
                    argument == QCOM_SCM_RESTORE_SEC_CFG ||
                    argument == QCOM_SCM_QSEE_REGISTER_LOG_BUFFER ||
                    argument == QCOM_SCM_QSEE_APP_LOOKUP ||
                    argument == QCOM_SCM_QSEE_APP_SEND;
        break;
    case QCOM_SCM_INFO_GET_FEATURE_VERSION:
        if (arginfo != QCOM_SCM_ARGS_ONE_VALUE) {
            status = QCOM_SCM_EINVAL;
            break;
        }
        /*
         * No secure-storage feature is implemented. A zero feature version
         * must not advertise the capabilities of a real QSEE environment.
         */
        status = 0;
        break;
    case QCOM_SCM_RESTORE_SEC_CFG:
        /*
         * The MDSS probe already has its default non-secure access state:
         * it has no TrustZone firewall or protected DMA mappings to restore.
         * Model this operation only for MDSS, with its reserved argument 0.
         */
        status = arginfo == QCOM_SCM_ARGS_TWO_VALUES &&
                 argument == QCOM_SCM_DEVICE_MDSS && !env->xregs[3] ?
                 0 : QCOM_SCM_EINVAL;
        break;
    case QCOM_SCM_QSEE_REGISTER_LOG_BUFFER:
        if (arginfo != QCOM_SCM_ARGS_RW_BUFFER_LENGTH || !argument ||
            !env->xregs[3] || env->xregs[3] > UINT32_MAX) {
            status = QCOM_SCM_EINVAL;
            break;
        }
        /*
         * Accept only a writable diagnostic buffer. There is no secure-world
         * log producer yet, so registration never writes fabricated data.
         */
        status = qcom8998_scm_ram_buffer(cpu, argument, env->xregs[3], true) ?
                 0 : QCOM_SCM_EINVAL;
        break;
    case QCOM_SCM_QSEE_APP_LOOKUP:
        {
            char name[QCOM_QSEE_APP_NAME_SIZE + 1] = { 0 };
            uint64_t length = env->xregs[3];
            uint32_t app_id = 0;

            if (arginfo != QCOM_SCM_ARGS_RW_BUFFER_LENGTH ||
                length > QCOM_QSEE_APP_NAME_SIZE ||
                !qcom8998_scm_ram_buffer(cpu, argument, length, false) ||
                address_space_read(cpu_get_address_space(CPU(cpu), 0),
                                   argument, MEMTXATTRS_UNSPECIFIED,
                                   name, length) != MEMTX_OK) {
                status = QCOM_SCM_EINVAL;
                break;
            }
            /*
             * The wire length can exclude the terminating NUL. The local
             * extra byte keeps both bounded and padded names terminated.
             * Stable handles identify the model's service backends, not
             * vendor binaries loaded into a real TrustZone environment.
             * Unsupported application messages must still fail explicitly.
             */
            if (!strcmp(name, "qcom.tz.winsecapp")) {
                app_id = QCOM_QSEE_APP_WINSEC;
            } else if (!strcmp(name, "qcom.tz.uefisecapp")) {
                app_id = QCOM_QSEE_APP_UEFISEC;
            }
            status = 0;
            result[0] = app_id ? 0 : QCOM_QSEE_RESULT_FAILURE;
            result[1] = QCOM_QSEE_RESPONSE_APP_ID;
            result[2] = app_id;
            trace_qcom8998_qsee_app_lookup(name, app_id);
        }
        break;
    case QCOM_SCM_QSEE_APP_SEND:
        {
            uint64_t args[QCOM_SCM_MAX_ARGS] = { 0 };
            uint8_t request[QCOM_QSEE_TRACE_BYTES];
            unsigned bytes;

            if (arginfo != QCOM_SCM_ARGS_APP_SEND ||
                !qcom8998_scm_arguments(cpu, args)) {
                status = QCOM_SCM_EINVAL;
                break;
            }
            trace_qcom8998_qsee_message(args[0], args[1], args[2], args[3],
                                       args[4]);
            if (!qcom8998_scm_ram_buffer(cpu, args[1], args[2], false) ||
                !qcom8998_scm_ram_buffer(cpu, args[3], args[4], true)) {
                status = QCOM_SCM_EINVAL;
                break;
            }
            bytes = MIN(args[2], sizeof(request));
            if (address_space_read(cpu_get_address_space(CPU(cpu), 0),
                                   args[1], MEMTXATTRS_UNSPECIFIED,
                                   request, bytes) != MEMTX_OK) {
                status = QCOM_SCM_EINVAL;
                break;
            }
            for (unsigned offset = 0; offset + 4 <= bytes; offset += 4) {
                trace_qcom8998_qsee_request_word(args[0], offset,
                                                ldl_le_p(request + offset));
            }
            if (args[0] == QCOM_QSEE_APP_UEFISEC) {
                status = qcom8998_qsee_uefi(cpu, args);
                if (!status) {
                    result[1] = QCOM_QSEE_RESPONSE_APP_ID;
                    result[2] = args[0];
                }
            }
        }
        break;
    }

    trace_qcom8998_scm_call(env->xregs[0], arginfo, env->xregs[2],
                           env->xregs[3], env->xregs[4], status, result[0],
                           result[1], result[2]);
    env->xregs[0] = env->xregs[0] & QCOM_SCM_CALL_64BIT ? (int64_t)status :
                                                       (uint32_t)status;
    env->xregs[1] = result[0];
    env->xregs[2] = result[1];
    env->xregs[3] = result[2];
    env->xregs[6] = 0; /* No interrupted-call continuation context. */
    return true;
}

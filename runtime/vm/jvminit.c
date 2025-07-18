/*******************************************************************************
 * Copyright IBM Corp. and others 1991
 *
 * This program and the accompanying materials are made available under
 * the terms of the Eclipse Public License 2.0 which accompanies this
 * distribution and is available at https://www.eclipse.org/legal/epl-2.0/
 * or the Apache License, Version 2.0 which accompanies this distribution and
 * is available at https://www.apache.org/licenses/LICENSE-2.0.
 *
 * This Source Code may also be made available under the following
 * Secondary Licenses when the conditions for such availability set
 * forth in the Eclipse Public License, v. 2.0 are satisfied: GNU
 * General Public License, version 2 with the GNU Classpath
 * Exception [1] and GNU General Public License, version 2 with the
 * OpenJDK Assembly Exception [2].
 *
 * [1] https://www.gnu.org/software/classpath/license.html
 * [2] https://openjdk.org/legal/assembly-exception.html
 *
 * SPDX-License-Identifier: EPL-2.0 OR Apache-2.0 OR GPL-2.0-only WITH Classpath-exception-2.0 OR GPL-2.0-only WITH OpenJDK-assembly-exception-1.0
 *******************************************************************************/

#if defined(WIN32)
#include <windows.h>
#define USER32_DLL "user32.dll"
#include <WinSDKVer.h>

#if defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE)
#include <VersionHelpers.h>
#endif
#endif /* defined(WIN32) */

/* _GNU_SOURCE forces GLIBC_2.0 sscanf/vsscanf/fscanf for RHEL5 compatibility */
#if defined(LINUX) && !defined(J9ZTPF)
#define _GNU_SOURCE
#endif /* defined(LINUX) && !defined(J9ZTPF) */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <limits.h>
#include "util_api.h"
#if JAVA_SPEC_VERSION >= 16
#include "vm_internal.h"
#endif /* JAVA_SPEC_VERSION >= 16 */

#if !defined(stdout)
#define stdout NULL
#endif

#if !defined(WIN32)
/* Needed for JCL dependency on JVM to set SIGPIPE to SIG_IGN */
#include <signal.h>
#endif

#if defined(J9ZOS390)
#include "atoe.h"
#endif

#include "omrcfg.h"
#include "jvminitcommon.h"
#include "j9user.h"
#include "j9.h"
#include "omr.h"
#include "j9protos.h"
#include "jni.h"
#include "j9port.h"
#if defined(J9VM_OPT_SNAPSHOTS)
#include "j9port_generated.h"
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
#include "omrthread.h"
#include "j9consts.h"
#include "j9dump.h"
#include "jvminit.h"
#include "vm_api.h"
#include "vmaccess.h"
#include "vmhook_internal.h"
#include "mmhook.h"
#include "portsock.h"
#include "vmi.h"
#include "vm_internal.h"
#include "javaPriority.h"
#include "thread_api.h"
#include "jvmstackusage.h"
#include "omrsig.h"
#include "bcnames.h"
#include "jimagereader.h"
#include "vendor_version.h"
#include "omrlinkedlist.h"

#ifdef J9VM_OPT_ZIP_SUPPORT
#include "zip_api.h"
#endif

#define _UTE_STATIC_
#include "ut_j9vm.h"

#ifdef J9OS_I5
#include "Xj9I5OSDebug.H"
#endif

#ifdef J9ZTPF
#include <tpf/cujvm.h> /* for z/TPF OS VM exit hook: cjvm_jvm_shutdown_hook() */
#endif

/*#define JVMINIT_UNIT_TEST*/

#include "j2sever.h"

#include "locknursery.h"

#include "vmargs_api.h"
#include "rommeth.h"

#if defined(J9VM_OPT_SHARED_CLASSES)
#include "SCAbstractAPI.h"
#endif

#if JAVA_SPEC_VERSION >= 19
#include "omrutil.h"
#endif /* JAVA_SPEC_VERSION >= 19 */

#if defined(AIXPPC) && !defined(J9OS_I5)
#include <sys/systemcfg.h> /* for isPPC64bit() */
#endif /* AIXPPC && !J9OS_I5 */

J9_EXTERN_BUILDER_SYMBOL(cInterpreter);

/* Generic rounding macro - result is a UDATA */
#define ROUND_TO(granularity, number) (((UDATA)(number) + (granularity) - 1) & ~((UDATA)(granularity) - 1))

extern vmiError J9VMI_Initialize(J9JavaVM* vm);

#if (defined(J9VM_OPT_SIDECAR))
void sidecarInit (J9VMThread *mainThread);
#endif

typedef void (JNICALL *J9_EXIT_HANDLER_PROC)(jint);
typedef void (JNICALL *J9_ABORT_HANDLER_PROC)(void);

struct J9VMIgnoredOption {
	char *optionName;
	IDATA match;
};

typedef struct {
	void * vm_args;
	void * osMainThread;
	J9JavaVM * vm;
	J9JavaVM** globalJavaVM;
	UDATA j2seVersion;
	char* j2seRootDirectory;
	char* j9libvmDirectory;
} J9InitializeJavaVMArgs;

#define IGNORE_ME_STRING "_ignore_me"
#define SILENT_EXIT_STRING "_silent_exit"

#define OPT_NONE "none"
#define OPT_NONE_CAPS "NONE"
#define DJCOPT_JITC "jitc"
#define OPT_TRUE "true"

#define XRUN_LEN (sizeof(VMOPT_XRUN)-1)
#define DLLNAME_LEN 32									/* this value should be consistent with that in J9VMDllLoadInfo */
#define SMALL_STRING_BUF_SIZE 64
#define MED_STRING_BUF_SIZE 128
#define LARGE_STRING_BUF_SIZE 256

#define FUNCTION_VM_INIT "VMInitStages"
#define FUNCTION_THREAD_INIT "threadInitStages"
#define FUNCTION_ZERO_INIT	"zeroInitStages"

static const struct J9VMIgnoredOption ignoredOptionTable[] = {
	{ IGNORE_ME_STRING, EXACT_MATCH },
	{ VMOPT_XDEBUG, EXACT_MATCH },
	{ VMOPT_XNOAGENT, EXACT_MATCH },
	{ VMOPT_XINCGC, EXACT_MATCH }, /* JNLP param */
	{ VMOPT_XMIXED, EXACT_MATCH }, /* JNLP param */
	{ VMOPT_XPROF,  EXACT_MATCH }, /* JNLP param */
	{ VMOPT_XBATCH, EXACT_MATCH }, /* JNLP param */
	{ VMOPT_PORT_LIBRARY, EXACT_MATCH },
	{ VMOPT_BFU_JAVA, EXACT_MATCH },
	{ VMOPT_BP_JXE, EXACT_MATCH },
	{ VMOPT_NEEDS_JCL, EXACT_MATCH },
	{ VMOPT_VFPRINTF, EXACT_MATCH },
	{ VMOPT_EXIT, EXACT_MATCH },
	{ VMOPT_ABORT, EXACT_MATCH },
	{ VMOPT_XNOQUICKSTART, EXACT_MATCH },
	{ VMOPT_XJ9, EXACT_MATCH },
	{ VMOPT_XMXCL, STARTSWITH_MATCH },
	/* extra-extended options start with -XX. Ignore any not explicitly processed. */
#if defined(J9VM_OPT_SIDECAR)
	{ VMOPT_XJVM, STARTSWITH_MATCH },
	{ VMOPT_SERVER, EXACT_MATCH },
	{ VMOPT_X142BOOSTGCTHRPRIO, EXACT_MATCH },

	{ MAPOPT_XSIGCATCH, EXACT_MATCH },
	{ MAPOPT_XNOSIGCATCH, EXACT_MATCH },
	{ MAPOPT_XINITACSH, EXACT_MEMORY_MATCH },
	{ MAPOPT_XINITSH, EXACT_MEMORY_MATCH },
	{ MAPOPT_XINITTH, EXACT_MEMORY_MATCH },
	{ MAPOPT_XK, EXACT_MEMORY_MATCH },
	{ MAPOPT_XP, EXACT_MEMORY_MATCH },
	{ VMOPT_XNORTSJ, EXACT_MATCH },
#if !(defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS))
	{ VMOPT_XCOMPRESSEDREFS, EXACT_MATCH },
	{ VMOPT_XNOCOMPRESSEDREFS, EXACT_MATCH },
#endif /* !(defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)) */
#endif /* defined(J9VM_OPT_SIDECAR) */
};
#define ignoredOptionTableSize (sizeof(ignoredOptionTable) / sizeof(struct J9VMIgnoredOption))

IDATA VMInitStages (J9JavaVM *vm, IDATA stage, void* reserved);
IDATA registerCmdLineMapping (J9JavaVM* vm, char* sov_option, char* j9_option, UDATA mapFlags);
IDATA postInitLoadJ9DLL (J9JavaVM* vm, const char* dllName, void* argData);
void freeJavaVM (J9JavaVM * vm);
IDATA threadInitStages (J9JavaVM* vm, IDATA stage, void* reserved);
IDATA zeroInitStages (J9JavaVM* vm, IDATA stage, void* reserved);
UDATA runJVMOnLoad (J9JavaVM* vm, J9VMDllLoadInfo* loadInfo, char* options);

static void consumeVMArgs (J9JavaVM* vm, J9VMInitArgs* j9vm_args);
static BOOLEAN isEmpty (const char * str);

#if (defined(J9VM_OPT_SIDECAR))
static UDATA initializeJVMExtensionInterface (J9JavaVM* vm);
#endif /* J9VM_OPT_SIDECAR */
static UDATA initializeVprintfHook (J9JavaVM* vm);
#if (defined(J9VM_INTERP_VERBOSE))
static const char* getNameForLoadStage (IDATA stage);
#endif /* J9VM_INTERP_VERBOSE */
static jint runInitializationStage (J9JavaVM* vm, IDATA stage);
static IDATA setSignalOptions(J9JavaVM *vm, J9PortLibrary *portLibrary);
#if (defined(J9VM_OPT_SIDECAR))
void sidecarExit(J9VMThread* shutdownThread);
#endif /* J9VM_OPT_SIDECAR */
static jint runLoadStage (J9JavaVM *vm, IDATA flags);
#if defined(J9VM_GC_DYNAMIC_CLASS_UNLOADING)
static void freeClassNativeMemory (J9HookInterface** hook, UDATA eventNum, void* eventData, void* userData);
static void vmHookAnonClassesUnload(J9HookInterface** hook, UDATA eventNum, void* eventData, void* userData);
#endif /* GC_DYNAMIC_CLASS_UNLOADING */
static jint runShutdownStage (J9JavaVM* vm, IDATA stage, void* reserved, UDATA filterFlags);
static jint modifyDllLoadTable (J9JavaVM * vm, J9Pool* loadTable, J9VMInitArgs* j9vm_args);
static jint processVMArgsFromFirstToLast(J9JavaVM * vm);
static void processCompressionOptions(J9JavaVM *vm);
static IDATA setMemoryOptionToOptElse (J9JavaVM* vm, UDATA* thingToSet, char* optionName, UDATA defaultValue, UDATA doConsumeArg);
static IDATA setIntegerValueOptionToOptElse (J9JavaVM* vm, UDATA* thingToSet, char* optionName, UDATA defaultValue, UDATA doConsumeArg);

#ifdef JVMINIT_UNIT_TEST
static void testFindArgs (J9JavaVM* vm);
static void testOptionValueOps (J9JavaVM* vm);
#endif
static jint initializeXruns (J9JavaVM* vm);
static void unloadDLL (void* dllLoadInfo, void* userDataTemp);
#if (defined(J9VM_PROF_COUNT_ARGS_TEMPS))
static void report (J9JavaVM * vm);
#endif /* J9VM_PROF_COUNT_ARGS_TEMPS */
#if (defined(J9VM_OPT_SIDECAR))
static IDATA createMapping (J9JavaVM* vm, char* j9Name, char* mapName, UDATA flags, IDATA atIndex);
#endif /* J9VM_OPT_SIDECAR */
#if (defined(J9VM_OPT_JVMTI))
static void detectAgentXruns (J9JavaVM* vm);
#endif /* J9VM_OPT_JVMTI */
static void vfprintfHook (struct OMRPortLibrary *portLib, const char *format, ...);
static IDATA vfprintfHook_file_write_text(struct OMRPortLibrary *portLibrary, IDATA fd, const char *buf, IDATA nbytes);
static void runJ9VMDllMain (void* dllLoadInfo, void* userDataTemp);
static jint checkPostStage (J9JavaVM* vm, IDATA stage);
static void runUnOnloads (J9JavaVM* vm, UDATA shutdownDueToExit);
static void generateMemoryOptionParseError (J9JavaVM* vm, J9VMDllLoadInfo* loadInfo, UDATA errorType, char* optionWithError);
static void loadDLL (void* dllLoadInfo, void* userDataTemp);
static void registerIgnoredOptions (J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args);
static UDATA protectedInitializeJavaVM (J9PortLibrary* portLibrary, void * userData);
static J9Pool *initializeDllLoadTable (J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args, UDATA verboseFlags, J9JavaVM *vm);
#if defined(J9VM_OPT_SIDECAR) && (JAVA_SPEC_VERSION < 21)
static IDATA checkDjavacompiler (J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args);
#endif /* defined(J9VM_OPT_SIDECAR) && (JAVA_SPEC_VERSION < 21) */
static void* getOptionExtraInfo (J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args, IDATA match, char* optionName);
static void closeAllDLLs (J9JavaVM* vm);

#if (defined(J9VM_INTERP_VERBOSE))
static const char* getNameForStage (IDATA stage);
#endif /* J9VM_INTERP_VERBOSE */
#if (defined(J9VM_OPT_SIDECAR))
static IDATA registerVMCmdLineMappings (J9JavaVM* vm);
#endif /* J9VM_OPT_SIDECAR */
static void checkDllInfo (void* dllLoadInfo, void* userDataTemp);
static UDATA initializeVTableScratch(J9JavaVM* vm);
static jint initializeDDR (J9JavaVM * vm);
static void setThreadNameAsyncHandler(J9VMThread *currentThread, IDATA handlerKey, void *userData);
#if defined(J9VM_INTERP_CUSTOM_SPIN_OPTIONS)
static void cleanCustomSpinOptions(void *element, void *userData);
#endif /* J9VM_INTERP_CUSTOM_SPIN_OPTIONS */

/* Imports from vm/rasdump.c */
extern void J9RASInitialize (J9JavaVM* javaVM);
extern void J9RelocateRASData (J9JavaVM* javaVM);
extern void J9RASShutdown (J9JavaVM* javaVM);
extern void populateRASNetData (J9JavaVM *javaVM, J9RAS *rasStruct);

const U_8 J9CallInReturnPC[] = { 0xFF, 0x00, 0x00, 0xFF }; /* impdep2, parm, parm, impdep2 */
#if defined(J9VM_OPT_METHOD_HANDLE)
const U_8 J9Impdep1PC[] = { 0xFE, 0x00, 0x00, 0xFE }; /* impdep1, parm, parm, impdep1 */
#endif /* defined(J9VM_OPT_METHOD_HANDLE) */

static jint (JNICALL * vprintfHookFunction)(FILE *fp, const char *format, va_list args) = NULL;
static IDATA (* portLibrary_file_write_text) (struct OMRPortLibrary *portLibrary, IDATA fd, const char *buf, IDATA nbytes) = NULL;

#if !defined(WIN32)
static UDATA sigxfszHandler(struct J9PortLibrary* portLibrary, U_32 gpType, void* gpInfo, void* userData);
#endif /* !defined(WIN32) */

/* SSE2 support on 32 bit linux_x86 and win_x86 */
#ifdef J9VM_ENV_SSE2_SUPPORT_DETECTION

extern U_32 J9SSE2cpuidFeatures(void);
static BOOLEAN isSSE2SupportedOnX86();
#endif /* J9VM_ENV_SSE2_SUPPORT_DETECTION */

#if (defined(AIXPPC) || defined(LINUXPPC)) && !defined(J9OS_I5)
static BOOLEAN isPPC64bit(void);
#endif /* (AIXPPC || LINUXPPC) & !J9OS_I5 */

static UDATA predefinedHandlerWrapper(struct J9PortLibrary *portLibrary, U_32 gpType, void *gpInfo, void *userData);
static void signalDispatch(J9VMThread *vmThread, I_32 sigNum);

static UDATA parseGlrConfig(J9JavaVM* jvm, char* options);
static UDATA parseGlrOption(J9JavaVM* jvm, char* option);

J9_DECLARE_CONSTANT_UTF8(j9_int_void, "(I)V");
J9_DECLARE_CONSTANT_UTF8(j9_dispatch, "dispatch");

/* The appropriate bytecodeLoop is selected based on interpreter mode */
#if defined(OMR_GC_FULL_POINTERS)
UDATA bytecodeLoopFull(J9VMThread *currentThread);
UDATA debugBytecodeLoopFull(J9VMThread *currentThread);
#endif /* defined(OMR_GC_FULL_POINTERS) */

#if defined(OMR_GC_COMPRESSED_POINTERS)
UDATA bytecodeLoopCompressed(J9VMThread *currentThread);
UDATA debugBytecodeLoopCompressed(J9VMThread *currentThread);
#endif /* defined(OMR_GC_COMPRESSED_POINTERS) */

#if (defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)) || (defined(LINUX) && defined(J9VM_GC_REALTIME))
static BOOLEAN isGCPolicyMetronome(J9JavaVM *javaVM);
#endif /* (defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)) || (defined(LINUX) && defined(J9VM_GC_REALTIME)) */

#if defined(COUNT_BYTECODE_PAIRS)
static jint
initializeBytecodePairs(J9JavaVM *vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	jint rc = JNI_ENOMEM;
	UDATA allocSize = sizeof(UDATA) * 256 * 256;
	UDATA *matrix = j9mem_allocate_memory(allocSize, OMRMEM_CATEGORY_VM);
	if (NULL != matrix) {
		memset(matrix, 0, allocSize);
		vm->debugField1 = (UDATA)matrix;
		rc = JNI_OK;
	}
	return rc;
}

void
printBytecodePairs(J9JavaVM *vm)
{
	UDATA *matrix = (UDATA*)vm->debugField1;
	if (NULL != matrix) {
		PORT_ACCESS_FROM_JAVAVM(vm);
		UDATA i = 0;
		UDATA j = 0;
		for (i = 0; i < 256; ++i) {
			for (j = 0; j < 256; ++j) {
				UDATA count = matrix[(i * 256) + j];
				if (0 != count) {
					j9tty_printf(PORTLIB, "%09zu %s->%s\n", count, JavaBCNames[i], JavaBCNames[j]);
				}
			}
		}
	}
}

static void
freeBytecodePairs(J9JavaVM *vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	UDATA *matrix = (UDATA*)vm->debugField1;
	printBytecodePairs(vm);
	vm->debugField1 = 0;
	j9mem_free_memory(matrix);
}
#endif /* COUNT_BYTECODE_PAIRS */

static void
print_verbose_stackusage_of_nonsystem_threads(J9VMThread* vmThread)
{
	J9VMThread * currentThread;
	J9JavaVM * vm = vmThread->javaVM;
	if ((vm->runtimeFlags & J9_RUNTIME_REPORT_STACK_USE) && vmThread->stackObject && (vm->verboseLevel & VERBOSE_STACK)) {
		/* Failing to check for NULL leads to a endless spin.*/
		if((NULL == vm->vmThreadListMutex) || omrthread_monitor_try_enter(vm->vmThreadListMutex)) {
			/*vmThreadListMutex is null. Nothing to get lock on.*/
			/*If omrthread_monitor_try_enter returns true, it failed to get the lock. If it succeeds to enter, then it returns 0*/
			PORT_ACCESS_FROM_JAVAVM(vm);
			j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VERB_STACK_USAGE_FOR_RUNNING_THREADS_FAILURE_1);
		}else{
			/*got the lock*/
			currentThread = vmThread->linkNext;
			while (currentThread != vmThread) {
				J9VMThread * nextThread = currentThread->linkNext;
				if (currentThread->privateFlags & J9_PRIVATE_FLAGS_SYSTEM_THREAD ) {
					/*DO NOTHING. These threads will be handled automatically by system.exit()*/
				} else {
					/*
					 * Print stack usage info for running non-system threads.
					 * When system.exit() is called, it prints stack usage info for system thread and a thread that calls system.exit()
					 */
					print_verbose_stackUsage(currentThread, TRUE);
				}
				currentThread = nextThread;
			}
			/*Release the lock*/
			omrthread_monitor_exit(vm->vmThreadListMutex);
		}
	}
}

void
print_verbose_stackUsage(J9VMThread* vmThread, UDATA stillRunning)
{
	UDATA *stackSlot = J9_LOWEST_STACK_SLOT(vmThread);
	UDATA nbyteUsed = (vmThread->stackObject->end - stackSlot) * sizeof(UDATA);
	UDATA cbyteUsed = omrthread_get_stack_usage(vmThread->osThread);
	J9JavaVM * vm = vmThread->javaVM;
	while (*stackSlot == J9_RUNTIME_STACK_FILL) {
		stackSlot++;
		nbyteUsed -= sizeof(UDATA);
	}

	if (vmThread->threadObject) {
		char* name = getOMRVMThreadName(vmThread->omrVMThread);
		PORT_ACCESS_FROM_JAVAVM(vm);
		if(stillRunning == FALSE){
			/* J9NLS_VERB_STACK_USAGE=Verbose stack: \"%.*s\" used %zd/%zd bytes on Java/C stacks */
			j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VERB_STACK_USAGE, strlen(name), name, nbyteUsed, cbyteUsed);
		}else{ /*if stillRunning == TRUE*/
			/* J9NLS_VERB_STACK_USAGE_FOR_RUNNING_THREADS=Verbose stack: Running \"%2$.*1$s\" is using %3$zd/%4$zd bytes on Java/C stacks*/
			j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VERB_STACK_USAGE_FOR_RUNNING_THREADS, strlen(name), name, nbyteUsed, cbyteUsed);
		}
		releaseOMRVMThreadName(vmThread->omrVMThread);
	}
	if (nbyteUsed > vm->maxStackUse) vm->maxStackUse = nbyteUsed;
	if (cbyteUsed > vm->maxCStackUse) vm->maxCStackUse = cbyteUsed;
}

/**
 * @internal
 *
 * Detect -Xipt and prevent enablement of iconv converter initialization
 * in portlibrary.
 *
 * @return 0 if success, 1 otherwise
 */
static UDATA
setGlobalConvertersAware(J9JavaVM *vm)
{
	UDATA rc = 0;

	if ((FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XIPT, NULL)) >= 0) {
		/* -Xipt detected. Prevent enablement of cached UTF8 converters. */
		rc = 0;
	}
#if defined(J9VM_USE_ICONV) || defined(J9ZOS390)
	{
		PORT_ACCESS_FROM_JAVAVM(vm);
		rc = j9port_control(J9PORT_CTLDATA_NOIPT, 1);
	}
#endif
	return rc;
}

void OMRNORETURN
exitJavaVM(J9VMThread * vmThread, IDATA rc)
{
	J9JavaVM *vm = NULL;

	/* if no VM is specified, just try to exit from the first VM.
	 * Arguably, we should shutdown ALL the VMs.
	 */
	if (vmThread == NULL) {
		jint nVMs = 0;
		if (JNI_OK == J9_GetCreatedJavaVMs((JavaVM **)&vm, 1, &nVMs)) {
			if (nVMs == 1) {
				vmThread = currentVMThread(vm);
			}
		}
	} else {
		vm = vmThread->javaVM;
		if ((vm->runtimeFlags & J9_RUNTIME_REPORT_STACK_USE) && vmThread->stackObject && (vm->verboseLevel & VERBOSE_STACK)) {
			print_verbose_stackusage_of_nonsystem_threads(vmThread);
			print_verbose_stackUsage(vmThread, FALSE);
		}
	}

	if (vm != NULL) {
		PORT_ACCESS_FROM_JAVAVM(vm);

#if defined(J9VM_INTERP_ATOMIC_FREE_JNI)
		/* exitJavaVM is always called from a JNI context */
		enterVMFromJNI(vmThread);
		releaseVMAccess(vmThread);
#endif /* J9VM_INTERP_ATOMIC_FREE_JNI */

		/* we only let the shutdown code run once */

		if(vm->runtimeFlagsMutex != NULL) {
			omrthread_monitor_enter(vm->runtimeFlagsMutex);
		}

		if(vm->runtimeFlags & J9_RUNTIME_EXIT_STARTED) {
			if (vm->runtimeFlagsMutex != NULL) {
				omrthread_monitor_exit(vm->runtimeFlagsMutex);
			}

			if (vmThread->publicFlags & J9_PUBLIC_FLAGS_VM_ACCESS) {
				internalReleaseVMAccess(vmThread);
			}

			/* Do nothing. Wait for the process to exit. */
			while (1) {
				omrthread_suspend();
			}
		}

		vm->runtimeFlags |= J9_RUNTIME_EXIT_STARTED;
		if(vm->runtimeFlagsMutex != NULL) {
			omrthread_monitor_exit(vm->runtimeFlagsMutex);
		}

#ifdef J9VM_OPT_SIDECAR
		if (vm->sidecarExitHook)
			(*(vm->sidecarExitHook))(vm);
#endif

#ifdef J9VM_PROF_COUNT_ARGS_TEMPS
		report(vm);
#endif

		if (vmThread) {
			/* we can only perform these shutdown steps if the current thread is attached */
			TRIGGER_J9HOOK_VM_SHUTTING_DOWN(vm->hookInterface, vmThread, rc);
		}

		/* exitJavaVM runs special exit stage, but doesn't close the libraries or deallocate the VM thread */
		runExitStages(vm, vmThread);

		/* Acquire exclusive VM access to bring all threads to a safe
		 * point before shutting down.  This prevents intermittent crashes (particularly
		 * in the GC) when attempting to access the entryLocalStorage on the native
		 * stack of a thread that the OS is in the process of destroying.
		 *
		 * This is safe from deadlock with the exclusive access in DestroyJavaVM because
		 * J9_RUNTIME_EXIT_STARTED is set in DestroyJavaVM before acquiring exclusive
		 * access.  This function has already checked that J9_RUNTIME_EXIT_STARTED was not
		 * set before reaching this point.
		 *
		 * In rare cases, the VM may fatal exit while holding exclusive access, so don't attempt to
		 * acquire it if another thread already has it.
		 */
		if (J9_XACCESS_NONE == vm->exclusiveAccessState) {
			internalAcquireVMAccess(vmThread);
			acquireExclusiveVMAccess(vmThread);
		}

#if defined(WIN32)
		/* Do not attempt to exit while a JNI shared library open is in progress */
		omrthread_monitor_enter(vm->classLoaderBlocksMutex);
#endif

#if defined(J9VM_OPT_SNAPSHOTS)
		if (IS_SNAPSHOT_RUN(vm)) {
			teardownVMSnapshotImpl(vm);
		}
#endif /* defined(J9VM_OPT_SNAPSHOTS) */

#if defined(COUNT_BYTECODE_PAIRS)
		printBytecodePairs(vm);
#endif /* COUNT_BYTECODE_PAIRS */

#ifdef J9ZTPF
		/* run z/TPF OS exit hook */
		cjvm_jvm_shutdown_hook();
#endif

		if (vm->exitHook) {
#if defined(J9VM_ZOS_3164_INTEROPERABILITY)
			if (J9_IS_31BIT_INTEROP_TARGET(vm->exitHook)) {
				execute31BitExitHook(vm, (jint) rc);
			} else
#endif /* defined(J9VM_ZOS_3164_INTEROPERABILITY) */
			{
				vm->exitHook((jint) rc);
			}
		}

		j9exit_shutdown_and_exit((I_32) rc);
	}

	/* If we got here, then either we couldn't find a VM(!) or j9exit_shutdown_and_exit() returned(!)
	 * Neither of those scenarios should be possible
	 * But if somehow it does happen, we don't have many options here!
	 * We can't even print a message, since we don't have a port library.
	 */
	exit( (int)rc );
dontreturn: goto dontreturn; /* avoid warnings */
}

#if defined(J9VM_INTERP_CUSTOM_SPIN_OPTIONS)
static void
cleanCustomSpinOptions(void *element, void *userData)
{
	J9PortLibrary *portLibrary = (J9PortLibrary *)userData;
	PORT_ACCESS_FROM_PORT(portLibrary);
	J9VMCustomSpinOptions *option = (J9VMCustomSpinOptions *)element;
	j9mem_free_memory((char *)option->className);
}
#endif /* J9VM_INTERP_CUSTOM_SPIN_OPTIONS */


#if defined(J9VM_OPT_JITSERVER)
BOOLEAN
isJITServerEnabled(J9JavaVM *vm)
{
	return J9_ARE_ALL_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_ENABLE_START_JITSERVER);
}
#endif /* J9VM_OPT_JITSERVER */

void
freeJavaVM(J9JavaVM * vm)
{
	BOOLEAN hotReferenceFieldRequired = FALSE;
	J9PortLibrary *tmpLib = NULL;
	PORT_ACCESS_FROM_JAVAVM(vm);
	J9VMThread *currentThread = currentVMThread(vm);
	IDATA traceDescriptor = 0;
#if !defined(WIN32)
	j9sig_set_async_signal_handler(sigxfszHandler, NULL, 0);
#endif /* !defined(WIN32) */

#if defined(J9VM_OPT_JFR)
	if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_JFR_ENABLED)) {
		shutdownJFRIDs(vm);
	}
#endif /* defined(J9VM_OPT_JFR) */

#if JAVA_SPEC_VERSION >= 16
	if (NULL != vm->cifNativeCalloutDataCache) {
		pool_state poolState;
		void *cifNode = pool_startDo(vm->cifNativeCalloutDataCache, &poolState);
		while (NULL != cifNode) {
			freeAllStructFFITypes(currentThread, cifNode);
			cifNode = pool_nextDo(&poolState);
		}
		pool_kill(vm->cifNativeCalloutDataCache);
		vm->cifNativeCalloutDataCache = NULL;
	}

	if (NULL != vm->cifArgumentTypesCache) {
		pool_state poolState;
		J9CifArgumentTypes *cifArgTypesNode = pool_startDo(vm->cifArgumentTypesCache, &poolState);
		while (NULL != cifArgTypesNode) {
			j9mem_free_memory(cifArgTypesNode->argumentTypes);
			cifArgTypesNode = pool_nextDo(&poolState);
		}
		pool_kill(vm->cifArgumentTypesCache);
		vm->cifArgumentTypesCache = NULL;
	}

	/* Delete the layout string hashtable if exists. */
	if (NULL != vm->layoutStrFFITypeTable) {
		releaseLayoutStrFFITypeTable(vm->layoutStrFFITypeTable);
		vm->layoutStrFFITypeTable = NULL;
	}

	/* Empty the thunk heap list if exists. */
	if (NULL != vm->thunkHeapHead) {
		releaseThunkHeap(vm);
	}
#endif /* JAVA_SPEC_VERSION >= 16 */

	/* Remove the predefinedHandlerWrapper. */
	j9sig_set_single_async_signal_handler(predefinedHandlerWrapper, vm, 0, NULL);

	/* Unload before trace engine exits */
	UT_MODULE_UNLOADED(J9_UTINTERFACE_FROM_VM(vm));

	if (0 != vm->vmRuntimeStateListener.minIdleWaitTime) {
		stopVMRuntimeStateListener(vm);
	}

	if (NULL != vm->dllLoadTable) {
		runShutdownStage(vm, INTERPRETER_SHUTDOWN, NULL, 0);
	}

#if defined(J9VM_OPT_SNAPSHOTS)
	if (IS_SNAPSHOTTING_ENABLED(vm)) {
		teardownVMSnapshotImpl(vm);
	}
#endif /* defined(J9VM_OPT_SNAPSHOTS) */

	/* Kill global hot field class info pool and its monitor if dynamicBreadthFirstScanOrdering is enabled */
	if (NULL != vm->memoryManagerFunctions) {
		hotReferenceFieldRequired = vm->memoryManagerFunctions->j9gc_hot_reference_field_required(vm);
		if (hotReferenceFieldRequired && NULL != vm->hotFieldClassInfoPool) {
			pool_kill(vm->hotFieldClassInfoPool);
			vm->hotFieldClassInfoPool = NULL;
		}

		if (hotReferenceFieldRequired && NULL != vm->hotFieldClassInfoPoolMutex) {
			omrthread_monitor_destroy(vm->hotFieldClassInfoPoolMutex);
		}

		if (hotReferenceFieldRequired && NULL != vm->globalHotFieldPoolMutex) {
			omrthread_monitor_destroy(vm->globalHotFieldPoolMutex);
		}
	}

	if (NULL != vm->classMemorySegments) {
		J9ClassWalkState classWalkState;
		J9Class * clazz;

		clazz = allClassesStartDo(&classWalkState, vm, NULL);
		while (NULL != clazz) {
			j9mem_free_memory(clazz->jniIDs);
			clazz->jniIDs = NULL;
			clazz = allClassesNextDo(&classWalkState);
		}
		allClassesEndDo(&classWalkState);
	}

	if (NULL != vm->classLoaderBlocks) {
		pool_state clState = {0};
		void *clToFree = NULL;

		if (NULL != currentThread) {
			internalAcquireVMAccess(currentThread);
		}
		clToFree = pool_startDo(vm->classLoaderBlocks, &clState);
		while (NULL != clToFree) {
			void *tmpToFree = NULL;
			tmpToFree = clToFree;
			clToFree = pool_nextDo(&clState);
			freeClassLoader(tmpToFree, vm, currentThread, JNI_TRUE);
		}
		if (NULL != currentThread) {
			internalReleaseVMAccess(currentThread);
		}
	}

	if (NULL != vm->classLoadingConstraints) {
		hashTableFree(vm->classLoadingConstraints);
		vm->classLoadingConstraints = NULL;
	}

#ifdef J9VM_OPT_ZIP_SUPPORT
	if (NULL != vm->zipCachePool) {
		zipCachePool_kill(vm->zipCachePool);
		vm->zipCachePool = NULL;
	}
#endif

#if defined(J9VM_OPT_JFR)
	j9mem_free_memory(vm->jfrState.jfrFileName);
	vm->jfrState.jfrFileName = NULL;
#endif /* defined(J9VM_OPT_JFR) */

#if defined(J9VM_INTERP_ATOMIC_FREE_JNI_USES_FLUSH)
	shutDownExclusiveAccess(vm);
#endif /* J9VM_INTERP_ATOMIC_FREE_JNI_USES_FLUSH */

	freeNativeMethodBindTable(vm);
	if (!IS_SNAPSHOTTING_ENABLED(vm)) {
		freeHiddenInstanceFieldsList(vm);
	}
	cleanupLockwordConfig(vm);
	cleanupEnsureHashedConfig(vm);

	destroyJvmInitArgs(vm->portLibrary, vm->vmArgsArray);
	vm->vmArgsArray = NULL;

	if (NULL != vm->modulesPathEntry) {
		j9mem_free_memory(vm->modulesPathEntry);
		vm->modulesPathEntry = NULL;
	}

	if (NULL != vm->unnamedModuleForSystemLoader) {
		vm->internalVMFunctions->freeJ9Module(vm, vm->unnamedModuleForSystemLoader);
		vm->unnamedModuleForSystemLoader = NULL;
	}

	if (NULL != vm->modularityPool) {
		pool_kill(vm->modularityPool);
		vm->modularityPool = NULL;

		/* vm->javaBaseModule should have already been freed when vm->systemClassLoader was freed earlier */
		vm->javaBaseModule = NULL;
	}

	if (NULL != vm->jniGlobalReferences) {
		pool_kill(vm->jniGlobalReferences);
		vm->jniGlobalReferences = NULL;
	}

	if (NULL != vm->dllLoadTable) {
		J9VMDllLoadInfo *traceLoadInfo = NULL;

		if (NULL != currentThread) {
			/* Send thread destroy event now to free some things before memcheck does the unfreed block scan */
			TRIGGER_J9HOOK_VM_THREAD_DESTROY(vm->hookInterface, currentThread);
		}
		runShutdownStage(vm, LIBRARIES_ONUNLOAD, NULL, 0);
		runUnOnloads(vm, FALSE);
		runShutdownStage(vm, HEAP_STRUCTURES_FREED, NULL, 0);
		if (NULL != currentThread) {
			/* No need to worry about the zombie counter at this point */
			deallocateVMThread(currentThread, FALSE, FALSE);
		}
		runShutdownStage(vm, GC_SHUTDOWN_COMPLETE, NULL, 0);

		/* zOS: Do not close any of the DLLs. This is necessary
		 * because we do not know for sure whether all the threads
		 * that may depend on these DLLs have terminated.
		 *
		 * Note that this solution is also suggested in the
		 * 'z/OS XL C/C++ Programming Guide' V1R13 (Chapter 21
		 * under the heading 'DLL Restrictions').
		 */
#if !defined(J9ZOS390)
		closeAllDLLs(vm);
		/* Remember the file descriptor of the trace DLL. This has to be closed later than other DLLs. */
		traceLoadInfo = FIND_DLL_TABLE_ENTRY(J9_RAS_TRACE_DLL_NAME);
		if (NULL != traceLoadInfo) {
			traceDescriptor = traceLoadInfo->descriptor;
		}
#endif /* !defined(J9ZOS390) */
		freeDllLoadTable(vm->dllLoadTable);
		vm->dllLoadTable = NULL;
	}

	/* Detach the VM from OMR */
	detachVMFromOMR(vm);

	if (NULL != vm->jniWeakGlobalReferences) {
		pool_kill(vm->jniWeakGlobalReferences);
		vm->jniWeakGlobalReferences = NULL;
	}

	if (NULL != vm->classLoaderBlocks) {
		pool_kill(vm->classLoaderBlocks);
		vm->classLoaderBlocks = NULL;
	}

	if (NULL != vm->classLoadingStackPool) {
		pool_kill(vm->classLoadingStackPool);
		vm->classLoadingStackPool = NULL;
	}

#if defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES)
	if (NULL != vm->valueTypeVerificationStackPool) {
		pool_kill(vm->valueTypeVerificationStackPool);
		vm->valueTypeVerificationStackPool = NULL;
	}
#endif /* defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES) */

#if JAVA_SPEC_VERSION >= 19
	if (NULL != vm->tlsPool) {
		pool_kill(vm->tlsPool);
		vm->tlsPool = NULL;
	}

	if (NULL != vm->continuationT2Cache) {
		for (U_32 i = 0; i < vm->continuationT2Size; i++) {
			if (NULL != vm->continuationT2Cache[i]) {
				freeJavaStack(vm, vm->continuationT2Cache[i]->stackObject);
				j9mem_free_memory(vm->continuationT2Cache[i]);
			}
		}
		j9mem_free_memory(vm->continuationT2Cache);
	}
#endif /* JAVA_SPEC_VERSION >= 19 */

	j9mem_free_memory(vm->vTableScratch);
	vm->vTableScratch = NULL;

	j9mem_free_memory(vm->osrGlobalBuffer);
	vm->osrGlobalBuffer = NULL;

#if defined(COUNT_BYTECODE_PAIRS)
	freeBytecodePairs(vm);
#endif /* COUNT_BYTECODE_PAIRS */
	deleteStatistics(vm);

	terminateVMThreading(vm);
	tmpLib = vm->portLibrary;
#ifdef J9VM_INTERP_VERBOSE
	if (J9_ARE_ANY_BITS_SET(vm->runtimeFlags, J9_RUNTIME_REPORT_STACK_USE)) {
		/* J9NLS_VERB_MAX_STACK_USAGE=Verbose stack: maximum stack use was %zd/%zd bytes on Java/C stacks\n */
		j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VERB_MAX_STACK_USAGE, vm->maxStackUse, vm->maxCStackUse);
	}
#endif

#ifdef J9VM_PROF_COUNT_ARGS_TEMPS
	report(vm);
#endif

#if defined(J9VM_OPT_SHARED_CLASSES)
	if (NULL != vm->sharedClassPreinitConfig) {
		j9mem_free_memory(vm->sharedClassPreinitConfig);
		vm->sharedClassPreinitConfig = NULL;
	}
#endif

#ifdef J9VM_OPT_SIDECAR
	if (NULL != vm->jvmExtensionInterface) {
		j9mem_free_memory((void*)(vm->jvmExtensionInterface));
		vm->jvmExtensionInterface = NULL;
	}
#endif

	shutdownVMHookInterface(vm);

	freeSystemProperties(vm);

	if (NULL != vm->j9ras) {
		J9RASShutdown(vm);
	}

	contendedLoadTableFree(vm);

#ifndef J9VM_SIZE_SMALL_CODE
	fieldIndexTableFree(vm);
#endif

	/* Close the trace DLL. This has to be after all hashtable and pool free events, otherwise we'll crash on pool tracepoints */
	if (0 != traceDescriptor) {
		j9sl_close_shared_library(traceDescriptor);
	}

#if !defined(WIN32)
	/* restore any handler we may have overwritten */
	if (NULL != vm->originalSIGPIPESignalAction) {
		sigaction(SIGPIPE,(struct sigaction *)vm->originalSIGPIPESignalAction, NULL);
		j9mem_free_memory(vm->originalSIGPIPESignalAction);
		vm->originalSIGPIPESignalAction = NULL;
	}
#endif

#if defined(J9VM_INTERP_CUSTOM_SPIN_OPTIONS)
	/* Free custom spin options */
	if (NULL != vm->customSpinOptions) {
		pool_do(vm->customSpinOptions, cleanCustomSpinOptions, (void *)tmpLib);
		pool_kill(vm->customSpinOptions);
		vm->customSpinOptions = NULL;
	}
#endif /* J9VM_INTERP_CUSTOM_SPIN_OPTIONS */

	if (NULL != vm->realtimeSizeClasses) {
		j9mem_free_memory(vm->realtimeSizeClasses);
		vm->realtimeSizeClasses = NULL;
	}

#if defined(J9VM_OPT_OPENJDK_METHODHANDLE)
	if (NULL != vm->memberNameListsMutex) {
		omrthread_monitor_destroy(vm->memberNameListsMutex);
		vm->memberNameListsMutex = NULL;
	}

	if (NULL != vm->memberNameListNodePool) {
		pool_kill(vm->memberNameListNodePool);
		vm->memberNameListNodePool = NULL;
	}
#endif /* defined(J9VM_OPT_OPENJDK_METHODHANDLE) */

#if defined(J9VM_OPT_CRIU_SUPPORT)
	{
		J9Pool *hookRecords = vm->checkpointState.hookRecords;
		J9VMInitArgs *restoreArgsList = vm->checkpointState.restoreArgsList;
		J9Pool *classIterationRestoreHookRecords = vm->checkpointState.classIterationRestoreHookRecords;

		j9mem_free_memory(vm->checkpointState.restoreArgsChars);

		if (NULL != hookRecords) {
			pool_kill(hookRecords);
			vm->checkpointState.hookRecords = NULL;
		}

		if (NULL != classIterationRestoreHookRecords) {
			pool_kill(classIterationRestoreHookRecords);
			vm->checkpointState.classIterationRestoreHookRecords = NULL;
		}

		j9sl_close_shared_library(vm->checkpointState.libCRIUHandle);

		if (NULL != vm->delayedLockingOperationsMutex) {
			omrthread_monitor_destroy(vm->delayedLockingOperationsMutex);
			vm->delayedLockingOperationsMutex = NULL;
		}

		while (NULL != restoreArgsList) {
			J9VMInitArgs *previousArgs = restoreArgsList->previousArgs;
			destroyJvmInitArgs(vm->portLibrary, restoreArgsList);
			restoreArgsList = previousArgs;
		}
	}
#endif /* defined(J9VM_OPT_CRIU_SUPPORT) */

#if JAVA_SPEC_VERSION >= 22
	if (NULL != vm->closeScopeMutex) {
		omrthread_monitor_destroy(vm->closeScopeMutex);
		vm->closeScopeMutex = NULL;
	}
#endif /* JAVA_SPEC_VERSION >= 22 */

#if defined(J9VM_OPT_SNAPSHOTS)
	if (IS_SNAPSHOTTING_ENABLED(vm)) {
		VMSnapshotImplPortLibrary *imagePortLibrary = vm->vmSnapshotImplPortLibrary;
		shutdownVMSnapshotImpl(imagePortLibrary);
	} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
	{
		j9mem_free_memory(vm);
	}

	if (NULL != tmpLib->self_handle) {
		tmpLib->port_shutdown_library(tmpLib);
	} else {
		/* VM did not create this port library, so it is NOT responsible for shutting it down */
	}
}

/**
 * Do a runtime check of the processor and operating system to determine if
 * this is a supported configuration.
 *
 * @param[in] portLibrary - the port library.
 * @return TRUE if the environment is supported, FALSE otherwise.
 *
 * @note If the function returns FALSE it must print a descriptive error message.
 */
static BOOLEAN
isValidProcessorAndOperatingSystem(J9PortLibrary* portLibrary)
{
#if defined(WIN32)
	PORT_ACCESS_FROM_PORT(portLibrary);
#if defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE)
	/* OS Versions:  https://msdn.microsoft.com/en-us/library/windows/desktop/ms724832(v=vs.85).aspx */
	if (IsWindowsXPOrGreater()) {
		return TRUE;
	} else {
		j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_WINDOWS_XP_OR_GREATER_REQUIRED, j9sysinfo_get_OS_type());
		return FALSE;
	}
#else /* ! (defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE)) */
	OSVERSIONINFOEX versionInfo;

	versionInfo.dwOSVersionInfoSize = sizeof(versionInfo);
	if (0 == GetVersionEx((OSVERSIONINFO *) &versionInfo)) {
		return FALSE;
	}

	/* Version 5.1 == Windows XP */
	if ((versionInfo.dwMajorVersion > 5) || ( (versionInfo.dwMajorVersion == 5) && (versionInfo.dwMinorVersion >= 1) )) {
		return TRUE;
	} else {
		j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_WINDOWS_XP_OR_GREATER_REQUIRED, j9sysinfo_get_OS_type());
		return FALSE;
	}
#endif /* defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE) */
#endif /* defined(WIN32) */
	return TRUE;
}

/**
 * Called by JNI_CreateJavaVM to initialize the JavaVM.
 * First, this ensures that a port library exists.
 * Then it delegates most of the work to protectedInitializeJavaVM, which it wraps in a structured signal handler
 */
jint
initializeJavaVM(void * osMainThread, J9JavaVM ** vmPtr, J9CreateJavaVMParams *createParams)
{
	extern J9InternalVMFunctions J9InternalFunctions;
	J9JavaVM* vm = NULL;
	J9InitializeJavaVMArgs initArgs;
	J9PortLibrary *portLibrary = createParams->portLibrary;
	UDATA result;
	J9PortLibrary *privatePortLibrary;

	Assert_VM_notNull(portLibrary);

	/* this is here only so that we can use the portlib macros later on */
	privatePortLibrary = portLibrary;

#if (defined(AIXPPC) || defined(LINUXPPC)) && !defined(J9OS_I5)
	if (FALSE == isPPC64bit()) {
		j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_PPC32_HARDWARE_NOT_SUPPORTED);
		return JNI_ERR;
	}
#endif /* (AIXPPC || LINUXPPC) && !J9OS_I5 */

#ifdef J9VM_ENV_SSE2_SUPPORT_DETECTION
	if (FALSE == isSSE2SupportedOnX86()) {
		j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_SSE2_NOT_SUPPORTED);
		return JNI_ERR;
	}
#endif /* J9VM_ENV_SSE2_SUPPORT_DETECTION */

	if (FALSE == isValidProcessorAndOperatingSystem(portLibrary)) {
		return JNI_ERR;
	}

#if defined(J9VM_OPT_SNAPSHOTS)
	if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_SNAPSHOT)) {
		BOOLEAN isSnapshotRun = TRUE;
		void *vmSnapshotImpl = NULL;

		/* This check cannot use port functions like j9file_stat becuase the port library's
		 * function table has not yet been initialized.
		 */
		if (-1 != access(createParams->vmSnapshotFilePath, F_OK)) {
			isSnapshotRun = FALSE;
		}

		vmSnapshotImpl = createVMSnapshotImpl(portLibrary, isSnapshotRun, createParams->vmSnapshotFilePath);
		if (NULL == vmSnapshotImpl) {
			return JNI_ENOMEM;
		}

		if (isSnapshotRun) {
			vm = allocateJavaVMWithOMR((J9PortLibrary *)getPortLibraryFromVMSnapshotImpl(vmSnapshotImpl));
		} else {
			vm = getJ9JavaVMFromVMSnapshotImpl(vmSnapshotImpl);
		}

		if (NULL == vm) {
			return JNI_ENOMEM;
		}

		if (!postInitializeVMSnapshotImpl(vmSnapshotImpl, vm, isSnapshotRun)) {
			return JNI_ENOMEM;
		}

		if (isSnapshotRun) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_RAMSTATE_SNAPSHOT_RUN;
		} else {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_RAMSTATE_RESTORE_RUN;
		}
	} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
	{
		/* Allocate the VM, including the extra OMR structures */
		vm = allocateJavaVMWithOMR(portLibrary);
		if (NULL == vm) {
			return JNI_ENOMEM;
		}
	}

#if defined(J9VM_THR_ASYNC_NAME_UPDATE)
	vm->threadNameHandlerKey = -1;
#endif /* J9VM_THR_ASYNC_NAME_UPDATE */

#if defined(J9VM_JIT_RUNTIME_INSTRUMENTATION)
	/* Protection in case updateJITRuntimeInstrumentationFlags is called before initializeJITRuntimeInstrumentation */
	vm->jitRIHandlerKey = -1;
#endif

#if defined(OMR_GC_CONCURRENT_SCAVENGER) && defined(J9VM_ARCH_S390)
	vm->invokeJ9ReadBarrier = invokeJ9ReadBarrier;
#endif

	/* Make this a valid JavaVM.
	 * (This initialized the JavaVM->JNIInvokeInterface_ field, too, since the internal functions just hang off the end
	 * of the invoke interface
	 */
	vm->javaVM = vm;
	vm->reserved1_identifier = (void*)J9VM_IDENTIFIER;
	vm->internalVMFunctions = GLOBAL_TABLE(J9InternalFunctions);
	vm->portLibrary = portLibrary;
	vm->localMapFunction = j9localmap_LocalBitsForPC;

	vm->internalVMLabels = (J9InternalVMLabels*)-1001;
	vm->cInterpreter = J9_BUILDER_SYMBOL(cInterpreter);
	vm->threadDllHandle = createParams->threadDllHandle;
#if defined(J9VM_OPT_JFR)
	vm->loadedClassCount = 0;
	vm->jfrState.blobFileDescriptor = -1;
#endif /* defined(J9VM_OPT_JFR) */

#if JAVA_SPEC_VERSION >= 19
	/* tid 1 will be use by main thread, first usable tid starts at 2 */
	vm->nextTID = 2;
#endif /* JAVA_SPEC_VERSION >= 19 */

	*vmPtr = vm;

	initArgs.vm_args = ((J9VMInitArgs*) createParams->vm_args)->actualVMArgs;
	if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_VERBOSE_INIT)) {
		vm->verboseLevel |= VERBOSE_INIT;
	}
#ifndef WIN32
	if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_ARGENCODING_LATIN)) {
		vm->runtimeFlags |= J9_RUNTIME_ARGENCODING_LATIN;
	} else if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_ARGENCODING_UTF8)) {
		vm->runtimeFlags |= J9_RUNTIME_ARGENCODING_UTF8;
	} else
#endif
	if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_ARGENCODING_PLATFORM)) {
		vm->runtimeFlags |= J9_RUNTIME_ARGENCODING_UNICODE;
	}
#if defined(J9VM_OPT_JITSERVER)
	if (J9_ARE_ALL_BITS_SET(createParams->flags, J9_CREATEJAVAVM_START_JITSERVER)) {
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_START_JITSERVER;
	}
#endif /* J9VM_OPT_JITSERVER */

#if defined(J9HAMMER)
{
	OMRPORT_ACCESS_FROM_J9PORT(PORTLIB);
	OMRProcessorDesc desc;
	omrsysinfo_get_processor_description(&desc);

	/*
	 * Set runtime flag J9_EXTENDED_RUNTIME_USE_VECTOR_REGISTERS
	 * if AVX is supported. This is used to determine if the vzeroupper
	 * instruction is needed in JIT helper code to avoid performance
	 * penalties when transitioning between AVX and legacy SSE code.
	 */
	if (omrsysinfo_processor_has_feature(&desc, OMR_FEATURE_X86_AVX)
		&& omrsysinfo_processor_has_feature(&desc, OMR_FEATURE_X86_XSAVE_AVX)
	) {
		vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_USE_VECTOR_REGISTERS;
	}
}
#endif /* defined(J9HAMMER) */

	initArgs.j2seVersion = createParams->j2seVersion;
	initArgs.j2seRootDirectory = createParams->j2seRootDirectory;
	initArgs.j9libvmDirectory = createParams->j9libvmDirectory;
	initArgs.globalJavaVM = createParams->globalJavaVM;
	initArgs.osMainThread = osMainThread;
	initArgs.vm = vm;

	vm->vmArgsArray = createParams->vm_args;

	/* Process the signal options as early as possible (BEFORE we call sig_protect). */
	if (0 != setSignalOptions(vm, portLibrary)) {
		return JNI_ERR;
	}

	/* initialize the mappings between omrthread and java priorities */
	initializeJavaPriorityMaps(vm);

	if (0 != j9sig_protect(
		protectedInitializeJavaVM, &initArgs,
		structuredSignalHandlerVM, vm,
		J9PORT_SIG_FLAG_SIGALLSYNC | J9PORT_SIG_FLAG_MAY_CONTINUE_EXECUTION,
		&result)
	) {
		result = JNI_ERR;
	}

	if (0 != result) {
		freeJavaVM(vm);
	}

	return (jint)result;

}

#if (defined(J9VM_PROF_COUNT_ARGS_TEMPS))
static void
report(J9JavaVM * vm)
{
	UDATA ** counts = (UDATA **) vm->reserved2;
	UDATA count;
	UDATA args;
	UDATA temps;

#define MAX_ARGS 5
#define MAX_TEMPS 5

	count = 0;
	for (args = 0; args < 256; ++args) {
		count += counts[args][0];
	}
	printf("\n\nMethods with 0 temps = %u", count);

	count = 0;
	for (args = 0; args < 256; ++args) {
		for (temps = 1; temps < 256; ++temps) {
			count += counts[args][temps];
		}
	}
	printf("\nMethods with n temps = %u", count);

	count = 0;
	for (args = 0; args < 256; ++args) {
		count += counts[0][args];
	}
	printf("\nMethods with 0 args  = %u", count);

	count = 0;
	for (args = 1; args < 256; ++args) {
		for (temps = 0; temps < 256; ++temps) {
			count += counts[args][temps];
		}
	}
	printf("\nMethods with n args  = %u", count);

	printf("\n\n     temps  ");
	for (temps = 0; temps <= MAX_TEMPS; ++temps) {
		printf("%u      ", temps);
	}
	printf("> %u", MAX_TEMPS);
	printf("\nargs");
	for (args = 0; args <= MAX_ARGS; ++args) {
		printf("\n%u           ", args);
		for (temps = 0; temps <= MAX_TEMPS; ++temps) {
			printf("%05u  ", counts[args][temps]);
		}
		count = 0;
		for (temps = MAX_TEMPS + 1; temps < 256; ++temps) {
			count += counts[args][temps];
		}
		printf("%05u", count);
	}
	printf("\n> %u         ", MAX_ARGS);
	for (temps = 0; temps <= MAX_TEMPS; ++temps) {
		count = 0;
		for (args = MAX_ARGS + 1; args < 256; ++args) {
			count += counts[args][temps];
		}
		printf("%05u  ", count);
	}
	count = 0;
	for (temps = MAX_TEMPS + 1; temps < 256; ++temps) {
		for (args = MAX_ARGS + 1; args < 256; ++args) {
			count += counts[args][temps];
		}
	}
	printf("%05u", count);

	printf("\n\n");
}
#endif /* J9VM_PROF_COUNT_ARGS_TEMPS */

static void
vfprintfHook(struct OMRPortLibrary *portLib, const char *format, ...)
{
	va_list args;
	jint (JNICALL * * vprintfHookFunctionPtr)(FILE *fp, const char *format, va_list args) = GLOBAL_DATA(vprintfHookFunction);

	va_start( args, format );
	(*vprintfHookFunctionPtr)(stderr, format, args);
	va_end( args );
}

static IDATA vfprintfHook_file_write_text(struct OMRPortLibrary *portLibrary, IDATA fd, const char *buf, IDATA nbytes)
{
	const char *format="%.*s";
	if (J9PORT_TTY_ERR == fd) {
		vfprintfHook(portLibrary, format, nbytes, buf);
		return 0;
	}

	return	portLibrary_file_write_text(portLibrary, fd, buf, nbytes);
}

/* Run using a pool_do, this method loads all libraries that match the flags given. */

static void
loadDLL(void* dllLoadInfo, void* userDataTemp)
{
	J9VMDllLoadInfo* entry = (J9VMDllLoadInfo*) dllLoadInfo;
	LoadInitData* userData = (LoadInitData*) userDataTemp;

	if (J9_ARE_NO_BITS_SET(entry->loadFlags, (MAGIC_LOAD | NOT_A_LIBRARY | BUNDLED_COMP)) && J9_ARE_ANY_BITS_SET(entry->loadFlags, userData->flags)) {
		I_64 start = 0;
		I_64 end = 0;
		BOOLEAN loadDllSuccess = FALSE;
		const char * const dllName = (entry->loadFlags & ALTERNATE_LIBRARY_USED) ? entry->alternateDllName : entry->dllName;

		PORT_ACCESS_FROM_JAVAVM(userData->vm);

		start = j9time_nano_time();

		/* this function will enter information, such as the descriptor, into the dllLoadInfo */
		loadDllSuccess = loadJ9DLL(userData->vm, entry);

		end = j9time_nano_time();

		if (loadDllSuccess) {
			JVMINIT_VERBOSE_INIT_VM_TRACE1(userData->vm, "\tLoaded library %s\n", dllName);
		} else {
			JVMINIT_VERBOSE_INIT_VM_TRACE1(userData->vm, "\tFailed to load library %s\n", dllName);
		}

		JVMINIT_VERBOSE_INIT_VM_TRACE2(userData->vm, "\t\tcompleted with rc=%d in %lld nano sec.\n", loadDllSuccess, (end-start));
		JVMINIT_VERBOSE_INIT_TRACE_WORKING_SET(userData->vm);
	}
}

/* This is the "table" of libraries and virtual libraries used during VM initialization.
 *  Every library used by the VM should have an entry here, except for any user Xruns.
 */
static J9Pool *
initializeDllLoadTable(J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args, UDATA verboseFlags, J9JavaVM *vm)
{
	J9Pool *returnVal = pool_new(sizeof(J9VMDllLoadInfo),  0, 0, 0, J9_GET_CALLSITE(), OMRMEM_CATEGORY_VM, POOL_FOR_PORT(portLibrary));
	IDATA i;
	char* testString, *options;
	J9VMDllLoadInfo* newEntry;
	char dllNameBuffer[SMALL_STRING_BUF_SIZE];			/* Plenty big enough - needs to be at least DLLNAME_LEN */
	const char *gcDLLName = J9_GC_DLL_NAME;
	const char *gccheckDLLName = J9_CHECK_GC_DLL_NAME;
	const char *verboseDLLName = J9_VERBOSE_DLL_NAME;

	PORT_ACCESS_FROM_PORT(portLibrary);

	if (NULL == returnVal)
		goto _error;

#if defined(OMR_MIXED_REFERENCES_MODE_STATIC)
	if (!J9JAVAVM_COMPRESS_OBJECT_REFERENCES(vm)) {
		gcDLLName = J9_GC_FULL_DLL_NAME;
		gccheckDLLName = J9_CHECK_GC_FULL_DLL_NAME;
		verboseDLLName = J9_VERBOSE_FULL_DLL_NAME;
	}
#endif /* defined(OMR_MIXED_REFERENCES_MODE_STATIC) */

	JVMINIT_VERBOSE_INIT_TRACE(verboseFlags, "\nInitializing DLL load table:\n");
#if defined(J9VM_OPT_JAVA_OFFLOAD_SUPPORT)
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_IFA_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
#endif /* defined(J9VM_OPT_JAVA_OFFLOAD_SUPPORT) */

	if (NULL == createLoadInfo(portLibrary, returnVal, J9_JIT_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;

	/* make these magic for now */
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_JIT_DEBUG_DLL_NAME, (MAGIC_LOAD | NO_J9VMDLLMAIN | SILENT_NO_DLL), NULL, verboseFlags))
		goto _error;

	if (NULL == createLoadInfo(portLibrary, returnVal, J9_VERIFY_DLL_NAME, NOT_A_LIBRARY, (void*)&j9bcv_J9VMDllMain, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, gcDLLName, (LOAD_BY_DEFAULT | FATAL_NO_DLL), NULL, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_DYNLOAD_DLL_NAME, NOT_A_LIBRARY, (void*)&bcutil_J9VMDllMain, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, verboseDLLName, ALLOW_POST_INIT_LOAD, NULL, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_RAS_DUMP_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_RAS_TRACE_DLL_NAME, NEVER_CLOSE_DLL, NULL, verboseFlags))
		goto _error;
	if (!createLoadInfo(portLibrary, returnVal, J9_CHECK_JNI_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_CHECK_VM_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
/* This needs to be loaded prior to the shared classes module since the 'crossguest' suboption
 * to -Xshareclasses requires that the -Xvirt option has been seen.
 */
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_SHARED_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
#ifdef J9VM_OPT_JVMTI
	if (NULL == createLoadInfo(portLibrary, returnVal, J9_JVMTI_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
#endif
	if (NULL == createLoadInfo(portLibrary, returnVal, gccheckDLLName, 0, NULL, verboseFlags))
		goto _error;

	if (NULL == createLoadInfo(portLibrary, returnVal, J9_DEBUG_DLL_NAME, 0, NULL, verboseFlags))
		goto _error;
/* This needs to be BEFORE the JCL library as we need to install hooks that are used
 * in the initialization of JCL library */

	if (!createLoadInfo(portLibrary, returnVal, J9_JAVA_SE_DLL_NAME, (LOAD_BY_DEFAULT | FATAL_NO_DLL), NULL, verboseFlags))
		goto _error;

	if (NULL == createLoadInfo(portLibrary, returnVal, FUNCTION_ZERO_INIT, NOT_A_LIBRARY, (void*)&zeroInitStages, verboseFlags))
		goto _error;
	if (!createLoadInfo(portLibrary, returnVal, FUNCTION_VM_INIT, NOT_A_LIBRARY, (void*)&VMInitStages, verboseFlags))
		goto _error;
	if (NULL == createLoadInfo(portLibrary, returnVal, FUNCTION_THREAD_INIT, NOT_A_LIBRARY, (void*)&threadInitStages, verboseFlags))
		goto _error;


	/* Search for Xrun libraries and add them to the table. Searches right to left and ignores duplicates. */
	for (i = (j9vm_args->actualVMArgs->nOptions - 1); i >= 0 ; i--) {
		testString = getOptionString(j9vm_args, i);				/* may return mapped value */
		if (strstr(testString, VMOPT_XRUN) == testString) {
			/* No need to scan for -Xrunjdwp as a special case. We should never see that option directly as it is either mapped or translated. */
			memset(dllNameBuffer, 0, SMALL_STRING_BUF_SIZE);
			strncpy(dllNameBuffer, testString + strlen(VMOPT_XRUN), (SMALL_STRING_BUF_SIZE - 1));
			if (0 != (options = strchr(dllNameBuffer, ':'))) {											/* get the name and remove the options */
				*options = '\0';
			}
			if (NULL != findDllLoadInfo(returnVal, dllNameBuffer)) {								/* look for duplicate Xrun */
				continue;
			}
			newEntry = createLoadInfo(portLibrary, returnVal, dllNameBuffer, (LOAD_BY_DEFAULT | XRUN_LIBRARY | NO_J9VMDLLMAIN | FATAL_NO_DLL), NULL, verboseFlags);
			if (NULL != newEntry) {
				if (OPTION_OK != optionValueOperations(PORTLIB, j9vm_args, i, GET_OPTION, &options, 0, ':', 0, NULL)) {				/* knows how to deal with mapped options */
					goto _error;
				}
				newEntry->reserved = "";								/* Libraries don't like NULL options */
				if (NULL != options)
					newEntry->reserved = (void*)options;		/* Store any options in reserved */
			} else {
				goto _error;
			}
		}
	}

	return returnVal;

_error :
	j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_OUT_OF_MEM_FOR_DLL_POOL);
	return NULL;
}

#if (defined(J9VM_OPT_DYNAMIC_LOAD_SUPPORT))

UDATA
initializeClassPath(J9JavaVM *vm, char *classPath, U_8 classPathSeparator, U_16 cpFlags, BOOLEAN initClassPathEntry, J9ClassPathEntry ***classPathEntries)
{
	UDATA classPathEntryCount = 0;
	U_32 i = 0;
	U_32 cpLength = 0;
	U_32 classPathLength = 0;
	BOOLEAN lastWasSeparator = TRUE;

	PORT_ACCESS_FROM_JAVAVM(vm);
#if defined(J9VM_OPT_SNAPSHOTS)
	VMSNAPSHOTIMPLPORT_ACCESS_FROM_JAVAVM(vm);
#endif /* defined(J9VM_OPT_SNAPSHOTS) */

	if (NULL == classPath) {
		*classPathEntries = NULL;
		goto _end;
	}

	cpLength = (U_32)strlen(classPath);

	for (i = 0; i < cpLength; i++) {
		if (classPathSeparator == classPath[i]) {
			lastWasSeparator = TRUE;
		} else {
			if (TRUE == lastWasSeparator) {
				classPathEntryCount += 1;
				lastWasSeparator = FALSE;
			}
			classPathLength += 1;
		}
	}

	if (0 == classPathEntryCount) {
		*classPathEntries = NULL;
	} else {
		UDATA cpePtrArraySize = 0;
		UDATA cpePtrArrayMemSize = 0;
		J9ClassPathEntry **cpePtrArray = NULL;
		UDATA classPathMemSize = 0;
		J9ClassPathEntry *cpEntries = NULL;

		cpePtrArraySize = ROUND_UP_TO(CPE_COUNT_INCREMENT, classPathEntryCount);
		cpePtrArrayMemSize = sizeof(*classPathEntries) * cpePtrArraySize;
#if defined(J9VM_OPT_SNAPSHOTS)
		if (IS_SNAPSHOTTING_ENABLED(vm)) {
			cpePtrArray = (J9ClassPathEntry **)vmsnapshot_allocate_memory(cpePtrArrayMemSize, OMRMEM_CATEGORY_VM);
		} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
		{
			cpePtrArray = (J9ClassPathEntry **)j9mem_allocate_memory(cpePtrArrayMemSize, OMRMEM_CATEGORY_VM);
		}
		/* classPathEntryCount is for number of null characters */
		classPathMemSize = (sizeof(J9ClassPathEntry) * classPathEntryCount) + classPathLength + classPathEntryCount;
#if defined(J9VM_OPT_SNAPSHOTS)
		if (IS_SNAPSHOTTING_ENABLED(vm)) {
			cpEntries = vmsnapshot_allocate_memory(classPathMemSize, OMRMEM_CATEGORY_VM);
		} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
		{
			cpEntries = j9mem_allocate_memory(classPathMemSize, OMRMEM_CATEGORY_VM);
		}

		if ((NULL == cpePtrArray) || (NULL == cpEntries)) {
#if defined(J9VM_OPT_SNAPSHOTS)
			if (IS_SNAPSHOTTING_ENABLED(vm)) {
				vmsnapshot_free_memory(cpePtrArray);
				vmsnapshot_free_memory(cpEntries);
			} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
			{
				j9mem_free_memory(cpePtrArray);
				j9mem_free_memory(cpEntries);
			}
			*classPathEntries = NULL;
			classPathEntryCount = -1;
		} else {
			U_8 *cpPathMemory = (U_8 *)(cpEntries + classPathEntryCount);
			J9ClassPathEntry *cpEntry = cpEntries;
			char *entryStart = classPath;
			char *entryEnd = classPath;
			char *cpEnd = classPath + cpLength;

			memset(cpePtrArray, 0, cpePtrArrayMemSize);
			memset(cpEntries, 0, classPathMemSize);

			for (i = 0; i < classPathEntryCount; ) {
				/* walk to the end of the entry */
				entryEnd = entryStart;
				while ((entryEnd != cpEnd) && (*entryEnd != classPathSeparator)) {
					entryEnd += 1;
				}

				/* allocate space for entry */
				cpEntry->pathLength = (U_32)(entryEnd - entryStart);
				if (cpEntry->pathLength > 0) {
					cpEntry->path = cpPathMemory;
					/* copy info into entry */
					memcpy(cpEntry->path, entryStart, cpEntry->pathLength);
					cpEntry->path[cpEntry->pathLength] = 0;  /* null terminate */
					cpEntry->extraInfo = NULL;
					cpEntry->type = CPE_TYPE_UNKNOWN;
					cpEntry->flags = cpFlags;
					if (TRUE == initClassPathEntry) {
						initializeClassPathEntry(vm, cpEntry);
					}
					cpePtrArray[i] = cpEntry;
					cpPathMemory += (cpEntry->pathLength + 1);
					cpEntry++;
					i++;
				}
				entryStart = entryEnd + 1;
			}
			*classPathEntries = cpePtrArray;
		}
	}

_end:
	return classPathEntryCount;
}

IDATA
initializeClassPathEntry (J9JavaVM * javaVM, J9ClassPathEntry *cpEntry)
{
	PORT_ACCESS_FROM_JAVAVM(javaVM);
	int32_t attr = 0;

	/* If we know what it is, then go for it */
	if ((CPE_TYPE_UNKNOWN != cpEntry->type) && !IS_RESTORE_RUN(javaVM)) {
		return (IDATA)cpEntry->type;
	}

	/* clear the status field first if we have not init the CP entry */
	cpEntry->status = 0;

	/* Start guessing. Is it a directory? */
	attr = j9file_attr((char *)cpEntry->path);
	if (EsIsDir == attr) {
		cpEntry->type = CPE_TYPE_DIRECTORY;
		return CPE_TYPE_DIRECTORY;
	}

#ifdef J9VM_OPT_ZIP_SUPPORT
	if (EsIsFile == attr) {
		VMI_ACCESS_FROM_JAVAVM((JavaVM *)javaVM);
		VMIZipFunctionTable *zipFunctions = (*VMI)->GetZipFunctions(VMI);
		VMIZipFile *zipFile = NULL;

		cpEntry->extraInfo = NULL;
		zipFile = j9mem_allocate_memory((UDATA) sizeof(*zipFile), J9MEM_CATEGORY_CLASSES);
		if (NULL != zipFile) {
			I_32 rc = 0;

			memset(zipFile, 0, sizeof(*zipFile));
			rc = zipFunctions->zip_openZipFile(VMI, (char *)cpEntry->path, zipFile, ZIP_FLAG_OPEN_CACHE | ZIP_FLAG_BOOTSTRAP);
			if (0 == rc) {
				/* Save the zipFile */
				cpEntry->extraInfo = zipFile;
				cpEntry->type = CPE_TYPE_JAR;
				return CPE_TYPE_JAR;
			} else {
				Trc_VM_initializeClassPathEntry_loadZipFailed(cpEntry->pathLength, cpEntry->path, rc);
				j9mem_free_memory(zipFile);
			}
		}
	}
#endif /* J9VM_OPT_ZIP_SUPPORT */

	/* Ok. There's nothing here. Give up. */
	cpEntry->type = CPE_TYPE_UNUSABLE;
	cpEntry->extraInfo = NULL;
	return CPE_TYPE_UNUSABLE;
}

IDATA
initializeModulesPathEntry(J9JavaVM * javaVM, J9ClassPathEntry *cpEntry)
{
	PORT_ACCESS_FROM_JAVAVM(javaVM);
	int32_t attr = 0;
#if defined(WIN32)
	char *convertedModulesPath = NULL;
#endif /* defined(WIN32) */

	/* Start guessing. Is it a directory? */
	attr = j9file_attr((char *)cpEntry->path);
	if (EsIsDir == attr) {
		cpEntry->type = CPE_TYPE_DIRECTORY;
		return CPE_TYPE_DIRECTORY;
	}

	if (EsIsFile == attr) {
		J9JImageIntf *jimageIntf = javaVM->jimageIntf;
		if (NULL != jimageIntf) {
			I_32 rc = 0;
			UDATA jimageHandle = 0;
			char *modulesPath = (char *)cpEntry->path;
#if defined(WIN32)
			IDATA modulesPathLen = cpEntry->pathLength;
			IDATA convertedModulesPathLen = j9str_convert(J9STR_CODE_MUTF8, J9STR_CODE_WINDEFAULTACP, modulesPath, modulesPathLen, NULL, 0);
			if (convertedModulesPathLen > 0) {
				convertedModulesPath = j9mem_allocate_memory(convertedModulesPathLen + 1, OMRMEM_CATEGORY_VM);
				if (convertedModulesPath != NULL) {
					convertedModulesPathLen = j9str_convert(J9STR_CODE_MUTF8, J9STR_CODE_WINDEFAULTACP, modulesPath, modulesPathLen, convertedModulesPath, convertedModulesPathLen);
					if (convertedModulesPathLen > 0) {
						convertedModulesPath[convertedModulesPathLen] = '\0';
						modulesPath = convertedModulesPath;
					} else {
						goto _error;
					}
				} else {
					goto _error;
				}
			} else {
				goto _error;
			}
#endif /* defined(WIN32) */
			rc = jimageIntf->jimageOpen(jimageIntf, modulesPath, &jimageHandle);
#if defined(WIN32)
			if (convertedModulesPath != NULL) {
				j9mem_free_memory(convertedModulesPath);
				convertedModulesPath = NULL;
			}
#endif /* defined(WIN32) */

			if (J9JIMAGE_NO_ERROR == rc) {
				cpEntry->type = CPE_TYPE_JIMAGE;
				cpEntry->extraInfo = (void *)jimageHandle;
				return CPE_TYPE_JIMAGE;
			} else {
				Trc_VM_initializeModulesPathEntry_loadJImageFailed(cpEntry->pathLength, cpEntry->path, rc);
			}
		}
	}

#if defined(WIN32)
_error:
	if (convertedModulesPath != NULL) {
		j9mem_free_memory(convertedModulesPath);
		convertedModulesPath = NULL;
	}
#endif /* defined(WIN32) */

	/* Ok. There's nothing here. Give up. */
	cpEntry->type = CPE_TYPE_UNUSABLE;
	cpEntry->extraInfo = NULL;
	return CPE_TYPE_UNUSABLE;
}
#endif /* J9VM_OPT_DYNAMIC_LOAD_SUPPORT */

/**
 * Create and initialize modules path entries.
 * Currently JVM searches system modules at following locations:
 * 	<JAVA_HOME>/lib/modules - should be a jimage file
 * 	<JAVA-HOME>/modules - should be a directory containing modules in exploded form
 */
IDATA
initializeModulesPath(J9JavaVM *vm)
{
	UDATA rc = 0;
	IDATA modulesPathLen = 0;
	U_8 *modulesPath = NULL;
	J9VMSystemProperty *javaHome = NULL;
	char *javaHomeValue = NULL;
	IDATA javaHomeValueLen = 0;
	PORT_ACCESS_FROM_JAVAVM(vm);

	rc = getSystemProperty(vm, "java.home", &javaHome);
	if (J9SYSPROP_ERROR_NOT_FOUND == rc) {
		goto _error;
	}
	javaHomeValue = javaHome->value;
	javaHomeValueLen = strlen(javaHome->value);

	/* If <JAVA_HOME>/lib/modules is a jimage file, it is used for searching system modules.
	 * If it is not present, then <JAVA_HOME>/modules is searched for modules in exploded form.
	 */
	modulesPathLen = javaHomeValueLen + LITERAL_STRLEN(DIR_SEPARATOR_STR) + LITERAL_STRLEN("lib") + LITERAL_STRLEN(DIR_SEPARATOR_STR) + LITERAL_STRLEN("modules");
	vm->modulesPathEntry = j9mem_allocate_memory(sizeof(J9ClassPathEntry) + modulesPathLen + 1, OMRMEM_CATEGORY_VM);
	if (NULL == vm->modulesPathEntry) {
		goto _error;
	}
	memset(vm->modulesPathEntry, 0, sizeof(J9ClassPathEntry));
	modulesPath = (U_8 *)(vm->modulesPathEntry + 1);
	j9str_printf((char *)modulesPath, (U_32)modulesPathLen + 1, "%s" DIR_SEPARATOR_STR "lib" DIR_SEPARATOR_STR "modules", javaHomeValue);

	vm->modulesPathEntry->path = modulesPath;
	vm->modulesPathEntry->pathLength = (U_32)modulesPathLen;
	rc = initializeModulesPathEntry(vm, vm->modulesPathEntry);
	if (CPE_TYPE_UNUSABLE == rc) {
		vm->modulesPathEntry->type = CPE_TYPE_UNKNOWN;
		/* If <JAVA_HOME>/lib/modules is not usable, try to use <JAVA_HOME>/modules dir */
		modulesPathLen = javaHomeValueLen + LITERAL_STRLEN(DIR_SEPARATOR_STR) + LITERAL_STRLEN("modules");
		j9str_printf((char *)modulesPath, (U_32)modulesPathLen + 1, "%s" DIR_SEPARATOR_STR "modules", javaHomeValue);
		vm->modulesPathEntry->pathLength = (U_32)modulesPathLen;
		rc = initializeModulesPathEntry(vm, vm->modulesPathEntry);
		if (CPE_TYPE_UNUSABLE == rc) {
			goto _error;
		}
	}

	return 0;

_error:
	return -1;
}

BOOLEAN
setBootLoaderModulePatchPaths(J9JavaVM * javaVM, J9Module * j9module, const char * moduleName)
{
	J9VMSystemProperty *property = NULL;
	pool_state walkState = {0};
	UDATA length = strlen(moduleName);
	BOOLEAN result = TRUE;
	char *patchProperty = SYSPROP_JDK_MODULE_PATCH;
	UDATA patchPropLen = sizeof(SYSPROP_JDK_MODULE_PATCH) - 1;
	J9ModuleExtraInfo moduleInfo = {0};
	PORT_ACCESS_FROM_JAVAVM(javaVM);

	Assert_VM_true(j9module->classLoader == javaVM->systemClassLoader);

	property = pool_startDo(javaVM->systemProperties, &walkState);
	while (NULL != property) {
		if ((0 == strncmp(property->name, patchProperty, patchPropLen))
			&& (strlen(property->value) >= (length + 1))
			&& ('=' == property->value[length])
			&& (0 == strncmp(moduleName, property->value, length))
		) {
			char pathSeparator = (char) j9sysinfo_get_classpathSeparator();
			J9ClassLoader *loader = javaVM->systemClassLoader;

			omrthread_monitor_enter(javaVM->classLoaderModuleAndLocationMutex);

			if (NULL == loader->moduleExtraInfoHashTable) {
				loader->moduleExtraInfoHashTable = hashModuleExtraInfoTableNew(javaVM, 1);
				if (NULL == loader->moduleExtraInfoHashTable) {
					result = FALSE;
					goto _exitMutex;
				}
			}

			moduleInfo.j9module = j9module;
			moduleInfo.patchPathCount = initializeClassPath(javaVM, property->value + length + 1, pathSeparator, 0, FALSE, &moduleInfo.patchPathEntries);
			if (-1 == moduleInfo.patchPathCount) {
				result = FALSE;
				goto _exitMutex;
			} else {
				void *node = hashTableAdd(loader->moduleExtraInfoHashTable, (void *)&moduleInfo);
				if (NULL == node) {
					J9VMThread *currentThread = javaVM->internalVMFunctions->currentVMThread(javaVM);
					freeClassLoaderEntries(currentThread, moduleInfo.patchPathEntries, moduleInfo.patchPathCount, moduleInfo.patchPathCount);
#if defined(J9VM_OPT_SNAPSHOTS)
					if (IS_SNAPSHOTTING_ENABLED(javaVM)) {
						VMSNAPSHOTIMPLPORT_ACCESS_FROM_JAVAVM(javaVM);
						vmsnapshot_free_memory(moduleInfo.patchPathEntries);
					} else
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
					{
						j9mem_free_memory(moduleInfo.patchPathEntries);
					}
					moduleInfo.patchPathEntries = NULL;
					result = FALSE;
					goto _exitMutex;
				}
			}

			result = TRUE;
_exitMutex:
			omrthread_monitor_exit(javaVM->classLoaderModuleAndLocationMutex);
			break;
		}
		property = pool_nextDo(&walkState);
	}

	return result;
}

static VMINLINE void
processMemoryInterleaveOptions(J9JavaVM * vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	BOOLEAN enabled = FALSE;
	IDATA argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_OPT_XXINTERLEAVEMEMORY, NULL);
	if (-1 != argIndex) {
		IDATA argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_OPT_XXNOINTERLEAVEMEMORY, NULL);
		if (argIndex > argIndex2) {
			enabled = TRUE;
		}
	}
	j9port_control(J9PORT_CTLDATA_VMEM_NUMA_INTERLEAVE_MEM, enabled? 1 : 0);
}

#if JAVA_SPEC_VERSION >= 19
/**
 * -XX:ContinuationCache:t1=<U_32>,t2=<U_32>
 *
 * This helper searches for and consumes the Continuation cache option,
 * if option found, it is parsed based on the above syntax.
 * if option not found, default values are set for T1 and T2 cache size.
 *
 * Returns 0 on success, -1 if option parsing failed.
 */
static VMINLINE IDATA
processContinuationCacheOptions(J9JavaVM *vm)
{
	IDATA rc = -1;
	IDATA argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXCONTINUATIONCACHE, NULL);
	if (-1 != argIndex) {
		char *cursor = NULL;
		char* scanEnd = NULL;
		U_32 cacheSize = 0;

		GET_OPTION_OPTION(argIndex, ':', ':', &cursor);
		scanEnd = cursor + strlen(cursor);

		while (cursor < scanEnd) {
			if (try_scan(&cursor, "t1=") && (0 == omr_scan_u32(&cursor, &cacheSize))) {
				vm->continuationT1Size = cacheSize;

				if (try_scan(&cursor, ",t2=") && (0 == omr_scan_u32(&cursor, &cacheSize))){
					vm->continuationT2Size = cacheSize;
					rc = 0;
				}
			} else if (try_scan(&cursor, "printSummary")) {
				/* Set VM flag. */
				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_CONTINUATION_CACHE_SUMMARY;
				rc = 0;
			} else {
				rc = -1;
				break;
			}

			/* Skip ',' delimiter. */
			if (',' == *cursor) {
				cursor++;
			}
		}
	}
	/* Set default cache size. */
	if (0 == vm->continuationT1Size) {
		PORT_ACCESS_FROM_JAVAVM(vm);
		vm->continuationT1Size = 1;
		vm->continuationT2Size = (U_32)(j9sysinfo_get_number_CPUs_by_type(J9PORT_CPU_TARGET) * 2);
		rc = 0;
	}

	return rc;
}
#endif /* JAVA_SPEC_VERSION >= 19 */

static VMINLINE void
dumpClassLoader(J9JavaVM *vm, J9ClassLoader *loader, IDATA fd)
{
	J9HashTableState walkState = {0};
	PORT_ACCESS_FROM_JAVAVM(vm);
	J9InternalVMFunctions *vmFuncs = vm->internalVMFunctions;
	J9Class* clazz = vmFuncs->hashClassTableStartDo(loader, &walkState, 0);
	while (NULL != clazz) {
		J9ROMClass* romClass = clazz->romClass;
		J9UTF8* utf = J9ROMCLASS_CLASSNAME(romClass);
		j9file_printf(fd, "%.*s\n", (UDATA)J9UTF8_LENGTH(utf), J9UTF8_DATA(utf));
		clazz = vmFuncs->hashClassTableNextDo(&walkState);
	}
}

void
dumpLoadedClassList(J9HookInterface **hookInterface, uintptr_t eventNum, void *eventData, void *userData)
{
	J9VMShutdownEvent *event = eventData;
	J9JavaVM *vm = event->vmThread->javaVM;
	PORT_ACCESS_FROM_JAVAVM(vm);
	IDATA fd = -1;
	if (NULL != userData) {
		fd = j9file_open((char*) userData, EsOpenWrite | EsOpenCreate, 0666);
		if (-1 == fd) {
			/* If we fail to open file, silently fail. This matches Oracles behaviour */
			return;
		}
		dumpClassLoader(vm, vm->systemClassLoader, fd);
		dumpClassLoader(vm, vm->extensionClassLoader, fd);
		dumpClassLoader(vm, vm->applicationClassLoader, fd);

		j9file_close(fd);
	}

}

/* Print out the internal version information for openj9 */
static void
j9print_internal_version(J9PortLibrary *portLib)
{
	PORT_ACCESS_FROM_PORT(portLib);

#if defined(OPENJ9_BUILD)
#if defined(J9JDK_EXT_VERSION) && defined(J9JDK_EXT_NAME)
	j9tty_err_printf("Eclipse OpenJ9 %s %s-bit Server VM (%s) from %s-%s JRE with %s %s, built on %s %s by %s with %s\n",
			J9PRODUCT_NAME, J9TARGET_CPU_BITS, J9VERSION_STRING, J9TARGET_OS, J9TARGET_CPU_OSARCH,
			J9JDK_EXT_NAME, J9JDK_EXT_VERSION,__DATE__, __TIME__, J9USERNAME, J9COMPILER_VERSION_STRING);
#else
	j9tty_err_printf("Eclipse OpenJ9 %s %s-bit Server VM (%s) from %s-%s JRE, built on %s %s by %s with %s\n",
			J9PRODUCT_NAME, J9TARGET_CPU_BITS, J9VERSION_STRING, J9TARGET_OS, J9TARGET_CPU_OSARCH,
			__DATE__, __TIME__, J9USERNAME, J9COMPILER_VERSION_STRING);
#endif /* J9JDK_EXT_VERSION && J9JDK_EXT_NAME */
#else /* OPENJ9_BUILD */
	j9tty_err_printf("internal version not supported\n");
#endif /* OPENJ9_BUILD */
}

/* The equivalent of a J9VMDllMain for the VM init module */

IDATA
VMInitStages(J9JavaVM *vm, IDATA stage, void* reserved)
{
	J9VMDllLoadInfo *loadInfo = NULL;
	IDATA returnVal = J9VMDLLMAIN_OK;
	IDATA argIndex = -1;
	IDATA argIndex2 = -1;
	char *optionValue = NULL;
	char *parseErrorOption = NULL;
	IDATA parseError = 0;
	BOOLEAN lockwordWhat = FALSE;
	UDATA rc = 0;
	PORT_ACCESS_FROM_JAVAVM(vm);
	OMRPORT_ACCESS_FROM_J9PORT(PORTLIB);

	switch (stage) {
		case PORT_LIBRARY_GUARANTEED :
			processMemoryInterleaveOptions(vm);
			if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, &(vm->classLoadingMaxStack), VMOPT_XMSCL, 0, TRUE))) {
				parseErrorOption = VMOPT_XMSCL;
				goto _memParseError;
			}

			/* Set user-specified CPUs as early as possible, i.e. as soon as PORT_LIBRARY_GUARANTEED */
			argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXACTIVEPROCESSORCOUNT_EQUALS, NULL);
			if (argIndex >= 0) {
				UDATA value = 0;
				char *optname = VMOPT_XXACTIVEPROCESSORCOUNT_EQUALS;

				parseError = GET_INTEGER_VALUE(argIndex, optname, value);
				if (OPTION_OK != parseError) {
					parseErrorOption = VMOPT_XXACTIVEPROCESSORCOUNT_EQUALS;
					goto _memParseError;
				}

				j9sysinfo_set_number_user_specified_CPUs(value);
			}

			/* -Xits option is not being used anymore. We find and consume it for backward compatibility. */
			/* Otherwise, usage of this option would not be recognised and warning would be printed.  */
			FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XITS, NULL);

			if (OPTION_OK != (parseError = setIntegerValueOptionToOptElse(vm, &(vm->maxInvariantLocalTableNodeCount), VMOPT_XITN, J9_INVARIANT_INTERN_TABLE_NODE_COUNT, TRUE))) {
				parseErrorOption = VMOPT_XITN;
				goto _memParseError;
			}

			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XX_NOSUBALLOC32BITMEM, NULL) >= 0) {
				j9port_control(J9PORT_CTLDATA_NOSUBALLOC32BITMEM, J9PORT_DISABLE_ENSURE_CAP32);
			}
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_INTERNALVERSION, NULL) >= 0){
				j9print_internal_version(PORTLIB);
				exit(0);
			}

#if defined(J9VM_OPT_SHARED_CLASSES)
			{
				IDATA argIndex3 = 0;
				IDATA argIndex4 = 0;
				IDATA argIndex5 = 0;
				IDATA argIndex6 = 0;
				IDATA argIndex7 = 0;
				IDATA argIndex8 = 0;
				IDATA argIndex9 = 0;
				BOOLEAN sharedClassDisabled = FALSE;
#if defined(J9VM_OPT_PORTABLE_SHARED_CACHE)
				BOOLEAN processPortableSharedCacheFlag = FALSE;

				IDATA argIndexXXPortableSharedCache = 0;
				IDATA argIndexXXNoPortableSharedCache = 0;
#if defined(J9VM_OPT_JITSERVER)
				IDATA argIndexXXJITServerAOTCache = 0;
				IDATA argIndexXXNoJITServerAOTCache = 0;
				IDATA argIndexXXJITServerAOTCacheIgnoreLocalSCC = 0;
				IDATA argIndexXXNoJITServerAOTCacheIgnoreLocalSCC = 0;
#endif /* defined(J9VM_OPT_JITSERVER) */
#endif /* defined(J9VM_OPT_PORTABLE_SHARED_CACHE) */

				vm->sharedClassPreinitConfig = NULL;
				argIndex = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, VMOPT_XSHARECLASSES, NULL);
				if (argIndex >= 0) {
					char optionsBuffer[2 * J9SH_MAXPATH];
					char* optionsBufferPtr = (char*)optionsBuffer;

					if (GET_OPTION_VALUES(argIndex, ':', ',', &optionsBufferPtr, 2 * J9SH_MAXPATH) == OPTION_OK) {
						while (*optionsBufferPtr) {
							if (try_scan(&optionsBufferPtr, OPT_NONE)) {
								sharedClassDisabled = TRUE;
								break;
							}
							optionsBufferPtr += strlen(optionsBufferPtr) + 1;
						}
					}
				}

				argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCMX, NULL);
				argIndex3 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCMINAOT, NULL);
				argIndex4 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCMAXAOT, NULL);
				argIndex5 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCDMX, NULL);
				argIndex6 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XITSN, NULL);
				argIndex7 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCMINJITDATA, NULL);
				argIndex8 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XSCMAXJITDATA, NULL);
				argIndex9 = FIND_AND_CONSUME_VMARG(EXACT_MEMORY_MATCH, VMOPT_XXSHARED_CACHE_HARD_LIMIT_EQUALS, NULL);

#if defined(J9VM_OPT_PORTABLE_SHARED_CACHE)
				argIndexXXPortableSharedCache = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXPORTABLESHAREDCACHE, NULL);
				argIndexXXNoPortableSharedCache = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOPORTABLESHAREDCACHE, NULL);
#if defined(J9VM_OPT_JITSERVER)
				argIndexXXJITServerAOTCache = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-XX:+JITServerUseAOTCache", NULL);
				argIndexXXNoJITServerAOTCache = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-XX:-JITServerUseAOTCache", NULL);
				argIndexXXJITServerAOTCacheIgnoreLocalSCC = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-XX:+JITServerAOTCacheIgnoreLocalSCC", NULL);
				argIndexXXNoJITServerAOTCacheIgnoreLocalSCC = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-XX:-JITServerAOTCacheIgnoreLocalSCC", NULL);
#endif /* defined(J9VM_OPT_JITSERVER) */
#endif /* defined(J9VM_OPT_PORTABLE_SHARED_CACHE) */

				if (!sharedClassDisabled
					&& !J9_SHARED_CACHE_DEFAULT_BOOT_SHARING(vm)
					&& (argIndex < 0)
				) {
					IDATA shareOrphansArgIndex1 = FIND_ARG_IN_VMARGS(EXACT_MATCH, VMOPT_XXENABLESHAREORPHANS, NULL);
					IDATA shareOrphansArgIndex2 = FIND_ARG_IN_VMARGS(EXACT_MATCH, VMOPT_XXDISABLESHAREORPHANS, NULL);
					/* -XX:+ShareOrphans turns on class sharing automatically. */
					if (shareOrphansArgIndex2 >= shareOrphansArgIndex1) {
						sharedClassDisabled = TRUE;
					}
				}

				if (sharedClassDisabled) {
					if (argIndex2 >= 0) {
						/* If -Xscmx used without -Xshareclasses, don't bomb out with "unrecognised option" */
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCMX_IGNORED);
					} else
					if (argIndex3 >= 0) {
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCMINAOT_IGNORED);
					} else
					if (argIndex4 >= 0) {
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCMAXAOT_IGNORED);
					} else
					if (argIndex5 >= 0) {
						/* If -Xscdmx used without -Xshareclasses, don't bomb out with "unrecognised option" */
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCDMX_IGNORED);
					} else
					if (argIndex6 >= 0) {
						/* If -Xitsn used without -Xshareclasses, don't bomb out with "unrecognised option" */
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XITSN_IGNORED);
					} else
					if (argIndex7 >=0 ) {
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCMINJITDATA_IGNORED);
					} else
					if (argIndex8 >=0 ) {
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XSCMAXJITDATA_IGNORED);
					} else if (argIndex9 >= 0) {
						j9nls_printf(PORTLIB, J9NLS_INFO, J9NLS_VM_XXSHARED_CACHE_HARD_LIMIT_EQUALS_IGNORED);
					}
				} else {
					J9SharedClassPreinitConfig* piConfig;

					if (NULL == (piConfig = (J9SharedClassPreinitConfig*)j9mem_allocate_memory(sizeof(J9SharedClassPreinitConfig), OMRMEM_CATEGORY_VM))) {
						goto _error;
					}
					if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassDebugAreaBytes), VMOPT_XSCDMX, -1, FALSE))) {
						parseErrorOption = VMOPT_XSCDMX;
						goto _memParseError;
					}
					if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassMinAOTSize), VMOPT_XSCMINAOT, -1, FALSE))) {
						parseErrorOption = VMOPT_XSCMINAOT;
						goto _memParseError;
					}
					if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassMaxAOTSize), VMOPT_XSCMAXAOT, -1, FALSE))) {
						parseErrorOption = VMOPT_XSCMAXAOT;
						goto _memParseError;
					}
					if (OPTION_OK != (parseError = setIntegerValueOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassInternTableNodeCount), VMOPT_XITSN, -1, FALSE))) {
						parseErrorOption = VMOPT_XITSN;
						goto _memParseError;
					}
					if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassMinJITSize), VMOPT_XSCMINJITDATA, -1, FALSE))) {
						parseErrorOption = VMOPT_XSCMINJITDATA;
						goto _memParseError;
					}
					if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassMaxJITSize), VMOPT_XSCMAXJITDATA, -1, FALSE))) {
						parseErrorOption = VMOPT_XSCMAXJITDATA;
						goto _memParseError;
					}
					if (argIndex9 >= 0) {
						/* if option "-XX:SharedCacheHardLimit=" presents */
						if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, (UDATA*)&(piConfig->sharedClassSoftMaxBytes), VMOPT_XSCMX, -1, FALSE))) {
							parseErrorOption = VMOPT_XSCMX;
							goto _memParseError;
						}
						if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, &(piConfig->sharedClassCacheSize), VMOPT_XXSHARED_CACHE_HARD_LIMIT_EQUALS, 0, FALSE))) {
							parseErrorOption = VMOPT_XXSHARED_CACHE_HARD_LIMIT_EQUALS;
							goto _memParseError;
						}
					} else {
						piConfig->sharedClassSoftMaxBytes = -1;
						if (OPTION_OK != (parseError = setMemoryOptionToOptElse(vm, &(piConfig->sharedClassCacheSize), VMOPT_XSCMX, 0, FALSE))) {
							parseErrorOption = VMOPT_XSCMX;
							goto _memParseError;
						}
					}
					piConfig->sharedClassReadWriteBytes = -1;					/* -1 == proportion of cache size */
					vm->sharedClassPreinitConfig = piConfig;

					if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_ENABLE_AOT)) {
#if defined(J9VM_OPT_PORTABLE_SHARED_CACHE)
						processPortableSharedCacheFlag = TRUE;
#endif /* defined(J9VM_OPT_PORTABLE_SHARED_CACHE) */
					}
				}
#if defined(J9VM_OPT_PORTABLE_SHARED_CACHE)
#if defined(J9VM_OPT_JITSERVER)
				if ((argIndexXXJITServerAOTCache > argIndexXXNoJITServerAOTCache)
					&& (argIndexXXJITServerAOTCacheIgnoreLocalSCC >= argIndexXXNoJITServerAOTCacheIgnoreLocalSCC)
				) {
					/* If we are using the JITServer AOT cache and ignoring the local SCC (which we detect by checking
					 * that -XX:-JITServerAOTCacheIgnoreLocalSCC is not explicitly set, since it's on by default) then the portable
					 * SCC flag must still be set as necessary, even if the SCC itself is disabled. */
					processPortableSharedCacheFlag = TRUE;
				}
#endif /* defined(J9VM_OPT_JITSERVER) */
				if (TRUE == processPortableSharedCacheFlag) {
					if (argIndexXXPortableSharedCache > argIndexXXNoPortableSharedCache) {
						vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_PORTABLE_SHARED_CACHE;
					} else if (argIndexXXPortableSharedCache == argIndexXXNoPortableSharedCache) {
						/* both "-XX:+PortableSharedCache" and "-XX:-PortableSharedCache" were not found thus following the default behavior for portable shared cache */
						OMRPORT_ACCESS_FROM_J9PORT(vm->portLibrary);
						BOOLEAN inContainer = omrsysinfo_is_running_in_container();
						if (TRUE == inContainer) {
							/* by default, enable portable shared cache in containers and disable otherwise */
							vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_PORTABLE_SHARED_CACHE;
						}
					}
				}
#endif /* defined(J9VM_OPT_PORTABLE_SHARED_CACHE) */
			}
#endif

#if defined(J9VM_OPT_SNAPSHOTS)
			FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XSNAPSHOT, NULL);
#endif /* defined(J9VM_OPT_SNAPSHOTS) */

			/* The -Xdfpbd option is used by JIT, consuming it here to allow VM to continue */
			FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XDFPBD, NULL);

			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XAGGRESSIVE, NULL) >= 0) {
				vm->runtimeFlags |= J9_RUNTIME_AGGRESSIVE;
			}
			processCompressionOptions(vm);



			if (0 != initializeHiddenInstanceFieldsList(vm)) {
				goto _error;
			}

#if defined(J9VM_OPT_JFR)
			if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_JFR_ENABLED)) {
				if (0 != initializeJFRIDs(vm)) {
					goto _error;
				}
			}
#endif /* defined(J9VM_OPT_JFR) */

			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XALLOWCONTENDEDCLASSLOAD, NULL) >= 0) {
				contendedLoadTableFree(vm);
			}

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISCLAIMJITSCRATCH, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODISCLAIMJITSCRATCH, NULL);

#if defined(AIXPPC) || defined(J9ZOS390)
			if (argIndex > argIndex2)  {
				vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_FLAG_JSCRATCH_ADV_ON_FREE;
			}
#else
			/* make +DisclaimJitScratch the default behavior for non AIX platforms*/
			if (argIndex2 < 0 || (argIndex > argIndex2))  {
				vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_FLAG_JSCRATCH_ADV_ON_FREE;
			}
#endif

			IDATA xtuneQuickstartIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_TUNE_QUICKSTART, NULL);
			IDATA xquickstartIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XQUICKSTART, NULL);
			IDATA clientIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_CLIENT, NULL);
			/* VMOPT_TUNE_QUICKSTART, VMOPT_XQUICKSTART and VMOPT_CLIENT are
			 * aliases, so take the largest index of these 3 */
			if (xtuneQuickstartIndex < xquickstartIndex) {
				xtuneQuickstartIndex = xquickstartIndex;
			}
			if (xtuneQuickstartIndex < clientIndex) {
				xtuneQuickstartIndex = clientIndex;
			}
			/* -Xtune:virtualized was added so that consumers could start adding it to their command lines */
			IDATA xtuneVirtualizedIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_TUNE_VIRTUALIZED, NULL);
			IDATA xtuneThroughputIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_TUNE_THROUGHPUT, NULL);

			/* Only the last appearance of any of VMOPT_TUNE_VIRTUALIZED, VMOPT_TUNE_THROUGHPUT, VMOPT_TUNE_QUICKSTART will take effect */
			if (xtuneVirtualizedIndex > xtuneThroughputIndex) {
				if (xtuneVirtualizedIndex > xtuneQuickstartIndex) {
					vm->runtimeFlags |= J9_RUNTIME_TUNE_VIRTUALIZED;
				} else {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_TUNE_QUICKSTART;
				}
			} else {
				if (xtuneThroughputIndex > xtuneQuickstartIndex) {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_TUNE_THROUGHPUT;
				} else if (xtuneQuickstartIndex >= 0) {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_TUNE_QUICKSTART;
				}
			}

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODISCLAIMVIRTUALMEMORY, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISCLAIMVIRTUALMEMORY, NULL);
			{
				IDATA argIndex3;
				argIndex3 = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XSOFTMX, NULL);
				/* if -Xsoftmx is a parameter, +DisclaimVirtualMemory is set */
				if (argIndex3 >= 0) {
					j9port_control(J9PORT_CTLDATA_VMEM_ADVISE_OS_ONFREE, 1);
				}
				/* last instance of +/- DisclaimVirtualMemory found on the command line wins and
				 * overrules -Xsoftmx +DisclaimVirtualMemory setting */
				if (argIndex2 > argIndex) {
					j9port_control(J9PORT_CTLDATA_VMEM_ADVISE_OS_ONFREE, 1);
				} else if (argIndex > argIndex2) {
					j9port_control(J9PORT_CTLDATA_VMEM_ADVISE_OS_ONFREE, 0);
				}
			}

			/* -XX commandline option for +/- TransparentHugepage */
			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOTRANSPARENT_HUGEPAGE, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXTRANSPARENT_HUGEPAGE, NULL);
			{
				/* Last instance of +/- TransparentHugepage found on the command line wins
				 *
				 * Default to use OMR setting (Enable for all Linux with THP set to madvise)
				 */
				if (argIndex2 > argIndex) {
					j9port_control(J9PORT_CTLDATA_VMEM_ADVISE_HUGEPAGE, 1);
				} else if (argIndex > argIndex2) {
					j9port_control(J9PORT_CTLDATA_VMEM_ADVISE_HUGEPAGE, 0);
				}
			}

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNLSMESSAGES, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNONLSMESSAGES, NULL);
			if (argIndex2 > argIndex) {
				j9port_control(OMRPORT_CTLDATA_NLS_DISABLE, 1);
			}

			if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_PAGE_ALIGN_DIRECT_MEMORY)) {
				J9VMSystemProperty* prop = NULL;

				if ((J9SYSPROP_ERROR_NONE != getSystemProperty(vm, "sun.nio.PageAlignDirectMemory", &prop))
				|| (J9SYSPROP_ERROR_NONE != setSystemProperty(vm, prop, "true"))
				) {
					loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_VM_INIT );
					setErrorJ9dll(PORTLIB, loadInfo, "cannot set system property sun.nio.PageAlignDirectMemory=true", FALSE);
					goto _error;
				}
			}

#if !defined(WIN32) && !defined(J9ZTPF)
			/* Override the soft limit on the number of open file descriptors for
			 * compatibility with reference implementation.
			 */
			{
				uint64_t limit = 0;
				uint32_t rc = omrsysinfo_get_limit(OMRPORT_RESOURCE_FILE_DESCRIPTORS | J9PORT_LIMIT_HARD, &limit);
				if (OMRPORT_LIMIT_UNKNOWN != rc ) {
					omrsysinfo_set_limit(OMRPORT_RESOURCE_FILE_DESCRIPTORS | OMRPORT_LIMIT_SOFT, limit);
				}
			}
#endif /* !defined(WIN32) && !defined(J9ZTPF) */

#if defined(AIXPPC)
			/* Override the AIX soft limit on the data segment to avoid getting EAGAIN when creating a new thread,
			 * which results in an OutOfMemoryException. Also provides compatibility with IBM Java 8.
			 */
			{
				uint64_t limit = 0;
				uint32_t rc = omrsysinfo_get_limit(OMRPORT_RESOURCE_DATA | OMRPORT_LIMIT_SOFT, &limit);
				if (OMRPORT_LIMIT_UNLIMITED != rc) {
					uint32_t rc = omrsysinfo_get_limit(OMRPORT_RESOURCE_DATA | OMRPORT_LIMIT_HARD, &limit);
					if (OMRPORT_LIMIT_UNKNOWN != rc) {
						omrsysinfo_set_limit(OMRPORT_RESOURCE_DATA | OMRPORT_LIMIT_SOFT, limit);
					}
				}
			}
#endif /* defined(AIXPPC) */

			/* Parse options related to idle tuning */
			{
				IDATA argIndexGcOnIdleEnable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGGCONIDLEENABLE, NULL);
				IDATA argIndexGcOnIdleDisable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGGCONIDLEDISABLE, NULL);
				IDATA argIndexCompactOnIdleEnable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGCOMPACTONIDLEENABLE, NULL);
				IDATA argIndexCompactOnIdleDisable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGCOMPACTONIDLEDISABLE, NULL);
				IDATA argIndexIgnoreUnrecognizedOptionsEnable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGIGNOREUNRECOGNIZEDOPTIONSENABLE, NULL);
				IDATA argIndexIgnoreUnrecognizedOptionsDisable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXIDLETUNINGIGNOREUNRECOGNIZEDOPTIONSDISABLE, NULL);
				BOOLEAN enableGcOnIdle = FALSE;
				BOOLEAN inContainer = omrsysinfo_is_running_in_container();

				/*
				 * GcOnIdle is enabled only if:
				 * 1. -XX:+IdleTuningGcOnIdle is set, or
				 * 2. running in container, or
				 * 3. if java version is 9 or above and -Xtune:virtualized is set as VM option
				 */
				if (argIndexGcOnIdleEnable > argIndexGcOnIdleDisable) {
					enableGcOnIdle = TRUE;
				} else if (-1 == argIndexGcOnIdleDisable) {
					if (inContainer
						|| ((J2SE_VERSION(vm) >= J2SE_V11) && J9_ARE_ANY_BITS_SET(vm->runtimeFlags, J9_RUNTIME_TUNE_VIRTUALIZED))
					) {
						enableGcOnIdle = TRUE;
					}
				}
				if (TRUE == enableGcOnIdle) {
					vm->vmRuntimeStateListener.idleTuningFlags |= (UDATA)J9_IDLE_TUNING_GC_ON_IDLE;
					/*
					 *
					 * CompactOnIdle is enabled only if XX:+IdleTuningGcOnIdle is set and
					 * -XX:+IdleTuningCompactOnIdle is set
					 *
					 * Setting Xtune:virtualized on java versions 9 or above does not enable CompactOnIdle.
					 */
					if (argIndexCompactOnIdleEnable > argIndexCompactOnIdleDisable) {
						vm->vmRuntimeStateListener.idleTuningFlags |= (UDATA)J9_IDLE_TUNING_COMPACT_ON_IDLE;
					} else {
						vm->vmRuntimeStateListener.idleTuningFlags &= ~(UDATA)J9_IDLE_TUNING_COMPACT_ON_IDLE;
					}
				} else {
					vm->vmRuntimeStateListener.idleTuningFlags &= ~(UDATA)J9_IDLE_TUNING_GC_ON_IDLE;
				}
				/* default ignore if idle tuning options not supported */
				vm->vmRuntimeStateListener.idleTuningFlags |= (UDATA)J9_IDLE_TUNING_IGNORE_UNRECOGNIZED_OPTIONS;
				if (argIndexIgnoreUnrecognizedOptionsDisable > argIndexIgnoreUnrecognizedOptionsEnable) {
					vm->vmRuntimeStateListener.idleTuningFlags &= ~(UDATA)J9_IDLE_TUNING_IGNORE_UNRECOGNIZED_OPTIONS;
				}

				if (J9_ARE_ANY_BITS_SET(vm->vmRuntimeStateListener.idleTuningFlags, J9_IDLE_TUNING_GC_ON_IDLE | J9_IDLE_TUNING_COMPACT_ON_IDLE)) {
					vm->vmRuntimeStateListener.minIdleWaitTime = 180000; /* in msecs */
					vm->vmRuntimeStateListener.idleMinFreeHeap = 0;
				}

				if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXIDLETUNINGMINIDLEWATITIME_EQUALS, NULL)) >= 0) {
					UDATA value;
					char *optname = VMOPT_XXIDLETUNINGMINIDLEWATITIME_EQUALS;

					parseError = GET_INTEGER_VALUE(argIndex, optname, value);
					if (OPTION_OK != parseError) {
						parseErrorOption = VMOPT_XXIDLETUNINGMINIDLEWATITIME_EQUALS;
						goto _memParseError;
					}

					if (value > (INT_MAX / 1000)) {
						parseErrorOption = VMOPT_XXIDLETUNINGMINIDLEWATITIME_EQUALS;
						parseError = OPTION_OVERFLOW;
						goto _memParseError;
					}

					vm->vmRuntimeStateListener.minIdleWaitTime = (U_32)value * 1000; /* convert to msecs */
				}

				if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXIDLETUNINGMINFREEHEAPONIDLE_EQUALS, NULL)) >= 0) {
					UDATA value;
					char *optname = VMOPT_XXIDLETUNINGMINFREEHEAPONIDLE_EQUALS;

					parseError = GET_INTEGER_VALUE(argIndex, optname, value);
					if (OPTION_OK != parseError) {
						parseErrorOption = VMOPT_XXIDLETUNINGMINFREEHEAPONIDLE_EQUALS;
						goto _memParseError;
					}

					if (value > 100) {
						parseErrorOption = VMOPT_XXIDLETUNINGMINFREEHEAPONIDLE_EQUALS;
						parseError = OPTION_OUTOFRANGE;
						goto _memParseError;
					}

					vm->vmRuntimeStateListener.idleMinFreeHeap = value;
				}
			}
			vm->vmRuntimeStateListener.runtimeStateListenerState = J9VM_RUNTIME_STATE_LISTENER_UNINITIALIZED;
			vm->vmRuntimeStateListener.vmRuntimeState = J9VM_RUNTIME_STATE_ACTIVE;

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXUSECONTAINERSUPPORT, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOUSECONTAINERSUPPORT, NULL);

			/* Enable -XX:+UseContainerSupport by default */
			if (argIndex >= argIndex2) {
				uint64_t subsystemsEnabled = omrsysinfo_cgroup_enable_subsystems(OMR_CGROUP_SUBSYSTEM_ALL);

				if (OMR_CGROUP_SUBSYSTEM_ALL != subsystemsEnabled) {
					uint64_t subsystemsAvailable = omrsysinfo_cgroup_get_available_subsystems();
					Trc_VM_CgroupSubsystemsNotEnabled(vm->mainThread, subsystemsAvailable, subsystemsEnabled);
				}

				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_USE_CONTAINER_SUPPORT;
			}

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDEEP_SCAN, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODEEP_SCAN, NULL);

			/* Enable Deep Structure Priority Scan by default */
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_DEEPSCAN;
			if (argIndex2 > argIndex) {
				vm->extendedRuntimeFlags2 &= ~J9_EXTENDED_RUNTIME2_ENABLE_DEEPSCAN;
			}

			parseError = setMemoryOptionToOptElse(vm, &(vm->directByteBufferMemoryMax),
					VMOPT_XXMAXDIRECTMEMORYSIZEEQUALS, ~(UDATA)0, TRUE);
			if (OPTION_OK != parseError) {
				parseErrorOption = VMOPT_XXMAXDIRECTMEMORYSIZEEQUALS;
				goto _memParseError;
			}

			/* workaround option in case if OMRPORT_VMEM_ALLOC_QUICK Smart Address feature still be not reliable  */
			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOFORCE_FULL_HEAP_ADDRESS_RANGE_SEARCH, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXFORCE_FULL_HEAP_ADDRESS_RANGE_SEARCH, NULL);

			if (argIndex2 > argIndex) {
				j9port_control(OMRPORT_CTLDATA_VMEM_PERFORM_FULL_MEMORY_SEARCH, 1);
			} else {
				j9port_control(OMRPORT_CTLDATA_VMEM_PERFORM_FULL_MEMORY_SEARCH, 0);
			}

#if JAVA_SPEC_VERSION >= 15
			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSHOW_EXTENDED_NPE_MESSAGE, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOSHOW_EXTENDED_NPE_MESSAGE, NULL);
			if (argIndex2 <= argIndex) {
				vm->requiredDebugAttributes |= J9VM_DEBUG_ATTRIBUTE_LOCAL_VARIABLE_TABLE;
				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_SHOW_EXTENDED_NPEMSG;
			}
#endif /* JAVA_SPEC_VERSION >= 15 */

			break;

		case ALL_DEFAULT_LIBRARIES_LOADED :
			vm->exitHook = (J9_EXIT_HANDLER_PROC) J9_COMPATIBLE_FUNCTION_POINTER(getOptionExtraInfo(PORTLIB, vm->vmArgsArray, EXACT_MATCH, VMOPT_EXIT));
			vm->abortHook = (J9_ABORT_HANDLER_PROC) J9_COMPATIBLE_FUNCTION_POINTER(getOptionExtraInfo(PORTLIB, vm->vmArgsArray, EXACT_MATCH, VMOPT_ABORT));

			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXDECOMP_COLON, NULL)) >= 0) {
				GET_OPTION_VALUE(argIndex, ':', &optionValue);
				vm->decompileName = optionValue;
			}

			/* Warm up the VM Interface */
			if (VMI_ERROR_NONE != J9VMI_Initialize(vm)) {
				goto _error;
			}

			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_SHOWVERSION, NULL) >= 0) {
				vm->runtimeFlags |= J9_RUNTIME_SHOW_VERSION;
			}

#if defined(J9ZOS390)
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XASCII_FILETAG, NULL) >= 0) {
				atoe_enableFileTagging();
			}
#endif
			break;

		case ALL_LIBRARIES_LOADED :
#if defined(J9VM_OPT_JVMTI)
			detectAgentXruns(vm);
#endif
			/* Idle GC tuning is currently enabled for Linux x86-64, i386. For other supported GC policies and platforms, need further
			 * study as heap layout/structure and commit/decommit native support differs. For example, balanced GC policy: the logic
			 * needs to arrive at the right no. & regions from right set needs to picked up for releasing the pages. Line Item: 50329
			 * continued to be extended for other GC policies and platforms.
			 */
			if (J9_ARE_ANY_BITS_SET(vm->vmRuntimeStateListener.idleTuningFlags, J9_IDLE_TUNING_GC_ON_IDLE | J9_IDLE_TUNING_COMPACT_ON_IDLE)) {
				BOOLEAN idleGCTuningSupported = FALSE;
#if (defined(LINUX) && (defined(J9HAMMER) || defined(J9X86) || defined(S39064) || defined(PPC64) || defined(J9AARCH64) || defined(RISCV64))) || defined(J9ZOS39064)
				/* & only for gencon GC policy */
				if (J9_GC_POLICY_GENCON == ((OMR_VM *)vm->omrVM)->gcPolicy) {
					idleGCTuningSupported = TRUE;
				}
#endif
				if (!idleGCTuningSupported) {
					vm->vmRuntimeStateListener.idleTuningFlags &= ~(J9_IDLE_TUNING_GC_ON_IDLE | J9_IDLE_TUNING_COMPACT_ON_IDLE);
					if (J9_IDLE_TUNING_IGNORE_UNRECOGNIZED_OPTIONS != (vm->vmRuntimeStateListener.idleTuningFlags & J9_IDLE_TUNING_IGNORE_UNRECOGNIZED_OPTIONS)) {
						j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_IDLE_TUNING_PLATFORM_OR_GC_POLICY_NOT_SUPPORTED);
						goto _error;
					}
				}
			}
			break;

		case DLL_LOAD_TABLE_FINALIZED :
			break;
		case VM_THREADING_INITIALIZED :
			break;
		case HEAP_STRUCTURES_INITIALIZED :
#ifdef J9VM_OPT_VM_LOCAL_STORAGE
			initializeVMLocalStorage(vm);
#endif

			if (NULL == (vm->jniGlobalReferences = pool_new(sizeof(UDATA), 0, 0, POOL_NO_ZERO, J9_GET_CALLSITE(), J9MEM_CATEGORY_JNI, POOL_FOR_PORT(vm->portLibrary)))) {
				goto _error;
			}

			if (0 != initializeNativeMethodBindTable(vm)) {
				goto _error;
			}
			initializeJNITable(vm);
			/* vm->jniFunctionTable = GLOBAL_TABLE(EsJNIFunctions); */

			if (NULL == (vm->jniWeakGlobalReferences = pool_new(sizeof(UDATA), 0, 0, POOL_NO_ZERO, J9_GET_CALLSITE(), J9MEM_CATEGORY_JNI, POOL_FOR_PORT(vm->portLibrary))))
				goto _error;

			if (NULL == (vm->classLoadingStackPool = pool_new(sizeof(J9StackElement),  0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_CLASSES, POOL_FOR_PORT(vm->portLibrary))))
				goto _error;

#if defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES)
			if (NULL == (vm->valueTypeVerificationStackPool = pool_new(sizeof(J9StackElement),  0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_CLASSES, POOL_FOR_PORT(vm->portLibrary))))
				goto _error;
#endif /* defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES)*/

#if defined(J9VM_OPT_SNAPSHOTS)
			/* By this point during a restore run, the immortal class loaders are restored. */
			if (IS_SNAPSHOT_RUN(vm)) {
				if (NULL == (vm->classLoaderBlocks = pool_new(sizeof(J9ClassLoader), 0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_CLASSES, POOL_FOR_PORT(VMSNAPSHOTIMPL_OMRPORT_FROM_JAVAVM(vm))))) {
					goto _error;
				}
			} else if (!IS_RESTORE_RUN(vm))
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
			{
				if (NULL == (vm->classLoaderBlocks = pool_new(sizeof(J9ClassLoader), 0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_CLASSES, POOL_FOR_PORT(vm->portLibrary)))) {
					goto _error;
				}
			}

			if (J2SE_VERSION(vm) >= J2SE_V11) {
#if defined(J9VM_OPT_SNAPSHOTS)
				/* By this point during a restore run, the modularityPool is restored. */
				if (IS_SNAPSHOT_RUN(vm)) {
					if (NULL == (vm->modularityPool = pool_new(OMR_MAX(sizeof(J9Package), sizeof(J9Module)), 0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_MODULES, POOL_FOR_PORT(VMSNAPSHOTIMPL_OMRPORT_FROM_JAVAVM(vm))))) {
						goto _error;
					}
				} else if (!IS_RESTORE_RUN(vm))
#endif /* defined(J9VM_OPT_SNAPSHOTS) */
				{
					if (NULL == (vm->modularityPool = pool_new(OMR_MAX(sizeof(J9Package), sizeof(J9Module)), 0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_MODULES, POOL_FOR_PORT(vm->portLibrary)))) {
						goto _error;
					}
				}
			}
#if JAVA_SPEC_VERSION >= 19
			if (NULL == (vm->tlsPool = pool_new(sizeof(void *) * J9JVMTI_MAX_TLS_KEYS,  0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_JVMTI, POOL_FOR_PORT(vm->portLibrary))))
				goto _error;
#endif /* JAVA_SPEC_VERSION >= 19 */

			break;

		case ALL_VM_ARGS_CONSUMED :
			/*
			 * defer initialization of network data until after heap allocated, since the initialization
			 * can initiate DLL loads which prevent allocation of large heaps.
			 */
			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXREADIPINFOFORRAS, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOREADIPINFOFORRAS, NULL);
			if (argIndex >= argIndex2) {
				JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\t\tenabled network query to determine host name and IP address for RAS.\n");
				populateRASNetData(vm, vm->j9ras);
			} else {
				JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\t\tdisabled network query to determine host name and IP address for RAS.\n");
			}

			consumeVMArgs(vm, vm->vmArgsArray);
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXALLOWVMSHUTDOWN, NULL) >= 0) {
				vm->runtimeFlags &= ~(UDATA)J9_RUNTIME_DISABLE_VM_SHUTDOWN;
			}
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEVMSHUTDOWN, NULL) >= 0) {
				vm->runtimeFlags |= J9_RUNTIME_DISABLE_VM_SHUTDOWN;
			}
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XFUTURE, NULL) >= 0) {
				/* Launcher converts -Xfuture to -Xverify:all
				 * Runtimeflag XFUTURE can also be set in bcverify.c if -Xverify:all is seen
				 */
				vm->runtimeFlags |= J9_RUNTIME_XFUTURE;
			}
			if ((J2SE_VERSION(vm) & J2SE_VERSION_MASK) >= J2SE_V11) {
				if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_ENABLE_PREVIEW, NULL) >= 0) {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_PREVIEW;
				}
			}
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XSIGQUITTOFILE, NULL)) >= 0) {
				GET_OPTION_VALUE(argIndex, ':', &optionValue);
				vm->sigquitToFileDir = (optionValue == NULL) ? "." : optionValue;
			} else {
				vm->sigquitToFileDir = NULL;
			}
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XFASTRESOLVE, NULL)) >= 0) {
				UDATA t;
				char *optname = VMOPT_XFASTRESOLVE;
				GET_INTEGER_VALUE(argIndex, optname, t);
				vm->fieldIndexThreshold = t;
			} else {
				vm->fieldIndexThreshold = 65000;
			}
			/* Consumed here as the option is dealt with before the consumed args list exists */
			FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XOPTIONSFILE_EQUALS, NULL);

#ifdef J9VM_OPT_METHOD_HANDLE
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXMHCOMPILECOUNT_EQUALS, NULL)) >= 0) {
				UDATA mhCompileCount = 0;
				char *optname = VMOPT_XXMHCOMPILECOUNT_EQUALS;
				GET_INTEGER_VALUE(argIndex, optname, mhCompileCount);
				vm->methodHandleCompileCount = mhCompileCount;
			} else {
				vm->methodHandleCompileCount = 30;
			}
#endif
			vm->romMethodSortThreshold = UDATA_MAX;
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_ROMMETHODSORTTHRESHOLD_EQUALS, NULL)) >= 0) {
				UDATA threshold = 0;
				char *optname = VMOPT_ROMMETHODSORTTHRESHOLD_EQUALS;
				GET_INTEGER_VALUE(argIndex, optname, threshold);
				if (threshold > 0) {
					vm->romMethodSortThreshold = threshold;
				}
			}

#if defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES)
			/* By default flattening is disabled */
			vm->valueFlatteningThreshold = 0;
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_VALUEFLATTENINGTHRESHOLD_EQUALS, NULL)) >= 0) {
				UDATA threshold = 0;
				char *optname = VMOPT_VALUEFLATTENINGTHRESHOLD_EQUALS;
				GET_INTEGER_VALUE(argIndex, optname, threshold);
				vm->valueFlatteningThreshold = threshold;
			}

			{
				IDATA enableFlattenedArrays = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_VTARRAYFLATTENING_EQUALS, NULL);
				IDATA disableFlattenedArrays = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_VTDISABLEARRAYFLATTENING_EQUALS, NULL);
				if (disableFlattenedArrays > enableFlattenedArrays) {
					vm->extendedRuntimeFlags2 &= ~J9_EXTENDED_RUNTIME2_ENABLE_VT_ARRAY_FLATTENING;
				} else if (disableFlattenedArrays < enableFlattenedArrays) {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_VT_ARRAY_FLATTENING;
				}
			}
#endif /* defined(J9VM_OPT_VALHALLA_FLATTENABLE_VALUE_TYPES) */
#if JAVA_SPEC_VERSION >= 16
			if ((argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDIAGNOSE_SYNC_ON_VALUEBASED_CLASSES_EQUALS1, NULL)) >= 0) {
				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_VALUE_BASED_EXCEPTION;
			} else if ((argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDIAGNOSE_SYNC_ON_VALUEBASED_CLASSES_EQUALS2, NULL)) >= 0) {
				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_VALUE_BASED_WARNING;
			}
#endif /* JAVA_SPEC_VERSION >= 16 */
#if JAVA_SPEC_VERSION >= 19
			if (0 != processContinuationCacheOptions(vm)) {
				parseErrorOption = VMOPT_XXCONTINUATIONCACHE;
				parseError = OPTION_MALFORMED;
				goto _memParseError;
			}
#endif /* JAVA_SPEC_VERSION >= 19 */
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXDUMPLOADEDCLASSLIST, NULL)) >= 0) {
				J9HookInterface **vmHooks = vm->internalVMFunctions->getVMHookInterface(vm);
				GET_OPTION_VALUE(argIndex, '=', &optionValue);
				if (NULL == optionValue){
					parseErrorOption = VMOPT_XXDUMPLOADEDCLASSLIST;
					parseError = OPTION_MALFORMED;
					goto _memParseError;
				}
				(*vmHooks)->J9HookRegisterWithCallSite(vmHooks, J9HOOK_VM_SHUTTING_DOWN, dumpLoadedClassList, OMR_GET_CALLSITE(), optionValue);
			}

#if defined(AIXPPC)
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSETHWPREFETCH_NONE, NULL) >= 0) {
				vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_SET_HW_PREFETCH;
			}
			if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSETHWPREFETCH_OS_DEFAULT, NULL) >= 0) {
				vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_SET_HW_PREFETCH;
			}
			if (FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXSETHWPREFETCH_EQUALS, NULL) >= 0) {
				vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_SET_HW_PREFETCH;
			}
#if defined(OPENJ9_BUILD)
			FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXUSEZLIBNX, NULL);
			FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOUSEZLIBNX, NULL);
#endif /* defined(OPENJ9_BUILD) */
#endif /* defined(AIXPPC) */

			/* set the default mode */
			vm->lockwordMode =LOCKNURSERY_ALGORITHM_ALL_BUT_ARRAY;

			/* parse the lockword options */
			argIndex = FIND_AND_CONSUME_VMARG_FORWARD(STARTSWITH_MATCH, VMOPT_XLOCKWORD, NULL);
			while(argIndex >= 0){
				optionValue = NULL;
				GET_OPTION_VALUE(argIndex, ':', &optionValue);
				if (JNI_OK != parseLockwordConfig(vm,optionValue,&lockwordWhat)){
					goto _error;
				}
				argIndex = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XLOCKWORD, NULL, argIndex);
			}
			if (TRUE == lockwordWhat){
				printLockwordWhat(vm);
			}

			argIndex = FIND_AND_CONSUME_VMARG_FORWARD(STARTSWITH_MATCH, VMOPT_XXENABLEENSUREHASHED, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG_FORWARD(STARTSWITH_MATCH, VMOPT_XXDISABLEENSUREHASHED, NULL);
			while ((argIndex >= 0) || (argIndex2 >= 0)) {
				optionValue = NULL;
				BOOLEAN isAdd = FALSE;

				if (argIndex < argIndex2) {
					if (argIndex < 0) {
						/* Disable only */
						GET_OPTION_OPTION(argIndex2, ':', ':', &optionValue);
						isAdd = FALSE;
						argIndex2 = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XXDISABLEENSUREHASHED, NULL, argIndex2);
					} else {
						/* First parse the enable */
						GET_OPTION_OPTION(argIndex, ':', ':', &optionValue);
						isAdd = TRUE;
						argIndex = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XXENABLEENSUREHASHED, NULL, argIndex);
					}
				} else if (argIndex2 < argIndex) {
					if (argIndex2 < 0) {
						/* Enable only */
						GET_OPTION_OPTION(argIndex, ':', ':', &optionValue);
						isAdd = TRUE;
						argIndex = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XXENABLEENSUREHASHED, NULL, argIndex);
					} else {
						/* First parse the disable */
						GET_OPTION_OPTION(argIndex2, ':', ':', &optionValue);
						isAdd = FALSE;
						argIndex2 = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XXDISABLEENSUREHASHED, NULL, argIndex2);
					}
				}

				if (JNI_OK != parseEnsureHashedConfig(vm, optionValue, isAdd)){
					goto _error;
				}
			}

			/* Global Lock Reservation is currently only supported on Power. */
#if defined(AIXPPC) || defined(LINUXPPC)
			/* Global Lock Reservation is ON by default on Power. */
			vm->enableGlobalLockReservation = 1;
#else
			/* Global Lock Reservation is OFF by default on other platforms. */
			vm->enableGlobalLockReservation = 0;
#endif /* defined(AIXPPC) || defined(LINUXPPC) */

			/* Set default parameters for Global Lock Reservation. */
			vm->reservedTransitionThreshold = 1;
			vm->reservedAbsoluteThreshold = 10;
			vm->minimumReservedRatio = 1024;
			vm->cancelAbsoluteThreshold = 10;
			vm->minimumLearningRatio = 256;

			argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOGLOBALLOCKRESERVATION, NULL);
			argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXGLOBALLOCKRESERVATION, NULL);

			if ((argIndex >= 0) && (argIndex > argIndex2)) {
				vm->enableGlobalLockReservation = 0;
			}

			argIndex2 = FIND_AND_CONSUME_VMARG_FORWARD(STARTSWITH_MATCH, VMOPT_XXGLOBALLOCKRESERVATIONCOLON, NULL);

			while (argIndex2 >= 0) {
				/* Global Lock Reservation is currently only supported on Power. */
#if defined(AIXPPC) || defined(LINUXPPC)
				if (argIndex2 > argIndex) {
					vm->enableGlobalLockReservation = 1;
				}
#endif /* defined(AIXPPC) || defined(LINUXPPC) */

				optionValue = NULL;
				GET_OPTION_OPTION(argIndex2, ':', ':', &optionValue);

				if (JNI_OK != parseGlrConfig(vm, optionValue)) {
					goto _error;
				}
				argIndex2 = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XXGLOBALLOCKRESERVATIONCOLON, NULL, argIndex2);
			}

#if defined(J9VM_OPT_CRIU_SUPPORT)
			vm->checkpointState.maxRetryForNotCheckpointSafe = 100;
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXMAXRETRYFORNOTCHECKPOINTSAFE_EQUALS, NULL)) >= 0) {
				UDATA maxRetryForNotCheckpointSafe = 0;
				char *optname = VMOPT_XXMAXRETRYFORNOTCHECKPOINTSAFE_EQUALS;
				parseError = GET_INTEGER_VALUE(argIndex, optname, maxRetryForNotCheckpointSafe);
				if (OPTION_OK != parseError) {
					parseErrorOption = VMOPT_XXMAXRETRYFORNOTCHECKPOINTSAFE_EQUALS;
					goto _memParseError;
				}
				vm->checkpointState.maxRetryForNotCheckpointSafe = maxRetryForNotCheckpointSafe;
			}

			vm->checkpointState.sleepMillisecondsForNotCheckpointSafe = 10;
			if ((argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXSLEEPMILLISECONDSFORNOTCHECKPOINTSAFE_EQUALS, NULL)) >= 0) {
				UDATA sleepMillisecondsForNotCheckpointSafe = 0;
				char *optname = VMOPT_XXSLEEPMILLISECONDSFORNOTCHECKPOINTSAFE_EQUALS;
				parseError = GET_INTEGER_VALUE(argIndex, optname, sleepMillisecondsForNotCheckpointSafe);
				if (OPTION_OK != parseError) {
					parseErrorOption = VMOPT_XXSLEEPMILLISECONDSFORNOTCHECKPOINTSAFE_EQUALS;
					goto _memParseError;
				}
				if (sleepMillisecondsForNotCheckpointSafe < 1) {
					parseErrorOption = VMOPT_XXSLEEPMILLISECONDSFORNOTCHECKPOINTSAFE_EQUALS;
					parseError = OPTION_OUTOFRANGE;
					goto _memParseError;
				}
				vm->checkpointState.sleepMillisecondsForNotCheckpointSafe = sleepMillisecondsForNotCheckpointSafe;
			}

			vm->checkpointState.requiredGhostFileLimit = 1024 * 1024; /* 1 MB is the default ghost limit set by the CRIU library*/
#endif /* defined(J9VM_OPT_CRIU_SUPPORT) */

			break;

		case BYTECODE_TABLE_SET:
			if (J2SE_VERSION(vm) >= J2SE_V11) {
				rc = initializeModulesPath(vm);
				if (0 != rc) {
					loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_VM_INIT );
					setErrorJ9dll(PORTLIB, loadInfo, "cannot initialize modules path", FALSE);
					goto _error;
				}
			}

			break;

		case SYSTEM_CLASSLOADER_SET :

			loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_VM_INIT );
			/* systemClassLoader is already set during the restore run. */
			if (!IS_RESTORE_RUN(vm)) {
				if (NULL == (vm->systemClassLoader = allocateClassLoader(vm))) {
					setErrorJ9dll(PORTLIB, loadInfo, "cannot allocate system classloader", FALSE);
					goto _error;
				}
			}

			if (J2SE_VERSION(vm) >= J2SE_V11) {
				/* javaBaseModule and unnamedModuleForSystemLoader are already setup during a restore run. */
				if (!IS_RESTORE_RUN(vm)) {
					BOOLEAN patchPathResult = FALSE;
					vm->javaBaseModule = pool_newElement(vm->modularityPool);
					if (NULL == vm->javaBaseModule) {
						setErrorJ9dll(PORTLIB, loadInfo, "cannot allocate java.base module", FALSE);
						goto _error;
					}
					vm->javaBaseModule->classLoader = vm->systemClassLoader;

					vm->unnamedModuleForSystemLoader = pool_newElement(vm->modularityPool);
					if (NULL == vm->unnamedModuleForSystemLoader) {
						setErrorJ9dll(PORTLIB, loadInfo, "cannot allocate unnamed module for bootloader", FALSE);
						goto _error;
					}
					vm->unnamedModuleForSystemLoader->classLoader = vm->systemClassLoader;

					patchPathResult = setBootLoaderModulePatchPaths(vm, vm->javaBaseModule, JAVA_BASE_MODULE);
					if (FALSE == patchPathResult) {
						setErrorJ9dll(PORTLIB, loadInfo, "cannot set patch paths for java.base module", FALSE);
						goto _error;
					}
				}
			}

#if defined(J9VM_OPT_INVARIANT_INTERNING)
			/* the system class loader does not get unloaded, and thus is safe to share its strings for string interning */
			vm->systemClassLoader->flags |= J9CLASSLOADER_INVARIANTS_SHARABLE;
			if ((NULL != vm->dynamicLoadBuffers) && (J9_ARE_ANY_BITS_SET(vm->dynamicLoadBuffers->flags, BCU_ENABLE_INVARIANT_INTERNING))) {
				/* vm->sharedClassPreinitConfig is not NULL if shareclasses is enabled */
				if (NULL != vm->sharedClassPreinitConfig) {
					if ((0 == vm->maxInvariantLocalTableNodeCount) && ((0 == vm->sharedClassPreinitConfig->sharedClassInternTableNodeCount) || (NULL == vm->sharedClassConfig))) {
						vm->dynamicLoadBuffers->flags &= ~BCU_ENABLE_INVARIANT_INTERNING;
					}
				}  else {
					if (0 == vm->maxInvariantLocalTableNodeCount) {
						vm->dynamicLoadBuffers->flags &= ~BCU_ENABLE_INVARIANT_INTERNING;
					}
				}
			}

#endif
			vm->pathSeparator = DIR_SEPARATOR;

#ifdef J9VM_PROF_COUNT_ARGS_TEMPS
			{
				UDATA i;

				vm->reserved2 = j9mem_allocate_memory((256 * sizeof(UDATA *)), OMRMEM_CATEGORY_VM);
				for (i = 0; i < 256; ++i) {
					((UDATA **) vm->reserved2)[i] = j9mem_allocate_memory((256 * sizeof(UDATA)), OMRMEM_CATEGORY_VM);
					memset(((UDATA **) vm->reserved2)[i], 0, 256 * sizeof(UDATA));
				}
			}
#endif
			vm->callInReturnPC = (U_8 *) J9CallInReturnPC;
#if defined(J9VM_OPT_METHOD_HANDLE)
			vm->impdep1PC = (U_8 *) J9Impdep1PC;
#endif /* defined(J9VM_OPT_METHOD_HANDLE) */
			break;

		case JIT_INITIALIZED :
			/* Register this module with trace */
			UT_MODULE_LOADED(J9_UTINTERFACE_FROM_VM(vm));
			Trc_VM_VMInitStages_Event1(vm->mainThread);
			Trc_VM_Created(vm->mainThread,
				vm,
				vm->internalVMFunctions,
				vm->portLibrary,
				vm->j9ras
			);

#if defined(OMR_THR_YIELD_ALG) && defined(LINUX)
			Trc_VM_yieldAlgorithmSelected(vm->mainThread, j9util_sched_compat_yield_value(vm), **(UDATA**)omrthread_global("yieldAlgorithm"), **(UDATA**)omrthread_global("yieldUsleepMultiplier"));
#endif /* defined(OMR_THR_YIELD_ALG) && defined(LINUX) */

			if (vm->dynamicLoadBuffers && (vm->dynamicLoadBuffers->flags & BCU_ENABLE_INVARIANT_INTERNING)) {
				Trc_VM_VMInitStages_InvariantInterningEnabled();
			} else {
				Trc_VM_VMInitStages_InvariantInterningDisabled();
			}
			Trc_VM_classloaderLocking(J9_ARE_NO_BITS_SET(vm->extendedRuntimeFlags,J9_EXTENDED_RUNTIME_CLASSLOADER_LOCKING_ENABLED)? "disabled": "enabled" );

			if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_REDUCE_CPU_MONITOR_OVERHEAD)) {
				Trc_VM_VMInitStages_ReduceCPUMonitorOverhead("enabled", "false");
			} else {
				Trc_VM_VMInitStages_ReduceCPUMonitorOverhead("disabled", "true");
			}

#if defined(AIXPPC)
#if defined(J9OS_I5)
	/* Nothing to do, as IBM i does not support the customer modifying the DSCR. */
#else
		if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_SET_HW_PREFETCH)) {
			IDATA xxSetPrefetchOSDfltIndex = FIND_ARG_IN_VMARGS(EXACT_MATCH, VMOPT_XXSETHWPREFETCH_OS_DEFAULT, NULL);
			IDATA xxSetPrefetchNoneIndex = FIND_ARG_IN_VMARGS(EXACT_MATCH, VMOPT_XXSETHWPREFETCH_NONE, NULL);
			IDATA xxSetPrefetchIndex = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, VMOPT_XXSETHWPREFETCH_EQUALS, NULL);

			IDATA xxIdx = (xxSetPrefetchNoneIndex > xxSetPrefetchOSDfltIndex) ? xxSetPrefetchNoneIndex : xxSetPrefetchOSDfltIndex;
			UDATA prefetchValue = (xxIdx == xxSetPrefetchOSDfltIndex) ? XXSETHWPREFETCH_OS_DEFAULT_VALUE : XXSETHWPREFETCH_NONE_VALUE;

			if (xxIdx > xxSetPrefetchIndex) {
				setHWPrefetch(prefetchValue);
			} else {
				char *optname = VMOPT_XXSETHWPREFETCH_EQUALS;
				if (OPTION_OK == GET_INTEGER_VALUE(xxSetPrefetchIndex, optname, prefetchValue)) {
					setHWPrefetch(prefetchValue);
				} else {
					parseError = OPTION_MALFORMED;
					parseErrorOption = VMOPT_XXSETHWPREFETCH_EQUALS;
					goto _memParseError;
				}
			}
		}
#endif /*if defined(J9OS_I5)*/
#endif /*if defined(AIXPPC)*/

			break;

		case AGENTS_STARTED:
			/* Do the -Xrun options - these have to happen before ABOUT_TO_BOOTSTRAP hook below */
			if (JNI_OK != initializeXruns(vm)) {
				return J9VMDLLMAIN_SILENT_EXIT_VM;			/* We have already complained about the Xrun failure, so no need to say "VMInitStages failed" */
			}
			break;

		case ABOUT_TO_BOOTSTRAP :
#if defined(J9VM_INTERP_ATOMIC_FREE_JNI_USES_FLUSH)
			if (0 != initializeExclusiveAccess(vm)) {
				goto _error;
			}
#endif /* defined(J9VM_INTERP_ATOMIC_FREE_JNI_USES_FLUSH) */
#if defined(J9VM_OPT_CRIU_SUPPORT)
			if (isDebugOnRestoreEnabled(vm)) {
				Trc_VM_VMInitStages_isDebugOnRestoreEnabled();
				/* enable jvmtiCapabilities.can_get_source_debug_extension */
				vm->requiredDebugAttributes |= J9VM_DEBUG_ATTRIBUTE_SOURCE_DEBUG_EXTENSION;
				/* enable jvmtiCapabilities.can_access_local_variables */
				vm->requiredDebugAttributes |= J9VM_DEBUG_ATTRIBUTE_CAN_ACCESS_LOCALS;
				/* enable jvmtiCapabilities.can_maintain_original_method_order */
				vm->requiredDebugAttributes |= J9VM_DEBUG_ATTRIBUTE_MAINTAIN_ORIGINAL_METHOD_ORDER;
			}
#endif /* defined(J9VM_OPT_CRIU_SUPPORT) */
			TRIGGER_J9HOOK_VM_ABOUT_TO_BOOTSTRAP(vm->hookInterface, vm->mainThread);
			/* At this point, the decision about which interpreter to use has been made */

			if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_DEBUG_MODE)) {
				if (J9JAVAVM_COMPRESS_OBJECT_REFERENCES(vm)) {
#if defined(OMR_GC_COMPRESSED_POINTERS)
					vm->bytecodeLoop = debugBytecodeLoopCompressed;
#endif /* defined(OMR_GC_COMPRESSED_POINTERS) */
				} else {
#if defined(OMR_GC_FULL_POINTERS)
					vm->bytecodeLoop = debugBytecodeLoopFull;
#endif /* defined(OMR_GC_FULL_POINTERS) */
				}
			} else {
				if (J9JAVAVM_COMPRESS_OBJECT_REFERENCES(vm)) {
#if defined(OMR_GC_COMPRESSED_POINTERS)
					vm->bytecodeLoop = bytecodeLoopCompressed;
#endif /* defined(OMR_GC_COMPRESSED_POINTERS) */
				} else {
#if defined(OMR_GC_FULL_POINTERS)
					vm->bytecodeLoop = bytecodeLoopFull;
#endif /* defined(OMR_GC_FULL_POINTERS) */
				}
			}
			break;

		case JCL_INITIALIZED :
#ifdef OMR_THR_ADAPTIVE_SPIN
			jlm_adaptive_spin_init();
#endif
			break;

		case VM_INITIALIZATION_COMPLETE:
			if ((NULL != vm->jitConfig)
				&& (NULL != vm->jitConfig->samplerThread)
				&& (0 != vm->vmRuntimeStateListener.minIdleWaitTime)
			) {
				startVMRuntimeStateListener(vm);
			}
			break;

		case INTERPRETER_SHUTDOWN:
			vm->threadDllHandle = 0;
			break;
	}
	return returnVal;
	_memParseError :
		loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_VM_INIT );
		generateMemoryOptionParseError(vm, loadInfo, parseError, parseErrorOption);
	_error :
		return J9VMDLLMAIN_FAILED;
}


/* Run after all command-line args should have been consumed. Returns TRUE or FALSE. */

UDATA
checkArgsConsumed(J9JavaVM * vm, J9PortLibrary* portLibrary, J9VMInitArgs* j9vm_args)
{
	UDATA i = 0;
	PORT_ACCESS_FROM_PORT(portLibrary);
	jboolean ignoreUnrecognized = j9vm_args->actualVMArgs->ignoreUnrecognized;
	jboolean ignoreUnrecongizedTopLevelOption = JNI_FALSE;
	jboolean ignoreUnrecongizedXXColonOptions = JNI_TRUE;
	IDATA xxIgnoreUnrecognizedVMOptionsEnableIndex = -1;
	IDATA xxIgnoreUnrecognizedVMOptionsDisableIndex = -1;
	IDATA xxIgnoreUnrecognizedXXColonOptionsEnableIndex = -1;
	IDATA xxIgnoreUnrecognizedXXColonOptionsDisableIndex = -1;

	if (J9_ARE_ANY_BITS_SET(vm->compatibilityFlags, J9COMPATIBILITY_ELASTICSEARCH)
		|| (findArgInVMArgs(PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXVM_IGNOREUNRECOGNIZED, NULL, TRUE) >= 0)
	) {
		ignoreUnrecognized = JNI_TRUE;
	}

	xxIgnoreUnrecognizedVMOptionsEnableIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXIGNOREUNRECOGNIZEDVMOPTIONSENABLE, NULL, TRUE);
	if (xxIgnoreUnrecognizedVMOptionsEnableIndex >= 0) {
		xxIgnoreUnrecognizedVMOptionsDisableIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXIGNOREUNRECOGNIZEDVMOPTIONSDISABLE, NULL, TRUE);
		if (xxIgnoreUnrecognizedVMOptionsEnableIndex > xxIgnoreUnrecognizedVMOptionsDisableIndex) {
			ignoreUnrecongizedTopLevelOption = JNI_TRUE;
		}
	}

	xxIgnoreUnrecognizedXXColonOptionsDisableIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXIGNOREUNRECOGNIZEDXXCOLONOPTIONSDISABLE, NULL, TRUE);
	if (xxIgnoreUnrecognizedXXColonOptionsDisableIndex >= 0) {
		xxIgnoreUnrecognizedXXColonOptionsEnableIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXIGNOREUNRECOGNIZEDXXCOLONOPTIONSENABLE, NULL, TRUE);
		if (xxIgnoreUnrecognizedXXColonOptionsDisableIndex > xxIgnoreUnrecognizedXXColonOptionsEnableIndex) {
			ignoreUnrecongizedXXColonOptions = JNI_FALSE;
		}
	}

	/* Consuming the shared class options if it is used without -Xshareclasses */
	if (!ignoreUnrecongizedXXColonOptions && !vm->sharedCacheAPI->xShareClassesPresent) {
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXSHARECLASSESENABLEBCI, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXSHARECLASSESDISABLEBCI, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXENABLESHAREANONYMOUSCLASSES, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXDISABLESHAREANONYMOUSCLASSES, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXENABLESHAREUNSAFECLASSES, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXDISABLESHAREUNSAFECLASSES, NULL, TRUE);
	}

	for (i=0; i<j9vm_args->nOptions; i++) {
		if (IS_CONSUMABLE( j9vm_args, i ) && !IS_CONSUMED( j9vm_args, i )) {
			char* optString = j9vm_args->actualVMArgs->options[i].optionString;
			char* envVar = j9vm_args->j9Options[i].fromEnvVar;

			/* If ignoreUnrecognized is set to JNI_TRUE, we should not reject any options that are:
				empty or contain only whitespace, or unrecognized options beginning with -X or _ */
			if (ignoreUnrecognized && (NULL != optString) && (isEmpty(optString) || !strncmp(optString, "-X", 2) || *optString=='_')) {
				continue;
			}
			if (REQUIRES_LIBRARY( j9vm_args, i )) {
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_CANNOT_LOAD_LIBRARY, optString);
			} else if (envVar) {
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_INVALID_ENV_VAR, envVar);
			} else if (HAS_MAPPING( j9vm_args, i) && (MAPPING_FLAGS( j9vm_args, i ) & INVALID_OPTION)) {
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_INVALID_CMD_LINE_OPT, optString);
			} else {
				/* If ignoreUnrecognizedXXOptions is set to JNI_TRUE, we should ignore any options that start with -XX: */
				if (ignoreUnrecongizedXXColonOptions && (0 == strncmp(optString, VMOPT_XX, (sizeof(VMOPT_XX) - 1)))) {
					continue;
				}
				/* If ignoreUnrecongizedTopLevelOption is set to JNI_TRUE, we should ignore any unrecognized top-level option */
				if (ignoreUnrecongizedTopLevelOption) {
					continue;
				}
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, optString);
			}
			return FALSE;
		}
	}

	return TRUE;
}

/* Returns TRUE if a string is empty or if it contains only whitespace characters. */
static BOOLEAN
isEmpty(const char * str)
{
	BOOLEAN isEmpty = TRUE;
	while('\0' != *str) {
		if (0 == isspace((unsigned char) *str)) {
			isEmpty = FALSE;
			break;
		}
		str++;
	}
	return isEmpty;
}

/* Run using a pool_do after each initialization stage. If any errors were reported by libraries,
	a flag is set to FALSE and the error is printed. See checkPostStage. */

static void
checkDllInfo(void* dllLoadInfo, void* userDataTemp)
{
	J9VMDllLoadInfo* entry = (J9VMDllLoadInfo*)dllLoadInfo;
	CheckPostStageData* userData = (CheckPostStageData*)userDataTemp;
	int isInitialization = userData->stage < INTERPRETER_SHUTDOWN;

	PORT_ACCESS_FROM_JAVAVM(userData->vm);

	if (entry->fatalErrorStr!=NULL && strlen(entry->fatalErrorStr)>0) {
		if (strcmp(entry->fatalErrorStr, SILENT_EXIT_STRING)==0) {
			userData->success = RC_SILENT_EXIT;
			exit(1);
		}
		userData->success = RC_FAILED;
		if ((entry->loadFlags & FAILED_TO_LOAD)) {
			j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_UNABLE_TO_LOAD_DLL, entry->dllName, entry->fatalErrorStr);
		} else if ((entry->loadFlags & FAILED_TO_UNLOAD)) {
			j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_UNABLE_TO_UNLOAD_DLL, entry->dllName, entry->fatalErrorStr);
		} else if ((entry->loadFlags & (NOT_A_LIBRARY | BUNDLED_COMP))) {
			if (isInitialization) {
				j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_INITIALIZATION_ERROR_IN_FUNCTION, entry->dllName, userData->stage, entry->fatalErrorStr);
			} else {
				j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_SHUTDOWN_ERROR_IN_FUNCTION, entry->dllName, userData->stage, entry->fatalErrorStr);
			}
		} else {
			if (isInitialization) {
				j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_INITIALIZATION_ERROR_FOR_LIBRARY, entry->dllName, userData->stage, entry->fatalErrorStr);
			} else {
				j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_SHUTDOWN_ERROR_FOR_LIBRARY, entry->dllName, userData->stage, entry->fatalErrorStr);
			}
		}

		/* If library failed to unload, or library failed to load and this is not fatal, do not exit */
		if ( (entry->loadFlags & FAILED_TO_UNLOAD) || ((entry->loadFlags & FAILED_TO_LOAD) && !(entry->loadFlags & FATAL_NO_DLL)) ) {
			userData->success = JNI_OK;
		}
		setErrorJ9dll(PORTLIB, entry, NULL, FALSE); /* ensure that error is not reported more than once */
	}
}


/* This method consumes args which cannot be consumed by any libraries.
	Eg. -Xint - the JIT library is not loaded and therefore cannot consume the option.
	Do not consume any arguments here unless absolutely necessary. */

static void
consumeVMArgs(J9JavaVM* vm, J9VMInitArgs* j9vm_args)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	BOOLEAN assertOptionFound = FALSE;

	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XINT, NULL, TRUE);
	/* If -Xverify:none, other -Xverify args previous to that should be ignored. As of Java 13 -Xverify:none and -noverify are deprecated. */
	if (findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XVERIFY_COLON, OPT_NONE, TRUE) >= 0) {
		if (JAVA_SPEC_VERSION >= 13) {
			j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_XVERIFYNONE_DEPRECATED);
		}
		findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XVERIFY, NULL, TRUE);
	}
#if defined(J9VM_INTERP_VERBOSE)
	/* If -verbose:none, other -verbose args previous to that should be ignored */
	if (findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, OPT_VERBOSE_COLON, OPT_NONE, TRUE) >= 0) {
		findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, OPT_VERBOSE, NULL, TRUE);
	}
#endif
	/* Consume remaining dump options in case library is missing... */
	findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XDUMP, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XNOAOT, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XNOJIT, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XRUN, NULL, TRUE);

	if (J2SE_VERSION(vm) < J2SE_V11) {
		findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XBOOTCLASSPATH_COLON, NULL, TRUE);
		findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XBOOTCLASSPATH_P_COLON, NULL, TRUE);
	}

	findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XBOOTCLASSPATH_A_COLON, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XNOLINENUMBERS, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XLINENUMBERS, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XSERVICE_EQUALS, NULL, TRUE);

	/*
	 * '-Xlp:codecache:' and '-XtlhPrefetch' options are consumed by JIT.
	 * However, if -Xint is set, JIT won't startup, leaving any '-Xlp:codecache:' and '-XtlhPrefetch'
	 * option as un-consumed and resulting in JVM to report them as unrecognized options.
	 * To prevent this, consume any '-Xlp:codecache:' and '-XtlhPrefetch' option here.
	 */
	findArgInVMArgs( PORTLIB, j9vm_args, STARTSWITH_MATCH, VMOPT_XLP_CODECACHE, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XTLHPREFETCH, NULL, TRUE);

	/* consume options for capturing command line in environment variable, handled earlier in initialArgumentScan */
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXOPENJ9COMMANDLINEENV, NULL, TRUE);
	findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXNOOPENJ9COMMANDLINEENV, NULL, TRUE);

	/* Consume these without asking questions for now. Ultimately, if/when we use these, we will need logic
		so that the VM knows that -ea = -enableassertions. */
	assertOptionFound = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_EA, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_ENABLE_ASSERTIONS, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_DA, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_DISABLE_ASSERTIONS, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_ESA, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_ENABLE_SYSTEM_ASSERTIONS, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_DSA, NULL, TRUE) >= 0;
	assertOptionFound |= findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_DISABLE_SYSTEM_ASSERTIONS, NULL, TRUE) >= 0;

	if (assertOptionFound) {
		vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_FOUND_JAVA_ASSERT_OPTION;
	}
}




/*
 * Use this method to stop or force certain libraries to load by default.
 * This is run after the DLL table is created and gives an opportunity for the VM to "whack" the table,
 * based on certain variables.
 * Should contain minimum parsing necessary to determine load/not load
 */
static jint
modifyDllLoadTable(J9JavaVM * vm, J9Pool* loadTable, J9VMInitArgs* j9vm_args)
{
	jint rc = 0;
	J9VMDllLoadInfo* entry = NULL;
	JavaVMInitArgs* vm_args = j9vm_args->actualVMArgs;
	BOOLEAN xsnw = FALSE;
	BOOLEAN xint = FALSE;
	BOOLEAN xjit = FALSE;
	BOOLEAN xnojit = FALSE;
	BOOLEAN xverify = FALSE;
	BOOLEAN xxverboseverification = FALSE;
	BOOLEAN verbose = FALSE;
	BOOLEAN xnolinenumbers = FALSE;
	BOOLEAN xnoaot = FALSE;
	BOOLEAN xshareclasses = FALSE;
	BOOLEAN xdump = FALSE;
	BOOLEAN xverbosegclog = FALSE;
	BOOLEAN xaot = FALSE;
	BOOLEAN xxverifyerrordetails = TRUE;

	char* optionValue, *testString;
	IDATA xsnwIndex, xverifyIndex, verboseIndex, xshareclassesIndex, xdumpnoneIndex, xdumpIndex, xverbosegclogIndex, i;
	IDATA xxverboseverificationIndex = -1;
	IDATA xxnoverboseverificationIndex = -1;
	IDATA xxverifyerrordetailsIndex = -1;
	IDATA xxnoverifyerrordetailsIndex = -1;

	IDATA xxjitdirectoryIndex = 0;
	PORT_ACCESS_FROM_JAVAVM(vm);

#define ADD_FLAG_AT_INDEX(flag, index) j9vm_args->j9Options[index].flags |= flag
#define SET_FLAG_AT_INDEX(flag, index) j9vm_args->j9Options[index].flags = flag | (j9vm_args->j9Options[index].flags & ARG_MEMORY_ALLOCATION)

	xverify = ((xverifyIndex = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XVERIFY, NULL, FALSE))>=0);

	xxverboseverificationIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXVERBOSEVERIFICATION, NULL, FALSE);
	xxnoverboseverificationIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXNOVERBOSEVERIFICATION, NULL, FALSE);
	xxverboseverification = (xxverboseverificationIndex > xxnoverboseverificationIndex);

	xxverifyerrordetailsIndex = findArgInVMArgs(PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXVERIFYERRORDETAILS, NULL, FALSE);
	xxnoverifyerrordetailsIndex = findArgInVMArgs(PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XXNOVERIFYERRORDETAILS, NULL, FALSE);
	xxverifyerrordetails = (xxverifyerrordetailsIndex >= xxnoverifyerrordetailsIndex);

	xsnw = ((xsnwIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XSNW, NULL, FALSE))>=0);
	verbose = ((verboseIndex = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, OPT_VERBOSE, NULL, FALSE))>=0);
	xverbosegclog = ((xverbosegclogIndex = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, OPT_XVERBOSEGCLOG, NULL, FALSE))>=0);
	xnolinenumbers = (findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XNOLINENUMBERS, NULL, FALSE) >= 0);
	xshareclasses = ((xshareclassesIndex = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XSHARECLASSES, NULL, FALSE))>=0);


	xdumpIndex = findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XDUMP, NULL, FALSE);
	xdumpnoneIndex = findArgInVMArgs( PORTLIB, j9vm_args, EXACT_MATCH, VMOPT_XDUMP_NONE, NULL, FALSE);

	/* Dump support is on by default, disabled by -Xdump:none as last (rightmost) -Xdump parameter */
	if (xdumpnoneIndex == -1 || (xdumpIndex > xdumpnoneIndex)) {
		xdump = TRUE;
	} else {
		xdump = FALSE;
	}

	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\nCustomizing DLL load table:\n");

	/*
	 * Work out the logic of which JIT and AOT cmd line args should be recognised and which should be ignored.
	 *
	 * Assume that right-most argument wins, with the following rules:
	 *
	 * As the right-most argument...
	 * -Xnojit can co-exist with -Xnoaot and -Xaot, but overrides the others.
	 * -Xnoaot can co-exist with -Xnojit and -Xjit.
	 * -Xaot can co-exist with -Xnojit and -Xjit.
	 * -Xjit can co-exist with -Xnoaot and -Xaot, but overrides the others.
	 * -Xint overrides everything.
	 */
	xint = xjit = xnojit = xnoaot = xaot = FALSE;
	for (i=(vm_args->nOptions - 1); i>=0 ; i--) {
		testString = getOptionString(j9vm_args, i);				/* may return mapped value */

#if defined(J9VM_OPT_JVMTI)
		if ((strstr(testString, VMOPT_AGENTLIB_COLON)==testString) || (strstr(testString, VMOPT_AGENTPATH_COLON)==testString)) {
			ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, i );
		}
#endif

		if (strcmp(testString, VMOPT_XINT)==0) {
			if (xjit || xnojit || xnoaot || xaot) {
				SET_FLAG_AT_INDEX( NOT_CONSUMABLE_ARG, i );
			} else {
				xint = TRUE;
			}
		} else if ((strcmp(testString, VMOPT_XJIT)==0) ||
					(strstr(testString, VMOPT_XJIT_COLON)==testString)) {
			if (xint || xnojit) {
				SET_FLAG_AT_INDEX( NOT_CONSUMABLE_ARG, i );
			} else {
				ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, i );
				xjit = TRUE;
			}
		} else if (strcmp(testString, VMOPT_XNOJIT)==0) {
			if (xint || xjit) {
				SET_FLAG_AT_INDEX( NOT_CONSUMABLE_ARG, i );
			} else {
				xnojit = TRUE;
			}
		} else if (strcmp(testString, VMOPT_XNOAOT)==0) {
			if (xint || xaot) {
				SET_FLAG_AT_INDEX( NOT_CONSUMABLE_ARG, i );
			} else {
				xnoaot = TRUE;
			}
		} else if ((strcmp(testString, VMOPT_XAOT)==0) ||
					(strstr(testString, VMOPT_XAOT_COLON)==testString)) {
			if (xint || xnoaot) {
				SET_FLAG_AT_INDEX( NOT_CONSUMABLE_ARG, i );
			} else {
				xaot = TRUE;
			}
		}
	}

	/* Temporarily disable JIT/AOT until it is fully implemented */
#if defined(RISCV64)
	xint = TRUE;
	xjit = FALSE;
	xaot = FALSE;
	xnoaot = FALSE;
#endif

	if (xint) {
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "-Xint set\n");
	}
	if (xnoaot) {
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "-Xnoaot set\n");
	}
	if (xaot) {
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "-Xaot set\n");
	}
	if (xnojit) {
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "-Xnojit set\n");
	}
	if (xjit) {
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "-Xjit set\n");
	}

	if (xint == FALSE && xnoaot == FALSE) {
		/* Enable AOT if neither -Xint nor -Xnoaot is set as that is the default behavior */
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_AOT;
	}

#if defined(J9VM_OPT_JVMTI)
	entry = findDllLoadInfo(loadTable, J9_JVMTI_DLL_NAME);
	entry->loadFlags |= LOAD_BY_DEFAULT;
	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "JVMTI required... whacking table\n");
#endif

/**
 * JIT DLL should always be loaded unless -Xint. Should load even with -Xnojit
 * The following code has become a mess. Hence use of comments to attempt to clarify.
 */
#if defined(J9VM_INTERP_NATIVE_SUPPORT)

	/* On platforms where JIT is the default, load unless Xint. Otherwise, only load if Xjit explicitly set */
	if (!((xnojit && xnoaot) || xint) || xjit) {
		entry = findDllLoadInfo(loadTable, J9_JIT_DLL_NAME);
#if defined(J9VM_INTERP_JIT_ON_BY_DEFAULT)

		/* On default JIT platforms, set load by default flag */
		entry->loadFlags |= LOAD_BY_DEFAULT;
#else
		/* On other platforms, only set if explicit */
		if (xjit) {
			entry->loadFlags |= LOAD_BY_DEFAULT;
		}
#endif
		if (entry->loadFlags & LOAD_BY_DEFAULT) {
			JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "JIT required... whacking table\n");
		}

		/* Check for an alternate JIT Directory */
		xxjitdirectoryIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXJITDIRECTORY_EQUALS, NULL);
		if (xxjitdirectoryIndex >= 0) {
			char dllCheckPath[EsMaxPath];
			char *dllCheckPathPtr = dllCheckPath;
			char *jitdirectoryValue = NULL;
			UDATA expectedPathLength = 0;
			UDATA jitDirectoryLength = 0;
			UDATA openFlags = (entry->loadFlags & XRUN_LIBRARY) ? J9PORT_SLOPEN_DECORATE | J9PORT_SLOPEN_LAZY : J9PORT_SLOPEN_DECORATE;
			UDATA jitFileHandle = 0;
			UDATA rc = 0;

			/*
			* On Linux on Z libj9jit dynamically loads libj9zlib as it is used for AOT method data compression
			* which is currently only enabled on Z platform. We want to ensure that when the JVM loads libj9jit,
			* libj9zlib is already loaded. See eclipse-openj9/openj9#8561 for more details.
			*/
#if (defined(S390) && defined(LINUX))
			{
			char zlibDll[EsMaxPath];
			char *zlibDllDir = zlibDll;
			UDATA expectedZlibPathLength = 0;
			UDATA zlibDllLength = 0;
			UDATA zlibFileHandle = 0;
			UDATA zlibRC = 0;

			zlibDllLength = strlen(vm->j9libvmDirectory);
			expectedZlibPathLength = zlibDllLength + (sizeof(DIR_SEPARATOR_STR) - 1) + strlen(J9_ZIP_DLL_NAME) + 1;
			if (expectedZlibPathLength > EsMaxPath) {
				zlibDllDir = j9mem_allocate_memory(expectedZlibPathLength, OMRMEM_CATEGORY_VM);
				if (NULL == zlibDllDir) {
					return JNI_ERR;
				}
			}
			j9str_printf(zlibDllDir, expectedZlibPathLength, "%s%s%s",
					vm->j9libvmDirectory, DIR_SEPARATOR_STR, J9_ZIP_DLL_NAME);
			zlibFileHandle = j9sl_open_shared_library(zlibDllDir, &(entry->descriptor), openFlags);
			if (0 != zlibFileHandle) {
				j9tty_printf(PORTLIB, "Error: Failed to open zlib DLL %s (%s)\n", zlibDllDir, j9error_last_error_message());
				zlibRC = JNI_ERR;
			}
			if (zlibDll != zlibDllDir) {
				j9mem_free_memory(zlibDllDir);
				zlibDllDir = NULL;
			}
			if (zlibRC != 0) {
				return JNI_ERR;
			}
			}
#endif /* defined(S390) && defined(LINUX) */

			optionValueOperations(PORTLIB, j9vm_args, xxjitdirectoryIndex, GET_OPTION, &jitdirectoryValue, 0, '=', 0, NULL); /* get option value for xxjitdirectory= */
			jitDirectoryLength = strlen(jitdirectoryValue);
			/* Test that the alternate JIT Directory contains a valid JIT library for this release */

#if defined(J9ZTPF)
			openFlags = 0; /* for z/TPF don't decorate nor open */
#endif /* defined(J9ZTPF) */
			/* expectedPathLength - %s%s%s - +1 includes NUL terminator */
			expectedPathLength = jitDirectoryLength + (sizeof(DIR_SEPARATOR_STR) - 1) + strlen(entry->dllName) + 1;
			if (expectedPathLength > EsMaxPath) {
				dllCheckPathPtr = j9mem_allocate_memory(expectedPathLength, OMRMEM_CATEGORY_VM);
				if (NULL == dllCheckPathPtr) {
					return JNI_ERR;
				}
			}
			j9str_printf(dllCheckPathPtr, expectedPathLength, "%s%s%s",
					jitdirectoryValue, DIR_SEPARATOR_STR, entry->dllName);

			jitFileHandle = j9sl_open_shared_library(dllCheckPathPtr, &(entry->descriptor), openFlags);
			/* Confirm that we have a valid path being set */
			if (0 == jitFileHandle) {
				vm->alternateJitDir = j9mem_allocate_memory(jitDirectoryLength + 1, OMRMEM_CATEGORY_VM);
				if (NULL == vm->alternateJitDir) {
					rc = JNI_ERR;
				} else {
					memcpy(vm->alternateJitDir, jitdirectoryValue, jitDirectoryLength + 1);
					JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "Custom JIT Directory found %s\n", vm->alternateJitDir);
				}
			} else {
				/* If we cannot find the library, exit */
				j9tty_printf(PORTLIB, "Error: Failed to open JIT DLL %s (%s)\n", dllCheckPathPtr, j9error_last_error_message());
				rc = JNI_ERR;
			}
			if (dllCheckPath != dllCheckPathPtr) {
				j9mem_free_memory(dllCheckPathPtr);
			}
			if (rc != 0) {
				return JNI_ERR;
			}
		}
	}
#endif /* J9VM_INTERP_NATIVE_SUPPORT */

	/* Consume xxjitdirectory but do nothing with it if xint or xnojit and xnoaot is specified */
	if ((xnojit && xnoaot) || xint) {
		FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXJITDIRECTORY_EQUALS, NULL);
	}

	if (xverify) {
		ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, xverifyIndex );
		optionValueOperations(PORTLIB, j9vm_args, xverifyIndex, GET_OPTION, &optionValue, 0, ':', 0, NULL);			/* get option value for xverify: */
	} else {
		optionValue = NULL;
	}

	if ((optionValue != NULL) && (strcmp(optionValue, OPT_NONE) == 0)) {
		entry = findDllLoadInfo(loadTable, J9_VERIFY_DLL_NAME);
		/* Unsetting the DllMain function pointer so library is never called */
		entry->j9vmdllmain = 0;
	} else if (xxverboseverification || xxverifyerrordetails) {
		entry = getVerboseDllLoadInfo(vm);
		entry->loadFlags |= LOAD_BY_DEFAULT;
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "verbose support required... whacking table\n");
	}

#if defined( J9VM_OPT_SIDECAR )
	entry = getVerboseDllLoadInfo(vm);
	entry->loadFlags |= LOAD_BY_DEFAULT;
	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "verbose support required in j2se... whacking table\n");
#endif

	if ( xverbosegclog ) {
		ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, xverbosegclogIndex );
		entry = getVerboseDllLoadInfo(vm);
		entry->loadFlags |= LOAD_BY_DEFAULT;
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "verbosegclog support required... whacking table\n");
	}

#if defined(J9VM_INTERP_VERBOSE)
	if ( verbose ) {
		optionValueOperations(PORTLIB, j9vm_args, verboseIndex, GET_OPTION, &optionValue, 0, ':', 0, NULL);			/* get option value for verbose: */
		if ( !(optionValue!=NULL && strcmp(optionValue, OPT_NONE)==0) ) {
			ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, verboseIndex );
			entry = getVerboseDllLoadInfo(vm);
			entry->loadFlags |= LOAD_BY_DEFAULT;
			JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "verbose support required... whacking table\n");
		}
	}

	if ( xsnw ) {
		ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, xsnwIndex );
		entry = getVerboseDllLoadInfo(vm);
		entry->loadFlags |= LOAD_BY_DEFAULT;
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "verbose support required... whacking table\n");
	}
#endif

#ifndef J9VM_OPT_DYNAMIC_LOAD_SUPPORT
		entry = findDllLoadInfo(loadTable, J9_DYNLOAD_DLL_NAME);
		/* Unsetting the DllMain function pointer so library is never called */
		entry->j9vmdllmain = 0;
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "No dynamic load support... whacking table\n");
#endif

#if defined(J9VM_OPT_DEBUG_INFO_SERVER)
	if (!xnolinenumbers) {
		vm->requiredDebugAttributes |= (J9VM_DEBUG_ATTRIBUTE_LINE_NUMBER_TABLE | J9VM_DEBUG_ATTRIBUTE_SOURCE_FILE);
	}
#endif

#if defined(J9VM_OPT_SHARED_CLASSES)
	if (xshareclasses) {
		char optionsBuffer[256];
		char* optionsBufferPtr = (char*)optionsBuffer;
		char* walkPtr;
		UDATA noneFound = 0;
		UDATA buflen = 256;
		IDATA option_rc;

		do {
			option_rc = optionValueOperations(PORTLIB, j9vm_args, xshareclassesIndex, GET_OPTIONS, &optionsBufferPtr, buflen, ':', ',', NULL);
			if (option_rc == OPTION_BUFFER_OVERFLOW) {
				if (optionsBufferPtr != (char*)optionsBuffer) {
					j9mem_free_memory(optionsBufferPtr);
					optionsBufferPtr = NULL;
				}
				buflen *= 2;
				optionsBufferPtr = (char*)j9mem_allocate_memory(buflen, OMRMEM_CATEGORY_VM);
			}
		} while ((option_rc == OPTION_BUFFER_OVERFLOW) && (optionsBufferPtr != NULL));

		if (optionsBufferPtr == NULL) {
			return JNI_ERR;
		}

		walkPtr = optionsBufferPtr;
		while (*walkPtr) {
			if (try_scan(&walkPtr, OPT_NONE)) {
				/* If none is found, consume all Xshareclasses options */
				findArgInVMArgs( PORTLIB, j9vm_args, OPTIONAL_LIST_MATCH, VMOPT_XSHARECLASSES, NULL, TRUE);
				noneFound = 1;
				break;
			}
			walkPtr += strlen(walkPtr)+1;
		}

		if (optionsBufferPtr != (char*)optionsBuffer) {
			j9mem_free_memory(optionsBufferPtr);
		}

		if (!noneFound && (entry = findDllLoadInfo(loadTable, J9_SHARED_DLL_NAME))) {
			entry->loadFlags |= LOAD_BY_DEFAULT;
			JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "shared classes support required... whacking table\n");
			ADD_FLAG_AT_INDEX( ARG_REQUIRES_LIBRARY, xshareclassesIndex );
		}
	} else {
		if ((entry = findDllLoadInfo(loadTable, J9_SHARED_DLL_NAME))) {
			entry->loadFlags |= (LOAD_BY_DEFAULT | SILENT_NO_DLL);
			JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "shared classes support required... whacking table\n");
		}
	}
#endif

	if ( xdump ) {
		entry = findDllLoadInfo(loadTable, J9_RAS_DUMP_DLL_NAME);
		entry->loadFlags |= EARLY_LOAD;
		JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "Dump support required... whacking table\n");
	}

#if defined(J9VM_OPT_JAVA_OFFLOAD_SUPPORT)
	entry = findDllLoadInfo(loadTable, J9_IFA_DLL_NAME);
	entry->loadFlags |= LOAD_BY_DEFAULT;
	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "IFA support required... whacking table\n");
#endif /* defined(J9VM_OPT_JAVA_OFFLOAD_SUPPORT) */

	rc = processXCheckOptions(vm, loadTable, j9vm_args);
	JVMINIT_VERBOSE_INIT_TRACE_WORKING_SET(vm);
	return rc;
}


/* Process VM args that are order-dependent */
static jint
processVMArgsFromFirstToLast(J9JavaVM * vm)
{
	vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_CLASSLOADER_LOCKING_ENABLED | J9_EXTENDED_RUNTIME_REDUCE_CPU_MONITOR_OVERHEAD; /* enabled by default */
	vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ENABLE_CPU_MONITOR; /* Cpu monitoring is enabled by default */
	vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ALLOW_CONTENDED_FIELDS; /* Allow contended fields on bootstrap classes */
	vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_DISABLE_EXTENDED_HCR; /* disable extended HCR by default */
	if (J2SE_VERSION(vm) >= J2SE_V11) {
		vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_RESTRICT_IFA; /* Enable zAAP switching for Registered Natives and JVMTI callbacks by default in Java 9 and later. */
	}
	vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_UTF_CACHE; /* Enable UTF cache by default */

	{
		IDATA noDynamicAgentLoading = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOENABLEDYNAMICAGENTLOADING, NULL);
		IDATA dynamicAgentLoading = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEDYNAMICAGENTLOADING, NULL);
		if (noDynamicAgentLoading > dynamicAgentLoading) {
			vm->runtimeFlags &= ~J9_RUNTIME_ALLOW_DYNAMIC_AGENT;
			vm->extendedRuntimeFlags &= ~(J9_EXTENDED_RUNTIME_OSR_SAFE_POINT | J9_EXTENDED_RUNTIME_ENABLE_HCR);
		} else {
			vm->runtimeFlags |= J9_RUNTIME_ALLOW_DYNAMIC_AGENT;
			vm->extendedRuntimeFlags |= (J9_EXTENDED_RUNTIME_OSR_SAFE_POINT | J9_EXTENDED_RUNTIME_ENABLE_HCR);
		}
	}

#if defined(J9VM_ARCH_X86) || defined(J9VM_ARCH_POWER) || defined(J9VM_ARCH_S390)
	/* Enabled field watch by default on x86, Power, and S390 platforms */
	vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_JIT_INLINE_WATCHES;
#endif /* J9VM_ARCH_X86, J9VM_ARCH_POWER, J9VM_ARCH_S390 */
	{
		IDATA noStackTraceInThrowable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOSTACKTRACEINTHROWABLE, NULL);
		IDATA stackTraceInThrowable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSTACKTRACEINTHROWABLE, NULL);
		if (noStackTraceInThrowable > stackTraceInThrowable) {
			vm->runtimeFlags |= J9_RUNTIME_OMIT_STACK_TRACES;
		} else if (noStackTraceInThrowable < stackTraceInThrowable) {
			vm->runtimeFlags &= ~(UDATA)J9_RUNTIME_OMIT_STACK_TRACES;
		}
	}

	{
		IDATA alwaysCopyJNICritical = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXALWAYSCOPYJNICRITICAL, NULL);
		IDATA noAlwaysCopyJNICritical = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOALWAYSCOPYJNICRITICAL, NULL);
		if (alwaysCopyJNICritical > noAlwaysCopyJNICritical) {
			vm->runtimeFlags |= J9_RUNTIME_ALWAYS_COPY_JNI_CRITICAL;
		} else if (alwaysCopyJNICritical < noAlwaysCopyJNICritical) {
			vm->runtimeFlags &= ~(UDATA)J9_RUNTIME_ALWAYS_COPY_JNI_CRITICAL;
		}
	}

	{
		IDATA alwaysUseJNICritical = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXALWAYSUSEJNICRITICAL, NULL);
		IDATA noAlwaysUseJNICritical = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOALWAYSUSEJNICRITICAL, NULL);
		if (alwaysUseJNICritical > noAlwaysUseJNICritical) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_ALWAYS_USE_JNI_CRITICAL;
		} else if (alwaysUseJNICritical < noAlwaysUseJNICritical) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ALWAYS_USE_JNI_CRITICAL;
		}
	}

	{
		IDATA debugVmAccess = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDEBUGVMACCESS, NULL);
		IDATA noDebugVmAccess = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODEBUGVMACCESS, NULL);
		if (debugVmAccess > noDebugVmAccess) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_DEBUG_VM_ACCESS;
		} else if (debugVmAccess < noDebugVmAccess) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_DEBUG_VM_ACCESS;
		}
	}
#ifdef J9VM_OPT_METHOD_HANDLE
	{
		IDATA mhAllowI2J = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXMHALLOWI2J, NULL);
		IDATA nomhAllowI2J = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOMHALLOWI2J, NULL);
		if (mhAllowI2J > nomhAllowI2J) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_I2J_MH_TRANSITION_ENABLED;
		} else if (mhAllowI2J < nomhAllowI2J) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_I2J_MH_TRANSITION_ENABLED;
		}
	}
#endif
	{
		IDATA lazySymbolResolution = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXLAZYSYMBOLRESOLUTION, NULL);
		IDATA nolazySymbolResolution = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOLAZYSYMBOLRESOLUTION, NULL);
		if (lazySymbolResolution > nolazySymbolResolution) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_LAZY_SYMBOL_RESOLUTION;
		} else if (lazySymbolResolution < nolazySymbolResolution) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_LAZY_SYMBOL_RESOLUTION;
		}
	}

	{
		IDATA vmLockClassLoaderEnable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXVMLOCKCLASSLOADERENABLE, NULL);
		IDATA vmLockClassLoaderDisable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXVMLOCKCLASSLOADERDISABLE, NULL);
		if (vmLockClassLoaderEnable > vmLockClassLoaderDisable) {
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_CLASSLOADER_LOCKING_ENABLED;
		} else if (vmLockClassLoaderEnable < vmLockClassLoaderDisable) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_CLASSLOADER_LOCKING_ENABLED;
		}
	}

	{
		IDATA pageAlignDirectMemory = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXPAGEALIGNDIRECTMEMORY, NULL);
		IDATA noPageAlignDirectMemory = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOPAGEALIGNDIRECTMEMORY, NULL);
		if (pageAlignDirectMemory > noPageAlignDirectMemory) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_PAGE_ALIGN_DIRECT_MEMORY;
		} else if (pageAlignDirectMemory < noPageAlignDirectMemory) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_PAGE_ALIGN_DIRECT_MEMORY;
		}
	}

	{
		IDATA fastClassHashTable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXFASTCLASSHASHTABLE, NULL);
		IDATA noFastClassHashTable = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOFASTCLASSHASHTABLE, NULL);
		if (fastClassHashTable > noFastClassHashTable) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_DISABLE_FAST_CLASS_HASH_TABLE;
		} else if (fastClassHashTable < noFastClassHashTable) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_DISABLE_FAST_CLASS_HASH_TABLE;
		}
	}

#if (JAVA_SPEC_VERSION <= 19)
	{
		/**
		 * From the spec, the ACC_SUPER semantics became mandatory. The bit CFR_ACC_SUPER is reused as CFR_ACC_IDENTITY in Valhalla.
		 * This means -XX:+AllowNonVirtualCalls is not supported in Valhalla.
		 */
		IDATA allowNonVirtualCalls = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXALLOWNONVIRTUALCALLS, NULL);
		IDATA noAllowNonVirtualCalls = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDONTALLOWNONVIRTUALCALLS, NULL);
		if (allowNonVirtualCalls > noAllowNonVirtualCalls) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_ALLOW_NON_VIRTUAL_CALLS;
		} else if (allowNonVirtualCalls < noAllowNonVirtualCalls) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ALLOW_NON_VIRTUAL_CALLS;
		}
	}
#endif /* (JAVA_SPEC_VERSION <= 19) */

	{
		IDATA debugInterpreter = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDEBUGINTERPRETER, NULL);
		IDATA noDebugInterpreter = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODEBUGINTERPRETER, NULL);
		if (debugInterpreter > noDebugInterpreter) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_DEBUG_MODE;
		} else if (debugInterpreter < noDebugInterpreter) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_DEBUG_MODE;
		}
	}

	{
		IDATA enableHugePagesMmap = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_PORT_VMEM_HUGE_PAGES_MMAP_ENABLED, NULL);
		IDATA disableHugePagesMmap = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_PORT_VMEM_HUGE_PAGES_MMAP_DISABLED, NULL);
		PORT_ACCESS_FROM_JAVAVM(vm);
		if (enableHugePagesMmap > disableHugePagesMmap) {
			j9port_control(J9PORT_CTLDATA_VMEM_HUGE_PAGES_MMAP_ENABLED, 1);
		} else {
			j9port_control(J9PORT_CTLDATA_VMEM_HUGE_PAGES_MMAP_ENABLED, 0);
		}
	}

	if (FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, SYSPROP_COM_SUN_MANAGEMENT, NULL) != -1) {
		vm->jclFlags |= J9_JCL_FLAG_COM_SUN_MANAGEMENT_PROP;
	} else if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXFORCECLASSFILEASINTERMEDIATEDATA, NULL) != -1) {
		vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_FORCE_CLASSFILE_AS_INTERMEDIATE_DATA;
	} else if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXRECREATECLASSFILEONLOAD, NULL) != -1) {
		vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_RECREATE_CLASSFILE_ONLOAD;
	}

	{
		IDATA noReduceCPUMonitorOverhead = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOREDUCECPUMONITOROVERHEAD, NULL);
		IDATA reduceCPUMonitorOverhead = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXREDUCECPUMONITOROVERHEAD, NULL);
		if (noReduceCPUMonitorOverhead > reduceCPUMonitorOverhead) {
#if defined(J9ZOS390)
			PORT_ACCESS_FROM_JAVAVM(vm);
			/* Disabling this option on z/OS as this introduces a 50% startup regression and smaller throughput regresssions */
			j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_NO_REDUCE_CPU_OVERHEAD_UNSUPPORTED_ZOS, VMOPT_XXNOREDUCECPUMONITOROVERHEAD);
			return JNI_ERR;
#else
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_REDUCE_CPU_MONITOR_OVERHEAD;
#endif
		} else if (noReduceCPUMonitorOverhead < reduceCPUMonitorOverhead) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_REDUCE_CPU_MONITOR_OVERHEAD;
		}
	}

	{
		IDATA enableCPUMonitor = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLECPUMONITOR, NULL);
		IDATA disableCPUMonitor = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLECPUMONITOR, NULL);
		if (enableCPUMonitor > disableCPUMonitor) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_ENABLE_CPU_MONITOR;
		} else if (enableCPUMonitor < disableCPUMonitor) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ENABLE_CPU_MONITOR;
		}
	}

	{
		IDATA restrictContended = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXRESTRICTCONTENDED, NULL);
		IDATA noRestrictContended = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNORESTRICTCONTENDED, NULL);
		if (restrictContended > noRestrictContended) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ALLOW_APPLICATION_CONTENDED_FIELDS;
		} else if (restrictContended < noRestrictContended) { /* enabling application contended fields implicitly turns on bootstrap contended fields */
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ALLOW_APPLICATION_CONTENDED_FIELDS;
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ALLOW_CONTENDED_FIELDS;
		}
	}

	{
		IDATA noContendedFields = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOCONTENDEDFIELDS, NULL);
		IDATA contendedFields = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCONTENDEDFIELDS, NULL);
		if (noContendedFields > contendedFields) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ALLOW_CONTENDED_FIELDS;
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ALLOW_APPLICATION_CONTENDED_FIELDS;
		} else if (noContendedFields < contendedFields) {
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ALLOW_CONTENDED_FIELDS;
		}
	}

	{
		IDATA restrictIFA = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXRESTRICTIFA, NULL);
		IDATA noRestrictIFA = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNORESTRICTIFA, NULL);
		if (restrictIFA > noRestrictIFA) {
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_RESTRICT_IFA;
		} else if (restrictIFA < noRestrictIFA) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_RESTRICT_IFA;
		}
	}

	{
		IDATA enableHCR = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEHCR, NULL);
		IDATA noEnableHCR = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOENABLEHCR, NULL);
		if (enableHCR > noEnableHCR) {
			vm->extendedRuntimeFlags |= (UDATA)J9_EXTENDED_RUNTIME_ENABLE_HCR;
		} else if (enableHCR < noEnableHCR) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_ENABLE_HCR;
		}
	}

	{
		IDATA enableOSRSafePoint = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEOSRSAFEPOINT, NULL);
		IDATA disableOSRSafePoint = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEOSRSAFEPOINT, NULL);
		if (enableOSRSafePoint > disableOSRSafePoint) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_OSR_SAFE_POINT;
		} else if (enableOSRSafePoint < disableOSRSafePoint) {
			vm->extendedRuntimeFlags &= ~(UDATA)(J9_EXTENDED_RUNTIME_OSR_SAFE_POINT| J9_EXTENDED_RUNTIME_OSR_SAFE_POINT_FV);
		}
	}

	{
		IDATA enableOSRSafePointFV = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEOSRSAFEPOINTFV, NULL);
		IDATA disableOSRSafePointFV = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEOSRSAFEPOINTFV, NULL);
		if (enableOSRSafePointFV > disableOSRSafePointFV) {
			vm->extendedRuntimeFlags |= (J9_EXTENDED_RUNTIME_OSR_SAFE_POINT| J9_EXTENDED_RUNTIME_OSR_SAFE_POINT_FV);
		} else if (enableOSRSafePointFV < disableOSRSafePointFV) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_OSR_SAFE_POINT_FV;
		}
	}

	{
		IDATA enableJITWatch = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEJITWATCH, NULL);
		IDATA disableJITWatch = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEJITWATCH, NULL);
		if (enableJITWatch > disableJITWatch) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_JIT_INLINE_WATCHES;
		} else if (enableJITWatch < disableJITWatch) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_JIT_INLINE_WATCHES;
		}
	}

#if defined(J9VM_OPT_CRIU_SUPPORT)
	{
		IDATA enableCRIU = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLECRIU, NULL);
		IDATA disableCRIU = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLECRIU, NULL);
		if (enableCRIU > disableCRIU) {
			PORT_ACCESS_FROM_JAVAVM(vm);
			vm->checkpointState.flags |= J9VM_CRIU_IS_CHECKPOINT_ENABLED | J9VM_CRIU_IS_CHECKPOINT_ALLOWED;
			vm->portLibrary->isCheckPointAllowed = TRUE;
			j9port_control(J9PORT_CTLDATA_CRIU_SUPPORT_FLAGS, OMRPORT_CRIU_SUPPORT_ENABLED);
		}
	}

#if defined(J9VM_OPT_CRAC_SUPPORT)
	{
		IDATA xxCRaCCheckpointToIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XXCRACCHECKPOINTTO, NULL);
		if (xxCRaCCheckpointToIndex >= 0) {
			PORT_ACCESS_FROM_JAVAVM(vm);
			if (J9_ARE_ANY_BITS_SET(vm->checkpointState.flags, J9VM_CRIU_IS_CHECKPOINT_ENABLED)) {
				/* -XX:+EnableCRIUSupport was specified which is incompatible with CRaC */
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_CRIU_CRAC_INCOMPATIBLE_SETTING, NULL);
				return JNI_ERR;
			} else {
				char *cracCheckpointToValue = NULL;

				optionValueOperations(PORTLIB, vm->vmArgsArray, xxCRaCCheckpointToIndex, GET_OPTION, &cracCheckpointToValue, 0, '=', 0, NULL);
				Trc_VM_crac_checkpointTo(cracCheckpointToValue);
				vm->checkpointState.cracCheckpointToDir = cracCheckpointToValue;

				vm->checkpointState.flags |= J9VM_CRAC_IS_CHECKPOINT_ENABLED | J9VM_CRIU_IS_CHECKPOINT_ALLOWED;
				vm->portLibrary->isCheckPointAllowed = TRUE;
				j9port_control(J9PORT_CTLDATA_CRIU_SUPPORT_FLAGS, OMRPORT_CRIU_SUPPORT_ENABLED);
			}
		}
	}
#endif /* defined(J9VM_OPT_CRAC_SUPPORT) */

	{
		IDATA enableCRIUSecProvider = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLECRIUSECPROVIDER, NULL);
		IDATA disableCRIUSecProvider = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLECRIUSECPROVIDER, NULL);
		if (enableCRIUSecProvider >= disableCRIUSecProvider) {
			if (J9_IS_CRIU_OR_CRAC_CHECKPOINT_ENABLED(vm)) {
				vm->checkpointState.flags |= J9VM_CRIU_ENABLE_CRIU_SEC_PROVIDER;
			}
		}
	}

	{
		IDATA enableCRIUNonPortableMode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLECRIUNONPORTABLEMODE, NULL);
		IDATA disableCRIUNonPortableMode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLECRIUNONPORTABLEMODE, NULL);
		if (enableCRIUNonPortableMode >= disableCRIUNonPortableMode) {
			if (J9_IS_CRIU_OR_CRAC_CHECKPOINT_ENABLED(vm)) {
				vm->checkpointState.flags |= J9VM_CRIU_IS_NON_PORTABLE_RESTORE_MODE;
			}
		}
	}

	{
		IDATA enableJVMRestorePortableeMode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEJVMRESTOREPORTABLEMODE, NULL);
		IDATA disableJVMRestorePortableMode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEJVMRESTOREPORTABLEMODE, NULL);
		if (enableJVMRestorePortableeMode > disableJVMRestorePortableMode) {
			if (J9_IS_CRIU_OR_CRAC_CHECKPOINT_ENABLED(vm)) {
				vm->checkpointState.flags |= J9VM_CRIU_IS_PORTABLE_JVM_RESTORE_MODE;
			}
		}
	}

	{
		IDATA enableThrowOnDelayedCheckpointOperation = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLETHROWONDELAYECHECKPOINTOPERATION, NULL);
		IDATA disableThrowOnDelayedCheckpointOperation = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLETHROWONDELAYECHECKPOINTOPERATION, NULL);
		if (disableThrowOnDelayedCheckpointOperation >= enableThrowOnDelayedCheckpointOperation) {
			vm->checkpointState.flags |= J9VM_CRIU_IS_THROW_ON_DELAYED_CHECKPOINT_ENABLED;
		}
	}

	{
		IDATA enableSupportDebugOnRestore = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEDEBUGONRESTORE, NULL);
		IDATA disableSupportDebugOnRestore = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEDEBUGONRESTORE, NULL);
		if (enableSupportDebugOnRestore > disableSupportDebugOnRestore) {
			vm->checkpointState.flags |= J9VM_CRIU_SUPPORT_DEBUG_ON_RESTORE;
		}
	}

	{
		if ((FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, MAPOPT_AGENTLIB_JDWP_EQUALS, NULL) >= 0)
			|| (FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, MAPOPT_XRUNJDWP, NULL) >= 0)
		) {
			vm->checkpointState.flags |= J9VM_CRIU_IS_JDWP_ENABLED;
		}
		memset(vm->checkpointState.javaDebugThreads, 0, sizeof(vm->checkpointState.javaDebugThreads));
		vm->checkpointState.javaDebugThreadCount = 0;
	}

	{
		IDATA enableTimeCompensation = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLETIMECOMPENSATION, NULL);
		IDATA disableTimeCompensation = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLETIMECOMPENSATION, NULL);
		if (enableTimeCompensation >= disableTimeCompensation) {
			vm->checkpointState.flags |= J9VM_CRIU_ENABLE_TIME_COMPENSATION;
		}
	}

	vm->checkpointState.lastRestoreTimeInNanoseconds = -1;
	vm->checkpointState.processRestoreStartTimeInNanoseconds = -1;
#endif /* defined(J9VM_OPT_CRIU_SUPPORT) */

	{
		IDATA enableAlwaysSplitByCodes = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEALWAYSSPLITBYTECODES, NULL);
		IDATA disableAlwaysSplitByCodes = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEALWAYSSPLITBYTECODES, NULL);
		if (enableAlwaysSplitByCodes > disableAlwaysSplitByCodes) {
			vm->runtimeFlags |= J9_RUNTIME_ALWAYS_SPLIT_BYTECODES;
		} else if (enableAlwaysSplitByCodes < disableAlwaysSplitByCodes) {
			vm->runtimeFlags &= ~(UDATA)J9_RUNTIME_ALWAYS_SPLIT_BYTECODES;
		}
	}

	{
		IDATA enablePositiveHashCode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEPOSITIVEHASHCODE, NULL);
		IDATA disablePositiveHashCode = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEPOSITIVEHASHCODE, NULL);
		if (enablePositiveHashCode > disablePositiveHashCode) {
			vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_POSITIVE_HASHCODE;
		} else if (enablePositiveHashCode < disablePositiveHashCode) {
			vm->extendedRuntimeFlags &= ~(UDATA)J9_EXTENDED_RUNTIME_POSITIVE_HASHCODE;
		}
	}

	{
		IDATA enableLegacyMangling = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLELEGACYMANGLING, NULL);
		IDATA disableLegacyMangling = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLELEGACYMANGLING, NULL);
		if (enableLegacyMangling > disableLegacyMangling) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_LEGACY_MANGLING;
		} else if (enableLegacyMangling < disableLegacyMangling) {
			vm->extendedRuntimeFlags2 &= ~(UDATA)J9_EXTENDED_RUNTIME2_LEGACY_MANGLING;
		}
	}

#if defined(J9VM_ZOS_3164_INTEROPERABILITY)
	{
		IDATA enable3164Interop = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLE3164INTEROPERABILITY, NULL);
		IDATA disable3164Interop = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLE3164INTEROPERABILITY, NULL);
		if (enable3164Interop > disable3164Interop) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_3164_INTEROPERABILITY;
		} else if (enable3164Interop < disable3164Interop) {
			vm->extendedRuntimeFlags2 &= ~(UDATA)J9_EXTENDED_RUNTIME2_3164_INTEROPERABILITY;
		}
	}
#endif /* defined(J9VM_ZOS_3164_INTEROPERABILITY) */

	{
		IDATA enableUTFCache = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEUTFCACHE, NULL);
		IDATA disableUTFCache = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEUTFCACHE, NULL);
		if (enableUTFCache > disableUTFCache) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ENABLE_UTF_CACHE;
		} else if (enableUTFCache < disableUTFCache) {
			vm->extendedRuntimeFlags2 &= ~(UDATA)J9_EXTENDED_RUNTIME2_ENABLE_UTF_CACHE;
		}
	}

	/* -Xbootclasspath and -Xbootclasspath/p are not supported from Java 9 onwards */
	if (J2SE_VERSION(vm) >= J2SE_V11) {
		PORT_ACCESS_FROM_JAVAVM(vm);
		if (FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XBOOTCLASSPATH_COLON, NULL) != -1) {
			j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, "-Xbootclasspath");
			return JNI_ERR;
		}
		if (FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XBOOTCLASSPATH_P_COLON, NULL) != -1) {
			j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, "-Xbootclasspath/p");
			return JNI_ERR;
		}
	}

#if defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)
	{
		IDATA compressed = OMR_MAX(FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XCOMPRESSEDREFS, NULL),
			FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXUSECOMPRESSEDOOPS, NULL));
		IDATA nocompressed = OMR_MAX(FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XNOCOMPRESSEDREFS, NULL),
			FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOUSECOMPRESSEDOOPS, NULL));
		/* if unspecified, use compressed refs by default, unless there is a specified -Xmx over the threshold */
		if (compressed > nocompressed) {
			/* use compressed refs if explicitly requested */
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_COMPRESS_OBJECT_REFERENCES;
		} else if (compressed == nocompressed) {
			/* switching to nocompressedrefs based on -Xmx, similar logic as redirector.c:chooseJVM() */
			const char* option = "-Xmx";
			UDATA requestedHeapSize = 0;
			U_64 maxHeapForCR = 0;
			IDATA element = FIND_AND_CONSUME_VMARG2(EXACT_MEMORY_MATCH, option, NULL);
			if (element >= 0) {
				GET_MEMORY_VALUE(element, option, requestedHeapSize);
			}
			if (requestedHeapSize > 0) {
#if defined(J9ZOS39064)
				maxHeapForCR = zosGetMaxHeapSizeForCR();
#else /* defined(J9ZOS39064) */
				maxHeapForCR = MAXIMUM_HEAP_SIZE_RECOMMENDED_FOR_COMPRESSEDREFS;
#endif /* defined(J9ZOS39064) */
				/* Sizes Table for Segregated heap does not support 4-bit shift so do not use it for Metronome */
				if ((maxHeapForCR > 0) && (J9_GC_POLICY_METRONOME == isGCPolicyMetronome(vm))) {
					maxHeapForCR = MAXIMUM_HEAP_SIZE_RECOMMENDED_FOR_3BIT_SHIFT_COMPRESSEDREFS;
				}
			}
			if (requestedHeapSize <= maxHeapForCR) {
				vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_COMPRESS_OBJECT_REFERENCES;
			}
		}
	}
#endif /* defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS) */

#if JAVA_SPEC_VERSION >= 18
	{
		/* JEP 421: Deprecate Finalization for Removal - Expecting values: "enabled" and "disabled".
		 * The default value for now will be "enabled", i.e., finalization is enabled by default.
		 * Other values will cause an error, additional values may be added in the future.
		 */
		IDATA argIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_DISABLE_FINALIZATION, NULL);
		if (argIndex >= 0) {
			PORT_ACCESS_FROM_JAVAVM(vm);
			char *optionArg = NULL;
			GET_OPTION_VALUE(argIndex, '=', &optionArg);
			if (NULL != optionArg) {
				if (strcmp(optionArg, "disabled") == 0) {
					vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_DISABLE_FINALIZATION;
				} else if (strcmp(optionArg, "enabled") == 0) {
					/* do nothing as finalization is enabled by default */
				} else {
					j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, VMOPT_DISABLE_FINALIZATION);
				}
			} else {
				j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, VMOPT_DISABLE_FINALIZATION);
			}
		}
	}
#endif /* JAVA_SPEC_VERSION >= 18 */

	{
		IDATA showHiddenFrames = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSHOWHIDDENFRAMES, NULL);
		IDATA noshowHiddenFrames = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOSHOWHIDDENFRAMES, NULL);
		if (showHiddenFrames > noshowHiddenFrames) {
			vm->runtimeFlags |= J9_RUNTIME_SHOW_HIDDEN_FRAMES;
		}
	}

	{
		IDATA dynamicHeapification = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDYNAMICHEAPIFICATION, NULL);
		IDATA noDynamicHeapification = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNODYNAMICHEAPIFICATION, NULL);
		if (dynamicHeapification > noDynamicHeapification) {
			vm->runtimeFlags |= J9_RUNTIME_DYNAMIC_HEAPIFICATION;
		} else if (dynamicHeapification < noDynamicHeapification) {
			vm->runtimeFlags &= ~(UDATA)J9_RUNTIME_DYNAMIC_HEAPIFICATION;
		}
	}

#if JAVA_SPEC_VERSION >= 19
	{
		IDATA showCarrierFrames = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSHOWCARRIERFRAMES, NULL);
		IDATA noShowCarrierFrames = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOSHOWCARRIERFRAMES, NULL);
		if (showCarrierFrames > noShowCarrierFrames) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_SHOW_CARRIER_FRAMES;
		}
	}
#endif /* JAVA_SPEC_VERSION >= 19 */

	{
		IDATA cpuLoadCompatibility = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCPULOADCOMPATIBILITY, NULL);
		IDATA noCpuLoadCompatibility = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOCPULOADCOMPATIBILITY, NULL);
		if (cpuLoadCompatibility > noCpuLoadCompatibility) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_CPU_LOAD_COMPATIBILITY;
		}
	}
	{
		IDATA enableExtendedHCR =  FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXENABLEEXTENDEDHCR, NULL);
		IDATA disableExtendedHCR =  FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXDISABLEEXTENDEDHCR, NULL);
		if (enableExtendedHCR > disableExtendedHCR) {
			vm->extendedRuntimeFlags2 &= ~J9_EXTENDED_RUNTIME2_DISABLE_EXTENDED_HCR;
		} else if (enableExtendedHCR < disableExtendedHCR) {
			vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_DISABLE_EXTENDED_HCR;
		}
	}

#if defined(J9VM_OPT_JFR)
	{
		IDATA flightRecorder = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXFLIGHTRECORDER, NULL);
		IDATA noFlightRecorder = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOFLIGHTRECORDER, NULL);

		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_JFR_ENABLED;

		if (flightRecorder < noFlightRecorder) {
			vm->extendedRuntimeFlags2 &= ~(UDATA)J9_EXTENDED_RUNTIME2_JFR_ENABLED;
		}
	}
	{
		if (0 <= FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXSTARTFLIGHTRECORDING, NULL)) {
			vm->extendedRuntimeFlags3 |= J9_EXTENDED_RUNTIME3_START_FLIGHT_RECORDING;
		}
	}
#endif /* defined(J9VM_OPT_JFR) */

#if JAVA_SPEC_VERSION >= 24
	{
		IDATA enableYieldPinning = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXYIELDPINNEDVIRTUALTHREADS, NULL);
		IDATA disableYieldPinning = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOYIELDPINNEDVIRTUALTHREADS, NULL);

		/* Enable yielding of pinned continuations by default. */
		vm->extendedRuntimeFlags3 |= J9_EXTENDED_RUNTIME3_YIELD_PINNED_CONTINUATION;

		if (enableYieldPinning < disableYieldPinning) {
			/* Disable yielding of pinned continuations. */
			vm->extendedRuntimeFlags3 &= ~J9_EXTENDED_RUNTIME3_YIELD_PINNED_CONTINUATION;
		}
	}
#endif /* JAVA_SPEC_VERSION >= 24 */

	{
		IDATA cacheMaps = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCACHEMAPS, NULL);
		IDATA noCacheMaps = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOCACHEMAPS, NULL);

		if (cacheMaps > noCacheMaps) {
			vm->extendedRuntimeFlags3 |= J9_EXTENDED_RUNTIME3_CACHE_MAPS;
		} else if (cacheMaps < noCacheMaps) {
			vm->extendedRuntimeFlags3 &= ~J9_EXTENDED_RUNTIME3_CACHE_MAPS;
		}
	}

	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXKEEPJNIIDS, NULL) != -1) {
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_ALWAYS_KEEP_JNI_IDS;
	}
	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOKEEPJNIIDS, NULL) != -1) {
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_NEVER_KEEP_JNI_IDS;
	}
#if JAVA_SPEC_VERSION >= 22
#define VMOPT_XFFIPROTO "-Xffiproto"
	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XFFIPROTO, NULL) != -1) {
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_FFI_PROTO;
	}
#endif /* JAVA_SPEC_VERSION >= 22 */

	return JNI_OK;
}


/* Called for each stage. Runs J9VMDllMain for every library in the table. */

static jint
runInitializationStage(J9JavaVM* vm, IDATA stage)
{
	RunDllMainData userData;
	J9VMThread *mainThread = vm->mainThread;

	/* Once the main J9VMThread has been created, each init stage expects the thread
	 * to have entered the VM and released VM access.
	 */
	if (NULL != mainThread) {
#if defined(J9VM_INTERP_ATOMIC_FREE_JNI)
		if (mainThread->inNative) {
			enterVMFromJNI(mainThread);
			releaseVMAccess(mainThread);
		} else
#endif /* J9VM_INTERP_ATOMIC_FREE_JNI */
		if (J9_ARE_ANY_BITS_SET(mainThread->publicFlags, J9_PUBLIC_FLAGS_VM_ACCESS)) {
			releaseVMAccess(mainThread);
		}
	}

	userData.vm = vm;
	userData.stage = stage;
	userData.reserved = 0;
	userData.filterFlags = 0;

#ifdef J9VM_INTERP_VERBOSE
	JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\nRunning initialization stage %s\n", getNameForStage(stage));
#endif
	pool_do(vm->dllLoadTable, runJ9VMDllMain, &userData);

	return checkPostStage(vm, stage);
}


/* Run from a pool_do for each library. Invokes the J9VMDllMain method.
	Errors are reported in the entry->fatalErrorStr. */

static void
runJ9VMDllMain(void* dllLoadInfo, void* userDataTemp)
{
	J9VMDllLoadInfo* entry = (J9VMDllLoadInfo*) dllLoadInfo;
	RunDllMainData* userData = (RunDllMainData*) userDataTemp;
	IDATA (*J9VMDllMainFunc)(J9JavaVM*, IDATA, void*);
	IDATA rc = 0;

	PORT_ACCESS_FROM_JAVAVM(userData->vm);

	if (entry->loadFlags & NO_J9VMDLLMAIN)
		return;

	if (entry->loadFlags & BUNDLED_COMP) {
		if (!(entry->loadFlags & (EARLY_LOAD | LOAD_BY_DEFAULT | FORCE_LATE_LOAD))) {
			return;
		}
	}

	/* If filterFlags is set, only run J9VMDllMain for table entries who's flags match ALL of the filterFlags */
	if (userData->filterFlags==0 || ((userData->filterFlags & entry->loadFlags) == userData->filterFlags)) {

		/* Loaded libraries are guaranteed to have a descriptor. NOT_A_LIBRARY entries will already have a j9vmdllmain entry */
		if (!entry->j9vmdllmain && entry->descriptor) {
			if (j9sl_lookup_name (entry->descriptor , "J9VMDllMain", (void *) &J9VMDllMainFunc, "PLpL")) {
				setErrorJ9dll(PORTLIB, entry, j9nls_lookup_message(J9NLS_DO_NOT_APPEND_NEWLINE | J9NLS_ERROR, J9NLS_VM_J9VMDLLMAIN_NOT_FOUND, NULL), FALSE);
				return;
			} else {
				entry->j9vmdllmain = J9VMDllMainFunc;
			}
		}
		if (entry->j9vmdllmain) {
			I_64 start = 0;
			I_64 end = 0;
			char* dllName = (entry->loadFlags & ALTERNATE_LIBRARY_USED) ? entry->alternateDllName : entry->dllName;

			J9VMDllMainFunc = entry->j9vmdllmain;
			JVMINIT_VERBOSE_INIT_VM_TRACE1(userData->vm, "\tfor library %s...\n", dllName);

			/* reserved = shutdownDueToExit */
			if (userData->vm->verboseLevel & VERBOSE_INIT) {
				start = j9time_nano_time();
			}
			rc = (*J9VMDllMainFunc) (userData->vm, userData->stage, userData->reserved);
			if (userData->vm->verboseLevel & VERBOSE_INIT) {
				end = j9time_nano_time();
			}

			if (rc==J9VMDLLMAIN_FAILED && (entry->fatalErrorStr==NULL || strlen(entry->fatalErrorStr)==0)) {
				setErrorJ9dll(PORTLIB, entry, j9nls_lookup_message(J9NLS_DO_NOT_APPEND_NEWLINE | J9NLS_ERROR, J9NLS_VM_J9VMDLLMAIN_FAILED, NULL), FALSE);
			}
			if (rc==J9VMDLLMAIN_SILENT_EXIT_VM) {
				setErrorJ9dll(PORTLIB, entry, SILENT_EXIT_STRING, FALSE);
			}
			/* Only COMPLETE_STAGE for initialization stages */
			if (userData->stage >=0) {
				COMPLETE_STAGE(entry->completedBits, userData->stage);
			}
			JVMINIT_VERBOSE_INIT_VM_TRACE2(userData->vm, "\t\tcompleted with rc=%d in %lld nano sec.\n", rc, (end-start));
			JVMINIT_VERBOSE_INIT_TRACE_WORKING_SET(userData->vm);
		}
	}
}





/* Loads libraries based on flags. */

static jint
runLoadStage(J9JavaVM *vm, IDATA flags)
{
	struct LoadInitData userData;

	userData.vm = vm;
	userData.flags = flags;

#ifdef J9VM_INTERP_VERBOSE
	JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\nLoading libraries at load stage %s:\n", getNameForLoadStage(flags));
#endif
	pool_do(vm->dllLoadTable, loadDLL, &userData);

	return checkPostStage(vm, LOAD_STAGE);
}



/**
 * \brief	Run the specified shutdown stage with JVMTI first in the list
 *
 * @param[in] vm            VM Structure
 * @param[in] userData      RunDllMainData structure describing the stage and filter flags
 * @return none
 *
 *	Run the specified shutdown stage, with JVMTI first in the list. This is done mainly to avoid
 *  the JIT going away before TI can deallocate things via vm->jitConfig while freeing environments.
 */
static void
runShutdownStageJvmtiFirst(J9JavaVM * vm, RunDllMainData * userData)
{
	J9VMDllLoadInfo* jvmtiLoadInfo;
	void *anElement;
	pool_state aState;


	jvmtiLoadInfo = FIND_DLL_TABLE_ENTRY(J9_JVMTI_DLL_NAME);
	if (jvmtiLoadInfo) {
		runJ9VMDllMain(jvmtiLoadInfo, userData);
	}

	anElement = pool_startDo(vm->dllLoadTable, &aState);
	while (anElement) {
		if (anElement != jvmtiLoadInfo) {
			runJ9VMDllMain(anElement, userData);
		}
		anElement = pool_nextDo(&aState);
	}

	return;
}

/* Called for each stage. Runs J9VMDllMain for every library in the table. */

static jint
runShutdownStage(J9JavaVM* vm, IDATA stage, void* reserved, UDATA filterFlags)
{
	RunDllMainData userData;

	userData.vm = vm;
	userData.stage = stage;
	userData.reserved = reserved;
	userData.filterFlags = filterFlags;	/* only run J9VMDllMain for DLL table entries whos flags match ALL filterFlags */

#ifdef J9VM_INTERP_VERBOSE
	JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\nRunning shutdown stage %s...\n", getNameForStage(stage));
#endif

	if ((stage == JVM_EXIT_STAGE) || (stage == LIBRARIES_ONUNLOAD)) {
		IDATA serializeResult = 0;

		TRIGGER_J9HOOK_VM_SERIALIZE_SHARED_CACHE(vm->hookInterface, vm, serializeResult);
	}

#ifdef J9VM_OPT_JVMTI
	runShutdownStageJvmtiFirst(vm, &userData);
#else
	pool_do(vm->dllLoadTable, runJ9VMDllMain, &userData);
#endif

	if (stage==JVM_EXIT_STAGE) {		/* We are exiting - we don't care about shutdown error messages */
		return JNI_OK;
	} else {
		return checkPostStage(vm, stage);
	}
}


/* Runs after each stage. Returns FALSE if an error occurred. */

static jint
checkPostStage(J9JavaVM* vm, IDATA stage)
{
	CheckPostStageData userData;

	userData.vm = vm;
	userData.stage = stage;
	userData.success = JNI_OK;

#ifdef J9VM_INTERP_VERBOSE
	JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\nChecking results for stage %s\n", getNameForStage(stage));
#endif
	pool_do(vm->dllLoadTable, checkDllInfo, &userData);

	return userData.success;
}


/* Unloads libraries being forced to unload. */

static jint
runForcedUnloadStage(J9JavaVM *vm)
{
	struct LoadInitData userData;

	userData.vm = vm;
	userData.flags = FORCE_UNLOAD;
	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\nForcing unload of libraries...\n");
	pool_do(vm->dllLoadTable, unloadDLL, &userData);
	return checkPostStage(vm, UNLOAD_STAGE);
}


/* Run using a pool_do, this method shuts down all libraries that match the flags given. */

static void
unloadDLL(void* dllLoadInfo, void* userDataTemp)
{
	J9VMDllLoadInfo* entry = (J9VMDllLoadInfo*) dllLoadInfo;
	LoadInitData* userData = (LoadInitData*) userDataTemp;

	if (entry->descriptor && (entry->loadFlags & userData->flags)) {
		if (shutdownDLL(userData->vm, entry->descriptor, FALSE) == 0) {
			entry->descriptor = 0;
			entry->j9vmdllmain = 0;
			entry->loadFlags &= ~LOADED;
			JVMINIT_VERBOSE_INIT_VM_TRACE1(userData->vm, "\tfor %s\n", entry->dllName);
		} else {
			entry->loadFlags |= FAILED_TO_UNLOAD;
		}
	}
}


/* Marks all -Dfoo=bar java options as NOT_CONSUMABLE.
	Also walks the ignoredOptionTable and if it sees any of those options in the vm_args and marks them as ignored */

static void
registerIgnoredOptions(J9PortLibrary *portLibrary, J9VMInitArgs *j9vm_args)
{
	UDATA i = 0;
	const char *option = NULL;
	const struct J9VMIgnoredOption *ignoredOptionTablePtr = (const struct J9VMIgnoredOption *)GLOBAL_TABLE(ignoredOptionTable);

	PORT_ACCESS_FROM_PORT(portLibrary);

	for (i = 0; i < j9vm_args->nOptions; i++) {
		option = getOptionString(j9vm_args, i);				/* may return mapped value */
		if ((strlen(option) > 2) && ('-' == option[0])) {
			switch (option[1]) {
				/* Mark all -D options as non-consumable */
				case 'D' :
					SET_FLAG_AT_INDEX(NOT_CONSUMABLE_ARG, i);
					break;
			}
		}
	}

	/* Mark all ignored options as consumed */
	for (i = 0; i < ignoredOptionTableSize; i++) {
		const char *optionName = ignoredOptionTablePtr[i].optionName;
#if JAVA_SPEC_VERSION >= 22
		IDATA index =
#endif /* JAVA_SPEC_VERSION >= 22 */
				findArgInVMArgs( PORTLIB, j9vm_args, ignoredOptionTablePtr[i].match, optionName, NULL, TRUE);
#if JAVA_SPEC_VERSION >= 22
		if ((index >= 0)
			&& ((0 == strcmp(VMOPT_XDEBUG, optionName)) || (0 == strcmp(VMOPT_XNOAGENT, optionName)))
		) {
			j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_DEPRECATED_OPTION, JAVA_VM_NAME, optionName);
		}
#endif /* JAVA_SPEC_VERSION >= 22 */
	}
}

#if (defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)) || (defined(LINUX) && defined(J9VM_GC_REALTIME))
/**
 * Check if -Xgcpolicy:metronome is specified on the command line
 *
 * @param javaVM[in] java VM
 * @returns TRUE if -Xgcpolicy:metronome is specified, FALSE otherwise
 */
static BOOLEAN
isGCPolicyMetronome(J9JavaVM *javaVM)
{
	BOOLEAN isSRT = FALSE;
	JavaVMInitArgs *args =javaVM->vmArgsArray->actualVMArgs;
	int i = 0;
	/* look for the option to see if it has been specified on the command line */
	for (i = 0; i < args->nOptions; i++ ) {
		if (0 == strcmp(args->options[i].optionString, "-Xgcpolicy:metronome")) {
			isSRT = TRUE;
			break;
		}
	}

	return isSRT;
}
#endif /* (defined(OMR_GC_COMPRESSED_POINTERS) && defined(OMR_GC_FULL_POINTERS)) || (defined(LINUX) && defined(J9VM_GC_REALTIME)) */

#if defined(LINUX) && defined(J9VM_GC_REALTIME)
/* Linux SRT only */

/**
 * Set the scheduling policy according to if -Xgcpolicy:metronome is on the command line
 *
 * @param javaVM[in] java VM
 * @returns 0 upon success, -1 otherwise
 */
static IDATA
setJ9ThreadSchedulingAlgorithm(J9JavaVM *javaVM)
{
	UDATA shouldUseRealtimeScheduling = J9THREAD_LIB_CONTROL_USE_REALTIME_SCHEDULING_DISABLED;
	IDATA success = -1;

	if (isGCPolicyMetronome(javaVM)) {
		shouldUseRealtimeScheduling = J9THREAD_LIB_CONTROL_USE_REALTIME_SCHEDULING_ENABLED;
	}
	success = omrthread_lib_control(J9THREAD_LIB_CONTROL_USE_REALTIME_SCHEDULING, shouldUseRealtimeScheduling);

	return success;
}
#endif /* defined(LINUX) && defined(J9VM_GC_REALTIME) */


IDATA
threadInitStages(J9JavaVM* vm, IDATA stage, void* reserved)
{
	J9VMDllLoadInfo* loadInfo;
	IDATA returnVal = J9VMDLLMAIN_OK;
	IDATA parseError = 0;
	char* parseErrorOption = NULL;
	IDATA xThrIndex, xJNIIndex;
	char* thrOptions = NULL;
	char* jniOptions = NULL;

	PORT_ACCESS_FROM_JAVAVM(vm);

	switch(stage) {
		case PORT_LIBRARY_GUARANTEED :
			/* default stack size for forked Java threads */
			if (0 != (parseError = setMemoryOptionToOptElse(vm, &(vm->defaultOSStackSize), VMOPT_XMSO, J9_OS_STACK_SIZE, TRUE))) {
				parseErrorOption = VMOPT_XMSO;
				goto _memParseError;
			}

#if defined(J9ZOS39064)
			/* Use a 1MB OS stack on z/OS 64-bit as this is what the OS
			 * allocates anyway, using IARV64 GETSTOR to allocate a segment.
			 */
			if (vm->defaultOSStackSize < J9_OS_STACK_SIZE) {
				vm->defaultOSStackSize = J9_OS_STACK_SIZE;
			}
#endif /* defined(J9ZOS39064) */

#if defined(J9VM_INTERP_GROWABLE_STACKS)
			if (0 != (parseError = setMemoryOptionToOptElse(vm, &(vm->initialStackSize), VMOPT_XISS, J9_INITIAL_STACK_SIZE, TRUE))) {
				parseErrorOption = VMOPT_XISS;
				goto _memParseError;
			}
			if (0 != (parseError = setMemoryOptionToOptElse(vm, &(vm->stackSizeIncrement), VMOPT_XSSI, J9_STACK_SIZE_INCREMENT, TRUE))) {
				parseErrorOption = VMOPT_XSSI;
				goto _memParseError;
			}
#endif
			if (0 != (parseError = setMemoryOptionToOptElse(vm, &(vm->stackSize), VMOPT_XSS, J9_STACK_SIZE, TRUE))) {
				parseErrorOption = VMOPT_XSS;
				goto _memParseError;
			}
			break;
		case ALL_DEFAULT_LIBRARIES_LOADED :
			break;
		case ALL_LIBRARIES_LOADED :
			break;
		case DLL_LOAD_TABLE_FINALIZED :
			break;
		case VM_THREADING_INITIALIZED :
			loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_THREAD_INIT );
#if defined(J9VM_INTERP_VERBOSE)
			if (vm->verboseLevel & VERBOSE_STACK) {
				vm->runtimeFlags |= (J9_RUNTIME_REPORT_STACK_USE | J9_RUNTIME_PAINT_STACK);
				omrthread_enable_stack_usage(1);
			}
#endif

#if defined(LINUX) && defined(J9VM_GC_REALTIME)
			/* Linux SRT only */
			if (0 != setJ9ThreadSchedulingAlgorithm(vm)) {
				goto _error;
			}
#endif /* defined(LINUX) && defined(J9VM_GC_REALTIME) */

			if ((xThrIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XTHR_COLON, NULL)) >= 0) {
				GET_OPTION_VALUE( xThrIndex, ':', &thrOptions);
			}
			if (threadParseArguments(vm, thrOptions) != 0) {
				setErrorJ9dll(PORTLIB, loadInfo, "cannot parse -Xthr:", FALSE);
				goto _error;
			}

			if (initializeVMThreading(vm) != 0) {
				setErrorJ9dll(PORTLIB, loadInfo, "cannot initialize VM threading", FALSE);
				goto _error;
			}

			if ((xJNIIndex = FIND_AND_CONSUME_VMARG(STARTSWITH_MATCH, VMOPT_XJNI_COLON, NULL)) >= 0) {
				GET_OPTION_VALUE( xJNIIndex, ':', &jniOptions);
			}
			returnVal = jniParseArguments(vm, jniOptions);
			if (returnVal != J9VMDLLMAIN_OK) {
				setErrorJ9dll(PORTLIB, loadInfo, "cannot parse -Xjni:", FALSE);
				return returnVal;
			}
			break;
		case HEAP_STRUCTURES_INITIALIZED :
			break;
		case ALL_VM_ARGS_CONSUMED :
			break;
		case BYTECODE_TABLE_SET :
			break;
		case SYSTEM_CLASSLOADER_SET :
			break;
		case DEBUG_SERVER_INITIALIZED :
			break;
		case JIT_INITIALIZED :
			break;
		case ABOUT_TO_BOOTSTRAP :
#if defined(J9VM_THR_ASYNC_NAME_UPDATE)
			vm->threadNameHandlerKey = J9RegisterAsyncEvent(vm, setThreadNameAsyncHandler, vm);
			if (vm->threadNameHandlerKey < 0) {
				loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_THREAD_INIT );
				setErrorJ9dll(PORTLIB, loadInfo, "cannot initialize threadNameHandlerKey", FALSE);
				goto _error;
			}
#endif /* J9VM_THR_ASYNC_NAME_UPDATE */
			break;
		case JCL_INITIALIZED :
			break;
	}
	return returnVal;
	_memParseError :
		loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_THREAD_INIT );
		generateMemoryOptionParseError(vm, loadInfo, parseError, parseErrorOption);
	_error :
		return J9VMDLLMAIN_FAILED;
}


IDATA
zeroInitStages(J9JavaVM* vm, IDATA stage, void* reserved)
{
	J9VMDllLoadInfo* loadInfo;
	IDATA returnVal = J9VMDLLMAIN_OK;
	IDATA argIndex1 = -1;
	BOOLEAN describe = FALSE;

	PORT_ACCESS_FROM_JAVAVM(vm);

	switch(stage) {
		case PORT_LIBRARY_GUARANTEED :
			/* -Xzero option is removed from Java 9 */
			if (J2SE_VERSION(vm) >= J2SE_V11) {
				vm->zeroOptions = 0;
			} else {
				vm->zeroOptions = J9VM_ZERO_SHAREBOOTZIPCACHE;
				argIndex1 = FIND_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, VMOPT_XZERO, NULL);
				while(argIndex1 >= 0) {
					char *optionString;
					char optionsBuffer[LARGE_STRING_BUF_SIZE];
					char* optionsBufferPtr = (char*)optionsBuffer;
					UDATA rc;

					optionString = vm->vmArgsArray->actualVMArgs->options[argIndex1].optionString;
					if (strcmp(optionString, VMOPT_XZERO) == 0) {
						/* Found -Xzero, enable the default Zero options */
#if defined(J9VM_OPT_ZERO)
						vm->zeroOptions = J9VM_ZERO_DEFAULT_OPTIONS;
#endif
						CONSUME_ARG(vm->vmArgsArray, argIndex1);
						argIndex1 = FIND_NEXT_ARG_IN_VMARGS_FORWARD( STARTSWITH_MATCH, VMOPT_XZERO, NULL, argIndex1);
						continue;
					} else if (strncmp(optionString, VMOPT_XZERO_COLON, sizeof(VMOPT_XZERO_COLON) - 1) != 0) {
						/* If the option does not start with -Xzero:, do not consume it so it later causes option unrecognised */
						argIndex1 = FIND_NEXT_ARG_IN_VMARGS_FORWARD( STARTSWITH_MATCH, VMOPT_XZERO, NULL, argIndex1);
						continue;
					}
					CONSUME_ARG(vm->vmArgsArray, argIndex1);
					rc = GET_OPTION_VALUES(argIndex1, ':', ',', &optionsBufferPtr, LARGE_STRING_BUF_SIZE);
					if (rc == OPTION_OK) {
						if (*optionsBufferPtr == 0) {
							/* Enable default Zero options */
#if defined(J9VM_OPT_ZERO)
							vm->zeroOptions = J9VM_ZERO_DEFAULT_OPTIONS;
#endif
						} else {
							while (*optionsBufferPtr) {
								char *errorBuffer;

								if (strcmp(optionsBufferPtr, VMOPT_ZERO_NONE) == 0) {
									vm->zeroOptions = 0;
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, VMOPT_ZERO_J9ZIP) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, "no"VMOPT_ZERO_J9ZIP) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, VMOPT_ZERO_SHAREZIP) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, "no"VMOPT_ZERO_SHAREZIP) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, VMOPT_ZERO_SHAREBOOTZIP) == 0) {
									vm->zeroOptions |= J9VM_ZERO_SHAREBOOTZIPCACHE;
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, "no"VMOPT_ZERO_SHAREBOOTZIP) == 0) {
									vm->zeroOptions &= ~J9VM_ZERO_SHAREBOOTZIPCACHE;
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, VMOPT_ZERO_SHARESTRING) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, "no"VMOPT_ZERO_SHARESTRING) == 0) {
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else if (strcmp(optionsBufferPtr, VMOPT_ZERO_DESCRIBE) == 0) {
									describe = TRUE;
									optionsBufferPtr += strlen(optionsBufferPtr) + 1;
									continue;
								} else {
									errorBuffer = j9mem_allocate_memory(LARGE_STRING_BUF_SIZE, OMRMEM_CATEGORY_VM);
									if (errorBuffer) {
										strcpy(errorBuffer, VMOPT_XZERO_COLON);
										safeCat(errorBuffer, optionsBufferPtr, LARGE_STRING_BUF_SIZE);
										j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, errorBuffer);
										j9mem_free_memory(errorBuffer);
									} else {
										loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_ZERO_INIT );
										setErrorJ9dll(PORTLIB, loadInfo, "Cannot allocate memory for error message", FALSE);
									}
									goto _error;
								}
							}
						}
					} else {
						loadInfo = FIND_DLL_TABLE_ENTRY( FUNCTION_ZERO_INIT );
						setErrorJ9dll(PORTLIB, loadInfo, "Error parsing " VMOPT_XZERO_COLON " options", FALSE);
					}
					argIndex1 = FIND_NEXT_ARG_IN_VMARGS_FORWARD( STARTSWITH_MATCH, VMOPT_XZERO, NULL, argIndex1);
				}
				if (describe) {
					BOOLEAN foundOption = FALSE;
					j9tty_printf(PORTLIB, VMOPT_XZERO_COLON);
					if ((vm->zeroOptions & J9VM_ZERO_SHAREBOOTZIPCACHE) != 0) {
						j9tty_printf(PORTLIB, "%s"VMOPT_ZERO_SHAREBOOTZIP, foundOption ? "," : "");
						foundOption = TRUE;
					}
					j9tty_printf(PORTLIB, "%s\n", foundOption ? "" : VMOPT_ZERO_NONE);
				}
			}
			break;
		case ALL_DEFAULT_LIBRARIES_LOADED :
			break;
		case ALL_LIBRARIES_LOADED :
			break;
		case DLL_LOAD_TABLE_FINALIZED :
			break;
		case VM_THREADING_INITIALIZED :
			break;
		case HEAP_STRUCTURES_INITIALIZED :
			break;
		case ALL_VM_ARGS_CONSUMED :
			break;
		case BYTECODE_TABLE_SET :
			break;
		case SYSTEM_CLASSLOADER_SET :
			break;
		case DEBUG_SERVER_INITIALIZED :
			break;
		case JIT_INITIALIZED :
			break;
		case ABOUT_TO_BOOTSTRAP :
			break;
		case JCL_INITIALIZED :
			break;
	}
	return returnVal;
	_error :
		return J9VMDLLMAIN_FAILED;
}


static void *
getOptionExtraInfo(J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args, IDATA match, char* optionName)
{
	PORT_ACCESS_FROM_PORT(portLibrary);
	IDATA element = findArgInVMArgs( PORTLIB, j9vm_args, match, optionName, NULL, FALSE);
	if (element >=0)
		return (void*) (j9vm_args->actualVMArgs->options[element].extraInfo);
	else
		return (void*) NULL;
}

/* This function runs the JVM_OnLoad of a loaded library */

UDATA
runJVMOnLoad(J9JavaVM* vm, J9VMDllLoadInfo* loadInfo, char* options)
{
	jint (JNICALL * jvmOnLoadFunc)(JavaVM *, char *, void *);
	jint rc;

	PORT_ACCESS_FROM_JAVAVM(vm);

	if (loadInfo->descriptor) {
		if (j9sl_lookup_name ( loadInfo->descriptor , "JVM_OnLoad", (void *) &jvmOnLoadFunc, "iLLL")) {
			setErrorJ9dll(PORTLIB, loadInfo, "JVM_OnLoad not found", FALSE);
		} else {
			JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "Running JVM_OnLoad for %s\n", loadInfo->dllName);
			if ((rc = (*jvmOnLoadFunc) ((JavaVM*)vm, options, NULL)) != JNI_OK)
				setErrorJ9dll(PORTLIB, loadInfo, "JVM_OnLoad failed", FALSE);
			return (rc == JNI_OK);
		}
	}
	return FALSE;
}


/* Close all loaded DLLs. All library shutdown code should have been run. */

static void
closeAllDLLs(J9JavaVM* vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);

	if (vm->dllLoadTable) {
		J9VMDllLoadInfo* entry;
		pool_state aState;

		entry = (J9VMDllLoadInfo*) pool_startDo(vm->dllLoadTable, &aState);
		while(entry) {
			if (entry->descriptor && !(entry->loadFlags & NEVER_CLOSE_DLL)) {
				char* dllName = (entry->loadFlags & ALTERNATE_LIBRARY_USED) ? entry->alternateDllName : entry->dllName;

				j9sl_close_shared_library(entry->descriptor);
				JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "Closing library %s\n", dllName);
			}
			entry = (J9VMDllLoadInfo*) pool_nextDo(&aState);
		}
	}
}


/* Initializes all Xrun libraries that have been loaded (note: Does not initialize agent Xruns). */

static jint
initializeXruns(J9JavaVM* vm)
{
	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\nInitializing Xrun libraries:\n");

	if (vm->dllLoadTable) {
		J9VMDllLoadInfo* entry;
		pool_state aState;
		char* options;

		entry = (J9VMDllLoadInfo*) pool_startDo(vm->dllLoadTable, &aState);
		while(entry) {
			if ((entry->loadFlags & (XRUN_LIBRARY | AGENT_XRUN)) == XRUN_LIBRARY) {
				options = (char*)entry->reserved;				/* reserved is used to store any options for Xruns*/
				JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\tRunning JVM_OnLoad for library %s\n", entry->dllName);
				if (!runJVMOnLoad(vm, entry, options)) {
					break;
				}
			}
			entry = (J9VMDllLoadInfo*) pool_nextDo(&aState);
		}
	}
	return checkPostStage(vm, XRUN_INIT_STAGE);
}


/* Runs UnOnload functions for libraries such as Xruns */

static void
runUnOnloads(J9JavaVM* vm, UDATA shutdownDueToExit)
{
	PORT_ACCESS_FROM_JAVAVM(vm);

	if (vm->dllLoadTable) {
		jint (JNICALL * j9OnUnLoadFunc)(J9JavaVM *, void*);
		J9VMDllLoadInfo* entry;
		pool_state aState;

		entry = (J9VMDllLoadInfo*) pool_startDo(vm->dllLoadTable, &aState);
		while(entry) {
			if (entry->descriptor && ((entry->loadFlags & (NO_J9VMDLLMAIN | AGENT_XRUN)) == NO_J9VMDLLMAIN)) {
				if(0 == j9sl_lookup_name ( entry->descriptor , "JVM_OnUnload", (void *) &j9OnUnLoadFunc, "iLL")) {
					JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "Running JVM_OnUnload for %s\n", entry->dllName);
					(*j9OnUnLoadFunc) (vm, (void *) shutdownDueToExit);
				}
			}
			entry = (J9VMDllLoadInfo*) pool_nextDo(&aState);
		}
	}
}

#if (defined(J9VM_OPT_SIDECAR))
/* If a command-line option exists that must be mapped to a J9 option, this creates the mapping struct and associates it with the option

	This function can be called from the internal function table so that libraries themselves can register their own mappings.
	This must be done at the earliest opportunity possible stage.
*/

IDATA
registerCmdLineMapping(J9JavaVM* vm, char* sov_option, char* j9_option, UDATA mapFlags)
{
	IDATA index = 0;
	UDATA match = 0;

	PORT_ACCESS_FROM_JAVAVM(vm);

	/* Enforce a search type for a given map type, otherwise the permutations become impossible to test! */
	if (mapFlags & EXACT_MAP_NO_OPTIONS) {
		match = EXACT_MATCH;
	} else
	if (mapFlags & EXACT_MAP_WITH_OPTIONS) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & STARTSWITH_MAP_NO_OPTIONS) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & MAP_TWO_COLONS_TO_ONE) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & MAP_ONE_COLON_TO_TWO) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & MAP_WITH_INCLUSIVE_OPTIONS) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & MAP_MEMORY_OPTION) {
		match = EXACT_MEMORY_MATCH;
	} else
	if (mapFlags & STARTSWITH_INVALID_OPTION) {
		match = STARTSWITH_MATCH;
	} else
	if (mapFlags & EXACT_INVALID_OPTION) {
		match = EXACT_MATCH;
	}

	/* Walk the command-line and create a mapping for each instance of the option */
	do {
		/* Note: (index << STOP_AT_INDEX_SHIFT) is copied from FIND_NEXT_ARG_IN_VMARGS which we can't use here as we have no function table yet */
		index = findArgInVMArgs( PORTLIB, vm->vmArgsArray, (match | (index << STOP_AT_INDEX_SHIFT)), sov_option, NULL, FALSE);

		/* If the option exists and has not already been mapped... */
		if (index >= 0 && !HAS_MAPPING(vm->vmArgsArray, index)) {
			if (createMapping(vm, j9_option, sov_option, mapFlags, index)==RC_FAILED) {
				return RC_FAILED;
			}
		}
	} while (index > 0);	/* If index == 0, this would cause findArgInVMArgs to search from the start again, which would cause an infinite loop */

	return 0;
}
#endif /* J9VM_OPT_SIDECAR */


#if (defined(J9VM_OPT_SIDECAR))
/* Registers j9 command-line args that map to sovereign/other command-line args
	Note that the order in which the mappings are registered is important.
	If a mapping already exists for an option, another mapping cannot be registered for that option. */

static IDATA
registerVMCmdLineMappings(J9JavaVM* vm)
{
#if JAVA_SPEC_VERSION < 21
	char jitOpt[SMALL_STRING_BUF_SIZE];				/* Plenty big enough */
	char* changeCursor;
	IDATA bufLeft = 0;

	/* Allow a single string -Djava.compiler=<value> to have different values */
	strcpy(jitOpt, SYSPROP_DJAVA_COMPILER_EQUALS);
	bufLeft = SMALL_STRING_BUF_SIZE - strlen(jitOpt) - 1;
	changeCursor = &jitOpt[strlen(jitOpt)];
#endif /* JAVA_SPEC_VERSION < 21 */

#ifdef J9VM_OPT_JVMTI
	if (registerCmdLineMapping(vm, MAPOPT_JAVAAGENT_COLON, MAPOPT_AGENTLIB_INSTRUMENT_EQUALS, MAP_WITH_INCLUSIVE_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
#endif

	if (registerCmdLineMapping(vm, MAPOPT_XCOMP, MAPOPT_XJIT_COUNT0, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
#if JAVA_SPEC_VERSION < 21
	strncpy(changeCursor, DJCOPT_JITC, bufLeft);
	if (registerCmdLineMapping(vm, jitOpt, VMOPT_XJIT, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	strncpy(changeCursor, J9_JIT_DLL_NAME, bufLeft);
	if (registerCmdLineMapping(vm, jitOpt, VMOPT_XJIT, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	if (registerCmdLineMapping(vm, SYSPROP_DJAVA_COMPILER_EQUALS, VMOPT_XINT, STARTSWITH_MAP_NO_OPTIONS) == RC_FAILED) {				/* any other -Djava.compiler= found is mapped to -Xint */
		return RC_FAILED;
	}
#endif /* JAVA_SPEC_VERSION < 21 */
	if (registerCmdLineMapping(vm, MAPOPT_XDISABLEJAVADUMP, MAPOPT_XDUMP_JAVA_NONE, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	if (registerCmdLineMapping(vm, MAPOPT_XVERIFY_REMOTE, VMOPT_XVERIFY, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	if (registerCmdLineMapping(vm, MAPOPT_VERBOSE_XGCCON, MAPOPT_VERBOSE_GC, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	if (registerCmdLineMapping(vm, MAPOPT_VERBOSEGC, MAPOPT_VERBOSE_GC, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -Xloggc:<file> to -Xverbosegclog:<file> in release Java 829 and Java 9 */
	if (registerCmdLineMapping(vm, MAPOPT_XLOGGC, MAPOPT_XVERBOSEGCLOG, MAP_WITH_INCLUSIVE_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* -agentlib:healthcenter is aliased as -Xhealthcenter */
	if (registerCmdLineMapping(vm, MAPOPT_XHEALTHCENTER, VMOPT_AGENTLIB_HEALTHCENTER, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	if (registerCmdLineMapping(vm, MAPOPT_XHEALTHCENTER_COLON, VMOPT_AGENTLIB_HEALTHCENTER_EQUALS, MAP_WITH_INCLUSIVE_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Need to support -Xsoftrefthreshold (lowercase) for backwards compatibility */
	if (registerCmdLineMapping(vm, MAPOPT_XSOFTREFTHRESHOLD, VMOPT_XSOFTREFTHRESHOLD, STARTSWITH_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -Xshare:on to -Xshareclasses */
	if (registerCmdLineMapping(vm, MAPOPT_XSHARE_ON, VMOPT_XSHARECLASSES, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -Xshare:off to -Xshareclasses:utilities */
	if (registerCmdLineMapping(vm, MAPOPT_XSHARE_OFF, MAPOPT_XSHARECLASSES_UTILITIES, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -Xshare:auto to -Xshareclasses:nonfatal */
	if (registerCmdLineMapping(vm, MAPOPT_XSHARE_AUTO, MAPOPT_XSHARECLASSES_NONFATAL, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:+DisableExplicitGC to -Xdisableexplicitgc */
	if (registerCmdLineMapping(vm, MAPOPT_XXDISABLEEXPLICITGC, "-Xdisableexplicitgc", EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:+EnableExplicitGC to -Xenableexplicitgc */
	if (registerCmdLineMapping(vm, MAPOPT_XXENABLEEXPLICITGC, "-Xenableexplicitgc", EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:HeapDumpPath= to -Xdump:directory= */
	if (registerCmdLineMapping(vm, MAPOPT_XXHEAPDUMPPATH_EQUALS, VMOPT_XDUMP_DIRECTORY_EQUALS, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:MaxHeapSize= to -Xmx */
	if (registerCmdLineMapping(vm, MAPOPT_XXMAXHEAPSIZE_EQUALS, VMOPT_XMX, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:InitialHeapSize= to -Xms */
	if (registerCmdLineMapping(vm, MAPOPT_XXINITIALHEAPSIZE_EQUALS, VMOPT_XMS, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:OnOutOfMemoryError= to -Xdump:tool:events=systhrow,filter=java/lang/OutOfMemoryError,exec= */
	if (registerCmdLineMapping(vm, MAPOPT_XXONOUTOFMEMORYERROR_EQUALS, VMOPT_XDUMP_TOOL_OUTOFMEMORYERROR_EXEC_EQUALS, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:+ExitOnOutOfMemoryError to -Xdump:exit:events=systhrow,filter=java/lang/OutOfMemoryError */
	if (registerCmdLineMapping(vm, MAPOPT_XXENABLEEXITONOUTOFMEMORYERROR, VMOPT_XDUMP_EXIT_OUTOFMEMORYERROR, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:-ExitOnOutOfMemoryError to -Xdump:exit:none:events=systhrow,filter=java/lang/OutOfMemoryError */
	if (registerCmdLineMapping(vm, MAPOPT_XXDISABLEEXITONOUTOFMEMORYERROR, VMOPT_XDUMP_EXIT_OUTOFMEMORYERROR_DISABLE, EXACT_MAP_NO_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:ParallelCMSThreads=N to -Xconcurrentbackground */
	if (registerCmdLineMapping(vm, MAPOPT_XXPARALLELCMSTHREADS_EQUALS, VMOPT_XCONCURRENTBACKGROUND, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:ConcGCThreads=N  to -Xconcurrentbackground */
	if (registerCmdLineMapping(vm, MAPOPT_XXCONCGCTHREADS_EQUALS, VMOPT_XCONCURRENTBACKGROUND, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}
	/* Map -XX:ParallelGCThreads=N  to -XgcthreadsN */
	if (registerCmdLineMapping(vm, MAPOPT_XXPARALLELGCTHREADS_EQUALS, VMOPT_XGCTHREADS, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}

	/* Map -XX:ParallelMaxGCThreads=N  to -XgcmaxthreadsN */
	if (registerCmdLineMapping(vm, MAPOPT_XXPARALLELGCMAXTHREADS_EQUALS, VMOPT_XGCMAXTHREADS, EXACT_MAP_WITH_OPTIONS) == RC_FAILED) {
		return RC_FAILED;
	}

	return 0;
}
#endif /* J9VM_OPT_SIDECAR */


#ifdef JVMINIT_UNIT_TEST
static void
testFindArgs(J9JavaVM* vm)
{
	char* origOption0, *origOption1, *origOption2, *origOption3;
#ifdef J9VM_OPT_SIDECAR
	J9CmdLineMapping* origMapping1, *origMapping2, *origMapping3;
#endif
	IDATA intResult;
	char* failed = NULL;

	PORT_ACCESS_FROM_JAVAVM(vm);

	printf("\nTesting findArgInVMArgs...\n");

#define FAIL (failed = "Failed ")
#define PASS ("Passed ")
#define SET_TO(element, string) vm->vmArgsArray->actualVMArgs->options[element].optionString = string; printf("\nTesting: %s \t\t", string)
#define TEST_INT(value, expected) printf( (value==expected) ? PASS : FAIL )

#ifdef J9VM_OPT_SIDECAR
#define SET_MAP_TO(element, j9opt, sovopt, mapflags) registerCmdLineMapping(vm, sovopt, j9opt, mapflags)
#define REMOVE_MAPPING(element) j9mem_free_memory(vm->vmArgsArray->j9Options[element].mapping); vm->vmArgsArray->j9Options[element].mapping=NULL

	origMapping1 = vm->vmArgsArray->j9Options[1].mapping;
	if (origMapping1)
		REMOVE_MAPPING(1);
	origMapping2 = vm->vmArgsArray->j9Options[2].mapping;
	if (origMapping2)
		REMOVE_MAPPING(2);
	origMapping3 = vm->vmArgsArray->j9Options[3].mapping;
	if (origMapping3)
		REMOVE_MAPPING(3);
#endif

	origOption0 = vm->vmArgsArray->actualVMArgs->options[0].optionString;
	origOption1 = vm->vmArgsArray->actualVMArgs->options[1].optionString;
	origOption2 = vm->vmArgsArray->actualVMArgs->options[2].optionString;
	origOption3 = vm->vmArgsArray->actualVMArgs->options[3].optionString;

	SET_TO(1, "-Xfoo1");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo1", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo2sdf");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo2", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo2");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo2sdf", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo3:parse");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo3", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo4");
	SET_TO(2, "-Xfoo4");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo4", NULL);
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(EXACT_MATCH, "-Xfoo4", NULL);
	TEST_INT(intResult, 1);
	SET_TO(2, "");

	SET_TO(1, "-Xfoo1");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo1", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo2sdf");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo2", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo3:parse");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo3", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo4:wibble");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo4", "wibble");
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo5sdf:wobble");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo5", "wobble");
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo51sdf:ded,cat");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo5", "ded");
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo52sdf:ded,cat");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo5", "cat");
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo6");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo6sdf", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo:7");
	SET_TO(2, "-Xfoo:8");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", NULL);
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoo:", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo:ded");
	SET_TO(2, "-Xfoo:cat");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "ded");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "cat");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoo:", "ded");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoo:", "cat");
	TEST_INT(intResult, 2);

	SET_TO(1, "-Xfoo:ding,dong,merrily");
	SET_TO(2, "-Xfoo:fee,fie,fo,fum");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "dong");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "ding");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "merrily");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "fee");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "fie");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "fo");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoo:", "fum");
	TEST_INT(intResult, 2);
	SET_TO(2, "");

	SET_TO(0, "-Xfoob:zero");
	SET_TO(1, "-Xfoob:ding");
	SET_TO(2, "-Xfoob:dong");
	SET_TO(3, "-Xfoob:merrily");
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", NULL);
	TEST_INT(intResult, 3);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 2);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 1);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 0);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, -1);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", "merrily", 3);
	TEST_INT(intResult, -1);
	intResult = FIND_NEXT_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfoob:", "dong", 3);
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoob:", NULL);
	TEST_INT(intResult, 0);
	intResult = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 1);
	intResult = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 2);
	intResult = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, 3);
	intResult = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, "-Xfoob:", NULL, intResult);
	TEST_INT(intResult, -1);
	SET_TO(3, "");
	SET_TO(0, "");

	SET_TO(1, "-Xfoo32m");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoofoo32m");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo32m");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoofoo", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo32k");
	SET_TO(2, "-Xfoo64m");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo32k");
	SET_TO(2, "-Xfoo");
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(EXACT_MEMORY_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 1);
	SET_TO(2, "");

	SET_TO(1, "-Xfoo1");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo1", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo2sdf");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo2", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo3:parse");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo3", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo4:wibble");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo4", "wibble");
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo5sdf:wobble");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo5", "wobble");
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo6");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo6sdf", NULL);
	TEST_INT(intResult, -1);

	SET_TO(1, "-Xfoo:7");
	SET_TO(2, "-Xfoo:8");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS_FORWARD(OPTIONAL_LIST_MATCH, "-Xfoo", NULL);
	TEST_INT(intResult, 1);

	SET_TO(1, "-Xfoo:ded");
	SET_TO(2, "-Xfoo:cat");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "ded");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "cat");
	TEST_INT(intResult, 2);

	SET_TO(1, "-Xfoo:ding,dong,merrily");
	SET_TO(2, "-Xfoo:fee,fie,fo,fum");
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "dong");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "ding");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "merrily");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "fee");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "fie");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "fo");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfoo", "fum");
	TEST_INT(intResult, 2);
	SET_TO(2, "");

#ifdef J9VM_OPT_SIDECAR
	SET_TO(1, "-Xfoo1");
	SET_MAP_TO(1, "-Xfooj91", "-Xfoo1", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj91", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo2");
	SET_MAP_TO(1, "-Xfooj92", "-Xfoo2", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfoo2", NULL);
	TEST_INT(intResult, -1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo3");
	SET_MAP_TO(1, "-Xfooj93", "-Xfoo3", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj93sdf", NULL);
	TEST_INT(intResult, -1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo4");
	SET_MAP_TO(1, "-Xfooj94", "-Xfoo4", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj9", NULL);
	TEST_INT(intResult, -1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfooj91");
	SET_TO(2, "-Xfoo1");
	SET_MAP_TO(2, "-Xfooj91", "-Xfoo1", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj91", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfoo1");
	SET_TO(2, "-Xfooj91");
	SET_MAP_TO(1, "-Xfooj91", "-Xfoo1", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj91", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(1);
	SET_TO(2, "");

	SET_TO(1, "-Xfoo1:wibble");
	SET_MAP_TO(1, "-Xfooj92:wobble", "-Xfoo1:wibble", EXACT_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xfooj92", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo5");
	SET_MAP_TO(1, "-Xfooj9", "-Xfoo5", EXACT_MAP_WITH_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj9", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfooj91:wibble");
	SET_TO(2, "-Xfoo1:wobble");
	SET_MAP_TO(2, "-Xfooj91", "-Xfoo1", EXACT_MAP_WITH_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj91", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfooj92:wibble");
	SET_TO(2, "-Xfoo2:wobble");
	SET_MAP_TO(2, "-Xfooj92", "-Xfoo2", EXACT_MAP_WITH_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92", "wibble");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92", "wobble");
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfoosov1:ben");
	SET_MAP_TO(1, "-fooj91:bar=", "-Xfoosov1:", EXACT_MAP_WITH_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-fooj91", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Dfoo1.foo2=wibble");
	SET_MAP_TO(1, "-Xj9opt:optValue=", "-Dfoo1.foo2=", EXACT_MAP_WITH_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(OPTIONAL_LIST_MATCH, "-Xj9opt", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo8:arg");
	SET_MAP_TO(1, "-fooj98:bar=", "-Xfoo8:", MAP_WITH_INCLUSIVE_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-fooj98:", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo0=wibble");
	SET_MAP_TO(1, "-Xfooj90", "-Xfoo0", STARTSWITH_MAP_NO_OPTIONS);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MATCH, "-Xfooj90", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo6");
	SET_MAP_TO(1, "-Xfooj9", "-Xfoo6", MAP_TWO_COLONS_TO_ONE);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj9", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfooj91:wibble");
	SET_TO(2, "-Xfoo1:wobble:hello");
	SET_MAP_TO(2, "-Xfooj91", "-Xfoo1:wobble", MAP_TWO_COLONS_TO_ONE);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj91", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfooj92:wibble");
	SET_TO(2, "-Xfoo2:wobble:hello");
	SET_MAP_TO(2, "-Xfooj92", "-Xfoo2:wobble", MAP_TWO_COLONS_TO_ONE);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92", "wibble");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92", "hello");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92", "wobble");			/* this is not considered a value as we're mapping 2 colons to 1 */
	TEST_INT(intResult, -1);
	REMOVE_MAPPING(2);
	SET_TO(2, "");

	SET_TO(1, "-Xfooj91:wobble:hello");
	SET_TO(2, "-Xfoo1:wibble");
	SET_MAP_TO(2, "-Xfooj91:wobble", "-Xfoo1", MAP_ONE_COLON_TO_TWO);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj91:wobble", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfooj92:wobble:hello");
	SET_TO(2, "-Xfoo2:wibble");
	SET_MAP_TO(2, "-Xfooj92:wobble", "-Xfoo2", MAP_ONE_COLON_TO_TWO);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92:wobble", "wibble");
	TEST_INT(intResult, 2);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92:wobble", "hello");
	TEST_INT(intResult, 1);
	intResult = FIND_ARG_IN_VMARGS(STARTSWITH_MATCH, "-Xfooj92:wobble", "wobble");
	TEST_INT(intResult, -1);
	REMOVE_MAPPING(2);
	SET_TO(2, "");

	SET_TO(1, "-Xfoo32m");
	SET_MAP_TO(1, "-Xfooj9", "-Xfoo", MAP_MEMORY_OPTION);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfooj9", NULL);
	TEST_INT(intResult, 1);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfooj32m");
	SET_TO(2, "-Xfoosov64m");
	SET_MAP_TO(2, "-Xfooj", "-Xfoosov", MAP_MEMORY_OPTION);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfooj", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(2);

	SET_TO(1, "-Xfoosov64m");
	SET_TO(2, "-Xfooj32m");
	SET_MAP_TO(1, "-Xfooj", "-Xfoosov", MAP_MEMORY_OPTION);
	intResult = FIND_ARG_IN_VMARGS(EXACT_MEMORY_MATCH, "-Xfooj", NULL);
	TEST_INT(intResult, 2);
	REMOVE_MAPPING(1);
	SET_TO(2, "");

	vm->vmArgsArray->j9Options[1].mapping = origMapping1;
	vm->vmArgsArray->j9Options[2].mapping = origMapping2;
	vm->vmArgsArray->j9Options[3].mapping = origMapping3;
#endif

	vm->vmArgsArray->actualVMArgs->options[0].optionString = origOption0;
	vm->vmArgsArray->actualVMArgs->options[1].optionString = origOption1;
	vm->vmArgsArray->actualVMArgs->options[2].optionString = origOption2;
	vm->vmArgsArray->actualVMArgs->options[3].optionString = origOption3;

	printf((failed==NULL) ? "\n\nTESTS PASSED\n" : "\n\nTESTS FAILED\n");
}


static void
testOptionValueOps(J9JavaVM* vm)
{
	char* result;
	char* origOption1, *origOption2, *origOption3, *origOption4, *origOption5, *origOption6;
	char* optName;
#ifdef J9VM_OPT_SIDECAR
	J9CmdLineMapping* origMapping1, *origMapping2, *origMapping3, *origMapping4, *origMapping5, *origMapping6;
#endif
	char optionsBuffer[SMALL_STRING_BUF_SIZE];
	char* optionsBufferPtr = optionsBuffer;
	IDATA intResult;
	char* failed = NULL;
	char copyBuffer[SMALL_STRING_BUF_SIZE];
	char* copyBufferPtr = copyBuffer;
	IDATA copyBufSize = SMALL_STRING_BUF_SIZE;
	UDATA uResult = 0;
	IDATA returnVal;

	PORT_ACCESS_FROM_JAVAVM(vm);

	printf("\nTesting optionValueOperations...\n");

#define FAIL (failed = "Failed ")
#define PASS ("Passed ")
#define COMPARE(result, expected) printf((result==NULL) ? FAIL : ((strcmp(result, expected)==0) ? PASS : FAIL ))
#define IS_NULL(result) printf((result==NULL) ? PASS : FAIL)
#define IS_EMPTY_STRING(result) printf((result!=NULL && strlen(result)==0) ? PASS : FAIL)
#define IS_0(result) printf((result=='\0') ? PASS : FAIL)
#define SET_TO(element, string) vm->vmArgsArray->actualVMArgs->options[element].optionString = string; printf("\nTesting: %s \t\t", string)
#define NEXT_ELEMENT(array) (array += strlen(array) + 1)
#define TEST_INT(value, expected) printf( (value==expected) ? PASS : FAIL )

#ifdef J9VM_OPT_SIDECAR
#define SET_MAP_TO(element, j9opt, sovopt, mapflags) registerCmdLineMapping(vm, sovopt, j9opt, mapflags)
#define REMOVE_MAPPING(element) j9mem_free_memory(vm->vmArgsArray->j9Options[element].mapping); vm->vmArgsArray->j9Options[element].mapping=NULL

	origMapping1 = vm->vmArgsArray->j9Options[1].mapping;
	if (origMapping1)
		REMOVE_MAPPING(1);
	origMapping2 = vm->vmArgsArray->j9Options[2].mapping;
	if (origMapping2)
		REMOVE_MAPPING(2);
	origMapping3 = vm->vmArgsArray->j9Options[3].mapping;
	if (origMapping3)
		REMOVE_MAPPING(3);
	origMapping4 = vm->vmArgsArray->j9Options[4].mapping;
	if (origMapping4)
		REMOVE_MAPPING(4);
	origMapping5 = vm->vmArgsArray->j9Options[5].mapping;
	if (origMapping5)
		REMOVE_MAPPING(5);
	origMapping6 = vm->vmArgsArray->j9Options[6].mapping;
	if (origMapping6)
		REMOVE_MAPPING(6);
#endif
	origOption1 = vm->vmArgsArray->actualVMArgs->options[1].optionString;
	origOption2 = vm->vmArgsArray->actualVMArgs->options[2].optionString;
	origOption3 = vm->vmArgsArray->actualVMArgs->options[3].optionString;
	origOption4 = vm->vmArgsArray->actualVMArgs->options[4].optionString;
	origOption5 = vm->vmArgsArray->actualVMArgs->options[5].optionString;
	origOption6 = vm->vmArgsArray->actualVMArgs->options[6].optionString;

	SET_TO(1, "-Xfoo1");
	GET_OPTION_VALUE(1, ':', &result);
	IS_NULL(result);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	IS_EMPTY_STRING(copyBufferPtr);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
#ifdef J9VM_OPT_SIDECAR
	TEST_INT(returnVal, OPTION_ERROR);
#else
	TEST_INT(returnVal, OPTION_OK);			/* For non-sidecar, GET_COMPOUND is not supported, so it does same as COPY_OPTION */
#endif
	IS_EMPTY_STRING(copyBufferPtr);

	SET_TO(1, "-Xfoo2:bar1");
	GET_OPTION_VALUE(1, ':', &result);
	COMPARE(result, "bar1");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar1");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar1");

	SET_TO(1, "-Xfoob:123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");

	SET_TO(1, "-Xfoom:1234567890123456789012345678901234567890123456789012345678901234");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");

	SET_TO(1, "-Xfoo3=bar2");
	returnVal = GET_OPTION_VALUE(1, '=', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar2");

	SET_TO(1, "-Xfoo4:bar3:bar4");
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar3:bar4");

	SET_TO(1, "-Xfoo5=");
	returnVal = GET_OPTION_VALUE(1, '=', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "");

	SET_TO(1, "-Xfoo8:bar9,bar0");
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar9,bar0");

	SET_TO(1, "-Xfoo6:bar5:bar6");
	returnVal = GET_OPTION_OPTION(1, ':', ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar6");

	SET_TO(1, "-Xfoo7:bar7=bar8");
	returnVal = GET_OPTION_OPTION(1, ':', '=', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar8");

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo00:");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo0:bar0");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar0");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoom:123456789,123456789,123456789,123456789,123456789,123456789,12");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "123456789");
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "12");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoob:123456789,123456789,123456789,123456789,123456789,123456789,123");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(optionsBufferPtr, "123456789");
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "12");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo1:bar1,bar2");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar1");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar2");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo2:bar3,bar4,bar5");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar3");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar4");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar5");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo3:bar1=wibble,bar2");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar1=wibble");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar2");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo4:bar3,bar4=parse");
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar3");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar4=parse");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo5=bar1,bar2");
	returnVal = GET_OPTION_VALUES(1, '=', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar1");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar2");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo6=bar3,bar4,");
	returnVal = GET_OPTION_VALUES(1, '=', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar3");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "bar4");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);

#ifdef J9VM_OPT_SIDECAR
	SET_TO(1, "-Xfoosov0");
	SET_MAP_TO(1, "-Xfooj90", "-Xfoosov0", EXACT_MAP_NO_OPTIONS);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	IS_NULL(result);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	IS_EMPTY_STRING(copyBufferPtr);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_ERROR);
	IS_EMPTY_STRING(copyBufferPtr);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo:wibble");
	SET_MAP_TO(1, "-Xfooj9:wobble", "-Xfoo:wibble", EXACT_MAP_NO_OPTIONS);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "wobble");				/* With exact map, we don't want the sov option value, we actually want the j9 option value */
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "wobble");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "wobble");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoosov4=wibble");
	SET_MAP_TO(1, "-Xfooj94", "-Xfoosov4", STARTSWITH_MAP_NO_OPTIONS);
	returnVal = GET_OPTION_VALUE(1, '=', &result);
	TEST_INT(returnVal, OPTION_OK);
	IS_NULL(result);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, '=', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	IS_EMPTY_STRING(copyBufferPtr);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, '=', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_ERROR);
	IS_EMPTY_STRING(copyBufferPtr);
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoosov1:barsov");
	SET_MAP_TO(1, "-Xfooj91:", "-Xfoosov1:", EXACT_MAP_WITH_OPTIONS);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "barsov");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "barsov");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "barsov");
	REMOVE_MAPPING(1);

	vm->vmArgsArray->j9Options[1].fromEnvVar = NULL;
	SET_TO(1, "-Xj9opt:arg1");
	SET_TO(2, "-Xsovopt:arg2");
	SET_TO(3, "-nothing");
	SET_TO(4, "-Xj9opt:arg3,arg3a");
	SET_TO(5, "-Xsovopt:arg4");
	SET_TO(6, "-Xj9opt:arg5");
	SET_MAP_TO(2, "-Xj9opt:", "-Xsovopt:", EXACT_MAP_WITH_OPTIONS);
	SET_MAP_TO(5, "-Xj9opt:", "-Xsovopt:", EXACT_MAP_WITH_OPTIONS);

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(6, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg2,arg4,arg5");

	vm->vmArgsArray->j9Options[1].fromEnvVar = "anEnvVar";			/* Since this arg is now from an environment variable, it should be included */
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(6, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg1,arg2,arg4,arg5");

	vm->vmArgsArray->j9Options[1].fromEnvVar = NULL;
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(5, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg2,arg3,arg3a,arg4");

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(4, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg2,arg3,arg3a");

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(3, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_ERROR);				/* No : found in option */
	IS_EMPTY_STRING(copyBufferPtr);

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(2, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg1,arg2");

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg1");

	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUES(5, ':', ',', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg2");
	NEXT_ELEMENT(copyBufferPtr);
	COMPARE(copyBufferPtr, "arg3");
	NEXT_ELEMENT(copyBufferPtr);
	COMPARE(copyBufferPtr, "arg3a");
	NEXT_ELEMENT(copyBufferPtr);
	COMPARE(copyBufferPtr, "arg4");
	NEXT_ELEMENT(copyBufferPtr);
	IS_0(*copyBufferPtr);

	copyBufferPtr = copyBuffer;
	SET_TO(6, "-Xj9opt:12345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(6, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "arg2,arg4,12345678901234567890123456789012345678901234567890123");

	SET_TO(6, "-Xj9opt:123456789012345678901234567890123456789012345678901234");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(6, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "arg2,arg4,12345678901234567890123456789012345678901234567890123");
	REMOVE_MAPPING(2);
	REMOVE_MAPPING(5);
	SET_TO(2, "");
	SET_TO(3, "");
	SET_TO(4, "");
	SET_TO(5, "");
	SET_TO(6, "");

	SET_TO(1, "-Xfoosov2:bar1:bar2");
	SET_MAP_TO(1, "-Xfooj92:", "-Xfoosov2:bar1:", MAP_TWO_COLONS_TO_ONE);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar2");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar2");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar2");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoosovm:bar1:123456789012345678901234567890123456789012345678901234567890123");
	SET_MAP_TO(1, "-Xfooj9m:", "-Xfoosovm:bar1:", MAP_TWO_COLONS_TO_ONE);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoosovb:bar1:1234567890123456789012345678901234567890123456789012345678901234");
	SET_MAP_TO(1, "-Xfooj9b:", "-Xfoosovb:bar1:", MAP_TWO_COLONS_TO_ONE);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "1234567890123456789012345678901234567890123456789012345678901234");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "123456789012345678901234567890123456789012345678901234567890123");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoosov2:bar3");
	SET_MAP_TO(1, "-Xfooj92:parse:", "-Xfoosov2:", MAP_ONE_COLON_TO_TWO);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(result, "bar3");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar3");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar3");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoo8:arg");
	SET_MAP_TO(1, "-fooj98:bar=", "-Xfoo8:", MAP_WITH_INCLUSIVE_OPTIONS);
	returnVal = GET_OPTION_VALUE(1, ':', &result);
	TEST_INT(returnVal, OPTION_ERROR);					/* COPY_OPTION_VALUE should be used */
	IS_NULL(result);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=arg");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=arg");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoom:12345678901234567890123456789012345678901234567890123456789");
	SET_MAP_TO(1, "-fooj9m:bar=", "-Xfoom:", MAP_WITH_INCLUSIVE_OPTIONS);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=12345678901234567890123456789012345678901234567890123456789");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=12345678901234567890123456789012345678901234567890123456789");
	REMOVE_MAPPING(1);

	SET_TO(1, "-Xfoob:123456789012345678901234567890123456789012345678901234567890");
	SET_MAP_TO(1, "-fooj9m:bar=", "-Xfoob:", MAP_WITH_INCLUSIVE_OPTIONS);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "bar=12345678901234567890123456789012345678901234567890123456789");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_BUFFER_OVERFLOW);
	COMPARE(copyBufferPtr, "bar=12345678901234567890123456789012345678901234567890123456789");
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoo6:arg");
	SET_MAP_TO(1, "-fooj96:bar=", "-Xfoo6:", MAP_WITH_INCLUSIVE_OPTIONS);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bar=arg");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=arg");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bar=arg");
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Dfoo.bar=wibble");
	SET_MAP_TO(1, "-Xfooj99:bling:bar=", "-Dfoo.bar=", MAP_WITH_INCLUSIVE_OPTIONS);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "bling:bar=wibble");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = COPY_OPTION_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bling:bar=wibble");
	memset(copyBufferPtr, 0, copyBufSize);
	returnVal = GET_COMPOUND_VALUE(1, ':', &copyBufferPtr, copyBufSize);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(copyBufferPtr, "bling:bar=wibble");
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoosov3:barsov");
	SET_MAP_TO(1, "-Xfooj93:", "-Xfoosov3:", EXACT_MAP_WITH_OPTIONS);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "barsov");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoosov4:barsov1,barsov2");
	SET_MAP_TO(1, "-Xfooj94:", "-Xfoosov4:", EXACT_MAP_WITH_OPTIONS);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "barsov1");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "barsov2");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoosov5:hello:barsov3,barsov4");
	SET_MAP_TO(1, "-Xfooj95:", "-Xfoosov5:hello:", MAP_TWO_COLONS_TO_ONE);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "barsov3");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "barsov4");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	REMOVE_MAPPING(1);

	optionsBufferPtr = optionsBuffer;
	SET_TO(1, "-Xfoosov6:barsov5,barsov6");
	SET_MAP_TO(1, "-Xfooj96:hello:", "-Xfoosov6:", MAP_ONE_COLON_TO_TWO);
	returnVal = GET_OPTION_VALUES(1, ':', ',', &optionsBufferPtr, SMALL_STRING_BUF_SIZE);
	TEST_INT(returnVal, OPTION_OK);
	COMPARE(optionsBufferPtr, "barsov5");
	NEXT_ELEMENT(optionsBufferPtr);
	COMPARE(optionsBufferPtr, "barsov6");
	NEXT_ELEMENT(optionsBufferPtr);
	IS_0(*optionsBufferPtr);
	REMOVE_MAPPING(1);
#endif

	SET_TO(1, "-Xfoi5");
	optName = "-Xfoi";
	intResult = GET_INTEGER_VALUE(1, optName, uResult);
	TEST_INT(uResult, 5);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoj5k");
	optName = "-Xfoj";
	intResult = GET_INTEGER_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfok5347534875438758474");
	optName = "-Xfok";
	intResult = GET_INTEGER_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfoo31");
	optName = "-Xfoo";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 32);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoa34587348574875938475983745");
	optName = "-Xfoa";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfob4294967295");			/* (2^32 - 1) */
	optName = "-Xfob";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoc4294967280");			/* 0xfffffff0 */
	optName = "-Xfoc";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0xfffffff0);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfop64k");
	optName = "-Xfop";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (64*1024));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoq65K");
	optName = "-Xfoq";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (65*1024));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfor66m");
	optName = "-Xfor";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (66*1024*1024));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfos67M");
	optName = "-Xfos";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (67*1024*1024));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoa1G");
	optName = "-Xfoa";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (1024*1024*1024));
	TEST_INT(intResult, OPTION_OK);

#if defined(J9VM_ENV_DATA64)
	SET_TO(1, "-Xfog16777215T");			/* (2^24-1)*2^40 */
	optName = "-Xfog";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (16777215UL*1024UL*1024UL*1024UL*1024UL));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoh1t");
	optName = "-Xfoh";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (1024*1024*1024*1024L));
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoi16777216T");			/* 2^64 */
	optName = "-Xfoi";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoj16777216t");			/* 2^64 */
	optName = "-Xfoj";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(intResult, OPTION_OVERFLOW);
#else	/* defined(J9VM_ENV_DATA64) */
	SET_TO(1, "-Xfog1T");
	optName = "-Xfog";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoh1t");
	optName = "-Xfoh";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoi0T");
	optName = "-Xfoi";
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoj0t");
	optName = "-Xfoj";
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OK);
#endif /* defined(J9VM_ENV_DATA64) */

	SET_TO(1, "-Xfoz0.1");
	optName = "-Xfoz";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfob0.1");
	optName = "-Xfob";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 10);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfox1");
	optName = "-Xfox";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 100);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfob1.5");
	optName = "-Xfob";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoc0.76");
	optName = "-Xfoc";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 76);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfod0.435236");
	optName = "-Xfod";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 43);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoe1.0");
	optName = "-Xfoe";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 100);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfof2.0");
	optName = "-Xfof";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfof32");
	optName = "-Xfof";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoba0,1");
	optName = "-Xfoba";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 10);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoba1,5");
	optName = "-Xfoba";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfoca0,76");
	optName = "-Xfoca";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 76);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoda0,435236");
	optName = "-Xfoda";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 43);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfoea1,0");
	optName = "-Xfoea";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 100);
	TEST_INT(intResult, OPTION_OK);

	SET_TO(1, "-Xfofa2,0");
	optName = "-Xfofa";
	intResult = GET_PERCENT_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

	SET_TO(1, "-Xfot68w");
	optName = "-Xfot";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfou99999999999M");
	optName = "-Xfou";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_MALFORMED);

	SET_TO(1, "-Xfow99999M");
	optName = "-Xfow";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, 0);
	TEST_INT(intResult, OPTION_OVERFLOW);

#ifdef J9VM_OPT_SIDECAR

	SET_TO(1, "-Xfoo32k");
	SET_MAP_TO(1, "-Xfooj9", "-Xfoo", MAP_MEMORY_OPTION);
	optName = "-Xfooj9";
	intResult = GET_MEMORY_VALUE(1, optName, uResult);
	TEST_INT(uResult, (32*1024));
	TEST_INT(intResult, OPTION_OK);
	REMOVE_MAPPING(1);

	vm->vmArgsArray->j9Options[1].mapping = origMapping1;
	vm->vmArgsArray->j9Options[2].mapping = origMapping2;
	vm->vmArgsArray->j9Options[3].mapping = origMapping3;
	vm->vmArgsArray->j9Options[4].mapping = origMapping4;
	vm->vmArgsArray->j9Options[5].mapping = origMapping5;
	vm->vmArgsArray->j9Options[6].mapping = origMapping6;

#endif

	vm->vmArgsArray->actualVMArgs->options[1].optionString = origOption1;
	vm->vmArgsArray->actualVMArgs->options[2].optionString = origOption2;
	vm->vmArgsArray->actualVMArgs->options[3].optionString = origOption3;
	vm->vmArgsArray->actualVMArgs->options[4].optionString = origOption4;
	vm->vmArgsArray->actualVMArgs->options[5].optionString = origOption5;
	vm->vmArgsArray->actualVMArgs->options[6].optionString = origOption6;

	printf((failed==NULL) ? "\n\nTESTS PASSED\n" : "\n\nTESTS FAILED\n");
}


#endif


#if defined(J9VM_OPT_SIDECAR) && (JAVA_SPEC_VERSION < 21)
/* Whine about -Djava.compiler if the option is not used correctly */

static IDATA
checkDjavacompiler(J9PortLibrary *portLibrary, J9VMInitArgs* j9vm_args)
{
	IDATA index = 0;
	char* jitValue = NULL;
	PORT_ACCESS_FROM_PORT(portLibrary);

	index = findArgInVMArgs( PORTLIB,  j9vm_args, STARTSWITH_MATCH, SYSPROP_DJAVA_COMPILER_EQUALS, NULL, FALSE);
	if (index >= 0) {
		if (optionValueOperations(PORTLIB, j9vm_args, index, GET_OPTION, &jitValue, 0, '=', 0, NULL) != OPTION_OK) {
			return RC_FAILED;
		}
		if (jitValue && strcmp(jitValue, OPT_NONE)!=0 &&
								strcmp(jitValue, OPT_NONE_CAPS)!=0 &&
								strcmp(jitValue, "")!=0 &&
								strcmp(jitValue, J9_JIT_DLL_NAME)!=0 &&
								strcmp(jitValue, DJCOPT_JITC)!=0) {
			j9nls_printf(portLibrary, J9NLS_WARNING, J9NLS_VM_UNRECOGNISED_JIT_COMPILER, jitValue);
		}
	}
	return 0;
}
#endif /* defined(J9VM_OPT_SIDECAR) && (JAVA_SPEC_VERSION < 21) */


static IDATA
setMemoryOptionToOptElse(J9JavaVM* vm, UDATA* thingToSet, char* optionName, UDATA defaultValue, UDATA doConsumeArg)
{
	IDATA index, status;
	UDATA newValue;
	PORT_ACCESS_FROM_JAVAVM(vm);
	if ((index = findArgInVMArgs( PORTLIB, vm->vmArgsArray, EXACT_MEMORY_MATCH, optionName, NULL, doConsumeArg)) >= 0) {
		status = optionValueOperations(PORTLIB, vm->vmArgsArray, index, GET_MEM_VALUE, &optionName, 0, 0, 0, &newValue);
		if (status == OPTION_OK) {
			*thingToSet = newValue;
		} else {
			return status;
		}
	} else {
		*thingToSet = defaultValue;
	}
	return OPTION_OK;
}

static IDATA
setIntegerValueOptionToOptElse(J9JavaVM* vm, UDATA* thingToSet, char* optionName, UDATA defaultValue, UDATA doConsumeArg)
{
	IDATA index, status;
	UDATA newValue;
	PORT_ACCESS_FROM_JAVAVM(vm);
	if ((index = findArgInVMArgs( PORTLIB, vm->vmArgsArray, EXACT_MEMORY_MATCH, optionName, NULL, doConsumeArg)) >= 0) {
		status = optionValueOperations(PORTLIB, vm->vmArgsArray, index, GET_INT_VALUE, &optionName, 0, 0, 0, &newValue);
		if (status == OPTION_OK) {
			*thingToSet = newValue;
		} else {
			return status;
		}
	} else {
		*thingToSet = defaultValue;
	}
	return OPTION_OK;
}



#if (defined(J9VM_OPT_SIDECAR))
static IDATA
createMapping(J9JavaVM* vm, char* j9Name, char* mapName, UDATA flags, IDATA atIndex)
{
	J9CmdLineMapping* newMapping;
	IDATA j9NameLen = (j9Name) ? (strlen(j9Name) + 1) : 2;
	IDATA mapNameLen = (mapName) ? (strlen(mapName) + 1) : 2;
	IDATA cmdLineMapSize = sizeof(struct J9CmdLineMapping);
	IDATA totalSize = cmdLineMapSize + j9NameLen + mapNameLen;

	PORT_ACCESS_FROM_JAVAVM(vm);

	JVMINIT_VERBOSE_INIT_VM_TRACE2(vm, "Creating command-line mapping from %s to %s\n", mapName, j9Name);

	newMapping = j9mem_allocate_memory(totalSize, OMRMEM_CATEGORY_VM);
	if (newMapping) {
		memset(newMapping, 0, totalSize);
		newMapping->j9Name = (char*)(((UDATA)newMapping) + cmdLineMapSize);
		newMapping->mapName = newMapping->j9Name + j9NameLen;
		strncpy(newMapping->j9Name, ((j9Name) ? j9Name : ""), j9NameLen);
		strncpy(newMapping->mapName, ((mapName) ? mapName : ""), mapNameLen);
		newMapping->flags = flags;
		vm->vmArgsArray->j9Options[atIndex].mapping = newMapping;
	} else {
		return RC_FAILED;
	}
	return 0;
}
#endif /* J9VM_OPT_SIDECAR */

static void
generateMemoryOptionParseError(J9JavaVM* vm, J9VMDllLoadInfo* loadInfo, UDATA errorType, char* optionWithError)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	char *errorBuffer = (char *)j9mem_allocate_memory(LARGE_STRING_BUF_SIZE, OMRMEM_CATEGORY_VM);
	if (NULL != errorBuffer) {
		strcpy(errorBuffer, "Parse error for ");
		safeCat(errorBuffer, optionWithError, LARGE_STRING_BUF_SIZE);
		if (OPTION_MALFORMED == errorType) {
			safeCat(errorBuffer, " - option malformed.", LARGE_STRING_BUF_SIZE);
		} else if (OPTION_OVERFLOW == errorType) {
			safeCat(errorBuffer, " - option overflow.", LARGE_STRING_BUF_SIZE);
		} else if (OPTION_OUTOFRANGE == errorType) {
			safeCat(errorBuffer, " - value out of range.", LARGE_STRING_BUF_SIZE);
		}
		setErrorJ9dll(PORTLIB, loadInfo, errorBuffer, TRUE);
	} else {
		setErrorJ9dll(PORTLIB, loadInfo, "Cannot allocate memory for error message", FALSE);
	}
}






/* This function is a public entry point for calling exit code of loaded DLLs */

void
runExitStages(J9JavaVM* vm, J9VMThread* vmThread)
{
#ifdef J9VM_INTERP_NATIVE_SUPPORT
	if (vm->jitConfig) {
		if (vm->jitConfig->jitExclusiveVMShutdownPending) {
			vm->jitConfig->jitExclusiveVMShutdownPending(vmThread);
		}
	}
#endif
#if JAVA_SPEC_VERSION >= 19
	if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_ENABLE_CONTINUATION_CACHE_SUMMARY)
	&& (0 < (vm->t1CacheHit + vm->t2CacheHit + vm->cacheMiss))
	) {
		PORT_ACCESS_FROM_JAVAVM(vm);
		j9tty_printf(PORTLIB, "\nContinuation Cache Summary:");
		j9tty_printf(PORTLIB, "\n    T1 size: %u    T2 size: %u\n", vm->continuationT1Size, vm->continuationT2Size);
		j9tty_printf(PORTLIB, "\nTotal Continuation entries: %u\n", vm->t1CacheHit + vm->t2CacheHit + vm->cacheMiss);
		j9tty_printf(PORTLIB, "\nCache Hits:                 %u", vm->t1CacheHit + vm->t2CacheHit);
		j9tty_printf(PORTLIB, "\n     T1 Cache Hits:             %u", vm->t1CacheHit);
		j9tty_printf(PORTLIB, "\n     T2 Cache Hits:             %u", vm->t2CacheHit);
		j9tty_printf(PORTLIB, "\nCache Miss:                 %u", vm->cacheMiss);
#if defined(J9VM_PROF_CONTINUATION_ALLOCATION)
		j9tty_printf(PORTLIB, "\n     Fast Alloc (<10000ns):     %u", vm->fastAlloc);
		j9tty_printf(PORTLIB, "\n     Avg Fast Alloc Time:       %lld ns", (vm->fastAlloc > 0 ? (vm->fastAllocAvgTime / (I_64)vm->fastAlloc) : 0));
		j9tty_printf(PORTLIB, "\n     Slow Alloc (>10000ns):     %u", vm->slowAlloc);
		j9tty_printf(PORTLIB, "\n     Avg Slow Alloc Time:       %lld ns", (vm->slowAlloc > 0 ? (vm->slowAllocAvgTime / (I_64)vm->slowAlloc) : 0));
		j9tty_printf(PORTLIB, "\nAvg Cache Lookup Time:      %lld ns", (vm->avgCacheLookupTime / (I_64)(vm->t1CacheHit + vm->t2CacheHit + vm->fastAlloc + vm->slowAlloc)));
#endif /* defined(J9VM_PROF_CONTINUATION_ALLOCATION) */
		j9tty_printf(PORTLIB, "\n\nCache store:                %u", vm->t1CacheHit + vm->t2CacheHit + vm->cacheMiss - vm->cacheFree);
		j9tty_printf(PORTLIB, "\n     T1 Cache store:            %u", vm->t1CacheHit + vm->t2CacheHit + vm->cacheMiss - vm->cacheFree - vm->t2store);
		j9tty_printf(PORTLIB, "\n     T2 Cache store:            %u", vm->t2store);
		j9tty_printf(PORTLIB, "\nCache Freed:                %u\n", vm->cacheFree);
		j9tty_printf(PORTLIB, "\nAvg Cache Stack Size:       %.2f KB\n", (double)vm->totalContinuationStackSize / (vm->t1CacheHit + vm->t2CacheHit + vm->cacheMiss) / 1024);
	}
#endif /* JAVA_SPEC_VERSION >= 19 */

	/* Unload before trace engine exits */
	UT_MODULE_UNLOADED(J9_UTINTERFACE_FROM_VM(vm));

	if (vm->dllLoadTable) {
		runShutdownStage(vm, JVM_EXIT_STAGE, NULL, 0);
	}
}


/**
 * PostInit load is used to load or initialize a library after initialization has completed.
 * It is not currently protected by a mutex.
 * The library must be tagged with ALLOW_POST_INIT_LOAD in initializeDllLoadTable to allow this.
 * postInitLoadJ9DLL can be called on a library even if it is already loaded.
 * It will run the J9VMDLLMain with a special POST_INIT_STAGE stage.
 * It will pass argData into J9VMDllMain using the "reserved" param.
 * Returns JNI_OK if all is well.
 */
IDATA postInitLoadJ9DLL(J9JavaVM* vm, const char* dllName, void* argData)
{
	RunDllMainData userRunData;
	CheckPostStageData userCheckData;
	J9VMDllLoadInfo* entry = NULL;

	entry = findDllLoadInfo(vm->dllLoadTable, dllName);

	if (!entry) {
		return POSTINIT_LIBRARY_NOT_FOUND;
	}

	if (!(entry->loadFlags & ALLOW_POST_INIT_LOAD)) {
		return POSTINIT_NOT_PERMITTED;
	}

	if (!(entry->loadFlags & LOADED)) {
		if (!loadJ9DLL(vm, entry)) {
			return POSTINIT_LOAD_FAILED;
		}
	}

	userRunData.vm = vm;
	userRunData.stage = POST_INIT_STAGE;
	userRunData.reserved = argData;
	userRunData.filterFlags = 0;

	/* Args are passed into J9VMDllMain through reserved parameter */
	runJ9VMDllMain(entry, &userRunData);

	userCheckData.vm = vm;
	userCheckData.stage = POST_INIT_STAGE;
	userCheckData.success = JNI_OK;

	checkDllInfo(entry, &userCheckData);

	return userCheckData.success;
}

#if defined(WIN32)
static void
findLargestUnallocatedRegion(UDATA *regionStart, UDATA *regionSize)
{
	UDATA bestRegionStart = 0;
	UDATA bestRegionSize = 0;
	UDATA base = 0;
	MEMORY_BASIC_INFORMATION memInfo;

	memset(&memInfo, 0, sizeof(memInfo));
	while (0 != VirtualQuery((LPCVOID)base, &memInfo, sizeof(memInfo))) {
		if (MEM_FREE == memInfo.State) {
			UDATA regionSize = memInfo.RegionSize;

			if (regionSize > bestRegionSize) {
				bestRegionStart = base;
				bestRegionSize = regionSize;
			}
		}

		base += memInfo.RegionSize;
	}
	*regionSize = bestRegionSize;
	*regionStart = bestRegionStart;
}

static IDATA
preloadUser32Dll(J9JavaVM *vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	IDATA preloadUser32ArgIndex;
	IDATA noPreloadUser32ArgIndex;
	IDATA protectContiguousArgIndex;
	IDATA noProtectContiguousArgIndex;
	BOOLEAN preloadUser32 = FALSE;

	/* Check for these options right away, so that we always consume them. */
	preloadUser32ArgIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XPRELOADUSER32, NULL);
	noPreloadUser32ArgIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XNOPRELOADUSER32, NULL);
	protectContiguousArgIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XPROTECTCONTIGUOUS, NULL);
	noProtectContiguousArgIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XNOPROTECTCONTIGUOUS, NULL);

	/*
	 * By default, we only preload user32.dll on pre-Windows Vista systems. We do this because
	 * on pre-Windows Vista systems, user32.dll be loaded at its reserved address - so we must
	 * load it early so that no other library or memory allocation can take its spot.
	 *
	 * In addition to the above default, the user may explicitly tell us to either preload
	 * user32.dll or not, using the -Xpreloaduser32 and -Xnopreloaduser32 command line options,
	 * which will override the default behaviour.
	 */
	if (preloadUser32ArgIndex >= 0) {
		/* If "preload" was specified, preload unless there was a "no preload" option following it. */
		preloadUser32 = (preloadUser32ArgIndex > noPreloadUser32ArgIndex);
	}

	if (preloadUser32) {
		BOOLEAN protectContiguous = FALSE;
		UDATA contiguousRegionStart = 0;
		UDATA contiguousRegionSize;

		/*
		 * The -Xprotectcontiguous and -Xnoprotectcontiguous options specify whether
		 * we should protect the largest contiguous free memory region while loading
		 * user32.dll.
		 *
		 * This is done to avoid fragmenting the address space (which would limit the
		 * maximum size of the Java heap), if user32.dll will cause other DLLs to be
		 * loaded via the LoadAppInit_DLLs mechanism. By default, we don't do this.
		 */
		if (protectContiguousArgIndex >= 0) {
			protectContiguous = (protectContiguousArgIndex > noProtectContiguousArgIndex);
		}

		if (protectContiguous) {
			/* Find the largest unallocated memory region to reserve. */
			findLargestUnallocatedRegion(&contiguousRegionStart, &contiguousRegionSize);
			if (0 != contiguousRegionStart) {
				/* Align up contiguousRegionStart to be on a 65k boundary for VirtualAlloc(). */
				const UDATA alignment = 0x10000;
				const UDATA alignmentBits = (alignment - 1);
				UDATA adjustedRegionStart = (contiguousRegionStart + alignmentBits) & ~alignmentBits;

				contiguousRegionSize -= (adjustedRegionStart - contiguousRegionStart);
				contiguousRegionSize &= ~alignmentBits;
				contiguousRegionStart = adjustedRegionStart;
			}
		}

		if (0 != contiguousRegionStart) {
			contiguousRegionStart = (UDATA)VirtualAlloc((LPVOID)contiguousRegionStart, contiguousRegionSize, MEM_RESERVE, PAGE_NOACCESS);
		}
		LoadLibrary(USER32_DLL);
		if (0 != contiguousRegionStart) {
			VirtualFree((LPVOID)contiguousRegionStart, 0, MEM_RELEASE);
		}
	}

	return 0;
}
#endif /* defined(WIN32) */

/**
 * Process the commandline options and enable or disable String compression accordingly.
 *
 * @param vm pointer to the vm struct
 */
static void
processCompressionOptions(J9JavaVM *vm)
{
	IDATA argIndex1;
	IDATA argIndex2;

	/* The last (right-most) argument gets precedence */
	argIndex1 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCOMPACTSTRINGS, NULL);
	argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOCOMPACTSTRINGS, NULL);

#if JAVA_SPEC_VERSION >= 11
	/* Default setting */
	vm->strCompEnabled = TRUE;

	// Default to enable string compression unless there's the option to disable string compression
	// is explicitly defined.
	if (argIndex2 > argIndex1) {
		vm->strCompEnabled = FALSE;
	}
#else /* JAVA_SPEC_VERSION >= 11 */
	/* Default setting */
	vm->strCompEnabled = FALSE;

	if (argIndex1 > argIndex2) {
		vm->strCompEnabled = TRUE;
	}
#endif /* JAVA_SPEC_VERSION >= 11 */
}

/**
 * Returns the original *source.
 * Find the next delimiter and replace it with NULL. Set *source to the character
 * after the delimiter. If no delimiter is found, set *source to NULL.
 */
static char *
strUpToDelimiter(char **source, char delimiter)
{
	char ch = '\0';
	char *result = *source;
	while ('\0' != (ch = **source)) {
		if (ch == delimiter) {
			**source = '\0';
			*source += 1;
			return result;
		}
		*source += 1;
	}
	*source = NULL;
	return result;
}

/**
 * For compatibility when -XX:-LegacyXlogOption is not set, parse -Xlog:gc options and add mappings.
 * Anything besides -Xlog:gc, -Xlog:gc:stderr|<file>|file=<file>[:[:]] gives an unrecognized warning,
 * although any superset such as -Xlog, -Xlog:all, -Xlog:something,gc, etc. enables gc logging as well.
 */
static IDATA
parseXlogForCompatibility(J9JavaVM *vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	IDATA rc = JNI_OK;
	J9VMInitArgs *j9vm_args = vm->vmArgsArray;
	IDATA xlogindex = FIND_ARG_IN_VMARGS_FORWARD(OPTIONAL_LIST_MATCH, MAPOPT_XLOG_OPT, NULL);
	while (xlogindex >= 0) {
		BOOLEAN unrecognizedOption = FALSE;
		BOOLEAN setgclog = FALSE;
		BOOLEAN gclog = FALSE;
		char *gclogfile = NULL;
		char xlogoptionsbuf[LARGE_STRING_BUF_SIZE];
		char *xlogoptions = xlogoptionsbuf;
		/* Include room for the trailing NULL */
		UDATA optionLen = strlen(j9vm_args->actualVMArgs->options[xlogindex].optionString) + 1;

		if (optionLen > sizeof(xlogoptionsbuf)) {
			xlogoptions = j9mem_allocate_memory(optionLen, OMRMEM_CATEGORY_VM);
			if (NULL == xlogoptions) {
				rc = JNI_ERR;
				goto xlogret;
			}
		}

		if (OPTION_OK != COPY_OPTION_VALUE(xlogindex, ':', &xlogoptions, optionLen)) {
			rc = JNI_ERR;
			goto xlogret;
		}

		if (NULL == xlogoptions) {
			/* Reached if just -Xlog is specified */
			setgclog = TRUE;
			gclog = TRUE;
			unrecognizedOption = TRUE;
		} else {
			/* -Xlog:gc+other=off,a+b=c:file=abc:dec:options */
			char *fullOptions = xlogoptions;
			char *upToColon = strUpToDelimiter(&fullOptions, ':');
#if defined(DEBUG_XLOG)
			j9tty_printf(PORTLIB, "upToColon %s fullOptions %s\n", upToColon, fullOptions);
#endif /* defined(DEBUG_XLOG) */
			if ('\0' == *upToColon) {
				/* Nothing specified means all are enabled */
				setgclog = TRUE;
				gclog = TRUE;
				unrecognizedOption = TRUE;
			} else {
				char *upToComma = strUpToDelimiter(&upToColon, ',');
				while (TRUE) {
					BOOLEAN checkTagLevel = FALSE;
					char *upToEquals = NULL;
#if defined(DEBUG_XLOG)
					j9tty_printf(PORTLIB, "upToComma %s upToColon %s\n", upToComma, upToColon);
#endif /* defined(DEBUG_XLOG) */
					if ('\0' == *upToComma) {
						/* Missing tag */
xlogerr:
						CONSUME_ARG(j9vm_args, xlogindex);
						j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_OPTION_MALFORMED, j9vm_args->actualVMArgs->options[xlogindex].optionString);
						rc = JNI_ERR;
						goto xlogret;
					}
					upToEquals = strUpToDelimiter(&upToComma, '=');
#if defined(DEBUG_XLOG)
					j9tty_printf(PORTLIB, "upToEquals %s upToComma %s\n", upToEquals, upToComma);
#endif /* defined(DEBUG_XLOG) */
					if ('\0' == *upToEquals) {
						/* Missing tag */
						goto xlogerr;
					} else {
						UDATA numtags = 0;
						const char *tag = strUpToDelimiter(&upToEquals, '+');
						while (TRUE) {
#if defined(DEBUG_XLOG)
							j9tty_printf(PORTLIB, "tag %s upToEquals %s\n", tag, upToEquals);
#endif /* defined(DEBUG_XLOG) */
							if ('\0' == *tag) {
								/* Missing tag */
								goto xlogerr;
							}
							if (0 == j9_cmdla_stricmp("all", tag)) {
								if ((numtags > 0) || (NULL != upToEquals)) {
									/* Invalid use of keyword all */
									goto xlogerr;
								} else {
									checkTagLevel = TRUE;
									/* only gc is recognized from all */
									unrecognizedOption = TRUE;
								}
							} else if (0 == j9_cmdla_stricmp("disable", tag)) {
								if ((numtags > 0)
									|| (NULL != upToEquals)
									|| (NULL != upToComma)
									|| (NULL != upToColon)
									|| (NULL != fullOptions)
								) {
									/* Invalid use of keyword disable */
									goto xlogerr;
								} else {
									setgclog = TRUE;
									gclog = FALSE;
								}
							} else if ((0 == j9_cmdla_stricmp("gc", tag)) || (0 == j9_cmdla_stricmp("gc*", tag))) {
								checkTagLevel = TRUE;
							} else {
								unrecognizedOption = TRUE;
							}
							numtags += 1;
							if (NULL == upToEquals) {
								break;
							}
							tag = strUpToDelimiter(&upToEquals, '+');
						}
					}
					if (checkTagLevel) {
						/* look for a level after the '=' */
						setgclog = TRUE;
						if (NULL == upToComma) {
							gclog = TRUE;
						} else {
							if ('\0' == *upToComma) {
								goto xlogerr;
							}
							/* only levels trace, debug, info, and not off, warning, error, enable the gc verbose log */
							if (0 == j9_cmdla_stricmp("off", upToComma)) {
								gclog = FALSE;
							} else if ((0 == j9_cmdla_stricmp("warning", upToComma))
								|| (0 == j9_cmdla_stricmp("error", upToComma))
							) {
								gclog = FALSE;
								/* OpenJ9 doesn't understand log levels */
								unrecognizedOption = TRUE;
							} else if ((0 == j9_cmdla_stricmp("trace", upToComma))
								|| (0 == j9_cmdla_stricmp("debug", upToComma))
								|| (0 == j9_cmdla_stricmp("info", upToComma))
							) {
								gclog = TRUE;
								/* OpenJ9 doesn't understand log levels */
								unrecognizedOption = TRUE;
							} else {
								/* unrecognized log level */
								unrecognizedOption = TRUE;
							}
						}
					}
					if (NULL == upToColon) {
						break;
					}
					upToComma = strUpToDelimiter(&upToColon, ',');
				}
			}
			if (NULL != fullOptions) {
				/* look at the output method */
				char *output = NULL;
				upToColon = strUpToDelimiter(&fullOptions, ':');
#if defined(DEBUG_XLOG)
				j9tty_printf(PORTLIB, "upToColon %s fullOptions %s\n", upToColon, fullOptions);
#endif /* defined(DEBUG_XLOG) */
				output = strUpToDelimiter(&upToColon, '=');
#if defined(DEBUG_XLOG)
				j9tty_printf(PORTLIB, "output %s upToColon %s\n", output, upToColon);
#endif /* defined(DEBUG_XLOG) */
				/* stderr | stdout | file=<filename> | <filename> */
				if (0 == j9_cmdla_stricmp("stderr", output)) {
					if (NULL != upToColon) {
						goto xlogerr;
					}
				} else if (0 == j9_cmdla_stricmp("stdout", output)) {
					if (NULL != upToColon) {
						goto xlogerr;
					}
					/* Can't log to stdout yet */
					unrecognizedOption = TRUE;
				} else if (NULL == upToColon) {
					if ('\0' != *output) {
						/* Without `=` anything non-empty is a filename */
						gclogfile = output;
					}
				} else if (0 == j9_cmdla_stricmp("file", output)) {
					if (NULL == upToColon) {
						/* `file=` with no filename specified */
						goto xlogerr;
					}
					gclogfile = upToColon;
				} else {
					/* Anything else such as `<something>=<something>` is an error */
					goto xlogerr;
				}

				if (NULL != fullOptions) {
					/* Anything besides trailing ':'s is unrecognized */
					if (('\0' != *fullOptions) && (':' != *fullOptions)) {
						unrecognizedOption = TRUE;
					} else {
						strUpToDelimiter(&fullOptions, ':');
						if ((NULL != fullOptions) && ('\0' != *fullOptions)) {
							unrecognizedOption = TRUE;
						}
					}
				}
			}
		}
#if defined(DEBUG_XLOG)
		j9tty_printf(PORTLIB, "setgclog %d gclog %d file %s\n", setgclog, gclog, gclogfile);
#endif /* defined(DEBUG_XLOG) */
		if (setgclog && gclog) {
			if (NULL != gclogfile) {
				IDATA mapRc = 0;
				char *buf = NULL;
				char timeBuf[20];
				UDATA sizeRequired = 0;
				struct J9StringTokens *stringTokens = j9str_create_tokens(j9time_current_time_millis());
				if (NULL == stringTokens) {
					rc = JNI_ERR;
					goto xlogret;
				}
				j9str_subst_tokens(timeBuf, sizeof(timeBuf), "%Y-%m-%d_%H-%M-%S", stringTokens);

				if ((0 != j9str_set_token(stringTokens, "p", "%lld", j9sysinfo_get_pid()))
				||  (0 != j9str_set_token(stringTokens, "t", "%s", timeBuf))
				) {
					j9str_free_tokens(stringTokens);
					rc = JNI_ERR;
					goto xlogret;
				}
				sizeRequired = j9str_subst_tokens(NULL, 0, gclogfile, stringTokens);
				/* sizeRequired already includes space for the NULL */
				buf = j9mem_allocate_memory(sizeof(MAPOPT_XVERBOSEGCLOG) - 1 + sizeRequired, OMRMEM_CATEGORY_VM);
				if (NULL == buf) {
					j9str_free_tokens(stringTokens);
					rc = JNI_ERR;
					goto xlogret;
				}
				strcpy(buf, MAPOPT_XVERBOSEGCLOG);
				j9str_subst_tokens(buf + sizeof(MAPOPT_XVERBOSEGCLOG) - 1, sizeRequired, gclogfile, stringTokens);
				j9str_free_tokens(stringTokens);
#if defined(DEBUG_XLOG)
				j9tty_printf(PORTLIB, "mapping %s to %s\n", j9vm_args->actualVMArgs->options[xlogindex].optionString, buf);
#endif /* defined(DEBUG_XLOG) */
				mapRc = registerCmdLineMapping(vm, j9vm_args->actualVMArgs->options[xlogindex].optionString, buf, EXACT_MAP_NO_OPTIONS);
				j9mem_free_memory(buf);
				if (RC_FAILED == mapRc) {
					rc = JNI_ERR;
					goto xlogret;
				}
			} else {
				if (RC_FAILED == registerCmdLineMapping(vm, j9vm_args->actualVMArgs->options[xlogindex].optionString, MAPOPT_VERBOSE_GC, EXACT_MAP_NO_OPTIONS)) {
					rc = JNI_ERR;
					goto xlogret;
				}
			}
		} else if (setgclog) {
			/* ATM there is no OpenJ9 option to disable gc logging */
			unrecognizedOption = TRUE;
		} else {
			unrecognizedOption = TRUE;
		}
		if (unrecognizedOption) {
			CONSUME_ARG(j9vm_args, xlogindex);
			j9nls_printf(PORTLIB, J9NLS_WARNING, J9NLS_VM_UNRECOGNISED_CMD_LINE_OPT, j9vm_args->actualVMArgs->options[xlogindex].optionString);
		}

xlogret:
		if (xlogoptionsbuf != xlogoptions) {
			j9mem_free_memory(xlogoptions);
		}
		if (JNI_OK != rc) {
			break;
		}

		xlogindex = FIND_NEXT_ARG_IN_VMARGS_FORWARD(STARTSWITH_MATCH, MAPOPT_XLOG_OPT, NULL, xlogindex);
	}
	return rc;
}

/*
 * Compute the heap region size that G1GC would use for the specified maximum
 * heap size. G1GC aims for 2048 regions with the size of each region being a
 * power of 2, no less than 1MB and no more than 32MB.
 */
static size_t
computeG1HeapRegionSize(size_t maxHeapSize)
{
	size_t minSize = 1024 * 1024;
	size_t maxSize = 32 * minSize;
	size_t regionSize = (maxHeapSize + 2047) / 2048;
	size_t result = minSize;

	/* A simple loop is sufficient here. It clearly achieves our goal, and
	 * the cost is low (we iterate at most 5 times and this function is used
	 * at most once).
	 */
	while ((result < regionSize) && (result < maxSize)) {
		result <<= 1;
	}

	return result;
}

static UDATA
protectedInitializeJavaVM(J9PortLibrary* portLibrary, void * userData)
{
	J9InitializeJavaVMArgs * initArgs = userData;
	void * osMainThread = initArgs->osMainThread;
	J9JavaVM * vm = initArgs->vm;
	extern struct JNINativeInterface_ EsJNIFunctions;
	J9VMThread *env = NULL;
	UDATA parseError = FALSE;
	jint stageRC = 0;
	UDATA requiredDebugAttributes;
	UDATA localVerboseLevel = 0;
	BOOLEAN doParseXlogForCompatibility = FALSE;
	U_32 * stat;
#if defined(LINUX)
	int filter = -1;
#endif
	J9JavaVM** BFUjavaVM;
	IDATA xxUseG1GC = 0; /* +1 if -XX:+UseG1GC used; -1 if -XX:-UseG1GC used */

#if defined(J9VM_GC_DYNAMIC_CLASS_UNLOADING)
	J9HookInterface** vmHooks;
#endif

#if !defined(WIN32)
	/* Needed for JCL dependency on JVM to set SIGPIPE to SIG_IGN */
	struct sigaction newSignalAction;
#endif

	PORT_ACCESS_FROM_PORT(portLibrary);

	/* check processor support for cache writeback */
	vm->dCacheLineSize = 0;
	vm->cpuCacheWritebackCapabilities = 0;

#if JAVA_SPEC_VERSION >= 16
	/* ffi_cif should be allocated on demand */
	vm->cifNativeCalloutDataCache = NULL;
	vm->cifArgumentTypesCache = NULL;
	/* The thunk block should be allocated on demand */
	vm->thunkHeapHead = NULL;
#endif /* JAVA_SPEC_VERSION >= 16 */

#if defined(J9X86) || defined(J9HAMMER)
	{
		OMRPORT_ACCESS_FROM_J9PORT(PORTLIB);
		OMRProcessorDesc desc;
		omrsysinfo_get_processor_description(&desc);
		/* cache line size in bytes is the value of bits 8-15 * 8 */
		vm->dCacheLineSize = ((desc.features[2] & 0xFF00) >> 8) * 8;
		if (omrsysinfo_processor_has_feature(&desc, OMR_FEATURE_X86_CLWB)) {
			vm->cpuCacheWritebackCapabilities = OMR_FEATURE_X86_CLWB;
		} else if (omrsysinfo_processor_has_feature(&desc, OMR_FEATURE_X86_CLFLUSHOPT)) {
			vm->cpuCacheWritebackCapabilities = OMR_FEATURE_X86_CLFLUSHOPT;
		} else if (omrsysinfo_processor_has_feature(&desc, OMR_FEATURE_X86_CLFSH)) {
			vm->cpuCacheWritebackCapabilities = OMR_FEATURE_X86_CLFSH;
		}
	}
#endif /* x86 */

	if (vm->dCacheLineSize == 0) {
		IDATA queryResult = 0;
		J9CacheInfoQuery cQuery = {0};
		cQuery.cmd = J9PORT_CACHEINFO_QUERY_LINESIZE;
		cQuery.level = 1;
		cQuery.cacheType = J9PORT_CACHEINFO_DCACHE;
		queryResult = j9sysinfo_get_cache_info(&cQuery);
		if (queryResult > 0) {
			vm->dCacheLineSize = (UDATA)queryResult;
		} else {
			Trc_VM_contendedLinesizeFailed(queryResult);
		}
	}

	/* check for -Xipt flag and run the iconv_global_init accordingly.
	 * If this function fails, bail out of VM init instead of hitting some random
	 * weird undebuggable crash later on
	 */
	if (0 != setGlobalConvertersAware(vm)) {
		goto error;
	}
#if !defined(WIN32)
	vm->originalSIGPIPESignalAction = j9mem_allocate_memory(sizeof(struct sigaction), OMRMEM_CATEGORY_VM);
	if (NULL == vm->originalSIGPIPESignalAction) {
		goto error;
	}

	/* Needed for JCL dependency on JVM to set SIGPIPE to SIG_IGN */
	sigemptyset(&newSignalAction.sa_mask);
#ifndef J9ZTPF
	newSignalAction.sa_flags = SA_RESTART;
#else /* !defined(J9ZTPF) */
	newSignalAction.sa_flags = 0;
#endif /* defined(J9ZTPF) */
	newSignalAction.sa_handler = SIG_IGN;
	sigaction(SIGPIPE,&newSignalAction,(struct sigaction *)vm->originalSIGPIPESignalAction);
#endif

#ifdef J9VM_OPT_SIDECAR
	vm->j2seVersion = initArgs->j2seVersion;
	vm->j2seRootDirectory = initArgs->j2seRootDirectory;
	vm->j9libvmDirectory = initArgs->j9libvmDirectory;
#endif /* J9VM_OPT_SIDECAR */

	/* -Xrealtime is special: we used to support it, but now we don't - tell the user. */
	if (0 <= FIND_ARG_IN_VMARGS(EXACT_MATCH, VMOPT_XREALTIME, NULL)) {
		j9nls_printf(PORTLIB, J9NLS_ERROR, J9NLS_VM_XOPTION_NO_LONGER_SUPPORTED, VMOPT_XREALTIME);
		goto error;
	}

#ifdef WIN32
	if (0 != preloadUser32Dll(vm)) {
		goto error;
	}
#endif

#if defined(J9VM_OPT_METHOD_HANDLE)
	/* Enable i2j MethodHandle transitions by default */
	vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_I2J_MH_TRANSITION_ENABLED;
#endif

	/* Default to using lazy in all but realtime */
	vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_LAZY_SYMBOL_RESOLUTION;

	/* Initialize the pool and mutex for per-class MemberName lists. */
#if defined(J9VM_OPT_OPENJDK_METHODHANDLE)
	vm->memberNameListNodePool = pool_new(sizeof(J9MemberNameListNode), 0, 0, 0, J9_GET_CALLSITE(), OMRMEM_CATEGORY_VM, POOL_FOR_PORT(portLibrary));
	if (NULL == vm->memberNameListNodePool) {
		goto error;
	}

	if (0 != omrthread_monitor_init_with_name(&vm->memberNameListsMutex, 0, "MemberName lists mutex")) {
		goto error;
	}
#endif /* defined(J9VM_OPT_OPENJDK_METHODHANDLE) */

	/* Scans cmd-line arguments in order */
	if (JNI_OK != processVMArgsFromFirstToLast(vm)) {
		goto error;
	}

	/* At this point, the compressed/full determination has been made */

	J9RASInitialize(vm);

	initializeROMClasses(vm);

#ifdef J9VM_INTERP_VERBOSE
	localVerboseLevel = vm->verboseLevel;
#endif

	stat = addStatistic (vm, (U_8*)"J9MemTag Version", J9STAT_U32);
	if (NULL != stat) {
		*stat = J9MEMTAG_VERSION;
	}

	/* Memory eyecatchers are stored as their complement to avoid scanner confusion */
	stat = addStatistic (vm, (U_8*)"J9MemTag Eyecatcher Alloc Header", J9STAT_U32);
	if (NULL != stat) {
		*stat = ~J9MEMTAG_EYECATCHER_ALLOC_HEADER;
	}
	stat = addStatistic (vm, (U_8*)"J9MemTag Eyecatcher Alloc Footer", J9STAT_U32);
	if (NULL != stat) {
		*stat = ~J9MEMTAG_EYECATCHER_ALLOC_FOOTER;
	}
	stat = addStatistic (vm, (U_8*)"J9MemTag Eyecatcher Freed Header", J9STAT_U32);
	if (NULL != stat) {
		*stat = ~J9MEMTAG_EYECATCHER_FREED_HEADER;
	}
	stat = addStatistic (vm, (U_8*)"J9MemTag Eyecatcher Freed Footer", J9STAT_U32);
	if (NULL != stat) {
		*stat = ~J9MEMTAG_EYECATCHER_FREED_FOOTER;
	}

#ifdef LINUX
	stat = addStatistic (vm, (U_8*)"J9OSDump ProcSelfMaps Eyecatcher", J9STAT_U32);
	if (NULL != stat) {
		*stat = ~J9OSDUMP_EYECATCHER;
	}

	/* with kernel patch introducing /proc/<pid>/coredump_filter parameter we need to
	 * update the filter to include all pages.
	 */
	filter = j9file_open("/proc/self/coredump_filter", EsOpenTruncate|EsOpenWrite, 0);
	if (-1 != filter) {
		/* /proc returns EINVAL if the data isn't valid for the target virtual file. We must
		 * write the bit flags as a string rather than as a single byte for this to work.
		 * 0x7F includes ELF headers (kernel 2.6.24) and huge pages (kernel 2.6.28).
		 */
		j9file_printf(filter, "0x7F\n");
		/* the only expected error is that we're on a system where coredump_filter doesn't exist (ENOENT) */

		j9file_close(filter);
	}
#endif

	vm->walkStackFrames = walkStackFrames;
	vm->walkFrame = walkFrame;

#if defined(COUNT_BYTECODE_PAIRS)
	if (JNI_OK != initializeBytecodePairs(vm)) {
		goto error;
	}
#endif /* COUNT_BYTECODE_PAIRS */
	if (JNI_OK != initializeVTableScratch(vm)) {
		goto error;
	}

	if (JNI_OK != initializeVprintfHook(vm)) {
		goto error;
	}

	if (NULL == contendedLoadTableNew(vm, portLibrary)) {
		goto error;
	}

	/* Handle -Xsyslog and legacy -Xlog early, so that any future init failures can be reported to the system log */
	{
		IDATA enabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXLEGACYXLOGOPTION, NULL);
		IDATA disabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOLEGACYXLOGOPTION, NULL);
		/* Default to -XX:-LegacyXlogOption */
		if (enabled > disabled) {
			if (RC_FAILED == registerCmdLineMapping(vm, MAPOPT_XLOG_OPT, VMOPT_XSYSLOG_OPT, EXACT_MAP_NO_OPTIONS)) {
				goto error;
			}
			if (RC_FAILED == registerCmdLineMapping(vm, MAPOPT_XLOG_OPT_COLON, MAPOPT_XSYSLOG_OPT_COLON, MAP_WITH_INCLUSIVE_OPTIONS)) {
				goto error;
			}
		} else {
			doParseXlogForCompatibility = TRUE;
		}
	}

	{
		IDATA argIndex = FIND_AND_CONSUME_ARG_FORWARD(vm->vmArgsArray, STARTSWITH_MATCH, VMOPT_XXCOMPATIBILITY_EQUALS, NULL);
		const char *mode = NULL;

		if (argIndex >= 0) {
			GET_OPTION_VALUE(argIndex, '=', &mode);

			if (0 == j9_cmdla_stricmp(mode, "elasticsearch")) {
				vm->compatibilityFlags |= J9COMPATIBILITY_ELASTICSEARCH;
				doParseXlogForCompatibility = FALSE;
			} else {
				j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_COMPATIBILITY_UNSUPPORTED, mode);
				goto error;
			}

			if (FIND_AND_CONSUME_NEXT_ARG_FORWARD(vm->vmArgsArray, STARTSWITH_MATCH, VMOPT_XXCOMPATIBILITY_EQUALS, NULL, argIndex) >= 0) {
				j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_COMPATIBILITY_REPEATED);
				goto error;
			}
		}
	}

	if (J9_ARE_ANY_BITS_SET(vm->compatibilityFlags, J9COMPATIBILITY_ELASTICSEARCH)) {
		IDATA enabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCOMPATIBILITY_ENABLEG1GC, NULL);
		IDATA disabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXCOMPATIBILITY_DISABLEG1GC, NULL);

		if (enabled > disabled) {
			xxUseG1GC = 1;
		} else if (disabled >= 0) {
			xxUseG1GC = -1;
		}
	}

	if (JNI_OK != processXLogOptions(vm)) {
		parseError = TRUE;
		goto error;
	}

	if (JNI_OK != initializeDDR(vm)) {
		goto error;
	}

	if (J9SYSPROP_ERROR_NONE != initializeSystemProperties(vm)) {
		goto error;
	}

	if (0 != initializeVMHookInterface(vm)) {
		goto error;
	}

#ifndef J9VM_SIZE_SMALL_CODE
	if (NULL == fieldIndexTableNew(vm, portLibrary)) {
		goto error;
	}
#endif

#ifdef J9VM_OPT_ZIP_SUPPORT
	if (NULL == vm->zipCachePool) {
		vm->zipCachePool = zipCachePool_new(portLibrary, vm);
		if (NULL == vm->zipCachePool) {
			goto error;
		}
	}
#endif

	/* Setup primordial dump facade as early as possible */
	if (JNI_OK != configureRasDump(vm)) {
		goto error;
	}

#ifdef J9VM_OPT_SIDECAR
	if (JNI_OK != initializeJVMExtensionInterface(vm)) {
		goto error;
	}
#endif

#ifdef J9VM_OPT_SIDECAR
#if JAVA_SPEC_VERSION < 21
	/* Whine about -Djava.compiler after extra VM options are added, but before mappings are set */
	if (RC_FAILED == checkDjavacompiler(portLibrary, vm->vmArgsArray)) {
		goto error;
	}
#endif /* JAVA_SPEC_VERSION < 21 */

	if (doParseXlogForCompatibility) {
		if (JNI_OK != parseXlogForCompatibility(vm)) {
			parseError = TRUE;
			goto error;
		}
	}

#if JAVA_SPEC_VERSION > 8
	if ((0 <= findArgInVMArgs(PORTLIB, vm->vmArgsArray, EXACT_MATCH, MAPOPT_XHEALTHCENTER, NULL, FALSE))
	||  (0 <= findArgInVMArgs(PORTLIB, vm->vmArgsArray, STARTSWITH_MATCH, VMOPT_AGENTLIB_HEALTHCENTER_EQUALS, NULL, FALSE))
	) {
		/* implicitly include module for -Xhealthcenter or -Xhealthcenter: */
		vm->extendedRuntimeFlags2 |= J9_EXTENDED_RUNTIME2_LOAD_HEALTHCENTER_MODULE;
	}
#endif /* JAVA_SPEC_VERSION > 8 */

	/* Registers any unrecognised arguments that need to be mapped to J9 options */
	if (RC_FAILED == registerVMCmdLineMappings(vm)) {
		goto error;
	}
#endif /* J9VM_OPT_SIDECAR */

	vm->dllLoadTable = initializeDllLoadTable(portLibrary, vm->vmArgsArray, localVerboseLevel, vm);
	if (NULL == vm->dllLoadTable) {
		goto error;
	}
	JVMINIT_VERBOSE_INIT_TRACE_WORKING_SET(vm);

	/* Scans cmd-line and whacks the table for entries like -Xint */
	if (JNI_OK != modifyDllLoadTable(vm, vm->dllLoadTable, vm->vmArgsArray)) {
		goto error;
	}

#if !defined(WIN32)
	if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags,J9_EXTENDED_RUNTIME_HANDLE_SIGXFSZ)) {
		j9sig_set_async_signal_handler(sigxfszHandler, NULL, J9PORT_SIG_FLAG_SIGXFSZ);
	}
#endif

	if ( J9_ARE_ALL_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_ENABLE_CPU_MONITOR)) {
		omrthread_lib_set_flags(J9THREAD_LIB_FLAG_ENABLE_CPU_MONITOR);
	} else {
		omrthread_lib_clear_flags(J9THREAD_LIB_FLAG_ENABLE_CPU_MONITOR);
	}

	registerIgnoredOptions(PORTLIB, vm->vmArgsArray);				/* Tags -D java options and options in ignoredOptionTable as not consumable */

#if !defined(J9VM_INTERP_MINIMAL_JNI)
	vm->EsJNIFunctions = GLOBAL_TABLE(EsJNIFunctions);
#endif

	configureRasTrace( vm, vm->vmArgsArray );

#ifdef JVMINIT_UNIT_TEST
	testFindArgs(vm);
	testOptionValueOps(vm);
#endif

	/* Attach the VM to OMR */
	if (JNI_OK != attachVMToOMR(vm)) {
		goto error;
	}

#if defined(J9VM_OPT_JFR)
	if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags2, J9_EXTENDED_RUNTIME2_JFR_ENABLED)) {
		if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags3, J9_EXTENDED_RUNTIME3_START_FLIGHT_RECORDING)) {
			if (JNI_OK != initializeJFR(vm, FALSE)) {
				goto error;
			}
		}
	}
#endif /* defined(J9VM_OPT_JFR) */

	/* Use this stage to load libraries which need to set up hooks as early as possible */
	if (JNI_OK != runLoadStage(vm, EARLY_LOAD)) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, PORT_LIBRARY_GUARANTEED))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, ALL_DEFAULT_LIBRARIES_LOADED))) {
		goto error;
	}

	if (JNI_OK != runLoadStage(vm, LOAD_BY_DEFAULT)) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, ALL_LIBRARIES_LOADED))) {
		goto error;
	}

	J9RelocateRASData(vm);
	if (JNI_OK != runLoadStage(vm, FORCE_LATE_LOAD)) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, DLL_LOAD_TABLE_FINALIZED))) {
		goto error;
	}

	/* Run shutdown stage for any libraries being forced to unload */
	/* Note that INTERPRETER_SHUTDOWN is not run here. The interpreter is not yet started... */
	if (JNI_OK != runShutdownStage(vm, LIBRARIES_ONUNLOAD, (void*)FALSE, FORCE_UNLOAD)) {
		goto error;
	}

	if (JNI_OK != runForcedUnloadStage(vm)) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, VM_THREADING_INITIALIZED))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, HEAP_STRUCTURES_INITIALIZED))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, ALL_VM_ARGS_CONSUMED))) {
		goto error;
	}

	if (FALSE == checkArgsConsumed(vm, portLibrary, vm->vmArgsArray)) {
		parseError = TRUE;
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, BYTECODE_TABLE_SET))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, SYSTEM_CLASSLOADER_SET))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, DEBUG_SERVER_INITIALIZED))) {
		goto error;
	}

#if defined(J9VM_GC_DYNAMIC_CLASS_UNLOADING)
	vmHooks = getVMHookInterface(vm);
	if(0 != (*vmHooks)->J9HookRegisterWithCallSite(vmHooks, J9HOOK_VM_CLASS_UNLOAD, freeClassNativeMemory, OMR_GET_CALLSITE(), NULL)
		|| 0 != (*vmHooks)->J9HookRegisterWithCallSite(vmHooks, J9HOOK_VM_ANON_CLASSES_UNLOAD, vmHookAnonClassesUnload, OMR_GET_CALLSITE(), NULL)
	) {
		goto error;
	}
#endif

	/* env is not used, but must be passed for compatibility */
	/* use NO_OBJECT, because it's too early to allocate an object -- we'll take care of that later in standardInit() or tinyInit() */
	if (JNI_OK != internalAttachCurrentThread(vm, &env, NULL, J9_PRIVATE_FLAGS_NO_OBJECT, osMainThread)) {
		goto error;
	}
	env->gpProtected = TRUE;

	if (JNI_OK != (stageRC = runInitializationStage(vm, TRACE_ENGINE_INITIALIZED))) {
		goto error;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, JIT_INITIALIZED))) {
		goto error;
	}

	/* If the JIT started, set the java.compiler/openj9.compiler system property and allocate the global OSR buffer */
	if (NULL != vm->jitConfig) {
#ifndef DELETEME
		UDATA osrGlobalBufferSize = sizeof(J9JITDecompilationInfo);
#endif
#if JAVA_SPEC_VERSION < 21
		J9VMSystemProperty * property = NULL;

		if (J9SYSPROP_ERROR_NONE == getSystemProperty(vm, "java.compiler", &property)) {
			setSystemProperty(vm, property, J9_JIT_DLL_NAME);
			property->flags &= ~J9SYSPROP_FLAG_WRITEABLE;
		}
#else /* JAVA_SPEC_VERSION < 21 */
		if (J9SYSPROP_ERROR_NONE != addSystemProperty(vm, "openj9.compiler", J9_JIT_DLL_NAME, 0)) {
			goto error;
		}
#endif /* JAVA_SPEC_VERSION < 21 */
#ifndef DELETEME
		osrGlobalBufferSize += ROUND_TO(sizeof(UDATA), vm->jitConfig->osrFramesMaximumSize);
		osrGlobalBufferSize += ROUND_TO(sizeof(UDATA), vm->jitConfig->osrScratchBufferMaximumSize);
		osrGlobalBufferSize += ROUND_TO(sizeof(UDATA), vm->jitConfig->osrStackFrameMaximumSize);
		vm->osrGlobalBufferSize = osrGlobalBufferSize;
		vm->osrGlobalBuffer = j9mem_allocate_memory(osrGlobalBufferSize, OMRMEM_CATEGORY_JIT);
		if (NULL == vm->osrGlobalBuffer) {
			goto error;
		}
#endif
		if (J9_ARE_ANY_BITS_SET(vm->extendedRuntimeFlags, J9_EXTENDED_RUNTIME_USE_VECTOR_REGISTERS)) {
			j9port_control(J9PORT_CTLDATA_VECTOR_REGS_SUPPORT_ON, 1);
		}
#if defined(OMR_GC_CONCURRENT_SCAVENGER) && defined(J9VM_ARCH_S390)
		/* Concurrent scavenge enabled and JIT loaded implies running on supported h/w.
		 * As such, per-thread initialization must occur.
		 *
		 * This code path is only executed by the Main Thread because the Main Thread
		 * is attached to the VM before the JIT has been initialized. All other threads
		 * will get initialized in vmthread.cpp::threadAboutToStart
		 */
		if (vm->memoryManagerFunctions->j9gc_concurrent_scavenger_enabled(vm)) {
			if (0 == j9gs_initializeThread(env)) {
				fatalError((JNIEnv *)env, "Failed to initialize thread; please disable Concurrent Scavenge.\n");
			}
		}
#endif
		/* Create the global hot field class info pool and global hot field class info pool monitor if scavenger dynamicBreadthFirstScanOrdering is enabled
		 *
		 * The global hot field class info pool and global hot field class info pool monitor will be used to store hot
		 * field information for all classes containing hot fields. In addition to this, the globalHotFieldPoolMutex that is created
		 * is used when a new hot field pool for a classLoader is to be dynamically created.
		 */
		if (vm->memoryManagerFunctions->j9gc_hot_reference_field_required(vm)) {
			vm->hotFieldClassInfoPool = pool_new(sizeof(J9ClassHotFieldsInfo),  0, 0, 0, J9_GET_CALLSITE(), J9MEM_CATEGORY_CLASSES, POOL_FOR_PORT(portLibrary));	/* Create the hot field class pool */
			if ((NULL == vm->hotFieldClassInfoPool)
				|| (0 != omrthread_monitor_init_with_name(&vm->hotFieldClassInfoPoolMutex, 0, "hotFieldClassInfoPoolMutex"))
				|| (0 != omrthread_monitor_init_with_name(&vm->globalHotFieldPoolMutex, 0, "globalHotFieldPoolMutex"))
			) {
				goto error;
			}
		}

	} else {
#if JAVA_SPEC_VERSION >= 21
		if (J9SYSPROP_ERROR_NONE != addSystemProperty(vm, "openj9.compiler", "", 0)) {
			goto error;
		}
#endif /* JAVA_SPEC_VERSION >= 21 */

		/* If there is no JIT, change the vm phase so RAS will enable level 2 tracepoints */
		jvmPhaseChange(vm, J9VM_PHASE_NOT_STARTUP);
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, AGENTS_STARTED))) {
		goto error;
	}

#ifdef J9VM_OPT_DEBUG_INFO_SERVER
	/* After agents have started, finalize the set of required debug attributes */

	requiredDebugAttributes = 0;
	TRIGGER_J9HOOK_VM_REQUIRED_DEBUG_ATTRIBUTES(vm->hookInterface, env, requiredDebugAttributes);
	if (vm->extendedRuntimeFlags & J9_EXTENDED_RUNTIME_ENABLE_HCR) {
		requiredDebugAttributes |= (J9VM_DEBUG_ATTRIBUTE_ALLOW_RETRANSFORM | J9VM_DEBUG_ATTRIBUTE_CAN_REDEFINE_CLASSES);
	}
	vm->requiredDebugAttributes |= requiredDebugAttributes;
#endif

#if defined (J9VM_OPT_SHARED_CLASSES)
	if ((NULL != vm->sharedClassConfig) && (NULL != vm->sharedClassConfig->sharedAPIObject)) {
		SCAbstractAPI * sharedapi = (SCAbstractAPI *)(vm->sharedClassConfig->sharedAPIObject);
		if (J9VMDLLMAIN_OK != sharedapi->sharedClassesFinishInitialization(vm)) {
			goto error;
		}
	}
#endif

	initializeInitialMethods(vm);

	if (JNI_OK != (stageRC = runInitializationStage(vm, ABOUT_TO_BOOTSTRAP))) {
		goto error;
	}

	/* At this point the systemClassLoader and classTableMutex both exist, and
	 * (if using the JIT) the jitConfig is fully initialized (so in particular
	 * the JIT can be notified of permanent loaders). Mark systemClassLoader
	 * permanent before we run (and therefore before we compile) any Java code.
	 */
	omrthread_monitor_enter(vm->classTableMutex);
	markLoaderPermanent(vm->mainThread, vm->systemClassLoader);
	omrthread_monitor_exit(vm->classTableMutex);

	/* Set the BFUjavaVM obtained from vm_args to the created vm */
	BFUjavaVM = initArgs->globalJavaVM;
	if (NULL != BFUjavaVM) {
		*BFUjavaVM = vm;
	}

	if (JNI_OK != (stageRC = runInitializationStage(vm, JCL_INITIALIZED))) {
		goto error;
	}

	/* With JCL initialization complete, extensionClassLoader and
	 * applicationClassLoader are immutable from now on, so the loaders they
	 * point to are now guaranteed never to be unloaded.
	 */
	omrthread_monitor_enter(vm->classTableMutex);
	markLoaderPermanent(vm->mainThread, vm->extensionClassLoader);
	markLoaderPermanent(vm->mainThread, vm->applicationClassLoader);
	omrthread_monitor_exit(vm->classTableMutex);

	if (JNI_OK != (stageRC = runInitializationStage(vm, VM_INITIALIZATION_COMPLETE))) {
		goto error;
	}

	sidecarInit(env);
#if defined(J9VM_INTERP_ATOMIC_FREE_JNI)
	/* sidecarInit leaves the thread in a JNI context */
	enterVMFromJNI(env);
	releaseVMAccess(env);
#endif /* J9VM_INTERP_ATOMIC_FREE_JNI */

	if (0 != vm->memoryManagerFunctions->gcStartupHeapManagement(vm)) {
		goto error;
	}

	TRIGGER_J9HOOK_VM_STARTED(vm->hookInterface, env);

	env->gpProtected = FALSE;

#ifdef J9OS_I5
	/* debug code */
	Xj9BreakPoint("jvminit");
#endif

	{
		IDATA enabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXPRINTFLAGSFINALENABLE, NULL);
		IDATA disabled = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXPRINTFLAGSFINALDISABLE, NULL);
		if (enabled > disabled) {
			size_t maxHeapSize = (size_t) (vm->memoryManagerFunctions->j9gc_get_maximum_heap_size(vm));
			uint64_t maxDirectMemorySize = (uint64_t) (((~(UDATA)0) == vm->directByteBufferMemoryMax) ? 0 : vm->directByteBufferMemoryMax);

			/*
			 * Emulate Hotspot -XX:+PrintFlagsFinal output for two specific flags:
			 *
			 * [Global flags]
			 *    size_t MaxHeapSize                              = 4294967296                                {product} {ergonomic}
			 *  uint64_t MaxDirectMemorySize                      = 3758096384                                {product} {default}
			 *
			 * When -XX:Compatibility=elasticsearch is specified, four more flags are included:
			 *
			 *    size_t G1HeapRegionSize                         = 2097152                                   {product} {ergonomic}
			 *     uintx G1ReservePercent                         = 10                                        {product} {default}
			 *     uintx InitiatingHeapOccupancyPercent           = 45                                        {product} {default}
			 *      bool UseG1GC                                  = true                                      {product} {ergonomic}
			 *
			 * NOTE that Hotspot produces this output on STDOUT, and applications
			 * expect to parse it there, which means this code does not use
			 * j9tty_printf() like most of the VM (which prints to STDERR).
			 * Instead, j9file_printf(J9PORT_TTY_OUT, ...) is used.
			 */

#define PRINT_FLAG(fmt, type, name, value, howset) \
	j9file_printf(J9PORT_TTY_OUT, \
			"%9s %-40s = %-41" fmt " {product} {%s}\n", \
			(type), (name), (value), (howset))

			j9file_printf(J9PORT_TTY_OUT, "[Global flags]\n");

			PRINT_FLAG("zu", "size_t", "MaxHeapSize",
					maxHeapSize,
					(findArgInVMArgs(PORTLIB, vm->vmArgsArray, STARTSWITH_MATCH, VMOPT_XMX, NULL, 0) >= 0)
							? "command line" : "ergonomic");

			PRINT_FLAG("llu", "uint64_t", "MaxDirectMemorySize",
					maxDirectMemorySize,
					(findArgInVMArgs(PORTLIB, vm->vmArgsArray, STARTSWITH_MATCH, VMOPT_XXMAXDIRECTMEMORYSIZEEQUALS, NULL, 0) >= 0)
							?  "command line" : "ergonomic");

			if (J9_ARE_ANY_BITS_SET(vm->compatibilityFlags, J9COMPATIBILITY_ELASTICSEARCH)) {
				PRINT_FLAG("zu", "size_t", "G1HeapRegionSize",
						computeG1HeapRegionSize(maxHeapSize), "ergonomic");
				PRINT_FLAG("u", "uintx", "G1ReservePercent",
						10, "default");
				PRINT_FLAG("u", "uintx", "InitiatingHeapOccupancyPercent",
						45, "default");
				PRINT_FLAG("s", "bool", "UseG1GC",
						(0 <= xxUseG1GC) ? "true" : "false",
						(0 != xxUseG1GC) ? "command line" : "ergonomic");
			}
		}

#undef PRINT_FLAG
	}

	return JNI_OK;

error:
	if (RC_SILENT_EXIT == stageRC) {
		/* Returning RC_SILENT_EXIT is understood by launcher and thereby hide the message
		 * "Could not create the Java virtual machine."
		 * Don't change the value of RC_SILENT_EXIT (-72) in vm as it has a external dependency.
		 */
		return stageRC;
	}
	return ((TRUE == parseError) ? JNI_EINVAL : JNI_ENOMEM);
}

#if !defined(WIN32)
static UDATA
sigxfszHandler(struct J9PortLibrary* portLibrary, U_32 gpType, void* gpInfo, void* userData)
{
	return 0;
}
#endif

#if (defined(J9VM_OPT_SIDECAR)) /* priv. proto (autogen) */
void
sidecarInit(J9VMThread *mainThread)
{
	/* we load java.lang.Shutdown so that it is already loaded when sidecarShutdown is called.
	 * This prevents memory allocations that will occur after an OutOfMemoryError or
	 * a StackOverflowError has occurred.
	 */
	if (NULL == mainThread->functions->FindClass((JNIEnv *) mainThread, "java/lang/Shutdown")) {
		/* if the class load fails, we simply move on.  This is not a reason to halt startup. */
		mainThread->functions->ExceptionClear((JNIEnv *) mainThread);
	}
}
#endif /* J9VM_OPT_SIDECAR (autogen) */

static UDATA
initializeVTableScratch(J9JavaVM* vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
#ifdef J9VM_ENV_DATA64
	UDATA vtableBytes = 4096;
#else
	UDATA vtableBytes = 2048;
#endif

	vm->vTableScratch = j9mem_allocate_memory(vtableBytes, OMRMEM_CATEGORY_VM);
	if (vm->vTableScratch == NULL) {
		return JNI_ENOMEM;
	}
	vm->vTableScratchSize = vtableBytes;

	return JNI_OK;
}


static UDATA
initializeVprintfHook(J9JavaVM* vm)
{
	IDATA index;
	PORT_ACCESS_FROM_JAVAVM(vm);

	index = findArgInVMArgs( PORTLIB,  vm->vmArgsArray, EXACT_MATCH, VMOPT_VFPRINTF, NULL, FALSE);
	if (index >= 0) {
		jint (JNICALL * * vprintfHookFunctionPtr)(FILE *fp, const char *format, va_list args) = GLOBAL_DATA(vprintfHookFunction);

		*vprintfHookFunctionPtr = (jint(JNICALL *)(FILE*, const char*, va_list))vm->vmArgsArray->actualVMArgs->options[index].extraInfo;

		if (*vprintfHookFunctionPtr != NULL) {
			OMRPORT_FROM_J9PORT(vm->portLibrary)->tty_printf = &vfprintfHook;
			portLibrary_file_write_text = OMRPORT_FROM_J9PORT(vm->portLibrary)->file_write_text;
			OMRPORT_FROM_J9PORT(vm->portLibrary)->file_write_text = &vfprintfHook_file_write_text;
		}
	}

	return JNI_OK;
}



#if (defined(J9VM_OPT_SIDECAR))
static UDATA
initializeJVMExtensionInterface(J9JavaVM* vm)
{
	struct JVMExtensionInterface_  *jvmExt;
	PORT_ACCESS_FROM_JAVAVM(vm);

	jvmExt = j9mem_allocate_memory(sizeof(struct JVMExtensionInterface_), OMRMEM_CATEGORY_VM);
	if (jvmExt == NULL) {
		return JNI_ENOMEM;
	}

	memcpy(jvmExt->eyecatcher, "EJVM", 4);
	jvmExt->length = sizeof(struct JVMExtensionInterface_);
	jvmExt->version = 1;
	jvmExt->modification = 1;
	jvmExt->vm = (JavaVM*)vm;
	jvmExt->ResetJavaVM = ResetJavaVM;
	jvmExt->QueryJavaVM = QueryJavaVM;
	jvmExt->QueryGCStatus = QueryGCStatus;

	vm->jvmExtensionInterface = (JVMExt) jvmExt;

	return JNI_OK;
}

#endif /* J9VM_OPT_SIDECAR */


/**
 * @internal
 *
 * Parse the -Xrs, -Xsigchain, -Xnosigchain, etc. options.
 * This is called as early as possible, so that the rest of initialization
 * can be protected from errors (unless -Xrs is specified, of course)
 *
 * @param[in] vm pointer to a J9JavaVM
 *
 * @return 0 on success and -1 on failure
 */
static IDATA
setSignalOptions(J9JavaVM *vm, J9PortLibrary *portLibrary)
{
	IDATA argIndex, argIndex2;
	UDATA defaultSigChain;
	U_32 sigOptions = 0;
	PORT_ACCESS_FROM_JAVAVM(vm);

#if defined(J9ZOS390)
	defaultSigChain = J9_SIG_NO_SIG_CHAIN;
#else
	defaultSigChain = 0;
#endif

	argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XNOSIGCHAIN, NULL);
	argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XSIGCHAIN, NULL);

	/* Use some logic to decide right-most precedence for these mutually exclusive args */
	if ((argIndex < 0) && (argIndex2 < 0)) {
		vm->sigFlags |= defaultSigChain;
	} else if (argIndex > argIndex2) {
		sigOptions |= J9PORT_SIG_OPTIONS_OMRSIG_NO_CHAIN;
		vm->sigFlags |= J9_SIG_NO_SIG_CHAIN;
	}

	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XNOSIGINT, NULL) >= 0) {
		vm->sigFlags |= J9_SIG_NO_SIG_INT;
	}


	argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOHANDLESIGXFSZ, NULL);
	argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXHANDLESIGXFSZ, NULL);

	if (argIndex2 >= argIndex) {
		sigOptions |= J9PORT_SIG_OPTIONS_SIGXFSZ;
		vm->extendedRuntimeFlags |= J9_EXTENDED_RUNTIME_HANDLE_SIGXFSZ;
	}

	if ((argIndex = FIND_AND_CONSUME_VMARG(OPTIONAL_LIST_MATCH, VMOPT_XRS, NULL)) >= 0) {

		char* optionValue;

		GET_OPTION_VALUE(argIndex, ':', &optionValue);

		if ((NULL != optionValue) && (0 == strcmp(optionValue, "sync"))) {
			vm->sigFlags |= J9_SIG_XRS_SYNC;
			sigOptions |= J9PORT_SIG_OPTIONS_REDUCED_SIGNALS_SYNCHRONOUS;
		} else if ((NULL != optionValue) && (0 == strcmp(optionValue, "async"))) {
			vm->sigFlags |= (J9_SIG_XRS_ASYNC | J9_SIG_NO_SIG_QUIT | J9_SIG_NO_SIG_USR2);
			sigOptions |= J9PORT_SIG_OPTIONS_REDUCED_SIGNALS_ASYNCHRONOUS;
		} else {
			vm->sigFlags |= (J9_SIG_XRS_SYNC | J9_SIG_XRS_ASYNC | J9_SIG_NO_SIG_QUIT | J9_SIG_NO_SIG_USR2);
			sigOptions |= (J9PORT_SIG_OPTIONS_REDUCED_SIGNALS_SYNCHRONOUS | J9PORT_SIG_OPTIONS_REDUCED_SIGNALS_ASYNCHRONOUS);
		}
	}

#if defined(WIN32) && defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE)
	HMODULE h_kernel32 = GetModuleHandle(TEXT("kernel32.dll"));

	if ((NULL != h_kernel32) && IsWindows8OrGreater()) {
		typedef BOOL (WINAPI *PMP_func)(HANDLE, PROCESS_MITIGATION_POLICY, PVOID, SIZE_T);
		PMP_func getProcessMitigationPolicyFunc = (PMP_func)GetProcAddress(h_kernel32, "GetProcessMitigationPolicy");

		PROCESS_MITIGATION_CONTROL_FLOW_GUARD_POLICY cfgPolicy = {0};

		if (((PMP_func)NULL != getProcessMitigationPolicyFunc)
			&& getProcessMitigationPolicyFunc(
					GetCurrentProcess(),
					ProcessControlFlowGuardPolicy,
					&cfgPolicy,
					sizeof(cfgPolicy))
			&& cfgPolicy.EnableControlFlowGuard
		) {
			vm->sigFlags |= J9_SIG_WINDOWS_MITIGATION_POLICY_CFG_ENABLED;
		}
	}
#endif /* defined(WIN32) && defined(_WIN32_WINNT_WINBLUE) && (_WIN32_WINNT_MAXVER >= _WIN32_WINNT_WINBLUE) */

	argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOHANDLESIGABRT, NULL);
	argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXHANDLESIGABRT, NULL);

	if (argIndex2 > argIndex) {
		/* Enable the JVM abort handler since -XX:+HandleSIGABRT is seen last. */
		if (OMR_ARE_ALL_BITS_SET(vm->sigFlags, J9_SIG_XRS_SYNC)) {
			/* Throw error message if both -XX:+HandleSIGABRT and -Xrs/-Xrs:sync are supplied. */
			j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_INCOMPATIBLE_CMDLINE_OPTIONS_ERROR, VMOPT_XXHANDLESIGABRT, VMOPT_XRS);
			return -1;
		}
	} else if (argIndex > argIndex2) {
		/* Disable the JVM abort handler since -XX:-HandleSIGABRT is seen last. */
		vm->sigFlags |= J9_SIG_NO_SIG_ABRT;
	} else {
		/* argIndex == argIndex2 i.e. no option supplied. Enable the JVM abort handler by default. */
	}

	argIndex = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXNOHANDLESIGUSR2, NULL);
	argIndex2 = FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XXHANDLESIGUSR2, NULL);

	if (argIndex2 > argIndex) {
		/* Enable the JVM SIGUSR2 handler since -XX:+HandleSIGUSR2 is seen last. */
		if (OMR_ARE_ALL_BITS_SET(vm->sigFlags, J9_SIG_XRS_ASYNC)) {
			/* Throw error message if both -XX:+HandleSIGUSR2 and -Xrs/-Xrs:async are supplied. */
			j9nls_printf(portLibrary, J9NLS_ERROR, J9NLS_VM_INCOMPATIBLE_CMDLINE_OPTIONS_ERROR, VMOPT_XXHANDLESIGUSR2, VMOPT_XRS);
			return -1;
		}
	} else if (argIndex > argIndex2) {
		/* Disable the JVM SIGUSR2 handler since -XX:-HandleSIGUSR2 is seen last. */
		vm->sigFlags |= J9_SIG_NO_SIG_USR2;
	} else {
		/* argIndex == argIndex2 i.e. no option supplied. Enable the JVM SIGUSR2 handler by default. */
	}

#if defined(J9VM_PORT_ZOS_CEEHDLRSUPPORT)
	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XUSE_CEEHDLR, NULL) >= 0) {
		sigOptions |= J9PORT_SIG_OPTIONS_ZOS_USE_CEEHDLR;
		vm->sigFlags |= J9_SIG_ZOS_CEEHDLR;
	}

	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_XUSE_CEEHDLR_PERCOLATE, NULL) >= 0) {
		sigOptions |= J9PORT_SIG_OPTIONS_ZOS_USE_CEEHDLR;
		vm->sigFlags |= J9_SIG_ZOS_CEEHDLR;
		vm->sigFlags |= J9_SIG_PERCOLATE_CONDITIONS;
	}

#endif

#if defined(J9ZOS390)
	if (FIND_AND_CONSUME_VMARG(EXACT_MATCH, VMOPT_SIGNAL_POSIX_SIGNAL_HANDLER_COOPERATIVE_SHUTDOWN, NULL) >= 0) {
		vm->sigFlags |= J9_SIG_POSIX_COOPERATIVE_SHUTDOWN;
		sigOptions |= J9PORT_SIG_OPTIONS_COOPERATIVE_SHUTDOWN;
	}
#endif

	j9sig_set_options(sigOptions);

	/* deprecated way of configuring the port lib */
	j9port_control(J9PORT_CTLDATA_SIG_FLAGS, vm->sigFlags);

	return 0;
}

#if (defined(J9VM_INTERP_VERBOSE))
/* Runs after each stage. Returns FALSE if an error occurred. */

static const char *
getNameForStage(IDATA stage)
{
	switch (stage) {
		case PORT_LIBRARY_GUARANTEED :
			return "PORT_LIBRARY_GUARANTEED";
		case ALL_DEFAULT_LIBRARIES_LOADED :
			return "ALL_DEFAULT_LIBRARIES_LOADED";
		case ALL_LIBRARIES_LOADED :
			return "ALL_LIBRARIES_LOADED";
		case DLL_LOAD_TABLE_FINALIZED :
			return "DLL_LOAD_TABLE_FINALIZED";
		case VM_THREADING_INITIALIZED :
			return "VM_THREADING_INITIALIZED";
		case HEAP_STRUCTURES_INITIALIZED :
			return "HEAP_STRUCTURES_INITIALIZED";
		case ALL_VM_ARGS_CONSUMED :
			return "ALL_VM_ARGS_CONSUMED";
		case BYTECODE_TABLE_SET :
			return "BYTECODE_TABLE_SET";
		case SYSTEM_CLASSLOADER_SET :
			return "SYSTEM_CLASSLOADER_SET";
		case DEBUG_SERVER_INITIALIZED :
			return "DEBUG_SERVER_INITIALIZED";
		case TRACE_ENGINE_INITIALIZED :
			return "TRACE_ENGINE_INITIALIZED";
		case JIT_INITIALIZED :
			return "JIT_INITIALIZED";
		case AGENTS_STARTED :
			return "AGENTS_STARTED";
		case ABOUT_TO_BOOTSTRAP :
			return "ABOUT_TO_BOOTSTRAP";
		case JCL_INITIALIZED :
			return "JCL_INITIALIZED";
		case VM_INITIALIZATION_COMPLETE :
			return "VM_INITIALIZATION_COMPLETE";
		case INTERPRETER_SHUTDOWN :
			return "INTERPRETER_SHUTDOWN";
		case LIBRARIES_ONUNLOAD :
			return "LIBRARIES_ONUNLOAD";
		case HEAP_STRUCTURES_FREED :
			return "HEAP_STRUCTURES_FREED";
		case GC_SHUTDOWN_COMPLETE :
			return "GC_SHUTDOWN_COMPLETE";
		case LOAD_STAGE :
			return "LOAD_STAGE";
		case UNLOAD_STAGE :
			return "UNLOAD_STAGE";
		case XRUN_INIT_STAGE :
			return "XRUN_INIT_STAGE";
		case JVM_EXIT_STAGE :
			return "JVM_EXIT_STAGE";
		case POST_INIT_STAGE :
			return "POST_INIT_STAGE";
		default :
			return "";
	}
}
#endif /* J9VM_INTERP_VERBOSE */


#if (defined(J9VM_INTERP_VERBOSE))
/* Runs after each stage. Returns FALSE if an error occurred. */

static const char *
getNameForLoadStage(IDATA stage)
{
	switch (stage) {
		case EARLY_LOAD :
			return "EARLY_LOAD";
		case LOAD_BY_DEFAULT :
			return "LOAD_BY_DEFAULT";
		case FORCE_LATE_LOAD :
			return "FORCE_LATE_LOAD";
		default :
			return "";
	}
}
#endif /* J9VM_INTERP_VERBOSE */

#if (defined(J9VM_OPT_JVMTI))
/* Detects agent libraries that are being invoked as Xruns */
static void
detectAgentXruns(J9JavaVM* vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);

	JVMINIT_VERBOSE_INIT_VM_TRACE(vm, "\nDetecting agent Xruns...\n");

	if (vm->dllLoadTable) {
		J9VMDllLoadInfo* entry;
		pool_state aState;
		UDATA dummyFunc = 0;
		UDATA lookupResult;

		entry = (J9VMDllLoadInfo*) pool_startDo(vm->dllLoadTable, &aState);
		while(entry) {
			if (entry->loadFlags & XRUN_LIBRARY) {
				/* If no JVM_OnLoad found */
				lookupResult = j9sl_lookup_name( entry->descriptor, "JVM_OnLoad", &dummyFunc, "iLLL" );
				if (lookupResult) {
					/* If Agent_OnLoad is found */
					lookupResult = j9sl_lookup_name( entry->descriptor, "Agent_OnLoad", &dummyFunc, "ILLL" );
					if (!lookupResult) {
						entry->loadFlags |= AGENT_XRUN;
						JVMINIT_VERBOSE_INIT_VM_TRACE1(vm, "\tFound agent Xrun %s\n", entry->dllName);
					}
				}
			}
			entry = (J9VMDllLoadInfo*) pool_nextDo(&aState);
		}
	}
}
#endif /* J9VM_OPT_JVMTI */


#if defined(J9VM_GC_DYNAMIC_CLASS_UNLOADING)

static void
freeClassNativeMemory(J9HookInterface** hook, UDATA eventNum, void* eventData, void* userData)
{
	J9VMClassUnloadEvent * data = eventData;
	J9VMThread *currentThread = data->currentThread;
	J9JavaVM *vm = currentThread->javaVM;
	J9Class * clazz = data->clazz;
	PORT_ACCESS_FROM_JAVAVM(vm);

	/* Free the ID table for this class, but do not free any of the IDs.  They will be freed by killing their
		pools when the class loader is unloaded.
	*/

	j9mem_free_memory(clazz->jniIDs);
	clazz->jniIDs = NULL;

	/* If the class is an interface, free the HCR method ordering table */
	if (J9ROMCLASS_IS_INTERFACE(clazz->romClass)) {
		j9mem_free_memory(J9INTERFACECLASS_METHODORDERING(clazz));
		J9INTERFACECLASS_SET_METHODORDERING(clazz, NULL);
	}

	/* Free the MemberName list. */
#if defined(J9VM_OPT_OPENJDK_METHODHANDLE)
	omrthread_monitor_enter(vm->memberNameListsMutex);

	if (NULL != clazz->memberNames) {
		J9InternalVMFunctions *vmFuncs = vm->internalVMFunctions;
		J9MemberNameListNode *node = clazz->memberNames;
		clazz->memberNames = NULL;
		while (NULL != node) {
			J9MemberNameListNode *next = node->next;
			/* The weak JNI ref must have been cleared by now. Otherwise, the
			 * MemberName is still live, and this class should be too.
			 */
			Assert_VM_true(NULL == J9_JNI_UNWRAP_REFERENCE(node->memberName));
			vmFuncs->j9jni_deleteGlobalRef((JNIEnv*)currentThread, node->memberName, JNI_TRUE);
			pool_removeElement(vm->memberNameListNodePool, node);
			node = next;
		}
	}

	omrthread_monitor_exit(vm->memberNameListsMutex);
#endif /* defined(J9VM_OPT_OPENJDK_METHODHANDLE) */
}


static void
vmHookAnonClassesUnload(J9HookInterface** hook, UDATA eventNum, void* eventData, void* userData)
{
	J9VMAnonymousClassesUnloadEvent * unloadedEvent = (J9VMAnonymousClassesUnloadEvent *)eventData;
	J9VMThread * vmThread = unloadedEvent->currentThread;
	J9JavaVM *vm = vmThread->javaVM;
	for (J9Class* j9clazz = unloadedEvent->anonymousClassesToUnload; j9clazz; j9clazz = j9clazz->gcLink) {
		/* AnonClass->classLoader points to the hostclass->classLoader not the anonClassLoader. */
		J9ClassLoader *classLoader = j9clazz->classLoader;
		/* Anon classes are unloaded piecemeal, so clear the map cache where the anon maps may be cached */
		freeMapCaches(classLoader);
		if (J9VM_SHOULD_CLEAR_JNIIDS_FOR_ASGCT(vm, classLoader)) {
			void **jniIDs = j9clazz->jniIDs;
			if (NULL != jniIDs) {
				UDATA size = J9VM_NUM_OF_ENTRIES_IN_CLASS_JNIID_TABLE(j9clazz->romClass);
				for (UDATA i = 0; i < size; i++) {
					J9GenericJNIID *id = (J9GenericJNIID*)(jniIDs[i]);
					memset(id, -1, sizeof(J9GenericJNIID));
				}
			}
		}
#if JAVA_SPEC_VERSION >= 22
		hashClassTablePackageDelete(vmThread, j9clazz->classLoader, j9clazz->romClass);
#endif /* JAVA_SPEC_VERSION >= 22 */
	}
}

#endif /* GC_DYNAMIC_CLASS_UNLOADING */

/**
 * Invoke jdk.internal.misc.Signal.dispatch(int number) in Java 9 and
 * onwards. Invoke sun.misc.Signal.dispatch(int number) in Java 8.
 *
 * @param[in] vmThread pointer to a J9VMThread
 * @param[in] signal integer value of the signal
 *
 * @return void
 */
static void
signalDispatch(J9VMThread *vmThread, I_32 signal)
{
	J9JavaVM *vm = vmThread->javaVM;
	J9NameAndSignature nas = {0};
	I_32 args[] = {signal};

	Trc_VM_signalDispatch_signalValue(vmThread, signal);

	nas.name = (J9UTF8 *)&j9_dispatch;
	nas.signature = (J9UTF8 *)&j9_int_void;

	enterVMFromJNI(vmThread);

	if (J2SE_VERSION(vm) >= J2SE_V11) {
		runStaticMethod(vmThread, (U_8 *)"jdk/internal/misc/Signal", &nas, 1, (UDATA *)args);
	} else {
		runStaticMethod(vmThread, (U_8 *)"sun/misc/Signal", &nas, 1, (UDATA *)args);
	}

	/* An exception shouldn't happen over here. */
	Assert_VM_true(NULL == vmThread->currentException);

	releaseVMAccess(vmThread);
}

/* @brief This handler will be invoked by the asynchSignalReporterThread
 * in omrsignal.c once registered using j9sig_set_*async_signal_handler
 * for a specific signal.
 *
 * @param[in] portLibrary the port library
 * @param[in] gpType port library signal flag
 * @param[in] gpInfo GPF information (will be NULL in this case)
 * @param[in] userData user data (will be a pointer to J9JavaVM in this case)
 *
 * @return 0 on success and non-zero on failure
 *
 */
static UDATA
predefinedHandlerWrapper(struct J9PortLibrary *portLibrary, U_32 gpType, void *gpInfo, void *userData)
{
	J9JavaVM *vm = (J9JavaVM *)userData;
	J9JavaVMAttachArgs attachArgs = {0};
	J9VMThread *vmThread = NULL;
	IDATA result = JNI_ERR;
	BOOLEAN invokeHandler = TRUE;
	I_32 signal = 0;
	U_32 runtimeFlags = 0;
	PORT_ACCESS_FROM_JAVAVM(vm);

	signal = j9sig_map_portlib_signal_to_os_signal(gpType);
	/* Don't invoke handler if signal is 0 or negative, or if -Xrs or -Xrs:async is specified */
	if ((signal <= 0) || J9_ARE_ANY_BITS_SET(vm->sigFlags, J9_SIG_XRS_ASYNC)) {
		return 1;
	}

	/* Don't invoke handler if JVM hasn't initialized or JVM exit has started. */
	issueReadBarrier();
	runtimeFlags = vm->runtimeFlags;
	if (J9_ARE_NO_BITS_SET(runtimeFlags, J9_RUNTIME_INITIALIZED)
	|| J9_ARE_ANY_BITS_SET(runtimeFlags, J9_RUNTIME_EXIT_STARTED)
	) {
		invokeHandler = FALSE;
	}

	if (!invokeHandler) {
		return 1;
	}

	attachArgs.version = JNI_VERSION_1_8;
	attachArgs.name = "JVM Signal Thread";
	attachArgs.group = vm->systemThreadGroupRef;

	/* Attach current thread as a daemon thread */
	result = internalAttachCurrentThread(vm, &vmThread, &attachArgs,
				J9_PRIVATE_FLAGS_DAEMON_THREAD | J9_PRIVATE_FLAGS_SYSTEM_THREAD | J9_PRIVATE_FLAGS_ATTACHED_THREAD,
				omrthread_self());

	if (JNI_OK != result) {
		/* Thread couldn't be attached. So, we can't run Java code. */
		return 1;
	}

	/* Run handler (Java code). */
	signalDispatch(vmThread, signal);

	DetachCurrentThread((JavaVM *)vm);

	return 0;
}

IDATA
registerPredefinedHandler(J9JavaVM *vm, U_32 signal, void **oldOSHandler)
{
	IDATA rc = 0;
	U_32 portlibSignalFlag = 0;

	PORT_ACCESS_FROM_JAVAVM(vm);

	portlibSignalFlag = j9sig_map_os_signal_to_portlib_signal(signal);
	if (0 != portlibSignalFlag) {
		rc = j9sig_set_single_async_signal_handler(predefinedHandlerWrapper, vm, portlibSignalFlag, oldOSHandler);
	} else {
		Trc_VM_registerPredefinedHandler_invalidPortlibSignalFlag(portlibSignalFlag);
	}

	return rc;
}

IDATA
registerOSHandler(J9JavaVM *vm, U_32 signal, void *newOSHandler, void **oldOSHandler)
{
	IDATA rc = 0;
	U_32 portlibSignalFlag = 0;

	PORT_ACCESS_FROM_JAVAVM(vm);

	portlibSignalFlag = j9sig_map_os_signal_to_portlib_signal(signal);
	if (0 != portlibSignalFlag) {
		rc = j9sig_register_os_handler(portlibSignalFlag, newOSHandler, oldOSHandler);
	} else {
		Trc_VM_registerOSHandler_invalidPortlibSignalFlag(portlibSignalFlag);
	}

	return rc;
}

static jint
initializeDDR(J9JavaVM * vm)
{
	PORT_ACCESS_FROM_JAVAVM(vm);
	I_64 ddrFileLength;
	char * filename = J9DDR_DAT_FILE;
	char * j9ddrDatDir = NULL;
	jint rc = JNI_OK;

#if defined(J9VM_OPT_SIDECAR)
	/* Append the VM path to the filename if it's available */
	if (J2SE_VERSION(vm) >= J2SE_V11) {
		j9ddrDatDir = vm->j9libvmDirectory;
	} else {
		j9ddrDatDir = vm->j2seRootDirectory;
	}
	if (NULL != j9ddrDatDir) {
		filename = j9mem_allocate_memory(strlen(j9ddrDatDir) + 1 + sizeof(J9DDR_DAT_FILE), OMRMEM_CATEGORY_VM);	/* +1 for slash, sizeof includes nul char already */
		if (filename == NULL) {
			rc = JNI_ENOMEM;
			goto done;
		}
		strcpy(filename, j9ddrDatDir);
		strcat(filename, DIR_SEPARATOR_STR);
		strcat(filename, J9DDR_DAT_FILE);
	}
#endif

	/* Open the DDR data file and read it into memory - if the file is not there, do not raise an error */

	ddrFileLength = j9file_length(filename);
	if ((ddrFileLength > 0) && (ddrFileLength <= J9CONST64(0x7FFFFFFF))) {
		void * ddrData;
		IDATA fd;
		IDATA length = (IDATA) ddrFileLength;

		ddrData = j9mem_allocate_memory((UDATA) length, OMRMEM_CATEGORY_VM);
		if (ddrData == NULL) {
			rc = JNI_ENOMEM;
			goto done;
		}
		vm->j9ras->ddrData = ddrData;

		fd = j9file_open(filename, EsOpenRead, 0);
		if (fd == -1) {
			rc = JNI_ERR;
			goto done;
		}

		if (length != j9file_read(fd, ddrData, length)) {
			rc = JNI_ERR;
		}
		j9file_close(fd);
	}

done:

#if defined(J9VM_OPT_SIDECAR)
	if ((NULL != j9ddrDatDir) && (NULL != filename)) {
		j9mem_free_memory(filename);
	}
#endif

	return rc;
}


#ifdef J9VM_ENV_SSE2_SUPPORT_DETECTION

/* Constant taken from JIT */
#define SSE2_FLAG 0x05000000

#ifdef LINUX
/* Must be static as set by the Linux signal handler */
static BOOLEAN osSupportsSSE = FALSE;
static void
handleSIGILLForSSE(int signal, struct sigcontext context)
{
	/* The STMXCSR instruction is four bytes long.
	* The instruction pointer must be incremented manually to avoid
	* repeating the exception-throwing instruction.
	*/
	context.eip += 4;
	osSupportsSSE = FALSE;
}
#endif


BOOLEAN
isSSE2SupportedOnX86()
{
	BOOLEAN result = FALSE;

	if ((J9SSE2cpuidFeatures() & SSE2_FLAG) == SSE2_FLAG) {

#if defined(WIN32)
		/* Use Structured Exception Handling when building on Win32. */
		__try {
			_mm_getcsr();
			result = TRUE;
		}
		__except(EXCEPTION_EXECUTE_HANDLER)
		{
			result = FALSE;
		}
#elif defined(LINUX)
		/* Use POSIX signals when building on Linux. If an "illegal instruction"
		 * signal is encountered, the signal handler will set osSupportsSSE to FALSE.
		 */
		U_32 mxcsr = 0;
		struct sigaction oldHandler;
		sigaction(SIGILL, NULL, &oldHandler);
		signal(SIGILL, (void (*)(int))handleSIGILLForSSE);
		osSupportsSSE = TRUE;
		asm("stmxcsr %0"::"m"(mxcsr) : );
		sigaction(SIGILL, &oldHandler, NULL);
		result = osSupportsSSE;
#endif
	}
	return result;
}
#endif /* J9VM_ENV_SSE2_SUPPORT_DETECTION */

#if (defined(AIXPPC) || defined(LINUXPPC)) && !defined(J9OS_I5)
BOOLEAN
isPPC64bit()
{
#if defined(AIXPPC)
	if (J9_ADDRMODE_64 != sysconf(_SC_AIX_KERNEL_BITMODE)) {
		return FALSE;
	}
#else /* AIXPPC */
	char *cpu_name = NULL;
	FILE *fp = fopen("/proc/cpuinfo", "r");

	if (NULL == fp) {
		return TRUE;
	}

	while (!feof(fp)) {
#define CPU_NAME_SIZE 120
		char buffer[CPU_NAME_SIZE];
		char *position_l = NULL;
		char *position_r = NULL;
		if (NULL == fgets(buffer, CPU_NAME_SIZE, fp)) {
			break;
		}
#undef CPU_NAME_SIZE
		position_l = strstr(buffer, "cpu");
		if (NULL != position_l) {
			position_l = strchr(position_l, ':');
			if (NULL == position_l) {
				/* leave cpu_name NULL to denote default case */
				break;
			}
			do {
				++position_l;
			} while (' ' == *position_l);

			position_r = strchr(position_l, '\n');
			if (NULL == position_r) {
				/* leave cpu_name NULL to denote default case */
				break;
			}
			while (' ' == *(position_r - 1)) {
				--position_r;
			}

			/* localize the cpu name */
			cpu_name = position_l;
			*position_r = '\000';
			break;
		}
	}
	fclose(fp);
	if (cpu_name == NULL) return TRUE;

	if (0 == j9_cmdla_strnicmp(cpu_name, "403", 3))              return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "405", 3))              return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "440GP", 5))            return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "601", 3))              return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "603", 3))              return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "7400", 4))             return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "82xx", 4))             return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "750FX", 5))            return FALSE;
	if (0 == j9_cmdla_strnicmp(cpu_name, "604", 3))              return FALSE;
#endif /* AIXPPC */

	return TRUE;
}
#endif /* (AIXPPC || LINUXPPC) && !J9OS_I5 */

void
initializeExecutionModel(J9VMThread *currentThread)
{
	/* Build initial call-out frame. */
	J9SFJNINativeMethodFrame *frame = ((J9SFJNINativeMethodFrame*)currentThread->stackObject->end) - 1;
	frame->method = NULL;
	frame->specialFrameFlags = 0;
	frame->savedCP = NULL;
	frame->savedPC = (U_8*)(UDATA)J9SF_FRAME_TYPE_END_OF_STACK;
	frame->savedA0 = (UDATA*)(UDATA)J9SF_A0_INVISIBLE_TAG;
	currentThread->sp = (UDATA*)frame;
	currentThread->literals = (J9Method*)0;
	currentThread->pc = (U_8*)J9SF_FRAME_TYPE_JNI_NATIVE_METHOD;
	currentThread->arg0EA = (UDATA*)&frame->savedA0;

	/* Initialize JNI function table */
	currentThread->functions = currentThread->javaVM->jniFunctionTable;
}


#if defined(J9VM_THR_ASYNC_NAME_UPDATE)

/**
* Called from the async message handler.
*
* Set the thread name again on systems which limit naming to the current thread.
* Synchronization is guaranteed by the limitation.
*
* The current thread has VM access.
*
* @param currentThread
* @param handlerKey
* @param userData
*/
static void
setThreadNameAsyncHandler(J9VMThread *currentThread, IDATA handlerKey, void *userData)
{
	J9JavaVM *javaVM = (J9JavaVM*)userData;
	j9object_t threadObject = currentThread->threadObject;
	j9object_t threadLock = J9VMJAVALANGTHREAD_LOCK(currentThread, threadObject);
	BOOLEAN shouldUpdateName = TRUE;

#if defined(LINUX)
	{
		/* On linux, don't update the name of the thread that launched the JVM.
		 * This should only effect custom launchers, not the java executable.
		 */
		pid_t pid = getpid();
		UDATA tid = omrthread_get_ras_tid();
		shouldUpdateName = (UDATA)pid != tid;
	}
#endif /* defined(LINUX) */

	if (shouldUpdateName) {
		/* don't allow another thread to change the name while I am reading it */
		threadLock = (j9object_t)objectMonitorEnter(currentThread, threadLock);
		if (J9_OBJECT_MONITOR_ENTER_FAILED(threadLock)) {
			/* We may be out of memory - try again later */
			J9SignalAsyncEvent(javaVM, currentThread, handlerKey);
		} else {
			/* This function accesses threadName without locking it */
			omrthread_set_name(currentThread->osThread, (char*)currentThread->omrVMThread->threadName);
			objectMonitorExit(currentThread, threadLock);
		}
	}
}

#endif /* J9VM_THR_ASYNC_NAME_UPDATE */

static UDATA
parseGlrConfig(J9JavaVM* jvm, char* options)
{
	UDATA result = JNI_OK;
	char* nextOption = NULL;
	char* cursor = options;
	PORT_ACCESS_FROM_JAVAVM(jvm);

	/* parse out each of the options */
	while ((JNI_OK == result) && (strstr(cursor, ",") != NULL)) {
		nextOption = scan_to_delim(PORTLIB, &cursor, ',');
		if (NULL == nextOption) {
			result = JNI_ERR;
		} else {
			result = parseGlrOption(jvm, nextOption);
			j9mem_free_memory(nextOption);
		}
	}
	if (result == JNI_OK) {
		result = parseGlrOption(jvm, cursor);
	}

	return result;
}

static UDATA
parseGlrOption(J9JavaVM* jvm, char* option)
{
	char* valueString = strstr(option, "=");
	UDATA value = 0;

	if (NULL == valueString) {
		return JNI_ERR;
	}

	/* This trims off the leading equal sign. */
	valueString = valueString + 1;

	if (scan_udata(&valueString, &value) != 0) {
		return JNI_ERR;
	}

	/* Thresholds are compared with 16 bit numbers so they never need to be higher than 0x10000. */
	if (value > 0x10000) {
		value = 0x10000;
	}

	if (strncmp(option, "reservedTransitionThreshold=", strlen("reservedTransitionThreshold=")) == 0) {
		jvm->reservedTransitionThreshold = (U_32)value;
		return JNI_OK;
	} else if (strncmp(option, "reservedAbsoluteThreshold=", strlen("reservedAbsoluteThreshold=")) == 0) {
		jvm->reservedAbsoluteThreshold = (U_32)value;
		return JNI_OK;
	} else if (strncmp(option, "minimumReservedRatio=", strlen("minimumReservedRatio=")) == 0) {
		jvm->minimumReservedRatio = (U_32)value;
		return JNI_OK;
	} else if (strncmp(option, "cancelAbsoluteThreshold=", strlen("cancelAbsoluteThreshold=")) == 0) {
		jvm->cancelAbsoluteThreshold = (U_32)value;
		return JNI_OK;
	} else if (strncmp(option, "minimumLearningRatio=", strlen("minimumLearningRatio=")) == 0) {
		jvm->minimumLearningRatio = (U_32)value;
		return JNI_OK;
	}

	return JNI_ERR;
}

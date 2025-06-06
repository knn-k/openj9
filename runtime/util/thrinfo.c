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

/* #define J9VM_DBG */

#include <string.h>
#include "j9.h"
#include "j9cfg.h"
#include "j9protos.h"
#include "omrthread.h"
#include "j9protos.h"
#include "j9consts.h"
#include "util_internal.h"
#include "ut_j9vmutil.h"
#include "monhelp.h"


#define INCLUDE_RAW_MONITORS		TRUE
#define DONT_INCLUDE_RAW_MONITORS	FALSE


#define J9THREAD_MONITOR_FROM_LOCKWORD(lockWord) ((J9ThreadAbstractMonitor *)(J9_INFLLOCK_OBJECT_MONITOR(lockWord)->monitor))

static j9objectmonitor_t getLockWord(J9VMThread *vmThread, j9object_t object);
static UDATA getVMThreadStateHelper(J9VMThread *targetThread, 
		j9object_t *pLockObject, omrthread_monitor_t *pRawLock,
		J9VMThread **pLockOwner, UDATA *pCount, BOOLEAN includeRawMonitors);
static void getInflatedMonitorState(const J9VMThread *targetThread,
		const omrthread_t j9self, const omrthread_state_t *j9state,
		UDATA *vmstate, omrthread_monitor_t *rawLock,
		J9VMThread **lockOwner, UDATA *count);


/**
 * CMVC 135066
 * getVMThreadObjectStatesAll() returns the thread states in a bit mask that is
 * compliant with JVMTI. Blocked/waiting/running J9VMTHREAD_STATE_XXX states can
 * be combined with J9VMTHREAD_STATE_INTERRUPTED or SUSPENDED.
 * However, users of getVMThreadObjectState() expect all the J9VMTHREAD_STATE_XXX
 * states to be mutually exclusive.
 * Therefore, this method calls getVMThreadObjectStatesAll() but:
 * - ignores the J9VMTHREAD_STATE_INTERRUPTED bit
 * - makes J9VMTHREAD_STATE_SUSPENDED mutually exclusive of other states
 * @deprecated
 */
UDATA 
getVMThreadObjectState(J9VMThread *targetThread,
		j9object_t *pLockObject, J9VMThread **pLockOwner, UDATA *pCount)
{
	UDATA rc = getVMThreadObjectStatesAll(targetThread, pLockObject, pLockOwner, pCount);

	/* hack out bits that aren't supported by users of this function */
	rc &= ~J9VMTHREAD_STATE_INTERRUPTED; 
	if (rc & J9VMTHREAD_STATE_SUSPENDED) {
		rc = J9VMTHREAD_STATE_SUSPENDED;
		if (pLockObject) {
			*pLockObject = NULL;
		}
		if (pLockOwner) {
			*pLockOwner = NULL;
		}
		if (pCount) {
			*pCount = 0;
		}
	}
	
	return rc; 
}

UDATA 
getVMThreadObjectStatesAll(J9VMThread *targetThread,
		j9object_t *pLockObject, J9VMThread **pLockOwner, UDATA *pCount)
{
	return getVMThreadStateHelper(targetThread, pLockObject, NULL, pLockOwner,
			pCount, DONT_INCLUDE_RAW_MONITORS);
}

/**
 * CMVC 135066
 * getVMThreadRawStatesAll() returns the thread states in a bit mask that is
 * compliant with JVMTI. Blocked/waiting/running J9VMTHREAD_STATE_XXX states can
 * be combined with J9VMTHREAD_STATE_INTERRUPTED or SUSPENDED.
 * However, users of getVMThreadRawState() expect all the J9VMTHREAD_STATE_XXX
 * states to be mutually exclusive.
 * Therefore, this method calls getVMThreadObjectStatesAll() but:
 * - ignores the J9VMTHREAD_STATE_INTERRUPTED bit
 * - makes J9VMTHREAD_STATE_SUSPENDED mutually exclusive of other states
 * @deprecated
 */
UDATA
getVMThreadRawState(J9VMThread *targetThread,
		j9object_t *pLockObject, omrthread_monitor_t *pRawMonitor, 
		J9VMThread **pLockOwner, UDATA *pCount) 
{
	UDATA rc = getVMThreadRawStatesAll(targetThread, pLockObject, pRawMonitor, pLockOwner, pCount);

	/* hack out bits that aren't supported by users of this function */
	rc &= ~J9VMTHREAD_STATE_INTERRUPTED; 
	if (rc & J9VMTHREAD_STATE_SUSPENDED) {
		rc = J9VMTHREAD_STATE_SUSPENDED;
		if (pLockObject) {
			*pLockObject = NULL;
		}
		if (pRawMonitor) {
			*pRawMonitor = NULL;
		}
		if (pLockOwner) {
			*pLockOwner = NULL;
		}
		if (pCount) {
			*pCount = 0;
		}
	}
	
	return rc; 
}

UDATA
getVMThreadRawStatesAll(J9VMThread *targetThread,
		j9object_t *pLockObject, omrthread_monitor_t *pRawMonitor, 
		J9VMThread **pLockOwner, UDATA *pCount) 
{
	return getVMThreadStateHelper(targetThread, pLockObject, pRawMonitor, pLockOwner,
			pCount, INCLUDE_RAW_MONITORS);
}

/*
 * Determine the state of a thread.
 * Results are undefined if the thread is not suspended.
 * The caller may pass NULL for parameters it is not interested in.
 *
 * If (includeRawMonitors == FALSE), ignore raw monitors (i.e. non-object monitors).
 * Threads that are blocked on raw monitors will be reported as RUNNING.
 *
 * Returns one of:
 *   J9VMTHREAD_STATE_RUNNING
 *   J9VMTHREAD_STATE_BLOCKED
 *   J9VMTHREAD_STATE_WAITING
 * 	 J9VMTHREAD_STATE_WAITING_TIMED
 *   J9VMTHREAD_STATE_PARKED
 *   J9VMTHREAD_STATE_PARKED_TIMED
 *   J9VMTHREAD_STATE_SLEEPING
 *   J9VMTHREAD_STATE_DEAD
 *   J9VMTHREAD_STATE_UNKNOWN
 * And if interrupted or suspended, combined with:
 *   J9VMTHREAD_STATE_INTERRUPTED
 *   J9VMTHREAD_STATE_SUSPENDED
 * 
 * Depending on the thread's state, the returned parameters have slightly
 * different interpretations.
 *
 * UNKNOWN/RUNNING/SLEEPING/DEAD
 * lockObject = NULL
 * rawLock = NULL
 * lockOwner = NULL
 * count = 0
 *
 * PARKED/PARKED_TIMED
 * lockObject = The parkBlocker object specified via LockSupport.park(parkBlocker)
 * rawLock = NULL
 * lockOwner = parkBlocker.getExclusiveOwnerThread(), if parkBlocker is an AbstractOwnableSynchronizer.
 * 		NULL otherwise.
 * count = 0
 *
 * BLOCKED/WAITING/WAITING_TIMED on an OBJECT monitor
 * lockObject = the blocking object
 * rawLock = the fat monitor associated with the blocking object
 * lockOwner = the thread that owns the object monitor
 * count = the lockOwner's recursion count.
 *
 * If the object monitor is not inflated, the owner and count reflect
 * the owner and count of the flat lock, not of the associated fat lock.
 *
 * BLOCKED/WAITING/WAITING_TIMED on a RAW monitor
 * lockObject = NULL
 * rawLock = the raw monitor 
 * lockOwner = the thread that owns the raw monitor
 * count = the lockOwner's recursion count.
 *
 * For BLOCKED/WAITING/WAITING_TIMED threads, lockOwner may be NULL if the owner of the
 * blocking monitor is unattached.
 *
 * count may be inaccurate if the lockOwner thread is not also suspended.
 * 
 * Out-of-process notes:
 *    Input targetThread is a LOCAL pointer to a J9VMThread.
 *    It must be bound to a LOCAL J9JavaVM.
 *    Output *pLockObject, *pRawLock and *pLockOwner are TARGET pointers
 * 
 */
static UDATA
getVMThreadStateHelper(J9VMThread *targetThread,
		j9object_t *pLockObject, omrthread_monitor_t *pRawLock,
		J9VMThread **pLockOwner, UDATA *pCount, 
		BOOLEAN includeRawMonitors)
{
	UDATA vmstate = J9VMTHREAD_STATE_UNKNOWN;
	j9object_t lockObject = NULL;
	omrthread_monitor_t rawLock = NULL;
	J9VMThread *lockOwner = NULL;
	UDATA count = 0;
	UDATA publicFlags;
	omrthread_t j9self;
	omrthread_state_t j9state;
	
	if (targetThread) {
		vmstate = J9VMTHREAD_STATE_RUNNING;
		publicFlags = targetThread->publicFlags;
		j9self = targetThread->osThread;
		/* j9self may be NULL if this function is used by RAS on a corrupt VM */
		
		if (j9self) {
			omrthread_get_state(j9self, &j9state);
		} else {
			memset(&j9state, 0, sizeof(j9state));
		}
		
		if (publicFlags & (J9_PUBLIC_FLAGS_THREAD_BLOCKED | J9_PUBLIC_FLAGS_THREAD_WAITING)) {
			j9objectmonitor_t lockWord;
			
			Assert_VMUtil_true(targetThread->blockingEnterObject != NULL);
	
			lockObject = targetThread->blockingEnterObject;
			lockWord = getLockWord(targetThread, lockObject);
	
			if (J9_LOCK_IS_INFLATED(lockWord)) {
				J9ThreadAbstractMonitor *objmon = getInflatedObjectMonitor(targetThread->javaVM, lockObject, lockWord);

				/*
				 * If the monitor is out-of-line and NULL, then it was never entered,
				 * so the target thread is still runnable.
				 */

				if (objmon) {
					omrthread_t j9owner;
					
					j9owner = objmon->owner;
					count = objmon->count;
					
					if (publicFlags & J9_PUBLIC_FLAGS_THREAD_BLOCKED) {
						if (j9owner && (j9owner != j9self)) {
							/* 
							 * The omrthread may be accessing other raw monitors, but
							 * the vmthread is blocked while the object monitor is
							 * owned by a competing thread.
							 */
							vmstate = J9VMTHREAD_STATE_BLOCKED;
							if (!IS_J9_OBJECT_MONITOR_OWNER_DETACHED(j9owner)) {
								lockOwner = getVMThreadFromOMRThread(targetThread->javaVM, j9owner);
							}
							rawLock = (omrthread_monitor_t)objmon;
						}
					} else {
						if (!j9self) {
							if (publicFlags & J9_PUBLIC_FLAGS_THREAD_TIMED) {
								vmstate = J9VMTHREAD_STATE_WAITING_TIMED;
							} else {
								vmstate = J9VMTHREAD_STATE_WAITING;
							}
							if (!IS_J9_OBJECT_MONITOR_OWNER_DETACHED(j9owner)) {
								lockOwner = getVMThreadFromOMRThread(targetThread->javaVM, j9owner);
							}
							rawLock = (omrthread_monitor_t)objmon;
							
						} else if ((omrthread_monitor_t)objmon == j9state.blocker) {
							getInflatedMonitorState(targetThread, j9self, &j9state,
									&vmstate, &rawLock, &lockOwner, &count);
						}
						/* 
						 * If the omrthread is using a different monitor, it must be for vm access.
						 * So the vmthread is either not waiting yet or already awakened.
						 */
					}
				}

#if JAVA_SPEC_VERSION >= 19
				/* For a J9VMThread with a mounted virtual thread that is waiting on a
				 * lock, the virtual thread object should be returned as the lock object
				 * and the corresponding J9VMThread should be returned as the lock
				 * owner.
				 *
				 * ThreadMXBean documentation states that it does not support VirtualThread
				 * and behaviour on locks, related to VirtualThread, has not been defined.
				 * This code matches the RI's ThreadMXBean API behaviour.
				 */
				if (((J9VMTHREAD_STATE_WAITING == vmstate) || (J9VMTHREAD_STATE_WAITING_TIMED == vmstate))
				&& (NULL != targetThread->currentContinuation)
				) {
					lockObject = targetThread->threadObject;
					lockOwner = targetThread;
				}
#endif /* JAVA_SPEC_VERSION >= 19 */
			} else {
				/* 
				 * Can't wait on an uninflated object monitor, so the thread
				 * must be blocked.
				 */
				Assert_VMUtil_true(publicFlags & J9_PUBLIC_FLAGS_THREAD_BLOCKED);
				
				lockOwner = (J9VMThread *)J9_FLATLOCK_OWNER(lockWord);
	
				if (lockOwner && (lockOwner != targetThread)) {
					count = J9_FLATLOCK_COUNT(lockWord);
					rawLock = (omrthread_monitor_t)monitorTablePeekMonitor(targetThread->javaVM, lockObject);
					vmstate = J9VMTHREAD_STATE_BLOCKED;
				}
			}
			/* 
			 * targetThread may be blocked attempting to reacquire VM access, after 
			 * succeeding to acquire the object monitor. In this case, the returned 
			 * vmstate depends on includeRawMonitors.
			 * includeRawMonitors == FALSE: the vmstate is RUNNING.
			 * includeRawMonitors == TRUE: the vmstate depends on the state of
			 * the omrthread. e.g. The omrthread may be blocked on publicFlagsMutex.
			 */
			
		} else if (publicFlags & J9_PUBLIC_FLAGS_THREAD_PARKED) {
			/* if the osthread is not parked, then the thread is runnable */
			if (!j9self || (j9state.flags & J9THREAD_FLAG_PARKED)) {
				lockObject = targetThread->blockingEnterObject;
				if (publicFlags & J9_PUBLIC_FLAGS_THREAD_TIMED) {
					vmstate = J9VMTHREAD_STATE_PARKED_TIMED;
				} else {
					vmstate = J9VMTHREAD_STATE_PARKED;
				}
#if defined(J9VM_OPT_SIDECAR)
				/* If lockObject is an abstract ownable synchronizer, get its owner. */
				if (lockObject) {
					J9Class *aosClazz;
					J9Class *clazz;
					j9object_t lockOwnerObject = NULL;
					
					aosClazz = J9VMJAVAUTILCONCURRENTLOCKSABSTRACTOWNABLESYNCHRONIZER_OR_NULL(targetThread->javaVM);
					/* skip this step if aosClazz doesn't exist */
					if (aosClazz) {
						clazz = J9OBJECT_CLAZZ(targetThread, lockObject);

						/* PR 80305 : Do not write back to the castClassCache as this code may be running while the GC is unloading the class */
						if (instanceOfOrCheckCastNoCacheUpdate(clazz, aosClazz)) {
							/* Simplify the macro usage by using the _VM version so we do not need to pass in the current thread */
							lockOwnerObject =
								J9VMJAVAUTILCONCURRENTLOCKSABSTRACTOWNABLESYNCHRONIZER_EXCLUSIVEOWNERTHREAD_VM(targetThread->javaVM, lockObject);
							if (lockOwnerObject) {
								lockOwner = (J9VMThread *)J9VMJAVALANGTHREAD_THREADREF_VM(targetThread->javaVM, lockOwnerObject);
							}
						}
					}
				}
#endif /* defined(J9VM_OPT_SIDECAR) */
			}
				
		} else if (publicFlags & J9_PUBLIC_FLAGS_THREAD_SLEEPING) {
			/* if the osthread is not sleeping, then the thread is runnable */
			if (!j9self || (j9state.flags & J9THREAD_FLAG_SLEEPING)) {
				vmstate = J9VMTHREAD_STATE_SLEEPING;
			}

		} else { 
			/* no vmthread flags apply, so go through the omrthread flags */
			if (!j9self) {
				vmstate = J9VMTHREAD_STATE_UNKNOWN;
			} else if (j9state.flags & J9THREAD_FLAG_PARKED) {
				if (j9state.flags & J9THREAD_FLAG_TIMER_SET) {
					vmstate = J9VMTHREAD_STATE_PARKED_TIMED;
				} else {
					vmstate = J9VMTHREAD_STATE_PARKED;
				}
			} else if (j9state.flags & J9THREAD_FLAG_SLEEPING) {
				vmstate = J9VMTHREAD_STATE_SLEEPING;
			} else if (j9state.flags & J9THREAD_FLAG_DEAD) {
				vmstate = J9VMTHREAD_STATE_DEAD;
			} 
		}
		
		if (J9VMTHREAD_STATE_RUNNING == vmstate) {
			if (includeRawMonitors) {
				/* check if the omrthread is blocked/waiting on a raw monitor */
				lockObject = NULL;
				getInflatedMonitorState(targetThread, j9self, &j9state, &vmstate,
						&rawLock, &lockOwner, &count);
			}
		}
		
		/* sanitize irrelevant parameters */
		if ((J9VMTHREAD_STATE_RUNNING == vmstate) 
				|| (J9VMTHREAD_STATE_SUSPENDED == vmstate)
				|| (J9VMTHREAD_STATE_UNKNOWN == vmstate)) {
			lockObject = NULL;
			rawLock = NULL;
			lockOwner = NULL;
			count = 0;
		}
		
		if (rawLock && pLockObject && (!lockObject)) {
			if (((((J9ThreadAbstractMonitor *)rawLock)->flags) & J9THREAD_MONITOR_OBJECT) == J9THREAD_MONITOR_OBJECT) {
				lockObject = (j9object_t)(((J9ThreadAbstractMonitor *)rawLock)->userData);
			}
		}

		/*
		 * Refer to the JVMTI docs for Get Thread State.
		 * INTERRUPTED and SUSPENDED are not mutually exclusive with the other states.
		 */
		/* j9state was zeroed if j9self is NULL */
		if (j9state.flags & J9THREAD_FLAG_INTERRUPTED) {
			vmstate |= J9VMTHREAD_STATE_INTERRUPTED;
		}
		if (j9state.flags & J9THREAD_FLAG_SUSPENDED) {
			vmstate |= J9VMTHREAD_STATE_SUSPENDED;
		}
		if (FALSE == includeRawMonitors) {
			/* 
			 * For compatibility with getVMThreadStatus(), ignore this flag if
			 * raw state is requested. 
			 */
			if (publicFlags & J9_PUBLIC_FLAGS_HALT_THREAD_JAVA_SUSPEND) {
				vmstate |= J9VMTHREAD_STATE_SUSPENDED;
			}
		}
	} /* if (targetThread) */

	if (pLockObject) {
		*pLockObject = lockObject;
	}
	if (pLockOwner) {
		*pLockOwner = lockOwner;
	}
	if (pRawLock) {
		*pRawLock = rawLock;
	}
	if (pCount) {
		*pCount = count;
	}
	return vmstate;
}

static void
getInflatedMonitorState(const J9VMThread *targetThread, const omrthread_t j9self,
		const omrthread_state_t *j9state, UDATA *vmstate,
		omrthread_monitor_t *rawLock, J9VMThread **lockOwner, UDATA *count)
{
	*vmstate = J9VMTHREAD_STATE_RUNNING;

	if (!j9self) {
		*vmstate = J9VMTHREAD_STATE_UNKNOWN;
		
	} else if (j9state->flags & J9THREAD_FLAG_BLOCKED) {
		/* Check for BLOCKED before WAITING to catch waiting threads that
		 * have been notified.
		 */
		if (j9state->owner && (j9state->owner != j9self)) {
			*lockOwner = getVMThreadFromOMRThread(targetThread->javaVM, j9state->owner);
			*count = j9state->count;
			*rawLock = j9state->blocker;
			*vmstate = J9VMTHREAD_STATE_BLOCKED;
		}
	} else if (j9state->flags & J9THREAD_FLAG_WAITING) {
		/* The blocking object of a waiting thread need not be owned. */
		if (j9state->owner != j9self) {
			if (j9state->owner) {
				*lockOwner = getVMThreadFromOMRThread(targetThread->javaVM, j9state->owner);
				*count = j9state->count;
			} else {
				*lockOwner = NULL;
				*count = 0;
			}
			*rawLock = j9state->blocker;
			if (j9state->flags & J9THREAD_FLAG_TIMER_SET) {
				*vmstate = J9VMTHREAD_STATE_WAITING_TIMED;
			} else {
				*vmstate = J9VMTHREAD_STATE_WAITING;
			}
		}
	}
}


/**
 * Get the inflated monitor corresponding to an object, if it exists.
 * The inflated monitor is usually stored in the object lockword, but
 * this function may need to look up the monitor in vm->monitorTable.
 * 
 * This function may block on vm->monitorTableMutex.
 * This function can work out-of-process.
 * 
 * @pre The object monitor must be inflated.
 * 
 * @param[in] vm the JavaVM. For out-of-process: may be a local or target pointer. 
 * vm->monitorTable must be a target value.
 * @param[in] object the object. For out-of-process: a target pointer.
 * @param[in] lockWord The object's lockword.
 * @returns a J9ThreadAbstractMonitor 
 * @retval NULL There is no inflated object monitor.
 * 
 * @see monitorTablePeekMonitor, monitorTablePeek
 */
J9ThreadAbstractMonitor *
getInflatedObjectMonitor(J9JavaVM *vm, j9object_t object, j9objectmonitor_t lockWord)
{
	return J9THREAD_MONITOR_FROM_LOCKWORD(lockWord);
}

static j9objectmonitor_t
getLockWord(J9VMThread *vmThread, j9object_t object)
{
	j9objectmonitor_t lockWord = 0;

	if (LN_HAS_LOCKWORD(vmThread,object)) {
		j9objectmonitor_t *lwEA = J9OBJECT_MONITOR_EA(vmThread, object);
		lockWord = J9_LOAD_LOCKWORD(vmThread, lwEA);
	} else {
		J9ObjectMonitor *objectMonitor = monitorTablePeek(vmThread->javaVM, object);
		if (objectMonitor != NULL){
			lockWord = J9_LOAD_LOCKWORD(vmThread, &objectMonitor->alternateLockword);
		}
	}

	return lockWord;
}

/**
 * Search vm->monitorTable for the inflated monitor corresponding to an object.
 * Similar to monitorTableAt(), but doesn't add the monitor if it isn't found in the hashtable.
 * 
 * This function may block on vm->monitorTableMutex.
 * This function can work out-of-process.
 * 
 * @param[in] vm the JavaVM. For out-of-process: may be a local or target pointer. 
 * vm->monitorTable must be a target value.
 * @param[in] object the object. For out-of-process: a target pointer.
 * @returns the J9ThreadAbstractMonitor from a hashtable entry
 * @retval NULL There is no corresponding monitor in vm->monitorTable.
 * 
 * @see monitorTablePeek
 */
J9ThreadAbstractMonitor *
monitorTablePeekMonitor(J9JavaVM *vm, j9object_t object)
{
	J9ThreadAbstractMonitor *monitor = NULL;
	J9ObjectMonitor *objectMonitor = NULL;
	
	objectMonitor = monitorTablePeek(vm, object);
	if (objectMonitor) {
		monitor = (J9ThreadAbstractMonitor *)objectMonitor->monitor;
	}
	return monitor;
}

/**
 * Search vm->monitorTable for the inflated monitor corresponding to an object.
 * Similar to monitorTableAt(), but doesn't add the monitor if it isn't found in the hashtable.
 * 
 * This function may block on vm->monitorTableMutex.
 * This function can work out-of-process.
 * 
 * @param[in] vm the JavaVM. For out-of-process: may be a local or target pointer. 
 * vm->monitorTable must be a target value.
 * @param[in] targetVMThread	the target J9VMThread
 * @param[in] object the object. For out-of-process: a target pointer.
 * @returns a J9ObjectMonitor from the monitor hashtable
 * @retval NULL There is no corresponding monitor in vm->monitorTable.
 * 
 * @see monitorTablePeekMonitor
 */
J9ObjectMonitor *
monitorTablePeek(J9JavaVM *vm, j9object_t object)
{

	J9ObjectMonitor *monitor = NULL;
	/*
	 * As far as object's hash code is been using for hashing, object must have HASH flag(s) set in object header
	 * however RAS Dump might call this code on random data (in "shared access" mode in case of heap mutation).
	 * This check suppose to protect heap from corruption
	 * (If OBJECT_HEADER_HAS_BEEN_HASHED_IN_CLASS bit happen to be not set in given data hashCode() would set it
	 * and corrupt heap by expected read-only operation. So "sniff" hash bits before and ignore request if hash bit is not set)
	 * If hash bit for object is not set such object can not be found in hash table any way
	 *
	 * For more information see CMVC 179161
	 */
	if (0 != (J9OBJECT_FLAGS_FROM_CLAZZ_VM(vm, object) & (OBJECT_HEADER_HAS_BEEN_HASHED_IN_CLASS | OBJECT_HEADER_HAS_BEEN_MOVED_IN_CLASS))) {
		J9HashTable *monitorTable = NULL;
		omrthread_monitor_t mutex = vm->monitorTableMutex;
		J9ObjectMonitor key_objectMonitor;
		J9ThreadAbstractMonitor key_monitor;

		/* Create a "fake" monitor just to probe the hash-table */
		key_monitor.userData = (UDATA)object;
		key_objectMonitor.monitor = (omrthread_monitor_t) &key_monitor;
		key_objectMonitor.hash = objectHashCode(vm, object);

		omrthread_monitor_enter(mutex);
		if (NULL == monitorTable) {
			UDATA index = key_objectMonitor.hash % (U_32)vm->monitorTableCount;
			monitorTable = vm->monitorTables[index];
		}

		monitor = hashTableFind(monitorTable, &key_objectMonitor);

		omrthread_monitor_exit(mutex);
	}
	return monitor;
}



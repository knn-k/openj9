/*[INCLUDE-IF Sidecar18-SE]*/
/*
 * Copyright IBM Corp. and others 2015
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
 */

package com.ibm.lang.management;

/**
 * The OpenJ9 extension interface to {@link java.lang.management.ThreadMXBean}.  It publishes
 * APIs specific to {@link com.ibm.lang.management}, exposing additional
 * information than what the standard ThreadMXBean does.
 */
public interface ThreadMXBean extends com.sun.management.ThreadMXBean
{
	/**
	 * Fetches an array of the native (operating system assigned) identifiers
	 * corresponding to unique TIDs (as returned by java/lang/Thread.getId()) specified to it.
	 * @param threadIDs An array of thread identifiers that the user wishes to obtain
	 * native thread identifiers for.
	 * @return An array of operating system assigned native thread identifiers. If a thread among the
	 * 			given set of IDs is no longer alive or does not exist, a -1 is set in the corresponding
	 * 			element of the returned array.
	 * @throws IllegalArgumentException is thrown if any of the thread identifiers passed is invalid (&lt;=0).
	/*[IF JAVA_SPEC_VERSION < 24]
	 * @throws SecurityException is thrown if the caller does not have sufficient permissions
	 * (ManagementPermission("monitor"))
	/*[ENDIF] JAVA_SPEC_VERSION >= 24
	 */
	public long[] getNativeThreadIds(long[] threadIDs)
			throws IllegalArgumentException
			/*[IF JAVA_SPEC_VERSION < 24]*/
			, SecurityException
			/*[ENDIF] JAVA_SPEC_VERSION >= 24 */
			;

	/**
	 * Find the native (operating system assigned) thread identifiers corresponding
	 * to a unique TID (as returned by java/lang/Thread.getId()). When querying multiple threadIDs,
	 * consider using getNativeThreadIds(long[]) as it is more efficient than getNativeThreadId().
	 * @param threadId The Java runtime allocated thread identifier.
	 * @return Operating system assigned native thread identifier. If the thread corresponding to the
	 * 			ID is no longer alive or does not exist, -1 is returned.
	 * @throws IllegalArgumentException is thrown if the thread identifier passed is invalid (&lt;=0).
	/*[IF JAVA_SPEC_VERSION < 24]
	 * @throws SecurityException is thrown if the caller does not have sufficient permissions
	 * (ManagementPermission("monitor"))
	/*[ENDIF] JAVA_SPEC_VERSION >= 24
	 */
	public long getNativeThreadId(long threadId)
			throws IllegalArgumentException
			/*[IF JAVA_SPEC_VERSION < 24]*/
			, SecurityException
			/*[ENDIF] JAVA_SPEC_VERSION >= 24 */
			;

	/**
	 * API method that fetches an array of ExtendedThreadInfo objects corresponding to
	 * threads in the virtual machine during the time it is invoked.
	 * Fetches an array of ExtendedThreadInfo objects that provide native thread
	 * identifiers along with java.lang.management.ThreadInfo object representing the thread.
	 * Consider using dumpAllExtendedThreads() in place of dumpAllThreads() as it provides
	 * additional thread identification information in an efficient manner.
	 * @param lockedMonitors
	 * 					boolean indication of whether or not information on all
	 *					currently locked object monitors is to be included in the
	 *					returned array
	 * @param lockedSynchronizers
	 * 					boolean indication of whether or not information on all
	 *					currently locked ownable synchronizers is to be included in
	 *					the returned array
	 * @return  Array of ExtendedThreadInfo objects.
	/*[IF JAVA_SPEC_VERSION < 24]
	 * @throws SecurityException is thrown if the caller does not have sufficient permissions
	 * (ManagementPermission("monitor"))
	/*[ENDIF] JAVA_SPEC_VERSION >= 24
	 * @throws UnsupportedOperationException is thrown if the JVM does not support monitoring
	 * object monitor usage or ownable synchronizer usage, even as it has been specified.
	 * @throws InternalError is thrown in case an error occurs while fetching thread information,
	 * typically, an internal error resulting from an inconsistency in the class library.
	 */
	public ExtendedThreadInfo[] dumpAllExtendedThreads(boolean lockedMonitors, boolean lockedSynchronizers)
		throws
			InternalError,
			/*[IF JAVA_SPEC_VERSION < 24]*/
			SecurityException,
			/*[ENDIF] JAVA_SPEC_VERSION < 24 */
			UnsupportedOperationException;
}

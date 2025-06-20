/*******************************************************************************
 * Copyright IBM Corp. and others 2000
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

#ifndef J9_CODECACHE_INCL
#define J9_CODECACHE_INCL


#ifndef J9_CODECACHE_COMPOSED
#define J9_CODECACHE_COMPOSED
namespace J9 { class CodeCache; }
namespace J9 { typedef CodeCache CodeCacheConnector; }
#endif


#include "env/jittypes.h"
#include "runtime/OMRCodeCache.hpp"
#include "env/IO.hpp"
#include "env/VMJ9.h"

struct J9MemorySegment;
struct J9ClassLoader;

class TR_OpaqueMethodBlock;
namespace TR { class CodeCacheMemorySegment; }
namespace TR { class CodeCacheManager; }

namespace J9
{

class OMR_EXTENSIBLE CodeCache : public OMR::CodeCacheConnector
   {
   TR::CodeCache *self();

public:
   CodeCache() { }

   /**
    * @brief Initialize an allocated CodeCache object
    *
    * @param[in] manager : the TR::CodeCacheManager
    * @param[in] codeCacheSegment : the code cache memory segment that has been allocated
    * @param[in] allocatedCodeCacheSizeInBytes : the size (in bytes) of the allocated code cache
    * @param[in] kind : the kind of this code cache
    *
    * @return true on a successful initialization; false otherwise.
    */
   bool                       initialize(TR::CodeCacheManager *manager,
                                         TR::CodeCacheMemorySegment *codeCacheSegment,
                                         size_t allocatedCodeCacheSizeInBytes,
                                         TR::CodeCacheKind kind);

   static TR::CodeCache *     allocate(TR::CodeCacheManager *cacheManager, size_t segmentSize, int32_t reservingCompThreadID, TR::CodeCacheKind kind);

   // Code Cache Reclamation
   void                       addFreeBlock(OMR::FaintCacheBlock *block);

   void                       dumpCodeCache();

   OMR::CodeCacheMethodHeader * addFreeBlock(void *voidMetaData);

   bool                       addUnresolvedMethod(void *constPool, int32_t constPoolIndex);
   bool                       addResolvedMethod(TR_OpaqueMethodBlock *method);
   int32_t                    reserveUnresolvedTrampoline(void *cp, int32_t cpIndex);

   void                       onClassRedefinition(TR_OpaqueMethodBlock *oldMethod, TR_OpaqueMethodBlock *newMethod);
   void                       onClassUnloading(J9ClassLoader *loaderPtr);
   void                       onFSDDecompile();

   void                       resolveHashEntry(OMR::CodeCacheHashEntry *entry, TR_OpaqueMethodBlock *method);

   void                       reportCodeLoadEvents();

   TR::CodeCacheMemorySegment* trj9segment();
   J9MemorySegment *          j9segment();

   void                       generatePerfToolEntries(TR::FILE *file);

   void                       adjustTrampolineReservation(TR_OpaqueMethodBlock *method,
                                                          void *cp,
                                                          int32_t cpIndex);

   OMR::CodeCacheHashEntry *  findUnresolvedMethod(void *constPool, int32_t constPoolIndex);

  /**
   * @brief Restore warmCodeAlloc/coldCodeAlloc and trampoline pointers to their initial positions
   */
   void resetCodeCache();

   int32_t disclaim(TR::CodeCacheManager *manager, bool canDisclaimOnSwap);

   private:
   /**
    * @brief Restore trampoline pointers to their initial positions
    */
   void resetTrampolines();
   /**
   * @brief Remember the initial position of warmCodeAlloc/coldCodeAlloc pointers
   */
   void setInitialAllocationPointers();
   /**
   * @brief Restore warmCodeAlloc/coldCodeAlloc pointers to their initial positions
   */
   void resetAllocationPointers();

   uint8_t *_warmCodeAllocBase; // used to reset the allocation pointers to initial values
   uint8_t *_coldCodeAllocBase;
#ifdef LINUX
   uint8_t *_smallPageAreaStart; // used for code cache disclaiming to remember where the small page area starts/ends
   uint8_t *_smallPageAreaEnd;
#endif
   };


} // namespace J9



/*****************************************************************************
 *  API Prototypes
 */

extern "C"
   {

   OMR::CodeCacheTrampolineCode * mcc_replaceTrampoline(TR_OpaqueMethodBlock *method,
                                                      void *callSite,
                                                      void *oldTrampoline,
                                                      void *oldTargetPC,
                                                      void *newTargetPC,
                                                      bool needSync);

   }

#endif /* !defined(J9_CODECACHE_INCL) */

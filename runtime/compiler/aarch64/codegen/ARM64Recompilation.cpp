/*******************************************************************************
 * Copyright IBM Corp. and others 2019
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

#include "codegen/ARM64Recompilation.hpp"
#include "codegen/ARM64RecompilationSnippet.hpp"
#include "codegen/CodeGenerator.hpp"
#include "codegen/GenerateInstructions.hpp"
#include "env/j9method.h"

TR_ARM64Recompilation::TR_ARM64Recompilation(TR::Compilation *comp)
    : TR::Recompilation(comp)
{
    _countingSupported = true;

    setupMethodInfo();
}

TR::Recompilation *TR_ARM64Recompilation::allocate(TR::Compilation *comp)
{
    if (comp->isRecompilationEnabled()) {
        return new (comp->trHeapMemory()) TR_ARM64Recompilation(comp);
    }

    return NULL;
}

TR_PersistentMethodInfo *TR_ARM64Recompilation::getExistingMethodInfo(TR_ResolvedMethod *method)
{
    TR_PersistentJittedBodyInfo *bodyInfo = (static_cast<TR_ResolvedJ9Method *>(method))->getExistingJittedBodyInfo();
    return bodyInfo ? bodyInfo->getMethodInfo() : NULL;
}

TR::Instruction *TR_ARM64Recompilation::generatePrePrologue()
{
    // If in Full Speed Debug, allow to go through
    if (!couldBeCompiledAgain() && !_compilation->getOption(TR_FullSpeedDebug))
        return NULL;

    // x9 may contain the vtable offset, and must be preserved here
    // See PicBuilder.spp and Recompilation.spp
    TR::Instruction *cursor = NULL;
    TR::Machine *machine = cg()->machine();
    TR::Register *x8 = machine->getRealRegister(TR::RealRegister::x8);
    TR::Register *lr = machine->getRealRegister(TR::RealRegister::lr); // Link Register
    TR::Register *xzr = machine->getRealRegister(TR::RealRegister::xzr); // zero register
    TR::Node *firstNode = comp()->getStartTree()->getNode();
    TR::SymbolReference *recompileMethodSymRef
        = cg()->symRefTab()->findOrCreateRuntimeHelper(TR_ARM64samplingRecompileMethod);
    TR_PersistentJittedBodyInfo *info = getJittedBodyInfo();

    // Force creation of switch to interpreter preprologue if in Full Speed Debug
    if (cg()->mustGenerateSwitchToInterpreterPrePrologue()) {
        cursor = cg()->generateSwitchToInterpreterPrePrologue(cursor, firstNode);
    }

    if (useSampling() && couldBeCompiledAgain()) {
        // x8 must contain the saved LR; see Recompilation.spp
        // cannot use generateMovInstruction() here
        cursor = new (cg()->trHeapMemory())
            TR::ARM64Trg1Src2Instruction(TR::InstOpCode::orrx, firstNode, x8, xzr, lr, cursor, cg());
        cursor = generateImmSymInstruction(cg(), TR::InstOpCode::bl, firstNode,
            (uintptr_t)recompileMethodSymRef->getMethodAddress(),
            new (cg()->trHeapMemory()) TR::RegisterDependencyConditions(0, 0, cg()->trMemory()), recompileMethodSymRef,
            NULL, cursor);
        cursor = generateRelocatableImmInstruction(cg(), TR::InstOpCode::dd, firstNode, (uintptr_t)info,
            TR_BodyInfoAddress, cursor);

        // space for preserving original jitEntry instruction
        cursor = generateImmInstruction(cg(), TR::InstOpCode::dd, firstNode, 0, cursor);
    }

    return cursor;
}

TR::Instruction *TR_ARM64Recompilation::generatePrologue(TR::Instruction *cursor)
{
#if 1
    TR::Recompilation *recomp = comp()->getRecompilationInfo();
    if (!recomp->useSampling()) {
        // counting recompilation
        TR_UNIMPLEMENTED();
    }
#else
    if (couldBeCompiledAgain()) {
        // counting recompilation
        // x9 may contain the vtable offset, and must be preserved here
        // See PicBuilder.spp and Recompilation.spp
        TR::Machine *machine = cg()->machine();
        TR::Register *x8 = machine->getRealRegister(TR::RealRegister::x8);
        TR::Register *x10 = machine->getRealRegister(TR::RealRegister::x10);
        TR::Register *lr = machine->getRealRegister(TR::RealRegister::lr); // Link Register
        TR::Register *xzr = machine->getRealRegister(TR::RealRegister::xzr); // zero register
        TR::Node *firstNode = _compilation->getStartTree()->getNode();
        TR::LabelSymbol *snippetLabel = generateLabelSymbol(cg());

        // 4 instructions to load the address in x10: Need to be fixed length
        intptr_t addr = (intptr_t)getCounterAddress();
        cursor = generateTrg1ImmInstruction(cg(), TR::InstOpCode::movzx, firstNode, x10,
                                            (addr & 0xFFFF), cursor);
        cursor = generateTrg1ImmInstruction(cg(), TR::InstOpCode::movkx, firstNode, x10,
                                            (((addr >> 16) & 0xFFFF) | TR::MOV_LSL16), cursor);
        cursor = generateTrg1ImmInstruction(cg(), TR::InstOpCode::movkx, firstNode, x10,
                                            (((addr >> 32) & 0xFFFF) | TR::MOV_LSL32), cursor);
        cursor = generateTrg1ImmInstruction(cg(), TR::InstOpCode::movkx, firstNode, x10,
                                            (((addr >> 48) & 0xFFFF) | TR::MOV_LSL48), cursor);

        cursor = generateTrg1MemInstruction(cg(), TR::InstOpCode::ldrimmw, firstNode, x8,
                                            TR::MemoryReference::createWithDisplacement(cg(), x10, 0), cursor);

        if (isProfilingCompilation()) {
            // This only applies to JitProfiling, as JProfiling uses sampling
            TR_ASSERT(_compilation->getProfilingMode() == JitProfiling, "JProfiling should not use counting mechanism to trip recompilation");

            cursor = generateCompareImmInstruction(cg(), firstNode, x8, 0, false, cursor); // false is for 32-bit comparison
            // This is just padding for consistent code length
            cursor = generateInstruction(cg(), TR::InstOpCode::nop, firstNode, cursor);
        } else {
            cursor = generateTrg1Src1ImmInstruction(cg(), TR::InstOpCode::subsimmw, firstNode, x8, x8, 1, cursor);
            cursor = generateMemSrc1Instruction(cg(), TR::InstOpCode::strimmw, firstNode,
                                                TR::MemoryReference::createWithDisplacement(cg(), x10, 0), x8, cursor);
        }

        // x8 must contain the saved LR; see Recompilation.spp
        cursor = generateTrg1Src2Instruction(cg(), TR::InstOpCode::orrx, firstNode, x8, xzr, lr, cursor);
        // This instruction is replaced after successful recompilation
        cursor = generateInstruction(cg(), TR::InstOpCode::nop, firstNode, cursor);
        // This instruction checks the flag set by subsimmw or by CompareImm above
        cursor = generateConditionalBranchInstruction(cg(), firstNode, snippetLabel, TR::CC_LT, cursor);
        TR::Snippet *snippet = new (cg()->trHeapMemory()) TR::ARM64RecompilationSnippet(snippetLabel, cursor, cg());
        cg()->addSnippet(snippet);
    }
#endif
    return cursor;
}

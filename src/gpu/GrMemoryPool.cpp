/*
 * Copyright 2012 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "include/private/SkMalloc.h"
#include "src/gpu/GrMemoryPool.h"
#include "src/gpu/ops/GrOp.h"
#ifdef SK_DEBUG
    #include <atomic>
#endif

#ifdef SK_DEBUG
    #define VALIDATE this->validate()
#else
    #define VALIDATE
#endif

std::unique_ptr<GrMemoryPool> GrMemoryPool::Make(size_t preallocSize, size_t minAllocSize) {
    preallocSize = std::max(preallocSize, kMinAllocationSize);
    static constexpr size_t kPoolSize = GrAlignTo(sizeof(GrMemoryPool), kAlignment);
    size_t size = kPoolSize + preallocSize;
    void* mem = operator new(size);
    void* preallocStart = static_cast<char*>(mem) + kPoolSize;
    return std::unique_ptr<GrMemoryPool>(
            new (mem) GrMemoryPool(preallocStart, preallocSize, minAllocSize));
}

GrMemoryPool::GrMemoryPool(void* preallocStart, size_t preallocSize, size_t minAllocSize) {
    SkDEBUGCODE(fAllocationCnt = 0);
    SkDEBUGCODE(fAllocBlockCnt = 0);

    minAllocSize = std::max(minAllocSize, kMinAllocationSize);

    fMinAllocSize = minAllocSize;
    fSize = 0;

    fHead = InitBlock(preallocStart, preallocSize);
    fTail = fHead;
    fHead->fNext = nullptr;
    fHead->fPrev = nullptr;
    VALIDATE;
};

GrMemoryPool::~GrMemoryPool() {
    VALIDATE;
#ifdef SK_DEBUG
    int i = 0;
    int n = fAllocatedIDs.count();
    fAllocatedIDs.foreach([&i, n] (int32_t id) {
        if (++i == 1) {
            SkDebugf("Leaked IDs (in no particular order): %d", id);
        } else if (i < 11) {
            SkDebugf(", %d%s", id, (n == i ? "\n" : ""));
        } else if (i == 11) {
            SkDebugf(", ...\n");
        }
    });
#endif
    SkASSERT(0 == fAllocationCnt);
    SkASSERT(fHead == fTail);
    SkASSERT(0 == fHead->fLiveCount);
    SkASSERT(kAssignedMarker == fHead->fBlockSentinal);
};

void* GrMemoryPool::allocate(size_t size) {
    VALIDATE;
    size += kPerAllocPad;
    size = GrAlignTo(size, kAlignment);
    if (fTail->fFreeSize < size) {
        size_t blockSize = size + kHeaderSize;
        blockSize = std::max(blockSize, fMinAllocSize);
        BlockHeader* block = CreateBlock(blockSize);

        block->fPrev = fTail;
        block->fNext = nullptr;
        SkASSERT(nullptr == fTail->fNext);
        fTail->fNext = block;
        fTail = block;
        fSize += block->fSize;
        SkDEBUGCODE(++fAllocBlockCnt);
    }
    SkASSERT(kAssignedMarker == fTail->fBlockSentinal);
    SkASSERT(fTail->fFreeSize >= size);
    intptr_t ptr = fTail->fCurrPtr;
    // We stash a pointer to the block header, just before the allocated space,
    // so that we can decrement the live count on delete in constant time.
    AllocHeader* allocData = reinterpret_cast<AllocHeader*>(ptr);
    SkDEBUGCODE(allocData->fSentinal = kAssignedMarker);
    SkDEBUGCODE(allocData->fID = []{
        static std::atomic<int32_t> nextID{1};
        return nextID++;
    }());
    // You can set a breakpoint here when a leaked ID is allocated to see the stack frame.
    SkDEBUGCODE(fAllocatedIDs.add(allocData->fID));
    allocData->fHeader = fTail;
    ptr += kPerAllocPad;
    fTail->fPrevPtr = fTail->fCurrPtr;
    fTail->fCurrPtr += size;
    fTail->fFreeSize -= size;
    fTail->fLiveCount += 1;
    SkDEBUGCODE(++fAllocationCnt);
    VALIDATE;
    return reinterpret_cast<void*>(ptr);
}

void GrMemoryPool::release(void* p) {
    VALIDATE;
    intptr_t ptr = reinterpret_cast<intptr_t>(p) - kPerAllocPad;
    AllocHeader* allocData = reinterpret_cast<AllocHeader*>(ptr);
    SkASSERT(kAssignedMarker == allocData->fSentinal);
    SkDEBUGCODE(allocData->fSentinal = kFreedMarker);
    SkDEBUGCODE(fAllocatedIDs.remove(allocData->fID));
    BlockHeader* block = allocData->fHeader;
    SkASSERT(kAssignedMarker == block->fBlockSentinal);
    if (1 == block->fLiveCount) {
        // the head block is special, it is reset rather than deleted
        if (fHead == block) {
            fHead->fCurrPtr = reinterpret_cast<intptr_t>(fHead) + kHeaderSize;
            fHead->fLiveCount = 0;
            fHead->fFreeSize = fHead->fSize - kHeaderSize;
        } else {
            BlockHeader* prev = block->fPrev;
            BlockHeader* next = block->fNext;
            SkASSERT(prev);
            prev->fNext = next;
            if (next) {
                next->fPrev = prev;
            } else {
                SkASSERT(fTail == block);
                fTail = prev;
            }
            fSize -= block->fSize;
            DeleteBlock(block);
            SkDEBUGCODE(fAllocBlockCnt--);
        }
    } else {
        --block->fLiveCount;
        // Trivial reclaim: if we're releasing the most recent allocation, reuse it
        if (block->fPrevPtr == ptr) {
            block->fFreeSize += (block->fCurrPtr - block->fPrevPtr);
            block->fCurrPtr = block->fPrevPtr;
        }
    }
    SkDEBUGCODE(--fAllocationCnt);
    VALIDATE;
}

GrMemoryPool::BlockHeader* GrMemoryPool::CreateBlock(size_t blockSize) {
    blockSize = std::max(blockSize, kHeaderSize);
    return InitBlock(sk_malloc_throw(blockSize), blockSize);
}

auto GrMemoryPool::InitBlock(void* mem, size_t blockSize) -> BlockHeader* {
    SkASSERT(!(reinterpret_cast<intptr_t>(mem) % kAlignment));
    auto block = reinterpret_cast<BlockHeader*>(mem);
    SkDEBUGCODE(block->fBlockSentinal = kAssignedMarker);
    block->fLiveCount = 0;
    block->fFreeSize = blockSize - kHeaderSize;
    block->fCurrPtr = reinterpret_cast<intptr_t>(block) + kHeaderSize;
    block->fPrevPtr = 0; // gcc warns on assigning nullptr to an intptr_t.
    block->fSize = blockSize;
    return block;
}

void GrMemoryPool::DeleteBlock(BlockHeader* block) {
    SkASSERT(kAssignedMarker == block->fBlockSentinal);
    SkDEBUGCODE(block->fBlockSentinal = kFreedMarker); // FWIW
    sk_free(block);
}

void GrMemoryPool::validate() {
#ifdef SK_DEBUG
    BlockHeader* block = fHead;
    BlockHeader* prev = nullptr;
    SkASSERT(block);
    int allocCount = 0;
    do {
        SkASSERT(kAssignedMarker == block->fBlockSentinal);
        allocCount += block->fLiveCount;
        SkASSERT(prev == block->fPrev);
        if (prev) {
            SkASSERT(prev->fNext == block);
        }

        intptr_t b = reinterpret_cast<intptr_t>(block);
        size_t ptrOffset = block->fCurrPtr - b;
        size_t totalSize = ptrOffset + block->fFreeSize;
        intptr_t userStart = b + kHeaderSize;

        SkASSERT(!(b % kAlignment));
        SkASSERT(!(totalSize % kAlignment));
        SkASSERT(!(block->fCurrPtr % kAlignment));
        if (fHead != block) {
            SkASSERT(block->fLiveCount);
            SkASSERT(totalSize >= fMinAllocSize);
        } else {
            SkASSERT(totalSize == block->fSize);
        }
        if (!block->fLiveCount) {
            SkASSERT(ptrOffset ==  kHeaderSize);
            SkASSERT(userStart == block->fCurrPtr);
        } else {
            AllocHeader* allocData = reinterpret_cast<AllocHeader*>(userStart);
            SkASSERT(allocData->fSentinal == kAssignedMarker ||
                     allocData->fSentinal == kFreedMarker);
            SkASSERT(block == allocData->fHeader);
        }

        prev = block;
    } while ((block = block->fNext));
    SkASSERT(allocCount == fAllocationCnt);
    SkASSERT(fAllocationCnt == fAllocatedIDs.count());
    SkASSERT(prev == fTail);
    SkASSERT(fAllocBlockCnt != 0 || fSize == 0);
#endif
}

////////////////////////////////////////////////////////////////////////////////////////

static constexpr size_t kOpPoolSize = GrAlignTo(sizeof(GrOpMemoryPool), GrMemoryPool::kAlignment);

GrOpMemoryPool::~GrOpMemoryPool() { this->pool()->~GrMemoryPool(); }

std::unique_ptr<GrOpMemoryPool> GrOpMemoryPool::Make(size_t preallocSize, size_t minAllocSize) {
    preallocSize = std::max(preallocSize, GrMemoryPool::kMinAllocationSize);
    static constexpr size_t kOpPoolSize =
            GrAlignTo(sizeof(GrOpMemoryPool), GrMemoryPool::kAlignment);
    static constexpr size_t kPoolSize = GrAlignTo(sizeof(GrMemoryPool), GrMemoryPool::kAlignment);
    size_t size = kOpPoolSize + kPoolSize + preallocSize;
    void* mem = operator new(size);
    void* memPoolPtr = static_cast<char*>(mem) + kOpPoolSize;
    void* preallocStart = static_cast<char*>(mem) + kOpPoolSize + kPoolSize;
    new (memPoolPtr) GrMemoryPool(preallocStart, preallocSize, minAllocSize);
    return std::unique_ptr<GrOpMemoryPool>(new (mem) GrOpMemoryPool());
}

void GrOpMemoryPool::release(std::unique_ptr<GrOp> op) {
    GrOp* tmp = op.release();
    SkASSERT(tmp);
    tmp->~GrOp();
    this->pool()->release(tmp);
}

GrMemoryPool* GrOpMemoryPool::pool() const {
    auto addr = reinterpret_cast<const char*>(this) + kOpPoolSize;
    return reinterpret_cast<GrMemoryPool*>(const_cast<char*>(addr));
}

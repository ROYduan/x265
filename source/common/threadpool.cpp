/*****************************************************************************
 * Copyright (C) 2013-2020 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
 *          Min Chen <chenm003@163.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at license @ s265.com
 *****************************************************************************/

#include "common.h"
#include "threadpool.h"
#include "threading.h"

#include <new>

#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7
#include <winnt.h>
#endif

#if X86_64

#ifdef __GNUC__

#define SLEEPBITMAP_CTZ(id, x)     id = (unsigned long)__builtin_ctzll(x)
#define SLEEPBITMAP_OR(ptr, mask)  __sync_fetch_and_or(ptr, mask)
#define SLEEPBITMAP_AND(ptr, mask) __sync_fetch_and_and(ptr, mask)

#elif defined(_MSC_VER)

#define SLEEPBITMAP_CTZ(id, x)     _BitScanForward64(&id, x)
#define SLEEPBITMAP_OR(ptr, mask)  InterlockedOr64((volatile LONG64*)ptr, (LONG)mask)
#define SLEEPBITMAP_AND(ptr, mask) InterlockedAnd64((volatile LONG64*)ptr, (LONG)mask)

#endif // ifdef __GNUC__

#else

/* use 32-bit primitives defined in threading.h */
#define SLEEPBITMAP_CTZ CTZ
#define SLEEPBITMAP_OR  ATOMIC_OR
#define SLEEPBITMAP_AND ATOMIC_AND

#endif

/* TODO FIX: Macro __MACH__ ideally should be part of MacOS definition, but adding to Cmake
   behaving is not as expected, need to fix this. */

#if MACOS && __MACH__
#include <sys/param.h>
#include <sys/sysctl.h>
#endif
#if HAVE_LIBNUMA
#include <numa.h>
#endif
#if defined(_MSC_VER)
# define strcasecmp _stricmp
#endif

#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7
const uint64_t m1 = 0x5555555555555555; //binary: 0101...
const uint64_t m2 = 0x3333333333333333; //binary: 00110011..
const uint64_t m3 = 0x0f0f0f0f0f0f0f0f; //binary:  4 zeros,  4 ones ...
const uint64_t h01 = 0x0101010101010101; //the sum of 256 to the power of 0,1,2,3...

static int popCount(uint64_t x)
{
    x -= (x >> 1) & m1;
    x = (x & m2) + ((x >> 2) & m2);
    x = (x + (x >> 4)) & m3;
    return (x * h01) >> 56;
}
#endif

namespace S265_NS {
// s265 private namespace

class WorkerThread : public Thread
{
private:

    ThreadPool&  m_pool;
    int          m_id; //线程id 此ID表明其在某个pool中的位置
    Event        m_wakeEvent;

    WorkerThread& operator =(const WorkerThread&);

public:

    JobProvider*     m_curJobProvider;//每个work线程都有自己的当前’领导‘给其分配任务
    BondedTaskGroup* m_bondMaster;

    WorkerThread(ThreadPool& pool, int id) : m_pool(pool), m_id(id) {}
    virtual ~WorkerThread() {}

    void threadMain();// 子类WorkerThread的threadMain 覆盖基类thread 的 threadMain
    void awaken()           { m_wakeEvent.trigger(); } // 调用 pthread_cond_signal(&m_cond); 唤醒一个被阻塞的线程
};

// 线程函数
void WorkerThread::threadMain()
{
    THREAD_NAME("Worker", m_id);

#if _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#else
    __attribute__((unused)) int val = nice(10);
#endif

    m_pool.setCurrentThreadAffinity();

    sleepbitmap_t idBit = (sleepbitmap_t)1 << m_id;
    m_curJobProvider = m_pool.m_jpTable[0];//线程启动时给的初始值，线程的默认领导是 线程池里面的第一个’领导‘
    m_bondMaster = NULL;//初始值为null

    SLEEPBITMAP_OR(&m_curJobProvider->m_ownerBitmap, idBit); // 将该线程加入其所属的jobprovider(‘领导’)的owerBitmap中进行管理
    SLEEPBITMAP_OR(&m_pool.m_sleepBitmap, idBit);// 将该线程加入其所属pool里面的sleepBitmap 管理
    m_wakeEvent.wait();// 阻塞 等待m_wakeEvent.trigger信号触发继续

    while (m_pool.m_isActive)//该线程池中的线程是否已经都起来了 每个worker 共用其所在的pool 的m_isActive
    {
        if (m_bondMaster)//当前work线程是否已经绑定了需要执行的任务
        {
            // 注意 processTasks 在完成一个任务后，发现如果还有job没有做完，会接着再领一个任务做
            m_bondMaster->processTasks(m_id);// 通过基类的虚函数接口调用派生类自己rewrite的processTasks
            m_bondMaster->m_exitedPeerCount.incr();//该线程需要做的任务都完成了可以退出了，exitedPeerCount + 1
            m_bondMaster = NULL;//完成任务后，本线程剥离 m_bondMaster
        }

        do
        {
            /* do pending work for current job provider */
            // 多态 调用派生类的方法 一类是 lookahead  一类是 wavefront
            m_curJobProvider->findJob(m_id);

            /* if the current job provider still wants help, only switch to a
             * higher priority provider (lower slice type). Else take the first
             * available job provider with the highest priority */
            // 是否当前的领导的工作需要帮忙，如果需要，先记录当前领导的工作的优先级
            int curPriority = (m_curJobProvider->m_helpWanted) ? m_curJobProvider->m_sliceType :
                                                                 INVALID_SLICE_PRIORITY + 1;
            int nextProvider = -1;
            //遍历该线程所属线程池里面的 ‘领导们（jobprovider）’
            for (int i = 0; i < m_pool.m_numProviders; i++)
            {
                //找到那些还有任务没有做完需要帮忙的’领导‘,并且 找出具有最紧急任务要帮做的’领导‘, curPriority 值越小优先级越高
                if (m_pool.m_jpTable[i]->m_helpWanted &&
                    m_pool.m_jpTable[i]->m_sliceType < curPriority)
                {
                    nextProvider = i;
                    curPriority = m_pool.m_jpTable[i]->m_sliceType;
                }
            }
            //找到了具有最紧急任务要做的领导，并且该领导和自己的当前领导不一致时
            if (nextProvider != -1 && m_curJobProvider != m_pool.m_jpTable[nextProvider])
            {
                // 在自己当前的领导管理的线程中，去掉‘自己线程’
                SLEEPBITMAP_AND(&m_curJobProvider->m_ownerBitmap, ~idBit);
                // 将自己的领导设为新的领导
                m_curJobProvider = m_pool.m_jpTable[nextProvider];
                // 把自己纳入新领导管理的名单中
                SLEEPBITMAP_OR(&m_curJobProvider->m_ownerBitmap, idBit);
            }
        }
        while (m_curJobProvider->m_helpWanted);

        /* While the worker sleeps, a job-provider or bond-group may acquire this
         * worker's sleep bitmap bit. Once acquired, that thread may modify 
         * m_bondMaster or m_curJobProvider, then waken the thread */
        SLEEPBITMAP_OR(&m_pool.m_sleepBitmap, idBit);//该线程现在已经完成了所需要做的工作,重新归入sleep状态
        m_wakeEvent.wait();//当次任务已经完成,进入挂起等待被唤起
    }

    SLEEPBITMAP_OR(&m_pool.m_sleepBitmap, idBit);// 如果线程池已经被标记停止工作，则该线程需要重新归入sleep状态
}

// 从线程池中唤醒一个sleep状态的线程继续执行
void JobProvider::tryWakeOne()
{
    //优先从’领导‘ 自己管理的线程m_ownerBitmap 中找到一个sleep 状态的线程
    int id = m_pool->tryAcquireSleepingThread(m_ownerBitmap, ALL_POOL_THREADS);
    if (id < 0)//如果没有找到可用的线程，则标记自己需要帮助，然后返回
    {
        m_helpWanted = true; //‘领导’有个任务 没有找到可用线程派发出去，所以标记为需要帮助
        return;
    }

    WorkerThread& worker = m_pool->m_workers[id];// 取出该线程
    //如果该线程的当下领导不是 ’自己‘,则改为自己
    if (worker.m_curJobProvider != this) /* poaching */
    {
        sleepbitmap_t bit = (sleepbitmap_t)1 << id;
        //先将该worker原来的jobprovider“领导”管理的线程中除名
        SLEEPBITMAP_AND(&worker.m_curJobProvider->m_ownerBitmap, ~bit);
        // 更改 领导 为 自己
        worker.m_curJobProvider = this;
        // 将对应的bit 加入到 自己管理的bitmap
        SLEEPBITMAP_OR(&worker.m_curJobProvider->m_ownerBitmap, bit);
    }
    worker.awaken(); // 唤醒 worker 继续, m_wakeEvent.trigger
}

int ThreadPool::tryAcquireSleepingThread(sleepbitmap_t firstTryBitmap, sleepbitmap_t secondTryBitmap)
{
    unsigned long id;

    // 首先从firstTryBitmap 中去得首个处于sleep状态的线程对应的id
    sleepbitmap_t masked = m_sleepBitmap & firstTryBitmap;
    while (masked)
    {
        SLEEPBITMAP_CTZ(id, masked);

        sleepbitmap_t bit = (sleepbitmap_t)1 << id;
        // 如果对应id的bit对应的线程目前处于sleep状态,则直接清零线程的sleep状态，将改id返回
        if (SLEEPBITMAP_AND(&m_sleepBitmap, ~bit) & bit)// SLEEPBITMAP_AND （原子操作) 先返回m_sleepBitmap 的原始值，再做and操作
            return (int)id;

        masked = m_sleepBitmap & firstTryBitmap;
    }
    // 如果没有则继续从 secondTryBitmap 寻找sleep状态的线程id
    masked = m_sleepBitmap & secondTryBitmap;
    while (masked)
    {
        SLEEPBITMAP_CTZ(id, masked);

        sleepbitmap_t bit = (sleepbitmap_t)1 << id;
        if (SLEEPBITMAP_AND(&m_sleepBitmap, ~bit) & bit)//  清零线程的sleep状态，将改id返回
            return (int)id;

        masked = m_sleepBitmap & secondTryBitmap;
    }

    return -1;
}

int ThreadPool::tryBondPeers(int maxPeers, sleepbitmap_t peerBitmap, BondedTaskGroup& master)
{
    int bondCount = 0;
    do
    {
        // 找到一个sleep 状态的线程
        int id = tryAcquireSleepingThread(peerBitmap, 0);
        if (id < 0)
            return bondCount;

        m_workers[id].m_bondMaster = &master;//奖此线程bond to 任务提供者
        m_workers[id].awaken();// 唤醒线程执行master->processTasks 多态形式，BondedTaskGroup 是一个基类，这里通过虚函数调用不同派生类自己实现的 processTasks
        bondCount++;
    }
    while (bondCount < maxPeers);

    return bondCount;
}
ThreadPool* ThreadPool::allocThreadPools(s265_param* p, int& numPools, bool isThreadsReserved)
{
    enum { MAX_NODE_NUM = 127 };
    int cpusPerNode[MAX_NODE_NUM + 1];
    int threadsPerPool[MAX_NODE_NUM + 2];
    uint64_t nodeMaskPerPool[MAX_NODE_NUM + 2];
    int totalNumThreads = 0;

    memset(cpusPerNode, 0, sizeof(cpusPerNode));
    memset(threadsPerPool, 0, sizeof(threadsPerPool));
    memset(nodeMaskPerPool, 0, sizeof(nodeMaskPerPool));

   // 获取有多少个numanode 节点(每个节点对应一个颗物理cpu)
    int numNumaNodes = S265_MIN(getNumaNodeCount(), MAX_NODE_NUM);
    bool bNumaSupport = false;

#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7 
    bNumaSupport = true;
#elif HAVE_LIBNUMA
    bNumaSupport = numa_available() >= 0;
#endif


#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7
    PGROUP_AFFINITY groupAffinityPointer = new GROUP_AFFINITY;
    for (int i = 0; i < numNumaNodes; i++)
    {
        GetNumaNodeProcessorMaskEx((UCHAR)i, groupAffinityPointer);
        cpusPerNode[i] = popCount(groupAffinityPointer->Mask);
    }
    delete groupAffinityPointer;
#elif HAVE_LIBNUMA
    if (bNumaSupport)
    {
        struct bitmask* bitMask = numa_allocate_cpumask();
        for (int i = 0; i < numNumaNodes; i++)
        {
            int ret = numa_node_to_cpus(i, bitMask);
            if (!ret)
                cpusPerNode[i] = numa_bitmask_weight(bitMask);
            else
                s265_log(p, S265_LOG_ERROR, "Failed to genrate CPU mask\n");
        }
        numa_free_cpumask(bitMask);
    }
#else // NUMA not supported
    //如果不支持numa,则表示只有一个numa 节点
    cpusPerNode[0] = getCpuCount();//所有cpu 都归属在node[0]
#endif

    if (bNumaSupport && p->logLevel >= S265_LOG_DEBUG)
    for (int i = 0; i < numNumaNodes; i++)
        s265_log(p, S265_LOG_DEBUG, "detected NUMA node %d with %d logical cores\n", i, cpusPerNode[i]);
    /* limit threads based on param->numaPools
     * For windows because threads can't be allocated to live across sockets
     * changing the default behavior to be per-socket pools -- FIXME */
#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7
        //     默认指针为空                                         //指定为“*”                      //或者未指定内容
    if (!p->numaPools || (strcmp(p->numaPools, "NULL") == 0 || strcmp(p->numaPools, "*") == 0 || strcmp(p->numaPools, "") == 0))
    {
         char poolString[50] = "";
         for (int i = 0; i < numNumaNodes; i++)
         {
             char nextCount[10] = "";
             if (i)
                 sprintf(nextCount, ",%d", cpusPerNode[i]);
             else
                   sprintf(nextCount, "%d", cpusPerNode[i]);
             strcat(poolString, nextCount);
         }
         s265_param_parse(p, "pools", poolString);
     }
#endif
    if (p->numaPools && *p->numaPools)
    {
        const char *nodeStr = p->numaPools;
        for (int i = 0; i < numNumaNodes; i++)
        {
            if (!*nodeStr)//如果某个numanode上的cpu 个数为0
            {
                threadsPerPool[i] = 0;
                continue;
            }
            else if (*nodeStr == '-')//或者指定了不使用该node
                threadsPerPool[i] = 0;
            else if (*nodeStr == '*' || !strcasecmp(nodeStr, "NULL"))//如果从某一个开始指定为*或者不指定
            {
                for (int j = i; j < numNumaNodes; j++)
                {   //
                    threadsPerPool[numNumaNodes] += cpusPerNode[j];
                    nodeMaskPerPool[numNumaNodes] |= ((uint64_t)1 << j);
                }
                break;
            }
            else if (*nodeStr == '+')
            {
                threadsPerPool[numNumaNodes] += cpusPerNode[i];
                nodeMaskPerPool[numNumaNodes] |= ((uint64_t)1 << i);
            }
            else //如果指定了该node 限制使用的线程个数
            {
                int count = atoi(nodeStr);
                if (i > 0 || strchr(nodeStr, ','))   // it is comma -> old logic
                {
                    threadsPerPool[i] = S265_MIN(count, cpusPerNode[i]);//取指定值与该numanode的cpu个数的较小的值
                    nodeMaskPerPool[i] = ((uint64_t)1 << i);//该node存在mask
                }
                else                                 // new logic: exactly 'count' threads on all NUMAs
                {
                    threadsPerPool[numNumaNodes] = S265_MIN(count, numNumaNodes * MAX_POOL_THREADS);
                    nodeMaskPerPool[numNumaNodes] = ((uint64_t)-1 >> (64 - numNumaNodes));
                }
            }
            // 指向下一个node对应的配置字符
            /* consume current node string, comma, and white-space */
            while (*nodeStr && *nodeStr != ',')
               ++nodeStr;
            if (*nodeStr == ',' || *nodeStr == ' ')
               ++nodeStr;
        }
    }
    else
    {
        for (int i = 0; i < numNumaNodes; i++)
        {
            threadsPerPool[numNumaNodes]  += cpusPerNode[i];
            nodeMaskPerPool[numNumaNodes] |= ((uint64_t)1 << i);
        }
    }
 
    // If the last pool size is > MAX_POOL_THREADS, clip it to spawn thread pools only of size >= 1/2 max (heuristic)
    if ((threadsPerPool[numNumaNodes] > MAX_POOL_THREADS) &&
        ((threadsPerPool[numNumaNodes] % MAX_POOL_THREADS) < (MAX_POOL_THREADS / 2)))
    {
        threadsPerPool[numNumaNodes] -= (threadsPerPool[numNumaNodes] % MAX_POOL_THREADS);
        s265_log(p, S265_LOG_DEBUG,
                 "Creating only %d worker threads beyond specified numbers with --pools (if specified) to prevent asymmetry in pools; may not use all HW contexts\n", threadsPerPool[numNumaNodes]);
    }

    numPools = 0;
    for (int i = 0; i < numNumaNodes + 1; i++)
    {
        if (bNumaSupport)
            s265_log(p, S265_LOG_DEBUG, "NUMA node %d may use %d logical cores\n", i, cpusPerNode[i]);
        if (threadsPerPool[i])
        {
            numPools += (threadsPerPool[i] + MAX_POOL_THREADS - 1) / MAX_POOL_THREADS;
            totalNumThreads += threadsPerPool[i];
        }
    }
    if (!isThreadsReserved)
    { // 0 表示 编码线程池
        if (!numPools)
        {
            s265_log(p, S265_LOG_DEBUG, "No pool thread available. Deciding frame-threads based on detected CPU threads\n");
            totalNumThreads = ThreadPool::getCpuCount(); // auto-detect frame threads
        }

        if (!p->frameNumThreads)//如果帧级多线程没有指定，则根据可用线程总数决定
            ThreadPool::getFrameThreadsCount(p, totalNumThreads);//内部改变p->frameNumThreads
    }
    
    if (!numPools)
        return NULL;
    //线程池的个数必须小于等于帧级线程的个数
    if (numPools > p->frameNumThreads)
    {
        s265_log(p, S265_LOG_DEBUG, "Reducing number of thread pools for frame thread count\n");
        numPools = S265_MAX(p->frameNumThreads / 2, 1);
    }
    if (isThreadsReserved) // 1: lookahead 专用线程池
        numPools = 1;//如果是lookahead 线程池，限定一个线程池
    ThreadPool *pools = new ThreadPool[numPools];//新建numpools 个线程池对象
    if (pools)
    {
        //lookahead 线程只用0号线程池里面的线程
        // for lookahead 1; for else > 2 + 1
        // 每个 pool 最多有 mzxproviders 个’领导‘
        int maxProviders = (p->frameNumThreads + numPools - 1) / numPools + !isThreadsReserved; /* +1 is Lookahead, always assigned to threadpool 0 */
        int node = 0;
        for (int i = 0; i < numPools; i++)
        {
            while (!threadsPerPool[node])// 找到一个可以启线程的node
                node++;
            int numThreads = S265_MIN(MAX_POOL_THREADS, threadsPerPool[node]);
            int origNumThreads = numThreads;
            if (i == 0 && p->lookaheadThreads > numThreads / 2)
            {
                // 如果开了lookahead 线程池，则 第0个线程池用于lookahead，如果lookahead线程的个数要大于该node可以启动的线程的数量的一半
                p->lookaheadThreads = numThreads / 2;
                s265_log(p, S265_LOG_DEBUG, "Setting lookahead threads to a maximum of half the total number of threads\n");
            }

            if (isThreadsReserved)
            {   // lookahead 线程只允许一个providers
                numThreads = p->lookaheadThreads;
                maxProviders = 1;
            }
            else if (i == 0)//否则 非lookahead线程的首个线程池，共线程数减去lookahead占去的线程数(留后后面单独建立lookahead线程池）,余下的线程数
                numThreads -= p->lookaheadThreads;
            
            if (!pools[i].create(numThreads, maxProviders, nodeMaskPerPool[node]))
            {
                S265_FREE(pools);
                numPools = 0;
                return NULL;
            }
            if (numNumaNodes > 1)
            {
                char *nodesstr = new char[64 * strlen(",63") + 1];
                int len = 0;
                for (int j = 0; j < 64; j++)
                    if ((nodeMaskPerPool[node] >> j) & 1)
                        len += sprintf(nodesstr + len, ",%d", j);
                s265_log(p, S265_LOG_INFO, "Thread pool %d using %d threads on numa nodes %s\n", i, numThreads, nodesstr + 1);
                delete[] nodesstr;
            }
            else
                s265_log(p, S265_LOG_INFO, "Thread pool created using %d threads\n", numThreads);
            //注意:单个pool里面最多32个线程，如果某个node里面的线程多余32，则该node对应存在多个线程池
            threadsPerPool[node] -= origNumThreads;
        }
    }
    else
        numPools = 0;
    return pools;
}

ThreadPool::ThreadPool()
{
    memset(this, 0, sizeof(*this));
}

bool ThreadPool::create(int numThreads, int maxProviders, uint64_t nodeMask)
{
    S265_CHECK(numThreads <= MAX_POOL_THREADS, "a single thread pool cannot have more than MAX_POOL_THREADS threads\n");

#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7 
    memset(&m_groupAffinity, 0, sizeof(GROUP_AFFINITY));
    for (int i = 0; i < getNumaNodeCount(); i++)
    {
        int numaNode = ((nodeMask >> i) & 0x1U) ? i : -1;
        if (numaNode != -1)
        if (GetNumaNodeProcessorMaskEx((USHORT)numaNode, &m_groupAffinity))
            break;
    }
    m_numaMask = &m_groupAffinity.Mask;
#elif HAVE_LIBNUMA
    if (numa_available() >= 0)
    {
        struct bitmask* nodemask = numa_allocate_nodemask();
        if (nodemask)
        {
            *(nodemask->maskp) = nodeMask;
            m_numaMask = nodemask;
        }
        else
            s265_log(NULL, S265_LOG_ERROR, "unable to get NUMA node mask for %lx\n", nodeMask);
    }
#else
    (void)nodeMask;
#endif

    m_numWorkers = numThreads; //worker 用来完成某个具体任务的子线程
    // 线程资源分配 该线程池中 分配numThreads个工作线程
    m_workers = S265_MALLOC(WorkerThread, numThreads);

    /* placement new initialization */
    //该线程池中的线程对象都使用同一个同一个线程池，每一个线程有自己的线程id
    if (m_workers)
        for (int i = 0; i < numThreads; i++)
            new (m_workers + i)WorkerThread(*this, i);// placement new
    // 之所以先malloc 内存再 使用placement new 主要是为了内存对齐
    //二级指针
    m_jpTable = S265_MALLOC(JobProvider*, maxProviders);
    m_numProviders = 0;// 初始时该线程池没有“领导”

    return m_workers && m_jpTable;
}

bool ThreadPool::start()
{
    m_isActive = true;
    for (int i = 0; i < m_numWorkers; i++)
    {
        if (!m_workers[i].start())// 将线程池中的所有线程都启动起来 “实际是创建并运行线程“
        {
            m_isActive = false;
            return false;
        }
    }
    return true;
}

void ThreadPool::stopWorkers()
{
    if (m_workers)
    {
        m_isActive = false;// 设置线程结束while循环的标志
        for (int i = 0; i < m_numWorkers; i++)
        {
            while (!(m_sleepBitmap & ((sleepbitmap_t)1 << i)))//如果线程i 不在sleep状态,等一会儿继续判断该线程，直至线程编程sleep状态
                GIVE_UP_TIME();
            m_workers[i].awaken();//唤醒被阻塞的线程
            m_workers[i].stop();//等待线程i运行结束通过m_isActive=false 使得线程可以退出while死循环
        }
    }
}

ThreadPool::~ThreadPool()
{
    if (m_workers)
    {
        for (int i = 0; i < m_numWorkers; i++)
            m_workers[i].~WorkerThread();
    }

    S265_FREE(m_workers);
    S265_FREE(m_jpTable);

#if HAVE_LIBNUMA
    if(m_numaMask)
        numa_free_nodemask((struct bitmask*)m_numaMask);
#endif
}

void ThreadPool::setCurrentThreadAffinity()
{
    setThreadNodeAffinity(m_numaMask);
}

void ThreadPool::setThreadNodeAffinity(void *numaMask)
{
#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7 
    UNREFERENCED_PARAMETER(numaMask);
    GROUP_AFFINITY groupAffinity;
    memset(&groupAffinity, 0, sizeof(GROUP_AFFINITY));
    groupAffinity.Group = m_groupAffinity.Group;
    groupAffinity.Mask = m_groupAffinity.Mask;
    const PGROUP_AFFINITY affinityPointer = &groupAffinity;
    if (SetThreadGroupAffinity(GetCurrentThread(), affinityPointer, NULL))
        return;
    else
        s265_log(NULL, S265_LOG_ERROR, "unable to set thread affinity for NUMA node mask\n");
#elif HAVE_LIBNUMA
    if (numa_available() >= 0)
    {
        numa_run_on_node_mask((struct bitmask*)numaMask);
        numa_set_interleave_mask((struct bitmask*)numaMask);
        numa_set_localalloc();
        return;
    }
    s265_log(NULL, S265_LOG_ERROR, "unable to set thread affinity for NUMA node mask\n");
#else
    (void)numaMask;
#endif
    return;
}

/* static */
int ThreadPool::getNumaNodeCount()
{
#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7 
    ULONG num = 1;
    if (GetNumaHighestNodeNumber(&num))
        num++;
    return (int)num;
#elif HAVE_LIBNUMA
    if (numa_available() >= 0)
        return numa_max_node() + 1;
    else
        return 1;
#else
    return 1;
#endif
}

/* static */
int ThreadPool::getCpuCount()
{
#if defined(_WIN32_WINNT) && _WIN32_WINNT >= _WIN32_WINNT_WIN7
    enum { MAX_NODE_NUM = 127 };
    int cpus = 0;
    int numNumaNodes = S265_MIN(getNumaNodeCount(), MAX_NODE_NUM);
    GROUP_AFFINITY groupAffinity;
    for (int i = 0; i < numNumaNodes; i++)
    {
        GetNumaNodeProcessorMaskEx((UCHAR)i, &groupAffinity);
        cpus += popCount(groupAffinity.Mask);
    }
    return cpus;
#elif _WIN32
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    return sysinfo.dwNumberOfProcessors;
#elif __unix__ && S265_ARCH_ARM
    /* Return the number of processors configured by OS. Because, most embedded linux distributions
     * uses only one processor as the scheduler doesn't have enough work to utilize all processors */
    return sysconf(_SC_NPROCESSORS_CONF);
#elif __unix__
    return sysconf(_SC_NPROCESSORS_ONLN);
#elif MACOS && __MACH__
    int nm[2];
    size_t len = 4;
    uint32_t count;

    nm[0] = CTL_HW;
    nm[1] = HW_AVAILCPU;
    sysctl(nm, 2, &count, &len, NULL, 0);

    if (count < 1)
    {
        nm[1] = HW_NCPU;
        sysctl(nm, 2, &count, &len, NULL, 0);
        if (count < 1)
            count = 1;
    }

    return count;
#else
    return 2; // default to 2 threads, everywhere else
#endif
}
// 计算fpp 帧级并行的线程个数
void ThreadPool::getFrameThreadsCount(s265_param* p, int cpuCount)
{
    int rows = (p->sourceHeight + p->maxCUSize - 1) >> g_log2Size[p->maxCUSize];
    if (!p->bEnableWavefront)
        p->frameNumThreads = S265_MIN3(cpuCount, (rows + 1) / 2, S265_MAX_FRAME_THREADS);
    else if (cpuCount >= 32)
        p->frameNumThreads = (p->sourceHeight > 2000) ? 6 : 5; 
    else if (cpuCount >= 16)
        p->frameNumThreads = 4; 
    else if (cpuCount >= 8)
        p->frameNumThreads = 3;
    else if (cpuCount >= 4)
        p->frameNumThreads = 2;
    else
        p->frameNumThreads = 1;
}

} // end namespace S265_NS

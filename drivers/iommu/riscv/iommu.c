// SPDX-License-Identifier: GPL-2.0-only
/*
 * IOMMU API for RISC-V IOMMU implementations.
 *
 * Copyright © 2022-2023 Rivos Inc.
 * Copyright © 2023 FORTH-ICS/CARV
 *
 * Authors
 *	Tomasz Jeznach <tjeznach@rivosinc.com>
 *	Nick Kossifidis <mick@ics.forth.gr>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/compiler.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>

#include "iommu-bits.h"
#include "iommu.h"

MODULE_DESCRIPTION("Driver for RISC-V IOMMU");
MODULE_AUTHOR("Tomasz Jeznach <tjeznach@rivosinc.com>");
MODULE_AUTHOR("Nick Kossifidis <mick@ics.forth.gr>");
MODULE_ALIAS("riscv-iommu");
MODULE_LICENSE("GPL v2");

/* Timeouts in [us] */
#define RISCV_IOMMU_DDTP_TIMEOUT	50000	/* DDTP.BUSY limit */
#define RISCV_IOMMU_QCSR_TIMEOUT	50000	/* CSR.BUSY limit */
#define RISCV_IOMMU_QUEUE_TIMEOUT	10000	/* queue head/tail access limit */
#define RISCV_IOMMU_IOFENCE_TIMEOUT	1500000	/* IOFENCE.C timeout */

/* Number of entries per CMD/FLT queue, should be <= INT_MAX */
#define RISCV_IOMMU_DEF_CQ_COUNT	8192
#define RISCV_IOMMU_DEF_FQ_COUNT	8192

/* RISC-V IOMMU PPN <> PHYS address conversions, PHYS <=> PPN[53:10] */
#define phys_to_ppn(va)  (((va) >> 2) & (((1ULL << 44) - 1) << 10))
#define ppn_to_phys(pn)	 (((pn) << 2) & (((1ULL << 44) - 1) << 12))

#define dev_to_iommu(dev) \
	container_of((dev)->iommu->iommu_dev, struct riscv_iommu_device, iommu)

#define iommu_domain_to_riscv(iommu_domain) \
	container_of(iommu_domain, struct riscv_iommu_domain, domain)

/* IOMMU PSCID allocation namespace. */
static DEFINE_IDA(riscv_iommu_pscids);
#define RISCV_IOMMU_MAX_PSCID		BIT(20)

/* Device resource-managed allocations */
struct riscv_iommu_devres {
	unsigned long addr;
	unsigned int order;
};

static void riscv_iommu_devres_pages_release(struct device *dev, void *res)
{
	struct riscv_iommu_devres *devres = res;

	free_pages(devres->addr, devres->order);
}

static int riscv_iommu_devres_pages_match(struct device *dev, void *res, void *p)
{
	struct riscv_iommu_devres *devres = res;
	struct riscv_iommu_devres *target = p;

	return devres->addr == target->addr;
}

static unsigned long riscv_iommu_get_pages(struct riscv_iommu_device *iommu,
					   unsigned int order)
{
	struct riscv_iommu_devres *devres;
	struct page *pages;

	pages = alloc_pages_node(dev_to_node(iommu->dev),
				 GFP_KERNEL_ACCOUNT | __GFP_ZERO, order);
	if (unlikely(!pages)) {
		dev_err(iommu->dev, "Page allocation failed, order %u\n", order);
		return 0;
	}

	devres = devres_alloc(riscv_iommu_devres_pages_release,
			      sizeof(struct riscv_iommu_devres), GFP_KERNEL);

	if (unlikely(!devres)) {
		__free_pages(pages, order);
		return 0;
	}

	devres->addr = (unsigned long)page_address(pages);
	devres->order = order;

	devres_add(iommu->dev, devres);

	return devres->addr;
}

static void riscv_iommu_free_pages(struct riscv_iommu_device *iommu, unsigned long addr)
{
	struct riscv_iommu_devres devres = { .addr = addr };

	WARN_ON(devres_release(iommu->dev, riscv_iommu_devres_pages_release,
			       riscv_iommu_devres_pages_match, &devres));
}

/*
 * Hardware queue allocation and management.
 */

/* Setup queue base, control registers and default queue length */
#define RISCV_IOMMU_QUEUE_INIT(q, name) do {					\
	struct riscv_iommu_queue *_q = q;					\
	_q->qid = RISCV_IOMMU_INTR_ ## name;					\
	_q->qbr = RISCV_IOMMU_REG_ ## name ## B;				\
	_q->qcr = RISCV_IOMMU_REG_ ## name ## CSR;				\
	_q->mask = _q->mask ?: (RISCV_IOMMU_DEF_ ## name ## _COUNT) - 1;	\
} while (0)

/* Note: offsets are the same for all queues */
#define Q_HEAD(q) ((q)->qbr + (RISCV_IOMMU_REG_CQH - RISCV_IOMMU_REG_CQB))
#define Q_TAIL(q) ((q)->qbr + (RISCV_IOMMU_REG_CQT - RISCV_IOMMU_REG_CQB))
#define Q_ITEM(q, index) ((q)->mask & (index))
#define Q_IPSR(q) BIT((q)->qid)

/*
 * Discover queue ring buffer hardware configuration, allocate in-memory
 * ring buffer or use fixed I/O memory location, configure queue base register.
 * Must be called before hardware queue is enabled.
 *
 * @queue - data structure, configured with RISCV_IOMMU_QUEUE_INIT()
 * @entry_size - queue single element size in bytes.
 */
static int riscv_iommu_queue_alloc(struct riscv_iommu_device *iommu,
				   struct riscv_iommu_queue *queue,
				   size_t entry_size)
{
	struct device *dev = iommu->dev;
	unsigned int logsz;
	unsigned long addr = 0;
	u64 qb, rb;

	/*
	 * Use WARL base register property to discover maximum allowed
	 * number of entries and optional fixed IO address for queue location.
	 */
	riscv_iommu_writeq(iommu, queue->qbr, RISCV_IOMMU_QUEUE_LOGSZ_FIELD);
	qb = riscv_iommu_readq(iommu, queue->qbr);

	/*
	 * Calculate and verify hardware supported queue length, as reported
	 * by the field LOGSZ, where max queue length is equal to 2^(LOGSZ + 1).
	 * Update queue size based on hardware supported value.
	 */
	logsz = ilog2(queue->mask);
	if (logsz > FIELD_GET(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, qb))
		logsz = FIELD_GET(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, qb);

	/*
	 * Use WARL base register property to discover an optional fixed IO address
	 * for queue ring buffer location. Otherwise allocate contigus system memory.
	 */
	if (FIELD_GET(RISCV_IOMMU_PPN_FIELD, qb)) {
		const size_t queue_size = entry_size << (logsz + 1);

		queue->phys = ppn_to_phys(FIELD_GET(RISCV_IOMMU_PPN_FIELD, qb));
		queue->base = devm_ioremap(dev, queue->phys, queue_size);
	} else {
		do {
			const unsigned int order = get_order(entry_size << (logsz + 1));

			addr = riscv_iommu_get_pages(iommu, order);
			queue->base = (u64 *)addr;
			queue->phys = __pa(addr);
		} while (!queue->base && logsz-- > 0);
	}

	if (!queue->base)
		return -ENOMEM;

	qb = phys_to_ppn(queue->phys) |
	     FIELD_PREP(RISCV_IOMMU_QUEUE_LOGSZ_FIELD, logsz);

	/* Update base register and read back to verify hw accepted our write */
	riscv_iommu_writeq(iommu, queue->qbr, qb);
	rb = riscv_iommu_readq(iommu, queue->qbr);
	if (rb != qb) {
		if (addr)
			riscv_iommu_free_pages(iommu, addr);
		return -ENODEV;
	}

	/* Update actual queue mask */
	if (queue->mask != (2U << logsz) - 1) {
		queue->mask = (2U << logsz) - 1;
		dev_warn(dev, "queue #%u restricted to 2^%u entries",
			 queue->qid, logsz + 1);
	}

	queue->iommu = iommu;

	return 0;
}

/* Check interrupt queue status, IPSR */
static irqreturn_t riscv_iommu_queue_ipsr(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;

	if (riscv_iommu_readl(q->iommu, RISCV_IOMMU_REG_IPSR) & Q_IPSR(q))
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

/*
 * Enable queue processing in the hardware, register interrupt handler.
 *
 * @queue - data structure, already allocated with riscv_iommu_queue_alloc()
 * @irq_handler - threaded interrupt handler.
 */
static int riscv_iommu_queue_enable(struct riscv_iommu_queue *queue,
				    irq_handler_t irq_handler)
{
	struct riscv_iommu_device *iommu = queue->iommu;
	const int vec = (iommu->ivec >> (queue->qid * 4)) % RISCV_IOMMU_INTR_COUNT;
	const unsigned int irq = iommu->irqs[vec];
	u32 csr;
	int rc;

	/* Polling not implemented */
	if (!irq)
		return -ENODEV;

	rc = request_threaded_irq(irq, riscv_iommu_queue_ipsr, irq_handler,
				  IRQF_ONESHOT | IRQF_SHARED, dev_name(iommu->dev), queue);
	if (rc)
		return rc;

	/*
	 * Enable queue with interrupts, clear any memory fault if any.
	 * Wait for the hardware to acknowledge request and activate queue processing.
	 * Note: All CSR bitfields are in the same offsets for all queues.
	 */
	riscv_iommu_writel(iommu, queue->qcr,
			   RISCV_IOMMU_QUEUE_ENABLE |
			   RISCV_IOMMU_QUEUE_INTR_ENABLE |
			   RISCV_IOMMU_QUEUE_MEM_FAULT);

	riscv_iommu_readl_timeout(iommu, queue->qcr,
				  csr, !(csr & RISCV_IOMMU_QUEUE_BUSY),
				  10, RISCV_IOMMU_QCSR_TIMEOUT);

	if (RISCV_IOMMU_QUEUE_ACTIVE != (csr & (RISCV_IOMMU_QUEUE_ACTIVE |
						RISCV_IOMMU_QUEUE_BUSY |
						RISCV_IOMMU_QUEUE_MEM_FAULT))) {
		/* Best effort to stop and disable failing hardware queue. */
		riscv_iommu_writel(iommu, queue->qcr, 0);
		free_irq(irq, queue);
		return -EBUSY;
	}

	queue->active = true;

	/* Clear any pending interrupt flag. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(queue));

	return 0;
}

/*
 * Disable queue. Wait for the hardware to acknowledge request and
 * stop processing enqueued requests. Report errors but continue.
 */
static void riscv_iommu_queue_disable(struct riscv_iommu_queue *queue)
{
	struct riscv_iommu_device *iommu = queue->iommu;
	const int vec = (iommu->ivec >> (queue->qid * 4)) % RISCV_IOMMU_INTR_COUNT;
	u32 csr;

	if (!iommu || !queue->active)
		return;

	queue->active = false;
	free_irq(iommu->irqs[vec], queue);
	riscv_iommu_writel(iommu, queue->qcr, 0);
	riscv_iommu_readl_timeout(iommu, queue->qcr,
				  csr, !(csr & RISCV_IOMMU_QUEUE_BUSY),
				  10, RISCV_IOMMU_QCSR_TIMEOUT);

	if (csr & (RISCV_IOMMU_QUEUE_ACTIVE | RISCV_IOMMU_QUEUE_BUSY))
		dev_err(iommu->dev, "fail to disable hardware queue #%u, csr 0x%x\n",
			queue->qid, csr);
}

/*
 * Returns number of available valid queue entries and the first item index or negative
 * error code.  Update shadow producer index if nessesary.
 */
static int riscv_iommu_queue_consume(struct riscv_iommu_queue *q, unsigned int *index)
{
	unsigned int head = atomic_read(&q->head);
	unsigned int tail = atomic_read(&q->tail);
	unsigned int last = Q_ITEM(q, tail);
	int available = (int)(tail - head);

	*index = head;

	if (available > 0)
		return available;

	/* read hardware producer index, check reserved register bits are not set. */
	if (riscv_iommu_readl_timeout(q->iommu, Q_TAIL(q), tail, (tail & ~q->mask) == 0,
				      0, RISCV_IOMMU_QUEUE_TIMEOUT))
		return -EBUSY;

	if (tail == last)
		return 0;

	/* update shadow producer index */
	return (int)(atomic_add_return((tail - last) & q->mask, &q->tail) - head);
}

/*
 * Release processed queue entries, should match riscv_iommu_queue_consume() calls.
 */
static void riscv_iommu_queue_release(struct riscv_iommu_queue *q, int count)
{
	const unsigned int head = atomic_add_return(count, &q->head);

	riscv_iommu_writel(q->iommu, Q_HEAD(q), Q_ITEM(q, head));
}

/*
 * Waits for available producer slot in the queue. MP safe.
 * Returns negative error code in case of timeout.
 * Submission via riscv_iommu_queue_submit() should happen as soon as possible.
 */
static int riscv_iommu_queue_aquire(struct riscv_iommu_queue *q, unsigned int *index,
				    unsigned int timeout_us)
{
	unsigned int prod = atomic_fetch_add(1, &q->prod);
	unsigned int head = atomic_read(&q->head);

	*index = prod;

	if ((prod - head) > q->mask) {
		/* Wait for queue space availability */
		if (readx_poll_timeout(atomic_read, &q->head, head, (prod - head) < q->mask,
				       0, timeout_us))
			return -EBUSY;
	} else if ((prod - head) == q->mask) {
		/*
		 * Update consumer shadow index and check reserved register bits are not set,
		 * and wait for space availability.
		 */
		const unsigned int last = Q_ITEM(q, head);

		if (riscv_iommu_readl_timeout(q->iommu, Q_HEAD(q), head,
					      !(head & ~q->mask) && head != last,
					      0, timeout_us))
			return -EBUSY;
		atomic_add((head - last) & q->mask, &q->head);
	}

	return 0;
}

/*
 * Ordered write to producer hardware register.
 * @index should match value allocated by riscv_iommu_queue_aquire() call.
 */
static int riscv_iommu_queue_submit(struct riscv_iommu_queue *q, unsigned int index)
{
	unsigned int tail;

	if (readx_poll_timeout(atomic_read, &q->tail, tail, index == tail,
			       0, RISCV_IOMMU_QUEUE_TIMEOUT))
		return -EBUSY;

	riscv_iommu_writel(q->iommu, Q_TAIL(q), Q_ITEM(q, index + 1));
	atomic_inc(&q->tail);

	return 0;
}

/* Wait for submitted item to be processed. */
static int riscv_iommu_queue_wait(struct riscv_iommu_queue *q, unsigned int index,
				  unsigned int timeout_us)
{
	const unsigned int cons = atomic_read(&q->head);
	const unsigned int last = Q_ITEM(q, cons);
	unsigned int head;

	/* Already processed by the consumer */
	if ((int)(cons - index) > 0)
		return 0;

	/* Monitor consumer index */
	return riscv_iommu_readl_timeout(q->iommu, Q_HEAD(q), head, !(head & ~q->mask) &&
					 (int)(cons + ((head - last) & q->mask) - index) > 0,
					 0, timeout_us);
}

/* Enqueue command and wait to be processed if timeout_us > 0 */
static int riscv_iommu_queue_send(struct riscv_iommu_queue *q,
				  struct riscv_iommu_command *cmd,
				  unsigned int timeout_us)
{
	unsigned int idx;

	if (WARN_ON(riscv_iommu_queue_aquire(q, &idx, RISCV_IOMMU_QUEUE_TIMEOUT)))
		return -EBUSY;

	((struct riscv_iommu_command *)q->base)[Q_ITEM(q, idx)] = *cmd;

	if (WARN_ON(riscv_iommu_queue_submit(q, idx)))
		return -EBUSY;

	if (timeout_us)
		riscv_iommu_queue_wait(q, idx, timeout_us);

	return 0;
}

/*
 * IOMMU Command queue chapter 3.1
 */

/* Command queue interrupt handler thread function */
static irqreturn_t riscv_iommu_cmdq_process(int irq, void *data)
{
	const struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	unsigned int ctrl;

	/* Clear MF/CQ errors, complete error recovery to be implemented. */
	ctrl = riscv_iommu_readl(q->iommu, q->qcr);
	if (ctrl & (RISCV_IOMMU_CQCSR_CQMF | RISCV_IOMMU_CQCSR_CMD_TO |
		    RISCV_IOMMU_CQCSR_CMD_ILL | RISCV_IOMMU_CQCSR_FENCE_W_IP)) {
		riscv_iommu_writel(q->iommu, q->qcr, ctrl);
		dev_warn(q->iommu->dev,
			 "Queue #%u error; fault:%d timeout:%d illegal:%d fence_w_ip:%d\n",
			 q->qid,
			 !!(ctrl & RISCV_IOMMU_CQCSR_CQMF),
			 !!(ctrl & RISCV_IOMMU_CQCSR_CMD_TO),
			 !!(ctrl & RISCV_IOMMU_CQCSR_CMD_ILL),
			 !!(ctrl & RISCV_IOMMU_CQCSR_FENCE_W_IP));
	}

	/* Placeholder for command queue interrupt notifiers */

	/* Clear command interrupt pending. */
	riscv_iommu_writel(q->iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(q));

	return IRQ_HANDLED;
}

static inline void riscv_iommu_cmd_inval_vma(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOTINVAL_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOTINVAL_FUNC_VMA);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_inval_set_addr(struct riscv_iommu_command *cmd,
						  u64 addr)
{
	cmd->dword0 |= RISCV_IOMMU_CMD_IOTINVAL_AV;
	cmd->dword1 = FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_ADDR, phys_to_pfn(addr));
}

static inline void riscv_iommu_cmd_inval_set_pscid(struct riscv_iommu_command *cmd,
						   int pscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_PSCID, pscid) |
		       RISCV_IOMMU_CMD_IOTINVAL_PSCV;
}

static inline void riscv_iommu_cmd_inval_set_gscid(struct riscv_iommu_command *cmd,
						   int gscid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IOTINVAL_GSCID, gscid) |
		       RISCV_IOMMU_CMD_IOTINVAL_GV;
}

static inline void riscv_iommu_cmd_iofence(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOFENCE_FUNC_C);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iofence_set_av(struct riscv_iommu_command *cmd,
						  u64 addr, u32 data)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IOFENCE_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IOFENCE_FUNC_C) |
		      FIELD_PREP(RISCV_IOMMU_CMD_IOFENCE_DATA, data) |
		      RISCV_IOMMU_CMD_IOFENCE_AV;
	cmd->dword1 = addr >> 2;
}

static inline void riscv_iommu_cmd_iodir_inval_ddt(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IODIR_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_DDT);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iodir_inval_pdt(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE,
				 RISCV_IOMMU_CMD_IODIR_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC,
				 RISCV_IOMMU_CMD_IODIR_FUNC_INVAL_PDT);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_iodir_set_did(struct riscv_iommu_command *cmd,
						 unsigned int devid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IODIR_DID, devid) |
		       RISCV_IOMMU_CMD_IODIR_DV;
}

static inline void riscv_iommu_cmd_iodir_set_pid(struct riscv_iommu_command *cmd,
						 unsigned int pasid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_IODIR_PID, pasid);
}

static inline void riscv_iommu_cmd_ats_inval(struct riscv_iommu_command *cmd)
{
	cmd->dword0 = FIELD_PREP(RISCV_IOMMU_CMD_OPCODE, RISCV_IOMMU_CMD_ATS_OPCODE) |
		      FIELD_PREP(RISCV_IOMMU_CMD_FUNC, RISCV_IOMMU_CMD_ATS_FUNC_INVAL);
	cmd->dword1 = 0;
}

static inline void riscv_iommu_cmd_ats_set_devid(struct riscv_iommu_command *cmd,
						 unsigned int devid)
{
	const unsigned int seg = (devid & 0x0ff0000) >> 16;
	const unsigned int rid = (devid & 0x000ffff);

	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_DSEG, seg) | RISCV_IOMMU_CMD_ATS_DSV |
		       FIELD_PREP(RISCV_IOMMU_CMD_ATS_RID, rid);
}

static inline void riscv_iommu_cmd_ats_set_pid(struct riscv_iommu_command *cmd,
					       unsigned int pid)
{
	cmd->dword0 |= FIELD_PREP(RISCV_IOMMU_CMD_ATS_PID, pid) | RISCV_IOMMU_CMD_ATS_PV;
}

static inline void riscv_iommu_cmd_ats_set_range(struct riscv_iommu_command *cmd,
						 unsigned long start, unsigned long end,
						 bool global_inv)
{
	size_t len = end - start + 1;
	u64 payload = 0;

	/*
	 * PCI Express specification
	 * Section 10.2.3.2 Translation Range Size (S) Field
	 */
	if (len < PAGE_SIZE)
		len = PAGE_SIZE;
	else
		len = __roundup_pow_of_two(len);

	payload = (start & ~(len - 1)) | (((len - 1) >> 12) << 11);

	if (global_inv)
		payload |= RISCV_IOMMU_CMD_ATS_INVAL_G;

	cmd->dword1 = payload;
}

static inline void riscv_iommu_cmd_ats_set_all(struct riscv_iommu_command *cmd,
					       bool global_inv)
{
	u64 payload = GENMASK_ULL(62, 11);

	if (global_inv)
		payload |= RISCV_IOMMU_CMD_ATS_INVAL_G;

	cmd->dword1 = payload;
}

/*
 * IOMMU Fault/Event queue chapter 3.2
 */

static void riscv_iommu_fault(struct riscv_iommu_device *iommu,
			      struct riscv_iommu_fq_record *event)
{
	unsigned int err = FIELD_GET(RISCV_IOMMU_FQ_HDR_CAUSE, event->hdr);
	unsigned int devid = FIELD_GET(RISCV_IOMMU_FQ_HDR_DID, event->hdr);

	/* Placeholder for future fault handling implementation, report only. */
	if (err)
		dev_warn_ratelimited(iommu->dev,
				     "Fault %d devid: 0x%x iotval: %llx iotval2: %llx\n",
				     err, devid, event->iotval, event->iotval2);
}

/* Fault queue interrupt hanlder thread function */
static irqreturn_t riscv_iommu_fltq_process(int irq, void *data)
{
	struct riscv_iommu_queue *q = (struct riscv_iommu_queue *)data;
	struct riscv_iommu_device *iommu = q->iommu;
	struct riscv_iommu_fq_record *events;
	unsigned int ctrl, idx;
	int cnt, len;

	events = (struct riscv_iommu_fq_record *)q->base;

	/* Clear fault interrupt pending and process all received fault events. */
	riscv_iommu_writel(iommu, RISCV_IOMMU_REG_IPSR, Q_IPSR(q));

	do {
		cnt = riscv_iommu_queue_consume(q, &idx);
		for (len = 0; len < cnt; idx++, len++)
			riscv_iommu_fault(iommu, &events[Q_ITEM(q, idx)]);
		riscv_iommu_queue_release(q, cnt);
	} while (cnt > 0);

	/* Clear MF/OF errors, complete error recovery to be implemented. */
	ctrl = riscv_iommu_readl(iommu, q->qcr);
	if (ctrl & (RISCV_IOMMU_FQCSR_FQMF | RISCV_IOMMU_FQCSR_FQOF)) {
		riscv_iommu_writel(iommu, q->qcr, ctrl);
		dev_warn(iommu->dev,
			 "Queue #%u error; memory fault:%d overflow:%d\n",
			 q->qid,
			 !!(ctrl & RISCV_IOMMU_FQCSR_FQMF),
			 !!(ctrl & RISCV_IOMMU_FQCSR_FQOF));
	}

	return IRQ_HANDLED;
}

/* Lookup and initialize device context info structure. */
static struct riscv_iommu_dc *riscv_iommu_get_dc(struct riscv_iommu_device *iommu,
						 unsigned int devid)
{
	const bool base_format = !(iommu->caps & RISCV_IOMMU_CAP_MSI_FLAT);
	unsigned int depth;
	unsigned long ddt, ptr, old, new;
	u8 ddi_bits[3] = { 0 };
	u64 *ddtp = NULL;

	/* Make sure the mode is valid */
	if (iommu->ddt_mode < RISCV_IOMMU_DDTP_MODE_1LVL ||
	    iommu->ddt_mode > RISCV_IOMMU_DDTP_MODE_3LVL)
		return NULL;

	/*
	 * Device id partitioning for base format:
	 * DDI[0]: bits 0 - 6   (1st level) (7 bits)
	 * DDI[1]: bits 7 - 15  (2nd level) (9 bits)
	 * DDI[2]: bits 16 - 23 (3rd level) (8 bits)
	 *
	 * For extended format:
	 * DDI[0]: bits 0 - 5   (1st level) (6 bits)
	 * DDI[1]: bits 6 - 14  (2nd level) (9 bits)
	 * DDI[2]: bits 15 - 23 (3rd level) (9 bits)
	 */
	if (base_format) {
		ddi_bits[0] = 7;
		ddi_bits[1] = 7 + 9;
		ddi_bits[2] = 7 + 9 + 8;
	} else {
		ddi_bits[0] = 6;
		ddi_bits[1] = 6 + 9;
		ddi_bits[2] = 6 + 9 + 9;
	}

	/* Make sure device id is within range */
	depth = iommu->ddt_mode - RISCV_IOMMU_DDTP_MODE_1LVL;
	if (devid >= (1 << ddi_bits[depth]))
		return NULL;

	/* Get to the level of the non-leaf node that holds the device context */
	for (ddtp = iommu->ddt_root; depth-- > 0;) {
		const int split = ddi_bits[depth];
		/*
		 * Each non-leaf node is 64bits wide and on each level
		 * nodes are indexed by DDI[depth].
		 */
		ddtp += (devid >> split) & 0x1FF;

		/*
		 * Check if this node has been populated and if not
		 * allocate a new level and populate it.
		 */
		do {
			ddt = READ_ONCE(*(unsigned long *)ddtp);
			if (ddt & RISCV_IOMMU_DDTE_VALID) {
				ddtp = __va(ppn_to_phys(ddt));
				break;
			}

			ptr = riscv_iommu_get_pages(iommu, 0);
			if (!ptr)
				return NULL;

			new = phys_to_ppn(__pa(ptr)) | RISCV_IOMMU_DDTE_VALID;
			old = cmpxchg_relaxed((unsigned long *)ddtp, ddt, new);

			if (old == ddt) {
				ddtp = (u64 *)ptr;
				break;
			}

			/* Race setting DDT detected, re-read and retry. */
			riscv_iommu_free_pages(iommu, ptr);
		} while (1);
	}

	/*
	 * Grab the node that matches DDI[depth], note that when using base
	 * format the device context is 4 * 64bits, and the extended format
	 * is 8 * 64bits, hence the (3 - base_format) below.
	 */
	ddtp += (devid & ((64 << base_format) - 1)) << (3 - base_format);

	return (struct riscv_iommu_dc *)ddtp;
}

/*
 * Discover supported DDT modes starting from requested value,
 * configure DDTP register with accepted mode and root DDT address.
 * Accepted iommu->ddt_mode is updated on success.
 */
static int riscv_iommu_set_ddtp_mode(struct riscv_iommu_device *iommu,
				     unsigned int ddtp_mode)
{
	struct device *dev = iommu->dev;
	struct riscv_iommu_command cmd;
	u64 ddtp, rq_ddtp;
	unsigned int mode, rq_mode = ddtp_mode;

	readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
				   ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				   10, RISCV_IOMMU_DDTP_TIMEOUT);

	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/* Disallow state transtion from xLVL to xLVL. */
	switch (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp)) {
	case RISCV_IOMMU_DDTP_MODE_BARE:
	case RISCV_IOMMU_DDTP_MODE_OFF:
		break;
	default:
		if (rq_mode != RISCV_IOMMU_DDTP_MODE_BARE &&
		    rq_mode != RISCV_IOMMU_DDTP_MODE_OFF)
			return -EINVAL;
		break;
	}

	do {
		rq_ddtp = FIELD_PREP(RISCV_IOMMU_DDTP_MODE, rq_mode);
		if (rq_mode > RISCV_IOMMU_DDTP_MODE_BARE)
			rq_ddtp |= phys_to_ppn(iommu->ddt_phys);

		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP, rq_ddtp);

		readq_relaxed_poll_timeout(iommu->reg + RISCV_IOMMU_REG_DDTP,
					   ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					   10, RISCV_IOMMU_DDTP_TIMEOUT);

		if (ddtp & RISCV_IOMMU_DDTP_BUSY) {
			dev_warn(dev, "timeout when setting ddtp (ddt mode: %u)\n",
				 rq_mode);
			return -EBUSY;
		}

		/* Verify IOMMU hardware accepts new DDTP config. */
		mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
		if (rq_mode > RISCV_IOMMU_DDTP_MODE_BARE && rq_ddtp != ddtp) {
			dev_warn(dev, "DDTP update failed hw: %llx vs %llx\n", ddtp, rq_ddtp);
			return -EINVAL;
		}

		/* DDTP mode has been accepted. */
		if (rq_mode == mode)
			break;

		/*
		 * Mode field is WARL, an IOMMU may support a subset of
		 * directory table levels in which case if we tried to set
		 * an unsupported number of levels we'll readback either
		 * a valid xLVL or off/bare. If we got off/bare, try again
		 * with a smaller xLVL.
		 */
		if (mode < RISCV_IOMMU_DDTP_MODE_1LVL &&
		    rq_mode > RISCV_IOMMU_DDTP_MODE_1LVL) {
			dev_dbg(dev, "DDTP hw mode %u vs %u\n", mode, rq_mode);
			rq_mode--;
			continue;
		}

		/*
		 * We tried all supported modes and IOMMU hardware failed to
		 * accept new settings, something went very wrong since off/bare
		 * and at least one xLVL must be supported.
		 */
		dev_warn(dev, "DDTP hw mode %u, failed to set %u\n", mode, ddtp_mode);
		return -EINVAL;
	} while (1);

	iommu->ddt_mode = mode;
	if (mode != ddtp_mode)
		dev_warn(dev, "DDTP failover to %u mode, requested %u\n",
			 mode, ddtp_mode);

	/* Invalidate device context cache */
	riscv_iommu_cmd_iodir_inval_ddt(&cmd);
	WARN_ON(riscv_iommu_queue_send(&iommu->cmdq, &cmd, 0));

	/* Invalidate address translation cache */
	riscv_iommu_cmd_inval_vma(&cmd);
	WARN_ON(riscv_iommu_queue_send(&iommu->cmdq, &cmd, 0));

	/* IOFENCE.C */
	riscv_iommu_cmd_iofence(&cmd);
	return riscv_iommu_queue_send(&iommu->cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
}

static int riscv_iommu_ddt_alloc(struct riscv_iommu_device *iommu)
{
	u64 ddtp;
	unsigned int mode;

	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);

	if (ddtp & RISCV_IOMMU_DDTP_BUSY)
		return -EBUSY;

	/*
	 * It is optional for the hardware to report a fixed address for device
	 * directory root page when DDT.MODE is OFF or BARE.
	 */
	mode = FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp);
	if (mode != RISCV_IOMMU_DDTP_MODE_BARE && mode != RISCV_IOMMU_DDTP_MODE_OFF) {
		/* Use WARL to discover hardware fixed DDT PPN */
		riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_DDTP,
				   FIELD_PREP(RISCV_IOMMU_DDTP_MODE, mode));
		riscv_iommu_readl_timeout(iommu, RISCV_IOMMU_REG_DDTP,
					  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
					  10, RISCV_IOMMU_DDTP_TIMEOUT);
		if (ddtp & RISCV_IOMMU_DDTP_BUSY)
			return -EBUSY;

		iommu->ddt_phys = ppn_to_phys(ddtp);
		if (iommu->ddt_phys)
			iommu->ddt_root = devm_ioremap(iommu->dev, iommu->ddt_phys, PAGE_SIZE);
		if (iommu->ddt_root)
			memset(iommu->ddt_root, 0, PAGE_SIZE);
	}

	if (!iommu->ddt_root) {
		iommu->ddt_root = (u64 *)riscv_iommu_get_pages(iommu, 0);
		iommu->ddt_phys = __pa(iommu->ddt_root);
	}

	if (!iommu->ddt_root)
		return -ENOMEM;

	return 0;
}

static void riscv_iommu_detach_device(struct device *dev, struct riscv_iommu_dc *dc)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_queue *cmdq = &iommu->cmdq;
	struct riscv_iommu_command cmd;
	u64 tc, ta, fsc;

	if (!ep->attached)
		return;

	/* Invalidate device context */
	fsc = READ_ONCE(dc->fsc);
	ta = READ_ONCE(dc->ta);
	tc = READ_ONCE(dc->tc);
	dc->tc = 0;

	if (!(tc & RISCV_IOMMU_DC_TC_V))
		return;

	/* Make device context updates globaly observable. */
	wmb();

	/* Invalidate device context cache */
	riscv_iommu_cmd_iodir_inval_ddt(&cmd);
	riscv_iommu_cmd_iodir_set_did(&cmd, ep->devid);
	WARN_ON(riscv_iommu_queue_send(cmdq, &cmd, 0));

	if (FIELD_GET(RISCV_IOMMU_DC_FSC_MODE, fsc) > RISCV_IOMMU_DC_FSC_MODE_BARE) {
		/* Invalidate address translation cache */
		riscv_iommu_cmd_inval_vma(&cmd);
		riscv_iommu_cmd_inval_set_pscid(&cmd,
						FIELD_GET(RISCV_IOMMU_DC_TA_PSCID, ta));
		WARN_ON(riscv_iommu_queue_send(cmdq, &cmd, 0));
	}

	if (ep->pasid_enabled) {
		pci_disable_pasid(to_pci_dev(dev));
		ep->pasid_enabled = false;
	}

	if (ep->ats_enabled) {
		list_del(&ep->ats_link);
		riscv_iommu_cmd_ats_inval(&cmd);
		riscv_iommu_cmd_ats_set_all(&cmd, true);
		riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
		pci_disable_ats(to_pci_dev(dev));
		ep->ats_enabled = false;
	}

	/* IOFENCE.C */
	riscv_iommu_cmd_iofence(&cmd);
	WARN_ON(riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT));

	ep->attached = false;
}

/*
 * IOVA page translation tree management.
 */

#define IOMMU_PAGE_SIZE_4K     BIT_ULL(12)
#define IOMMU_PAGE_SIZE_2M     BIT_ULL(21)
#define IOMMU_PAGE_SIZE_1G     BIT_ULL(30)
#define IOMMU_PAGE_SIZE_512G   BIT_ULL(39)

#define PT_SHIFT (PAGE_SHIFT - ilog2(sizeof(pte_t)))

static void riscv_iommu_flush_iotlb_all(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_queue *cmdq = &domain->iommu->cmdq;
	struct riscv_iommu_endpoint *ep;
	struct riscv_iommu_command cmd;

	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
	riscv_iommu_queue_send(cmdq, &cmd, 0);

	list_for_each_entry(ep, &domain->ats_devs, ats_link) {
		riscv_iommu_cmd_ats_inval(&cmd);
		riscv_iommu_cmd_ats_set_all(&cmd, true);
		riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
}

static void riscv_iommu_iotlb_sync(struct iommu_domain *iommu_domain,
				   struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_queue *cmdq = &domain->iommu->cmdq;
	struct riscv_iommu_endpoint *ep;
	struct riscv_iommu_command cmd;
	unsigned long iova;

	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);

	for (iova = gather->start; iova <= gather->end; iova += PAGE_SIZE) {
		riscv_iommu_cmd_inval_set_addr(&cmd, iova);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	list_for_each_entry(ep, &domain->ats_devs, ats_link) {
		riscv_iommu_cmd_ats_inval(&cmd);
		riscv_iommu_cmd_ats_set_range(&cmd, gather->start, gather->end, true);
		riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);
}

static inline size_t get_page_size(size_t size)
{
	if (size >= IOMMU_PAGE_SIZE_512G)
		return IOMMU_PAGE_SIZE_512G;
	if (size >= IOMMU_PAGE_SIZE_1G)
		return IOMMU_PAGE_SIZE_1G;
	if (size >= IOMMU_PAGE_SIZE_2M)
		return IOMMU_PAGE_SIZE_2M;
	return IOMMU_PAGE_SIZE_4K;
}

#define _io_pte_present(pte)	((pte) & (_PAGE_PRESENT | _PAGE_PROT_NONE))
#define _io_pte_leaf(pte)	((pte) & _PAGE_LEAF)
#define _io_pte_none(pte)	((pte) == 0)
#define _io_pte_entry(pn, prot)	((_PAGE_PFN_MASK & ((pn) << _PAGE_PFN_SHIFT)) | (prot))

static void riscv_iommu_pte_free(struct riscv_iommu_domain *domain,
				 unsigned long pte, struct list_head *freelist)
{
	unsigned long *ptr;
	int i;

	if (!_io_pte_present(pte) || _io_pte_leaf(pte))
		return;

	ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));

	/* Recursively free all sub page table pages */
	for (i = 0; i < PTRS_PER_PTE; i++) {
		pte = READ_ONCE(ptr[i]);
		if (!_io_pte_none(pte) && cmpxchg_relaxed(ptr + i, pte, 0) == pte)
			riscv_iommu_pte_free(domain, pte, freelist);
	}

	if (freelist)
		list_add_tail(&virt_to_page(ptr)->lru, freelist);
	else
		free_page((unsigned long)ptr);
}

static unsigned long *riscv_iommu_pte_alloc(struct riscv_iommu_domain *domain,
					    unsigned long iova, size_t pgsize, gfp_t gfp)
{
	unsigned long *ptr = (unsigned long *)domain->pgd_root;
	unsigned long pte, old;
	int level = domain->pt_mode - RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39 + 2;
	struct page *page;

	do {
		const int shift = PAGE_SHIFT + PT_SHIFT * level;

		ptr += ((iova >> shift) & (PTRS_PER_PTE - 1));
		/*
		 * Note: returned entry might be a non-leaf if there was existing mapping
		 * with smaller granularity. Up to the caller to replace and invalidate.
		 */
		if (((size_t)1 << shift) == pgsize)
			return ptr;
pte_retry:
		pte = READ_ONCE(*ptr);
		/*
		 * This is very likely incorrect as we should not be adding new mapping
		 * with smaller granularity on top of existing 2M/1G mapping. Fail.
		 */
		if (_io_pte_present(pte) && _io_pte_leaf(pte))
			return NULL;
		/*
		 * Non-leaf entry is missing, allocate and try to add to the page table.
		 * This might race with other mappings, retry on error.
		 */
		if (_io_pte_none(pte)) {
			page = alloc_pages_node(domain->numa_node,
						GFP_KERNEL_ACCOUNT | __GFP_ZERO | gfp, 0);
			if (!page)
				return NULL;
			old = pte;
			pte = _io_pte_entry(page_to_pfn(page), _PAGE_TABLE);
			if (cmpxchg_relaxed(ptr, old, pte) != old) {
				__free_pages(page, 0);
				goto pte_retry;
			}
		}
		ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));
	} while (level-- > 0);

	return NULL;
}

static unsigned long *riscv_iommu_pte_fetch(struct riscv_iommu_domain *domain,
					    unsigned long iova, size_t *pte_pgsize)
{
	unsigned long *ptr = (unsigned long *)domain->pgd_root;
	unsigned long pte;
	int level = domain->pt_mode - RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39 + 2;

	do {
		const int shift = PAGE_SHIFT + PT_SHIFT * level;

		ptr += ((iova >> shift) & (PTRS_PER_PTE - 1));
		pte = READ_ONCE(*ptr);
		if (_io_pte_present(pte) && _io_pte_leaf(pte)) {
			*pte_pgsize = (size_t)1 << shift;
			return ptr;
		}
		if (_io_pte_none(pte))
			return NULL;
		ptr = (unsigned long *)pfn_to_virt(__page_val_to_pfn(pte));
	} while (level-- > 0);

	return NULL;
}

static int riscv_iommu_map_pages(struct iommu_domain *iommu_domain,
				 unsigned long iova, phys_addr_t phys,
				 size_t pgsize, size_t pgcount, int prot,
				 gfp_t gfp, size_t *mapped)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	size_t size = 0;
	size_t page_size = get_page_size(pgsize);
	unsigned long *ptr;
	unsigned long pte, old, pte_prot;

	if (!(prot & IOMMU_WRITE))
		pte_prot = _PAGE_BASE | _PAGE_READ;
	else
		pte_prot = _PAGE_BASE | _PAGE_READ | _PAGE_WRITE | _PAGE_DIRTY;

	while (pgcount) {
		ptr = riscv_iommu_pte_alloc(domain, iova, page_size, gfp);
		if (!ptr) {
			*mapped = size;
			return -ENOMEM;
		}

		old = READ_ONCE(*ptr);
		pte = _io_pte_entry(phys_to_pfn(phys), pte_prot);
		if (cmpxchg_relaxed(ptr, old, pte) != old)
			continue;

		/* TODO: deal with __old being a valid non-leaf entry */

		size += page_size;
		iova += page_size;
		phys += page_size;
		--pgcount;
	}

	*mapped = size;

	return 0;
}

static size_t riscv_iommu_unmap_pages(struct iommu_domain *iommu_domain,
				      unsigned long iova, size_t pgsize, size_t pgcount,
				      struct iommu_iotlb_gather *gather)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	size_t size = pgcount << __ffs(pgsize);
	unsigned long *ptr, old, pte;
	size_t unmapped = 0;
	size_t pte_size;

	while (unmapped < size) {
		ptr = riscv_iommu_pte_fetch(domain, iova, &pte_size);
		if (!ptr)
			return unmapped;

		old = READ_ONCE(*ptr);
		pte = cmpxchg_relaxed(ptr, old, 0);
		if (old != pte)
			continue;

		iommu_iotlb_gather_add_page(&domain->domain, gather, iova,
					    pte_size);

		iova = (iova & ~(pte_size - 1)) + pte_size;
		/* unmap unalligned IOVA ? */
		unmapped += pte_size;
	}

	return unmapped;
}

static phys_addr_t riscv_iommu_iova_to_phys(struct iommu_domain *iommu_domain, dma_addr_t iova)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	unsigned long pte_size;
	unsigned long *ptr;

	ptr = riscv_iommu_pte_fetch(domain, iova, &pte_size);
	if (_io_pte_none(*ptr) || !_io_pte_present(*ptr))
		return 0;

	return pfn_to_phys(__page_val_to_pfn(*ptr)) | (iova & (pte_size - 1));
}

static void riscv_iommu_free_paging_domain(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if (domain->pgd_root) {
		const unsigned long pfn = virt_to_pfn(domain->pgd_root);

		riscv_iommu_pte_free(domain, _io_pte_entry(pfn, _PAGE_TABLE), NULL);
	}

	if ((int)domain->pscid > 0)
		ida_free(&riscv_iommu_pscids, domain->pscid);

	kfree(domain);
}

static bool riscv_iommu_pt_supported(struct riscv_iommu_device *iommu, int pt_mode)
{
	switch (pt_mode) {
	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV39:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV39;

	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV48:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV48;

	case RISCV_IOMMU_DC_FSC_IOSATP_MODE_SV57:
		return iommu->caps & RISCV_IOMMU_CAP_S_SV57;
	}
	return false;
}

static int riscv_iommu_attach_paging_domain(struct iommu_domain *iommu_domain,
					    struct device *dev)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_dc *dc;
	struct riscv_iommu_pc *pc;
	struct page *page;
	u64 tc;

	if (!riscv_iommu_pt_supported(iommu, domain->pt_mode))
		return -ENODEV;

	if (!domain->iommu)
		domain->iommu = iommu;
	else if (domain->iommu != iommu)
		return -ENODEV;

	if (domain->numa_node == NUMA_NO_NODE)
		domain->numa_node = dev_to_node(iommu->dev);

	dc = riscv_iommu_get_dc(iommu, ep->devid);
	if (!dc)
		return -ENODEV;

	riscv_iommu_detach_device(dev, dc);

	if (!domain->pgd_root) {
		page = alloc_pages_node(domain->numa_node,
					GFP_KERNEL_ACCOUNT | __GFP_ZERO, 0);
		if (!page)
			return -ENOMEM;
		domain->pgd_root = (unsigned long)page_to_virt(page);
	}

	tc = RISCV_IOMMU_DC_TC_V;

	if (ep->ats_supported)
		ep->ats_enabled = pci_enable_ats(to_pci_dev(dev), PAGE_SHIFT) == 0;

	if (ep->ats_enabled) {
		list_add(&domain->ats_devs, &ep->ats_link);
		tc |= RISCV_IOMMU_DC_TC_EN_ATS;
	}

	if (ep->ats_enabled && ep->pasid_supported)
		ep->pasid_enabled = pci_enable_pasid(to_pci_dev(dev),
						     pci_pasid_features(to_pci_dev(dev))) == 0;

	dc->iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE, RISCV_IOMMU_DC_IOHGATP_MODE_BARE);

	if (ep->pasid_enabled) {
		if (!ep->pc) {
			page = alloc_pages_node(domain->numa_node,
						GFP_KERNEL_ACCOUNT | __GFP_ZERO, 0);
			if (!page)
				return -ENOMEM;
			ep->pc = (struct riscv_iommu_pc *)page_address(page);
		}
		tc |= RISCV_IOMMU_DC_TC_DPE | RISCV_IOMMU_DC_TC_PDTV;

		pc = ep->pc;
		pc->fsc = FIELD_PREP(RISCV_IOMMU_PC_FSC_MODE, domain->pt_mode) |
			  FIELD_PREP(RISCV_IOMMU_PC_FSC_PPN, virt_to_pfn(domain->pgd_root));
		pc->ta  = FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, domain->pscid) | RISCV_IOMMU_PC_TA_V;
		dc->fsc = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_PDTP_MODE_PD8) |
			  FIELD_PREP(RISCV_IOMMU_DC_FSC_PPN, virt_to_pfn(ep->pc));
		dc->ta  = 0;
	} else {
		dc->fsc     = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, domain->pt_mode) |
			      FIELD_PREP(RISCV_IOMMU_DC_FSC_PPN, virt_to_pfn(domain->pgd_root));
		dc->ta      = FIELD_PREP(RISCV_IOMMU_DC_TA_PSCID, domain->pscid);
	}

	/* Make device context data globaly observable before marking it valid. */
	wmb();
	dc->tc = tc;
	ep->attached = true;

	return 0;
}

static const struct iommu_domain_ops riscv_iommu_paging_domain_ops = {
	.attach_dev = riscv_iommu_attach_paging_domain,
	.free = riscv_iommu_free_paging_domain,
	.map_pages = riscv_iommu_map_pages,
	.unmap_pages = riscv_iommu_unmap_pages,
	.iova_to_phys = riscv_iommu_iova_to_phys,
	.iotlb_sync = riscv_iommu_iotlb_sync,
	.flush_iotlb_all = riscv_iommu_flush_iotlb_all,
};

struct riscv_iommu_mn {
	struct mmu_notifier		mn;
	refcount_t			refs;
	int				pasid;
	struct list_head		list;
	struct riscv_iommu_domain	*domain;
	struct riscv_iommu_endpoint	*endpoint;
};

static void riscv_iommu_free_sva_domain(struct iommu_domain *iommu_domain)
{
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);

	if ((int)domain->pscid > 0)
		ida_free(&riscv_iommu_pscids, domain->pscid);

	kfree(domain);
}

static void riscv_iommu_mm_release(struct mmu_notifier *mn, struct mm_struct *mm)
{
	/* FROM INTEL SVM
	 *
	 * This might end up being called from exit_mmap(), *before* the page
	 * tables are cleared. And __mmu_notifier_release() will delete us from
	 * the list of notifiers so that our invalidate_range() callback doesn't
	 * get called when the page tables are cleared. So we need to protect
	 * against hardware accessing those page tables.
	 *
	 * We do it by clearing the entry in the PASID table and then flushing
	 * the IOTLB and the PASID table caches. This might upset hardware;
	 * perhaps we'll want to point the PASID to a dummy PGD (like the zero
	 * page) so that we end up taking a fault that the hardware really
	 * *has* to handle gracefully without affecting other processes.
	 *
	 *

	rcu_read_lock();
	list_for_each_entry_rcu(sdev, &svm->devs, list)
		intel_pasid_tear_down_entry(sdev->iommu, sdev->dev,
					    svm->pasid, true);
	rcu_read_unlock();

	 */
}

static int riscv_iommu_mm_invalidate(struct mmu_notifier *mn,
				     const struct mmu_notifier_range *range)
{
	struct riscv_iommu_mn *notifier = container_of(mn, struct riscv_iommu_mn, mn);
	struct riscv_iommu_endpoint *ep = notifier->endpoint;
	struct riscv_iommu_domain *domain = notifier->domain;
	struct riscv_iommu_queue *cmdq = &domain->iommu->cmdq;
	struct riscv_iommu_command cmd;
	unsigned long iova;

	if ( /* disable for now, as teardown is not implemented */ 1 )
		return 0;

	if (range->end > range->start) {
		for (iova = range->start; iova < range->end; iova += PAGE_SIZE) {
			riscv_iommu_cmd_inval_vma(&cmd);
			riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
			riscv_iommu_cmd_inval_set_addr(&cmd, iova);
			riscv_iommu_queue_send(cmdq, &cmd, 0);
		}
	} else {
		riscv_iommu_cmd_inval_vma(&cmd);
		riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	if (ep->ats_enabled) {
		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
		if (range->end > range->start) {
			/* Cover only the range that is needed */
			for (iova = range->start; iova < range->end; iova += PAGE_SIZE) {
				riscv_iommu_cmd_ats_inval(&cmd);
				riscv_iommu_cmd_inval_set_addr(&cmd, iova);
				riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
				riscv_iommu_cmd_ats_set_pid(&cmd, notifier->pasid);
				riscv_iommu_queue_send(cmdq, &cmd, 0);
			}
		} else {
			riscv_iommu_cmd_ats_inval(&cmd);
			riscv_iommu_cmd_ats_set_all(&cmd, true);
			riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
			riscv_iommu_cmd_ats_set_pid(&cmd, notifier->pasid);
			riscv_iommu_queue_send(cmdq, &cmd, 0);
		}
	}

	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);

	return 0;
}

static const struct mmu_notifier_ops riscv_iommu_mm_uops = {
	.release = riscv_iommu_mm_release,
	.invalidate_range_start = riscv_iommu_mm_invalidate,
};

// this will be called with the same iommu_dmain for each device in the domain group!
// protected by mutex_lock(&group->mutex);
static int riscv_iommu_set_dev_pasid(struct iommu_domain *iommu_domain,
				     struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct iommu_domain *dev_iommu_domain = iommu_get_domain_for_dev(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_pc *pc;
	struct riscv_iommu_mn *mn;

	/* Process Context table should be set for pasid enabled endpoints. */
	if (!ep || !ep->sva_enabled) {
		dev_err(dev, "no sva enabled!\n");
		return -ENODEV;
	}

	/* no PC yet. move bind and set domain @ pasid */
	if (!ep->pc) {
		dev_err(dev, "no pasid table attached!\n");
		return -ENOMEM;
	}

	if (!dev_iommu_domain) {
		dev_err(dev, "no primary domain attached!\n");
		return -ENOMEM;
	}

	if (!domain->iommu)
		domain->iommu = iommu;
	else if (domain->iommu != iommu)
		return -ENODEV;

	mn = kzalloc(sizeof(*mn), GFP_KERNEL);
	if (!mn)
		return -ENOMEM;

	mn->domain = domain;
	mn->endpoint = ep;
	mn->pasid = pasid;
	mn->mn.ops = &riscv_iommu_mm_uops;

	/* register mm notifier */
	if (mmu_notifier_register(&mn->mn, iommu_domain->mm)) {
		dev_err(dev, "can't register notifier mn!\n");
		return -ENODEV;
	}

	pc = ep->pc + pasid;

	if (pc->ta & RISCV_IOMMU_PC_TA_V)
		dev_err(dev, "PASID %x already configured for device %x\n", pasid, ep->devid);

	pc->fsc = FIELD_PREP(RISCV_IOMMU_PC_FSC_MODE, domain->pt_mode) |
		  FIELD_PREP(RISCV_IOMMU_PC_FSC_PPN, virt_to_pfn(iommu_domain->mm->pgd));
	pc->ta  = FIELD_PREP(RISCV_IOMMU_PC_TA_PSCID, domain->pscid) | RISCV_IOMMU_PC_TA_V;

	return 0;
}

static void riscv_iommu_remove_dev_pasid(struct device *dev, ioasid_t pasid)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_queue *cmdq = &iommu->cmdq;
	struct iommu_domain *iommu_domain = iommu_get_domain_for_dev_pasid(dev, pasid, 0);
	struct riscv_iommu_domain *domain = iommu_domain_to_riscv(iommu_domain);
	struct riscv_iommu_pc *pc;
	struct riscv_iommu_command cmd;

	pc = ep->pc + pasid;
	pc->fsc     = 0;
	pc->ta      = 0;

	riscv_iommu_cmd_iodir_inval_pdt(&cmd);
	riscv_iommu_cmd_iodir_set_did(&cmd, ep->devid);
	riscv_iommu_cmd_iodir_set_pid(&cmd, pasid);
	riscv_iommu_queue_send(cmdq, &cmd, 0);

	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_queue_send(cmdq, &cmd, 0);

	riscv_iommu_cmd_inval_vma(&cmd);
	riscv_iommu_cmd_inval_set_pscid(&cmd, domain->pscid);
	riscv_iommu_queue_send(cmdq, &cmd, 0);

	if (ep->ats_enabled) {
		riscv_iommu_cmd_iofence(&cmd);
		riscv_iommu_queue_send(cmdq, &cmd, 0);

		riscv_iommu_cmd_ats_inval(&cmd);
		riscv_iommu_cmd_ats_set_all(&cmd, true);
		riscv_iommu_cmd_ats_set_devid(&cmd, ep->devid);
		riscv_iommu_queue_send(cmdq, &cmd, 0);
	}

	riscv_iommu_cmd_iofence(&cmd);
	riscv_iommu_queue_send(cmdq, &cmd, RISCV_IOMMU_QUEUE_TIMEOUT);

	dev_warn(dev, "PASID %x PSCID: %x removed\n", pasid, domain->pscid);
}

static const struct iommu_domain_ops riscv_iommu_sva_domain_ops = {
	.set_dev_pasid = riscv_iommu_set_dev_pasid,
	.free = riscv_iommu_free_sva_domain,
};

static struct iommu_domain *riscv_iommu_domain_alloc(unsigned int type)
{
	struct iommu_domain_geometry *geometry;
	struct riscv_iommu_domain *domain;
	int pscid;

	if (type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_SVA &&
	    type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	pscid = ida_alloc_range(&riscv_iommu_pscids, 1,
				RISCV_IOMMU_MAX_PSCID - 1, GFP_KERNEL);
	if (pscid < 0)
		return NULL;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain) {
		ida_free(&riscv_iommu_pscids, pscid);
		return NULL;
	}
	INIT_LIST_HEAD(&domain->ats_devs);

	/*
	 * Note: RISC-V Privilege spec mandates that virtual addresses
	 * need to be sign-extended, so if (VA_BITS - 1) is set, all
	 * bits >= VA_BITS need to also be set or else we'll get a
	 * page fault. However the code that creates the mappings
	 * above us (e.g. iommu_dma_alloc_iova()) won't do that for us
	 * for now, so we'll end up with invalid virtual addresses
	 * to map. As a workaround until we get this sorted out
	 * limit the available virtual addresses to VA_BITS - 1.
	 */
	geometry = &domain->domain.geometry;
	geometry->aperture_start = 0;
	geometry->aperture_end = DMA_BIT_MASK(VA_BITS - 1);
	geometry->force_aperture = true;

	/*
	 * Follow system address translation mode.
	 * RISC-V IOMMU ATP mode values match RISC-V CPU SATP mode values.
	 */
	domain->pt_mode = satp_mode >> SATP_MODE_SHIFT;
	domain->numa_node = NUMA_NO_NODE;
	domain->pscid = pscid;
	if (type == IOMMU_DOMAIN_SVA)
		domain->domain.ops = &riscv_iommu_sva_domain_ops;
	else
		domain->domain.ops = &riscv_iommu_paging_domain_ops;

	return &domain->domain;
}

static int riscv_iommu_attach_identity_domain(struct iommu_domain *domain,
					      struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc;

	/* Global pass-through already enabled, do nothing. */
	if (iommu->ddt_mode == RISCV_IOMMU_DDTP_MODE_BARE)
		return 0;

	dc = riscv_iommu_get_dc(iommu, ep->devid);

	if (!dc)
		return -ENODEV;

	riscv_iommu_detach_device(dev, dc);

	dc->iohgatp = FIELD_PREP(RISCV_IOMMU_DC_IOHGATP_MODE, RISCV_IOMMU_DC_IOHGATP_MODE_BARE);
	dc->fsc = FIELD_PREP(RISCV_IOMMU_DC_FSC_MODE, RISCV_IOMMU_DC_FSC_MODE_BARE);
	dc->ta = 0;
	/* Make device context data globaly observable before marking it valid. */
	wmb();
	dc->tc = RISCV_IOMMU_DC_TC_V;
	ep->attached = true;

	return 0;
}

static struct iommu_domain riscv_iommu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &(const struct iommu_domain_ops) {
		.attach_dev = riscv_iommu_attach_identity_domain,
	}
};

static int riscv_iommu_device_domain_type(struct device *dev)
{
	return 0;
}

static struct iommu_group *riscv_iommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	return generic_device_group(dev);
}

static int riscv_iommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static int riscv_iommu_dev_enable_feat(struct device *dev, enum iommu_dev_features feat)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	int rc;

	if (feat == IOMMU_DEV_FEAT_SVA) {
		if (!ep || !ep->pasid_enabled)
			return -ENODEV;
		if (!iommu->pq_work)
			return -ENODEV;

		rc = iopf_queue_add_device(iommu->pq_work, dev);
		if (rc)
			return rc;
		rc = iommu_register_device_fault_handler(dev, iommu_queue_iopf, dev);
		if (rc) {
			iopf_queue_remove_device(iommu->pq_work, dev);
			return rc;
		}
		ep->sva_enabled = true;
		return 0;
	}
	if (feat == IOMMU_DEV_FEAT_IOPF) {
		if (ep && ep->pasid_enabled)
			return 0;
	}

	return -ENODEV;
}

static int riscv_iommu_dev_disable_feat(struct device *dev, enum iommu_dev_features feat)
{
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);

	if (feat == IOMMU_DEV_FEAT_IOPF)
		return 0;

	if (feat == IOMMU_DEV_FEAT_SVA) {
		ep->sva_enabled = false;
		return iommu_unregister_device_fault_handler(dev);
	}

	return -ENODEV;
}

static struct iommu_device *riscv_iommu_probe_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct pci_dev *pdev = dev_is_pci(dev) ? to_pci_dev(dev) : NULL;
	struct riscv_iommu_device *iommu;
	struct riscv_iommu_endpoint *ep;
	unsigned int devid;

	/* Early bus-scan exit, will retry. */
	if (!fwspec || !fwspec->iommu_fwnode)
		return ERR_PTR(-ENODEV);

	/* Pending IOMMU driver initialization, will retry */
	if (!fwspec->iommu_fwnode->dev)
		return ERR_PTR(-ENODEV);

	iommu = dev_get_drvdata(fwspec->iommu_fwnode->dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	if (pdev)
		devid = pci_dev_id(pdev) | pci_domain_nr(pdev->bus) << 16;
	else if (fwspec->num_ids)
		devid = fwspec->ids[0];
	else
		return ERR_PTR(-ENODEV);

	ep = kzalloc(sizeof(*ep), GFP_KERNEL_ACCOUNT);
	if (!ep)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&ep->ats_link);
	ep->devid = devid;

	if (pdev) {
		if (iommu->caps & RISCV_IOMMU_CAP_ATS)
			ep->ats_supported = pci_ats_supported(pdev);
		if (ep->ats_supported)
			ep->ats_queue_depth = pci_ats_queue_depth(pdev);
		if (iommu->iommu.max_pasids)
			ep->pasid_supported = pci_pasid_features(pdev) >= 0;
		if (ep->pasid_supported)
			ep->max_pasid = pci_max_pasids(pdev);
		if (ep->max_pasid <= 0)
			ep->pasid_supported = false;
		if (ep->max_pasid > 0 && ep->max_pasid < iommu->iommu.max_pasids)
			iommu->iommu.max_pasids = ep->max_pasid;
	}

	dev_iommu_priv_set(dev, ep);

	return &iommu->iommu;
}

static void riscv_iommu_probe_finalize(struct device *dev)
{
	iommu_setup_dma_ops(dev, 0, U64_MAX);
}

static void riscv_iommu_release_device(struct device *dev)
{
	struct riscv_iommu_device *iommu = dev_to_iommu(dev);
	struct riscv_iommu_endpoint *ep = dev_iommu_priv_get(dev);
	struct riscv_iommu_dc *dc = riscv_iommu_get_dc(iommu, ep->devid);

	riscv_iommu_detach_device(dev, dc);
	dev_iommu_priv_set(dev, NULL);
	kfree(ep);
}

static const struct iommu_ops riscv_iommu_ops = {
	.owner = THIS_MODULE,
	.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G,
	.of_xlate = riscv_iommu_of_xlate,
	.identity_domain = &riscv_iommu_identity_domain,
	.domain_alloc = riscv_iommu_domain_alloc,
	.def_domain_type = riscv_iommu_device_domain_type,
	.device_group = riscv_iommu_device_group,
	.probe_device = riscv_iommu_probe_device,
	.probe_finalize = riscv_iommu_probe_finalize,
	.release_device = riscv_iommu_release_device,
	.remove_dev_pasid = riscv_iommu_remove_dev_pasid,
	.dev_enable_feat = riscv_iommu_dev_enable_feat,
	.dev_disable_feat = riscv_iommu_dev_disable_feat,
};

void riscv_iommu_remove(struct riscv_iommu_device *iommu)
{
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
	riscv_iommu_queue_disable(&iommu->cmdq);
	riscv_iommu_queue_disable(&iommu->fltq);
	iommu_device_unregister(&iommu->iommu);
	iommu_device_sysfs_remove(&iommu->iommu);
	riscv_iommu_debugfs_remove(iommu);
}

static int riscv_iommu_init_check(struct riscv_iommu_device *iommu)
{
	u64 ddtp;

	/* Hardware must be configured in OFF | BARE mode at system initialization. */
	riscv_iommu_readq_timeout(iommu, RISCV_IOMMU_REG_DDTP,
				  ddtp, !(ddtp & RISCV_IOMMU_DDTP_BUSY),
				  10, RISCV_IOMMU_DDTP_TIMEOUT);
	if (FIELD_GET(RISCV_IOMMU_DDTP_MODE, ddtp) > RISCV_IOMMU_DDTP_MODE_BARE)
		return -EBUSY;

	/* Configure accesses to in-memory data structures for CPU-native byte order. */
	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE)) {
		if (!(iommu->caps & RISCV_IOMMU_CAP_END))
			return -EINVAL;
		riscv_iommu_writel(iommu, RISCV_IOMMU_REG_FCTL,
				   iommu->fctl ^ RISCV_IOMMU_FCTL_BE);
		iommu->fctl = riscv_iommu_readl(iommu, RISCV_IOMMU_REG_FCTL);
		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN) != !!(iommu->fctl & RISCV_IOMMU_FCTL_BE))
			return -EINVAL;
	}

	/* Set 1:1 mapping for interrupt vectors if available */
	iommu->ivec = iommu->irqs_count < 4 ? 0 :
		      FIELD_PREP(RISCV_IOMMU_IVEC_CIV,  0) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_FIV,  1) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_PMIV, 2) |
		      FIELD_PREP(RISCV_IOMMU_IVEC_PIV,  3);
	riscv_iommu_writeq(iommu, RISCV_IOMMU_REG_IVEC, iommu->ivec);

	/* Check PASID capabilities */
	if (iommu->caps & RISCV_IOMMU_CAP_PD20)
		iommu->iommu.max_pasids = 1u << 20;
	else if (iommu->caps & RISCV_IOMMU_CAP_PD17)
		iommu->iommu.max_pasids = 1u << 17;
	else if (iommu->caps & RISCV_IOMMU_CAP_PD8)
		iommu->iommu.max_pasids = 1u << 8;

	dma_set_mask_and_coherent(iommu->dev,
				  DMA_BIT_MASK(FIELD_GET(RISCV_IOMMU_CAP_PAS, iommu->caps)));

	return 0;
}

int riscv_iommu_init(struct riscv_iommu_device *iommu)
{
	int rc;

	RISCV_IOMMU_QUEUE_INIT(&iommu->cmdq, CQ);
	RISCV_IOMMU_QUEUE_INIT(&iommu->fltq, FQ);

	rc = riscv_iommu_init_check(iommu);
	if (rc)
		return dev_err_probe(iommu->dev, rc, "unexpected device state\n");

	riscv_iommu_debugfs_setup(iommu);

	rc = riscv_iommu_ddt_alloc(iommu);
	if (WARN(rc, "cannot allocate device directory\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->cmdq, sizeof(struct riscv_iommu_command));
	if (WARN(rc, "cannot allocate command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_alloc(iommu, &iommu->fltq, sizeof(struct riscv_iommu_fq_record));
	if (WARN(rc, "cannot allocate fault queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->cmdq, riscv_iommu_cmdq_process);
	if (WARN(rc, "cannot enable command queue\n"))
		goto err_init;

	rc = riscv_iommu_queue_enable(&iommu->fltq, riscv_iommu_fltq_process);
	if (WARN(rc, "cannot enable fault queue\n"))
		goto err_init;

	iommu->pq_work = iopf_queue_alloc(dev_name(iommu->dev));

	rc = riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_MAX);
	if (WARN(rc, "cannot enable iommu device\n"))
		goto err_init;

	rc = iommu_device_sysfs_add(&iommu->iommu, NULL, NULL, "riscv-iommu@%s",
				    dev_name(iommu->dev));
	if (WARN(rc, "cannot register sysfs interface\n"))
		goto err_sysfs;

	rc = iommu_device_register(&iommu->iommu, &riscv_iommu_ops, iommu->dev);
	if (WARN(rc, "cannot register iommu interface\n"))
		goto err_iommu;

	return 0;

err_iommu:
	iommu_device_sysfs_remove(&iommu->iommu);
err_sysfs:
	riscv_iommu_set_ddtp_mode(iommu, RISCV_IOMMU_DDTP_MODE_OFF);
err_init:
	riscv_iommu_queue_disable(&iommu->fltq);
	riscv_iommu_queue_disable(&iommu->cmdq);
	riscv_iommu_debugfs_remove(iommu);
	return rc;
}

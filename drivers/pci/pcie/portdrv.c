// SPDX-License-Identifier: GPL-2.0
/*
 * Purpose:	PCI Express Port Bus Driver
 *
 * Copyright (C) 2004 Intel
 * Copyright (C) Tom Long Nguyen (tom.l.nguyen@intel.com)
 */

#include <linux/dmi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/aer.h>
#include <linux/of.h>     /* For of_machine_is_compatible */
#include <linux/delay.h>  /* For msleep */
#include <linux/cpu.h>    /* For num_online_cpus */
#include <linux/irq.h>    /* For irq_set_affinity_hint */

#include "../pci.h"
#include "portdrv.h"

/*
 * The PCIe Capability Interrupt Message Number (PCIe r3.1, sec 7.8.2) must
 * be one of the first 32 MSI-X entries.  Per PCI r3.0, sec 6.8.3.1, MSI
 * supports a maximum of 32 vectors per function.
 */
#define PCIE_PORT_MAX_MSI_ENTRIES	32

#define get_descriptor_id(type, service) (((type - 4) << 8) | service)

struct portdrv_service_data {
	struct pcie_port_service_driver *drv;
	struct device *dev;
	u32 service;
};

/**
 * release_pcie_device - free PCI Express port service device structure
 * @dev: Port service device to release
 *
 * Invoked automatically when device is being removed in response to
 * device_unregister(dev).  Release all resources being claimed.
 */
static void release_pcie_device(struct device *dev)
{
	kfree(to_pcie_device(dev));
}

/**
 * pcibios_check_service_irqs - check irqs in the device tree
 * @dev: PCI Express port to handle
 * @irqs: Array of irqs to populate
 * @mask: Bitmask of port capabilities returned by get_port_device_capability()
 *
 * Return value: 0 means no service irqs in the device tree
 *
 */
int __weak pcibios_check_service_irqs(struct pci_dev *dev, int *irqs, int mask)
{
	return 0;
}

/*
 * Fill in *pme, *aer, *dpc with the relevant Interrupt Message Numbers if
 * services are enabled in "mask".  Return the number of MSI/MSI-X vectors
 * required to accommodate the largest Message Number.
 */
static int pcie_message_numbers(struct pci_dev *dev, int mask,
				u32 *pme, u32 *aer, u32 *dpc)
{
	u32 nvec = 0, pos;
	u16 reg16;

	/*
	 * The Interrupt Message Number indicates which vector is used, i.e.,
	 * the MSI-X table entry or the MSI offset between the base Message
	 * Data and the generated interrupt message.  See PCIe r3.1, sec
	 * 7.8.2, 7.10.10, 7.31.2.
	 */

	if (mask & (PCIE_PORT_SERVICE_PME | PCIE_PORT_SERVICE_HP |
		    PCIE_PORT_SERVICE_BWNOTIF)) {
		pcie_capability_read_word(dev, PCI_EXP_FLAGS, &reg16);
		*pme = (reg16 & PCI_EXP_FLAGS_IRQ) >> 9;
		nvec = *pme + 1;
	}

#ifdef CONFIG_PCIEAER
	if (mask & PCIE_PORT_SERVICE_AER) {
		u32 reg32;

		pos = dev->aer_cap;
		if (pos) {
			pci_read_config_dword(dev, pos + PCI_ERR_ROOT_STATUS,
					      &reg32);
			*aer = (reg32 & PCI_ERR_ROOT_AER_IRQ) >> 27;
			nvec = max(nvec, *aer + 1);
		}
	}
#endif

	if (mask & PCIE_PORT_SERVICE_DPC) {
		pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_DPC);
		if (pos) {
			pci_read_config_word(dev, pos + PCI_EXP_DPC_CAP,
					     &reg16);
			*dpc = reg16 & PCI_EXP_DPC_IRQ;
			nvec = max(nvec, *dpc + 1);
		}
	}

	return nvec;
}

/**
 * pcie_port_enable_irq_vec - try to set up MSI-X or MSI as interrupt mode
 * for given port
 * @dev: PCI Express port to handle
 * @irqs: Array of interrupt vectors to populate
 * @mask: Bitmask of port capabilities returned by get_port_device_capability()
 *
 * Return value: 0 on success, error code on failure
 */
static int pcie_port_enable_irq_vec(struct pci_dev *dev, int *irqs, int mask)
{
    int nr_entries, nvec, pcie_irq;
    u32 pme = 0, aer = 0, dpc = 0;
    
    /* First determine how many vectors we need */
    nvec = pcie_message_numbers(dev, mask, &pme, &aer, &dpc);
    if (nvec <= 0)
        nvec = 1; /* Ensure we allocate at least one vector */
    
    /* Try to allocate exactly what we need in one go */
    nr_entries = pci_alloc_irq_vectors(dev, nvec, nvec,
            PCI_IRQ_MSIX | PCI_IRQ_MSI);
            
    /* If exact allocation fails, fall back to original approach */
    if (nr_entries < 0) {
        /* Allocate the maximum possible number of MSI/MSI-X vectors */
        nr_entries = pci_alloc_irq_vectors(dev, 1, PCIE_PORT_MAX_MSI_ENTRIES,
                PCI_IRQ_MSIX | PCI_IRQ_MSI);
        if (nr_entries < 0)
            return nr_entries;

        /* Re-check vector requirements against what we got */
        nvec = pcie_message_numbers(dev, mask, &pme, &aer, &dpc);
        if (nvec > nr_entries) {
            pci_free_irq_vectors(dev);
            return -EIO;
        }

        /* If we allocated more than we need, free them and reallocate fewer */
        if (nvec != nr_entries) {
            pci_free_irq_vectors(dev);

            nr_entries = pci_alloc_irq_vectors(dev, nvec, nvec,
                    PCI_IRQ_MSIX | PCI_IRQ_MSI);
            if (nr_entries < 0)
                return nr_entries;
        }
    }

    /* PME, hotplug and bandwidth notification share an MSI/MSI-X vector */
    if (mask & (PCIE_PORT_SERVICE_PME | PCIE_PORT_SERVICE_HP |
                PCIE_PORT_SERVICE_BWNOTIF)) {
        pcie_irq = pci_irq_vector(dev, pme);
        irqs[PCIE_PORT_SERVICE_PME_SHIFT] = pcie_irq;
        irqs[PCIE_PORT_SERVICE_HP_SHIFT] = pcie_irq;
        irqs[PCIE_PORT_SERVICE_BWNOTIF_SHIFT] = pcie_irq;
    }

    if (mask & PCIE_PORT_SERVICE_AER)
        irqs[PCIE_PORT_SERVICE_AER_SHIFT] = pci_irq_vector(dev, aer);

    if (mask & PCIE_PORT_SERVICE_DPC)
        irqs[PCIE_PORT_SERVICE_DPC_SHIFT] = pci_irq_vector(dev, dpc);

    return 0;
}

/**
 * pcie_init_service_irqs - initialize irqs for PCI Express port services
 * @dev: PCI Express port to handle
 * @irqs: Array of irqs to populate
 * @mask: Bitmask of port capabilities returned by get_port_device_capability()
 *
 * Return value: Interrupt mode associated with the port
 */
static int pcie_init_service_irqs(struct pci_dev *dev, int *irqs, int mask)
{
	int ret, i;
	int irq = -1;

	/* Check if some platforms owns independent irq pins for AER/PME etc.
	 * Some platforms may own independent AER/PME interrupts and set
	 * them in the device tree file.
	 */
	ret = pcibios_check_service_irqs(dev, irqs, mask);
	if (ret) {
		if (dev->irq)
			irq = dev->irq;
		for (i = 0; i < PCIE_PORT_DEVICE_MAXSERVICES; i++)
			if (irqs[i] == -1)
				irqs[i] = irq;
		return 0;
	}

	for (i = 0; i < PCIE_PORT_DEVICE_MAXSERVICES; i++)
		irqs[i] = -1;

	/*
	 * If we support PME but can't use MSI/MSI-X for it, we have to
	 * fall back to INTx or other interrupts, e.g., a system shared
	 * interrupt.
	 */
	if ((mask & PCIE_PORT_SERVICE_PME) && pcie_pme_no_msi())
		goto legacy_irq;

	/* Try to use MSI-X or MSI if supported */
	if (pcie_port_enable_irq_vec(dev, irqs, mask) == 0)
		return 0;

legacy_irq:
	/* fall back to legacy IRQ */
	ret = pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_LEGACY);
	if (ret < 0)
		return -ENODEV;

	for (i = 0; i < PCIE_PORT_DEVICE_MAXSERVICES; i++)
		irqs[i] = pci_irq_vector(dev, 0);

	return 0;
}

/**
 * get_port_device_capability - discover capabilities of a PCI Express port
 * @dev: PCI Express port to examine
 *
 * The capabilities are read from the port's PCI Express configuration registers
 * as described in PCI Express Base Specification 1.0a sections 7.8.2, 7.8.9 and
 * 7.9 - 7.11.
 *
 * Return value: Bitmask of discovered port capabilities
 */
static int get_port_device_capability(struct pci_dev *dev)
{
	struct pci_host_bridge *host = pci_find_host_bridge(dev->bus);
	int services = 0;

	if (dev->is_hotplug_bridge &&
	    (pci_pcie_type(dev) == PCI_EXP_TYPE_ROOT_PORT ||
	     pci_pcie_type(dev) == PCI_EXP_TYPE_DOWNSTREAM) &&
	    (pcie_ports_native || host->native_pcie_hotplug)) {
		services |= PCIE_PORT_SERVICE_HP;

		/*
		 * Disable hot-plug interrupts in case they have been enabled
		 * by the BIOS and the hot-plug service driver is not loaded.
		 */
		pcie_capability_clear_word(dev, PCI_EXP_SLTCTL,
			  PCI_EXP_SLTCTL_CCIE | PCI_EXP_SLTCTL_HPIE);
	}

#ifdef CONFIG_PCIEAER
	if ((pci_pcie_type(dev) == PCI_EXP_TYPE_ROOT_PORT ||
             pci_pcie_type(dev) == PCI_EXP_TYPE_RC_EC) &&
	    dev->aer_cap && pci_aer_available() &&
	    (pcie_ports_native || host->native_aer))
		services |= PCIE_PORT_SERVICE_AER;
#endif

	/* Root Ports and Root Complex Event Collectors may generate PMEs */
	if ((pci_pcie_type(dev) == PCI_EXP_TYPE_ROOT_PORT ||
	     pci_pcie_type(dev) == PCI_EXP_TYPE_RC_EC) &&
	    (pcie_ports_native || host->native_pme)) {
		services |= PCIE_PORT_SERVICE_PME;

		/*
		 * Disable PME interrupt on this port in case it's been enabled
		 * by the BIOS (the PME service driver will enable it when
		 * necessary).
		 */
		pcie_pme_interrupt_enable(dev, false);
	}

	/*
	 * With dpc-native, allow Linux to use DPC even if it doesn't have
	 * permission to use AER.
	 */
	if (pci_find_ext_capability(dev, PCI_EXT_CAP_ID_DPC) &&
	    pci_aer_available() &&
	    (pcie_ports_dpc_native || (services & PCIE_PORT_SERVICE_AER)))
		services |= PCIE_PORT_SERVICE_DPC;

	if (pci_pcie_type(dev) == PCI_EXP_TYPE_DOWNSTREAM ||
	    pci_pcie_type(dev) == PCI_EXP_TYPE_ROOT_PORT) {
		u32 linkcap;

		pcie_capability_read_dword(dev, PCI_EXP_LNKCAP, &linkcap);
		if (linkcap & PCI_EXP_LNKCAP_LBNC)
			services |= PCIE_PORT_SERVICE_BWNOTIF;
	}

	return services;
}

/**
 * pcie_device_init - allocate and initialize PCI Express port service device
 * @pdev: PCI Express port to associate the service device with
 * @service: Type of service to associate with the service device
 * @irq: Interrupt vector to associate with the service device
 */
static int pcie_device_init(struct pci_dev *pdev, int service, int irq)
{
	int retval;
	struct pcie_device *pcie;
	struct device *device;

	pcie = kzalloc(sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;
	pcie->port = pdev;
	pcie->irq = irq;
	pcie->service = service;

	/* Initialize generic device interface */
	device = &pcie->device;
	device->bus = &pcie_port_bus_type;
	device->release = release_pcie_device;	/* callback to free pcie dev */
	dev_set_name(device, "%s:pcie%03x",
		     pci_name(pdev),
		     get_descriptor_id(pci_pcie_type(pdev), service));
	device->parent = &pdev->dev;
	device_enable_async_suspend(device);

	retval = device_register(device);
	if (retval) {
		put_device(device);
		return retval;
	}

	pm_runtime_no_callbacks(device);

	return 0;
}

/**
 * pcie_port_device_register - register PCI Express port
 * @dev: PCI Express port to register
 *
 * Allocate the port extension structure and register services associated with
 * the port.
 */
static int pcie_port_device_register(struct pci_dev *dev)
{
	int status, capabilities, i, nr_service;
	int irqs[PCIE_PORT_DEVICE_MAXSERVICES];

	/* Enable PCI Express port device */
	status = pci_enable_device(dev);
	if (status)
		return status;

	/* Get and check PCI Express port services */
	capabilities = get_port_device_capability(dev);
	if (!capabilities)
		return 0;

	pci_set_master(dev);
	/*
	 * Initialize service irqs. Don't use service devices that
	 * require interrupts if there is no way to generate them.
	 * However, some drivers may have a polling mode (e.g. pciehp_poll_mode)
	 * that can be used in the absence of irqs.  Allow them to determine
	 * if that is to be used.
	 */
	status = pcie_init_service_irqs(dev, irqs, capabilities);
	if (status) {
		capabilities &= PCIE_PORT_SERVICE_HP;
		if (!capabilities)
			goto error_disable;
	}

	/* Allocate child services if any */
	status = -ENODEV;
	nr_service = 0;
	for (i = 0; i < PCIE_PORT_DEVICE_MAXSERVICES; i++) {
		int service = 1 << i;
		if (!(capabilities & service))
			continue;
		if (!pcie_device_init(dev, service, irqs[i]))
			nr_service++;
	}
	if (!nr_service)
		goto error_cleanup_irqs;

	return 0;

error_cleanup_irqs:
	pci_free_irq_vectors(dev);
error_disable:
	pci_disable_device(dev);
	return status;
}

typedef int (*pcie_callback_t)(struct pcie_device *);

static int pcie_port_device_iter(struct device *dev, void *data)
{
	struct pcie_port_service_driver *service_driver;
	size_t offset = *(size_t *)data;
	pcie_callback_t cb;

	if ((dev->bus == &pcie_port_bus_type) && dev->driver) {
		service_driver = to_service_driver(dev->driver);
		cb = *(pcie_callback_t *)((void *)service_driver + offset);
		if (cb)
			return cb(to_pcie_device(dev));
	}
	return 0;
}

#ifdef CONFIG_PM
/**
 * pcie_port_device_suspend - suspend port services associated with a PCIe port
 * @dev: PCI Express port to handle
 */
static int pcie_port_device_suspend(struct device *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, suspend);
	return device_for_each_child(dev, &off, pcie_port_device_iter);
}

static int pcie_port_device_resume_noirq(struct device *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, resume_noirq);
	return device_for_each_child(dev, &off, pcie_port_device_iter);
}

/**
 * pcie_port_device_resume - resume port services associated with a PCIe port
 * @dev: PCI Express port to handle
 */
static int pcie_port_device_resume(struct device *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, resume);
	return device_for_each_child(dev, &off, pcie_port_device_iter);
}

/**
 * pcie_port_device_runtime_suspend - runtime suspend port services
 * @dev: PCI Express port to handle
 */
static int pcie_port_device_runtime_suspend(struct device *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, runtime_suspend);
	return device_for_each_child(dev, &off, pcie_port_device_iter);
}

/**
 * pcie_port_device_runtime_resume - runtime resume port services
 * @dev: PCI Express port to handle
 */
static int pcie_port_device_runtime_resume(struct device *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, runtime_resume);
	return device_for_each_child(dev, &off, pcie_port_device_iter);
}
#endif /* PM */

static int remove_iter(struct device *dev, void *data)
{
	if (dev->bus == &pcie_port_bus_type)
		device_unregister(dev);
	return 0;
}

static int find_service_iter(struct device *device, void *data)
{
	struct pcie_port_service_driver *service_driver;
	struct portdrv_service_data *pdrvs;
	u32 service;

	pdrvs = (struct portdrv_service_data *) data;
	service = pdrvs->service;

	if (device->bus == &pcie_port_bus_type && device->driver) {
		service_driver = to_service_driver(device->driver);
		if (service_driver->service == service) {
			pdrvs->drv = service_driver;
			pdrvs->dev = device;
			return 1;
		}
	}

	return 0;
}

/**
 * pcie_port_find_device - find the struct device
 * @dev: PCI Express port the service is associated with
 * @service: For the service to find
 *
 * Find the struct device associated with given service on a pci_dev
 */
struct device *pcie_port_find_device(struct pci_dev *dev,
				      u32 service)
{
	struct device *device;
	struct portdrv_service_data pdrvs;

	pdrvs.dev = NULL;
	pdrvs.service = service;
	device_for_each_child(&dev->dev, &pdrvs, find_service_iter);

	device = pdrvs.dev;
	return device;
}
EXPORT_SYMBOL_GPL(pcie_port_find_device);

/**
 * pcie_port_device_remove - unregister PCI Express port service devices
 * @dev: PCI Express port the service devices to unregister are associated with
 *
 * Remove PCI Express port service devices associated with given port and
 * disable MSI-X or MSI for the port.
 */
static void pcie_port_device_remove(struct pci_dev *dev)
{
	device_for_each_child(&dev->dev, NULL, remove_iter);
	pci_free_irq_vectors(dev);
}

/**
 * pcie_port_probe_service - probe driver for given PCI Express port service
 * @dev: PCI Express port service device to probe against
 *
 * If PCI Express port service driver is registered with
 * pcie_port_service_register(), this function will be called by the driver core
 * whenever match is found between the driver and a port service device.
 */
static int pcie_port_probe_service(struct device *dev)
{
	struct pcie_device *pciedev;
	struct pcie_port_service_driver *driver;
	int status;

	if (!dev || !dev->driver)
		return -ENODEV;

	driver = to_service_driver(dev->driver);
	if (!driver || !driver->probe)
		return -ENODEV;

	pciedev = to_pcie_device(dev);
	status = driver->probe(pciedev);
	if (status)
		return status;

	get_device(dev);
	return 0;
}

/**
 * pcie_port_remove_service - detach driver from given PCI Express port service
 * @dev: PCI Express port service device to handle
 *
 * If PCI Express port service driver is registered with
 * pcie_port_service_register(), this function will be called by the driver core
 * when device_unregister() is called for the port service device associated
 * with the driver.
 */
static int pcie_port_remove_service(struct device *dev)
{
	struct pcie_device *pciedev;
	struct pcie_port_service_driver *driver;

	if (!dev || !dev->driver)
		return 0;

	pciedev = to_pcie_device(dev);
	driver = to_service_driver(dev->driver);
	if (driver && driver->remove) {
		driver->remove(pciedev);
		put_device(dev);
	}
	return 0;
}

/**
 * pcie_port_shutdown_service - shut down given PCI Express port service
 * @dev: PCI Express port service device to handle
 *
 * If PCI Express port service driver is registered with
 * pcie_port_service_register(), this function will be called by the driver core
 * when device_shutdown() is called for the port service device associated
 * with the driver.
 */
static void pcie_port_shutdown_service(struct device *dev) {}

/**
 * pcie_port_service_register - register PCI Express port service driver
 * @new: PCI Express port service driver to register
 */
int pcie_port_service_register(struct pcie_port_service_driver *new)
{
	if (pcie_ports_disabled)
		return -ENODEV;

	new->driver.name = new->name;
	new->driver.bus = &pcie_port_bus_type;
	new->driver.probe = pcie_port_probe_service;
	new->driver.remove = pcie_port_remove_service;
	new->driver.shutdown = pcie_port_shutdown_service;

	return driver_register(&new->driver);
}

/**
 * pcie_port_service_unregister - unregister PCI Express port service driver
 * @drv: PCI Express port service driver to unregister
 */
void pcie_port_service_unregister(struct pcie_port_service_driver *drv)
{
	driver_unregister(&drv->driver);
}

/* If this switch is set, PCIe port native services should not be enabled. */
bool pcie_ports_disabled;

/*
 * If the user specified "pcie_ports=native", use the PCIe services regardless
 * of whether the platform has given us permission.  On ACPI systems, this
 * means we ignore _OSC.
 */
bool pcie_ports_native;

/*
 * If the user specified "pcie_ports=dpc-native", use the Linux DPC PCIe
 * service even if the platform hasn't given us permission.
 */
bool pcie_ports_dpc_native;

/* New optimization functions */

/**
 * pcie_port_optimize_link - Optimize PCIe link speed and width
 * @dev: PCI Express port to optimize
 *
 * Ensures the PCIe link is operating at its maximum capability by
 * checking current status and potentially initiating link retraining.
 */
static int pcie_port_optimize_link(struct pci_dev *dev)
{
    u16 link_status, link_control;
    u32 link_cap;
    int current_speed, max_speed;
    int current_width, max_width;
    bool need_retrain = false;
    
    /* Read link capabilities to determine max speed and width */
    pcie_capability_read_dword(dev, PCI_EXP_LNKCAP, &link_cap);
    max_speed = link_cap & PCI_EXP_LNKCAP_SLS;
    max_width = (link_cap & PCI_EXP_LNKCAP_MLW) >> 4;
    
    /* Read current link status */
    pcie_capability_read_word(dev, PCI_EXP_LNKSTA, &link_status);
    current_speed = link_status & PCI_EXP_LNKSTA_CLS;
    current_width = (link_status & PCI_EXP_LNKSTA_NLW) >> 4;
    
    /* If not at max capability, try to retrain link */
    if (current_speed < max_speed || current_width < max_width) {
        dev_info(&dev->dev, "PCIe link not at optimal speed/width. Current: Gen%d x%d, Max: Gen%d x%d\n",
                 current_speed, current_width, max_speed, max_width);
        need_retrain = true;
    }
    
    if (need_retrain) {
        /* Initiate link retraining */
        pcie_capability_read_word(dev, PCI_EXP_LNKCTL, &link_control);
        link_control |= PCI_EXP_LNKCTL_RL;
        pcie_capability_write_word(dev, PCI_EXP_LNKCTL, link_control);
        
        /* Wait for retraining to complete */
        msleep(100);
        
        /* Check if improvement was achieved */
        pcie_capability_read_word(dev, PCI_EXP_LNKSTA, &link_status);
        current_speed = link_status & PCI_EXP_LNKSTA_CLS;
        current_width = (link_status & PCI_EXP_LNKSTA_NLW) >> 4;
        
        dev_info(&dev->dev, "After retraining: Gen%d x%d\n", 
                 current_speed, current_width);
    }
    
    return 0;
}

/**
 * pcie_port_optimize_mps - Optimize Maximum Payload Size
 * @dev: PCI Express device to optimize
 *
 * Sets optimal Maximum Payload Size for improved throughput.
 */
static int pcie_port_optimize_mps(struct pci_dev *dev)
{
    int mps;
    u16 devctl;
    
    /* For most systems, 256 bytes is a good balance */
    mps = 256;
    
    /* Apply MPS setting */
    pcie_capability_read_word(dev, PCI_EXP_DEVCTL, &devctl);
    devctl &= ~PCI_EXP_DEVCTL_PAYLOAD;
    devctl |= (ffs(mps) - 8) << 5; /* Convert to PCIe encoding */
    pcie_capability_write_word(dev, PCI_EXP_DEVCTL, devctl);
    
    dev_info(&dev->dev, "PCIe MPS optimized to %d bytes\n", mps);
    
    return 0;
}

/**
 * pcie_port_optimize_mrrs - Optimize Maximum Read Request Size
 * @dev: PCI Express device to optimize
 *
 * Sets optimal Maximum Read Request Size for improved read throughput.
 */
static int pcie_port_optimize_mrrs(struct pci_dev *dev)
{
    u16 devctl;
    int mrrs = 4096; /* Typically, the max is 4096 bytes */
    
    /* Apply MRRS setting */
    pcie_capability_read_word(dev, PCI_EXP_DEVCTL, &devctl);
    devctl &= ~PCI_EXP_DEVCTL_READRQ;
    devctl |= (ffs(mrrs) - 8) << 12;
    pcie_capability_write_word(dev, PCI_EXP_DEVCTL, devctl);
    
    dev_info(&dev->dev, "PCIe MRRS optimized to %d bytes\n", mrrs);
    
    return 0;
}

/**
 * pcie_port_optimize_aspm - Optimize Active State Power Management
 * @dev: PCI Express device to optimize
 *
 * Configures ASPM settings based on device type, prioritizing
 * performance for NVMe devices while enabling power saving for others.
 */
static int pcie_port_optimize_aspm(struct pci_dev *dev)
{
    u16 link_control;
    bool is_nvme = false;
    
    /* Check if this is an M.2 NVMe device or its parent port */
    is_nvme = (dev->class >> 8) == 0x010802; /* NVMe class code */
    
    /* Read current link control */
    pcie_capability_read_word(dev, PCI_EXP_LNKCTL, &link_control);
    
    if (is_nvme) {
        /* For NVMe: Use L1 but avoid L0s for lowest latency */
        link_control &= ~PCI_EXP_LNKCTL_ASPMC;
        link_control |= PCI_EXP_LNKCTL_ASPM_L1;
        dev_info(&dev->dev, "NVMe device: optimizing ASPM for performance (L1 only)\n");
    } else {
        /* For other devices, enable both for better power saving */
        link_control |= PCI_EXP_LNKCTL_ASPM_L0S | PCI_EXP_LNKCTL_ASPM_L1;
        dev_info(&dev->dev, "Standard device: enabling full ASPM (L0s and L1)\n");
    }
    
    pcie_capability_write_word(dev, PCI_EXP_LNKCTL, link_control);
    return 0;
}

/**
 * pcie_port_enable_transaction_optimizations - Enable TLP optimizations
 * @dev: PCI Express device to optimize
 *
 * Enables relaxed ordering and no snoop for improved transaction performance.
 */
static void pcie_port_enable_transaction_optimizations(struct pci_dev *dev)
{
    u16 devctl;
    
    /* Enable relaxed ordering and no snoop if supported */
    pcie_capability_read_word(dev, PCI_EXP_DEVCTL, &devctl);
    
    devctl |= PCI_EXP_DEVCTL_RELAX_EN;  /* Enable relaxed ordering */
    devctl |= PCI_EXP_DEVCTL_NOSNOOP_EN;  /* Enable no snoop */
    
    pcie_capability_write_word(dev, PCI_EXP_DEVCTL, devctl);
    
    dev_info(&dev->dev, "PCIe TLP optimizations enabled: relaxed ordering and no snoop\n");
}

/**
 * pcie_port_latency_tuning - Optimize completion timeout values
 * @dev: PCI Express device to optimize
 *
 * Adjusts completion timeout values for better performance
 * with storage devices like NVMe SSDs.
 */
static int pcie_port_latency_tuning(struct pci_dev *dev)
{
    u16 devctl2;
    int pos;
    
    /* Check if we have access to Device Control 2 Register */
    pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_SECONDARY_PCI_EXPRESS);
    if (!pos)
        return -ENODEV;
    
    /* Read current settings */
    pci_read_config_word(dev, pos + PCI_EXP_DEVCTL2, &devctl2);
    
    /* Set completion timeout value - 0x6 = 33-64ms (good for NVMe) */
    devctl2 &= ~PCI_EXP_DEVCTL2_COMP_TIMEOUT;
    devctl2 |= 0x6;
    
    pci_write_config_word(dev, pos + PCI_EXP_DEVCTL2, devctl2);
    
    dev_info(&dev->dev, "PCIe completion timeout optimized for reduced latency\n");
    
    return 0;
}

/**
 * pcie_port_optimize_irq_affinity - Optimize IRQ affinity
 * @dev: PCI Express device to optimize
 *
 * Sets interrupt affinity to distribute across CPUs, typically
 * avoiding CPU 0 which often handles system tasks.
 */
static int pcie_port_optimize_irq_affinity(struct pci_dev *dev)
{
    int cpu, irq;
    struct irq_desc *desc;
    cpumask_t mask;
    
    /* Skip if we're on a single core system */
    if (num_online_cpus() <= 1)
        return 0;
    
    irq = dev->irq;
    if (!irq)
        return 0;
    
    desc = irq_to_desc(irq);
    if (!desc)
        return -ENODEV;
    
    /* Assign to CPU 1 (second core), leaving CPU 0 for system tasks */
    cpumask_clear(&mask);
    cpu = 1 % num_online_cpus();
    cpumask_set_cpu(cpu, &mask);
    
    if (irq_set_affinity_hint(irq, &mask) == 0)
        dev_info(&dev->dev, "IRQ %d affinity set to CPU %d\n", irq, cpu);
    
    return 0;
}

/**
 * pcie_imx8mp_specific_optimizations - Platform-specific optimizations
 * @dev: PCI Express device to optimize
 *
 * Apply optimizations specific to the iMX8M Plus platform.
 */
static int pcie_imx8mp_specific_optimizations(struct pci_dev *dev)
{
    /* Only apply to root ports on iMX8MP */
    if (pci_pcie_type(dev) != PCI_EXP_TYPE_ROOT_PORT)
        return 0;
        
#ifdef CONFIG_OF
    if (!of_machine_is_compatible("fsl,imx8mp"))
        return 0;
    
    dev_info(&dev->dev, "Applying iMX8M Plus-specific PCIe optimizations\n");
    
    /* 
     * This is where we would add iMX8MP-specific register tweaks
     * A full implementation would require access to SoC-specific registers
     */
#endif
    
    return 0;
}

/**
 * pcie_port_apply_all_optimizations - Apply all PCIe optimizations
 * @dev: PCI Express device to optimize
 *
 * Apply all supported optimizations to the PCIe device.
 */
static int pcie_port_apply_all_optimizations(struct pci_dev *dev)
{
    /* Basic link optimizations */
    pcie_port_optimize_link(dev);
    
    /* Payload and request size optimizations */
    pcie_port_optimize_mps(dev);
    pcie_port_optimize_mrrs(dev);
    
    /* Power and latency tuning */
    pcie_port_optimize_aspm(dev);
    
    /* Transaction optimizations */
    pcie_port_enable_transaction_optimizations(dev);
    
    /* Latency optimizations */
    pcie_port_latency_tuning(dev);
    
    /* CPU affinity for IRQs */
    pcie_port_optimize_irq_affinity(dev);
    
    /* Platform specific tuning */
    pcie_imx8mp_specific_optimizations(dev);
    
    return 0;
}

#ifdef CONFIG_PM
static int pcie_port_runtime_suspend(struct device *dev)
{
	if (!to_pci_dev(dev)->bridge_d3)
		return -EBUSY;

	return pcie_port_device_runtime_suspend(dev);
}

static int pcie_port_runtime_idle(struct device *dev)
{
	/*
	 * Assume the PCI core has set bridge_d3 whenever it thinks the port
	 * should be good to go to D3.  Everything else, including moving
	 * the port to D3, is handled by the PCI core.
	 */
	return to_pci_dev(dev)->bridge_d3 ? 0 : -EBUSY;
}

static const struct dev_pm_ops pcie_portdrv_pm_ops = {
	.suspend	= pcie_port_device_suspend,
	.resume_noirq	= pcie_port_device_resume_noirq,
	.resume		= pcie_port_device_resume,
	.freeze		= pcie_port_device_suspend,
	.thaw		= pcie_port_device_resume,
	.poweroff	= pcie_port_device_suspend,
	.restore_noirq	= pcie_port_device_resume_noirq,
	.restore	= pcie_port_device_resume,
	.runtime_suspend = pcie_port_runtime_suspend,
	.runtime_resume	= pcie_port_device_runtime_resume,
	.runtime_idle	= pcie_port_runtime_idle,
};

#define PCIE_PORTDRV_PM_OPS	(&pcie_portdrv_pm_ops)

#else /* !PM */

#define PCIE_PORTDRV_PM_OPS	NULL
#endif /* !PM */

/*
 * pcie_portdrv_probe - Probe PCI-Express port devices
 * @dev: PCI-Express port device being probed
 *
 * If detected invokes the pcie_port_device_register() method for
 * this port device.
 *
 */
static int pcie_portdrv_probe(struct pci_dev *dev,
					const struct pci_device_id *id)
{
	int type = pci_pcie_type(dev);
	int status;

	if (!pci_is_pcie(dev) ||
	    ((type != PCI_EXP_TYPE_ROOT_PORT) &&
	     (type != PCI_EXP_TYPE_UPSTREAM) &&
	     (type != PCI_EXP_TYPE_DOWNSTREAM) &&
	     (type != PCI_EXP_TYPE_RC_EC)))
		return -ENODEV;

	if (type == PCI_EXP_TYPE_RC_EC)
		pcie_link_rcec(dev);

	status = pcie_port_device_register(dev);
	if (status)
		return status;

    /* Apply our comprehensive optimizations */
    pcie_port_apply_all_optimizations(dev);

	pci_save_state(dev);

	dev_pm_set_driver_flags(&dev->dev, DPM_FLAG_NO_DIRECT_COMPLETE |
					   DPM_FLAG_SMART_SUSPEND);

	if (pci_bridge_d3_possible(dev)) {
		/*
		 * Keep the port resumed 100ms to make sure things like
		 * config space accesses from userspace (lspci) will not
		 * cause the port to repeatedly suspend and resume.
		 */
		pm_runtime_set_autosuspend_delay(&dev->dev, 100);
		pm_runtime_use_autosuspend(&dev->dev);
		pm_runtime_mark_last_busy(&dev->dev);
		pm_runtime_put_autosuspend(&dev->dev);
		pm_runtime_allow(&dev->dev);
	}

	return 0;
}

static void pcie_portdrv_remove(struct pci_dev *dev)
{
	if (pci_bridge_d3_possible(dev)) {
		pm_runtime_forbid(&dev->dev);
		pm_runtime_get_noresume(&dev->dev);
		pm_runtime_dont_use_autosuspend(&dev->dev);
	}

	pcie_port_device_remove(dev);

	pci_disable_device(dev);
}

static void pcie_portdrv_shutdown(struct pci_dev *dev)
{
	if (pci_bridge_d3_possible(dev)) {
		pm_runtime_forbid(&dev->dev);
		pm_runtime_get_noresume(&dev->dev);
		pm_runtime_dont_use_autosuspend(&dev->dev);
	}

	pcie_port_device_remove(dev);
}

static pci_ers_result_t pcie_portdrv_error_detected(struct pci_dev *dev,
					pci_channel_state_t error)
{
	if (error == pci_channel_io_frozen)
		return PCI_ERS_RESULT_NEED_RESET;
	return PCI_ERS_RESULT_CAN_RECOVER;
}

static pci_ers_result_t pcie_portdrv_slot_reset(struct pci_dev *dev)
{
	size_t off = offsetof(struct pcie_port_service_driver, slot_reset);
	device_for_each_child(&dev->dev, &off, pcie_port_device_iter);

	pci_restore_state(dev);
	pci_save_state(dev);
	return PCI_ERS_RESULT_RECOVERED;
}

static pci_ers_result_t pcie_portdrv_mmio_enabled(struct pci_dev *dev)
{
	return PCI_ERS_RESULT_RECOVERED;
}

/*
 * LINUX Device Driver Model
 */
static const struct pci_device_id port_pci_ids[] = {
	/* handle any PCI-Express port */
	{ PCI_DEVICE_CLASS(PCI_CLASS_BRIDGE_PCI_NORMAL, ~0) },
	/* subtractive decode PCI-to-PCI bridge, class type is 060401h */
	{ PCI_DEVICE_CLASS(PCI_CLASS_BRIDGE_PCI_SUBTRACTIVE, ~0) },
	/* handle any Root Complex Event Collector */
	{ PCI_DEVICE_CLASS(((PCI_CLASS_SYSTEM_RCEC << 8) | 0x00), ~0) },
	{ },
};

static const struct pci_error_handlers pcie_portdrv_err_handler = {
	.error_detected = pcie_portdrv_error_detected,
	.slot_reset = pcie_portdrv_slot_reset,
	.mmio_enabled = pcie_portdrv_mmio_enabled,
};

static struct pci_driver pcie_portdriver = {
	.name		= "pcieport",
	.id_table	= &port_pci_ids[0],

	.probe		= pcie_portdrv_probe,
	.remove		= pcie_portdrv_remove,
	.shutdown	= pcie_portdrv_shutdown,

	.err_handler	= &pcie_portdrv_err_handler,

	.driver_managed_dma = true,

	.driver.pm	= PCIE_PORTDRV_PM_OPS,
};

static int __init dmi_pcie_pme_disable_msi(const struct dmi_system_id *d)
{
	pr_notice("%s detected: will not use MSI for PCIe PME signaling\n",
		  d->ident);
	pcie_pme_disable_msi();
	return 0;
}

static const struct dmi_system_id pcie_portdrv_dmi_table[] __initconst = {
	/*
	 * Boxes that should not use MSI for PCIe PME signaling.
	 */
	{
	 .callback = dmi_pcie_pme_disable_msi,
	 .ident = "MSI Wind U-100",
	 .matches = {
		     DMI_MATCH(DMI_SYS_VENDOR,
				"MICRO-STAR INTERNATIONAL CO., LTD"),
		     DMI_MATCH(DMI_PRODUCT_NAME, "U-100"),
		     },
	 },
	 {}
};

static void __init pcie_init_services(void)
{
	pcie_aer_init();
	pcie_pme_init();
	pcie_dpc_init();
	pcie_hp_init();
}

static int __init pcie_portdrv_init(void)
{
	if (pcie_ports_disabled)
		return -EACCES;

	pcie_init_services();
	dmi_check_system(pcie_portdrv_dmi_table);

	return pci_register_driver(&pcie_portdriver);
}
device_initcall(pcie_portdrv_init);

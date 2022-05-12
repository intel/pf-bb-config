# Introduction

SPDX-License-Identifier: Apache-2.0
Copyright © 2021 Intel Corporation

The Physical Function (PF) Baseband Device (BBDEV) Configuration Application
(pf_bb_config) provides a means to
configure a baseband device at the host level. The program accesses the
configuration space and sets various parameters through memory-mapped I/O
(MMIO) reads and writes.

The parameters are parsed from a given configuration file (with .cfg extensions)
that is specific to a particular baseband device, although they follow same
format.

## Building the PF BBDEV Config Application

Build the program by typing the `make` command.
The result is that the `pf_bb_config` binary file is produced.

## Usage

The application requires the name of the BBDEV device to configure. Other
arguments are optional.
The application executes as follows:

    ./pf_bb_config DEVICE_NAME [-h] [-a] [-c CFG_FILE] [-p PCI_ID] [-v VFIO_TOKEN]

* `DEVICE_NAME`: Specifies the device to configure; for example: 'FPGA_5GNR' or 'ACC100'
* `-a`: Configures all PCI devices matching the given `DEVICE_NAME`
* `-h`: Prints help
* `-c CFG_FILE`: Specifies configuration file to use
* `-p PCI_ID`: Specifies PCI ID of device to configure
* `-v VFIO_TOKEN` : `VFIO_TOKEN is UUID formatted VFIO VF token required when bound with vfio-pci

## Example

This section describes an example of configuring the ACC100 device so that the
workload can be run from the Virtual Function (VF).
Note the usage of DPDK and bbdev-test
(https://doc.dpdk.org/guides/tools/testbbdev.html) for this example.
The example in this section uses the igb_uio module and assumes that
Intel® Virtualization Technology for Directed I/O (Intel® VT-d) is disabled.

Bind the PF with the igb_uio module (or alternatively with pci-pf-stub):

    $dpdkPath/usertools/dpdk-devbind.py --bind=igb_uio $pfPciDeviceAddr

Create 2 VFs from the PF using the exposed sysfs interface:

    echo 2 | sudo tee /sys/bus/pci/devices/0000:$pfPciDeviceAddr/max_vfs

Bind both VFs using either the igb_uio module (make sure that Intel® VT-d is
disabled) or using the vfio-pci module (make sure that Intel® VT-d is enabled):

    $dpdkPath/usertools/dpdk-devbind.py --bind=igb_uio $vfPciDeviceAddr1 $vfPciDeviceAddr2

Configure the devices using the pf_bb_config application for VF usage with both
5G and 4G enabled:

    ./pf_bb_config ACC100 -c acc100/acc100_config_2vf_4g5g.cfg

Test that the VF is functional on the device using bbdev-test:

    ../../${RTE_TARGET}/app/testbbdev -c F0 -w${VF_PF_PCI_ADDR} -- -c validation -v ./ldpc_dec_default.data

From DPDK 20.11, the previous command must be updated as follows:

    ../../build/app/dpdk-test-bbdev -c F0 -a${VF_PF_PCI_ADDR} -- -c validation -v ./ldpc_dec_default.data

Additional documentation can be found on DPDK for BBDEV usage
(https://doc.dpdk.org/guides/prog_guide/bbdev.html)
and within Intel FlexRAN documentation on the Intel Resource & Design Center
(https://www.intel.com/content/www/us/en/design/resource-design-center.html).

### Using vfio-pci driver

Load the vfio-pci driver with sriov capability enabled:

    modprobe vfio-pci enable_sriov=1 disable_idle_d3=1

Bind the PF with the vfio-pci module:

    $dpdkPath/usertools/dpdk-devbind.py --bind=vfio-pci $pfPciDeviceAddr

Configure the device using the pf_bb_config application for VF usage with both
5G and 4G enabled:

    ./pf_bb_config ACC100 -v 00112233-4455-6677-8899-aabbccddeeff -c acc100/acc100_config_2vf_4g5g.cfg

Create 2 VFs from the PF using the exposed sysfs interface:

    echo 2 | sudo tee /sys/bus/pci/devices/0000:$pfPciDeviceAddr/sriov_numvfs

Bind both VFs using the vfio-pci module (make sure that Intel® VT-d is enabled):

    $dpdkPath/usertools/dpdk-devbind.py --bind=vfio-pci $vfPciDeviceAddr1 $vfPciDeviceAddr2

Test that the VF is functional on the device using bbdev-test:

    ../../build/app/dpdk-test-bbdev -c F0 -a${VF_PF_PCI_ADDR} --vfio-vf-token=00112233-4455-6677-8899-aabbccddeeff -- -c validation -v ./ldpc_dec_default.data

NOTE:

    UUID(Universally Unique Identifier) used in the above example
    (00112233-4455-6677-8899-aabbccddeeff) is for testing purpose only.
    User should use a random generated UUID in real application, for example
    using the tools like `uuidgen`.  The UUID format should match
    the standard defined in RFC4122.

    For binding PF interface with vfio-pci kernel driver:
    VFIO VF token support is upstreamed in Linux kernel version 5.7.
    Recommended to use kernel version 5.7 or above or make sure all vfio vf token
    feature are back ported to your kernel version.

    In case of VFIO mode, pf_bb_config runs daemon mode. To reconfigure first
    kill the existing pf_bb_config process using:
    pkill pf_bb_config

## Notes on the IOMMU usage

Alternatively, the previous example can be run using the vfio-pci module to
bind the VF with Intel® VT-d enabled in the kernel (`intel_iommu=on iommu=pt`)
and in BIOS (as Intel® VT-d).
Intel® VT-d is implemented in the Input-Output Memory Management Unit (IOMMU).
To check that IOMMU is actually enabled at run time use the following command:

     dmesg | grep "DMAR: IOMMU"

Additional information on vfio can be found on the related kernel documentation
page (https://www.kernel.org/doc/Documentation/vfio.txt)

## Details for ACC100 Configuration

* A number of configuration file examples are available in the acc100 directory.
These examples include the following parameters, which can be modified for a
specific use case.

* The `pf_mode_en` parameter refers to the case for which the workload is run
from PF, or alternatively from the VF (Single Root I/O Virtualization (SR-IOV)).

* There are eight Queue Groups (QGroups) available (made of up to 16 queues
x 16 VFs each), which can be allocated to any available operation
(4GUL/4GDL/5GUL/5GDL) based on the `num_qgroups` parameter.
For example, 4x QGroups for 5GUL and 4x QGroups for 5GDL can be allocated when
only 5G processing is required, and these QGroups cover four distinct
incremental priority levels.
The only limitation (which would be flagged by the configuration tool if
exceeded) is for the total number of QGroups x num_aqs x aq_depth x vf_bundles
to be less than 32K. (The num_aqs parameter is the number of atomic queues,
the aq_depth parameter is the atomic queue depth, and the vf_bundles parameter
is the number of VF bundles.).

* The default DDR size allocated per VF depends on the `num_vf_bundles` set
(1 to 16). Assuming 4GB of DDR is populated on the board, which is then
split between VFs.

* The `num_aqs_per_groups` defines the number of atomic queues within a VF
bundle for a given QGroup.
The range is 1 to 16, with 16 being the default value.
There can be a maximum of 16 * 8 * 16 = 2048 total queues.

* Note that if an early version of the ACC100 device is used
(Engineering Sample prior to PRQ), then pf_bb_config displays:
`Note: Not on DDR PRQ version  1302020 != 10092020`.
Intel recommends using the PRQ version of the ACC100 device.

## Details for Intel® FPGA Programmable Acceleration Card N3000 Configuration

* A number of configuration file examples are available in the `fpga_5gnr` and
`fpga_lte` directories. These examples include the following parameters, which
can be modified for a specific usecase.

* The `pf_mode_en` parameter refers to the case for which the workload is run
from PF, or alternatively from the VF
(Single Root I/O Virtualization (SR-IOV)).

* The `vfqmap` list parameter defines how many queues are mapped for each VF,
the absolute total being 32 for either UL or DL.
As an example: "16, 16, 0, 0...",
which means that two VFs would be supported with 16 queues each.
All queues have the same priority level. This parameter is set independently
for the actual number of VFs exposed through the SR-IOV PCIe configuration
capability through the sysfs interface.

* The parameters `bandwidth` and `load_balance` are optional parameters
impacting arbitration but are expected to be kept as default.

## Device Reset Procedure Using PF FLR and VF FLR

The accelerators are exposed as a PCIe endpoint and therefore can be reset
using Function-Level Reset (FLR) as described in the PCI Express® Base
Specification Revision 3.1a.
The FLR mechanism enables software to quiesce and reset endpoint hardware
with Function-Level granularity, that is, for either the Physical Function
(PF) or one of the Virtual Functions (VFs):
- When the function targeted is the PF, then the full device is reset.
The device must then be reconfigured using pf_bb_config as is after a
cold start.
- When the function targeted is the VF, then only the internal state related to
that same VF is impacted (queues are flushed, VF registers reset back to
default values). This VF can then still be used without requiring pf_bb_config
to be run again (that is, running the bbdev-test command as described in the
Example section).

In an application, FLR can be triggered from the sysfs interface, as long as
these devices are bounded by using the following commands.
The actual bus:device.function (BDF) can be retrieved using the lspci command.
Trigger PF FLR with the following command:

    echo 1 >> /sys/bus/pci/devices/0000\:${PF_PCI_ADDR}/reset

Trigger VF FLR on one of the VFs (the one under $(VF_PCI_ADDR) BDF) with the
following command:

    echo 1 >> /sys/bus/pci/devices/0000\:${VF_PCI_ADDR}/reset

## Conversion to PDF

To convert this readme to a PDF file, first install the npm tool, followed by
installing the `markdown-pdf` conversion program:

    sudo yum install npm
    npm install -g markdown-pdf

Then convert to PDF by running the markdown-pdf conversion program:

    markdown-pdf README.md README.pdf

## License
SPDX-License-Identifier: Apache-2.0 (see included LICENSE file located at
https://github.com/intel/pf-bb-config/blob/master/LICENSE)

Copyright(c) 2021 Intel Corporation

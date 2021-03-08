# Introduction

SPDX-License-Identifier: Apache-2.0
Copyright © 2020 Intel Corporation

The PF BBDEV (baseband device) Configuration Application "pf_bb_config" provides a means to
configure the a baseband device at the host-level. The program accesses the
configuration space and sets the various parameters through memory-mapped IO
read/writes.

The parameters are parsed from a given configuration file (with .cfg extentions)
that is specific to particular baseband device, although they follow same format.

## External Dependencies

The third party .INI parser is a pre-requisite for building the application.
It can be downloaded from [github](https://github.com/benhoyt/inih), or through git as:

    git clone https://github.com/benhoyt/inih

Use the version release 44 tracked by tag 'r44':

    git checkout r44

The application features a makefile in the `extra/` directory which generates
the library, `libinih.a`.
To compile the inih library, run make as:

    make -f Makefile.static
    /bin/cp libinih.a ..

## Building  PF BBDEV Config App

Before building the application, set the following environment variables with
the location where the INI library is located:

    export INIH_PATH=<path-to-inih-lib>

If not set, makefile will look into current folder.

Next, build the program by typing `make`. The binary `pf_bb_config` should
have been produced.

## Usage

The application requires the name of the bbdev device to look for. Other
arguments are optional.
The application executes as the following:

    ./pf_bb_config DEVICE_NAME [-h] [-a] [-c CFG_FILE] [-p PCI_ID]

* `DEVICE_NAME`: Specifies device to configure, example: 'FPGA_5GNR' or 'ACC100'
* `-c CFG_FILE`: Specifies configuration file to use
* `-p PCI_ID`: Specifies PCI ID of device to configure
* `-a`: Configures all PCI devices matching the given `DEVICE_NAME`
* `-h`: Prints help

## Example

See below an example when configuring the ACC100 device so that the workload can be run from the VF.
Note the usage of DPDK and bbdev-test (https://doc.dpdk.org/guides/tools/testbbdev.html)
The example below is using igb_uio and assuming IOMMU disabled. Conversely this can be run using vfio-pci to bind the VF with IOMMU/VT-d enabled in kernel and BIOS.

Bind the PF with igb_uio (or alternatively with pci-pf-stub):

    $dpdkPath/usertools/dpdk-devbind.py –bind=igb_uio $pfPciDeviceAddr

Create 2 VFs from the PF using exposed sysfs:

    echo 2 | sudo tee /sys/bus/pci/devices/0000:$pfPciDeviceAddr/max_vfs

Bind both VFs using either igb_uio (make sure IOMMU/VT-d is disabled) or using vfio-pci (make sure IOMMU/VT-d is enabled):

    $dpdkPath/usertools/dpdk-devbind.py –bind=igb_uio $vfPciDeviceAddr1 $vfPciDeviceAddr2

Configure the Devices using pf_bb_config app for VF usage with both 5G and 4G enabled:

    ./pf_bb_config ACC100 -c acc100/acc100_config_vf.cfg

Test the VF is functional on the device using bbdev-test:

    ../../${RTE_TARGET}/app/testbbdev -c F0 -w${VF_PF_PCI_ADDR} -- -c validation -v ./ldpc_dec_default.data

Additional documentation can also be found on DPDK for bbdev usage and within Intel FlexRAN documentation.

## Details for ACC100 configuration

* A number of configuration files examples are available in the acc100 directory. These includes the parameters below which can be modified for a specific usecase.

* The `pf_mode_en` parameter refers to the case whether the workload will be run from PF, or conversely from the VF (SRIOV).

* There are 8 Qgroups available (made of up to 16 queues x 16 vfs each) which can be allocated to any available operation
(UL4G/DL4G/UL5G/DL5G) based on parameter `num_qgroups`. For instance 4x of 5GUL and 4x for 5GDL when only 5G processing is required.
In that case they cover 4 distinct incremental priority levels.
The only limitation (which would be flagged by the configuration tool if exceeded) is for the total num of qgroups x num_aqs x aq_depth x vf_bundles to be less than 32k.

* The default DDR size allocated per VF depends on `num_vf_bundles` set (1 to 16). Assuming 4GB populated on the board, which is then split between VFs.

* The `num_aqs_per_groups` defines the number of atomic queues within a VF bundle for a given Qgroup. The range is 1 to 16, 16 being default value. Said otherwise there can be up a maximum of up to 16 * 8 * 16 = 2048 total queues.

## Details for N3000 configuration

* A number of configuration files examples are available in the fpga_5gnr and fpga_lte directories. These includes the parameters below which can be modified for a specific usecase.

* The `pf_mode_en` parameter refers to the case whether the workload will be run from PF, or conversely from the VF (SRIOV).

* The list `vfqmap` defines how many queues are mapped for each VF. The absolute total being 32 for either UL or DL. As example 16, 16, 0, 0... means that 2 VFs would be supported with 16 queues each. All queues have the same priority level. This parameter is set independently for the actual number of VFs exposed through SRIOV PCIe config capability through sysfs.

* The parameters `bandwidth` and `load_balance` are optional parameters impacting arbitration but are expected to be kept as default.

## Conversion to PDF

To convert this readme to a PDF file, first install npm tool, followed by
`markdown-pdf` conversion program installation:

    sudo yum install npm
    npm install -g markdown-pdf

Convert to PDF:

    markdown-pdf README.md README.pdf

## License
SPDX-License-Identifier: Apache-2.0

Copyright(c) 2018 Intel Corporation

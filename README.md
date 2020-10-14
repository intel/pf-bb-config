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

Configure the Devices using pf_bb_config app for VF usage with 5G enabled:

    ./pf_bb_config ACC100 -c acc100/acc100_config_vf_5g.cfg

Test the VF is functional on the device using bbdev-test:

    ../../${RTE_TARGET}/app/testbbdev -c F0 -w${VF_PF_PCI_ADDR} -- -c validation -v ./ldpc_dec_default.data

Additional documentation can also be found on DPDK for bbdev usage and within Intel FlexRAN documentation.

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

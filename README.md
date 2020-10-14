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

* `DEVICE_NAME`: Specifies device to configure, example: 'FPGA'
* `-c CFG_FILE`: Specifies configuration file to use
* `-p PCI_ID`: Specifies PCI ID of device to configure
* `-a`: Configures all PCI devices matching the given `DEVICE_NAME`
* `-h`: Prints help

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

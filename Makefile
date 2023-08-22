#******************************************************************************
#
#   Copyright (c) 2020 Intel.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#******************************************************************************/

CC=gcc
CFLAGS=-Wall -D_FORTIFY_SOURCE=2 -fstack-protector-strong -fPIE -z relro -z now -z noexecstack -s -O1
ODIR=build
DEPS=

INCLUDE=-I. -I./acc100 -I./vrb1 -I./vrb2 -I./fpga_lte -I./fpga_5gnr -I./agx100 -I./cfg_reader
LDFLAGS=-L.

SRC = config_app.c acc100/acc100_cfg_app.c acc100/acc100_cfg_parser.c \
	fpga_lte/fpga_lte_cfg_app.c fpga_lte/fpga_lte_cfg_parser.c \
	fpga_5gnr/fpga_5gnr_cfg_app.c fpga_5gnr/fpga_5gnr_cfg_parser.c \
	vrb1/vrb1_cfg_app.c vrb1/vrb1_cfg_parser.c cfg_reader/cfg_reader.c \
	vrb2/vrb2_cfg_app.c vrb2/vrb2_cfg_parser.c \
	agx100/agx100_cfg_app.c agx100/agx100_cfg_parser.c \
	bb_acc_vfio.c daemon.c bb_acc_log.c bb_acc_common.c
OBJ = $(patsubst %.c,$(ODIR)/%.o,$(SRC))

.PHONY: clean

all: pf_bb_config

$(ODIR):
	mkdir -p $(ODIR)

$(OBJ): $(ODIR)/%.o: ./%.c | $(DEPS) $(ODIR)
	@mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

pf_bb_config: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	rm -rf $(ODIR)
	rm -rf pf_bb_config

ROOT_DIR 	 := $(abspath $(CURDIR)/..)
BUILD_DIR 	 ?= $(abspath $(ROOT_DIR)/build)
HLS_TARGET 	 ?=
HLS_PKG_TYPE ?= xo
HLS_WORK_DIR := $(BUILD_DIR)/hls/$(HLS_TARGET)
HLS_SRC_DIR  := $(CURDIR)/hls

HLS_BUILD_RAW  := $(BUILD_DIR)/hls/raw
HLS_TARGET_RAW := $(HLS_BUILD_RAW)/$(HLS_TARGET).cpp
HLS_HPP_FILES := $(shell find hls -type f -name '*.hpp' 2>/dev/null)
HLS_HDR_FILES := $(patsubst hls/%,$(HLS_BUILD_RAW)/%,$(HLS_HPP_FILES))


$(HLS_BUILD_RAW)/%.hpp: hls/%.hpp
	ln -sfn $(abspath $<) $@

$(HLS_TARGET_RAW):
	@mkdir -p $(HLS_BUILD_RAW)
	@ln -s $(HLS_SRC_DIR)/$(HLS_TARGET).cpp $(HLS_BUILD_RAW)/$(HLS_TARGET).cpp
	@ln -s $(CURDIR)/tests/$(HLS_TARGET).cfg $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg
	@ln -s $(CURDIR)/tests/$(HLS_TARGET)_tb.cpp $(HLS_BUILD_RAW)/$(HLS_TARGET)_tb.cpp

csim: hls/$(HLS_TARGET).cpp $(HLS_TARGET_RAW) $(HLS_HDR_FILES)
	vitis-run --mode hls --csim --config $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg --work_dir $(HLS_WORK_DIR)
csyn: $(BUILD_DIR)/hls/raw
	v++ -c --mode hls --config $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg --work_dir $(HLS_WORK_DIR)
cosim: $(BUILD_DIR)/hls/raw
	vitis-run --mode hls --cosim --config $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg --work_dir $(HLS_WORK_DIR)
pkg: $(BUILD_DIR)/hls/raw
	vitis-run --mode hls --package --config $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg --work_dir $(HLS_WORK_DIR)
impl: $(BUILD_DIR)/hls/raw
	vitis-run --mode hls --impl --config $(HLS_BUILD_RAW)/$(HLS_TARGET).cfg --work_dir $(HLS_WORK_DIR)


$(BUILD_DIR)/kernels/%.$(TYPE): $(HLS_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
#	@v++ -c --mode hls --config $(basename $<).cfg --work_dir $(BUILD_DIR)

kernels:



VITIS_CLANGPP:=$(XILINX_VITIS)/lnx64/tools/clang-16/bin/clang++
HLS_SYN_HDR_DIR:=$(XILINX_VITIS)/common/technology/autopilot
HLS_SIM_HDR_DIR:=$(XILINX_VITIS)/include

$(BUILD_DIR)/compile_commands/hls/%.json: $(HLS_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@cp $(CURDIR)/misc/.hls.compile_commands.json $@
	@sed -i 's|BUILD_DIR|$(BUILD_DIR)|g' $@
	@sed -i 's|VITIS_CLANGPP|$(VITIS_CLANGPP)|g' $@
	@sed -i 's|HLS_SYN_HDR_DIR|$(HLS_SYN_HDR_DIR)|g' $@
	@sed -i 's|TOP|$(basename $(notdir $<))|g' $@
	@sed -i 's|FILE|$<|g' $@
	@echo "generated compile command for '$*.cpp'"

$(BUILD_DIR)/compile_commands/tb/%.json: $(CURDIR)/tests/%.cpp
	@mkdir -p $(dir $@)
	@cp $(CURDIR)/misc/.tb.compile_commands.json $@
	@sed -i 's|BUILD_DIR|$(BUILD_DIR)|g' $@
	@sed -i 's|VITIS_CLANGPP|$(VITIS_CLANGPP)|g' $@
	@sed -i 's|HLS_SRC_DIR|$(HLS_SRC_DIR)|g' $@
	@sed -i 's|HLS_SIM_HDR_DIR|$(HLS_SIM_HDR_DIR)|g' $@
	@sed -i 's|FILE|$<|g' $@
	@echo "generated compile command for '$*.cpp'"

compile_commands: $(patsubst $(HLS_SRC_DIR)/%.cpp,$(BUILD_DIR)/compile_commands/hls/%.json,$(shell find $(HLS_SRC_DIR) -name '*.cpp' 2>/dev/null)) \
                  $(patsubst $(CURDIR)/tests/%.cpp,$(BUILD_DIR)/compile_commands/tb/%.json,$(shell find $(CURDIR)/tests -name '*.cpp' 2>/dev/null))
	@mkdir -p $(BUILD_DIR)
	@jq -s '.' $^ > $(BUILD_DIR)/compile_commands.json
	@rm -rf $(BUILD_DIR)/compile_commands


.PHONY: echo compile_commands csim kernels
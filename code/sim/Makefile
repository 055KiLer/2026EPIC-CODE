# 主机侧仿真台（在 WSL/Linux 下用 gcc 编译；不参与 Keil 上机构建）
#
#   make            构建 out/sim（几何已回填）与 out/sim_zero（出厂配置）
#   make run        依次跑全部场景并汇总
#   make s1         只跑 S1
#   make s1 A=v     只跑 S1 并打印逐事件（A= 后面的参数原样透传给 out/sim）
#   make clean
#
# 一个进程只跑一个场景：产品代码的 static 记账状态没有复位接口。

CC       := gcc
INC      := -I. -Ishim -I../Config \
            -I../User/Algorithm/Inc -I../User/App/Inc \
            -I../User/Bsp/Inc -I../User/Devices/Inc
WARN     := -Wall -Wextra -Wno-unused-parameter -Wno-address-of-packed-member
# -DSIM：产品代码里的仿真分支开关（当前唯一分叉点 = bsp_uart 的发送路径走
# HAL_UART_Transmit —— 仿真台的屏丢帧注入/按波特率计时挂在它上面，见 bsp_uart.c）
CFLAGS   := -std=gnu99 -O0 -g $(WARN) $(INC) -DSIM
LDLIBS   := -lm

SRC      := $(wildcard ../User/Algorithm/Src/*.c) \
            $(wildcard ../User/App/Src/*.c) \
            $(wildcard ../User/Bsp/Src/*.c) \
            $(wildcard ../User/Devices/Src/*.c) \
            $(wildcard ./*.c)

# 目标名里绝不能出现 ".."：out/fill/../User/x.o 与 out/zero/../User/x.o
# 会被文件系统解析成同一个文件，两套构建互相覆盖。改用 VPATH + 纯文件名。
VPATH    := ../User/Algorithm/Src:../User/App/Src:../User/Bsp/Src:../User/Devices/Src:.
NAMES    := $(notdir $(SRC))
DEPFLAGS := -MMD -MP
CFLAGS   += $(DEPFLAGS)

OBJ_FILL := $(addprefix out/fill/,$(NAMES:.c=.o))
OBJ_ZERO := $(addprefix out/zero/,$(NAMES:.c=.o))

SCENES   := s1 s2 s3 s6 s7 s8 s9 s10 s11 s12 s13 s14 s15 s16 s17 s19 s20 s21 s22 s23


all: out/sim out/sim_zero

out/sim: $(OBJ_FILL)
	@mkdir -p $(dir $@)
	$(CC) $^ -o $@ $(LDLIBS)

out/sim_zero: $(OBJ_ZERO)
	@mkdir -p $(dir $@)
	$(CC) $^ -o $@ $(LDLIBS)

out/fill/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -DSIM_FILL_GEOMETRY -c $< -o $@

out/zero/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

-include $(OBJ_FILL:.o=.d) $(OBJ_ZERO:.o=.d)

define SCENE_RULE
$(1): all
	@out/sim $(1) $(A)
endef
$(foreach sc,$(SCENES),$(eval $(call SCENE_RULE,$(sc))))


list: all
	@out/sim 2>/dev/null || true

run: all
	@pass=0; fail=0; \
	for sc in $(SCENES); do \
	  if out/sim $$sc > out/log_$$sc.txt 2>&1; then \
	    echo "  PASS  $$sc  $$(grep -m1 '^场景' out/log_$$sc.txt)"; \
	    pass=$$((pass+1)); \
	  else \
	    echo "  FAIL  $$sc  $$(grep -m1 '^场景' out/log_$$sc.txt)"; \
	    fail=$$((fail+1)); \
	  fi; \
	done; \
	echo "------------------------------------------"; \
	echo "  PASS $$pass / FAIL $$fail  （日志 out/log_*.txt）"

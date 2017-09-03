## @author: Bharat Tak  ##

OUT ?= ../out
OUT_DIR := $(OUT)/bin
MAVCONN_INC := ../libmavconn/include/
MAVLINK_INC := ../libmavconn/include/mavlink/include
CC := gcc
CFLAGS := -g -std=c++11
override INCLUDES += -I$(MAVCONN_INC)
override INCLUDES += -I$(MAVLINK_INC)
SYS := $(shell $(CC) -dumpmachine)

TARGET := $(OUT_DIR)/camera_interface
SRC_DIR := sources
SRC_DIR1 := ../libmavconn/src
OBJ_DIR := .obj
OBJ_DIR1 := .obj1
SOURCES := main.cpp socket.cpp
SOURCES1 := interface.cpp mavlink_helpers.cpp serial.cpp tcp.cpp udp.cpp

OBJECTS := $(addprefix $(OBJ_DIR)/, $(SOURCES:.cpp=.o))
OBJECTS1 := $(addprefix $(OBJ_DIR1)/, $(SOURCES1:.cpp=.o))
OBJECTS += $(OBJECTS1)
DEPENDS := $(patsubst %.o,%.d, $(OBJECTS))
DEPENDS1 := $(patsubst %.o,%.d, $(OBJECTS1))

all: $(OUT_DIR) $(OBJ_DIR) $(OBJ_DIR1) $(TARGET)

objs: $(OBJ_DIR) $(OBJ_DIR1) $(OBJECTS)

clean:
	rm -rf $(TARGET) $(OBJ_DIR) $(OBJ_DIR1)

$(TARGET): $(OBJECTS)
	$(CC) -L$(OUT)/lib -o $@ $^ -lmavconn -lstdc++ -lpthread -lboost_system

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR1):
	mkdir -p $(OBJ_DIR1)

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

$(OBJ_DIR1)/%.o : $(SRC_DIR1)/%.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

$(OBJ_DIR)/%.d : $(SRC_DIR)/%.cpp
	@[ -d $(OBJ_DIR) ] || mkdir -p $(OBJ_DIR)
	$(CC) -MM $(CFLAGS) $(INCLUDES) $< \
	| sed 's/$(notdir $*).o/$(subst /,\/,$(patsubst %.d,%.o,$@) $@)/' > $@ ; \
	[ -s $@ ] || rm -f $@

$(OBJ_DIR1)/%.d : $(SRC_DIR1)/%.cpp
	@[ -d $(OBJ_DIR1) ] || mkdir -p $(OBJ_DIR1)
	$(CC) -MM $(CFLAGS) $(INCLUDES) $< \
	| sed 's/$(notdir $*).o/$(subst /,\/,$(patsubst %.d,%.o,$@) $@)/' > $@ ; \
	[ -s $@ ] || rm -f $@

-include $(DEPENDS)

.PHONY: all clean

# Makefile
# ==============================================================================
# 07.02.2002 cmc :  Created based on sample from Kevin Nickels.
# 07.15.2004 cmc :  Updated and applied to sn_sim project.
# 08.05.2004 cmc :  Added tags to removed files in distclean, added "doc" target
# 12.16.2004 cmc :  Remove output files with distclean.
# ------------------------------------------------------------------------------
#  TODO: Figure out why LD won't link C++ object files with standard args.
# ==============================================================================

PROG = test
EPUCKLIBROOT = ../libs/epuck_libIRcom/e-puck/src/

# ==============================================================================
# Potential Epuck Libraries
# ------------------------------------------------------------------------------


EXTERNAL_OBJS = #blank

EXTERNAL_OBJS += $(wildcard $(EPUCKLIBROOT)/epfl/*.o)

EXTERNAL_OBJS += $(wildcard $(EPUCKLIBROOT)/btcom/*.o)

EXTERNAL_OBJS += $(wildcard $(EPUCKLIBROOT)/ircom/*.o)




# ==============================================================================
# Local variables
# ------------------------------------------------------------------------------
#  first, find all the pertinent filenames
ASMSRCS = $(wildcard *.s)
CSRCS   = $(wildcard *.c)
CXXSRCS = $(wildcard *.cc) $(wildcard *.cpp) $(wildcard *.C)
SRCS = $(CSRCS) $(CXXSRCS) $(ASMSRCS)
HDRS = $(wildcard *.h) $(wildcard *.hh)
OBJS = $(addsuffix .o,$(basename $(SRCS)))
DEPS = $(addsuffix .d,$(basename $(SRCS)))

#  program aliases
CTAGS    = ctags
ASM      = pic30-elf-as
CC       = pic30-elf-gcc
LD	 = pic30-elf-ld

#  common options
DEPCFLAGS = -Wall -I. -I$(EPUCKLIBROOT)/std_microchip/support/h -I$(EPUCKLIBROOT)/std_microchip/include -I$(EPUCKLIBROOT) -I$(EPUCKLIBROOT)/std_microchip/support/h
CFLAGS   = $(DEPCFLAGS) -mcpu=30F6014A -DROBOT_ID=$(ROBOT_ID)

LDFLAGS  = -L$(EPUCKLIBROOT)/std_microchip/lib --defsym=__ICD2RAM=1 --script=$(EPUCKLIBROOT)/std_microchip/support/gld/p30f6014a.gld -mpic30_elf32
LDLIBS   = -lpic30-elf -lm-elf -lc-elf
#LDLIBS   = -lpic30-coff -lm-coff -lc-coff
#LDLIBS   = --start-group -lpic30-coff -lm-coff -lc-coff --end-group
#LDLIBS   = --start-group -lpic30-elf -lm-elf -lc-elf --end-group

ifeq ($(MAKECMDGOALS),debug)
CFLAGS += -g -DDEBUG -DVERBOSITY=0
endif


# ==============================================================================
# Dependencies & rules
# ------------------------------------------------------------------------------

echo:
	@echo $(ASM)

all: $(PROG).hex tags $(DEPS)
	

.PHONY: debug
	debug: all

%.hex: %.cof
	pic30-elf-bin2hex $(PROG).cof

%.cof: $(OBJS)
	$(LD) $(LDFLAGS) --start-group $(LDLIBS) $(OBJS) $(EXTERNAL_OBJS) --end-group --heap=512 -o $@

%.o: %.s
	$(ASM) -I$(EPUCKLIBROOT)/std_microchip/support/inc -p=30F6014A $< -o $@
# ==============================================================================
# Clean up directory
# ------------------------------------------------------------------------------
.PHONY:	clean
clean:
	- $(RM) $(OBJS) *~ core.* *.rpo $(PROG).cof $(PROG).hex

.PHONY: distclean
distclean: clean
	- $(RM) $(DEPS) tags *.a *.so $(OUTPUT)
# ==============================================================================


# ==============================================================================
# make tags files for vim
# ------------------------------------------------------------------------------
tags:	$(SRCS) $(HDRS)
	$(CTAGS) $(SRCS) $(HDRS)
# ==============================================================================


# ==============================================================================
# a default rule for building dependency files
# ------------------------------------------------------------------------------
%.d: %.c
	@ $(SHELL) -ec '$(CXX) -MM $(DEPCFLAGS) $< > $@'
%.d: %.cc
	@ $(SHELL) -ec '$(CXX) -MM $(DEPCFLAGS) $< > $@'
%.d: %.C
	@ $(SHELL) -ec '$(CXX) -MM $(DEPCFLAGS) $< > $@'
%.d: %.cpp
	@ $(SHELL) -ec '$(CXX) -MM $(DEPCFLAGS) $< > $@'
%.d: %.s
	cat /dev/null > ./$@
# ==============================================================================


# ==============================================================================
# include the source code dependencies
# ------------------------------------------------------------------------------
ifneq ($(MAKECMDGOALS),clean)
include $(DEPS)
endif
# ==============================================================================


# ==============================================================================
# end of Makefile

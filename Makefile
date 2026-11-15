# Command-line build. The OMNeT++ IDE does the same from .oppbuildspec.
#
#   make makefiles            # once (INET next to this project: ../inet4.5)
#   make makefiles INET_PROJ=/path/to/inet4.5
#   make                      # debug build;  make MODE=release  for speed
#
INET_PROJ ?= ../inet4.5
INET_ABS := $(abspath $(INET_PROJ))

.PHONY: all clean cleanall makefiles checkmakefiles

all: checkmakefiles
	@$(MAKE) -C src

clean: checkmakefiles
	@$(MAKE) -C src clean

cleanall: checkmakefiles
	@$(MAKE) -C src MODE=release clean
	@$(MAKE) -C src MODE=debug clean
	@rm -f src/Makefile

makefiles:
	@test -d "$(INET_ABS)/src/inet" || (echo "INET not found in $(INET_ABS): use make makefiles INET_PROJ=/path/to/inet4.5"; exit 1)
	@cd src && opp_makemake -f --deep -o aiensim -O out -KINET4_5_PROJ=$(INET_ABS) -DINET_IMPORT \
	    -I'$$(INET4_5_PROJ)/src' -L'$$(INET4_5_PROJ)/src' -lINET'$$(D)'

checkmakefiles:
	@if [ ! -f src/Makefile ]; then \
	echo; echo '=== src/Makefile missing: run "make makefiles" first ==='; echo; exit 1; fi


.PHONY: all clean

all clean:
ifneq "$(SUBDIRS)" ""
	(for i in ${SUBDIRS}; do (cd $$i; pwd; ${MAKE} $@); done)
endif
ifneq "$(SUBMAKES)" ""
	(for i in ${SUBMAKES}; do (${MAKE} -f $$i $@;); done)
endif

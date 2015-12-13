TOP=.

include $(TOP)/configure/CONFIG

DBD			= 	evg.dbd

LIBRARY_IOC	=	evg
evg_SRCS	+= 	evg.c
evg_SRCS	+= 	bo.c
evg_SRCS	+= 	longout.c
evg_LIBS	+= 	$(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

TOP=.

include $(TOP)/configure/CONFIG

DBD			=	evg.dbd

LIBRARY_IOC	=	evg
evg_SRCS	+= 	evg.c parse.c
evg_SRCS	+= 	bi.c
evg_SRCS	+= 	bo.c
evg_SRCS	+= 	ai.c
evg_SRCS	+= 	ao.c
evg_SRCS	+= 	longin.c
evg_SRCS	+= 	longout.c
evg_SRCS	+= 	mbbi.c
evg_SRCS	+= 	mbbo.c
evg_LIBS	+= 	$(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

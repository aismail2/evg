TOP=..

include $(TOP)/configure/CONFIG

# Library
LIBRARY_IOC	=	evg
evg_SRCS	+= 	evg.c
evg_SRCS	+= 	bo.c
evg_SRCS	+= 	longout.c
evg_LIBS	+= 	$(EPICS_BASE_IOC_LIBS)
INSTALL_DBDS+= 	$(INSTALL_DBD)/evg.dbd

# IOC
PROD_IOC	=	evgApp
evgApp_SRCS	+=	evgApp_registerRecordDeviceDriver.cpp
evgApp_SRCS	+=	app.cpp
evgApp_LIBS	+=	evg
evgApp_LIBS	+=	$(EPICS_BASE_IOC_LIBS)
evgApp_DBD	+=	base.dbd
evgApp_DBD	+=	evg.dbd
DBD			+=	evgApp.dbd
DB			+= 	mrf-vmeevg230.db
DB			+= 	mrf-vmeevg230-event.db
DB			+= 	mrf-vmeevg230-counter.db


include $(TOP)/configure/RULES

#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=hex
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=hex
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=FERRET_Control.asm

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/FERRET_Control.o
POSSIBLE_DEPFILES=${OBJECTDIR}/FERRET_Control.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/FERRET_Control.o

# Source Files
SOURCEFILES=FERRET_Control.asm


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=EEPROM8
MP_LINKER_DEBUG_OPTION= 
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FERRET_Control.o: FERRET_Control.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FERRET_Control.o.d 
	@${RM} ${OBJECTDIR}/FERRET_Control.o 
	@${FIXDEPS} dummy.d -e "C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.ERR" $(SILENT) -rsi ${MP_AS_DIR} -c ${MP_AS} $(MP_EXTRA_AS_PRE) -d__DEBUG  -q -p$(MP_PROCESSOR_OPTION)  $(ASM_OPTIONS)    \"C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.asm\" 
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.O ${OBJECTDIR}/FERRET_Control.o
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.ERR ${OBJECTDIR}/FERRET_Control.o.err
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.LST ${OBJECTDIR}/FERRET_Control.o.lst
	@${DEP_GEN} -d "${OBJECTDIR}/FERRET_Control.o"
	@${FIXDEPS} "${OBJECTDIR}/FERRET_Control.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
else
${OBJECTDIR}/FERRET_Control.o: FERRET_Control.asm  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FERRET_Control.o.d 
	@${RM} ${OBJECTDIR}/FERRET_Control.o 
	@${FIXDEPS} dummy.d -e "C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.ERR" $(SILENT) -rsi ${MP_AS_DIR} -c ${MP_AS} $(MP_EXTRA_AS_PRE) -q -p$(MP_PROCESSOR_OPTION)  $(ASM_OPTIONS)    \"C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.asm\" 
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.O ${OBJECTDIR}/FERRET_Control.o
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.ERR ${OBJECTDIR}/FERRET_Control.o.err
	@${MV}  C:/Users/Luke/Documents/Github_Repositories/ME218C_FERRET/ME218C_FERRET.X/FERRET_Control.LST ${OBJECTDIR}/FERRET_Control.o.lst
	@${DEP_GEN} -d "${OBJECTDIR}/FERRET_Control.o"
	@${FIXDEPS} "${OBJECTDIR}/FERRET_Control.o.d" $(SILENT) -rsi ${MP_AS_DIR} -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${MV}  "FERRET_Control.HEX" dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${MV}  "FERRET_Control.HEX" dist/${CND_CONF}/${IMAGE_TYPE}/ME218C_FERRET.X.${IMAGE_TYPE}.hex 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

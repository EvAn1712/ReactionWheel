#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=RealTarget
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/SetpointLib.o \
	${OBJECTDIR}/SocketLib.o \
	${OBJECTDIR}/WheelHMI.o \
	${OBJECTDIR}/WheelLaws.o \
	${OBJECTDIR}/WheelTasks.o \
	${OBJECTDIR}/real/Analog.o \
	${OBJECTDIR}/real/Encoder.o \
	${OBJECTDIR}/real/Gyroscope.o \
	${OBJECTDIR}/real/HardwareLib.o \
	${OBJECTDIR}/real/Motor.o


# C Compiler Flags
CFLAGS=$(XENOCFLAGS)

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lm

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/reactionwheel

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/reactionwheel: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/reactionwheel ${OBJECTFILES} ${LDLIBSOPTIONS} $(XENOLDFLAGS)

${OBJECTDIR}/SetpointLib.o: SetpointLib.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SetpointLib.o SetpointLib.c

${OBJECTDIR}/SocketLib.o: SocketLib.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SocketLib.o SocketLib.c

${OBJECTDIR}/WheelHMI.o: WheelHMI.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/WheelHMI.o WheelHMI.c

${OBJECTDIR}/WheelLaws.o: WheelLaws.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/WheelLaws.o WheelLaws.c

${OBJECTDIR}/WheelTasks.o: WheelTasks.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/WheelTasks.o WheelTasks.c

${OBJECTDIR}/real/Analog.o: real/Analog.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}/real
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/real/Analog.o real/Analog.c

${OBJECTDIR}/real/Encoder.o: real/Encoder.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}/real
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/real/Encoder.o real/Encoder.c

${OBJECTDIR}/real/Gyroscope.o: real/Gyroscope.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}/real
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/real/Gyroscope.o real/Gyroscope.c

${OBJECTDIR}/real/HardwareLib.o: real/HardwareLib.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}/real
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/real/HardwareLib.o real/HardwareLib.c

${OBJECTDIR}/real/Motor.o: real/Motor.c nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} -p ${OBJECTDIR}/real
	${RM} "$@.d"
	$(COMPILE.c) -I. -Ireal -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/real/Motor.o real/Motor.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

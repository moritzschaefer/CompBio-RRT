PROJECT(tutorialPlan)

FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(Cgal REQUIRED)
FIND_PACKAGE(Coin REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Iconv)
FIND_PACKAGE(Solid)
FIND_PACKAGE(LibXml2 REQUIRED)
FIND_PACKAGE(Bullet REQUIRED)
FIND_PACKAGE(SoQt REQUIRED)
FIND_PACKAGE(ZLIB)

ADD_DEFINITIONS(
	-D_SCL_SECURE_NO_WARNINGS
	-DCOIN_NOT_DLL
	-DCGAL_DISABLE_ROUNDING_MATH_CHECK
	-DNOMINMAX
	-DSOQT_NOT_DLL
	${QT_DEFINITIONS}
)

INCLUDE_DIRECTORIES(
	BEFORE
	${rl_SOURCE_DIR}/src
	${Boost_INCLUDE_DIR}
	${CGAL_INCLUDE_DIRS}
	${COIN_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIRS}
	${QT_INCLUDES}
	${BULLET_INCLUDE_DIRS}
	${SOQT_INCLUDE_DIRS}
	${SOLID_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
)

SET(
	HDRS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
	TutorialPlanSystem.h
    YourPlanner.h
)

SET(
	SRCS
        qt_visualization/QtWindow.cpp
        qt_visualization/QtViewer.cpp
        qt_visualization/QtPlanningThread.cpp
	tutorialPlan.cpp
	TutorialPlanSystem.cpp
    YourPlanner.cpp
)

QT4_WRAP_CPP(
        MOC_SRCS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
)

ADD_EXECUTABLE(
	tutorialPlan
	${HDRS}
	${SRCS}
        ${MOC_SRCS}
)

TARGET_LINK_LIBRARIES(
	tutorialPlan
	rlplan
	rlkin
	rlsg
	${BULLET_LIBRARIES}
	${CGAL_LIBRARIES}
	${COIN_LIBRARIES}
	${LIBXML2_LIBRARIES}
#	${ODE_LIBRARIES}
#	${PQP_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	${SOLID_LIBRARIES}
	${SOQT_LIBRARIES}
#	${SWIFTPP_LIBRARIES}
)

FILE(
    COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/xml
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

SET_TARGET_PROPERTIES(
	tutorialPlan
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

INSTALL(
	TARGETS
	tutorialPlan
	ARCHIVE DESTINATION share/rl/examples
	LIBRARY DESTINATION share/rl/examples
	RUNTIME DESTINATION share/rl/examples
)

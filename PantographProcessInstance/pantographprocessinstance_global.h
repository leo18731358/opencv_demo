#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(PANTOGRAPHPROCESSINSTANCE_LIB)
#  define PANTOGRAPHPROCESSINSTANCE_EXPORT Q_DECL_EXPORT
# else
#  define PANTOGRAPHPROCESSINSTANCE_EXPORT Q_DECL_IMPORT
# endif
#else
# define PANTOGRAPHPROCESSINSTANCE_EXPORT
#endif
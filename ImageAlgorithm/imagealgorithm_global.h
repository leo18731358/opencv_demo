#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(IMAGEALGORITHM_LIB)
#  define IMAGEALGORITHM_EXPORT Q_DECL_EXPORT
# else
#  define IMAGEALGORITHM_EXPORT Q_DECL_IMPORT
# endif
#else
# define IMAGEALGORITHM_EXPORT
#endif

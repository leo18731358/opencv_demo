#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(DEALCOMMON_LIB)
#  define DEALCOMMON_EXPORT Q_DECL_EXPORT
# else
#  define DEALCOMMON_EXPORT Q_DECL_IMPORT
# endif
#else
# define DEALCOMMON_EXPORT
#endif

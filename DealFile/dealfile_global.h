#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(DEALFILE_LIB)
#  define DEALFILE_EXPORT Q_DECL_EXPORT
# else
#  define DEALFILE_EXPORT Q_DECL_IMPORT
# endif
#else
# define DEALFILE_EXPORT
#endif

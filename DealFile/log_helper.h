#ifndef LOGHELPER_H
#define LOGHELPER_H

#include "dealfile_global.h"
#include <QObject>
#include <QMutex>
#include "..\DealCommon\path_define_file.h"

class DEALFILE_EXPORT LogHelper
{

private:
	QString createDayFile(QString pathBase);        
	QString createHourFile(QString pathBase);
	QString __makeMultiDir(QString dir);

public:
	virtual QString createAllTimeFile(QString pathBase);
	virtual void writeError(QString strName, QString strError);

private:
	QMutex mt_Log;
	QMutex mt_WriteError;
	QString m_strRootFilePath;

public:
	static LogHelper *_instance;

};

extern "C"
{
	DEALFILE_EXPORT LogHelper *LogInstance();
}


#endif // !LOGHELPER
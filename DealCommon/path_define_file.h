#ifndef PATHDEFINEFILE_H
#define PATHDEFINEFILE_H

#include <QObject>
#include <QCoreApplication>


static QString m_strAllSoftRootPath = QCoreApplication::applicationDirPath() + "\\";
static QString m_strAllSoftGrabLogPath = m_strAllSoftRootPath + "Log\\";
static QString m_strAllSoftGrabParamPath = m_strAllSoftRootPath + "Params\\";

void static SetGlobalParamPath(QString AppRootPath)
{
	m_strAllSoftRootPath = AppRootPath + "\\";
	m_strAllSoftGrabLogPath = m_strAllSoftRootPath + "Log\\";
	m_strAllSoftGrabParamPath = m_strAllSoftRootPath + "Params\\";
}

static bool m_bAllThreadNeadRun = true;



#endif // !PATHDEFINEFILE_H


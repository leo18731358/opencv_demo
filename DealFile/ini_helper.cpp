#include "ini_helper.h"

#include <QSettings>
#include <QDebug>

void IniHelper::writeIni(QString section, QString key, QString value, QString filePath)
{
	QSettings setting(filePath, QSettings::IniFormat);
	setting.beginGroup(section);
	setting.setValue(key, value);
	setting.endGroup();
}

QString IniHelper::readIniStr(QString section, QString key, QString filePath)
{
	QSettings setting(filePath, QSettings::IniFormat);
	QString objReturn = setting.value(section + "/" + key).toString();
	return objReturn;
}

double IniHelper::readIniDouble(QString section, QString key, QString filePath)
{
	QSettings setting(filePath, QSettings::IniFormat);
	double objReturn = setting.value(section + "/" + key).toDouble();
	return objReturn;
}

int IniHelper::readIniInt(QString section, QString key, QString filePath)
{
	QSettings setting(filePath, QSettings::IniFormat);
	double objReturn = setting.value(section + "/" + key).toInt();
	return objReturn;
}


IniHelper *IniHelper::_instance = NULL;
DEALFILE_EXPORT IniHelper *IniInstance()
{
	if (IniHelper::_instance == NULL)
	{
		IniHelper::_instance = new IniHelper();
	}
	return IniHelper::_instance;
}
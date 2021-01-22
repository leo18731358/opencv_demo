#ifndef INIHELPER_H
#define INIHELPER_H

#include "dealfile_global.h"
#include <iostream>
#include "..\DealCommon\path_define_file.h"

class DEALFILE_EXPORT IniHelper
{

public:
	virtual void writeIni(QString section, QString key, QString value, QString filePath);
	virtual QString readIniStr(QString section, QString key, QString filePath);
	virtual double readIniDouble(QString section, QString key, QString filePath);
	virtual int readIniInt(QString section, QString key, QString filePath);


public:
	static IniHelper *_instance;


};

extern "C"
{


	DEALFILE_EXPORT IniHelper *IniInstance();
}


#endif // !INIHELPER_H
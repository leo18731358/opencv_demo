#ifndef IMPORTDLLCLASS_H
#define IMPORTDLLCLASS_H


#include <QObject>
#include <QLibrary>
#include "..\ImageAlgorithm\algorithm_base.h"
#include "..\DealFile\ini_helper.h"
#include "..\DealFile\log_helper.h"


class  ImportDllClass
{
public:
	 ImportDllClass();
	 ~ImportDllClass();


public:
	AlgorithmBase *m_pAlgorithmBase = NULL;
	IniHelper *m_pIniHelper = NULL;
	LogHelper *m_pLogHelper = NULL;

	static ImportDllClass *_instance;
	static ImportDllClass *GetInstance();

};


#endif // !IMPORTDLLCLASS_H

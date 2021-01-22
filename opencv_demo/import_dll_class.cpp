
#include "import_dll_class.h"


ImportDllClass::ImportDllClass()
{
	typedef AlgorithmBase* (*AlgorithmBaseFun)();
	QLibrary *AlgorithmBaseDll = new QLibrary("ImageAlgorithm.dll");
	if (AlgorithmBaseDll->load())
	{
		AlgorithmBaseFun algorithmFun = (AlgorithmBaseFun)AlgorithmBaseDll->resolve("getAlgorithmBaseInstance");
		if (algorithmFun)
		{
			m_pAlgorithmBase = algorithmFun();
		}
	}

	typedef PantographProcessInstance* (*PantographProcessInstanceFun)();
	QLibrary *PantographProcessInstanceDll = new QLibrary("PantographProcessInstance.dll");
	if (PantographProcessInstanceDll->load())
	{
		PantographProcessInstanceFun pantographProcessInstanceFun = (PantographProcessInstanceFun)PantographProcessInstanceDll->resolve("getPantographProcessInstance");
		if (pantographProcessInstanceFun)
		{
			m_pPantographProcessInstance = pantographProcessInstanceFun();
		}
	}

	typedef IniHelper* (*IniHelperFun)();
	typedef LogHelper* (*LogHelperFun)();
	QLibrary *DealFileDll = new QLibrary("DealFile.dll");
	if (DealFileDll->load())
	{
		IniHelperFun iniHelperFun = (IniHelperFun)DealFileDll->resolve("IniInstance");
		if (iniHelperFun)
		{
			m_pIniHelper = iniHelperFun();
		}
		LogHelperFun logHelperFun = (LogHelperFun)DealFileDll->resolve("LogInstance");
		if (logHelperFun)
		{
			m_pLogHelper = logHelperFun();
		}
	}

}
ImportDllClass::~ImportDllClass()
{
}

ImportDllClass* ImportDllClass::_instance = NULL;
ImportDllClass* ImportDllClass::GetInstance()
{
	if (NULL == ImportDllClass::_instance)
	{
		ImportDllClass::_instance = new ImportDllClass();
	}
	return ImportDllClass::_instance;
}

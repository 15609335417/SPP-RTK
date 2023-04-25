#include"Config.h"
int liyuan = 0;
Config cfg = Config("配置文件.cfg");
Config _SetConfig(const char* filename)
{
	Config _cfg;
	FILE* fp = NULL;
	char buffer[256];
	if (fopen_s(&fp, filename, "r+b"))
	{
		printf("Open Error! Please check out .cfg!");
		return _cfg;
	}
	//逐行读取配置文件
	fgets(buffer, 256, fp);
	fgets(buffer, 256, fp);         _cfg.type = atoi(buffer + 32);
	fgets(buffer, 256, fp);         _cfg.mode = atoi(buffer + 32);
	fgets(buffer, 256, fp);         _cfg.method = atoi(buffer + 32);

	fgets(buffer, 256, fp);         _cfg.COMNo = atoi(buffer + 32);
									_cfg.baudrate = atoi(buffer + 34);
	fgets(buffer, 256, fp);         memcpy(_cfg.baseIPaddress, buffer + 32, 14);
									_cfg.baseport = atoi(buffer + 47);
	fgets(buffer, 256, fp);         memcpy(_cfg.roverIPaddress, buffer + 32, 14);
	                                _cfg.roverport = atoi(buffer + 47);
	fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", _cfg.baseobsdatafile, 256);
	fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", _cfg.roverobsdatafile, 256);
	fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", _cfg.postionresultfile, 256);
	fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", _cfg.postiondifffile, 256);
	fgets(buffer, 256, fp);         _cfg.pseudorangenoise = atof(buffer + 32);
	fgets(buffer, 256, fp);         _cfg.eleuationmask = atoi(buffer + 32);

	fclose(fp);
	return _cfg;
}

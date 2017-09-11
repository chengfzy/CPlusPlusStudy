#include <iostream>
#define GOOGLE_GLOG_DLL_DECL           // 使用静态glog库用这个
//#define GLOG_NO_ABBREVIATED_SEVERITIES // 没这个编译会出错,传说因为和Windows.h冲突
#include "glog/logging.h"

using namespace std;

int main(int argc, char* argv[]) {
	google::InitGoogleLogging(argv[0]);
	//google::SetLogDestination(google::GLOG_INFO, "./mylog.info");
	FLAGS_logtostderr = true;
	

	LOG(INFO) << "google log begin..." << endl;
	LOG(INFO) << "something at info level";
	LOG(WARNING) << "something at warn level";
	LOG(ERROR) << "something at error level";
	//LOG(FATAL) << "something at fatal level";

	system("pause");
	return 0;
}
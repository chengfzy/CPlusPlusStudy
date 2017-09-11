#include <iostream>
#define GOOGLE_GLOG_DLL_DECL           // ʹ�þ�̬glog�������
//#define GLOG_NO_ABBREVIATED_SEVERITIES // û�����������,��˵��Ϊ��Windows.h��ͻ
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
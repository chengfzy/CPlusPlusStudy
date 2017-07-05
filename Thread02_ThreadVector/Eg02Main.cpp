#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using namespace std;
using namespace chrono;

class Blender
{
public:
	Blender(int dataNum)
		: m_nDataNum(dataNum), m_aData(m_nDataNum)
	{
		int bandNum{ 6 };
		for (int i = 0; i < m_nDataNum; ++i)
		{
			vector<int> data(bandNum);
			m_aData[i] = data;
		}
	}

	//handle data
	void HandleData(int index)
	{
		vector<int>& data = m_aData[index];
		for (int i = 0; i < data.size(); ++i)
		{
			for (int j = 0; j < 10; ++j)
			{
				++data[i];
				_sleep(100);
			}
		}
	}
private:

	int m_nDataNum;
	vector<vector<int>> m_aData;
};


int main()
{
	int nDataNum{ 10 };
	Blender blender(nDataNum);

	system_clock::time_point t0 = system_clock::now();

#if 0
	for (int i = 0; i < nDataNum; ++i)
	{
		blender.HandleData(i);
	}
#else
	vector<thread> aThread;
	for (int i = 0; i < nDataNum; ++i)
	{
		aThread.push_back(thread([&]()
		{
			blender.HandleData(i);
		}));
	}

	//wait thread complete
	for (auto it = aThread.begin(); it != aThread.end(); ++it)
	{
		it->join();
	}
#endif

	system_clock::time_point t1 = system_clock::now();
	cout << "Complete, time = " << duration_cast<milliseconds>(t1 - t0).count() << " ms" << endl;
	system("pause");
}
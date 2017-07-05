#include <iostream>
#include <thread>

using namespace std;

//task function
void task01()
{
	cout << "Thread task" << endl;
}

void test1()
{
	cout << "test01" << endl;

	thread t(task01);
	t.join();

	cout << "test01 end" << endl;
}


//task02
class Task02
{
public:
	Task02(int& i) : m_i(i) {}

	void operator()()
	{
		for (int i = 0; i < 1000000; ++i)
		{
			m_i++;
			_sleep(10);
		}
		cout << "task02, i = " << m_i << endl;
	}
private:
	int& m_i;
};

void oops()
{
	int someLocalState{ 0 };
	Task02 task02(someLocalState);
	thread t(task02);
	t.detach();
}

int main()
{
	test1();
	oops();

	//system("pause");
}
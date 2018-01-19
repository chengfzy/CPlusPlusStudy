#include <iostream>
using namespace std;

class Point
{
public:
	Point(int x = 0, int y = 0)
		: m_x(x), m_y(y)
	{}

	~Point() {}


	int GetX() const
	{
		return m_x;
	}

	int GetY() const
	{
		return m_y;
	}


	void SetX(int x)
	{
		m_x = x;
	}

	void SetY(int y)
	{
		m_y = y;
	}
private:
	int m_x;
	int m_y;
};


void test1(Point* point)
{
	cout << "Test Point (" << point->GetX() << ", " << point->GetY() << ")" << endl;
}

void test2(Point*& point)
{
	cout << "Test Point (" << point->GetX() << ", " << point->GetY() << ")" << endl;
}

int main()
{
	std::string str1 = "ABCDE";
	std::string str2 = "ABCDE";
	bool b1 = str1 == str2;
	
	int a1 = 3;
	int& a2 = a1;
	int& a3 = a2;

	a3 = 2;

	Point* pPoint = new Point(12, 34);

	test1(pPoint);
	test2(pPoint);
	test1(nullptr);
	//test2(nullptr);		//±àÒëError

	system("pause");
	return 0;
}
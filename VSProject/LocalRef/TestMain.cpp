#include <iostream>
#include <vector>
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


Point GeneratePoint()
{
	Point p(1, 2);
	return p;
}

void PrintPoint(const Point& point)
{
	cout << "(" << point.GetX() << ", " << point.GetY() << ")" << endl;
}


vector<Point> GeneratePointList()
{
	vector<Point> aPoint;
	aPoint.push_back(Point(1, 2));
	aPoint.push_back(Point(2, 3));
	aPoint.push_back(Point(3, 4));
	aPoint.push_back(Point(4, 5));
	return aPoint;
}

void PrintPointList(const vector<Point>& pointList)
{
	for (auto point : pointList)
	{
		cout << "(" << point.GetX() << ", " << point.GetY() << ")  ";
	}
}

int main()
{
	for (int i = 0; i < 10; ++i)
		PrintPoint(GeneratePoint());


	PrintPointList(GeneratePointList());

	system("pause");
	return 0;
}
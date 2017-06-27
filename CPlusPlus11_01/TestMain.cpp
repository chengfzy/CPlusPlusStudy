#include <iostream>
#include <vector>
#include <functional>
using namespace std;
using namespace std::placeholders;

int func(int x, int y, int a, int b)
{
	return a + b + x + y;
}

void func2(ostream& o, int x)
{
	o << x << endl;
}

int main()
{
	int x{ 10 };
	int y{ 20 };
	auto f = [=, &y](int a, int b) {++y; return a + b + x + y; };
	cout << f(1, 2) << endl;
	cout << y << endl;


	auto f_wrap = bind(func, x, y, _1, _2);
	cout << f_wrap(33, 44) << endl;

	auto f2_wrap = bind(func2, ref(cout), _1);
	f2_wrap(x);

	system("pause");
	return 0;
}
#include <iostream>
#include <string>
#include <sstream>
#include <regex>

using namespace std;

int main(int argc, char *argv[])
{
	string routeList = "D0->C5->C7->C10->C23->D0\nD0->C6->C8->C11->C24->D0\n";
	regex listPattern("(D0((->(C\\d{1,3}))+)->D0\\s)+");
	regex routePattern("D0((->(C\\d{1,3}))+)->D0\\s");
	regex customerPattern("->C(\\d{1,3})+");
	smatch listMatch;
	if (regex_search(routeList, listMatch, listPattern))
	{
		cout << "Input Correct" << endl;

		string routeStr = listMatch[0];
		smatch routeMatch;
		while (regex_search(routeStr, routeMatch, routePattern))
		{
			cout << "Route: " << routeMatch[0] << endl;

			string customerStr = routeMatch[1];
			smatch customerMatch;
			while (regex_search(customerStr, customerMatch, customerPattern))
			{
				cout << customerMatch[0] << ", " << customerMatch[1] << endl;

				stringstream strStream;
				strStream << customerMatch[1] << endl;
				int id;
				strStream >> id;

				customerStr = customerMatch.suffix();
			}
			routeStr = routeMatch.suffix();
		}
	}
	//regex pattern("D(\\d)((->C(\\d{1,3}))*)->D(\\d)");
	//string str = "D0->C5->C7->C10->C23->D0";
	//match_results<std::string::const_iterator> result;
	//bool valid = regex_match(str, result, pattern);

	//for (int i = 1; i < result.size(); ++i)
	//{
	//	cout << "[" << result[i] << "] ";
	//}

	//string subStr = result[2];
	//regex subPattern("->C(\\d{1,3})");
	//match_results<std::string::const_iterator> subResult;
	//valid = regex_match(subStr, subResult, subPattern);
	//cout << "SubString" << endl;
	//for (int i = 1; i < subResult.size(); ++i)
	//{
	//	cout << "[" << result[i] << "] ";
	//}


	//cout << endl << endl;
	//for (auto it = result.begin() + 1; it != result.end(); ++it)
	//{
	//	cout << *it << endl;
	//}

	//cout << endl << endl;
	//std::sregex_token_iterator end;
	//for (std::sregex_token_iterator it(str.begin(), str.end(), pattern); it != end; ++it)
	//{
	//	cout << "[" << *it << "] ";
	//}

	system("pause");
	return 0;
}

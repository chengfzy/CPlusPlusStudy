#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <set>

using namespace std;

class Quote
{
public:
	Quote() = default;
	Quote(const string& book, double sales_price)
		: bookNo(book), price(sales_price) {}
	
	void add_item(const Quote& sale);
	string isbn() const
	{
		return bookNo;
	}
	virtual double net_price(std::size_t n) const
	{
		return n * price;
	}
	virtual Quote* clone() const &
	{
		return new Quote(*this);
	}
	virtual Quote* clone() &&
	{
		return new Quote(std::move(*this));
	}

	virtual ~Quote() = default;
private:
	string bookNo;
protected:
	double price = 0.0;
};


class DiscQuote :public Quote
{
public:
	DiscQuote() = default;
	DiscQuote(const string& book, double price, size_t qty, double disc)
		: Quote(book, price), quantity(qty), discount(disc) {}
	double net_price(std::size_t) const = 0;
protected:
	size_t quantity = 0;
	double discount = 0.0;
};


class BulkQuote final : public DiscQuote
{
public:
	BulkQuote() = default;
	BulkQuote(const string& book, double p, size_t qty, double disc)
		: DiscQuote(book, p, qty, disc) {}
	double net_price(size_t cnt) const override
	{
		if (cnt >= quantity)
			return cnt * (1 - discount) * price;
		else
			return cnt * price;
	}
	virtual BulkQuote* clone() const&
	{
		return new BulkQuote(*this);
	}
	virtual BulkQuote* clone() &&
	{
		return new BulkQuote(std::move(*this));
	}
};

double print_total(std::ostream& os, const Quote& item, size_t n)
{
	double ret = item.net_price(n);
	os << "ISBN: " << item.isbn() << " # sold: " << n << " total due: " << ret << std::endl;
	return ret;
}


class Basket
{
public:
	void add_item(const std::shared_ptr<Quote>& sale)
	{
		items.insert(sale);
	}
	void add_item(const Quote& sale)
	{
		//items.insert(shared_ptr<Quote>(new Quote(std::move(sale))));	//不会动态绑定
		items.insert(shared_ptr<Quote>(sale.clone()));
	}
	void add_item(Quote&& sale)
	{
		//items.insert(std::shared_ptr<Quote>(new Quote(std::move(sale))));	//不会动态绑定
		items.insert(std::shared_ptr<Quote>(std::move(sale).clone()));
	}
	double total_recipt(std::ostream& os) const
	{
		double sum = 0.0;
		for (auto iter = items.cbegin(); iter != items.cend(); iter = items.upper_bound(*iter))
		{
			sum += print_total(os, **iter, items.count(*iter));
		}
		os << "Total Sale: " << sum << std::endl;
		return sum;
	}

private:
	static bool compare(const std::shared_ptr<Quote>& lhs, const shared_ptr<Quote>& rhs)
	{
		return lhs->isbn() < rhs->isbn();
	}
	std::multiset<std::shared_ptr<Quote>, decltype(compare)*> items{ compare };
};


int main()
{
	Basket bsk;
	//bsk.add_item(std::make_shared<Quote>("CppPrimer", 45));
	//bsk.add_item(std::make_shared<BulkQuote>("EffectiveCpp", 50, 2, 0.15));
	//bsk.add_item(std::make_shared<BulkQuote>("EffectiveCpp", 50, 2, 0.15));
	//bsk.add_item(std::make_shared<BulkQuote>("EffectiveCpp", 50, 2, 0.15));
	
	bsk.add_item(Quote("CppPrimer", 45));
	bsk.add_item(BulkQuote("EffectiveCpp", 50, 2, 0.15));
	bsk.add_item(BulkQuote("EffectiveCpp", 50, 2, 0.15));
	bsk.add_item(BulkQuote("EffectiveCpp", 50, 2, 0.15));

	bsk.total_recipt(std::cout);

	system("pause");
	return 0;
}
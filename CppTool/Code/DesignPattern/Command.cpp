#include <iostream>
#include <list>
#include <memory>


// https://www.cnblogs.com/wzxNote/p/13305597.html
class Chef
{
public:
	static void KungPaoChicken()
	{
		std::cout << "¹¬±£¼¦¶¡" << std::endl;
	}

	static void FishFlavoredShreddedPork()
	{
		std::cout << "ÓãÏãÈâË¿" << std::endl;
	}

	static void BigPlateChicken()
	{
		std::cout << "´óÅÌ¼¦" << std::endl;
	}
};

class Command
{
public:
	virtual ~Command() {}
	virtual void ExcuteCmd() = 0;
};

class FishFlavoredShreddedPorkCmd : public Command
{
public:
	FishFlavoredShreddedPorkCmd() :Command() {}
	void ExcuteCmd() override
	{
		Chef::FishFlavoredShreddedPork();
	}
};

class KungPaoChickenCmd : public Command
{
public:
	KungPaoChickenCmd() :Command() {}
	void ExcuteCmd() override
	{
		Chef::KungPaoChicken();
	}
};

class BigPlateChickenCmd : public Command
{
public:
	BigPlateChickenCmd() :Command() {}
	void ExcuteCmd() override
	{
		Chef::BigPlateChicken();
	}
};


class Waiter
{
public:
	void AddCmd(Command *cmd)
	{
		m_CmdList.push_back(cmd);
	}
	void DelCmd(Command *cmd)
	{
		m_CmdList.remove(cmd);
	}
	void Nodify()
	{
		for (auto cmd : m_CmdList)
		{
			if (cmd)
				cmd->ExcuteCmd();
		}
	}
private:
	std::list<Command*>     m_CmdList;
};


int testCommand()
{
	std::shared_ptr<Chef> chef = std::make_shared<Chef>();
	Waiter waiter;

	std::shared_ptr<FishFlavoredShreddedPorkCmd> ffspc = std::make_shared<FishFlavoredShreddedPorkCmd>();
	std::shared_ptr<KungPaoChickenCmd> kpcc = std::make_shared<KungPaoChickenCmd>();
	std::shared_ptr<BigPlateChickenCmd> bpcc = std::make_shared<BigPlateChickenCmd>();

	waiter.AddCmd(ffspc.get());
	waiter.AddCmd(kpcc.get());
	waiter.AddCmd(bpcc.get());
	//waiter.DelCmd(kpcc.get);

	waiter.Nodify();
	return 0;
}

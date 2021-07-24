#pragma once
#include "TSingleton.hpp"

class People
{
public:
	People(long _id = 0x1234, int _age = 18, const std::string& _name = "Will") :id(_id), age(_age), name(_name)
	{
	}
	void SetAge(int age_)
	{
		age = age_;
	}
	int SayAge() const
	{
		return age;
	}

private:
	long id;
	int age;
	std::string name;
};

using SinglePeople = TSingleton<People>;

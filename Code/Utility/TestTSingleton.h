#pragma once
#include "TSingleton.hpp"

class People
{
public:
	void SetAge(int age_) { age = age_; }
	int SayAge()const { return age; }

private:
	int age;
};

using SinglePeople = TSingleton<People>;

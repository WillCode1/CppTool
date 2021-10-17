#include <memory>
#include "State.h"
using namespace std;

void LifeState::Handle(People * people)
{
	int time = people->Time();
	if (time >= 9 && time < 18)
		people->ChangeState(std::make_shared<WorkState>());
}

void PlayState::Handle(People * people)
{
	int time = people->Time();
	if (time >= 0 && time < 9)
		people->ChangeState(std::make_shared<LifeState>());
}

void WorkState::Handle(People * people)
{
	int time = people->Time();
	if (time >= 18 && time < 24)
		people->ChangeState(std::make_shared<PlayState>());
}

int testState()
{
	std::shared_ptr<State> state1 = std::make_shared<WorkState>();
	People p(state1);

	p.Show();
	p.SetTime(9);
	p.Show();
	p.SetTime(11);
	p.Show();
	p.SetTime(13);
	p.Show();
	p.SetTime(20);
	p.Show();
	p.SetTime(22);
	p.Show();
	return 0;
}

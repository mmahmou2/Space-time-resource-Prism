#pragma once
#include "stdafx.h"
using namespace std;

class CAgent
{
public:
	CAgent();
	int agent_id;
	double from_x;
	double from_y;
	float departure_time_in_min;
	double to_x;
	double to_y;
	std::vector<int> m_PathNodeIDs;
	std::vector<float> m_PathCost;
};

class CCostofArc;

class CDemands
{
public:
	CDemands();
	~CDemands();
	int m_agent_number;
	std::vector<CAgent> m_agents;
	int m_eligible_agent_number;
	std::vector<CAgent*> m_eligible_agents;
	int* m_agent_eligible_status;
	void read_agents();
	void regenerate_agents_demands(CCostofArc** arcNeedUpdate)


};


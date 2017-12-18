#pragma once

#include <thread>
#include <mutex>
#include"network.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


using namespace std;

class CAgent;

class CCostofArc
{
public:
	CCostofArc();
	CArc* m_pArc;
	int m_number_of_passenger;
	int* m_passenger_index;
	float m_miu;
	float m_cost;
	~CCostofArc();
};

class CShortestPath  // mainly for shortest path calculation, not just physical network
{
private:
	int m_number_of_nodes;
public:
	CShortestPath();
	~CShortestPath();
	
	std::mutex m_mutex;
protected:
#pragma region Scan Eligible List
	int m_ListFront;
	int m_ListTail;
	int* m_SENodeList;
	// SEList: scan eligible List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_ListFront = -1;
		m_ListTail = -1;
	}
	void SEList_push_front(int node)
	{
		if (m_ListFront == -1)  // start from empty
		{
			m_SENodeList[node] = -1;
			m_ListFront = node;
			m_ListTail = node;
		}
		else
		{
			m_SENodeList[node] = m_ListFront;
			m_ListFront = node;
		}
	}
	void SEList_push_back(int node)
	{
		if (m_ListFront == -1)  // start from empty
		{
			m_ListFront = node;
			m_ListTail = node;
			m_SENodeList[node] = -1;
		}
		else
		{
			m_SENodeList[m_ListTail] = node;
			m_SENodeList[node] = -1;
			m_ListTail = node;
		}
	}
	bool SEList_empty()
	{
		return(m_ListFront == -1);
	}
	int SEList_front()
	{
		return m_ListFront;
	}
	void SEList_pop_front()
	{
		int tempFront = m_ListFront;
		m_ListFront = m_SENodeList[m_ListFront];
		m_SENodeList[tempFront] = -1;
	}

#pragma endregion
	//agent's origin vertex
	CVertex m_origin_vertex;

	float** m_node_label_cost;
	int** m_node_predecessor;
	int** m_node_predecessor_n;
	int* m_node_status_array;

	int* m_departure_stop_vertex;
	int* m_dest_stops_id;
	//the minutes it takes from destination stop to agent's destination by walking. the key is stop id and the value is time
	float* m_dest_walking_cost;


	int m_best_destination_vertex_no;
	int m_best_transfer_times;
public:
	CCostofArc* m_ArcCosts;

	void AllocateMemory(int number_of_nodes);

	bool UpdateState(int number_of_iteration);

	int pre_label_correcting(CAgent* pAgent);

	int optimal_label_correcting(float ending_time);

	void find_path_for_agent(int number_of_threads, int id);

	void WritePassengerLoaded();
};

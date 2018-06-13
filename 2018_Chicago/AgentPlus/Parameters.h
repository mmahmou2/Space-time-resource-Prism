#pragma once
#include "CSVParser.h"

class CParameters
{
public:
	CParameters();
	~CParameters();
	static int m_number_of_threads;
	static int m_shortest_path_debugging_flag;
	static double m_max_transfer_distance;//unit: mile, the max tolerated straight distance for a transfer  
	static int m_max_Transfer_time; //unit:minute
	static double m_avg_walking_speed; //unit: mile/minute
	static int m_max_Transfer_count;
	static int m_max_travel_time; //unit: minute
	static int m_max_number_of_vertex_nearby_agent_OD;
	static int m_max_iteration;
	static void init_parameters();
	static void read_parameters();
private:

};

CParameters::CParameters()
{
}

CParameters::~CParameters()
{
}
void CParameters::init_parameters()
{
	m_number_of_threads = 1;
	m_shortest_path_debugging_flag = 0;
	m_max_transfer_distance = 0.5;//unit: mile, the max tolerated straight distance for a transfer  
	m_max_Transfer_time = 30; //unit:minute
	m_max_Transfer_count = 3;
	m_avg_walking_speed = 0.5; //unit: mile/minute
	m_max_travel_time = 120;
	m_max_number_of_vertex_nearby_agent_OD = 100;
	m_max_iteration = 1;
}
void CParameters::read_parameters()
{
	CCSVParser parser;

	if (parser.OpenCSVFile("Agent_plus_config.csv", true))
	{
		int value;
		string key;
		while (parser.ReadRecord())
		{
			if (parser.GetValueByFieldName("key", key) == false)
				continue;
			if (key == "number_of_thread")
			{
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_number_of_threads = value;
			}
			else if (key == "max_transfer_count")
			{
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_max_Transfer_count = value;
			}
			else if (key == "max_transfer_distance")
			{
				float fvalue = 0;
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_max_transfer_distance = value;
			}
			else if (key == "max_Transfer_time")
			{
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_max_Transfer_time = value;
			}
			else if (key == "max_travel_time")
			{
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_max_travel_time = value;
			}
			else if (key == "max_iteration")
			{
				if (parser.GetValueByFieldName("value", value) == false)
					continue;
				m_max_iteration = value;

			}

		}
		parser.CloseCSVFile();
	}

}
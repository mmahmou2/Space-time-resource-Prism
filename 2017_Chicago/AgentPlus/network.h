#pragma once


using namespace std;
#include "stdafx.h"

class CArc
{
public:
	CArc()
	{
		index_in_all_trip_segment = -1;
	}
	int arc_id;
	int to_vertex_no;
	int from_vertex_no;
	float cost;
	int type; //0=trip segment, 1= transfer arc 2 = waiting arc
	int passenger_number;
	int capacity;
	std::vector<int> passenger_id;
	int index_in_all_trip_segment;
};
class CVertex;

class CStop
{
public:
	int stop_id;
	double x;
	double y;
	string name;
	string original_id;
	vector<CVertex*> vertexes;
};
class CRoute
{
public:
	int route_id;
	string original_route_id;
	string short_name;
};
class CTrip
{
public:
	int trip_id;
	string original_trip_id;
	int route_id;
	string trip_head_sign;
	int capacity;
	CRoute* RouteObj;
	vector<CArc*> arcs;
	int number_of_passenger;
	vector<int> onboarding_passengers_index;
};

class CVertex
{
public:
	CVertex();

	int vertex_id;
	int stop_id;
	CStop* StopObj;
	CTrip* TripObj;
	float time_in_min;
	int time_in_sec;
	int	trip_id;
	int type;
	double x;
	double y;

	std::vector<CArc> m_outgoing_vertex_vector;
	CArc* getArc(int to_vertex_id);
};

class grid3D
{
public:
	std::vector<int> vertexes_in_grid3D;

public:

	void add_vertex(int vertex_no)
	{
		vertexes_in_grid3D.push_back(vertex_no);
	}
};

class grid2D
{
public:
	std::vector<int> stops_in_grid2D;
	std::vector<double> stop_x;
	std::vector<double> stop_y;
	std::map<int, int> stopid_map;
public:
	void add_stopid(int stopid, double x, double y);
};

class Network
{
public:
	Network();
	~Network();
	//stop set
	std::map<int, CStop> m_stop_map;
	//trip set
	std::map<int, CTrip> m_trip_map;
	//route set
	std::map<int, CRoute> m_route_map;
	int m_number_of_nodes;
	int m_number_of_links;
	std::map<int, int> m_internal_node_no_map;
	std::map<int, int> m_external_node_id_map;
	//vertex set
	std::vector<CVertex> m_vertex_vector;
	int m_number_of_trip_segment_arc;
	void read_network();
protected:
	//read vertexes and arcs from a binary file
	int read_BinaryFile();
	//dump all vertexes and arces to a binary file to speed up the data loading process
	int write_BinaryFile();

#pragma region Grids
protected:
	//The number of rows of the grided network
	int m_grid3D_rows;
	//The number of columes of the grided network
	int m_grid3D_cols;
	//Spacial-temporay grid set for the network
	std::map<string, grid3D*> m_grid3D_map;
	//Spacial grid set for the network
	std::map<string, grid2D*> m_grid2D_map;
	//The following 4 members are used to mark the minimum bounding rectangle of the network.
	double m_area_min_x;
	double m_area_min_y;
	double m_area_max_x;
	double m_area_max_y;

	void update_bounding_rectangle(double x, double y);
	int make_grid2D();
	int make_grid3D();	
	void free_grids_memory();
public:
	int find_near_vertexes(double x, double y, float t, int* pRslt, int length_of_pRslt, bool bLog = false);
	int find_near_stopid(double x, double y, int* pRslt, float* pCosts, int length_of_rslt_array, bool bLog = false);
#pragma endregion

};
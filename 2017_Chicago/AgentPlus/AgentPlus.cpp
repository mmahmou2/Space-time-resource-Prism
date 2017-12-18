// Forward and backward dynamic programming for finding the shortest path in a space-time-state network

#include "stdafx.h"
#include "windows.h"
#include <iostream>
#include <vector>
#include "CSVParser.h"
using namespace std;

FILE* g_pFileDebugLog = NULL;
FILE* g_pFilenodes_for_plotLog = NULL;
FILE* g_pFileprism_for_plotLog = NULL;
FILE* g_pFilelinks_for_plotLog = NULL;
FILE* g_pFileSTDebugLog = NULL;
FILE* g_pFileinput_vertexLog = NULL;
FILE* g_pFileinput_arcLog = NULL;
FILE* g_pFileinput_STvertexLog = NULL;
FILE* g_pFileinput_STarcLog = NULL;

void g_ProgramStop()
{
	cout << "Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};

template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}



class CVertex
{
public:
	int vertex_index;
	int vertex_time;
	int vertex_node;
	int vertex_state;
	int vertex_field;
	double label;
	int pred;
	int succ;
};

class CArc
{
public:
	int arc_index;
	int arc_from_vertex;
	int arc_to_vertex;
	int arc_from_time;
	int arc_to_time;
	int arc_from_node;
	int arc_to_node;
	int arc_from_state;
	int arc_to_state;
	double arc_cost;
};

class CSTVertex
{
public:
	int STvertex_index;
	int STvertex_time;
	int STvertex_node;
	int STvertex_field;
	double STlabel;
	int STpred;
	int STsucc;
};

class CSTArc
{
public:
	int STarc_index;
	int STarc_from_vertex;
	int STarc_to_vertex;
	int STarc_from_time;
	int STarc_to_time;
	int STarc_from_node;
	int STarc_to_node;
	double STarc_cost;
};

class CNode
{
public:
	int node_id;
	int field;
	double x_coordinate;
	double y_coordinate;
};

class CLink
{
public:
	int link_id;
	int from_node_id;
	int to_node_id;
	int direction;
	int length;
	int number_of_lanes;
	int speed_limit;
	int lane_cap;
	int link_type;
	int jam_density;
	int wave_speed;
	int BPR_alpha_term;
	int BPR_beta_term;
	int free_flow_time;
	int toll;
	int volume;
	double cost;
};


//define parameters
const int max_vertex_no = 5000000;
const int max_STvertex_no = 300000;
const int max_time_no = 122;	// 1 time unit more for sink node
const int max_node_no = 935;  // number of nodes (i.e. 6) + sink node + 1
const int max_state_no = 1002;  // energy
const int max_link_no = 2967;
const double max_value = 99999.0;
const double max_tolerant_cost = 1.0;
const double min_fuel = 0.0;

// for SST network

int g_GetSpaceTimeKey(int node_no, int time_no)
{
	return node_no*1000 + time_no;  // 1000 is the total number of time points
}


int g_GetSpaceTimeStateKey(int node_no, int time_no, int state)
{
	return node_no * 1000*1000 + time_no*1000 + state;  // 1000 is the total number of time points
}

std::map<int, int> g_key_to_vertex_index_map;
std::map<int, int> g_STSkey_to_vertex_index_map;

int g_GetVertexIndex(int node_no, int time_no)
{
	int key = g_GetSpaceTimeKey(node_no, time_no);
	if (g_key_to_vertex_index_map.find(key) != g_key_to_vertex_index_map.end())
		return g_key_to_vertex_index_map[key];
	else
		return -1;

}

int g_GetSTSVertexIndex(int node_no, int time_no, int state)
{
	int key = g_GetSpaceTimeStateKey(node_no, time_no, state);
	if (g_STSkey_to_vertex_index_map.find(key) != g_STSkey_to_vertex_index_map.end())
		return g_STSkey_to_vertex_index_map[key];
	else
		return -1;

}

//int time_space_state_index[max_time_no][max_node_no][max_state_no];
//double forward_time_space_state_label[max_node_no][max_time_no][max_state_no];  //forward space-time-state label
//double backward_time_space_state_label[max_node_no][max_time_no][max_state_no];  //forward space-time-state label	
//int time_space_state_prism_flag[max_node_no][max_time_no][max_state_no];  //if both forward and backward SST label are feasible or less than some budget cost

int ***time_space_state_index = NULL;
double ***forward_time_space_state_label = NULL;  //forward space-time-state label
double ***backward_time_space_state_label = NULL;  //forward space-time-state label	
int ***time_space_state_prism_flag = NULL;  //if both forward and backward SST label are feasible or less than some budget cost

// for ST network
int time_space_index[max_time_no][max_node_no];
double forward_time_space_label[max_node_no][max_time_no];  //forward space-time label
double backward_time_space_label[max_node_no][max_time_no];  //forward space-time label	
int space_prism_flag[max_node_no];  //if both forward and backward ST label are feasible or less than some budget cost
int space_time_flag[max_node_no][max_time_no];

int recharging_station_node_index[max_node_no];

// for SST network
int initial_vertex1_index;  // origin vertex
int initial_vertex2_index;	// sink vertex


// for SST network
int sink_vertex_index;
int sink_vertex_node;
int sink_vertex_time;
int sink_vertex_state;

// for ST network
int STinitial_vertex1_index;  // origin vertex
int STinitial_vertex2_index;	// sink vertex

// for ST network
int STsink_vertex_index;
int STsink_vertex_node;
int STsink_vertex_time;

// for SST network
// Adjacent_outgoing_vertexes matrix is constructed to store all adjacent outgoing vertexes of each vertex + 
// store travel costs imposed by traveling from each vertex to the corresponding adjacent vertexes
struct SAdjacent_outgoing_vertexes
{
	int adjacent_outgoing_vertex_index;
	double adjacent_outgoing_vertex_travel_cost;
};

SAdjacent_outgoing_vertexes temp_1;

// Adjacent_ingoing_vertexes matrix is constructed to store all adjacent ingoing vertexes of each vertex + 
// store travel costs imposed by traveling from the corresponding adjacent vertexes to that particular vertex.
struct SAdjacent_ingoing_vertexes
{
	int adjacent_ingoing_vertex_index;
	double adjacent_ingoing_vertex_travel_cost;
};

SAdjacent_ingoing_vertexes temp_2;


// for ST network
// Adjacent_outgoing_vertexes matrix is constructed to store all adjacent outgoing ST vertexes of each vertex + 
// store travel costs imposed by traveling from each vertex to the corresponding adjacent vertexes
struct STAdjacent_outgoing_vertexes
{
	int STadjacent_outgoing_vertex_index;
	double STadjacent_outgoing_vertex_travel_cost;
};

STAdjacent_outgoing_vertexes STtemp_1;

// Adjacent_ingoing_vertexes matrix is constructed to store all adjacent ingoing vertexes of each vertex + 
// store travel costs imposed by traveling from the corresponding adjacent vertexes to that particular vertex.
struct STAdjacent_ingoing_vertexes
{
	int STadjacent_ingoing_vertex_index;
	double STadjacent_ingoing_vertex_travel_cost;
};

STAdjacent_ingoing_vertexes STtemp_2;

// for SST network
//define CVertex, CArc, SAdjacent_outgoing_vertexes, SAdjacent_ingoing_vertexes, and time_space_state_index
vector<CVertex> vertex1_vector;
vector<CVertex> vertex2_vector;
vector<CArc> arc1_vector;
vector<CArc> arc2_vector;
//vector<SAdjacent_outgoing_vertexes> adjacent_outgoing_vertexes_vector[max_vertex_no];
//vector<SAdjacent_ingoing_vertexes> adjacent_ingoing_vertexes_vector[max_vertex_no];

vector<SAdjacent_outgoing_vertexes>* adjacent_outgoing_vertexes_vector;
vector<SAdjacent_ingoing_vertexes>* adjacent_ingoing_vertexes_vector;

// for ST network
//define CSTVertex, CSTArc, STAdjacent_outgoing_vertexes, STAdjacent_ingoing_vertexes, and time_space_index
vector<CSTVertex> STvertex1_vector;
vector<CSTVertex> STvertex2_vector;
vector<CSTArc> STarc1_vector;
vector<CSTArc> STarc2_vector;
vector<STAdjacent_outgoing_vertexes> STadjacent_outgoing_vertexes_vector[max_vertex_no];
vector<STAdjacent_ingoing_vertexes> STadjacent_ingoing_vertexes_vector[max_vertex_no];

vector<CNode> node_vector;

// read input_node and input_link and convert them to input_vertex and input_arc
void g_read_input_data()
{
	fprintf(g_pFilenodes_for_plotLog, "x, y, t, node_id\n");
	fprintf(g_pFileprism_for_plotLog, "x, y, count, node_id, geometry\n");
	fprintf(g_pFilelinks_for_plotLog, "x, y, node_id\n");
	//Open input_node.csv file
	vector<CSTVertex> STvertex_vector;


	CCSVParser parser_node;
	if (parser_node.OpenCSVFile("input_node.csv", true))
	{
		int number_of_nodes = 0;
		while (parser_node.ReadRecord())
		{
			int node_id;
			int field;
			double x_coordinate;
			double y_coordinate;
			parser_node.GetValueByFieldName("node_id", node_id);
			parser_node.GetValueByFieldName("field", field);
			parser_node.GetValueByFieldName("x", x_coordinate, false);
			parser_node.GetValueByFieldName("y", y_coordinate, false);

			CNode node;
			node.node_id = node_id;
			node.field = field;		// field is 1 if it is origin, is 2 if destination, and 0 otherwise.
			node.x_coordinate = x_coordinate;
			node.y_coordinate = y_coordinate;
			fprintf(g_pFilenodes_for_plotLog, "%f, %f, %d, %d,\n",
				node.x_coordinate,
				node.y_coordinate,
				0,
				node.node_id);
			node_vector.push_back(node);

			number_of_nodes++;
		}
		parser_node.CloseCSVFile();
	};






	//create input_STvertex.csv
	g_pFileinput_STvertexLog = fopen("input_STvertex.csv", "w");

	int STindex = 0;
	int STn = 0;
	int res_rechg_no = 0;

	fprintf(g_pFileinput_STvertexLog, "index, time, node, field, \n");

	for (int n = 0; n < max_node_no - 1; n++)
	{
		int repeat = 0;
		for (int tt = 0; tt < max_time_no - 1; tt++)
		{
			if (n < max_node_no - 2 && tt < max_time_no - 2)
			{

				CSTVertex STvertex;
				STvertex.STvertex_index = STindex;
				STvertex.STvertex_node = n;
				STvertex.STvertex_time = tt;
				STvertex.STvertex_field = 0;

				if (STvertex.STvertex_time == 0 && node_vector[n].field == 1)
				{
					STvertex.STvertex_field = 1;
					STinitial_vertex1_index = STvertex.STvertex_index;
				}

				if (STvertex.STvertex_time == max_time_no - 3 && node_vector[n].field == 2)
				{
					STvertex.STvertex_field = 2;
				}



				if (node_vector[n].field == 4)
				{
					STvertex.STvertex_field = 4;
					if (repeat == 0)
					{
						recharging_station_node_index[res_rechg_no] = n;
						res_rechg_no++;
						repeat++;
					}
				}

				STvertex.STlabel = max_value;
				STvertex.STpred = -1;  // default value
				STvertex.STsucc = -1;  // default value
				STvertex_vector.push_back(STvertex);
				STvertex1_vector.push_back(STvertex);
				STvertex2_vector.push_back(STvertex);
				time_space_index[tt][n] = STindex;

				int key = g_GetSpaceTimeKey(n, tt);
				g_key_to_vertex_index_map[key] = STindex;  // story the key to vertex index mapping 
				STn++;
				STindex++;
				fprintf(g_pFileinput_STvertexLog, "%d, %d, %d, %d\n",
					STvertex.STvertex_index,
					STvertex.STvertex_time,
					STvertex.STvertex_node,
					STvertex.STvertex_field);
			}

			if (n == max_node_no - 2 && tt == max_time_no - 2)
			{
				CSTVertex STvertex;
				STvertex.STvertex_index = STindex;
				STvertex.STvertex_node = n;
				STvertex.STvertex_time = tt;
				STvertex.STvertex_field = 3;  // field is 3 if it is sink vertex
				STvertex.STlabel = max_value;
				STvertex.STpred = -1;  // default value
				STvertex.STsucc = -1;  // default value
				STvertex_vector.push_back(STvertex);
				STvertex1_vector.push_back(STvertex);
				STvertex2_vector.push_back(STvertex);
				time_space_index[STvertex.STvertex_time][STvertex.STvertex_node] = STindex;
				fprintf(g_pFileinput_STvertexLog, "%d, %d, %d, %d\n",
					STvertex.STvertex_index,
					STvertex.STvertex_time,
					STvertex.STvertex_node,
					STvertex.STvertex_field);

				STsink_vertex_index = STindex;
				STinitial_vertex2_index = STindex;
				STsink_vertex_node = n;
				STsink_vertex_time = tt;
				STn++;
				STindex++;
			}
		}
	}

	fclose(g_pFileinput_STvertexLog);


	//create input_STarc.csv
	g_pFileinput_STarcLog = fopen("input_STarc.csv", "w");

	vector<CSTArc> arc_STvector;
	vector<CLink> link_vector;

	fprintf(g_pFileinput_STarcLog, "index, from_vertex, to_vertex, from_time, to_time, from_node, to_node, cost, \n");

	// read input_link.csv
	CCSVParser parser_link;
	if (parser_link.OpenCSVFile("input_link.csv", true))
	{
		int number_of_links = 0;
		while (parser_link.ReadRecord())
		{
			int link_id;
			int from_node_id;
			int to_node_id;
			int direction;
			int length;
			int number_of_lanes;
			int speed_limit;
			int lane_cap;
			int link_type;
			int jam_density;
			int wave_speed;
			int BPR_alpha_term;
			int BPR_beta_term;
			int free_flow_time;
			int toll;
			int volume;
			double cost;

			parser_link.GetValueByFieldName("link_id", link_id);
			parser_link.GetValueByFieldName("from_node_id", from_node_id);
			parser_link.GetValueByFieldName("to_node_id", to_node_id);
			parser_link.GetValueByFieldName("direction", direction);
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("number_of_lanes", number_of_lanes);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);
			parser_link.GetValueByFieldName("link_type", link_type);
			parser_link.GetValueByFieldName("jam_density", jam_density);
			parser_link.GetValueByFieldName("wave_speed", wave_speed);
			parser_link.GetValueByFieldName("BPR_alpha_term", BPR_alpha_term);
			parser_link.GetValueByFieldName("BPR_beta_term", BPR_beta_term);
			parser_link.GetValueByFieldName("free_flow_time", free_flow_time);
			parser_link.GetValueByFieldName("toll", toll);
			parser_link.GetValueByFieldName("volume", volume);
			parser_link.GetValueByFieldName("cost", cost);

			CLink link;
			link.link_id = link_id;
			link.from_node_id = from_node_id;
			link.to_node_id = to_node_id;
			link.direction = direction;
			link.length = length;
			link.number_of_lanes = number_of_lanes;
			link.speed_limit = speed_limit;
			link.lane_cap = lane_cap;
			link.link_type = link_type;
			link.jam_density = jam_density;
			link.wave_speed = wave_speed;
			link.BPR_alpha_term = BPR_alpha_term;
			link.BPR_beta_term = BPR_beta_term;
			link.free_flow_time = free_flow_time;
			link.toll = toll;
			link.volume = volume;
			link.cost = cost;
			link_vector.push_back(link);
			number_of_links++;
		}

		parser_link.CloseCSVFile();
	};

	for (int l = 0; l < max_link_no - 1; l++)
	{
		fprintf(g_pFilelinks_for_plotLog, "%.6f, %.6f, %d, %.6f, %.6f, %d, \n",
			node_vector[link_vector[l].from_node_id].x_coordinate,
			node_vector[link_vector[l].from_node_id].y_coordinate,
			0,
			node_vector[link_vector[l].to_node_id].x_coordinate - node_vector[link_vector[l].from_node_id].x_coordinate,
			node_vector[link_vector[l].to_node_id].y_coordinate - node_vector[link_vector[l].from_node_id].y_coordinate,
			0);
	}
	for (int l = 0; l < max_link_no - 1; l++)
	{
		fprintf(g_pFilenodes_for_plotLog, "%.6f, %.6f, %d, \n",
			node_vector[link_vector[l].from_node_id].x_coordinate,
			node_vector[link_vector[l].from_node_id].y_coordinate,
			node_vector[link_vector[l].from_node_id].node_id);
		fprintf(g_pFilenodes_for_plotLog, "%.6f, %.6f, %d, \n\n",
			node_vector[link_vector[l].to_node_id].x_coordinate,
			node_vector[link_vector[l].to_node_id].y_coordinate,
			node_vector[link_vector[l].to_node_id].node_id);
	}


	// create ST transportation arcs
	int travel_time[max_link_no] = { 0 };
	int STindex_1 = 0;

	for (int l = 0; l < max_link_no - 1; l++)
	{


		travel_time[l] = link_vector[l].free_flow_time;

		for (int tt = 0; tt < max_time_no - 3; tt++)
		{

			if (tt < max_time_no - 2 - travel_time[l])
			{

				CSTArc STarc;
				STarc.STarc_index = STindex_1;
				STarc.STarc_cost = travel_time[l];
				STarc.STarc_from_node = link_vector[l].from_node_id;
				STarc.STarc_to_node = link_vector[l].to_node_id;
				STarc.STarc_from_time = tt;
				STarc.STarc_to_time = tt + travel_time[l];


				STarc.STarc_from_vertex = g_GetVertexIndex(STarc.STarc_from_node, STarc.STarc_from_time);
				STarc.STarc_to_vertex = g_GetVertexIndex(STarc.STarc_to_node, STarc.STarc_to_time);


				STarc1_vector.push_back(STarc);
				STarc2_vector.push_back(STarc);

				STtemp_1.STadjacent_outgoing_vertex_index = STarc.STarc_to_vertex;
				STtemp_1.STadjacent_outgoing_vertex_travel_cost = STarc.STarc_cost;

				STadjacent_outgoing_vertexes_vector[STarc.STarc_from_vertex].push_back(STtemp_1);



				STtemp_2.STadjacent_ingoing_vertex_index = STarc.STarc_from_vertex;
				STtemp_2.STadjacent_ingoing_vertex_travel_cost = STarc.STarc_cost;

				STadjacent_ingoing_vertexes_vector[STarc.STarc_to_vertex].push_back(STtemp_2);

				fprintf(g_pFileinput_STarcLog, "%d, %d, %d, %d, %d, %d, %d, %.3f,\n",
					STarc.STarc_index,
					STarc.STarc_from_vertex,
					STarc.STarc_to_vertex,
					STarc.STarc_from_time,
					STarc.STarc_to_time,
					STarc.STarc_from_node,
					STarc.STarc_to_node,
					STarc.STarc_cost);
				cout << "..." << endl;
				STindex_1++;
			}
		}
	}


	// create waiting arcs
	for (int x = 0; x < max_node_no - 2; x++)
	{
		for (int t = 0; t < max_time_no - 3; t++)
		{

			CSTArc STarc;
			STarc.STarc_index = STindex_1;
			STarc.STarc_from_time = t;
			STarc.STarc_to_time = t + 1;
			STarc.STarc_from_node = x;
			STarc.STarc_to_node = x;
			STarc.STarc_cost = 0.0;

			STarc.STarc_from_vertex = g_GetVertexIndex(STarc.STarc_from_node, STarc.STarc_from_time);
			STarc.STarc_to_vertex = g_GetVertexIndex(STarc.STarc_to_node, STarc.STarc_to_time);


			if (STarc.STarc_from_vertex >= 0 && STarc.STarc_to_vertex >= 0)
			{


				STarc1_vector.push_back(STarc);
				STarc2_vector.push_back(STarc);

				STtemp_1.STadjacent_outgoing_vertex_index = STarc.STarc_to_vertex;
				STtemp_1.STadjacent_outgoing_vertex_travel_cost = STarc.STarc_cost;

				STadjacent_outgoing_vertexes_vector[STarc.STarc_from_vertex].push_back(STtemp_1);



				STtemp_2.STadjacent_ingoing_vertex_index = STarc.STarc_from_vertex;
				STtemp_2.STadjacent_ingoing_vertex_travel_cost = STarc.STarc_cost;

				STadjacent_ingoing_vertexes_vector[STarc.STarc_to_vertex].push_back(STtemp_2);

				fprintf(g_pFileinput_STarcLog, "%d, %d, %d, %d, %d, %d, %d, %.3f,\n",
					STarc.STarc_index,
					STarc.STarc_from_vertex,
					STarc.STarc_to_vertex,
					STarc.STarc_from_time,
					STarc.STarc_to_time,
					STarc.STarc_from_node,
					STarc.STarc_to_node,
					STarc.STarc_cost);

				if(STindex_1%100 ==0)
				{
				cout << "..." << endl;
				}

				STindex_1++;
				}
		}

	}

	// create arcs from destination vertexes to the sink vertex
	for (int l = 0; l < STn; l++)
	{
		if (STvertex_vector[l].STvertex_field == 2)
		{
			CSTArc STarc;
			STarc.STarc_index = STindex_1;
			STarc.STarc_from_time = STvertex_vector[l].STvertex_time;
			STarc.STarc_from_node = STvertex_vector[l].STvertex_node;

			STarc.STarc_to_time = STsink_vertex_time;
			STarc.STarc_to_node = STsink_vertex_node;

			STarc.STarc_cost = 0.0;
			STarc.STarc_from_vertex = STvertex_vector[l].STvertex_index;
			STarc.STarc_to_vertex = STsink_vertex_index;
			STarc1_vector.push_back(STarc);
			STarc2_vector.push_back(STarc);

			STtemp_1.STadjacent_outgoing_vertex_index = STarc.STarc_to_vertex;
			STtemp_1.STadjacent_outgoing_vertex_travel_cost = STarc.STarc_cost;

			STadjacent_outgoing_vertexes_vector[STarc.STarc_from_vertex].push_back(STtemp_1);


			STtemp_2.STadjacent_ingoing_vertex_index = STarc.STarc_from_vertex;
			STtemp_2.STadjacent_ingoing_vertex_travel_cost = STarc.STarc_cost;

			STadjacent_ingoing_vertexes_vector[STarc.STarc_to_vertex].push_back(STtemp_2);

			fprintf(g_pFileinput_STarcLog, "%d, %d, %d, %d, %d, %d, %d, %.3f,\n",
				STarc.STarc_index,
				STarc.STarc_from_vertex,
				STarc.STarc_to_vertex,
				STarc.STarc_from_time,
				STarc.STarc_to_time,
				STarc.STarc_from_node,
				STarc.STarc_to_node,
				STarc.STarc_cost);
			cout << "..." << endl;
			STindex_1++;
		}
	}
	fclose(g_pFileinput_STarcLog);

	g_pFileSTDebugLog = fopen("STDebug.txt", "w");
	if (g_pFileSTDebugLog == NULL)
	{
		cout << "File STDebug.txt cannot be opened." << endl;
	}

	// forward DP

	fprintf(g_pFileSTDebugLog, "Forward dynamic programming\n");

	//define an initial origin vertex for forward dynamic programming
	forward_time_space_label[STvertex1_vector[STinitial_vertex1_index].STvertex_node][STvertex1_vector[STinitial_vertex1_index].STvertex_time] = 0;
	STvertex1_vector[STinitial_vertex1_index].STlabel = 0;
	STvertex1_vector[STinitial_vertex1_index].STpred = 0;

	//forward dynamic programming for finding shortest path in a space-time network
	int xx = 0;

	for (int t = 0; t < max_time_no - 1; t++) // for time
	{
		for (int n = 0; n < max_node_no - 1; n++) // for node
		{

			if (time_space_index[t][n] == STinitial_vertex1_index && xx == 0)
			{
				xx = 1;
			}

			if (time_space_index[t][n] > -1 && xx == 1)
			{
				int STfrom_vertex_index = time_space_index[t][n];

				if (STfrom_vertex_index == -1 || STadjacent_outgoing_vertexes_vector[STfrom_vertex_index].size() == 0) continue;

				for (int temp_adjacent_outgoing_vertex_index = 0; temp_adjacent_outgoing_vertex_index <= STadjacent_outgoing_vertexes_vector[STfrom_vertex_index].size() - 1; temp_adjacent_outgoing_vertex_index++)
				{
					int STadjacent_outgoing_vertex_index = STadjacent_outgoing_vertexes_vector[STfrom_vertex_index][temp_adjacent_outgoing_vertex_index].STadjacent_outgoing_vertex_index;

					double temp_label_cost = STvertex1_vector[STfrom_vertex_index].STlabel + STadjacent_outgoing_vertexes_vector[STfrom_vertex_index][temp_adjacent_outgoing_vertex_index].STadjacent_outgoing_vertex_travel_cost;

					if (temp_label_cost < STvertex1_vector[STadjacent_outgoing_vertex_index].STlabel)
					{
						forward_time_space_label[STvertex1_vector[STadjacent_outgoing_vertex_index].STvertex_node][STvertex1_vector[STadjacent_outgoing_vertex_index].STvertex_time] = temp_label_cost;
						STvertex1_vector[STadjacent_outgoing_vertex_index].STlabel = temp_label_cost;
						STvertex1_vector[STadjacent_outgoing_vertex_index].STpred = STfrom_vertex_index;
						fprintf(g_pFileSTDebugLog, "the label of vertex %d (node %d at time %d) is updated to %.3f.\n",
							STadjacent_outgoing_vertex_index,
							STvertex1_vector[STadjacent_outgoing_vertex_index].STvertex_node,
							STvertex1_vector[STadjacent_outgoing_vertex_index].STvertex_time,
							STvertex1_vector[STadjacent_outgoing_vertex_index].STlabel);
						fprintf(g_pFileSTDebugLog, "the predecessor of vertex %d is vertex %d.\n\n", STadjacent_outgoing_vertex_index, STvertex1_vector[STadjacent_outgoing_vertex_index].STpred);
					}
				}
			}
		}
	}

	// backward DP
	fprintf(g_pFileSTDebugLog, "\n\n\n\nBackward dynamic programming\n");

	//define an initial destination vertex for backward dynamic programming
	backward_time_space_label[STvertex2_vector[STinitial_vertex2_index].STvertex_node][STvertex2_vector[STinitial_vertex2_index].STvertex_time] = 0;
	STvertex2_vector[STinitial_vertex2_index].STlabel = 0;
	STvertex2_vector[STinitial_vertex2_index].STsucc = 0;

	//backward dynamic programming for finding shortest path in a space-time-state network
	int y = 0;
	for (int t = max_time_no - 2; t > -1; t--) // for time
	{
		for (int n = 0; n < max_node_no - 1; n++) // for node
		{

			if (time_space_index[t][n] == STinitial_vertex2_index && y == 0)
			{
				y = 1;
			}

			if (time_space_index[t][n] > -1 && y == 1)
			{
				int STto_vertex_index = time_space_index[t][n];

				if (STto_vertex_index == -1 || STadjacent_ingoing_vertexes_vector[STto_vertex_index].size() == 0) continue;

				for (int temp_adjacent_ingoing_vertex_index = 0; temp_adjacent_ingoing_vertex_index <= STadjacent_ingoing_vertexes_vector[STto_vertex_index].size() - 1; temp_adjacent_ingoing_vertex_index++)
				{
					int STadjacent_ingoing_vertex_index = STadjacent_ingoing_vertexes_vector[STto_vertex_index][temp_adjacent_ingoing_vertex_index].STadjacent_ingoing_vertex_index;

					double temp_label_cost = STvertex2_vector[STto_vertex_index].STlabel + STadjacent_ingoing_vertexes_vector[STto_vertex_index][temp_adjacent_ingoing_vertex_index].STadjacent_ingoing_vertex_travel_cost;

					if (temp_label_cost < STvertex2_vector[STadjacent_ingoing_vertex_index].STlabel)
					{
						backward_time_space_label[STvertex2_vector[STadjacent_ingoing_vertex_index].STvertex_node][STvertex2_vector[STadjacent_ingoing_vertex_index].STvertex_time] = temp_label_cost;
						STvertex2_vector[STadjacent_ingoing_vertex_index].STlabel = temp_label_cost;
						STvertex2_vector[STadjacent_ingoing_vertex_index].STsucc = STto_vertex_index;
						fprintf(g_pFileSTDebugLog, "the label of vertex %d (node %d at time %d) is updated to %.3f.\n",
							STadjacent_ingoing_vertex_index,
							STvertex2_vector[STadjacent_ingoing_vertex_index].STvertex_node,
							STvertex2_vector[STadjacent_ingoing_vertex_index].STvertex_time,
							STvertex2_vector[STadjacent_ingoing_vertex_index].STlabel);
						fprintf(g_pFileSTDebugLog, "the successor of vertex %d is vertex %d.\n\n", STadjacent_ingoing_vertex_index, STvertex2_vector[STadjacent_ingoing_vertex_index].STsucc);
					}
				}
			}

		}
	}

	// ST prism
	fprintf(g_pFileSTDebugLog, "\n\n\nSpace-time (SST) prism):\n");

	for (int n = 0; n < max_node_no - 1; n++) // for node
	{
		for (int t = 0; t < max_time_no - 1; t++) // for time
		{

			if (forward_time_space_label[n][t] + backward_time_space_label[n][t] <= max_time_no - 2 &&
				time_space_index[t][n] > -1)
			{
				space_prism_flag[n] = 1;
				fprintf(g_pFileSTDebugLog, "vertex %d which is node %d at time %d with the forward and backward optimal label costs of %.3f in total.\n",
					time_space_index[t][n],
					n,
					t,
					forward_time_space_label[n][t] + backward_time_space_label[n][t]);
			}

		}
	}





	//create input_vertex.csv
	vector<CVertex> vertex_vector;
	g_pFileinput_vertexLog = fopen("input_vertex.csv", "w");

	int index = 0;

	fprintf(g_pFileinput_vertexLog, "index, time, node, state, field, \n");

	for (int nn = 0; nn < max_node_no - 1; nn++)
	{
		for (int tt = 0; tt < max_time_no - 1; tt++)
		{
			if (nn < max_node_no - 2 &&
				space_prism_flag[nn] == 1 &&
				tt > forward_time_space_label[nn][tt] - 1 &&
				tt < max_time_no - 3 - backward_time_space_label[nn][tt] + 1)
			{
				space_time_flag[nn][tt] = 1;
				for (int ss = max_state_no - 2; ss > (min_fuel * 1000 -1); ss -= 4)
				{
					CVertex vertex;
					vertex.vertex_index = index;
					vertex.vertex_node = nn;
					vertex.vertex_time = tt;
					vertex.vertex_state = ss;
					vertex.vertex_field = 0;

					if (vertex.vertex_time == 0 && vertex.vertex_state == max_state_no - 2 && node_vector[nn].field == 1)
					{
						vertex.vertex_field = 1;
						initial_vertex1_index = vertex.vertex_index;
					}

					if (vertex.vertex_time == max_time_no - 3 && node_vector[nn].field == 2)
					{
						vertex.vertex_field = 2;
					}

					if (node_vector[nn].field == 4)
					{
						vertex.vertex_field = 4;
						vertex.vertex_state = max_state_no - 2;
						ss = -1;
					}

					vertex.label = max_value;
					vertex.pred = -1;  // default value
					vertex.succ = -1;  // default value
					vertex_vector.push_back(vertex);
					vertex1_vector.push_back(vertex);
					vertex2_vector.push_back(vertex);
					time_space_state_index[vertex.vertex_time][vertex.vertex_node][vertex.vertex_state] = index;

					int key = g_GetSpaceTimeStateKey(vertex.vertex_node, vertex.vertex_time, vertex.vertex_state);
					g_STSkey_to_vertex_index_map[key] = index;
					index++;
					fprintf(g_pFileinput_vertexLog, "%d, %d, %d, %d, %d\n",
						vertex.vertex_index,
						vertex.vertex_time,
						vertex.vertex_node,
						vertex.vertex_state,
						vertex.vertex_field);
				}
			}

			if (nn == max_node_no - 2 && tt == max_time_no - 2 && space_prism_flag[nn] == 1)
			{
				space_time_flag[nn][tt] = 1;
				CVertex vertex;
				vertex.vertex_index = index;
				vertex.vertex_node = nn;
				vertex.vertex_time = tt;
				vertex.vertex_state = 0;
				vertex.vertex_field = 3;  // field is 3 if it is sink vertex
				vertex.label = max_value;
				vertex.pred = -1;  // default value
				vertex.succ = -1;  // default value
				vertex_vector.push_back(vertex);
				vertex1_vector.push_back(vertex);
				vertex2_vector.push_back(vertex);
				time_space_state_index[vertex.vertex_time][vertex.vertex_node][vertex.vertex_state] = index;
				fprintf(g_pFileinput_vertexLog, "%d, %d, %d, %d, %d\n",
					vertex.vertex_index,
					vertex.vertex_time,
					vertex.vertex_node,
					vertex.vertex_state,
					vertex.vertex_field);

				sink_vertex_index = index;
				initial_vertex2_index = index;
				sink_vertex_node = nn;
				sink_vertex_time = tt;
				sink_vertex_state = 0;
				int key = g_GetSpaceTimeStateKey(vertex.vertex_node, vertex.vertex_time, vertex.vertex_state);
				g_STSkey_to_vertex_index_map[key] = index;

				index++;
			}



		}
	}

	for (int n = 0; n < max_node_no - 1; n++)
	{
		for (int t = 0; t < max_time_no - 1; t++)
		{
			fprintf(g_pFileSTDebugLog, "%d, %d, %d,\n",
				n, t, space_time_flag[n][t]);
		}
	}

	fclose(g_pFileinput_vertexLog);
	fclose(g_pFileSTDebugLog);


	// creat SST arcs		
	vector<CArc> arc_vector;
	g_pFileinput_arcLog = fopen("input_arc.csv", "w");
	fprintf(g_pFileinput_STarcLog, "index, from_vertex, to_vertex, from_time, to_time, from_node, to_node, from_state, to_state, cost, \n");
	double resource_consumption[max_link_no] = { 0.0 };
	double resource_consumption_per_min[max_link_no] = { 0.0 };
	double rounded_resource_consumption_per_min[max_link_no] = { 0.0 };
	//double speed[max_link_no] = { 0.0 };
	int index_1 = 0;

	for (int mm = 0; mm < max_link_no - 1; mm++)
	{
		travel_time[mm] = link_vector[mm].free_flow_time;
		resource_consumption_per_min[mm] = (-0.064 + 0.0056 * link_vector[mm].speed_limit + 0.00026 * pow(link_vector[mm].speed_limit - 50, 2)) / 60; // speed is in mph
		// we know that resource consumption per hour is a number between 0.18 and 1.88.
		// Refer to https://arxiv.org/ftp/arxiv/papers/1602/1602.06889.pdf
		// then resource consumption per minute is a number between 0.003 and 0.030


		rounded_resource_consumption_per_min[mm] = round(1000 * resource_consumption_per_min[mm]) / 1000;



		for (int tt = 0; tt < max_time_no - 3; tt++)
		{
			if (space_time_flag[link_vector[mm].from_node_id][tt] == 1
				&& space_time_flag[link_vector[mm].to_node_id][tt + travel_time[mm]] == 1)
			{
				for (int ss = max_state_no - 2; ss > min_fuel * 1000 -1; ss -= 4)
				{
					resource_consumption[mm] = rounded_resource_consumption_per_min[mm] * travel_time[mm];

					if (tt < max_time_no - 2 - travel_time[mm])
					{
						if (ss - 1000 * resource_consumption[mm] > -1)
						{
							CArc arc;
							arc.arc_index = index_1;
							arc.arc_cost = resource_consumption[mm];
							arc.arc_from_node = link_vector[mm].from_node_id;
							arc.arc_to_node = link_vector[mm].to_node_id;
							arc.arc_from_time = tt;
							arc.arc_to_time = tt + travel_time[mm];
							arc.arc_from_state = ss;
							arc.arc_to_state = arc.arc_from_state - 1000 * resource_consumption[mm];

							for (int ii = 0; ii < res_rechg_no; ii++)
							{
								if (link_vector[mm].from_node_id == recharging_station_node_index[ii])
								{
									arc.arc_from_state = max_state_no - 2;
									arc.arc_to_state = arc.arc_from_state - 1000 * resource_consumption[mm];
								}

								if (link_vector[mm].to_node_id == recharging_station_node_index[ii])
								{
									arc.arc_to_state = max_state_no - 2;
								}
							}

							arc.arc_from_vertex = g_GetSTSVertexIndex(arc.arc_from_node, arc.arc_from_time, arc.arc_from_state);
							arc.arc_to_vertex = g_GetSTSVertexIndex(arc.arc_to_node, arc.arc_to_time, arc.arc_to_state);

							if (arc.arc_from_vertex >= 0 && arc.arc_to_vertex >= 0)  // feasible arcs
							{
														


							arc1_vector.push_back(arc);
							arc2_vector.push_back(arc);

							temp_1.adjacent_outgoing_vertex_index = arc.arc_to_vertex;
							temp_1.adjacent_outgoing_vertex_travel_cost = arc.arc_cost;

							adjacent_outgoing_vertexes_vector[arc.arc_from_vertex].push_back(temp_1);



							temp_2.adjacent_ingoing_vertex_index = arc.arc_from_vertex;
							temp_2.adjacent_ingoing_vertex_travel_cost = arc.arc_cost;

							adjacent_ingoing_vertexes_vector[arc.arc_to_vertex].push_back(temp_2);

							fprintf(g_pFileinput_arcLog, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %.3f,\n",
								arc.arc_index,
								arc.arc_from_vertex,
								arc.arc_to_vertex,
								arc.arc_from_time,
								arc.arc_to_time,
								arc.arc_from_node,
								arc.arc_to_node,
								arc.arc_from_state,
								arc.arc_to_state,
								arc.arc_cost);

							if (index_1 % 100 == 0)
							{ 
							cout << "..." << endl;
							}
							index_1++;

							}
						}
					}
				}
			}
		}
	}

	// create waiting arcs
	for (int x = 0; x < max_node_no - 2; x++)
	{
		if (space_prism_flag[x] == 1)
		{
			for (int t = 0; t < max_time_no - 3; t++)
			{
				if (t > forward_time_space_label[x][t] - 1 &&
					t < max_time_no - 3 - backward_time_space_label[x][t] + 1 &&
					t + 1 > forward_time_space_label[x][t + 1] - 1 &&
					t + 1 < max_time_no - 3 - backward_time_space_label[x][t + 1] + 1)
				{
					for (int s = max_state_no - 2; s > (min_fuel * 1000 -1); s -= 4)
					{
						CArc arc;
						arc.arc_index = index_1;
						arc.arc_from_time = t;
						arc.arc_to_time = t + 1;
						arc.arc_from_node = x;
						arc.arc_to_node = x;
						arc.arc_from_state = s;
						arc.arc_to_state = s;
						arc.arc_cost = 0.0;

						for (int ii = 0; ii < res_rechg_no; ii++)
						{
							if (x == recharging_station_node_index[ii])
							{
								arc.arc_from_state = max_state_no - 2;
								arc.arc_to_state = arc.arc_from_state;
							}
						}

						arc.arc_from_vertex = g_GetSTSVertexIndex(arc.arc_from_node, arc.arc_from_time, arc.arc_from_state);
						arc.arc_to_vertex = g_GetSTSVertexIndex(arc.arc_to_node, arc.arc_to_time, arc.arc_to_state);

						if (arc.arc_from_vertex >= 0 && arc.arc_to_vertex >= 0)  // feasible arcs
						{ 

						arc1_vector.push_back(arc);
						arc2_vector.push_back(arc);

						temp_1.adjacent_outgoing_vertex_index = arc.arc_to_vertex;
						temp_1.adjacent_outgoing_vertex_travel_cost = arc.arc_cost;

						adjacent_outgoing_vertexes_vector[arc.arc_from_vertex].push_back(temp_1);



						temp_2.adjacent_ingoing_vertex_index = arc.arc_from_vertex;
						temp_2.adjacent_ingoing_vertex_travel_cost = arc.arc_cost;

						adjacent_ingoing_vertexes_vector[arc.arc_to_vertex].push_back(temp_2);

						fprintf(g_pFileinput_arcLog, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %.3f,\n",
							arc.arc_index,
							arc.arc_from_vertex,
							arc.arc_to_vertex,
							arc.arc_from_time,
							arc.arc_to_time,
							arc.arc_from_node,
							arc.arc_to_node,
							arc.arc_from_state,
							arc.arc_to_state,
							arc.arc_cost);
						if (index_1 % 100 == 0)
						{
							cout << "..." << endl;
						}
						index_1++;
						}
					}
				}
			}
		}
	}

	// create arcs from destination vertexes to the sink vertex
	for (int nn = 0; nn < index; nn++)
	{
		if (vertex_vector[nn].vertex_field == 2)
		{
			CArc arc;
			arc.arc_index = index_1;
			arc.arc_from_time = vertex_vector[nn].vertex_time;
			arc.arc_from_node = vertex_vector[nn].vertex_node;
			arc.arc_from_state = vertex_vector[nn].vertex_state;
			arc.arc_to_time = sink_vertex_time;
			arc.arc_to_node = sink_vertex_node;
			arc.arc_to_state = sink_vertex_state;
			arc.arc_cost = 0.0;
			arc.arc_from_vertex = vertex_vector[nn].vertex_index;
			arc.arc_to_vertex = sink_vertex_index;
			arc1_vector.push_back(arc);
			arc2_vector.push_back(arc);

			temp_1.adjacent_outgoing_vertex_index = arc.arc_to_vertex;
			temp_1.adjacent_outgoing_vertex_travel_cost = arc.arc_cost;

			adjacent_outgoing_vertexes_vector[arc.arc_from_vertex].push_back(temp_1);


			temp_2.adjacent_ingoing_vertex_index = arc.arc_from_vertex;
			temp_2.adjacent_ingoing_vertex_travel_cost = arc.arc_cost;

			adjacent_ingoing_vertexes_vector[arc.arc_to_vertex].push_back(temp_2);

			fprintf(g_pFileinput_arcLog, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %.3f,\n",
				arc.arc_index,
				arc.arc_from_vertex,
				arc.arc_to_vertex,
				arc.arc_from_time,
				arc.arc_to_time,
				arc.arc_from_node,
				arc.arc_to_node,
				arc.arc_from_state,
				arc.arc_to_state,
				arc.arc_cost);
			cout << "..." << endl;
			index_1++;
		}
	}

	fclose(g_pFileinput_arcLog);
}



//main function
int main()
{

	time_space_state_index = Allocate3DDynamicArray<int>(max_time_no, max_node_no, max_state_no);
	forward_time_space_state_label = Allocate3DDynamicArray<double>( max_node_no, max_time_no, max_state_no);
	backward_time_space_state_label = Allocate3DDynamicArray<double>(max_node_no, max_time_no, max_state_no);
	time_space_state_prism_flag = Allocate3DDynamicArray<int>(max_node_no, max_time_no, max_state_no);

	adjacent_outgoing_vertexes_vector = new vector<SAdjacent_outgoing_vertexes>[max_vertex_no];
	 adjacent_ingoing_vertexes_vector = new vector<SAdjacent_ingoing_vertexes>[max_vertex_no];



	double clock_t, start_t, end_t, total_t;
	start_t = clock();
	g_pFilenodes_for_plotLog = fopen("nodes_for_plot.csv", "w");
	g_pFileprism_for_plotLog = fopen("prism_for_plot.csv", "w");
	g_pFilelinks_for_plotLog = fopen("links_for_plot.csv", "w");
	for (int n = 0; n < max_node_no - 1; n++) // for node
	{
		for (int t = 0; t < max_time_no - 1; t++) // for time
		{
			for (int m = max_state_no - 2; m > (min_fuel * 1000 - 1); m -= 4)  // for energy 
			{
				time_space_state_index[t][n][m] = -1; // initialization

				forward_time_space_state_label[n][t][m] = 99999.0;  //forward space-time-state label

				backward_time_space_state_label[n][t][m] = 99999.0;  //backward space-time-state label

				time_space_state_prism_flag[n][t][m] = 0;

				time_space_index[t][n] = -1; // initialization

				forward_time_space_label[n][t] = 99999.0;  //forward space-time-state label

				backward_time_space_label[n][t] = 99999.0;  //backward space-time-state label

				space_prism_flag[n] = 0;
				space_time_flag[n][t] = 0;

			}
		}
	}

	g_read_input_data();

	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
	}



	// forward DP
	fprintf(g_pFileDebugLog, "Forward dynamic programming\n");

	//define an initial origin vertex for forward dynamic programming
	forward_time_space_state_label[vertex1_vector[initial_vertex1_index].vertex_node][vertex1_vector[initial_vertex1_index].vertex_time][vertex1_vector[initial_vertex1_index].vertex_state] = 0;
	vertex1_vector[initial_vertex1_index].label = 0;
	vertex1_vector[initial_vertex1_index].pred = 0;

	//forward dynamic programming for finding shortest path in a space-time-state network
	int x = 0;

	for (int t = 0; t < max_time_no - 1; t++) // for time
	{
		for (int n = 0; n < max_node_no - 1; n++) // for node
		{
			if (space_prism_flag[n] == 1)
			{
				for (int m = max_state_no - 2; m > (min_fuel * 1000 - 1); m -= 4)  // for energy 
				{
					if (time_space_state_index[t][n][m] == initial_vertex1_index && x == 0)
					{
						x = 1;
					}

					if (time_space_state_index[t][n][m] > -1 && x == 1)
					{
						int from_vertex_index = time_space_state_index[t][n][m];

						if (from_vertex_index == -1 || adjacent_outgoing_vertexes_vector[from_vertex_index].size() == 0) continue;

						for (int temp_adjacent_outgoing_vertex_index = 0; temp_adjacent_outgoing_vertex_index <= adjacent_outgoing_vertexes_vector[from_vertex_index].size() - 1; temp_adjacent_outgoing_vertex_index++)
						{
							int adjacent_outgoing_vertex_index = adjacent_outgoing_vertexes_vector[from_vertex_index][temp_adjacent_outgoing_vertex_index].adjacent_outgoing_vertex_index;

							double temp_label_cost = vertex1_vector[from_vertex_index].label + adjacent_outgoing_vertexes_vector[from_vertex_index][temp_adjacent_outgoing_vertex_index].adjacent_outgoing_vertex_travel_cost;

							if (temp_label_cost < vertex1_vector[adjacent_outgoing_vertex_index].label)
							{
								forward_time_space_state_label[vertex1_vector[adjacent_outgoing_vertex_index].vertex_node][vertex1_vector[adjacent_outgoing_vertex_index].vertex_time][vertex1_vector[adjacent_outgoing_vertex_index].vertex_state] = temp_label_cost;
								vertex1_vector[adjacent_outgoing_vertex_index].label = temp_label_cost;
								vertex1_vector[adjacent_outgoing_vertex_index].pred = from_vertex_index;
								fprintf(g_pFileDebugLog, "the label of vertex %d (node %d at time %d and state %d) is updated to %.3f.\n",
									adjacent_outgoing_vertex_index,
									vertex1_vector[adjacent_outgoing_vertex_index].vertex_node,
									vertex1_vector[adjacent_outgoing_vertex_index].vertex_time,
									vertex1_vector[adjacent_outgoing_vertex_index].vertex_state,
									vertex1_vector[adjacent_outgoing_vertex_index].label);
								fprintf(g_pFileDebugLog, "the predecessor of vertex %d is vertex %d.\n\n", adjacent_outgoing_vertex_index, vertex1_vector[adjacent_outgoing_vertex_index].pred);
							}
						}
					}
				}
			}
		}
	}

	// backward DP
	fprintf(g_pFileDebugLog, "\n\n\n\nBackward dynamic programming\n");

	//define an initial destination vertex for backward dynamic programming
	backward_time_space_state_label[vertex2_vector[initial_vertex2_index].vertex_node][vertex2_vector[initial_vertex2_index].vertex_time][vertex2_vector[initial_vertex2_index].vertex_state] = 0;
	vertex2_vector[initial_vertex2_index].label = 0;
	vertex2_vector[initial_vertex2_index].succ = 0;

	//backward dynamic programming for finding shortest path in a space-time-state network
	int y = 0;
	for (int t = max_time_no - 2; t > -1; t--) // for time
	{
		for (int n = 0; n < max_node_no - 1; n++) // for node
		{
			if (space_prism_flag[n] == 1)
			{
				for (int m = max_state_no - 2; m > -1; m -= 4)  // for energy 
				{
					if (time_space_state_index[t][n][m] == initial_vertex2_index && y == 0)
					{
						y = 1;
					}

					if (time_space_state_index[t][n][m] > -1 && y == 1)
					{
						int to_vertex_index = time_space_state_index[t][n][m];

						if (to_vertex_index == -1 || adjacent_ingoing_vertexes_vector[to_vertex_index].size() == 0) continue;

						for (int temp_adjacent_ingoing_vertex_index = 0; temp_adjacent_ingoing_vertex_index <= adjacent_ingoing_vertexes_vector[to_vertex_index].size() - 1; temp_adjacent_ingoing_vertex_index++)
						{
							int adjacent_ingoing_vertex_index = adjacent_ingoing_vertexes_vector[to_vertex_index][temp_adjacent_ingoing_vertex_index].adjacent_ingoing_vertex_index;

							double temp_label_cost = vertex2_vector[to_vertex_index].label + adjacent_ingoing_vertexes_vector[to_vertex_index][temp_adjacent_ingoing_vertex_index].adjacent_ingoing_vertex_travel_cost;

							if (temp_label_cost < vertex2_vector[adjacent_ingoing_vertex_index].label)
							{
								backward_time_space_state_label[vertex2_vector[adjacent_ingoing_vertex_index].vertex_node][vertex2_vector[adjacent_ingoing_vertex_index].vertex_time][vertex2_vector[adjacent_ingoing_vertex_index].vertex_state] = temp_label_cost;
								vertex2_vector[adjacent_ingoing_vertex_index].label = temp_label_cost;
								vertex2_vector[adjacent_ingoing_vertex_index].succ = to_vertex_index;
								fprintf(g_pFileDebugLog, "the label of vertex %d (node %d at time %d and state %d) is updated to %.3f.\n",
									adjacent_ingoing_vertex_index,
									vertex2_vector[adjacent_ingoing_vertex_index].vertex_node,
									vertex2_vector[adjacent_ingoing_vertex_index].vertex_time,
									vertex2_vector[adjacent_ingoing_vertex_index].vertex_state,
									vertex2_vector[adjacent_ingoing_vertex_index].label);
								fprintf(g_pFileDebugLog, "the successor of vertex %d is vertex %d.\n\n", adjacent_ingoing_vertex_index, vertex2_vector[adjacent_ingoing_vertex_index].succ);
							}
						}
					}
				}
			}
		}
	}

	//// backtrace the final optimal solution
	////print out
	//int vertex_sequence = 0;

	//int reversed_path_vertex_sequence[max_time_no];


	//reversed_path_vertex_sequence[vertex_sequence] = sink_vertex_index;

	//// backtrack to the origin (based on node and time predecessors)
	//int pred_vertex = vertex1_vector[sink_vertex_index].pred;

	//while (vertex1_vector[pred_vertex].pred != 0 && vertex_sequence < max_time_no - 1) // scan backward in the predessor array of the shortest path calculation results
	//{
	//	vertex_sequence++;
	//	reversed_path_vertex_sequence[vertex_sequence] = pred_vertex;

	//	//record current values of node and time predecessors, and update PredNode and PredTime

	//	int current_vector_index = reversed_path_vertex_sequence[vertex_sequence];
	//	pred_vertex = vertex1_vector[current_vector_index].pred;
	//}

	//vertex_sequence++;
	//reversed_path_vertex_sequence[vertex_sequence] = pred_vertex;

	////reverse the node sequence 
	//int path_vertex_sequence[max_time_no];

	//fprintf(g_pFileDebugLog, "the vertex sequence is ");
	//for (int n = 0; n <= vertex_sequence; n++)
	//{
	//	path_vertex_sequence[n] = reversed_path_vertex_sequence[vertex_sequence - n];

	//	fprintf(g_pFileDebugLog, "%d, ",
	//		vertex1_vector[path_vertex_sequence[n]].vertex_index);
	//}

	//fprintf(g_pFileDebugLog, "\nthe node sequence is ");
	//for (int n = 0; n <= vertex_sequence; n++)
	//{
	//	fprintf(g_pFileDebugLog, "%d, ",
	//		vertex1_vector[path_vertex_sequence[n]].vertex_node);
	//}

	//fprintf(g_pFileDebugLog, "\nthe time sequence is ");
	//for (int n = 0; n <= vertex_sequence; n++)
	//{
	//	fprintf(g_pFileDebugLog, "%d, ",
	//		vertex1_vector[path_vertex_sequence[n]].vertex_time);
	//}

	//fprintf(g_pFileDebugLog, "\nthe state sequence is ");
	//for (int n = 0; n <= vertex_sequence; n++)
	//{
	//	fprintf(g_pFileDebugLog, "%d, ",
	//		vertex1_vector[path_vertex_sequence[n]].vertex_state);
	//}

	//fprintf(g_pFileDebugLog, "\nthe label sequence is ");
	//for (int n = 0; n <= vertex_sequence; n++)
	//{
	//	fprintf(g_pFileDebugLog, "%.3f, ",
	//		vertex1_vector[path_vertex_sequence[n]].label);
	//}

	// SST prism
	fprintf(g_pFileDebugLog, "\n\n\nSpace-state-time (SST) prism):\n");

	float label_value;
	for (int n = 0; n < max_node_no - 1; n++) // for node
	{
		int count = 0;
		for (int t = 0; t < max_time_no - 1; t++) // for time
		{
			if (space_prism_flag[n] == 1)
			{
				for (int m = max_state_no - 2; m > (min_fuel * 1000 - 1); m -= 4)  // for energy 
				{
					if (forward_time_space_state_label[n][t][m] + backward_time_space_state_label[n][t][m] <= max_tolerant_cost &&
						time_space_state_index[t][n][m] > -1)
					{
						time_space_state_prism_flag[n][t][m] = 1;

						label_value = forward_time_space_state_label[n][t][m] + backward_time_space_state_label[n][t][m];
						fprintf(g_pFileDebugLog, "vertex %d which is node %d at time %d and state %d with the forward and backward optimal label costs of %.3f in total.\n",
							time_space_state_index[t][n][m],
							n,
							t,
							m,
							label_value);

						if (n < max_node_no - 2)
						{
							count++;
						}


					}
				}
			}
		}
		int nn = n;
		if (count > 0)
		{
		fprintf(g_pFileprism_for_plotLog, "%.6f, %.6f, %d, %d, <Point><coordinates>%f,%f</coordinates></Point>\n",
			node_vector[nn].x_coordinate,
			node_vector[nn].y_coordinate,
			count,
			node_vector[nn].node_id,
			node_vector[nn].x_coordinate,
			node_vector[nn].y_coordinate);

		}


	}

	end_t = clock();
	total_t = (end_t - start_t) / 1000;
	fprintf(g_pFileDebugLog, "\n\nCPU Running Time = %.1f seconds\n", total_t);


	system("complete: pause");

	Deallocate3DDynamicArray(time_space_state_index, max_time_no, max_node_no);
	Deallocate3DDynamicArray(forward_time_space_state_label, max_node_no, max_time_no);
	Deallocate3DDynamicArray(backward_time_space_state_label, max_node_no, max_time_no);
	Deallocate3DDynamicArray(time_space_state_prism_flag, max_node_no, max_time_no);


	fclose(g_pFilenodes_for_plotLog);
	fclose(g_pFileprism_for_plotLog);
	fclose(g_pFilelinks_for_plotLog);
	fclose(g_pFileDebugLog);
	return 0;
}

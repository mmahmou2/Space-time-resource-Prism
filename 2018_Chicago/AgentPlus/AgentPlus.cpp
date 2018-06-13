// Forward and backward dynamic programming for finding the shortest path in a space-time-state network

#include "stdafx.h"
#include "windows.h"
#include <iostream>
#include <vector>
#include "CSVParser.h"
#define _MAX_LABEL_COST 99999.0
using namespace std;

FILE* g_pFileDebugLog = NULL;
FILE* g_pFileprism_for_plotLog = NULL;


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
	int state_change_cost;  // resource state change externally given by users 
};


vector<CNode> node_vector;
vector<CLink> link_vector;


//define parameters
const int max_time_no = 120;	// 1 time unit more for sink node
const int max_node_no = 12145;  // number of nodes (i.e. 6) + sink node + 1
const int max_state_no = 1000;  // energy
const int max_link_no = 30697;
const double max_tolerant_cost = 5000;
const double min_state_no = 300;


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


double ***forward_time_space_state_label = NULL;  //forward space-time-state label
double ***backward_time_space_state_label = NULL;  //forward space-time-state label	
//int ***time_space_state_prism_flag = NULL;  //if both forward and backward SST label are feasible or less than some budget cost


int recharging_station_node_index[max_node_no];
int origin_node_id;
int destination_node_id;
int r_s = 0; // number of recharging stations 


// read input_node and input_link and convert them to input_vertex and input_arc
void g_read_input_data()
{
	fprintf(g_pFileprism_for_plotLog, "x, y, count, node_id, geometry\n");
	
	CCSVParser parser_node;

	int node_id;
	int field;
	double x_coordinate;
	double y_coordinate;

	int number_of_nodes = 0;


	if (parser_node.OpenCSVFile("input_node.csv", true))
	{

		while (parser_node.ReadRecord())
		{
			parser_node.GetValueByFieldName("node_id", node_id);
			parser_node.GetValueByFieldName("field", field);
			parser_node.GetValueByFieldName("x", x_coordinate, false);
			parser_node.GetValueByFieldName("y", y_coordinate, false);

			CNode node;
			node.node_id = node_id;
			node.field = field;		// field is 1 if it is origin, is 2 if destination, is 4 if recharging stations, and 0 otherwise.
			node.x_coordinate = x_coordinate;
			node.y_coordinate = y_coordinate;
			
			if (node.field == 1)
			{
				origin_node_id = node_id;
			}

			if (node.field == 2)
			{
				destination_node_id = node_id;
			}

			if (node.field == 4)
			{
				recharging_station_node_index [r_s] = node_id;
				r_s++;
			}

			node_vector.push_back(node);
			number_of_nodes++;
		}

		parser_node.CloseCSVFile();
		cout << "number of nodes = " << node_vector .size() << endl;
	};


	// read input_link.csv
	CCSVParser parser_link;
	if (parser_link.OpenCSVFile("input_link.csv", true))
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
		int state_change_cost;

		int number_of_links = 0;

		while (parser_link.ReadRecord())
		{
			parser_link.GetValueByFieldName("link_id", link_id);
			parser_link.GetValueByFieldName("from_node_id", from_node_id);
			parser_link.GetValueByFieldName("to_node_id", to_node_id);
			parser_link.GetValueByFieldName("direction", direction);
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("number_of_lanes", number_of_lanes);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);
			parser_link.GetValueByFieldName("link_type", link_type);
			parser_link.GetValueByFieldName("free_flow_time", free_flow_time);
			parser_link.GetValueByFieldName("state_change_cost", state_change_cost);

			
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
			link.free_flow_time = free_flow_time;
			link.state_change_cost = state_change_cost;
			link_vector.push_back(link);
			number_of_links++;
		}

		cout << "number of links = "<< link_vector.size() << endl;
		parser_link.CloseCSVFile();
	};
}



//main function
int main()
{
	double clock_t, start_t, end_t, total_t;
	start_t = clock();
	
	g_pFileprism_for_plotLog = fopen("prism_for_plot.csv", "w");

	g_read_input_data();  // read input node and link

	g_pFileDebugLog = fopen("Debug.txt", "w");

	bool b_debug_flag = false;
	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
	}

	forward_time_space_state_label = Allocate3DDynamicArray<double>(max_node_no, max_time_no, max_state_no);
	backward_time_space_state_label = Allocate3DDynamicArray<double>(max_node_no, max_time_no, max_state_no);

	for (int n = 0; n < node_vector.size(); n++) // for node
	{
		for (int t = 0; t < max_time_no; t++) // for time
		{
			for (int m = max_state_no; m >= min_state_no; m--)  // for fuel 
			{
				forward_time_space_state_label[n][t][m] = _MAX_LABEL_COST;  //forward space-time-state label
				backward_time_space_state_label[n][t][m] = _MAX_LABEL_COST;  //forward space-time-state label
			}
		}
	}

	
	// forward DP
	fprintf(g_pFileDebugLog, "Forward dynamic programming\n");

	
	forward_time_space_state_label[origin_node_id][0][max_state_no] = 0;


	//forward dynamic programming for finding shortest path in a space-time-state network
	int x = 0;

	for (int t = 0; t < max_time_no; t++) // for time
	{
		cout << "updating at time " << t << endl;
			
			for (int l = 0; l < link_vector.size(); l++) // for link
			{
			CLink link = link_vector[l];

			for (int m = max_state_no; m >= min_state_no; m--)  // for resource use
				{
					int n = link.from_node_id;
					int next_n = link.to_node_id;

					if (forward_time_space_state_label[n][t][m] < _MAX_LABEL_COST)
					{

						double temp_label_cost = forward_time_space_state_label[n][t][m] + link.state_change_cost;
						int next_time = t + link.free_flow_time;
						int next_m = m - link.state_change_cost;

						
						for (int rrs = 0; rrs < r_s; rrs++)
						{
							if (next_n == recharging_station_node_index[rrs])
							{
								next_m = max_state_no;
							}
						}

							if (next_time < max_time_no && temp_label_cost < forward_time_space_state_label[next_n][next_time][next_m])  //from _MAX_LABEL_COST states to feasible values of next_m
							{
								forward_time_space_state_label[next_n][next_time][next_m] = temp_label_cost;
								
							}

							// waiting arcs
							 temp_label_cost = forward_time_space_state_label[n][t][m] ;  // keep the same consumption 
							 next_time = t + 1;
							 next_m = m ;
							 next_n = n;  // from node id;  waiting at from node id

							if (next_time < max_time_no && temp_label_cost < forward_time_space_state_label[next_n][next_time][next_m])  //from _MAX_LABEL_COST states to feasible values of next_m
							{
								forward_time_space_state_label[next_n][next_time][next_m] = temp_label_cost;

							}

					}  // feasible from vertex
				}  // for energy
		}  // for link

	}


	for (int t = 0; t < max_time_no; t++) // for time
	{

		for (int n = 0; n < node_vector.size(); n++) // for link
		{

			for (int m = max_state_no; m >= min_state_no; m--)  // for resource use
			{

				if (forward_time_space_state_label[n][t][m] < _MAX_LABEL_COST)
				{
					fprintf(g_pFileDebugLog, "%d, %d, %d\n", n, t, m);
				}
			}
		}
	}

	

// backward DP
fprintf(g_pFileDebugLog, "Backward dynamic programming\n");


backward_time_space_state_label[destination_node_id][0][max_state_no] = 0;


// backward dynamic programming 

x = 0;

for (int t = 0; t < max_time_no; t++) // for time
{
	cout << "updating at time " << t << endl;

	for (int l = 0; l < link_vector.size(); l++) // for link
	{
		CLink link = link_vector[l];

		for (int m = max_state_no; m >= min_state_no; m--)  // for resource use
		{
			int n = link.from_node_id;
			int next_n = link.to_node_id;
			if (backward_time_space_state_label[n][t][m] < _MAX_LABEL_COST)
			{

				double temp_label_cost = backward_time_space_state_label[n][t][m] + link.state_change_cost;
				int next_time = t + link.free_flow_time;
				int next_m = m - link.state_change_cost;

				for (int rrs = 0; rrs < r_s; rrs++)
				{
					if (next_n == recharging_station_node_index[rrs])
					{
						next_m = max_state_no;
					}
				}

				if (next_time < max_time_no && temp_label_cost < backward_time_space_state_label[next_n][next_time][next_m])  //from _MAX_LABEL_COST states to feasible values of next_m
				{
					backward_time_space_state_label[next_n][next_time][next_m] = temp_label_cost;

				}

				// waiting arcs
				temp_label_cost = backward_time_space_state_label[n][t][m];  // keep the same consumption 
				next_time = t + 1;
				next_m = m;
				next_n = n;  // from node id;  waiting at from node id
				if (next_time < max_time_no && temp_label_cost < backward_time_space_state_label[next_n][next_time][next_m])  //from _MAX_LABEL_COST states to feasible values of next_m
				{
					backward_time_space_state_label[next_n][next_time][next_m] = temp_label_cost;

				}

			}  // feasible from vertex
		}  // for energy
	}  // for link

}


// SST prism
fprintf(g_pFileDebugLog, "\n\n\nSpace-state-time (SST) prism):\n");

int min_forward_label_destination = _MAX_LABEL_COST;


	for (int t = 0; t < max_time_no; t++) // for time
	{
		for (int m = max_state_no; m >= min_state_no; m--)  // for energy 
		{
			if (forward_time_space_state_label[destination_node_id][t][m] < min_forward_label_destination)
			{
				min_forward_label_destination = forward_time_space_state_label[destination_node_id][t][m];
			}
		}
	}


int min_backward_label_origin = 0;


	for (int t = 0; t < max_time_no; t++) // for time
	{
		for (int m = max_state_no; m >= min_state_no; m--)  // for energy 
		{
			if (backward_time_space_state_label[origin_node_id][t][m] < min_backward_label_origin)
			{
				min_backward_label_origin = backward_time_space_state_label[origin_node_id][t][m];
			}
		}
	}

	int margin = min_forward_label_destination + min_backward_label_origin;


for (int n = 0; n < node_vector.size(); n++) // for node
{
	int count = 0;
	for (int t = 0; t < max_time_no; t++) // for time
	{
		for (int m = max_state_no; m >= min_state_no; m--)  // for energy 
		{
			if (forward_time_space_state_label[n][t][m] + backward_time_space_state_label[n][t][m] < _MAX_LABEL_COST && 
				forward_time_space_state_label[n][t][m] + backward_time_space_state_label[n][t][m] > margin)
			{

					count++;
				
			}
		}
	}

	if (count > 0)
	{
		fprintf(g_pFileprism_for_plotLog, "%f, %f, %d, %d, <Point><coordinates>%f,%f</coordinates></Point>\n",
			node_vector[n].x_coordinate,
			node_vector[n].y_coordinate,
			count,
			node_vector[n].node_id,
			node_vector[n].x_coordinate,
			node_vector[n].y_coordinate);

	}
}
	end_t = clock();
	total_t = (end_t - start_t) / 1000;
	fprintf(g_pFileDebugLog, "\n\nCPU Running Time = %.1f seconds\n", total_t);


	system("complete: pause");

	//Deallocate3DDynamicArray(forward_time_space_state_label, max_node_no, max_time_no);
	//Deallocate3DDynamicArray(backward_time_space_state_label, max_node_no, max_time_no);


	fclose(g_pFileprism_for_plotLog);
	fclose(g_pFileDebugLog);
	return 0;
}

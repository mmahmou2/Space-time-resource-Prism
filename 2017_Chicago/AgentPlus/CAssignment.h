#pragma once

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

class CShortestPath;
class CAssignment
{
public:
	CAssignment();
	~CAssignment();
	int create_shortest_path_calculator();
	int do_assignment();

protected:
	int m_number_of_threads;
	CShortestPath* m_pNetworkForSP ;
	CCostofArc** m_arcs_need_updated;
	int* m_numbers_of_agents_lower;
	int* m_numbers_of_agents_upper;

	void do_shortest_path();
	int regenerate_agents_demands()
	{
		float reassign_ratio = 0.1;
		g_eligible_agent.clear();

	}

	void writePassengerPath()
	{
		FILE* pFileAgentPathLog;
		pFileAgentPathLog = fopen("agent_path.csv", "w");
		if (pFileAgentPathLog == NULL)
		{
			cout << "Cannot open File agent_path.csv" << endl;
			//g_ProgramStop();
		}
		else
		{

			fprintf(pFileAgentPathLog, "from_arc_no,vtx_no,stop_id,original_stop_id,stop_name,x,y,time,cost,trip_id,original_trip_id,trip_headsign,route_id,original_route_id,short_route_name,transfer_times\n");
			//fprintf(pFileAgentPathLog, "from_arc_no,vtx_no,stop_id,original_stop_id,stop_name,x,y,time,route_id,original_route_id,short_route_name\n");
		}
		float cost = 0;
		int vtx_no = 0;
		for (int i = 0; i < g_agent_number; i++)
		{
			CAgentDemand* p_agent = &(g_agent_demands[i]);
			//writting the agent information
			cost = p_agent->m_PathCost[0];
			vtx_no = p_agent->m_PathNodeIDs[0];
			fprintf(pFileAgentPathLog, "agent_id=%d,O={%9.6f %9.6f},D={%9.6f %9.6f},departure_time=%6.1f arrival_time= %6.1f cost=%6.1f, Deboarding stop_id=%d\n",
				p_agent->agent_id,
				p_agent->from_x,
				p_agent->from_y,
				p_agent->to_x,
				p_agent->to_y,
				p_agent->departure_time_in_min,
				cost,
				cost - p_agent->departure_time_in_min,
				g_vertex_vector[vtx_no].stop_id
			);

			CArc* pArc = NULL;
			CVertex* pV = NULL;
			int arc_no = -1;
			int last_vertex_no = -1;
			int transfer_times = 0;
			float cur_time = p_agent->departure_time_in_min;
			float cur_cost = 0;
			for (int j = p_agent->m_PathNodeIDs.size() - 2; j >= 0; j--)
			{
				vtx_no = p_agent->m_PathNodeIDs[j];
				if (last_vertex_no >= 0)
				{
					pArc = g_vertex_vector[last_vertex_no].getArc(vtx_no);

					if (pArc->type != 0)
						transfer_times++;
					arc_no = pArc->arc_id;
				}
				last_vertex_no = vtx_no;
				pV = &g_vertex_vector[vtx_no];

				fprintf(pFileAgentPathLog, "%d,%d,%d,%s,%s,%9.6f,%9.6f,%6.1f,%6.1f,%d,%s,%s,%d,%s,%s,%d\n",
					arc_no,
					pV->vertex_id,
					pV->StopObj->stop_id,
					pV->StopObj->original_id.c_str(),
					pV->StopObj->name.c_str(),
					pV->StopObj->x,
					pV->StopObj->y,
					pV->time_in_min,
					p_agent->m_PathCost[j],
					pV->TripObj->trip_id,
					pV->TripObj->original_trip_id.c_str(),
					pV->TripObj->trip_head_sign.c_str(),
					pV->TripObj->RouteObj->route_id,
					pV->TripObj->RouteObj->original_route_id.c_str(),
					pV->TripObj->RouteObj->short_name.c_str(),
					transfer_times
				);
			}

		}
		fclose(pFileAgentPathLog);
	}


};
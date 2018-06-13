#pragma once

#include "DataStruct.h"
#include "GlobalDef.h"

using namespace std;

class gridTool
{
public:
	static int m_grid3D_rows;
	static int m_grid3D_cols;
	static std::map<string, grid3D*> m_grid3D_map;
	static std::map<string, grid2D*> m_grid2D_map;
	
	static double m_grid3D_len;//the spatial length of a grid,unit:degree of lat or lon
	static double m_grid3D_time_len; // the temporal length of a 3D grid,unit:minute

public:
public:
	static void InitTool()
	{
		m_grid3D_rows = 0;
		m_grid3D_cols = 0;
		m_grid3D_map.clear();
		m_grid2D_map.clear();
		m_grid3D_len = 0.01;
		m_grid3D_time_len = 15;
		m_area_min_x = 999999999;
		m_area_min_y = 999999999;
		m_area_max_x = -999999999;
		m_area_max_y = -999999999;
	}
	static void Dispose()
	{
		free_grid2Ds_memory();
		free_grid3Ds_memory();
	}
private:

};

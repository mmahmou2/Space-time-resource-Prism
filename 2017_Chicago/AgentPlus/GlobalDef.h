#pragma once

#define _GRID_SPATIAL_LEN 0.01
#define _GRID_TEMPORAL_LEN 15
#define _MAX_LABEL_COST 99999
#define _FLOAT_ERROR 0.00000001
#define _MAX_VEH_CAPACITY 30
class CDemands;
class Network;

extern CDemands g_demands;
extern Network g_network;

//global functions declaration
std::string g_convert_keys_to_string(int key1, int key2);

std::string g_convert_3Dkeys_to_string(int key1, int key2, int key3);

std::string g_convert_2Dkeys_to_string(int key1, int key2);

void g_ProgramStop();

double g_get_straight_distance(double x1, double y1, double x2, double y2);

double g_get_manhattan_distance(double x1, double y1, double x2, double y2);

template <typename T> T** AllocateDynamicArray(int nRows, int nCols, int initial_value = 0);

template <typename T> void DeallocateDynamicArray(T** dArray, int nRows, int nCols);

template <typename T> T***Allocate3DDynamicArray(int nX, int nY, int nZ);

template <typename T> void Deallocate3DDynamicArray(T*** dArray, int nX, int nY);

template <typename T> T**** Allocate4DDynamicArray(int nM, int nX, int nY, int nZ);

template <typename T> void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY);

template <typename T> T***** Allocate5DDynamicArray(int nM, int nX, int nY, int nZ, int nW);

template <typename T> void Deallocate5DDynamicArray(T**** dArray, int nM, int nX, int nY, int nW);

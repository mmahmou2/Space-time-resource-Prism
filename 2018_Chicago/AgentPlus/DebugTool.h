#pragma once
#include <thread>
#include <mutex>

using namespace std;

class CDebugTool
{
private:
	static std::recursive_mutex m_file_mutex;
	static FILE* m_output_file;
public:
	static void write_debug_info(char *fmt, ...)
	{
		m_file_mutex.lock();
		FILE* pDebugFile = fopen("Debug.txt", "a+");
		if (pDebugFile == NULL)
			return;
		va_list argptr;
		va_start(argptr, fmt);
		vfprintf(pDebugFile, fmt, argptr);
		va_end(argptr);
		fclose(pDebugFile);
		m_file_mutex.unlock();
	}
	static void write_output_info(char *fmt, ...)
	{
		m_file_mutex.lock();
		if (m_output_file == NULL)
		{
			m_output_file = fopen("output.txt", "w");
			if (m_output_file == NULL)
				return;
		}
		va_list argptr;
		va_start(argptr, fmt);
		vfprintf(m_output_file, fmt, argptr);
		va_end(argptr);
		m_file_mutex.unlock();
	}
	static void StartTool()
	{
		m_output_file = fopen("output.txt", "w");
	}
	static void StopTool()
	{
		if (m_output_file != NULL)
			fclose(m_output_file);
	}
	static void write_output_info(char *fmt, ...)

};


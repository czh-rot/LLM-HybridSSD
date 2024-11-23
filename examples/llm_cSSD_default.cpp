//

#include <iostream>

#include "openai.hpp"

#include "stdlib.h"
#include "memory.h"
#include "math.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <deque>
#include <numeric>
#include <cmath>


// Data types
#define		UINT16			unsigned short
#define		UINT32			unsigned
#define		UINT64			unsigned long long

typedef int  int32_t;
float usedRatio;

UINT32			slice_size = 209715200;
std::vector<float>  latencies;
const double percentile = 99.00; // 99.99%的尾延迟
const double percentile1 = 99.99; // 99.99%的尾延迟
int 			total_write_length = 0;
int 			real_write_length = 0;



// *************************************     Configurable parameters
#define		LBA_SIZE			16384  //512
#define		MU_SIZE				16384  //512
#define		Slice_SIZE			209715200 //1024*1024*200  //200 MB -- 209715200
#define		NUM_TEMP_ZONES		3
#define		LOG_CAP				(UINT64)34359738368 //32GB 
//page number = 34359738368 / 16384 = 2097152-----32GB
#define		EFF_OP				1  //  1//1.3//1.5//1.5F //1 //1.63F
#define		MIN_RDY				6

// *************************************     Derived parameters
#define		NUM_Slice		LOG_CAP/Slice_SIZE  //200 MB
#define		RAW_CAP			(UINT64)(LOG_CAP * EFF_OP)
#define		MU_COUNT		LOG_CAP/MU_SIZE  // 10485760
#define		RAW_MUS			(RAW_CAP/MU_SIZE)  // 10485760
#define		GCU_COUNT		(UINT64)(RAW_MUS/GCU_SIZE/4)  //34359738368/16384/256/4 = 2048 BLOCK
#define		GCU_SIZE		(UINT64) 256//131072  // (UINT64)100 //(UINT64)589824  //589824    //137329.1015625 
//QLC page 2048*512*4 = 4194304

// *************************************     General constants
#define		STATE_GC		1
#define		STATE_HOST		2
#define		STATE_USED		3
#define		TYPE_HOST		1
#define		TYPE_GC			0
#define		INVALIDATED		0xffffffff


//// *************************************    Data structures


struct Config {
    int windows_size;
    int kmeans_threshold;
    int max_iterations;
    long slice_size;
    int rl_training_interval;
    float rl_reward;
    float rl_learning_rate;
    int conversion_granularity;
    int gc_granularity;
};


std::vector<Config> config_history;
std::vector<std::string> performance_history;

typedef struct FwdTblEntry
{
	UINT16			GcuIndex;
	UINT32			GcuOffset;
	bool			Mapped;
	UINT16			ZoneNum;
} FwdTblEntry;

typedef struct MappingTBL
{
	UINT16			ZoneNum;
	bool			Mapped;
} MappingTBL;

typedef struct Gcu
{
	UINT16			ThisIndex;
	UINT16			Type;
	UINT16			State;
	UINT32			Written;
	UINT32			StaleCount;
	UINT32			CurFillIndx;
	UINT32			Zone;
	struct Gcu		*NextRdy;
	UINT32			Mua[GCU_SIZE * 4];
	UINT32			numErasesCmpl;
} Gcu;

typedef struct RdyGcus
{
	Gcu		*Head;
	Gcu		*Tail;
	UINT32	RdyCount;
} RdyGcus;

typedef struct GcPool
{
	UINT64	AvailGcu;
	float	Priority[GCU_COUNT];  // how many writes to trigger one GC based on number of available GCUs
	UINT64	NWrites;
} GcPool;


// *************************************    Glogal variables
FwdTblEntry		*FwdTbl;
MappingTBL      *MapTBL;
Gcu				*GcuTbl;
RdyGcus			RdyList1, RdyList2, RdyList3;
GcPool			GcList;
Gcu				*CurHostGcu;
Gcu			    *ConvGcu = NULL;
UINT64			TotGcGcus = 0;
float			ins_time = 0;
int 			rl_write_length = 0;
UINT64 			period_count = 0;
UINT64          slice_access[NUM_Slice];
UINT64          recognition[NUM_Slice];
Gcu				*CurZoneGcu[NUM_TEMP_ZONES];
Gcu				*CurGcZoneGcu[NUM_TEMP_ZONES];
UINT64			CurOpByZone[NUM_TEMP_ZONES];
UINT64			NumberGc = 0;
UINT64			Numberconvert = 0;
UINT64			Prec_finish = 0;
float 			RdLatency[3] = { 0.02, 0.02, 0.140 }; //ms
float 			WtLatency[3] = { 0.2, 0.2, 3 }; //ms
float 			PgLatency[3] = { 2, 2, 20 }; //ms
int				ZoneSpace[3] = { 1147, 901 }; //0.56 0.44
UINT64			GCU_SIZE_MUS[NUM_TEMP_ZONES] = { GCU_SIZE, GCU_SIZE, GCU_SIZE * 4 };
float 			overall_latency = 0;
float 			overall_read_latency = 0;
float 			overall_write_latency = 0;
float 			overall_gc_latency = 0;
float 			overall_convert_latency = 0;
float 			overall_response_time = 0;
UINT64			Num_Unsigned_GCU = 0;

int 			max_iterations = 10;
int				dataAllocation = 0;
int				conversion_granularity = 1;
int				gc_granularity = 1;

UINT64			Num_Used_Mu = 0;
UINT64			Total_Mu = 0;
UINT64			Total_Mu_w = 2097152;
UINT64			Num_Tot_Gcu = 0;
UINT64			DevCondition = 1;
UINT64			curGCUIndex = 0;
UINT32			write_period = 1000;
UINT32			window_size = 1000;
UINT32			change_threshold = 1500;

UINT32			newwrite = 0;
UINT32			newwritelength = 0;
float			average_time = 1.6;
std::vector<int> available_GCU;
std::vector<int> available_GCU_0; //未分配类型SLC的block索引列表
std::vector<int> available_GCU_1; //未分配类型MLC的block索引列表
std::vector<double> rolling_std;
UINT64			Do_GC_SLC_QLC = 0;

float 			last_overall_latency = 0.0;
float 			current_write_latency = 0.0;
int 			gpt_period = 1000000;
int 			last_period_count = 0;
int 			last_NumberGc = 0;
int 			last_Numberconvert = 0;
int 			last_newwritelength = 0;
int 			last_TotalReadNumber = 0;
float 			current_overall_latency = 0.0;
float 			current_slc_ratio = 0.0;
int 			current_period_count = 0;
int 			current_NumberGc = 0;
int 			current_Numberconvert = 0;
int 			current_newwritelength = 0;
int 			current_TotalReadNumber = 0;
UINT32 			ZoneCount[NUM_TEMP_ZONES];

//UINT64			ZoneUsedPages[NUM_TEMP_ZONES]; //已经使用的page数
//UINT64			ZoneUnUsedPages[NUM_TEMP_ZONES]; //未使用的page数
//UINT32 			ZoneUsedBlocks[NUM_TEMP_ZONES];

UINT64			ZoneUsedblocks[NUM_TEMP_ZONES]; //已经使用的block数
//UINT64			ZoneUnUsedPages[NUM_TEMP_ZONES]; //未使用的page数
UINT64			TotalWriteSize;
UINT64			TotalWriteNumber;
UINT64			TotalReadSize;
UINT64			TotalReadNumber;


//********K-Means Algorithm Functions**********//
using namespace std;
//void Conversion(UINT32 Zone1, UINT32 Zone2);
void Conversion2(UINT32 Zone1, UINT32 Zone2);
Gcu *GetNewGcu(UINT32 Zone);


class Point
{
private:
	int id_point, id_cluster;
	vector<double> values;
	int total_values;
	string name;

public:
	Point(int id_point, vector<double>& values, string name = "")
	{
		this->id_point = id_point;
		total_values = values.size();

		for (int i = 0; i < total_values; i++)
			this->values.push_back(values[i]);

		this->name = name;
		id_cluster = -1;
	}

	int getID()
	{
		return id_point;
	}

	void setCluster(int id_cluster)
	{
		this->id_cluster = id_cluster;
	}

	int getCluster()
	{
		return id_cluster;
	}

	double getValue(int index)
	{
		return values[index];
	}

	int getTotalValues()
	{
		return total_values;
	}

	void addValue(double value)
	{
		values.push_back(value);
	}

	string getName()
	{
		return name;
	}
};

class Cluster
{
private:
	int id_cluster;
	vector<double> central_values;
	vector<Point> points;

public:
	Cluster(int id_cluster, Point point)
	{
		this->id_cluster = id_cluster;

		int total_values = point.getTotalValues();

		for (int i = 0; i < total_values; i++)
			central_values.push_back(point.getValue(i));

		points.push_back(point);
	}

	void addPoint(Point point)
	{
		points.push_back(point);
	}

	bool removePoint(int id_point)
	{
		int total_points = points.size();

		for (int i = 0; i < total_points; i++)
		{
			if (points[i].getID() == id_point)
			{
				points.erase(points.begin() + i);
				return true;
			}
		}
		return false;
	}

	double getCentralValue(int index)
	{
		return central_values[index];
	}

	void setCentralValue(int index, double value)
	{
		central_values[index] = value;
	}

	Point getPoint(int index)
	{
		return points[index];
	}

	int getTotalPoints()
	{
		return points.size();
	}

	int getID()
	{
		return id_cluster;
	}
};

class KMeans
{
private:
	int K; // number of clusters
	int total_values, total_points, max_iterations;
	vector<Cluster> clusters;

	// return ID of nearest center (uses euclidean distance)
	int getIDNearestCenter(Point point)
	{
		double sum = 0.0, min_dist;
		int id_cluster_center = 0;

		for (int i = 0; i < total_values; i++)
		{
			sum += pow(clusters[0].getCentralValue(i) -
				point.getValue(i), 2.0);
		}

		min_dist = sqrt(sum);

		for (int i = 1; i < K; i++)
		{
			double dist;
			sum = 0.0;

			for (int j = 0; j < total_values; j++)
			{
				sum += pow(clusters[i].getCentralValue(j) -
					point.getValue(j), 2.0);
			}

			dist = sqrt(sum);

			if (dist < min_dist)
			{
				min_dist = dist;
				id_cluster_center = i;
			}
		}

		return id_cluster_center;
	}

public:
	KMeans(int K, int total_points, int total_values, int max_iterations)
	{
		this->K = K;
		this->total_points = total_points;
		this->total_values = total_values;
		this->max_iterations = max_iterations;
	}

	void run(vector<Point> & points)
	{
		if (K > total_points)
			return;

		vector<int> prohibited_indexes;

		// choose K distinct values for the centers of the clusters
		for (int i = 0; i < K; i++)
		{
			while (true)
			{
				int index_point = rand() % total_points;

				if (find(prohibited_indexes.begin(), prohibited_indexes.end(),
					index_point) == prohibited_indexes.end())
				{
					prohibited_indexes.push_back(index_point);
					points[index_point].setCluster(i);
					Cluster cluster(i, points[index_point]);
					clusters.push_back(cluster);
					break;
				}
			}
		}

		int iter = 1;

		while (true)
		{
			bool done = true;

			// associates each point to the nearest center
			for (int i = 0; i < total_points; i++)
			{
				int id_old_cluster = points[i].getCluster();
				int id_nearest_center = getIDNearestCenter(points[i]);

				if (id_old_cluster != id_nearest_center)
				{
					if (id_old_cluster != -1)
						clusters[id_old_cluster].removePoint(points[i].getID());

					points[i].setCluster(id_nearest_center);
					clusters[id_nearest_center].addPoint(points[i]);
					done = false;
				}
			}

			// recalculating the center of each cluster
			for (int i = 0; i < K; i++)
			{
				for (int j = 0; j < total_values; j++)
				{
					int total_points_cluster = clusters[i].getTotalPoints();
					double sum = 0.0;

					if (total_points_cluster > 0)
					{
						for (int p = 0; p < total_points_cluster; p++)
							sum += clusters[i].getPoint(p).getValue(j);
						clusters[i].setCentralValue(j, sum / total_points_cluster);
					}
				}
			}

			if (done == true || iter >= max_iterations)
			{
				//cout << "Break in iteration " << iter << "\n\n";
				break;
			}

			iter++;
		}

		// Recognition
		for (int i = 0; i < K; i++)
		{
			int total_points_cluster = clusters[i].getTotalPoints();
			for (int j = 0; j < total_points_cluster; j++)
			{
				recognition[clusters[i].getPoint(j).getID()] = i;
				//cout <<i<<" "<< clusters[i].getPoint(j).getID()<<" " << slice_access[clusters[i].getPoint(j).getID()] << " "<<clusters[i].getPoint(j).getValue(0)<<endl;
			}
		}
	}
};

static int32_t get_random_number(void){
	int32_t randNum;
	randNum = rand() % 10000 + 1;
	return randNum;
}

// ***************************  RL parameter ****************//
#define gamma 8
static int32_t epsilon1 = 90;
static int32_t epsilon2 = 99;
//#define alpha 1
float 	alpha = 0.1;

// define the max number of options of each state
#define SLC_space_max 9
#define utilization_max 5
#define previous_action_max 5
#define SLC_freq_max 2
#define write_freq_max 2
#define state_max SLC_space_max*utilization_max*write_freq_max*previous_action_max*SLC_freq_max
#define action_max 5

// define the state space
struct state {
	int32_t SLC_space;
	int32_t utilization;
	int32_t previous_action;
	int32_t write_freq;
	int32_t SLC_freq;
};  //state_diff


// initialize the current state
static struct state current_state = { 0,0,0,0,0 };
static int32_t current_action;
static int32_t reward;
static struct state next_state;
static int32_t next_state_max_q;

//define the Q-Table  //4500
static int32_t q[state_max][action_max] = { 0 };

// get the corresponding index of a certain state
static int32_t get_row_q(struct state* s){
	int32_t idx = s->SLC_space*(utilization_max*write_freq_max*previous_action_max*SLC_freq_max) + \
		s->utilization*(write_freq_max*previous_action_max*SLC_freq_max) + \
		s->write_freq*(previous_action_max*SLC_freq_max) + \
		s->previous_action*SLC_freq_max + \
		s->SLC_freq;
	//fprintf(stderr, "row: %lu %lu %lu %lu %lu\n", idx, s->current_phase, s->SLC_space, s->write_freq, s->Buffer_space);
	return idx;
}

// check if the Q-values are undefined for a certain state
static int32_t is_empty_q(struct state* s) {
	int32_t flag = 1;
	int32_t row = get_row_q(s);
	int32_t i = 0;
	for (;i < action_max;i++) {
		if (q[row][i] != 0)
			flag = 0;
	}
	return flag;
}

// get the max Q-value
static int32_t max_value_q(struct state* s) {
	int32_t i = 1;
	int32_t row = get_row_q(s);
	int32_t max = q[row][0];
	for (;i < action_max;i++) {
		if (q[row][i] > max)
			max = q[row][i];
	}
	return max;
}

// get the max action
static int32_t max_action(struct state* s) {
	int32_t i = 1, a = 0;
	int32_t row = get_row_q(s);
	float max = q[row][0];
	for (;i < action_max;i++) {
		//fprintf(stderr, "max: %f %d %d %d\n", max, q[row][i], row, q[row][i]>max);
		if (q[row][i] > max) {
			max = q[row][i];
			a = i;
		}
	}
	return a;
}

// define the reward function, the thresholds here are just examples 
// and it can be adjusted according to your workload and the requirments of QoS
static int32_t get_reward(int32_t response_time) {
	int32_t r = 0;
	
	response_time /= rl_write_length;

	if (response_time > average_time) {
		r = -100;
	}
	else {
		r = 100;
	}
	
	return r;
} //reward function

static int32_t get_SLC_space() {
	int32_t stale_space = 0;
	for (long long unsigned int i = 1; i <= GCU_COUNT; i++)
	{
		if (GcuTbl[i].Zone == 0 || GcuTbl[i].Zone == 1)
		{
			stale_space = 1 + stale_space;
		}
	}
	stale_space = stale_space + available_GCU_0.size();

	float fra = float(stale_space) / float(GCU_COUNT);

	int r;
	if (fra > 0.9) r = 0;
	else if (fra > 0.8) r = 1;
	else if (fra > 0.7) r = 2;
	else if (fra > 0.6) r = 3;
	else if (fra > 0.5) r = 4;
	else if (fra > 0.4) r = 5;
	else if (fra > 0.3) r = 6;
	else if (fra > 0.2) r = 7;
	else r = 8;

	return r;
}  // SLC_space : 9 

static int32_t get_utilization() {
	int r = 0;
	int32_t stale_space = 0;
	for (long long unsigned int i = 1; i <= GCU_COUNT; i++)
	{
		stale_space = GcuTbl[i].StaleCount + stale_space;
	}

	float fra = float(stale_space) / float(RAW_CAP/MU_SIZE); //  无效页面的总数 / 总页面数

	if (fra > 0.8) r = 0;
	else if (fra > 0.6) r = 1;
	else if (fra > 0.4) r = 2;
	else if (fra > 0.2) r = 3;
	else r = 4;

	return r;
} // space_utilization : 5

static int get_SLC_freq(int SLC_write) {
	int r = 0;
	
	float fra = float(SLC_write) / float(write_period);

	if (fra > 0.5) r = 0;
	else r = 1;

	return r;
}


static int32_t get_write_period(int w) {
	int32_t r = 0;
	float fra = float(w) / float(write_period);

	if (fra > 0.5) r = 0;
	else r = 1;

	return r;
}

// generate the next state
static struct state get_next_state(int32_t _current_action, int w, int SLC_write) {
	struct state ns;
	
	ns.utilization = get_utilization();
	ns.SLC_space = get_SLC_space();
	ns.previous_action = _current_action;
	ns.write_freq = get_write_period(w);
	ns.SLC_freq = get_SLC_freq(SLC_write);
	return ns;
}

// *************************************    Support functions
// Remove a GCU from the ready list
Gcu	*GetRdyGcu(RdyGcus *RdyList)
{
	Gcu	*RetGcu = NULL;

	if (RdyList->Head != NULL)
	{
		RetGcu = RdyList->Head;
		RdyList->Head = RetGcu->NextRdy;
		if (RdyList->Tail == RetGcu)
		{
			RdyList->Tail = RdyList->Head;
		}
		RdyList->RdyCount--;
	}
	GcList.AvailGcu = RdyList->RdyCount;

	return(RetGcu);
}


// Add a GCU to the ready list.
void AddRdyGcu(Gcu	*RdyGcu, RdyGcus *RdyList)
{
	UINT32	Index = RdyGcu->ThisIndex;

	memset(RdyGcu, 0, sizeof(Gcu));

	RdyGcu->ThisIndex = Index;

	if (RdyList->Tail)
	{
		RdyList->Tail->NextRdy = RdyGcu;
	}
	else
	{
		RdyList->Head = RdyGcu;
	}
	RdyList->Tail = RdyGcu;
	RdyGcu->NextRdy = NULL;
	RdyList->RdyCount++;
	GcList.AvailGcu = RdyList->RdyCount;
}

// Initialize all tables and variables
void BuildTbls(void)
{
	int	i;

	ConvGcu = (Gcu*)calloc(1, sizeof(Gcu));
	memset(ConvGcu, 0, sizeof(Gcu));
	ConvGcu->NextRdy = NULL;


	FwdTbl = (FwdTblEntry*)calloc(MU_COUNT, sizeof(FwdTblEntry));
	MapTBL = (MappingTBL*)calloc(MU_COUNT, sizeof(FwdTblEntry));
	GcuTbl = (Gcu*)calloc(GCU_COUNT + 1, sizeof(Gcu));
	for (i = 0;i < NUM_TEMP_ZONES;i++) {  // 3个zone
		CurZoneGcu[i] = (Gcu*)calloc(1, sizeof(Gcu));
		memset(CurZoneGcu[i], 0, sizeof(*CurZoneGcu[i]));
		ZoneUsedblocks[i] = 1;
	}

	memset(FwdTbl, 0, sizeof(*FwdTbl));
	memset(MapTBL, 0, sizeof(*MapTBL));

	for (long long unsigned int i = 0; i < NUM_Slice; i++)
	{
		slice_access[i] = 0;
		//recognition[i] = i % 2; // 初始是平分
		recognition[i] = 0;
	}

	for (long long unsigned int i = 0; i < GCU_COUNT; i++) //2048
	{
		GcuTbl[i].ThisIndex = i;
		GcuTbl[i].numErasesCmpl = 0;
		
		if (i == 0) { // 3个类型各分配一个block，分别加入各自的ready list，共3个ready list
			AddRdyGcu(&GcuTbl[i], &RdyList1);
			GcuTbl[i].Zone = 0;
			ZoneUsedblocks[0]++;
			ZoneCount[0] = ZoneCount[0] + 1;
			Total_Mu = Total_Mu + GCU_SIZE; // The size of the available space that has been allocated
		} // Total_Mu 指已经分配的page的个数，总的逻辑page的个数为 MU_COUNT
		else if (i == 1) {
			AddRdyGcu(&GcuTbl[i], &RdyList2);
			ZoneUsedblocks[1]++;
			ZoneCount[1] = ZoneCount[1] + 1;
			Total_Mu = Total_Mu + GCU_SIZE;
			GcuTbl[i].Zone = 1;
		}
		else if (i == 2) {
			AddRdyGcu(&GcuTbl[i], &RdyList3);
			ZoneUsedblocks[2]++;
			ZoneCount[2] = ZoneCount[2] + 1;
			Total_Mu = Total_Mu + GCU_SIZE * 4;
			GcuTbl[i].Zone = 2;
		}
		else if ((int)i < ZoneSpace[0]) {
			available_GCU_0.push_back(i);
		}else{
			available_GCU_1.push_back(i);
		}
		
		
		
		/*
		if (i < ZoneSpace[0]) {
			if(i == 0){
				AddRdyGcu(&GcuTbl[i], &RdyList1);
				ZoneUsedblocks[0]++;
				GcuTbl[i].Zone = 0;
				//Total_Mu_w = Total_Mu_w + GCU_SIZE;
				Total_Mu = Total_Mu + GCU_SIZE;  // SLC
			}else{
				available_GCU_0.push_back(i);
			}
			//ZoneUnUsedPages[0] = ZoneUnUsedPages[0] + GCU_SIZE;
			ZoneCount[0] = ZoneCount[0] + 1;
			//ZoneUsedPages[0] = 0;
			
		}else { //1352
			if(i == 1147){
				AddRdyGcu(&GcuTbl[i], &RdyList2);
				ZoneUsedblocks[1]++;
				Total_Mu = Total_Mu + GCU_SIZE * 4;  // QLC
				//Total_Mu_w = Total_Mu_w + GCU_SIZE * 4;
				GcuTbl[i].Zone = 1;
			}else{
				available_GCU_1.push_back(i);
			}
			
			ZoneCount[1] = ZoneCount[1] + 1;
			//ZoneUsedPages[1] = 0;
			//ZoneUnUsedPages[1] = ZoneUnUsedPages[1] + GCU_SIZE * 4;
			
		}
		*/
		//fprintf(stderr, "period_count: %lld\n", period_count);
	}
	
	//fprintf(stderr, "ZoneUnUsedPages[0]: %lld, ZoneUnUsedPages[1]: %lld\n", ZoneUnUsedPages[0], ZoneUnUsedPages[1]);
	fprintf(stderr, "available_GCU_0: %ld \n", available_GCU_0.size());
	fprintf(stderr, "available_GCU_1: %ld \n", available_GCU_1.size());
	//fprintf(stderr, "ZoneUsedblocks[0]: %lld, ZoneUsedblocks[1]: %lld\n", ZoneUsedblocks[0], ZoneUsedblocks[1]);

	curGCUIndex = NUM_TEMP_ZONES;
	Num_Unsigned_GCU = GCU_COUNT - NUM_TEMP_ZONES;
	Num_Tot_Gcu = GCU_COUNT;
	/*
		for( i = 0; i< 32; i++ )
		{
			GcuTbl[i].ThisIndex = i;
			GcuTbl[i].numErasesCmpl = 0;
			AddRdyGcu( &GcuTbl[i] );
		}
		*/

	//fprintf(stderr, "ZoneUnUsedPages[1]: %lld\n", ZoneUnUsedPages[0]);
	//fprintf(stderr, "ZoneUnUsedPages[1]: %lld\n", ZoneUnUsedPages[1]);

	for (i = 0; i < NUM_TEMP_ZONES; i++)
	{
		if (i == 0)
		{
			CurHostGcu = GetRdyGcu(&RdyList1);
			CurHostGcu->State = STATE_HOST;
			CurHostGcu->Type = TYPE_HOST;
			CurHostGcu->Zone = i;
			CurZoneGcu[i] = CurHostGcu;
		}
		else if (i == 1)
		{
			CurHostGcu = GetRdyGcu(&RdyList2);
			CurHostGcu->State = STATE_HOST;
			CurHostGcu->Type = TYPE_HOST;
			CurHostGcu->Zone = i;
			CurZoneGcu[i] = CurHostGcu;
		}else{
			CurHostGcu = GetRdyGcu(&RdyList3);
			CurHostGcu->State = STATE_HOST;
			CurHostGcu->Type = TYPE_HOST;
			CurHostGcu->Zone = i;
			CurZoneGcu[i] = CurHostGcu;
		}
		CurOpByZone[i] = 0;
		CurGcZoneGcu[i] = CurHostGcu;
		CurGcZoneGcu[i]->State = STATE_GC;
		CurGcZoneGcu[i]->Type = TYPE_GC;
		CurGcZoneGcu[i]->Zone = i;
	}
}


Gcu	*ChooseBestGcu(UINT32 SelectZone){ // SelectZone是0代表slc，对应zone应该是0和1；SelectZone是2代表QLC

	UINT32	i;
	UINT32	MaxStaleness = 0;
	Gcu		*BestGcu = NULL;


	for( i = 0; i<GCU_COUNT; i++ )
	{
		if ( GcuTbl[i].State == STATE_USED )
		{
			if (SelectZone == 0){
				if ((GcuTbl[i].Zone == 0) || (GcuTbl[i].Zone == 1)){
					if ( GcuTbl[i].StaleCount >= MaxStaleness ){
						BestGcu =  &GcuTbl[i];
						MaxStaleness = GcuTbl[i].StaleCount;
					}
				}
			} else if (SelectZone == 2){
				if (GcuTbl[i].Zone == SelectZone){
					if ( GcuTbl[i].StaleCount >= MaxStaleness ){
						BestGcu =  &GcuTbl[i];
						MaxStaleness = GcuTbl[i].StaleCount;
					}
				}
			}
		}
	}

	


	if (BestGcu == NULL){
		//printf("ERROR: Null BestGcu pointer\n");
		for( i = 0; i<GCU_COUNT; i++ )
		{
			if ( GcuTbl[i].State == STATE_USED ){

				if ( GcuTbl[i].StaleCount > MaxStaleness )
				{
					BestGcu =  &GcuTbl[i];
					MaxStaleness = GcuTbl[i].StaleCount;
				}
			}
		}
	}

	if (BestGcu == NULL)
	{
		printf("ERROR: Null Gcu pointer\n");
		while (1);
	}

	//ZoneUsedblocks[BestGcu.Zone]--;

	return(BestGcu);
}


void DoGc(int Zone1, int Zone2){
	// GcStatus = 0: SLC -> SLC
	// GcStatus = 1: SLC -> QLC
	// GcStatus = 2: QLC -> QLC


	Gcu		*EvictingGcu = NULL;
	UINT32	MuIndex = 0;
	UINT32	EvictedCount = 0;
	
	//int Zone1 = Zone1;
	//int Zone2 = Zone2;

	UINT32  Temp_Num_ValidMu = 0;

	// latency update
	overall_latency = overall_latency + PgLatency[Zone1];
	current_write_latency += PgLatency[Zone1];
	ins_time = ins_time + PgLatency[Zone1];

	//responseTime[period_count] = responseTime[period_count] + PgLatency[Zone1];

	// The ready count has fallen below threshold so go do some garbage collection
	EvictingGcu = ChooseBestGcu(Zone1);
	// ZoneUsedblocks[Zone]--;

	Zone1 = EvictingGcu->Zone;
	//Zone2 = EvictingGcu->Zone;
	ZoneUsedblocks[Zone1]--;

	TotGcGcus--;
	CurOpByZone[Zone1] -= EvictingGcu->StaleCount;
	memset(ConvGcu, 0, sizeof(Gcu));
	for (MuIndex = 0; MuIndex < GCU_SIZE_MUS[Zone1]; MuIndex++)
	{
		if (EvictingGcu->Mua[MuIndex] != INVALIDATED &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].GcuIndex == EvictingGcu->ThisIndex &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].GcuOffset == MuIndex &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].Mapped == true)
		{
			ConvGcu->Mua[ConvGcu->CurFillIndx] = EvictingGcu->Mua[MuIndex];
			ConvGcu->CurFillIndx++;
			EvictingGcu->StaleCount++;
			Temp_Num_ValidMu++;
			EvictedCount++;
			overall_latency = overall_latency + RdLatency[Zone1] + WtLatency[Zone2];
			real_write_length += 1;
			current_write_latency += RdLatency[Zone1] + WtLatency[Zone2];
			ins_time = ins_time + RdLatency[Zone1] + WtLatency[Zone2];
			//responseTime[period_count] = responseTime[period_count] + RdLatency[Zone1] + WtLatency[Zone2];
			//ZoneUsedPages[Zone1]++;
			//ZoneUnUsedPages[Zone1]--;
		}
	}
	if (EvictingGcu->StaleCount != EvictingGcu->Written)
	{
		fprintf(stderr, "GC Eviction Error %d %d\n", EvictingGcu->StaleCount, EvictingGcu->Written);
	}
	else
	{
		// Adding the old zone to the new zone ready list
		if ( Zone1 == 0 || Zone1 == 1 ){
			Num_Unsigned_GCU = Num_Unsigned_GCU + 1;
			available_GCU_0.push_back(EvictingGcu->ThisIndex);
			ZoneCount[Zone1] = ZoneCount[Zone1] - 1;
		
			UINT32	temp_Index = EvictingGcu->ThisIndex;
			memset(EvictingGcu, 0, sizeof(Gcu));
			EvictingGcu->ThisIndex = temp_Index;

			//AddRdyGcu(EvictingGcu, &RdyList1);
				//Total_Mu = Total_Mu + GCU_SIZE*2;
		}else{
			Num_Unsigned_GCU = Num_Unsigned_GCU + 1;
			available_GCU_1.push_back(EvictingGcu->ThisIndex);
			ZoneCount[Zone1] = ZoneCount[Zone1] - 1;

			UINT32	temp_Index = EvictingGcu->ThisIndex;
			memset(EvictingGcu, 0, sizeof(Gcu));
			EvictingGcu->ThisIndex = temp_Index;

			//AddRdyGcu(EvictingGcu, &RdyList2);
				//Total_Mu = Total_Mu + GCU_SIZE*4;
		}
	}

		//CurHostGcu = CurZoneGcu[Zone2];
	//如果空间不足，新分配一个同类型block
	for (MuIndex = 0; MuIndex < Temp_Num_ValidMu; MuIndex++)
	{
		CurZoneGcu[Zone2]->Mua[CurZoneGcu[Zone2]->CurFillIndx] = ConvGcu->Mua[MuIndex];

		FwdTbl[ConvGcu->Mua[MuIndex]].GcuIndex = CurZoneGcu[Zone2]->ThisIndex;
		FwdTbl[ConvGcu->Mua[MuIndex]].GcuOffset = CurZoneGcu[Zone2]->CurFillIndx++;
		FwdTbl[ConvGcu->Mua[MuIndex]].Mapped = true;

		// When the currently writing GCU for GC is full get a new one.
		if (++(CurZoneGcu[Zone2]->Written) == GCU_SIZE_MUS[Zone2])
		{
				// This GCU is full
			TotGcGcus++;
			CurZoneGcu[Zone2]->State = STATE_USED;
			CurZoneGcu[Zone2] = GetNewGcu(Zone2);
			ZoneCount[Zone2] = ZoneCount[Zone2]+1;	

			CurZoneGcu[Zone2]->State = STATE_GC;
			CurZoneGcu[Zone2]->Type = TYPE_GC;
			CurZoneGcu[Zone2]->Zone = Zone2;
		}
	}

	if (Prec_finish == 1) {
		NumberGc++;
	}

	if (available_GCU_0.size() < MIN_RDY){
		Do_GC_SLC_QLC = 1;
	}
}


void Conversion2(UINT32 Zone1, UINT32 Zone2) { //need to convert 2 gcu to 1 and add 1to small and add 1 to large
	Gcu		*EvictingGcu = NULL;
	UINT32	MuIndex = 0;
	UINT32	EvictedCount = 0;
	UINT32  Temp_Num_ValidMu = 0;

	// latency update
	overall_latency = overall_latency + PgLatency[Zone1];
	current_write_latency += PgLatency[Zone1];
	ins_time = ins_time + PgLatency[Zone1];
	//responseTime[period_count] = responseTime[period_count] + PgLatency[Zone1];

	//fprintf(stderr, "conversion %d %d %d %d\n", Zone1, Zone2, available_GCU.size(), Total_Mu);
	// The ready count has fallen below threshold so go do some garbage collection
	EvictingGcu = ChooseBestGcu(Zone1);
	Zone1 = EvictingGcu->Zone;
	
	ZoneUsedblocks[Zone1]--;
	TotGcGcus--;
	CurOpByZone[Zone1] -= EvictingGcu->StaleCount;
	memset(ConvGcu, 0, sizeof(Gcu));
	for (MuIndex = 0; MuIndex < GCU_SIZE_MUS[Zone1]; MuIndex++)
	{
		if (EvictingGcu->Mua[MuIndex] != INVALIDATED &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].GcuIndex == EvictingGcu->ThisIndex &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].GcuOffset == MuIndex &&
			FwdTbl[EvictingGcu->Mua[MuIndex]].Mapped == true)
		{
			ConvGcu->Mua[ConvGcu->CurFillIndx] = EvictingGcu->Mua[MuIndex];
			ConvGcu->CurFillIndx++;
			EvictingGcu->StaleCount++;
			Temp_Num_ValidMu++;
			EvictedCount++;
			overall_latency = overall_latency + RdLatency[Zone1] + WtLatency[Zone2];
			real_write_length += 1;
			current_write_latency += RdLatency[Zone1] + WtLatency[Zone2];
			ins_time = ins_time + RdLatency[Zone1] + WtLatency[Zone2];
			//responseTime[period_count] = responseTime[period_count] + RdLatency[Zone1] + WtLatency[Zone2];
			//ZoneUsedPages[1]++;
			//ZoneUnUsedPages[1]--;
		}
	}
	if (EvictingGcu->StaleCount != EvictingGcu->Written)
	{
		fprintf(stderr, "Conv Eviction Error %d %d %d\n", EvictingGcu->StaleCount, EvictingGcu->Written, Zone1);
	}
	else
	{
		// Adding the old zone to the new zone ready list

		Num_Unsigned_GCU = Num_Unsigned_GCU + 1;
		//available_GCU.push_back(EvictingGcu->ThisIndex);
		available_GCU_1.push_back(EvictingGcu->ThisIndex);
		ZoneCount[Zone1] = ZoneCount[Zone1]-1;
		UINT32	temp_Index = EvictingGcu->ThisIndex;
		memset(EvictingGcu, 0, sizeof(Gcu));
		EvictingGcu->ThisIndex = temp_Index;
		//AddRdyGcu(EvictingGcu, &RdyList2);
		//GcuTbl[EvictingGcu->ThisIndex].Zone = 1;
		//ZoneUnUsedPages[1] = ZoneUnUsedPages[1] + GCU_SIZE * 4;
		//ZoneUsedPages[0] = ZoneUsedPages[0] - GCU_SIZE;
	}

	for (MuIndex = 0; MuIndex < Temp_Num_ValidMu; MuIndex++)
	{
		CurZoneGcu[Zone2]->Mua[CurZoneGcu[Zone2]->CurFillIndx] = ConvGcu->Mua[MuIndex];
		FwdTbl[ConvGcu->Mua[MuIndex]].GcuIndex = CurZoneGcu[Zone2]->ThisIndex;
		FwdTbl[ConvGcu->Mua[MuIndex]].GcuOffset = CurZoneGcu[Zone2]->CurFillIndx++;
		FwdTbl[ConvGcu->Mua[MuIndex]].Mapped = true;
		// When the currently writing GCU for GC is full get a new one.
		if (++(CurZoneGcu[Zone2]->Written) == GCU_SIZE_MUS[Zone2])
		{
			// This GCU is full
			TotGcGcus++;
			CurZoneGcu[Zone2]->State = STATE_USED;
			CurZoneGcu[Zone2] = GetNewGcu(Zone2);
			ZoneCount[Zone2] = ZoneCount[Zone2]+1;

			CurZoneGcu[Zone2]->State = STATE_GC;
			CurZoneGcu[Zone2]->Type = TYPE_GC;
			CurZoneGcu[Zone2]->Zone = Zone2;
		}
	}

	if (Prec_finish == 1) {
		Numberconvert++;
	}
}


Gcu *GetNewGcu(UINT32 Zone)
{  // 0,1,2
	//fprintf(stderr, "Get new Gcu\n");
	Gcu *tempCurHostGcu = NULL;

	if(Zone == 0){
		if (available_GCU_0.empty() == 0){
			int i = available_GCU_0.back();
			available_GCU_0.pop_back();
			GcuTbl[i].ThisIndex = i;
			GcuTbl[i].numErasesCmpl = 0;

			AddRdyGcu(&GcuTbl[i], &RdyList1);
			tempCurHostGcu = GetRdyGcu(&RdyList1);
			ZoneUsedblocks[0]++;
		}
		tempCurHostGcu->Zone = Zone;
		curGCUIndex = curGCUIndex + 1; //已分配类型的block数量
		Num_Unsigned_GCU = Num_Unsigned_GCU - 1; //未分配类型的block数量
	}
	if(Zone == 1){
		if (available_GCU_0.empty() == 0){
			int i = available_GCU_0.back();
			available_GCU_0.pop_back();
			GcuTbl[i].ThisIndex = i;
			GcuTbl[i].numErasesCmpl = 0;

			AddRdyGcu(&GcuTbl[i], &RdyList2);
			ZoneUsedblocks[1]++;
			tempCurHostGcu = GetRdyGcu(&RdyList2);
		}
		tempCurHostGcu->Zone = Zone;
		curGCUIndex = curGCUIndex + 1; //已分配类型的block数量
		Num_Unsigned_GCU = Num_Unsigned_GCU - 1; //未分配类型的block数量
	}
	if(Zone == 2){
		if (available_GCU_1.empty() == 0){
			int i = available_GCU_1.back();
			available_GCU_1.pop_back();
			GcuTbl[i].ThisIndex = i;
			GcuTbl[i].numErasesCmpl = 0;

			AddRdyGcu(&GcuTbl[i], &RdyList3);
			ZoneUsedblocks[2]++;
			tempCurHostGcu = GetRdyGcu(&RdyList3);
		}
		tempCurHostGcu->Zone = Zone;
		curGCUIndex = curGCUIndex + 1; //已分配类型的block数量
		Num_Unsigned_GCU = Num_Unsigned_GCU - 1; //未分配类型的block数量
	}
	

	if (tempCurHostGcu == NULL) {
		fprintf(stderr, "No space left!\n");
	}
	return tempCurHostGcu;
}



void DoHostWrite(UINT64 Lba, UINT64 Length, UINT32 Zone){
	UINT64	StartMua;
	UINT64	MuCount = 0;
	UINT64	i;
	//fprintf(stderr, "test\n");

	//float usedblock = ZoneUsedblocks[0] / 4 + ZoneUsedblocks[1];
	//usedRatio = usedblock / 2048;

	/*
	if ( available_GCU_0.size() > MIN_RDY ){ // 优先写到SLC中
		Zone = 0;
		CurHostGcu = CurZoneGcu[0];
	}else{  // SLC不够再写到QLC中
		Zone = 1;
		CurHostGcu = CurZoneGcu[1];
		if(available_GCU_1.size() < 2){
			fprintf(stderr, "QLC dead\n");
		}
	}
	*/
	//if(available_GCU_0.size() < MIN_RDY){
		//Zone = 2;
		//std::cout << "\n write to QLC\n";
	//}


	CurHostGcu = CurZoneGcu[Zone];
	if(available_GCU_0.size() < 2){
		fprintf(stderr, "space0 dead\n");
	}
	if(available_GCU_1.size() < 2){
		fprintf(stderr, "space1 dead\n");
	}

	// Find the start and end MUA's
	StartMua = (Lba *  LBA_SIZE) / MU_SIZE;
	
	if (MU_SIZE == LBA_SIZE)
	{
		// Mus are always aligned to LBA boundaries
		StartMua = Lba;
		MuCount = Length;
	}

	for (i = 0; i < MuCount; i++)
	{
		// Invalidate the old MU location
		if (FwdTbl[StartMua + i].Mapped == true)
		{
			GcuTbl[FwdTbl[StartMua + i].GcuIndex].Mua[FwdTbl[StartMua + i].GcuOffset] = INVALIDATED;
			GcuTbl[FwdTbl[StartMua + i].GcuIndex].StaleCount++;
			CurOpByZone[GcuTbl[FwdTbl[StartMua + i].GcuIndex].Zone]++;
		}
		MapTBL[StartMua + i].ZoneNum = Zone;
		if (MapTBL[StartMua + i].Mapped == false) {
			MapTBL[StartMua + i].Mapped = true;
			Num_Used_Mu = Num_Used_Mu + 1;
			//newwrite = newwrite + 1;
		}
		// Update the new location
		CurHostGcu->Mua[CurHostGcu->CurFillIndx] = StartMua + i;
		FwdTbl[StartMua + i].GcuIndex = CurHostGcu->ThisIndex;
		FwdTbl[StartMua + i].GcuOffset = CurHostGcu->CurFillIndx++;
		FwdTbl[StartMua + i].ZoneNum = Zone;
		FwdTbl[StartMua + i].Mapped = true;
		overall_latency = overall_latency + WtLatency[Zone];
		real_write_length += 1;
		current_write_latency += WtLatency[Zone];
		ins_time = ins_time + WtLatency[Zone];
		overall_response_time = overall_response_time + WtLatency[Zone];
		//responseTime[period_count] = responseTime[period_count] + WtLatency[Zone];
		//ZoneUsedPages[Zone]++; // 写一次page加一次
		//ZoneUnUsedPages[Zone]--;

		//Advance fill location, get a ready GCU if needed.
		if (++(CurHostGcu->Written) == GCU_SIZE_MUS[Zone])
		{
			// This GCU is full
			CurHostGcu->State = STATE_USED;
			CurZoneGcu[Zone] = GetNewGcu(Zone);
			ZoneCount[Zone] = ZoneCount[Zone]+1;
			CurHostGcu = CurZoneGcu[Zone];
			CurHostGcu->State = STATE_HOST;
			//if(CurHostGcu->ThisIndex == 1 | CurHostGcu->ThisIndex == 2)	printf("find it!\n");
			CurHostGcu->Type = TYPE_HOST;
			CurZoneGcu[Zone] = CurHostGcu;
			CurHostGcu->Zone = Zone;
		}
	}


	if (available_GCU_0.size() < MIN_RDY){
		DoGc(0, 0);
	} //先尝试SLC的gc
	if (Do_GC_SLC_QLC == 1){
		if (available_GCU_0.size() < MIN_RDY){
			DoGc(0, 2);
		} //解决不了的话就将SLC的数据移动到QLC
		Do_GC_SLC_QLC = 0;
	}
	if (available_GCU_1.size() < MIN_RDY){
		DoGc(2, 2); //处理一下QLC的垃圾回收
		//Conversion2(0,1);
	}


	//UINT64 unused = available_GCU_0.size() + available_GCU_1.size();
	
	//float usedblock = ZoneUsedblocks[0] / 4 + ZoneUsedblocks[1];
	//UINT32 usedblock = GCU_COUNT - available_GCU_0.size() - available_GCU_1.size();
	//usedRatio = usedblock / (float)GCU_COUNT;

	if (available_GCU_1.size() < MIN_RDY){
		Conversion2(0,2);
	}  //QLC满了再实现转化

	
	//printf("usedblock: %d \n", usedblock);
	//printf("usedRatio: %f \n", usedRatio);
/*
	if (usedRatio > 0.2f && usedRatio <= 0.3f){
		while( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() > 1024 ){
			Conversion2(0,2); // SLC->QLC
		}
	}
	if (usedRatio > 0.3f && usedRatio <= 0.4f){
		while( ZoneUsedblocks[0] + available_GCU_0.size() > 819 ){
			Conversion2(0,2);
		}
	}
	if (usedRatio > 0.4f && usedRatio <= 0.5f){
		while( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() > 614 ){
			Conversion2(0,2);
		}
	}
	if (usedRatio > 0.5f && usedRatio <= 0.6f){
		while( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() > 512 ){
			Conversion2(0,2);
		}
	}
	if (usedRatio > 0.6f && usedRatio <= 0.7f){
		while( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() > 409 ){
			Conversion2(0,2);
		}
	}
	if (usedRatio > 0.7f){
		//printf("70");
		while( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() > 204 ){
			Conversion2(0,2);
		}
	}
*/
	

}

bool compare(float a, float b){ 
	return a<b;
}
	
int slice_clusting() {
	/*
	int K = 2;
	int total_points = NUM_Slice;
	int total_values = 1;
	int max_iterations = 10;
	int has_name = 0;

	vector<Point> points;
	string point_name;

	for(int i = 0; i < total_points; i++)
	{
		vector<double> values;
		values.push_back(slice_access[i]);

		Point p(i, values);
		points.push_back(p);

	}
	KMeans kmeans(K, total_points, total_values, max_iterations);
	kmeans.run(points);
	*/
	long long unsigned int min = 10000;
	long long unsigned int max = 0;
	for (long long unsigned int i = 0; i < NUM_Slice; i++)
	{
		if (min > slice_access[i]) min = slice_access[i];
		if (max < slice_access[i]) max = slice_access[i];
	}
	//int hot_threshold = (max - min) / 2 + min;
	int hot_threshold = (max - min) / 2;

	return hot_threshold;
}

void slice_clusting_kmeans() { // 应该记录一个表，表里是每个slice对应的冷热情况
	int K = 2;
	int total_points = NUM_Slice;
	int total_values = 1;

	vector<Point> points;

	for(int i = 0; i < total_points; i++){
		vector<double> values;
		values.push_back(slice_access[i]);

		Point p(i, values);
		points.push_back(p);
	}
	KMeans kmeans(K, total_points, total_values, max_iterations);
	kmeans.run(points);
}

// 实时计算滑动窗口标准差的函数
double calculateCurrentStd(const std::deque<int>& window) {
    //int window_size = window.size();
    double sum = std::accumulate(window.begin(), window.end(), 0);
    double mean = sum / window_size;

    double variance = 0.0;
    for (int value : window) {
        variance += (value - mean) * (value - mean);
    }
    variance /= window_size;

    return std::sqrt(variance);
}


std::string extractNewConfiguration(const std::string &input) {
    std::string startDelimiter = "```";
    std::string endDelimiter = "```";

    // Find the start and end positions of the new configuration
    size_t startPos = input.find(startDelimiter);
    size_t endPos = input.find(endDelimiter, startPos + startDelimiter.length());

    // If both delimiters are found, extract the content in between
    if (startPos != std::string::npos && endPos != std::string::npos) {
        startPos += startDelimiter.length();  // Move to the character after the start delimiter
        return input.substr(startPos, endPos - startPos);
    }

    return "";  // Return an empty string if the delimiters are not found
}


Config extractConfigurationValues(const std::string &config) {
	Config  cfg; // Create a Config struct to hold the values
    std::istringstream stream(config);
    std::string line;
    std::string value;

	dataAllocation = 1;
    // Go through each line of the configuration text
    while (std::getline(stream, line)) {
        if (line.find("Windows size:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.windows_size = std::stoi(value);
			window_size = std::stoi(value);
        } else if (line.find("K-means trigger threshold:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.kmeans_threshold = std::stoi(value);
			change_threshold = std::stoi(value);
        } else if (line.find("Max iterations:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.max_iterations = std::stoi(value);
			max_iterations = std::stoi(value);
        } else if (line.find("Slice size:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.slice_size = std::stol(value);
        } else if (line.find("RL training interval:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.rl_training_interval = std::stoi(value);
			write_period = std::stoi(value);
        } else if (line.find("RL Reward:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.rl_reward = std::stof(value);
			average_time = std::stof(value);
        } else if (line.find("RL learning rate:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.rl_learning_rate = std::stof(value);
			alpha = std::stof(value);
        } else if (line.find("Conversion granularity:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.conversion_granularity = std::stoi(value);
			conversion_granularity = std::stoi(value);
        } else if (line.find("GC granularity:") != std::string::npos) {
            value = line.substr(line.find(":") + 1);
            cfg.gc_granularity = std::stoi(value);
			gc_granularity = std::stoi(value);
        }
    }

    return cfg; // Return the populated config structure
}

std::string configToString(const Config &cfg) {
    std::ostringstream oss;
    oss << "Windows size: " << cfg.windows_size << "\n"
        << "K-means trigger threshold: " << cfg.kmeans_threshold << "\n"
        << "Max iterations: " << cfg.max_iterations << "\n"
        << "Slice size: " << cfg.slice_size << " bytes\n"
        << "RL training interval: " << cfg.rl_training_interval << "\n"
        << "RL Reward: " << cfg.rl_reward << "\n"
        << "RL learning rate: " << cfg.rl_learning_rate << "\n"
        << "Conversion granularity: " << cfg.conversion_granularity << "\n"
        << "GC granularity: " << cfg.gc_granularity << "\n";

    return oss.str();
}

void requestGPT(int TotalWriteNumber, float current_overall_latency, int current_period_count, int current_NumberGc, int current_Numberconvert, int current_newwritelength, int current_TotalReadNumber, float current_slc_ratio) {

	openai::start();

	std::string model_name = "gpt-4o";
	std::string history_config_change = "First, the historical performance and configuration changes are shown.";

	std::string performation_message = 
		"The performance is: During " + std::to_string(TotalWriteNumber-gpt_period) + "-" + std::to_string(TotalWriteNumber) + " writes, "
		"the workload has been re-identified " + std::to_string(current_period_count) + " times, the number of garbage collection "
        "is " + std::to_string(current_NumberGc) + " times, and the space conversion is " + std::to_string(current_Numberconvert) + ". "
		"There are " + std::to_string(available_GCU_0.size()) + " and " + std::to_string(available_GCU_1.size()) + " SLC and QLC blocks remaining in the current space "
        ", of which the SLC area occupies " + std::to_string(current_slc_ratio) + "% of the whole space, and the rest is the QLC area. "
		"During this period, " + std::to_string(current_newwritelength) + " pages were written and "
        "" + std::to_string(current_TotalReadNumber) + " pages were read, "
		"and the total execution time during this period was " + std::to_string(current_overall_latency) + "s.\n";

	for (size_t i = 0; i < config_history.size(); ++i) {
        //std::cout << "Configuration " << i + 1 << ":\n";
        //std::cout << config_history[i] << std::endl;
		history_config_change += performance_history[i];
		history_config_change += "The point in time configuration changes are as follows: \n";
		history_config_change += configToString(config_history[i]);
    }
	

	//std::string model_name = "gpt-3.5-turbo";
    std::string system_message = 
        "You are a SSD Expert. You are being consulted by a company to help improve their SSD configuration by "
        "optimizing their options file based on their system information and benchmark output. The SSD consists of "
        "SLC and QLC modes, which can switch between each other. The total capacity of the SSD is 32GB with a page "
        "size of 16KB. Each block contains 1024 pages. Garbage collection is triggered when the remaining available "
        "space is less than or equal to 6 blocks. The garbage collection uses a greedy strategy, selecting the block "
        "with the fewest valid pages each time, and the granularity of garbage collection is one block. The SLC mode "
        "has a page read delay of 0.02ms, a page write delay of 0.2ms, and a block erase delay of 2ms; the QLC mode "
        "has a page read delay of 0.14ms, a page write delay of 3ms, and a block erase delay of 20ms. Initially, the "
        "SLC mode occupies 56% of the total space, with the remaining space in QLC. Before the workload runs, 70% of "
        "the data has been pre-filled. You need to update the parameter configuration after a fixed number of writes. "
        "First explain the reasoning, then show the option file in original format.\n"
        "The current SSD scheme is as follows: (1) A sliding window is used to identify workloads. The standard deviation "
        "of logical addresses within a fixed-size window is calculated. If the absolute change between the standard deviations "
        "of two consecutive windows exceeds a set threshold, the workload will be re-evaluated for hot and cold data classification. "
        "(2) The workload classification for hot and cold data is done using the K-means algorithm, dividing data into two categories: "
        "cold and hot. Based on the assumption that adjacent logical addresses have similar levels of activity, consecutive logical addresses "
        "are treated as a 'slice.' Therefore, the unit for K-means clustering during hot and cold data separation is a slice. The number of "
        "updates accumulated for each slice is recorded and used for the hot-cold separation process. The K-means scheme includes a max iterations "
        "parameter. Initially, after the separation, cold data is written to the QLC region, while hot data is written to the SLC region. "
		"(3) A space management scheme based on Q-learning is employed, where Q-learning is invoked after a fixed number of write operations. "
		"Based on the results from the Q-table, either garbage collection or space conversion between SLC and QLC modes is selected. "
		"The reward function in Q-learning is set to the average write latency of SLC and QLC modes (1.6ms). "
		"After each cycle, the Q-table is updated by comparing the average write latency from the previous cycle with the value of the reward function."
		"The system starts with the default configuration:\n"
        "1. Windows size: 2000;\n"
        "2. K-means trigger threshold: 10000;\n"
        "3. Max iterations: 10;\n"
        "4. Slice size: 209715200 bytes;\n"
		"5. RL training interval: 1000;\n"
		"6. RL Reward: 1.6;\n"
		"7. RL learning rate: 0.1;\n"
		"8. Conversion granularity: 1;\n"
		"9. GC granularity: 1;\n";

    std::string user_message = 
		history_config_change + 
		"\n Here's the latest information: \n" +
		performation_message +
        "The current option file is:\n"
        "1. Windows size: " + std::to_string(window_size) + ";\n"
        "2. K-means trigger threshold: " + std::to_string(change_threshold) + ";\n"
        "3. Max iterations: " + std::to_string(max_iterations) + ";\n"
        "4. Slice size: " + std::to_string(slice_size) + " bytes;\n"
		"5. RL training interval: " + std::to_string(write_period) + ";\n"
		"6. RL Reward: " + std::to_string(average_time) + ";\n"
		"7. RL learning rate: " + std::to_string(alpha) + ";\n"
		"8. Conversion granularity: " + std::to_string(conversion_granularity) + ";\n"
		"9. GC granularity: " + std::to_string(gc_granularity) + ";\n"
        "Based on these information generate a new file in the same format as the options_file to improve the SSD performance. Enclose the new options file in ```.";
	int max_tokens_value = 4096;
    float temperature_value = 0.0f;


	//std::cout << system_message << user_message << std::endl;

    // 构建 JSON 对象
    nlohmann::json request_payload = {
        {"model", model_name},
        {"messages", nlohmann::json::array({
            {
                {"role", "system"}, 
                {"content", system_message}
            },
            {
                {"role", "user"}, 
                {"content", user_message}
            }
        })},
        {"max_tokens", max_tokens_value},
        {"temperature", temperature_value}
    };

    // 打印请求 JSON 以供调试
    //std::cout << "Request JSON:\n" << request_payload.dump(2) << '\n';
	
	

    // 假设你已经导入了 OpenAI 的 SDK 并配置好 API 密钥
    auto chat = openai::chat().create(request_payload);  // 实际的 API 调用

    // 提取返回的 content 部分
    std::string content = chat["choices"][0]["message"]["content"].get<std::string>();

    //std::cout << "Extracted content: " << content << '\n';

	std::string configuration = extractNewConfiguration(content);

    // Output the extracted configuration
    if (!configuration.empty()) {
        //std::cout << "New configuration found:\n" << configuration << std::endl;
    } else {
        std::cout << "New configuration not found." << std::endl;
    }

	//extractConfigurationValues(configuration);
	config_history.push_back(extractConfigurationValues(configuration));
	performance_history.push_back(performation_message);


}

float calculateTailLatency(std::vector<float>& latencies, double percentile) {
    if (latencies.empty()) {
        return 0.0f;
    }
    
    std::sort(latencies.begin(), latencies.end());
    size_t index = static_cast<size_t>(percentile * latencies.size() / 100.0);
    return latencies[index];
}

// *************************************    Main routine
int main(int argc, char** argv){
	
	UINT64	Lba = 0;
	UINT64	Length = 2;
	std::deque<int> sliding_window;
	std::vector<double> std_dev_results;
	double last_std_dev = 0.0;

	BuildTbls();
	fprintf(stderr, "Building tables done!\n");
	srand(time(NULL));
	//srand(1);
	int SeqLba = 0;
	int j;
	int TempZone;



	//// Do a sequential write preconditioning
	float ratio = 0.7;
	for (j = 0; j < 1; j++){
		//fprintf(stderr, "MU_COUNT: %lld\n", MU_COUNT);
		SeqLba = 0;
		while (SeqLba < int(RAW_MUS*(MU_SIZE / LBA_SIZE)*ratio)) //  10485760 * 0.3 
		{
			DoHostWrite(SeqLba, (MU_SIZE / LBA_SIZE), 0);
			SeqLba += (MU_SIZE / LBA_SIZE);
			//printf("available_GCU_0: %ld, available_GCU_1: %ld\n", available_GCU_0.size(), available_GCU_1.size());
		}
		//printf("30 utilization has been written: %f (s)\n", overall_latency / 1000);
		//fprintf(stderr, "ttttttt\n");
		SeqLba = 0;
	}
	//fprintf(stderr, "Precondition is done, %f\% utilization has been written\n", ratio * 100);
	
	rl_write_length = 0;
	newwrite = 0;
	ins_time = 0;
	overall_latency = 0;
	overall_response_time = 0;
	std::string str;
	//printf("%d  %d\n", FwdTbl[0].GcuIndex, GcuTbl[FwdTbl[0].GcuIndex].Mua[353913]);
	//fprintf(stderr, "Preconditioning Complete\n");


	Prec_finish = 1;
	int hot_threshold = 0;
	int SLC_write = 0;

	//******* reading pretrained Q-Table *******//
	// ******* writing Q-Table to file *******//
	std::ifstream file3("QTable1.txt");
	int q_state_i = 0;
	int q_act_j = 0;
	
	 if (file3.is_open())
	{
		while (std::getline(file3, str))
		{
			q[q_state_i][q_act_j] = atoll(str.c_str());
			q_act_j = q_act_j + 1;

			if (q_act_j == action_max) {
				q_act_j = 0;
				q_state_i = q_state_i + 1;
			}
		}
	}
	else cout << "Unable to open file";
	file3.close();



	char* filename = argv[1];
	//filename = (char *)"usr_0.csv";
	filename = (char *)"ssdtrace.txt";

	std::ifstream file(filename);
	std::string splitter = " ";

	std::string operation = "Write";
	std::string operation2 = "W";
	UINT64 period = 6; // unit s
	period_count = 0;
	int RL_count = 0;
	//int num =0;

	if (std::string(filename) == "../LUN0.csv" || std::string(filename) == "../LUN1.csv" || std::string(filename) == "../LUN3.csv") period = 3600 * 6;

	//file.open (filename, std::ifstream::in);
	fprintf(stderr, "open file: %s  %s\n", filename, operation.c_str());
	if (file.is_open()) {
		while (std::getline(file, str))
		{
			// No#2 time, No# 11 lba, No#12 NBLKs
			std::size_t current = 0;
			std::size_t next = 0;

			//printf("%d\n", str.c_str());	
			next = str.find_first_of(splitter, current);
			std::string temp1 = str.substr(current, next - current);
			current = next + 1;
			next = str.find_first_of(splitter, current);
			UINT64 temp2 = atoi(str.substr(current, next - current).c_str());
			current = next + 1;
			next = str.find_first_of(splitter, current);
			UINT64 temp3 = atoi(str.substr(current, next - current).c_str());
			current = next + 1;
			next = str.find_first_of(splitter, current);
			UINT64 temp4 = atoi(str.substr(current, next - current).c_str());
			//UINT64 temp4 = atoll(str.substr(current, next - current).c_str());
			current = next + 1;
			next = str.find_first_of(splitter, current);
			UINT64 temp5 = atoll(str.substr(current, next - current).c_str());
			current = next + 1;


			//if (std::string(filename) == "../LUN0.csv" | std::string(filename) == "../LUN1.csv" | std::string(filename) == "../LUN3.csv") temp5 = temp5 / 100;
			//else temp5 = temp5 / 20;

			next = str.find_first_of(splitter, current);
			//UINT64 temp6 = atoll(str.substr(current, next - current).c_str());
			std::string temp6 = str.substr(current, next - current);
			current = next + 1;

			next = str.find_first_of(splitter, current);
			//UINT64 temp7 = atoi(str.substr(current, next - current).c_str());
			current = next + 1;
	
			
			if (newwrite > write_period) {
				int32_t row, randNum;
				RL_count++;

				//******** starting with RL ********//
				next_state = get_next_state(current_action, newwrite, SLC_write);
				next_state_max_q = max_value_q(&next_state);
				row = get_row_q(&current_state);

				reward = get_reward(ins_time);
				//average_time = ins_time/TotalWriteNumber*1000 + ins_time*(TotalWriteNumber-1000)/TotalWriteNumber;
				//printf("average_time: %f (s)\n", average_time);
				//printf("ins_time: %f (s)\n", ins_time);
				//average_time = 700 ;
				

				q[row][current_action] = q[row][current_action] + alpha * (reward + gamma * next_state_max_q / 10 - q[row][current_action]);
				current_state = next_state;

				randNum = get_random_number();
				if (RL_count < 100000) {
					//if (period_count < 1000) {
					if ((randNum / 100 > epsilon1) || (is_empty_q(&current_state) == 1)) { //epsilon1
						current_action = get_random_number() % (action_max);
					}
					else {
						current_action = max_action(&current_state);
					}
				}
				else {
					if ((randNum / 100 > epsilon2) || (is_empty_q(&current_state) == 1)) { //epsilon2
						current_action = get_random_number() % (action_max);
					}
					else {
						current_action = max_action(&current_state);
					}
				}
				DevCondition = current_action;

				if(DevCondition == 1){
					for (int n = 0; n < gc_granularity; n++){
						DoGc(0, 0);
					}
				}else if(DevCondition == 2 ){
					for (int n = 0; n < gc_granularity; n++){
						DoGc(0, 2);
					}
				}else if(DevCondition == 3 ){
					for (int n = 0; n < gc_granularity; n++){
						DoGc(2, 2);
					}
				}else if(DevCondition == 4){
					for (int n = 0; n < conversion_granularity; n++){
						Conversion2(0,2);
					}
				}

				newwrite = 0;
				ins_time = 0;
				rl_write_length = 0;
			}


			if (operation == temp1 || operation2 == temp1) { //write operation
				current_write_latency = 0.0;
				//TotalWriteSize += temp5 * 512; 
				Lba = temp2 % MU_COUNT; //4194304 - 1 = 4194303
				Length = temp4 *512 / LBA_SIZE;
				if(Length == 0){
					Length = 1;
				}

				sliding_window.push_back(Lba);
				if (sliding_window.size() > window_size) {
            		sliding_window.pop_front();
        		}

				if (sliding_window.size() == window_size) {
            		double std_dev = calculateCurrentStd(sliding_window);
					std_dev_results.push_back(std_dev);
           	 		//std::cout << "Current Standard Deviation: " << std_dev << std::endl;

					if (last_std_dev > 0 && std::abs(std_dev - last_std_dev) >= change_threshold) {
						hot_threshold = slice_clusting();
						slice_clusting_kmeans();
						period_count = period_count + 1;
						/*
						for (i = 0; i < NUM_Slice ; i++){
							slice_access[i] = slice_access[i] * 0.2;
						}
						*/
            		}
					last_std_dev = std_dev;
				}
				TotalWriteNumber += 1;
				newwrite = newwrite + Length;
				total_write_length += Length;
				newwritelength += Length;
				rl_write_length += Length;
				UINT64 slice_number = floor(temp2 / Slice_SIZE); //属于哪个slice
				if (slice_number >= NUM_Slice - 1) slice_number = NUM_Slice - 1;

				slice_access[slice_number] = slice_access[slice_number] + Length;

				/*
				if (slice_access[slice_number] > hot_threshold){
					TempZone = 0; // hot --> SLC_H
				} else{
					TempZone = 1; //cold --> SLC_C
				}*/

				TempZone = recognition[slice_number];
				
				if(dataAllocation == 0){
					if (TempZone == 1){
						TempZone = 2;
					}
				}
				DoHostWrite(Lba, Length, TempZone); 

				/*
				if (TotalWriteNumber % gpt_period == 0){
					current_overall_latency = (overall_latency - last_overall_latency) / 1000;
					current_period_count = period_count - last_period_count;
					current_NumberGc = NumberGc - last_NumberGc;
					current_Numberconvert = Numberconvert - last_Numberconvert;
					current_newwritelength = newwritelength - last_newwritelength;
					current_TotalReadNumber = TotalReadNumber - last_TotalReadNumber;
					current_slc_ratio = ( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() ) / (float)GCU_COUNT ;

					printf("\n-----test %lld----\n", TotalWriteNumber);
					//printf("Total read time: %f (s)\n", overall_read_latency / 1000);
					printf("Total execution time: %f (s)\n", (overall_latency - last_overall_latency) / 1000);
					printf("period_count: %lld\n", period_count - last_period_count);
					printf("NumberGc: %lld\n", NumberGc - last_NumberGc);
					printf("Numberconvert: %lld\n", Numberconvert - last_Numberconvert);
					printf("newwritelength: %d\n", newwritelength - last_newwritelength);
					printf("TotalReadNumber: %lld\n", TotalReadNumber - last_TotalReadNumber);
					printf("available_GCU_0: %ld, available_GCU_1: %ld\n", available_GCU_0.size(), available_GCU_1.size());
					printf("SLC ratio: %f\n\n", ( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() ) / (float)GCU_COUNT);
					//printf("SLC invalid ratio: %lld\n", (CurOpByZone[0]+CurOpByZone[1]) / ( ZoneCount[0] + ZoneCount[1] + available_GCU_0.size() ) / 256 );
					
					last_overall_latency = overall_latency;
					last_period_count = period_count;
					last_NumberGc = NumberGc;
					last_Numberconvert = Numberconvert;
					last_newwritelength = newwritelength;
					last_TotalReadNumber = TotalReadNumber;
					
					requestGPT(TotalWriteNumber, current_overall_latency, current_period_count, current_NumberGc, current_Numberconvert, current_newwritelength, current_TotalReadNumber, current_slc_ratio);

					//window_size = 3000;
					//change_threshold = 15000;
				}*/

				latencies.push_back(current_write_latency);
				current_write_latency = 0.0;

			}else { //read operation
				//TotalReadSize += temp5 * 512;
				Lba = temp2 % MU_COUNT; //4194304 - 1 = 4194303
				Length = temp4 *512 / LBA_SIZE;
				if(Length == 0){
					Length = 1;
				}
				TotalReadNumber+=Length;
				for (long long unsigned int i = 0; i < Length; i++) {
					overall_latency = overall_latency + RdLatency[MapTBL[Lba + i].ZoneNum];
					// ins_time = ins_time + RdLatency[MapTBL[Lba + i].ZoneNum];
					overall_response_time = overall_response_time + RdLatency[MapTBL[Lba + i].ZoneNum];
					//printf("%d\n", MapTBL[Lba + i].ZoneNum);
					overall_read_latency = overall_read_latency + RdLatency[MapTBL[Lba + i].ZoneNum];
					//responseTime[period_count] = responseTime[period_count] + RdLatency[MapTBL[Lba + i].ZoneNum];
				}
			}
		}
	}
	else printf("Can't open file \n");
	file.close();


	std::ofstream file2("QTable2.txt");
	if (file2.is_open())
	{
		for (int i = 0; i < state_max; i++) {
			for (int j = 0; j < action_max; j++) {
				file2 << q[i][j] << endl;
			}
		}
	}
	else cout << "Unable to open file";
	file2.close();

	printf("Total read time: %f (s)\n", overall_read_latency / 1000);
	printf("Total execution time: %f (s)\n", overall_latency / 1000);
	printf("Total response time: %f (s)\n", overall_response_time / 1000);
	printf("period_count: %lld\n", period_count);
	printf("NumberGc: %lld\n", NumberGc);
	printf("TotalWriteNumber: %lld\n", TotalWriteNumber);
	printf("TotalWriteLength: %lld\n", total_write_length);
	printf("RealWriteLength: %lld\n", real_write_length);
	printf("WA: %f (s)\n", real_write_length / total_write_length);
	float tailLatency = calculateTailLatency(latencies, percentile);
    std::cout << "99.99% tail latency: " << tailLatency << " microseconds" << "\n" << std::endl;
	float tailLatency1 = calculateTailLatency(latencies, percentile1);
    std::cout << "99.99% tail latency: " << tailLatency1 << " microseconds" << "\n" << std::endl;

	
	for (size_t i = 0; i < config_history.size(); ++i) {
        std::cout << "Configuration " << i + 1 << ":\n";
        std::cout << "Windows size: " << config_history[i].windows_size << "\n";
        std::cout << "K-means trigger threshold: " << config_history[i].kmeans_threshold << "\n";
        std::cout << "Max iterations: " << config_history[i].max_iterations << "\n";
        std::cout << "Slice size: " << config_history[i].slice_size << " bytes\n";
        std::cout << "RL training interval: " << config_history[i].rl_training_interval << "\n";
        std::cout << "RL Reward: " << config_history[i].rl_reward << "\n";
        std::cout << "RL learning rate: " << config_history[i].rl_learning_rate << "\n";
        std::cout << "Conversion granularity: " << config_history[i].conversion_granularity << "\n";
        std::cout << "GC granularity: " << config_history[i].gc_granularity << "\n\n";
    }

	/*
    auto chat = openai::chat().create(R"(
    {
        "model": "gpt-3.5-turbo",
        "messages":[{"role":"user", "content":"say hi, sweety."}],
        "max_tokens": 17,
        "temperature": 0
    }
    )"_json);
    //std::cout << "Response is:\n" << chat.dump(2) << '\n'; 

	// 提取 content 部分
    std::string content = chat["choices"][0]["message"]["content"].get<std::string>();

    // 输出提取到的 content
    std::cout << content << std::endl;
	*/

	//std::string model_name = "gpt-3.5-turbo";
	
	return 0;
}
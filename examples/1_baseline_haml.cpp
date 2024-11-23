//

#include <iostream>

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
int 			total_write_length = 0;
int 			real_write_length = 0;
std::vector<float>  latencies;
const double percentile = 99.00; // 99.99%的尾延迟
const double percentile1 = 99.99; // 99.99%的尾延迟
float 			current_write_latency = 0.0;

// *************************************     Configurable parameters
#define		LBA_SIZE			16384  //512
#define		MU_SIZE				16384  //512
#define		Slice_SIZE			209715200 //1024*1024*200  //200 MB -- 209715200
//#define		Slice_SIZE			52428800 //1024*1024*200  //200 MB -- 209715200

#define		NUM_TEMP_ZONES		3
#define		LOG_CAP				(UINT64)34359738368 //32GB
//page number = 34359738368 / 16384 = 2097152-----32GB
#define		EFF_OP				1  //  1//1.3//1.5//1.5F //1 //1.63F
#define		MIN_RDY				6
//#define		MIN_RDY				24

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

UINT64			Num_Used_Mu = 0;
UINT64			Total_Mu = 0;
UINT64			Total_Mu_w = 2097152;
UINT64			Num_Tot_Gcu = 0;
UINT64			DevCondition = 1;
UINT64			curGCUIndex = 0;

UINT32			write_period = 50000;

//UINT32			write_period = 10000;

UINT32			window_size = 2000;
UINT32			change_threshold = 10000;
UINT32			newwrite = 0;
UINT32			newwritelength = 0;
float			average_time = 0;
std::vector<int> available_GCU;
std::vector<int> available_GCU_0; //未分配类型SLC的block索引列表
std::vector<int> available_GCU_1; //未分配类型MLC的block索引列表
std::vector<double> rolling_std;
UINT64			Do_GC_SLC_QLC = 0;

float 			last_overall_latency = 0.0;
int 			last_period_count = 0;
int 			last_NumberGc = 0;
int 			last_Numberconvert = 0;
int 			last_newwritelength = 0;
int 			last_TotalReadNumber = 0;
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
		memset(CurZoneGcu[i], 0, sizeof(CurZoneGcu[i]));
		ZoneUsedblocks[i] = 1;
	}

	memset(FwdTbl, 0, sizeof(FwdTbl));
	memset(MapTBL, 0, sizeof(MapTBL));

	for (i = 0; i < NUM_Slice; i++)
	{
		slice_access[i] = 0;
		//recognition[i] = i % 2; // 初始是平分
		recognition[i] = 0;
	}

	for (i = 0; i < GCU_COUNT; i++) //2048
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
		else if (i < ZoneSpace[0]) {
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
	bool	OpEnforced = false;
	


	for( i = 0; i<GCU_COUNT; i++ )
	{
		if ( GcuTbl[i].State == STATE_USED )
		{
			if (SelectZone == 0){
				if ((GcuTbl[i].Zone == 0) | (GcuTbl[i].Zone == 1)){
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
					OpEnforced = true;
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
	bool	DidGc = false;
	UINT32	EvictionZone;
	UINT32  NewZone = 0;
	
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
			current_write_latency +=  RdLatency[Zone1] + WtLatency[Zone2];
			real_write_length += 1;
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
		if ( Zone1 == 0 | Zone1 == 1 ){
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
	bool	DidGc = false;
	UINT32	EvictionZone;
	UINT32  NewZone = 0;
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
			current_write_latency +=  RdLatency[Zone1] + WtLatency[Zone2];
			real_write_length += 1;
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
	UINT64	LengthRemain;

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
			MapTBL[StartMua + i].Mapped == true;
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
		current_write_latency += WtLatency[Zone];
		real_write_length += 1;
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
	int min = 10000;
	int max = 0;
	for (int i = 0; i < NUM_Slice; i++)
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
	int has_name = 0;

	vector<Point> points;
	string point_name;

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
	UINT64	i;
	std::deque<int> sliding_window;
	std::vector<double> std_dev_results;
	double last_std_dev = 0.0;

	BuildTbls();
	fprintf(stderr, "Building tables done!\n");
	srand(time(NULL));
	//srand(1);
	unsigned long long randLongLong = 0;
	unsigned long long SeqLba = 0;
	int j;
	int TempZone;



	//// Do a sequential write preconditioning
	float ratio = 0.7;
	for (j = 0; j < 1; j++){
		//fprintf(stderr, "MU_COUNT: %lld\n", MU_COUNT);
		SeqLba = 0;
		while (SeqLba < int(RAW_MUS*(MU_SIZE / LBA_SIZE)*ratio)) //  10485760 * 0.3 
		{
			int tempxzone = 0;
			DoHostWrite(SeqLba, (MU_SIZE / LBA_SIZE), 0);
			SeqLba += (MU_SIZE / LBA_SIZE);
			//printf("available_GCU_0: %ld, available_GCU_1: %ld\n", available_GCU_0.size(), available_GCU_1.size());
		}
		//printf("30 utilization has been written: %f (s)\n", overall_latency / 1000);
		//fprintf(stderr, "ttttttt\n");
		SeqLba = 0;
	}
	//fprintf(stderr, "Precondition is done, %f\% utilization has been written\n", ratio * 100);
	
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


	char* filename = argv[1];
	filename = (char *)"usr_0.csv";
	//filename = (char *)"ssdtrace.txt";

	std::ifstream file(filename);
	std::string splitter = ",";

	std::string operation = "Write";
	std::string operation2 = "W";
	UINT64 period = 6; // unit s
	period_count = 0;
	int RL_count = 0;
	int num =0;

	if (std::string(filename) == "../LUN0.csv" | std::string(filename) == "../LUN1.csv" | std::string(filename) == "../LUN3.csv") period = 3600 * 6;

	//file.open (filename, std::ifstream::in);
	fprintf(stderr, "open file: %s  %s\n", filename, operation.c_str());
	if (file.is_open()) {
		while (std::getline(file, str))
		{
			// No#2 time, No# 11 lba, No#12 NBLKs
			//cout << str << endl;
			std::size_t current = 0;
			std::size_t next = 0;

			// FIU: [ts in ns] [pid] [process] [lba] [size in 512 Bytes blocks] [Write or Read] [major device number] [minor device number] [MD5 per 4096 Bytes]
			//       0 4892 syslogd 904265560 8 W 6 0 531e779e
			//       lba --- TEMP4   ;   size in 512 Bytes blocks --- TEMP5  ;  opration --- TEMP6

			//OLTP: 8,21888,32768,r,2077.065185
			//       lba --- TEMP2   ;   size  --- TEMP3  ;  opration --- TEMP4

			//ssd trace: W 282624 + 8 [0]
			//       lba --- TEMP2   ;   size  --- TEMP4  ;  opration --- TEMP1

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
			std::string temp4 = str.substr(current, next - current);
			current = next + 1;
			next = str.find_first_of(splitter, current);
			UINT64 temp5 = atoll(str.substr(current, next - current).c_str());
			current = next + 1;


			//if (std::string(filename) == "../LUN0.csv" | std::string(filename) == "../LUN1.csv" | std::string(filename) == "../LUN3.csv") temp5 = temp5 / 100;
			//else temp5 = temp5 / 20;

			next = str.find_first_of(splitter, current);
			UINT64 temp6 = atoll(str.substr(current, next - current).c_str());
			//std::string temp6 = str.substr(current, next - current);
			current = next + 1;

			next = str.find_first_of(splitter, current);
			UINT64 temp7 = atoi(str.substr(current, next - current).c_str());
			current = next + 1;
			
			
			if (newwrite > write_period) {
				hot_threshold = slice_clusting();
				slice_clusting_kmeans();
				period_count = period_count + 1;
				newwrite = 0;
				ins_time = 0;
			}


			if (operation == temp4 | operation2 == temp4) { //write operation
				current_write_latency = 0.0;
				//TotalWriteSize += temp5 * 512; 
				//Lba = temp5 % MU_COUNT; //4194304 - 1 = 4194303
				Lba = temp5 % MU_COUNT;
				Length = temp6 / LBA_SIZE;
				newwrite = newwrite + Length;
				if(Length == 0){
					Length = 1;
				}
				TotalWriteNumber += 1;
				newwritelength += Length;
				total_write_length += Length;
				UINT64 slice_number = floor(temp5 / Slice_SIZE); //属于哪个slice
				if (slice_number >= NUM_Slice - 1) slice_number = NUM_Slice - 1;

				slice_access[slice_number] = slice_access[slice_number] + Length;

				/*
				if (slice_access[slice_number] > hot_threshold){
					TempZone = 0; // hot --> SLC_H
				} else{
					TempZone = 1; //cold --> SLC_C
				}*/

				TempZone = recognition[slice_number];
				if (TempZone == 1){
					TempZone = 2;
				}
				DoHostWrite(Lba, Length, TempZone); 
				latencies.push_back(current_write_latency);
				current_write_latency = 0.0;
			}else { //read operation
				//TotalReadSize += temp5 * 512;
				//Lba = temp5 % MU_COUNT; //4194304 - 1 = 4194303
				Lba = temp5 % MU_COUNT;
				Length = temp6 / LBA_SIZE;
				if(Length == 0){
					Length = 1;
				}
				TotalReadNumber+=Length;
				for (int i = 0; i < Length; i++) {
					overall_latency = overall_latency + RdLatency[MapTBL[Lba + i].ZoneNum];
					ins_time = ins_time + RdLatency[MapTBL[Lba + i].ZoneNum];
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


	printf("Total read time: %f (s)\n", overall_read_latency / 1000);
	printf("Total execution time: %f (s)\n", overall_latency / 1000);
	printf("Total response time: %f (s)\n", overall_response_time / 1000);
	printf("period_count: %lld\n", period_count);
	printf("NumberGc: %lld\n", NumberGc);
	printf("TotalWriteNumber: %lld\n", TotalWriteNumber);
	printf("TotalWriteLength: %lld\n", total_write_length);
	printf("RealWriteLength: %lld\n", real_write_length);
	printf("WA: %.2f\n", (double)real_write_length / total_write_length);
	float tailLatency = calculateTailLatency(latencies, percentile);
    std::cout << "99% tail latency: " << tailLatency << " microseconds" << "\n" << std::endl;
	float tailLatency1 = calculateTailLatency(latencies, percentile1);
    std::cout << "99.99% tail latency: " << tailLatency1 << " microseconds" << "\n" << std::endl;

	return 0;
}
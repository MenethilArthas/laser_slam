#ifndef MAPPER_H_
#define MAPPER_H_
#include<vector>
#include"types.h"

class OccupancyGrid
{
public:
	OccupancyGrid(int width,int height,Pose2 originPose,float resolution)
	:m_width(width),m_height(height),m_originPose(originPose),m_resolution(resolution)
	{
		m_originPoint.SetX(originPose.GetX()/resolution);
		m_originPoint.SetY(originPose.GetY()/resolution);
		m_size=m_width*m_height;
		m_data=new int8_t[m_width*m_height];
		memset(m_data,girdStatus_Unknow,m_width*m_height);
	}
	OccupancyGrid(float resolution):m_resolution(resolution)
	{}
	~OccupancyGrid()
	{
		delete m_data;
	}
	void SetMapValue(int index,uint8_t value)
	{
		m_data[index]=value;
	}
	inline int8_t GetMapValue(int index)
	{
		return m_data[index];
	}
	inline void SetOriginPose(Pose2 rOriginPose)
	{
		m_originPose=rOriginPose;
		m_originPoint.SetX(rOriginPose.GetX()/m_resolution);
		m_originPoint.SetY(rOriginPose.GetY()/m_resolution);
	}
	inline Pose2 GetOriginPose()	const
	{
		return m_originPose;
	}
	inline Point GetOriginPoint()	const
	{
		return m_originPoint;
	}
// 	inline Point GetOdomPoint()	const
// 	{
// 		return m_odomPoint;
// 	}
	inline int GetGridIndex(Point readingPoint)
	{		
		int disX=std::abs(readingPoint.GetX()-m_originPoint.GetX());
		int disY=std::abs(readingPoint.GetY()-m_originPoint.GetY());
		return (disY*m_width+disX);
	}
	inline void SetWidth(int rWidth)
	{
		m_width=rWidth;
	}
	inline void SetHeight(int rHeight)
	{
		m_height=rHeight;
	}
	inline int GetWidth()	const
	{
		return m_width;
	}
	inline int GetHeight()	const
	{
		return m_height;
	}
	inline void Resize(int rSize)
	{
		m_data=new int8_t[rSize];
		memset(m_data,girdStatus_Unknow,rSize);
	}
	inline int GetSize()	const
	{
		return m_size;
	}
	inline int8_t* GetDataPtr()	const
	{
		return m_data;
	}
	inline float GetResolution() const
	{
		return m_resolution;
	}
private:
	Pose2 m_originPose;
	Point m_originPoint; //relate to world frame
	//Point m_odomPoint; //relate to world frame
	float m_resolution;  //[m/cell]
	int m_width;
	int m_height;
	int m_size;
	int8_t *m_data;
};

class LinearFunc
{
public:
	LinearFunc (Point A,Point B);	
	double CalcResult(int var);
	int GetType();
private:
	int m_type;//为0代表横向迭代，即给定x求y；为1代表纵向迭代，即给定y求x；为2代表垂直于x轴；为3代表垂直于y轴
	double m_k;
	double m_b;
	int m_fixedX;
};

class LocalizedScan
{
public:
	LocalizedScan(VectorFloat readings,Pose2 odomPose/*,VectorPoint readingPoints,OccupancyGrid *occGrid*/)
	:m_readings(readings),m_odomPose(odomPose)/*,m_readingsPoints(readingPoints),m_occGrid(occGrid)*/
	{
// 		m_originPose.SetX(m_occGrid->GetOriginPoint().GetX()*m_occGrid->GetResolution());
// 		m_originPose.SetY(m_occGrid->GetOriginPoint().GetY()*m_occGrid->GetResolution());
	}
	~LocalizedScan()
	{
		//delete m_occGrid;
	}
	inline Pose2 GetOdomPose()
	{
		return m_odomPose;
	}
	inline Pose2 GetCorrectedPose()
	{
		return m_correctedPose;
	}
	inline void SetOdomPose(Pose2 odomPose)
	{
		m_odomPose=odomPose;
	}
	inline void SetCorrectedPose(Pose2 correctedPose)
	{
		m_correctedPose=correctedPose;
	}
// 	inline Pose2 GetOriginPose()	const
// 	{
// 		return m_originPose;
// 	}
	inline VectorFloat GetReadings()
	{
		return m_readings;
	}
private:
	//Pose2 m_originPose;

	Pose2 m_odomPose;
	Pose2 m_correctedPose;
	//OccupancyGrid *m_occGrid;
	VectorFloat m_readings;
	//VectorPoint m_readingsPoints; 
public:
    static float m_angleIncrement;
	static float m_maxRange;
	static float m_minRange;
};
typedef std::vector<LocalizedScan*> VectorLocalizedScan;

typedef struct CellBlock
{
public:
	inline CellBlock():averagePoint(),pointNum(0),m_occupiedFlag(false){}
	inline void AddFallInPoint(Point fallInPoint)
	{
		averagePoint+=fallInPoint;
		pointNum++;
		averagePoint/=pointNum;
	}
	inline double Score(Point rPoint)
	{
		int distance=averagePoint.GetSquaredDistance(rPoint);
		return std::exp(1.0/(distance+1));
	}

	bool m_occupiedFlag;
	Point averagePoint;
	int pointNum;
	static int size;
}cellBlock;

class Cell
{
public:
	Cell():m_pData(nullptr){}
	Cell(int width,int height,Point originPoint):m_width(width),m_height(height),m_originPoint(originPoint)
	{
		m_pData=new cellBlock[width*height];
	}
	Cell(const Cell& rOther)
	:m_width(rOther.m_width),m_height(rOther.m_height),m_originPoint(rOther.m_originPoint),m_pData(rOther.m_pData)
	{
		
	}
	~Cell()
	{
		delete m_pData;
	}
	inline void AddPoint(Point fallInPoint)
	{
		int index=0;
		if(GetCellIndex(fallInPoint,index)==true)
		{
			if(m_pData[index].m_occupiedFlag!=true)
				m_pData[index].m_occupiedFlag=true;
			m_pData[index].AddFallInPoint(fallInPoint);		
		}			
	}
	inline double GetScore(int index,Point rPoint)
	{
		if(m_pData[index].m_occupiedFlag==true)
			return m_pData[index].Score(rPoint);
		else
			return 0.0;
	}
	inline bool GetCellIndex(Point point,int& index)
	{
		int cellDisX= (point.GetX()-m_originPoint.GetX())/cellBlock::size;
		int cellDisY= (point.GetY()-m_originPoint.GetY())/cellBlock::size;
		//check boundary
		if(cellDisX>=m_width||cellDisX<0||cellDisY>=m_height||cellDisY<0)
			return false;

		index=cellDisY*m_width+cellDisX;
		return true;
	}
	inline const Cell& operator = (const Cell& rOther )
	{
		m_width=rOther.m_width;
		m_height=rOther.m_height;
		m_originPoint=rOther.m_originPoint;
		m_pData=new cellBlock[m_width*m_height];
		return *this;
	}
	inline void InitCellBlocks()
	{
		m_pData=new cellBlock[m_width*m_height];
	}
	inline void SetWidth(int width)
	{
		m_width=width;
	}
	inline void SetHeight(int height)
	{
		m_height=height;
	}
	inline void SetOriginPoint(Point point)
	{
		m_originPoint=point;
	}

private:
	int m_width;
	int m_height;
	Point m_originPoint;
	cellBlock* m_pData;
};


class Mapper
{
public:
	Mapper():m_pLastScan(nullptr),m_runningScanMaxSize(60),m_runningScanMaxSquaredDistance(3.0),
	m_minTravelSquaredDistance(0.04),m_minTravelHeading(0.174),m_resolution(0.05)
	{
		
	}
	inline LocalizedScan* GetLastScanPtr()	const
	{
		return m_pLastScan;
	}
	inline VectorLocalizedScan GetRunningScans()
	{
		return m_runningScans;
	}
	inline VectorLocalizedScan GetProcessedScans()
	{
		return m_processedScans;
	}
	inline void SetLastScanPtr(LocalizedScan* pLastScan)
	{
		m_pLastScan=pLastScan;
	}
	inline float GetResolution()
	{
		return m_resolution;
	}
	void GetCellBlocks(Cell& cell);
	void FillCellBlocks(Cell* cell);
	bool Process(LocalizedScan* rScan);
	bool HasMovedEnough(LocalizedScan* rScan,LocalizedScan* rLastScan);
	double ScanMatch(LocalizedScan* rScan,Pose2& rbestPose,Matrix3& rCovariance);
	void AddRunningScan(LocalizedScan* rScan);
	void AddProcessedScan(LocalizedScan* rScan);
	double Score(Cell* cell,Pose2 rPose,LocalizedScan* rScan);
	void CreateMap(OccupancyGrid& rOccGrid);
private:
	double m_minTravelSquaredDistance;
	double m_minTravelHeading;
	double m_runningScanMaxSquaredDistance;
	float m_resolution;
	int m_runningScanMaxSize;
	LocalizedScan* m_pLastScan;
	VectorLocalizedScan m_runningScans;
	VectorLocalizedScan m_processedScans;
};


#endif
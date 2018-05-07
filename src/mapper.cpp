#include "types.h"
#include "mapper.h"
#include "ros/ros.h"
#include <iostream>

LinearFunc::LinearFunc (Point A,Point B)
{
	/*vertical to x axis*/
	if(A.GetX()==B.GetX())
	{
		m_k=0.0;
		m_b=0.0;
		m_fixedX=A.GetX();
		m_type=2;
	}
	/*vertical to y axis*/
	else if(A.GetY()==B.GetY())
	{
		m_k=0.0;
		m_b=A.GetY();
		m_type=3;
	}
	else
	{
		m_k = (double)(B.GetY() - A.GetY()) / (double)(B.GetX() - A.GetX());
		m_b = B.GetY() - m_k * B.GetX();
		if ((m_k) < 1)
			m_type = 0;
		else
			m_type = 1;
	}	

}
double LinearFunc::CalcResult(int var)
{
	if (m_type == 0)
		return (m_k * var +m_b);
	else if(m_type==1)
		return (var - m_b) / m_k;
	else if(m_type==2)
		return m_fixedX;
	else if(m_type==3)
		return m_b;			
}
int LinearFunc::GetType()
{
	return m_type;
}


void Mapper::AddRunningScan(LocalizedScan* rScan)
{
	m_runningScans.push_back(rScan);
	Pose2 startPose=m_runningScans.front()->GetCorrectedPose();
	Pose2 endPose=m_runningScans.back()->GetCorrectedPose();
	double squaredDistance=startPose.GetSquaredDistance(endPose);
	while(m_runningScans.size()>m_runningScanMaxSize||squaredDistance>m_runningScanMaxSquaredDistance)
	{
		m_runningScans.erase(m_runningScans.begin());
		startPose=m_runningScans.front()->GetCorrectedPose();
		endPose=m_runningScans.back()->GetCorrectedPose();
		squaredDistance=startPose.GetSquaredDistance(endPose);
	}	
}
void Mapper::AddProcessedScan(LocalizedScan* rScan)
{
	m_processedScans.push_back(rScan);
}

bool Mapper::Process(LocalizedScan* rScan)
{	
	static bool updateFlag=true;
	LocalizedScan* lastScan=GetLastScanPtr();
	if(lastScan==nullptr)
	{			
		rScan->SetCorrectedPose(rScan->GetOdomPose());
		AddRunningScan(rScan);
		AddProcessedScan(rScan);
		SetLastScanPtr(rScan);		
	}
	else
	{
		rScan->SetCorrectedPose(rScan->GetOdomPose()+(lastScan->GetCorrectedPose()-lastScan->GetOdomPose()));
		if(HasMovedEnough(rScan,lastScan)&&(updateFlag==true))
		{
// 			std::cout<<"now pose."<<rScan->GetCorrectedPose().GetX()<<"\t"<<rScan->GetCorrectedPose().GetY()
// 			<<"\t"<<rScan->GetOdomPose().GetHeading()<<std::endl;
// 			std::cout<<"last pose."<<lastScan->GetCorrectedPose().GetX()<<"\t"<<lastScan->GetCorrectedPose().GetY()
// 			<<"\t"<<lastScan->GetCorrectedPose().GetHeading()<<std::endl;
			
// 			ROS_INFO("robot has moved enough.");
			Pose2 bestPose;
			Matrix3 covariance;
			covariance.SetToIdentity();
			ScanMatch(rScan,bestPose,covariance);
			rScan->SetCorrectedPose(bestPose);
			SetLastScanPtr(rScan);		
			AddRunningScan(rScan);
			AddProcessedScan(rScan);
			//updateFlag=false;
		}
		else			
		{
			return false;
		}
		
	}	
	return true;
}

bool Mapper::HasMovedEnough(LocalizedScan* rScan, LocalizedScan* rLastScan)
{
	Pose2 newOdomPose=rScan->GetCorrectedPose();
	Pose2 lastOdomPose=rLastScan->GetCorrectedPose();
	double travelDistance=newOdomPose.GetSquaredDistance(lastOdomPose);
	double travelHeading=std::abs(newOdomPose.GetHeading()-lastOdomPose.GetHeading());
	//ROS_INFO("distance=%f,heading=%f",travelDistance,travelHeading);
	if(travelDistance>m_minTravelSquaredDistance)
		return true;
	else if(NormalizeAngle(travelHeading)>m_minTravelHeading)
		return true;
	else
		return false;
}


enum MOVE
{
	FORWARD=1,
	BACKWARD,
	LEFT,
	RIGHT,
	ROTATE_LEFT,
	ROTATE_RIGHT
};
double Mapper::ScanMatch(LocalizedScan* rScan,Pose2& rbestPose,Matrix3& rCovariance)
{
	Pose2 bestPose=rScan->GetCorrectedPose();
	Pose2 movePose;
	Pose2 bestMovePose;
	double searchPosStep=0.2;
	double searchAngleStep=0.07;
	int maxIterations=5;
	int iterations=0;
	double bestScore=0.0;
	double maxMoveScore=0.0;
	double moveScore=0.0;
	int move;
	Cell cell;
	GetCellBlocks(cell);
	FillCellBlocks(&cell);
	bestScore=Score(&cell,bestPose,rScan);
	
	Pose2 newPose(0.0 , 0.0 , 0.0);
	ros::Time lastTime=ros::Time::now();
	
	while(iterations<maxIterations)
	{
		maxMoveScore=bestScore;
		std::cout<<"iter="<<iterations<<std::endl;
 		std::cout<<"maxMoveScore="<<maxMoveScore<<"\tsearchAngleStep="<<searchAngleStep<<"\tsearchPosStep="<<searchPosStep<<std::endl;
		for(move=FORWARD;move!=(ROTATE_RIGHT+1);move++)
		{
			switch(move)
			{
				case FORWARD:
 					std::cout<<"forward=";
					newPose.SetX(searchPosStep);
					movePose=bestPose+newPose;
					break;
				case BACKWARD:
 					std::cout<<"backward=";
					newPose.SetX(-searchPosStep);
					movePose=bestPose+newPose;
					break;
				case LEFT:
					std::cout<<"left=";
					newPose.SetY(searchPosStep);
					movePose=bestPose+newPose;
					break;
				case RIGHT:
 					std::cout<<"right=";
					newPose.SetY(-searchPosStep);
					movePose=bestPose+newPose;
					break;
				case ROTATE_LEFT:
 					std::cout<<"rotate_left=";
					newPose.SetHeading(searchAngleStep);
					movePose=bestPose+newPose;
					break;
				case ROTATE_RIGHT:
					std::cout<<"rotate_right=";
					newPose.SetHeading(-searchAngleStep);
					movePose=bestPose+newPose;
					break;
			}
			moveScore=Score(&cell,movePose,rScan);
 			std::cout<<moveScore<<std::endl;
			if(moveScore>maxMoveScore)
			{
				maxMoveScore=moveScore;
				bestMovePose=movePose;
			}
			newPose.SetX(0.0);
			newPose.SetY(0.0);
			newPose.SetHeading(0.0);
		}
		if(maxMoveScore>bestScore)
		{
			bestScore=maxMoveScore;
			bestPose=bestMovePose;
		}
		else
		{
			searchAngleStep/=2;
			searchPosStep/=2;
			iterations++;
		}	
	
	}
	ros::Time nowTime=ros::Time::now();
	rbestPose=bestPose;
 	ROS_INFO("bestPose:x=%f,y=%f,angle=%f\r\n",bestPose.GetX(),bestPose.GetY(),bestPose.GetHeading());
	ROS_INFO("scan match time=%f",(nowTime-lastTime).toSec());
	return bestScore;
}

void Mapper::GetCellBlocks(Cell& cell)
{
	float poseX2World,poseY2World;
	Pose2 correctedPose;
	float angle=0.0;
	float minX=LocalizedScan::m_maxRange,minY=LocalizedScan::m_maxRange,maxX=0.0,maxY=0.0;
	int	minPointX=0;
	int	minPointY=0;
	Point originPoint;
	int gridWidth=0;
	int gridHeight=0;
	int cellWidth=0;
	int cellHeight=0;
	float angleIncrement=LocalizedScan::m_angleIncrement;
	for(std::vector<LocalizedScan*>::const_iterator itLocalizedScan=m_runningScans.begin() ; itLocalizedScan!=m_runningScans.end() ; itLocalizedScan++)
	{
		correctedPose=(*itLocalizedScan)->GetCorrectedPose();
		VectorFloat	readings=(*itLocalizedScan)->GetReadings();
		for(std::vector<float>::const_iterator itReading=readings.begin();itReading!=readings.end();itReading++)
		{
			if((*itReading)<=LocalizedScan::m_maxRange&&(*itReading)>=LocalizedScan::m_minRange)
			{
				poseX2World=-(*itReading) * std::cos(angle + correctedPose.GetHeading()) +correctedPose.GetX();
				poseY2World=-(*itReading) * std::sin(angle + correctedPose.GetHeading()) + correctedPose.GetY();
				if(poseX2World<minX)
					minX=poseX2World;
				else if(poseX2World>maxX)
					maxX=poseX2World;
				if(poseY2World<minY)
					minY=poseY2World;
				else if(poseY2World>maxY)
					maxY=poseY2World;
			}
			angle+=angleIncrement;
		}
		angle=0.0;
	}
	//std::cout<<"maxX="<<maxX<<"\tmaxY="<<maxY<<"\tminX="<<minX<<"\tminY="<<minY<<std::endl;	
	minPointX=(int)(minX/m_resolution);
	minPointY=(int)(minY/m_resolution);
	originPoint.SetX(minPointX);
	originPoint.SetY(minPointY);
	gridWidth=(int)((maxX-minX)/m_resolution);
	gridHeight=(int)((maxY-minY)/m_resolution);
	
	if(gridWidth%cellBlock::size==0)
		cellWidth=gridWidth/(cellBlock::size);
	else 
		cellWidth=gridWidth/(cellBlock::size)+1;
	if(gridHeight%cellBlock::size==0)
		cellHeight=gridHeight/(cellBlock::size);
	else 
		cellHeight=gridHeight/(cellBlock::size)+1;
	//ROS_INFO("gridWidth=%d\t,gridHeight=%d\t,cellWidth=%d\t,cellHeight=%d",gridWidth,gridHeight,cellWidth,cellHeight);
	cell.SetOriginPoint(originPoint);
	cell.SetHeight(cellHeight);
	cell.SetWidth(cellWidth);
	cell.InitCellBlocks();
	
}

void Mapper::FillCellBlocks(Cell* cell)
{
	float poseX2World,poseY2World;
	Pose2 correctedPose;
	Point hitPoint;
	float angle=0.0;
	Point originPoint;
	float angleIncrement=LocalizedScan::m_angleIncrement;
	std::cout<<"running scan size="<<m_runningScans.size()<<std::endl;
	for(std::vector<LocalizedScan*>::const_iterator itLocalizedScan=m_runningScans.begin() ; itLocalizedScan!=m_runningScans.end() ; itLocalizedScan++)
	{
		correctedPose=(*itLocalizedScan)->GetCorrectedPose();
		VectorFloat readings=(*itLocalizedScan)->GetReadings();
		for(std::vector<float>::const_iterator itReading=readings.begin();itReading!=readings.end();itReading++)
		{
			if((*itReading)<=LocalizedScan::m_maxRange&&(*itReading)>=LocalizedScan::m_minRange)
			{
				poseX2World=-(*itReading) * std::cos(angle + correctedPose.GetHeading()) + correctedPose.GetX();
				poseY2World=-(*itReading) * std::sin(angle + correctedPose.GetHeading()) +correctedPose.GetY();
				hitPoint.SetX((int)(poseX2World/m_resolution));
				hitPoint.SetY((int)(poseY2World/m_resolution));
				cell->AddPoint(hitPoint);
			}
			angle+=angleIncrement;
		}
		angle=0.0;
	}
}

double Mapper::Score(Cell* cell,Pose2 rPose,LocalizedScan* rScan)
{
	float poseX2World,poseY2World;
	Point hitPoint;
	float angle=0.0;
	float angleIncrement=LocalizedScan::m_angleIncrement;
	int index=0;
	double score=0.0;
	for(std::vector<float>::const_iterator iter=rScan->GetReadings().begin();iter!=rScan->GetReadings().end();iter++)
	{
		if((*iter)<=LocalizedScan::m_maxRange&&(*iter)>=LocalizedScan::m_minRange)
		{
			poseX2World=-(*iter) * std::cos(angle + rPose.GetHeading()) + rPose.GetX();
			poseY2World=-(*iter) * std::sin( angle + rPose.GetHeading()) + rPose.GetY();
			hitPoint.SetX((int)(poseX2World/m_resolution));
			hitPoint.SetY((int)(poseY2World/m_resolution));
			if(cell->GetCellIndex(hitPoint,index)==true)
			{
// 				if(index>800)
// 					std::cout<<"index\t"<<index;
				score+=cell->GetScore(index,hitPoint);			
			}
			else
			{
				continue;
			}
		}
		angle+=angleIncrement;
	}
	return score;

}

void Mapper::CreateMap(OccupancyGrid& rOccGrid)
{
	int size=0;
	int up=0;
	int down=0;
	float result=0.0;
	VectorFloat		readings;
	Pose2 correctedPose;
	Point tmp(0,0);
	Point correctedPoint;
	float poseX2World,poseY2World;
	float angle=0.0;
	float maxRange=LocalizedScan::m_maxRange;
	float minRange=LocalizedScan::m_minRange;
	float minX=maxRange,minY=maxRange,maxX=-maxRange,maxY=-maxRange;
	int	pointX2World=0;
	int	pointY2World=0;
	float angleIncrement=LocalizedScan::m_angleIncrement;
	
	/*obtain the size of the map that will be created*/
	for(VectorLocalizedScan::const_iterator itLocalizedScan=m_processedScans.begin();itLocalizedScan!=m_processedScans.end();++itLocalizedScan)
	{
		correctedPose=(*itLocalizedScan)->GetCorrectedPose();
		readings=(*itLocalizedScan)->GetReadings();
		for(std::vector<float>::const_iterator itReading=readings.begin();itReading!=readings.end();itReading++)
		{
			if((*itReading)<=maxRange&&(*itReading)>=minRange)
			{
				poseX2World=-(*itReading) * std::cos(angle + correctedPose.GetHeading()) + correctedPose.GetX() ;
				poseY2World=-(*itReading) * std::sin(angle + correctedPose.GetHeading()) + correctedPose.GetY();
				if(poseX2World<minX)
					minX=poseX2World;
				else if(poseX2World>maxX)
					maxX=poseX2World;
				if(poseY2World<minY)
					minY=poseY2World;
				else if(poseY2World>maxY)
					maxY=poseY2World;
			}
			angle+=angleIncrement;
		}	
	}
	//std::cout<<"maxX="<<maxX<<"\tmaxY="<<maxY<<"\tminX="<<minX<<"\tminY="<<minY<<std::endl;	
	std::cout<<"processed scan size="<<m_processedScans.size()<<std::endl;
	int width=(int)((maxX-minX)/GetResolution());
	int height=(int)((maxY-minY)/GetResolution());
	Pose2 borderPose(minX,minY,0.0);
	rOccGrid.SetHeight(height);
	rOccGrid.SetWidth(width);
	rOccGrid.SetOriginPose(borderPose);
	rOccGrid.Resize(height*width);
	/*fill in the grid value of the occupancy grid map*/
	for(VectorLocalizedScan::const_iterator itLocalizedScan=m_processedScans.begin();itLocalizedScan!=m_processedScans.end();++itLocalizedScan)
	{
		correctedPose=(*itLocalizedScan)->GetCorrectedPose();
		correctedPoint.SetX(correctedPose.GetX()/GetResolution());
		correctedPoint.SetY(correctedPose.GetY()/GetResolution());
		readings=(*itLocalizedScan)->GetReadings();

		for(std::vector<float>::const_iterator itReading=readings.begin();itReading!=readings.end();itReading++)
		{
			if((*itReading)<=maxRange&&(*itReading)>=minRange)
			{
				poseX2World=-(*itReading) * std::cos(angle + correctedPose.GetHeading()) + correctedPose.GetX() ;
				poseY2World=-(*itReading) * std::sin(angle + correctedPose.GetHeading()) + correctedPose.GetY();
				pointX2World=(int)(poseX2World/GetResolution());
				pointY2World=(int)(poseY2World/GetResolution());
				
				Point readingPoint(pointX2World,pointY2World);
				rOccGrid.SetMapValue(rOccGrid.GetGridIndex(readingPoint),gridStatus_Occupied);
// 				std::cout<<"pointX2World="<<pointX2World<<"...pointY2World="<<pointY2World;
// 				std::cout<<"...index"<<rOccGrid.GetGridIndex(readingPoint)<<std::endl;
				LinearFunc lf(readingPoint,correctedPoint);
				if(lf.GetType()==0||lf.GetType()==3)
				{		
					if(readingPoint.GetX()>correctedPoint.GetX())
					{
						up = readingPoint.GetX();
						down=correctedPoint.GetX();
					}
					else
					{
						down = readingPoint.GetX();
						up = correctedPoint.GetX();
					}
					size = up - down;
					for (int step = 1; step < size; step++)
					{
						result =(int)lf.CalcResult(down + step);
						tmp.SetX(down+step);
						tmp.SetY(result);
						rOccGrid.SetMapValue(rOccGrid.GetGridIndex(tmp),gridStatus_Free);
						
					}
				}
				else if(lf.GetType()==1||lf.GetType()==2)
				{
					if(readingPoint.GetY()>correctedPoint.GetY())
					{
						up = readingPoint.GetY();
						down=correctedPoint.GetY();
					}
					else
					{
						down = readingPoint.GetY();
						up = correctedPoint.GetY();
					}
					size = up - down;
					for (int step = 1; step < size; step++)
					{
						result =(int)lf.CalcResult(down + step);
			
						tmp.SetX(result);
						tmp.SetY(down+step);
						rOccGrid.SetMapValue(rOccGrid.GetGridIndex(tmp),gridStatus_Free);
					}
				}
			}
			angle+=angleIncrement;
		}
	}
}

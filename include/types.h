#ifndef TYPES_H_
#define TYPES_H_
#include <cmath>
#include <string.h>
#include  <string>
#include <iostream>
#include	<vector>
#include "Math.h"

const int8_t		girdStatus_Unknow=-1;
const int8_t		gridStatus_Occupied=100;
const int8_t		gridStatus_Free=0;

typedef std::vector<double> VectorDouble;
typedef std::vector<float> VectorFloat;

template<typename T>
class Vector2
{
public:
	/*default constructor*/
	Vector2()
	{
		m_values[0]=0;
		m_values[1]=0;
	}
	/*initialized by given vector location*/
	Vector2(T x,T y)
	{
		m_values[0]=x;
		m_values[1]=y;
	}
	inline void SetX(T x)
	{
		m_values[0]=x;
	}
	inline	T	GetX()	const
	{
		return m_values[0];
	}
	inline void SetY(T y)
	{
		m_values[1]=y;
	}
	inline	T	GetY()	const
	{
		return m_values[1];
	}
	inline T GetSquaredLength() const
	{
		return (m_values[0]*m_values[0])+(m_values[1]*m_values[1]);
	}
	inline T GetSquaredDistance(const Vector2 &rOther)	const
	{
		return (*this-rOther).GetSquaredLength();
	}
	inline T GetDistance(const Vector2 &rOther)	const
	{
		return std::sqrt<T>(GetSquaredDistance(rOther));
	}
	inline const Vector2 operator - (const Vector2& rOther) const
	{
		return Vector2(m_values[0]-rOther.m_values[0],m_values[1]-rOther.m_values[1]);
	}
	inline const Vector2 operator + (const Vector2& rOther) const
	{
		return Vector2(m_values[0]+rOther.m_values[0],m_values[1]+rOther.m_values[1]);
	}
	inline const Vector2& operator =(const Vector2& rOther)
	{
		m_values[0]=rOther.GetX();
		m_values[1]=rOther.GetY();
		return *this;
	}
	inline void operator += (const Vector2 &rOther) 
	{
		m_values[0]+=rOther.GetX();
		m_values[1]+=rOther.GetY();
	}
	inline void operator -= (const Vector2 &rOther) 
	{
		m_values[0]-=rOther.GetX();
		m_values[1]-=rOther.GetY();
	}
	inline Vector2 operator / (const T scalar) const
	{
		return Vector2(m_values[0] / scalar, m_values[1] / scalar);
	}
	inline void operator /= (T scalar)
    {
      m_values[0] /= scalar;
      m_values[1] /= scalar;
    }
private:
	T m_values[2];
};

typedef	std::vector<Vector2<double>> VectorDoublePos;
typedef	Vector2<int> Point; 
typedef	std::vector<Point> VectorPoint;

class Pose2
{
public:
	/*default constructor*/
	Pose2():m_heading(0.0)
	{		
	}
	/*initialized by given pose*/
	Pose2(Vector2<double> position,double heading):m_position(position),m_heading(heading)
	{
	}
	/*initialized by given pose*/
	Pose2(double x,double y,double heading)
	:m_position(x,y),m_heading(heading)
	{
	}
	inline  double GetX() const
	{
		return m_position.GetX();
	}
	inline  double GetY() const
	{
		return m_position.GetY();
	}
	inline const Vector2<double> &GetPosition() const
	{
		return m_position;
	}
	inline double GetHeading() const
	{
		return m_heading;
	}
	inline void SetX(double x)
	{
		m_position.SetX(x);
	}
	inline void SetY(double y)
	{
		m_position.SetY(y);
	}
	inline void SetPosition(Vector2<double> position)
	{
		m_position=position;
	}
	inline void SetHeading(double heading)
	{
		m_heading=heading;
	}
	inline double GetSquaredDistance(const Pose2& rOther) const
    {
      return m_position.GetSquaredDistance(rOther.m_position);
    }
    inline const Pose2& operator = (const Pose2& rOther)
	{
		m_position=rOther.m_position;
		m_heading=rOther.m_heading;
		return *this;
	}
	inline const Pose2 operator - (const Pose2& rOther)	const
	{
		return Pose2(m_position-rOther.GetPosition() , NormalizeAngle(m_heading-rOther.GetHeading()));
	}
	inline const Pose2 operator + (const Pose2& rOther)	const
	{
		return Pose2(m_position+rOther.GetPosition() , NormalizeAngle(m_heading+rOther.GetHeading()));
	}
private:
	Vector2<double> m_position;
	/*in radians*/
	double m_heading;
};
	  
class LaserParamManager
{
public:
	LaserParamManager():m_updateFlag(false)
	{
		
	}
	inline bool IsUpdate()
	{
		return m_updateFlag;
	}
	inline void SetUpdateFlag(bool updateFlag)
	{
		m_updateFlag=updateFlag;
	}
	
	inline float GetMinAngle()
	{
		return m_minAngle;
	}
	inline float GetMaxAngle()
	{
			return m_maxAngle;
	}
	inline float	GetMinRange()
	{
		return m_minRange;
	}
	inline float GetMaxRange()
	{
		return m_maxRange;
	}
	inline float GetAngleIncrement()
	{
		return m_angleIncrement;
	}
	inline void SetMinAngle(float minAngle)
	{
		m_minAngle=minAngle;
	}
	inline void SetMaxAngle(float maxAngle)
	{
		m_maxAngle=maxAngle;
	}
	inline void SetMinRange(float minRange)
	{
		m_minRange=minRange;
	}
	inline void SetMaxRange(float maxRange)
	{
		m_maxRange=maxRange;
	}
	inline void SetAngleIncrement(float angleIncrement)
	{
		m_angleIncrement=angleIncrement;
	}
private:
    // sensor m_Parameters
    float m_minAngle;
    float m_maxAngle;
    float m_angleIncrement;
    float m_minRange;
    float m_maxRange;
	bool m_updateFlag;
};

class Matrix3
{
public:
	Matrix3()
	{
	}
	/*copy constructor*/
	Matrix3(const Matrix3& rOther)
	{
		memcpy(m_matrix,rOther.m_matrix,sizeof(double)*m_size);
	}
	/*get rotation matrix accordind to yaw angle*/
	Matrix3(double radians)
	{
		m_matrix[0][0]=std::cos(radians);
		m_matrix[0][1]=std::sin(-radians);
		m_matrix[0][2]=0.0;
		m_matrix[1][0]=std::sin(radians);
		m_matrix[1][1]=std::cos(radians);
		m_matrix[1][2]=0.0;
		m_matrix[2][0]=0.0;
		m_matrix[2][1]=0.0;
		m_matrix[2][2]=1.0;
	}
	/*set to unit matrix*/
	void SetToIdentity()
	{
		memset(m_matrix, 0, m_size*sizeof(double));
		for (int i = 0; i < 3; i++)
		{
			m_matrix[i][i] = 1.0;
		}
	}
	inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_matrix, rOther.m_matrix,m_size*sizeof(double));
      return *this;
    }
    
    inline Matrix3 operator * (const Matrix3& rOther) const
    {
      Matrix3 result;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          result.m_matrix[row][col] = m_matrix[row][0]*rOther.m_matrix[0][col] +
                                       m_matrix[row][1]*rOther.m_matrix[1][col] +
                                       m_matrix[row][2]*rOther.m_matrix[2][col];
        }
      }
      return result;
    }
    
	inline Pose2 operator * (const Pose2& rPose2) const
    {
      Pose2 result;

      result.SetX(m_matrix[0][0] * rPose2.GetX() + m_matrix[0][1] *
                 rPose2.GetY() + m_matrix[0][2] * rPose2.GetHeading());
      result.SetY(m_matrix[1][0] * rPose2.GetX() + m_matrix[1][1] *
                 rPose2.GetY() + m_matrix[1][2] * rPose2.GetHeading());
      result.SetHeading(m_matrix[2][0] * rPose2.GetX() + m_matrix[2][1] *
                       rPose2.GetY() + m_matrix[2][2] * rPose2.GetHeading());

      return result;
    }

    
private:
	double m_matrix[3][3];
	const int m_size=9;
	
};

class Transform
{
public:
	Transform(const Pose2& rPose1,const Pose2& rPose2)
	{
		SetTransform(rPose1,rPose2);
	}
	void SetTransform(const Pose2& rPose1,const Pose2& rPose2)
	{
		m_rotation=Matrix3(rPose2.GetHeading()-rPose1.GetHeading());
		m_transform=Pose2(rPose2-rPose1);
	}
	inline Pose2 TransformPose(const Pose2& rSourcePose)
	{
		Pose2 newPosition = rSourcePose+m_transform;
	};
private:
	Pose2 m_transform;
	Matrix3 m_rotation;
};

#endif
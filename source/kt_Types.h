/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __KARTO_TYPES__
#define __KARTO_TYPES__

#include <string>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>

/**
 * Karto defs for windows dynamic build 
 */
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
# if defined( _LIB ) || defined( KARTO_STATIC ) || defined( STATIC_BUILD )
#  define KARTO_EXPORT
# else
#  ifdef KARTO_DYNAMIC
#    define KARTO_EXPORT __declspec(dllexport)
#  else
#    define KARTO_EXPORT __declspec(dllimport)
#  endif // KARTO_DYNAMIC 
# endif
#else
#  define KARTO_EXPORT
#endif 

/**
 * Karto type definition 
 * Ensures platform independent types for windows, linux and mac
 */
#if defined(_MSC_VER)

  typedef signed __int8 kt_int8s;
  typedef unsigned __int8 kt_int8u;

  typedef signed __int16 kt_int16s;
  typedef unsigned __int16 kt_int16u;

  typedef signed __int32 kt_int32s;
  typedef unsigned __int32 kt_int32u;

  typedef signed __int64 kt_int64s;
  typedef unsigned __int64 kt_int64u;

#else

  #include <stdint.h>

  typedef int8_t kt_int8s;
  typedef uint8_t kt_int8u;

  typedef int16_t kt_int16s;
  typedef uint16_t kt_int16u;

  typedef int32_t kt_int32s;
  typedef uint32_t kt_int32u;

  typedef signed long long kt_int64s;
  typedef unsigned long long kt_int64u;

#endif

typedef char kt_char;
typedef float kt_float;
typedef double kt_double;
typedef bool kt_bool;

#define KARTO_Object(name) \
  virtual const char* GetClassName() const { return #name; } \
  virtual kt_objecttype GetObjectType() const { return ObjectType_##name; }

typedef kt_int32u kt_objecttype;

const kt_objecttype ObjectType_None									= 0x00000000;
const kt_objecttype ObjectType_DatasetObject				= 0x00001000;
const kt_objecttype ObjectType_Device								= 0x00003000;
const kt_objecttype ObjectType_DeviceState					= 0x00004000;
const kt_objecttype ObjectType_CustomData  					= 0x00008000;
const kt_objecttype ObjectType_Misc									= 0x10000000;

const kt_objecttype ObjectType_Drive								= ObjectType_Device | 0x01;
const kt_objecttype ObjectType_LaserRangeFinder			= ObjectType_Device | 0x02;

const kt_objecttype ObjectType_DrivePose						= ObjectType_DeviceState | 0x01;
const kt_objecttype ObjectType_LaserRangeScan		    = ObjectType_DeviceState | 0x02;
const kt_objecttype ObjectType_LocalizedRangeScan		= ObjectType_DeviceState | 0x03;

const kt_objecttype ObjectType_Parameters				    = ObjectType_Misc | 0x03;
const kt_objecttype ObjectType_DatasetInfo			    = ObjectType_Misc | 0x03;

/** 
 * Helper defines for std iterator loops 
 */
#define forEach( listtype, list ) \
  for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define forEachAs( listtype, list, iter ) \
  for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define const_forEach( listtype, list ) \
  for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define const_forEachAs( listtype, list, iter ) \
  for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

#define forEachR( listtype, list ) \
  for ( listtype::iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

#define const_forEachR( listtype, list ) \
  for ( listtype::const_iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

namespace karto
{
  /**
   * Platform independent pi definitions
   */
  const kt_double KT_PI         =  3.14159265358979323846;  // The value of PI
  const kt_double KT_2PI        =  6.28318530717958647692;  // 2 * PI
  const kt_double KT_PI_2       =  1.57079632679489661923;  // PI / 2
  const kt_double KT_PI_180     =  0.01745329251994329577;  // PI / 180
  const kt_double KT_180_PI     = 57.29577951308232087685;  // 180 / PI
  
  /**
   * Lets define a small number!
   */
  const kt_double KT_TOLERANCE  = 1e-06;
  
  class Math
  {    
  public:
    /**
     * Converts degrees into radians
     * @param degrees
     * @return radian equivalent of degrees
     */
    inline static kt_double DegreesToRadians(kt_double degrees)
    {
      return degrees * KT_PI_180;
    }
    
    /**
     * Converts radians into degrees
     * @param radians
     * @return degree equivalent of radians
     */
    inline static kt_double RadiansToDegrees(kt_double radians)
    {
      return radians * KT_180_PI;
    }
    
    /**
     * Square function
     * @param value
     * @return square of value
     */
    template<typename T>
    inline static T Square(T value)
    {
      return (value * value);
    }
    
    /**
     * Round function
     * @param value
     * @return rounds value to the nearest whole number (as double)
     */
    inline static kt_double Round(kt_double value)
    {
      return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
    }
    
    /**
     * Binary minimum function
     * @param value1
     * @param value2
     * @return the lesser of value1 and value2
     */
    template<typename T>
    inline static T Minimum(T value1, T value2)
    {
      return value1 < value2 ? value1 : value2;
    }
    
    /**
     * Binary maximum function
     * @param value1
     * @param value2
     * @return the greater of value1 and value2
     */
    template<typename T>
    inline static T Maximum(T value1, T value2)
    {
      return value1 > value2 ? value1 : value2;
    }
    
    /**
     * Clips a number to the specified minimum and maximum values.
     * @param n number to be clipped
     * @param minValue minimum value
     * @param maxValue maximum value
     * @return the clipped value
     */
    template<typename T> 
    inline static T Clip(T n, T minValue, T maxValue)
    {
      return Minimum(Maximum(n, minValue), maxValue);
    }
    
    /**
     * Checks whether two numbers are equal within a certain tolerance.
     * @param a
     * @param b
     * @return true if a and b differ by at most a certain tolerance.
     */
    inline static kt_bool DoubleEqual(kt_double a, kt_double b)
    {
      double delta = a - b; 
      return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
    }
    
    /**
     * Checks whether value is in the range [0;maximum)
     * @param value
     * @param maximum
     */
    template<typename T>
    inline static kt_bool IsUpTo(T value, T maximum)
    {
      return (value >= 0 && value < maximum);
    }
    
    /**
     * Checks whether value is in the range [a;b]
     * @param value
     * @param a
     * @param b
     */
    template<typename T>
    inline static kt_bool InRange(T value, T a, T b)
    {
      return (value >= a && value <= b);
    }
    
    /**
     * Normalizes angle to be in the range of [-pi, pi]
     * @param angle to be normalized
     * @return normalized angle
     */
    inline static kt_double NormalizeAngle(kt_double angle)
    {
      while (angle < -KT_PI)
      {
        if (angle < -KT_2PI)
        {
          angle += (kt_int32u)(angle / -KT_2PI) * KT_2PI;
        }
        else
        {
          angle += KT_2PI;
        }
      }
      
      while (angle > KT_PI)
      {
        if (angle > KT_2PI)
        {
          angle -= (kt_int32u)(angle / KT_2PI) * KT_2PI;
        }
        else
        {
          angle -= KT_2PI;
        }
      }
      
      assert(Math::InRange(angle, -KT_PI, KT_PI));
      
      return angle;
    }
    
    /**
     * Returns an equivalent angle to the first parameter such that the difference
     * when the second parameter is subtracted from this new value is an angle
     * in the normalized range of [-pi, pi], i.e. abs(minuend - subtrahend) <= pi.
     * @param minuend
     * @param subtrahend
     * @return normalized angle
     */
    inline static kt_double NormalizeAngleDifference(kt_double minuend, kt_double subtrahend)
    {
      while (minuend - subtrahend < -KT_PI)
      {
        minuend += KT_2PI;
      }
      
      while (minuend - subtrahend > KT_PI)
      {
        minuend -= KT_2PI;
      }
      
      return minuend;
    }
    
    /**
     * Align a value to the alignValue. 
     * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
     * @param value
     * @param alignValue
     * @return aligned value
     */
    template<class T>
    inline static T AlignValue(size_t value, size_t alignValue = 8)
    {
      return static_cast<T> ((value + (alignValue - 1)) & ~(alignValue - 1));
    }
  }; // Math
    
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class KARTO_EXPORT Object
  {
  public:
    /**
     * Constructs an object with the given name
     * @param rName
     */
    Object(const std::string& rName)
      : m_Name(rName)
    {
    }

    /**
     * Default constructor
     */
    virtual ~Object()
    {
    }

  public:
    /**
     * Gets the name of this object
     * @return name
     */
    inline const std::string& GetName() const 
    {
      return m_Name; 
    }

    /**
     * Gets the class name of this object
     * @return class name
     */
    virtual const char* GetClassName() const = 0;
    
    /**
     * Gets the type of this object
     * @return object type
     */
    virtual kt_objecttype GetObjectType() const = 0;

  private:
    std::string m_Name;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a point (x, y) in 2-dimensional real space.
   */
  template<typename T>
  class Size2
  {
  public:
    /**
     * Default constructor
     */
    Size2()
      : m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * Constructor initializing point location
     * @param width
     * @param height
     */
    Size2(T width, T height)
      : m_Width(width)
      , m_Height(height)
    {
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Size2(const Size2& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Gets the width
     * @return the width
     */
    inline const T GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width
     * @param width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Gets the height
     * @return the height
     */
    inline const T GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height
     * @param height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Assignment operator
     */
    inline Size2& operator = (const Size2& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;

      return(*this);
    }

    /**
     * Write Size2 onto output stream
     * @param rSize to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Size2& rSize)
    {
      rStream << "(" << rSize.m_Width << ", " << rSize.m_Height << ")";
      return rStream;
    }

  private:
    T m_Width;
    T m_Height;
  }; // Size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a point (x, y) in 2-dimensional real space.
   */
  template<typename T>
  class KARTO_EXPORT Point2
  {
  public:
    /**
     * Default constructor
     */
    Point2()
      : m_X(0)
      , m_Y(0)
    {
    }

    /**
     * Constructor initializing point location
     * @param x
     * @param y
     */
    Point2(T x, T y)
      : m_X(x)
      , m_Y(y)
    {
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Point2(const Point2& rOther)
      : m_X(rOther.m_X)
      , m_Y(rOther.m_Y)
    {
    }

  public:
    /**
     * Gets the x-coordinate of this point
     * @return the y-coordinate of the point
     */
    inline const T GetX() const
    {
      return m_X;
    }

    /**
     * Sets the x-coordinate of this point
     * @param x the x-coordinate of the point
     */
    inline void SetX(T x)
    {
      m_X = x;
    }

    /**
     * Gets the y-coordinate of this point
     * @return the y-coordinate of the point
     */
    inline const T GetY() const
    {
      return m_Y;
    }

    /**
     * Sets the y-coordinate of this point
     * @param y the y-coordinate of the point
     */
    inline void SetY(T y)
    {
      m_Y = y;
    }

    /**
     * Squared length of this point from the origin
     * @return the squared length of this point from the origin
     */
    inline T SquaredLength() const
    {
      return m_X * m_X + m_Y * m_Y;
    }

    /**
     * Floor point operator
     * @param rOther Point2
     */
    inline void MakeFloor(const Point2& rOther)
    {
      if ( rOther.m_X < m_X ) m_X = rOther.m_X;
      if ( rOther.m_Y < m_Y ) m_Y = rOther.m_Y;
    }

    /**
     * Ceiling point operator
     * @param rOther Point2
     */
    inline void MakeCeil(const Point2& rOther)
    {
      if ( rOther.m_X > m_X ) m_X = rOther.m_X;
      if ( rOther.m_Y > m_Y ) m_Y = rOther.m_Y;
    }

    /**
     * Assignment operator
     */
    inline Point2& operator = (const Point2& rOther)
    {
      m_X = rOther.m_X;
      m_Y = rOther.m_Y;

      return *this;
    }

    /**
     * In place Point2 addition.
     */
    inline void operator += (const Point2& rOther)
    {
      m_X += rOther.m_X;
      m_Y += rOther.m_Y;
    }

    /**
     * In place scalar multiplication operator
     * @param scalar
     */
    inline void operator *= (T scalar)
    {
      m_X *= scalar;
      m_Y *= scalar;
    }

    /**
     * In place scalar division operator
     * @param scalar
     */
    inline void operator /= (T scalar)
    {
      m_X /= scalar;
      m_Y /= scalar;
    }

    /**
     * Divides a Point2
     * @param scalar
     * @return scalar product
     */
    inline Point2 operator / (T scalar) const
    {
      return Point2(m_X / scalar, m_Y / scalar);
    }

    /**
     * Scales a Point2
     * @param scalar
     * @return scalar product
     */
    inline Point2 operator * (T scalar) const
    {
      return Point2(m_X * scalar, m_Y * scalar);
    }

    /**
     * Addition operator
     * @param rOther
     * @return point resulting from adding this point with the given point
     */
    inline const Point2 operator + (const Point2& rOther) const
    {
      return Point2(m_X + rOther.m_X, m_Y + rOther.m_Y);
    }

    /**
     * Subtraction operator
     * @param rOther
     * @return point resulting from subtracting this point from the given point
     */
    inline const Point2 operator - (const Point2& rOther) const
    {
      return Point2(m_X - rOther.m_X, m_Y - rOther.m_Y);
    }

    /**
     * Equality operator
     */
		inline kt_bool operator == (const Point2& rOther) const
		{
			return (m_X == rOther.m_X && m_Y == rOther.m_Y);
		}

    /**
     * Inequality operator
     */
		inline kt_bool operator != (const Point2& rOther) const
		{
      return (m_X != rOther.m_X || m_Y != rOther.m_Y);
    }

    /**
     * Less than operator
     * @param rOther
     * @return true if left point is less than right point
     */
    inline kt_bool operator < (const Point2& rOther) const
    {
      return m_X < rOther.m_X || (m_X == rOther.m_X && m_Y < rOther.m_Y); 
    }

    /**
     * Write Point2 onto output stream
     * @param rPoint to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Point2& rPoint)
    {
      rStream << "(" << rPoint.m_X << ", " << rPoint.m_Y << ")";
      return rStream;
    }
  
  private:
    T m_X;
    T m_Y;
  }; // Point2<T>

  /**
   * Typedef for vectors of Point2
   */
  typedef std::vector< Point2<kt_double> > PointVectorDouble;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Stores x, y, width and height that represents the location and size of a rectangle
   * (x, y) is at bottom left in mapper!
   */
  class KARTO_EXPORT Rectangle2  
  {
  public:
    /**
     * Default constructor
     */
    Rectangle2()
      : m_X(0)
      , m_Y(0)
      , m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * Constructor initializing rectangle parameters
     * @param x x-coordinate of left edge of rectangle
     * @param y y-coordinate of bottom edge of rectangle
     * @param width width of rectangle
     * @param height height of rectangle
     */
    Rectangle2(kt_int32s x, kt_int32s y, kt_int32s width, kt_int32s height)
      : m_X(x)
      , m_Y(y)
      , m_Width(width)
      , m_Height(height)
    {
    }

  public:
    /**
     * Gets the x-coordinate of the left edge of this rectangle
     * @return the x-coordinate of the left edge of this rectangle
     */
    inline kt_int32s GetX() const
    {
      return m_X;
    }
    
    /**
     * Sets the x-coordinate of the left edge of this rectangle
     * @param x the x-coordinate of the left edge of this rectangle
     */
    inline void SetX(kt_int32s x) 
    {
      m_X = x;
    }

    /**
     * Gets the y-coordinate of the bottom edge of this rectangle
     * @return the y-coordinate of the bottom edge of this rectangle
     */
    inline kt_int32s GetY() const
    {
      return m_Y;
    }

    /**
     * Sets the y-coordinate of the bottom edge of this rectangle
     * @param y the y-coordinate of the bottom edge of this rectangle
     */
    inline void SetY(kt_int32s y)
    {
      m_Y = y;
    }
    
    /**
     * Gets the width of this rectangle
     * @return the width of this rectangle
     */
    inline kt_int32s GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width of this rectangle
     * @param width the width of this rectangle
     */
    inline void SetWidth(kt_int32s width)
    {
      m_Width = width;
    }
    
    /**
     * Gets the height of this rectangle
     * @return the height of this rectangle
     */
    inline kt_int32s GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height of this rectangle
     * @param height the height of this rectangle
     */
    inline void SetHeight(kt_int32s height)
    {
      m_Height = height;
    }
    
  private:
    kt_int32s m_X;
    kt_int32s m_Y;
    kt_int32s m_Width;
    kt_int32s m_Height;
  }; // Rectangle2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a position (x, y) in 2-dimensional space and heading.
   */
  class KARTO_EXPORT Pose2
  {
  public:
    /**
     * Default Constructor
     */
    Pose2()
      : m_Heading(0.0)
    {
    }

    /**
     * Constructor initializing pose parameters
     * @param rPosition position
     * @param heading heading
     **/
    Pose2(const Point2<kt_double>& rPosition, kt_double heading)
      : m_Position(rPosition)
      , m_Heading(heading)
    {
    }

    /**
     * Constructor initializing pose parameters
     * @param x x-coordinate
     * @param y y-coordinate
     * @param heading heading
     **/
    Pose2(kt_double x, kt_double y, kt_double heading)
      : m_Position(x, y)
      , m_Heading(heading)
    {
    }

    /**
     * Copy constructor
     */
    Pose2(const Pose2& rOther)
      : m_Position(rOther.m_Position)
      , m_Heading(rOther.m_Heading)
    {
    }

  public:
    /**
     * Returns the x-coordinate
     * @return the x-coordinate of the pose
     */
    inline kt_double GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate
     * @param the x-coordinate of the pose
     */
    inline void SetX(kt_double x)
    {
      m_Position.SetX(x);
    }

    /**
     * Returns the y-coordinate
     * @return the y-coordinate of the pose
     */
    inline kt_double GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate
     * @param the y-coordinate of the pose
     */
    inline void SetY(kt_double y)
    {
      m_Position.SetY(y);
    }

    /**
     * Returns the position
     * @return the position of the pose
     */
    inline const Point2<kt_double>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position
     * @param rPosition of the pose
     */
    inline void SetPosition(const Point2<kt_double>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Returns the heading of the pose (in radians)
     * @return the heading of the pose
     */
    inline kt_double GetHeading() const
    {
      return m_Heading;
    }

    /**
     * Sets the heading
     * @param the heading of the pose
     */
    inline void SetHeading(kt_double heading)
    {
      m_Heading = heading;
    }

    /**
     * Returns the square of the length of the pose from the origin.
     * @return square of the length of the pose from the origin
     */
    inline kt_double SquaredLength() const
    {
      return m_Position.SquaredLength();
    }

    /**
     * Returns the length of the pose (x and y).
     * @return Length of the pose
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns the square distance to the given pose
     * @returns square distance to the given pose
     */
    inline kt_double SquaredDistance(const Pose2& rPose) const
    {
      return (*this - rPose).SquaredLength();
    }

  public:
    /**
     * Assignment operator
     */
    inline Pose2& operator = (const Pose2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Heading = rOther.m_Heading;

      return *this;
    }

    /**
     * Equality operator
     */
		inline kt_bool operator == (const Pose2& rOther) const
		{
			return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
		}

    /**
     * Inequality operator
     */
		inline kt_bool operator != (const Pose2& rOther) const
		{
      return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
    }

    /**
     * In place Pose2 add.
     */
    inline void operator += (const Pose2& rOther)
    {
      m_Position += rOther.m_Position;
      m_Heading = Math::NormalizeAngle(m_Heading + rOther.m_Heading);
    }

    /**
     * Binary Pose2 add
     * @param rOther
     * @return Pose2 sum
     */
    inline Pose2 operator + (const Pose2& rOther) const
    {
      return Pose2(m_Position + rOther.m_Position, Math::NormalizeAngle(m_Heading + rOther.m_Heading));
    }

    /**
     * Binary Pose2 subtract
     * @param rOther
     * @return Pose2 difference
     */
    inline Pose2 operator - (const Pose2& rOther) const
    {
      return Pose2(m_Position - rOther.m_Position, Math::NormalizeAngle(m_Heading - rOther.m_Heading));
    }

    /**
     * Read pose from input stream
     * @param rPose to write
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Pose2& /*rPose*/)
    {
      // Implement me!!
      return rStream;
    }
    
    /**
     * Write this pose onto output stream
     * @param rPose to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Pose2& rPose)
    {
      rStream << "(" << rPose.m_Position.GetX() << ", " << rPose.m_Position.GetY() << ", " << rPose.m_Heading << ")";
      return rStream;
    }

  private:
    Point2<kt_double> m_Position;

    kt_double m_Heading;
  }; // Pose2

  /**
   * Typedef for vectors of Pose2
   */
  typedef std::vector< Pose2 > Pose2Vector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

	/**
	 * Defines a Matrix 3 x 3 class.
	 */
  class KARTO_EXPORT Matrix3
  {
  public:
    /**
     * Default constructor
     */
    Matrix3()
    {
      Clear();
    }

    /**
     * Copy constructor
     */
    inline Matrix3(const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
    }

  public:
    /**
     * Sets this matrix to identity matrix
     */
    void SetToIdentity()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));

      for (kt_int32s i = 0; i < 3; i++)
      {
        m_Matrix[i][i] = 1.0;
      }
    }

    /**
     * Sets this matrix to zero matrix
     */
    void Clear()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));
    }

    /**
     * Sets this matrix to be the rotation matrix of rotation around given axis
     * @param x x-coordinate of axis
     * @param y y-coordinate of axis
     * @param z z-coordinate of axis
     * @param radians amount of rotation
     */
    void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
    {
      kt_double cosRadians = cos(radians);
      kt_double sinRadians = sin(radians);
      kt_double oneMinusCos = 1.0 - cosRadians;

      kt_double xx = x * x;
      kt_double yy = y * y;
      kt_double zz = z * z;

      kt_double xyMCos = x * y * oneMinusCos;
      kt_double xzMCos = x * z * oneMinusCos;
      kt_double yzMCos = y * z * oneMinusCos;

      kt_double xSin = x * sinRadians;
      kt_double ySin = y * sinRadians;
      kt_double zSin = z * sinRadians;

      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
      m_Matrix[0][1] = xyMCos - zSin;
      m_Matrix[0][2] = xzMCos + ySin;

      m_Matrix[1][0] = xyMCos + zSin;
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
      m_Matrix[1][2] = yzMCos - xSin;

      m_Matrix[2][0] = xzMCos - ySin;
      m_Matrix[2][1] = yzMCos + xSin;
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
    }

    /**
     * Returns transposed version of this matrix
     * @return transposed matrix
     */
    Matrix3 Transpose() const
    {
      Matrix3 transpose;

      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          transpose.m_Matrix[row][col] = m_Matrix[col][row];
        }
      }

      return transpose;
    }

    /**
     * Returns the inverse of the matrix
     */
		Matrix3 Inverse() const
    {
      Matrix3 kInverse = *this;
      kt_bool haveInverse = InverseFast(kInverse, 1e-14);
      assert(haveInverse);
      return kInverse;
    }

    ///**
    // * Internal helper method for inverse matrix calculation
    // * This code is lifted from the OgreMatrix3 class!!
    // */
    kt_bool InverseFast(Matrix3& rkInverse, kt_double fTolerance = KT_TOLERANCE) const
    {
      // Invert a 3x3 using cofactors.  This is about 8 times faster than
      // the Numerical Recipes code which uses Gaussian elimination.
      rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
      rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
      rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
      rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
      rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
      rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
      rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
      rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
      rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];

      kt_double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] + m_Matrix[0][1]*rkInverse.m_Matrix[1][0]+ m_Matrix[0][2]*rkInverse.m_Matrix[2][0];

      if (fabs(fDet) <= fTolerance)
      {
        return false;
      }

      kt_double fInvDet = 1.0/fDet;
      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          rkInverse.m_Matrix[row][col] *= fInvDet;
        }
      }

      return true;
    }

  public:
    /**
     * Assignment operator
     */
    inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
      return *this;
    }

    /**
     * Matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return reference to mat(r,c)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
		{
			return m_Matrix[row][column];
		}

    /**
     * Read-only matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return mat(r,c)
     */
    inline kt_double operator()(kt_int32u row, kt_int32u column) const
    {
      return m_Matrix[row][column];
    }

    /**
     * Binary Matrix3 multiplication.
     * @param rkMatrix
     * @return Matrix3 product
     */
    Matrix3 operator * (const Matrix3& rOther) const
    {
      Matrix3 product;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] + m_Matrix[row][1]*rOther.m_Matrix[1][col] + m_Matrix[row][2]*rOther.m_Matrix[2][col];
        }
      }

      return product;
    }

    /**
     * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
     * @param rPose2
     * @return Pose2 product
     */
    inline Pose2 operator * (const Pose2& rPose2) const
    {
      Pose2 pose2;

      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] * rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] * rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] * rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

      return pose2;
    }

    /**
     * In place Matrix3 add.
     * @param rkMatrix
     */
    inline void operator += (const Matrix3& rkMatrix)
    {
      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
        }
      }
    }
    
    /**
     * Write Matrix3 onto output stream
     * @param rMatrix to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Matrix3& rMatrix)
    {
      rStream << "[ " << std::setw(12) << rMatrix(0, 0) << ", " << std::setw(12) << rMatrix(0, 1) << ", " << std::setw(12) << rMatrix(0, 2) << " ]" << std::endl;
      rStream << "[ " << std::setw(12) << rMatrix(1, 0) << ", " << std::setw(12) << rMatrix(1, 1) << ", " << std::setw(12) << rMatrix(1, 2) << " ]" << std::endl;
      rStream << "[ " << std::setw(12) << rMatrix(2, 0) << ", " << std::setw(12) << rMatrix(2, 1) << ", " << std::setw(12) << rMatrix(2, 2) << " ]" << std::endl;
      return rStream;
    }

  private:
    kt_double m_Matrix[3][3];
  }; // Matrix3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
	 * Defines a general Matrix class.
	 */
  class KARTO_EXPORT Matrix
  {
  public:
    /**
     * Constructs a matrix of size rows x columns
     */
    Matrix(kt_int32u rows, kt_int32u columns)
      : m_Rows(rows)
      , m_Columns(columns)
      , m_pData(NULL)
    {
      Allocate();

      Clear();
    }

    /**
     * Destructor
     */
    virtual ~Matrix()
    {
      delete [] m_pData;
    }

  public:
    /**
     * Set all entries to 0
     */
    void Clear()
    {
      if (m_pData != NULL)
      {
        memset(m_pData, 0, sizeof(kt_double) * m_Rows * m_Columns);
      }
    }

    /**
     * Gets the number of rows of the matrix
     * @return nubmer of rows
     */
    inline kt_int32u GetRows() const
    {
      return m_Rows;
    }

    /**
     * Gets the number of columns of the matrix
     * @return nubmer of columns
     */
    inline kt_int32u GetColumns() const
    {
      return m_Columns;
    }

    /**
     * Returns a reference to the entry at (row,column)
     * @param row
     * @param column
     * @return reference to entry at (row,column)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

    /**
     * Returns a const reference to the entry at (row,column)
     * @param row
     * @param column
     * @return const reference to entry at (row,column)
     */
    inline const kt_double& operator()(kt_int32u row, kt_int32u column) const
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

  private:
    /**
     * Allocate space for the matrix
     */
    void Allocate()
    {
      try
      {
        if (m_pData != NULL)
        {
          delete[] m_pData;
        }
        
        m_pData = new kt_double[m_Rows * m_Columns];
      }
      catch (std::bad_alloc exception)
      {
        assert(false);
        throw std::runtime_error("Matrix allocation error");
      }
      
      if (m_pData == NULL)
      {
        assert(false);
        throw std::runtime_error("Matrix allocation error");
      }
    }
    
    /**
     * Checks if (row,column) is a valid entry into the matrix
     * @param row
     * @param column
     */
    inline void RangeCheck(kt_int32u row, kt_int32u column) const
    {
      if (Math::IsUpTo(row, m_Rows) == false)
      {
        throw std::runtime_error("Matrix - RangeCheck ERROR!!!!");
      }

      if (Math::IsUpTo(column, m_Columns) == false)
      {
        throw std::runtime_error("Matrix - RangeCheck ERROR!!!!");
      }
    }

  private:
    kt_int32u m_Rows;
    kt_int32u m_Columns;
    
    kt_double* m_pData;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class KARTO_EXPORT Quaternion
  {
  public:
    /**
     * Create a quaternion with default (x=0, y=0, z=0, w=1) values
     */
    inline Quaternion()
    {
      m_Values[0] = 0.0;
      m_Values[1] = 0.0;
      m_Values[2] = 0.0;
      m_Values[3] = 1.0; 
    }

    /**
     * Create a quaternion using x, y, z, w values.
     * @param x
     * @param y
     * @param z
     * @param w
     */
    inline Quaternion(kt_double x, kt_double y, kt_double z, kt_double w)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
      m_Values[3] = w;
    }

    /**
     * Copy constructor
     */
    inline Quaternion(const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];
    }

  public:
    /**
     * Returns the X-value
     * @return Return the X-value of the quaternion
     */
    inline kt_double GetX() const
    {
      return m_Values[0]; 
    }

    /**
     * Sets the X-value
     * @param X-value of the quaternion
     */
    inline void SetX(kt_double x)
    {
      m_Values[0] = x; 
    }

    /**
     * Returns the Y-value
     * @return Return the Y-value of the quaternion
     */
    inline kt_double GetY() const
    {
      return m_Values[1]; 
    }

    /**
     * Sets the Y-value
     * @param Y-value of the quaternion
     */
    inline void SetY(kt_double y)
    {
      m_Values[1] = y; 
    }

    /**
     * Returns the Z-value
     * @return Return the Z-value of the quaternion
     */
    inline kt_double GetZ() const
    {
      return m_Values[2]; 
    }

    /**
     * Sets the Z-value
     * @param Z-value of the quaternion
     */
    inline void SetZ(kt_double z)
    {
      m_Values[2] = z; 
    }

    /**
     * Returns the W-value
     * @return Return the W-value of the quaternion
     */
    inline kt_double GetW() const
    {
      return m_Values[3]; 
    }

    /**
     * Sets the W-value
     * @param W-value of the quaternion
     */
    inline void SetW(kt_double w)
    {
      m_Values[3] = w; 
    }

    /**
     * Converts this quaternion into Euler angles
     * @param rYaw
     * @param rPitch
     * @param rRoll
     */
    void ToEulerAngles(kt_double& rYaw, kt_double& rPitch, kt_double& rRoll) const 
    {
      kt_double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

      if (test > 0.499)
      { 
        // singularity at north pole
        rPitch = 2 * atan2(m_Values[0], m_Values[3]);
        rRoll = KT_PI_2;
        rYaw = 0;
      }
      else if (test < -0.499)
      { 
        // singularity at south pole
        rPitch = -2 * atan2(m_Values[0], m_Values[3]);
        rRoll = -KT_PI_2;
        rYaw = 0;
      }
      else
      {
        kt_double sqx = m_Values[0] * m_Values[0];
        kt_double sqy = m_Values[1] * m_Values[1];
        kt_double sqz = m_Values[2] * m_Values[2];

        rPitch = atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2], 1 - 2 * sqy - 2 * sqz);
        rRoll = asin(2 * test);
        rYaw = atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2], 1 - 2 * sqx - 2 * sqz);
      }
    }

    /**
     * Set x,y,z,w values of the quaternion based on Euler angles. 
     * @param yaw
     * @param pitch
     * @param roll
     */
    void FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll)
    {
      kt_double angle;

      angle = yaw * 0.5; 
      kt_double cYaw = cos(angle);
      kt_double sYaw = sin(angle);

      angle = pitch * 0.5; 
      kt_double cPitch = cos(angle);
      kt_double sPitch = sin(angle);

      angle = roll * 0.5; 
      kt_double cRoll = cos(angle);
      kt_double sRoll = sin(angle);

      m_Values[0] = cPitch * cRoll * sYaw + sPitch * sRoll * cYaw;
      m_Values[1] = sPitch * cRoll * cYaw + cPitch * sRoll * sYaw;
      m_Values[2] = cPitch * sRoll * cYaw - sPitch * cRoll * sYaw;
      m_Values[3] = cPitch * cRoll * cYaw - sPitch * sRoll * sYaw;
    }

    /**
     * Assignment operator
     * @param rQuaternion
     */
    inline Quaternion& operator = (const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];

      return(*this);
    }

    /**
     * Equality operator returns true if the corresponding x,y,z,w values of each quaternion are the same values.
     * @param rhs
     */
    inline bool operator == (const Quaternion& rhs) const
    {
      return (m_Values[0] == rhs.m_Values[0] && m_Values[1] == rhs.m_Values[1] && m_Values[2] == rhs.m_Values[2] && m_Values[3] == rhs.m_Values[3]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x,y,z,w values of each quaternion not the same.
     * @param rhs
     */
    inline bool operator != (const Quaternion& rhs) const
    {
      return (m_Values[0] != rhs.m_Values[0] || m_Values[1] != rhs.m_Values[1] || m_Values[2] != rhs.m_Values[2] || m_Values[3] != rhs.m_Values[3]);
    }

    /**
     * Write this quaternion onto output stream
     * @param rStream
     * @param rQuaternion
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Quaternion& rQuaternion)
    {
      rStream << "(" << rQuaternion.m_Values[0] << ", " << rQuaternion.m_Values[1] << ", " << rQuaternion.m_Values[2] << ", " << rQuaternion.m_Values[3] << ")";
      return rStream;
    }

  private:
    kt_double m_Values[4];
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a bounding box in 2-dimensional real space.
   */
  class KARTO_EXPORT BoundingBox2
  {
  public:
    /*
     * Default constructor
     */
    BoundingBox2()
      : m_Minimum(DBL_MAX, DBL_MAX)
      , m_Maximum(DBL_MIN, DBL_MIN)
    {
    }

  public:
    /** 
     * Get bounding box minimum
     */
    inline const Point2<kt_double>& GetMinimum() const
    { 
      return m_Minimum; 
    }

    /** 
     * Set bounding box minimum
     */
    inline void SetMinimum(const Point2<kt_double>& mMinimum)
    {
      m_Minimum = mMinimum;
    }

    /** 
     * Get bounding box maximum
     */
    inline const Point2<kt_double>& GetMaximum() const
    { 
      return m_Maximum;
    }

    /** 
     * Set bounding box maximum
     */
    inline void SetMaximum(const Point2<kt_double>& rMaximum)
    {
      m_Maximum = rMaximum;
    }

    /**
     * Get the size of the bounding box 
     */
    inline Size2<kt_double> GetSize() const
    {
      Point2<kt_double> size = m_Maximum - m_Minimum;

      return Size2<kt_double>(size.GetX(), size.GetY());
    }

        /**
     * Add vector to bounding box
     */
    inline void Add(const Point2<kt_double>& rPoint)
    {
      m_Minimum.MakeFloor(rPoint);
      m_Maximum.MakeCeil(rPoint);
    }

    /**
     * Add other bounding box to bounding box
     */
    inline void Add(const BoundingBox2& rBoundingBox)
    {
      Add(rBoundingBox.GetMinimum());
      Add(rBoundingBox.GetMaximum());
    }

  private:
    Point2<kt_double> m_Minimum;
    Point2<kt_double> m_Maximum;
  }; // size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Implementation of a Pose2 transform
   */
  class KARTO_EXPORT Transform
  {
	public:
    /**
     * Constructs a transformation from the origin to the given pose
     * @param rPose pose
     */
    Transform(const Pose2& rPose)
    {
      SetTransform(Pose2(), rPose);
    }

    /**
     * Constructs a transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    Transform(const Pose2& rPose1, const Pose2& rPose2)
    {
      SetTransform(rPose1, rPose2);
    }

	public:
		/**
     * Transforms the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 TransformPose(const Pose2& rSourcePose)
    {
	    Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
      kt_double angle = Math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

      return Pose2(newPosition.GetPosition(), angle);
    }

    /**
     * Inverse transformation of the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 InverseTransformPose(const Pose2& rSourcePose)
    {
	    Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
      kt_double angle = Math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

      // components of transform
	    return Pose2(newPosition.GetPosition(), angle);
    }

  private:
		/**
		 * Sets this to be the transformation from the first pose to the second pose
		 * @param rPose1 first pose
		 * @param rPose2 second pose
		 */
    void SetTransform(const Pose2& rPose1, const Pose2& rPose2)
    {
	    if (rPose1 == rPose2)
	    {
		    m_Rotation.SetToIdentity();
		    m_InverseRotation.SetToIdentity();
        m_Transform = Pose2();
		    return;
	    }

	    // heading transformation
	    m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
	    m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

	    // position transformation
	    Pose2 newPosition;
	    if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
	    {
		    newPosition = rPose2 - m_Rotation * rPose1;
	    }
	    else
	    {
		    newPosition = rPose2;
	    }

	    m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
    }

  private:
    // pose transformation
    Pose2 m_Transform;

    Matrix3 m_Rotation;
    Matrix3 m_InverseRotation;
  }; // Transform

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  typedef enum
  {
    LaserRangeFinder_Custom = 0,

    LaserRangeFinder_Sick_LMS100 = 1,
    LaserRangeFinder_Sick_LMS200 = 2,
    LaserRangeFinder_Sick_LMS291 = 3,

    LaserRangeFinder_Hokuyo_UTM_30LX = 4,
    LaserRangeFinder_Hokuyo_URG_04LX = 5    
  } LaserRangeFinderType;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class AbstractParameter;

  /**
   * Typedef for vectors of Parameter
   */
  typedef std::vector<AbstractParameter*> ParameterVector;

  class KARTO_EXPORT ParameterManager
  {
  public:
    /**
     * Default constructor
     */
    ParameterManager();
    
    /**
     * Destructor
     */
    virtual ~ParameterManager();

  public:
    /**
     * Adds the parameter to this manager
     * @param pParameter
     */
    void Add(AbstractParameter* pParameter);
    
    /**
     * Gets the parameter of the given name
     * @param rName
     * @return parameter of given name
     */
    AbstractParameter* Get(const std::string& rName);
    
    /**
     * Clears the manager of all parameters
     */
    void Clear();
    
    /**
     * Gets all parameters
     * @return vector of all parameters
     */
    inline const ParameterVector& GetParameterVector() const;

  public:
    /**
     * Gets the parameter with the given name
     * @param rName
     * @return parameter of given name
     */
    AbstractParameter* operator() (const std::string& rName);

  private:
    ParameterVector m_Parameters;
    std::map<std::string, AbstractParameter*> m_ParameterLookup;

  }; // ParameterManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class KARTO_EXPORT AbstractParameter 
  {
  public:
    /**
     * Constructs a parameter with the given name
     * @param rName
     */
    AbstractParameter(const std::string& rName, ParameterManager* pParameterManger = NULL)
      : m_Name(rName)
    {
      // if parameter manager is provided add myself to it!
      if (pParameterManger != NULL)
      {
        pParameterManger->Add(this);
      }
    }

    /**
     * Constructs a parameter with the given name and description
     * @param rName
     * @param rDescription
     */
    AbstractParameter(const std::string& rName, const std::string& rDescription, ParameterManager* pParameterManger = NULL)
      : m_Name(rName)
      , m_Description(rDescription)
    {
      // if parameter manager is provided add myself to it!
      if (pParameterManger != NULL)
      {
        pParameterManger->Add(this);
      }
    }

    /**
     * Destructor
     */
    virtual ~AbstractParameter()
    {
    }

  public:
    /**
     * Gets the name of this object
     * @return name
     */
    inline const std::string& GetName() const 
    {
      return m_Name; 
    }

    /**
     * Returns the parameter description
     * @return parameter description
     */
    inline const std::string& GetDescription() const
    {
      return m_Description;
    }

    /**
     * Get parameter value as string.
     * @return value as string
     */
    virtual const std::string GetValueAsString() const = 0;

    /**
     * Set parameter value from string.
     * @param rStringValue value as string
     */
    virtual void SetValueFromString(const std::string& rStringValue) = 0;

    /**
     * Clones the parameter
     * @return clone
     */
    virtual AbstractParameter* Clone() = 0;

  public:
    /**
     * Write this parameter onto output stream
     * @param rStream
     * @param rParameter
     */
    friend std::ostream& operator << (std::ostream& rStream, const AbstractParameter& rParameter)
    {
      rStream.precision(6);
      rStream.flags(std::ios::fixed);

      rStream << rParameter.GetName() << " = " << rParameter.GetValueAsString();
      return rStream;
    }

  private:
    std::string m_Name;
    std::string m_Description;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class KARTO_EXPORT Parameter : public AbstractParameter
  {
  public:
    /**
     * Parameter with given name and value
     * @param rName
     * @param value
     */
    Parameter(const std::string& rName, T value, ParameterManager* pParameterManger = NULL)
      : AbstractParameter(rName, pParameterManger)
      , m_Value(value)
    {
    }
    
    /**
     * Parameter with given name, description and value
     * @param rName
     * @param value
     */
    Parameter(const std::string& rName, const std::string& rDescription, T value, ParameterManager* pParameterManger = NULL)
      : AbstractParameter(rName, rDescription, pParameterManger)
      , m_Value(value)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~Parameter()
    {
    }
    
  public:
    /**
     * Gets value of parameter
     * @return parameter value
     */
    inline const T& GetValue() const
    {
      return m_Value;
    }
    
    /**
     * Sets value of parameter
     * @param rValue
     */
    inline void SetValue(const T& rValue)
    {
      m_Value = rValue;
    }
    
    /**
     * Gets value of parameter as string
     * @return string version of value
     */
    virtual const std::string GetValueAsString() const
    {
      std::stringstream converter;
      converter << m_Value;
      return converter.str();
    }
    
    /**
     * Sets value of parameter from string
     * @param rStringValue
     */
    virtual void SetValueFromString(const std::string& rStringValue)
    {
      std::stringstream converter;      
      converter.str(rStringValue);
      converter >> m_Value;
    }
    
    /**
     * Clone this parameter
     * @return clone of this parameter
     */
    virtual Parameter* Clone()
    {
      return new Parameter(GetName(), GetDescription(), GetValue());
    }
    
  public:
    /**
     * Assignment operator
     */
    Parameter& operator = (const Parameter& rOther)
    {
      m_Value = rOther.m_Value;

      return *this;
    }

    /**
     * Sets the value of this parameter to given value
     */
    T operator = (T value)
    {
      m_Value = value;

      return m_Value;
    }

  protected:
    T m_Value;
  }; // AbstractParameter
    
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represent enum parameter
   */
  class ParameterEnum : public Parameter<kt_int32s>
  {
    typedef std::map<std::string, kt_int32s> EnumMap;

  public:
    /**
     * Construct a Parameter object with name and value
     * @param rName parameter name
     * @param value of parameter
     */
    ParameterEnum(const std::string& rName, kt_int32s value, ParameterManager* pParameterManger = NULL)
      : Parameter<kt_int32s>(rName, value, pParameterManger)
    {
    }

    /**
     * Destructor
     */
    virtual ~ParameterEnum()
    {
    }

  public:
    /**
     * Return a clone of this instance
     * @return clone
     */
    virtual Parameter<kt_int32s>* Clone()
    {
      ParameterEnum* pEnum = new ParameterEnum(GetName(), GetValue());

      pEnum->m_EnumDefines = m_EnumDefines;

      return pEnum;
    }

    /**
     * Set parameter value from string.
     * @param rStringValue value as string
     */
    virtual void SetValueFromString(const std::string& rStringValue)
    {
      if (m_EnumDefines.find(rStringValue) != m_EnumDefines.end())
      {
        m_Value = m_EnumDefines[rStringValue];
      }
      else
      {
        assert(false);
        throw std::runtime_error("Unable set enum");
      }
    }

    /**
     * Get parameter value as string.
     * @return value as string
     */
    virtual const std::string GetValueAsString() const
    {
      const_forEach(EnumMap, &m_EnumDefines)
      {
        if (iter->second == m_Value)
        {
          return iter->first;
        }
      }

      assert(false);
      throw std::runtime_error("Unable to lookup enum");
    }

    /**
     * Defines the enum with the given name as having the given value
     * @param value
     * @param rName
     */
    void DefineEnumValue(kt_int32s value, const std::string& rName)
    {
      if (m_EnumDefines.find(rName) == m_EnumDefines.end())
      {
        m_EnumDefines[rName] = value;
      }
      else
      {
        m_EnumDefines[rName] = value;

        std::cerr << "Overriding enum value" << std::endl;
        assert(false);
      }
    }

  public:
    /**
     * Assignment operator
     */
    ParameterEnum& operator = (const ParameterEnum& rOther)
    {
      SetValue(rOther.GetValue());
     
      return *this;
    }

    /**
     * Assignment operator
     */
    kt_int32s operator = (kt_int32s value)
    {
      SetValue(value);

      return m_Value;
    }

  private:
    EnumMap m_EnumDefines;
  }; // ParameterEnum

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class KARTO_EXPORT DatasetObject : public Object
  {
  public:
    KARTO_Object(DatasetObject)

  public:
    /**
     * Default constructor
     */
    DatasetObject()
      : Object("")
    {
      m_pParameterManager = new ParameterManager();
    }

    /**
     * Constructs a dataset object with the given name
     * @param rName
     */
    DatasetObject(const std::string& rName)
      : Object(rName)
    {
      m_pParameterManager = new ParameterManager();
    }

    /**
     * Destructor
     */
    virtual ~DatasetObject()
    {
      delete m_pParameterManager;
    }

  public:
    /**
     * Gets the parameter manager of this dataset
     * @return parameter manager
     */
    inline ParameterManager* GetParameterManager() const
    {
      return m_pParameterManager;
    }

  private:
    ParameterManager* m_pParameterManager;
  };

  typedef std::vector<DatasetObject*> DatasetObjectVector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Parameters : public DatasetObject
  {
  public:
    KARTO_Object(Parameters)

  public:
    /**
     * Parameters
     * @param rName
     */
    Parameters(const std::string& rName)
      : DatasetObject(rName)
    {
    }

    /**
     * Destructor
     */
    virtual ~Parameters()
    {
    }
  }; // Parameters

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Abstract Device base class
   */
  class KARTO_EXPORT Device : public Object
  {
  protected:
    Device(const std::string& rName, kt_int32s id)
      : Object(rName)
      , m_Id(id)
    {
      m_pParameterManager = new ParameterManager();

      m_pOffsetPose = new Parameter<Pose2>("OffsetPose", Pose2(), m_pParameterManager);
    }

  public:
    /**
     * Destructor
     */
    virtual ~Device()
    {
      delete m_pParameterManager;
      m_pParameterManager = NULL;
    }

  public:
    /**
     * Gets the id of this device
     * @return device id
     */
    inline kt_int32s GetId() const
    {
      return m_Id;
    }

    /**
     * Gets this range finder device's offset
     * @return offset pose
     */
    inline const Pose2& GetOffsetPose() const
    {
      return m_pOffsetPose->GetValue();
    }
    
    /**
     * Sets this range finder device's offset
     * @param rPose
     */
    inline void SetOffsetPose(const Pose2& rPose)
    {
      m_pOffsetPose->SetValue(rPose);
    }

    /**
     * Get parameters for device
     */
    inline ParameterManager* GetParameterManager() const
    {
      return m_pParameterManager;
    }

    /**
     * Validates device parameters
     * @return true if valid
     */
    virtual kt_bool Validate() = 0;

  private:
    /**
     * Restrict the copy constructor
     */
    Device(const Device&);

    /**
     * Restrict the assignment operator
     */
    const Device& operator=(const Device&);

  private:
    /**
     * Device id
     */
    kt_int32s m_Id;

    /**
     * Device offset pose
     */
    Parameter<Pose2>* m_pOffsetPose;

    ParameterManager* m_pParameterManager;
  }; // Device

  typedef std::vector<Device*> DeviceVector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The LaserRangeFinder defines a laser device that provides the pose offset position of a localized range scan relative to the robot.
   * The user can set an offset pose for the device relative to the robot coordinate system. If no value is provided
   * by the user, the device is set to be at the origin of the robot coordinate system.
   * The LaserRangeFinder contains parameters for physical laser device used by the mapper for scan matching
   * Also contains information about the maximum range of the device and provides a threshold
   * for limiting the range of readings.
   * The optimal value for the range threshold depends on the angular resolution of the scan and
   * the desired map resolution.  RangeThreshold should be set as large as possible while still
   * providing "solid" coverage between consecutive range readings.  The diagram below illustrates
   * the relationship between map resolution and the range threshold.
   */
  class KARTO_EXPORT LaserRangeFinder : public Device
  {
  public:
    KARTO_Object(LaserRangeFinder)

  public:
    /**
     * Destructor
     */
    virtual ~LaserRangeFinder()
    {
    }

  public:
    /**
     * Gets this range finder device's minimum range
     * @return minimum range
     */
    inline kt_double GetMinimumRange() const
    {
      return m_pMinimumRange->GetValue();
    }

    /**
     * Sets this range finder device's minimum range
     * @param minimumRange
     */
    inline void SetMinimumRange(kt_double minimumRange)
    {
      m_pMinimumRange->SetValue(minimumRange);
      
      SetRangeThreshold(GetRangeThreshold());
    }

    /**
     * Gets this range finder device's maximum range
     * @return maximum range
     */
    inline kt_double GetMaximumRange() const
    {
      return m_pMaximumRange->GetValue();
    }

    /**
     * Sets this range finder device's maximum range
     * @param maximumRange
     */
    inline void SetMaximumRange(kt_double maximumRange)
    {
      m_pMaximumRange->SetValue(maximumRange);

      SetRangeThreshold(GetRangeThreshold());
    }
    
    /**
     * Gets this range finder device's minimum angle
     * @return minimum angle
     */
    inline kt_double GetMinimumAngle() const
    {
      return m_pMinimumAngle->GetValue();
    }
    
    /**
     * Sets this range finder device's minimum angle
     * @param minimumAngle
     */
    inline void SetMinimumAngle(kt_double minimumAngle)
    {
      m_pMinimumAngle->SetValue(minimumAngle);

      Update();
    }

    /**
     * Gets the range threshold
     * @return range threshold
     */
    inline kt_double GetRangeThreshold()
    {
      return m_pRangeThreshold->GetValue();
    }

    /**
     * Sets the range threshold
     * @param rangeThreshold
     */
    inline void SetRangeThreshold(kt_double rangeThreshold)
    {
      // make sure rangeThreshold is within laser range finder range
      m_pRangeThreshold->SetValue(Math::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange()));

      if (Math::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false)
      {
        std::cout << "Info: clipped range threshold to be within minimum and maximum range!" << std::endl;
      }
    }

    /**
     * Gets this range finder device's maximum angle
     * @return maximum angle
     */
    inline kt_double GetMaximumAngle() const
    {
      return m_pMaximumAngle->GetValue();
    }
    
    /**
     * Sets this range finder device's maximum angle
     * @param maximumAngle
     */
    inline void SetMaximumAngle(kt_double maximumAngle)
    {
      m_pMaximumAngle->SetValue(maximumAngle);

      Update();
    }
    
    /**
     * Gets this range finder device's angular resolution
     * @return angular resolution
     */
    inline kt_double GetAngularResolution() const
    {
      return m_pAngularResolution->GetValue();
    }
    
    /**
     * Sets this range finder device's angular resolution
     * @param angularResolution
     */
    inline void SetAngularResolution(kt_double angularResolution)
    {
      if (m_pType->GetValue() == LaserRangeFinder_Custom)
      {
        m_pAngularResolution->SetValue(angularResolution);
      }
      else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS100)
      {
        if (Math::DoubleEqual(angularResolution, Math::DegreesToRadians(0.25)))
        {
          m_pAngularResolution->SetValue(Math::DegreesToRadians(0.25));
        }
        else if (Math::DoubleEqual(angularResolution, Math::DegreesToRadians(0.50)))
        {
          m_pAngularResolution->SetValue(Math::DegreesToRadians(0.50));
        }
        else
        {
          std::cerr << "Invalid value for Sick LMS100" << std::endl;
          assert(false);
        }
      }
      else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS200 || m_pType->GetValue() == LaserRangeFinder_Sick_LMS291)
      {
        if (Math::DoubleEqual(angularResolution, Math::DegreesToRadians(0.25)))
        {
          m_pAngularResolution->SetValue(Math::DegreesToRadians(0.25));
        }
        else if (Math::DoubleEqual(angularResolution, Math::DegreesToRadians(0.50)))
        {
          m_pAngularResolution->SetValue(Math::DegreesToRadians(0.50));
        }
        else if (Math::DoubleEqual(angularResolution, Math::DegreesToRadians(1.00)))
        {
          m_pAngularResolution->SetValue(Math::DegreesToRadians(1.00));
        }
        else
        {
          std::cerr << "Invalid value for Sick LMS291" << std::endl;
          assert(false);
        }
      }
      else
      {
        std::cerr << "Can't set AngularResolution, please create a LaserRangeFinder of type User" << std::endl;
        assert(false);

        return;
      }

      Update();
    }

    /**
     * Gets the number of range readings each localized range scan must contain to be a valid scan.
     * @return number of range readings
     */
    inline kt_int32u GetNumberOfRangeReadings() const
    {
      return m_nRangeReadings;
    }

    virtual kt_bool Validate()
    {
      Update();

      return true;
    }

  public:
    /**
     * Create a laser range finder of the given type and ID
     * @param type
     * @param id
     * @return laser range finder
     */
    static LaserRangeFinder* CreateLaserRangeFinder(std::string& rName, LaserRangeFinderType type, kt_int32s id)
    {
      LaserRangeFinder* pLrf = NULL;

      switch(type)
      {
        // see http://www.hizook.com/files/publications/SICK_LMS100.pdf
        // set range threshold to 18m
        case LaserRangeFinder_Sick_LMS100:
        {
          rName = (rName != "") ? rName : "Sick LMS 100";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 18 meters (at 10% reflectivity, max range of 20 meters), with an error of about 20mm
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(20.0);

          // 270° range, 50 Hz 
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-135)); 
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(135)); 

          // 0.25° angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(0.25)); 

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 1081;
        }
        break;

        // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
        // set range threshold to 10m
        case LaserRangeFinder_Sick_LMS200:
        {
          rName = (rName != "") ? rName : "Sick LMS 200";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 80 meters
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range, 75 Hz 
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-90)); 
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(90)); 

          // 0.5 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(0.5)); 

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 361;
        }
        break;

        // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
        // set range threshold to 30m
        case LaserRangeFinder_Sick_LMS291:
        {
          rName = (rName != "") ? rName : "Sick LMS 291";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 80 meters
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range, 75 Hz 
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-90)); 
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(90)); 

          // 0.5 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(0.5)); 

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 361;
        }
        break;

        // see http://www.hizook.com/files/publications/Hokuyo_UTM_LaserRangeFinder_LIDAR.pdf
        // set range threshold to 30m
        case LaserRangeFinder_Hokuyo_UTM_30LX:
        {
          rName = (rName != "") ? rName : "Hokuyo UTM-30LX";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 30 meters
          pLrf->m_pMinimumRange->SetValue(0.1);
          pLrf->m_pMaximumRange->SetValue(30.0);

          // 270 degree range, 40 Hz 
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-135));
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(135)); 

          // 0.25 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(0.25)); 

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 1081;
        }
        break;

        // see http://www.hizook.com/files/publications/HokuyoURG_Datasheet.pdf
        // set range threshold to 3.5m
        case LaserRangeFinder_Hokuyo_URG_04LX:
        {
          rName = (rName != "") ? rName : "Hokuyo URG-04LX";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 4 meters. It has detection problems when scanning absorptive surfaces (such as black trimming). 
          pLrf->m_pMinimumRange->SetValue(0.02);
          pLrf->m_pMaximumRange->SetValue(4.0);

          // 240 degree range, 10 Hz 
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-135)); 
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(135));

          // 0.352 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(0.352)); 

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 751;
        }
        break;

        case LaserRangeFinder_Custom:
        {
          rName = (rName != "") ? rName : "User-Defined LaserRangeFinder";
          pLrf = new LaserRangeFinder(rName, id);

          // Sensing range is 80 meters.
          pLrf->m_pMinimumRange->SetValue(0.0);
          pLrf->m_pMaximumRange->SetValue(80.0);

          // 180 degree range
          pLrf->m_pMinimumAngle->SetValue(Math::DegreesToRadians(-90)); 
          pLrf->m_pMaximumAngle->SetValue(Math::DegreesToRadians(90));

          // 1.0 degree angular resolution
          pLrf->m_pAngularResolution->SetValue(Math::DegreesToRadians(1.0));

          pLrf->m_pType->SetValue(type);
          pLrf->m_nRangeReadings = 181;
        }
        break;
      }

      if (pLrf != NULL)
      {
        Pose2 defaultOffset;
        pLrf->SetOffsetPose(defaultOffset);
      }

      return pLrf;
    }

  private:
    /**
     * Constructs a LaserRangeFinder object with given ID
     */
    LaserRangeFinder(const std::string& rName, kt_int32s id)
      : Device(rName, id)
      , m_nRangeReadings(0)
    {
      m_pMinimumRange = new Parameter<kt_double>("MinimumRange", 0.0, GetParameterManager());
      m_pMaximumRange = new Parameter<kt_double>("MaximumRange", 80.0, GetParameterManager());

      m_pMinimumAngle = new Parameter<kt_double>("MinimumAngle", -KT_PI_2, GetParameterManager());
      m_pMaximumAngle = new Parameter<kt_double>("MaximumAngle", KT_PI_2, GetParameterManager());

      m_pAngularResolution = new Parameter<kt_double>("AngularResolution", Math::DegreesToRadians(1), GetParameterManager());

      m_pRangeThreshold = new Parameter<kt_double>("RangeThreshold", 12.0, GetParameterManager());

      m_pType = new ParameterEnum("Type", LaserRangeFinder_Custom, GetParameterManager());
      m_pType->DefineEnumValue(LaserRangeFinder_Custom, "Custom");
      m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS100, "Sick_LMS100");
      m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS200, "Sick_LMS200");
      m_pType->DefineEnumValue(LaserRangeFinder_Sick_LMS291, "Sick_LMS291");
      m_pType->DefineEnumValue(LaserRangeFinder_Hokuyo_UTM_30LX, "Hokuyo_UTM_30LX");
      m_pType->DefineEnumValue(LaserRangeFinder_Hokuyo_URG_04LX, "Hokuyo_URG_04LX");
    }
    
    /**
     * Set the number of range readings based on the minimum and maximum angles of the device and the angular resolution
     */
    void Update()
    {
      m_nRangeReadings = static_cast<kt_int32u>(Math::Round((GetMaximumAngle() - GetMinimumAngle()) / GetAngularResolution()) + 1);
    }

  private:
    // device m_Parameters
    Parameter<kt_double>* m_pMinimumAngle;
    Parameter<kt_double>* m_pMaximumAngle;

    Parameter<kt_double>* m_pAngularResolution;

    Parameter<kt_double>* m_pMinimumRange;
    Parameter<kt_double>* m_pMaximumRange;

    Parameter<kt_double>* m_pRangeThreshold;

    ParameterEnum* m_pType;

    kt_int32u m_nRangeReadings;
  }; // LaserRangeFinder

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Enumerated type for valid grid cell states
   */
  typedef enum
  {
    GridStates_Unknown = 0,
    GridStates_Occupied = 100,
    GridStates_Free = 255
  } GridStates;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The CoordinateConverter class is used to convert coordinates between world and grid coordinates
   * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
   * Default scale for coordinate converter is 20 that converters to 1 pixel = 0.05 meter
   */
  class KARTO_EXPORT CoordinateConverter
  {
  public:
    /** 
     * Default constructor
     */
    CoordinateConverter()
      : m_Scale(20.0)
    {
    }

  public:
    /**
     * Scales the value
     * @param value
     * @return scaled value
     */
    inline kt_double Transform(kt_double value)
    {
      return value * m_Scale;
    }

    /**
     * Converts the point from world coordinates to grid coordinates
     * @param rWorld world coordinate
     * @return grid coordinate
     */
    inline Point2<kt_int32s> WorldToGrid(const Point2<kt_double>& rWorld) const
    {
      Point2<kt_double> scaledPoint = (rWorld - m_Offset) * m_Scale;
      return Point2<kt_int32s>(static_cast<kt_int32s>(Math::Round(scaledPoint.GetX())), static_cast<kt_int32s>(Math::Round(scaledPoint.GetY())));
    }

    /**
     * Converts the point from grid coordinates to world coordinates
     * @param rGrid world coordinate
     * @return world coordinate
     */
    inline Point2<kt_double> GridToWorld(const Point2<kt_int32s>& rGrid) const
    {
      kt_double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
      kt_double worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;

      return Point2<kt_double>(worldX, worldY);
    }

    /**
     * Gets the scale
     * @return scale
     */
    inline kt_double GetScale() const
    {
      return m_Scale;
    }

    /**
     * Sets the scale
     * @param scale
     */
    inline void SetScale(kt_double scale)
    {
      m_Scale = scale;
    }

    /**
     * Gets the offset
     * @return offset
     */
    inline const Point2<kt_double>& GetOffset() const
    {
      return m_Offset;
    }

    /**
     * Sets the offset
     * @param offset
     */
    inline void SetOffset(const Point2<kt_double>& rOffset)
    {
      m_Offset = rOffset;
    }
    
    /**
     * Gets the resolution
     * @return resolution
     */
    inline kt_double GetResolution() const
    {
      return 1.0 / m_Scale;
    }

    /**
     * Sets the resolution
     * @param resolution
     */
    inline void SetResolution(kt_double resolution)
    {
      m_Scale = 1.0 / resolution;
    }
    
  private:
    kt_double m_Scale;

    Point2<kt_double> m_Offset;
  }; // CoordinateConverter

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a grid class
   */
  template<typename T>
  class Grid
  {
  public:
    /**
     * Creates a grid of given size and resolution
     * @param width
     * @param height
     * @param resolution
     * @return grid pointer
     */
    static Grid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution)
    {
      Grid* pGrid = new Grid(width, height);
      
      pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);
      
      return pGrid;
    }
    
    /**
     * Destructor
     */
    virtual ~Grid()
    {
      delete [] m_pData;
      delete m_pCoordinateConverter;
    }

  public:
    /**
     * Clear out the grid data
     */
    void Clear()
    {
      memset(m_pData, 0, GetDataSize() * sizeof(T));
    }
    
    /**
     * Checks whether the given coordinates are valid grid indices
     * @param x
     * @param y
     */
    inline kt_bool IsValidGridIndex(const Point2<kt_int32s>& rGrid) const
    {
      return (Math::IsUpTo(rGrid.GetX(), m_Width) && Math::IsUpTo(rGrid.GetY(), m_Height));
    }
    
    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck default value is true
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Point2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
    {
      if (boundaryCheck == true)
      { 
        if (IsValidGridIndex(rGrid) == false)
        {
          std::stringstream error;
          error << "Index " << rGrid << " out of range.  Index must be between [0; " << m_Width << "] and [0; " << m_Height << "]";
          throw std::runtime_error(error.str());          
        }
      }

      kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);
      
      if (boundaryCheck == true)
      {
        assert(Math::IsUpTo(index, GetDataSize()));
      }
      
      return index;
    }

    Point2<kt_int32s> IndexToGrid(kt_int32s index) const
    {
      Point2<kt_int32s> grid;

      grid.SetY(index / m_WidthStep);
      grid.SetX(index - grid.GetY() * m_WidthStep);

      return grid;
    }

    /**
     * Gets pointer to data at given grid coordinate
     * @param rGrid grid coordinate
     * @return grid point
     */
    T* GetDataPointer(const Point2<kt_int32s>& rGrid)
    {
      kt_int32s index = GridIndex(rGrid, true);      
      return m_pData + index;
    }

    /**
     * Gets pointer to data at given grid coordinate
     * @param rGrid grid coordinate
     * @return grid point
     */
    T* GetDataPointer(const Point2<kt_int32s>& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid, true);      
      return m_pData + index;
    }

    /**
     * Gets the width of the grid
     * @return width of the grid
     */
    inline kt_int32s GetWidth() const
    {
      return m_Width;
    };

    /**
     * Gets the height of the grid
     * @return height of the grid
     */
    inline kt_int32s GetHeight() const
    {
      return m_Height;
    };

    /**
     * Gets the width step in bytes
     * @return width step
     */
    inline kt_int32s GetWidthStep() const
    {
      return m_WidthStep;
    }

    /**
     * Gets the grid data pointer
     * @return data pointer
     */
    inline T* GetDataPointer()
    {
      return m_pData;
    }

    /**
     * Gets const grid data pointer
     * @return data pointer
     */
    inline T* GetDataPointer() const
    {
      return m_pData;
    }

    /**
     * Gets the allocated grid size in bytes
     * @return data size
     */
    inline kt_int32s GetDataSize() const
    {
      return m_WidthStep * m_Height;
    }

    /**
     * Get value at given grid coordinate
     * @param rGrid grid coordinate
     * @return value
     */
    inline T GetValue(const Point2<kt_int32s>& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid);
      return m_pData[index];
    }

    /**
     * Gets the coordinate converter for this grid
     * @return coordinate converter
     */
    inline CoordinateConverter* GetCoordinateConverter() const
    {
      return m_pCoordinateConverter;
    }

    public:
    /**
     * Increments all the grid cells from (x0, y0) to (x1, y1)
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     */
    void TraceLine(kt_int32s x0, kt_int32s y0, kt_int32s x1, kt_int32s y1)
    {
      kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
      if (steep)
      {
        std::swap(x0, y0);
        std::swap(x1, y1);
      }
      if (x0 > x1)
      {
        std::swap(x0, x1);
        std::swap(y0, y1);
      }

      kt_int32s deltaX = x1 - x0;
      kt_int32s deltaY = abs(y1 - y0);
      kt_int32s error = 0;
      kt_int32s ystep;
      kt_int32s y = y0;

      if (y0 < y1)
      {
        ystep = 1;
      }
      else
      {
        ystep = -1;
      }

      kt_int32s pointX;
      kt_int32s pointY;
      for (kt_int32s x = x0; x <= x1; x++)
      {
        if (steep)
        {
          pointX = y;
          pointY = x;
        }
        else
        {
          pointX = x;
          pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
          y += ystep;
          error -= deltaX;
        }

        Point2<kt_int32s> gridIndex(pointX, pointY);
        if (IsValidGridIndex(gridIndex))
        {
          kt_int32s index = GridIndex(gridIndex, false);
          T* pGridPointer = GetDataPointer();

          pGridPointer[index]++;
        }
      }
    }

  protected:
    /**
     * Constructs grid of given size
     * @param width
     * @param height
     */
    Grid(kt_int32s width, kt_int32s height)
    : m_Width(width)
    , m_Height(height)
    , m_WidthStep(0)
    , m_pData(NULL)
    , m_pCoordinateConverter(new CoordinateConverter())
    {
      m_WidthStep = Math::AlignValue<kt_int32s>(width, 8);
      m_pData = new T[GetDataSize()];
      
      Clear();
    }
    
  private:
    kt_int32s m_Width;       // width of grid
    kt_int32s m_Height;      // height of grid
    kt_int32s m_WidthStep;   // 8 bit aligned width of grid
    T* m_pData;              // grid data

    CoordinateConverter* m_pCoordinateConverter; // coordinate converter to convert between world coordinates and grid coordinates
  }; // Grid
 
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class CustomData : public Object
  {
  public:
    KARTO_Object(CustomData)

  public:
    CustomData()
      : Object("")
    {
    }

    virtual ~CustomData()
    {
    }

  public:
    virtual const std::string Write() const = 0;
    virtual void Read(const std::string& rValue) = 0;
  };

  typedef std::vector<CustomData*> CustomDataVector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * DeviceState is a base class for all device states
   */
  class KARTO_EXPORT DeviceState : public DatasetObject
  {
  public:
    KARTO_Object(DeviceState)

  public:
    /**
     * Destructor
     */
    virtual ~DeviceState()
    {
    }

  public:
    /**
     * Gets device state id
     * @return id
     */
    inline kt_int32s GetId() const
    {
      return m_Id;
    }

    /**
     * Sets device state id
     * @param id
     */
    inline void SetId(kt_int32s id)
    {
      m_Id = id;
    }

    /**
     * Gets device state time
     * @return time
     */
    inline kt_double GetTime() const
    {
      return m_Time;
    }

    /**
     * Sets device state time
     * @param time
     */
    inline void SetTime(kt_double time)
    {
      m_Time = time;
    }

    /**
     * Get the device that created this device state
     * @return device
     */
    inline Device* GetDevice() const
    {
      return m_pDevice;
    }

    /**
     * Validate that the device state contains valid data
     * @return true if valid
     */
    virtual kt_bool Validate() = 0;

    /**
     * Add a CustomData object to device state
     * @param pCustomData 
     */
    inline void AddCustomData(CustomData* pCustomData)
    {
      m_CustomData.push_back(pCustomData);
    }

    /**
     * Get all custom data objects assigned to device state
     * @return CustomDataVector&
     */
    inline const CustomDataVector& GetCustomData() const
    {
      return m_CustomData;
    }

  protected:
    DeviceState(Device* pDevice)
      : DatasetObject()
      , m_Id(-1)
      , m_pDevice(pDevice)
    {
    }
    
  private:
    /**
     * Restrict the copy constructor
     */
    DeviceState(const DeviceState&);

    /**
     * Restrict the assignment operator
     */
    const DeviceState& operator=(const DeviceState&);

  private:
    /**
     * Device state id
     */
    kt_int32s m_Id;

    /**
     * Device that created this device state
     */
    Device* m_pDevice;

    /**
     * Time the device state was created
     */
    kt_double m_Time;

    CustomDataVector m_CustomData;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  typedef std::vector<kt_double> RangeReadingsVector;

  /**
   * LaserRangeScan representing the range readings from a laser range finder device.
   */
  class KARTO_EXPORT LaserRangeScan : public DeviceState
  {
  public:
    KARTO_Object(LaserRangeScan)

  public:
    /**
     * Constructs a scan from the given device with the given readings
     * @param pDevice
     * @param rReadings
     */
    LaserRangeScan(LaserRangeFinder* pLrf, const RangeReadingsVector& rReadings)
      : DeviceState(pLrf)
      , m_pRangeReadings(NULL)
    {
      assert(pLrf != NULL);
      
      kt_int32u nRangeReadings = pLrf->GetNumberOfRangeReadings();
      if (rReadings.size() != nRangeReadings)
      {
        std::stringstream error;
        error << "Given number of readings (" << rReadings.size() << ") does not match expected number of range finder (" << nRangeReadings << ")";
        throw std::runtime_error(error.str());
      }
      
      if (pLrf->GetNumberOfRangeReadings() > 0)
      {
        // allocate range readings
        m_pRangeReadings = new kt_double[nRangeReadings];

        // copy readings
        kt_int32u index = 0;
        const_forEach(RangeReadingsVector, &rReadings)
        {
          m_pRangeReadings[index++] = *iter;
        }
      }
    }

    /**
     * Destructor
     */
    virtual ~LaserRangeScan()
    {
      delete [] m_pRangeReadings;
    }

  public:
    /**
     * Gets the range readings of this scan
     * @return range readings of this scan
     */
    inline kt_double* GetRangeReadings() const
    {
      return m_pRangeReadings;
    }

    /**
     * Gets the number of range readings
     * @return number of range readings
     */
    inline kt_int32u GetNumberOfReadings() const
    {
      return dynamic_cast<LaserRangeFinder*>(GetDevice())->GetNumberOfRangeReadings();
    }
    
  private:
    kt_double* m_pRangeReadings;
  }; // LaserRangeScan

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * DrivePose representing the pose value of a drive device.
   */
  class DrivePose : public DeviceState
  {
  public:
    KARTO_Object(DrivePose)

  public:
    /**
     * Constructs a pose of the given drive device
     * @param pDevice
     */
    DrivePose(Device* pDevice)
      : DeviceState(pDevice)
    {
    }

    /**
     * Destructor
     */
    virtual ~DrivePose()
    {
    }

  public:
    /**
     * Gets the odometric pose of this scan
     * @return odometric pose of this scan
     */
    inline const Pose2& GetOdometricPose() const 
    {
      return m_OdometricPose;
    }

    /**
     * Sets the odometric pose of this scan
     * @param rPose
     */
    inline void SetOdometricPose(const Pose2& rPose)
    {
      m_OdometricPose = rPose;
    }

  private:
    /**
     * Odometric pose of robot
     */
    Pose2 m_OdometricPose;
  }; // class DrivePose

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The LocalizedRangeScan contains range data from a single sweep of a laser range finder device
   * in a two-dimensional space and position information. The odometer position is the position 
   * reported by the robot when the range data was recorded. The corrected position is the position
   * calculated by the mapper (or localizer)
   */
  class KARTO_EXPORT LocalizedRangeScan : public LaserRangeScan
  {
  public:
    KARTO_Object(LocalizedRangeScan)

  public:
    /**
     * Constructs a range scan from the given range finder with the given readings
     */
    LocalizedRangeScan(LaserRangeFinder* pLrf, const RangeReadingsVector& rReadings)
      : LaserRangeScan(pLrf, rReadings)
      , m_IsDirty(true)
    {
    }

    /**
     * Destructor
     */
    virtual ~LocalizedRangeScan()
    {
    }

  public:
    /**
     * Gets the odometric pose of this scan
     * @return odometric pose of this scan
     */
    inline const Pose2& GetOdometricPose() const 
    {
      return m_OdometricPose;
    }

    /**
     * Sets the odometric pose of this scan
     * @param rPose
     */
    inline void SetOdometricPose(const Pose2& rPose)
    {
      m_OdometricPose = rPose;
    }

    /**
     * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
     * is usually set by an external module such as a localization or mapping module when it is determined
     * that the original pose was incorrect.  The external module will set the correct pose based on
     * additional sensor data and any context information it has.  If the pose has not been corrected,
     * a call to this method returns the same pose as GetOdometricPose().
     * @return corrected pose
     */
    inline const Pose2& GetCorrectedPose() const 
    {
      return m_CorrectedPose;
    }

    /**
     * Moves the scan by moving the robot pose to the given location.
     * @param rPose new pose of the robot of this scan
     */
    inline void SetCorrectedPose(const Pose2& rPose)
    {
      m_CorrectedPose = rPose;

      m_IsDirty = true;
    }

    /**
     * Gets barycenter of point readings
     */
    inline const Pose2& GetBarycenterPose() const 
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedRangeScan*>(this)->Update();
      }

      return m_BarycenterPose;
    }
    
    /**
     * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
     * @param useBarycenter
     * @return barycenter if parameter is true, otherwise scanner pose
     */
    inline Pose2 GetReferencePose(kt_bool useBarycenter) const 
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedRangeScan*>(this)->Update();
      }
      
      return useBarycenter ? m_BarycenterPose : GetScannerPose();
    }    

    /**
     * Computes the position of the scanner
     * @return scan pose
     */
    inline Pose2 GetScannerPose() const
    {
      return GetScannerAt(m_CorrectedPose);
    }
    
    /**
     * Computes the position of the scanner if the robot were at the given pose
     * @param rPose
     * @return scanner pose
     */
    inline Pose2 GetScannerAt(const Pose2& rPose) const
    {
      return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
    }    
    
    /**
     * Computes the robot pose given the corrected scan pose
     * @param rScanPose pose of scanner device
     */
    void SetScannerPose(const Pose2& rScanPose)
    {
      Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
      kt_double offsetLength = deviceOffsetPose2.Length();
      kt_double offsetHeading = deviceOffsetPose2.GetHeading();
      kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
      kt_double correctedHeading = Math::NormalizeAngle(rScanPose.GetHeading());
      Pose2 worldDeviceOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                                      offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                                      offsetHeading);

      m_CorrectedPose = rScanPose - worldDeviceOffset;

      m_IsDirty = true;
      Update();
    }
    
    /**
     * Gets the laser range finder device that generated this scan
     * @return laser range finder device of this scan
     */
    inline LaserRangeFinder* GetLaserRangeFinder() const
    {
      return dynamic_cast<LaserRangeFinder*>(GetDevice());
    }

    /**
     * Gets the bounding box of this scan
     * @return bounding box of this scan
     */
    inline const BoundingBox2& GetBoundingBox() const
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedRangeScan*>(this)->Update();
      }

      return m_BoundingBox;
    }

    /**
     * Get point readings in local coordinates
     */
    inline const PointVectorDouble& GetPointReadings() const
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedRangeScan*>(this)->Update();
      }

      return m_PointReadings;
    }

    /**
     * Get point readings (potentially scale readings if given coordinate converter is not null)
     * @param pCoordinateConverter
     */
    inline const PointVectorDouble GetPointReadings(CoordinateConverter* pCoordinateConverter, kt_bool ignoreThresholdPoints = true) const
    {
      PointVectorDouble pointReadings;

      LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

      kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetScannerPose();

      // compute point readings
      kt_int32u beamNum = 0;
      for (kt_int32u i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++)
      {
        kt_double rangeReading = GetRangeReadings()[i];

        if (ignoreThresholdPoints)
        {
          if (!Math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
          {
            continue;
          }
        }
        else
        {
          rangeReading = Math::Clip(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold);
        }

        if (pCoordinateConverter != NULL)
        {
          pCoordinateConverter->Transform(rangeReading);
        }

        kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Point2<kt_double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        pointReadings.push_back(point);
      }

      return pointReadings;
    }

    /**
     * Validate that scans contains valid data
     */ 
    kt_bool Validate()
    {
      LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

      if (pLaserRangeFinder == NULL)     
      {
        return false;
      }

      if (Math::InRange(pLaserRangeFinder->GetRangeThreshold(), pLaserRangeFinder->GetMinimumRange(), pLaserRangeFinder->GetMaximumRange()) == false)
      {
        std::cout << "Please set range threshold to a value between [" << pLaserRangeFinder->GetMinimumRange() << ";" << pLaserRangeFinder->GetMaximumRange() << "]" << std::endl;
        return false;
      }

      return pLaserRangeFinder->Validate();
    }

  private:    
    /**
     * Compute point readings based on range readings (potentially scale readings if given coordinater is not null)
     * Only range readings within [minimum range; range threshold] are returned
     * @param pCoordinateConverter
     */
    void Update()
    {
      LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

      m_PointReadings.clear();

      kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetScannerPose();

      // compute point readings
      Point2<kt_double> pointSum;
      kt_int32u beamNum = 0;
      for (kt_int32u i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++)
      {
        kt_double rangeReading = GetRangeReadings()[i];

        if (!Math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
        {
          continue;
        }

        kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Point2<kt_double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        m_PointReadings.push_back(point);
        pointSum += point;
      }

      // compute barycenter
      kt_double nPoints = static_cast<kt_double>(m_PointReadings.size());
      if (nPoints != 0.0)
      {
        m_BarycenterPose = Pose2(pointSum / nPoints, 0.0);
      }
      else
      {
        m_BarycenterPose = scanPose;
      }

      // calculate bounding box of scan
      m_BoundingBox = BoundingBox2();
      m_BoundingBox.Add(Point2<kt_double>(scanPose.GetPosition()));
      forEach(PointVectorDouble, &m_PointReadings)
      {
        m_BoundingBox.Add(*iter);
      }    

      // only clear dirty flag if ignoreThresholdPoints == true. ignoreThresholdPoints = false is only used
      // in creating occupancy grid
      m_IsDirty = false;
    }

  private:
    /**
     * Odometric pose of robot
     */
    Pose2 m_OdometricPose;
        
    /**
     * Corrected pose of robot calculated by mapper (or localizer)
     */
    Pose2 m_CorrectedPose;

    /**
     * Average of all the point readings
     */
    Pose2 m_BarycenterPose;

    /**
     * Internal flag used to update point readings, barycenter and bounding box
     */
    kt_bool m_IsDirty;

    /**
     * Vector of point readings
     */
    PointVectorDouble m_PointReadings;

    /**
     * Bounding box of localized range scan
     */
    BoundingBox2 m_BoundingBox;
  }; // LocalizedRangeScan

  typedef std::vector<LocalizedRangeScan*> LocalizedRangeScanVector;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Occupancy grid definition. See GridStates for possible grid values.
   */
  class KARTO_EXPORT OccupancyGrid : public Grid<kt_int8u>
  {
  public:
    /**
     * Destructor
     */
    virtual ~OccupancyGrid()
    {
    }

  public:    
    /**
     * Create an occupancy grid from the given scans using the given resolution
     * @param scans
     * @param resolution
     */
    static OccupancyGrid* CreateFromScans(LocalizedRangeScanVector scans, kt_double resolution)
    {
      assert(resolution != 0.0);

      if (scans.size() == 0)
      {
        return NULL;
      }
      
      BoundingBox2 boundingBox;
      forEach(LocalizedRangeScanVector, &scans)
      {
        boundingBox.Add((*iter)->GetBoundingBox());
      }
      
      OccupancyGrid* pOccupancyGrid = OccupancyGrid::CreateOccupancyGrid(boundingBox, resolution);      
      pOccupancyGrid->CreateFromScans(scans);

      return pOccupancyGrid;
    }

    /**
     * Make a clone 
     */
    OccupancyGrid* Clone()
    {
      OccupancyGrid* pOccupancyGrid = new OccupancyGrid(GetWidth(), GetHeight());
      memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

      pOccupancyGrid->GetCoordinateConverter()->SetScale(GetCoordinateConverter()->GetScale());
      pOccupancyGrid->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

      return pOccupancyGrid;
    }

  private:
    /**
     * Constructs an occupancy grid of given size
     * @param width
     * @param height
     */
    OccupancyGrid(kt_int32s width, kt_int32s height) 
      : Grid<kt_int8u>(width, height)
      , m_pCellPassCnt(NULL)
      , m_pCellHitsCnt(NULL)
    {
    }
    
    /**
     * Create an occupancy grid from a bounding box and resolution
     * @param boundingBox
     * @param resolution
     */
    static OccupancyGrid* CreateOccupancyGrid(const BoundingBox2& rBoundingBox, kt_double resolution)
    {
      assert(resolution != 0.0);
      
      kt_double scale = 1.0 / resolution;
      
      Size2<kt_double> size = rBoundingBox.GetSize();
      
      OccupancyGrid* pOccupancyGrid = new OccupancyGrid(static_cast<kt_int32s>(Math::Round(size.GetWidth() * scale)), static_cast<kt_int32s>(Math::Round(size.GetHeight() * scale)));
      pOccupancyGrid->GetCoordinateConverter()->SetScale(scale);
      pOccupancyGrid->GetCoordinateConverter()->SetOffset(rBoundingBox.GetMinimum());
      
      return pOccupancyGrid;
    }

    /**
     * Create grid using scans 
     * @param scans
     */
    void CreateFromScans(LocalizedRangeScanVector scans)
    {
      m_pCellPassCnt = Grid<kt_int32u>::CreateGrid(GetWidth(), GetHeight(), GetCoordinateConverter()->GetResolution());
      m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());
      
      m_pCellHitsCnt = Grid<kt_int32u>::CreateGrid(GetWidth(), GetHeight(), GetCoordinateConverter()->GetResolution());
      m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());
      
      forEach(LocalizedRangeScanVector, &scans)
      {
        LocalizedRangeScan* pScan = *iter;
        
        kt_double rangeThreshold = pScan->GetLaserRangeFinder()->GetRangeThreshold();
        
        Point2<kt_double> scanPosition = pScan->GetScannerPose().GetPosition();
        
        // get scan point readings 
        const PointVectorDouble& rPointReadings = pScan->GetPointReadings(GetCoordinateConverter());      
        
        // draw lines from scan position to all point readings 
        int pointIndex = 0;
        const_forEachAs(PointVectorDouble, &rPointReadings, pointsIter)
        {
          RayTrace(scanPosition, *pointsIter, pScan->GetRangeReadings()[pointIndex] > (rangeThreshold - KT_TOLERANCE));
          pointIndex++;
        }
      }
      
      Update();
      
      delete m_pCellPassCnt;
      m_pCellPassCnt = NULL;
      
      delete m_pCellHitsCnt;   
      m_pCellHitsCnt = NULL;
    }
    
    /**
     * Traces a beam from the start position to the end position marking
     * the bookkeeping arrays accordingly.
     * @param rWorldFrom start position of beam
     * @param rWorldTo end position of beam
     * @param isMaxReading is the length of this beam a max range reading?
     * @param pCoordinatorConverter converter from world to grid coordinates
     */
    void RayTrace(const Point2<kt_double>& rWorldFrom, const Point2<kt_double>& rWorldTo, kt_bool isMaxReading)
    {
      assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

      Point2<kt_int32s> gridFrom = m_pCellPassCnt->GetCoordinateConverter()->WorldToGrid(rWorldFrom);
      Point2<kt_int32s> gridTo = m_pCellPassCnt->GetCoordinateConverter()->WorldToGrid(rWorldTo);

      m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(), gridTo.GetY());

      // for the end point
      if (!isMaxReading)
      {
        if (m_pCellPassCnt->IsValidGridIndex(gridTo))
        {
          kt_int32s index = m_pCellPassCnt->GridIndex(gridTo, false);

          kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
          kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();
  
          // increment cell pass through and hit count
          pCellPassCntPtr[index]++;
          pCellHitCntPtr[index]++;
        }
      }
    }

    /**
     * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
     *
     * @param minPassThrough Number of beams that must pass through a cell before it will be considered
     * to be occupied or unoccupied.  This prevents stray beams from messing up the map.
     * @param occupancyThreshold Minimum ratio of beams hitting cell to beams passing through cell for
     * cell to be marked as occupied
     *
     * NOTE: These two values are dependent on the resolution.  If the resolution is too small,
     * then not many beams will hit the cell!
     */
    void Update(kt_int32u minPassThrough = 2, kt_double occupancyThreshold = 0.1)
    {
      assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);
      
      // clear grid
      Clear();

      // set occupancy status of cells
      kt_int8u* pDataPtr = GetDataPointer();
      kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
      kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

      kt_int32u nBytes = GetDataSize();
      for (kt_int32u i = 0; i < nBytes; i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++)
      {
        if (*pCellPassCntPtr > minPassThrough)
        {
          kt_double hitRatio = static_cast<kt_double>(*pCellHitCntPtr) / static_cast<kt_double>(*pCellPassCntPtr);

          if (hitRatio > occupancyThreshold)
          {
            *pDataPtr = GridStates_Occupied;
          }
          else
          {
            *pDataPtr = GridStates_Free;
          }
        }
      }
    }
    
  private:
    // Counters of number of times a beam passed through a cell
    Grid<kt_int32u>* m_pCellPassCnt;
    
    // Counters of number of times a beam ended at a cell    
    Grid<kt_int32u>* m_pCellHitsCnt;
  }; // OccupancyGrid

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * \ingroup kt
   * Dataset info
   * Contains title, author and other information about the dataset
   */
  class DatasetInfo : public DatasetObject
  {
  public:
    KARTO_Object(DatasetInfo)

  public:
    DatasetInfo()
    {
      m_pTitle = new Parameter<std::string>("Title", "", GetParameterManager());
      m_pAuthor = new Parameter<std::string>("Author", "", GetParameterManager());
      m_pDescription = new Parameter<std::string>("Description", "", GetParameterManager());
      m_pCopyright = new Parameter<std::string>("Copyright", "", GetParameterManager());
    }

    virtual ~DatasetInfo()
    {
    }

  public:
    /**
     * Dataset title
     */
    const std::string& GetTitle() const
    {
      return m_pTitle->GetValue();
    }

    /**
     * Dataset author(s)
     */
    const std::string& GetAuthor() const
    {
      return m_pAuthor->GetValue();
    }

    /**
     * Dataset description
     */
    const std::string& GetDescription() const
    {
      return m_pDescription->GetValue();
    }

    /**
     * Dataset copyrights
     */
    const std::string& GetCopyright() const
    {
      return m_pCopyright->GetValue();
    }

    /**
     * Dataset modified time
     */
//    kt::DateTime Modified;

    /**
     * Dataset created time
     */
//    kt::DateTime Created;

    /**
     * Dataset revision number
     */
//    kt_int16u Revision;

  private:
    Parameter<std::string>* m_pTitle;
    Parameter<std::string>* m_pAuthor;
    Parameter<std::string>* m_pDescription;
    Parameter<std::string>* m_pCopyright;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Karto dataset. Stores LocalizedRangeScans and manages memory of allocated LaserRangeFinders 
   * and LocalizedRangeScans
   */
  class Dataset
  {
    typedef std::map<kt_int32s, Device*> DeviceMap;

  public:
    /**
     * Default constructor
     */
    Dataset()
      : m_pDatasetInfo(NULL)
    {
    }

    /**
     * Destructor
     */
    virtual ~Dataset()
    {
      Clear();
    }

  public:
    /**
     * Adds scan to this dataset
     * @param pScan
     */
    void Add(DatasetObject* pDatasetObject)
    {
      if (pDatasetObject != NULL)
      {
        if (dynamic_cast<DeviceState*>(pDatasetObject))
        {
          DeviceState* pDeviceState = dynamic_cast<DeviceState*>(pDatasetObject);
          if (pDeviceState->Validate())
          {
            m_DatasetObjects.push_back(pDeviceState);

            kt_int32s id = pDeviceState->GetDevice()->GetId();
            if (m_Devices.find(id) == m_Devices.end())
            {
              m_Devices[id] = pDeviceState->GetDevice();
            }
          }
        }
        else if (dynamic_cast<DatasetInfo*>(pDatasetObject))
        {
          m_pDatasetInfo = dynamic_cast<DatasetInfo*>(pDatasetObject);
        }
        else
        {
          m_DatasetObjects.push_back(pDatasetObject);
        }
      }
    }

    /**
     * Get device states
     * @return device state
     */
    inline const DatasetObjectVector& GetDatasetObjects() const
    {
      return m_DatasetObjects;
    }

    inline DatasetInfo* GetDatasetInfo()
    {
      return m_pDatasetInfo;
    }

    /**
     * Delete all stored data
     */
    void Clear()
    {
      forEach(DatasetObjectVector, &m_DatasetObjects)
      {
        delete *iter;
      }
      m_DatasetObjects.clear();

      forEach(DeviceMap, &m_Devices)
      {
        delete iter->second;
      }
      m_Devices.clear();

      if (m_pDatasetInfo != NULL)
      {
        delete m_pDatasetInfo;
        m_pDatasetInfo = NULL;
      }
    }

  private:
    DatasetObjectVector m_DatasetObjects;
    DeviceMap m_Devices;
    DatasetInfo* m_pDatasetInfo;
  };
}

#endif // __KARTO_TYPES__

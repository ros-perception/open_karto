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

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>

#include <math.h>
#include <assert.h>

#include "kt_Mapper.h"

namespace karto
{

  // enable this for verbose debug information
  //#define KARTO_DEBUG

  #define MAX_VARIANCE           500.0

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages the scan data for a device
   */
  class ScanManager
  {
  public:
    /**
     * Default constructor
     */
    ScanManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_pLastScan(NULL)
      , m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~ScanManager()
    {
      Clear();
    }

  public:
    /**
     * Adds scan to vector of processed scans
     * @param pScan
     */
    inline void AddScan(LocalizedRangeScan* pScan)
    {
      // assign id to scan
      pScan->SetId(m_Scans.size());
      
      // add it to scan buffer
      m_Scans.push_back(pScan);
    }
    
    /**
     * Gets last scan
     * @param deviceId
     * @return last localized range scan
     */
    inline LocalizedRangeScan* GetLastScan()
    {
      return m_pLastScan;
    }

    /**
     * Sets the last scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan)
    {
      m_pLastScan = pScan;
    }

    /**
     * Gets scans
     * @return scans
     */
    inline LocalizedRangeScanVector& GetScans()
    {
      return m_Scans;
    }

    /**
     * Gets running scans
     * @return running scans
     */
    inline LocalizedRangeScanVector& GetRunningScans()
    {
      return m_RunningScans;
    }

    /**
     * Adds scan to vector of running scans
     * @param pScan
     */
    void AddRunningScan(LocalizedRangeScan* pScan)
    {
      m_RunningScans.push_back(pScan);
      
      // vector has at least one element (first line of this function), so this is valid
      Pose2 frontScanPose = m_RunningScans.front()->GetScannerPose();
      Pose2 backScanPose = m_RunningScans.back()->GetScannerPose();
      
      // cap vector size and remove all scans from front of vector that are too far from end of vector
      kt_double squaredDistance = frontScanPose.SquaredDistance(backScanPose);
      while (m_RunningScans.size() > m_RunningBufferMaximumSize || squaredDistance > Math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
      {
        // remove front of running scans
        m_RunningScans.erase(m_RunningScans.begin());
        
        // recompute stats of running scans
        frontScanPose = m_RunningScans.front()->GetScannerPose();
        backScanPose = m_RunningScans.back()->GetScannerPose();
        squaredDistance = frontScanPose.SquaredDistance(backScanPose);
      }
    }
    
    /**
     * Deletes data of this buffered device
     */
    void Clear()
    {
      m_Scans.clear();
      m_RunningScans.clear();
    }
    
  private:
    LocalizedRangeScanVector m_Scans;
    LocalizedRangeScanVector m_RunningScans;
    LocalizedRangeScan* m_pLastScan;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;    
  }; // ScanManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages the devices of a mapper
   */
  class DeviceManager
  {
    typedef std::map<kt_int32s, ScanManager*> ScanManagerMap;
    
  public:
    /**
     * 
     */
    DeviceManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~DeviceManager()
    {
      Clear();
    }

  public:
    /**
     * Gets scan from given device with given ID 
     * @param deviceId
     * @param scanNum
     * @return localized range scan
     */
    inline LocalizedRangeScan* GetScan(kt_int32s deviceId, kt_int32s scanIndex)
    {
      return GetScanManager(deviceId)->GetScans().at(scanIndex);
    }

    /**
     * Gets IDs of all devices
     * @return device IDs
     */
    inline std::vector<kt_int32s> GetDeviceIds()
    {
      std::vector<kt_int32s> ids;
      const_forEach(ScanManagerMap, &m_ScanManagers)
      {
        ids.push_back(iter->first);
      }
      return ids;
    }
    
    /**
     * Gets last scan of given device
     * @param deviceId
     * @return last localized range scan of device
     */
    inline LocalizedRangeScan* GetLastScan(LaserRangeFinder* pLaserRangeFinder)
    {
      kt_int32s id = pLaserRangeFinder->GetId();

      // verify that we have a scan manager for device
      if (m_ScanManagers.find(id) == m_ScanManagers.end())
      {
        m_ScanManagers[id] = new ScanManager(m_RunningBufferMaximumSize, m_RunningBufferMaximumDistance);
      }

      return GetScanManager(id)->GetLastScan();
    }

    /**
     * Sets the last scan of device of given scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan)
    {
      GetScanManager(pScan)->SetLastScan(pScan);
    }

    /**
     * Adds scan to scan vector of device that recorded scan
     * @param pScan
     */
    inline void AddScan(LocalizedRangeScan* pScan)
    {
      GetScanManager(pScan)->AddScan(pScan);
    }

    /**
     * Adds scan to running scans of device that recorded scan
     * @param pScan
     */
    inline void AddRunningScan(LocalizedRangeScan* pScan)
    {
      GetScanManager(pScan)->AddRunningScan(pScan);
    }

    /**
     * Gets scans of device
     * @param deviceId
     * @return scans of device
     */
    inline LocalizedRangeScanVector& GetScans(kt_int32s deviceId)
    {
      return GetScanManager(deviceId)->GetScans();
    }

    /**
     * Gets running scans of device
     * @param deviceId
     * @return running scans of device
     */
    inline LocalizedRangeScanVector& GetRunningScans(kt_int32s deviceId)
    {
      return GetScanManager(deviceId)->GetRunningScans();
    }

    /**
     * Gets all scans of all devices
     * @return all scans of all devices
     */
    LocalizedRangeScanVector GetAllScans()
    {
      LocalizedRangeScanVector scans;

      forEach(ScanManagerMap, &m_ScanManagers)
      {
        LocalizedRangeScanVector& rScans = iter->second->GetScans();

        scans.insert(scans.end(), rScans.begin(), rScans.end());
      }

      return scans;
    }

    /**
     * Deletes all scan managers of all devices
     */
    void Clear()
    {
      forEach(ScanManagerMap, &m_ScanManagers)
      {
        delete iter->second;
      }

      m_ScanManagers.clear();
    }

  private:
    /**
     * Get scan manager for localized range scan
     * @return ScanManager
     */
    ScanManager* GetScanManager(LocalizedRangeScan* pScan)
    {
      return GetScanManager(pScan->GetLaserRangeFinder()->GetId());
    }

    /**
     * Get scan manager for id
     * @return ScanManager
     */
    ScanManager* GetScanManager(kt_int32s id)
    {
      return m_ScanManagers[id];
    }

  private:
    // map from device ID to scan data
    ScanManagerMap m_ScanManagers;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
  }; // DeviceManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * An array that can be resized as long as the size
   * does not exceed the initial capacity
   */
  class LookupArray
  {
  public:
    /**
     * Constructs lookup array
     * @param arraySize
     */
    LookupArray()
      : m_pArray(NULL)
      , m_Capacity(0)
      , m_Size(0)
    {
      m_pArray = new kt_int32s[m_Capacity];
    }

    /**
     * Destructor
     */
    virtual ~LookupArray()
    {
      assert(m_pArray != NULL);
      
      delete[] m_pArray;
      m_pArray = NULL;
    }

  public:
    /**
     * Clear array
     */
    void Clear()
    {
      memset(m_pArray, 0, sizeof(kt_int32s) * m_Capacity);
    }

    /**
     * Gets size of array
     * @return array size
     */
    kt_int32u GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets size of array (resize if not big enough)
     * @param size
     */
    void SetSize(kt_int32u size)
    {
      assert(size != 0);
      
      if (size > m_Capacity)
      {
        if (m_pArray != NULL)
        {
          delete [] m_pArray;
        }
        m_Capacity = size;
        m_pArray = new kt_int32s[m_Capacity];
      }
      
      m_Size = size;
    }

    /**
     * Gets reference to value at given index
     * @param index
     * @return reference to value at index
     */
    inline kt_int32s& operator [] (kt_int32u index) 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets value at given index
     * @param index
     * @return value at index
     */
    inline kt_int32s operator [] (kt_int32u index) const 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer()
    {
      return m_pArray;
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer() const
    {
      return m_pArray;
    }

  private:
    kt_int32s* m_pArray;
    kt_int32u m_Capacity;
    kt_int32u m_Size;
  }; // LookupArray

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Create lookup tables for point readings at varying angles in grid.
   * For each angle, grid indexes are calculated for each range reading.
   * This is to speed up finding best angle/position for a localized range scan
   * 
   * Used heavily in mapper and localizer. 
   * 
   * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
   * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
   * calculated.  So when calculating the particle probability at a specific angle, the index table is used
   * to look up probability in probability grid!
   * 
   */
  template<typename T>
  class GridIndexLookup
  {
  public:
    GridIndexLookup(Grid<T>* pGrid)
      : m_pGrid(pGrid)
      , m_Capacity(0)
      , m_Size(0)
      , m_ppLookupArray(NULL)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~GridIndexLookup()
    {
      DestroyArrays();
    }

  public:
    /**
     * Gets the lookup array for a particular angle index
     * @param index
     * @return lookup array
     */
    const LookupArray* GetLookupArray(kt_int32u index) const
    {
      assert(Math::IsUpTo(index, m_Size));
      
      return m_ppLookupArray[index];
    }
    
    /**
     * Compute lookup table of the points of the given scan for the given angular space
     * @param pScan the scan
     * @param angleCenter
     * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
     * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
     */
    void ComputeOffsets(LocalizedRangeScan* pScan, kt_double angleCenter, kt_double angleOffset, kt_double angleResolution)
    {
      assert(angleOffset != 0.0);
      assert(angleResolution != 0.0);
 
      kt_int32u nAngles = static_cast<kt_int32u>(Math::Round(angleOffset * 2.0 / angleResolution) + 1);
      SetSize(nAngles);

      //////////////////////////////////////////////////////
      // convert points into local coordinates of scan pose

      const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

      // compute transform to scan pose
      Transform transform(pScan->GetScannerPose());

      Pose2Vector localPoints;
      const_forEach(PointVectorDouble, &rPointReadings)
      {
        // do inverse transform to get points in local coordinates
        Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
        localPoints.push_back(vec);
      }

      //////////////////////////////////////////////////////
      // create lookup array for different angles
      kt_double angle = 0.0;
      kt_double startAngle = angleCenter - angleOffset;
      for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * angleResolution;
        ComputeOffsets(angleIndex, angle, localPoints);
      }
      assert(Math::DoubleEqual(angle, angleCenter + angleOffset));
    }

  private:
    /**
     * Compute lookup value of points for given angle
     * @param angleIndex
     * @param angle
     * @param rLocalPoints
     */
    void ComputeOffsets(kt_int32u angleIndex, kt_double angle, const Pose2Vector& rLocalPoints)
    {
      m_ppLookupArray[angleIndex]->SetSize(rLocalPoints.size());
      
      // set up point array by computing relative offsets to points readings
      // when rotated by given angle
      
      const Point2<kt_double>& gridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();
      
      kt_double cosine = cos(angle);
      kt_double sine = sin(angle);
      
      kt_int32u readingIndex = 0;

      kt_int32s* pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

      const_forEach(Pose2Vector, &rLocalPoints)
      {
        const Point2<kt_double>& rPosition = (*iter).GetPosition();
        
        // counterclockwise rotation and that rotation is about the origin (0, 0).
        Point2<kt_double> offset;
        offset.SetX(cosine * rPosition.GetX() -   sine * rPosition.GetY());
        offset.SetY(  sine * rPosition.GetX() + cosine * rPosition.GetY());
        
        // have to compensate for the grid offset when getting the grid index
        Point2<kt_int32s> gridPoint = m_pGrid->GetCoordinateConverter()->WorldToGrid(offset + gridOffset);
        
        // use base GridIndex to ignore ROI 
        kt_int32s lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

        pAngleIndexPointer[readingIndex] = lookupIndex;

        readingIndex++;
      }
    }
        
    /**
     * Sets size of lookup table (resize if not big enough)
     * @param size
     */
    void SetSize(kt_int32u size)
    {
      assert(size != 0);
      
      if (size > m_Capacity)
      {
        if (m_ppLookupArray != NULL)
        {
          DestroyArrays();
        }
        
        m_Capacity = size;
        m_ppLookupArray = new LookupArray*[m_Capacity];
        for (kt_int32u i = 0; i < m_Capacity; i++)
        {
          m_ppLookupArray[i] = new LookupArray();
        }        
      }
      
      m_Size = size;
    }

    /**
     * Delete the arrays
     */
    void DestroyArrays()
    {
      for (kt_int32u i = 0; i < m_Capacity; i++)
      {
        delete m_ppLookupArray[i];
      }
      
      delete[] m_ppLookupArray;
      m_ppLookupArray = NULL;      
    }
    
  private:
    Grid<T>* m_pGrid; 

    kt_int32u m_Capacity;
    kt_int32u m_Size;
    
    LookupArray **m_ppLookupArray;
  }; // class GridIndexLookup

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Implementation of a correlation grid used for scan matching
   */
  class CorrelationGrid : public Grid<kt_int8u>
  {
    friend class ScanMatcher;
    
  public:
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
      delete [] m_pKernel;
      delete m_pGridLookup;
    }

  public:
    /**
     * Create a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param params
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(kt_int32s width, kt_int32s height, kt_double searchSize, kt_double resolution, kt_double smearDeviation)
    {
      assert(resolution != 0.0);

      // +1 in case of roundoff
      kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;
      
      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, searchSize, resolution, smearDeviation);
      pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);

      return pGrid;
    }

    /**
     * Marks cells where scans' points hit as being occupied
     * @param rScans scans whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     */
    void AddScans(const LocalizedRangeScanVector& rScans, Point2<kt_double> viewPoint)
    {
      Clear();

      // add all scans to grid
      const_forEach(LocalizedRangeScanVector, &rScans)
      {
        AddScan(*iter, viewPoint);
      }
    }

    /**
     * Get response at given position for given rotation (only look up valid points)
     * @param angleIndex
     * @param gridPositionIndex
     * @return response
     */
    kt_double GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex)
    {
      kt_double response = 0.0;

      // add up value for each point
      kt_int8u* pByte = GetDataPointer() + gridPositionIndex;

      const LookupArray* pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
      assert(pOffsets != NULL);

      // get number of points in offset list
      kt_int32u nPoints = pOffsets->GetSize();
      if (nPoints == 0)
      {
        return response;
      }

      // calculate response
      kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
      for (kt_int32u i = 0; i < nPoints; i++)
      {
#ifdef KARTO_DEBUG
        // bounds check
        kt_int32s test = gridPositionIndex + (*pOffsets)[i];
        if (!Math::IsUpTo(test, GetDataSize()))
        {
          throw std::runtime_error("Mapper FATAL Error: Unable to calculate response");
        }
#endif
        // uses index offsets to efficiently find location of point in the grid
        response += pByte[pAngleIndexPointer[i]];
      }

      // normalize response
      response /= (nPoints * GridStates_Occupied);
      assert(fabs(response) <= 1.0);

      return response;
    }

    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Point2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
    {
      kt_int32s x = rGrid.GetX() + m_Roi.GetX();
      kt_int32s y = rGrid.GetY() + m_Roi.GetY();
      
      return Grid<kt_int8u>::GridIndex(Point2<kt_int32s>(x, y), boundaryCheck);
    }
    
    /**
     * Get the Region Of Interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2& GetROI() const
    {
      return m_Roi;
    }
    
    /**
     * Sets the Region Of Interest (ROI)
     * @param roi
     */
    inline void SetROI(const Rectangle2& roi)
    {
      m_Roi = roi;
    }
    
  private:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param parameters
     */
    CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize, kt_double searchSize, kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
      , m_SearchSize(searchSize)
      , m_Resolution(resolution)
      , m_SmearDeviation(smearDeviation)
      , m_pKernel(NULL)
      , m_pGridLookup(NULL)
    {
      assert(width % 2 == 1 && height % 2 == 1);

      m_pGridLookup = new GridIndexLookup<kt_int8u>(this);
      
      // setup region of interest
      m_Roi = Rectangle2(borderSize, borderSize, width, height);
      
      // calculate kernel 
      CalculateKernel();
    }
    
    /**
     * Sets up the kernel for grid smearing.
     */
    void CalculateKernel()
    {
      assert(m_Resolution != 0.0);
      assert(m_SmearDeviation != 0.0);

      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * m_Resolution;
      const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * m_Resolution;
      
      // check if given value too small or too big
      if (!Math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        std::stringstream error;
        error << "Mapper Error:  Smear deviation too small:  Must be between " << MIN_SMEAR_DISTANCE_DEVIATION << " and " << MAX_SMEAR_DISTANCE_DEVIATION;
        throw std::runtime_error(error.str());
      }
      
      // NOTE:  Currently assumes a two-dimensional kernel
      
      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, m_Resolution) + 1;
      
      // allocate kernel
      m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
      if (m_pKernel == NULL)
      {
        throw std::runtime_error("Unable to allocate memory for kernel!");
      }
      
      // calculate kernel
      kt_int32s halfKernel = m_KernelSize / 2;
      for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
      {
        for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
        {
#ifdef WIN32
          kt_double distanceFromMean = _hypot(i * m_Resolution, j * m_Resolution);
#else
          kt_double distanceFromMean = hypot(i * m_Resolution, j * m_Resolution);
#endif
          kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));
          
          kt_int32u kernelValue = static_cast<kt_int32u>(Math::Round(z * GridStates_Occupied));
          assert(Math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));
          
          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
        }
      }
    }
    
    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param params
     * @return kernel half-size based on the parameters
     */
    static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
    {
      assert(resolution != 0.0);

      return static_cast<kt_int32s>(Math::Round(2.0 * smearDeviation / resolution));
    }
    
    /**
     * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
     * @param pScan scan whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     * @param doSmear whether the points will be smeared
     */
    void AddScan(LocalizedRangeScan* pScan, const Point2<kt_double>& rViewPoint, kt_bool doSmear = true)
    {
      PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);
      
      // put in all valid points
      const_forEach(PointVectorDouble, &validPoints)
      {
        Point2<kt_int32s> gridPoint = GetCoordinateConverter()->WorldToGrid(*iter);
        if (!Math::IsUpTo(gridPoint.GetX(), m_Roi.GetWidth()) || !Math::IsUpTo(gridPoint.GetY(), m_Roi.GetHeight()))
        {
          // point not in grid
          continue;
        }
        
        int gridIndex = GridIndex(gridPoint);
        
        // set grid cell as occupied
        if (GetDataPointer()[gridIndex] == GridStates_Occupied)
        {
          // value already set
          continue;
        }
        
        GetDataPointer()[gridIndex] = GridStates_Occupied;
        
        // smear grid
        if (doSmear == true)
        {
          assert(m_pKernel != NULL);
          
          kt_int32s halfKernel = m_KernelSize / 2;
          
          // apply kernel
          for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
          {
            kt_int8u* pGridAdr = GetDataPointer(Point2<kt_int32s>(gridPoint.GetX(), gridPoint.GetY() + j));
            
            kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);
            
            // if a point is on the edge of the grid, there is no problem
            // with running over the edge of allowable memory, because
            // the grid has margins to compensate for the kernel size
            for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
            {
              kt_int32s kernelArrayIndex = i + kernelConstant;
              
              kt_int8u kernelValue = m_pKernel[kernelArrayIndex];
              if (kernelValue > pGridAdr[i])
              {
                // kernel value is greater, so set it to kernel value
                pGridAdr[i] = kernelValue;
              }
            }
          }
        }
      }
    }
    
    /**
     * Compute which points in a scan are on the same side as the given viewpoint
     * @param pScan
     * @param rViewPoint
     * @return points on the same side
     */
    PointVectorDouble FindValidPoints(LocalizedRangeScan* pScan, const Point2<kt_double>& rViewPoint)
    {
      const PointVectorDouble& rPointReadings = pScan->GetPointReadings();
      
      // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
      const kt_double minSquareDistance = Math::Square(0.1); // in m^2

      // this iterator lags from the main iterator adding points only when the points are on
      // the same side as the viewpoint
      PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
      PointVectorDouble validPoints;

      Point2<kt_double> firstPoint;
      kt_bool firstTime = true;
      const_forEach(PointVectorDouble, &rPointReadings)
      {
        Point2<kt_double> currentPoint = *iter;

        if (firstTime)
        {
          firstPoint = currentPoint;
          firstTime = false;
        }
        else if ((firstPoint - currentPoint).SquaredLength() > minSquareDistance)
        {
          // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
          // Which computes the direction of rotation, if the rotation is counterclock
          // wise then we are looking at data we should keep. If it's negative rotation
          // we should not included in in the matching
          // have enough distance, check viewpoint
          double a = rViewPoint.GetY() - firstPoint.GetY();
          double b = firstPoint.GetX() - rViewPoint.GetX();
          double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
          double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

          // reset beginning point
          firstPoint = currentPoint;

          if (ss < 0.0)	// wrong side, skip and keep going
          {
            trailingPointIter = iter;
          }
          else
          {
            for (; trailingPointIter != iter; trailingPointIter++)
            {
              validPoints.push_back(*trailingPointIter);
            }
          }
        }
      }

      return validPoints;
    }

  private:
    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    kt_double m_SearchSize;

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    kt_double m_Resolution;

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    kt_double m_SmearDeviation;

    // Size of one side of the kernel
    kt_int32s m_KernelSize; 

    // Cached kernel for smearing
    kt_int8u* m_pKernel;
   
    GridIndexLookup<kt_int8u>* m_pGridLookup;

    // region of interest
    Rectangle2 m_Roi;
  }; // CorrelationGrid

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanMatcher
  {
  public:
    /**
     * Default constructor
     */
    ScanMatcher(Mapper* pMapper)
      : m_pMapper(pMapper)
      , m_pCorrelationGrid(NULL)
      , m_pSearchSpaceProbs(NULL)
    {
    }

    /**
     * Destructor
     */
    virtual ~ScanMatcher()
    {
      delete m_pCorrelationGrid;
      delete m_pSearchSpaceProbs;
    }

  public:
    /**
     * Create a scan matcher with the given parameters
     */
    static ScanMatcher* Create(Mapper* pMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold)
    {
      // invalid parameters
      if (resolution <= 0)
      {
        return NULL;
      }
      if (searchSize <= 0)
      {
        return NULL;
      }
      if (smearDeviation < 0)
      {
        return NULL;
      }
      if (rangeThreshold <= 0)
      {
        return NULL;
      }
      
      assert(Math::DoubleEqual(Math::Round(searchSize / resolution), (searchSize / resolution)));
      
      // calculate search space in grid coordinates 
      kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(Math::Round(searchSize / resolution) + 1);

      // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
      // if a scan is on the border of the search space)
      kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

      kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

      // create correlation grid 
      CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, searchSize, resolution, smearDeviation);

      // create search space probabilities
      Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);

      ScanMatcher* pScanMatcher = new ScanMatcher(pMapper);
      pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
      pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;

      return pScanMatcher;
    }

    /**
     * Match given scan against set of scans
     * @param pScan scan being scan-matched
     * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doPenalize whether to penalize matches further from the search center
     * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
     * @return strength of response
     */
    kt_double MatchScan(LocalizedRangeScan* pScan, const LocalizedRangeScanVector& rBaseScans, Pose2& rMean, Matrix3& rCovariance,
                        kt_bool doPenalize = true, kt_bool doRefineMatch = true)
    {
      ///////////////////////////////////////
      // set scan pose to be center of grid

      // 1. get scan position
      Pose2 scanPose = pScan->GetScannerPose();

      // scan has no readings; cannot do scan matching
      // best guess of pose is based off of adjusted odometer reading
      if (pScan->GetNumberOfReadings() == 0)
      {
        rMean = scanPose;

        // maximum covariance
        rCovariance(0, 0) = MAX_VARIANCE; // XX
        rCovariance(1, 1) = MAX_VARIANCE; // YY
        rCovariance(2, 2) = 4 * Math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue()); // TH*TH

        return 0.0;
      }

      // 2. get size of grid
      Rectangle2 roi = m_pCorrelationGrid->m_Roi;

      // 3. compute offset (in meters - lower left corner)
      Point2<kt_double> offset;
      offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->m_Resolution));
      offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->m_Resolution));

      // 4. set offset
      m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

      ///////////////////////////////////////

      // set up correlation grid
      m_pCorrelationGrid->AddScans(rBaseScans, scanPose.GetPosition());

      // compute how far to search in each direction
      Point2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(), m_pSearchSpaceProbs->GetHeight());
      Point2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->m_Resolution, 0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->m_Resolution);

      // a coarse search only checks half the cells in each dimension
      Point2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->m_Resolution, 2 * m_pCorrelationGrid->m_Resolution);

      // actual scan-matching
      kt_double bestResponse = CorrelateScan(pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, m_pMapper->m_pCoarseSearchAngleOffset->GetValue(), m_pMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);

      if (m_pMapper->m_pUseResponseExpansion->GetValue() == true)
      {
        if (Math::DoubleEqual(bestResponse, 0.0))
        {
#ifdef KARTO_DEBUG
          std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
          // try and increase search angle offset with 20 degrees and do another match
          kt_double newSearchAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
          for (kt_int32u i = 0; i < 3; i++)
          {
            newSearchAngleOffset += Math::DegreesToRadians(20);

            bestResponse = CorrelateScan(pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, newSearchAngleOffset, m_pMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);

            if (Math::DoubleEqual(bestResponse, 0.0) == false)
            {
              break;
            }
          }

#ifdef KARTO_DEBUG
          if (Math::DoubleEqual(bestResponse, 0.0))
          {
            std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
          }
#endif
        }
      }

      if (doRefineMatch)
      {
        Point2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
        Point2<kt_double> fineSearchResolution(m_pCorrelationGrid->m_Resolution, m_pCorrelationGrid->m_Resolution);
        bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution, 0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(), m_pMapper->m_pFineSearchAngleOffset->GetValue(), doPenalize, rMean, rCovariance, true);
      }

#ifdef KARTO_DEBUG
      std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = " << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif

      assert(Math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

      return bestResponse;
    }

    /**
     * Finds the best pose for the scan centering the search in the correlation grid
     * at the given pose and search in the space by the vector and angular offsets
     * in increments of the given resolutions
     * @param rScan scan to match against correlation grid
     * @param rSearchCenter the center of the search space
     * @param searchSpaceOffset searches poses in the area offset by this vector around search center
     * @param searchSpaceResolution how fine a granularity to search in the search space
     * @param searchAngleOffset searches poses in the angles offset by this angle around search center
     * @param searchAngleResolution how fine a granularity to search in the angular search space
     * @param doPenalize whether to penalize matches further from the search center
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doingFineMatch whether to do a finer search after coarse search
     * @return strength of response
     */
    kt_double CorrelateScan(LocalizedRangeScan* pScan, const Pose2& rSearchCenter, const Point2<kt_double>& rSearchSpaceOffset, const Point2<kt_double>& rSearchSpaceResolution,
                            kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch)
    {
      assert(searchAngleResolution != 0.0);

      // setup lookup arrays
      m_pCorrelationGrid->m_pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);

      // only initialize probability grid if computing positional covariance (during coarse match)
      if (!doingFineMatch)
      {
        m_pSearchSpaceProbs->Clear();

        // position search grid - finds lower left corner of search grid
        Point2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
        m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
      }

      // calculate position arrays
      
      std::vector<kt_double> xPoses;
      kt_int32u nX = static_cast<kt_int32u>(Math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
      kt_double startX = -rSearchSpaceOffset.GetX();
      for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
      {
        xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
      }
      assert(Math::DoubleEqual(xPoses.back(), -startX));
      
      std::vector<kt_double> yPoses;
      kt_int32u nY = static_cast<kt_int32u>(Math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
      kt_double startY = -rSearchSpaceOffset.GetY();
      for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
      {
        yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
      }
      assert(Math::DoubleEqual(yPoses.back(), -startY));

      // calculate pose response array size
      kt_int32u nAngles = static_cast<kt_int32u>(Math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

      kt_int32u poseResponseSize = xPoses.size() * yPoses.size() * nAngles;

      // allocate array
      std::pair<kt_double, Pose2>* pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];

      Point2<kt_int32s> startGridPoint = m_pCorrelationGrid->GetCoordinateConverter()->WorldToGrid(Point2<kt_double>(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));
      
      kt_int32u poseResponseCounter = 0;
      forEachAs(std::vector<kt_double>, &yPoses, yIter)
      {
        kt_double y = *yIter;
        kt_double newPositionY = rSearchCenter.GetY() + y;
        kt_double squareY = Math::Square(y);

        forEachAs(std::vector<kt_double>, &xPoses, xIter)
        {
          kt_double x = *xIter;
          kt_double newPositionX = rSearchCenter.GetX() + x;
          kt_double squareX = Math::Square(x);

          Point2<kt_int32s> gridPoint = m_pCorrelationGrid->GetCoordinateConverter()->WorldToGrid(Point2<kt_double>(newPositionX, newPositionY));          
          kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
          assert(gridIndex >= 0);
          
          kt_double angle = 0.0;
          kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
          for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
          {
            angle = startAngle + angleIndex * searchAngleResolution;

            kt_double response = m_pCorrelationGrid->GetResponse(angleIndex, gridIndex);
            if (doPenalize && (Math::DoubleEqual(response, 0.0) == false))
            {
              // simple model (approximate Gaussian) to take odometry into account

              kt_double squaredDistance = squareX + squareY;
              kt_double distancePenalty = 1.0 - (0.2 * squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());
              distancePenalty = Math::Maximum(distancePenalty, m_pMapper->m_pMinimumDistancePenalty->GetValue());

              kt_double squaredAngleDistance = Math::Square(angle - rSearchCenter.GetHeading());              
              kt_double anglePenalty = 1.0 - (0.2 * squaredAngleDistance / m_pMapper->m_pAngleVariancePenalty->GetValue());
              anglePenalty = Math::Maximum(anglePenalty, m_pMapper->m_pMinimumAnglePenalty->GetValue());

              response *= (distancePenalty * anglePenalty);
            }

            // store response and pose
            pPoseResponse[poseResponseCounter] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, Math::NormalizeAngle(angle)));
            poseResponseCounter++;
          }
          
          assert(Math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
        }
      }

      assert(poseResponseSize == poseResponseCounter);

      // find value of best response (in [0; 1])
      kt_double bestResponse = -1;
      for (kt_int32u i = 0; i < poseResponseSize; i++)
      {
        bestResponse = Math::Maximum(bestResponse, pPoseResponse[i].first);

        // will compute positional covariance, save best relative probability for each cell
        if (!doingFineMatch)
        {
          const Pose2& rPose = pPoseResponse[i].second;
          Point2<kt_int32s> grid = m_pSearchSpaceProbs->GetCoordinateConverter()->WorldToGrid(rPose.GetPosition());

          kt_double* ptr = (kt_double*)m_pSearchSpaceProbs->GetDataPointer(grid);
          if (ptr == NULL)
          {
            throw std::runtime_error("Mapper FATAL ERROR - Index out of range in probability search!");
          }

          *ptr = Math::Maximum(pPoseResponse[i].first, *ptr);
        }
      }

      // average all poses with same highest response
      Point2<kt_double> averagePosition;
      kt_double thetaX = 0.0;
      kt_double thetaY = 0.0;
      kt_int32s averagePoseCount = 0;
      for (kt_int32u i = 0; i < poseResponseSize; i++)
      {
        if (Math::DoubleEqual(pPoseResponse[i].first, bestResponse))
        {
          averagePosition += pPoseResponse[i].second.GetPosition();

          kt_double heading = pPoseResponse[i].second.GetHeading();
          thetaX += cos(heading);
          thetaY += sin(heading);

          averagePoseCount++;
        }
      }

      Pose2 averagePose;
      if (averagePoseCount > 0)
      {
        averagePosition /= averagePoseCount;

        thetaX /= averagePoseCount;
        thetaY /= averagePoseCount;

        averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
      }
      else
      {
        throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
      }

      // delete pose response array
      delete [] pPoseResponse;

#ifdef KARTO_DEBUG
      std::cout << "bestPose: " << averagePose << std::endl;
      std::cout << "bestResponse: " << bestResponse << std::endl;
#endif

      if (!doingFineMatch)
      {
        ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset, rSearchSpaceResolution, searchAngleResolution, rCovariance);
      }
      else
      {
        ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter, searchAngleOffset, searchAngleResolution, rCovariance);
      }

      rMean = averagePose;

#ifdef KARTO_DEBUG
      std::cout << "bestPose: " << averagePose << std::endl;
#endif

      if (bestResponse > 1.0)
      {
        bestResponse = 1.0;
      }

      assert(Math::InRange(bestResponse, 0.0, 1.0));
      assert(Math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

      return bestResponse;
    }
    
    /**
     * Computes the positional covariance of the best pose
     * @param rBestPose
     * @param bestResponse
     * @param rSearchCenter
     * @param rSearchSpaceOffset
     * @param rSearchSpaceResolution
     * @param searchAngleResolution
     * @param rCovariance
     */
    void ComputePositionalCovariance(const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                     const Point2<kt_double>& rSearchSpaceOffset, const Point2<kt_double>& rSearchSpaceResolution,
                                     kt_double searchAngleResolution, Matrix3& rCovariance)
    {
      // reset covariance to identity matrix
      rCovariance.SetToIdentity();
      
      // if best response is vary small return max variance
      if (bestResponse < KT_TOLERANCE) 
      {
        rCovariance(0, 0) = MAX_VARIANCE; // XX
        rCovariance(1, 1) = MAX_VARIANCE; // YY
        rCovariance(2, 2) = 4 * Math::Square(searchAngleResolution); // TH*TH
        
        return;
      }
      
      kt_double accumulatedVarianceXX = 0;
      kt_double accumulatedVarianceXY = 0;
      kt_double accumulatedVarianceYY = 0;
      kt_double norm = 0;
      
      kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
      kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();
      
      kt_double offsetX = rSearchSpaceOffset.GetX();
      kt_double offsetY = rSearchSpaceOffset.GetY();
      
      kt_int32u nX = static_cast<kt_int32u>(Math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
      kt_double startX = -offsetX;
      assert(Math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));
      
      kt_int32u nY = static_cast<kt_int32u>(Math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
      kt_double startY = -offsetY;
      assert(Math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));
      
      for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
      {
        kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();
        
        for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
        {
          kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();
          
          Point2<kt_int32s> gridPoint = m_pSearchSpaceProbs->GetCoordinateConverter()->WorldToGrid(Point2<kt_double>(rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
          kt_double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));
          
          // response is not a low response
          if (response >= (bestResponse - 0.1))
          {
            norm += response;
            accumulatedVarianceXX += (Math::Square(x - dx) * response);
            accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
            accumulatedVarianceYY += (Math::Square(y - dy) * response);
          }
        }
      }
      
      if (norm > KT_TOLERANCE)
      {
        kt_double varianceXX = accumulatedVarianceXX / norm;
        kt_double varianceXY = accumulatedVarianceXY / norm;
        kt_double varianceYY = accumulatedVarianceYY / norm;
        kt_double varianceTHTH = 4 * Math::Square(searchAngleResolution);
        
        // lower-bound variances so that they are not too small;
        // ensures that links are not too tight
        kt_double minVarianceXX = 0.1 * Math::Square(rSearchSpaceResolution.GetX());
        kt_double minVarianceYY = 0.1 * Math::Square(rSearchSpaceResolution.GetY());        
        varianceXX = Math::Maximum(varianceXX, minVarianceXX);
        varianceYY = Math::Maximum(varianceYY, minVarianceYY);
        
        // increase variance for poorer responses
        kt_double multiplier = 1.0 / bestResponse;
        rCovariance(0, 0) = varianceXX * multiplier;
        rCovariance(0, 1) = varianceXY * multiplier;
        rCovariance(1, 0) = varianceXY * multiplier;
        rCovariance(1, 1) = varianceYY * multiplier;
        rCovariance(2, 2) = varianceTHTH; // this value will be set in ComputeAngularCovariance
      }
      
      // if values are 0, set to MAX_VARIANCE
      // values might be 0 if points are too sparse and thus don't hit other points
      if (Math::DoubleEqual(rCovariance(0, 0), 0.0))
      {
        rCovariance(0, 0) = MAX_VARIANCE;
      }
      
      if (Math::DoubleEqual(rCovariance(1, 1), 0.0))
      {
        rCovariance(1, 1) = MAX_VARIANCE;
      }
    }

    /**
     * Computes the angular covariance of the best pose
     * @param rBestPose
     * @param bestResponse
     * @param rSearchCenter
     * @param rSearchAngleOffset
     * @param searchAngleResolution
     * @param rCovariance
     */
    void ComputeAngularCovariance(const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                  kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance)
    {
      // NOTE: do not reset covariance matrix

      // normalize angle difference
      kt_double bestAngle = Math::NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());

      Point2<kt_int32s> gridPoint = m_pCorrelationGrid->GetCoordinateConverter()->WorldToGrid(rBestPose.GetPosition());
      kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

      kt_int32u nAngles = static_cast<kt_int32u>(Math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

      kt_double angle = 0.0;
      kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

      kt_double norm = 0.0;
      kt_double accumulatedVarianceThTh = 0.0;
      for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * searchAngleResolution;
        kt_double response = m_pCorrelationGrid->GetResponse(angleIndex, gridIndex);

        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceThTh += (Math::Square(angle - bestAngle) * response);
        }
      }      
      assert(Math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

      if (norm > KT_TOLERANCE)
      {
        if (accumulatedVarianceThTh < KT_TOLERANCE)
        {
          accumulatedVarianceThTh = Math::Square(searchAngleResolution);
        }

        accumulatedVarianceThTh /= norm;
      }
      else
      {
        accumulatedVarianceThTh = 1000 * Math::Square(searchAngleResolution);
      }

      rCovariance(2, 2) = accumulatedVarianceThTh;
    }

  private:
    Mapper* m_pMapper;

    CorrelationGrid* m_pCorrelationGrid;
    Grid<kt_double>* m_pSearchSpaceProbs;
  }; // ScanMatcher
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  template<typename T>
  class BreadthFirstTraversal : public GraphTraversal<T>
  {
  public:
    /**
     * Constructs a breadth-first traverser for the given graph
     */
    BreadthFirstTraversal(Graph<T>* pGraph)
    : GraphTraversal<T>(pGraph)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~BreadthFirstTraversal()
    {
    }
    
  public:
    /**
     * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
     * @param pStartVertex
     * @param pVisitor
     * @return visited vertices
     */
    virtual std::vector<T*> Traverse(Vertex<T>* pStartVertex, Visitor<T>* pVisitor)
    {
      std::queue<Vertex<T>*> toVisit;
      std::set<Vertex<T>*> seenVertices;
      std::vector<Vertex<T>*> validVertices;
      
      toVisit.push(pStartVertex);
      seenVertices.insert(pStartVertex);
      
      do
      {
        Vertex<T>* pNext = toVisit.front();
        toVisit.pop();
        
        if (pVisitor->Visit(pNext))
        {
          // vertex is valid, explore neighbors
          validVertices.push_back(pNext);
          
          std::vector<Vertex<T>*> adjacentVertices = pNext->GetAdjacentVertices();
          forEach(typename std::vector<Vertex<T>*>, &adjacentVertices)
          {
            Vertex<T>* pAdjacent = *iter;
            
            // adjacent vertex has not yet been seen, add to queue for processing
            if (seenVertices.find(pAdjacent) == seenVertices.end())
            {
              toVisit.push(pAdjacent);
              seenVertices.insert(pAdjacent);
            }
          }
        }
      } while (toVisit.empty() == false);
      
      std::vector<T*> objects;
      forEach(typename std::vector<Vertex<T>*>, &validVertices)
      {
        objects.push_back((*iter)->GetObject());
      }
      
      return objects;
    }
    
  }; // class BreadthFirstTraversal
    
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class NearScanVisitor : public Visitor<LocalizedRangeScan>
  {
  public:
    NearScanVisitor(LocalizedRangeScan* pScan, kt_double maxDistance, kt_bool useScanBarycenter)
      : m_MaxDistanceSquared(Math::Square(maxDistance))
      , m_UseScanBarycenter(useScanBarycenter)
    {
      m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
    }

    virtual kt_bool Visit(Vertex<LocalizedRangeScan>* pVertex)
    {      
      LocalizedRangeScan* pScan = pVertex->GetObject();
      
      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);
      
      kt_double squaredDistance = pose.SquaredDistance(m_CenterPose);
      return (squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE);
    }

  protected:
    Pose2 m_CenterPose;
    kt_double m_MaxDistanceSquared;
    kt_bool m_UseScanBarycenter;
    
  }; // class NearScanVisitor
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class MapperGraph : public Graph<LocalizedRangeScan>
  {
  public:
    /**
     * Graph for graph SLAM
     * @param pMapper
     * @param rParameters
     * @param pDeviceManager
     */
    MapperGraph(Mapper* pMapper, kt_double rangeThreshold)
      : m_pMapper(pMapper)
    {
      m_pLoopScanMatcher = ScanMatcher::Create(pMapper, m_pMapper->m_pLoopSearchSpaceDimension->GetValue(), m_pMapper->m_pLoopSearchSpaceResolution->GetValue(), m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
      assert(m_pLoopScanMatcher);

      m_pTraversal = new BreadthFirstTraversal<LocalizedRangeScan>(this);
    }

    /**
     * Destructor
     */
    virtual ~MapperGraph()
    {
      delete m_pLoopScanMatcher;
      m_pLoopScanMatcher = NULL;
      
      delete m_pTraversal;
      m_pTraversal = NULL;
    }

  public:
    /**
     * Adds a vertex representing the given scan to the graph
     * @param pScan
     */
    void AddVertex(LocalizedRangeScan* pScan)
    {
      Vertex<LocalizedRangeScan>* pVertex = new Vertex<LocalizedRangeScan>(pScan);
      Graph<LocalizedRangeScan>::Add(pScan->GetLaserRangeFinder()->GetId(), pVertex);
    }

    /**
     * Link scan to last scan and nearby chains of scans
     * @param pScan
     * @param rCovariance uncertainty of match
     */
    void AddEdges(LocalizedRangeScan* pScan, const Matrix3& rCovariance)
    {
      DeviceManager* pDeviceManager = m_pMapper->m_pDeviceManager;
      
      kt_int32s parentId = pScan->GetLaserRangeFinder()->GetId();

      // link to previous scan
      kt_int32s previousScanNum = pScan->GetId() - 1;
      if (pDeviceManager->GetLastScan(pScan->GetLaserRangeFinder()) != NULL)
      {
        assert(previousScanNum >= 0);
        LinkScans(pDeviceManager->GetScan(parentId, previousScanNum), pScan, pScan->GetScannerPose(), rCovariance);
      }

      Pose2Vector means;
      std::vector<Matrix3> covariances;

      // first scan (link to first scan of other robots)
      if (pDeviceManager->GetLastScan(pScan->GetLaserRangeFinder()) == NULL)
      {
        assert(pDeviceManager->GetScans(parentId).size() == 1);

        std::vector<kt_int32s> deviceIds = pDeviceManager->GetDeviceIds();
        forEach(std::vector<kt_int32s>, &deviceIds)
        {
          kt_int32s candidateDeviceId = *iter;
          
          // skip if candidate device is the same or other device has no scans
          if ((candidateDeviceId == parentId) || (pDeviceManager->GetScans(candidateDeviceId).empty()))
          {
            continue;
          }
          
          Pose2 bestPose;
          Matrix3 covariance;          
          kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, pDeviceManager->GetScans(candidateDeviceId), bestPose, covariance);
          LinkScans(pScan, pDeviceManager->GetScan(candidateDeviceId, 0), bestPose, covariance);

          // only add to means and covariances if response was high "enough"
          if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue())
          {
            means.push_back(bestPose);
            covariances.push_back(covariance);
          }
        }
      }
      else
      {
        // link to running scans
        Pose2 scanPose = pScan->GetScannerPose();
        means.push_back(scanPose);
        covariances.push_back(rCovariance);
        LinkChainToScan(pDeviceManager->GetRunningScans(parentId), pScan, scanPose, rCovariance);
      }

      // link to other near chains (chains that include new scan are invalid)
      LinkNearChains(pScan, means, covariances);

      if (!means.empty())
      {
        pScan->SetScannerPose(ComputeWeightedMean(means, covariances));
      }
    }

    /**
     * Tries to close a loop using the given scan
     * @param pScan
     */
    void TryCloseLoop(LocalizedRangeScan* pScan, kt_int32s deviceId)
    {
      kt_int32u scanIndex = 0;

      LocalizedRangeScanVector candidateChain = FindPossibleLoopClosure(pScan, deviceId, scanIndex);
      while (!candidateChain.empty())
      {        
        Pose2 bestPose;
        Matrix3 covariance;
        kt_double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false, false);

        std::stringstream stream;
        stream << "COARSE RESPONSE: " << coarseResponse << " (>" << m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue() << ")" << std::endl;
        stream << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (<" << m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() << ")";
        m_pMapper->FireLoopClosureCheck(stream.str());
        
        if ((coarseResponse > m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue()) &&
            (covariance(0, 0) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()) &&
            (covariance(1, 1) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()))
        {
          // save for reversion
          Pose2 oldPose = pScan->GetScannerPose();

          pScan->SetScannerPose(bestPose);
          kt_double fineResponse = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false);

          std::stringstream stream;
          stream << "FINE RESPONSE: " << fineResponse << " (>" << m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue() << ")" << std::endl;
          m_pMapper->FireLoopClosureCheck(stream.str());
          
          if (fineResponse < m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue())
          {
            // failed verification test, revert
            pScan->SetScannerPose(oldPose);

            m_pMapper->FireLoopClosureCheck("REJECTED!");
          }
          else
          {
            m_pMapper->FireBeginLoopClosure("Closing loop...");
            
            pScan->SetScannerPose(bestPose);
            LinkChainToScan(candidateChain, pScan, bestPose, covariance);
            CorrectPoses();
            
            m_pMapper->FireEndLoopClosure("Loop closed!");
          }
        }

        candidateChain = FindPossibleLoopClosure(pScan, deviceId, scanIndex);
      }
    }

  private:
    /**
     * Gets the vertex associated with the given scan
     * @param pScan
     * @return vertex of scan
     */
    Vertex<LocalizedRangeScan>* GetVertex(LocalizedRangeScan* pScan)
    {
      return m_Vertices[pScan->GetLaserRangeFinder()->GetId()][pScan->GetId()];
    }

    /**
     * Finds the closest scan in the vector to the given pose
     * @param rScans
     * @param rPose
     */
    LocalizedRangeScan* GetClosestScanToPose(const LocalizedRangeScanVector& rScans, const Pose2& rPose)
    {
      LocalizedRangeScan* pClosestScan = NULL;
      kt_double bestSquaredDistance = DBL_MAX;

      const_forEach(LocalizedRangeScanVector, &rScans)
      {
        Pose2 scanPose = (*iter)->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

        kt_double squaredDistance = rPose.SquaredDistance(scanPose);
        if (squaredDistance < bestSquaredDistance)
        {
          bestSquaredDistance = squaredDistance;
          pClosestScan = *iter;
        }
      }

      return pClosestScan;
    }

    /**
     * Creates an edge between the source scan vertex and the target scan vertex if it
     * does not already exist; otherwise return the existing edge
     * @param pSourceScan
     * @param pTargetScan
     * @param rIsNewEdge set to true if the edge is new
     * @return edge between source and target scan vertices
     */
    Edge<LocalizedRangeScan>* AddEdge(LocalizedRangeScan* pSourceScan, LocalizedRangeScan* pTargetScan, kt_bool& rIsNewEdge)
    {
      // check that vertex exists
      assert(pSourceScan->GetId() < (kt_int32s)m_Vertices[pSourceScan->GetLaserRangeFinder()->GetId()].size());
      assert(pTargetScan->GetId() < (kt_int32s)m_Vertices[pTargetScan->GetLaserRangeFinder()->GetId()].size());

      Vertex<LocalizedRangeScan>* v1 = m_Vertices[pSourceScan->GetLaserRangeFinder()->GetId()][pSourceScan->GetId()];
      Vertex<LocalizedRangeScan>* v2 = m_Vertices[pTargetScan->GetLaserRangeFinder()->GetId()][pTargetScan->GetId()];

      // see if edge already exists
      const_forEach(std::vector<Edge<LocalizedRangeScan>*>, &(v1->GetEdges()))
      {
        Edge<LocalizedRangeScan>* pEdge = *iter;

        if (pEdge->GetTarget() == v2)
        {
          rIsNewEdge = false;
          return pEdge;
        }
      }

      Edge<LocalizedRangeScan>* pEdge = new Edge<LocalizedRangeScan>(v1, v2);
      Graph<LocalizedRangeScan>::Add(pEdge);
      rIsNewEdge = true;
      return pEdge;
    }

    /**
     * Adds an edge between the two scans and labels the edge with the given mean and covariance
     * @param pFromScan
     * @param pToScan
     * @param rMean
     * @param rCovariance
     */
    void LinkScans(LocalizedRangeScan* pFromScan, LocalizedRangeScan* pToScan, const Pose2& rMean, const Matrix3& rCovariance)
    {
      kt_bool isNewEdge = true;
      Edge<LocalizedRangeScan>* pEdge = AddEdge(pFromScan, pToScan, isNewEdge);

      // only attach link information if the edge is new
      if (isNewEdge == true)
      {
        pEdge->SetLabel(new LinkInfo(pFromScan->GetScannerPose(), rMean, rCovariance));
      }
    }

    /**
     * Find nearby chains of scans and link them to scan if response is high enough
     * @param pScan
     * @param rMeans
     * @param rCovariances
     */
    void LinkNearChains(LocalizedRangeScan* pScan, Pose2Vector& rMeans, std::vector<Matrix3>& rCovariances)
    {
      const std::vector<LocalizedRangeScanVector> nearChains = FindNearChains(pScan);
      const_forEach(std::vector<LocalizedRangeScanVector>, &nearChains)
      {
        if (iter->size() < m_pMapper->m_pLoopMatchMinimumChainSize->GetValue())
        {
          continue;
        }

        Pose2 mean;
        Matrix3 covariance;
        // match scan against "near" chain
        kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, *iter, mean, covariance, false);
        if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE)
        {
          rMeans.push_back(mean);
          rCovariances.push_back(covariance);
          LinkChainToScan(*iter, pScan, mean, covariance);
        }
      }
    }

    /**
     * Link the chain of scans to the given scan by finding the closest scan in the chain to the given scan
     * @param rChain
     * @param pScan
     * @param rMean
     * @param rCovariance
     */
    void LinkChainToScan(const LocalizedRangeScanVector& rChain, LocalizedRangeScan* pScan, const Pose2& rMean, const Matrix3& rCovariance)
    {    
      Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      LocalizedRangeScan* pClosestScan = GetClosestScanToPose(rChain, pose);
      assert(pClosestScan != NULL);

      Pose2 closestScanPose = pClosestScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      kt_double squaredDistance = pose.SquaredDistance(closestScanPose);
      if (squaredDistance < Math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        LinkScans(pClosestScan, pScan, rMean, rCovariance);        			
      }
    }

    /**
     * Find chains of scans that are close to given scan
     * @param pScan
     * @return chains of scans
     */
    std::vector<LocalizedRangeScanVector> FindNearChains(LocalizedRangeScan* pScan)
    {
      std::vector<LocalizedRangeScanVector> nearChains;

      Pose2 scanPose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      // to keep track of which scans have been added to a chain
      LocalizedRangeScanVector processed;

      const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan, m_pMapper->m_pLinkScanMaximumDistance->GetValue());
      const_forEach(LocalizedRangeScanVector, &nearLinkedScans)
      {
        LocalizedRangeScan* pNearScan = *iter;

        if (pNearScan == pScan)
        {
          continue;
        }

        // scan has already been processed, skip
        if (find(processed.begin(), processed.end(), pNearScan) != processed.end())
        {
          continue;
        }

        processed.push_back(pNearScan);

        // build up chain
        kt_bool isValidChain = true;
        std::list<LocalizedRangeScan*> chain;

        // add scans before current scan being processed
        for (kt_int32s candidateScanNum = pNearScan->GetId() - 1; candidateScanNum >= 0; candidateScanNum--)
        {
          LocalizedRangeScan* pCandidateScan = m_pMapper->m_pDeviceManager->GetScan(pNearScan->GetLaserRangeFinder()->GetId(), candidateScanNum);

          // chain is invalid--contains scan being added
          if (pCandidateScan == pScan)
          {
            isValidChain = false;
          }

          Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());
          kt_double squaredDistance = scanPose.SquaredDistance(candidatePose);

          if (squaredDistance < Math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
          {
            chain.push_front(pCandidateScan);
            processed.push_back(pCandidateScan);
          }
          else
          {
            break;
          }
        }

        chain.push_back(pNearScan);

        // add scans after current scan being processed
        kt_int32u end = m_pMapper->m_pDeviceManager->GetScans(pNearScan->GetLaserRangeFinder()->GetId()).size();
        for (kt_int32u candidateScanNum = pNearScan->GetId() + 1; candidateScanNum < end; candidateScanNum++)
        {
          LocalizedRangeScan* pCandidateScan = m_pMapper->m_pDeviceManager->GetScan(pNearScan->GetLaserRangeFinder()->GetId(), candidateScanNum);

          if (pCandidateScan == pScan)
          {
            isValidChain = false;
          }

          Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());;
          kt_double squaredDistance = scanPose.SquaredDistance(candidatePose);

          if (squaredDistance < Math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
          {
            chain.push_back(pCandidateScan);
            processed.push_back(pCandidateScan);
          }
          else
          {
            break;
          }
        }

        if (isValidChain)
        {
          // change list to vector
          LocalizedRangeScanVector tempChain;
          std::copy(chain.begin(), chain.end(), inserter(tempChain, tempChain.begin()));
          // add chain to collection
          nearChains.push_back(tempChain);
        }
      }

      return nearChains;
    }

    /**
     * Find "nearby" (no further than given distance away) scans through graph links
     * @param pScan
     * @param maxDistance
     */
    LocalizedRangeScanVector FindNearLinkedScans(LocalizedRangeScan* pScan, kt_double maxDistance)
    {
      NearScanVisitor* pVisitor = new NearScanVisitor(pScan, maxDistance, m_pMapper->m_pUseScanBarycenter->GetValue());
      LocalizedRangeScanVector nearLinkedScans = m_pTraversal->Traverse(GetVertex(pScan), pVisitor);
      delete pVisitor;

      return nearLinkedScans;
    }

    /**
     * Compute mean of poses weighted by covariances
     * @param rMeans
     * @param rCovariances
     * @return weighted mean
     */
    Pose2 ComputeWeightedMean(const Pose2Vector& rMeans, const std::vector<Matrix3>& rCovariances)
    {
      assert(rMeans.size() == rCovariances.size());

      // compute sum of inverses and create inverse list
      std::vector<Matrix3> inverses;
      inverses.reserve(rCovariances.size());

      Matrix3 sumOfInverses;
      const_forEach(std::vector<Matrix3>, &rCovariances)
      {
        Matrix3 inverse = iter->Inverse();
        inverses.push_back(inverse);

        sumOfInverses += inverse;
      }
      Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();

      // compute weighted mean
      Pose2 accumulatedPose;
      kt_double thetaX = 0.0;
      kt_double thetaY = 0.0;

      Pose2Vector::const_iterator meansIter = rMeans.begin();
      const_forEach(std::vector<Matrix3>, &inverses)
      {
        Pose2 pose = *meansIter;
        kt_double angle = pose.GetHeading();
        thetaX += cos(angle);
        thetaY += sin(angle);

        Matrix3 weight = inverseOfSumOfInverses * (*iter);
        accumulatedPose += weight * pose;

        meansIter++;
      }

      thetaX /= rMeans.size();
      thetaY /= rMeans.size();
      accumulatedPose.SetHeading(atan2(thetaY, thetaX));

      return accumulatedPose;
    }

    /**
     * Tries to find a chain of scan from the given device id starting at the 
     * given scan index that could possibly close a loop with the given scan
     * @param pScan
     * @param rStartNum
     * @return chain that can possibly close a loop with given scan
     */
    LocalizedRangeScanVector FindPossibleLoopClosure(LocalizedRangeScan* pScan, kt_int32s deviceId, kt_int32u& rStartNum)
    {
      LocalizedRangeScanVector chain; // return value

      Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      // possible loop closure chain should not include close scans that have a
      // path of links to the scan of interest
      const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan, m_pMapper->m_pLoopSearchMaximumDistance->GetValue());

      kt_int32u nScans = m_pMapper->m_pDeviceManager->GetScans(deviceId).size();
      for (; rStartNum < nScans; rStartNum++)
      {
        LocalizedRangeScan* pCandidateScan = m_pMapper->m_pDeviceManager->GetScan(deviceId, rStartNum);

        Pose2 candidateScanPose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

        kt_double squaredDistance = candidateScanPose.SquaredDistance(pose);
        if (squaredDistance < Math::Square(m_pMapper->m_pLoopSearchMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          // a linked scan cannot be in the chain
          if (find(nearLinkedScans.begin(), nearLinkedScans.end(), pCandidateScan) != nearLinkedScans.end())
          {
            chain.clear();
          }
          else
          {
            chain.push_back(pCandidateScan);
          }
        }
        else
        {
          // return chain if it is long "enough"
          if (chain.size() >= m_pMapper->m_pLoopMatchMinimumChainSize->GetValue())
          {
            return chain;
          }
          else
          {
            chain.clear();
          }
        }
      }

      return chain;
    }

    /**
     * Optimizes scan poses
     */
    void CorrectPoses()
    {
      Pose2Vector poses;

      kt_int32u poseIndex = 0;
      std::map<LocalizedRangeScan*, kt_int32u> scanToNum;

      // add ALL poses to vector; give each pose a unique index
      LocalizedRangeScanVector allScans = m_pMapper->m_pDeviceManager->GetAllScans();
      forEach(LocalizedRangeScanVector, &allScans)
      {
        scanToNum[*iter] = poseIndex++;

        poses.push_back((*iter)->GetScannerPose());
      }

      const std::vector<Edge<LocalizedRangeScan>*>& edges = GetEdges();

      Pose2Vector poseDifferences;
      std::vector<Matrix3> covariances;
      Matrix scanLinkIDs(2, edges.size()); // mapping of source scan to target scan indices      
      for (kt_int32u edgeNum = 0; edgeNum < edges.size(); edgeNum++)
      {
        Edge<LocalizedRangeScan>* edge = edges[edgeNum];

        LinkInfo* linkInfo = (LinkInfo*)edge->GetLabel();
        poseDifferences.push_back(linkInfo->GetPoseDifference());
        covariances.push_back(linkInfo->GetCovariance());

        scanLinkIDs(0, edgeNum) = scanToNum[edge->GetSource()->GetObject()];
        scanLinkIDs(1, edgeNum) = scanToNum[edge->GetTarget()->GetObject()];
      }

      // optimize scans!
      ScanSolver* pSolver = m_pMapper->m_pScanOptimizer;
      if (pSolver != NULL)
      {
        pSolver->Initialize(poses, poseDifferences, covariances, scanLinkIDs);

        pSolver->Compute();

        // update all scan poses
        Pose2Vector::const_iterator correctedPoseIter = pSolver->GetCorrectedPoses().begin();
        forEach(LocalizedRangeScanVector, &allScans)
        {
          (*iter)->SetScannerPose(*correctedPoseIter);
          correctedPoseIter++;
        }

        pSolver->Clear();
      }
    }
    
  private:
    Mapper* m_pMapper;  
    
    ScanMatcher* m_pLoopScanMatcher;
  }; // MapperGraph

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Default constructor
   */
  Mapper::Mapper()
    : m_Initialized(false)
    , m_pSequentialScanMatcher(NULL)
    , m_pDeviceManager(NULL)
    , m_pGraph(NULL)
    , m_pScanOptimizer(NULL)
  {
    // initialize parameters
    m_pParameters = new ParameterManager();

    /** 
     * When set to true, the mapper will use a scan matching algorithm. In most real-world situations
     * this should be set to true so that the mapper algorithm can correct for noise and errors in
     * odometry and scan data. In some simulator environments where the simulated scan and odometry
     * data are very accurate, the scan matching algorithm can produce worse results. In those cases
     * set this to false to improve results.
     * Default value is true.
     */
    m_pUseScanMatching = new Parameter<kt_bool>("UseScanMatching", true, m_pParameters);

    /**
     * Default value is true.
     */
    m_pUseScanBarycenter = new Parameter<kt_bool>("UseScanBarycenter", true, m_pParameters);

    /**
     * Sets the minimum travel between scans.  If a new scan's position is more than minimumTravelDistance
     * from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
     * new scan if it also does not meet the minimum change in heading requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 0.2 (meters).
     */
    m_pMinimumTravelDistance = new Parameter<kt_double>("MinimumTravelDistance", 0.2, m_pParameters);

    /**
     * Sets the minimum heading change between scans. If a new scan's heading is more than minimumTravelHeading
     * from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
     * new scan if it also does not meet the minimum travel distance requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 10 degrees.
     */
    m_pMinimumTravelHeading = new Parameter<kt_double>("MinimumTravelHeading", Math::DegreesToRadians(10), m_pParameters);

    /**
     * Scan buffer size is the length of the scan chain stored for scan matching.
     * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
     * The idea is to get an area approximately 20 meters long for scan matching.
     * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
     * should be 20 / 0.3 = 67.)
     * Default value is 67.
     */
    m_pScanBufferSize = new Parameter<kt_int32u>("ScanBufferSize", 70, m_pParameters);

    /**
     * Scan buffer maximum scan distance is the maximum distance between the first and last scans
     * in the scan chain stored for matching.
     * Default value is 20.0.
     */
    m_pScanBufferMaximumScanDistance = new Parameter<kt_double>("ScanBufferMaximumScanDistance", 20.0, m_pParameters);

    /**
     * Scans are linked only if the correlation response value is greater than this value.
     * Default value is 0.4
     */
    m_pLinkMatchMinimumResponseFine = new Parameter<kt_double>("LinkMatchMinimumResponseFine", 0.8, m_pParameters);

    /**
     * Maximum distance between linked scans.  Scans that are farther apart will not be linked
     * regardless of the correlation response value.
     * Default value is 6.0 meters.
     */
    m_pLinkScanMaximumDistance = new Parameter<kt_double>("LinkScanMaximumDistance", 10.0, m_pParameters);

    /**
     * Scans less than this distance from the current position will be considered for a match
     * in loop closure.
     * Default value is 4.0 meters.
     */
    m_pLoopSearchMaximumDistance = new Parameter<kt_double>("LoopSearchMaximumDistance", 4.0, m_pParameters);

    /**
     * When the loop closure detection finds a candidate it must be part of a large
     * set of linked scans. If the chain of scans is less than this value we do not attempt
     * to close the loop.
     * Default value is 10.
     */
    m_pLoopMatchMinimumChainSize = new Parameter<kt_int32u>("LoopMatchMinimumChainSize", 10, m_pParameters);

    /**
     * The co-variance values for a possible loop closure have to be less than this value
     * to consider a viable solution. This applies to the coarse search.
     * Default value is 0.16.
     */
    m_pLoopMatchMaximumVarianceCoarse = new Parameter<kt_double>("LoopMatchMaximumVarianceCoarse", Math::Square(0.4), m_pParameters);

    /**
     * If response is larger then this, then initiate loop closure search at the coarse resolution.
     * Default value is 0.7.
     */
    m_pLoopMatchMinimumResponseCoarse = new Parameter<kt_double>("LoopMatchMinimumResponseCoarse", 0.8, m_pParameters);

    /**
     * If response is larger then this, then initiate loop closure search at the fine resolution.
     * Default value is 0.7.
     */
    m_pLoopMatchMinimumResponseFine = new Parameter<kt_double>("LoopMatchMinimumResponseFine", 0.8, m_pParameters);

    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters correlationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>("CorrelationSearchSpaceDimension", 0.3, m_pParameters);

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>("CorrelationSearchSpaceResolution", 0.01, m_pParameters);

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>("CorrelationSearchSpaceSmearDeviation", 0.03, m_pParameters);


    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters loopCorrelationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    m_pLoopSearchSpaceDimension = new Parameter<kt_double>("LoopSearchSpaceDimension", 8.0, m_pParameters);

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    m_pLoopSearchSpaceResolution = new Parameter<kt_double>("LoopSearchSpaceResolution", 0.05, m_pParameters);

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    m_pLoopSearchSpaceSmearDeviation = new Parameter<kt_double>("LoopSearchSpaceSmearDeviation", 0.03, m_pParameters);

    //////////////////////////////////////////////////////////////////////////////
    // ScanMatcherParameters;

    // Variance of penalty for deviating from odometry when scan-matching.
    // The penalty is a multiplier (less than 1.0) is a function of the
    // delta of the scan position being tested and the odometric pose
    m_pDistanceVariancePenalty = new Parameter<kt_double>("SistanceVariancePenalty", Math::Square(0.3), m_pParameters);
    m_pAngleVariancePenalty = new Parameter<kt_double>("AngleVariancePenalty", Math::Square(Math::DegreesToRadians(20)), m_pParameters);

    // The range of angles to search during a coarse search and a finer search
    m_pFineSearchAngleOffset = new Parameter<kt_double>("FineSearchAngleOffset", Math::DegreesToRadians(0.2), m_pParameters);
    m_pCoarseSearchAngleOffset = new Parameter<kt_double>("CoarseSearchAngleOffset", Math::DegreesToRadians(20), m_pParameters);

    // Resolution of angles to search during a coarse search
    m_pCoarseAngleResolution = new Parameter<kt_double>("CoarseAngleResolution", Math::DegreesToRadians(2), m_pParameters);

    // Minimum value of the penalty multiplier so scores do not
    // become too small
    m_pMinimumAnglePenalty = new Parameter<kt_double>("MinimumAnglePenalty", 0.9, m_pParameters);
    m_pMinimumDistancePenalty = new Parameter<kt_double>("MinimumDistancePenalty", 0.5, m_pParameters);

    // whether to increase the search space if no good matches are initially found
    m_pUseResponseExpansion = new Parameter<kt_bool>("UseResponseExpansion", false, m_pParameters);
  }

  /**
   * Destructor
   */
  Mapper::~Mapper()
  {
    Reset();

    delete m_pDeviceManager;

    delete m_pParameters;
  }

  void Mapper::Initialize(kt_double rangeThreshold)
  {
    if (m_Initialized == false)
    {
      // create sequential scan and loop matcher
      m_pSequentialScanMatcher = ScanMatcher::Create(this, m_pCorrelationSearchSpaceDimension->GetValue(), m_pCorrelationSearchSpaceResolution->GetValue(), m_pCorrelationSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
      assert(m_pSequentialScanMatcher);

      m_pDeviceManager = new DeviceManager(m_pScanBufferSize->GetValue(), m_pScanBufferMaximumScanDistance->GetValue());

      m_pGraph = new MapperGraph(this, rangeThreshold);

      m_Initialized = true;
    }
  }

  void Mapper::Reset()
  {
    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = NULL;

    delete m_pGraph;
    m_pGraph = NULL;

    delete m_pDeviceManager;
    m_pDeviceManager = NULL;

    m_Initialized = false;
  }

  kt_bool Mapper::Process(LocalizedRangeScan* pScan)
  {
    // validate scan
    if (pScan == NULL || pScan->Validate() == false)
    {
      return false; 
    }

    if (m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      Initialize(pScan->GetLaserRangeFinder()->GetRangeThreshold());
    }

    // get last scan
    LocalizedRangeScan* pLastScan = m_pDeviceManager->GetLastScan(pScan->GetLaserRangeFinder());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL)
    {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!HasMovedEnough(pScan, pLastScan))
    {
      return false;
    }
    
    Matrix3 covariance;
    covariance.SetToIdentity();
    
    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL)
    {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan, m_pDeviceManager->GetRunningScans(pScan->GetLaserRangeFinder()->GetId()), bestPose, covariance);
      pScan->SetScannerPose(bestPose);
    }
    
    // add scan to buffer and assign id
    m_pDeviceManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue())
    {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, covariance);
      
      m_pDeviceManager->AddRunningScan(pScan);

      std::vector<kt_int32s> deviceIds = m_pDeviceManager->GetDeviceIds();
      forEach(std::vector<kt_int32s>, &deviceIds)
      {
        m_pGraph->TryCloseLoop(pScan, *iter);
      }
    }
    
    m_pDeviceManager->SetLastScan(pScan);
        
    return true;
  }

  /**
   * Is the scan sufficiently far from the last scan?
   * @param pScan
   * @param pLastScan
   * @return true if the scans are sufficiently far
   */
  kt_bool Mapper::HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const
  {
    // test if first scan
    if (pLastScan == NULL)
    {
      return true;
    }
    
    Pose2 lastScannerPose = pLastScan->GetScannerAt(pLastScan->GetOdometricPose());
    Pose2 scannerPose = pScan->GetScannerAt(pScan->GetOdometricPose());

    // test if we have turned enough
    kt_double deltaHeading = Math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
    if (fabs(deltaHeading) >= m_pMinimumTravelHeading->GetValue())
    {
      return true;
    }

    // test if we have moved enough
    kt_double squaredTravelDistance = lastScannerPose.SquaredDistance(scannerPose);
    if (squaredTravelDistance >= Math::Square(m_pMinimumTravelDistance->GetValue()) - KT_TOLERANCE)
    {
      return true;
    }

    return false;
  }

  /**
   * Gets all the processed scans
   * @return all scans
   */
  LocalizedRangeScanVector Mapper::GetAllProcessedScans() const
  {
    LocalizedRangeScanVector allScans;

    if (m_pDeviceManager != NULL)
    {
      allScans = m_pDeviceManager->GetAllScans();
    }

    return allScans;
  }

  /**
   * Adds a listener
   * @param pListener
   */
  void Mapper::AddListener(MapperListener* pListener)
  {
    m_Listeners.push_back(pListener);
  }

  /**
   * Removes a listener
   * @param pListener
   */
  void Mapper::RemoveListener(MapperListener* pListener)
  {
    std::vector<MapperListener*>::iterator iter = std::find(m_Listeners.begin(), m_Listeners.end(), pListener);
    if (iter != m_Listeners.end())
    {
      m_Listeners.erase(iter);
    }
  }

  void Mapper::FireInfo(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      (*iter)->Info(rInfo);
    }
  }

  void Mapper::FireDebug(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperDebugListener* pListener = dynamic_cast<MapperDebugListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->Debug(rInfo);
      }
    }
  }

  void Mapper::FireLoopClosureCheck(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->LoopClosureCheck(rInfo);
      }
    }
  }

  void Mapper::FireBeginLoopClosure(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->BeginLoopClosure(rInfo);
      }
    }
  }

  void Mapper::FireEndLoopClosure(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->EndLoopClosure(rInfo);
      }
    }
  }

  void Mapper::SetScanOptimizer(ScanSolver* pScanOptimizer)
  {
    m_pScanOptimizer = pScanOptimizer;
  }

  Graph<LocalizedRangeScan>* Mapper::GetGraph() const
  {
    return m_pGraph;
  }
  
  ParameterManager* Mapper::GetParameterManager() const
  {
    return m_pParameters;
  }

}

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

#ifndef __KARTO_MAPPER__
#define __KARTO_MAPPER__

#include <map>
#include <vector>

#include "kt_Types.h"

namespace karto
{
  ////////////////////////////////////////////////////////////////////////////////////////
  // Listener classes
  
  class MapperListener
  {
  public:
    virtual void Info(const std::string& rInfo) const = 0;
  };

  class MapperDebugListener
  {
  public:
    virtual void Debug(const std::string& rInfo) const = 0;
  };

  class MapperLoopClosureListener : public MapperListener
  {
  public:
    virtual void LoopClosureCheck(const std::string& rInfo) const = 0;
    virtual void BeginLoopClosure(const std::string& rInfo) const = 0;
    virtual void EndLoopClosure(const std::string& rInfo) const = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class EdgeLabel
  {
  public:
    /**
     * Default constructor
     */
    EdgeLabel()
    {
    }

    /**
     * Destructor
     */
    virtual ~EdgeLabel()
    {
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // A LinkInfo object contains the requisite information for the "spring"
  // that links two scans together--the pose difference and the uncertainty
  // (represented by a covariance matrix).
  class LinkInfo : public EdgeLabel
  {
  public:
    /**
     * Constructs a link between the given poses
     * @param rPose1
     * @param rPose2
     * @param rCovariance
     */
    LinkInfo(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance)
    {
      Update(rPose1, rPose2, rCovariance);
    }
    
    /**
     * Destructor
     */
    virtual ~LinkInfo()
    {
    }
    
  public:
    /**
     * Changes the link information to be the given parameters
     * @param rPose1
     * @param rPose2
     * @param rCovariance
     */
    void Update(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance)
    {
      m_Pose1 = rPose1;
      m_Pose2 = rPose2;
      
      // transform second pose into the coordinate system of the first pose
      Transform transform(rPose1, Pose2());
      m_PoseDifference = transform.TransformPose(rPose2);
      
      // transform covariance into reference of first pose
      Matrix3 rotationMatrix;
      rotationMatrix.FromAxisAngle(0, 0, 1, -rPose1.GetHeading());
      
      m_Covariance = rotationMatrix * rCovariance * rotationMatrix.Transpose();
    }
    
    /**
     * Gets the first pose
     * @return first pose
     */
    inline const Pose2& GetPose1()
    {
      return m_Pose1;
    }
    
    /**
     * Gets the second pose
     * @return second pose
     */
    inline const Pose2& GetPose2()
    {
      return m_Pose2;
    }
    
    /**
     * Gets the pose difference
     * @return pose difference
     */
    inline const Pose2& GetPoseDifference()
    {
      return m_PoseDifference;
    }
    
    /**
     * Gets the link covariance
     * @return link covariance
     */
    inline const Matrix3& GetCovariance()
    {
      return m_Covariance;
    }
    
  private:
    Pose2 m_Pose1;
    Pose2 m_Pose2;
    Pose2 m_PoseDifference;
    Matrix3 m_Covariance;
  }; // class LinkInfo

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Edge;

  /**
   * Represents an object in a graph
   */
  template<typename T>
  class Vertex
  {
    friend class Edge<T>;
    
  public:
    /**
     * Constructs a vertex representing the given object
     * @param pObject
     */
    Vertex(T* pObject)
      : m_pObject(pObject)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~Vertex()
    {
    }
    
    /**
     * Gets edges adjacent to this vertex
     * @return adjacent edges
     */
    inline const std::vector<Edge<T>*>& GetEdges() const
    {
      return m_Edges;
    }
    
    /**
     * Gets the object associated with this vertex
     * @return the object
     */
    inline T* GetObject() const
    {
      return m_pObject;
    }
  
    /**
     * Gets a vector of the vertices adjacent to this vertex
     * @return adjacent vertices
     */
    std::vector<Vertex<T>*> GetAdjacentVertices() const
    {
      std::vector<Vertex<T>*> vertices;
      
      const_forEach(typename std::vector<Edge<T>*>, &m_Edges)
      {
        Edge<T>* pEdge = *iter;
        
        // check both source and target because we have a undirected graph
        if (pEdge->GetSource() != this)
        {
          vertices.push_back(pEdge->GetSource());
        }
        
        if (pEdge->GetTarget() != this)
        {
          vertices.push_back(pEdge->GetTarget());
        }
      }
      
      return vertices;
    }
    
  private:
    /**
     * Adds the given edge to this vertex's edge list
     * @param pEdge edge to add
     */
    inline void AddEdge(Edge<T>* pEdge)
    {
      m_Edges.push_back(pEdge);
    }    
    
    T* m_pObject;
    std::vector<Edge<T>*> m_Edges;
  }; // class Vertex

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Edge
  {
  public:
    /**
     * Constructs an edge from the source to target vertex
     * @param pSource
     * @param pTarget
     */
    Edge(Vertex<T>* pSource, Vertex<T>* pTarget)
      : m_pSource(pSource)
      , m_pTarget(pTarget)
      , m_pLabel(NULL)
    {
      m_pSource->AddEdge(this);
      m_pTarget->AddEdge(this);
    }

    /**
     * Destructor
     */
    virtual ~Edge()
    {
      m_pSource = NULL;
      m_pTarget = NULL;

      if (m_pLabel != NULL)
      {
        delete m_pLabel;
        m_pLabel = NULL;
      }
    }

  public:
    /**
     * Gets the source vertex
     * @return source vertex
     */
    inline Vertex<T>* GetSource() const
    {
      return m_pSource;
    }

    /**
     * Gets the target vertex
     * @return target vertex
     */
    inline Vertex<T>* GetTarget() const
    {
      return m_pTarget;
    }

    /**
     * Gets the link info
     * @return link info
     */
    inline EdgeLabel* GetLabel()
    {
      return m_pLabel;
    }

    /**
     * Sets the link payload
     * @param pTag
     */
    inline void SetLabel(EdgeLabel* pLabel)
    {
      m_pLabel = pLabel;
    }

  private:
    Vertex<T>* m_pSource;
    Vertex<T>* m_pTarget;
    EdgeLabel* m_pLabel;
  }; // class Edge

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Visitor
  {
  public:
    /**
     * Applies the visitor to the vertex
     * @param pVertex
     * @return true if the visitor accepted the vertex, false otherwise
     */
    virtual kt_bool Visit(Vertex<T>* pVertex) = 0;
  }; // class Visitor

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Graph;

  template<typename T>
  class GraphTraversal
  {
  public:
    /**
     * Constructs a traverser for the given graph
     * @param pGraph
     */
    GraphTraversal(Graph<T>* pGraph)
      : m_pGraph(pGraph)
    {
    }

    /**
     * Destructor
     */
    virtual ~GraphTraversal()
    {
    }

  public:
    /**
     * Traverse the graph starting at the given vertex and applying the visitor to all visited nodes
     * @param pStartVertex
     * @param pVisitor
     */
    virtual std::vector<T*> Traverse(Vertex<T>* pStartVertex, Visitor<T>* pVisitor) = 0;

  protected:
    Graph<T>* m_pGraph;
  }; // class GraphTraversal  

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Graph
  {
  public:
    typedef std::map<kt_int32s, std::vector<Vertex<T>*> > VertexMap;
    
  public:
    /**
     * Default constructor
     */
    Graph()
      : m_pTraversal(NULL)
    {
    }

    /**
     * Destructor
     */
    virtual ~Graph()
    {
      Clear();
    }

  public:
    /**
     * Adds and indexes the given vertex into the map using the given id
     * @param id
     * @param pVertex
     */
    inline void Add(kt_int32s id, Vertex<T>* pVertex)
    {
      m_Vertices[id].push_back(pVertex);
    }
    
    /**
     * Adds an edge to the graph
     * @param pEdge
     */
    inline void Add(Edge<T>* pEdge)
    {
      m_Edges.push_back(pEdge);
    }

    /**
     * Deletes the graph data
     */
    void Clear()
    {
      forEachAs(typename VertexMap, &m_Vertices, indexIter)
      {
        // delete each vertex
        forEach(typename std::vector<Vertex<T>*>, &(indexIter->second))
        {
          delete *iter;
        }
      }
      m_Vertices.clear();

      const_forEach(typename std::vector<Edge<T>*>, &m_Edges)
      {
        delete *iter;
      }
      m_Edges.clear();
    }

    /**
     * Gets the edges of this graph
     * @return graph edges
     */
    inline const std::vector<Edge<T>*>& GetEdges() const
    {
      return m_Edges;
    }

    /**
     * Gets the vertices of this graph
     * @return graph vertices
     */
    inline const VertexMap& GetVertices() const
    {
      return m_Vertices;
    }

  protected:
    VertexMap m_Vertices;
    std::vector<Edge<T>*> m_Edges;
    GraphTraversal<T>* m_pTraversal;
  }; // class Graph

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanSolver
  {
  public:
    /**
     * Default constructor
     */
    ScanSolver()
      : m_Computed(false)
    {
    };

    /**
     * Destructor
     */
    virtual ~ScanSolver()
    {
    };

  public:
    /**
     * Solve!
     */
    void Compute()
    {
      if (!m_Computed)
      {
        InternalCompute();
        m_Computed = true;
      }
    }

    /**
     * Initialize the data needed by solver
     * @param rPoses
     * @param rPoseDifferences
     * @param rCovariances
     * @param rLinkScanIDs
     */
    virtual void Initialize(const Pose2Vector& rPoses, const Pose2Vector& rPoseDifferences, const std::vector<Matrix3>& rCovariances, const Matrix& rLinkScanIDs) = 0;

    /**
     * Get corrected poses after optimization
     * @return optimized poses
     */
    virtual const Pose2Vector& GetCorrectedPoses() const = 0;

    /**
     * Resets the solver
     */
    virtual void Clear() = 0;

  protected:
    /**
     * Specific to each algorithm
     */
    virtual void InternalCompute() = 0;

  protected:
    kt_bool m_Computed;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanMatcher;
  class DeviceManager;
  class MapperGraph;

  /**
   * The Mapper class
   */
  class KARTO_EXPORT Mapper
  {
    friend class MapperGraph;
    friend class ScanMatcher;
    
  public:
    /** 
     * Default constructor
     */
    Mapper();

    /**
     * Destructor
     */
    virtual ~Mapper();

  public:
    /**
     * Resets the mapper. 
     * Deallocate memory allocated in Initialize()
     */
    void Reset();

    /**
     * Process a localized range scan for incorporation into the map.  The scan must
     * be identified with a range finder device.  Once added to a map, the corrected pose information in the
     * localized scan will be updated to the correct pose as determined by the mapper.
     *
     * @param pScan A localized range scan that has pose information associated directly with the scan data.  The pose
     * is that of the range device originating the scan.  Note that the mapper will set corrected pose
     * information in the scan object.
     *
     * @return true if the scan was added successfully, false otherwise
     */
    kt_bool Process(LocalizedRangeScan* pScan);

    /**
     * Returns all processed scans added to the mapper.
     * NOTE: The returned scans have their corrected pose updated.
     * @return list of scans received and processed by the mapper. If no scans have been processed,
     * return an empty list.
     */
    LocalizedRangeScanVector GetAllProcessedScans() const;

    /**
     * Add a listener to mapper
     * @param pListener
     */
    void AddListener(MapperListener* pListener);

    /**
     * Remove a listener to mapper
     * @param pListener
     */
    void RemoveListener(MapperListener* pListener);

    /**
     * Set scan optimizer used by mapper when closing the loop
     * @param pSolver
     */
    void SetScanOptimizer(ScanSolver* pSolver);

    /**
     * Get scan link graph
     * @return graph
     */
    Graph<LocalizedRangeScan>* GetGraph() const;

    /**
     * Get the parameter manager
     * @return ParameterManager
     */
    ParameterManager* GetParameterManager() const;

  private:
    /**
     * Allocate memory needed for mapping
     * @param rangeThreshold
     */
    void Initialize(kt_double rangeThreshold);

    /**
     * Test if the scan is "sufficiently far" from the last scan added.
     * @param pScan scan to be checked
     * @param pLastScan last scan added to mapper
     * @return true if the scan is "sufficiently far" from the last scan added or
     * the scan is the first scan to be added
     */
    kt_bool HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const;

    /////////////////////////////////////////////
    // fire information for listeners!!

    /**
     * Fire a general message to listeners
     * @param rInfo
     */
    void FireInfo(const std::string& rInfo) const;

    /**
     * Fire a debug message to listeners
     * @param rInfo
     */
    void FireDebug(const std::string& rInfo) const;

    /**
     * Fire a message upon checking for loop closure to listeners
     * @param rInfo
     */
    void FireLoopClosureCheck(const std::string& rInfo) const;

    /**
     * Fire a message before loop closure to listeners
     * @param rInfo
     */
    void FireBeginLoopClosure(const std::string& rInfo) const;

    /**
     * Fire a message after loop closure to listeners
     * @param rInfo
     */
    void FireEndLoopClosure(const std::string& rInfo) const;

    // FireRunningScansUpdated

    // FireCovarianceCalculated

    // FireLoopClosureChain

  private:
    kt_bool m_Initialized;

    ParameterManager* m_pParameters;

    ScanMatcher* m_pSequentialScanMatcher;

    DeviceManager* m_pDeviceManager;

    MapperGraph* m_pGraph;
    ScanSolver* m_pScanOptimizer;

    std::vector<MapperListener*> m_Listeners;


    /** 
     * When set to true, the mapper will use a scan matching algorithm. In most real-world situations
     * this should be set to true so that the mapper algorithm can correct for noise and errors in
     * odometry and scan data. In some simulator environments where the simulated scan and odometry
     * data are very accurate, the scan matching algorithm can produce worse results. In those cases
     * set this to false to improve results.
     * Default value is true.
     */
    Parameter<kt_bool>* m_pUseScanMatching;

    /**
     * Default value is true.
     */
    Parameter<kt_bool>* m_pUseScanBarycenter;

    /**
     * Sets the minimum travel between scans.  If a new scan's position is more than minimumTravelDistance
     * from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
     * new scan if it also does not meet the minimum change in heading requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 0.2 (meters).
     */
    Parameter<kt_double>* m_pMinimumTravelDistance;

    /**
     * Sets the minimum heading change between scans. If a new scan's heading is more than minimumTravelHeading
     * from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
     * new scan if it also does not meet the minimum travel distance requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 10 degrees.
     */
    Parameter<kt_double>* m_pMinimumTravelHeading;

    /**
     * Scan buffer size is the length of the scan chain stored for scan matching.
     * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
     * The idea is to get an area approximately 20 meters long for scan matching.
     * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
     * should be 20 / 0.3 = 67.)
     * Default value is 67.
     */
    Parameter<kt_int32u>* m_pScanBufferSize;

    /**
     * Scan buffer maximum scan distance is the maximum distance between the first and last scans
     * in the scan chain stored for matching.
     * Default value is 20.0.
     */
    Parameter<kt_double>* m_pScanBufferMaximumScanDistance;

    /**
     * Scans are linked only if the correlation response value is greater than this value.
     * Default value is 0.4
     */
    Parameter<kt_double>* m_pLinkMatchMinimumResponseFine;

    /**
     * Maximum distance between linked scans.  Scans that are farther apart will not be linked
     * regardless of the correlation response value.
     * Default value is 6.0 meters.
     */
    Parameter<kt_double>* m_pLinkScanMaximumDistance;

    /**
     * Scans less than this distance from the current position will be considered for a match
     * in loop closure.
     * Default value is 4.0 meters.
     */
    Parameter<kt_double>* m_pLoopSearchMaximumDistance;

    /**
     * When the loop closure detection finds a candidate it must be part of a large
     * set of linked scans. If the chain of scans is less than this value we do not attempt
     * to close the loop.
     * Default value is 10.
     */
    Parameter<kt_int32u>* m_pLoopMatchMinimumChainSize;

    /**
     * The co-variance values for a possible loop closure have to be less than this value
     * to consider a viable solution. This applies to the coarse search.
     * Default value is 0.16.
     */
    Parameter<kt_double>* m_pLoopMatchMaximumVarianceCoarse;

    /**
     * If response is larger then this, then initiate loop closure search at the coarse resolution.
     * Default value is 0.7.
     */
    Parameter<kt_double>* m_pLoopMatchMinimumResponseCoarse;

    /**
     * If response is larger then this, then initiate loop closure search at the fine resolution.
     * Default value is 0.7.
     */
    Parameter<kt_double>* m_pLoopMatchMinimumResponseFine;

    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters correlationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    Parameter<kt_double>* m_pCorrelationSearchSpaceDimension;

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    Parameter<kt_double>* m_pCorrelationSearchSpaceResolution;

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    Parameter<kt_double>* m_pCorrelationSearchSpaceSmearDeviation;


    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters loopCorrelationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    Parameter<kt_double>* m_pLoopSearchSpaceDimension;

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    Parameter<kt_double>* m_pLoopSearchSpaceResolution;

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    Parameter<kt_double>* m_pLoopSearchSpaceSmearDeviation;

    //////////////////////////////////////////////////////////////////////////////
    // ScanMatcherParameters;

    // Variance of penalty for deviating from odometry when scan-matching.
    // The penalty is a multiplier (less than 1.0) is a function of the
    // delta of the scan position being tested and the odometric pose
    Parameter<kt_double>* m_pDistanceVariancePenalty;
    Parameter<kt_double>* m_pAngleVariancePenalty;

    // The range of angles to search during a coarse search and a finer search
    Parameter<kt_double>* m_pFineSearchAngleOffset;
    Parameter<kt_double>* m_pCoarseSearchAngleOffset;

    // Resolution of angles to search during a coarse search
    Parameter<kt_double>* m_pCoarseAngleResolution;

    // Minimum value of the penalty multiplier so scores do not
    // become too small
    Parameter<kt_double>* m_pMinimumAnglePenalty;
    Parameter<kt_double>* m_pMinimumDistancePenalty;

    // whether to increase the search space if no good matches are initially found
    Parameter<kt_bool>* m_pUseResponseExpansion;

  };

}

#endif // __KARTO_MAPPER__

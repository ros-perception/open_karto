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

#ifndef OPEN_KARTO_MAPPER_H
#define OPEN_KARTO_MAPPER_H

#include <map>
#include <vector>

#include <Eigen/Core>

#include <open_karto/Karto.h>

namespace karto
{
  ////////////////////////////////////////////////////////////////////////////////////////
  // Listener classes

  /**
   * Abstract class to listen to mapper general messages
   */
  class MapperListener
  {
  public:
    /**
     * Called with general message
     */
    virtual void Info(const std::string& /*rInfo*/) {};
  };

  /**
   * Abstract class to listen to mapper debug messages
   */
  class MapperDebugListener
  {
  public:
    /**
     * Called with debug message
     */
    virtual void Debug(const std::string& /*rInfo*/) {};
  };

  /**
   * Abstract class to listen to mapper loop closure messages
   */
  class MapperLoopClosureListener : public MapperListener
  {
  public:
    /**
     * Called when checking for loop closures
     */
    virtual void LoopClosureCheck(const std::string& /*rInfo*/) {};

    /**
     * Called when loop closure is starting
     */
    virtual void BeginLoopClosure(const std::string& /*rInfo*/) {};

    /**
     * Called when loop closure is over
     */
    virtual void EndLoopClosure(const std::string& /*rInfo*/) {};
  };  // MapperListener

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Class for edge labels
   */
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
  };  // EdgeLabel

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
  };  // LinkInfo

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
  };  // Vertex<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents an edge in a graph
   */
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
     * @param pLabel
     */
    inline void SetLabel(EdgeLabel* pLabel)
    {
      m_pLabel = pLabel;
    }

  private:
    Vertex<T>* m_pSource;
    Vertex<T>* m_pTarget;
    EdgeLabel* m_pLabel;
  };  // class Edge<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Visitor class
   */
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
  };  // Visitor<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Graph;

  /**
   * Graph traversal algorithm
   */
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
    /**
     * Graph being traversed
     */
    Graph<T>* m_pGraph;
  };  // GraphTraversal<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Graph
   */
  template<typename T>
  class Graph
  {
  public:
    /**
     * Maps names to vector of vertices
     */
    typedef std::map<Name, std::vector<Vertex<T>*> > VertexMap;

  public:
    /**
     * Default constructor
     */
    Graph()
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
     * Adds and indexes the given vertex into the map using the given name
     * @param rName
     * @param pVertex
     */
    inline void AddVertex(const Name& rName, Vertex<T>* pVertex)
    {
      m_Vertices[rName].push_back(pVertex);
    }

    /**
     * Adds an edge to the graph
     * @param pEdge
     */
    inline void AddEdge(Edge<T>* pEdge)
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
    /**
     * Map of names to vector of vertices
     */
    VertexMap m_Vertices;

    /**
     * Edges of this graph
     */
    std::vector<Edge<T>*> m_Edges;
  };  // Graph<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Mapper;
  class ScanMatcher;

  /**
   * Graph for graph SLAM algorithm
   */
  class KARTO_EXPORT MapperGraph : public Graph<LocalizedRangeScan>
  {
  public:
    /**
     * Graph for graph SLAM
     * @param pMapper
     * @param rangeThreshold
     */
    MapperGraph(Mapper* pMapper, kt_double rangeThreshold);

    /**
     * Destructor
     */
    virtual ~MapperGraph();

  public:
    /**
     * Adds a vertex representing the given scan to the graph
     * @param pScan
     */
    void AddVertex(LocalizedRangeScan* pScan);

    /**
     * Creates an edge between the source scan vertex and the target scan vertex if it
     * does not already exist; otherwise return the existing edge
     * @param pSourceScan
     * @param pTargetScan
     * @param rIsNewEdge set to true if the edge is new
     * @return edge between source and target scan vertices
     */
    Edge<LocalizedRangeScan>* AddEdge(LocalizedRangeScan* pSourceScan,
                                      LocalizedRangeScan* pTargetScan,
                                      kt_bool& rIsNewEdge);

    /**
     * Link scan to last scan and nearby chains of scans
     * @param pScan
     * @param rCovariance uncertainty of match
     */
    void AddEdges(LocalizedRangeScan* pScan, const Matrix3& rCovariance);

    /**
     * Tries to close a loop using the given scan with the scans from the given device
     * @param pScan
     * @param rSensorName
     */
    kt_bool TryCloseLoop(LocalizedRangeScan* pScan, const Name& rSensorName);

    /**
     * Optimizes scan poses
     */
    void CorrectPoses();

    /**
     * Find "nearby" (no further than given distance away) scans through graph links
     * @param pScan
     * @param maxDistance
     */
    LocalizedRangeScanVector FindNearLinkedScans(LocalizedRangeScan* pScan, kt_double maxDistance);

    /**
     * Gets the graph's scan matcher
     * @return scan matcher
     */
    inline ScanMatcher* GetLoopScanMatcher() const
    {
      return m_pLoopScanMatcher;
    }

  private:
    /**
     * Gets the vertex associated with the given scan
     * @param pScan
     * @return vertex of scan
     */
    inline Vertex<LocalizedRangeScan>* GetVertex(LocalizedRangeScan* pScan)
    {
      return m_Vertices[pScan->GetSensorName()][pScan->GetStateId()];
    }

    /**
     * Finds the closest scan in the vector to the given pose
     * @param rScans
     * @param rPose
     */
    LocalizedRangeScan* GetClosestScanToPose(const LocalizedRangeScanVector& rScans, const Pose2& rPose) const;

    /**
     * Adds an edge between the two scans and labels the edge with the given mean and covariance
     * @param pFromScan
     * @param pToScan
     * @param rMean
     * @param rCovariance
     */
    void LinkScans(LocalizedRangeScan* pFromScan,
                   LocalizedRangeScan* pToScan,
                   const Pose2& rMean,
                   const Matrix3& rCovariance);

    /**
     * Find nearby chains of scans and link them to scan if response is high enough
     * @param pScan
     * @param rMeans
     * @param rCovariances
     */
    void LinkNearChains(LocalizedRangeScan* pScan, Pose2Vector& rMeans, std::vector<Matrix3>& rCovariances);

    /**
     * Link the chain of scans to the given scan by finding the closest scan in the chain to the given scan
     * @param rChain
     * @param pScan
     * @param rMean
     * @param rCovariance
     */
    void LinkChainToScan(const LocalizedRangeScanVector& rChain,
                         LocalizedRangeScan* pScan,
                         const Pose2& rMean,
                         const Matrix3& rCovariance);

    /**
     * Find chains of scans that are close to given scan
     * @param pScan
     * @return chains of scans
     */
    std::vector<LocalizedRangeScanVector> FindNearChains(LocalizedRangeScan* pScan);

    /**
     * Compute mean of poses weighted by covariances
     * @param rMeans
     * @param rCovariances
     * @return weighted mean
     */
    Pose2 ComputeWeightedMean(const Pose2Vector& rMeans, const std::vector<Matrix3>& rCovariances) const;

    /**
     * Tries to find a chain of scan from the given device starting at the
     * given scan index that could possibly close a loop with the given scan
     * @param pScan
     * @param rSensorName
     * @param rStartNum
     * @return chain that can possibly close a loop with given scan
     */
    LocalizedRangeScanVector FindPossibleLoopClosure(LocalizedRangeScan* pScan,
                                                     const Name& rSensorName,
                                                     kt_int32u& rStartNum);

  private:
    /**
     * Mapper of this graph
     */
    Mapper* m_pMapper;

    /**
     * Scan matcher for loop closures
     */
    ScanMatcher* m_pLoopScanMatcher;

    /**
     * Traversal algorithm to find near linked scans
     */
    GraphTraversal<LocalizedRangeScan>* m_pTraversal;
  };  // MapperGraph

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Graph optimization algorithm
   */
  class ScanSolver
  {
  public:
    /**
     * Vector of id-pose pairs
     */
    typedef std::vector<std::pair<kt_int32s, Pose2> > IdPoseVector;

    /**
     * Default constructor
     */
    ScanSolver()
    {
    }

    /**
     * Destructor
     */
    virtual ~ScanSolver()
    {
    }

  public:
    /**
     * Solve!
     */
    virtual void Compute() = 0;

    /**
     * Get corrected poses after optimization
     * @return optimized poses
     */
    virtual const IdPoseVector& GetCorrections() const = 0;

    /**
     * Adds a node to the solver
     */
    virtual void AddNode(Vertex<LocalizedRangeScan>* /*pVertex*/)
    {
    }

    /**
     * Removes a node from the solver
     */
    virtual void RemoveNode(kt_int32s /*id*/)
    {
    }

    /**
     * Adds a constraint to the solver
     */
    virtual void AddConstraint(Edge<LocalizedRangeScan>* /*pEdge*/)
    {
    }

    /**
     * Removes a constraint from the solver
     */
    virtual void RemoveConstraint(kt_int32s /*sourceId*/, kt_int32s /*targetId*/)
    {
    }

    /**
     * Resets the solver
     */
    virtual void Clear(){};

    /**
     * Get graph stored
     */
    virtual void getGraph(std::vector<Eigen::Vector2d>& ag) {};

    /**
     * Modify a node's pose
     */
    virtual void ModifyNode(const int& unique_id, Eigen::Vector3d pose)
    {
      std::cout << "ModifyNode method not implemented for this solver type. Manual loop closure unavailable." << std::endl;
    };
    /**
     * Get node's yaw
     */
    virtual void GetNodeOrientation(const int& unique_id, double& pose)
    {
      std::cout << "GetNodeOrientation method not implemented for this solver type." << std::endl;
    };
  };  // ScanSolver

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Implementation of a correlation grid used for scan matching
   */
  class CorrelationGrid : public Grid<kt_int8u>
  {
  public:
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
      delete [] m_pKernel;
    }

  public:
    /**
     * Create a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param resolution
     * @param smearDeviation
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(kt_int32s width,
                                       kt_int32s height,
                                       kt_double resolution,
                                       kt_double smearDeviation)
    {
      assert(resolution != 0.0);

      // +1 in case of roundoff
      kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;

      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);

      return pGrid;
    }

    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
    {
      kt_int32s x = rGrid.GetX() + m_Roi.GetX();
      kt_int32s y = rGrid.GetY() + m_Roi.GetY();

      return Grid<kt_int8u>::GridIndex(Vector2<kt_int32s>(x, y), boundaryCheck);
    }

    /**
     * Get the Region Of Interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2<kt_int32s>& GetROI() const
    {
      return m_Roi;
    }

    /**
     * Sets the Region Of Interest (ROI)
     * @param roi
     */
    inline void SetROI(const Rectangle2<kt_int32s>& roi)
    {
      m_Roi = roi;
    }

    /**
     * Smear cell if the cell at the given point is marked as "occupied"
     * @param rGridPoint
     */
    inline void SmearPoint(const Vector2<kt_int32s>& rGridPoint)
    {
      assert(m_pKernel != NULL);

      int gridIndex = GridIndex(rGridPoint);
      if (GetDataPointer()[gridIndex] != GridStates_Occupied)
      {
        return;
      }

      kt_int32s halfKernel = m_KernelSize / 2;

      // apply kernel
      for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
      {
        kt_int8u* pGridAdr = GetDataPointer(Vector2<kt_int32s>(rGridPoint.GetX(), rGridPoint.GetY() + j));

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

  protected:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param borderSize
     * @param resolution
     * @param smearDeviation
     */
    CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize,
                    kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
      , m_SmearDeviation(smearDeviation)
      , m_pKernel(NULL)
    {
      GetCoordinateConverter()->SetScale(1.0 / resolution);

      // setup region of interest
      m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);

      // calculate kernel
      CalculateKernel();
    }

    /**
     * Sets up the kernel for grid smearing.
     */
    virtual void CalculateKernel()
    {
      kt_double resolution = GetResolution();

      assert(resolution != 0.0);
      assert(m_SmearDeviation != 0.0);

      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
      const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

      // check if given value too small or too big
      if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        std::stringstream error;
        error << "Mapper Error:  Smear deviation too small:  Must be between "
              << MIN_SMEAR_DISTANCE_DEVIATION
              << " and "
              << MAX_SMEAR_DISTANCE_DEVIATION;
        throw std::runtime_error(error.str());
      }

      // NOTE:  Currently assumes a two-dimensional kernel

      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;

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
          kt_double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
          kt_double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
          kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

          kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
          assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));

          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
        }
      }
    }

    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param smearDeviation
     * @param resolution
     * @return kernel half-size based on the parameters
     */
    static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
    {
      assert(resolution != 0.0);

      return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
    }

  private:
    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    kt_double m_SmearDeviation;

    // Size of one side of the kernel
    kt_int32s m_KernelSize;

    // Cached kernel for smearing
    kt_int8u* m_pKernel;

    // region of interest
    Rectangle2<kt_int32s> m_Roi;
  };  // CorrelationGrid

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Scan matcher
   */
  class KARTO_EXPORT ScanMatcher
  {
  public:
    /**
     * Destructor
     */
    virtual ~ScanMatcher();

  public:
    /**
     * Create a scan matcher with the given parameters
     */
    static ScanMatcher* Create(Mapper* pMapper,
                               kt_double searchSize,
                               kt_double resolution,
                               kt_double smearDeviation,
                               kt_double rangeThreshold);

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
    kt_double MatchScan(LocalizedRangeScan* pScan,
                        const LocalizedRangeScanVector& rBaseScans,
                        Pose2& rMean, Matrix3& rCovariance,
                        kt_bool doPenalize = true,
                        kt_bool doRefineMatch = true);

    /**
     * Finds the best pose for the scan centering the search in the correlation grid
     * at the given pose and search in the space by the vector and angular offsets
     * in increments of the given resolutions
     * @param pScan scan to match against correlation grid
     * @param rSearchCenter the center of the search space
     * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
     * @param rSearchSpaceResolution how fine a granularity to search in the search space
     * @param searchAngleOffset searches poses in the angles offset by this angle around search center
     * @param searchAngleResolution how fine a granularity to search in the angular search space
     * @param doPenalize whether to penalize matches further from the search center
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doingFineMatch whether to do a finer search after coarse search
     * @return strength of response
     */
    kt_double CorrelateScan(LocalizedRangeScan* pScan,
                            const Pose2& rSearchCenter,
                            const Vector2<kt_double>& rSearchSpaceOffset,
                            const Vector2<kt_double>& rSearchSpaceResolution,
                            kt_double searchAngleOffset,
                            kt_double searchAngleResolution,
                            kt_bool doPenalize,
                            Pose2& rMean,
                            Matrix3& rCovariance,
                            kt_bool doingFineMatch);

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
    void ComputePositionalCovariance(const Pose2& rBestPose,
                                     kt_double bestResponse,
                                     const Pose2& rSearchCenter,
                                     const Vector2<kt_double>& rSearchSpaceOffset,
                                     const Vector2<kt_double>& rSearchSpaceResolution,
                                     kt_double searchAngleResolution,
                                     Matrix3& rCovariance);

    /**
     * Computes the angular covariance of the best pose
     * @param rBestPose
     * @param bestResponse
     * @param rSearchCenter
     * @param searchAngleOffset
     * @param searchAngleResolution
     * @param rCovariance
     */
    void ComputeAngularCovariance(const Pose2& rBestPose,
                                  kt_double bestResponse,
                                  const Pose2& rSearchCenter,
                                  kt_double searchAngleOffset,
                                  kt_double searchAngleResolution,
                                  Matrix3& rCovariance);

    /**
     * Gets the correlation grid data (for debugging)
     * @return correlation grid
     */
    inline CorrelationGrid* GetCorrelationGrid() const
    {
      return m_pCorrelationGrid;
    }

  private:
    /**
     * Marks cells where scans' points hit as being occupied
     * @param rScans scans whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     */
    void AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint);

    /**
     * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
     * @param pScan scan whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     * @param doSmear whether the points will be smeared
     */
    void AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear = true);

    /**
     * Compute which points in a scan are on the same side as the given viewpoint
     * @param pScan
     * @param rViewPoint
     * @return points on the same side
     */
    PointVectorDouble FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const;

    /**
     * Get response at given position for given rotation (only look up valid points)
     * @param angleIndex
     * @param gridPositionIndex
     * @return response
     */
    kt_double GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const;

  protected:
    /**
     * Default constructor
     */
    ScanMatcher(Mapper* pMapper)
      : m_pMapper(pMapper)
      , m_pCorrelationGrid(NULL)
      , m_pSearchSpaceProbs(NULL)
      , m_pGridLookup(NULL)
    {
    }

  private:
    Mapper* m_pMapper;

    CorrelationGrid* m_pCorrelationGrid;
    Grid<kt_double>* m_pSearchSpaceProbs;

    GridIndexLookup<kt_int8u>* m_pGridLookup;
  };  // ScanMatcher

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanManager;

  /**
   * Manages the devices for the mapper
   */
  class KARTO_EXPORT MapperSensorManager //: public SensorManager
  {
    typedef std::map<Name, ScanManager*> ScanManagerMap;

  public:
    /**
     * Constructor
     */
    MapperSensorManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
      , m_NextScanId(0)
    {
    }

    MapperSensorManager(){
	}

    /**
     * Destructor
     */
    virtual ~MapperSensorManager()
    {
      Clear();
    }

  public:
    /**
     * Registers a sensor (with given name); do
     * nothing if device already registered
     * @param rSensorName
     */
    void RegisterSensor(const Name& rSensorName);

    /**
     * Gets scan from given sensor with given ID
     * @param rSensorName
     * @param scanIndex
     * @return localized range scan
     */
    LocalizedRangeScan* GetScan(const Name& rSensorName, kt_int32s scanIndex);

    /**
     * Gets names of all sensors
     * @return sensor names
     */
    inline std::vector<Name> GetSensorNames()
    {
      std::vector<Name> deviceNames;
      const_forEach(ScanManagerMap, &m_ScanManagers)
      {
        deviceNames.push_back(iter->first);
      }

      return deviceNames;
    }

    /**
     * Gets last scan of given sensor
     * @param rSensorName
     * @return last localized range scan of sensor
     */
    LocalizedRangeScan* GetLastScan(const Name& rSensorName);

    /**
     * Sets the last scan of device of given scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan);

    /**
     * Gets the scan with the given unique id
     * @param id
     * @return scan
     */
    inline LocalizedRangeScan* GetScan(kt_int32s id)
    {
      assert(math::IsUpTo(id, (kt_int32s)m_Scans.size()));

      return m_Scans[id];
    }

    /**
     * Adds scan to scan vector of device that recorded scan
     * @param pScan
     */
    void AddScan(LocalizedRangeScan* pScan);

    /**
     * Adds scan to running scans of device that recorded scan
     * @param pScan
     */
    void AddRunningScan(LocalizedRangeScan* pScan);

    /**
     * Gets scans of device
     * @param rSensorName
     * @return scans of device
     */
    LocalizedRangeScanVector& GetScans(const Name& rSensorName);

    /**
     * Gets running scans of device
     * @param rSensorName
     * @return running scans of device
     */
    LocalizedRangeScanVector& GetRunningScans(const Name& rSensorName);

    /**
     * Gets all scans of all devices
     * @return all scans of all devices
     */
    LocalizedRangeScanVector GetAllScans();

    /**
     * Deletes all scan managers of all devices
     */
    void Clear();

  private:
    /**
     * Get scan manager for localized range scan
     * @return ScanManager
     */
    inline ScanManager* GetScanManager(LocalizedRangeScan* pScan)
    {
      return GetScanManager(pScan->GetSensorName());
    }

    /**
     * Get scan manager for id
     * @param rSensorName
     * @return ScanManager
     */
    inline ScanManager* GetScanManager(const Name& rSensorName)
    {
      if (m_ScanManagers.find(rSensorName) != m_ScanManagers.end())
      {
        return m_ScanManagers[rSensorName];
      }

      return NULL;
    }

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(m_ScanManagers);
		ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumSize);
		ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumDistance);
		ar & BOOST_SERIALIZATION_NVP(m_NextScanId);
		ar & BOOST_SERIALIZATION_NVP(m_Scans);
	}

  private:
    // map from device ID to scan data
    ScanManagerMap m_ScanManagers;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;

    kt_int32s m_NextScanId;

    std::vector<LocalizedRangeScan*> m_Scans;
  };  // MapperSensorManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Graph SLAM mapper. Creates a map given a set of LocalizedRangeScans
   * The current Karto implementation is a proprietary, high-performance
   * scan-matching algorithm that constructs a map from individual range scans and corrects for
   * errors in the range and odometry data.
   *
   * The following parameters can be set on the Mapper.
   *
   *  \a UseScanMatching (ParameterBool)\n
   *     When set to true, the mapper will use a scan matching algorithm. In most real-world situations
   *     this should be set to true so that the mapper algorithm can correct for noise and errors in
   *     odometry and scan data. In some simulator environments where the simulated scan and odometry
   *     data are very accurate, the scan matching algorithm can produce worse results. In those cases
   *     set to false to improve results.
   *     Default value is true.
   *
   *  \a UseScanBarycenter (ParameterBool)\n
   *
   *  \a UseResponseExpansion (ParameterBool)\n
   *
   *  \a RangeThreshold (ParameterDouble - meters)\n
   *     The range threshold is used to truncate range scan distance measurement readings.  The threshold should
   *     be set such that adjacent range readings in a scan will generally give "solid" coverage of objects.
   *
   *     \image html doxygen/RangeThreshold.png
   *     \image latex doxygen/RangeThreshold.png "" width=3in
   *
   *     Having solid coverage depends on the map resolution and the angular resolution of the range scan device.
   *     The following are the recommended threshold values for the corresponding map resolution and range finder
   *     resolution values:
   *
   *     <table border=0>
   *      <tr>
   *       <td><b>Map Resolution</b></td>
   *       <td colspan=3 align=center><b>Laser Angular Resolution</b></td>
   *      </tr>
   *      <tr>
   *       <td></td>
   *       <td align=center><b>1.0 degree</b></td>
   *       <td align=center><b>0.5 degree</b></td>
   *       <td align=center><b>0.25 degree</b></td>
   *      </tr>
   *      <tr>
   *       <td align=center><b>0.1</b></td>
   *       <td align=center>5.7m</td>
   *       <td align=center>11.4m</td>
   *       <td align=center>22.9m</td>
   *      </tr>
   *      <tr>
   *       <td align=center><b>0.05</b></td>
   *       <td align=center>2.8m</td>
   *       <td align=center>5.7m</td>
   *       <td align=center>11.4m</td>
   *      </tr>
   *      <tr>
   *       <td align=center><b>0.01</b></td>
   *       <td align=center>0.5m</td>
   *       <td align=center>1.1m</td>
   *       <td align=center>2.3m</td>
   *      </tr>
   *     </table>
   *
   *     Note that the value of RangeThreshold should be adjusted taking into account the values of
   *     MinimumTravelDistance and MinimumTravelHeading (see also below).  By incorporating scans
   *     into the map more frequently, the RangeThreshold value can be increased as the additional scans
   *     will "fill in" the gaps of objects at greater distances where there is less solid coverage.
   *
   *     Default value is 12.0 (meters).
   *
   *  \a MinimumTravelDistance (ParameterDouble - meters)\n
   *     Sets the minimum travel between scans. If a new scan's position is more than minimumDistance from
   *     the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
   *     new scan if it also does not meet the minimum change in heading requirement.
   *     For performance reasons, generally it is a good idea to only process scans if the robot
   *     has moved a reasonable amount.
   *     Default value is 0.3 (meters).
   *
   *  \a MinimumTravelHeading (ParameterDouble - radians)\n
   *     Sets the minimum heading change between scans. If a new scan's heading is more than minimumHeading from
   *     the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
   *     new scan if it also does not meet the minimum travel distance requirement.
   *     For performance reasons, generally it is a good idea to only process scans if the robot
   *     has moved a reasonable amount.
   *     Default value is 0.08726646259971647 (radians) - 5 degrees.
   *
   *  \a ScanBufferSize (ParameterIn32u - size)\n
   *     Scan buffer size is the length of the scan chain stored for scan matching.
   *     "ScanBufferSize" should be set to approximately "ScanBufferMaximumScanDistance" / "MinimumTravelDistance".
   *     The idea is to get an area approximately 20 meters long for scan matching.
   *     For example, if we add scans every MinimumTravelDistance = 0.3 meters, then "ScanBufferSize"
   *     should be 20 / 0.3 = 67.)
   *     Default value is 67.
   *
   *  \a ScanBufferMaximumScanDistance (ParameterDouble - meters)\n
   *     Scan buffer maximum scan distance is the maximum distance between the first and last scans
   *     in the scan chain stored for matching.
   *     Default value is 20.0.
   *
   *  \a CorrelationSearchSpaceDimension (ParameterDouble - meters)\n
   *     The size of the correlation grid used by the matcher.
   *     Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
   *
   *  \a CorrelationSearchSpaceResolution (ParameterDouble - meters)\n
   *     The resolution (size of a grid cell) of the correlation grid.
   *     Default value is 0.01 meters.
   *
   *  \a CorrelationSearchSpaceSmearDeviation (ParameterDouble - meters)\n
   *     The robot position is smeared by this value in X and Y to create a smoother response.
   *     Default value is 0.03 meters.
   *
   *  \a LinkMatchMinimumResponseFine (ParameterDouble - probability (>= 0.0, <= 1.0))\n
   *     Scans are linked only if the correlation response value is greater than this value.
   *     Default value is 0.4
   *
   *  \a LinkScanMaximumDistance (ParameterDouble - meters)\n
   *     Maximum distance between linked scans.  Scans that are farther apart will not be linked
   *     regardless of the correlation response value.
   *     Default value is 6.0 meters.
   *
   *  \a LoopSearchSpaceDimension (ParameterDouble - meters)\n
   *     Dimension of the correlation grid used by the loop closure detection algorithm
   *     Default value is 4.0 meters.
   *
   *  \a LoopSearchSpaceResolution (ParameterDouble - meters)\n
   *     Coarse resolution of the correlation grid used by the matcher to determine a possible
   *     loop closure.
   *     Default value is 0.05 meters.
   *
   *  \a LoopSearchSpaceSmearDeviation (ParameterDouble - meters)\n
   *     Smearing distance in the correlation grid used by the matcher to determine a possible
   *     loop closure match.
   *     Default value is 0.03 meters.
   *
   *  \a LoopSearchMaximumDistance (ParameterDouble - meters)\n
   *     Scans less than this distance from the current position will be considered for a match
   *     in loop closure.
   *     Default value is 4.0 meters.
   *
   *  \a LoopMatchMinimumChainSize (ParameterIn32s)\n
   *     When the loop closure detection finds a candidate it must be part of a large
   *     set of linked scans. If the chain of scans is less than this value we do not attempt
   *     to close the loop.
   *     Default value is 10.
   *
   *  \a LoopMatchMaximumVarianceCoarse (ParameterDouble)\n
   *     The co-variance values for a possible loop closure have to be less than this value
   *     to consider a viable solution. This applies to the coarse search.
   *     Default value is 0.16.
   *
   *  \a LoopMatchMinimumResponseCoarse (ParameterDouble - probability (>= 0.0, <= 1.0))\n
   *     If response is larger then this then initiate loop closure search at the coarse resolution.
   *     Default value is 0.7.
   *
   *  \a LoopMatchMinimumResponseFine (ParameterDouble - probability (>= 0.0, <= 1.0))\n
   *     If response is larger then this then initiate loop closure search at the fine resolution.
   *     Default value is 0.5.
   */
  class KARTO_EXPORT Mapper : public Module
  {
    friend class MapperGraph;
    friend class ScanMatcher;

  public:
    /**
     * Default constructor
     */
    Mapper();

    /**
     * Constructor a mapper with a name
     * @param rName mapper name
     */
    Mapper(const std::string& rName);

    /**
     * Destructor
     */
    virtual ~Mapper();

  public:
    /**
     * Allocate memory needed for mapping
     * @param rangeThreshold
     */
    void Initialize(kt_double rangeThreshold);

    /**
     * Save map to file 
     * @param filename 
     */
    void SaveToFile(const std::string& filename);

    /**
     * Load map from file 
     * @param filename 
     */
    void LoadFromFile(const std::string& filename);

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
    virtual kt_bool Process(LocalizedRangeScan* pScan);

    /**
     * Process an Object
     */
    virtual kt_bool Process(Object* pObject);

    /**
     * Returns all processed scans added to the mapper.
     * NOTE: The returned scans have their corrected pose updated.
     * @return list of scans received and processed by the mapper. If no scans have been processed,
     * return an empty list.
     */
    virtual const LocalizedRangeScanVector GetAllProcessedScans() const;

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
    void SetScanSolver(ScanSolver* pSolver);

    /**
     * Get scan link graph
     * @return graph
     */
    virtual MapperGraph* GetGraph() const;

    /**
     * Gets the sequential scan matcher
     * @return sequential scan matcher
     */
    virtual ScanMatcher* GetSequentialScanMatcher() const;

    /**
     * Gets the loop scan matcher
     * @return loop scan matcher
     */
    virtual ScanMatcher* GetLoopScanMatcher() const;

    /**
     * Gets the device manager
     * @return device manager
     */
    inline MapperSensorManager* GetMapperSensorManager() const
    {
      return m_pMapperSensorManager;
    }

    /**
     * Tries to close a loop using the given scan with the scans from the given sensor
     * @param pScan
     * @param rSensorName
     */
    inline kt_bool TryCloseLoop(LocalizedRangeScan* pScan, const Name& rSensorName)
    {
      return m_pGraph->TryCloseLoop(pScan, rSensorName);
    }

    inline void CorrectPoses()
    {
      m_pGraph->CorrectPoses();
    }

  private:
    void InitializeParameters();

    /**
     * Test if the scan is "sufficiently far" from the last scan added.
     * @param pScan scan to be checked
     * @param pLastScan last scan added to mapper
     * @return true if the scan is "sufficiently far" from the last scan added or
     * the scan is the first scan to be added
     */
    kt_bool HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const;

  public:
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
    /**
     * Restrict the copy constructor
     */
    Mapper(const Mapper&);

    /**
     * Restrict the assignment operator
     */
    const Mapper& operator=(const Mapper&);

  public:
    void SetUseScanMatching(kt_bool val) { m_pUseScanMatching->SetValue(val); }

  private:
    kt_bool m_Initialized;

    ScanMatcher* m_pSequentialScanMatcher;

    MapperSensorManager* m_pMapperSensorManager;

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
     * Sets the minimum time between scans. If a new scan's time stamp is
     * longer than MinimumTimeInterval from the previously processed scan,
     * the mapper will use the data from the new scan. Otherwise, it will
     * discard the new scan if it also does not meet the minimum travel
     * distance and heading requirements. For performance reasons, it is
     * generally it is a good idea to only process scans if a reasonable
     * amount of time has passed. This parameter is particularly useful
     * when there is a need to process scans while the robot is stationary.
     * Default value is 3600 (seconds), effectively disabling this parameter.
     */
    Parameter<kt_double>* m_pMinimumTimeInterval;

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
     * Enable/disable loop closure.
     * Default is enabled.
     */
    Parameter<kt_bool>* m_pDoLoopClosing;

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

  public:
    /* Abstract methods for parameter setters and getters */

    /* Getters */
    // General Parameters
    bool getParamUseScanMatching();
    bool getParamUseScanBarycenter();
    double getParamMinimumTimeInterval();
    double getParamMinimumTravelDistance();
    double getParamMinimumTravelHeading();
    int getParamScanBufferSize();
    double getParamScanBufferMaximumScanDistance();
    double getParamLinkMatchMinimumResponseFine();
    double getParamLinkScanMaximumDistance();
    double getParamLoopSearchMaximumDistance();
    bool getParamDoLoopClosing();
    int getParamLoopMatchMinimumChainSize();
    double getParamLoopMatchMaximumVarianceCoarse();
    double getParamLoopMatchMinimumResponseCoarse();
    double getParamLoopMatchMinimumResponseFine();

    // Correlation Parameters - Correlation Parameters
    double getParamCorrelationSearchSpaceDimension();
    double getParamCorrelationSearchSpaceResolution();
    double getParamCorrelationSearchSpaceSmearDeviation();

    // Correlation Parameters - Loop Closure Parameters
    double getParamLoopSearchSpaceDimension();
    double getParamLoopSearchSpaceResolution();
    double getParamLoopSearchSpaceSmearDeviation();

    // Scan Matcher Parameters
    double getParamDistanceVariancePenalty();
    double getParamAngleVariancePenalty();
    double getParamFineSearchAngleOffset();
    double getParamCoarseSearchAngleOffset();
    double getParamCoarseAngleResolution();
    double getParamMinimumAnglePenalty();
    double getParamMinimumDistancePenalty();
    bool getParamUseResponseExpansion();

    /* Setters */
    // General Parameters
    void setParamUseScanMatching(bool b);
    void setParamUseScanBarycenter(bool b);
    void setParamMinimumTimeInterval(double d);
    void setParamMinimumTravelDistance(double d);
    void setParamMinimumTravelHeading(double d);
    void setParamScanBufferSize(int i);
    void setParamScanBufferMaximumScanDistance(double d);
    void setParamLinkMatchMinimumResponseFine(double d);
    void setParamLinkScanMaximumDistance(double d);
    void setParamLoopSearchMaximumDistance(double d);
    void setParamDoLoopClosing(bool b);
    void setParamLoopMatchMinimumChainSize(int i);
    void setParamLoopMatchMaximumVarianceCoarse(double d);
    void setParamLoopMatchMinimumResponseCoarse(double d);
    void setParamLoopMatchMinimumResponseFine(double d);

    // Correlation Parameters - Correlation Parameters
    void setParamCorrelationSearchSpaceDimension(double d);
    void setParamCorrelationSearchSpaceResolution(double d);
    void setParamCorrelationSearchSpaceSmearDeviation(double d);

    // Correlation Parameters - Loop Closure Parameters
    void setParamLoopSearchSpaceDimension(double d);
    void setParamLoopSearchSpaceResolution(double d);
    void setParamLoopSearchSpaceSmearDeviation(double d);

    // Scan Matcher Parameters
    void setParamDistanceVariancePenalty(double d);
    void setParamAngleVariancePenalty(double d);
    void setParamFineSearchAngleOffset(double d);
    void setParamCoarseSearchAngleOffset(double d);
    void setParamCoarseAngleResolution(double d);
    void setParamMinimumAnglePenalty(double d);
    void setParamMinimumDistancePenalty(double d);
    void setParamUseResponseExpansion(bool b);
  };
}  // namespace karto

#endif  // OPEN_KARTO_MAPPER_H

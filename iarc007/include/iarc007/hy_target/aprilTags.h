#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include "opencv2/opencv.hpp"
#include <ostream>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <errno.h>


namespace DualCoding {
        typedef unsigned char uchar;
        template<typename T> class Sketch;
}

/**********/

namespace AprilTags {

/**********************/
class Gaussian {

public:
  static bool warned;

  //! Returns a Gaussian filter of size n.
  /*! @param sigma standard deviation of the Gaussian
   *  @param n length of the Gaussian (must be odd)
   */
  static std::vector<float> makeGaussianFilter(float sigma, int n);

  //! Convolve the input 'a' (which begins at offset aoff and is alen elements in length) with the filter 'f'.
  /*! The result is deposited in 'r' at offset 'roff'. f.size() should be odd.
   *  The output is shifted by -f.size()/2, so that there is no net time delay.
   *  @param a input vector of pixels
   *  @param aoff
   *  @param alen
   *  @param f
   *  @param r the resultant array of pixels
   *  @param roff
   */
  static void convolveSymmetricCentered(const std::vector<float>& a, unsigned int aoff, unsigned int alen,
                                        const std::vector<float>& f, std::vector<float>& r, unsigned int roff);

};


/*************************************/
std::ostream& operator<<(std::ostream &os, const std::pair<float,float> &pt);

//! Miscellaneous math utilities and fast exp functions.
class MathUtil {
public:

        //! Returns the square of a value.
        static inline float square(float x) { return x*x; }

        static inline float distance2D(const std::pair<float,float> &p0, const std::pair<float,float> &p1) {
                float dx = p0.first - p1.first;
                float dy = p0.second - p1.second;
                return std::sqrt(dx*dx + dy*dy);
        }

        //! Returns a result in [-Pi, Pi]
        static inline float mod2pi(float vin) {
                const float twopi = 2 * (float)M_PI;
                const float twopi_inv = 1.f / (2.f * (float)M_PI);
                float absv = std::abs(vin);
                float q = absv*twopi_inv + 0.5f;
                int qi = (int) q;
                float r = absv - qi*twopi;
                return (vin<0) ? -r : r;
        }

        //! Returns a value of v wrapped such that ref and v differ by no more than +/- Pi
        static inline float mod2pi(float ref, float v) { return ref + mod2pi(v-ref); }

// lousy approximation of arctan function, but good enough for our purposes (about 4 degrees)
  static inline double fast_atan2(double y, double x) {
    double coeff_1 = M_PI/4;
    double coeff_2 = 3*coeff_1;
    double abs_y = fabs(y)+1e-10;      // kludge to prevent 0/0 condition

    double angle;

    if (x >= 0) {
      double r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
    } else {
      double r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
    }

    if (y < 0)
      return -angle;     // negate if in quad III or IV
    else
      return angle;
  }

};


/*******************************************/
class Segment {
public:
  Segment();

  static int const minimumSegmentSize = 4; //!< Minimum number of pixels in a segment before we'll fit a line to it.
  static float const minimumLineLength; //!< In pixels. Calculated based on minimum plausible decoding size for Tag9 family.

  float getX0() const { return x0; }
  void setX0(float newValue) { x0 = newValue; }

  float getY0() const { return y0; }
  void setY0(float newValue) { y0 = newValue; }

  float getX1() const { return x1; }
  void setX1(float newValue) { x1 = newValue; }

  float getY1() const { return y1; }
  void setY1(float newValue) { y1 = newValue; }

  float getTheta() const { return theta; }
  void setTheta(float newValue) { theta = newValue; }

  float getLength() const { return length; }
  void setLength(float newValue) { length = newValue; }

  //! Returns the length of the Segment.
  float segmentLength();

  //! Print endpoint coordinates of this segment.
  void printSegment();

  //! ID of Segment.
  int getId() const { return segmentId; }

  std::vector<Segment*> children;

private:
  float x0, y0, x1, y1;
  float theta; // gradient direction (points towards white)
  float length; // length of line segment in pixels
  int segmentId;
  static int idCounter;
};


/*****************************************************/
//! A lookup table in 2D for implementing nearest neighbor.
template <class T>
class Gridder {
  private:
        Gridder(const Gridder&); //!< don't call
        Gridder& operator=(const Gridder&); //!< don't call

  struct Cell {
    T* object;
    Cell *next;

    Cell() : object(NULL), next(NULL) {}

    Cell(const Cell& c) : object(c.object), next(c.next) {}

    // Destructor
    ~Cell() {
      delete next;
    }

    Cell& operator=(const Cell &other) {
      if (this == &other)
        return *this;

      object = other.object;
      next = other.next;
      return *this;
    }
  };

  //! Initializes Gridder constructor
  void gridderInit(float x0Arg, float y0Arg, float x1Arg, float y1Arg, float ppCell) {
    width = (int) ((x1Arg - x0Arg)/ppCell + 1);
    height = (int) ((y1Arg - y0Arg)/ppCell + 1);

    x1 = x0Arg + ppCell*width;
    y1 = y0Arg + ppCell*height;
    cells = std::vector< std::vector<Cell*> >(height, std::vector<Cell*>(width,(Cell*)NULL));
  }

  float x0,y0,x1,y1;
  int width, height;
  float pixelsPerCell; //pixels per cell
  std::vector< std::vector<Cell*> > cells;

public:
  Gridder(float x0Arg, float y0Arg, float x1Arg, float y1Arg, float ppCell)
    : x0(x0Arg), y0(y0Arg), x1(), y1(), width(), height(), pixelsPerCell(ppCell),
      cells() { gridderInit(x0Arg, y0Arg, x1Arg, y1Arg, ppCell); }

  // Destructor
  ~Gridder() {
    for (unsigned int i = 0; i < cells.size(); i++) {
      for (unsigned int j = 0; j < cells[i].size(); j++)
        delete cells[i][j];
    }
  }

  void add(float x, float y, T* object) {
    int ix = (int) ((x - x0)/pixelsPerCell);
    int iy = (int) ((y - y0)/pixelsPerCell);

    if (ix>=0 && iy>=0 && ix<width && iy<height) {
      Cell *c = new Cell;
      c->object = object;
      c->next = cells[iy][ix];
      cells[iy][ix] = c;
      // cout << "Gridder placed seg " << o->getId() << " at (" << ix << "," << iy << ")" << endl;
    }
  }

  // iterator begin();
  // iterator end();

  //! Iterator for Segment class.
  class Iterator {
  public:
    Iterator(Gridder* grid, float x, float y, float range)
      : outer(grid), ix0(), ix1(), iy0(), iy1(), ix(), iy(), c(NULL) { iteratorInit(x,y,range); }

          Iterator(const Iterator& it)
          : outer(it.outer), ix0(it.ix0), ix1(it.ix1), iy0(it.iy0), iy1(it.iy1), ix(it.ix), iy(it.iy), c(it.c) {}

          Iterator& operator=(const Iterator& it) {
                  outer = it.outer;
                  ix0 = it.ix0;
                  ix1 = it.ix1;
                  iy0 = it.iy0;
                  iy1 = it.iy1;
                  ix = it.ix;
                  iy = it.iy;
                  c = it.c;
          }

    bool hasNext() {
      if (c == NULL)
        findNext();
      return (c != NULL);
    }

    T& next() {
      T* thisObj = c->object;
      findNext();
      return *thisObj; // return Segment
    }

  private:
    void findNext() {
      if (c != NULL)
        c = c->next;
      if (c != NULL)
        return;

      ix++;
      while (true) {
        if (ix > ix1) {
          iy++;
          ix = ix0;
        }
        if (iy > iy1)
          break;

        c = outer->cells[iy][ix];

        if (c != NULL)
          break;
        ix++;
      }
    }

    //! Initializes Iterator constructor
    void iteratorInit(float x, float y, float range) {
      ix0 = (int) ((x - range - outer->x0)/outer->pixelsPerCell);
      iy0 = (int) ((y - range - outer->y0)/outer->pixelsPerCell);

      ix1 = (int) ((x + range - outer->x0)/outer->pixelsPerCell);
      iy1 = (int) ((y + range - outer->y0)/outer->pixelsPerCell);

      ix0 = std::max(0, ix0);
      ix0 = std::min(outer->width-1, ix0);

      ix1 = std::max(0, ix1);
      ix1 = std::min(outer->width-1, ix1);

      iy0 = std::max(0, iy0);
      iy0 = std::min(outer->height-1, iy0);

      iy1 = std::max(0, iy1);
      iy1 = std::min(outer->height-1, iy1);

      ix = ix0;
      iy = iy0;

      c = outer->cells[iy][ix];
    }

    Gridder* outer;
    int ix0, ix1, iy0, iy1;
    int ix, iy;
    Cell *c;
  };

  typedef Iterator iterator;
  iterator find(float x, float y, float range) { return Iterator(this,x,y,range); }
};



/**************************************************/
//! Implementation of disjoint set data structure using the union-find algorithm
class UnionFindSimple {
  //! Identifies parent ids and sizes.
  struct Data {
    int id;
    int size;
  };

public:
  explicit UnionFindSimple(int maxId) : data(maxId) {
    init();
  };

  int getSetSize(int thisId) { return data[getRepresentative(thisId)].size; }

  int getRepresentative(int thisId);

  //! Returns the id of the merged node.
  /*  @param aId
   *  @param bId
   */
  int connectNodes(int aId, int bId);

  void printDataVector() const;

private:
  void init();

  std::vector<Data> data;
};



//! Represents a triple holding an x value, y value, and weight value.
struct XYWeight {
  float x;
  float y;
  float weight;

  XYWeight(float xval, float yval, float weightval) :
    x(xval), y(yval), weight(weightval) {}

};


/****************************************/
//! A 2D line
class GLine2D {
public:

  //! Create a new line.
  GLine2D();

  //! Create a new line.
  /*  @param slope the slope
   *  @param b the y intercept
   */
  GLine2D(float slope, float b);

  //! Create a new line.
  /*  @param dx A change in X corresponding to dy
   *  @param dy A change in Y corresponding to dx
   *  @param p A point that the line passes through
   */
  GLine2D(float dX, float dY, const std::pair<float,float>& pt);

  //! Create a new line through two points.
  /*  @param p1 the first point
   *  @param p2 the second point
   */
  GLine2D(const std::pair<float,float>& p1, const std::pair<float,float>& p2);

  //! Get the coordinate of a point (on this line), with zero corresponding to the point
  //! on the that is perpendicular toa line passing through the origin and the line.
  /*  This allows easy computation if one point is between two other points on the line:
   *  compute the line coordinate of all three points and test if a<=b<=c. This is
   *  implemented by computing the dot product of the vector 'p' with the
   *  line's direct unit vector.
   */
  float getLineCoordinate(const std::pair<float,float>& p);

  //! The inverse of getLineCoordinate.
  std::pair<float,float> getPointOfCoordinate(float coord);

  //!Compute the point where two lines intersect, or (-1,0) if the lines are parallel.
  std::pair<float,float> intersectionWith(const GLine2D& line) const;

  static GLine2D lsqFitXYW(const std::vector<XYWeight>& xyweights);

  inline float getDx() const { return dx; }
  inline float getDy() const { return dy; }
  inline float getFirst() const { return p.first; }
  inline float getSecond() const { return p.second; }

protected:
  void normalizeSlope();
  void normalizeP();

private:
  float dx, dy;
  std::pair<float,float> p;  //!< A point the line passes through; when normalized, it is the point closest to the origin (hence perpendicular to the line)
  bool didNormalizeSlope;
  bool didNormalizeP;
};


/******************************************/
//! Represent an image as a vector of floats in [0,1]
class FloatImage {
private:
  int width;
  int height;
  std::vector<float> pixels;

public:

  //! Default constructor
  FloatImage();

  //! Construct an empty image
  FloatImage(int widthArg, int heightArg);

  //! Constructor that copies pixels from an array
  FloatImage(int widthArg, int heightArg, const std::vector<float>& pArg);

  FloatImage& operator=(const FloatImage& other);

  float get(int x, int y) const { return pixels[y*width + x]; }
  void set(int x, int y, float v) { pixels[y*width + x] = v; }

  int getWidth() const { return width; }
  int getHeight() const { return height; }
  int getNumFloatImagePixels() const { return width*height; }
  const std::vector<float>& getFloatImagePixels() const { return pixels; }

  //! TODO: Fix decimateAvg function. DO NOT USE!
  void decimateAvg();

  //! Rescale all values so that they are between [0,1]
  void normalize();

  void filterFactoredCentered(const std::vector<float>& fhoriz, const std::vector<float>& fvert);

  template<typename T>
  void copyToSketch(DualCoding::Sketch<T>& sketch) {
    for (int i = 0; i < getNumFloatImagePixels(); i++)
      sketch[i] = (T)(255.0f * getFloatImagePixels()[i]);
  }

  void printMinMax() const;
};


/****************************************/
//! A 2D line with endpoints.
class GLineSegment2D {
public:
  GLineSegment2D(const std::pair<float,float> &p0Arg, const std::pair<float,float> &p1Arg);
  static GLineSegment2D lsqFitXYW(const std::vector<XYWeight>& xyweight);
  std::pair<float,float> getP0() const { return p0; }
  std::pair<float,float> getP1() const { return p1; }

private:
  GLine2D line;
  std::pair<float,float> p0;
  std::pair<float,float> p1;
  int weight;
};


/********************************/
using std::min;
using std::max;

//! Represents an edge between adjacent pixels in the image.
/*! The edge is encoded by the indices of the two pixels. Edge cost
 *  is proportional to the difference in local orientations.
 */
class Edge {
public:
  static float const minMag;   //!< minimum intensity gradient for an edge to be recognized
  static float const maxEdgeCost;   //!< 30 degrees = maximum acceptable difference in local orientations
  static int const WEIGHT_SCALE; // was 10000
  static float const thetaThresh; //!< theta threshold for merging edges
  static float const magThresh; //!< magnitude threshold for merging edges

  int pixelIdxA;
  int pixelIdxB;
  int cost;

  //! Constructor
  Edge() : pixelIdxA(), pixelIdxB(), cost() {}

  //! Compare edges based on cost
  inline bool operator< (const Edge &other) const { return (cost < other.cost); }

  //! Cost of an edge between two adjacent pixels; -1 if no edge here
  /*! An edge exists between adjacent pixels if the magnitude of the
    intensity gradient at both pixels is above threshold.  The edge
    cost is proportional to the difference in the local orientation at
    the two pixels.  Lower cost is better.  A cost of -1 means there
    is no edge here (intensity gradien fell below threshold).
   */
  static int edgeCost(float  theta0, float theta1, float mag1);

  //! Calculates and inserts up to four edges into 'edges', a vector of Edges.
  static void calcEdges(float theta0, int x, int y,
                        const FloatImage& theta, const FloatImage& mag,
                        std::vector<Edge> &edges, size_t &nEdges);

  //! Process edges in order of increasing cost, merging clusters if we can do so without exceeding the thetaThresh.
  static void mergeEdges(std::vector<Edge> &edges, UnionFindSimple &uf, float tmin[], float tmax[], float mmin[], float mmax[]);

};


/**************************************************/
#define INTERPOLATE
// use stable version of homography recover (opencv, includes refinement step)
#define STABLE_H

//! Compute 3x3 homography using Direct Linear Transform
/*
 *
 *  DEPRECATED - DEPRECATED - DEPRECATED - DEPRECATED
 *
 *  use TagDetection::getRelativeTransform() instead
 *
 *
 *  y = Hx (y = image coordinates in homogeneous coordinates, H = 3x3
 *  homography matrix, x = homogeneous 2D world coordinates)
 *
 *  For each point correspondence, constrain y x Hx = 0 (where x is
 *  cross product). This means that they have the same direction, and
 *  ignores scale factor.
 *
 *  We rewrite this as Ah = 0, where h is the 9x1 column vector of the
 *  elements of H. Each point correspondence gives us 3 equations of
 *  rank 2. The solution h is the minimum right eigenvector of A,
 *  which we can obtain via SVD, USV' = A. h is the right-most column
 *  of V'.
 *
 *  We will actually maintain A'A internally, which is 9x9 regardless
 *  of how many correspondences we're given, and has the same
 *  eigenvectors as A.
 */
class Homography33 {
public:
  //! Constructor
  Homography33(const std::pair<float,float> &opticalCenter);

#ifdef STABLE_H
  void setCorrespondences(const std::vector< std::pair<float,float> > &srcPts,
                          const std::vector< std::pair<float,float> > &dstPts);
#else
  void addCorrespondence(float worldx, float worldy, float imagex, float imagey);
#endif

  //! Note that the returned H matrix does not reflect cxy.
  Eigen::Matrix3d& getH();

  const std::pair<float,float> getCXY() const { return cxy; }

  void compute();

  std::pair<float,float> project(float worldx, float worldy);

private:
  std::pair<float,float> cxy;
  Eigen::Matrix<double,9,9> fA;
  Eigen::Matrix3d H;
  bool valid;
#ifdef STABLE_H
  std::vector< std::pair<float,float> > srcPts, dstPts;
#endif
};



/***************************************************/
using std::min;
using std::max;

//! Represents four segments that form a loop, and might be a tag.
class Quad {
public:
  static const int minimumEdgeLength = 6; //!< Minimum size of a tag (in pixels) as measured along edges and diagonals
  static float const maxQuadAspectRatio; //!< Early pruning of quads with insane ratios.

  //! Constructor
  /*! (x,y) are the optical center of the camera, which is
   *   needed to correctly compute the homography. */
  Quad(const std::vector< std::pair<float,float> >& p, const std::pair<float,float>& opticalCenter);

  //! Interpolate given that the lower left corner of the lower left cell is at (-1,-1) and the upper right corner of the upper right cell is at (1,1).
  std::pair<float,float> interpolate(float x, float y);

  //! Same as interpolate, except that the coordinates are interpreted between 0 and 1, instead of -1 and 1.
  std::pair<float,float> interpolate01(float x, float y);

  //! Points for the quad (in pixel coordinates), in counter clockwise order. These points are the intersections of segments.
  std::vector< std::pair<float,float> > quadPoints;

  //! Segments composing this quad
  std::vector<Segment*> segments;

  //! Total length (in pixels) of the actual perimeter observed for the quad.
  /*! This is in contrast to the geometric perimeter, some of which
   *  may not have been directly observed but rather inferred by
   *  intersecting segments. Quads with more observed perimeter are
   *  preferred over others. */
  float observedPerimeter;

  //! Given that the whole quad spans from (0,0) to (1,1) in "quad space", compute the pixel coordinates for a given point within that quad.
  /*!  Note that for most of the Quad's existence, we will not know the correct orientation of the tag. */
  Homography33 homography;

  //! Searches through a vector of Segments to form Quads.
  /*  @param quads any discovered quads will be added to this list
   *  @param path  the segments currently part of the search
   *  @param parent the first segment in the quad
   *  @param depth how deep in the search are we?
   */
  static void search(const FloatImage& fImage, std::vector<Segment*>& path,
                     Segment& parent, int depth, std::vector<Quad>& quads,
                     const std::pair<float,float>& opticalCenter);

#ifdef INTERPOLATE
 private:
  Eigen::Vector2f p0, p3, p01, p32;
#endif

};//quads




 std::vector<Quad> extractTags(cv::Mat& image,int minimumSegmentSize, float minimumLineLength);


} // namespace

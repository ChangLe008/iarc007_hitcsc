#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <errno.h>
#include <limits>
#include <map>
#include <iostream>
#include <iterator>
#include "opencv2/opencv.hpp"
#include <ostream>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>


#include "iarc007/hy_target/aprilTags.h"

#include <sys/time.h>



//#define DEBUG_APRIL
#define my_DEBUG_APRIL

#ifdef DEBUG_APRIL
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

using namespace std;
using namespace cv;


namespace AprilTags {


/*********************************/
bool Gaussian::warned = false;

std::vector<float> Gaussian::makeGaussianFilter(float sigma, int n) {
  std::vector<float> f(n,0.0f);

  if (sigma == 0) {
    for (int i = 0; i < n; i++)
      f[i] = 0;
    f[n/2] = 1;
    return f;
  }

  double const inv_variance = 1./(2*sigma*sigma);
  float sum = 0;
  for (int i = 0; i < n; i++) {
    int j = i - n/2;
    f[i] = (float)std::exp(-j*j * inv_variance);
    sum += f[i];
  }

  // normalize the gaussian
  for (int i = 0; i < n; i++)
    f[i] /= sum;

  return f;
}

void Gaussian::convolveSymmetricCentered(const std::vector<float>& a, unsigned int aoff, unsigned int alen,
                                        const std::vector<float>& f, std::vector<float>& r, unsigned int roff) {
  if ((f.size()&1)== 0 && !warned) {
    std::cout<<"convolveSymmetricCentered Warning: filter is not odd length\n";
    warned = true;
  }

  for (size_t i = f.size()/2; i < f.size(); i++) {
    double acc = 0;
    for (size_t j = 0; j < f.size(); j++) {
      if ((aoff + i) < j || (aoff + i) >= (alen + j))
        acc += a[aoff] * f[j];
      else
        acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }

  for (size_t i = f.size(); i < alen; i++) {
    double acc = 0;
    for (unsigned int j = 0; j < f.size(); j++) {
      acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }

  for (size_t i = alen; i < alen + f.size()/2; i++) {
    double acc = 0;
    for (size_t j = 0; j < f.size(); j++) {
      if ((aoff + i) >= (alen + j) || (aoff + i) < j)
        acc += a[aoff + alen - 1] * f[j];
      else
        acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }
}


/************************************/
std::ostream& operator<<(std::ostream &os, const std::pair<float,float> &pt) {
  os << pt.first << "," << pt.second;
  return os;
}


const float Segment::minimumLineLength = 4;

Segment::Segment()
  : children(), x0(0), y0(0), x1(0), y1(0), theta(0), length(0), segmentId(++idCounter) {}

float Segment::segmentLength() {
  return std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void Segment::printSegment() {
  std::cout <<"("<< x0 <<","<< y0 <<"), "<<"("<< x1 <<","<< y1 <<")" << std::endl;
}

int Segment::idCounter = 0;



int UnionFindSimple::getRepresentative(int thisId) {
  // terminal case: a node is its own parent
  if (data[thisId].id == thisId)
    return thisId;

  // otherwise, recurse...
  int root = getRepresentative(data[thisId].id);

  // short circuit the path
  data[thisId].id = root;

  return root;
}

void UnionFindSimple::printDataVector() const {
  for (unsigned int i = 0; i < data.size(); i++)
    std::cout << "data[" << i << "]: " << " id:" << data[i].id << " size:" << data[i].size << std::endl;
}

int UnionFindSimple::connectNodes(int aId, int bId) {
  int aRoot = getRepresentative(aId);
  int bRoot = getRepresentative(bId);

  if (aRoot == bRoot)
    return aRoot;

  int asz = data[aRoot].size;
  int bsz = data[bRoot].size;

  if (asz > bsz) {
    data[bRoot].id = aRoot;
    data[aRoot].size += bsz;
    return aRoot;
  } else {
    data[aRoot].id = bRoot;
    data[bRoot].size += asz;
    return bRoot;
  }
}

void UnionFindSimple::init() {
  for (unsigned int i = 0; i < data.size(); i++) {
    // everyone is their own cluster of size 1
    data[i].id = i;
    data[i].size = 1;
  }
}


/*********************************************/
GLine2D::GLine2D()
  : dx(0), dy(0), p(0,0), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(float slope, float b)
  : dx(1), dy(slope), p(0,b), didNormalizeSlope(false), didNormalizeP(false){}

GLine2D::GLine2D(float dX, float dY, const std::pair<float,float>& pt)
  : dx(dX), dy(dY), p(pt), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(const std::pair<float,float>& p1, const std::pair<float,float>& p2)
  : dx(p2.first - p1.first), dy(p2.second - p1.second), p(p1), didNormalizeSlope(false), didNormalizeP(false) {}

float GLine2D::getLineCoordinate(const std::pair<float,float>& pt) {
  normalizeSlope();
  return pt.first*dx + pt.second*dy;
}

std::pair<float,float> GLine2D::getPointOfCoordinate(float coord) {
  normalizeP();
  return std::pair<float,float>(p.first + coord*dx, p.second + coord*dy);
}

std::pair<float,float> GLine2D::intersectionWith(const GLine2D& line) const {
  float m00 = dx;
  float m01 = -line.getDx();
  float m10 = dy;
  float m11 = -line.getDy();

  // determinant of 'm'
  float det = m00*m11 - m01*m10;

  // parallel lines? if so, return (-1,0).
  if (fabs(det) < 1e-10)
    return std::pair<float,float>(-1,0);

  // inverse of 'm'
  float i00 = m11/det;
  // float i11 = m00/det;
  float i01 = -m01/det;
  // float i10 = -m10/det;

  float b00 = line.getFirst() - p.first;
  float b10 = line.getSecond() - p.second;

  float x00 = i00*b00 + i01*b10;

  return std::pair<float,float>(dx*x00+p.first, dy*x00+p.second);
}

GLine2D GLine2D::lsqFitXYW(const std::vector<XYWeight>& xyweights) {
  float Cxx=0, Cyy=0, Cxy=0, Ex=0, Ey=0, mXX=0, mYY=0, mXY=0, mX=0, mY=0;
  float n=0;

  int idx = 0;
  for (unsigned int i = 0; i < xyweights.size(); i++) {
    float x = xyweights[i].x;
    float y = xyweights[i].y;
    float alpha = xyweights[i].weight;

    mY  += y*alpha;
    mX  += x*alpha;
    mYY += y*y*alpha;
    mXX += x*x*alpha;
    mXY += x*y*alpha;
    n   += alpha;

    idx++;
  }

  Ex  = mX/n;
  Ey  = mY/n;
  Cxx = mXX/n - MathUtil::square(mX/n);
  Cyy = mYY/n - MathUtil::square(mY/n);
  Cxy = mXY/n - (mX/n)*(mY/n);

  // find dominant direction via SVD
  float phi = 0.5f*std::atan2(-2*Cxy,(Cyy-Cxx));
  // float rho = Ex*cos(phi) + Ey*sin(phi); //why is this needed if he never uses it?
  std::pair<float,float> pts = std::pair<float,float>(Ex,Ey);

  // compute line parameters
        return GLine2D(-std::sin(phi), std::cos(phi), pts);
}

void GLine2D::normalizeSlope() {
  if ( !didNormalizeSlope ) {
    float mag = std::sqrt(dx*dx+dy*dy);
    dx /= mag;
    dy /= mag;
    didNormalizeSlope=true;
  }
}

void GLine2D::normalizeP() {
  if ( !didNormalizeP ) {
    normalizeSlope();
    // we already have a point (P) on the line, and we know the line vector U
    // and its perpendicular vector V: so, P'=P.*V *V
    float dotprod = -dy*p.first + dx*p.second;
    p = std::pair<float,float>(-dy*dotprod, dx*dotprod);
    didNormalizeP = true;
  }
}


/******************************************/
FloatImage::FloatImage() : width(0), height(0), pixels() {}

FloatImage::FloatImage(int widthArg, int heightArg)
  : width(widthArg), height(heightArg), pixels(widthArg*heightArg) {}

FloatImage::FloatImage(int widthArg, int heightArg, const std::vector<float>& pArg)
  : width(widthArg), height(heightArg), pixels(pArg) {}

FloatImage& FloatImage::operator=(const FloatImage& other) {
  width = other.width;
  height = other.height;
  if (pixels.size() != other.pixels.size())
    pixels.resize(other.pixels.size());
  pixels = other.pixels;
  return *this;
}

void FloatImage::decimateAvg() {
  int nWidth = width/2;
  int nHeight = height/2;

  for (int y = 0; y < nHeight; y++)
    for (int x = 0; x < nWidth; x++)
      pixels[y*nWidth+x] = pixels[(2*y)*width + (2*x)];

  width = nWidth;
  height = nHeight;
  pixels.resize(nWidth*nHeight);
}

void FloatImage::normalize() {
  const float maxVal = *max_element(pixels.begin(),pixels.end());
  const float minVal = *min_element(pixels.begin(),pixels.end());
  const float range = maxVal - minVal;
  const float rescale = 1 / range;
  for ( unsigned int i = 0; i < pixels.size(); i++ )
    pixels[i] = (pixels[i]-minVal) * rescale;
}

void FloatImage::filterFactoredCentered(const std::vector<float>& fhoriz, const std::vector<float>& fvert) {
  // do horizontal
  std::vector<float> r(pixels);

  for (int y = 0; y < height; y++) {
    Gaussian::convolveSymmetricCentered(pixels, y*width, width, fhoriz, r, y*width);
  }

  // do vertical
  std::vector<float> tmp(height); // column before convolution
  std::vector<float> tmp2(height); // column after convolution

  for (int x = 0; x < width; x++) {

    // copy the column out for locality
    for (int y = 0; y < height; y++)
      tmp[y] = r[y*width + x];

    Gaussian::convolveSymmetricCentered(tmp, 0, height, fvert, tmp2, 0);

    for (int y = 0; y < height; y++)
      pixels[y*width + x] = tmp2[y];
  }
}

void FloatImage::printMinMax() const {
  std::cout << "Min: " << *min_element(pixels.begin(),pixels.end()) << ", Max: " << *max_element(pixels.begin(),pixels.end()) << std::endl;
  //for (int i = 0; i < getNumFloatImagePixels(); i++)
  //  std::cout << "Index[" << i << "]: " << this->normalize().getFloatImagePixels()[i] << endl;
}



/********************************************/
GLineSegment2D::GLineSegment2D(const std::pair<float,float>& p0Arg, const std::pair<float,float>& p1Arg)
: line(p0Arg,p1Arg), p0(p0Arg), p1(p1Arg), weight() {}

GLineSegment2D GLineSegment2D::lsqFitXYW(const std::vector<XYWeight>& xyweight) {
        GLine2D gline = GLine2D::lsqFitXYW(xyweight);
        float maxcoord = -std::numeric_limits<float>::infinity();
        float mincoord = std::numeric_limits<float>::infinity();;

        for (unsigned int i = 0; i < xyweight.size(); i++) {
                std::pair<float,float> p(xyweight[i].x, xyweight[i].y);
                float coord = gline.getLineCoordinate(p);
                maxcoord = std::max(maxcoord, coord);
                mincoord = std::min(mincoord, coord);
        }

        std::pair<float,float> minValue = gline.getPointOfCoordinate(mincoord);
        std::pair<float,float> maxValue = gline.getPointOfCoordinate(maxcoord);
        return GLineSegment2D(minValue,maxValue);
}



/*****************************************/
float const Edge::minMag = 0.004f;
float const Edge::maxEdgeCost = 30.f * float(M_PI) / 180.f;
int const Edge::WEIGHT_SCALE = 100;
float const Edge::thetaThresh = 100;
float const Edge::magThresh = 1200;

int Edge::edgeCost(float  theta0, float theta1, float mag1) {
  if (mag1 < minMag)  // mag0 was checked by the main routine so no need to recheck here
    return -1;

  const float thetaErr = std::abs(MathUtil::mod2pi(theta1 - theta0));
  if (thetaErr > maxEdgeCost)
    return -1;

  const float normErr = thetaErr / maxEdgeCost;
  return (int) (normErr*WEIGHT_SCALE);
}

void Edge::calcEdges(float theta0, int x, int y,
                     const FloatImage& theta, const FloatImage& mag,
                     std::vector<Edge> &edges, size_t &nEdges) {
  int width = theta.getWidth();
  int thisPixel = y*width+x;

  // horizontal edge
  int cost1 = edgeCost(theta0, theta.get(x+1,y), mag.get(x+1,y));
  if (cost1 >= 0) {
    edges[nEdges].cost = cost1;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = y*width+x+1;
    ++nEdges;
  }

  // vertical edge
  int cost2 = edgeCost(theta0, theta.get(x, y+1), mag.get(x,y+1));
  if (cost2 >= 0) {
    edges[nEdges].cost = cost2;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x;
    ++nEdges;
  }

  // downward diagonal edge
  int cost3 = edgeCost(theta0, theta.get(x+1, y+1), mag.get(x+1,y+1));
  if (cost3 >= 0) {
    edges[nEdges].cost = cost3;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x+1;
    ++nEdges;
  }

  // updward diagonal edge
  int cost4 = (x == 0) ? -1 : edgeCost(theta0, theta.get(x-1, y+1), mag.get(x-1,y+1));
  if (cost4 >= 0) {
    edges[nEdges].cost = cost4;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x-1;
    ++nEdges;
  }
}

void Edge::mergeEdges(std::vector<Edge> &edges, UnionFindSimple &uf,
                      float tmin[], float tmax[], float mmin[], float mmax[]) {
  for (size_t i = 0; i < edges.size(); i++) {
    int ida = edges[i].pixelIdxA;
    int idb = edges[i].pixelIdxB;

    ida = uf.getRepresentative(ida);
    idb = uf.getRepresentative(idb);

    if (ida == idb)
      continue;

    int sza = uf.getSetSize(ida);
    int szb = uf.getSetSize(idb);

    float tmina = tmin[ida], tmaxa = tmax[ida];
    float tminb = tmin[idb], tmaxb = tmax[idb];

    float costa = (tmaxa-tmina);
    float costb = (tmaxb-tminb);

    // bshift will be a multiple of 2pi that aligns the spans of 'b' with 'a'
    // so that we can properly take the union of them.
    float bshift = MathUtil::mod2pi((tmina+tmaxa)/2, (tminb+tmaxb)/2) - (tminb+tmaxb)/2;

    float tminab = min(tmina, tminb + bshift);
    float tmaxab = max(tmaxa, tmaxb + bshift);

    if (tmaxab-tminab > 2*(float)M_PI) // corner case that's probably not too useful to handle correctly, oh well.
      tmaxab = tminab + 2*(float)M_PI;

    float mminab = min(mmin[ida], mmin[idb]);
    float mmaxab = max(mmax[ida], mmax[idb]);

    // merge these two clusters?
    float costab = (tmaxab - tminab);
    if (costab <= (min(costa, costb) + Edge::thetaThresh/(sza+szb)) &&
        (mmaxab-mminab) <= min(mmax[ida]-mmin[ida], mmax[idb]-mmin[idb]) + Edge::magThresh/(sza+szb)) {

      int idab = uf.connectNodes(ida, idb);

      tmin[idab] = tminab;
      tmax[idab] = tmaxab;

      mmin[idab] = mminab;
      mmax[idab] = mmaxab;
    }
  }
}



/********************************************************/
Homography33::Homography33(const std::pair<float,float> &opticalCenter) : cxy(opticalCenter), fA(), H(), valid(false) {
  fA.setZero();
  H.setZero();
}

Eigen::Matrix3d& Homography33::getH() {
  compute();
  return H;
}

#ifdef STABLE_H
void Homography33::setCorrespondences(const std::vector< std::pair<float,float> > &sPts,
                                      const std::vector< std::pair<float,float> > &dPts) {
  valid = false;
  srcPts = sPts;
  dstPts = dPts;
}
#else
void Homography33::addCorrespondence(float worldx, float worldy, float imagex, float imagey) {
  valid = false;
  imagex -= cxy.first;
  imagey -= cxy.second;

  /* Here are the rows of matrix A.  We will compute A'*A
   * A[3*i+0][3] = -worldxyh[i][0]*imagexy[i][2];
   * A[3*i+0][4] = -worldxyh[i][1]*imagexy[i][2];
   * A[3*i+0][5] = -worldxyh[i][2]*imagexy[i][2];
   * A[3*i+0][6] =  worldxyh[i][0]*imagexy[i][1];
   * A[3*i+0][7] =  worldxyh[i][1]*imagexy[i][1];
   * A[3*i+0][8] =  worldxyh[i][2]*imagexy[i][1];
   *
   * A[3*i+1][0] =  worldxyh[i][0]*imagexy[i][2];
   * A[3*i+1][1] =  worldxyh[i][1]*imagexy[i][2];
   * A[3*i+1][2] =  worldxyh[i][2]*imagexy[i][2];
   * A[3*i+1][6] = -worldxyh[i][0]*imagexy[i][0];
   * A[3*i+1][7] = -worldxyh[i][1]*imagexy[i][0];
   * A[3*i+1][8] = -worldxyh[i][2]*imagexy[i][0];
   *
   * A[3*i+2][0] = -worldxyh[i][0]*imagexy[i][1];
   * A[3*i+2][1] = -worldxyh[i][1]*imagexy[i][1];
   * A[3*i+2][2] = -worldxyh[i][2]*imagexy[i][1];
   * A[3*i+2][3] =  worldxyh[i][0]*imagexy[i][0];
   * A[3*i+2][4] =  worldxyh[i][1]*imagexy[i][0];
   * A[3*i+2][5] =  worldxyh[i][2]*imagexy[i][0];
   */

  // only update upper-right. A'A is symmetric, we'll finish the lower left later.
  float a03 = -worldx;
  float a04 = -worldy;
  float a05 = -1;
  float a06 = worldx*imagey;
  float a07 = worldy*imagey;
  float a08 = imagey;

  fA(3, 3) += a03*a03;
  fA(3, 4) += a03*a04;
  fA(3, 5) += a03*a05;
  fA(3, 6) += a03*a06;
  fA(3, 7) += a03*a07;
  fA(3, 8) += a03*a08;

  fA(4, 4) += a04*a04;
  fA(4, 5) += a04*a05;
  fA(4, 6) += a04*a06;
  fA(4, 7) += a04*a07;
  fA(4, 8) += a04*a08;

  fA(5, 5) += a05*a05;
  fA(5, 6) += a05*a06;
  fA(5, 7) += a05*a07;
  fA(5, 8) += a05*a08;

  fA(6, 6) += a06*a06;
  fA(6, 7) += a06*a07;
  fA(6, 8) += a06*a08;

  fA(7, 7) += a07*a07;
  fA(7, 8) += a07*a08;

  fA(8, 8) += a08*a08;

  float a10 = worldx;
  float a11 = worldy;
  float a12 = 1;
  float a16 = -worldx*imagex;
  float a17 = -worldy*imagex;
  float a18 = -imagex;

  fA(0, 0) += a10*a10;
  fA(0, 1) += a10*a11;
  fA(0, 2) += a10*a12;
  fA(0, 6) += a10*a16;
  fA(0, 7) += a10*a17;
  fA(0, 8) += a10*a18;

  fA(1, 1) += a11*a11;
  fA(1, 2) += a11*a12;
  fA(1, 6) += a11*a16;
  fA(1, 7) += a11*a17;
  fA(1, 8) += a11*a18;

  fA(2, 2) += a12*a12;
  fA(2, 6) += a12*a16;
  fA(2, 7) += a12*a17;
  fA(2, 8) += a12*a18;

  fA(6, 6) += a16*a16;
  fA(6, 7) += a16*a17;
  fA(6, 8) += a16*a18;

  fA(7, 7) += a17*a17;
  fA(7, 8) += a17*a18;

  fA(8, 8) += a18*a18;

  float a20 = -worldx*imagey;
  float a21 = -worldy*imagey;
  float a22 = -imagey;
  float a23 = worldx*imagex;
  float a24 = worldy*imagex;
  float a25 = imagex;

  fA(0, 0) += a20*a20;
  fA(0, 1) += a20*a21;
  fA(0, 2) += a20*a22;
  fA(0, 3) += a20*a23;
  fA(0, 4) += a20*a24;
  fA(0, 5) += a20*a25;

  fA(1, 1) += a21*a21;
  fA(1, 2) += a21*a22;
  fA(1, 3) += a21*a23;
  fA(1, 4) += a21*a24;
  fA(1, 5) += a21*a25;

  fA(2, 2) += a22*a22;
  fA(2, 3) += a22*a23;
  fA(2, 4) += a22*a24;
  fA(2, 5) += a22*a25;

  fA(3, 3) += a23*a23;
  fA(3, 4) += a23*a24;
  fA(3, 5) += a23*a25;

  fA(4, 4) += a24*a24;
  fA(4, 5) += a24*a25;

  fA(5, 5) += a25*a25;
}
#endif

#ifdef STABLE_H
void Homography33::compute() {
  if ( valid ) return;

  std::vector<cv::Point2f> sPts;
  std::vector<cv::Point2f> dPts;
  for (int i=0; i<4; i++) {
    sPts.push_back(cv::Point2f(srcPts[i].first, srcPts[i].second));
  }
  for (int i=0; i<4; i++) {
    dPts.push_back(cv::Point2f(dstPts[i].first - cxy.first, dstPts[i].second - cxy.second));
  }
  cv::Mat homography = cv::findHomography(sPts, dPts);
  for (int c=0; c<3; c++) {
    for (int r=0; r<3; r++) {
      H(r,c) = homography.at<double>(r,c);
    }
  }

  valid = true;
}
#else
void Homography33::compute() {
  if ( valid ) return;

  // make symmetric
  for (int i = 0; i < 9; i++)
    for (int j = i+1; j < 9; j++)
      fA(j,i) = fA(i,j);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(fA, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd eigV = svd.matrixV();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      H(i,j) = eigV(i*3+j, eigV.cols()-1);
    }
  }

  valid = true;
}
#endif

std::pair<float,float> Homography33::project(float worldx, float worldy) {
  compute();

  std::pair<float,float> ixy;
  ixy.first = H(0,0)*worldx + H(0,1)*worldy + H(0,2);
  ixy.second = H(1,0)*worldx + H(1,1)*worldy + H(1,2);
  float z = H(2,0)*worldx + H(2,1)*worldy + H(2,2);
  ixy.first = ixy.first/z + cxy.first;
  ixy.second = ixy.second/z + cxy.second;
  return ixy;
}



/******************************************************/
const float Quad::maxQuadAspectRatio = 6;

Quad::Quad(const std::vector< std::pair<float,float> >& p, const std::pair<float,float>& opticalCenter)
  : quadPoints(p), segments(), observedPerimeter(), homography(opticalCenter) {
#ifdef STABLE_H
  std::vector< std::pair<float,float> > srcPts;
  srcPts.push_back(std::make_pair(-1, -1));
  srcPts.push_back(std::make_pair( 1, -1));
  srcPts.push_back(std::make_pair( 1,  1));
  srcPts.push_back(std::make_pair(-1,  1));
  homography.setCorrespondences(srcPts, p);
#else
  homography.addCorrespondence(-1, -1, quadPoints[0].first, quadPoints[0].second);
  homography.addCorrespondence( 1, -1, quadPoints[1].first, quadPoints[1].second);
  homography.addCorrespondence( 1,  1, quadPoints[2].first, quadPoints[2].second);
  homography.addCorrespondence(-1,  1, quadPoints[3].first, quadPoints[3].second);
#endif

#ifdef INTERPOLATE
  p0 = Eigen::Vector2f(p[0].first, p[0].second);
  p3 = Eigen::Vector2f(p[3].first, p[3].second);
  p01 = (Eigen::Vector2f(p[1].first, p[1].second) - p0);
  p32 = (Eigen::Vector2f(p[2].first, p[2].second) - p3);
#endif
}

std::pair<float,float> Quad::interpolate(float x, float y) {
#ifdef INTERPOLATE
  Eigen::Vector2f r1 = p0 + p01 * (x+1.)/2.;
  Eigen::Vector2f r2 = p3 + p32 * (x+1.)/2.;
  Eigen::Vector2f r = r1 + (r2-r1) * (y+1)/2;
  return std::pair<float,float>(r(0), r(1));
#else
  return homography.project(x,y);
#endif
}

std::pair<float,float> Quad::interpolate01(float x, float y) {
  return interpolate(2*x-1, 2*y-1);
}

void Quad::search(const FloatImage& fImage, std::vector<Segment*>& path,
                  Segment& parent, int depth, std::vector<Quad>& quads,
                  const std::pair<float,float>& opticalCenter) {
  // terminal depth occurs when we've found four segments.
  //cout<<"the begining of quad.search"<<endl;
  if(depth >7 && depth < 10){

    if (path[depth] == path[0]){
     //cout << "Entered terminal depth" << endl; // debug code
    // Is the first segment the same as the last segment (i.e., a loop?)
      std::vector< std::pair<float,float> > p(depth);
      std::vector< std::pair<float,float> > p8(8);
      std::vector<float*> ThetaReplace(8);
      std::vector<float> Theta(depth);
      int Location1=0;
      int Location2=0;
      float calculatedPerimeter = 0;
      bool bad = false;
      for (int i = 0; i < depth; i++) {
        // compute intersections between all the lines. This will give us
        // sub-pixel accuracy for the corners of the quad.
        GLine2D linea(std::make_pair(path[i]->getX0(),path[i]->getY0()),
                      std::make_pair(path[i]->getX1(),path[i]->getY1()));
        GLine2D lineb(std::make_pair(path[i+1]->getX0(),path[i+1]->getY0()),
                      std::make_pair(path[i+1]->getX1(),path[i+1]->getY1()));

        p[i] = linea.intersectionWith(lineb);
        calculatedPerimeter += path[i]->getLength();
        // no intersection? Occurs when the lines are almost parallel.
        if (p[i].first == -1)
          bad = true;
      }

      //check that if there have been the same eight points
      if(!bad){
          for(unsigned int qi=0;qi<quads.size();qi++){
              Quad &quad = quads[qi];
              int pointSize=quad.quadPoints.size();
              int flag=0;
              for(unsigned int j=0;j<depth;j++){
                for(unsigned int i = 0;i<pointSize;i++){
                    if( (abs(p[j].first - quad.quadPoints[i].first)< 2) && (abs(p[j].second - quad.quadPoints[i].second)< 2)){
                        flag++;
                        break;
                    }
                }
                if(flag == 8){
                    bad=true;
                    break;
                }
              }
              if(flag == 8){
                  bad=true;
                  break;
              }
          }
      }

      // eliminate quads that don't form a simply connected loop, i.e., those
      // that form an hour glass, or wind the wrong way.
      float NAngleVeticalL=-1.90;
      float NAngleVeticalR=-1.24;
      float PAngleVeticalL=1.24;
      float PAngleVeticalR=-1.90;
      float AngleParallelL=-0.2;
      float AngleParallelR=0.2;
      if (!bad) {
        std::vector<float> t(depth);
        for(unsigned int i=0;i<depth-1;i++){
            t[i] = std::atan2(p[i+1].second-p[i].second, p[i+1].first-p[i].first);
        }
        t[depth-1]=std::atan2(p[0].second-p[depth-1].second, p[0].first-p[depth-1].first);

        int PVeticalNum=0;
        int NVeticalNum=0;
        float ttheta=0.0;
        for(unsigned int i=0;i<depth-1;i++){
            Theta[i] = MathUtil::mod2pi(t[i+1]-t[i]);
            //if( (Theta[i]>AngleVeticalL && Theta[i]<AngleVeticalR)){
            if (Theta[i]>NAngleVeticalL && Theta[i]<NAngleVeticalR){
                ttheta += Theta[i];
                NVeticalNum++;
            }else if(Theta[i]<-NAngleVeticalL && Theta[i]>-NAngleVeticalR){
                ttheta += Theta[i];
                PVeticalNum++;
            }else if (Theta[i]>AngleParallelL && Theta[i]<AngleParallelR) {
               ttheta += Theta[i];
            }else{
                bad=true;
                break;
            }
         }
        if(!bad){
            Theta[depth-1]= MathUtil::mod2pi(t[0]-t[depth-1]);
            if(Theta[depth-1]>NAngleVeticalL && Theta[depth-1]<NAngleVeticalR){
                ttheta += Theta[depth-1];
                NVeticalNum ++;
            }else if(Theta[depth-1]<-NAngleVeticalL && Theta[depth-1]>-NAngleVeticalR){
                ttheta += Theta[depth-1];
                PVeticalNum ++;
            }else if (Theta[depth-1]>AngleParallelL && Theta[depth-1]<AngleParallelR){
                ttheta = ttheta+Theta[depth-1];
                //cout << "ttheta=" << ttheta << endl;
            }else
                bad=true;
        }
        // the magic value is -2*PI. It should be exact,
        if(!bad){
            if (ttheta < -7 || ttheta > -5)
                  bad = true;
        }
        //there is more than six negative vetical angle and two positive vetical angle
        if(!bad){
            if(NVeticalNum != 6)
                bad=true;
            //there are two positive vetical angle
            //and their location are adjoin of two or four
            if(!bad){
                if(PVeticalNum==2){
                    int ThetaReplaceN=0;
                    int LocationFlag=0;
                    for(unsigned int i=0;i<depth;i++){
                        if( !(Theta[i]>AngleParallelL && Theta[i]<AngleParallelR)){
                            ThetaReplace[ThetaReplaceN]=&Theta[i];
                            ThetaReplaceN++;
                            if(Theta[i]<-NAngleVeticalL && Theta[i]>-NAngleVeticalR){
                                LocationFlag++;
                                if(LocationFlag==1)
                                    Location1=ThetaReplaceN;
                                if(LocationFlag==2)
                                    Location2=ThetaReplaceN;
                            }
                        }
                    }

                    if( (abs(Location1-Location2)==3) || (abs(Location1-Location2)==5) )
                        bad=false;
                    else
                        bad=true;

                }
                else
                    bad=true;

            }
            //cout<<" "<<bad<<endl;
        }
      }

/*
    if (!bad) {
      std::vector< float > d((depth*(depth-1)/2));
      int n=0;
      for(unsigned int i=0;i<depth;i++){
          for(unsigned int j=1;j<depth-i;j++){
              d[n]=MathUtil::distance2D(p[i], p[i+j]);
              n++;
          }
      }
      for(int i=0;i<(depth*(depth-1)/2);i++){
          if(d[i] < Quad::minimumEdgeLength){
              bad = true;
              //cout << "tagsize too small" << endl;
          }
      }
      //check aspect ratio
      if (!bad){
          float dmax=d[0];
          float dmin=d[0];
          for(unsigned int i=0;i<(depth*(depth-1)/2);i++){
              if(d[i]>dmax){
                  dmax=d[i];
              }
              if(d[i]<dmin)
                  dmin=d[i];
          }
          if (dmax > dmin*Quad::maxQuadAspectRatio) {
              bad = true;
               //cout << "aspect ratio too extreme" << endl;
            }
      }

    }
*/

    //screen the points,leaving the 8 points,and the first point is constant
    if(!bad){
        std::vector< std::pair<float,float> > pReplace(8);
        int pReplaceN=0;
        for(unsigned int i=0;i<depth-1;i++){
            if( !(Theta[i]>AngleParallelL && Theta[i]<AngleParallelR)){
                pReplace[pReplaceN]=p[i+1];
                pReplaceN++;
            }
        }
        if(pReplaceN != 8)
            pReplace[7]=p[0];
        if( Location2-Location1 == 3){
            p8[0]=pReplace[Location1-1];
            p8[1]=pReplace[Location1];
            p8[2]=pReplace[Location1+1];
            p8[3]=pReplace[Location2-1];
            for(unsigned int i=0;i<8-Location2;i++)
                p8[4+i]=pReplace[Location2+i];
            for(unsigned int i=0;i<(Location1-1);i++)
                p8[9-Location1+i]=pReplace[i];
        }
        if( (Location2-Location1) == 5){
            p8[0]=pReplace[Location2-1];
            p8[3]=pReplace[Location1-1];
            p8[4]=pReplace[Location1];
            p8[5]=pReplace[Location1+1];
            p8[6]=pReplace[Location1+2];
            p8[7]=pReplace[Location1+3];
            if(Location1 == 1){
                p8[1]=pReplace[6];
                p8[2]=pReplace[7];
            }
            if(Location1 == 2){
                p8[1]=pReplace[7];
                p8[2]=pReplace[0];
            }
            if(Location1 == 3){
                p8[1]=pReplace[0];
                p8[2]=pReplace[1];
            }

        }
        /*
        for(unsigned int i=0;i<8;i++)
            cout<<pReplace[i].first<<","<<pReplace[i].second<<" ";
        cout<<"||pReplace"<<endl;
        */
    }



    //computer the constant two lineas's distance,and compare them
    /*
    if(!bad){
        float d1=MathUtil::distance2D(p8[1], p8[0]);
        float d2=MathUtil::distance2D(p8[3], p8[2]);
        float d3=MathUtil::distance2D(p8[4], p8[3]);
        float d4=MathUtil::distance2D(p8[7], p8[0]);
        if( !(abs(d1-d2) < 4) || !(abs(d3-d4) < 8) )
            bad = true;
    }
    */
    if (!bad) {
      std::vector< float > d(8);
      int n=0;
      for(unsigned int i=0;i<7;i++){
              d[i]=MathUtil::distance2D(p[i], p[i+1]);
          }
      d[7]=MathUtil::distance2D(p[7], p[0]);

      //computer the constant two lineas's distance,and compare them
      if( !(abs(d[0]-d[2]) < 4) || !(abs(d[3]-d[7]) < 8) )
          bad = true;
      //check the tagssize
      if(!bad){
          for(int i=0;i<8;i++){
              if(d[i] < Quad::minimumEdgeLength){
                  bad = true;
                  break;
                  //cout << "tagsize too small" << endl;
              }
          }
      }
      //check aspect ratio
      if(!bad){
          if (d[5] > d[1]*Quad::maxQuadAspectRatio) {
              bad = true;
               //cout << "aspect ratio too extreme" << endl;
            }
          if(d[4] > d[2]*Quad::maxQuadAspectRatio)
              bad= true;
      }

    }


      if (!bad) {
/*
        for(unsigned int i=0;i<8;i++)
            cout<<p8[i].first<<","<<p8[i].second<<" ";
        cout<<"||p8"<<endl;
*/
        Quad q(p8, opticalCenter);
        //cout<<"befor the q.segments"<<endl;
        q.segments=path;
        q.observedPerimeter = calculatedPerimeter;
        //cout<<"obseredPerimeter:"<<calculatedPerimeter<<endl;
        //cout<<"q.obseredPerimeter:"<<q.observedPerimeter<<endl;
        quads.push_back(q);
        //cout<<"quads.size():"<<quads.size()<<endl;
      }
    }
    return;
    }
  if(depth == 10)
      return;

  //  if (depth >= 1) // debug code
  //cout << "depth: " << depth << endl;

  // Not terminal depth. Recurse on any children that obey the correct handedness.
  if(parent.children.size() > 0 ){
    for (unsigned int i = 0; i < parent.children.size(); i++) {
        Segment &child = *parent.children[i];
        //    cout << "  Child " << child.getId() << ":  ";
        // (handedness was checked when we created the children)
        //cout<<"Children.size:"<<parent.children.size()<<endl;

        // we could rediscover each quad 4 times (starting from
        // each corner). If we had an arbitrary ordering over
        // points, we can eliminate the redundant detections by
        // requiring that the first corner have the lowest
        // value. We're arbitrarily going to use theta...

        /*don't delete*/
        /*
        * if ( child.getTheta() > path[0]->getTheta() ) {
        cout << "theta failed: " << child.getTheta() << " > " << path[0]->getTheta() << endl;
        continue;
        }
        */
        //cout<<"depth+1: "<<depth+1<<endl;
        path[depth+1] = &child;
        //path.push_back(&child);
        //cout<<"path.size"<<path.size()<<endl;
        search(fImage, path, child, depth+1, quads, opticalCenter);
    }
  }
  return;
}




/****************************************************/
//std::vector<Segment> TagDetector::extractTags(cv::Mat& image) {
std::vector<Quad> extractTags(cv::Mat& image,int minimumSegmentSize, float minimumLineLength) {

  // convert to internal AprilTags image (todo: slow, change internally to OpenCV)
  int width = image.cols;
  int height = image.rows;
  AprilTags::FloatImage fimOrig(width, height);
  int i = 0;
  for (int y=0; y<height; y++) {
    for (int x=0; x<width; x++) {
      fimOrig.set(x, y, image.data[i]/255.);
      i++;
    }
  }
  std::pair<int,int> opticalCenter(width/2, height/2);

#ifdef DEBUG_APRIL
#if 0
{ // debug - write
  int height_ = fimOrig.getHeight();
  int width_  = fimOrig.getWidth();
  cv::Mat image(height_, width_, CV_8UC3);
  {
    for (int y=0; y<height_; y++) {
      for (int x=0; x<width_; x++) {
        cv::Vec3b v;
        //        float vf = fimMag.get(x,y);
        float vf = fimOrig.get(x,y);
        int val = (int)(vf * 255.);
        if ((val & 0xffff00) != 0) {printf("problem... %i\n", val);}
        for (int k=0; k<3; k++) {
          v(k) = val;
        }
        image.at<cv::Vec3b>(y, x) = v;
      }
    }
  }
  imwrite("out.bmp", image);
}
#endif
#if 0
FloatImage fimOrig = fimOrig_;
{ // debug - read

  cv::Mat image = cv::imread("test.bmp");
  int height_ = fimOrig.getHeight();
  int width_  = fimOrig.getWidth();
  {
    for (int y=0; y<height_; y++) {
      for (int x=0; x<width_; x++) {
        cv::Vec3b v = image.at<cv::Vec3b>(y,x);
        float val = (float)v(0)/255.;
        fimOrig.set(x,y,val);
      }
    }
  }
}
#endif
#endif

//================================================================
// Step one: preprocess image (convert to grayscale) and low pass if necessary

FloatImage fim = fimOrig;

//! Gaussian smoothing kernel applied to image (0 == no filter).
/*! Used when sampling bits. Filtering is a good idea in cases
 * where A) a cheap camera is introducing artifical sharpening, B)
 * the bayer pattern is creating artifcats, C) the sensor is very
 * noisy and/or has hot/cold pixels. However, filtering makes it
 * harder to decode very small tags. Reasonable values are 0, or
 * [0.8, 1.5].
 */
float sigma = 0;

//! Gaussian smoothing kernel applied to image (0 == no filter).
/*! Used when detecting the outline of the box. It is almost always
 * useful to have some filtering, since the loss of small details
 * won't hurt. Recommended value = 0.8. The case where sigma ==
 * segsigma has been optimized to avoid a redundant filter
 * operation.
 */
float segSigma = 0.8f;

if (sigma > 0) {
  int filtsz = ((int) max(3.0f, 3*sigma)) | 1;
  std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
  fim.filterFactoredCentered(filt, filt);
}

//================================================================
// Step two: Compute the local gradient. We store the direction and magnitude.
// This step is quite sensitve to noise, since a few bad theta estimates will
// break up segments, causing us to miss Quads. It is useful to do a Gaussian
// low pass on this step even if we don't want it for encoding.

FloatImage fimSeg;
if (segSigma > 0) {
  if (segSigma == sigma) {
    fimSeg = fim;
  } else {
    // blur anew
    int filtsz = ((int) max(3.0f, 3*segSigma)) | 1;
    std::vector<float> filt = Gaussian::makeGaussianFilter(segSigma, filtsz);
    fimSeg = fimOrig;
    fimSeg.filterFactoredCentered(filt, filt);
  }
} else {
  fimSeg = fimOrig;
}

FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());


#pragma omp parallel for
for (int y = 1; y < fimSeg.getHeight()-1; y++) {
  for (int x = 1; x < fimSeg.getWidth()-1; x++) {
    float Ix = fimSeg.get(x+1, y) - fimSeg.get(x-1, y);
    float Iy = fimSeg.get(x, y+1) - fimSeg.get(x, y-1);

    float mag = Ix*Ix + Iy*Iy;
#if 0 // kaess: fast version, but maybe less accurate?
    float theta = MathUtil::fast_atan2(Iy, Ix);
#else
    float theta = atan2(Iy, Ix);
#endif

    fimTheta.set(x, y, theta);
    fimMag.set(x, y, mag);
  }
}

#ifdef DEBUG_APRIL
int height_ = fimSeg.getHeight();
int width_  = fimSeg.getWidth();
cv::Mat image(height_, width_, CV_8UC3);
{
  for (int y=0; y<height_; y++) {
    for (int x=0; x<width_; x++) {
      cv::Vec3b v;
      //        float vf = fimMag.get(x,y);
      float vf = fimOrig.get(x,y);
      int val = (int)(vf * 255.);
      if ((val & 0xffff00) != 0) {printf("problem... %i\n", val);}
      for (int k=0; k<3; k++) {
        v(k) = val;
      }
      image.at<cv::Vec3b>(y, x) = v;
    }
  }
}
#endif

//================================================================
// Step three. Extract edges by grouping pixels with similar
// thetas together. This is a greedy algorithm: we start with
// the most similar pixels.  We use 4-connectivity.
UnionFindSimple uf(fimSeg.getWidth()*fimSeg.getHeight());

vector<Edge> edges(width*height*4);
size_t nEdges = 0;

// Bounds on the thetas assigned to this group. Note that because
// theta is periodic, these are defined such that the average
// value is contained *within* the interval.
{ // limit scope of storage
  /* Previously all this was on the stack, but this is 1.2MB for 320x240 images
   * That's already a problem for OS X (default 512KB thread stack size),
   * could be a problem elsewhere for bigger images... so store on heap */
  vector<float> storage(width*height*4);  // do all the memory in one big block, exception safe
  float * tmin = &storage[width*height*0];
  float * tmax = &storage[width*height*1];
  float * mmin = &storage[width*height*2];
  float * mmax = &storage[width*height*3];

  for (int y = 0; y+1 < height; y++) {
    for (int x = 0; x+1 < width; x++) {

      float mag0 = fimMag.get(x,y);
      if (mag0 < Edge::minMag)
        continue;
      mmax[y*width+x] = mag0;
      mmin[y*width+x] = mag0;

      float theta0 = fimTheta.get(x,y);
      tmin[y*width+x] = theta0;
      tmax[y*width+x] = theta0;

      // Calculates then adds edges to 'vector<Edge> edges'
      Edge::calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);

      // XXX Would 8 connectivity help for rotated tags?
      // Probably not much, so long as input filtering hasn't been disabled.
    }
  }

  edges.resize(nEdges);
  std::stable_sort(edges.begin(), edges.end());
  Edge::mergeEdges(edges,uf,tmin,tmax,mmin,mmax);
}

//================================================================
// Step four: Loop over the pixels again, collecting statistics for each cluster.
// We will soon fit lines (segments) to these points.

map<int, vector<XYWeight> > clusters;
for (int y = 0; y+1 < fimSeg.getHeight(); y++) {
  for (int x = 0; x+1 < fimSeg.getWidth(); x++) {
    if (uf.getSetSize(y*fimSeg.getWidth()+x) < minimumSegmentSize)
      continue;

    int rep = (int) uf.getRepresentative(y*fimSeg.getWidth()+x);

    map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
    if ( it == clusters.end() ) {
      clusters[rep] = vector<XYWeight>();
      it = clusters.find(rep);
    }
    vector<XYWeight> &points = it->second;
    points.push_back(XYWeight(x,y,fimMag.get(x,y)));
  }
}

//================================================================
// Step five: Loop over the clusters, fitting lines (which we call Segments).
std::vector<Segment> segments; //used in Step six
std::map<int, std::vector<XYWeight> >::const_iterator clustersItr;
for (clustersItr = clusters.begin(); clustersItr != clusters.end(); clustersItr++) {
  std::vector<XYWeight> points = clustersItr->second;
  GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

  // filter short lines
  float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());
  if (length < minimumLineLength)
    continue;

  Segment seg;
  float dy = gseg.getP1().second - gseg.getP0().second;
  float dx = gseg.getP1().first - gseg.getP0().first;

  float tmpTheta = std::atan2(dy,dx);

  seg.setTheta(tmpTheta);
  seg.setLength(length);

  // We add an extra semantic to segments: the vector
  // p1->p2 will have dark on the left, white on the right.
  // To do this, we'll look at every gradient and each one
  // will vote for which way they think the gradient should
  // go. This is way more retentive than necessary: we
  // could probably sample just one point!

  float flip = 0, noflip = 0;
  for (unsigned int i = 0; i < points.size(); i++) {
    XYWeight xyw = points[i];

    float theta = fimTheta.get((int) xyw.x, (int) xyw.y);
    float mag = fimMag.get((int) xyw.x, (int) xyw.y);

    // err *should* be +M_PI/2 for the correct winding, but if we
    // got the wrong winding, it'll be around -M_PI/2.
    float err = MathUtil::mod2pi(theta - seg.getTheta());

    if (err < 0)
      noflip += mag;
    else
      flip += mag;
  }

  if (flip > noflip) {
    float temp = seg.getTheta() + (float)M_PI;
    seg.setTheta(temp);
  }

  float dot = dx*std::cos(seg.getTheta()) + dy*std::sin(seg.getTheta());
  if (dot > 0) {
    seg.setX0(gseg.getP1().first); seg.setY0(gseg.getP1().second);
    seg.setX1(gseg.getP0().first); seg.setY1(gseg.getP0().second);
  }
  else {
    seg.setX0(gseg.getP0().first); seg.setY0(gseg.getP0().second);
    seg.setX1(gseg.getP1().first); seg.setY1(gseg.getP1().second);
  }

  segments.push_back(seg);
}

#ifdef my_DEBUG_APRIL
#if 1
{
    //cout<<"segments.size:"<<segments.size()<<endl;
    int i=0;
  for (vector<Segment>::iterator it = segments.begin(); it!=segments.end(); it++) {
    //long int r = random();
    i++;
    cv::line(image,
             cv::Point2f(it->getX0(), it->getY0()),
             cv::Point2f(it->getX1(), it->getY1()),
             cv::Scalar(255,255,255) ,2);
    //cout<<"segments.children.size:"<<segments[i].children.size()<<endl;
    //cv::Scalar(r%0xff,(r%0xff00)>>8,(r%0xff0000)>>16,0)
  }
  //cv::imshow("debug_april", image);
}
#endif
#endif

// Step six: For each segment, find segments that begin where this segment ends.
// (We will chain segments together next...) The gridder accelerates the search by
// building (essentially) a 2D hash table.
Gridder<Segment> gridder(0,0,width,height,10);

// add every segment to the hash table according to the position of the segment's
// first point. Remember that the first point has a specific meaning due to our
// left-hand rule above.
for (unsigned int i = 0; i < segments.size(); i++) {
  gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
}

// Now, find child segments that begin where each parent segment ends.
for (unsigned i = 0; i < segments.size(); i++) {
  Segment &parentseg = segments[i];

  //compute length of the line segment
  GLine2D parentLine(std::pair<float,float>(parentseg.getX0(), parentseg.getY0()),
                     std::pair<float,float>(parentseg.getX1(), parentseg.getY1()));

  Gridder<Segment>::iterator iter = gridder.find(parentseg.getX1(), parentseg.getY1(), 0.5f*parentseg.getLength());
  while(iter.hasNext()) {
    Segment &child = iter.next();
/*
    if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) {
      continue;
    }
*/

    // compute intersection of points
    GLine2D childLine(std::pair<float,float>(child.getX0(), child.getY0()),
                      std::pair<float,float>(child.getX1(), child.getY1()));

    std::pair<float,float> p = parentLine.intersectionWith(childLine);
    if (p.first == -1) {
      continue;
    }

    //cv::circle(image, cv::Point2f(p.first, p.second), 3, cv::Scalar(0,255,0,0), 1);

    float parentDist = MathUtil::distance2D(p, std::pair<float,float>(parentseg.getX1(),parentseg.getY1()));
    float childDist = MathUtil::distance2D(p, std::pair<float,float>(child.getX0(),child.getY0()));

    if (max(parentDist,childDist) > parentseg.getLength()) {
      // cout << "intersection too far" << endl;
      continue;
    }

    // everything's OK, this child is a reasonable successor.
    parentseg.children.push_back(&child);
  }
}



//================================================================
// Step seven: Search all connected segments to see if any form a loop of length 4.
// Add those to the quads list.
vector<Quad> quads;

struct timeval start;
 struct timeval end;
 gettimeofday( &start, NULL );

vector<Segment*> tmp(20);
for (unsigned int i = 0; i < segments.size(); i++) {
  tmp[0] = &segments[i];
  //tmp.push_back(&segments[i]);
  Quad::search(fimOrig, tmp, segments[i], 0, quads, opticalCenter);
}

gettimeofday( &end, NULL );
long timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
long timemse = timeuse/1000;
//cout<<"QuadSearch's Time:"<<timemse<<"ms"<<endl;

#ifdef my_DEBUG_APRIL
#if 1
{

  //cout <<"imshow line"<< endl;
  for (unsigned int qi = 0; qi < quads.size(); qi++ ) {
    Quad &quad = quads[qi];

    int pointSize=quad.quadPoints.size();
    std::vector< std::pair<float,float> > p(pointSize);
    for(unsigned int i = 0;i<pointSize;i++){
        p[i]=quad.quadPoints[i];
    }
    cv::circle(image, cv::Point2f(p[0].first, p[0].second), 3, cv::Scalar(0,255,0,0), 3);
    for(unsigned int i = 0;i<pointSize-1;i++){
        cv::line(image, cv::Point2f(p[i].first, p[i].second), cv::Point2f(p[i+1].first, p[i+1].second), cv::Scalar(0,255,0) );
    }
    cv::line(image, cv::Point2f(p[pointSize-1].first, p[pointSize-1].second), cv::Point2f(p[0].first, p[0].second), cv::Scalar(0,255,0) );
/*
    if(qi == 0)
        cv::imshow("Quad0",image);
    if(qi == 1)
        cv::imshow("Quad1",image);
    if(qi == 2)
        cv::imshow("Quad2",image);
    if(qi == 3)
        cv::imshow("Quad3",image);
    if(qi == 4)
        cv::imshow("Quad4",image);
    if(qi == 5)
        cv::imshow("Quad5",image);
    if(qi == 6)
        cv::imshow("Quad6",image);
    if(qi == 7)
        cv::imshow("Quad7",image);
    if(qi == 8)
        cv::imshow("Quad8",image);
    if(qi == 9)
        cv::imshow("Quad9",image);
    if(qi == 10)
        cv::imshow("Quad10",image);
*/

  }
  //cv::imshow("debug_april", image);
  //cv::imshow("Quad",image);
}
#endif
#endif

  return quads;
}
/*********************************/


} // namespace

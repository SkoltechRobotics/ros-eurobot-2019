#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
/** ********************************************/
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <set>

using namespace std;
using std::string;

#ifndef ASTAR_CPP
#define ASTAR_CPP

/**
 * @struct cells
 * @brief A struct that represents a cell and its fCost.
 */

namespace Astar_planner {
struct cells {
  cells(int currentCell_, int fCost_) : currentCell(currentCell_), fCost(fCost_){};
  int currentCell;
  float fCost;
  bool operator<(const cells& rhs) const {
    if (this->fCost == rhs.fCost) {
      return this->currentCell < rhs.currentCell;
    }
    return this->fCost < rhs.fCost;
  }
};

struct OccupancyGrid {
  int width;
  int height;
  int mapSize;
  boost::shared_array<bool> data;
};

struct Point {
  Point(int x_, int y_) : x(x_), y(y_){};
  int x;
  int y;
   Point operator / (double f) const {
        Point p(1, 1);
        p.x = x / f;
        p.y = y / f;
        return p;
    }

    Point operator * (double f) const {
        Point p(1, 1);
        p.x = x * f;
        p.y = y * f;
        return p;
    }

    Point operator *= (double f) {
        x *= f;
        y *= f;
        return *this;
    }

    Point operator + (const Point& rhs) const {
        Point p(1, 1);
        p.x = x + rhs.x;
        p.y = y + rhs.y;
        return p;
    }

    Point operator - (const Point& rhs) const {
        Point p(1, 1);
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        return p;
    }

    Point operator - () const {
        Point p(1, 1);
        p.x = -x;
        p.y = -y;
        return p;
    }

    bool operator == (const Point& rhs) const{
        return x==rhs.x && y==rhs.y;
    }

    bool operator != (const Point& rhs) const{
        return !(*this == rhs);
}
};
struct Pose2d{
  Pose2d (double x_, double y_) : x(x_), y(y_){};
  double x;
  double y;
  Pose2d operator / (double f) const {
        Pose2d p(1, 1);
        p.x = x / f;
        p.y = y / f;
        return p;
    }

    Pose2d operator * (double f) const {
        Pose2d p(1, 1);
        p.x = x * f;
        p.y = y * f;
        return p;
    }

    Pose2d operator *= (double f) {
        x *= f;
        y *= f;
        return *this;
    }

    Pose2d operator + (const Pose2d& rhs) const {
        Pose2d p(1, 1);
        p.x = x + rhs.x;
        p.y = y + rhs.y;
        return p;
    }

    Pose2d operator - (const Pose2d& rhs) const {
        Pose2d p(1, 1);
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        return p;
    }

    Pose2d operator - () const {
        Pose2d p(1, 1);
        p.x = -x;
        p.y = -y;
        return p;
    }

    bool operator == (const Pose2d& rhs) const{
        return x==rhs.x && y==rhs.y;
    }

    bool operator != (const Pose2d& rhs) const{
        return !(*this == rhs);
}
};

class AstarPlanner {
 public:
  /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  bool makePlan(const OccupancyGrid& map, const Point& start, const Point& goal, std::vector<Point>& output_path);
  void setNewMap(const OccupancyGrid& map);

 private:
  boost::shared_ptr<OccupancyGrid> map;
  vector<int> findPath(int startCell, int goalCell);
  vector<int> constructPath(int startCell, int goalCell, float g_score[]);

  float calculateHCost(int cellID, int goalCell) {
    int x1 = getCellRowID(goalCell);
    int y1 = getCellColID(goalCell);
    int x2 = getCellRowID(cellID);
    int y2 = getCellColID(cellID);
    return abs(x1 - x2) + abs(y1 - y2);
    // return min(abs(x1-x2),abs(y1-y2))*sqrt(2) +
    // max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  }
  void addNeighborCellToOpenList(multiset<cells>& OPL, int neighborCell, int goalCell, float g_score[]);
  vector<int> findFreeNeighborCell(int CellID);
  bool isStartAndGoalCellsValid(int startCell, int goalCell);
  float getMoveCost(int CellID1, int CellID2);
  float getMoveCost(int i1, int j1, int i2, int j2);
  bool isFree(int CellID);  // returns true if the cell is Free
  bool isFree(int i, int j);
  int getCellIndex(int row, int col)  // get the index of the cell to be used in Path
  {
    return (row * width) + col;
  }
  int getCellRowID(int index)  // get the row ID from cell index
  {
    return index / width;
  }
  int getCellColID(int index)  // get column ID from cell index
  {
    return index % width;
  }
  bool isPointInsideMap(int x, int y);
  int width = 1;
  int height = 1;
  int mapSize = 1;
  float tBreak = 1;
  boost::shared_array<bool> OGM = boost::shared_array<bool>(new bool[1]);
};

};  // namespace RAstar_planner
#endif
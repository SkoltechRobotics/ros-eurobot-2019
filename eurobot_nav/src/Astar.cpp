#include "Astar.h"
#include <stdio.h>
#include <climits>
#include <iostream>
#include <limits>

static const float INFINIT_COST = INT_MAX;  //!< cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();

inline vector<int> findFreeNeighborCell(int CellID);

namespace Astar_planner {

void AstarPlanner::setNewMap(const OccupancyGrid& map) {
  width = map.width;
  height = map.height;
  mapSize = width * height;
  tBreak = 1 + 1 / (mapSize);

  OGM.reset(new bool[mapSize]);
  for (int i = 0; i < map.mapSize; i++) {
    OGM[i] = map.data[i];
  }
  std::cout << "RAstar planner set map successfully" << std::endl;
}

bool AstarPlanner::makePlan(const OccupancyGrid& map,
                            const Point& start,
                            const Point& goal,
                            std::vector<Point>& output_path) {
  ////////////////////////////////////////////////////////
  // set new map
  std::cout << "START MAKE PLAN start: " << start.x << " " << start.y << " goal: " << goal.x << " " << goal.y
            << std::endl;
  setNewMap(map);
  // std::cout << "Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.position.x, start.position.y,
  // goal.position.x, goal.position.y);

  ////////////////////////////////////////////////////////
  // convert the start and goal positions
  int startX = start.x;
  int startY = start.y;
  int goalX = goal.x;
  int goalY = goal.y;
  if (!(isPointInsideMap(startX, startY) && isPointInsideMap(goalX, goalY))) {
    cout << "the start or goal is out of the map" << std::endl;
    return false;
  }
  int startCell = getCellIndex(startY, startX);
  int goalCell = getCellIndex(goalY, goalX);

  /////////////////////////////////////////////////////////
  // call global planner
  if (!isStartAndGoalCellsValid(startCell, goalCell)) {
    cout << "Not valid start or goal" << std::endl;
    return false;
  }
  vector<int> bestPath;
  bestPath.clear();
  bestPath = findPath(startCell, goalCell);

  /////////////////////////////////////////////////////////
  // make output path
  if (bestPath.size() == 0) {
    cout << "The planner failed to find a path, choose other" << std::endl;
    return false;
  }
  for (int i = 0; i < bestPath.size(); i++) {
    int x = getCellColID(bestPath[i]);
    int y = getCellRowID(bestPath[i]);
    output_path.push_back(Point(x, y));
  }
  return true;
}

bool AstarPlanner::isPointInsideMap(int x, int y) {
  return (x < width) && (y < height) && (x > 0) && (y > 0);
}

/*******************************************************************************/
// Function Name: findPath
// Inputs: the map layout, the start and the goal Cells and a boolean to
// indicate if we will use break ties or not Output: the best path Description:
// it is used to generate the robot free path
/*********************************************************************************/
vector<int> AstarPlanner::findPath(int startCell, int goalCell) {
  cout << "START FIND PATH " << startCell << " " << goalCell << std::endl;

  float g_score[mapSize];
  for (uint i = 0; i < mapSize; i++)
    g_score[i] = infinity;

  vector<int> bestPath;
  vector<int> emptyPath;
  cells CP(startCell, g_score[startCell] + calculateHCost(startCell, goalCell));

  multiset<cells> OPL;
  int currentCell;

  // calculate g_score and f_score of the start position
  g_score[startCell] = 0;

  // add the start cell to the open list
  OPL.insert(CP);
  currentCell = startCell;

  // while the open list is not empty continue   the search or
  // g_score(goalCell) is equal to infinity
  while (!OPL.empty() && g_score[goalCell] == infinity) {
    // choose the cell that has the lowest cost fCost in the open
    // set which is the begin of the multiset
    currentCell = OPL.begin()->currentCell;
    // remove the currentCell from the openList
    OPL.erase(OPL.begin());
    // search the neighbors of the current Cell
    vector<int> neighborCells;
    neighborCells = findFreeNeighborCell(currentCell);
    for (uint i = 0; i < neighborCells.size(); i++)  // for each neighbor v of current cell
    {
      // if the g_score of the neighbor is equal to INF:
      // unvisited cell
      float g_score1 = g_score[currentCell] + getMoveCost(currentCell, neighborCells[i]);
      if (g_score[neighborCells[i]] == infinity) {
        g_score[neighborCells[i]] = g_score1;
        addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score);
      } else {
        if (g_score1 < g_score[neighborCells[i]]) {
          OPL.erase(cells(neighborCells[i], g_score[neighborCells[i]] + calculateHCost(neighborCells[i], goalCell)));
          g_score[neighborCells[i]] = g_score1;
          OPL.insert(cells(neighborCells[i], g_score[neighborCells[i]] + calculateHCost(neighborCells[i], goalCell)));
        }
      }
      // end if
    }
    // end for
  }  // end while

  int newGoalCell = startCell;
  if (g_score[goalCell] == infinity) {
    float h_min = calculateHCost(newGoalCell, goalCell);
    for (int i = 0; i < mapSize; i++) {
      if (g_score[i] != infinity) {
        float h = calculateHCost(i, goalCell);
        if (h < h_min) {
          h_min = h;
          newGoalCell = i;
        }
      }
    }
  }

  if (g_score[goalCell] != infinity)  // if g_score(goalcell)==INF : construct path
  {
    bestPath = constructPath(startCell, goalCell, g_score);
    return bestPath;
  } else {
    cout << "Failure to find a path to goal !" << std::endl;
    bestPath = constructPath(startCell, newGoalCell, g_score);
    return bestPath;
  }
}

/*******************************************************************************/
// Function Name: constructPath
// Inputs: the start and the goal Cells
// Output: the best path
// Description: it is used to construct the robot path
/*********************************************************************************/
vector<int> AstarPlanner::constructPath(int startCell, int goalCell, float g_score[]) {
  cout << "CONSTRUCT THE ROBOT PATH" << endl;
  vector<int> bestPath;
  vector<int> path;

  path.insert(path.begin() + bestPath.size(), goalCell);
  int currentCell = goalCell;

  while (currentCell != startCell) {
    vector<int> neighborCells;
    neighborCells = findFreeNeighborCell(currentCell);

    vector<float> gScoresNeighbors;
    for (uint i = 0; i < neighborCells.size(); i++)
      gScoresNeighbors.push_back(g_score[neighborCells[i]]);

    int posMinGScore =
        distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
    currentCell = neighborCells[posMinGScore];

    // insert the neighbor in the path
    path.insert(path.begin() + path.size(), currentCell);
  }
  for (uint i = 0; i < path.size(); i++)
    bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

  return bestPath;
}

/*******************************************************************************/
// Function Name: addNeighborCellToOpenList
// Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell
// Output:
// Description: it is used to add a neighbor Cell to the open list
/*********************************************************************************/
void AstarPlanner::addNeighborCellToOpenList(multiset<cells>& OPL, int neighborCell, int goalCell, float g_score[]) {
  cells CP(neighborCell, g_score[neighborCell] + +calculateHCost(neighborCell, goalCell));
  OPL.insert(CP);
}

/*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and column of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell
 *********************************************************************************/

vector<int> AstarPlanner::findFreeNeighborCell(int CellID) {
  int rowID = getCellRowID(CellID);
  int colID = getCellColID(CellID);
  int neighborIndex;
  vector<int> freeNeighborCells;

  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++) {
      // check whether the index is valid
      if ((rowID + i >= 0) && (rowID + i < height) && (colID + j >= 0) && (colID + j < width) &&
          (!(i == 0 && j == 0))) {
        neighborIndex = getCellIndex(rowID + i, colID + j);
        if (isFree(neighborIndex))
          freeNeighborCells.push_back(neighborIndex);
      }
    }
  return freeNeighborCells;
}

/*******************************************************************************/
// Function Name: isStartAndGoalCellsValid
// Inputs: the start and Goal cells
// Output: true if the start and the goal cells are valid
// Description: check if the start and goal cells are valid
/*********************************************************************************/
bool AstarPlanner::isStartAndGoalCellsValid(int startCell, int goalCell) {
  bool isvalid = true;
  bool isFreeStartCell = isFree(startCell);
  bool isFreeGoalCell = isFree(goalCell);
  if (startCell == goalCell) {
    cout << "The Start and the Goal cells are the same..." << endl;
    isvalid = false;
    // } else if (!isFreeStartCell && !isFreeGoalCell) {
    //   cout << "The start and the goal cells are obstacle positions..." << endl;
    //   isvalid = false;
  } else if (!isFreeStartCell) {
    cout << "The start is an obstacle..." << endl;
    isvalid = false;
    // } else if (!isFreeGoalCell) {
    //   cout << "The goal cell is an obstacle..." << endl;
    //   isvalid = false;
    // } else if (findFreeNeighborCell(goalCell).size() == 0) {
    //   cout << "The goal cell is encountred by obstacles..." << endl;
    //   isvalid = false;
  } else if (findFreeNeighborCell(startCell).size() == 0) {
    cout << "The start cell is encountred by obstacles... " << endl;
    isvalid = false;
  }
  return isvalid;
}

float AstarPlanner::getMoveCost(int i1, int j1, int i2, int j2) {
  float moveCost = INFINIT_COST;  // start cost with maximum value. Change it
                                  // to real cost of cells are connected
  // if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
  if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) ||
      (j2 == j1 - 1 && i2 == i1 + 1)) {
    // moveCost = DIAGONAL_MOVE_COST;
    moveCost = 1.4;
  }
  // if cell 2(i2,j2) exists in the horizontal or vertical line with
  // cell1(i1,j1)
  else {
    if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) ||
        (i1 == i2 && j2 == j1 + 1)) {
      // moveCost = MOVE_COST;
      moveCost = 1;
    }
  }
  return moveCost;
}

float AstarPlanner::getMoveCost(int CellID1, int CellID2) {
  int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

  i1 = getCellRowID(CellID1);
  j1 = getCellColID(CellID1);
  i2 = getCellRowID(CellID2);
  j2 = getCellColID(CellID2);

  return getMoveCost(i1, j1, i2, j2);
}

// verify if the cell(i,j) is free
bool AstarPlanner::isFree(int i, int j) {
  int CellID = getCellIndex(i, j);
  return OGM[CellID];
}

// verify if the cell(i,j) is free
bool AstarPlanner::isFree(int CellID) {
  return OGM[CellID];
}
}  // namespace Astar_planner



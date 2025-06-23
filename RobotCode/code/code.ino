#include "dijkstra.h"
#include "Defines.h"
#include "move.h"
#include "utils.h"
#include "BluetoothSerial.h"


BluetoothSerial DebugSerial;

enum Directions { UP = -GRID_SIZE,
                  DOWN = GRID_SIZE,
                  LEFT = -1,
                  RIGHT = 1 };

ArraysLengthType currentNode = 144;
ArraysLengthType currentDir = 0;
ArraysLengthType currentFloor = 0;
bool overTheRamp = false;

int8_t ROTATION_DIRS[4]{ Directions::UP, Directions::RIGHT, Directions::DOWN, Directions::LEFT };

CameraInfo data;

ArraysLengthType unvisitedNodes[MAX_NODES / 2];
ArraysLengthType unvisitedNodesCount = 0;

ArraysLengthType seenNodes[MAX_NODES];
ArraysLengthType seenNodesCount = 0;

ArraysLengthType redPosts[3] = { 0, 0, 0 };
ArraysLengthType redPostsCount = 0;

ArraysLengthType greenPosts[3] = { 0, 0, 0 };
ArraysLengthType greenPostsCount = 0;

ArraysLengthType ramps[10];
ArraysLengthType rampsCount = 0;

ArraysLengthType secondFloor[30];
ArraysLengthType secondFloorCount = 0;

ArraysLengthType redLines[69];
ArraysLengthType redLinesCount = 0;


ArraysLengthType distances[MAX_NODES];
int16_t predecessors[MAX_NODES];
ArraysLengthType path[MAX_NODES];
Graph g;

//----------------------------------------------------------------------------------
void doScan() {
  if (!receive(&data)) {
    DebugSerial.println("-ERROR-check-connection--------------------------");
  } else {
    DebugSerial.print("Recieved: ");
    DebugSerial.print(data.red);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.line);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.green);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.blue);
    DebugSerial.print(" -> ");
    DebugSerial.println(data.floor);
  }

  if (!data.line) {
    if (data.floor == 1) {
      secondFloor[secondFloorCount] = currentNode + ROTATION_DIRS[currentDir];
      secondFloorCount++;
    }
    if (currentFloor == data.floor && data.red == 0 && data.green == 0 && data.blue == 0) {  // empty cell
      g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], 1);

      seenNodes[seenNodesCount] = currentNode + ROTATION_DIRS[currentDir];
      seenNodesCount++;

      unvisitedNodes[unvisitedNodesCount] = currentNode + ROTATION_DIRS[currentDir];
      unvisitedNodesCount++;

      DebugSerial.print("Added empty cell: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else if (currentFloor == data.floor && data.red == 2 && data.green == 0 && data.blue == 0) {  // Red post
      g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], 15);

      redPosts[redPostsCount] = currentNode + ROTATION_DIRS[currentDir];
      redPostsCount++;

      seenNodes[seenNodesCount] = currentNode + ROTATION_DIRS[currentDir];
      seenNodesCount++;

      DebugSerial.print("Added red post: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else if (currentFloor == data.floor && data.red == 0 && data.green == 3 && data.blue == 0) {  // Green post
      g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], 15);

      greenPosts[greenPostsCount] = currentNode + ROTATION_DIRS[currentDir];
      greenPostsCount++;

      seenNodes[seenNodesCount] = currentNode + ROTATION_DIRS[currentDir];
      seenNodesCount++;

      DebugSerial.print("Added green post: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else if (currentFloor == data.floor && data.red == 0 && data.green == 1 && data.blue == 0) {  // Green post sideways (found on the left)
      for (int i = 1; i < 4; i++) {
        g.addNode(currentNode + i * ROTATION_DIRS[currentDir], currentNode + i * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 1) % 4], 15);

        greenPosts[greenPostsCount] = currentNode + i * ROTATION_DIRS[currentDir];
        greenPostsCount++;

        seenNodes[seenNodesCount] = currentNode + i * ROTATION_DIRS[currentDir];
        seenNodesCount++;
      }

      g.addNode(currentNode + ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 1) % 4], currentNode + 2 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 1) % 4], 1);
      g.addNode(currentNode + 2 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 1) % 4], currentNode + 3 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 1) % 4], 1);

      DebugSerial.print("Added green post to the left of us: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else if (currentFloor == data.floor && data.red == 0 && data.green == 2 && data.blue == 0) {  // Green post sideways (found on the right)
      for (int i = 1; i < 4; i++) {
        g.addNode(currentNode + i * ROTATION_DIRS[currentDir], currentNode + i * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 3) % 4], 15);

        greenPosts[greenPostsCount] = currentNode + i * ROTATION_DIRS[currentDir];
        greenPostsCount++;

        seenNodes[seenNodesCount] = currentNode + i * ROTATION_DIRS[currentDir];
        seenNodesCount++;
      }

      g.addNode(currentNode + ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 3) % 4], currentNode + 2 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 3) % 4], 1);
      g.addNode(currentNode + 2 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 3) % 4], currentNode + 3 * ROTATION_DIRS[currentDir] + ROTATION_DIRS[(currentDir + 3) % 4], 1);

      DebugSerial.print("Added green post to the right of us: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else if (data.red == 1 && data.blue == 1) {  // Ramp
      g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], 3);
      g.addNode(currentNode + ROTATION_DIRS[currentDir], currentNode + 2 * ROTATION_DIRS[currentDir], 1);

      seenNodes[seenNodesCount] = currentNode + ROTATION_DIRS[currentDir];
      seenNodesCount++;
      seenNodes[seenNodesCount] = currentNode + 2 * ROTATION_DIRS[currentDir];
      seenNodesCount++;
      ramps[rampsCount] = currentNode + ROTATION_DIRS[currentDir];
      rampsCount++;

      if (!NO_SECOND_FLOOR) {
        unvisitedNodes[unvisitedNodesCount] = currentNode + 2 * ROTATION_DIRS[currentDir];
        unvisitedNodesCount++;
      }

      DebugSerial.print("Added ramp: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    } else {
      DebugSerial.print("Did NOT add: ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    }

  } else {
    DebugSerial.println("Lines are beeng filled");
    if (currentDir == 0) {
      for (int i = currentNode + ROTATION_DIRS[currentDir] - (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i <= GRID_SIZE - 1 + currentNode + ROTATION_DIRS[currentDir] - (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i += 1) {
        seenNodes[seenNodesCount] = i;
        seenNodesCount++;
        seenNodes[seenNodesCount] = i + 153;
        seenNodesCount++;

        redLines[redLinesCount] = i;
        redLinesCount++;
        redLines[redLinesCount] = i + 153;
        redLinesCount++;

        DebugSerial.print("Nodes: ");
        DebugSerial.print(i);
        DebugSerial.print(" ");
        DebugSerial.println(i + 153);
      }
    } else if (currentDir == 1) {  // Vertical
      for (int i = (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i < MAX_NODES; i += GRID_SIZE) {
        seenNodes[seenNodesCount] = i;
        seenNodesCount++;
        seenNodes[seenNodesCount] = i - 9;
        seenNodesCount++;

        redLines[redLinesCount] = i;
        redLinesCount++;
        redLines[redLinesCount] = i - 9;
        redLinesCount++;

        DebugSerial.print("Nodes: ");
        DebugSerial.print(i - 9);
        DebugSerial.print(" ");
        DebugSerial.println(i);
      }
    } else if (currentDir == 2) {
      for (int i = currentNode + ROTATION_DIRS[currentDir] - (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i < GRID_SIZE - 1 + currentNode + ROTATION_DIRS[currentDir] - (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i += 1) {
        seenNodes[seenNodesCount] = i;
        seenNodesCount++;
        seenNodes[seenNodesCount] = i - 153;
        seenNodesCount++;

        redLines[redLinesCount] = i;
        redLinesCount++;
        redLines[redLinesCount] = i - 153;
        redLinesCount++;


        DebugSerial.print("Nodes: ");
        DebugSerial.print(i - 153);
        DebugSerial.print(" ");
        DebugSerial.println(i);
      }
    } else if (currentDir == 3) {  // Vertical
      for (int i = (currentNode + ROTATION_DIRS[currentDir]) % GRID_SIZE; i < MAX_NODES; i += GRID_SIZE) {
        seenNodes[seenNodesCount] = i;
        seenNodesCount++;
        seenNodes[seenNodesCount] = i + 9;
        seenNodesCount++;

        redLines[redLinesCount] = i;
        redLinesCount++;
        redLines[redLinesCount] = i + 9;
        redLinesCount++;

        DebugSerial.print("Nodes: ");
        DebugSerial.print(i);
        DebugSerial.print(" ");
        DebugSerial.println(i + 9);
      }
    }
    DebugSerial.println("lines added");
  }
}

void scanOneCellFast() {
  // Scan only directions that were not seen
  ArraysLengthType leftDir = (currentDir + 3) % 4;
  ArraysLengthType backDir = (currentDir + 2) % 4;
  ArraysLengthType rightDir = (currentDir + 1) % 4;

  // check if we need to check the front
  if (!inArray(seenNodes, seenNodesCount, currentNode + ROTATION_DIRS[currentDir])) {
    // Did not visit front node
    doScan();
  } else if (((currentFloor == 0 && !inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[currentDir]))
              || (currentFloor == 1 && inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[currentDir])))
             && !inArray(redLines, redLinesCount, currentNode + ROTATION_DIRS[currentDir])
             && !inArray(redPosts, redPostsCount, currentNode + ROTATION_DIRS[currentDir])
             && !inArray(greenPosts, greenPostsCount, currentNode + ROTATION_DIRS[currentDir])
             && !inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[currentDir])) {
    g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], 1);
  }
  // check if we need to check the left
  if (!inArray(seenNodes, seenNodesCount, currentNode + ROTATION_DIRS[leftDir])) {
    // Did not visit left node
    // Turn to left node
    turnToAngle(currentDir, leftDir, currentFloor);
    delay(10);
    currentDir = leftDir;

    doScan();
  } else if (((currentFloor == 0 && !inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[leftDir]))
              || (currentFloor == 1 && inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[leftDir])))
             && !inArray(redLines, redLinesCount, currentNode + ROTATION_DIRS[leftDir])
             && !inArray(redPosts, redPostsCount, currentNode + ROTATION_DIRS[leftDir])
             && !inArray(greenPosts, greenPostsCount, currentNode + ROTATION_DIRS[leftDir])
             && !inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[leftDir])) {
    g.addNode(currentNode, currentNode + ROTATION_DIRS[leftDir], 1);
  }

  // check if we need to check the right
  if (!inArray(seenNodes, seenNodesCount, currentNode + ROTATION_DIRS[backDir])) {
    // Did not visit right node
    // Turn to right node
    turnToAngle(currentDir, backDir, currentFloor);
    delay(10);
    currentDir = backDir;
    doScan();
  }

  // check if we need to check the right
  if (!inArray(seenNodes, seenNodesCount, currentNode + ROTATION_DIRS[rightDir])) {
    // Did not visit right node
    // Turn to right node
    turnToAngle(currentDir, rightDir, currentFloor);
    delay(10);
    currentDir = rightDir;
    doScan();
  } else if (((currentFloor == 0 && !inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[rightDir]))
              || (currentFloor == 1 && inArray(secondFloor, secondFloorCount, currentNode + ROTATION_DIRS[rightDir])))
             && !inArray(redLines, redLinesCount, currentNode + ROTATION_DIRS[rightDir])
             && !inArray(redPosts, redPostsCount, currentNode + ROTATION_DIRS[rightDir])
             && !inArray(greenPosts, greenPostsCount, currentNode + ROTATION_DIRS[rightDir])
             && !inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[rightDir])) {
    g.addNode(currentNode, currentNode + ROTATION_DIRS[rightDir], 1);
  }
}

void travelToNextCellOptimized() {
  // DebugSerial.println("Seen nodes: ");
  // printArray(seenNodes, seenNodesCount);
  // DebugSerial.println("Unvisited nodes: ");
  // printArray(unvisitedNodes, unvisitedNodesCount);
  DebugSerial.println("STARTED-TRAVEL=============================================================");
  g.calculateDistances(currentNode, distances, predecessors);

  ArraysLengthType goingTo = 0;
  ArraysLengthType minPath = 65;
  for (int i = 0; i < unvisitedNodesCount; ++i) {
    ArraysLengthType pathLength = g.returnPath(currentNode, unvisitedNodes[i], predecessors, path);
    if (minPath > pathLength && pathLength != 1) {
      goingTo = i;
      minPath = pathLength;
    }
  }

  DebugSerial.print("Going to: ");
  DebugSerial.println(unvisitedNodes[goingTo]);
  // DebugSerial.print(" Path length: ");
  // DebugSerial.println(minPath);

  ArraysLengthType pathLength = g.returnPath(currentNode, unvisitedNodes[goingTo], predecessors, path);

  //-Following-path-

  ArraysLengthType i = 0;
  while (i < pathLength - 1) {
    // DebugSerial.print("Going from ");
    // DebugSerial.print(path[i]);
    // DebugSerial.print(" to ");
    // DebugSerial.println(path[i + 1]);

    turnToAngle(currentDir, getIndex(ROTATION_DIRS, 4, path[i + 1] - path[i]), currentFloor);
    currentDir = getIndex(ROTATION_DIRS, 4, path[i + 1] - path[i]);


    if (inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[currentDir])) {
      currentNode += 2 * ROTATION_DIRS[currentDir];
      i++;
      if (currentFloor) down_level();
      else up_level();

      currentFloor = currentFloor ? 0 : 1;
    } else {
      if (currentFloor) {
        currentNode += ROTATION_DIRS[currentDir];
        follow_line_up_level();
      } else {
        uint16_t count = 0;
        while (i < pathLength - 1 && path[i + 1] - path[i] == ROTATION_DIRS[currentDir] && !inArray(ramps, rampsCount, path[i + 1])) {
          count++;
          i++;
        }
        i--;
        currentNode += count * ROTATION_DIRS[currentDir];
        follow_cross_fast(count);
      }
    }

    DebugSerial.print("Moved to ");
    DebugSerial.println(currentNode);
    if (currentNode == path[pathLength - 1]) {
      break;
    }
    ++i;
  }
  removeFromArray(unvisitedNodes, unvisitedNodesCount, goingTo);
  unvisitedNodesCount -= 1;
  DebugSerial.println("ENDED-TRAVEL=============================================================");
}

void scanOneCell() {
  for (int i = 0; i < 3; ++i) {
    if (!receive(&data)) {
      DebugSerial.println("-ERROR-check-connection--------------------------");
    } else {
      DebugSerial.print("Recieved: ");
      DebugSerial.print(data.red);
      DebugSerial.print(" -> ");
      DebugSerial.print(data.line);
      DebugSerial.print(" -> ");
      DebugSerial.print(data.green);
      DebugSerial.print(" -> ");
      DebugSerial.print(data.blue);
      DebugSerial.print(" -> ");
      DebugSerial.println(data.floor);
    }

    if (!data.line) {
      if ((data.blue != 0 && data.red != 0 && data.floor == 0) || (data.floor == currentFloor && data.blue == 0 && data.red != 1)) {
        if (g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], (data.blue > 0) ? 3 : 1)) {
          DebugSerial.print("added ");
          DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);

          if (data.green != 0) {
            greenPosts[greenPostsCount] = currentNode + ROTATION_DIRS[currentDir];
            greenPostsCount++;
            DebugSerial.println("Found green");
          } else if (data.red > 1 && data.blue == 0) {
            redPosts[redPostsCount] = currentNode + ROTATION_DIRS[currentDir];
            redPostsCount++;
            DebugSerial.println("Found red");
          } else if (data.red > 1 && data.blue >= 1) {
            g.addNode(currentNode + ROTATION_DIRS[currentDir], currentNode + 2 * ROTATION_DIRS[currentDir], 1);

            ramps[rampsCount] = currentNode + ROTATION_DIRS[currentDir];
            rampsCount++;

            unvisitedNodes[unvisitedNodesCount] = currentNode + 2 * ROTATION_DIRS[currentDir];
            unvisitedNodesCount++;

            DebugSerial.println("Found ramp");
          } else {
            unvisitedNodes[unvisitedNodesCount] = currentNode + ROTATION_DIRS[currentDir];
            unvisitedNodesCount++;
          }
        } else {
          DebugSerial.print("Did not add ");
          DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
        }
      } else {
        DebugSerial.print("Did not add ");
        DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
      }
    } else {
      DebugSerial.print("Did not add ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    }

    turn_right(currentFloor ? -1 : 1);

    currentDir = (currentDir + 1) % 4;
  }

  if (!receive(&data)) {
    DebugSerial.println("ERROR----------------------------");
  } else {
    DebugSerial.print(data.red);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.line);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.green);
    DebugSerial.print(" -> ");
    DebugSerial.print(data.blue);
    DebugSerial.print(" -> ");
    DebugSerial.println(data.floor);
  }

  if (!data.line) {
    if ((data.blue != 0 && data.red != 0 && data.floor == 0) || (data.floor == currentFloor && data.blue == 0 && data.red != 1)) {
      if (g.addNode(currentNode, currentNode + ROTATION_DIRS[currentDir], (data.blue > 0) ? 3 : 1)) {
        DebugSerial.print("added ");
        DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);

        if (data.green != 0) {
          greenPosts[greenPostsCount] = currentNode + ROTATION_DIRS[currentDir];
          greenPostsCount++;
          DebugSerial.println("Found green");
        } else if (data.red > 1 && data.blue == 0) {
          redPosts[redPostsCount] = currentNode + ROTATION_DIRS[currentDir];
          redPostsCount++;
          DebugSerial.println("Found red");
        } else if (data.red > 1 && data.blue >= 1) {
          g.addNode(currentNode + ROTATION_DIRS[currentDir], currentNode + 2 * ROTATION_DIRS[currentDir], 1);

          ramps[rampsCount] = currentNode + ROTATION_DIRS[currentDir];
          rampsCount++;

          unvisitedNodes[unvisitedNodesCount] = currentNode + 2 * ROTATION_DIRS[currentDir];
          unvisitedNodesCount++;

          DebugSerial.println("Found ramp");
        } else {
          unvisitedNodes[unvisitedNodesCount] = currentNode + ROTATION_DIRS[currentDir];
          unvisitedNodesCount++;
        }
      } else {
        DebugSerial.print("Did not add ");
        DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
      }
    } else {
      DebugSerial.print("Did not add ");
      DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
    }
  } else {
    DebugSerial.print("Did not add ");
    DebugSerial.println(currentNode + ROTATION_DIRS[currentDir]);
  }

  // printArray(unvisitedNodes, unvisitedNodesCount);
  // g.printAllNodes();
}

void travelToNextCell() {
  // move to the other cell
  //-Calculate-shortest-paths-from-node-0----------------------------------------------------------

  DebugSerial.print("Going to: ");
  DebugSerial.println(unvisitedNodes[unvisitedNodesCount - 1]);

  //-Calculate-the-path-to-the-next-avalible-cell--------------------------------------------------
  g.calculateDistances(currentNode, distances, predecessors);
  ArraysLengthType pathLength = g.returnPath(currentNode, unvisitedNodes[unvisitedNodesCount - 1], predecessors, path);

  //-Print-found-path-and-its-length----------------------------------------------------------------
  DebugSerial.println(pathLength);
  for (int i = 0; i < pathLength; ++i) {
    DebugSerial.print(path[i]);
    DebugSerial.print(" -> ");
  }

  ArraysLengthType i = 0;
  while (i < pathLength - 1) {
    DebugSerial.print("Going from ");
    DebugSerial.print(path[i]);
    DebugSerial.print(" to ");
    DebugSerial.println(path[i + 1]);

    while (ROTATION_DIRS[currentDir] != path[i + 1] - path[i]) {  // Match needed rotation
      ArraysLengthType targetDir = getIndex(ROTATION_DIRS, 4, path[i + 1] - path[i]);
      DebugSerial.print("Current: ");
      DebugSerial.print(currentDir);
      DebugSerial.print(" Target: ");
      DebugSerial.print(targetDir);
      DebugSerial.print(" Delta: ");
      DebugSerial.print((ArraysLengthType)(targetDir - currentDir));
      DebugSerial.print(" %: ");
      DebugSerial.println((ArraysLengthType)(targetDir - currentDir) % 4);

      if ((ArraysLengthType)(targetDir - currentDir) % 4 == 3) {
        turn_left(currentFloor ? -1 : 1);
        currentDir = (currentDir + 3) % 4;
      } else {
        turn_right(currentFloor ? -1 : 1);
        currentDir = (currentDir + 1) % 4;
      }
    }

    if (inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[currentDir])) {
      currentNode += 2 * ROTATION_DIRS[currentDir];
      i++;
      if (currentFloor) down_level();
      else up_level();
    } else {
      currentNode += ROTATION_DIRS[currentDir];
      currentFloor ? follow_cross(1, -1) : follow_cross_fast(1);
    }

    // MoveOneSell();  //Move forward
    DebugSerial.print("Moved to ");
    DebugSerial.println(currentNode);
    if (currentNode == path[pathLength - 1]) {
      removeFromArray(unvisitedNodes, unvisitedNodesCount, 0);
      unvisitedNodesCount -= 1;
      break;
    }

    ++i;
  }
}

void travelPath(ArraysLengthType start, ArraysLengthType finish) {
  DebugSerial.println("STARTED-TUBES-TRAVEL=============================================================");
  DebugSerial.print("RED: ");
  printArray(redPosts, redPostsCount);
  DebugSerial.print("GREEN: ");
  printArray(greenPosts, greenPostsCount);
  // DebugSerial.print(" GREEN: ");
  g.calculateDistances(start, distances, predecessors);

  DebugSerial.print("From ");
  DebugSerial.print(start);
  DebugSerial.print(" to ");
  DebugSerial.println(finish);
  ArraysLengthType pathLength = g.returnPath(start, finish, predecessors, path);
  //-Following-path-
  DebugSerial.print("Path length: ");
  DebugSerial.println(pathLength);

  ArraysLengthType i = 0;
  while (i < pathLength - 1) {
    DebugSerial.print("Going from ");
    DebugSerial.print(path[i]);
    DebugSerial.print(" to ");
    DebugSerial.println(path[i + 1]);

    turnToAngle(currentDir, getIndex(ROTATION_DIRS, 4, path[i + 1] - path[i]), currentFloor);
    currentDir = getIndex(ROTATION_DIRS, 4, path[i + 1] - path[i]);

    if (path[i + 1] == finish) break;

    if (inArray(ramps, rampsCount, currentNode + ROTATION_DIRS[currentDir])) {
      currentNode += 2 * ROTATION_DIRS[currentDir];
      i++;
      if (currentFloor) down_level();
      else up_level();

      currentFloor = currentFloor ? 0 : 1;
    } else {
      // currentNode += ROTATION_DIRS[currentDir];
      // currentFloor ? follow_cross(1, -1) : follow_cross_fast(1);
      if (currentFloor) {
        currentNode += ROTATION_DIRS[currentDir];
        follow_line_up_level();
      } else {
        uint16_t count = 0;
        while (i < pathLength - 1 && path[i + 1] - path[i] == ROTATION_DIRS[currentDir] && !inArray(ramps, rampsCount, path[i + 1]) && path[i + 1] != finish) {
          count++;
          i++;
        }
        i--;
        currentNode += count * ROTATION_DIRS[currentDir];
        follow_cross_fast(count);
      }
    }

    DebugSerial.print("Moved to ");
    DebugSerial.println(currentNode);
    if (currentNode == path[pathLength - 1]) {
      break;
    }
    ++i;
  }
  DebugSerial.println("ENDED-TRAVEL=============================================================");
}

void simpleTubeCarry() {
  travelPath(currentNode, redPosts[0]);

  // delay(1000);
  take(1);
  currentDir = (currentDir + 2) % 4;
  travelPath(currentNode, greenPosts[0]);
  put(1);
  currentDir = (currentDir + 2) % 4;

  travelPath(currentNode, redPosts[1]);
  take(1);
  currentDir = (currentDir + 2) % 4;
  travelPath(currentNode, greenPosts[1]);
  put(1);
  currentDir = (currentDir + 2) % 4;
}

bool go = false, flag = false;
void setup() {
  //-----------------init----------------------------------------------------------------------------------
  DebugSerial.begin("ESP32");
  CameraSerial.begin(19200);
  DebugSerial.println("-------------------------------Started----------------------------------------");
  RobotSetup();
  seenNodes[seenNodesCount] = currentNode;
  seenNodesCount++;
  // follow_cross_fast(1);
}

void loop() {
  if (!digitalRead(32) and !flag) {
    flag = true;
  }
  if (digitalRead(32) and flag) {
    flag = false;
    go = true;
  }
  if (go) {
    currentFloor = (check_level() == 1) ? 0 : 1;
    delay(1000);
    // follow_line_up_level();
    for (int i = 0; i < 64; i++) {
      scanOneCellFast();
      travelToNextCellOptimized();
    }
    // simpleTubeCarry();
    // doScan();
    // scanOneCellFast();

    go = false;
  }
}

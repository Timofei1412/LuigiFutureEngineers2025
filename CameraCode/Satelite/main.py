from turtle import st
from collections import defaultdict

# from unittest import skip
import cv2
import numpy as np
import heapq

from itertools import permutations
import time
import math


cam = cv2.VideoCapture(3)


SIZE_RECT = 15

#stream = cv2.VideoCapture("rtsp://192.168.0.86:8554/live", cv2.CAP_ANY)
#ret, frame = stream.read()
#stream.release()
# ret, frame = cam.read()
frame = cv2.imread("Photo.png")

width = 600
height = 500
dim = (width, height)

# resize image
frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)


src_points = np.float32([[141, 14], [460, 21],[132, 490] , [462, 490]])
# # Destination points for the output image
dst_points = np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])
# # Compute the perspective transform matrix
matrix = cv2.getPerspectiveTransform(src_points, dst_points)
# # Apply the perspective transformation to the image
transformed_image = cv2.warpPerspective(frame, matrix, (500, 500))
cv2.imshow("transform", transformed_image)

imageHSV = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2HSV)

# transform_matrix = cv2.getPerspectiveTransform(corners_points, field_square)
# transformed_frame = cv2.warpPerspective(frame, transform_matrix, (700, 700))

# def rotate_image(image, angle):
#   image_center = tuple(np.array(image.shape[1::-1]) / 2)
#   rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
#   result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
#   return result

def getBlobsCoords(tresholdImage):
    
    contours, _ = cv2.findContours(tresholdImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centers = []
    
    for contour in contours:
        # Calculate contour moments
        M = cv2.moments(contour)
        
        # Skip if the area is too small
        if M['m00'] < 1:  # Adjust this threshold as needed
            continue
            
        # Calculate centroid
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            continue
            
        # Calculate circularity
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
        else:
            circularity = 0

        x,y,w,h = cv2.boundingRect(contour)
        # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            
        # orientation =  "Horizontal" if () else "Vertical"
        orientation = "Vertical" if  (w > h) else "Horizontal"
        # Only accept circular shapes
        if circularity > 0.2:  # Adjust this threshold as needed
            centers.append(((cx, cy), orientation))
        # box = np.int0(box)
        # print(math.acos(vy))        
    return (centers)

#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------

def findBlue(image, hsvFrame):
    B_low=np.array([98,50,50])
    B_high=np.array([139,255,255]) 

    b_mask=cv2.inRange(hsvFrame,B_low, B_high)

    blue=cv2.bitwise_and(image,image,mask=b_mask)

    kernel = np.ones((5, 5), np.uint8)
    blue = cv2.erode(blue, kernel)
    blue = cv2.dilate(blue, kernel)

    blueBGR = cv2.cvtColor(blue, cv2.COLOR_HSV2BGR)
    blueGRAY = cv2.cvtColor(blueBGR, cv2.COLOR_BGR2GRAY)
    ret, threshed = cv2.threshold(blueGRAY, 0, 255, cv2.THRESH_BINARY)
    
    centers = getBlobsCoords(threshed)

    print("Blue")
    for i in centers:
        print(i[0])
    print("--------------------------------------")

    cv2.imshow('Blue treshed Detector', threshed) 
    return centers

def findGreen(image, hsvFrame):
    G_low=np.array([50,90,100]) # setting the blue lower limit
    G_high=np.array([80 ,255,255]) # setting the blue upper limit

    g_mask=cv2.inRange(hsvFrame,G_low,G_high)
    # creating the mask using inRange() function
    # this will produce an image where the color of the objects
    # falling in the range will turn white and rest will be black
    green = cv2.bitwise_and(image,image,mask=g_mask)
    kernel = np.ones((5, 5), np.uint8)
    green = cv2.erode(green, kernel)
    green = cv2.dilate(green, kernel)

    greenBGR = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)
    greenGRAY = cv2.cvtColor(greenBGR, cv2.COLOR_BGR2GRAY)
    ret, threshed = cv2.threshold(greenGRAY, 0, 255, cv2.THRESH_BINARY)
    
    centers = getBlobsCoords(threshed)

    print("Green")
    for i in centers:
        print(i[0])
    print("--------------------------------------")

    cv2.imshow('Green Detector', threshed) # to display the blue object output
    
    return centers

def findRed(image, hsvFrame):
    R_low=np.array([0,60,50]) # setting the blue lower limit

    R_high=np.array([9,255,255]) # setting the blue upper limit


    R_low1=np.array([140,60,50]) # setting the blue lower limit
    R_high1=np.array([179,255,255]) # setting the blue upper limit

    r_mask = cv2.bitwise_or(cv2.inRange(hsvFrame,R_low,R_high), cv2.inRange(hsvFrame,R_low1,R_high1))
    # r_mask = (cv2.inRange(hsvFrame,R_low1,R_high1))
    # creating the mask using inRange() function
    # this will produce an image where the color of the objects
    # falling in the range will turn white and rest will be black
    red = cv2.bitwise_and(image,image,mask=r_mask)

    kernel = np.ones((5, 5), np.uint8)
    red = cv2.erode(red, kernel)
    red = cv2.dilate(red, kernel)

    redBGR = cv2.cvtColor(red, cv2.COLOR_HSV2BGR)
    redGRAY = cv2.cvtColor(redBGR, cv2.COLOR_BGR2GRAY)
    ret, threshed = cv2.threshold(redGRAY, 0, 255, cv2.THRESH_BINARY)
    # cutRed = fill_edge_blobs(threshed)
    
    centers = getBlobsCoords(threshed)

    cv2.imshow('Threshed red', threshed) # to display the blue object output
    # cv2.imshow('red Detector', cutRed) # to display the blue object output
    print("Red")
    for i in centers:
        print(i)
    print("--------------------------------------")

    return centers

def findRobot(image, hsvFrame):
    O_low=np.array([9,100,120]) # setting the blue lower limit
    O_high=np.array([21,255,255]) # setting the blue upper limit

    o_mask=cv2.inRange(hsvFrame, O_low, O_high)
    # creating the mask using inRange() function
    # this will produce an image where the color of the objects
    # falling in the range will turn white and rest will be black
    orange = cv2.bitwise_and(image,image,mask=o_mask)
    kernel = np.ones((5, 5), np.uint8)
    orange = cv2.erode(orange, kernel)
    orange = cv2.dilate(orange, kernel)

    orangeBGR = cv2.cvtColor(orange, cv2.COLOR_HSV2BGR)
    orangeGRAY = cv2.cvtColor(orangeBGR, cv2.COLOR_BGR2GRAY)
    ret, threshed = cv2.threshold(orangeGRAY, 0, 255, cv2.THRESH_BINARY)
    
    centers = getBlobsCoords(threshed)

    print("Orange")
    for i in centers:
        print(i)
    print("--------------------------------------")
    
    cv2.imshow('Robot Detector', threshed) # to display the blue object output
    
    return centers

#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------

def count_points_in_square(points, square):
    if isinstance(square, tuple) and len(square) == 4:
        x_min, x_max, y_min, y_max = square
    elif 'center' in square:
        center_x, center_y = square['center']
        half_side = square['side_length'] / 2
        x_min = center_x - half_side
        x_max = center_x + half_side
        y_min = center_y - half_side
        y_max = center_y + half_side
    else:  # assume min/max format
        x_min = square.get('x_min', square.get('left'))
        x_max = square.get('x_max', square.get('right'))
        y_min = square.get('y_min', square.get('bottom'))
        y_max = square.get('y_max', square.get('top'))
    
    points_inside = []
    for point in points:
        x, y = point[0]
        if x_min <= x <= x_max and y_min <= y <= y_max:
            points_inside.append(point)
    
    if not points_inside:
        return None
    if len(points_inside) == 1:
        return points_inside[0]
    
    # Calculate average point
    avg_x = sum(p[0][0] for p in points_inside) / len(points_inside)
    avg_y = sum(p[0][1] for p in points_inside) / len(points_inside)
    return ((avg_x, avg_y), points_inside[0][1] if points_inside[0][1] is not None else None)

greenPostesLocal = []
redPostesLocal = []
preGraph = []
robotCoords = ()
graph = [[None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
         [None, None, None, None, None, None, None, None],
        ]

def getLevels(image, hsvFrame):
    black = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, threshed = cv2.threshold(black, 120, 255, cv2.THRESH_BINARY)
    
    
    blue = findBlue(image, hsvFrame)
    green = findGreen(image, hsvFrame)
    robot = findRobot(image, hsvFrame)
    red = findRed(image, hsvFrame)
    
    cv2.line(image, (62 , 0), (61, 500), (0, 200, 200),  2)
    cv2.line(image, (123, 0), (123, 500), (0, 200, 200), 2)
    cv2.line(image, (187, 0), (187, 500), (0, 200, 200), 2)
    cv2.line(image, (250, 0), (250, 500), (0, 200, 200), 2)
    cv2.line(image, (312, 0), (312, 500), (0, 200, 200), 2)
    cv2.line(image, (375, 0), (375, 500), (0, 200, 200), 2)
    cv2.line(image, (440, 0), (440, 500), (0, 200, 200), 2)

    cv2.line(image, (0, 62), (500, 62), (0, 200, 200),   2)
    cv2.line(image, (0, 123), (500, 123), (0, 200, 200), 2)
    cv2.line(image, (0, 187), (500, 187), (0, 200, 200), 2)
    cv2.line(image, (0, 250), (500, 250), (0, 200, 200), 2)
    cv2.line(image, (0, 312), (500, 312), (0, 200, 200), 2)
    cv2.line(image, (0, 375), (500, 375), (0, 200, 200), 2)
    cv2.line(image, (0, 440), (500, 440), (0, 200, 200), 2)

    for i in range(8):
        for j in range(8):
            middle = (30 + j * 63, 24 + i * 63)
            xVal = 15 + j * 63
            yVal = 12 + i * 63
            height = 0 if threshed[yVal][xVal] > 100 else 1

            square1 = {'center': middle, 'side_length': 63}
            reds = count_points_in_square(red, square1)
            blues = count_points_in_square(blue, square1)
            greens = count_points_in_square(green, square1)
            robots = count_points_in_square(robot, square1)

            isAnRamp = 1 if (blues is not None) else 0

            rampDirection = None
            # find dx and dy and use that to calculate direction
            if blues is not None:
                dxRamp = abs(int(blues[0][0]) - middle[0])
                dyRamp = abs(int(blues[0][1]) - middle[1])

                if dxRamp < dyRamp:
                    rampDirection = 'Horizontal'
                else:
                    rampDirection = 'Vertical'
            
            redPost = 0
            redDirection = None
            if blues is None and reds is not None and robots is None:
                redPost = 1
                redDirection = reds[1]
            
            redPostesLocal.append((j, i))


            greenPost = 1 if greens is not None else 0
            greenPostesLocal.append(greens)
            # find dx and dy and use that to calculate direction
            
            robotPos = 1 if robots is not None else 0
            robotDirection = "up"
            if robotPos:
                print("ROBOT: ", j, i, " ----------------------------------------------------------")
                global robotCoords 
                robotCoords = (j, i)


            preGraph.append((height, isAnRamp, rampDirection, greenPost, robotPos, redPost, redDirection))
            cv2.circle(image, (xVal, yVal), (7 if height else 3), (0, 150, 200), -1 if not isAnRamp else 2) 
            if not greenPost:
                cv2.circle(image, (xVal, yVal), (7 if height else 3), (0, 150, 200), -1 if not isAnRamp else 2) 
            else:
                cv2.rectangle(image, (xVal-4, yVal-4),(xVal+4, yVal+4), (0, 150, 200), -1 if not isAnRamp else 2) 

            # cv2.imshow("Levels", image)
            # time.sleep(1)
    
    cv2.imshow("Levels", image)
    cv2.imshow("TreshedL", threshed)
    
    for i in preGraph:
        print(i)

    for i in range(8):
        for j in range(8):
            data = preGraph[i * 8 + j]
            listOfLines = []

            dataleft = None
            coordLeft = i * 8 + j - 1
            if coordLeft >= 0 and coordLeft < 64:
                dataleft = preGraph[coordLeft]

            dataRight = None
            coordRight = i * 8 + j + 1
            if coordRight >= 0 and coordRight< 64:
                dataRight = preGraph[coordRight]

            dataUp = None
            coordUp = (i - 1) * 8 + j
            if coordUp >= 0 and coordUp < 64:
                dataUp = preGraph[coordUp]

            dataDown = None
            coordUp = (i + 1) * 8 + j
            if coordUp >= 0 and coordUp < 64:
                dataDown = preGraph[coordUp]
            
            if dataleft is not None:
                if (dataleft[0] == data[0] and (dataleft[5] == 0 or (dataleft[5] == 1  and dataleft[6] == "Horizontal"))) or (dataleft[1] == 1 and dataleft[2] == "Horizontal") or (data[1] == 1 and data[2] == "Horizontal") : #if heights are the same
                    listOfLines.append(((i, j - 1), 3 if dataleft[1] else 1))

            if dataRight is not None:
                if (dataRight[0] == data[0] and (dataRight[5] == 0 or (dataRight[5] == 1  and dataRight[6] == "Horizontal"))) or (dataRight[1] == 1 and dataRight[2] == "Horizontal")  or (data[1] == 1 and data[2] == "Horizontal"): #if heights are the same
                    listOfLines.append(((i, j + 1), 3 if dataRight[1] else 1))
                    
            if dataUp is not None:
                if (dataUp[0] == data[0] and (dataUp[5] == 0 or (dataUp[5] == 1  and dataUp[6] == "Vertical"))) or (dataUp[1] == 1 and dataUp[2] == "Vertical")  or (data[1] == 1 and data[2] == "Vertical"): #if heights are the same
                    listOfLines.append(((i - 1, j), 3 if dataUp[1] else 1))
                    
            if dataDown is not None:
                if (dataDown[0] == data[0] and (dataDown[5] == 0 or (dataDown[5] == 1  and dataDown[6] == "Vertical"))) or (dataDown[1] == 1 and dataDown[2] == "Vertical")  or (data[1] == 1 and data[2] == "Vertical"): #if heights are the same
                    listOfLines.append(((i + 1, j), 3 if dataDown[1] else 1))
                    

            graph[i][j] = listOfLines

    for index, i in enumerate(graph):
        for index2, j in enumerate(i):
            print(index, "", index2," :", j)
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------

    # https://www.geeksforgeeks.org/dijkstras-algorithm-for-adjacency-list-representation-greedy-algo-8/
    

def dijkstra(graph, start):
    # Initialize distances with infinity for all nodes except start
    rows = len(graph)
    cols = len(graph[0]) if rows > 0 else 0
    distances = {(i, j): float('infinity') for i in range(rows) for j in range(cols)}
    distances[start] = 0
    
    # Priority queue: (distance, x, y)
    priority_queue = [(0, start[0], start[1])]
    
    # To keep track of visited nodes
    visited = set()
    
    # To reconstruct paths
    previous = {}
    
    while priority_queue:
        current_dist, x, y = heapq.heappop(priority_queue)
        current_node = (x, y)
        
        # Skip if we've already found a better path
        if current_dist > distances[current_node]:
            continue
            
        visited.add(current_node)
        
        # Explore neighbors
        for neighbor, weight in graph[x][y]:
            if neighbor not in distances:  # Skip if neighbor is out of bounds
                continue
                
            distance = current_dist + weight
            
            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor[0], neighbor[1]))
    
    return distances, previous

def reconstruct_path(previous, start, end):
    """
    Reconstruct the shortest path from start to end using the previous dict.
    
    Args:
        previous: Dictionary from dijkstra's algorithm
        start: Starting coordinates (x, y)
        end: Destination coordinates (x, y)
    
    Returns:
        List of coordinates representing the path from start to end
    """
    path = []
    current = end
    
    while current != start:
        path.append(current)
        if current not in previous:  # No path exists
            return None
        current = previous[current]
    
    path.append(start)
    path.reverse()
    return path
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------








# hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR)
# findBlue(transformed_image, imageHSV)
# findGreen(transformed_image, imageHSV)
# findRed(transformed_image, imageHSV)
# findRobot(transformed_image, imageHSV)
getLevels(transformed_image, imageHSV)
print(robotCoords)

distances, previous = dijkstra(graph, robotCoords)

end = redPostesLocal[0]
path = reconstruct_path(previous, robotCoords, end)

print(path)


# cv2.imshow("NAME", frame)
# cv2.moveWindow(window_name, window_origin[0], window_origin[1])
cv2.waitKey(0)
cv2.destroyAllWindows()
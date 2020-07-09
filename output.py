import cv2
import json
import math
import numpy as np

from datetime import datetime
from numpy import linalg as LA
from scipy.spatial import Delaunay


# https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
def computeBarycentricCoordinate(triangle, point):
    ab = np.subtract(triangle[1], triangle[0])
    ac = np.subtract(triangle[2], triangle[0])
    ap = np.subtract(point, triangle[0])
    
    #print("Triangle:\n", triangle)
    #print("Point:\n", point)
    
    #print("ab:\n", ab)
    #print("ac:\n", ac)
    #print("ap:\n", ap)
    
    ab2 = np.dot(ab, ab)
    abac = np.dot(ab, ac)
    ac2 = np.dot(ac, ac)
    apab = np.dot(ap, ab)
    apac = np.dot(ap, ac)
    
    denom = ab2 * ac2 - abac * abac
    
    #print("ab2:\n", ab2)
    #print("abac:\n", abac)
    #print("ac2:\n", ac2)
    #print("apab:\n", apab)
    #print("apac:\n", apac)
    
    v = (ac2 * apab - abac * apac) / denom
    w = (ab2 * apac - abac * apab) / denom
    
    #print()
    
    return [v, w, 1 - v - w]

def computeTriangleNormal(triangleVertices, triangleNormals):
    a = np.subtract(triangleVertices[0], triangleVertices[1])
    b = np.subtract(triangleVertices[1], triangleVertices[2])
    normal = np.cross(a,b)
    normal = normal/LA.norm(normal)
    dot = np.dot(normal, triangleNormals[0])
    if dot < 0:
        return -normal
    return normal

def computeTriangleCentroid(triangleVertices):
    sumX = 0
    sumY = 0
    sumZ = 0
    for v in triangleVertices:
        sumX = sumX + v[0]
        sumY = sumY + v[1]
        sumZ = sumZ + v[2]
    return [sumX/3, sumY/3, sumZ/3]
    
def quaternionBetweenVector(a, b):
    axb = np.cross(a,b)
    w = math.sqrt((LA.norm(a)**2) * (LA.norm(b)**2) + np.dot(a,b))
    q = np.append(w, axb)
    return q/LA.norm(q)

# https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
def rotatePoint(point, rotation):
    rot_vec = rotation[1:]
    rot_sca = rotation[0]
    return 2 * np.dot(rot_vec, point) * rot_vec + (rot_sca**2 - np.dot(rot_vec, rot_vec) * point + 2 * rot_sca * np.cross(rot_vec, point))

def rotatePointAroundPivot(point, rotation, pivot):
    return np.add(rotatePoint(np.subtract(point, pivot), rotation), pivot)
    
def triangleRasterization(triangleUVs, triangleColors, colorMap, normalMap):
    minX, minY, maxX, maxY = computeBoundingBox(triangleUVs)
    inTri = []
    inTriBary = []
    
    for x in range(int(minX), int(maxX) + 1):
        for y in range(int(minY), int(maxY) + 1):
            xInd = x
            yInd = y
            if xInd >= resolution:
                xInd = resolution - 1
            if yInd >= resolution:
                yInd = resolution - 1
            bc = computeBarycentricCoordinate(triangleUVs, [xInd, yInd])
            #print("Barycentric Coordinate:\n", bc)
            if bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0 and np.array_equal(colorMap[xInd,yInd], [0,0,0,0]):
                inTri.append([xInd, yInd])
                inTriBary.append(bc)
    
    #print("In triangle pixel count:\n", len(inTri))
    #print()
    for i in range(len(inTri)):
        color = (np.multiply(inTriBary[i][0], triangleColors[0]) + np.multiply(inTriBary[i][1], triangleColors[1]) + np.multiply(inTriBary[i][2], triangleColors[2])).tolist()
        color.append(255)
        colorMap[inTri[i][0], inTri[i][1]] = [color[2], color[1], color[0], color[3]]
        
def triangleSplitting(triangle, triangleColors, points, pointColors):
    triangles = []
    triangles.append(triangle)
    trianglesColors = []
    trianglesColors.append(triangleColors)
    for i in range(len(points)):
        #print("i:\n", i)
        for j in range(len(triangles)):
            #print("j:\n", j)
            tri = triangles[j]
            triColor = trianglesColors[j]
            point = points[i]
            pointColor = pointColors[i]
            if isPointInTriangle(triangle, point):
                # Create 3 new triangles
                tri1 = [tri[0], tri[1], point]
                tri2 = [tri[1], tri[2], point]
                tri3 = [tri[2], tri[0], point]
                triangles.remove(tri)
                triangles.extend([tri1, tri2, tri3])
                triC1 = [triColor[0], triColor[1], pointColor]
                triC2 = [triColor[1], triColor[2], pointColor]
                triC3 = [triColor[2], triColor[0], pointColor]
                trianglesColors.extend([triC1, triC2, triC3])
                trianglesColors.remove(triColor)
                break
    #print()
    return triangles, trianglesColors
    
def triangulation(points, pointColors):
    delaunay = Delaunay(points)
    triangles = []
    trianglesColors = []
    for triangleIndices in delaunay.simplices:
        triangle = []
        triangleColors = []
        for i in triangleIndices:
            triangle.append(points[i])
            triangleColors.append(pointColors[i])
        triangles.append(triangle)
        trianglesColors.append(triangleColors)
    return triangles, trianglesColors
    
            
def isPointInTriangle(triangle, point):
    bc = computeBarycentricCoordinate(triangle, point)
    return bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0
    
def computeBoundingBox(vertices):
    minX = float("inf")
    minY = float("inf")
    maxX = float("-inf")
    maxY = float("-inf")
    for v in vertices:
        if v[0] < minX:
            minX = v[0]
        if v[1] < minY:
            minY = v[1]
        if v[0] > maxX:
            maxX = v[0]
        if v[1] > maxY:
            maxY = v[1]
    return minX, minY, maxX, maxY


# Main
# For recording time
totalDataParseTime = 0
totalRotationTime = 0
totalDirectRasterizationTime = 0
totalPointsRotationTime = 0
totalPointsInTriangleTime = 0
totalTriangulationTime = 0
totalRasterizationTime = 0

totalStartTime = datetime.now()

# Create empty maps
resolution = 8192
colorMap = np.zeros((resolution, resolution, 4), np.uint8)
normalMap = np.zeros((resolution, resolution, 4), np.uint8)

# Read JSON from file
jsonReadStartTime = datetime.now()

jsonFilename = "pointsMesh.json"
with open(jsonFilename) as f:
    data = json.load(f)["data"]

totalDataParseTime += (datetime.now() - jsonReadStartTime).total_seconds()

for t in data:

    dataParseStartTime = datetime.now()

    triangle = t["triangle"]
    points = t["points"]

    triangleVertices = []
    triangleNormals = []
    triangleColors = []
    triangleUVs = []
    for tri in triangle:
        attributes = [float(s) for s in (tri["v"].split())]
        triangleVertices.append([attributes[0], attributes[1], attributes[2]])
        triangleNormals.append([attributes[3], attributes[4], attributes[5]])
        triangleColors.append([attributes[6], attributes[7], attributes[8]])
        triangleUVs.append(np.multiply([1 - attributes[10], attributes[9]], resolution))
    #print("Triangle Vertices:\n", triangleVertices)
    #print("Triangle UVs:\n", triangleUVs)
    #print("Triangle Colors:\n", triangleColors)
       
    pointsCoord = []
    pointsNormal = []
    pointsColor = []
    for p in points:
        attributes = [float(s) for s in (p["p"].split())]
        pointsCoord.append([attributes[0], attributes[1], attributes[2]])
        pointsNormal.append([attributes[3], attributes[4], attributes[5]])
        pointsColor.append([attributes[6], attributes[7], attributes[8]])
    #print("Points Coords:\n", pointsCoord)
    
    totalDataParseTime += (datetime.now() - dataParseStartTime).total_seconds()
    
    rotateTriangleStartTime = datetime.now()
    
    # Rotate Triangle
    normal = computeTriangleNormal(triangleVertices, triangleNormals)
    centroid = computeTriangleCentroid(triangleVertices);
    up = [0,1,0]
    rotation = quaternionBetweenVector(normal, up)
    rotatedTriangle = []
    for v in triangleVertices:
        rotated = rotatePointAroundPivot(v, rotation, centroid)
        rotatedTriangle.append([rotated[0], rotated[2]])
    #print("Rotated Triangle:\n", rotatedTriangle)
    
    totalRotationTime += (datetime.now() - rotateTriangleStartTime).total_seconds()
    
    rasterizationStartTime = datetime.now()
    
    # Rasterization
    if len(points) == 0:
        directRasterizationStartTime = datetime.now()
    
        triangleRasterization(triangleUVs, triangleColors, colorMap, normalMap)
        
        totalDirectRasterizationTime += (datetime.now() - directRasterizationStartTime).total_seconds()
    else:
    
        pointsRotationStartTime = datetime.now()
        
        # Rotate points
        rotatedPts = []
        rotatedPtsColors = []
        rotatedPtsNormals = []
        for i in range(len(pointsCoord)):
            p = pointsCoord[i]
            rotated = rotatePointAroundPivot(p, rotation, centroid)
            rotatedPts.append([rotated[0], rotated[2]])
            rotatedPtsColors.append(pointsColor[i])
            rotatedPtsNormals.append(pointsNormal[i])
            
        totalPointsRotationTime += (datetime.now() - pointsRotationStartTime).total_seconds()
        
        pointsInTriangleStartTime = datetime.now()
        
        # Obtain points in triangle
        ptsInTriImgCoords = []
        ptsInTriColors = []
        ptsInTriNormals = []
        for i in range(len(rotatedPts)):
            p = rotatedPts[i]
            bc = computeBarycentricCoordinate(rotatedTriangle, p)
            if bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0:
                x = bc[0] * triangleUVs[0][0] + bc[1] * triangleUVs[1][0] + bc[2] * triangleUVs[2][0]
                y = bc[0] * triangleUVs[0][1] + bc[1] * triangleUVs[1][1] + bc[2] * triangleUVs[2][1]
                ptsInTriImgCoords.append([x,y])
                ptsInTriColors.append(rotatedPtsColors[i])
                ptsInTriNormals.append(rotatedPtsNormals[i])
                
        totalPointsInTriangleTime += (datetime.now() - pointsInTriangleStartTime).total_seconds()
        
        triangulationStartTime = datetime.now()
        
        # Triangulation
        triangulationPts = []
        triangulationPts.extend(triangleUVs)
        triangulationPts.extend(ptsInTriImgCoords)
        triangulationPtsColors = []
        triangulationPtsColors.extend(triangleColors)
        triangulationPtsColors.extend(ptsInTriColors)
        triangulated, trianglatedColors = triangulation(triangulationPts, triangulationPtsColors)
        #print("Triangulated Length:\n", len(triangulated))
        #print("Triangulated Colors Length:\n", len(trianglatedColors))
        for t in range(len(triangulated)):
            # Ceiling and cap to resolution
            tInt = np.ceil(triangulated[t])
            #print("triangle UVs:\n", triangleUVs)
            #print("tInt at t:\n", tInt, t)
            for i in range(len(tInt)):
                for j in range(len(tInt[i])):
                    if tInt[i][j] >= resolution:
                        tInt[i][j] = resolution - 1
            triangleRasterization(tInt, trianglatedColors[t], colorMap, normalMap)
        #print()
        
        totalTriangulationTime += (datetime.now() - triangulationStartTime).total_seconds()
        
    totalRasterizationTime += (datetime.now() - rasterizationStartTime).total_seconds()
        
print("Total Time: ", (datetime.now() - totalStartTime).total_seconds())
print("Total Data Parse Time: ", totalDataParseTime)
print("Total Rotation Time: ", totalRotationTime)
print("Total Direct Rasterization Time: ", totalDirectRasterizationTime)
print("Total Points Rotation Time: ", totalPointsRotationTime)
print("Total Points in Triangle Time: ", totalPointsInTriangleTime)
print("Total Triangulation Time: ", totalTriangulationTime)
print("Total Rasterization Time: ", totalRasterizationTime)
        
# Save as PNG
cv2.imwrite("colorMap.png", colorMap)
cv2.imwrite("normalMap.png", normalMap)

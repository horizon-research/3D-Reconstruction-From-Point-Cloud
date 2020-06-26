import cv2
import json
import math
import numpy as np
from numpy import linalg as LA

# https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
def computeBarycentricCoordinate(triangle, point):
    ab = np.subtract(triangle[1], triangle[0])
    ac = np.subtract(triangle[2], triangle[0])
    ap = np.subtract(point - triangle[0])
    
    ab2 = np.dot(ab, ab)
    abac = np.dot(ab, ac)
    ac2 = np.dot(ac, ac)
    apab = np.dot(ap, ab)
    apac = np.dot(ap, ac)
    
    denom = ab2 * ac2 - abac * abac
    
    v = (ac2 * apab - abac * apac) / denom
    w = (ab2 * apac - abac * apab) / denom
    
    return [v, w, 1 - v - w]

def computeTriangleCentroid(triangle):
    sumX = 0
    sumY = 0
    sumZ = 0
    for v in triangle:
        sumX = sumX + v["x"]
        sumY = sumY + v["y"]
        sumZ = sumZ + v["z"]
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
    
def triangleRasterization(verticesUV, colors, colorMap, normalMap):
    minX, minY, maxX, maxY = computeBoundingBox(vertices)
    inTri = []
    inTriBary = []
    
    for x in range(minX, maxX):
        for y in range(minY, maxY):
            xInd = x
            yInd = y
            if xInd >= resolution:
                xInd = resolution - 1
            if yInd >= resolution:
                yInd = resolution - 1
            bc = computeBarycentricCoordinate(verticesUV, [xInd, yInd])
            if bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0 && colorMap[xInd][yInd] == (0,0,0,0):
                inTri.append([xInd, yInd])
                inTriBary.append(bc)
    
    for i in range(len(inTri)):
        color = [np.multiply(inTriBary[i][0], colors[0]), np.multiply(inTriBary[i][1], colors[1]), np.multiply(inTriBary[i][2], colors[2])]
        colorMap[inTri[i][0]][inTri[i][1]] = (color[0], color[1], color[2], 255)
        
def triangleSplitting(triangle, triangleColors, points, pointColors):
    triangles = []
    triangles.append(triangle)
    triColors = []
    triangleColors.append(triangleColors)
    for i in range(len(points)):
        for j in range(len(triangles)):
            tri = triangles[j]
            triColor = triangleColors[j]
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
                triangleColors.remove(triColor)
                triangleColors.extend([triC1, triC2, triC3])
                break
    return triangles, triangleColors
            
def isPointInTriangle(triangle, point):
    bc = computeBarycentricCoordinate(triangle, point)
    return bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0
    
def computeBoundingBox(vertices):
    minX = inf
    minY = inf
    maxX = -inf
    maxY = -inf
    for v in vertices:
        if vertices[0] < minX:
            minX = vertices[0]
        if vertices[1] < minY:
            minY = vertices[1]
        if vertices[0] > maxX:
            maxX = vertices[0]
        if vertices[1] > maxY:
            maxY = vertices[1]
    return minX, minY, maxX, maxY


# Main
# Create empty maps
resolution = 8192
colorMap = np.zeros((resolution, resolution, 4), np.uint8)
normalMap = np.zeros((resolution, resolution, 4), np.uint8)

# Read JSON from file
jsonFilename = "test.json"
with open(jsonFilename) as f:
    data = json.load(f)

triangle = data["mesh"]
points = data["points"]

triangleUVs = []
triangleColors = []
for v in triangle:
    triangleUVs.append([v["u"], v["t"]])
    triangleColors.append([v["b"], v["g"], v["r"]])

# Rotate Triangle and Points
normal = [triangle[0]["nx"], triangle[0]["ny"], triangle[2]["nx"]]
centroid = computeTriangleCentroid(triangle);
up = [0,1,0]
rotation = quaternionBetweenVector(normal, up)
rotatedTriangle = []
for v in triangle:
    vertex = [v["x"], v["y"], v["z"]]
    rotated = rotatePointAroundPivot(vertex, rotation, centroid)
    rotatedTriangle.append([rotated[0], rotated[2]])

# Rasterization
if len(points) == 0:
    triangleRasterization(triangleUVs, triangleColors, colorMap, normalMap)
else:
    # Rotate points
    rotatedPts = []
    for point in points:
        p = [point["x"], point["y"], point["z"]]
        rotated = rotatePointAroundPivot(p, rotation, centroid)
        rotatedPts.append([rotated[0], rotated[2]])
    # Obtain points in triangle
    pointsInTriImgCoords = []
    for p in rotatedPts:
        bc = computeBarycentricCoordinate(rotatedTriangle, p)
        if bc[0] >= 0 and bc[1] >= 0 and bc[2] >= 0:
            x = bc[0] * triangleUVs[0][0] + bc[1] * triangleUVs[1][0] + bc[2] * triangleUVs[2][0]
            y = bc[0] * triangleUVs[0][1] + bc[1] * triangleUVs[1][1] + bc[2] * triangleUVs[2][1]
            pointsInTriImgCoords.append([x,y])
    # Triangulation
    triangulated, trianglatedColors = triangleSplitting()
    for t in range(len(triangulated)):
        # Ceiling and cap to resolution
        tInt = np.ceil(t)
        for i in range(len(tInt)):
            for j in range(len(tInt[i])):
                if tInt[i][j] >= resolution:
                    tInt[i][j] = resolution - 1
        triangleRasterization(tInt, trianglatedColors[t], colorMap, normalMap)
        

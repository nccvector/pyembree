import numpy
import matplotlib.pyplot as plt
from pyembree import *
numpy.set_printoptions(suppress=True)

IMAGE_WIDTH, IMAGE_HEIGHT = 640, 480
ASPECT_RATIO = float(IMAGE_WIDTH) / float(IMAGE_HEIGHT)
VERTICAL_FOV = 45.0
HORIZONTAL_FOV = ASPECT_RATIO * VERTICAL_FOV


def MorphIntoImage(data):
    dataImage = data.reshape(IMAGE_HEIGHT, IMAGE_WIDTH, 3)
    print("Morphed into image: {}".format(dataImage.shape))
    return dataImage


def CreateCameraMatrix(image_width, image_height, vertical_fov, horizontal_fov):
    # Compute focal lengths in pixels
    fx = image_width / (2 * numpy.tan(numpy.radians(horizontal_fov) / 2))
    fy = image_height / (2 * numpy.tan(numpy.radians(vertical_fov) / 2))

    # Principal point (usually the center of the image)
    cx = (image_width - 1) / 2
    cy = (image_height - 1) / 2

    # Camera matrix
    K = numpy.array(
        [
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ]
    )

    return K


def GetPixelCoordinates(image_width, image_height):
    # Generate grid of coordinates
    y_coords, x_coords = numpy.meshgrid(numpy.arange(image_height), numpy.arange(image_width), indexing='ij')

    # Stack the coordinates to get (x, y) pairs
    pixel_coords = numpy.vstack((x_coords.ravel(), y_coords.ravel())).T

    return pixel_coords


def GetCameraRayDirections(cameraMatrixKInverse, pixelCoordinates):
    hPixelCoordinates = numpy.hstack([pixelCoordinates, numpy.ones_like(pixelCoordinates[:, 0][:, numpy.newaxis])])
    rays = hPixelCoordinates.dot(cameraMatrixKInverse.T)
    raysNorms = numpy.linalg.norm(rays, axis=1)
    rays[:, 0] /= raysNorms
    rays[:, 1] /= raysNorms
    rays[:, 2] /= raysNorms

    return rays


device = RTCDevice()
scene = RTCScene(device)

vertices = numpy.array([
    [-1, -1, 0],
    [1, -1, 0],
    [1, 1, 0],
    [-1, 1, 0]
], dtype=numpy.float32)

indices = numpy.array([
    [0, 1, 2],
    [0, 2, 3],
])

scene.CreateNewGeometry(vertices, indices)
scene.Commit()

K = CreateCameraMatrix(IMAGE_WIDTH, IMAGE_HEIGHT, VERTICAL_FOV, HORIZONTAL_FOV)
print("Camera Matrix K: {}".format(K))

Kinv = numpy.linalg.inv(K)
print("Camera Matrix inverse Kinv: {}".format(Kinv))

pixelCoords = GetPixelCoordinates(IMAGE_WIDTH, IMAGE_HEIGHT)
print("Pixel coords: {}".format(pixelCoords))
rays = numpy.zeros((pixelCoords.shape[0], 6), dtype=numpy.float)
rays[:, 2] = -1.0
rays[:, 3:] = GetCameraRayDirections(Kinv, pixelCoords)
print("Rays: {}".format(rays))


timeNearFars, normals, uvs, idMaskFlags = scene.CastRays(rays)
timeNearFars = numpy.array(timeNearFars)
normals = numpy.array(normals)
uvs = numpy.array(uvs)
idMaskFlags = numpy.array(idMaskFlags)
print("CastRays:\nGeom IDs: {}\nNormals: {}\nUVs: {}".format(idMaskFlags[:, 4] != RTC_INVALID_GEOMETRY_ID, normals, uvs))

uvsImage = MorphIntoImage(numpy.hstack((uvs, numpy.zeros_like(uvs[:, 0])[:, numpy.newaxis])))
print(uvsImage.shape)
plt.imshow(uvsImage)
plt.show()

normalsImage = MorphIntoImage(normals)
plt.imshow(normalsImage)
plt.show()

from IPython import embed; embed()

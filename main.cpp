#include <embree4/rtcore.h>
#include <cstdio>
#include <limits>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

void errorFunction(void *userPtr, enum RTCError error, const char *str) {
    printf("error %d: %s\n", error, str);
}

template<class T>
py::array_t<T> CreateNumpyArray(const std::vector<T> &vector) {
    return py::array_t<T>({(ssize_t) vector.size()}, vector.data());
}

struct RTC_ALIGN(16) Py_RTCRay {
    float org_x;        // x coordinate of ray origin
    float org_y;        // y coordinate of ray origin
    float org_z;        // z coordinate of ray origin

    float dir_x;        // x coordinate of ray direction
    float dir_y;        // y coordinate of ray direction
    float dir_z;        // z coordinate of ray direction

    float tnear;        // start of ray segment
    float tfar;         // end of ray segment (set to hit distance)

    float time;
    unsigned int mask;  // ray mask
    unsigned int id;    // ray ID
    unsigned int flags; // ray flags
};

/* Hit structure for a single ray */
struct RTC_ALIGN(16) Py_RTCHit {
    float Ng_x;          // x coordinate of geometry normal
    float Ng_y;          // y coordinate of geometry normal
    float Ng_z;          // z coordinate of geometry normal

    float u;             // barycentric u coordinate of hit
    float v;             // barycentric v coordinate of hit

    unsigned int primID; // primitive ID
    unsigned int geomID; // geometry ID
    std::array<unsigned int, RTC_MAX_INSTANCE_LEVEL_COUNT> instID; // instance ID
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    std::array<unsigned int, RTC_MAX_INSTANCE_LEVEL_COUNT> instPrimID; // instance primitive ID
#endif
};

struct Py_RTCRayHit {
    struct Py_RTCRay ray;
    struct Py_RTCHit hit;
};

struct RTC_ALIGN(16) Py_RTCRay4 {
    std::array<float, 4> org_x;
    std::array<float, 4> org_y;
    std::array<float, 4> org_z;

    std::array<float, 4> dir_x;
    std::array<float, 4> dir_y;
    std::array<float, 4> dir_z;

    std::array<float, 4> tnear;
    std::array<float, 4> tfar;

    std::array<float, 4> time;
    std::array<unsigned int, 4> mask;
    std::array<unsigned int, 4> id;
    std::array<unsigned int, 4> flags;
};

struct RTC_ALIGN(16) Py_RTCHit4 {
    std::array<float, 4> Ng_x;
    std::array<float, 4> Ng_y;
    std::array<float, 4> Ng_z;

    std::array<float, 4> u;
    std::array<float, 4> v;

    std::array<unsigned int, 4> primID;
    std::array<unsigned int, 4> geomID;
    std::array<std::array<unsigned int, 4>, RTC_MAX_INSTANCE_LEVEL_COUNT> instID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    std::array<std::array<unsigned int, 4>, RTC_MAX_INSTANCE_LEVEL_COUNT> instPrimID;
#endif
};

struct Py_RTCRayHit4 {
    struct Py_RTCRay4 ray;
    struct Py_RTCHit4 hit;
};

struct RTC_ALIGN(16) Py_RTCRay8 {
    std::array<float, 8> org_x;
    std::array<float, 8> org_y;
    std::array<float, 8> org_z;

    std::array<float, 8> dir_x;
    std::array<float, 8> dir_y;
    std::array<float, 8> dir_z;

    std::array<float, 8> tnear;
    std::array<float, 8> tfar;

    std::array<float, 8> time;
    std::array<unsigned int, 8> mask;
    std::array<unsigned int, 8> id;
    std::array<unsigned int, 8> flags;
};

struct RTC_ALIGN(16) Py_RTCHit8 {
    std::array<float, 8> Ng_x;
    std::array<float, 8> Ng_y;
    std::array<float, 8> Ng_z;

    std::array<float, 8> u;
    std::array<float, 8> v;

    std::array<unsigned int, 8> primID;
    std::array<unsigned int, 8> geomID;
    std::array<std::array<unsigned int, 8>, RTC_MAX_INSTANCE_LEVEL_COUNT> instID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    std::array<std::array<unsigned int, 8>, RTC_MAX_INSTANCE_LEVEL_COUNT> instPrimID;
#endif
};

struct Py_RTCRayHit8 {
    struct Py_RTCRay8 ray;
    struct Py_RTCHit8 hit;
};

struct RTC_ALIGN(16) Py_RTCRay16 {
    std::array<float, 16> org_x;
    std::array<float, 16> org_y;
    std::array<float, 16> org_z;

    std::array<float, 16> dir_x;
    std::array<float, 16> dir_y;
    std::array<float, 16> dir_z;

    std::array<float, 16> tnear;
    std::array<float, 16> tfar;

    std::array<float, 16> time;
    std::array<unsigned int, 16> mask;
    std::array<unsigned int, 16> id;
    std::array<unsigned int, 16> flags;
};

struct RTC_ALIGN(16) Py_RTCHit16 {
    std::array<float, 16> Ng_x;
    std::array<float, 16> Ng_y;
    std::array<float, 16> Ng_z;

    std::array<float, 16> u;
    std::array<float, 16> v;

    std::array<unsigned int, 16> primID;
    std::array<unsigned int, 16> geomID;
    std::array<std::array<unsigned int, 16>, RTC_MAX_INSTANCE_LEVEL_COUNT> instID;
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    std::array<std::array<unsigned int, 16>, RTC_MAX_INSTANCE_LEVEL_COUNT> instPrimID;
#endif
};

struct Py_RTCRayHit16 {
    struct Py_RTCRay16 ray;
    struct Py_RTCHit16 hit;
};

class Py_RTCDevice {
public:
    Py_RTCDevice() {
        rtcDevice = rtcNewDevice(nullptr);
        if (!rtcDevice)
            printf("error %d: cannot create device\n", rtcGetDeviceError(nullptr));

        rtcSetDeviceErrorFunction(rtcDevice, errorFunction, nullptr);
    }

    RTCDevice rtcDevice;
};

class Py_RTCScene {
public:
    Py_RTCScene(Py_RTCDevice py_device) {
        rtcDevice = py_device.rtcDevice;
        rtcScene = rtcNewScene(py_device.rtcDevice);
    }

    void CreateNewGeometry(py::array_t<float> vertices, py::array_t<int> indices) {
        py::buffer_info vertexBuffInfo = vertices.request();
        float *verticesPtr = static_cast<float *>(vertexBuffInfo.ptr);
        size_t verticesRows = vertexBuffInfo.shape[0];
        size_t verticesCols = vertexBuffInfo.shape[1];

        py::buffer_info indexBuffInfo = indices.request();
        unsigned *indicesPtr = static_cast<unsigned *>(indexBuffInfo.ptr);
        size_t indicesRows = indexBuffInfo.shape[0];
        size_t indicesCols = indexBuffInfo.shape[1];

        RTCGeometry geom = rtcNewGeometry(rtcDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

        float *mappedVertices = (float *) rtcSetNewGeometryBuffer(
            geom,
            RTC_BUFFER_TYPE_VERTEX,
            0,
            RTC_FORMAT_FLOAT3,
            verticesCols * sizeof(float),
            verticesRows
        );

        unsigned *mappedIndices = (unsigned *) rtcSetNewGeometryBuffer(
            geom,
            RTC_BUFFER_TYPE_INDEX,
            0,
            RTC_FORMAT_UINT3,
            indicesCols * sizeof(unsigned),
            indicesRows
        );

        for (size_t i = 0; i < verticesRows; i++) {
            for (size_t j = 0; j < verticesCols; j++) {
                mappedVertices[i * verticesCols + j] = verticesPtr[i * verticesCols + j];
            }
        }

        for (size_t i = 0; i < indicesRows; i++) {
            for (size_t j = 0; j < indicesCols; j++) {
                mappedIndices[i * indicesCols + j] = indicesPtr[i * indicesCols + j];
            }
        }

        rtcCommitGeometry(geom);
        rtcAttachGeometry(rtcScene, geom);
        rtcReleaseGeometry(geom);
    }

    void CommitScene() {
        rtcCommitScene(rtcScene);
    }

    std::tuple<
        py::array_t<float, 6>,  // ray
        py::array_t<float, 3>,  // time, near, far
        py::array_t<unsigned int, 5>   // mask, flags, primID, geomID, instID
    >
    CastRays() {
        return {};
    }


    std::tuple<
        py::array_t<float>,  // time, near, far
        py::array_t<float>,  // hit normal
        py::array_t<float>,  // hit uv
        py::array_t<unsigned int>   // id, mask, flags, primID, geomID, instID
    >
    CastRay(py::array_t<float, 6> ray) {
        py::buffer_info rayBuffInfo = ray.request();
        float *rayPtr = static_cast<float *>(rayBuffInfo.ptr);

        struct RTCRayHit rayhit;
        rayhit.ray.org_x = rayPtr[0];
        rayhit.ray.org_y = rayPtr[1];
        rayhit.ray.org_z = rayPtr[2];
        rayhit.ray.dir_x = rayPtr[3];
        rayhit.ray.dir_y = rayPtr[4];
        rayhit.ray.dir_z = rayPtr[5];
        rayhit.ray.tnear = 0;
        rayhit.ray.tfar = std::numeric_limits<float>::infinity();
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        rtcIntersect1((RTCScene) rtcScene, &rayhit);

        printf("%f, %f, %f: ", rayPtr[0], rayPtr[1], rayPtr[2]);
        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
            printf(
                "Found intersection on geometry %d, primitive %d at tfar=%f\n",
                rayhit.hit.geomID,
                rayhit.hit.primID,
                rayhit.ray.tfar
            );
        } else
            printf("Did not find any intersection.\n");

        py::array_t<float> timeNearFar = CreateNumpyArray<float>(
            {
                rayhit.ray.time,
                rayhit.ray.tnear,
                rayhit.ray.tfar
            }
        );

        py::array_t<float> hitNormal = CreateNumpyArray<float>(
            {
                rayhit.hit.Ng_x,
                rayhit.hit.Ng_y,
                rayhit.hit.Ng_z,
            }
        );

        py::array_t<float> hitUV = CreateNumpyArray<float>(
            {
                rayhit.hit.u,
                rayhit.hit.v,
            }
        );

        py::array_t<unsigned int> idMaskFlag = CreateNumpyArray<unsigned int>(
            {
                rayhit.ray.id,
                rayhit.ray.mask,
                rayhit.ray.flags,
                rayhit.hit.primID,
                rayhit.hit.geomID,
                rayhit.hit.instID[0]
            }
        );

        return std::make_tuple(
            timeNearFar,
            hitNormal,
            hitUV,
            idMaskFlag
        );
    }

    RTCScene rtcScene;
    RTCDevice rtcDevice;
};

PYBIND11_MODULE(pyembree, m) {
    m.doc() = "Intel Embree Python Bindings"; // optional module docstring

    pybind11::enum_<RTCGeometryType>(m, "RTCGeometryType")
        .value("RTC_GEOMETRY_TYPE_TRIANGLE", RTC_GEOMETRY_TYPE_TRIANGLE)
        .value("RTC_GEOMETRY_TYPE_QUAD", RTC_GEOMETRY_TYPE_QUAD)
        .value("RTC_GEOMETRY_TYPE_GRID", RTC_GEOMETRY_TYPE_GRID)
        .value("RTC_GEOMETRY_TYPE_SUBDIVISION", RTC_GEOMETRY_TYPE_SUBDIVISION)
        .value("RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE", RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE)
        .value("RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE", RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE", RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE)
        .value("RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE", RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE", RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE)
        .value("RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE)
        .value("RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE", RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE", RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE)
        .value("RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)
        .value("RTC_GEOMETRY_TYPE_SPHERE_POINT", RTC_GEOMETRY_TYPE_SPHERE_POINT)
        .value("RTC_GEOMETRY_TYPE_DISC_POINT", RTC_GEOMETRY_TYPE_DISC_POINT)
        .value("RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT", RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT)
        .value("RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE)
        .value("RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE)
        .value(
            "RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE", RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE
        )
        .value("RTC_GEOMETRY_TYPE_USER", RTC_GEOMETRY_TYPE_USER)
        .value("RTC_GEOMETRY_TYPE_INSTANCE", RTC_GEOMETRY_TYPE_INSTANCE)
        .value("RTC_GEOMETRY_TYPE_INSTANCE_ARRAY", RTC_GEOMETRY_TYPE_INSTANCE_ARRAY)
        .export_values();

    pybind11::enum_<RTCBufferType>(m, "RTCBufferType")
        .value("RTC_BUFFER_TYPE_INDEX", RTC_BUFFER_TYPE_INDEX)
        .value("RTC_BUFFER_TYPE_VERTEX", RTC_BUFFER_TYPE_VERTEX)
        .value("RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE", RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
        .value("RTC_BUFFER_TYPE_NORMAL", RTC_BUFFER_TYPE_NORMAL)
        .value("RTC_BUFFER_TYPE_TANGENT", RTC_BUFFER_TYPE_TANGENT)
        .value("RTC_BUFFER_TYPE_NORMAL_DERIVATIVE", RTC_BUFFER_TYPE_NORMAL_DERIVATIVE)
        .value("RTC_BUFFER_TYPE_GRID", RTC_BUFFER_TYPE_GRID)
        .value("RTC_BUFFER_TYPE_FACE", RTC_BUFFER_TYPE_FACE)
        .value("RTC_BUFFER_TYPE_LEVEL", RTC_BUFFER_TYPE_LEVEL)
        .value("RTC_BUFFER_TYPE_EDGE_CREASE_INDEX", RTC_BUFFER_TYPE_EDGE_CREASE_INDEX)
        .value("RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT", RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT)
        .value("RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX", RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX)
        .value("RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT", RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT)
        .value("RTC_BUFFER_TYPE_HOLE", RTC_BUFFER_TYPE_HOLE)
        .value("RTC_BUFFER_TYPE_TRANSFORM", RTC_BUFFER_TYPE_TRANSFORM)
        .value("RTC_BUFFER_TYPE_FLAGS", RTC_BUFFER_TYPE_FLAGS)
        .export_values();

    pybind11::enum_<RTCFormat>(m, "RTCFormat")
        .value("RTC_FORMAT_UNDEFINED", RTC_FORMAT_UNDEFINED)
        .value("RTC_FORMAT_UCHAR", RTC_FORMAT_UCHAR)
        .value("RTC_FORMAT_UCHAR2", RTC_FORMAT_UCHAR2)
        .value("RTC_FORMAT_UCHAR3", RTC_FORMAT_UCHAR3)
        .value("RTC_FORMAT_UCHAR4", RTC_FORMAT_UCHAR4)
        .value("RTC_FORMAT_CHAR", RTC_FORMAT_CHAR)
        .value("RTC_FORMAT_CHAR2", RTC_FORMAT_CHAR2)
        .value("RTC_FORMAT_CHAR3", RTC_FORMAT_CHAR3)
        .value("RTC_FORMAT_CHAR4", RTC_FORMAT_CHAR4)
        .value("RTC_FORMAT_USHORT", RTC_FORMAT_USHORT)
        .value("RTC_FORMAT_USHORT2", RTC_FORMAT_USHORT2)
        .value("RTC_FORMAT_USHORT3", RTC_FORMAT_USHORT3)
        .value("RTC_FORMAT_USHORT4", RTC_FORMAT_USHORT4)
        .value("RTC_FORMAT_SHORT", RTC_FORMAT_SHORT)
        .value("RTC_FORMAT_SHORT2", RTC_FORMAT_SHORT2)
        .value("RTC_FORMAT_SHORT3", RTC_FORMAT_SHORT3)
        .value("RTC_FORMAT_SHORT4", RTC_FORMAT_SHORT4)
        .value("RTC_FORMAT_UINT", RTC_FORMAT_UINT)
        .value("RTC_FORMAT_UINT2", RTC_FORMAT_UINT2)
        .value("RTC_FORMAT_UINT3", RTC_FORMAT_UINT3)
        .value("RTC_FORMAT_UINT4", RTC_FORMAT_UINT4)
        .value("RTC_FORMAT_INT", RTC_FORMAT_INT)
        .value("RTC_FORMAT_INT2", RTC_FORMAT_INT2)
        .value("RTC_FORMAT_INT3", RTC_FORMAT_INT3)
        .value("RTC_FORMAT_INT4", RTC_FORMAT_INT4)
        .value("RTC_FORMAT_ULLONG", RTC_FORMAT_ULLONG)
        .value("RTC_FORMAT_ULLONG2", RTC_FORMAT_ULLONG2)
        .value("RTC_FORMAT_ULLONG3", RTC_FORMAT_ULLONG3)
        .value("RTC_FORMAT_ULLONG4", RTC_FORMAT_ULLONG4)
        .value("RTC_FORMAT_LLONG", RTC_FORMAT_LLONG)
        .value("RTC_FORMAT_LLONG2", RTC_FORMAT_LLONG2)
        .value("RTC_FORMAT_LLONG3", RTC_FORMAT_LLONG3)
        .value("RTC_FORMAT_LLONG4", RTC_FORMAT_LLONG4)
        .value("RTC_FORMAT_FLOAT", RTC_FORMAT_FLOAT)
        .value("RTC_FORMAT_FLOAT2", RTC_FORMAT_FLOAT2)
        .value("RTC_FORMAT_FLOAT3", RTC_FORMAT_FLOAT3)
        .value("RTC_FORMAT_FLOAT4", RTC_FORMAT_FLOAT4)
        .value("RTC_FORMAT_FLOAT5", RTC_FORMAT_FLOAT5)
        .value("RTC_FORMAT_FLOAT6", RTC_FORMAT_FLOAT6)
        .value("RTC_FORMAT_FLOAT7", RTC_FORMAT_FLOAT7)
        .value("RTC_FORMAT_FLOAT8", RTC_FORMAT_FLOAT8)
        .value("RTC_FORMAT_FLOAT9", RTC_FORMAT_FLOAT9)
        .value("RTC_FORMAT_FLOAT10", RTC_FORMAT_FLOAT10)
        .value("RTC_FORMAT_FLOAT11", RTC_FORMAT_FLOAT11)
        .value("RTC_FORMAT_FLOAT12", RTC_FORMAT_FLOAT12)
        .value("RTC_FORMAT_FLOAT13", RTC_FORMAT_FLOAT13)
        .value("RTC_FORMAT_FLOAT14", RTC_FORMAT_FLOAT14)
        .value("RTC_FORMAT_FLOAT15", RTC_FORMAT_FLOAT15)
        .value("RTC_FORMAT_FLOAT16", RTC_FORMAT_FLOAT16)
        .value("RTC_FORMAT_FLOAT2X2_ROW_MAJOR", RTC_FORMAT_FLOAT2X2_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT2X3_ROW_MAJOR", RTC_FORMAT_FLOAT2X3_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT2X4_ROW_MAJOR", RTC_FORMAT_FLOAT2X4_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT3X2_ROW_MAJOR", RTC_FORMAT_FLOAT3X2_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT3X3_ROW_MAJOR", RTC_FORMAT_FLOAT3X3_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT3X4_ROW_MAJOR", RTC_FORMAT_FLOAT3X4_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT4X2_ROW_MAJOR", RTC_FORMAT_FLOAT4X2_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT4X3_ROW_MAJOR", RTC_FORMAT_FLOAT4X3_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT4X4_ROW_MAJOR", RTC_FORMAT_FLOAT4X4_ROW_MAJOR)
        .value("RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT2X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X4_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT3X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X2_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT4X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X2_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT4X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X3_COLUMN_MAJOR)
        .value("RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR)
        .value("RTC_FORMAT_GRID", RTC_FORMAT_GRID)
        .value("RTC_FORMAT_QUATERNION_DECOMPOSITION", RTC_FORMAT_QUATERNION_DECOMPOSITION)
        .export_values();


    pybind11::class_<Py_RTCDevice>(m, "RTCDevice")
        .def(pybind11::init<>());

    pybind11::class_<Py_RTCScene>(m, "RTCScene")
        .def(pybind11::init<Py_RTCDevice>())
        .def("CreateNewGeometry", &Py_RTCScene::CreateNewGeometry, "Function to create a new RTCScene")
        .def("Commit", &Py_RTCScene::CommitScene, "Commit the scene")
        .def("CastRay", &Py_RTCScene::CastRay, "Cast a ray in the scene");

    py::class_<Py_RTCRay>(m, "RTCRay")
        .def(py::init<>())
        .def_readwrite("org_x", &Py_RTCRay::org_x)
        .def_readwrite("org_y", &Py_RTCRay::org_y)
        .def_readwrite("org_z", &Py_RTCRay::org_z)
        .def_readwrite("dir_x", &Py_RTCRay::dir_x)
        .def_readwrite("dir_y", &Py_RTCRay::dir_y)
        .def_readwrite("dir_z", &Py_RTCRay::dir_z)
        .def_readwrite("tnear", &Py_RTCRay::tnear)
        .def_readwrite("tfar", &Py_RTCRay::tfar)
        .def_readwrite("time", &Py_RTCRay::time)
        .def_readwrite("mask", &Py_RTCRay::mask)
        .def_readwrite("id", &Py_RTCRay::id)
        .def_readwrite("flags", &Py_RTCRay::flags);

    py::class_<Py_RTCHit>(m, "RTCHit")
        .def(py::init<>())
        .def_readwrite("Ng_x", &Py_RTCHit::Ng_x)
        .def_readwrite("Ng_y", &Py_RTCHit::Ng_y)
        .def_readwrite("Ng_z", &Py_RTCHit::Ng_z)
        .def_readwrite("u", &Py_RTCHit::u)
        .def_readwrite("v", &Py_RTCHit::v)
        .def_readwrite("primID", &Py_RTCHit::primID)
        .def_readwrite("geomID", &Py_RTCHit::geomID)
        .def_readwrite("instID", &Py_RTCHit::instID);

    py::class_<Py_RTCRayHit>(m, "RTCRayHit")
        .def(py::init<>())
        .def_readwrite("ray", &Py_RTCRayHit::ray)
        .def_readwrite("hit", &Py_RTCRayHit::hit);
}


















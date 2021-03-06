///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#pragma once

class Camera;
class Shape;
class Intersection;
class Ray;
class Sphere;


extern const float PI;
const float Radians = PI / 180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    Vector3f Kd, Ks, Kt;
    float alpha, ior;
    unsigned int texid;

    float Pd, Pr, Pt;



    virtual bool isLight() { return false; }

    Material()  : Kd(Vector3f(1.0, 0.5, 0.0)), Ks(Vector3f(1,1,1)), alpha(1.0), texid(0) {}
    Material(const Vector3f d, const Vector3f s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    //Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }

    Material(const Material& rhs) :
        Kd(rhs.Kd), Ks(rhs.Ks), Kt(rhs.Kt), ior(rhs.ior), alpha(rhs.alpha), texid(rhs.texid), 
        Pd(rhs.Pd), Pr(rhs.Pr), Pt(rhs.Pt) {}

    Material(const Vector3f d, const Vector3f s, const float a, const Vector3f t, float ior);

    void setTexture(const std::string path);
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const Vector3f e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Scene


class Scene {
private:
    Vector3f MidVector(Vector3f ViewingDir, Vector3f LightDir) const;

    float ComputeIndexOfReflection(const Vector3f ViewDir, const Vector3f N, const Intersection& intersect) const;
    float VecProbTransmission(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, const Intersection& intersect) const;
    Vector3f ComputeBRDFTransmission(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, Intersection& intersect) const;

public:
    int width, height;
    Material* currentMat;
    Camera* camera;
    float RussianRoulette;
    Sphere* sphere;

    std::vector<Shape*> objects;
    std::vector<Shape*> lights;

    

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
        const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);

    Vector3f TracePath(const Ray& ray, KdBVH<float, 3, Shape*> Tree);


    Intersection TraceRay(const Ray& ray, const KdBVH<float, 3, Shape*>& Tree) const;

    Intersection TraceRayNoBVH(const Ray& ray) const;

    // tools
    Vector3f EvalRadiance(const Intersection& intersect) const;

    inline float PdfBrdf(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, const Intersection& intersect) const;

    Vector3f EvalScattering(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, Intersection& intersect, Intersection& it) const;

    Vector3f SampleBrdf(Vector3f ViewDir, Vector3f N, const Intersection& intersect) const;

    Vector3f SampleLobe(Vector3f N, float t1, float t2) const;

    Intersection SampleLight() const;

   
    
    float PdfLight(Intersection& intersect) const;

    void ExplicitLight(Vector3f ViewingDir, Vector3f &weight, Vector3f &color, Intersection& P, KdBVH<float, 3, Shape*> Tree) const;
    
};


float GeometryFactor(Intersection& A, Intersection& B);

float DistributionFunction(Vector3f m, const Intersection& intersect);

float DistributionPhong(Vector3f m, const Intersection& intersect);

float DistributionGGX(Vector3f m, const Intersection& intersect);

Vector3f Fresnel(float d, const Intersection& intersect);

// This is a part of Cook-Torrance
float GeometryFunction(Vector3f ViewingDir, Vector3f LightDir, Vector3f m, const Intersection& intersect);

float G1_Phong(Vector3f dir, Vector3f m, const Intersection& intersect);

float G1_GGX(Vector3f dir, Vector3f m, const Intersection& intersect);



void PrintVector(std::string str, Vector3f v);

//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"

#include "Camera.h"
#include "Obj.h"
#include "Minimizer.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "Timer.h"

#include <random>
extern std::mt19937_64 RNGen;
extern std::uniform_real_distribution<> myrandom;

#define NUM_PASS 100


Scene::Scene() 
    : RussianRoulette(0.44)
{ 
    camera = new Camera();
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    for (auto i : mesh->triangles) {
        objects.push_back(new Triangle(
            mesh->vertices[i[0]].pnt,
            mesh->vertices[i[1]].pnt,
            mesh->vertices[i[2]].pnt,
            mesh->vertices[i[0]].nrm,
            mesh->vertices[i[1]].nrm,
            mesh->vertices[i[2]].nrm,
            mesh->mat));
    }
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}



void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        camera->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
    }

    else if (c == "brdf")  {

        currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]); }

    else if (c == "light") {

        currentMat = new Light(Vector3f(f[1], f[2], f[3])); }
   
    else if (c == "sphere") {
        if (currentMat->isLight()) lights.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
        objects.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
    }

    else if (c == "box") {
        objects.push_back(new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat));
    }

    else if (c == "cylinder") {
        objects.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
    }


    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}


void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);
}




void Scene::TraceImage(Color* image, const int pass)
{
    KdBVH<float, 3, Shape*> Tree(objects.begin(), objects.end());

    for (int i = 0; i < NUM_PASS; ++i) 
    {
        std::cout << "Rendering " << i << "th pass" << std::endl;

        #pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) 
        {
            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; x++) 
            {
                const Ray ray = camera->generateRay(x, y, width, height);
                Color color(TracePath(ray, Tree) / NUM_PASS);

                if (Eigen::isinf(color).any()) continue;
                if (Eigen::isnan(color).any()) continue;
                
                image[y * width + x] += (color / NUM_PASS);
            }
        }
    }
}


Intersection Scene::TraceRay(const Ray& ray, const KdBVH<float, 3, Shape*> &Tree) const
{
    Minimizer minimizer(ray);

    BVMinimize(Tree, minimizer);

    // fill intersection base on Tracing result
    return minimizer.intersection;
}





Vector3f Scene::TracePath(const Ray& ray, KdBVH<float, 3, Shape*> Tree)
{
    Vector3f Color(0, 0, 0);
    Vector3f Weight(1, 1, 1);

    Intersection P = TraceRay(ray, Tree);   // inital ray

    if (!P.hasIntersection()) return Color;     // no intersection
    if (P.shape->mat->isLight()) return EvalRadiance(P);    // hit light source

    Vector3f ViewingDir = -ray.direction;       // preparation for reflection

    while (myrandom(RNGen) <= RussianRoulette)
    {     
        // Explicit Light Connection
        ExplicitLight(ViewingDir, Weight, Color, P, Tree);

        // Extend Path
        const Vector3f SampleDir = SampleBrdf(ViewingDir, P.normal, P).normalized();     // sampled direction

        Intersection Q = TraceRay(Ray(P.position, SampleDir), Tree); // Extend path
        if (!Q.hasIntersection()) break;

        float p = PdfBrdf(ViewingDir, P.normal, SampleDir, P);
        if (p < Epsilon) break;

        Weight = Weight.cwiseProduct(EvalScattering(ViewingDir, P.normal, SampleDir, P) / p);

        // Implicit Light Connection
        if (Q.shape->mat->isLight())    // Implicit light connection
        {      
            Color += 0.5f * EvalRadiance(Q).cwiseProduct(Weight);
            break;
        }

        // Step Forward
        P = Q;      // step forward
        ViewingDir = -SampleDir.normalized();
    }

    return Color;
}


Vector3f Scene::EvalScattering(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, Intersection& intersect) const
{
    Vector3f Kd = intersect.shape->mat->Kd;
    Vector3f DiffusePart = Kd / PI;     // Lambert

    Vector3f m = MidVector(ViewingDir, SampleDir);

    float D = DistributionFunction(m, intersect);
    float G = GeometryFunction(ViewingDir, SampleDir, m, intersect);
    Vector3f F = Fresnel(m.dot(SampleDir), intersect);

    float denominator = 4.0f * fabsf(ViewingDir.dot(N)) * fabsf(SampleDir.dot(N));

    Vector3f SpecularPart = D * G * F / denominator;    // Cook-Torrance
    
    return (DiffusePart + SpecularPart) * fabsf(N.dot(SampleDir));
}


// TODO
Vector3f Scene::SampleBrdf(Vector3f ViewDir, Vector3f N, const Intersection& intersect) const
{
    float t1 = myrandom(RNGen); // first unifromly distributed random number
    float t2 = myrandom(RNGen); // first unifromly distributed random number

    float random = myrandom(RNGen); // random possbility to choose over diffuse / reflection / transmission

    if (random < ProbChooseDiffuse(intersect)) // Diffuse
    {
        return SampleLobe(N, sqrtf(t1), 2.0f * PI * t2);
    }
    else if (random < ProbChooseSpecular(intersect))  // reflection    
    {
        // since D uses Phong Distribution
        float cosTheta = pow(t1, 1.0f / (intersect.shape->mat->alpha + 1.0f));

        Vector3f m = SampleLobe(N, cosTheta, 2.0f * PI * t2).normalized();
        //return 2.0f * ViewDir.dot(m) * m - ViewDir; // no transmission
        return 2.0f * fabsf(ViewDir.dot(m)) * m - ViewDir;
    }
    else     // transmission
    {
        float cosTheta = pow(t1, 1.0f / (intersect.shape->mat->alpha + 1.0f));

        // sample a normal in narrow lobe around N
        Vector3f m = SampleLobe(N, cosTheta, 2.0f * PI * t2).normalized();

        float IndexOfReflection = ComputeIndexOfReflection();
        float r = 1 - IndexOfReflection * IndexOfReflection * (1 - powf(ViewDir.dot(m), 2));

        if (r < 0) {

        }
        else {
            return 
        }
    }
}


Vector3f Scene::SampleLobe(Vector3f N, float t1, float t2) const
{
    float s = sqrtf(1 - t1 * t1);
    Vector3f K(s * cosf(t2), s * sinf(t2), t1);
    Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N);
    return q._transformVector(K);
}


Vector3f Scene::EvalRadiance(const Intersection& intersect) const 
{ 
    return intersect.shape->mat->Kd; 
}


Intersection Scene::SampleLight() const
{
    // Choose one light(uniformly) randomly.
    int index = myrandom(RNGen) * lights.size();

    // Choose a uniformly distributed point on the light, return intersection
    return static_cast<Sphere*>(lights[index])->SampleSphere();
}


float GeometryFactor(Intersection& A, Intersection& B)
{
    Vector3f D = A.position - B.position;
    float DdotD = D.dot(D);

    return fabsf(A.normal.dot(D) * B.normal.dot(D) / (DdotD * DdotD));
}


float Scene::PdfLight(Intersection& intersect) const
{
    Sphere *sphere = static_cast<Sphere*>(intersect.shape);     // all light sources are sphere
    float area = 4.0f * PI * sphere->radius * sphere->radius;      // area of sphere
    return 1.0f / (lights.size() * area);
}


bool compareVector(Vector3f a, Vector3f b)
{
    bool x = fabsf(a[0] - b[0]) < Epsilon;
    bool y = fabsf(a[1] - b[1]) < Epsilon;
    bool z = fabsf(a[2] - b[2]) < Epsilon;
    return x && y && z;
}


void Scene::ExplicitLight(Vector3f ViewingDir, Vector3f& weight, Vector3f& color, Intersection &P, KdBVH<float, 3, Shape*> Tree) const
{
    Intersection L = SampleLight();     // Explicit light connection
    float p = PdfLight(L) / GeometryFactor(P, L); // Probability of L, converted to angular measure

    if (p > 0.0f) {
        Vector3f Obj2LightDir = (L.position - P.position).normalized();   // direction, from current obj to light obj
        Intersection I = TraceRay(Ray(P.position, Obj2LightDir), Tree);   // trace a ray from current obj to random light

        if (I.hasIntersection() && compareVector(I.position, L.position))     // if intersection exists and is as as position in light
        {
            Vector3f f = EvalScattering(ViewingDir, P.normal, Obj2LightDir, P);
            color += 0.5f * (f / p).cwiseProduct(weight).cwiseProduct(EvalRadiance(L));
        }
    }
}





void PrintVector(std::string str, Vector3f v)
{
    std::cout << str << ", x: " << v.x() << ", y: " << v.y() << ", z: " << v.z() << std::endl;
}


float Scene::PdfBrdf(Vector3f ViewingDir, Vector3f N, Vector3f SampleDir, const Intersection& intersect) const
{ 
    float ProbDiffuse = ProbChooseDiffuse(intersect);

    float ProbSpecular = ProbChooseSpecular(intersect);

    float diffuse = fabsf(N.dot(SampleDir)) / PI;

    Vector3f m = MidVector(ViewingDir, SampleDir);
    float specular = DistributionFunction(m, intersect) * fabsf(m.dot(N)) / (4.0f * SampleDir.dot(m));

    return ProbDiffuse * diffuse + ProbSpecular * specular;
}


Vector3f Scene::MidVector(Vector3f ViewingDir, Vector3f LightDir) const
{
    return (ViewingDir + LightDir).normalized();
}



float Scene::ProbChooseDiffuse(const Intersection& intersect) const
{
    float Kd = intersect.shape->mat->Kd.norm();
    float Ks = intersect.shape->mat->Ks.norm();

    float s = Kd + Ks;

    return Kd / s;
}



float Scene::ProbChooseSpecular(const Intersection& intersect) const
{
    float Kd = intersect.shape->mat->Kd.norm();
    float Ks = intersect.shape->mat->Ks.norm();

    float s = Kd + Ks;

    return Ks / s;
}


// TODO
float DistributionFunction(Vector3f m, const Intersection& intersect)
{
    return DistributionPhong(m, intersect);
    //std::cout << "dis: " << DistributionPhong(m, intersect) << std::endl;
    //return DistributionGGX(m, intersect);
}


float DistributionPhong(Vector3f m, const Intersection& intersect)
{
    float alpha = intersect.shape->mat->alpha;
    float mDotN = m.dot(intersect.normal);
    if (mDotN < 0) return 0.0f;
    else return (alpha + 2.0f) / 2.0f / PI * pow(mDotN, alpha);
}

float DistributionGGX(Vector3f m, const Intersection& intersect)
{
    float alpha = intersect.shape->mat->alpha;
    float mDotN = m.dot(intersect.normal);
    if (mDotN < 0) return 0.0f;

    float tanTheta = sqrtf(1.0f - mDotN * mDotN) / mDotN;
    //float denominator = PI * pow(mDotN, 4) * pow((alpha * alpha + tanTheta * tanTheta), 2);

    float denominator = PI * pow((mDotN * mDotN * (alpha * alpha - 1) + 1), 2);

    return alpha * alpha / denominator;
}


Vector3f Fresnel(float d, const Intersection& intersect) {
    Vector3f Ks = intersect.shape->mat->Ks;
    return Ks + (Vector3f(1.0f, 1.0f, 1.0f) - Ks) * pow((1.0f - d), 5.0f);
}


float GeometryFunction(Vector3f ViewingDir, Vector3f LightDir, Vector3f m, const Intersection& intersect)
{
    return G1_Phong(ViewingDir, m, intersect) * G1_Phong(LightDir, m, intersect);
    //return G1_GGX(ViewingDir, m, intersect) * G1_GGX(ViewingDir, m, intersect);
}


// Phong
float G1_Phong(Vector3f dir, Vector3f m, const Intersection& intersect)
{
    float vDotN = dir.dot(intersect.normal);
    if (vDotN > 1.0f) return 1.0f;
    if (dir.dot(m) / vDotN < 0) return 0.0f;
    else {
        float tanTheta = sqrt(1.0f - vDotN * vDotN) / vDotN;
        if (tanTheta < 0.00001f) return 1.0f;
        float a = sqrt(intersect.shape->mat->alpha / 2.0f + 1.0f) / tanTheta;
        if (a > 1.6) return 1.0f;
        else return (3.535f * a + 2.181f * a * a) / (1.0f + 2.276f * a + 2.577 * a * a);
    }
}



float G1_GGX(Vector3f dir, Vector3f m, const Intersection& intersect)
{
    float vDotN = dir.dot(intersect.normal);

    if (vDotN > 1.0f) return 1.0f;
    if (dir.dot(m) / vDotN < 0) return 0.0f;

    float alpha = intersect.shape->mat->alpha;
    float tanTheta = sqrtf(1.0f - vDotN * vDotN) / vDotN;

    return 2.0f / (1 + sqrtf( 1 + alpha * alpha * tanTheta * tanTheta));
}
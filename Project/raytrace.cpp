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

#define NUM_PASS 8


Scene::Scene() 
    : RussianRoulette(0.8f)
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
                image[y * width + x] += Color(TracePath(ray, Tree) / NUM_PASS);
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

    while (myrandom(RNGen) <= RussianRoulette)
    {     
        ExplicitLight(Weight, Color, P, Tree);  

        const Vector3f SampleDir = SampleBrdf(P.normal);     // sampled direction

        Intersection Q = TraceRay(Ray(P.position, SampleDir), Tree); // Extend path
        if (!Q.hasIntersection()) break;

        float p = PdfBrdf(P.normal, SampleDir);
        if (p < Epsilon) break;

        Weight = Weight.cwiseProduct(EvalScattering(P.normal, SampleDir, P) / p);

        if (Q.shape->mat->isLight())    // Implicit light connection
        {      
            Color += 0.5f * EvalRadiance(Q).cwiseProduct(Weight);
            break;
        }

        P = Q;      // step forward
    }

    return Color;
}


Vector3f Scene::EvalScattering(Vector3f N, Vector3f SampleDir, Intersection& intersect) const
{
    Vector3f Kd = intersect.shape->mat->Kd;
    return fabsf(N.dot(SampleDir))* Kd / PI;
}



Vector3f Scene::SampleBrdf(Vector3f N) const
{
    float t1 = myrandom(RNGen); // first unifromly distributed random number
    float t2 = myrandom(RNGen); // first unifromly distributed random number

    return SampleLobe(N, t1, 2 * PI * t2);
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



void Scene::ExplicitLight(Vector3f& weight, Vector3f& color, Intersection &P, KdBVH<float, 3, Shape*> Tree) const
{
    Intersection L = SampleLight();     // Explicit light connection
    float p = PdfLight(L) / GeometryFactor(P, L); // Probability of L, converted to angular measure

    if (p > 0.0f) {
        Vector3f Obj2LightDir = L.position - P.position;   // direction, from current obj to light obj
        Intersection I = TraceRay(Ray(P.position, Obj2LightDir), Tree);   // trace a ray from current obj to random light

        if (I.hasIntersection() && I.position == L.position)     // if intersection exists and is as as position in light
        {
            Vector3f f = EvalScattering(P.normal, Obj2LightDir, I);
            color += 0.5f * (f / p).cwiseProduct(weight).cwiseProduct(EvalRadiance(I));
        }
    }
}





void PrintVector(std::string str, Vector3f v)
{
    std::cout << str << ", x: " << v.x() << ", y: " << v.y() << ", z: " << v.z() << std::endl;
}
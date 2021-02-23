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

#define NUM_PASS 40


Scene::Scene() 
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
    //Timer timer;

    for (int i = 0; i < NUM_PASS; ++i) {
        std::cout << "Rendering " << i << "th pass" << std::endl;

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {

            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; x++) {
                Color color = Color(0.3, 0.3, 0.3);

                const Ray ray = camera->generateRay(x, y, width, height);
                /*
                Intersection intersect = TraceRay(ray, Tree);

                if (intersect.hasIntersection())
                {
                    //float r = fabsf(intersect.normal.x());
                    //float g = fabsf(intersect.normal.y());
                    //float b = fabsf(intersect.normal.z());
                    //color = Vector3f(r, g, b);

                    //color = Color((intersect.t - 4.0f) / 3.0f); // depth

                    color = Color(intersect.shape->mat->Kd);    // material testing

                    //color = Color(1.0, 0.0, 0.0); // intersection test
                }

                image[y * width + x] = color;
                */
                
                image[y * width + x] += Color(TracePath(ray, Tree) / NUM_PASS);
            }
        }
    }
    //std::cout << "Total time: " << timer.elapsed() << std::endl;
    //fprintf(stderr, "\n");
}


Intersection Scene::TraceRay(const Ray& ray, const KdBVH<float, 3, Shape*> &Tree)
{
    Minimizer minimizer(ray);

    BVMinimize(Tree, minimizer);

    // fill intersection base on Tracing result
    return minimizer.intersection;
}





Vector3f Scene::TracePath(const Ray& ray, KdBVH<float, 3, Shape*> Tree)
{
    Vector3f C(0, 0, 0);
    Vector3f W(1, 1, 1);

    // inital ray
    Intersection P = TraceRay(ray, Tree);

    if (!P.hasIntersection()) return C;     // no intersection

    if (P.shape->mat->isLight()) return EvalRadiance(P);    // hit light source

    float RussianRoulette = 0.8f;
    while (myrandom(RNGen) <= RussianRoulette)
    {
        // Extend path
        Vector3f N = P.normal;

        Vector3f SampleDir = SampleBrdf(N);     // sampled direction
        Ray r(P.position, SampleDir);           // new ray

        Intersection Q = TraceRay(r, Tree);

        if (!Q.hasIntersection()) break;

        Vector3f f = EvalScattering(N, SampleDir, P);
        float p = PdfBrdf(N, SampleDir);

        if (p < 0.0000001f) break;

        W = W.cwiseProduct( f / p);

        // Implicit light connection
        if (Q.shape->mat->isLight()) 
        {
            C += applyWeight(EvalRadiance(Q), W);
            break;
        }

        // step forward
        P = Q;
    }

    return C;
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


Vector3f Scene::applyWeight(Vector3f v, Vector3f W) const
{
    float x = v.x() * W.x();
    float y = v.y() * W.y();
    float z = v.z() * W.z();
    return Vector3f(x, y, z);
}
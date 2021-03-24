#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define PI 3.14159265359
#include <random>
#include <iostream>
#include <string>
#include <stdio.h>
#include <algorithm>
#include <list>

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0, 1);

class Vector
{
public:
    explicit Vector(double x = 0, double y = 0, double z = 0)
    {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    }
    double operator[](int i) const { return coords[i]; };
    double &operator[](int i) { return coords[i]; };
    double sqrtNorm() const
    {
        return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];
    }
    Vector get_normalized()
    {
        double n = sqrt(sqrtNorm());
        return Vector(coords[0] / n, coords[1] / n, coords[2] / n);
    }

private:
    double coords[3];
};
Vector operator+(const Vector &a, const Vector &b)
{
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector &a, const Vector &b)
{
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(double a, const Vector &b)
{
    return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator*(const Vector &b, double a)
{
    return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator*(const Vector &a, const Vector &b)
{
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
Vector operator/(const Vector &b, double a)
{
    return Vector(b[0] / a, b[1] / a, b[2] / a);
}
Vector cross(const Vector &a, const Vector &b)
{
    return Vector(a[1] * b[2] - b[1] * a[2], a[2] * b[0] - b[2] * a[0], a[0] * b[1] - a[1] * b[0]);
}
double dot(const Vector &a, const Vector &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
};
double sqr(double x)
{
    return x * x;
}

Vector random_cos(const Vector &N)
{
    double u1 = uniform(engine);
    double u2 = uniform(engine);
    double x1 = cos(2 * PI * u1) * sqrt(1 - u2);
    double x2 = sin(2 * PI * u1) * sqrt(1 - u2);
    double x3 = sqrt(u2);
    Vector T;
    if (N[0] < N[1] && N[0] < N[2])
    {
        T = Vector(0, -N[2], N[1]);
    }
    else
    {
        if (N[1] < N[0] && N[1] < N[2])
        {
            T = Vector(N[2], 0, -N[0]);
        }

        else
        {
            T = Vector(-N[1], N[0], 0);
        }
    }
    T = T.get_normalized();
    Vector T2 = cross(N, T).get_normalized();
    Vector up = N * x3 + T * x1 + T2 * x2;
    return up.get_normalized();
}
class Ray
{
public:
    Vector C;
    Vector U;
    Ray(const Vector &C, Vector &U) : C(C), U(U.get_normalized())
    {
    }
};
class Object
{
public:
    Object(){};
    virtual bool intersect(const Ray &r, Vector &P, Vector &N, double &t, Vector &color) = 0;
    Vector albedo;
    Vector color;
    bool isMirror;
    bool isTransparent;
};
class BoundingBox
{
public:
    Vector mini, maxi;
    bool intersect(const Ray &r)
    {
        double t1x = (mini[0] - r.C[0]) / r.U[0];
        double t2x = (maxi[0] - r.C[0]) / r.U[0];
        double txMin = std::min(t1x, t2x);
        double txMax = std::max(t1x, t2x);

        double t1y = (mini[1] - r.C[1]) / r.U[1];
        double t2y = (maxi[1] - r.C[1]) / r.U[1];
        double tyMin = std::min(t1y, t2y);
        double tyMax = std::max(t1y, t2y);

        double t1z = (mini[2] - r.C[2]) / r.U[2];
        double t2z = (maxi[2] - r.C[2]) / r.U[2];
        double tzMin = std::min(t1z, t2z);
        double tzMax = std::max(t1z, t2z);

        double tMax = std::min(txMax, std::min(tyMax, tzMax));
        double tMin = std::max(txMin, std::max(tyMin, tzMin));
        if (tMax < 0)
            return false;
        return tMax > tMin;
    };
};
class Node
{
public:
    Node *fg, *fd;
    BoundingBox Bb;
    int start, end;
};
class TriangleIndices
{
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group){};
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;    // indices within the uv coordinates array
    int ni, nj, nk;       // indices within the normals array
    int group;            // face group
};
class TriangleMesh : public Object
{
public:
    ~TriangleMesh() {}
    TriangleMesh(const Vector &albedo, bool isMirror = false, bool isTransparent = false)
    {
        this->albedo = albedo;
        this->isMirror = isMirror;
        this->isTransparent = isTransparent;
        BVH = new Node;
    };

    void readOBJ(const char *obj)
    {

        char matfile[255];
        char grp[255];

        FILE *f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f))
        {
            char line[255];
            if (!fgets(line, 255, f))
                break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's')
            {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }

            if (line[0] == 'v' && line[1] == ' ')
            {
                Vector vec;

                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6)
                {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));

                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
                }
                else
                {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n')
            {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't')
            {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f')
            {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;

                char *consumedline = line + 1;
                int offset;

                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9)
                {
                    if (i0 < 0)
                        t.vtxi = vertices.size() + i0;
                    else
                        t.vtxi = i0 - 1;
                    if (i1 < 0)
                        t.vtxj = vertices.size() + i1;
                    else
                        t.vtxj = i1 - 1;
                    if (i2 < 0)
                        t.vtxk = vertices.size() + i2;
                    else
                        t.vtxk = i2 - 1;
                    if (j0 < 0)
                        t.uvi = uvs.size() + j0;
                    else
                        t.uvi = j0 - 1;
                    if (j1 < 0)
                        t.uvj = uvs.size() + j1;
                    else
                        t.uvj = j1 - 1;
                    if (j2 < 0)
                        t.uvk = uvs.size() + j2;
                    else
                        t.uvk = j2 - 1;
                    if (k0 < 0)
                        t.ni = normals.size() + k0;
                    else
                        t.ni = k0 - 1;
                    if (k1 < 0)
                        t.nj = normals.size() + k1;
                    else
                        t.nj = k1 - 1;
                    if (k2 < 0)
                        t.nk = normals.size() + k2;
                    else
                        t.nk = k2 - 1;
                    indices.push_back(t);
                }
                else
                {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6)
                    {
                        if (i0 < 0)
                            t.vtxi = vertices.size() + i0;
                        else
                            t.vtxi = i0 - 1;
                        if (i1 < 0)
                            t.vtxj = vertices.size() + i1;
                        else
                            t.vtxj = i1 - 1;
                        if (i2 < 0)
                            t.vtxk = vertices.size() + i2;
                        else
                            t.vtxk = i2 - 1;
                        if (j0 < 0)
                            t.uvi = uvs.size() + j0;
                        else
                            t.uvi = j0 - 1;
                        if (j1 < 0)
                            t.uvj = uvs.size() + j1;
                        else
                            t.uvj = j1 - 1;
                        if (j2 < 0)
                            t.uvk = uvs.size() + j2;
                        else
                            t.uvk = j2 - 1;
                        indices.push_back(t);
                    }
                    else
                    {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3)
                        {
                            if (i0 < 0)
                                t.vtxi = vertices.size() + i0;
                            else
                                t.vtxi = i0 - 1;
                            if (i1 < 0)
                                t.vtxj = vertices.size() + i1;
                            else
                                t.vtxj = i1 - 1;
                            if (i2 < 0)
                                t.vtxk = vertices.size() + i2;
                            else
                                t.vtxk = i2 - 1;
                            indices.push_back(t);
                        }
                        else
                        {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0)
                                t.vtxi = vertices.size() + i0;
                            else
                                t.vtxi = i0 - 1;
                            if (i1 < 0)
                                t.vtxj = vertices.size() + i1;
                            else
                                t.vtxj = i1 - 1;
                            if (i2 < 0)
                                t.vtxk = vertices.size() + i2;
                            else
                                t.vtxk = i2 - 1;
                            if (k0 < 0)
                                t.ni = normals.size() + k0;
                            else
                                t.ni = k0 - 1;
                            if (k1 < 0)
                                t.nj = normals.size() + k1;
                            else
                                t.nj = k1 - 1;
                            if (k2 < 0)
                                t.nk = normals.size() + k2;
                            else
                                t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }

                consumedline = consumedline + offset;

                while (true)
                {
                    if (consumedline[0] == '\n')
                        break;
                    if (consumedline[0] == '\0')
                        break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3)
                    {
                        if (i0 < 0)
                            t2.vtxi = vertices.size() + i0;
                        else
                            t2.vtxi = i0 - 1;
                        if (i2 < 0)
                            t2.vtxj = vertices.size() + i2;
                        else
                            t2.vtxj = i2 - 1;
                        if (i3 < 0)
                            t2.vtxk = vertices.size() + i3;
                        else
                            t2.vtxk = i3 - 1;
                        if (j0 < 0)
                            t2.uvi = uvs.size() + j0;
                        else
                            t2.uvi = j0 - 1;
                        if (j2 < 0)
                            t2.uvj = uvs.size() + j2;
                        else
                            t2.uvj = j2 - 1;
                        if (j3 < 0)
                            t2.uvk = uvs.size() + j3;
                        else
                            t2.uvk = j3 - 1;
                        if (k0 < 0)
                            t2.ni = normals.size() + k0;
                        else
                            t2.ni = k0 - 1;
                        if (k2 < 0)
                            t2.nj = normals.size() + k2;
                        else
                            t2.nj = k2 - 1;
                        if (k3 < 0)
                            t2.nk = normals.size() + k3;
                        else
                            t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    }
                    else
                    {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2)
                        {
                            if (i0 < 0)
                                t2.vtxi = vertices.size() + i0;
                            else
                                t2.vtxi = i0 - 1;
                            if (i2 < 0)
                                t2.vtxj = vertices.size() + i2;
                            else
                                t2.vtxj = i2 - 1;
                            if (i3 < 0)
                                t2.vtxk = vertices.size() + i3;
                            else
                                t2.vtxk = i3 - 1;
                            if (j0 < 0)
                                t2.uvi = uvs.size() + j0;
                            else
                                t2.uvi = j0 - 1;
                            if (j2 < 0)
                                t2.uvj = uvs.size() + j2;
                            else
                                t2.uvj = j2 - 1;
                            if (j3 < 0)
                                t2.uvk = uvs.size() + j3;
                            else
                                t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        }
                        else
                        {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2)
                            {
                                if (i0 < 0)
                                    t2.vtxi = vertices.size() + i0;
                                else
                                    t2.vtxi = i0 - 1;
                                if (i2 < 0)
                                    t2.vtxj = vertices.size() + i2;
                                else
                                    t2.vtxj = i2 - 1;
                                if (i3 < 0)
                                    t2.vtxk = vertices.size() + i3;
                                else
                                    t2.vtxk = i3 - 1;
                                if (k0 < 0)
                                    t2.ni = normals.size() + k0;
                                else
                                    t2.ni = k0 - 1;
                                if (k2 < 0)
                                    t2.nj = normals.size() + k2;
                                else
                                    t2.nj = k2 - 1;
                                if (k3 < 0)
                                    t2.nk = normals.size() + k3;
                                else
                                    t2.nk = k3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            }
                            else
                            {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1)
                                {
                                    if (i0 < 0)
                                        t2.vtxi = vertices.size() + i0;
                                    else
                                        t2.vtxi = i0 - 1;
                                    if (i2 < 0)
                                        t2.vtxj = vertices.size() + i2;
                                    else
                                        t2.vtxj = i2 - 1;
                                    if (i3 < 0)
                                        t2.vtxk = vertices.size() + i3;
                                    else
                                        t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                }
                                else
                                {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        fclose(f);
    }
    BoundingBox buildBB(int start, int end)
    {
        Bbox.mini = Vector(1E9, 1E9, 1E9);
        Bbox.maxi = Vector(-1E9, -1E9, -1E9);
        for (int i = start; i < end; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Bbox.mini[j] = std::min(Bbox.mini[j], vertices[indices[i].vtxi][j]);
                Bbox.maxi[j] = std::max(Bbox.maxi[j], vertices[indices[i].vtxi][j]);
                Bbox.mini[j] = std::min(Bbox.mini[j], vertices[indices[i].vtxj][j]);
                Bbox.maxi[j] = std::max(Bbox.maxi[j], vertices[indices[i].vtxj][j]);
                Bbox.mini[j] = std::min(Bbox.mini[j], vertices[indices[i].vtxk][j]);
                Bbox.maxi[j] = std::max(Bbox.maxi[j], vertices[indices[i].vtxk][j]);
            }
        }
        return Bbox;
    }
    void buildBVH(Node *n, int start, int end)
    {

        n->start = start;
        n->end = end;
        n->Bb = buildBB(n->start, n->end);
        Vector diag = n->Bb.maxi - n->Bb.mini;
        int dimension;
        if (diag[0] >= diag[1] && diag[0] >= diag[2])
        {
            dimension = 0;
        }
        else
        {
            if (diag[1] >= diag[0] && diag[1] >= diag[2])
            {
                dimension = 1;
            }
            else
            {
                dimension = 2;
            }
        }
        double middle = (n->Bb.mini[dimension] + n->Bb.maxi[dimension]) / 2;
        int i_pivot = n->start;

        for (int i = n->start; i < n->end; i++)
        {
            double triangle_middle = (vertices[indices[i].vtxi][dimension] + vertices[indices[i].vtxj][dimension] + vertices[indices[i].vtxk][dimension]) / 3;
            if (middle > triangle_middle)
            {
                std::swap(indices[i], indices[i_pivot]);
                i_pivot++;
            }
        }
        n->fg = NULL;
        n->fd = NULL;
        if (i_pivot == start || i_pivot == end || end - start < 5)
        {
            return;
        }

        n->fg = new Node;
        n->fd = new Node;

        buildBVH(n->fg, n->start, i_pivot);
        buildBVH(n->fd, i_pivot, n->end);
    };
    bool intersect(const Ray &r, Vector &P, Vector &N, double &t, Vector &color)
    {
        if (!BVH->Bb.intersect(r))
            return false;

        std::list<Node *> L;
        L.push_back(BVH);
        while (!L.empty())
        {
            Node *current_node = L.front();
            L.pop_front();
            if (current_node->fg)
            {
                if (current_node->fg->Bb.intersect(r))
                {
                    L.push_front(current_node->fg);
                }
                if (current_node->fd->Bb.intersect(r))
                {
                    L.push_front(current_node->fd);
                }
            }
            else
            {
                bool inter = false;
                for (int i = current_node->start; i < current_node->end; i++)
                {
                    const Vector &A = vertices[indices[i].vtxi];
                    const Vector &B = vertices[indices[i].vtxj];
                    const Vector &C = vertices[indices[i].vtxk];

                    Vector e1 = B - A;
                    Vector e2 = C - A;
                    Vector Nbis = cross(e1, e2);
                    Vector Ac = r.C - A;
                    double Un = dot(r.U, Nbis);

                    double beta = -dot(e2, cross(Ac, r.U)) / Un;
                    double gamma = dot(e1, cross(Ac, r.U)) / Un;
                    double alpha = 1 - gamma - beta;

                    double localt = -dot(Ac, Nbis) / Un;

                    if (beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && alpha >= 0 && localt > 0)
                    {
                        inter = true;
                        if (localt < t)
                        {
                            t = localt;
                            N = alpha * normals[indices[i].ni] + beta * normals[indices[i].nj] + gamma * normals[indices[i].nk];
                            N = N.get_normalized();
                            P = r.C + t * r.U;

                            Vector UV = alpha * uvs[indices[i].uvi] + beta * uvs[indices[i].uvj] + gamma * uvs[indices[i].uvk];
                            UV = UV * Vector(WidthTexture[indices[i].group], HeightTexture[indices[i].group], 0);
                            int uvx = UV[0] + 0.5;
                            int uvy = UV[1] + 0.5;
                            int W = WidthTexture[indices[i].group];
                            int H = HeightTexture[indices[i].group];
                            uvx = uvx % W;
                            uvy = uvy % H;
                            if (uvx < 0)
                                uvx += W;
                            if (uvy < 0)
                                uvy += H;
                            uvy = H - uvy - 1;
                            color = Vector(std::pow(textures[indices[i].group][(uvy * H + uvx) * 3] / 255., 2.2),
                                           std::pow(textures[indices[i].group][(uvy * H + uvx) * 3 + 1] / 255., 2.2),
                                           std::pow(textures[indices[i].group][(uvy * H + uvx) * 3 + 2] / 255., 2.2));
                        }
                    }
                }
                return inter;
            }
        }
        return false;
    };

    void loadTexture(const char *filename)
    {
        int W, H, C;
        unsigned char *texture = stbi_load(filename, &W, &H, &C, 3);
        WidthTexture.push_back(W);
        HeightTexture.push_back(H);
        textures.push_back(texture);
    };

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
    std::vector<unsigned char *> textures;
    std::vector<int> WidthTexture, HeightTexture;
    BoundingBox Bbox;
    Node *BVH;
};
class Sphere : public Object
{
public:
    Vector O;
    double R;
    Sphere(const Vector &O, double R, const Vector &albedo, const bool isMirror = false, const bool isTransparent = false) : O(O), R(R)
    {
        this->albedo = albedo;
        this->isMirror = isMirror;
        this->isTransparent = isTransparent;
        this->color = albedo;
    };
    bool intersect(const Ray &r, Vector &P, Vector &N, double &t, Vector &color)
    {
        // solves a*t^2 + b*t + c = 0
        double a = 1;
        double b = 2 * dot(r.U, r.C - O);
        double c = (r.C - O).sqrtNorm() - R * R;
        double delta = b * b - 4 * a * c;
        if (delta < 0)
            return false;

        double sqDelta = sqrt(delta);
        double t2 = (-b + sqDelta) / (2 * a);
        if (t2 < 0)
            return false;

        double t1 = (-b - sqDelta) / (2 * a);
        if (t1 > 0)
            t = t1;
        else
            t = t2;

        P = r.C + t * r.U;
        N = (P - O).get_normalized();

        color = this->albedo;
        return true;
    };
};

class Scene
{
public:
    Scene(){};
    bool intersect(const Ray &r, Vector &P, Vector &N, Vector &albedo, bool &mirror, bool &trans, double &t, int &objectId)
    {

        t = 1E90;
        bool inter = false;
        for (int i = 0; i < objects.size(); i++)
        {
            Vector localP, localN, localAlbedo;
            double localt;
            if (objects[i]->intersect(r, localP, localN, localt, localAlbedo) && localt < t)
            {
                t = localt;
                inter = true;
                albedo = localAlbedo;
                P = localP;
                N = localN;
                mirror = objects[i]->isMirror;
                trans = objects[i]->isTransparent;
                objectId = i;
            }
        }
        return inter;
    }
    Vector getColor(const Ray &r, int rebond)
    {
        if (rebond > 8)
        {
            return Vector(0., 0., 0.);
        }

        Vector P, N, albedo;
        double t;
        bool mirror, trans;
        int objectId;
        bool inter = intersect(r, P, N, albedo, mirror, trans, t, objectId);
        if (inter)
        {
            if (objectId == 0)
            {
                if (rebond == 0)
                {
                    return Vector(I, I, I) / (4 * PI * PI * dynamic_cast<Sphere *>(objects[0])->R * dynamic_cast<Sphere *>(objects[0])->R);
                }
                else
                {
                    return Vector(0., 0., 0.);
                }
            }
            if (mirror)
            {
                Vector reflectedDir = r.U - 2 * dot(r.U, N) * N;
                Ray reflectedRay(P + 0.01 * N, reflectedDir);
                return getColor(reflectedRay, rebond + 1);
            }
            else
            {
                if (trans)
                {
                    double n1 = 1, n2 = 1.4;
                    Vector N2 = N;
                    if (dot(r.U, N) > 0)
                    { // on sort de la sphere
                        std::swap(n1, n2);
                        N2 = -1 * N;
                    }
                    Vector Tt = n1 / n2 * (r.U - dot(r.U, N2) * N2);
                    double rad = 1 - sqr(n1 / n2) * (1 - sqr(dot(r.U, N2)));
                    if (rad < 0)
                    {
                        Vector reflectedDir = r.U - 2 * dot(r.U, N) * N;
                        Ray reflectedRay(P + 0.0001 * N, reflectedDir);
                        return getColor(reflectedRay, rebond + 1);
                    }
                    Vector Tn = -1 * sqrt(rad) * N2;
                    Vector refractedDir = Tt + Tn;
                    return getColor(Ray(P - 0.00001 * N2, refractedDir), rebond + 1);
                }
                else
                {

                    // eclairge direct
                    // Vector PL = L - P;
                    // double d = sqrt(PL.sqrtNorm());
                    // Vector shadowP, shadowN, shadowAlbedo;
                    // double shadowt;
                    // int objectId;
                    // bool shadowMirror, shadowTransparent;
                    // Ray shadowRay(P+0.01*N, PL);
                    // bool shadowInter = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransparent, shadowt, objectId);
                    // if (shadowInter && shadowt < d) {
                    //     color = Vector(0., 0., 0.);
                    // } else {
                    //     color = I/(4*PI) * albedo/PI * std::max(0., dot(N, PL/d)) / (d*d);
                    // }

                    // eclairage direct

                    Vector PL = L - P;
                    Vector w = random_cos(-1 * PL.get_normalized());
                    Vector xprime = w * dynamic_cast<Sphere *>(objects[0])->R + dynamic_cast<Sphere *>(objects[0])->O;
                    Vector Pxprime = xprime - P;
                    double d = sqrt(Pxprime.sqrtNorm());
                    Pxprime = Pxprime.get_normalized();

                    Vector shadowP, shadowN, shadowAlbedo;
                    double shadowt;
                    int objectId;
                    Vector color;
                    bool shadowMirror, shadowTransparent;
                    Ray shadowRay(P + 0.001 * N, Pxprime);
                    bool shadowInter = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransparent, shadowt, objectId);

                    if (shadowInter && shadowt < d - 0.001)
                    {
                        color = Vector(0., 0., 0.);
                    }
                    else
                    {
                        double prob = std::max(0., dot(-1 * PL, w)) / (PI * dynamic_cast<Sphere *>(objects[0])->R * dynamic_cast<Sphere *>(objects[0])->R);
                        double J = std::max(0., dot(w, -1 * Pxprime)) / (d * d);
                        color = I / (4 * PI * PI) * albedo / PI * std::max(0., dot(N, Pxprime)) / (dynamic_cast<Sphere *>(objects[0])->R * dynamic_cast<Sphere *>(objects[0])->R) * J / prob;
                    }

                    // eclairage indirect
                    Vector wi = random_cos(N.get_normalized());
                    Ray ri(P + 0.001 * N, wi);
                    return color + albedo * getColor(ri, rebond + 1);
                }
            }
        }
        else
        {
            return Vector(0., 0., 0.);
        }
    }
    std::vector<Object *> objects;
    Vector L;
    double I;
};

void integrate()
{
    int N = 100000;
    double sigma = 1;
    double s = 0;
    for (int i = 0; i < N; i++)
    {
        double u1 = uniform(engine);
        double u2 = uniform(engine);
        double u3 = uniform(engine);
        double u4 = uniform(engine);
        double x1 = sigma * cos(2 * PI * u1) * sqrt(-2 * log(u2));
        double x2 = sigma * sin(2 * PI * u1) * sqrt(-2 * log(u2));
        double x3 = sigma * cos(2 * PI * u3) * sqrt(-2 * log(u4));
        double x4 = sigma * sin(2 * PI * u3) * sqrt(-2 * log(u4));
        double p = std::pow(1 / (sigma * sqrt(2 * PI)), 4) * exp(-x1 * x1 / (2 * sigma * sigma)) * exp(-x2 * x2 / (2 * sigma * sigma)) * exp(-x3 * x3 / (2 * sigma * sigma)) * exp(-x4 * x4 / (2 * sigma * sigma));

        if ((x1 < -PI / 2 || x1 > PI / 2) || (x2 < -PI / 2 || x2 > PI / 2) || (x3 < -PI / 2 || x3 > PI / 2) || (x4 < -PI / 2 || x4 > PI / 2))
        {
            continue;
        }
        else
        {
            s += std::pow(cos(x1 + x2 + x3 + x4), 2) / p / N;
        }
        std::cout << s << std::endl;
    }
}

int main()
{

    int W = 512;
    int H = 512;
    Vector C(0, 0, 55);
    Scene scene;
    Vector O(0, 0, 0);
    double R = 10;
    Vector rho(1., 1., 1.);
    Sphere S(O, R, rho);
    scene.I = 1E11;
    scene.L = Vector(-55, 20, -20);

    Sphere SourceLumiere(scene.L, 3, Vector(1., 1., 1.));
    // Sphere S2(Vector(-10, 0, 20), 10, Vector(1., 0., 1.));
    // Sphere S3(Vector(10, 0, -20), 10, Vector(1., 1., 0.));
    Sphere Sol(Vector(0, -1000, 0), 960, Vector(1., 1., 1.));
    Sphere Plafond(Vector(0, 1000, 0), 940, Vector(1., 1., 1.));
    Sphere MurG(Vector(-1000, 0, 0), 940, Vector(1., 1., 1.));
    Sphere MurD(Vector(1000, 0, 0), 940, Vector(1., 1., 1.));
    Sphere Fond(Vector(0, 0, -1000), 940, Vector(0.2, 0.2, 1.));
    Sphere FondCaché(Vector(0, 0, 1000), 940, Vector(1., 1., 1.));
    TriangleMesh Dog(Vector(1., 1., 1.));
    Dog.readOBJ("13463_Australian_Cattle_Dog_v3.obj");
    Dog.loadTexture("Australian_Cattle_Dog_dif.jpg");
    for (int i = 0; i < Dog.vertices.size(); i++)
    {
        Dog.vertices[i][1] += 10;
        Dog.vertices[i][1] = -Dog.vertices[i][1];

        std::swap(Dog.vertices[i][1], Dog.vertices[i][2]);
        Dog.vertices[i][1] -= 10;
    }
    for (int i = 0; i < Dog.normals.size(); i++)
    {
        std::swap(Dog.normals[i][1], Dog.normals[i][2]);

        Dog.normals[i][2] = -Dog.normals[i][2];
    }
    Dog.buildBVH(Dog.BVH, 0, Dog.indices.size());
    scene.objects.push_back(&SourceLumiere);
    // scene.objects.push_back(&S);
    // scene.objects.push_back(&S2);
    // scene.objects.push_back(&S3);
    scene.objects.push_back(&Sol);
    scene.objects.push_back(&Plafond);
    scene.objects.push_back(&MurG);
    scene.objects.push_back(&MurD);
    scene.objects.push_back(&Fond);
    scene.objects.push_back(&FondCaché);
    scene.objects.push_back(&Dog);
    double fov = 60 * PI / 180;

    int nbrays = 500;

    std::vector<unsigned char> image(W * H * 3, 0);
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; i++)
    {
        for (int j = 0; j < W; j++)
        {
            Vector u(j - W / 2, i - H / 2, -W / (2. * tan(fov / 2)));
            Ray r(C, u);
            Vector color(0., 0., 0.);
            for (int k = 0; k < nbrays; k++)
            {
                double u1 = uniform(engine);
                double u2 = uniform(engine);
                double x1 = 0.25 * cos(2 * PI * u1) * sqrt(-2 * log(u2));
                double x2 = 0.25 * sin(2 * PI * u1) * sqrt(-2 * log(u2));

                u1 = uniform(engine);
                u2 = uniform(engine);
                double x3 = 0.25 * cos(2 * PI * u1) * sqrt(-2 * log(u2));
                double x4 = 0.25 * sin(2 * PI * u1) * sqrt(-2 * log(u2));

                u = Vector(j - W / 2 + x2 + 0.5, i - H / 2 + x1 + 0.5, -W / (2. * tan(fov / 2)));
                Vector target = C + 55 * u.get_normalized();
                Vector Cprime = C + Vector(x3, x4, 0);
                Vector uprime = (target - Cprime).get_normalized();
                r = Ray(Cprime, uprime);
                color = color + scene.getColor(r, 0);
            }
            color = color / nbrays;

            image[((H - i - 1) * W + j) * 3 + 0] = std::min(255., std::pow(color[0], 0.45));
            image[((H - i - 1) * W + j) * 3 + 1] = std::min(255., std::pow(color[1], 0.45));
            image[((H - i - 1) * W + j) * 3 + 2] = std::min(255., std::pow(color[2], 0.45));
        }
    }
    stbi_write_png("image3.png", W, H, 3, &image[0], 0);

    return 0;
};
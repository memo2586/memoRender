#include "tgaimage.h"
#include "geometry.h"

extern Matrix Modelview;
extern Matrix Viewport;
extern Matrix Projection;

void viewport(int x, int y, int w, int h);
void projection(float coeff = 1.f);     //coeff = -1/camera
void modelview(Vec3f camera, Vec3f center, Vec3f up);

struct IShader{
    virtual ~IShader(); //别忘了实现
    virtual Vec4f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

void triangle(mat<4, 3, float>& clipc, IShader& shader, TGAImage& image, float* zbuffer);
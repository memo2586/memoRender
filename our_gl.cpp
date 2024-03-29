#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix Modelview;
Matrix Viewport;
Matrix Projection;

IShader::~IShader() {}

/*
    viewport
    参数：视口起始x坐标，视口起始y坐标，视口宽度，视口高度
    返回：无
    功能：计算视口变换矩阵
*/
void viewport(int x, int y, int w, int h){
    Viewport = Matrix::identity();
    Viewport[0][3] = x + w / 2.f;
    Viewport[1][3] = y + h / 2.f;
    Viewport[2][3] = depth / 2.f;
    Viewport[0][0] = w / 2.f;
    Viewport[1][1] = h / 2.f;
    Viewport[2][2] = depth / 2.f;
}

/*
    projection
    参数：系数（-1/c）
    返回：无
    功能：计算透视变换矩阵
*/
void projection(float coeff){
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

/*
    modelview
    参数：相机坐标、相机观察的中心点坐标、指向原坐标系上方的方向向量up(0,1,0)
    返回：无
    功能：计算模型变换矩阵
*/
void modelview(Vec3f camera, Vec3f center, Vec3f up){
    //计算出z，根据z和up算出x，再算出y
    Vec3f z = (camera - center).normalize();
    Vec3f x = cross(up, z).normalize();
    Vec3f y = cross(z, x).normalize();
    Modelview = Matrix::identity();
    for (int i = 0; i < 3; i++){
        Modelview[0][i] = x[i];
        Modelview[1][i] = y[i];
        Modelview[2][i] = z[i];
        Modelview[i][3] = -center[i];
    }
}

// 两种不同的重心坐标求解方式，可以参考 https://zhuanlan.zhihu.com/p/144360079
/*
    barycentric
    参数：顶点坐标1，顶点坐标2，顶点坐标3，坐标P
    返回：Vec3f
    功能：返回点P在由三个顶点坐标围成的三角形中的重心坐标
*/
Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

Vec3f barycentric_(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
    float alpha = (-1 * (P.x - B.x) * (C.y - B.y) + (P.y - B.y) * (C.x - B.x)) / (-1 * (A.x - B.x) * (C.y - B.y) + (A.y - B.y) * (C.x - B.x));
    float beta = (-1 * (P.x - C.x) * (A.y - C.y) + (P.y - C.y) * (A.x - C.x)) / (-1 * (B.x - C.x) * (A.y - C.y) + (B.y - C.y) * (A.x - C.x));
    float gamma = 1.f - alpha - beta;
    if(alpha && beta && gamma) return Vec3f(alpha, beta, gamma);
    return Vec3f(-1, 1, 1);
}

/*
    triangle
    参数：顶点坐标数组，着色器，目标图像，深度缓冲区
    返回：无
    功能：光栅化器，将模型中每个三角形，使用指定的着色器来渲染这个三角形到目标图像上，同时利用深度缓冲区来正确处理遮挡关系。
          主函数的内循环完成时：
            - 光栅化器得到：三角形三个顶点经过MVP变换后的坐标（屏幕世界坐标）；
            - 着色器得到：存储该三角形每个顶点的纹理坐标的矩阵；
          光栅化器遍历位于该三角形内的所有片段，计算其重心坐标并将其传递给片段着色器，再将片段着色器返回的颜色值写入图像
*/
void triangle_(mat<4, 3, float>& clipc, IShader& shader, TGAImage& image, float* zbuffer) {
    mat<3, 4, float> pts = (Viewport * clipc).transpose();
    mat<3, 2, float> ndc_pts;
    for (int i = 0; i < 3; i++) ndc_pts[i] = proj<2>(pts[i] / pts[i][3]);

    Vec2f boundingbox_max(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
    Vec2f boundingbox_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            boundingbox_max[j] = std::min(clamp[j], std::max(ndc_pts[i][j], boundingbox_max[j]));
            boundingbox_min[j] = std::max(0.f, std::min(ndc_pts[i][j], boundingbox_min[j]));
        }
    }

    Vec2i P;
    TGAColor color;
    for (P.x = boundingbox_min.x; P.x <= boundingbox_max.x; P.x++) {
        for (P.y = boundingbox_min.y; P.y <= boundingbox_max.y; P.y++) {
            Vec3f bc_screen = barycentric(ndc_pts[0], ndc_pts[1], ndc_pts[2], P);
            // 重心坐标的透视矫正
            Vec3f bc_clip = Vec3f(bc_screen.x / pts[0][3], bc_screen.y / pts[1][3], bc_screen.z / pts[2][3]);
            bc_clip = bc_clip / (bc_clip.x + bc_clip.y + bc_clip.z);    // 保证矫正后重心坐标分量和为1
            float frag_depth = clipc[2] * bc_clip;
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z<0 || zbuffer[P.x + P.y * image.get_width()]>frag_depth) continue;
            bool discard = shader.fragment(bc_clip, color);
            if (!discard) {
                zbuffer[P.x + P.y * image.get_width()] = frag_depth;
                image.set(P.x, P.y, color);
            }
        }
    }
}

void triangle(Vec4f* pts, IShader& shader, TGAImage& image, float* zbuffer) {
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j] / pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j] / pts[i][3]);
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0] / pts[0][3]), proj<2>(pts[1] / pts[1][3]), proj<2>(pts[2] / pts[2][3]), proj<2>(P));
            float z = pts[0][2] * c.x + pts[1][2] * c.y + pts[2][2] * c.z;
            float w = pts[0][3] * c.x + pts[1][3] * c.y + pts[2][3] * c.z;
            int frag_depth = z / w;
            if (c.x < 0 || c.y < 0 || c.z<0 || zbuffer[P.x + P.y * image.get_width()]>frag_depth) continue;
            bool discard = shader.fragment(c, color);
            if (!discard) {
                zbuffer[P.x + P.y * image.get_width()] = frag_depth;
                image.set(P.x, P.y, color);
            }
        }
    }
}
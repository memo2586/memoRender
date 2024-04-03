#include <iostream> 
#include <sstream>
#include <cstring>

#include "geometry.h"
#include "model.h"
#include "our_gl.h"
#include "tgaimage.h"

Model* model = nullptr;
float* zbuffer = nullptr;
float* shadowbuffer = nullptr;
const int width = 1600;
const int height = 1600;

Vec3f camera(0, 0, 3);
Vec3f center(0, 0, 0);
Vec3f light_dir(1, 1, 1);
Vec3f up(0, 1, 0);

std::string getUniqueString() {
    auto now = std::time(nullptr);
    std::ostringstream oss;
    oss << now;
    return oss.str();
}

struct Shader : public IShader
{
    mat<2, 3, float> varying_uv;    // 存储三角形每个顶点的纹理坐标
    mat<4, 3, float> varying_tri;   // 齐次坐标下的三角形顶点
    mat<3, 3, float> ndc_tri;       // 标准坐标系下的三角形顶点
    mat<3, 3, float> varying_nrm;    // 三角形顶点法线
    mat<4, 4, float> uniform_M;     // Projection*ModelView，用于将模型世界坐标转换到屏幕空间坐标
    mat<4, 4, float> uniform_MIT;   // uniform_M 的逆转置，用于变换法线方向以保证法线方向依旧垂直于变换后的表面
    mat<4, 4, float> uniform_MS;    // 用于将 ndc_tri 逆变换到 shadowmapping 所在坐标系

    Shader(Matrix M, Matrix MIT, Matrix MS) : uniform_M(M), uniform_MIT(MIT), uniform_MS(MS), varying_uv(), varying_tri() {}

    /*
        vertex 顶点着色器
        参数: 面编号，顶点编号
        返回：Vec4f 三角形顶点信息，同时为片段着色器准备数据
    */
    virtual Vec4f vertex(int nface, int nvert) {
        // 处理顶点
        Vec4f gl_Vertex = embed<4>(model->vert(nface, nvert));
        gl_Vertex = uniform_M * gl_Vertex;
        varying_tri.set_col(nvert, gl_Vertex);
        ndc_tri.set_col(nvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        // 读取纹理贴图
        varying_uv.set_col(nvert, model->uv(nface, nvert));
        // 读取并处理法线贴图
        varying_nrm.set_col(nvert, proj<3>(uniform_MIT * embed<4>(model->normal(nface, nvert), 0.f)));
        return gl_Vertex;
    }

    /*
        fragment 片段着色器
        参数：重心坐标，颜色
        返回：bool 标志该片段是否丢弃，同时计算片段颜色
    */
    virtual bool fragment(Vec3f bc, TGAColor& color) {
        Vec2f uv = varying_uv * bc;
        Vec3f bn = (varying_nrm * bc).normalize();

        // shadow mapping
        Vec4f sb_p = uniform_MS * embed<4>(ndc_tri * bc); // 逆变换到 shadowmapping 所在坐标系
        sb_p = sb_p / sb_p[3];
        int idx = int(sb_p[0]) + int(sb_p[1]) * width; // index in the shadowbuffer array
        float shadow = .3 + .7 * (sb_p[2] - shadowbuffer[idx] > 0.01); // magic coeff to avoid z-fighting

        // 完成读取切线空间的法线贴图并将其变换到世界空间坐标
        mat<3, 3, float> A_I, B;
        A_I[0] = ndc_tri.col(1) - ndc_tri.col(0);
        A_I[1] = ndc_tri.col(2) - ndc_tri.col(0);
        A_I[2] = bn;
        A_I = A_I.invert();
        Vec3f i = A_I * Vec3f(varying_uv[0][1] - varying_uv[0][0], varying_uv[0][2] - varying_uv[0][0], 0);
        Vec3f j = A_I * Vec3f(varying_uv[1][1] - varying_uv[1][0], varying_uv[1][2] - varying_uv[1][0], 0);
        B.set_col(0, i.normalize());
        B.set_col(1, j.normalize());
        B.set_col(2, bn);
        Vec3f normal = (B * model->normal(uv)).normalize();

        Vec3f light = proj<3>(uniform_M * embed<4>(light_dir)).normalize();
        Vec3f eye = proj<3>(uniform_M * embed<4>(camera)).normalize();
        float diff = std::max(0.f, normal * light);
        // 半程向量与法线的夹角
        float spec = std::pow(std::max(0.f, (light + eye).normalize() * normal), model->specular(uv));
        TGAColor c = model->diffuse(uv);
        for (int i = 0; i < 3; i++) {
            color[i] = color[i] = std::min<float>(20 + c[i] * shadow * (1.2 * diff + .6 * spec), 255);
        }
        return false;
    }
};

struct DepthShader : public IShader {
    mat<3, 3, float> varying_tri;

    DepthShader() : varying_tri() {}

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
        gl_Vertex = Viewport * Projection * Modelview * gl_Vertex;
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bc, TGAColor& color) {
        Vec3f p = varying_tri * bc;
        color = TGAColor(255, 255, 255);
        color = color * (p.z / depth);
        return false;
    }
};

int main(int argc, char** argv) {
    std::string time = getUniqueString();
    std::string model_path = "C:/Users/memo2586/3D Objects/obj/";
    std::string model_name = "diablo3_pose.obj";
    std::string output_file_name = "memoRender_" + model_name + "_" + time + ".tga";
    std::string output_path = "C:/Users/memo2586/3D Objects/output/";

    model = new Model((model_path + model_name).c_str());
    zbuffer = new float[width * height];
    shadowbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max());
    light_dir.normalize();

    // shadow mapping
    {
        TGAImage depth_image(width, height, TGAImage::RGB);
        modelview(light_dir, center, up);
        viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        projection(0);

        DepthShader depthshader;
        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = depthshader.vertex(i, j);
            }
            triangle(screen_coords, depthshader, depth_image, shadowbuffer);
        }
        depth_image.flip_vertically(); // to place the origin in the bottom left corner of the image
        depth_image.write_tga_file("depth.tga");
    }

    Matrix M = Viewport * Projection * Modelview;   // 用于计算uniform_MS

    {
        TGAImage image(width, height, TGAImage::RGB);
        modelview(camera, center, up);
        viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
        projection(-1.f / (camera - center).norm());

        Shader shader(Projection * Modelview, (Projection * Modelview).invert_transpose(), M * ( Projection * Modelview).invert());

        Vec4f screen_coords[3];
        for (int i = 0; i < model->nfaces(); i++) {
            for (int j = 0; j < 3; j++) {
                screen_coords[j] = shader.vertex(i, j);
            }
            triangle_(shader.varying_tri, shader, image, zbuffer);
        }

        image.flip_vertically(); // to place the origin in the bottom left corner of the image
        image.write_tga_file((output_path + output_file_name).c_str());
    }

    delete[] shadowbuffer;
    delete[] zbuffer;
    delete model;
    return 0;
}
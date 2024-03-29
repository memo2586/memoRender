#include <iostream> 

#include "geometry.h"
#include "model.h"
#include "our_gl.h"
#include "tgaimage.h"

Model* model = nullptr;
const int width = 1600;
const int height = 1600;

Vec3f camera(1, 1, 3);
Vec3f center(0, 0, 0);
Vec3f light_dir(1, 1, 1);
Vec3f up(0, 1, 0);

struct GouraudShader : public IShader
{
    mat<2, 3, float> varying_uv;    // 存储三角形每个顶点的纹理坐标
    mat<4, 3, float> varying_tri;   // 齐次坐标下的三角形顶点
    mat<3, 3, float> ndc_tri;       // 标准坐标系下的三角形顶点
    mat<3, 3, float> varying_nrm;    // 存储三角形顶点法线
    mat<4, 4, float> unifrom_M;     // Projection*ModelView，用于将模型世界坐标转换到屏幕空间坐标
    mat<4, 4, float> unifrom_MIT;   // unifrom_M 的逆转置，用于变换法线方向以保证法线方向依旧垂直于变换后的表面

    /*
        vertex 顶点着色器
        参数: 面编号，顶点编号
        返回：Vec4f
        功能：1.根据接受的参数从模型中读取：顶点坐标、纹理坐标，并存储uv坐标
              2.计算并返回输入顶点经过MVP变换后的坐标
    */
    virtual Vec4f vertex(int nface, int nvert){
        // 处理顶点
        Vec4f gl_Vertex = embed<4>(model->vert(nface, nvert));
        gl_Vertex = unifrom_M * gl_Vertex;
        varying_tri.set_col(nvert, gl_Vertex);
        ndc_tri.set_col(nvert, proj<3>(gl_Vertex / gl_Vertex[3]));
        // 读取纹理贴图
        varying_uv.set_col(nvert, model->uv(nface, nvert));
        // 读取并处理法线贴图
        varying_nrm.set_col(nvert, proj<3>(unifrom_MIT * embed<4>(model->normal(nface, nvert), 0.f)));
        return gl_Vertex;
    }

    /*
        fragment 片段着色器
        参数：重心坐标，颜色
        返回：bool
        功能：纹理坐标插值、法线坐标系变换 & 光源坐标系变换、简化的phong模型
             最终经过以上计算得到颜色值，返回 bool 值标志该片段是否丢弃
    */
    virtual bool fragment(Vec3f bc, TGAColor &color){
        Vec2f uv = varying_uv * bc;
        Vec3f bn = (varying_nrm * bc).normalize();

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
        //Vec3f normal = proj<3>(unifrom_M * embed<4>(n)).normalize();

        Vec3f light = proj<3>(unifrom_M * embed<4>(light_dir)).normalize();
        Vec3f eye = proj<3>(unifrom_M * embed<4>(camera)).normalize();
        float diff = std::max(0.f, normal * light);
        // 半程向量与法线的夹角
        float spec = std::pow(std::max(0.f, (light + eye).normalize() * normal), model->specular(uv));
        TGAColor c = model->diffuse(uv);
        for (int i = 0; i < 3; i++) {
            color[i] = std::min((double)255.f, 5 + c[i] * (diff + 0.5 * spec));
        }
        return false;
    }
};

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("G:/memo/Code/tinyrenderer/Lesson_6_Shaders_for_the_software_renderer/obj/african_head.obj");
    }

    modelview(camera, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection(-1.f/(camera-center).norm());
    light_dir.normalize();

    TGAImage image  (width, height, TGAImage::RGB);
    float* zbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    GouraudShader shader;
    shader.unifrom_M = Projection * Modelview;
    shader.unifrom_MIT = shader.unifrom_M.invert_transpose();

    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(shader.varying_tri, shader, image, zbuffer);
    }

    image.  flip_vertically(); // to place the origin in the bottom left corner of the image
    image.  write_tga_file("output.tga");

    delete model;
    return 0;
}
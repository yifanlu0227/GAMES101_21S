#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // float n = -zNear;
    // float f = -zFar;

    // Eigen::Matrix4f M_pesp_to_orth = Eigen::Matrix4f::Zero();
    // M_pesp_to_orth(0, 0) = n;
    // M_pesp_to_orth(1, 1) = n;
    // M_pesp_to_orth(2, 2) = n+f;
    // M_pesp_to_orth(2, 3) = -n*f;
    // M_pesp_to_orth(3, 2) = 1;

    // float t = zNear * tan(eye_fov/2 *MY_PI/180);
    // float r = aspect_ratio * t;
    // float b = -t;
    // float l = -r;

    // Eigen::Matrix4f M_orth1 = Eigen::Matrix4f::Identity();
    // M_orth1(0, 0) = 1/r;
    // M_orth1(1, 1) = 1/t;
    // M_orth1(2, 2) = 2/(n-f);

    // Eigen::Matrix4f M_orth2 = Eigen::Matrix4f::Identity();
    // M_orth2(2, 3) = -(n+f)/2;

    // Eigen::Matrix4f M_orth = M_orth1 * M_orth2;

    // projection = M_orth * M_pesp_to_orth;

    // // https://zhuanlan.zhihu.com/p/509902950
    // // Eigen::Matrix4f inv_mat = Eigen::Matrix4f::Identity();
    // // inv_mat(2,2) = -1;

    // // projection = inv_mat * projection;

    // http://www.songho.ca/opengl/gl_projectionmatrix.htmls
    float n = zNear;
    float f = zFar;
    float t = zNear * tan(eye_fov/2 *MY_PI/180);
    float r = aspect_ratio * t;
    float b = -t;
    float l = -r;

    projection << n/r, 0, 0, 0,
                  0, n/t, 0, 0,
                  0, 0, -(f+n)/(f-n), -2*f*n/(f-n),
                  0, 0,-1, 0;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f vector_normal = normal.normalized();
        Eigen::Vector3f vector_light = (light.position - point).normalized();
        Eigen::Vector3f vector_view = (eye_pos - point).normalized();
        Eigen::Vector3f vector_half = (vector_light + vector_view).normalized();

        float r2 = (light.position - point).dot(light.position - point);

        Eigen::Vector3f L_d = kd.cwiseProduct(light.intensity / r2) * std::max(0.f, vector_normal.dot(vector_light));
        Eigen::Vector3f L_s = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.f, vector_normal.dot(vector_half)), p);
        Eigen::Vector3f L_a = ka.cwiseProduct(amb_light_intensity);

        result_color += (L_d + L_s + L_a);
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color; // diffuse color is the color of point inside triangle(interpolated.)
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}}; // Suppose in the view space.
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}}; // Suppose in the view space.

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // Eigen::Vector3f eye_pos{0, 0, 10};  // world coord? Here it should be view space coord.
    Eigen::Vector3f eye_pos{0, 0, 0};  // view space coord.

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos; // view space coord.
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f vector_normal = normal.normalized();
        Eigen::Vector3f vector_light = (light.position - point).normalized();
        Eigen::Vector3f vector_view = (eye_pos - point).normalized();
        Eigen::Vector3f vector_half = (vector_light + vector_view).normalized();

        float r2 = (light.position - point).dot(light.position - point);

        Eigen::Vector3f L_d = kd.cwiseProduct(light.intensity / r2) * std::max(0.f, vector_normal.dot(vector_light));
        Eigen::Vector3f L_s = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.f, vector_normal.dot(vector_half)), p);
        Eigen::Vector3f L_a = ka.cwiseProduct(amb_light_intensity);

        result_color += (L_d + L_s + L_a);
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    float x, y, z;
    Vector3f t, b;
    x = normal.x(), y = normal.y(), z = normal.z();
    t << x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z);
    b = normal.cross(t);

    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
           t.y(), b.y(), normal.y(),
           t.z(), b.z(), normal.z();

    float u, v, w, h;
    u = payload.tex_coords.x();
    v = payload.tex_coords.y();
    w = payload.texture->width;
    h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0 / w,v).norm() - payload.texture->getColor(u,v).norm());
    float dV = kh * kn * (payload.texture->getColor(u,v + 1.0 / h).norm() - payload.texture->getColor(u,v).norm());

    Vector3f ln;
    ln << -dU, dV, 1;

    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    point += (kn * normal * payload.texture->getColor(u, v).norm());

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f vector_normal = normal.normalized();
        Eigen::Vector3f vector_light = (light.position - point).normalized();
        Eigen::Vector3f vector_view = (eye_pos - point).normalized();
        Eigen::Vector3f vector_half = (vector_light + vector_view).normalized();

        float r2 = (light.position - point).dot(light.position - point);

        Eigen::Vector3f L_d = kd.cwiseProduct(light.intensity / r2) * std::max(0.f, vector_normal.dot(vector_light));
        Eigen::Vector3f L_s = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.f, vector_normal.dot(vector_half)), p);
        Eigen::Vector3f L_a = ka.cwiseProduct(amb_light_intensity);

        result_color += (L_d + L_s + L_a);

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    float x, y, z;
    Vector3f t, b;
    x = normal.x(), y = normal.y(), z = normal.z();
    t << x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z);
    b = normal.cross(t);

    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
           t.y(), b.y(), normal.y(),
           t.z(), b.z(), normal.z();

    float u, v, w, h;
    u = payload.tex_coords.x();
    v = payload.tex_coords.y();
    w = payload.texture->width;
    h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0 / w,v).norm() - payload.texture->getColor(u,v).norm());
    float dV = kh * kn * (payload.texture->getColor(u,v + 1.0 / h).norm() - payload.texture->getColor(u,v).norm());

    Vector3f ln;
    ln << -dU, dV, 1;

    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
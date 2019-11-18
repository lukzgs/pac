#version 330 core

// Atributos de vértice recebidos como entrada ("in") pelo Vertex Shader.
// Veja a função BuildTrianglesAndAddToVirtualScene() em "main.cpp".
layout (location = 0) in vec4 model_coefficients;
layout (location = 1) in vec4 normal_coefficients;
layout (location = 2) in vec2 texture_coefficients;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

#define SPHERE 0
#define BUNNY  1
#define PLANE  2
#define WALL   3
#define CRATE  4
#define CUBE   5
uniform int object_id;

uniform vec4 bbox_min;
uniform vec4 bbox_max;

uniform sampler2D brick;
uniform sampler2D obsidian;
uniform sampler2D pac;
uniform sampler2D pac3;
uniform sampler2D compcube;

bool hasTexture = false;
bool isPhong = false;

// phong stuff
out vec4 position_world_phong;
out vec4 position_model_phong;
out vec4 position_model_gouraud;
out vec4 normal;
out vec2 texcoords;

// gourad stuff
out vec3 color_gouraud;

void main()
{
    vec4 position_world;
    vec4 position_model;
    vec3 color;

    gl_Position = projection * view * model * model_coefficients;
    
    position_world = model * model_coefficients;
    position_world_phong = position_world;

    position_model = model_coefficients;
    position_model_phong = position_model;
    position_model_gouraud = position_model;

    // Normal do vértice atual no sistema de coordenadas global (World).
    // Veja slide 107 do documento "Aula_07_Transformacoes_Geometricas_3D.pdf".
    normal = inverse(transpose(model)) * normal_coefficients;
    normal.w = 0.0;

    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);

    vec4 camera_position = inverse(view) * origin;

    vec4 p = position_world;

    vec4 light_position = vec4(0.0, 2.0, 1.0, 1.0);
    vec4 light_way  = normalize(vec4 (0.0, -1.0, 0.0, 0.0));

    vec4 n = normalize(normal);
    vec4 l = normalize(light_position - p);
    vec4 v = normalize(camera_position - p);
    vec4 r = -l + 2*n*dot(n,l); 
    vec4 h = normalize(v+l);
    vec3 Kd; // Refletância difusa
    vec3 Ks; // Refletância especular
    vec3 Ka; // Refletância ambiente
    float q; // Expoente especular para o modelo de iluminação de Phong
    
    // Angulo alpha
    float alpha = cos(30*M_PI/180);
    // Angulo beta
    float beta = dot(-l,light_way); // Ângulo máximo do spotlight
    // Coordenadas de textura U e V
    float U = 0.0;
    float V = 0.0;

    texcoords = texture_coefficients; 

    if ( object_id == BUNNY )
    {
        hasTexture = false;
        vec4 bbox_center = (bbox_min + bbox_max) / 2.0;
        vec4 centerEdge_vector = (position_model_phong - bbox_center);
        float rho = sqrt(centerEdge_vector.x*centerEdge_vector.x +
                         centerEdge_vector.y*centerEdge_vector.y +
                         centerEdge_vector.z*centerEdge_vector.z);
        float theta = atan(centerEdge_vector.x,centerEdge_vector.z)+ M_PI;
        float phi = asin(centerEdge_vector.y/rho) + (M_PI/2);

        U = (theta)/(2*M_PI);
        V = (phi)/(M_PI);

        Kd = texture(pac, vec2(U,V)).rgb;
        Ks = vec3(0.0,0.0,0.0);
        Ka = vec3(0.4,0.2,0.04);
        q = 1.0;         
    }

    else {

        hasTexture = false;

        Kd = vec3(0.4,0.2,0.04);
        Ks = vec3(0.0,0.0,0.0);
        Ka = vec3(0.4,0.2,0.04);
        q = 1.0;
    }
        
    float lambert_diffuse_term = max(0,dot(n,l)); // PREENCHA AQUI o termo difuso de Lambert
    // Termo especular utilizando o modelo de iluminação de Phong
    float phong_specular_term  = pow(max(0,dot(r,v)),q); // PREENCH AQUI o termo especular de Phong
    // Espectro da fonte de iluminação
    vec3 light_spectrum = vec3(1.0,1.0,1.0); // PREENCH AQUI o espectro da fonte de luz
    // Espectro da luz ambiente
    vec3 ambient_light_spectrum = vec3(0.2,0.2,0.2); // PREENCHA AQUI o espectro da luz ambiente
    // Cor final do fragmento calculada com uma combinação dos termos difuso,
    // especular, e ambiente. Veja slide 131 do documento "Aula_17_e_18_Modelos_de_Iluminacao.pdf".
    
    color = hasTexture ?
            (Kd * light_spectrum * lambert_diffuse_term
                + Ka * ambient_light_spectrum
                + Ks * light_spectrum * phong_specular_term)
            : (Kd /** (pow(lambert_diffuse_term, 1) + 0.01)
                + Ka * ambient_light_spectrum
                + Ks * light_spectrum * phong_specular_term/*/);


    // Cor final com correção gamma, considerando monitor sRGB.
    // Veja https://en.wikipedia.org/w/index.php?title=Gamma_correction&oldid=751281772#Windows.2C_Mac.2C_sRGB_and_TV.2Fvideo_standard_gammas

    // Coordenadas de textura obtidas do arquivo OBJ (se existirem!)
    color_gouraud = pow(color, vec3(1.0,1.0,1.0)/2.2);
}


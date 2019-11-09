#version 330 core

// Atributos de fragmentos recebidos como entrada ("in") pelo Fragment Shader.
// Neste exemplo, este atributo foi gerado pelo rasterizador como a
// interpolação da posição global e a normal de cada vértice, definidas em
// "shader_vertex.glsl" e "main.cpp".
in vec4 position_world;
in vec4 normal;

// Posição do vértice atual no sistema de coordenadas local do modelo.
in vec4 position_model;

// Coordenadas de textura obtidas do arquivo OBJ (se existirem!)
in vec2 texcoords;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Identificador que define qual objeto está sendo desenhado no momento
#define SPHERE 0
#define BUNNY  1
#define PLANE  2
#define WALL   3
uniform int object_id;

// Parâmetros da axis-aligned bounding box (AABB) do modelo
uniform vec4 bbox_min;
uniform vec4 bbox_max;

uniform sampler2D brick;
uniform sampler2D obsidian;
uniform sampler2D pac;
uniform sampler2D pac3;

#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

// O valor de saída ("out") de um Fragment Shader é a cor final do fragmento.
out vec3 color;

void main()
{
    // Obtemos a posição da câmera utilizando a inversa da matriz que define o
    // sistema de coordenadas da câmera.
    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 camera_position = inverse(view) * origin;

    // O fragmento atual é coberto por um ponto que percente à superfície de um
    // dos objetos virtuais da cena. Este ponto, p, possui uma posição no
    // sistema de coordenadas global (World coordinates). Esta posição é obtida
    // através da interpolação, feita pelo rasterizador, da posição de cada
    // vértice.
    vec4 p = position_world;

    // Posição da fonte de luz
    vec4 light_position = vec4(0.0, 2.0, 1.0, 1.0);

    // Vetor que define a direção da luz spotlight
    vec4 light_way  = normalize(vec4 (0.0, -1.0, 0.0, 0.0));

    // Normal do fragmento atual, interpolada pelo rasterizador a partir das
    // normais de cada vértice.
    vec4 n = normalize(normal);

    // Vetor que define o sentido da fonte de luz em relação ao ponto atual.
    vec4 l = normalize(light_position - p);     

    // Vetor que define o sentido da câmera em relação ao ponto atual.
    vec4 v = normalize(camera_position - p);

    // Vetor que define o sentido da reflexão especular ideal.
    vec4 r = -l + 2*n*dot(n,l); // PREENCHA AQUI o vetor de reflexão especular ideal

    // Vetor que define o half vector.
    vec4 h = normalize(v+l); 

    // Parâmetros que definem as propriedades espectrais da superfície
    vec3 Kd; // Refletância difusa
    vec3 Ks; // Refletância especular
    vec3 Ka; // Refletância ambiente
    float q; // Expoente especular para o modelo de iluminação de Phong

    bool hasTexture = false; // Variável booleana auxiliar para identificar objetos com texturas
    
    // Angulo alpha
    float alpha = cos(30*M_PI/180); 
    
    // Angulo beta
    float beta = dot(-l,light_way); // Ângulo máximo do spotlight

    // Coordenadas de textura U e V
    float U = 0.0;
    float V = 0.0;
    
    if ( object_id == BUNNY )
    {
        hasTexture = true;

        vec4 bbox_center = (bbox_min + bbox_max) / 2.0;
        vec4 centerEdge_vector = (position_model - bbox_center);

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
    else if ( object_id == SPHERE )
    {
        hasTexture = true;

        float minx = bbox_min.x;
        float maxx = bbox_max.x;
        float miny = bbox_min.y;
        float maxy = bbox_max.y;
        float minz = bbox_min.z;
        float maxz = bbox_max.z;

        // pu = px-minx/maxx-minx
        // pv = py-minxy/maxy-miny
        U = (position_model.x - minx)/(maxx-minx);
        V = (position_model.y - miny)/(maxy-miny);    

        Kd = texture(pac3, vec2(U,V)).rgb;
        Ks = vec3(0.0,0.0,0.0);
        Ka = vec3(0.4,0.2,0.04);
        q = 1.0;   
        
    }
    else if ( object_id == PLANE )
    {
        hasTexture = true;
        vec4 bbox_center = (bbox_min + bbox_max) / 2.0;

        U = texcoords.x*10;
        V = texcoords.y*10;
        
        Kd = texture(brick, vec2(U,V)).rgb;
        Ks = vec3(0.1,0.1,0.1);
        Ka = vec3(0.3,0.3,0.3);
        q = 20.0;
        // Coordenadas de textura do plano, obtidas do arquivo OBJ.
        
    }

    else if ( object_id == WALL )
    {
        hasTexture = true;
        vec4 bbox_center = (bbox_min + bbox_max) / 2.0;

        U = texcoords.x*10;
        V = texcoords.y*10;
        
        Kd = texture(obsidian, vec2(U,V)).rgb;
        Ks = vec3(0.1,0.1,0.1);
        Ka = vec3(0.3,0.3,0.3);
        q = 20.0;
        // Coordenadas de textura do plano, obtidas do arquivo OBJ.
        
    }
/*
    // Obtemos a refletância difusa a partir da leitura da imagem TextureImage0
    vec3 Kd0 = texture(obsidian, vec2(U,V)).rgb;

    // Equação de Iluminação
    float lambert = max(0,dot(n,l));

    color = Kd0 * (lambert + 0.01);
    // Cor final com correção gamma, considerando monitor sRGB.
    // Veja https://en.wikipedia.org/w/index.php?title=Gamma_correction&oldid=751281772#Windows.2C_Mac.2C_sRGB_and_TV.2Fvideo_standard_gammas
    color = pow(color, vec3(1.0,1.0,1.0)/2.2);
} */
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
            (Kd /* * (pow(lambert_diffuse_term, 1) + 0.01)
                + Ka * ambient_light_spectrum
                + Ks * light_spectrum * phong_specular_term/*/)
            : (Kd * light_spectrum * lambert_diffuse_term
                + Ka * ambient_light_spectrum
                + Ks * light_spectrum * phong_specular_term);


    // Cor final com correção gamma, considerando monitor sRGB.
    // Veja https://en.wikipedia.org/w/index.php?title=Gamma_correction&oldid=751281772#Windows.2C_Mac.2C_sRGB_and_TV.2Fvideo_standard_gammas
    color = pow(color, vec3(1.0,1.0,1.0)/2.2);
}
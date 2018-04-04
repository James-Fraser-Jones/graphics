#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "glm/ext.hpp"
#include <omp.h>
using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;
using glm::ivec2;

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

vec4 cameraPos(0, 0, -3.001, 1); //removing the 0.001 will cause a crash to occour
vec4 cameraRot(0, 0, 0, 1);
vec4 cameraDir(0, 0, 1, 0);

struct Pixel {
    int x;
    int y;
    float z;
};

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
float focalLength = SCREEN_WIDTH/2;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void SafePutPixelSDL(screen* screen, int x, int y, vec3 color) {
  if ((x >= 0) && (x < SCREEN_WIDTH) && (y >= 0) && (y < SCREEN_HEIGHT)){
    PutPixelSDL(screen, x, y, color);
  }
}

void Update();
void Draw(screen* screen, const vector <Triangle>& triangles);
void InterpolatePixel( Pixel a, Pixel b, vector<Pixel>& result );

void TransformationMatrix(mat4& M, vec4 pos, vec4 rot){

    mat4 toOrigin (1,0,0,-pos.x,
                    0,1,0,-pos.y,
                    0,0,1,-pos.z,
                    0,0,0,1);

    /* //don't seem to need this at the moment
    mat4 toCamera (1,0,0,pos.x,
                    0,1,0,pos.y,
                    0,0,1,pos.z,
                    0,0,0,1);
    //*/

    mat4 rotationX(1,0,0,0,
                    0,cos(rot.x),-sin(rot.x),0,
                    0,sin(rot.x),cos(rot.x),0,
                    0,0,0,1);

    mat4 rotationY(cos(rot.y),0,sin(rot.y),0,
                    0,1,0,0,
                    -sin(rot.y),0,cos(rot.y),0,
                    0,0,0,1);

    mat4 rotationZ(cos(rot.z),-sin(rot.z),0,0,
                    sin(rot.z),cos(rot.z),0,0,
                    0,0,1,0,
                    0,0,0,1);

    M = toOrigin*(rotationZ*rotationY*rotationX);
}

void VertexShader(const vec4& v, Pixel& p){
    mat4 M;
    TransformationMatrix(M, cameraPos, cameraRot);
    vec4 localV = v*M; //for whatever reason they have to multiply this way around
    p.z = 1.0f/glm::abs(glm::length(cameraPos-v));
    cout << "In vertex Shader" << p.z << '\n';
    p.x = (focalLength * (localV.x*p.z) + (SCREEN_WIDTH/2));
    p.y = (focalLength * (localV.y*p.z) + (SCREEN_HEIGHT/2));
}
/*
void PixelShader( const Pixel& p ) {
    int x = p.x;
    int y = p.y;
    if( p.zinv > depthBuffer[y][x] ) {
        depthBuffer[y][x] = f.zinv;
        PutPixelSDL(screen, x, y, currentColor);
    }
}*/

void InterpolatePixel(Pixel a, Pixel b, vector<Pixel>& result) {
    int N = result.size(); //we have to know the size in advance
    //Pixel step, current;
    float x = (b.x - a.x) / float(max(N-1,1));
    float y = (b.y - a.y) / float(max(N-1,1));
    float z = (b.z - a.z) / float(max(N-1,1));

    float currentX = a.x;
    float currentY = a.y;
    float currentZ = a.z;

    for(int i=0; i<N; i++){
        result[i].x = currentX;
        result[i].y = currentY;
        result[i].z = currentZ;
        currentX = currentX+x;
        currentY = currentY+y;
        currentZ = currentZ+z;
    }
}

void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result){ //this returns every point on the line
    int N = result.size(); //we have to know the size in advance
    vec2 step = vec2(b-a) / float(max(N-1,1));
    vec2 current(a);
    for(int i=0; i<N; i++){
        result[i] = current;
        current += step;
    }
}

void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels) {
    //get maximum and minimum y values of the triangle
    int yMin = SCREEN_WIDTH, yMax = -1;
    for(int i = 0; i < 3; i++) {
        if(vertexPixels[i].y > yMax) {
            yMax = vertexPixels[i].y;
        }
        if(vertexPixels[i].y < yMin) {
            yMin = vertexPixels[i].y;
        }
    }

    //calculate number of rows needed to draw the triangle
    int ROWS = yMax - yMin + 1;
    leftPixels.resize(ROWS);
    rightPixels.resize(ROWS);

    //set default values for the vectors
    for(int i = 0; i < ROWS; i++) {
        //set left values as far right as possible and vice versa
        leftPixels[i].x = SCREEN_WIDTH;
        rightPixels[i].x = -1;

        //set y values to the height of the pixels that they represent
        leftPixels[i].y = yMin+i;
        rightPixels[i].y = yMin+i;
    }


    for (int q = 0; q < 3; q++){

        //hacky way of getting p and q to be every possible pair of vertices of the triangle
        int p = q + 1;
        if (q > 1){
          p = 0;
        }

        //get line between vertices p and q and ensure that it is long enough in y direction to have a pixel for every row
        int yDiff = glm::abs(vertexPixels[q].y - vertexPixels[p].y);
        vector<Pixel> line(yDiff+1);
        InterpolatePixel(vertexPixels[q], vertexPixels[p], line); //this is brok atm

        //use line to set x values for leftPixels and rightPixels
        for (int i = 0; i < line.size(); i++) {
            for (int j = 0; j < leftPixels.size(); j++) {
                if (line[i].y == leftPixels[j].y){
                    if (line[i].x > rightPixels[j].x){
                        rightPixels[j].x = line[i].x;
                        rightPixels[j].z = line[i].z;
                    }
                    if (line[i].x < leftPixels[j].x){
                        leftPixels[j].x = line[i].x;
                        leftPixels[j].z = line[i].z;
                    }
                }
            }
        }

    }
}

void DrawLineSDL(screen* screen, Pixel a, Pixel b, vec3 color) {
    Pixel deltaY;
    deltaY.y = glm::abs(a.y - b.y);
    Pixel deltaX;
    deltaX.x = glm::abs(a.x - b.x);

    int pixels = glm::max(deltaX.x, deltaY.y) + 1;
    vector<Pixel> line(pixels);
    InterpolatePixel(a, b, line);

    for (int i = 0; i < pixels; i++){
        if ((line[i].x >= 0) && (line[i].x < SCREEN_WIDTH) && (line[i].y >= 0) && (line[i].y < SCREEN_HEIGHT)) {
        SafePutPixelSDL(screen, line[i].x, line[i].y, color);
        }
    }
}

void DrawPolygonEdges( const vector<vec4>& vertices, screen* screen ){
    int V = vertices.size();
    // Transform each vertex from 3D world position to 2D image position:
    vector<Pixel> projectedVertices(V);
    for(int i=0; i<V; ++i) {
        VertexShader(vertices[i], projectedVertices[i]);
    }
    // Loop over all vertices and draw the edge from it to the next vertex:
    for(int i=0; i<V; ++i) {
        int j = (i+1)%V; // The next vertex
        vec3 color(1, 1, 1);
        DrawLineSDL(screen, projectedVertices[i], projectedVertices[j], color);
    }
}

void DrawRows(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, screen * screen, vec3 color) {
    for (int j = 0; j < leftPixels.size(); j++) {
        vector<Pixel> line(rightPixels[j].x - leftPixels[j].x +1);
        InterpolatePixel(leftPixels[j], rightPixels[j], line);
        for(int i = 0; i < line.size(); i++) {
            if(line[i].y >= 0 && line[i].x >= 0 && line[i].x < SCREEN_WIDTH && line[i].y < SCREEN_HEIGHT) {
                if (depthBuffer[line[i].y][line[i].x] < line[i].z) {
                    SafePutPixelSDL(screen, line[i].x, line[i].y, color);
                    depthBuffer[line[i].y][line[i].x] = line[i].z;
                }
            }
        }
    }
}

void DrawPolygon( const vector<vec4>& vertices, screen* screen, vec3 color){
    //map vector of 4d points into vector of 2d points
    int vs = vertices.size();
    vector<Pixel> vertexPixels(vs);
    for(int i = 0; i < vs; i++) {
        cout << "begin\n";
        VertexShader(vertices[i], vertexPixels[i]);
        cout << vertexPixels[i].z << '\n';
    }

    //fill leftPixels and rightPixels vectors
    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
    //cout << "here" << leftPixels[2].z << '\n';
    //draw the rows
    DrawRows(leftPixels, rightPixels, screen, color);
}

void DrawVertecies(screen* screen, vector<vec4> vertices){
    vec3 color(0.75f, 0.15f, 0.15f);

    int vs = vertices.size();
    vector<Pixel> vertexPixels(vs);
    for(int i = 0; i < vs; i++) {
        VertexShader(vertices[i], vertexPixels[i]);
        //cout << "x: " << vertexPixels[i].x << "\n" << "y: " << vertexPixels[i].y << "\n";
    }

    for (uint32_t i = 0; i < vertices.size(); i++){
      SafePutPixelSDL(screen, vertexPixels[i].x, vertexPixels[i].y-1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x, vertexPixels[i].y+1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x, vertexPixels[i].y, color);
      SafePutPixelSDL(screen, vertexPixels[i].x+1, vertexPixels[i].y-1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x+1, vertexPixels[i].y+1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x+1, vertexPixels[i].y, color);
      SafePutPixelSDL(screen, vertexPixels[i].x-1, vertexPixels[i].y-1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x-1, vertexPixels[i].y+1, color);
      SafePutPixelSDL(screen, vertexPixels[i].x-1, vertexPixels[i].y, color);
    }
}

/*Place your drawing here*/
void Draw(screen* screen, const vector <Triangle>& triangles){
    #pragma omp parallel for
    for( int y=0; y<SCREEN_HEIGHT; ++y ) {
        for( int x=0; x<SCREEN_WIDTH; ++x ) {
            depthBuffer[y][x] = 0;
        }
    }
    memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

    for( uint32_t i=0; i<triangles.size(); ++i ) {
        vector<vec4> vertices(3);
        vertices[0] = triangles[i].v0;
        vertices[1] = triangles[i].v1;
        vertices[2] = triangles[i].v2;
        //DrawPolygonEdges(vertices, screen);
        DrawPolygon(vertices, screen, triangles[i].color);
        //DrawVertecies(screen, vertices);
    }
}

/*Place updates of parameters here*/
void Update(){
    /* //Compute frame time
    static int t = SDL_GetTicks();
    int t2 = SDL_GetTicks();
    float dt = float(t2-t);
    t = t2;
    std::cout << "Render time: " << dt << " ms." << std::endl;
    //*/

    float lookSpeed = 0.02;
    float moveSpeed = 0.02;

    //Collect button inputs
    vec4 lookVector(0, 0, 0, 1);
    vec4 moveVector(0, 0, 0, 1);
    const uint8_t* keystate = SDL_GetKeyboardState( 0 );
    if(keystate[SDL_SCANCODE_LEFT]){
    lookVector.y = -1;
    }
    else if(keystate[SDL_SCANCODE_RIGHT]){
    lookVector.y = 1;
    }
    if(keystate[SDL_SCANCODE_UP]){
    lookVector.x = 1;
    }
    else if(keystate[SDL_SCANCODE_DOWN]){
    lookVector.x = -1;
    }
    if(keystate[SDL_SCANCODE_W]){
    moveVector.z = -1;
    }
    else if(keystate[SDL_SCANCODE_S]){
    moveVector.z = 1;
    }
    if(keystate[SDL_SCANCODE_A]){
    moveVector.x = 1;
    }
    else if(keystate[SDL_SCANCODE_D]){
    moveVector.x = -1;
    }
    if(keystate[SDL_SCANCODE_SPACE]){
    moveVector.y = 1;
    }
    else if(keystate[SDL_SCANCODE_LCTRL]){
    moveVector.y = -1;
    }

    //*
    //Modify global variables
    mat4 rotate, rotation, translate;

    TransformationMatrix(rotate, lookVector*lookSpeed, vec4(0,0,0,0)); //translation of vector which stores camera rotation values
    TransformationMatrix(rotation, vec4(0,0,0,0), cameraRot*rotate); //rotation by new camera rotation values
    TransformationMatrix(translate, rotation*moveVector*moveSpeed, vec4(0,0,0,0)); //translation of vector which stores camera position values

    cameraRot = cameraRot*rotate;
    cameraPos = cameraPos*translate;
    //*/
}

int main( int argc, char* argv[] ) {
    screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
    vector<Triangle> triangles;
    LoadTestModel(triangles);

    while( NoQuitMessageSDL() ){
        Update();
        Draw(screen, triangles);
        SDL_Renderframe(screen);
    }
    SDL_SaveImage(screen, "screenshot.bmp");
    KillSDL(screen);
    return 0;
}

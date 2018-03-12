#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "glm/ext.hpp"

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

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

vec4 cameraPos(0, 0, -3.001, 1); //removing the 0.001 will cause a crash to occour
vec4 cameraRot(0, 0, 0, 1);
vec4 cameraDir(0, 0, 1, 0);

float focalLength = SCREEN_WIDTH/2;

void Update();
void Draw(screen* screen, const vector <Triangle>& triangles);

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

void VertexShader(const vec4& v, ivec2& p){
  mat4 M;
  TransformationMatrix(M, cameraPos, cameraRot);

  vec4 localV = v*M; //for whatever reason they have to multiply this way around

  p.x = (focalLength * (localV.x/localV.z) + (SCREEN_WIDTH/2));
  p.y = (focalLength * (localV.y/localV.z) + (SCREEN_HEIGHT/2));
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

void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels) {
    //large yMin to be reduced to actual value, reverse for yMax
    int yMin = 2*SCREEN_HEIGHT, yMax = 0;
    for(int i = 0; i < 3; i++) {
        if(vertexPixels[i].y > yMax) {
            yMax = vertexPixels[i].y;
        }
        if(vertexPixels[i].y < yMin) {
            yMin = vertexPixels[i].y;
        }
    }
    int ROWS = yMax - yMin;
    leftPixels.resize(ROWS);
    rightPixels.resize(ROWS);
    for( int i=0; i<ROWS; ++i )
    {
        leftPixels[i].x = +numeric_limits<int>::max();
        rightPixels[i].x = -numeric_limits<int>::max();
    }
    //TODO:assign left and right pixels vars to actual values on triangle
}

void DrawLineSDL(screen* screen, ivec2 a, ivec2 b, vec3 color) {
  ivec2 delta = glm::abs(a - b);
  int pixels = glm::max(delta.x, delta.y) + 1;
  vector<ivec2> line(pixels);
  Interpolate(a, b, line);

  for (int i = 0; i < pixels; i++){
    if ((line[i].x >= 0) && (line[i].x < SCREEN_WIDTH) && (line[i].y >= 0) && (line[i].y < SCREEN_HEIGHT)) {
      PutPixelSDL(screen, line[i].x, line[i].y, color);
    }
  }
}

int main( int argc, char* argv[] ){
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
void DrawPolygonEdges( const vector<vec4>& vertices, screen* screen )
{
    int V = vertices.size();
    // Transform each vertex from 3D world position to 2D image position:
    vector<ivec2> projectedVertices(V);
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
void DrawRows(const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels) {

}
void DrawPolygon( const vector<vec4>& vertices )
{
    int V = vertices.size();
    vector<ivec2> vertexPixels( V );
    for( int i=0; i<V; ++i ) {
        VertexShader( vertices[i], vertexPixels[i] );
    }
    vector<ivec2> leftPixels;
    vector<ivec2> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    //DrawPolygonRows(leftPixels, rightPixels);
}
/*Place your drawing here*/
void Draw(screen* screen, const vector <Triangle>& triangles){

    memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

    for( uint32_t i=0; i<triangles.size(); ++i ) {
        vector<vec4> vertices(3);
        vertices[0] = triangles[i].v0;
        vertices[1] = triangles[i].v1;
        vertices[2] = triangles[i].v2;

        DrawPolygonEdges(vertices, screen);
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

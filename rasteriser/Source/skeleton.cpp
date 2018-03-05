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
using glm::ivec2;

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

vec4 cameraPos(0, 0, -3.001, 1);
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
  */

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

int main( int argc, char* argv[] ){

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen, triangles);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, const vector <Triangle>& triangles){
    /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( uint32_t i=0; i<triangles.size(); ++i ){

      vector<vec4> vertices(3);
      vertices[0] = triangles[i].v0;
      vertices[1] = triangles[i].v1;
      vertices[2] = triangles[i].v2;
      for(int v=0; v<3; ++v){
          ivec2 projPos;
          VertexShader(vertices[v], projPos);
          vec3 color(1,1,1);
          if ((projPos.x >= 0) && (projPos.x < SCREEN_WIDTH) && (projPos.y >= 0) && (projPos.y < SCREEN_HEIGHT)){
            PutPixelSDL(screen, projPos.x, projPos.y, color);
          }
      }
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

  //Modify global variables
  mat4 rotate, rotation, translate;

  TransformationMatrix(rotate, lookVector*lookSpeed, vec4(0,0,0,0)); //translation
  TransformationMatrix(rotation, vec4(0,0,0,0), cameraRot*rotate); //rotation
  TransformationMatrix(translate, rotation*moveVector*moveSpeed, vec4(0,0,0,0)); //translation

  cameraRot = cameraRot*rotate;
  cameraPos = cameraPos*translate;
}

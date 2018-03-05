#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);

void TransformationMatrix(glm::mat4x4 M, float x, float y, float z, float pitch, float yaw, float roll){

  mat4 toOrigin (1,0,0,-x,
                 0,1,0,-y,
                 0,0,1,-z,
                 0,0,0,1);

 mat4 toCamera (1,0,0,x,
                0,1,0,y,
                0,0,1,z,
                0,0,0,1);

  mat4 rotationX(1,0,0,0,
                 0,cos(pitch),-sin(pitch),0,
                 0,sin(pitch),cos(pitch),0,
                 0,0,0,1);

  mat4 rotationY(cos(yaw),0,sin(yaw),0,
                 0,1,0,0,
                 -sin(yaw),0,cos(yaw),0,
                 0,0,0,1);

  mat4 rotationZ(cos(roll),-sin(roll),0,0,
                 sin(roll),cos(roll),0,0,
                 0,0,1,0,
                 0,0,0,1);

  M = toCamera*((rotationZ*rotationY*rotationX)*toOrigin);
}

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
    /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( uint32_t i=0; i<triangles.size(); ++i )
  {
      vector<vec4> vertices(3);
      vertices[0] = triangles[i].v0;
      vertices[1] = triangles[i].v1;
      vertices[2] = triangles[i].v2;
      for(int v=0; v<3; ++v)
      {
          ivec2 projPos;
          VertexShader( vertices[v], projPos );
          vec3 color(1,1,1);
          PutPixelSDL( screen, projPos.x, projPos.y, color );
      }
  }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

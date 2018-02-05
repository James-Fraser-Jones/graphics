#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Intersection{
  vec3 position;
  float distance;
  int triangleIndex;
};

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

float camX = 0.0;
float camY = 0.0;
float camZ = -3.0;

glm::mat4 rotation;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen, const vector<Triangle>& triangles);
vec3 DirectLight(const Intersection& i, vec4 lightPos, vec3 lightColor);
//void Rotation()

bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle>& triangles, Intersection& closestIntersection){
  bool intersection = false;

  for (uint32_t i=0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];

    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;
    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);
    mat3 A(-dir, e1, e2);
    vec3 x = glm::inverse(A) * b;

    float t = x.x;
    float u = x.y;
    float v = x.z;

    if ((t >= 0) && (u >= 0) && (v >= 0) && (u + v <= 1)){ //use less than or equal to here, instead of less than
      intersection = true;

      if (t <= closestIntersection.distance){
        closestIntersection.position = (start + (dir * t));
        closestIntersection.distance = t;
        closestIntersection.triangleIndex = i;
      }
    }
  }

  return intersection;
}

int main(int argc, char* argv[]){
  //Initialize the screen
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  //Load the test model into an empty vector of triangles
  vector<Triangle> triangles;
  LoadTestModel(triangles);
  //Enter the rendering loop
  while(NoQuitMessageSDL()){
      Update();
      Draw(screen, triangles);
      SDL_Renderframe(screen);
  }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, const vector<Triangle>& triangles){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));


  //lightSource
  vec4 lightPos( 0, -0.5, -0.7, 1.0 );
  vec3 lightColor = 14.f * vec3( 1, 1, 1 );
  //light power at point = P/4*{Pi}*r^2

  //Loop over all pixels in the image and calculate a ray for each one.
  for (int y = 0; y < SCREEN_HEIGHT; y++){ //don't use unsigned ints here!!!
    for (int x = 0; x < SCREEN_WIDTH; x++){

      float focalLength = SCREEN_WIDTH; //this is the correct focal length

      //vec4 cameraPos(0.0, 0.0, -3.0, 1.0);
      vec3 start(camX, camY, camZ);
      vec3 dir(x-(SCREEN_WIDTH/2), y-(SCREEN_HEIGHT/2), focalLength);

      Intersection closestIntersection = {start, std::numeric_limits<float>::max(), -1};

      if (ClosestIntersection(start, dir, triangles, closestIntersection)){
        vec3 colour = triangles[closestIntersection.triangleIndex].color;
        vec3 bwColour = DirectLight(closestIntersection, lightPos, lightColor);
        PutPixelSDL(screen, x, y, bwColour);
      }
    }
  }
}

//return resulting direct illumination
vec3 DirectLight(const Intersection& i, vec4 lightPos, vec3 lightColor) {
    vec4 squaredEuclidDistance;
    squaredEuclidDistance.x = (lightPos.x - i.position.x) * (lightPos.x - i.position.x);
    squaredEuclidDistance.y = (lightPos.y - i.position.y) * (lightPos.y - i.position.y);
    squaredEuclidDistance.z = (lightPos.z - i.position.z) * (lightPos.z - i.position.z);

    int temp = lightColor.x / (4 * M_PI * sqrt(squaredEuclidDistance.x+squaredEuclidDistance.y+squaredEuclidDistance.z));
    lightColor.x = temp;
    lightColor.y = temp;
    lightColor.z = temp;
    return lightColor;
}

/*Place updates of parameters here*/
void Update(){
  /* Compute frame time */
  static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  /* Display render time */
  std::cout << "Render time: " << dt << " ms." << std::endl;

  float speed = 0.01;

  /* Update variables*/
  const uint8_t* keystate = SDL_GetKeyboardState( 0 );
  if(keystate[SDL_SCANCODE_UP]){
    camY -= speed;
  }
  if(keystate[SDL_SCANCODE_DOWN]){
    camY += speed;
  }
  if(keystate[SDL_SCANCODE_LEFT]){
    camX -= speed;
  }
  if(keystate[SDL_SCANCODE_RIGHT]){
    camX += speed;
  }
}

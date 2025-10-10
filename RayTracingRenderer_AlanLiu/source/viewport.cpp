//-------------------------------------------------------------------------------
///
/// \file       viewport.cpp 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    7.0
/// \date       September 24, 2025
///
/// \brief Example source for CS 6620 - University of Utah.
///
/// Copyright (c) 2019 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <stdlib.h>
#include <time.h>

#include "renderer.h"
#include "objects.h"
#include "lights.h"
#include "materials.h"
#include "texture.h"

#ifdef USE_GLUT
# ifdef __APPLE__
#  include <GLUT/glut.h>
# else
#  include <GL/glut.h>
# endif
#else
# include <GL/freeglut.h>
#endif

//-------------------------------------------------------------------------------
// How to use:

// Call this function to open a window that shows the scene to be rendered.
// If beginRendering is true, rendering begins as soon as the window opens
// and the window closes when the rendering is finishes.
void ShowViewport( Renderer *renderer, bool beginRendering );

// The window also handles mouse and keyboard input:
static const char *uiControlsString =
 "F1    - Shows help.\n"
 "F5    - Reloads the scene file.\n"
 "1     - Shows OpenGL view.\n"
 "2     - Shows the rendered image.\n"
 "3     - Shows the z (depth) image.\n"
 "Space - Starts/stops rendering.\n"
 "Esc   - Terminates software.\n"
 "Mouse Left Click - Writes the pixel information to the console.\n";

//-------------------------------------------------------------------------------
// OpenGL Extensions:
#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
# define GL_TEXTURE_MAX_ANISOTROPY_EXT 0x84FE
#endif
//-------------------------------------------------------------------------------

Renderer *theRenderer = nullptr;

//-------------------------------------------------------------------------------

#define WINDOW_TITLE        "Ray Tracer - CS 6620"
#define WINDOW_TITLE_OPENGL WINDOW_TITLE " - OpenGL"
#define WINDOW_TITLE_IMAGE  WINDOW_TITLE " - Rendered Image"
#define WINDOW_TITLE_Z      WINDOW_TITLE " - Z (Depth) Image"

enum Mode {
	MODE_READY,			// Ready to render
	MODE_RENDERING,		// Rendering the image
	MODE_RENDER_DONE	// Rendering is finished
};

enum ViewMode
{
	VIEWMODE_OPENGL,
	VIEWMODE_IMAGE,
	VIEWMODE_Z,
};

enum MouseMode {
	MOUSEMODE_NONE,
	MOUSEMODE_DEBUG,
};

static Mode      mode      = MODE_READY;		// Rendering mode
static ViewMode	 viewMode  = VIEWMODE_OPENGL;	// Display mode
static MouseMode mouseMode = MOUSEMODE_NONE;	// Mouse mode
static int       startTime;						// Start time of rendering
static GLuint    viewTexture;
static bool      closeWhenDone;
static GLint     maxLights;

MtlBlinn defaultMaterial;

#define terminal_clear()      printf("\033[H\033[J")
#define terminal_goto(x,y)    printf("\033[%d;%dH", (y), (x))
#define terminal_erase_line() printf("\33[2K\r")

//-------------------------------------------------------------------------------

void GlutDisplay();
void GlutReshape(int w, int h);
void GlutIdle();
void GlutKeyboard(unsigned char key, int x, int y);
void GlutKeyboard2(int key, int x, int y);
void GlutMouse(int button, int state, int x, int y);
void GlutMotion(int x, int y);

void BeginRendering( int value=0 );

//-------------------------------------------------------------------------------

void ShowViewport( Renderer *renderer, bool beginRendering )
{
	theRenderer = renderer;

	Scene &scene = renderer->GetScene();
	Camera &camera = renderer->GetCamera();

#ifdef _WIN32
	SetProcessDPIAware();
#endif

	int argc = 1;
	char argstr[] = "raytrace";
	char *argv = argstr;
	glutInit(&argc,&argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	if (glutGet(GLUT_SCREEN_WIDTH) > 0 && glutGet(GLUT_SCREEN_HEIGHT) > 0){
		glutInitWindowPosition( (glutGet(GLUT_SCREEN_WIDTH) - camera.imgWidth)/2, (glutGet(GLUT_SCREEN_HEIGHT) - camera.imgHeight)/2 );
	}
	else glutInitWindowPosition( 50, 50 );
	glutInitWindowSize(camera.imgWidth, camera.imgHeight);
#ifdef FREEGLUT
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

	glutCreateWindow(WINDOW_TITLE_OPENGL);
	glutDisplayFunc (GlutDisplay);
	glutReshapeFunc (GlutReshape);
	glutIdleFunc    (GlutIdle);
	glutKeyboardFunc(GlutKeyboard);
	glutSpecialFunc (GlutKeyboard2);
	glutMouseFunc   (GlutMouse);
	glutMotionFunc  (GlutMotion);

	Color bg = scene.background.GetValue();
	glClearColor(bg.r,bg.g,bg.b,0);

	glEnable( GL_CULL_FACE );

	float zero[] = {0,0,0,0};
	glLightModelfv( GL_LIGHT_MODEL_AMBIENT, zero );
	glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, 1 );
	glGetIntegerv(GL_MAX_LIGHTS, &maxLights);

	glEnable(GL_NORMALIZE);

	glGenTextures(1,&viewTexture);
	glBindTexture(GL_TEXTURE_2D, viewTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	if ( beginRendering ) {
		glutTimerFunc( 30, BeginRendering, 1 );
	}

	glutMainLoop();
}

//-------------------------------------------------------------------------------

void InitProjection()
{
	Scene &scene = theRenderer->GetScene();
	Camera &camera = theRenderer->GetCamera();
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	float r = float(camera.imgWidth) / float(camera.imgHeight);
	Box const &box = scene.rootNode.GetChildBoundBox();
	float len = (box.pmax - box.pmin).Length();
	gluPerspective( camera.fov, r, len/100000, len );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
}

//-------------------------------------------------------------------------------

void GlutReshape( int w, int h )
{
	Camera &camera = theRenderer->GetCamera();
	if( w != camera.imgWidth || h != camera.imgHeight ) {
		glutReshapeWindow( camera.imgWidth, camera.imgHeight );
	} else {
		glViewport( 0, 0, w, h );
		InitProjection();
	}
}

//-------------------------------------------------------------------------------

void DrawNode( Node const *node )
{
	glPushMatrix();

	const Material *mtl = node->GetMaterial();
	if ( !mtl ) mtl = &defaultMaterial;
	mtl->SetViewportMaterial();

	Matrix4f m(node->GetTransform());
	glMultMatrixf( m.cell );

	const Object *obj = node->GetNodeObj();
	if ( obj ) obj->ViewportDisplay(mtl);

	for ( int i=0; i<node->GetNumChild(); i++ ) {
		DrawNode( node->GetChild(i) );
	}

	glPopMatrix();
}

//-------------------------------------------------------------------------------

void DrawScene( bool flipped=false )
{
	Scene &scene = theRenderer->GetScene();
	Camera &camera = theRenderer->GetCamera();

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	if ( flipped ) {
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glScalef(1,-1,1);
		glMatrixMode(GL_MODELVIEW);
		glFrontFace(GL_CW);
	}

	const TextureMap *bgMap = scene.background.GetTexture();
	if ( bgMap ) {
		glDepthMask(GL_FALSE);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		if ( flipped ) glScalef(1,-1,1);
		glMatrixMode(GL_MODELVIEW);
		Color c = scene.background.GetValue();
		glColor3f(c.r,c.g,c.b);
		if ( bgMap->SetViewportTexture() ) {
			glEnable( GL_TEXTURE_2D );
			glMatrixMode( GL_TEXTURE );
			Matrix4f m( bgMap->GetInverseTransform() );
			glLoadMatrixf( m.cell );
			glMatrixMode( GL_MODELVIEW );
		} else {
			glDisable( GL_TEXTURE_2D );
		}
		const float y = 1;
		glBegin(GL_QUADS);
		glTexCoord2f(0,1);
		glVertex2f(-1,-y);
		glTexCoord2f(1,1);
		glVertex2f( 1,-y);
		glTexCoord2f(1,0);
		glVertex2f( 1, y);
		glTexCoord2f(0,0);
		glVertex2f(-1, y);
		glEnd();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glDepthMask(GL_TRUE);

		glDisable( GL_TEXTURE_2D );
	}

	glEnable( GL_LIGHTING );
	glEnable( GL_DEPTH_TEST );

	glPushMatrix();
	Vec3f p = camera.pos;
	Vec3f t = camera.pos + camera.dir;
	Vec3f u = camera.up;
	gluLookAt( p.x, p.y, p.z,  t.x, t.y, t.z,  u.x, u.y, u.z );

	int nLights = 1;
	if ( scene.lights.size() > 0 ) {
		nLights = Min( (int)scene.lights.size(), maxLights );
		for ( int i=0; i<nLights; i++ ) {
			scene.lights[i]->SetViewportLight(i);
		}
	} else {
		// Default lighting for scenes without a light
		float white[] = {1,1,1,1};
		float black[] = {0,0,0,0};
		Vec4f p(camera.pos, 1);
		glEnable ( GL_LIGHT0 );
		glLightfv( GL_LIGHT0, GL_AMBIENT,  black );
		glLightfv( GL_LIGHT0, GL_DIFFUSE,  white );
		glLightfv( GL_LIGHT0, GL_SPECULAR, white );
		glLightfv( GL_LIGHT0, GL_POSITION, &p.x );
	}
	for ( int i=nLights; i<maxLights; i++ ) {
		glDisable ( GL_LIGHT0 + i );
	}

	DrawNode( &scene.rootNode );

	glPopMatrix();

	if ( flipped ) {
		glFrontFace(GL_CCW);
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );
	glDisable( GL_TEXTURE_2D );
}

//-------------------------------------------------------------------------------

void DrawImage( void const *data, GLenum type, GLenum format )
{
	RenderImage &renderImage = theRenderer->GetRenderImage();

	glBindTexture(GL_TEXTURE_2D, viewTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, renderImage.GetWidth(), renderImage.GetHeight(), 0, format, type, data); 

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	glEnable(GL_TEXTURE_2D);

	glMatrixMode( GL_TEXTURE );
	glLoadIdentity();

	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();

	glColor3f(1,1,1);
	glBegin(GL_QUADS);
	glTexCoord2f(0,1);
	glVertex2f(-1,-1);
	glTexCoord2f(1,1);
	glVertex2f(1,-1);
	glTexCoord2f(1,0);
	glVertex2f(1,1);
	glTexCoord2f(0,0);
	glVertex2f(-1,1);
	glEnd();

	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );

	glDisable(GL_TEXTURE_2D);
}

//-------------------------------------------------------------------------------

void DrawProgressBar( float done, int height )
{
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();

	float y = -1 + 1.0f/height;
	glBegin(GL_LINES);
	glColor3f(1,1,1);
	glVertex2f(-1,y);
	glVertex2f(done*2-1,y);
	glColor3f(0,0,0);
	glVertex2f(done*2-1,y);
	glVertex2f(1,y);
	glEnd();

	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
}

//-------------------------------------------------------------------------------

void DrawRenderProgressBar()
{
	RenderImage &renderImage = theRenderer->GetRenderImage();
	int rp = renderImage.GetNumRenderedPixels();
	int np = renderImage.GetWidth() * renderImage.GetHeight();
	if ( rp >= np ) return;
	float done = (float) rp / (float) np;
	DrawProgressBar( done, renderImage.GetHeight() );
}

//-------------------------------------------------------------------------------

void GlutDisplay()
{
	Camera &camera = theRenderer->GetCamera();
	RenderImage &renderImage = theRenderer->GetRenderImage();

	switch ( viewMode ) {
	case VIEWMODE_OPENGL:
		DrawScene();
		break;
	case VIEWMODE_IMAGE:
		DrawImage( renderImage.GetPixels(), GL_UNSIGNED_BYTE, GL_RGB );
		break;
	case VIEWMODE_Z:
		renderImage.ComputeZBufferImage();
		DrawImage( renderImage.GetZBufferImage(), GL_UNSIGNED_BYTE, GL_LUMINANCE );
		break;
	}
	if ( mode == MODE_RENDERING ) DrawRenderProgressBar();

	glutSwapBuffers();
}

//-------------------------------------------------------------------------------

void GlutIdle()
{
	RenderImage &renderImage = theRenderer->GetRenderImage();

	static int lastRenderedPixels = 0;
	if ( mode == MODE_RENDERING ) {
		int nrp = renderImage.GetNumRenderedPixels();
		if ( lastRenderedPixels != nrp ) {
			lastRenderedPixels = nrp;
			if ( renderImage.IsRenderDone() ) {
				if ( ! closeWhenDone ) mode = MODE_RENDER_DONE;
				int endTime = (int) time(nullptr);
				int t = endTime - startTime;
				int h = t / 3600;
				int m = (t % 3600) / 60;
				int s = t % 60;
				printf("\nRender time is %d:%02d:%02d.\n",h,m,s);
			}
			glutPostRedisplay();
		}
		if ( closeWhenDone && !theRenderer->IsRendering() ) {
			mode = MODE_RENDER_DONE;
#ifdef FREEGLUT
			glutIdleFunc(nullptr);
			glutLeaveMainLoop();
#endif
		}
	}
}

//-------------------------------------------------------------------------------

void SwitchToOpenGLView()
{
	viewMode = VIEWMODE_OPENGL;
	glutSetWindowTitle(WINDOW_TITLE_OPENGL);
	glutPostRedisplay();
}

//-------------------------------------------------------------------------------

void GlutKeyboard(unsigned char key, int x, int y)
{
	switch ( key ) {
	case 27:	// ESC
		exit(0);
		break;
	case ' ':
		switch ( mode ) {
		case MODE_READY: 
			BeginRendering();
			break;
		case MODE_RENDERING:
			theRenderer->StopRender();
			mode = MODE_READY;
			glutPostRedisplay();
			break;
		case MODE_RENDER_DONE: 
			mode = MODE_READY;
			SwitchToOpenGLView();
			break;
		}
		break;
	case '1':
		SwitchToOpenGLView();
		break;
	case '2':
		viewMode = VIEWMODE_IMAGE;
		glutSetWindowTitle(WINDOW_TITLE_IMAGE);
		glutPostRedisplay();
		break;
	case '3':
		viewMode = VIEWMODE_Z;
		glutSetWindowTitle(WINDOW_TITLE_Z);
		glutPostRedisplay();
		break;
	}
}

//-------------------------------------------------------------------------------

void GlutKeyboard2( int key, int x, int y )
{
	switch ( key ) {
	case GLUT_KEY_F1:
		terminal_clear();
		printf("The following keys and mouse events are supported:\n");
		printf(uiControlsString);
		break;
	case GLUT_KEY_F5:
		if ( mode == MODE_RENDERING ) {
			printf("ERROR: Cannot reload the scene while rendering.\n");
		} else {
			if ( theRenderer->SceneFileName().empty() ) {
				printf("ERROR: No scene loaded.\n");
			} else {
				const char *filename = theRenderer->SceneFileName().c_str();
				printf("Reloading scene %s...\n", filename);
				if ( theRenderer->LoadScene(filename) ) {
					printf("Done.\n");
					Camera &camera = theRenderer->GetCamera();
					glutReshapeWindow( camera.imgWidth, camera.imgHeight );
					InitProjection();
					SwitchToOpenGLView();
					mode = MODE_READY;
				}
			}
		}
		break;
	}
}

//-------------------------------------------------------------------------------

void PrintPixelData(int x, int y)
{
	RenderImage &renderImage = theRenderer->GetRenderImage();

	if ( x >= 0 && y >= 0 && x < renderImage.GetWidth() && y < renderImage.GetHeight() ) {
		Color24 *colors = renderImage.GetPixels();
		float *zbuffer = renderImage.GetZBuffer();
		int i = y*renderImage.GetWidth() + x;
		printf("   Pixel: %4d, %4d\n   Color:  %3d,  %3d,  %3d\n", x, y, colors[i].r, colors[i].g, colors[i].b );
		terminal_erase_line();
		if ( zbuffer[i] == BIGFLOAT ) printf("Z-Buffer: max\n" );
		else printf("Z-Buffer: %f\n", zbuffer[i] );
	}
}

//-------------------------------------------------------------------------------

void GlutMouse(int button, int state, int x, int y)
{
	if ( state == GLUT_UP ) {
		mouseMode = MOUSEMODE_NONE;
	} else {
		switch ( button ) {
			case GLUT_LEFT_BUTTON:
				mouseMode = MOUSEMODE_DEBUG;
				terminal_clear();
				PrintPixelData(x,y);
				break;
		}
	}
}

//-------------------------------------------------------------------------------

void GlutMotion(int x, int y)
{
	switch ( mouseMode ) {
		case MOUSEMODE_DEBUG:
			terminal_goto(0,0);
			PrintPixelData(x,y);
			break;
	}
}

//-------------------------------------------------------------------------------

void BeginRendering( int value )
{
	Camera &camera = theRenderer->GetCamera();
	RenderImage &renderImage = theRenderer->GetRenderImage();

	mode = MODE_RENDERING;
	viewMode = VIEWMODE_IMAGE;
	glutSetWindowTitle(WINDOW_TITLE_IMAGE);
	DrawScene(true);
	glReadPixels( 0, 0, renderImage.GetWidth(), renderImage.GetHeight(), GL_RGB, GL_UNSIGNED_BYTE, renderImage.GetPixels() );
	startTime = (int) time(nullptr);
	theRenderer->BeginRender();
	closeWhenDone = value;
}

//-------------------------------------------------------------------------------
// Viewport Methods for various classes
//-------------------------------------------------------------------------------
void Sphere::ViewportDisplay( Material const *mtl ) const
{
	static GLUquadric *q = nullptr;
	if ( q == nullptr ) {
		q = gluNewQuadric();
		gluQuadricTexture(q,true);
	}
	gluSphere(q,1,50,50);
}
void Plane::ViewportDisplay( Material const *mtl ) const
{
	const int resolution = 32;
	float xyInc = 2.0f / resolution;
	float uvInc = 1.0f / resolution;
	glPushMatrix();
	glNormal3f(0,0,1);
	glBegin(GL_QUADS);
	float y1=-1, y2=xyInc-1, v1=0, v2=uvInc;
	for ( int y=0; y<resolution; y++ ) {
		float x1=-1, x2=xyInc-1, u1=0, u2=uvInc;
		for ( int x=0; x<resolution; x++ ) {
			glTexCoord2f( u1, v1 );
			glVertex3f  ( x1, y1, 0 );
			glTexCoord2f( u2, v1 );
			glVertex3f  ( x2, y1, 0 );
			glTexCoord2f( u2, v2 );
			glVertex3f  ( x2, y2, 0 );
			glTexCoord2f( u1, v2 );
			glVertex3f  ( x1, y2, 0 );
			x1=x2; x2+=xyInc; u1=u2; u2+=uvInc;
		}
		y1=y2; y2+=xyInc; v1=v2; v2+=uvInc;
	}
	glEnd();
	glPopMatrix();
}
void TriObj::ViewportDisplay( Material const *mtl ) const
{
	unsigned int nextMtlID = 0;
	unsigned int nextMtlSwith = NF();
	if ( mtl && NM() > 0 ) {
		mtl->SetViewportMaterial(0);
		nextMtlSwith = GetMaterialFaceCount(0);
		nextMtlID = 1;
	}
	glBegin(GL_TRIANGLES);
	for ( unsigned int i=0; i<NF(); i++ ) {
		while ( i >= nextMtlSwith ) {
			if ( nextMtlID >= NM() ) nextMtlSwith = NF();
			else {
				glEnd();
				nextMtlSwith += GetMaterialFaceCount(nextMtlID);
				mtl->SetViewportMaterial(nextMtlID);
				nextMtlID++;
				glBegin(GL_TRIANGLES);
			}
		}
		for ( int j=0; j<3; j++ ) {
			if ( HasTextureVertices() ) glTexCoord3fv( &VT( FT(i).v[j] ).x );
			if ( HasNormals() ) glNormal3fv( &VN( FN(i).v[j] ).x );
			glVertex3fv( &V( F(i).v[j] ).x );
		}
	}
	glEnd();
}
void GenLight::SetViewportParam( int lightID, ColorA const &ambient, ColorA const &intensity, Vec4f const &pos ) const
{
	glEnable ( GL_LIGHT0 + lightID );
	glLightfv( GL_LIGHT0 + lightID, GL_AMBIENT,  &ambient.r );
	glLightfv( GL_LIGHT0 + lightID, GL_DIFFUSE,  &intensity.r );
	glLightfv( GL_LIGHT0 + lightID, GL_SPECULAR, &intensity.r );
	glLightfv( GL_LIGHT0 + lightID, GL_POSITION, &pos.x );
}
void SetDiffuseTextureMap( TextureMap const *dm )
{
	if ( dm && dm->SetViewportTexture() ) {
		glEnable( GL_TEXTURE_2D );
		glMatrixMode( GL_TEXTURE );
		Matrix4f m( dm->GetInverseTransform() );
		glLoadMatrixf( m.cell );
		glMatrixMode( GL_MODELVIEW );
	} else {
		glDisable( GL_TEXTURE_2D );
	}
}
void MtlPhong::SetViewportMaterial( int subMtlID ) const
{
	ColorA d(diffuse.GetValue());
	ColorA s(specular.GetValue());
	float g = glossiness.GetValue();
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, &d.r );
	glMaterialfv( GL_FRONT, GL_SPECULAR, &s.r );
	glMaterialf ( GL_FRONT, GL_SHININESS, g*2 );
	SetDiffuseTextureMap( diffuse.GetTexture() );
}
void MtlBlinn::SetViewportMaterial( int subMtlID ) const
{
	ColorA d(diffuse.GetValue());
	ColorA s(specular.GetValue());
	float g = glossiness.GetValue();
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, &d.r );
	glMaterialfv( GL_FRONT, GL_SPECULAR, &s.r );
	glMaterialf ( GL_FRONT, GL_SHININESS, g );
	SetDiffuseTextureMap( diffuse.GetTexture() );
}
void MtlMicrofacet::SetViewportMaterial( int subMtlID ) const
{
	const Color bc    = baseColor.GetValue();
	const float rough = roughness.GetValue();
	const float metal = metallic .GetValue();
	float ff = (ior-1)/(ior+1);
	float f0d = ff*ff;
	Color f0 = (1 - metal)*f0d + metal*bc;
	float a = rough * rough;
	float k = a / 2;
	float D = 1 / ( Pi<float>() * a*a );
	float t = a*(3 - 2*rough);
	ColorA d( bc*( (1-metal)*(1-f0) + metal*t*0.25f )/Pi<float>() );
	ColorA s( f0*D/4 );
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, &d.r );
	glMaterialfv( GL_FRONT, GL_SPECULAR, &s.r );
	glMaterialf ( GL_FRONT, GL_SHININESS, (1-rough)*128 );
	SetDiffuseTextureMap( baseColor.GetTexture() );
}
bool TextureFile::SetViewportTexture() const
{
	if ( viewportTextureID == 0 ) {
		glGenTextures(1,&viewportTextureID);
		glBindTexture(GL_TEXTURE_2D,viewportTextureID);
		gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, &data[0].r );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
	}
	glBindTexture(GL_TEXTURE_2D,viewportTextureID);
	return true;
}
bool TextureChecker::SetViewportTexture() const
{
	if ( viewportTextureID == 0 ) {
		const int texSize = 1024;
		glGenTextures(1,&viewportTextureID);
		glBindTexture(GL_TEXTURE_2D,viewportTextureID);
		Color24 *tex = new Color24[texSize*texSize];
		for ( int y=0, i=0; y<texSize; ++y ) {
			float v = (y + 0.5f) / texSize;
			for ( int x=0; x<texSize; ++x, ++i ) {
				float u = (x + 0.5f) / texSize;
				Vec3f uvw( u, v, 0.5f );
				tex[i] = Color24( color[ ( (u <= 0.5f) ^ (v <= 0.5f) ) ].Eval(uvw) );
			}
		}
		gluBuild2DMipmaps( GL_TEXTURE_2D, 3, texSize, texSize, GL_RGB, GL_UNSIGNED_BYTE, &tex[0].r );
		delete [] tex;
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16.0f );
	}
	glBindTexture(GL_TEXTURE_2D,viewportTextureID);
	return true;
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

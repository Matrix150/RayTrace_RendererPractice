//-------------------------------------------------------------------------------
///
/// \file       xmlload.cpp 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    1.0
/// \date       September 19, 2025
///
/// \brief Example source for CS 6620 - University of Utah.
///
/// Copyright (c) 2019 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------

#include "renderer.h"
#include "xmlload.h"
#include "objects.h"

//-------------------------------------------------------------------------------

Sphere theSphere;

//-------------------------------------------------------------------------------

void LoadNode( Loader loader, Node &parent );

//-------------------------------------------------------------------------------

bool Renderer::LoadScene( char const *filename )
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError e = doc.LoadFile(filename);

	if ( e != tinyxml2::XML_SUCCESS ) {
		printf("ERROR: Failed to load the file \"%s\"\n", filename);
		return false;
	}

	tinyxml2::XMLElement *xml = doc.FirstChildElement("xml");
	if ( ! xml ) {
		printf("ERROR: No \"xml\" tag found.\n");
		return false;
	}

	tinyxml2::XMLElement *xscene = xml->FirstChildElement("scene");
	if ( ! xscene ) {
		printf("ERROR: No \"scene\" tag found.\n");
		return false;
	}

	tinyxml2::XMLElement *xcam = xml->FirstChildElement("camera");
	if ( ! xcam ) {
		printf("ERROR: No \"camera\" tag found.\n");
		return false;
	}

	scene.Load( Loader(xscene) );
	camera.Load( Loader(xcam) );
	renderImage.Init( camera.imgWidth, camera.imgHeight );

	sceneFile = filename;

	return true;
}

//-------------------------------------------------------------------------------

void Scene::Load( Loader const &sceneLoader )
{
	rootNode.Init();

	for ( Loader loader : sceneLoader ) {
		if ( loader == "object" ) LoadNode( loader, rootNode  );
	}
}

//-------------------------------------------------------------------------------

void Camera::Load( Loader const &loader )
{
	Init();
	loader.Child("position" ).ReadVec3f( pos       );
	loader.Child("target"   ).ReadVec3f( dir       );
	loader.Child("up"       ).ReadVec3f( up        );
	loader.Child("fov"      ).ReadFloat( fov       );
	loader.Child("width"    ).ReadInt  ( imgWidth  );
	loader.Child("height"   ).ReadInt  ( imgHeight );
	dir -= pos;
	dir.Normalize();
	Vec3f x = dir ^ up;
	up = (x ^ dir).GetNormalized();
}

//-------------------------------------------------------------------------------

void LoadNode( Loader loader, Node &parent )
{
	Node *node = new Node;
	parent.AppendChild(node);

	// name
	char const *name = loader.Attribute("name");
	node->SetName(name);

	// type
	Loader::String type = loader.Attribute("type");
	if ( type ) {
		if ( type == "sphere" ) node->SetNodeObj( &theSphere );
		else printf("ERROR: Unknown object type %s\n", static_cast<char const*>(type));
	}

	if ( node->GetNodeObj() ) node->GetNodeObj()->Load(loader);	// loads object-specific parameters (if any)
	node->Load( loader );	// loads the transformation

	// Load child nodes
	for ( Loader L : loader ) {
		if ( L == "object" ) LoadNode( L, *node );
	}
}

//-------------------------------------------------------------------------------

void Transformation::Load( Loader const &loader )
{
	for ( Loader const &L : loader ) {
		if ( L == "scale" ) {
			Vec3f s;
			L.ReadVec3f(s, Vec3f(1,1,1));
			Scale(s);
		} else if ( L == "rotate" ) {
			Vec3f s;
			L.ReadVec3f(s);
			s.Normalize();
			float a = 0.0f;
			L.ReadFloat(a,"angle");
			Rotate(s,a);
		} else if ( L == "translate" ) {
			Vec3f t;
			L.ReadVec3f(t);
			Translate(t);
		}
	}
}

//-------------------------------------------------------------------------------

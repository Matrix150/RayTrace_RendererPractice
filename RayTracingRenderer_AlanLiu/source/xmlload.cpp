//-------------------------------------------------------------------------------
///
/// \file       xmlload.cpp 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    4.0
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
#include "lights.h"
#include "materials.h"

//-------------------------------------------------------------------------------

Sphere theSphere;

//-------------------------------------------------------------------------------

void LoadNode     ( Loader loader, Node         &parent );
void LoadLight    ( Loader loader, LightList    &lights );
void LoadMaterial ( Loader loader, MaterialList &materials );
void SetNodeMaterials( Node *node, MaterialList &materials );

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
	rootNode .Init();
	lights   .DeleteAll();
	materials.DeleteAll();

	for ( Loader loader : sceneLoader ) {
		if      ( loader == "object"      ) LoadNode    ( loader, rootNode );
		else if ( loader == "light"       ) LoadLight   ( loader, lights );
		else if ( loader == "material"    ) LoadMaterial( loader, materials );
		else printf("WARNING: Unknown tag \"%s\"\n", static_cast<char const*>(loader.Tag()));
	}

	SetNodeMaterials( &rootNode, materials );
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

	// material
	char const *mtlName = loader.Attribute("material");
	if ( mtlName ) {
		node->SetMaterial( (Material*)mtlName );	// temporarily set the material pointer to a string of the material name
	}

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

void LoadLight( Loader loader, LightList &lights )
{
	Loader::String type = loader.Attribute("type");
	Light *light = nullptr;
	if      ( type == "ambient" ) light = new AmbientLight;
	else if ( type == "direct"  ) light = new DirectLight;
	else if ( type == "point"   ) light = new PointLight;
	else {
		printf("ERROR: Unknown light type %s\n", static_cast<char const*>(type));
		return;
	}

	light->SetName(loader.Attribute("name"));
	light->Load(loader);
	lights.push_back(light);
}

//-------------------------------------------------------------------------------

void AmbientLight::Load( Loader const &loader )
{
	loader.Child("intensity").ReadColor( intensity );
}

//-------------------------------------------------------------------------------

void DirectLight::Load( Loader const &loader )
{
	loader.Child("intensity").ReadColor( intensity );
	loader.Child("direction").ReadVec3f( direction );
	direction.Normalize();
}

//-------------------------------------------------------------------------------

void PointLight::Load( Loader const &loader )
{
	loader.Child("intensity").ReadColor( intensity );
	loader.Child("position" ).ReadVec3f( position  );
}

//-------------------------------------------------------------------------------

void LoadMaterial( Loader loader, MaterialList &materials )
{
	Material *mtl = nullptr;

	Loader::String type = loader.Attribute("type");
	if      ( type == "phong"      ) mtl = new MtlPhong;
	else if ( type == "blinn"      ) mtl = new MtlBlinn;
	else if ( type == "microfacet" ) mtl = new MtlMicrofacet;
	else {
		printf("ERROR: Unknown material type %s\n", static_cast<char const*>(type));
		return;
	}

	mtl->SetName( loader.Attribute("name") );
	mtl->Load( loader );
	materials.push_back(mtl);
}

//-------------------------------------------------------------------------------

void MtlBasePhongBlinn::Load( Loader const &loader )
{
	loader.Child("diffuse"   ).ReadColor( diffuse    );
	loader.Child("specular"  ).ReadColor( specular   );
	loader.Child("glossiness").ReadFloat( glossiness );
	loader.Child("reflection").ReadColor( reflection );
	loader.Child("refraction").ReadColor( refraction );
	loader.Child("refraction").ReadFloat( ior, "index" );
	loader.Child("absorption").ReadColor( absorption );
}

//-------------------------------------------------------------------------------

void MtlMicrofacet::Load( Loader const &loader )
{
	loader.Child("color"        ).ReadColor( baseColor     );
	loader.Child("roughness"    ).ReadFloat( roughness     );
	loader.Child("metallic"     ).ReadFloat( metallic      );
	loader.Child("transmittance").ReadColor( transmittance );
	loader.Child("ior"          ).ReadFloat( ior           );
	loader.Child("absorption"   ).ReadColor( absorption    );
}

//-------------------------------------------------------------------------------

void SetNodeMaterials( Node *node, MaterialList &materials )
{
	int n = node->GetNumChild();
	if ( node->GetMaterial() ) {
		const char *mtlName = (const char*) node->GetMaterial();
		Material *mtl = materials.Find( mtlName );	// mtl can be null
		node->SetMaterial(mtl);
	}
	for ( int i=0; i<n; i++ ) SetNodeMaterials( node->GetChild(i), materials );
}

//-------------------------------------------------------------------------------

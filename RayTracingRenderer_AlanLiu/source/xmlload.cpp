//-------------------------------------------------------------------------------
///
/// \file       xmlload.cpp 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    9.0
/// \date       October 2, 2025
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
#include "texture.h"

//-------------------------------------------------------------------------------

Sphere theSphere;
Plane  thePlane;

//-------------------------------------------------------------------------------

void LoadNode     ( Loader loader, Node         &parent,    ObjFileList &objList );
void LoadLight    ( Loader loader, LightList    &lights );
void LoadMaterial ( Loader loader, MaterialList &materials, TextureFileList &texFiles );
void SetNodeMaterials( Node *node, MaterialList &materials, TextureFileList &texFiles );

TextureFile* ReadTextureFile( TextureFileList &texFiles, char const *filename );
Material*    CreateMultiMtl ( TextureFileList &texFiles, TriObj const *tobj   );

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
	objList  .DeleteAll();
	lights   .DeleteAll();
	materials.DeleteAll();
	texFiles .DeleteAll();

	for ( Loader loader : sceneLoader ) {
		if      ( loader == "object"      ) LoadNode    ( loader, rootNode,  objList  );
		else if ( loader == "light"       ) LoadLight   ( loader, lights );
		else if ( loader == "material"    ) LoadMaterial( loader, materials, texFiles );
		else if ( loader == "background"  ) loader.ReadTexturedColor( background,  texFiles );
		else if ( loader == "environment" ) loader.ReadTexturedColor( environment, texFiles );
		else printf("WARNING: Unknown tag \"%s\"\n", static_cast<char const*>(loader.Tag()));
	}

	rootNode.ComputeChildBoundBox();

	SetNodeMaterials( &rootNode, materials, texFiles );
}

//-------------------------------------------------------------------------------

void Camera::Load( Loader const &loader )
{
	Init();
	loader.Child("position" ).ReadVec3f( pos       );
	loader.Child("target"   ).ReadVec3f( dir       );
	loader.Child("up"       ).ReadVec3f( up        );
	loader.Child("fov"      ).ReadFloat( fov       );
	loader.Child("focaldist").ReadFloat( focaldist );
	loader.Child("dof"      ).ReadFloat( dof       );
	loader.Child("width"    ).ReadInt  ( imgWidth  );
	loader.Child("height"   ).ReadInt  ( imgHeight );
	dir -= pos;
	dir.Normalize();
	Vec3f x = dir ^ up;
	up = (x ^ dir).GetNormalized();
}

//-------------------------------------------------------------------------------

void LoadNode( Loader loader, Node &parent, ObjFileList &objList )
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
		if      ( type == "sphere" ) node->SetNodeObj( &theSphere );
		else if ( type == "plane"  ) node->SetNodeObj( &thePlane );
		else if ( type == "obj"    ) {
			TriObj *tobj = (TriObj*) objList.Find(name);
			if ( tobj == nullptr ) {	// object is not on the list, so we should load it now
				tobj = new TriObj;
				if ( ! tobj->Load( name ) ) {
					printf("ERROR: Cannot load file \"%s.\"", name);
					delete tobj;
					tobj = nullptr;
				} else {
					tobj->SetName(name);
					objList.push_back(tobj);	// add to the list
				}
			}
			if ( mtlName==nullptr && tobj && tobj->NM()>0 ) node->SetMaterial( (Material*)tobj );	// temporarily set the material pointer to the object
			node->SetNodeObj( tobj );
		} else printf("ERROR: Unknown object type %s\n", static_cast<char const*>(type));
	}

	if ( node->GetNodeObj() ) node->GetNodeObj()->Load(loader);	// loads object-specific parameters (if any)
	node->Load( loader );	// loads the transformation

	// Load child nodes
	for ( Loader L : loader ) {
		if ( L == "object" ) LoadNode( L, *node, objList );
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

void LoadMaterial( Loader loader, MaterialList &materials, TextureFileList &texFiles )
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
	mtl->Load( loader, texFiles );
	materials.push_back(mtl);
}

//-------------------------------------------------------------------------------

void MtlBasePhongBlinn::Load( Loader const &loader, TextureFileList &tfl )
{
	loader.Child("diffuse"   ).ReadTexturedColor( diffuse,    tfl );
	loader.Child("specular"  ).ReadTexturedColor( specular,   tfl );
	loader.Child("glossiness").ReadTexturedFloat( glossiness, tfl );
	loader.Child("reflection").ReadTexturedColor( reflection, tfl );
	loader.Child("refraction").ReadTexturedColor( refraction, tfl );
	loader.Child("refraction").ReadFloat( ior, "index" );
	loader.Child("absorption").ReadColor( absorption );
}

//-------------------------------------------------------------------------------

void MtlMicrofacet::Load( Loader const &loader, TextureFileList &tfl )
{
	loader.Child("color"        ).ReadTexturedColor( baseColor,     tfl );
	loader.Child("roughness"    ).ReadTexturedFloat( roughness,     tfl );
	loader.Child("metallic"     ).ReadTexturedFloat( metallic,      tfl );
	loader.Child("transmittance").ReadTexturedColor( transmittance, tfl );
	loader.Child("ior"          ).ReadFloat( ior        );
	loader.Child("absorption"   ).ReadColor( absorption );
}

//-------------------------------------------------------------------------------

void SetNodeMaterials( Node *node, MaterialList &materials, TextureFileList &texFiles )
{
	int n = node->GetNumChild();
	if ( node->GetMaterial() ) {
		if ( node->GetNodeObj() == (Object*) node->GetMaterial() ) {
			// if the material pointer was set to the object, we must create the object's material.
			Material *mtl = materials.Find( node->GetName() );
			if ( !mtl ) {
				mtl = CreateMultiMtl( texFiles, (TriObj*) node->GetNodeObj() );
				mtl->SetName( node->GetName() );
				materials.push_back(mtl);
			}
			node->SetMaterial(mtl);
		} else {
			const char *mtlName = (const char*) node->GetMaterial();
			Material *mtl = materials.Find( mtlName );	// mtl can be null
			node->SetMaterial(mtl);
		}
	}
	for ( int i=0; i<n; i++ ) SetNodeMaterials( node->GetChild(i), materials, texFiles );
}

//-------------------------------------------------------------------------------

Material* CreateMultiMtl( TextureFileList &texFiles, TriObj const *tobj )
{
	// generate multi-material
	MultiMtl *mm = new MultiMtl;
	for ( unsigned int i=0; i<tobj->NM(); i++ ) {
		MtlBlinn *m = new MtlBlinn;
		TriMesh::Mtl const &mtl = tobj->M(i);
		m->SetDiffuse( Color(mtl.Kd) );
		m->SetSpecular( Color(mtl.Ks) );
		m->SetGlossiness( mtl.Ns );
		m->SetIOR( mtl.Ni );
		if ( mtl.map_Kd.data != nullptr ) m->SetDiffuseTexture( new TextureMap(ReadTextureFile(texFiles,mtl.map_Kd.data)) );
		if ( mtl.map_Ks.data != nullptr ) m->SetDiffuseTexture( new TextureMap(ReadTextureFile(texFiles,mtl.map_Ks.data)) );
		if ( mtl.illum > 2 && mtl.illum <= 7 ) {
			m->SetReflection( Color(mtl.Ks) );
			if ( mtl.map_Ks.data != nullptr ) m->SetReflectionTexture( new TextureMap(ReadTextureFile(texFiles,mtl.map_Ks.data)) );
			float gloss = std::acos(std::pow(2.0f,1.0f/mtl.Ns));
			if ( mtl.illum >= 6 ) {
				m->SetRefraction( 1 - Color(mtl.Tf) );
			}
		}
		mm->AppendMaterial(m);
	}
	return mm;
}

//-------------------------------------------------------------------------------

TextureMap* Loader::ReadTextureMap( TextureFileList &texFiles ) const
{
	Loader::String texName = Attribute("texture");
	if ( ! texName ) return nullptr;

	Texture *tex = nullptr;
	if ( texName == "checkerboard" ) {
		tex = new TextureChecker;
		tex->Load(*this,texFiles);	// loads the texture parameters
		tex->SetName(texName);
	} else {
		tex = ReadTextureFile( texFiles, texName );
	}
	if ( ! tex ) return nullptr;

	TextureMap *map = new TextureMap(tex);
	map->Load( *this );	// loads the transformations
	return map;
}

//-------------------------------------------------------------------------------

void TextureChecker::Load( Loader const &loader, TextureFileList &texFiles )
{
	loader.Child("color1").ReadTexturedColor( color[0], texFiles );
	loader.Child("color2").ReadTexturedColor( color[1], texFiles );
}

//-------------------------------------------------------------------------------

TextureFile* ReadTextureFile( TextureFileList &texFiles, char const *texName )
{
	TextureFile *tex = (TextureFile*) texFiles.Find( texName );
	if ( tex == nullptr ) {
		tex = new TextureFile;
		tex->SetName(texName);
		if ( ! tex->LoadFile() ) {
			printf("ERROR: cannot load file %s\n", texName);
			delete tex;
			tex = nullptr;
		} else {
			tex->SetName(texName);
			texFiles.push_back(tex);
		}
	}
	return tex;
}

//-------------------------------------------------------------------------------

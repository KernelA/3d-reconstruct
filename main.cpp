/**
 * Lum, Darrell and Zipkin, Greg
 * CSC 570 Final Project
 * Winter 2007, Z. Wood
 *
 * This program creates a cut path for a geometry image in real time.  The cut-path
 * edges are color-coded based on the current step in the algorithm.  The cut-path
 * is then highlighted on the original mesh when the algorithm is finished.

 * The second part of our program shows the steps for rendering a mesh from a
 * geometry image and a normal map.  We show the program stepping through the
 * geometry image pixels and draw the geometry at the same time.  Edges can be turned
 * on and off by the user.  Once this is done, the user can turn on culling and
 * move a culling box around to cull triangles and also see the results of the
 * quad tree in the geometry image.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <GL\glut.h>
#include <assert.h>
#include <map>
#include <time.h>
#include <windows.h>

using namespace std;
#define FLT_MIN 1.1754E-38F
#define FLT_MAX 1.1754E+38F

#pragma warning (disable:4018) // signed/unsigned mismatch
#pragma warning (disable:4996) // sprintf, sscanf, fopen_s depreciated
#pragma warning (disable:4267) // size_t to int, possible loss of data

/* Forward declarations */
void createInitialCutPart1();
void createInitialCutPart2();
void recreateMesh();
void reshape(int w, int h);

/** Constants */
// Debugging flag (print out variable values to the console)
const int __DEBUG = 0;
const float PI = 3.14159265358979323846; // PI
const float normScale = 0.01;     // Scaling factor for normals
const float MAX_SCALE = 3.0;      // Maximum scale factor
const float MIN_SCALE = 0.2;      // Minimum scale factor
const float MAX_TRANSLATE = 2.0;  // Maximum translation window
const float MIN_TRANSLATE = -2.0; // Minimum translation window

const int MATERIAL_GOLD = 1;      // Gold surface material
const int MATERIAL_TURQUOISE = 2; // Turqoise surface material
const int MATERIAL_EMERALD = 3;   // Emerald surface material

const int SHADE_WIREFRAME = 4;    // No shading (just display wireframe)
const int SHADE_FILL = 5;         // Fill in polygons
const int SHADE_FLAT = 6;         // Flat (constant) polygon shading
const int SHADE_SMOOTH = 7;       // Smooth (Gouraud) polygon shading

const int OBJECT_MESH = 8;        // User will manipulate the mesh
const int OBJECT_LIGHT = 9;       // User will manipulate the light

const int USER_TRANSLATE = 10;     // User will translate the figure
const int USER_SCALE = 11;        // User will scale the figure
const int USER_ROTATE = 12;       // User will rotate the figure
const int USER_NULL = 13;         // User doesn't wish to transform

/** Flags */
int user_material;  // User's choice of which surface material to use
int user_object;    // User's choice of which object to manipulate
int user_transform; // User's choice of transformation
int user_shade;     // User's choice of how to shade the mesh
bool removedEandT;  // Removed all edges and triangles (cutting part 1)
bool removedVandE;  // Removed all vertices and edges (cutting part 2)
bool finishedInitialCut; // Completed initial mesh cut
bool recreating;    // true if drawing triangles of reconstructed mesh
bool recreatedMesh; // true if mesh has already been reconstructed from GIM
bool stepping_through; // true to prevent multiple simultaneous simulations
bool cullingon;
bool drawReconEdges;

float eyez;

/** Other global variables */
GLfloat myMatrix[16];// Rotation matrix
int GW;             // Global width
int GH;             // Global height
float lightScale;   // Light scaling factor
float scaleFactor;  // Uniform scaling factor
float theta;        // Rotation angle
float xw;           // Mouse click x position in world coordinates
float yw;           // Mouse click y position in world coordinates
int xTri;           // Current figure x position in pixel coordinates
int yTri;           // Current figure y position in pixel coordinates
float xTrans;       // Translation of x movement in world coordinates
float yTrans;       // Translation of y movement in world coordinates
float cubex;
float cubey;
float cubez;
const float CULLING_STEP_SIZE = 0.05;

/*
 * Data structure for the image used for texture mapping.
 */
typedef struct Image {
	unsigned long sizeX;  // Width of image
	unsigned long sizeY;  // Height of image
	char *data;           // RGB values of image
} Image;

Image *TextureImage;    // Holds loaded textures
Image *TextureNormalImage;    // Holds loaded textures

/*
 * Data structure for RGB values.  0 <= r, g, b <= 1.
 */
typedef struct RGB {
	float r;  // Red value
	float g;  // Green value
	float b;  // Blue value
} RGB;

int GIMxInd; // X-coordinate for stepping through mesh reconstruction from GIM
int GIMyInd; // Y-coordinate for stepping through mesh reconstruction from GIM


RGB myimage[65][65]; // Holds a pixel-by-pixel representation of a GIM, loaded from a texture
RGB myimage2[65][65]; // For culling
RGB myimage3[65][65]; // For restoring before culling
RGB mynormalimage[65][65];
vector<int> pixelToTri[65][65];

struct GIMQuadTreeNode {
	RGB value;

	int size;
	int startPosX; // Upper-left X
	int startPosY; // Upper-left Y

	/* Quads */
	GIMQuadTreeNode * upperLeft;
	GIMQuadTreeNode * upperRight;
	GIMQuadTreeNode * lowerLeft;
	GIMQuadTreeNode * lowerRight;

	GIMQuadTreeNode(int in_size, int in_startPosX, int in_startPosY) {
		size = in_size;
		startPosX = in_startPosX;
		startPosY = in_startPosY;
	}
};

GIMQuadTreeNode * quadTree;
GIMQuadTreeNode * createQuadTree(int size, int startX, int startY, RGB grid[65][65]);
void cull();

RGB* pixel;  // Temporary holder for a bitmap pixel
int ImageLoad(char *filename, Image *image);
GLvoid LoadTexture(char* image_file, int tex_id);
int NormalImageLoad(char *filename, Image *image);
GLvoid LoadTextureNormal(char* image_file, int tex_id);
void init_tex();
void init();
void makeMyImage();
void makeMyNormalImage();

/**
 * Vector3 is a structure that stores 3D points.
 */
typedef struct Vector3
{
	float x;
	float y;
	float z;

	Vector3(float in_x, float in_y, float in_z) : x(in_x), y(in_y), z(in_z) {}
	Vector3() {}
} Vector3;

/**
 * Point3 is a structure that stores 3D points and normals to those points.
 */
typedef struct Point3
{
	float x;
	float y;
	float z;
	Vector3 normal;

	Point3(Vector3 v_in) : x(v_in.x), y(v_in.y), z(v_in.z)
	{
		normal.x = normal.y = normal.z = 0;
	}
	Point3() {}
} Point3;

/**
 * Tri is a structure that stores triangles whose shape is determined by
 * three vertices.
 */
typedef struct Tri {
	int v1;
	int v2;
	int v3;
	Vector3 normal; // Normal to the face of the triangle
	Vector3 color;
	bool drawn; // true if not culled

	Tri(int in_v1, int in_v2, int in_v3) : v1(in_v1), v2(in_v2), v3(in_v3),
		normal(0, 1, 0), drawn(true) {}
	Tri() : normal(0, 1, 0) {}
} Tri;

/* Mesh variables */
// STL vector to store all the triangles in the mesh
vector<Tri *> Triangles;
// STL vector to store all the vertices in the mesh
vector<Vector3 *> Vertices;
// STL vector to store all the points in the mesh (redundancy so that normals
// to vertices can be stored)
vector<Point3 *> VPoints;

/* Cut-path variables */
typedef struct Edge {
	int v1;
	int v2;
	Vector3 color;

	Edge(int in_v1, int in_v2) : v1(in_v1), v2(in_v2) {}
	Edge(int in_v1, int in_v2, Vector3 in_color) : v1(in_v1), v2(in_v2), color(in_color) {}
} Edge;

vector<Edge *> cutPathEdges;
vector<Point3 *> originalVPoints;
vector<Tri *> originalTriangles;
vector<Vector3 *> originalVertices;

vector<Tri *> newTriangles;
// STL vector to store all the vertices in the mesh
vector<Vector3 *> newVertices;
vector<Point3 *> newVPoints;

Vector3 vInitial;   // Initial vector from the origin to the user click
Vector3 vFinal;     // Final vector from the origin to where the user moved
Vector3 vCross;     // Cross product of vInitial and vFinal (in that order)

/** Globals for computing the center point and extent of the model */
Vector3 center;
float max_x, max_y, max_z, min_x, min_y, min_z;
float max_extent;

/** Globals for lighting */
GLfloat light_pos[4] = { 1.0, 1.0, 1.5, 1.0 }; // Light position

// White light color
GLfloat light_amb[4] = { 0.6, 0.6, 0.6, 1.0 };  // Light ambience component
GLfloat light_diff[4] = { 0.6, 0.6, 0.6, 1.0 }; // Light diffuse component
GLfloat light_spec[4] = { 0.8, 0.8, 0.8, 1.0 }; // Light specular component

int mat = 0;        // Color value from file

/**
 * materialStruct defines the lighting components for materials.
 */
typedef struct materialStruct
{
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess[1];
} materialStruct;

materialStruct gold = {
	{0.24725, 0.1995, 0.0745, 1.0},
	{0.75164, 0.60648, 0.22648, 1.0},
	{0.628281, 0.555802, 0.366065, 1.0},
	{0.4}
};

materialStruct turquoise = {
	{0.1, 0.18725, 0.1745, 1.0},
	{0.396, 0.74151, 0.69102, 1.0},
	{0.297254, 0.30829, 0.306678, 1.0},
	{0.1}
};

materialStruct emerald = {
	{0.0215, 0.1745, 0.0215, 1.0},
	{0.07568, 0.61424, 0.07568, 1.0},
	{0.633, 0.727811, 0.633, 1.0},
	{0.6}
};

/** Forward function declarations */
void crossProd(Vector3 a, Vector3 b, Vector3 &c);
void readLine(char* str);
void readStream(istream& is);
void drawTri(Tri * t);
void drawObjects();
void display();

/**
 * Set up a specific material.
 * materials:  the material to setup.
 */
void materials(materialStruct materials)
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, materials.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, materials.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, materials.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, materials.shininess);
}


void renderBitmapCharacter(float x, float y, float z, void *font, char *string)
{
	char *c;
	glRasterPos3f(x, y, z);
	for (c = string; *c != '\0'; c++)
	{
		glutBitmapCharacter(font, *c);
	}
}

/**
 * Convert an x pixel coordinate into an x image coordinate
 * xPixel:  an x pixel coordinate
 */
float p2i_x(int xPixel)
{
	return (2 / (float)GW)*xPixel + 1 / (float)GW - 1;
}

/**
 * Convert a y pixel coordinate into a y image coordinate
 * yPixel:  a y pixel coordinate
 */
float p2i_y(int yPixel)
{
	return (2 / (float)GH)*yPixel + 1 / (float)GH - 1;
}

/**
 * Convert an x image coordinate into an x world coordinate
 * xImage:  an x image coordinate
 */
float i2w_x(float xImage)
{
	return ((float)GW / (float)GH)*xImage;
}

/**
 * Convert a y image coordinate into a y world coordinate
 * yImage:  a y image coordinate
 */
float i2w_y(float yImage)
{
	return yImage;
}

/**
 * Convert an x pixel coordinate into an x world coordinate
 * xPixel:  an x pixel coordinate
 */
float p2w_x(int xPixel)
{
	return i2w_x(p2i_x(xPixel));
}

/**
 * Convert a y pixel coordinate into a y world coordinate
 * yPixel:  a y pixel coordinate
 */
float p2w_y(int yPixel)
{
	return i2w_y(p2i_y(yPixel));
}

/**
 * Computes the dot product of two 3D vectors.
 * v1:  first vector
 * v2:  second vector
 */
float dotProd(Vector3 v1, Vector3 v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

/**
 * Computes the cross product of two 3D vectors.
 * a:  first vector
 * b:  second vector
 * c:  resulting cross product of a and b
 * Postcondition:  c will have the cross product of a and b.
 */
void crossProd(Vector3 a, Vector3 b, Vector3 &c)
{
	c.x = a.y*b.z - a.z*b.y;
	c.y = a.z*b.x - a.x*b.z;
	c.z = a.x*b.y - a.y*b.x;
}

/**
 * Computes the angle between two vectors in radians.
 * v1:  first vector
 * v2:  second vector
 */
float getAngle(Vector3 v1, Vector3 v2)
{
	float len1 = sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
	float len2 = sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);

	return acos(dotProd(v1, v2) / (len1*len2));
}

/**
 * Converts an angle in radians to degrees.
 * radian:  the angle to convert
 */
float radToDeg(float radian)
{
	return radian * 180 / PI;
}

/**
 * Initialization calls for opengl for static light.  Lighting will still have
 * to be enabled in order for this to work.
 */
void init_lighting()
{
	// Turn on light0
	glEnable(GL_LIGHT0);

	// Set up the diffuse, ambient and specular components for the light
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diff);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_amb);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_spec);

	// Specify our lighting model as 1 normal per face
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
}

/**
 *  Set the light's position.
 */
void pos_light()
{
	glMatrixMode(GL_MODELVIEW);
	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
}

/**
 * Gets a vector given a pixel coordinate.  The resulting vector will be
 * based on a unit sphere.
 * v:  Vector from the origin to the given coordinates.
 * x:  x coordinate
 * y:  y coordinate
 * Precondition: x and y must be in pixel coordinates.
 * Postcondition:  v will store a vector from the origin to the given
 * set of coordinates (translated into world coordinates).
 */
void getVector(Vector3 &v, int x, int y)
{
	// Length of the vector
	float length = sqrt(p2w_x(x) * p2w_x(x) + p2w_y(y) * p2w_y(y));

	// IF the length reaches outside the unit sphere, project it onto the
	// surface of the unit sphere
	if (length > 1.)
	{
		v.x = p2w_x(x) / length;
		v.y = (p2w_y(GH) - 1 - p2w_y(y)) / length;
		v.z = 0.;
	}
	// ELSE the vector is inside the unit sphere
	else
	{
		v.x = p2w_x(x);
		v.y = p2w_y(GH) - 1 - p2w_y(y);
		v.z = sqrt(1 - p2w_x(x) * p2w_x(x) - p2w_y(y) * p2w_y(y));
	}
}

/**
 * Open a file for reading coordinates.
 * filename:  the file to open.
 */
void ReadFile(char * filename)
{
	printf("Reading coordinates from %s\n", filename);

	ifstream in_f(filename);
	if (!in_f)
	{
		printf("Could not open file %s\n", filename);
	}
	else
	{
		readStream(in_f);
	}
}

/**
 * Process the input stream from an input stream.
 * is:  the stream to read from.
 */
void readStream(istream& is)
{
	char str[256];
	for (; is;)
	{
		is >> ws;
		is.get(str, sizeof(str));
		if (!is) break;
		is.ignore(9999, '\n');
		readLine(str);
	}
}

/**
 * Calculate the normals for every vertex
 */
void calcAllVertexNormals()
{
	// For each face
	for (int i = 0; i < Triangles.size(); i++)
	{
		// Add that face's normal to each vertex's normal
		VPoints[Triangles[i]->v1 - 1]->normal.x += Triangles[i]->normal.x;
		VPoints[Triangles[i]->v1 - 1]->normal.y += Triangles[i]->normal.y;
		VPoints[Triangles[i]->v1 - 1]->normal.z += Triangles[i]->normal.z;

		VPoints[Triangles[i]->v2 - 1]->normal.x += Triangles[i]->normal.x;
		VPoints[Triangles[i]->v2 - 1]->normal.y += Triangles[i]->normal.y;
		VPoints[Triangles[i]->v2 - 1]->normal.z += Triangles[i]->normal.z;

		VPoints[Triangles[i]->v3 - 1]->normal.x += Triangles[i]->normal.x;
		VPoints[Triangles[i]->v3 - 1]->normal.y += Triangles[i]->normal.y;
		VPoints[Triangles[i]->v3 - 1]->normal.z += Triangles[i]->normal.z;
	}

	// For each vertex
	for (int j = 0; j < Vertices.size(); j++)
	{
		// Normalize that normal
		float length = sqrt(VPoints[j]->normal.x*VPoints[j]->normal.x
			+ VPoints[j]->normal.y*VPoints[j]->normal.y
			+ VPoints[j]->normal.z*VPoints[j]->normal.z);
		VPoints[j]->normal.x /= length;
		VPoints[j]->normal.y /= length;
		VPoints[j]->normal.z /= length;
	}
}

/**
 * Calculate the normals for every vertex (reconstructed mesh)
 */
void calcAllNewVertexNormals()
{
	// For each face
	for (int i = 0; i < newTriangles.size(); i++)
	{
		// Add that face's normal to each vertex's normal
		newVPoints[newTriangles[i]->v1 - 1]->normal.x += newTriangles[i]->normal.x;
		newVPoints[newTriangles[i]->v1 - 1]->normal.y += newTriangles[i]->normal.y;
		newVPoints[newTriangles[i]->v1 - 1]->normal.z += newTriangles[i]->normal.z;

		newVPoints[newTriangles[i]->v2 - 1]->normal.x += newTriangles[i]->normal.x;
		newVPoints[newTriangles[i]->v2 - 1]->normal.y += newTriangles[i]->normal.y;
		newVPoints[newTriangles[i]->v2 - 1]->normal.z += newTriangles[i]->normal.z;

		newVPoints[newTriangles[i]->v3 - 1]->normal.x += newTriangles[i]->normal.x;
		newVPoints[newTriangles[i]->v3 - 1]->normal.y += newTriangles[i]->normal.y;
		newVPoints[newTriangles[i]->v3 - 1]->normal.z += newTriangles[i]->normal.z;
	}

	// For each vertex
	for (int j = 0; j < newVertices.size(); j++)
	{
		// Normalize that normal
		float length = sqrt(newVPoints[j]->normal.x*newVPoints[j]->normal.x
			+ newVPoints[j]->normal.y*newVPoints[j]->normal.y
			+ newVPoints[j]->normal.z*newVPoints[j]->normal.z);
		newVPoints[j]->normal.x /= length;
		newVPoints[j]->normal.y /= length;
		newVPoints[j]->normal.z /= length;
	}
}

/**
 * Calculate the normal on the face given by t.
 * t:  Triangle for normal calculation.
 * Postcondition:  t->normal will contain the normal for t's face.
 */
void calcNormal(Tri* t)
{
	Vector3 w1;
	Vector3 w2;

	w1.x = Vertices[t->v3 - 1]->x - Vertices[t->v2 - 1]->x;
	w1.y = Vertices[t->v3 - 1]->y - Vertices[t->v2 - 1]->y;
	w1.z = Vertices[t->v3 - 1]->z - Vertices[t->v2 - 1]->z;

	w2.x = Vertices[t->v1 - 1]->x - Vertices[t->v2 - 1]->x;
	w2.y = Vertices[t->v1 - 1]->y - Vertices[t->v2 - 1]->y;
	w2.z = Vertices[t->v1 - 1]->z - Vertices[t->v2 - 1]->z;
	crossProd(w1, w2, t->normal);
}

/**
 * Process each line of input and save vertices and faces appropriately.
 * str:  Input line to process.
 */
void readLine(char * str)
{
	int indx = 0, vi;
	float x, y, z;
	float r, g, b;
	int mat;

	if (str[0] == '#')
	{
		return;
	}

	// Read a vertex or face
	if (str[0] == 'V' && !strncmp(str, "Vertex ", 7))
	{
		Vector3* v;
		Point3* p;
		assert(sscanf(str, "Vertex %d %g %g %g", &vi, &x, &y, &z) == 4);
		v = new Vector3(x, y, z);
		p = new Point3(*v);

		// Store the vertex
		Vertices.push_back(v);
		VPoints.push_back(p);
		// House keeping to display in center of the scene
		center.x += v->x;
		center.y += v->y;
		center.z += v->z;

		if (v->x > max_x) max_x = v->x; if (v->x < min_x) min_x = v->x;
		if (v->y > max_y) max_y = v->y; if (v->y < min_y) min_y = v->y;
		if (v->z > max_z) max_z = v->z; if (v->z < min_z) min_z = v->z;
	}
	else if (str[0] == 'F' && !strncmp(str, "Face ", 5))
	{
		Tri* t;
		t = new Tri();
		char * s = str + 4;
		int fi = -1;
		for (int t_i = 0; ; t_i++)
		{
			while (*s && isspace(*s))
				s++;
			// If we reach the end of the line break out of the loop
			if (!*s)
				break;

			// Save the position of the current character
			char * beg = s;

			// Advance to next space
			while (*s && isdigit(*s))
				s++;

			// Covert the character to an integer
			int j = atoi(beg);

			// The first number we encounter will be the face index, don't
			//store it
			if (fi < 0) { fi = j; continue; }

			// Otherwise process the digit we've grabbed in j as a vertex index
			// The first number will be the face id the following are vertex ids
			if (t_i == 1) t->v1 = j;
			else if (t_i == 2) t->v2 = j;
			else if (t_i == 3) t->v3 = j;

			// If there is more data to process break out
			if (*s == '{') break;
		}
		// Possibly process colors if the mesh has colors
		if (*s && *s == '{')
		{
			char *s1 = s + 1;
			cout << "trying to parse color " << !strncmp(s1, "rgb", 3) << endl;

			// If we're reading off a color
			if (!strncmp(s1, "rgb=", 4))
			{
				// Grab the values of the string
				assert(sscanf(s1, "rgb=(%g %g %g) matid=%d", &r, &g, &b, &mat) == 4);
				t->color.x = r; t->color.x = g; t->color.x = b;
				cout << "set color to: " << r << " " << g << " " << b << endl;
			}
		}
		// Calculate the normal for the triangle
		calcNormal(t);

		// Store the triangle read in
		Triangles.push_back(t);
	}
}

/**
 * Drawing routine to draw triangles as wireframe.
 * t:  Triangle to draw.
 */
void drawTria(Tri* t)
{
	float length = sqrt(t->normal.x*t->normal.x
		+ t->normal.y*t->normal.y + t->normal.z*t->normal.z);
	glShadeModel(GL_SMOOTH);
	/*if(user_shade == SHADE_FILL)
		glColor3f(0.0, 0.0, 0.5);
	if(user_shade == SHADE_FLAT)
		glNormal3f(t->normal.x/length*lightScale/max_extent,
			t->normal.y/length*lightScale/max_extent,
			t->normal.z/length*lightScale/max_extent);*/

			//if(user_shade == SHADE_WIREFRAME)
			//    glBegin(GL_LINE_LOOP);
			//else
	glBegin(GL_POLYGON);

	// Note that the vertices are indexed starting at 0, but the triangles
	// index them starting from 1, so we must offset by -1!!!
	//if(user_shade == SHADE_SMOOTH)
	if (!recreating)
	{

		glNormal3f(VPoints[t->v1 - 1]->normal.x*lightScale / max_extent,
			VPoints[t->v1 - 1]->normal.y*lightScale / max_extent,
			VPoints[t->v1 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(Vertices[t->v1 - 1]->x,
			Vertices[t->v1 - 1]->y,
			Vertices[t->v1 - 1]->z);

		//if(user_shade == SHADE_SMOOTH)
		glNormal3f(VPoints[t->v2 - 1]->normal.x*lightScale / max_extent,
			VPoints[t->v2 - 1]->normal.y*lightScale / max_extent,
			VPoints[t->v2 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(Vertices[t->v2 - 1]->x,
			Vertices[t->v2 - 1]->y,
			Vertices[t->v2 - 1]->z);

		//if(user_shade == SHADE_SMOOTH)
		glNormal3f(VPoints[t->v3 - 1]->normal.x*lightScale / max_extent,
			VPoints[t->v3 - 1]->normal.y*lightScale / max_extent,
			VPoints[t->v3 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(Vertices[t->v3 - 1]->x,
			Vertices[t->v3 - 1]->y,
			Vertices[t->v3 - 1]->z);
		glEnd();
	}
	else
	{
		glNormal3f(newVPoints[t->v1 - 1]->normal.x*lightScale / max_extent,
			newVPoints[t->v1 - 1]->normal.y*lightScale / max_extent,
			newVPoints[t->v1 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(newVertices[t->v1 - 1]->x,
			newVertices[t->v1 - 1]->y,
			newVertices[t->v1 - 1]->z);

		//if(user_shade == SHADE_SMOOTH)
		glNormal3f(newVPoints[t->v2 - 1]->normal.x*lightScale / max_extent,
			newVPoints[t->v2 - 1]->normal.y*lightScale / max_extent,
			newVPoints[t->v2 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(newVertices[t->v2 - 1]->x,
			newVertices[t->v2 - 1]->y,
			newVertices[t->v2 - 1]->z);

		//if(user_shade == SHADE_SMOOTH)
		glNormal3f(newVPoints[t->v3 - 1]->normal.x*lightScale / max_extent,
			newVPoints[t->v3 - 1]->normal.y*lightScale / max_extent,
			newVPoints[t->v3 - 1]->normal.z*lightScale / max_extent);
		glVertex3f(newVertices[t->v3 - 1]->x,
			newVertices[t->v3 - 1]->y,
			newVertices[t->v3 - 1]->z);
		glEnd();
	}
}

void drawGIM()
{
	glDisable(GL_LIGHTING);
	glPointSize(1.0);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, GW, 0.0, GH);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glBegin(GL_POINTS);
	if (!recreatedMesh)
	{
		for (int i = 0; i < 65; i++)
		{
			for (int j = 0; j < 65; j++)
			{
				glColor3f(myimage[i][j].r, myimage[i][j].g, myimage[i][j].b);
				glVertex2f(i + 735 - 60, j + 95);
			}
		}
	}
	else
	{
		for (int i = 0; i < 65; i++)
		{
			for (int j = 0; j < 65; j++)
			{
				glColor3f(myimage2[i][j].r, myimage2[i][j].g, myimage2[i][j].b);
				glVertex2f(i + 735 - 60, j + 95);
			}
		}
	}
	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);

	reshape(GW, GH);
}

void drawNormalGIM()
{
	glDisable(GL_LIGHTING);
	glPointSize(1.0);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, GW, 0.0, GH);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glBegin(GL_POINTS);
	for (int i = 0; i < 65; i++)
	{
		for (int j = 0; j < 65; j++)
		{
			glColor3f(mynormalimage[i][j].r, mynormalimage[i][j].g, mynormalimage[i][j].b);
			glVertex2f(i + 735 - 60, j + 6);
		}
	}
	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);

	reshape(GW, GH);
}

void drawEdge(Edge * e)
{
	glBegin(GL_LINES);
	glColor3f(e->color.x, e->color.y, e->color.z);
	glVertex3f(Vertices[e->v1 - 1]->x, Vertices[e->v1 - 1]->y, Vertices[e->v1 - 1]->z);
	glVertex3f(Vertices[e->v2 - 1]->x, Vertices[e->v2 - 1]->y, Vertices[e->v2 - 1]->z);
	glEnd();
}

/**
 * Draw a sphere.
 */
void drawSphere()
{
	glColor3f(1.0, 0.0, 0.0);
	glutWireSphere(0.35, 10, 10);
}

/**
 * Debugging routine which just draws the vertices of the mesh.
 */
void DrawAllVerts()
{
	glColor3f(1.0, 0.0, 1.0);
	glBegin(GL_POINTS);

	for (int k = 0; k < Vertices.size(); k++)
	{
		glVertex3f(Vertices[k]->x, Vertices[k]->y, Vertices[k]->z);
	}

	glEnd();
}

void drawHUD()
{
	glDisable(GL_LIGHTING);

	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, 1.0, 0.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//text
	glColor3f(1.0, 1.0, 1.0);

	if (!removedEandT && !removedVandE)
	{
		static char str1[50] = { 0 };
		sprintf(str1, "Geometry Images Demo - Cutting Algorithm Part 1");
		renderBitmapCharacter(0.24, 0.97, 1.0, GLUT_BITMAP_HELVETICA_18, str1);
	}
	else if (removedEandT && !removedVandE)
	{
		static char str1[50] = { 0 };
		sprintf(str1, "Geometry Images Demo - Cutting Algorithm Part 2");
		renderBitmapCharacter(0.24, 0.97, 1.0, GLUT_BITMAP_HELVETICA_18, str1);
	}
	else
	{
		if (!recreating)
		{
			static char str1[50] = { 0 };
			sprintf(str1, "Geometry Images Demo - Finished Cut-Path");
			renderBitmapCharacter(0.26, 0.97, 1.0, GLUT_BITMAP_HELVETICA_18, str1);
		}
		else
		{
			if (!cullingon)
			{
				static char str1[50] = { 0 };
				sprintf(str1, "Geometry Images Demo - Mesh Reconstruction");
				renderBitmapCharacter(0.25, 0.97, 1.0, GLUT_BITMAP_HELVETICA_18, str1);
			}
			else
			{
				static char str1[50] = { 0 };
				sprintf(str1, "Geometry Images Demo - Culling");
				renderBitmapCharacter(0.33, 0.97, 1.0, GLUT_BITMAP_HELVETICA_18, str1);
			}
		}
	}

	if (!removedEandT && !removedVandE)
	{
		static char str2[50] = { 0 };
		sprintf(str2, "Color Key for cutting algorithm part 1:");
		renderBitmapCharacter(0.01, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str2);
	}
	else if (removedEandT && !removedVandE)
	{
		static char str6[50] = { 0 };
		sprintf(str6, "Color Key for cutting algorithm part 2:");
		renderBitmapCharacter(0.01, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str6);
	}
	else
	{
		if (!recreating)
		{
			static char str2[50] = { 0 };
			sprintf(str2, "Color Key for finished cut:");
			renderBitmapCharacter(0.01, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str2);
		}
		else
		{
			static char str2[50] = { 0 };
			sprintf(str2, "Color Key for reconstruction:");
			renderBitmapCharacter(0.01, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str2);

			static char str19[50] = { 0 };
			sprintf(str19, "Geometry Image");
			renderBitmapCharacter(0.81, 0.21, 1.0, GLUT_BITMAP_9_BY_15, str19);

			static char str20[50] = { 0 };
			sprintf(str20, "Normal Map");
			renderBitmapCharacter(0.83, 0.098, 1.0, GLUT_BITMAP_9_BY_15, str20);
		}
	}

	if (!(removedEandT && !removedVandE))
	{
		static char str3[50] = { 0 };
		glColor3f(0.8, 0.6, 0.0);
		sprintf(str3, "Mesh faces");
		renderBitmapCharacter(0.04, 0.89, 1.0, GLUT_BITMAP_9_BY_15, str3);
	}

	if (recreating && drawReconEdges)
	{
		static char str18[50] = { 0 };
		glColor3f(1.0, 0.0, 0.0);
		sprintf(str18, "Mesh edges");
		renderBitmapCharacter(0.04, 0.87, 1.0, GLUT_BITMAP_9_BY_15, str18);
	}
	if (cullingon)
	{
		static char str18[50] = { 0 };
		glColor3f(0.0, 1.0, 1.0);
		sprintf(str18, "Culling box");
		renderBitmapCharacter(0.04, 0.85, 1.0, GLUT_BITMAP_9_BY_15, str18);
	}

	if (!removedEandT && !removedVandE)
	{
		static char str4[50] = { 0 };
		glColor3f(1.0, 0.0, 0.0);
		sprintf(str4, "Mesh edges");
		renderBitmapCharacter(0.04, 0.87, 1.0, GLUT_BITMAP_9_BY_15, str4);

		static char str5[50] = { 0 };
		glColor3f(0.0, 1.0, 0.0);
		sprintf(str5, "Possible cut-path edges to be removed");
		renderBitmapCharacter(0.04, 0.85, 1.0, GLUT_BITMAP_9_BY_15, str5);
	}
	else if (removedEandT && !removedVandE)
	{
		static char str4[50] = { 0 };
		glColor3f(0.0, 1.0, 0.0);
		sprintf(str4, "Cut-path edges");
		renderBitmapCharacter(0.04, 0.89, 1.0, GLUT_BITMAP_9_BY_15, str4);

		static char str5[50] = { 0 };
		glColor3f(1.0, 0.6, 0.6);
		sprintf(str5, "Possible cut-path edges to be removed");
		renderBitmapCharacter(0.04, 0.87, 1.0, GLUT_BITMAP_9_BY_15, str5);
	}
	else
	{
		if (!recreating)
		{
			static char str4[50] = { 0 };
			glColor3f(0.0, 1.0, 0.0);
			sprintf(str4, "Cut-path edges");
			renderBitmapCharacter(0.04, 0.87, 1.0, GLUT_BITMAP_9_BY_15, str4);
		}
	}

	glColor3f(1.0, 1.0, 1.0);

	if (removedEandT && removedVandE)
	{
		if (!recreating)
		{
			static char str7[50] = { 0 };
			sprintf(str7, "Number of edges in cut-path: %d", cutPathEdges.size());
			renderBitmapCharacter(0.6, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str7);
		}
		else
		{
			if (cullingon == false)
			{
				static char str7[50] = { 0 };
				sprintf(str7, "Number of triangles: %d", newTriangles.size());
				renderBitmapCharacter(0.7, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str7);
			}
			else
			{
				int count = 0;

				static char str7[50] = { 0 };
				for (int i = 0; i < newTriangles.size(); i++)
				{
					if (newTriangles[i]->drawn)
					{
						count++;
					}
				}
				sprintf(str7, "Number of triangles drawn: %d", count);
				renderBitmapCharacter(0.63, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str7);
			}
		}
	}
	else
	{
		static char str7[50] = { 0 };
		sprintf(str7, "Number of triangles: %d", Triangles.size());
		renderBitmapCharacter(0.7, 0.92, 1.0, GLUT_BITMAP_9_BY_15, str7);

		static char str8[50] = { 0 };
		sprintf(str8, "Number of edges: %d", cutPathEdges.size());
		renderBitmapCharacter(0.7, 0.90, 1.0, GLUT_BITMAP_9_BY_15, str8);
	}

	static char str9[50] = { 0 };
	sprintf(str9, "Controls:");
	renderBitmapCharacter(0.01, 0.19, 1.0, GLUT_BITMAP_9_BY_15, str9);

	static char str10[50] = { 0 };
	sprintf(str10, "A: Watch cutting algorithm part 1");
	renderBitmapCharacter(0.04, 0.16, 1.0, GLUT_BITMAP_9_BY_15, str10);

	static char str11[50] = { 0 };
	sprintf(str11, "S: Watch cutting algorithm part 2");
	renderBitmapCharacter(0.04, 0.14, 1.0, GLUT_BITMAP_9_BY_15, str11);

	static char str13[50] = { 0 };
	sprintf(str13, "D: Watch reconstruction algorithm");
	renderBitmapCharacter(0.04, 0.12, 1.0, GLUT_BITMAP_9_BY_15, str13);

	static char str14[50] = { 0 };
	sprintf(str14, "F: Turn on culling");
	renderBitmapCharacter(0.04, 0.10, 1.0, GLUT_BITMAP_9_BY_15, str14);

	static char str15[50] = { 0 };
	sprintf(str15, "NUMPAD: Move culling box");
	renderBitmapCharacter(0.04, 0.08, 1.0, GLUT_BITMAP_9_BY_15, str15);

	static char str16[50] = { 0 };
	sprintf(str16, "E: Show reconstructed mesh edges");
	renderBitmapCharacter(0.04, 0.06, 1.0, GLUT_BITMAP_9_BY_15, str16);

	static char str17[50] = { 0 };
	sprintf(str17, "< >: Camera Movement");
	renderBitmapCharacter(0.04, 0.04, 1.0, GLUT_BITMAP_9_BY_15, str17);

	static char str12[50] = { 0 };
	sprintf(str12, "SPACEBAR: Step through current algorithm manually");
	renderBitmapCharacter(0.04, 0.02, 1.0, GLUT_BITMAP_9_BY_15, str12);

	glPopMatrix();

	//go back to original viewing
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(60, (float)GW/(float)GH, 1.0, 300.0);
	gluPerspective(90.0, 1.0, 0.05, 15.0);
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, GW, GH);

	glEnable(GL_LIGHTING);
}

/**
 * This function allows image scaling upon window reshape.
 * w:  width of the new window size
 * h:  height of the new window size
 */
void reshape(int w, int h)
{
	GW = w;
	GH = h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(90.0, 1.0, 0.05, 15.0);

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);
}

void draw_edge(Vector3 start, Vector3 end)
{
	glBegin(GL_LINES);
	glVertex3f(start.x, start.y, start.z);
	glVertex3f(end.x, end.y, end.z);
	glEnd();
}

void draw_cube()
{
	Vector3 v1 = Vector3(cubex + 0, cubey + 0, cubez + 0);
	Vector3 v2 = Vector3(cubex + 0, cubey + 0, cubez + 1);
	Vector3 v3 = Vector3(cubex + 0, cubey + 1, cubez + 0);
	Vector3 v4 = Vector3(cubex + 0, cubey + 1, cubez + 1);
	Vector3 v5 = Vector3(cubex + 1, cubey + 0, cubez + 0);
	Vector3 v6 = Vector3(cubex + 1, cubey + 0, cubez + 1);
	Vector3 v7 = Vector3(cubex + 1, cubey + 1, cubez + 0);
	Vector3 v8 = Vector3(cubex + 1, cubey + 1, cubez + 1);
	draw_edge(v3, v4);
	draw_edge(v3, v7);
	draw_edge(v7, v8);
	draw_edge(v4, v8);
	draw_edge(v1, v5);
	draw_edge(v1, v2);
	draw_edge(v5, v6);
	draw_edge(v2, v6);
	draw_edge(v2, v4);
	draw_edge(v6, v8);
	draw_edge(v5, v7);
	draw_edge(v1, v3);
}

void drawReconstructedMeshEdges()
{
	glLineWidth(2.0);

	for (int i = 0; i < newTriangles.size(); i++)
	{
		draw_edge(*newVertices[newTriangles[i]->v1 - 1], *newVertices[newTriangles[i]->v2 - 1]);
		draw_edge(*newVertices[newTriangles[i]->v1 - 1], *newVertices[newTriangles[i]->v3 - 1]);
		draw_edge(*newVertices[newTriangles[i]->v2 - 1], *newVertices[newTriangles[i]->v3 - 1]);
	}

	glLineWidth(1.0);
}

/**
 * Draw the triangle mesh and sphere.
 */
void drawObjects()
{
	// Transforms for the mesh
	glPushMatrix();

	// IF rotating, update the rotation matrix
	if (user_transform == USER_ROTATE)
	{
		glPushMatrix();
		glLoadIdentity();
		glRotatef(theta, vCross.x, vCross.y, vCross.z);
		glGetFloatv(GL_MODELVIEW_MATRIX, myMatrix);
		glPopMatrix();
	}

	// Order of operations:  scale, rotate, then translate
	glTranslatef(xTrans, yTrans, 0);
	glMultMatrixf(myMatrix);
	if (!recreating)
	{
		glScalef(scaleFactor, scaleFactor, scaleFactor);
	}
	else
	{

	}

	// Scale object to window
	glScalef(1.0 / (float)max_extent, 1.0 / (float)max_extent, 1.0 / (float)max_extent);

	// Translate the object to the orgin
	glTranslatef(-(center.x), -(center.y), -(center.z));

	materials(gold);

	if (removedVandE)
	{
		if (!recreating)
		{
			// Draw the mesh with cut path
			for (int j = 0; j < originalTriangles.size(); j++)
			{
				drawTria(originalTriangles[j]);
			}
		}
		else
		{
			//printf("%d ", newTriangles.size());
			for (int j = 0; j < newTriangles.size(); j++)
			{
				if (newTriangles[j]->drawn)
				{
					drawTria(newTriangles[j]);
				}
			}
			if (drawReconEdges)
			{
				glDisable(GL_LIGHTING);
				glColor3f(1, 0, 0);
				drawReconstructedMeshEdges();
				glEnable(GL_LIGHTING);
			}
			if (cullingon)
			{
				glDisable(GL_LIGHTING);
				glColor3f(0, 1, 1);
				draw_cube();
				glEnable(GL_LIGHTING);
			}
		}
	}

	// Draw the wireframe mesh
	for (int j = 0; j < Triangles.size(); j++)
	{
		drawTria(Triangles[j]);
	}

	glDisable(GL_LIGHTING);
	glLineWidth(4.0);

	if (!recreating)
	{
		for (int k = 0; k < cutPathEdges.size(); k++)
		{
			drawEdge(cutPathEdges[k]);
		}
	}
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);

	glPopMatrix();

	// Transforms for the sphere
	glPushMatrix();

	// Draw the glut sphere behind the mesh
	glTranslatef(1.25, 0.0, -2.0);
	//drawSphere();
	if (removedVandE && recreating)
	{
		drawGIM();
		drawNormalGIM();
	}

	glPopMatrix();

	drawHUD();
}

/**
 * The display call back.  All drawing should be done in this function.
 */
void display()
{
	float numV = Vertices.size();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	// Set up the camera
	gluLookAt(0.0, 0.0, eyez,  // Eye position
		0.0, 0.0, 0.0,  // Point we are looking at
		0.0, 1.0, 0.0); // Up vector

// Position the light in the scene
	pos_light();
	drawObjects();
	glPopMatrix();

	glutSwapBuffers();

	glutPostRedisplay();
}

void stepThroughFirstPart(int x)
{
	createInitialCutPart1();

	if (!removedEandT)
	{
		glutTimerFunc(0, stepThroughFirstPart, 0);
	}
	else
	{
		stepping_through = false;
	}
}

void stepThroughSecondPart(int x)
{
	createInitialCutPart2();

	if (!removedVandE)
	{
		glutTimerFunc(0, stepThroughSecondPart, 0);
	}
	else
	{
		stepping_through = false;
	}
}

void stepThroughReconstruction(int x)
{
	recreateMesh();

	if (!recreatedMesh)
	{
		glutTimerFunc(0, stepThroughReconstruction, 0);
	}
	else
	{
		stepping_through = false;
	}
}

/**
 * Print mouse and keyboard functions to the console.
 */
void printHelp()
{
	printf("Look at the HUD!\n");
}

/**
 * This function provides the keyboard callback.
 * key:  the key that was pressed
 * x:  ?
 * y:  ?
 */
void keyboard(unsigned char key, int x, int y) {
	int i;
	switch (key)
	{
	case ',': case '<':
		eyez += 0.1;
		break;
	case '.': case '>':
		eyez -= 0.1;
		break;
		// Reset the object back to its original position
	case 'r': case 'R':
		scaleFactor = 1;
		xTrans = yTrans = 0;
		theta = 0;

		for (i = 0; i < 16; i++)
		{
			myMatrix[i] = 0.0;
		}

		myMatrix[0] = myMatrix[5] = myMatrix[10] = myMatrix[15] = 1.0;
		break;
		// Draw edges on reconstructed mesh
	case 'e': case 'E':
		if (recreating)
		{
			drawReconEdges = !drawReconEdges;
		}
		break;
		// Help
	case 'h': case 'H':
		printHelp();
		break;
	case 32:
		if (!removedEandT)
		{
			createInitialCutPart1();
		}
		else if (!removedVandE)
		{
			createInitialCutPart2();
		}
		else if (!recreatedMesh)
		{
			recreating = true;
			recreateMesh();
		}
		else
		{

		}
		break;
	case 'a': case 'A':
		if (!removedEandT && stepping_through == false)
		{
			stepping_through = true;
			glutTimerFunc(0, stepThroughFirstPart, 0);
			//keyboard(32, x, y);
		}
		break;
	case 's': case 'S':
		if (!removedVandE && stepping_through == false)
		{
			stepping_through = true;
			glutTimerFunc(0, stepThroughSecondPart, 0);
			//keyboard(32, x, y);
		}
		break;
	case 'd': case 'D':
		while (!recreatedMesh && stepping_through == false)
		{
			stepping_through = true;
			recreating = true;
			glutTimerFunc(0, stepThroughReconstruction, 0);
		}
		break;
	case 'f': case 'F':
		if (recreatedMesh)
		{
			cullingon = !cullingon;
			if (cullingon)
			{
				//cull();
			}
		}
		break;
		// Move up
	case '8':
		cubey += CULLING_STEP_SIZE; cull();
		break;
		// Move right
	case '6':
		cubex += CULLING_STEP_SIZE; cull();
		break;
		// Move left
	case '4':
		cubex -= CULLING_STEP_SIZE; cull();
		break;
		// Move down
	case '2':
		cubey -= CULLING_STEP_SIZE; cull();
		break;
		// Move towards camera
	case '3':
		cubez += CULLING_STEP_SIZE; cull();
		break;
		// Move away from camera
	case '9':
		cubez -= CULLING_STEP_SIZE; cull();
		break;
		// Quit
	case 'q': case 'Q':
		exit(EXIT_SUCCESS);
		break;
	}
	//glutPostRedisplay();
}

/**
 * Calculate the translation given a coordinate of mouse movement.
 * int x:  the x position of the mouse
 * int y:  the y position of the mouse
 * Postcondition:  the translation parameters will be updated depending on how
 * the mouse moved.
 */
void mouseTranslate(int x, int y)
{
	float xTemp = 2 * (p2w_x(x) - p2w_x(xTri));
	float yTemp = 2 * (p2w_y(GH) - 1 - p2w_y(y) - (p2w_y(GH) - 1 - p2w_y(yTri)));

	if (user_object == OBJECT_MESH)
	{
		if (xTrans + xTemp < MAX_TRANSLATE && xTrans + xTemp > MIN_TRANSLATE)
		{
			xTrans += xTemp;
		}
		if (yTrans + yTemp < MAX_TRANSLATE && yTrans + yTemp > MIN_TRANSLATE)
		{
			yTrans += yTemp;
		}
	}
	else // user_object == OBJECT_LIGHT
	{
		light_pos[0] += xTemp;
		light_pos[1] += yTemp;
	}
	xTri = x;
	yTri = y;
}

/**
 * Calculate the scaling factor given a coordinate of mouse movement.
 * int x:  the x position of the mouse
 * int y:  the y position of the mouse
 * Postcondition:  scaleFactor will be updated depending on how the mouse
 * moved.
 */
void mouseScale(int x, int y)
{
	// Distances from the cube's center to each 'point'
	float initialDistance, finalDistance;

	// The initial distance is the distance from the center of the cube to
	// the mouse click
	initialDistance = sqrt((xw - xTrans)*(xw - xTrans)
		+ (yw - yTrans)*(yw - yTrans));
	// The final distance is the distance from the center of the cube to
	// the mouse drag
	finalDistance = sqrt((p2w_x(x) - xTrans)*(p2w_x(x) - xTrans) +
		((p2w_y(GH) - 1 - p2w_y(y)) - yTrans)*((p2w_y(GH) - 1 - p2w_y(y))
			- yTrans));

	// IF the user is shrinking the cube
	if (initialDistance > finalDistance
		&& scaleFactor - (initialDistance - finalDistance) >= MIN_SCALE)
	{
		scaleFactor -= initialDistance - finalDistance;
	}
	// ELSE IF the user is enlarging the cube
	else if (finalDistance > initialDistance
		&& scaleFactor + (finalDistance - initialDistance) <= MAX_SCALE)
	{
		scaleFactor += finalDistance - initialDistance;
	}
	xw = p2w_x(x);
	yw = p2w_y(GH) - 1 - p2w_y(y);
}

/**
 * Calculate the axis and degree of rotation given a coordinate of mouse
 * movement.
 * int x:  the x position of the mouse
 * int y:  the y position of the mouse
 * Postcondition:  vFinal, vCross, and theta will be updated depending on how
 * the mouse moved.
 */
void mouseRotate(int x, int y)
{
	getVector(vFinal, x, y);
	crossProd(vInitial, vFinal, vCross);
	theta = radToDeg(getAngle(vInitial, vFinal));
	if (__DEBUG == 1)
	{
		printf("theta = %f\n", theta);
	}
}

/**
 * This function provides the mouse callback.
 * button:  the mouse button that triggered the callback (l or r)
 * state:  the state of the button (up or down)
 * x:  the x position of the mouse
 * y:  the y position of the mouse
 */
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON) {
		// IF the left button is clicked
		if (state == GLUT_DOWN) {
			if (__DEBUG == 1)
			{
				printf("mouse clicked at %d %d\n", x, y);
			}

			if (user_transform == USER_TRANSLATE)
			{
				xTri = x;
				yTri = y;
			}
			else if (user_transform == USER_SCALE)
			{
				if (user_object == OBJECT_MESH)
				{
					xw = p2w_x(x);
					yw = p2w_y(GH) - 1 - p2w_y(y);
				}

			}
			else if (user_transform == USER_ROTATE)
			{
				if (user_object == OBJECT_MESH)
				{
					getVector(vInitial, x, y);
					vFinal = vInitial;
				}
			}
		}
	}
}

/**
 * This function provides the mouse movement callback.
 * int x:  the x position of the mouse
 * int y:  the y position of the mouse
 */
void mouseMove(int x, int y)
{
	if (__DEBUG == 1)
	{
		printf("mouse moved at %d %d\n", x, y);
	}

	if (user_transform == USER_TRANSLATE)
	{
		mouseTranslate(x, y);
	}

	if (user_transform == USER_SCALE)
	{
		if (user_object == OBJECT_MESH)
		{
			mouseScale(x, y);
		}
	}

	if (user_transform == USER_ROTATE)
	{
		if (user_object == OBJECT_MESH)
		{
			mouseRotate(x, y);
		}
	}
}

/**
 * This function defines a right-click context menu.  Available choices
 * are material type, shading type, object to manipulate, and transformation
 * type.  Each of these choices has a related submenu with more detailed
 * choices.
 * int value:  the menu selection chosen by the user
 **/
void main_menu(int value)
{
	// There are no atomic high-level choices in the main menu
}

/**
 * This function defines the behavior of the "Material type" submenu.  It
 * allows the user to select from "Gold", "Turquoise", and "Emerald". After
 * the selection has been made, the wireframe object will now be colored
 * according to the selection.
 * int value:  the menu selection chosen by the user
 **/
void material_submenu(int value)
{
	user_material = value;
	switch (value)
	{
	case MATERIAL_GOLD:
		materials(gold);
		break;
	case MATERIAL_TURQUOISE:
		materials(turquoise);
		break;
	case MATERIAL_EMERALD:
		materials(emerald);
		break;
	default:
		printf("User selected emerald material.\n");
		break;
	}
	if (__DEBUG == 1)
	{
		if (value == MATERIAL_GOLD)
		{
			printf("User selected gold material.\n");
		}
		else if (value == MATERIAL_TURQUOISE)
		{
			printf("User selected turquoise material.\n");
		}
		else if (value == MATERIAL_EMERALD)
		{
			printf("User selected emerald material.\n");
		}
		else
		{
			printf("Invalid material chosen.\n");
		}
	}
}

/**
 * This function defines the behavior of the "Shade type" submenu.  It
 * allows the user to select from "Wireframe", "Fill", "Flat" and "Smooth".
 * After the selection has been made, the wireframe object will now be
 * shaded according to the selection.
 * int value:  the menu selection chosen by the user
 **/
void shade_submenu(int value)
{
	user_shade = value;
	switch (value)
	{
	case SHADE_WIREFRAME:
		break;
	case SHADE_FILL:
		break;
	case SHADE_FLAT:
		glShadeModel(GL_FLAT);
		break;
	case SHADE_SMOOTH:
		glShadeModel(GL_SMOOTH);
		break;
	}
	if (__DEBUG == 1)
	{
		if (value == SHADE_WIREFRAME)
		{
			printf("User selected to display only wireframe.\n");
		}
		else if (value == SHADE_FILL)
		{
			printf("User selected to display filled in polygons.\n");
		}
		else if (value == SHADE_FLAT)
		{
			printf("User selected flat polygon shading.\n");
		}
		else if (value == SHADE_SMOOTH)
		{
			printf("User selected smooth polygon shading.\n");
		}
		else
		{
			printf("Invalid shading style chosen.\n");
		}
	}
}

/**
 * This function defines the behavior of the "Object to manipulate" submenu.
 * It allows the user to select from "Mesh" and "Light".  After the selection
 * has been made, either the wireframe object or the point light source will
 * be manipulable.
 * int value:  the menu selection chosen by the user
 **/
void object_submenu(int value)
{
	user_object = value;
	if (__DEBUG == 1)
	{
		if (value == OBJECT_MESH)
		{
			printf("User selected to manipulate the mesh.\n");
		}
		else if (value == OBJECT_LIGHT)
		{
			printf("User selected to manipulate the light.\n");
		}
		else
		{
			printf("Invalid object to manipulate chosen.\n");
		}
	}
}


/**
 * This function defines the behavior of the "Transformation type" submenu.
 * It allows the user to select from "Translate", "Scale", "Rotate", and
 * "No transform".  After the selection has been made, either the wireframe
 * object or the point light source will be transformable with the mouse.
 * int value:  the menu selection chosen by the user
 **/
void transform_submenu(int value)
{
	user_transform = value;
	if (__DEBUG == 1)
	{
		if (value == USER_TRANSLATE)
		{
			printf("User chose to translate.\n");
		}
		else if (value == USER_SCALE)
		{
			printf("User chose to scale.\n");
		}
		else if (value == USER_ROTATE)
		{
			printf("User chose to rotate.\n");
		}
		else if (value == USER_NULL)
		{
			printf("User chose not to transform.\n");
		}
		else
		{
			printf("Invalid user choice.\n");
		}
	}
}

/**
 * This function will create the right-click context menus.
 **/
void doMenus()
{
	// Identifiers for the menus
	int menu, submenu1, submenu2, submenu3, submenu4;

	// Create the material submenu
	submenu1 = glutCreateMenu(material_submenu);
	glutAddMenuEntry("Gold", MATERIAL_GOLD);
	glutAddMenuEntry("Turquoise", MATERIAL_TURQUOISE);
	glutAddMenuEntry("Emerald", MATERIAL_EMERALD);

	// Create the shading submenu
	submenu2 = glutCreateMenu(shade_submenu);
	glutAddMenuEntry("Wireframe", SHADE_WIREFRAME);
	glutAddMenuEntry("Fill", SHADE_FILL);
	glutAddMenuEntry("Flat", SHADE_FLAT);
	glutAddMenuEntry("Smooth", SHADE_SMOOTH);

	// Create the object to manipulate submenu
	submenu3 = glutCreateMenu(object_submenu);
	glutAddMenuEntry("Mesh", OBJECT_MESH);
	glutAddMenuEntry("Light", OBJECT_LIGHT);

	// Create the transformation submenu
	submenu4 = glutCreateMenu(transform_submenu);
	glutAddMenuEntry("Translate", USER_TRANSLATE);
	glutAddMenuEntry("Scale", USER_SCALE);
	glutAddMenuEntry("Rotate", USER_ROTATE);
	glutAddMenuEntry("No transform", USER_NULL);

	// Create the main menu
	menu = glutCreateMenu(main_menu);
	glutAddSubMenu("Material type", submenu1);
	glutAddSubMenu("Shading type", submenu2);
	glutAddSubMenu("Object to manipulate", submenu3);
	glutAddSubMenu("Transform type", submenu4);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void calcEdges() {
	int j;
	bool found;

	for (int i = 0; i < Triangles.size(); i++)
	{
		found = false;
		for (j = 0; j < cutPathEdges.size(); j++)
		{
			if ((cutPathEdges[j]->v1 == Triangles[i]->v1
				&& cutPathEdges[j]->v2 == Triangles[i]->v2)
				|| (cutPathEdges[j]->v1 == Triangles[i]->v2
					&& cutPathEdges[j]->v2 == Triangles[i]->v1))
				found = true;
		}
		if (!found)
			cutPathEdges.push_back(new Edge(Triangles[i]->v1, Triangles[i]->v2, Vector3(1, 0, 0)));

		found = false;
		for (j = 0; j < cutPathEdges.size(); j++)
		{
			if ((cutPathEdges[j]->v1 == Triangles[i]->v2
				&& cutPathEdges[j]->v2 == Triangles[i]->v3)
				|| (cutPathEdges[j]->v1 == Triangles[i]->v3
					&& cutPathEdges[j]->v2 == Triangles[i]->v2))
				found = true;
		}
		if (!found)
			cutPathEdges.push_back(new Edge(Triangles[i]->v2, Triangles[i]->v3, Vector3(1, 0, 0)));

		found = false;
		for (j = 0; j < cutPathEdges.size(); j++)
		{
			if ((cutPathEdges[j]->v1 == Triangles[i]->v1
				&& cutPathEdges[j]->v2 == Triangles[i]->v3)
				|| (cutPathEdges[j]->v1 == Triangles[i]->v3
					&& cutPathEdges[j]->v2 == Triangles[i]->v1))
				found = true;
		}
		if (!found)
			cutPathEdges.push_back(new Edge(Triangles[i]->v1, Triangles[i]->v3, Vector3(1, 0, 0)));
	}
}

void copyGeometry() {
	int i;
	for (i = 0; i < Triangles.size(); i++)
	{
		originalTriangles.push_back(new Tri(Triangles[i]->v1, Triangles[i]->v2, Triangles[i]->v3));
	}

	for (i = 0; i < Vertices.size(); i++)
	{
		originalVertices.push_back(new Vector3(Vertices[i]->x, Vertices[i]->y, Vertices[i]->z));
	}

	for (i = 0; i < VPoints.size(); i++)
	{
		originalVPoints.push_back(new Point3(Vector3(VPoints[i]->x, VPoints[i]->y, VPoints[i]->z)));
	}
}

void removeSeedTriangle()
{
	int randTri = ((float)rand()) / RAND_MAX * Triangles.size();
	int i = 0, j;
	vector<Tri *>::iterator triItr;

	for (triItr = Triangles.begin(); triItr != Triangles.end(); triItr++)
	{
		if (i == randTri)
		{
			for (j = 0; j < cutPathEdges.size(); j++)
			{
				if ((cutPathEdges[j]->v1 == Triangles[i]->v1
					&& cutPathEdges[j]->v2 == Triangles[i]->v2)
					|| (cutPathEdges[j]->v1 == Triangles[i]->v2
						&& cutPathEdges[j]->v2 == Triangles[i]->v1))
				{
					cutPathEdges[j]->color.x = 0;
					cutPathEdges[j]->color.y = 1;
				}

				if ((cutPathEdges[j]->v1 == Triangles[i]->v2
					&& cutPathEdges[j]->v2 == Triangles[i]->v3)
					|| (cutPathEdges[j]->v1 == Triangles[i]->v3
						&& cutPathEdges[j]->v2 == Triangles[i]->v2))
				{
					cutPathEdges[j]->color.x = 0;
					cutPathEdges[j]->color.y = 1;
				}

				if ((cutPathEdges[j]->v1 == Triangles[i]->v1
					&& cutPathEdges[j]->v2 == Triangles[i]->v3)
					|| (cutPathEdges[j]->v1 == Triangles[i]->v3
						&& cutPathEdges[j]->v2 == Triangles[i]->v1))
				{
					cutPathEdges[j]->color.x = 0;
					cutPathEdges[j]->color.y = 1;
				}
			}
			Triangles.erase(triItr);
			break;
		}
		i++;
	}
}

void createInitialCutPart1()
{
	bool inMoreThanOneTriangle = false;
	bool foundEdge = false;
	int triInd, edgeInd;

	// IF there remains an edge e adjacent to only one triangle t
	//     remove e and t
	for (int i = 0; i < cutPathEdges.size(); i++)
	{
		foundEdge = false;
		for (int j = 0; j < Triangles.size(); j++)
		{
			if ((cutPathEdges[i]->v1 == Triangles[j]->v1
				&& cutPathEdges[i]->v2 == Triangles[j]->v2)
				|| (cutPathEdges[i]->v1 == Triangles[j]->v2
					&& cutPathEdges[i]->v2 == Triangles[j]->v1))
			{
				if (!foundEdge)
				{
					triInd = j;
					foundEdge = true;
				}
				else
				{
					inMoreThanOneTriangle = true;
					break;
				}
			}
			else if ((cutPathEdges[i]->v1 == Triangles[j]->v2
				&& cutPathEdges[i]->v2 == Triangles[j]->v3)
				|| (cutPathEdges[i]->v1 == Triangles[j]->v3
					&& cutPathEdges[i]->v2 == Triangles[j]->v2))
			{
				if (!foundEdge)
				{
					triInd = j;
					foundEdge = true;
				}
				else
				{
					inMoreThanOneTriangle = true;
					break;
				}
			}
			else if ((cutPathEdges[i]->v1 == Triangles[j]->v1
				&& cutPathEdges[i]->v2 == Triangles[j]->v3)
				|| (cutPathEdges[i]->v1 == Triangles[j]->v3
					&& cutPathEdges[i]->v2 == Triangles[j]->v1))
			{
				if (!foundEdge)
				{
					triInd = j;
					foundEdge = true;
				}
				else
				{
					inMoreThanOneTriangle = true;
					break;
				}
			}
			else
			{
				// Do nothing
			}
		}

		// IF one edge found
		if (foundEdge && !inMoreThanOneTriangle)
		{
			edgeInd = i;
			break;
		}
		else
		{
			inMoreThanOneTriangle = false;
		}
	}

	// IF found one edge, remove that edge and triangle
	if (foundEdge && !inMoreThanOneTriangle)
	{
		vector<Tri *>::iterator triItr;
		vector<Edge *>::iterator edgeItr;
		int edgeNum = 0, triNum = 0;

		for (triItr = Triangles.begin(); triItr != Triangles.end(); triItr++)
		{
			// IF this is the triangle to remove
			if (triInd == triNum)
			{
				// Search for each pair of vertices in cutPathEdges
				for (int i = 0; i < cutPathEdges.size(); i++)
				{
					if ((cutPathEdges[i]->v1 == Triangles[triInd]->v1
						&& cutPathEdges[i]->v2 == Triangles[triInd]->v2)
						|| (cutPathEdges[i]->v1 == Triangles[triInd]->v2
							&& cutPathEdges[i]->v2 == Triangles[triInd]->v1))
					{
						cutPathEdges[i]->color.x = 0;
						cutPathEdges[i]->color.y = 1;
					}
					else if ((cutPathEdges[i]->v1 == Triangles[triInd]->v2
						&& cutPathEdges[i]->v2 == Triangles[triInd]->v3)
						|| (cutPathEdges[i]->v1 == Triangles[triInd]->v3
							&& cutPathEdges[i]->v2 == Triangles[triInd]->v2))
					{
						cutPathEdges[i]->color.x = 0;
						cutPathEdges[i]->color.y = 1;
					}
					else if ((cutPathEdges[i]->v1 == Triangles[triInd]->v1
						&& cutPathEdges[i]->v2 == Triangles[triInd]->v3)
						|| (cutPathEdges[i]->v1 == Triangles[triInd]->v3
							&& cutPathEdges[i]->v2 == Triangles[triInd]->v1))
					{
						cutPathEdges[i]->color.x = 0;
						cutPathEdges[i]->color.y = 1;
					}
					else
					{
						// Do nothing
					}
				}
				Triangles.erase(triItr);
				break;
			}
			triNum++;
		}

		for (edgeItr = cutPathEdges.begin(); edgeItr != cutPathEdges.end(); edgeItr++)
		{
			// IF this is the edge to remove
			if (edgeInd == edgeNum)
			{
				cutPathEdges.erase(edgeItr);
				break;
			}
			edgeNum++;
		}
	}
	else
	{
		removedEandT = true;
	}
}

void createInitialCutPart2()
{
	// IF there remains a vertex v adjacent to only one edge e
	//     remove v and e
	bool inMoreThanOneEdge = false;
	bool foundVertex = false;
	int vertInd, edgeInd;
	int count = 0;

	// IF there remains a vertex v adjacent to only one edge e
	//     remove v and e
	for (int i = 0; i < Vertices.size(); i++)
	{
		foundVertex = false;
		for (int j = 0; j < cutPathEdges.size(); j++)
		{
			if ((i == cutPathEdges[j]->v1) || (i == cutPathEdges[j]->v2))
			{
				if (!foundVertex)
				{
					edgeInd = j;
					foundVertex = true;
				}
				else
				{
					inMoreThanOneEdge = true;
					break;
				}
			}
			else
			{
				// Do nothing
			}
		}

		// IF one vertex found
		if (foundVertex && !inMoreThanOneEdge)
		{
			vertInd = i;
			break;
		}
		else
		{
			inMoreThanOneEdge = false;
		}
	}

	// IF found one edge, remove that edge and triangle
	if (foundVertex && !inMoreThanOneEdge)
	{
		vector<Vector3 *>::iterator vertItr;
		vector<Edge *>::iterator edgeItr;
		int edgeNum = 0, vertNum = 0;

		if (cutPathEdges.size() <= 2)
		{
			removedVandE = true;

			for (int i = 0; i < cutPathEdges.size(); i++)
			{
				cutPathEdges[i]->color.x = 0;
				cutPathEdges[i]->color.y = 1;
				cutPathEdges[i]->color.z = 0;
			}

			return;
		}

		for (edgeItr = cutPathEdges.begin(); edgeItr != cutPathEdges.end(); edgeItr++)
		{
			// IF this is the edge to remove
			if (edgeInd == edgeNum)
			{
				cutPathEdges.erase(edgeItr);
				break;
			}
			edgeNum++;
		}
	}
	else
	{
		removedVandE = true;
	}


	for (int i = 0; i < cutPathEdges.size(); i++)
	{
		bool v1flag = false;
		bool v2flag = false;

		for (int j = 0; j < cutPathEdges.size(); j++)
		{
			if (i != j)
			{
				if (cutPathEdges[i]->v1 == cutPathEdges[j]->v1)
				{
					v1flag = true;
				}
				else if (cutPathEdges[i]->v1 == cutPathEdges[j]->v2)
				{
					v1flag = true;
				}
				else if (cutPathEdges[i]->v2 == cutPathEdges[j]->v1)
				{
					v2flag = true;
				}
				else if (cutPathEdges[i]->v2 == cutPathEdges[j]->v2)
				{
					v2flag = true;
				}
				else
				{
					// Do nothing
				}
			}
		}

		if (v1flag && v2flag)
		{

		}
		else
		{
			cutPathEdges[i]->color.x = 1;
			cutPathEdges[i]->color.y = .6;
			cutPathEdges[i]->color.z = .6;
		}
	}
}

void recreateMesh()
{
	//GIMxInd and GIMyInd are indices into the image 2d array of 256x256
	//myimage is the GIM
	float dist1 = sqrt((float)((myimage[GIMxInd][GIMyInd].r - myimage[GIMxInd + 1][GIMyInd + 1].r)*(myimage[GIMxInd][GIMyInd].r - myimage[GIMxInd + 1][GIMyInd + 1].r)
		+ (myimage[GIMxInd][GIMyInd].g - myimage[GIMxInd + 1][GIMyInd + 1].g)*(myimage[GIMxInd][GIMyInd].g - myimage[GIMxInd + 1][GIMyInd + 1].g)
		+ (myimage[GIMxInd][GIMyInd].b - myimage[GIMxInd + 1][GIMyInd + 1].b)*(myimage[GIMxInd][GIMyInd].b - myimage[GIMxInd + 1][GIMyInd + 1].b)));

	float dist2 = sqrt((float)((myimage[GIMxInd + 1][GIMyInd].r - myimage[GIMxInd][GIMyInd + 1].r)*(myimage[GIMxInd + 1][GIMyInd].r - myimage[GIMxInd][GIMyInd + 1].r)
		+ (myimage[GIMxInd + 1][GIMyInd].g - myimage[GIMxInd][GIMyInd + 1].g)*(myimage[GIMxInd + 1][GIMyInd].g - myimage[GIMxInd][GIMyInd + 1].g)
		+ (myimage[GIMxInd + 1][GIMyInd].b - myimage[GIMxInd][GIMyInd + 1].b)*(myimage[GIMxInd + 1][GIMyInd].b - myimage[GIMxInd][GIMyInd + 1].b)));

	// 0 (size - 3)
	newVertices.push_back(new Vector3(myimage[GIMxInd][GIMyInd].r, myimage[GIMxInd][GIMyInd].g, myimage[GIMxInd][GIMyInd].b));
	newVPoints.push_back(new Point3(Vector3(myimage[GIMxInd][GIMyInd].r, myimage[GIMxInd][GIMyInd].g, myimage[GIMxInd][GIMyInd].b)));
	newVPoints.at(newVPoints.size() - 1)->normal.x = mynormalimage[GIMxInd][GIMyInd].r;
	newVPoints.at(newVPoints.size() - 1)->normal.y = mynormalimage[GIMxInd][GIMyInd].g;
	newVPoints.at(newVPoints.size() - 1)->normal.z = mynormalimage[GIMxInd][GIMyInd].b;

	// 1 (size - 2)
	newVertices.push_back(new Vector3(myimage[GIMxInd + 1][GIMyInd].r, myimage[GIMxInd + 1][GIMyInd].g, myimage[GIMxInd + 1][GIMyInd].b));
	newVPoints.push_back(new Point3(Vector3(myimage[GIMxInd + 1][GIMyInd].r, myimage[GIMxInd + 1][GIMyInd].g, myimage[GIMxInd + 1][GIMyInd].b)));
	newVPoints.at(newVPoints.size() - 1)->normal.x = mynormalimage[GIMxInd + 1][GIMyInd].r;
	newVPoints.at(newVPoints.size() - 1)->normal.y = mynormalimage[GIMxInd + 1][GIMyInd].g;
	newVPoints.at(newVPoints.size() - 1)->normal.z = mynormalimage[GIMxInd + 1][GIMyInd].b;

	// 2 (size - 1)
	newVertices.push_back(new Vector3(myimage[GIMxInd][GIMyInd + 1].r, myimage[GIMxInd][GIMyInd + 1].g, myimage[GIMxInd][GIMyInd + 1].b));
	newVPoints.push_back(new Point3(Vector3(myimage[GIMxInd][GIMyInd + 1].r, myimage[GIMxInd][GIMyInd + 1].g, myimage[GIMxInd][GIMyInd + 1].b)));
	newVPoints.at(newVPoints.size() - 1)->normal.x = mynormalimage[GIMxInd][GIMyInd + 1].r;
	newVPoints.at(newVPoints.size() - 1)->normal.y = mynormalimage[GIMxInd][GIMyInd + 1].g;
	newVPoints.at(newVPoints.size() - 1)->normal.z = mynormalimage[GIMxInd][GIMyInd + 1].b;

	// 3 (size)
	newVertices.push_back(new Vector3(myimage[GIMxInd + 1][GIMyInd + 1].r, myimage[GIMxInd + 1][GIMyInd + 1].g, myimage[GIMxInd + 1][GIMyInd + 1].b));
	newVPoints.push_back(new Point3(Vector3(myimage[GIMxInd + 1][GIMyInd + 1].r, myimage[GIMxInd + 1][GIMyInd + 1].g, myimage[GIMxInd + 1][GIMyInd + 1].b)));
	newVPoints.at(newVPoints.size() - 1)->normal.x = mynormalimage[GIMxInd + 1][GIMyInd + 1].r;
	newVPoints.at(newVPoints.size() - 1)->normal.y = mynormalimage[GIMxInd + 1][GIMyInd + 1].g;
	newVPoints.at(newVPoints.size() - 1)->normal.z = mynormalimage[GIMxInd + 1][GIMyInd + 1].b;

	if (dist1 <= dist2)
	{
		// 0, 1, 3
		newTriangles.push_back(new Tri(newVertices.size() - 3, newVertices.size() - 2, newVertices.size()));
		// 0, 2, 3
		newTriangles.push_back(new Tri(newVertices.size() - 3, newVertices.size() - 1, newVertices.size()));
		pixelToTri[GIMxInd][GIMyInd].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd][GIMyInd].push_back(newTriangles.size() - 1);
		pixelToTri[GIMxInd + 1][GIMyInd].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd][GIMyInd + 1].push_back(newTriangles.size() - 1);
		pixelToTri[GIMxInd + 1][GIMyInd + 1].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd + 1][GIMyInd + 1].push_back(newTriangles.size() - 1);
	}
	else
	{
		// 1, 0, 2
		newTriangles.push_back(new Tri(newVertices.size() - 2, newVertices.size() - 3, newVertices.size() - 1));
		// 1, 3, 2
		newTriangles.push_back(new Tri(newVertices.size() - 2, newVertices.size(), newVertices.size() - 1));
		pixelToTri[GIMxInd][GIMyInd].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd + 1][GIMyInd].push_back(newTriangles.size() - 1);
		pixelToTri[GIMxInd + 1][GIMyInd].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd][GIMyInd + 1].push_back(newTriangles.size() - 1);
		pixelToTri[GIMxInd][GIMyInd + 1].push_back(newTriangles.size() - 2);
		pixelToTri[GIMxInd + 1][GIMyInd + 1].push_back(newTriangles.size() - 1);
	}

	myimage[GIMxInd][GIMyInd].r = myimage[GIMxInd][GIMyInd].g = myimage[GIMxInd][GIMyInd].b = 1.0;
	mynormalimage[GIMxInd][GIMyInd].r = mynormalimage[GIMxInd][GIMyInd].g = mynormalimage[GIMxInd][GIMyInd].b = 1.0;

	if (GIMyInd == 63)
	{
		myimage[GIMxInd][GIMyInd + 1].r = myimage[GIMxInd][GIMyInd + 1].g = myimage[GIMxInd][GIMyInd + 1].b = 1.0;
		mynormalimage[GIMxInd][GIMyInd + 1].r = mynormalimage[GIMxInd][GIMyInd + 1].g = mynormalimage[GIMxInd][GIMyInd + 1].b = 1.0;
	}

	if (GIMxInd < 63)
	{
		GIMxInd++;
	}
	else
	{
		myimage[GIMxInd + 1][GIMyInd].r = myimage[GIMxInd + 1][GIMyInd].g = myimage[GIMxInd + 1][GIMyInd].b = 1.0;
		mynormalimage[GIMxInd + 1][GIMyInd].r = mynormalimage[GIMxInd + 1][GIMyInd].g = mynormalimage[GIMxInd + 1][GIMyInd].b = 1.0;

		GIMxInd = 0;
		GIMyInd++;
		if (GIMyInd > 63)
		{
			myimage[64][64].r = myimage[64][64].g = myimage[64][64].b = 1.0;
			mynormalimage[64][64].r = mynormalimage[64][64].g = mynormalimage[64][64].b = 1.0;

			recreatedMesh = true;
			printf("Finished reconstruction!\n");

			// Build the quad tree
			quadTree = createQuadTree(65, 0, 0, myimage2);
			printf("Built quad tree\n");
		}
	}
	//calcAllNewVertexNormals();
}

int main(int argc, char** argv)
{
	srand(time(0));

	// Set up my window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Geometry Images: Cutting Algorithm Walkthrough + Rendering Walkthrough With Normal Map and Optional Culling");
	glClearColor(0.0, 0.0, 0.0, 1.0);

	// Register glut callback functions and menus
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMove);
	doMenus();

	// Enable z-buffe
	glEnable(GL_DEPTH_TEST);

	//LoadTexture("S:/csc570/FinalProject/Debug/images/bunny4.bmp", 1);
	//LoadTextureNormal("S:/csc570/FinalProject/Debug/images/bunnynormals.bmp", 2);
	LoadTexture("C:/Program Files/GIM/images/bunny4.bmp", 1);
	LoadTextureNormal("C:/Program Files/GIM/images/bunnynormals.bmp", 2);

	// Initialization
	max_x = max_y = max_z = FLT_MIN;
	min_x = min_y = min_z = FLT_MAX;
	center.x = 0;
	center.y = 0;
	center.z = 0;
	scaleFactor = 3.6;
	max_extent = 1.0;
	lightScale = 1;
	xw = yw = 0;
	user_material = MATERIAL_GOLD;
	user_object = OBJECT_MESH;
	user_transform = USER_ROTATE;
	user_shade = SHADE_SMOOTH;
	myMatrix[0] = myMatrix[5] = myMatrix[10] = myMatrix[15] = 1.0;
	theta = 0;
	vInitial = Vector3(0., 0., 0.);
	xTri = GW / 2;
	yTri = GH / 2;
	removedEandT = false;
	removedVandE = false;
	finishedInitialCut = false;
	recreating = false;
	recreatedMesh = false;
	cullingon = false;
	cubex = cubey = cubez = 0.0;
	eyez = 3.0;

	stepping_through = false;

	GIMxInd = 0;
	GIMyInd = 0;

	// Make sure a file to read is specified
	if (argc > 1)
	{
		cout << "file " << argv[1] << endl;
		// Read-in the mesh file specified
		ReadFile(argv[1]);
		//ReadFile("gargoyle500.m");

		// Once the file is parsed find out the maximum extent to center
		// and scale mesh
		max_extent = max_x - min_x;
		if (max_y - min_y > max_extent) max_extent = max_y - min_y;

		center.x = center.x / Vertices.size();
		center.y = center.y / Vertices.size();
		center.z = center.z / Vertices.size();
	}
	else
	{
		cout << "format is: meshparser filename" << endl;
	}
	calcAllVertexNormals();

	// Final project stuff
	copyGeometry();
	calcEdges();
	removeSeedTriangle();
	makeMyImage();
	makeMyNormalImage();
	init_lighting();
	glutMainLoop();
}

//general intialization for opengl for depth and texture mapping
void init() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
}

//initialization for texture mapping
void init_tex() {
	//glBindTexture(GL_TEXTURE_2D, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, pixel);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
}

//routines to load in a bmp files - must be 2^nx2^m and a 24bit bmp
GLvoid LoadTexture(char* image_file, int texID) {

	TextureImage = (Image *)malloc(sizeof(Image));
	if (TextureImage == NULL) {
		printf("Error allocating space for image");
		exit(1);
	}
	cout << "trying to load " << image_file << endl;
	if (!ImageLoad(image_file, TextureImage)) {
		exit(1);
	}
	/*  2d texture, level of detail 0 (normal), 3 components (red, green, blue),            */
	/*  x size from image, y size from image,                                              */
	/*  border 0 (normal), rgb color data, unsigned byte data, data  */
	glBindTexture(GL_TEXTURE_2D, texID);
	glTexImage2D(GL_TEXTURE_2D, 0, 3,
		TextureImage->sizeX, TextureImage->sizeY,
		0, GL_RGB, GL_UNSIGNED_BYTE, TextureImage->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); /*  cheap scaling when image bigger than texture */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); /*  cheap scaling when image smalled than texture*/

}


/* BMP file loader loads a 24-bit bmp file only */

/*
* getint and getshort are help functions to load the bitmap byte by byte
*/
static unsigned int getint(FILE *fp) {
	int c, c1, c2, c3;

	/*  get 4 bytes */
	c = getc(fp);
	c1 = getc(fp);
	c2 = getc(fp);
	c3 = getc(fp);

	return ((unsigned int)c) +
		(((unsigned int)c1) << 8) +
		(((unsigned int)c2) << 16) +
		(((unsigned int)c3) << 24);
}

static unsigned int getshort(FILE *fp) {
	int c, c1;

	/* get 2 bytes*/
	c = getc(fp);
	c1 = getc(fp);

	return ((unsigned int)c) + (((unsigned int)c1) << 8);
}

/*  quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.  */

int ImageLoad(char *filename, Image *image) {
	FILE *file;
	unsigned long size;                 /*  size of the image in bytes. */
	unsigned long i;                    /*  standard counter. */
	unsigned short int planes;          /*  number of planes in image (must be 1)  */
	unsigned short int bpp;             /*  number of bits per pixel (must be 24) */
	char temp;                          /*  used to convert bgr to rgb color. */

	/*  make sure the file is there. */
	if ((file = fopen(filename, "rb")) == NULL) {
		printf("File Not Found : %s\n", filename);
		return 0;
	}

	/*  seek through the bmp header, up to the width height: */
	fseek(file, 18, SEEK_CUR);

	/*  No 100% errorchecking anymore!!! */

	/*  read the width */    image->sizeX = getint(file);

	/*  read the height */
	image->sizeY = getint(file);

	/*  calculate the size (assuming 24 bits or 3 bytes per pixel). */
	size = image->sizeX * image->sizeY * 3;

	/*  read the planes */
	planes = getshort(file);
	if (planes != 1) {
		printf("Planes from %s is not 1: %u\n", filename, planes);
		return 0;
	}

	/*  read the bpp */
	bpp = getshort(file);
	if (bpp != 24) {
		printf("Bpp from %s is not 24: %u\n", filename, bpp);
		return 0;
	}

	/*  seek past the rest of the bitmap header. */
	fseek(file, 24, SEEK_CUR);

	/*  read the data.  */
	image->data = (char *)malloc(size);
	if (image->data == NULL) {
		printf("Error allocating memory for color-corrected image data");
		return 0;
	}

	if ((i = fread(image->data, size, 1, file)) != 1) {
		printf("Error reading image data from %s.\n", filename);
		return 0;
	}

	for (i = 0; i < size; i += 3) { /*  reverse all of the colors. (bgr -> rgb) */
		temp = image->data[i];
		image->data[i] = image->data[i + 2];
		image->data[i + 2] = temp;
	}

	fclose(file); /* Close the file and release the filedes */

	/*  we're done. */
	return 1;
}

void makeMyImage() {
	int myX = 0, myY = 0;

	for (int i = 0; i < TextureImage->sizeX * TextureImage->sizeY * 3; i += 3)
	{
		myimage[myX][myY].r = ((float)((unsigned char)(TextureImage->data[i]))) / 255.0;
		myimage[myX][myY].g = ((float)((unsigned char)(TextureImage->data[i + 1]))) / 255.0;
		myimage[myX][myY].b = ((float)((unsigned char)(TextureImage->data[i + 2]))) / 255.0;

		myimage2[myX][myY].r = ((float)((unsigned char)(TextureImage->data[i]))) / 255.0;
		myimage2[myX][myY].g = ((float)((unsigned char)(TextureImage->data[i + 1]))) / 255.0;
		myimage2[myX][myY].b = ((float)((unsigned char)(TextureImage->data[i + 2]))) / 255.0;

		myimage3[myX][myY].r = ((float)((unsigned char)(TextureImage->data[i]))) / 255.0;
		myimage3[myX][myY].g = ((float)((unsigned char)(TextureImage->data[i + 1]))) / 255.0;
		myimage3[myX][myY].b = ((float)((unsigned char)(TextureImage->data[i + 2]))) / 255.0;

		if (myX < 64)
		{
			myX++;
		}
		else
		{
			i += 1341;
			myX = 0;
			myY++;
		}

		if (myY > 64)
		{
			break;
		}
	}
}

GLvoid LoadTextureNormal(char* image_file, int texID) {

	TextureNormalImage = (Image *)malloc(sizeof(Image));
	if (TextureNormalImage == NULL) {
		printf("Error allocating space for image");
		exit(1);
	}
	cout << "trying to load " << image_file << endl;
	if (!NormalImageLoad(image_file, TextureNormalImage)) {
		exit(1);
	}
	/*  2d texture, level of detail 0 (normal), 3 components (red, green, blue),            */
	/*  x size from image, y size from image,                                              */
	/*  border 0 (normal), rgb color data, unsigned byte data, data  */
	glBindTexture(GL_TEXTURE_2D, texID);
	glTexImage2D(GL_TEXTURE_2D, 0, 3,
		TextureNormalImage->sizeX, TextureNormalImage->sizeY,
		0, GL_RGB, GL_UNSIGNED_BYTE, TextureNormalImage->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); /*  cheap scaling when image bigger than texture */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); /*  cheap scaling when image smalled than texture*/

}

int NormalImageLoad(char *filename, Image *image) {
	FILE *file;
	unsigned long size;                 /*  size of the image in bytes. */
	unsigned long i;                    /*  standard counter. */
	unsigned short int planes;          /*  number of planes in image (must be 1)  */
	unsigned short int bpp;             /*  number of bits per pixel (must be 24) */
	char temp;                          /*  used to convert bgr to rgb color. */

	/*  make sure the file is there. */
	if ((file = fopen(filename, "rb")) == NULL) {
		printf("File Not Found : %s\n", filename);
		return 0;
	}

	/*  seek through the bmp header, up to the width height: */
	fseek(file, 18, SEEK_CUR);

	/*  No 100% errorchecking anymore!!! */

	/*  read the width */    image->sizeX = getint(file);

	/*  read the height */
	image->sizeY = getint(file);

	/*  calculate the size (assuming 24 bits or 3 bytes per pixel). */
	size = image->sizeX * image->sizeY * 3;

	/*  read the planes */
	planes = getshort(file);
	if (planes != 1) {
		printf("Planes from %s is not 1: %u\n", filename, planes);
		return 0;
	}

	/*  read the bpp */
	bpp = getshort(file);
	if (bpp != 24) {
		printf("Bpp from %s is not 24: %u\n", filename, bpp);
		return 0;
	}

	/*  seek past the rest of the bitmap header. */
	fseek(file, 24, SEEK_CUR);

	/*  read the data.  */
	image->data = (char *)malloc(size);
	if (image->data == NULL) {
		printf("Error allocating memory for color-corrected image data");
		return 0;
	}

	if ((i = fread(image->data, size, 1, file)) != 1) {
		printf("Error reading image data from %s.\n", filename);
		return 0;
	}

	for (i = 0; i < size; i += 3) { /*  reverse all of the colors. (bgr -> rgb) */
		temp = image->data[i];
		image->data[i] = image->data[i + 2];
		image->data[i + 2] = temp;
	}

	fclose(file); /* Close the file and release the filedes */

	/*  we're done. */
	return 1;
}



void makeMyNormalImage()
{
	int myX = 0, myY = 0;

	for (int i = 0; i < TextureNormalImage->sizeX * TextureNormalImage->sizeY * 3; i += 3)
	{
		mynormalimage[myX][myY].r = ((float)((unsigned char)(TextureNormalImage->data[i]))) / 255.0;
		mynormalimage[myX][myY].g = ((float)((unsigned char)(TextureNormalImage->data[i + 1]))) / 255.0;
		mynormalimage[myX][myY].b = ((float)((unsigned char)(TextureNormalImage->data[i + 2]))) / 255.0;

		if (myX < 64)
		{
			myX++;
		}
		else
		{
			i += 1341;
			myX = 0;
			myY++;
		}

		if (myY > 64)
		{
			break;
		}
	}
}

bool insideBox(RGB value)
{
	return (value.r > cubex && value.r < cubex + 1
		&& value.g > cubey && value.g < cubey + 1
		&& value.b > cubez && value.b < cubez + 1);
}

bool insideBox(GIMQuadTreeNode * node)
{
	for (int i = node->startPosX; i < node->startPosX + node->size; i++)
	{
		for (int j = node->startPosY; j < node->startPosY + node->size; j++)
		{
			if (insideBox(myimage2[i][j]))
			{
				return true;
			}
		}
	}

	return false;
}

void traverseTree(GIMQuadTreeNode * node)
{
	// IF the pixels are inside the box, draw them
	if (node->size == 1)
	{
		if (insideBox(myimage2[node->startPosX][node->startPosY]))
		{
			for (int k = 0; k < pixelToTri[node->startPosX][node->startPosY].size(); k++)
			{
				newTriangles[pixelToTri[node->startPosX][node->startPosY][k]]->drawn = true;
			}
		}

		return;
	}
	else if (insideBox(node))
	{
		// Check each quad
		traverseTree(node->lowerLeft);
		traverseTree(node->lowerRight);
		traverseTree(node->upperLeft);
		traverseTree(node->upperRight);
	}
	// Not in box
	else
	{
		if (node->size == 65)
		{
			for (int i = node->startPosX; i < node->startPosX + node->size; i++)
			{
				for (int j = node->startPosY; j < node->startPosY + node->size; j++)
				{
					myimage2[i][j].r = myimage2[i][j].g = myimage2[i][j].b = 0;
				}
			}
		}
		else
		{
			// Make myimage2 black
			for (int i = node->startPosX; i <= node->startPosX + node->size; i++)
			{
				for (int j = node->startPosY; j <= node->startPosY + node->size; j++)
				{
					myimage2[i][j].r = myimage2[i][j].g = myimage2[i][j].b = 0;
				}
			}
		}
	}
}

void cull()
{
	// Restore myimage2 using myimage3
	for (int i = 0; i < 65; i++)
	{
		for (int j = 0; j < 65; j++)
		{
			myimage2[i][j] = myimage3[i][j];
		}
	}

	// Reset culling
	for (int i = 0; i < newTriangles.size(); i++)
	{
		newTriangles[i]->drawn = false;
	}

	// Traverse quadTree to determine what will be culled
	traverseTree(quadTree);
}

/* Tree starts from lower-left (like the image) */
GIMQuadTreeNode * createQuadTree(int size, int startX, int startY, RGB grid[65][65])
{
	GIMQuadTreeNode * node = new GIMQuadTreeNode(size, startX, startY);

	if (size == 1)
	{
		node->value.r = grid[startX][startY].r;
		node->value.g = grid[startX][startY].g;
		node->value.b = grid[startX][startY].b;

		node->upperLeft = NULL;
		node->upperRight = NULL;
		node->lowerLeft = NULL;
		node->lowerRight = NULL;

		return node;
	}
	else
	{
		// Create quads
		node->lowerLeft = createQuadTree(size / 2, startX, startY, grid);
		node->lowerRight = createQuadTree(size / 2, startX + size / 2, startY, grid);
		node->upperLeft = createQuadTree(size / 2, startX, startY + size / 2, grid);
		node->upperRight = createQuadTree(size / 2, startX + size / 2, startY + size / 2, grid);

		return node;
	}
}

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "hw2_types.h"
#include "hw2_math_ops.h"
#include "hw2_file_ops.h"
#include <iostream>
#include <algorithm>

using namespace std;

using namespace std;


Camera cameras[100];
int numberOfCameras = 0;

Model models[1000];
int numberOfModels = 0;

Color colors[100000];
int numberOfColors = 0;

Translation translations[1000];
int numberOfTranslations = 0;

Rotation rotations[1000];
int numberOfRotations = 0;

Scaling scalings[1000];
int numberOfScalings = 0;

Vec3 vertices[100000];
int numberOfVertices = 0;

Color backgroundColor;

// backface culling setting, default disabled
int backfaceCullingSetting = 0;

Color **image;



/*
	Initializes image with background color
*/
void initializeImage(Camera cam) {
    int i, j;

    for (i = 0; i < cam.sizeX; i++)
        for (j = 0; j < cam.sizeY; j++) {
            image[i][j].r = backgroundColor.r;
            image[i][j].g = backgroundColor.g;
            image[i][j].b = backgroundColor.b;

        }
}

/*
	Transformations, culling, rasterization are done here.
	You can define helper functions inside this file (rasterizer.cpp) only.
	Using types in "hw2_types.h" and functions in "hw2_math_ops.cpp" will speed you up while working.
*/

// My Funtions:
void printMatrix(double m[4][4]) {
	for(int i = 0; i < 4; i++) {
		cout << "[";
		for(int j = 0; j < 4; j++) {
			cout << m[i][j] << " ";
		}
		cout << "]\n";
	}
}

void TranslationMatrix(Translation t, double matrix[4][4]) {
	matrix[0][0] = 1;    matrix[0][1] = 0;    matrix[0][2] = 0;    matrix[0][3] = t.tx;    		
	matrix[1][0] = 0;    matrix[1][1] = 1;    matrix[1][2] = 0;    matrix[1][3] = t.ty;
	matrix[2][0] = 0;    matrix[2][1] = 0;    matrix[2][2] = 1;    matrix[2][3] = t.tz;
	matrix[3][0] = 0;    matrix[3][1] = 0;    matrix[3][2] = 0;    matrix[3][3] = 1;
}

void RotationMatrix(Rotation r, double matrix[4][4]) {
	double uvals[3];
	double vvals[3];
	Vec3 u;					// 
	Vec3 v;					//
	Vec3 w;					// 
	double M[4][4];			// Orthonormal matrix uvw
	double Rx_theta[4][4];	// Rotation around x axis

	u.x = r.ux;
	u.y = r.uy;
	u.z = r.uz;
	u = normalizeVec3(u)

	uvals[0] = u.x;
	uvals[1] = u.y;
	uvals[2] = u.z;

	int smallestIdx = 0;
	int smallest    = ABS(uvals[0]);
	for(int i = 1; i < 3; i++) {
		if (ABS(uvals[i]) < smallest) {
			smallest = ABS(uvals[i]);
			smallestIdx = i;
		}
	}

	if(smallestIdx == 0) {
		vvals[0] = 0;
		vvals[1] = -1 * uvals[2];
		vvals[2] = uvals[1];
	}
	else if(smallestIdx == 1) {
		vvals[0] = -1 * uvals[2];
		vvals[1] = 0;
		vvals[2] = uvals[0];
	}
	else {
		vvals[0] = -1 * uvals[1];
		vvals[1] = uvals[0];
		vvals[2] = 0;
	}

	v.x = vvals[0];
	v.y = vvals[1];
	v.z = vvals[2];
	v = normalizeVec3(v);

	w = crossProductVec3(u, v);
	w = normalizeVec3(w);

	M[0][0] = u.x;    M[0][1] = u.y;    M[0][2] = u.z;    M[0][3] = 0;	
	M[1][0] = v.x;    M[1][1] = v.y;    M[1][2] = v.z;    M[1][3] = 0;
	M[2][0] = w.x;    M[2][1] = w.y;    M[2][2] = w.z;    M[2][3] = 0;
	M[3][0] = 0;      M[3][1] = 0;      M[3][2] = 0;      M[3][3] = 1;

	Rx_theta[0][0] = 1;               Rx_theta[0][1] = 0;               Rx_theta[0][2] = 0;               Rx_theta[0][3] = 0;
	Rx_theta[1][0] = 0;               Rx_theta[1][1] = cos(r.angle);    Rx_theta[1][2] = -1*sin(r.angle); Rx_theta[1][3] = 0;
	Rx_theta[2][0] = 0;               Rx_theta[2][1] = sin(r.angle);    Rx_theta[2][2] = cos(r.angle);    Rx_theta[2][3] = 0;
	Rx_theta[3][0] = 0;               Rx_theta[3][1] = 0;               Rx_theta[3][2] = 0;               Rx_theta[3][3] = 1;
	
	// Rotate = Rx_theta * M
	multiplyMatrixWithMatrix(matrix, Rx_theta, M);
}

void ScalingMatrix(Scaling s, double matrix[4][4]) {
	matrix[0][0] = s.sx;	matrix[0][1] = 0;    matrix[0][2] = 0;    matrix[0][3] = 0;
	matrix[1][0] = 0;		matrix[1][1] = s.sy; matrix[1][2] = 0;	  matrix[1][3] = 0;	
	matrix[2][0] = 0;		matrix[2][1] = 0;	 matrix[2][2] = s.sz; matrix[2][3] = 0;
	matrix[3][0] = 0; 		matrix[3][1] = 0;	 matrix[3][2] = 0;	  matrix[3][3] = 1;
}

void ViewportTransform()

// ##########################################################################################################################################
// ##########################################################################################################################################
// ##########################################################################################################################################


void makeCameraTransformation(Camera c, double m[4][4]) {

    //create an identity matrix
    double I[4][4];
    makeIdentityMatrix(I);

    //identity matrix to translation matrix
    Translation t;
    t.tx = c.pos.x;
    t.ty = c.pos.y;
    t.tz = c.pos.z;
    makeTranslationMatrix(t, I);

    double translated[4][4];

    //translation to the point
    multiplyMatrixWithMatrix(translated, I, m);

    //create an identity matrix for rotation
    makeIdentityMatrix(I);

    //create rotation matrix
    I[0][0] = c.u.x;    I[0][1] = c.u.y;    I[0][2] = c.u.z;    I[0][3] = 0;
    I[1][0] = c.v.x;    I[1][1] = c.v.y;    I[1][2] = c.v.z;    I[1][3] = 0;
    I[2][0] = c.w.x;    I[2][1] = c.w.y;    I[2][2] = c.w.z;    I[2][3] = 0;
    I[3][0] = 0;        I[3][1] = 0;        I[3][2] = 0;        I[3][3] = 1;

    double rotated[4][4];

    multiplyMatrixWithMatrix(rotated, I, translated);

    m = rotated;

}


void makePerspectiveTransformation(Camera c, double m[4][4]) {

    //create an identity matrix
    double I[4][4];
    makeIdentityMatrix(I);


    I[0][0] = 2 * (c.n) / (c.r-c.l);    I[0][1] = 0;                        I[0][2] = (c.r + c.l) / (c.r - c.l);            I[0][3] = 0;
    I[1][0] = 0;                        I[1][1] = 2 * (c.n) / (c.t-c.b);    I[1][2] = (c.t + c.b) / (c.t - c.b);            I[1][3] = 0;
    I[2][0] = 0;                        I[2][1] = 0;                        I[2][2] = (-1) * (c.f + c.n) / (c.f - c.n);     I[2][3] = (-1) * (2*c.f*c.n) / (c.f - c.n);
    I[3][0] = 0;                        I[3][1] = 0;                        I[3][2] = -1;                                   I[3][3] = 0;


    double result[4][4];

    multiplyMatrixWithMatrix(result, I, m);

    m = result;



void forwardRenderingPipeline(Camera cam) {
    // TODO: IMPLEMENT HERE
}


int main(int argc, char **argv) {
    int i, j;

    if (argc < 2) {
        std::cout << "Usage: ./rasterizer <scene file> <camera file>" << std::endl;
        return 1;
    }

    // read camera and scene files
    readSceneFile(argv[1]);
    readCameraFile(argv[2]);

    image = 0;

    for (i = 0; i < numberOfCameras; i++) {

        // allocate memory for image
        if (image) {
			for (j = 0; j < cameras[i].sizeX; j++) {
		        delete image[j];
		    }

			delete[] image;
		}

        image = new Color*[cameras[i].sizeX];

        if (image == NULL) {
            std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
            exit(1);
        }

        for (j = 0; j < cameras[i].sizeX; j++) {
            image[j] = new Color[cameras[i].sizeY];
            if (image[j] == NULL) {
                std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
                exit(1);
            }
        }


        // initialize image with basic values
        initializeImage(cameras[i]);

        // do forward rendering pipeline operations
        forwardRenderingPipeline(cameras[i]);

        // generate PPM file
        writeImageToPPMFile(cameras[i]);

        // Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
        // Notice that os_type is not given as 1 (Ubuntu) or 2 (Windows), below call doesn't do conversion.
        // Change os_type to 1 or 2, after being sure that you have ImageMagick installed.
        convertPPMToPNG(cameras[i].outputFileName, 99);
    }

    return 0;

}

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "hw2_types.h"
#include "hw2_math_ops.h"
#include "hw2_file_ops.h"
#include <iostream>
#include <algorithm>
#include <vector>

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

// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// Structures that I defined

struct TransformedVertex {
	double t_points[4];	// transformed points = [x, y, z, w]
	int colorId;
};

struct TransformedTriangle {
	TransformedVertex t_vertices[3];
};

// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------

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

// It can multiply M = M * m2 or M = M * m1
void _multiplyMatrixWithMatrix(double r[4][4], double m1[4][4], double m2[4][4]) {
	int i, j, k;
	double total;
	double result[4][4];
	for(i = 0; i < 4; i++) {
		for(j = 0; j < 4; j++) {
			total = 0;
			for(k = 0; k < 4; k++) 
				total += m1[i][k] * m2[k][j];
			result[i][j] = total;
		}
	} 

	// At the end equalize result to r matrix:
	for(i = 0; i < 4; i++) {
		for(j = 0; j < 4; j++) {
			r[i][j] = result[i][j];
		}
	}
}

void _multiplyMatrixWithVec4d(double r[4], double m[4][4], double v[4]) {
	int i, j;
	double total;
	double result[4];
	for(i = 0; i < 4; i++) {
		total = 0;
		for(j = 0; j < 4; j++)
			total += m[i][j] * v[j];
		result[i] = total;
	}

	// At the end equalize result to r matrix
	for(i = 0; i < 4; i++)
		r[i] = result[i];
}

// Model Transformation for Translation
void ModelTransformationMatrix(Translation t, double matrix[4][4]) {
	matrix[0][0] = 1;    matrix[0][1] = 0;    matrix[0][2] = 0;    matrix[0][3] = t.tx;    		
	matrix[1][0] = 0;    matrix[1][1] = 1;    matrix[1][2] = 0;    matrix[1][3] = t.ty;
	matrix[2][0] = 0;    matrix[2][1] = 0;    matrix[2][2] = 1;    matrix[2][3] = t.tz;
	matrix[3][0] = 0;    matrix[3][1] = 0;    matrix[3][2] = 0;    matrix[3][3] = 1;
}

// Model Transformation for Rotation
void ModelTransformationMatrix(Rotation r, double matrix[4][4]) {
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
	u = normalizeVec3(u);

	uvals[0] = u.x;
	uvals[1] = u.y;
	uvals[2] = u.z;

	int smallestIdx = 0;
	int smallest    = abs(uvals[0]);
	for(int i = 1; i < 3; i++) {
		if (abs(uvals[i]) < smallest) {
			smallest = abs(uvals[i]);
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

	Rx_theta[0][0] = 1;               Rx_theta[0][1] = 0;               			 Rx_theta[0][2] = 0;               					Rx_theta[0][3] = 0;
	Rx_theta[1][0] = 0;               Rx_theta[1][1] = cos(r.angle*(M_PI / 180));    Rx_theta[1][2] = -1*sin(r.angle*(M_PI / 180)); 	Rx_theta[1][3] = 0;
	Rx_theta[2][0] = 0;               Rx_theta[2][1] = sin(r.angle*(M_PI / 180));    Rx_theta[2][2] = cos(r.angle*(M_PI / 180));    	Rx_theta[2][3] = 0;
	Rx_theta[3][0] = 0;               Rx_theta[3][1] = 0;               			 Rx_theta[3][2] = 0;               					Rx_theta[3][3] = 1;

	// Rotate = Rx_theta * M
	multiplyMatrixWithMatrix(matrix, Rx_theta, M);
}

// Model Transformation for Scaling
void ModelTransformationMatrix(Scaling s, double matrix[4][4]) {
	matrix[0][0] = s.sx;	matrix[0][1] = 0;    matrix[0][2] = 0;    matrix[0][3] = 0;
	matrix[1][0] = 0;		matrix[1][1] = s.sy; matrix[1][2] = 0;	  matrix[1][3] = 0;	
	matrix[2][0] = 0;		matrix[2][1] = 0;	 matrix[2][2] = s.sz; matrix[2][3] = 0;
	matrix[3][0] = 0; 		matrix[3][1] = 0;	 matrix[3][2] = 0;	  matrix[3][3] = 1;
}

void CameraTransformationMatrix(Camera c, double M_cam[4][4]) {
	M_cam[0][0] = c.u.x;	M_cam[0][1] = c.u.y;	M_cam[0][2] = c.u.z;	M_cam[0][3] = -((c.u.x * c.pos.x) + (c.u.y * c.pos.y) + (c.u.z * c.pos.z));
	M_cam[1][0] = c.v.x;	M_cam[1][1] = c.v.y;	M_cam[1][2] = c.v.z;	M_cam[1][3] = -((c.v.x * c.pos.x) + (c.v.y * c.pos.y) + (c.v.z * c.pos.z));
	M_cam[2][0] = c.w.x;	M_cam[2][1] = c.w.y;	M_cam[2][2] = c.w.z;	M_cam[2][3] = -((c.w.x * c.pos.x) + (c.w.y * c.pos.y) + (c.w.z * c.pos.z));
	M_cam[3][0] = 0;		M_cam[3][1] = 0;		M_cam[3][2] = 0;		M_cam[3][3] = 1;
}

void PerspectiveTransformationMatrix(Camera c, double M_per[4][4]) {
	M_per[0][0] = 2 * (c.n) / (c.r-c.l);    M_per[0][1] = 0;                        M_per[0][2] = (c.r + c.l) / (c.r - c.l);            M_per[0][3] = 0;
    M_per[1][0] = 0;                        M_per[1][1] = 2 * (c.n) / (c.t-c.b);    M_per[1][2] = (c.t + c.b) / (c.t - c.b);            M_per[1][3] = 0;
    M_per[2][0] = 0;                        M_per[2][1] = 0;                        M_per[2][2] = (-1) * (c.f + c.n) / (c.f - c.n);     M_per[2][3] = (-1) * (2*c.f*c.n) / (c.f - c.n);
    M_per[3][0] = 0;                        M_per[3][1] = 0;                        M_per[3][2] = -1;                                   M_per[3][3] = 0;
}

void ViewportTransformMatrix(Camera c, double M_vp[4][4]) {
	M_vp[0][0] = c.sizeX / 2; M_vp[0][1] = 0;               M_vp[0][2] = 0;       M_vp[0][3] = (c.sizeX-1) / 2;
    M_vp[1][0] = 0;           M_vp[1][1] = c.sizeY / 2;     M_vp[1][2] = 0;       M_vp[1][3] = (c.sizeY-1) / 2; 
    M_vp[2][0] = 0;           M_vp[2][1] = 0;               M_vp[2][2] = 1/2;     M_vp[2][3] = 1/2;
    M_vp[3][0] = 0;           M_vp[3][1] = 0;               M_vp[3][2] = 0;       M_vp[3][3] = 1;
}

void PerspectiveDivide(double v[4]) {
	v[0] /= v[3];
	v[1] /= v[3];
	v[2] /= v[3];
	v[3] /= v[3];
}

void EqualizeMatrix(double src[4][4], double dst[4][4]) {
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++) {
			dst[i][j] = src[i][j];
		} 
	}
}

// ##########################################################################################################################################
// ##########################################################################################################################################
// ##########################################################################################################################################

void forwardRenderingPipeline(Camera cam) {	 
    // TODO: IMPLEMENT HERE
    Model model;
    // Keep all transformed triangles
    std::vector<TransformedTriangle> transformedTriangles;
    for(int i = 0; i < numberOfModels; i++) {
    	model  = models[i];
    	double M_model[4][4];			// Define M_model (model transformation matrix)
    	double M_cam[4][4];				// Define M_cam (camera transformation matrix)
    	double M_per[4][4];				// Define M_per (perspective transformation matrix)
    	double M_vp[4][4];				// Define M_vp (viewport transformation matrix)
    	double M_total[4][4];			// Total transformations
    	makeIdentityMatrix(M_model);	// Define M_initally as identity matrix

    	for(int j = 0; j < model.numberOfTransformations; j++) {
    		double M[4][4];		// M_i: modelling transformations of each step
    		if(model.transformationTypes[j] == 't') {
    			Translation t = translations[model.transformationIDs[j]];
    			ModelTransformationMatrix(t, M);
    		}
    		else if(model.transformationTypes[j] == 'r') {
    			Rotation r = rotations[model.transformationIDs[j]];
    			ModelTransformationMatrix(r, M);
    		}
    		else if(model.transformationTypes[j] == 's') {
    			Scaling s = scalings[model.transformationIDs[j]];
    			ModelTransformationMatrix(s, M);
    		}
    		_multiplyMatrixWithMatrix(M_model, M, M_model);
    	}	
    	// Total Transformations is equal Model Transformations now
    	EqualizeMatrix(M_model, M_total);

    	// Camera transformations M_total = M_cam * M_total
    	CameraTransformationMatrix(cam, M_cam);
    	_multiplyMatrixWithMatrix(M_total, M_cam, M_total);

    	// Perspective transformations M_total = M_per * M_total
    	PerspectiveTransformationMatrix(cam, M_per);
    	_multiplyMatrixWithMatrix(M_total, M_per, M_total);

    	
    	for(int k = 0; k < model.numberOfTriangles; k++) {
    		Triangle triangle = model.triangles[k];		// Actual triangle without transformed
    		TransformedTriangle transformedTriangle;	// Transformed triangle with 3 points keep as t_vertices
    		for(int l = 0; l < 3; l++) {
    			double v[4];	// actual vertex point vector [x, y, z, 1]
    			v[0] = vertices[triangle.vertexIds[l]].x;
    			v[1] = vertices[triangle.vertexIds[l]].y;
    			v[2] = vertices[triangle.vertexIds[l]].z;
    			v[3] = 1;
    			//cout << "x: " << v[0] << ", y: " << v[1] << ", z: " << v[2] << ", w: " << v[3] << endl;

    			// Transform this vertex point to transformed points 
    			transformedTriangle.t_vertices[l].colorId = vertices[triangle.vertexIds[l]].colorId;
    			_multiplyMatrixWithVec4d(transformedTriangle.t_vertices[l].t_points, M_total, v);
    			PerspectiveDivide(transformedTriangle.t_vertices[l].t_points);	// Perform perspective divide to transformed points
    		}
    		transformedTriangles.push_back(transformedTriangle);
    	}

    	ViewportTransformMatrix(cam, M_vp);
    	// Perform viewport transformation against transformed vertex points
    	for(int k = 0; k < transformedTriangles.size(); k++) {
    		for(int l = 0; l < 3; l++) {
    			//cout << transformedTriangles[k].t_vertices[l].t_points[0] << " ";
    			_multiplyMatrixWithVec4d(transformedTriangles[k].t_vertices[l].t_points, M_vp, transformedTriangles[k].t_vertices[l].t_points);
    			//cout << transformedTriangles[k].t_vertices[l].t_points[0] << endl;
    		}
    	}
    }

    for(int i = 0; i < transformedTriangles.size(); i++) {
    	for(int j = 0; j < 3; j++) {
    		image[int(transformedTriangles[i].t_vertices[j].t_points[0])][int(transformedTriangles[i].t_vertices[j].t_points[1])].r = colors[transformedTriangles[i].t_vertices[j].colorId].r; 
    		image[int(transformedTriangles[i].t_vertices[j].t_points[0])][int(transformedTriangles[i].t_vertices[j].t_points[1])].g = colors[transformedTriangles[i].t_vertices[j].colorId].g;
    		image[int(transformedTriangles[i].t_vertices[j].t_points[0])][int(transformedTriangles[i].t_vertices[j].t_points[1])].b = colors[transformedTriangles[i].t_vertices[j].colorId].b;
    	}
    }	
    
    for(int i = 0; i < transformedTriangles.size(); i++) {
    	cout << "Triangle: " << i+1 << endl;
    	for(int j = 0; j < 3; j++) {
    		cout << "\t";
    		cout << "x: " << transformedTriangles[i].t_vertices[j].t_points[0];
    		cout << ", y: " << transformedTriangles[i].t_vertices[j].t_points[1]; 
    		cout << ", z: " << transformedTriangles[i].t_vertices[j].t_points[2];
    		cout << ", w: " << transformedTriangles[i].t_vertices[j].t_points[3];

    	}
    	cout << "\n";
    }
    cout << "****************************************************************************" << endl;
	
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

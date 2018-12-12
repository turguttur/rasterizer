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
#include <limits>

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

// It can multiply v = M * v
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
	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = 0;
	matrix[0][3] = t.tx;    		

	matrix[1][0] = 0;    
	matrix[1][1] = 1;    
	matrix[1][2] = 0;    
	matrix[1][3] = t.ty;

	matrix[2][0] = 0;    
	matrix[2][1] = 0;    
	matrix[2][2] = 1;    
	matrix[2][3] = t.tz;

	matrix[3][0] = 0;    
	matrix[3][1] = 0;    
	matrix[3][2] = 0;    
	matrix[3][3] = 1;
}

// Model Transformation for Rotation
void ModelTransformationMatrix(Rotation r, double matrix[4][4]) {
	double uvals[3];
	double vvals[3];
	Vec3 u;					// 
	Vec3 v;					//
	Vec3 w;					// 
	double M[4][4];			// Orthonormal matrix uvw: M
	double _M[4][4];		// Inverse of M
	double Rx_theta[4][4];	// Rotation around x axis

	u.x = r.ux;
	u.y = r.uy;
	u.z = r.uz;
	u = normalizeVec3(u);

	uvals[0] = u.x;
	uvals[1] = u.y;
	uvals[2] = u.z;

	int smallestIdx = 0;
	double smallest = abs(uvals[0]);
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

	_M[0][0] = u.x;    _M[0][1] = v.x;    _M[0][2] = w.x;    _M[0][3] = 0;	
	_M[1][0] = u.y;    _M[1][1] = v.y;    _M[1][2] = w.y;    _M[1][3] = 0;
	_M[2][0] = u.z;    _M[2][1] = v.z;    _M[2][2] = w.z;    _M[2][3] = 0;
	_M[3][0] = 0;      _M[3][1] = 0;      _M[3][2] = 0;      _M[3][3] = 1;	

	M[0][0] = u.x;    M[0][1] = u.y;    M[0][2] = u.z;    M[0][3] = 0;	
	M[1][0] = v.x;    M[1][1] = v.y;    M[1][2] = v.z;    M[1][3] = 0;
	M[2][0] = w.x;    M[2][1] = w.y;    M[2][2] = w.z;    M[2][3] = 0;
	M[3][0] = 0;      M[3][1] = 0;      M[3][2] = 0;      M[3][3] = 1;

	Rx_theta[0][0] = 1;               Rx_theta[0][1] = 0;               			 Rx_theta[0][2] = 0;               					Rx_theta[0][3] = 0;
	Rx_theta[1][0] = 0;               Rx_theta[1][1] = cos(r.angle*(M_PI / 180));    Rx_theta[1][2] = -sin(r.angle*(M_PI / 180)); 	    Rx_theta[1][3] = 0;
	Rx_theta[2][0] = 0;               Rx_theta[2][1] = sin(r.angle*(M_PI / 180));    Rx_theta[2][2] = cos(r.angle*(M_PI / 180));    	Rx_theta[2][3] = 0;
	Rx_theta[3][0] = 0;               Rx_theta[3][1] = 0;               			 Rx_theta[3][2] = 0;               					Rx_theta[3][3] = 1;

	// Rotate = (M' * Rx_theta * M)
	_multiplyMatrixWithMatrix(matrix, Rx_theta, M);
	_multiplyMatrixWithMatrix(matrix, _M, matrix);
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
	M_per[0][0] = (2.0d * c.n) / (c.r-c.l);    
	M_per[0][1] = 0;                        
	M_per[0][2] = (c.r + c.l) / (c.r - c.l);            
	M_per[0][3] = 0;

    M_per[1][0] = 0;                        
    M_per[1][1] = (2.0d * c.n) / (c.t-c.b);    
    M_per[1][2] = (c.t + c.b) / (c.t - c.b);            
    M_per[1][3] = 0;

    M_per[2][0] = 0;                        
    M_per[2][1] = 0;                        
    M_per[2][2] = -((c.f + c.n) / (c.f - c.n));     
    M_per[2][3] = -((2.0d * c.f * c.n) / (c.f - c.n));

    M_per[3][0] = 0;                        
    M_per[3][1] = 0;                        
    M_per[3][2] = -1.0d;                                   
    M_per[3][3] = 0;
}

void ViewportTransformMatrix(Camera c, double M_vp[4][4]) {
	M_vp[0][0] = double(c.sizeX) / 2.0d; 
	M_vp[0][1] = 0;               
	M_vp[0][2] = 0;       
	M_vp[0][3] = double(c.sizeX-1) / 2.0d;

    M_vp[1][0] = 0;           
    M_vp[1][1] = double(c.sizeY) / 2.0d;     
    M_vp[1][2] = 0;       
    M_vp[1][3] = double(c.sizeY-1) / 2.0d; 

    M_vp[2][0] = 0;           
    M_vp[2][1] = 0;               
    M_vp[2][2] = 1.0d/2.0d;     
    M_vp[2][3] = 1.0d/2.0d;

    M_vp[3][0] = 0;           
    M_vp[3][1] = 0;               
    M_vp[3][2] = 0;       
    M_vp[3][3] = 1;
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

// 2.0d yerine double() kullanulabilir


// f(x,y) = x(y0 - y1) + y(x1 - x0) + x0y1 - y0x1


void draw(int x, int y, Color c) {
	image[x][y].r = c.r;
	image[x][y].g = c.g;
	image[x][y].b = c.b;
}

void LineRasterization(double x0, double y0, double x1, double y1, Color c0, Color c1) {
	int x, y;
	bool flag = abs(y1 - y0) > abs(x1- x0);
	if(flag) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if(x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
		swap(c0, c1);
	}

	int dx = (int)x1 - (int)x0;
	int dy = abs((int)y1 - (int)y0);
	int d  = -dy + 0.5*(dx);

	Color c = c0;
	double dc_r = (c1.r - c0.r) / (x1 - x0);
	double dc_g = (c1.g - c0.g) / (x1 - x0);
	double dc_b = (c1.b - c0.b) / (x1 - x0);

	int ystep = (y0 < y1) ? 1 : -1;
	y = (int)y0;
	for(x = (int)x0; x <= (int)x1; x++) {
		if(flag) {
			draw(y, x, c);		
		}
		else {
			draw(x, y, c);
		}
		if(d < 0) {
			y += ystep;
			d += (-dy + dx);
		}
		else {
			d += (-dy);
		}
		c.r += dc_r;
		c.g += dc_g;
		c.b += dc_b;
	}
}


void TriangleRasterization(std::pair<double, double> vertex[3], Color color[3]) {
	// pairs first is x coordinate, and the second is y coordinate

	// We need to find bounding box first
	
	double minx = std::numeric_limits<double>::max();
	double miny = std::numeric_limits<double>::max();
	double maxx = std::numeric_limits<double>::min();
	double maxy = std::numeric_limits<double>::min();
	for(int i = 0; i < 3; i++) {
		if(vertex[i].first < minx)
			minx = vertex[i].first;
		if(vertex[i].second < miny) 
			miny = vertex[i].second;
		if(vertex[i].first > maxx) 
			maxx = vertex[i].first;
		if(vertex[i].second > maxy)
			maxy = vertex[i].second;
	}

	/*
	cout << "v0: (" << vertex[0].first << ", " << vertex[0].second << ")" << endl;
	cout << "v1: (" << vertex[1].first << ", " << vertex[1].second << ")" << endl;
	cout << "v2: (" << vertex[2].first << ", " << vertex[2].second << ")" << endl;
	cout << "minx: " << minx << " maxx: " << maxx << " miny: " << miny << " maxy: " << maxy << endl; 
	cout << "------------------------------------------------------------------------------------" << endl;
	
	double minx = min(vertex[0].first, vertex[1].first);
	minx = min(minx, vertex[2].first);
	double maxx = max(vertex[0].first, vertex[1].first);
	maxx = max(maxx, vertex[2].first);
	double miny = min(vertex[0].second, vertex[1].second);
	miny = min(miny, vertex[2].second);
	double maxy = max(vertex[0].second, vertex[1].second);
	maxy = max(maxy, vertex[2].second);
	*/
	//cout << "xmin: " << minx << " xmax: " << maxx << " ymin: " << miny << " ymax: " << maxy << endl;
 	// DOUBLE İNT FALAN DÜZELT

	double epsilon = 1e-10;
	int x0 = int(vertex[0].first);
	int y0 = int(vertex[0].second);
	int x1 = int(vertex[1].first);
	int y1 = int(vertex[1].second);
	int x2 = int(vertex[2].first);
	int y2 = int(vertex[2].second);
	Color c0 = color[0];
	Color c1 = color[1];
	Color c2 = color[2];

	double alpha, beta, gamma;
	Color c;
	for(int y = int(miny); y <= int(maxy); y++) {
		for(int x = int(minx); x <= int(maxx); x++) {
			//cout << "x: " << x << ", y: " << y << endl;
			alpha = double((x*(y1-y2))+(y*(x2-x1))+(x1*y2)-(y1*x2)) /
					double((x0*(y1-y2))+(y0*(x2-x1))+(x1*y2)-(y1*x2));
			beta  = double((x*(y2-y0))+(y*(x0-x2))+(x2*y0)-(y2*x0)) /
					double((x1*(y2-y0))+(y1*(x0-x2))+(x2*y0)-(y2*x0));
			gamma = double((x*(y0-y1))+(y*(x1-x0))+(x0*y1)-(y0*x1)) /
					double((x2*(y0-y1))+(y2*(x1-x0))+(x0*y1)-(y0*x1));
			//cout << "bary start:" << endl,
			//cout << alpha << " " <<  beta << " "  << gamma << endl;
			//cout << "bary end" << endl;
			if(alpha >= 0 && beta >= 0 && gamma >= 0) {
				//cout << alpha << " -- " << beta << " -- " << gamma << endl;
				c.r = (alpha * c0.r) + (beta * c1.r) + (gamma * c2.r);
				c.g = (alpha * c0.g) + (beta * c1.g) + (gamma * c2.g);
				c.b = (alpha * c0.b) + (beta * c1.b) + (gamma * c2.b);
				draw(x, y, c);
			}
		}
	}
	// f01 = x(y0 - y1) + y(x1 - x0) + x0y1 - y0x1
	// f12 = x(y1 - y2) + y(x2 - x1) + x1y2 - y1x2
	// f20 = x(y2 - y1) + y(x0 - x2) + x2y0 - y2x0
	// alpha = f12(x, y) / f12(x0, y0)
	// beta  = f20(x, y) / f20(x1, y1)
	// gamma = f01(x, y) / f01(x2, y2)

}

void forwardRenderingPipeline(Camera cam) {	 
    // TODO: IMPLEMENT HERE
    Model model;    
    for(int i = 0; i < numberOfModels; i++) {
    	model = models[i];
    	double M_model[4][4];			// Define M_model as model transformation matrix
    	double M_cam[4][4];				// Define M_cam as camera transformation matrix
    	double M_per[4][4]; 			// Define M_per as perspective transformation matrix
    	double M_vp[4][4];				// Define M_vp as viewport transformation matrix
    	double M_total[4][4]; 			// Define M_total to keep all transformations in single matrix
    	makeIdentityMatrix(M_model);	// Initially define M_model as identity matrix
    	
    	for(int j = 0; j < model.numberOfTransformations; j++) {
    		double M[4][4];			// Define M as model transformation in each step
    		if(model.transformationTypes[j] == 't') {
    			// Translation transform
    			Translation t = translations[model.transformationIDs[j]];
    			ModelTransformationMatrix(t, M);
    		}
    		else if(model.transformationTypes[j] == 'r') {
    			// Rotation transformation
    			Rotation r = rotations[model.transformationIDs[j]];
    			ModelTransformationMatrix(r, M);
    		}
    		else if(model.transformationTypes[j] == 's') {
    			// Scaling transformation
    			Scaling s = scalings[model.transformationIDs[j]];
    			ModelTransformationMatrix(s, M);
    		}
    		// To find out final modelling transformation
    		_multiplyMatrixWithMatrix(M_model, M, M_model);
    	}
    	// Equalize the model transformation to total transformation
    	EqualizeMatrix(M_model, M_total);

    	// Find camera transformation matrix and multiply it with total transformation until now
    	CameraTransformationMatrix(cam, M_cam);
    	_multiplyMatrixWithMatrix(M_total, M_cam, M_total);

    	// Find perspective transformation matrix and multiply it with total transformation until now
    	PerspectiveTransformationMatrix(cam, M_per);
    	_multiplyMatrixWithMatrix(M_total, M_per, M_total);

    	std::vector<TransformedTriangle> transformedTriangles;
    	if(backfaceCullingSetting) {
    		// Backface culling, apply it before perspective divide
    		ViewportTransformMatrix(cam, M_vp);
	    	for(int j = 0; j < model.numberOfTriangles; j++) {
	    		TransformedTriangle transformedTriangle;
				// Actual triangle itself;
				Triangle triangle = model.triangles[j];
				for(int k = 0; k < 3; k++) {
					double v[4];	// Actual vertex points [x, y, z, 1]
	    			v[0] = vertices[triangle.vertexIds[k]].x;
	    			v[1] = vertices[triangle.vertexIds[k]].y;
	    			v[2] = vertices[triangle.vertexIds[k]].z;
	    			v[3] = 1;

	    			// Transform actual vertex point to transformed point
	    			transformedTriangle.t_vertices[k].colorId = vertices[triangle.vertexIds[k]].colorId;
	    			_multiplyMatrixWithVec4d(transformedTriangle.t_vertices[k].t_points, M_total, v);
				}

				Vec3 v0, v1, v2, n, midPoint, v;
				v0.x = transformedTriangle.t_vertices[0].t_points[0];
				v0.y = transformedTriangle.t_vertices[0].t_points[1];
				v0.z = transformedTriangle.t_vertices[0].t_points[2];
				v1.x = transformedTriangle.t_vertices[1].t_points[0];
				v1.y = transformedTriangle.t_vertices[1].t_points[1];
				v1.z = transformedTriangle.t_vertices[1].t_points[2];
				v2.x = transformedTriangle.t_vertices[2].t_points[0];
				v2.y = transformedTriangle.t_vertices[2].t_points[1];
				v2.z = transformedTriangle.t_vertices[2].t_points[2];

				n = crossProductVec3(subtractVec3(v2, v0), subtractVec3(v1, v0)); 
	 			n = normalizeVec3(n);

	 			midPoint = multiplyVec3WithScalar(addVec3(addVec3(v0, v1), v2), (double)1/(double)3);
	 			v = subtractVec3(midPoint, cam.v);
	 			v = normalizeVec3(v);

	 			if(dotProductVec3(n, v) < 0) {
	 				for(int k = 0; k < 3; k++) {
	 					//cout << "lolo" << endl;
	 					//cout << transformedTriangle.t_vertices[k].t_points[4] << endl;
	 					PerspectiveDivide(transformedTriangle.t_vertices[k].t_points);
	 					_multiplyMatrixWithVec4d(transformedTriangle.t_vertices[k].t_points, M_vp, transformedTriangle.t_vertices[k].t_points);
	 				}
	 				transformedTriangles.push_back(transformedTriangle);
	 			}  
	    	}
    	}

    	else {
	    	// Find viewport transformation matrix and multiply it transformed and performed perspective divide points
	    	ViewportTransformMatrix(cam, M_vp);
	    	for(int j = 0; j < model.numberOfTriangles; j++) {
	    		Triangle triangle = model.triangles[j];		// Actual triangle without transforming
	    		TransformedTriangle transformedTriangle;	// Transfortmed triangle, keep vertex points in t_vertices
	    		for(int k = 0; k < 3; k++) {
	    			double v[4];	// Actual vertex points [x, y, z, 1]
	    			v[0] = vertices[triangle.vertexIds[k]].x;
	    			v[1] = vertices[triangle.vertexIds[k]].y;
	    			v[2] = vertices[triangle.vertexIds[k]].z;
	    			v[3] = 1;

	    			// Transform actual vertex point to transformed point
	    			transformedTriangle.t_vertices[k].colorId = vertices[triangle.vertexIds[k]].colorId;
	    			_multiplyMatrixWithVec4d(transformedTriangle.t_vertices[k].t_points, M_total, v);
	    			PerspectiveDivide(transformedTriangle.t_vertices[k].t_points);
	    			_multiplyMatrixWithVec4d(transformedTriangle.t_vertices[k].t_points, M_vp, transformedTriangle.t_vertices[k].t_points);
	    		}
	    		transformedTriangles.push_back(transformedTriangle);
	    	}
    	}

    	if(model.type == 0) {
    		for(int j = 0; j < transformedTriangles.size(); j++) {
    			for(int k = 0; k < 3; k++) {
    				double x0 = (transformedTriangles[j].t_vertices[k % 3].t_points[0]);
    				double y0 = (transformedTriangles[j].t_vertices[k % 3].t_points[1]);
    				double x1 = (transformedTriangles[j].t_vertices[(k+1) % 3].t_points[0]);
    				double y1 = (transformedTriangles[j].t_vertices[(k+1) % 3].t_points[1]);
    				Color c0  = colors[transformedTriangles[j].t_vertices[k % 3].colorId];
    				Color c1  = colors[transformedTriangles[j].t_vertices[(k+1) % 3].colorId]; 
    				LineRasterization(x0, y0, x1, y1, c0, c1);
    			}
    		}
    	}
    	// Else model type = 1: solid
    	else {
    		for(int j = 0; j < transformedTriangles.size(); j++) {
    			std::pair<double, double> triangle[3];
    			Color color[3];
    			for(int k = 0; k < 3; k++) {
    				triangle[k].first = transformedTriangles[j].t_vertices[k].t_points[0];	// x coordinate of the vertex
    				triangle[k].second = transformedTriangles[j].t_vertices[k].t_points[1];	// y coordinate of the vertex
    				color[k] = colors[transformedTriangles[j].t_vertices[k].colorId];		// color of the vertex
    			}
    			TriangleRasterization(triangle, color);
    		}
    	}
    }
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

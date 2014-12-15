
#include <GL/glut.h>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <string>

#define FREEGLUT_STATIC
//#define _LIB
//#define FREEGLUT_LIB_PRAGMAS = 0

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

const double pi = 3.14159265;
int toggle_raytrace = 0;
int view = 0;
int scale = (WINDOW_WIDTH + WINDOW_HEIGHT) / 200;
int sub_iteration = 1;
int antialias = 0;

struct Vector3
{
        double x;
        double y;
        double z;
		Vector3* n;
		
        bool is_odd;
        bool has_normal;
        Vector3() : x(0.0), y(0.0), z(0.0)
        {
        	is_odd = false;
        	has_normal = false;
        }

        Vector3(double x, double y, double z)
                : x(x), y(y), z(z)
        {
        	is_odd = false;
        	has_normal = false;
        }

        Vector3(const Vector3 & v)
                : x(v.x), y(v.y), z(v.z)
        {
        	is_odd = false;
        	has_normal = false;
        }

        Vector3 operator+(const Vector3 & rhs) const
        { return Vector3(x + rhs.x, y + rhs.y, z + rhs.z); }
        Vector3 operator-(const Vector3 & rhs) const
        { return Vector3(x - rhs.x, y - rhs.y, z - rhs.z); }
        Vector3 operator*(double rhs) const
        { return Vector3(x * rhs, y * rhs, z * rhs); }
        Vector3 operator/(double rhs) const
        { return Vector3(x / rhs, y / rhs, z / rhs); }
        Vector3 operator+=(const Vector3 & rhs)
        { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
        Vector3 operator-=(const Vector3 & rhs)
        { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
        Vector3 operator*=(double rhs)
        { x *= rhs; y *= rhs; z *= rhs; return *this; }
        Vector3 operator/=(double rhs)
        { x /= rhs; y /= rhs; z /= rhs; return *this; }

        double magnitude() const
        { return sqrt(x * x + y * y + z * z); }
        void normalize()
        { *this /= magnitude(); }
        double dot(const Vector3 & rhs) const
        {
                return x * rhs.x + y * rhs.y + z * rhs.z;
        }
        Vector3 cross(const Vector3 & rhs) const
        {
                return Vector3(y * rhs.z - z * rhs.y,
                                        z * rhs.x - x * rhs.z,
                                        x * rhs.y - y * rhs.x);
        }
        void print()
        {
        	printf("x: %f, y: %f, z: %f\n", x, y, z);
       	}
};


//stores shader information
struct Shaders
{
    double shiny;
    double ambient_ka;
    double diffuse_kd;
    
    Vector3 ambient;
    Vector3 diffuse;
    Vector3 specular;
    Vector3 specular_mat;
    
    Shaders(Vector3 ambient, Vector3 diffuse, Vector3 specular, Vector3 specular_mat,
            double shiny, double ambient_ka, double diffuse_kd)
        : ambient(ambient), diffuse(diffuse), specular(specular), specular_mat(specular_mat),
            shiny(shiny), ambient_ka(ambient_ka), diffuse_kd(diffuse_kd)
        {
        }
};

struct Edge
{
    Vector3* p1;
    Vector3* p2; 
};

struct Face
{
	Vector3* p1;
	Vector3* p2;
	Vector3* p3;
	
	//normal vector
	Vector3* n;
	
	//stores shader information for each face
	Shaders* s;
	
	double x_sum, y_sum, z_sum;
	
	Face(Vector3* p1, Vector3* p2, Vector3* p3)
	    : p1(p1), p2(p2), p3(p3)
	{
		x_sum = (p1->x + p2->x + p3->x) / 3.0;
		y_sum = (p1->y + p2->y + p3->y) / 3.0;
		z_sum = (p1->z + p2->z + p3->z) / 3.0;
	}
	
	Vector3* normal()
	{
		n = new Vector3;
		Vector3 u1 = *p2 - *p1;
		Vector3 v1 = *p3 - *p1;
		
		*n = u1.cross(v1);
		
		n->normalize();
		p1->n = n;
		
		return n;
	}
    Vector3 operator[](int i)
    {
        if(i == 0) return *p1;
        if(i == 1) return *p2;
        return *p3;
    }
};

struct Ray
{
    Vector3 pos;
    Vector3 dir;
    Ray()
    {
        pos = Vector3(0.0, 0.0, 0.0);
        dir = Vector3(0.0, 0.0, 1.0);
    }
    Ray(Vector3 new_pos, Vector3 new_dir)
    {
        pos = new_pos;
        dir = new_dir;
    }
};

//stores quaternions for rotation matrix
struct Quaternion
{
	double x, y, z, w;
    Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0)
    {
    }
    
    Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w)
    {
    }
    
	Quaternion operator* (Quaternion rhs)
	{
		double x2 = rhs.x;
		double y2 = rhs.y;
		double z2 = rhs.z;
		double w2 = rhs.w;
		
		return Quaternion(w*x2 + x*w2 + y*z2 - z*y2,
						  w*y2 - x*z2 + y*w2 + z*x2,
						  w*z2 + x*y2 - y*x2 + z*w2,
						  w*w2 - x*x2 - y*y2 - z*z2);
	}
};
//----------------------------------------------------------------------------------------------scene

std::vector<Vector3*> points;
std::vector<Vector3*> old_points;
std::vector<Vector3> vert_list;
std::vector<Vector3*> adj_list;
std::vector<Face> faces;
std::vector<Edge> edges;

std::vector< std::vector<Vector3> > fb(WINDOW_WIDTH, 
        std::vector<Vector3>(WINDOW_HEIGHT, Vector3(1.0,1.0,1.0)));
std::vector< std::vector<double> > window_t(WINDOW_WIDTH, 
        std::vector<double>(WINDOW_HEIGHT, 99999.0));
		
Vector3 obj_ambient(0.6, 0.2, 0.2);
Vector3 obj_diffuse(2.0, 0.0, 0.0);
Vector3 obj_specular(1.0, 1.0, 1.0);
Vector3 obj_specular_mat(1.0, 1.0, 1.0);
double obj_shiny = 50.0;
double obj_ambient_ka = 0.4;
double obj_diffuse_kd = 1.0;
Shaders obj(obj_ambient, obj_diffuse, obj_specular, obj_specular_mat,
            obj_shiny, obj_ambient_ka, obj_diffuse_kd);

Vector3 gnd_ambient(0.5, 0.5, 0.5);
Vector3 gnd_diffuse(0.5, 0.5, 0.5);
Vector3 gnd_specular(0.0, 0.0, 0.0);
Vector3 gnd_specular_mat(0.5, 0.5, 0.5);
double gnd_shiny = 200.0;
double gnd_ambient_ka = 0.2;
double gnd_diffuse_kd = 0.3;
Shaders gnd(gnd_ambient, gnd_diffuse, gnd_specular, gnd_specular_mat,
            gnd_shiny, gnd_ambient_ka, gnd_diffuse_kd);

const double depth = (double)WINDOW_WIDTH * 2.0;
Vector3 *gp1 = new Vector3(0.0, 0.0, depth);
Vector3 *gp2 = new Vector3((double)WINDOW_WIDTH, 0.0, depth); 
Vector3 *gp3 = new Vector3(0.0, (double)WINDOW_HEIGHT, depth); 
Vector3 *gp4 = new Vector3((double)WINDOW_WIDTH, (double)WINDOW_HEIGHT, depth);

Vector3 *gp5 = new Vector3(0.0, 0.0, 0.0);
Vector3 *gp6 = new Vector3((double)WINDOW_WIDTH, 0.0, 0.0); 
Vector3 *gp7 = new Vector3(0.0, (double)WINDOW_HEIGHT, 0.0); 
Vector3 *gp8 = new Vector3((double)WINDOW_WIDTH, (double)WINDOW_HEIGHT, 0.0);

//ground plane
Face gnd_f1(gp1, gp2, gp3);
Face gnd_f2(gp4, gp2, gp3);


//----------------------------------------------------------------------------------------------constants

void GL_render();
void GLInit(int* argc, char** argv);

//draws edges in the edges vector
void draw_object();

//creates edges based on the faces vector
void create_edges(std::vector<Face>& pfaces);

void DDA(double x_0, double y_0, double x_n, double y_n);
void read_file(std::string filename);
void renderPixel(double x, double y, double r = 1.0, double g = 1.0, double b = 1.0);
void Keyboard(unsigned char key, int x, int y);
void connect_faces();

//determinant of a 3x3 matrix
double det3(double (*M)[3])
{
    double a = M[0][0];
    double b = M[0][1];
    double c = M[0][2];

    double d = M[1][0];
    double e = M[1][1];
    double f = M[1][2];

    double g = M[2][0];
    double h = M[2][1];
    double i = M[2][2];

    double det = (a*e*i + b*f*g + c*d*h);
    det = det - a*f*h;
    det = det - b*d*i;
    det = det - c*e*g;
    
    return det;
}

//matrix-matrix multiplication for nxm vectors
std::vector< std::vector<double> > matrix_matrix_mult(
    std::vector< std::vector<double> > a, std::vector< std::vector<double> > b)
{
    if(a.size() <= 0 || b.size() <= 0 || a[0].size() != b.size())
    {
        printf("invalid matrix\n");
        return a;
    }
    
    std::vector< std::vector<double> > c(a[0].size(), std::vector<double>(b.size(), 0.0));
    
    for(unsigned int i = 0; i < c.size(); ++i) 
    {
        for (unsigned int j = 0; j < c[0].size(); ++j)
    	{
    		double sum = 0;
    		for(int k = 0; k < 4; ++k)
    		{
    	    	sum += a[i][k] * b[k][j];
    		}
    		c[i][j] = sum;
    	}
    }
    return c;
}

//rotate mesh with quaternions
void rotate(double angle, double x1, double y1, double z1, Vector3& p)
{
    int m = 4;
    int n = 4;
	std::vector< std::vector<double> > rot(m, std::vector<double>(n, 0.0));
    std::vector< std::vector<double> > new_p(m, std::vector<double>(1, 0.0));
    
    double fangle = (angle*pi)/180;
	Quaternion local(x1*sin(fangle/2), y1*sin(fangle/2), z1*sin(fangle/2), cos(fangle/2));
	Quaternion total(0,0,0,1);
    
    total = local * total;
    
    double x = total.x;
    double y = total.y;
    double z = total.z;
    double w = total.w;
    double relocate = 50.0;//((WINDOW_WIDTH + WINDOW_HEIGHT)/2) / 16;
    
    new_p[0][0] = p.x - relocate;
    new_p[1][0] = p.y - relocate;
    new_p[2][0] = p.z - relocate;
    new_p[3][0] = 1;

    rot[0][0] = (w*w)+(x*x)-(y*y)-(z*z);	rot[1][0] = 2*x*y - 2*w*z;				rot[2][0] = 2*x*z + 2*w*y;				rot[3][0] = 0.0;
	rot[0][1] = 2*x*y + 2*w*z;				rot[1][1] = (w*w)-(x*x)+(y*y)-(z*z);    rot[2][1] = 2*y*z + 2*w*x;    			rot[3][1] = 0.0;
	rot[0][2] = 2*x*z - 2*w*y;              rot[1][2] = 2*y*z - 2*w*x;     			rot[2][2] = (w*w)-(x*x)-(y*y)+(z*z);    rot[3][2] = 0.0;
	rot[0][3] = 0.0;               			rot[1][3] = 0.0;             			rot[2][3] = 0.0;             			rot[3][3] = 1.0;
	
	new_p = matrix_matrix_mult(rot, new_p);

	p.x = new_p[0][0] + relocate;
	p.y = new_p[1][0] + relocate;
	p.z = new_p[2][0] + relocate;
}

//transforms mesh
void transform(double dx, double dy, double dz, Vector3& p)
{
	p.x += dx;
	p.y += dy;
	p.z += dz;
}

//calculates intersection of ray. returns t.
double ray_intersect(const Vector3& r0, const Vector3& rd, const Vector3& a, const Vector3& b, const Vector3& c)
{
    //r = beta
    //s = gamma
    Vector3 eb = b-a;
    Vector3 ec = c-a;
    double t = 0.0;
    double r = 0.0;
    double s = 0.0;
    
    double mat_t[3][3];
    double mat_a[3][3];
    double mat_b[3][3];
    double mat_c[3][3];
    
    mat_t[0][0] = -eb.x; mat_t[1][0] =  -eb.y; mat_t[2][0] = -eb.z;
    mat_t[0][1] = -ec.x; mat_t[1][1] =  -ec.y; mat_t[2][1] = -ec.z;
    mat_t[0][2] = a.x-r0.x; mat_t[1][2] = a.y-r0.y; mat_t[2][2] = a.z-r0.z;

    mat_a[0][0] = -eb.x; mat_a[1][0] =  -eb.y; mat_a[2][0] = -eb.z;
    mat_a[0][1] = -ec.x; mat_a[1][1] =  -ec.y; mat_a[2][1] = -ec.z;
    mat_a[0][2] = rd.x; mat_a[1][2]  = rd.y; mat_a[2][2]   = rd.z;
 
    mat_b[0][0] = a.x-r0.x; mat_b[1][0] =  a.y-r0.y; mat_b[2][0] = a.z-r0.z;
    mat_b[0][1] = -ec.x; mat_b[1][1] =  -ec.y; mat_b[2][1] = -ec.z;
    mat_b[0][2] = rd.x; mat_b[1][2] = rd.y; mat_b[2][2] = rd.z;

    mat_c[0][0] = -eb.x; mat_c[1][0] =  -eb.y; mat_c[2][0] = -eb.z;
    mat_c[0][1] = a.x-r0.x; mat_c[1][1] = a.y-r0.y; mat_c[2][1] = a.z-r0.z;
    mat_c[0][2] = rd.x; mat_c[1][2] = rd.y; mat_c[2][2] = rd.z;
    
    t = det3(mat_t)/det3(mat_a);
    r = det3(mat_b)/det3(mat_a);
    s = det3(mat_c)/det3(mat_a);
    
    if(r > 1 || s > 1 || r+s > 1) return -1.0;
    
    if(t > 0)
        if(r > 0 && s > 0)
            if(r + s < 1)
                return t;
    
    return -1.0;
}

//calculates normals for each face
void calculate_normals()
{
    Vector3 centroid;
    for (unsigned int i = 0; i < faces.size(); ++i)
        centroid += Vector3(faces[i].x_sum, faces[i].y_sum, faces[i].z_sum);
    centroid /= faces.size();
    
    std::vector<Vector3> temp_normals;
    double sum = 0.0;
    for (unsigned int i = 0; i < faces.size(); ++i)
    {
    	Vector3* n = faces[i].normal();
    	if(n->dot(centroid) < 0.0)
    	    *n *= -1.0;
    	temp_normals.push_back(*n);
    }

   	//smooths normals by averaging them
    for (unsigned int i = 0; i < faces.size(); ++i)
    {
        for (unsigned int j = 0; j < temp_normals.size(); ++j)
        {
            faces[i].n->x += temp_normals[j].x;
            faces[i].n->y += temp_normals[j].y;
            faces[i].n->z += temp_normals[j].z;
        }
        faces[i].n->normalize();
    }
    
}

//calculates shaders for raytracing.
//sets color value to sum of ambient component + diffuse component + specular component
void ray_fill(Ray& r0, Ray& r1, Vector3& color,int x, int y, int max_x, int max_y, const Vector3& light)
{
    for (unsigned int i = 0; i < faces.size(); ++i)
    {
        Vector3 p1 = *faces[i].p1*scale;
        Vector3 p2 = *faces[i].p2*scale;
        Vector3 p3 = *faces[i].p3*scale;
        
        double t = ray_intersect(r0.pos, r0.dir, p1, p2, p3);
        double t2 = ray_intersect(r1.pos, r1.dir, p1, p2, p3);
        if(t > 0.0 && t < window_t[x][y])
        {
            window_t[x][y] = t;
            
            Vector3 pt = *faces[i].n;
			Vector3 new_light = (light * -1.0) - pt;
			new_light.normalize();
			double diff_dot = cos(faces[i].n->dot(new_light))-(t*0.001);
            if(diff_dot < 0.0) diff_dot = 0.0;
            else if(diff_dot > 1.0) diff_dot = 1.0;
            
            Vector3 sc(0.1, 0.1, 0.1);
            if(faces[i].s->specular.x > 0.01 || faces[i].s->specular.y > 0.01
                || faces[i].s->specular.z > 0.01)
            {
                Vector3 spec_mat = faces[i].s->diffuse *
                    faces[i].s->specular_mat.dot(faces[i].s->specular);
                    
                spec_mat.normalize(); 

                Vector3 v(max_x/2.0, max_y/2.0, -1.0);
                v.normalize();
                
                Vector3 h = light - pt;
                v = v - pt;
                h += v;
                h = h/2.0;
                h.normalize();
                
                double spec_dot = cos(h.dot(*faces[i].n));
                if(spec_dot > 1.0) spec_dot = 1.0;
                else if(spec_dot < 0.1) spec_dot = 0.0;
                
                double spec_power = pow(spec_dot, faces[i].s->shiny);
                sc = spec_mat*spec_power;
                //sc.normalize();
            }
            
            Vector3 ac = faces[i].s->ambient * faces[i].s->ambient_ka;
            Vector3 dc = faces[i].s->diffuse * faces[i].s->diffuse_kd * diff_dot;
            
            color = ac+dc+sc;
        }
        
        else if(t2 > 0.0 && t2 < window_t[x][y])
        {
            double shadow_val = 0.6;
            fb[x][y] = Vector3(shadow_val, shadow_val, shadow_val);
        }
    }
}

//main functon for raytrace.
void raytrace()
{ 
    Vector3 light((double)WINDOW_WIDTH, (double)WINDOW_HEIGHT, -20.0);
    light.normalize();
    
    fb = std::vector< std::vector<Vector3> >(WINDOW_WIDTH, std::vector<Vector3>(WINDOW_HEIGHT, Vector3(1.0,1.0,1.0)));
    window_t = std::vector< std::vector<double> >(WINDOW_WIDTH, 
                std::vector<double>(WINDOW_HEIGHT, 99999.0));
                
    //xy view
   	if(view == 0)
   	{
    	for(int x = 0; x < WINDOW_WIDTH; ++x)
    	{
        	for(int y = 0; y < WINDOW_HEIGHT; ++y)
        	{
            	window_t[x][y] = 999999.0;
            	fb[x][y] = Vector3(1.0, 1.0, 1.0);
            }
        }
   		
   		//shading
    	for(int x = 0; x < WINDOW_WIDTH; ++x)
    	{
        	for(int y = 0; y < WINDOW_HEIGHT; ++y)
        	{
            	Vector3 rpos(x, y, 0.0);
            	Vector3 rdir(0.0, 0.0, 1.0);
            	Ray r0(rpos, rdir);
                
                Vector3 shadow_rpos = rpos;
                shadow_rpos.x = WINDOW_HEIGHT - y;
                shadow_rpos.y = x;
                Vector3 shadow_rdir = light;
            	shadow_rdir.z = 1.0;
                Ray r1(shadow_rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, x, y, WINDOW_WIDTH, WINDOW_HEIGHT, light);

            	fb[x][y].x *= color.x;
            	fb[x][y].y *= color.y;
            	fb[x][y].z *= color.z;
                
                renderPixel(x, y, fb[x][y].x, fb[x][y].y, fb[x][y].z);
       		}
   		}
   	}
   	
   	//yz view
   	else if(view == 1)
   	{
    	for(int y = 0; y < WINDOW_WIDTH; ++y)
    	{
        	for(int z = 0; z < WINDOW_HEIGHT; ++z)
        	{
            	window_t[y][z] = 999999.0;
            	fb[y][z] = Vector3(1.0, 1.0, 1.0);
            }
        }	
    	for(int y = 0; y < WINDOW_WIDTH; ++y)
    	{
        	for(int z = 0; z < WINDOW_HEIGHT; ++z)
        	{
            	Vector3 rpos(0.0, y, z);
            	Vector3 rdir(1.0, 0.0, 0.0);
            	Ray r0(rpos, rdir);
            
            	Vector3 shadow_rdir = light;
            	shadow_rdir.x = -1.0;
                Ray r1(rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, y, z, WINDOW_WIDTH, WINDOW_HEIGHT, light);
            	fb[y][z].x *= color.x;
            	fb[y][z].y *= color.y;
            	fb[y][z].z *= color.z;
                
                renderPixel(y, z, fb[y][z].x, fb[y][z].y, fb[y][z].z);
       		}
   		}
   	}
   	
   	//xz view
   	else if(view == 2)
   	{
    	for(int x = 0; x < WINDOW_WIDTH; ++x)
    	{
        	for(int z = 0; z < WINDOW_HEIGHT; ++z)
        	{
            	window_t[x][z] = 999999.0;
            	fb[x][z] = Vector3(1.0, 1.0, 1.0);
            }
        }    	
    	for(int x = 0; x < WINDOW_WIDTH; ++x)
    	{
        	for(int z = 0; z < WINDOW_HEIGHT; ++z)
        	{
            	Vector3 rpos(x, 0.0, z);
            	Vector3 rdir(0.0, 1.0, 0.0);
            	Ray r0(rpos, rdir);
            
            	Vector3 shadow_rdir = light;
            	shadow_rdir.y = -1.0;
                Ray r1(rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, x, z, WINDOW_WIDTH, WINDOW_HEIGHT, light);
            	fb[x][z].x *= color.x;
            	fb[x][z].y *= color.y;
            	fb[x][z].z *= color.z;
                
                renderPixel(x, z, fb[x][z].x, fb[x][z].y, fb[x][z].z);
       		}
   		}
   	}
   	
}

void raytrace_aa()
{ 
    printf("antialias\n");
    int NEW_WINDOW_WIDTH = WINDOW_WIDTH * 2;
    int NEW_WINDOW_HEIGHT = WINDOW_HEIGHT * 2;
    
    fb = std::vector< std::vector<Vector3> >(NEW_WINDOW_WIDTH, std::vector<Vector3>(NEW_WINDOW_HEIGHT, Vector3(1.0,1.0,1.0)));
    window_t = std::vector< std::vector<double> >(NEW_WINDOW_WIDTH, 
                std::vector<double>(NEW_WINDOW_HEIGHT, 99999.0));
    
    std::vector< std::vector<Vector3> > new_fb(WINDOW_WIDTH, std::vector<Vector3>(WINDOW_HEIGHT, Vector3(1.0,1.0,1.0)));
    
    Vector3 light((double)NEW_WINDOW_WIDTH, (double)NEW_WINDOW_HEIGHT, -20.0);
    light.normalize();
    scale = scale * 2;
    //xy view
   	if(view == 0)
   	{
    	for(int x = 0; x < NEW_WINDOW_WIDTH; ++x)
    	{
        	for(int y = 0; y < NEW_WINDOW_HEIGHT; ++y)
        	{
            	window_t[x][y] = 999999.0;
            	fb[x][y] = Vector3(1.0, 1.0, 1.0);
            }
        }
   		
   		//shading
    	for(int x = 0; x < NEW_WINDOW_WIDTH; ++x)
    	{
        	for(int y = 0; y < NEW_WINDOW_HEIGHT; ++y)
        	{
            	Vector3 rpos(x, y, 0.0);
            	Vector3 rdir(0.0, 0.0, 1.0);
            	Ray r0(rpos, rdir);
                
                Vector3 shadow_rpos = rpos;
                shadow_rpos.x = NEW_WINDOW_HEIGHT - y;
                shadow_rpos.y = x;
                Vector3 shadow_rdir = light;
            	shadow_rdir.z = 1.0;
                Ray r1(shadow_rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, x, y, NEW_WINDOW_WIDTH, NEW_WINDOW_HEIGHT, light);

            	fb[x][y].x *= color.x;
            	fb[x][y].y *= color.y;
            	fb[x][y].z *= color.z;
                
                //renderPixel(x/2, y/2, fb[x][y].x, fb[x][y].y, fb[x][y].z);
       		}
   		}
   	}
   	
   	//yz view
   	else if(view == 1)
   	{
    	for(int y = 0; y < NEW_WINDOW_WIDTH; ++y)
    	{
        	for(int z = 0; z < NEW_WINDOW_HEIGHT; ++z)
        	{
            	window_t[y][z] = 999999.0;
            	fb[y][z] = Vector3(1.0, 1.0, 1.0);
            }
        }	
    	for(int y = 0; y < NEW_WINDOW_WIDTH; ++y)
    	{
        	for(int z = 0; z < NEW_WINDOW_HEIGHT; ++z)
        	{
            	Vector3 rpos(0.0, y, z);
            	Vector3 rdir(1.0, 0.0, 0.0);
            	Ray r0(rpos, rdir);
            
            	Vector3 shadow_rdir = light;
            	shadow_rdir.x = -1.0;
                Ray r1(rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, y, z, NEW_WINDOW_WIDTH, NEW_WINDOW_HEIGHT, light);
            	fb[y][z].x *= color.x;
            	fb[y][z].y *= color.y;
            	fb[y][z].z *= color.z;
                
                //renderPixel(y, z, fb[y][z].x, fb[y][z].y, fb[y][z].z);
       		}
   		}
   	}
   	
   	//xz view
   	else if(view == 2)
   	{
    	for(int x = 0; x < NEW_WINDOW_WIDTH; ++x)
    	{
        	for(int z = 0; z < NEW_WINDOW_HEIGHT; ++z)
        	{
            	window_t[x][z] = 999999.0;
            	fb[x][z] = Vector3(1.0, 1.0, 1.0);
            }
        }    	
    	for(int x = 0; x < NEW_WINDOW_WIDTH; ++x)
    	{
        	for(int z = 0; z < NEW_WINDOW_HEIGHT; ++z)
        	{
            	Vector3 rpos(x, 0.0, z);
            	Vector3 rdir(0.0, 1.0, 0.0);
            	Ray r0(rpos, rdir);
            
            	Vector3 shadow_rdir = light;
            	shadow_rdir.y = -1.0;
                Ray r1(rpos, shadow_rdir);
                
            	Vector3 color(0.0, 0.0, 0.0);
            	ray_fill(r0, r1, color, x, z, NEW_WINDOW_WIDTH, NEW_WINDOW_HEIGHT, light);
            	fb[x][z].x *= color.x;
            	fb[x][z].y *= color.y;
            	fb[x][z].z *= color.z;
                
                //renderPixel(x, z, fb[x][z].x, fb[x][z].y, fb[x][z].z);
       		}
   		}
   	}

	for(int x = 0; x < WINDOW_WIDTH-1; ++x)
	{
    	for(int y = 0; y < WINDOW_HEIGHT-1; ++y)
    	{
    	    Vector3 color1 = fb[x*2][y*2];
    	    Vector3 color2 = fb[x*2+1][y*2];
    	    Vector3 color3 = fb[x*2][y*2+1];
    	    Vector3 color4 = fb[x*2+1][y*2+1];
    	    new_fb[x][y] = color1 + color2 + color3 + color4;
    	    new_fb[x][y] /= 4.0;
    	    renderPixel(x, y, new_fb[x][y].x, new_fb[x][y].y, new_fb[x][y].z);
        }
    }
    scale = scale / 2;
}

int main(int argc, char** argv)
{	
    if(argc >= 2)
    {
        std::string filename = argv[1];
        read_file(filename);
    }
    else
    {
        printf("Enter a filename.\n");
        return 0;
    }
    
	GLInit(&argc, argv);
    glutDisplayFunc(GL_render);
    glutKeyboardFunc(Keyboard);
	glutMainLoop();
	return 0;
}

void create_edges(std::vector<Face>& pfaces)
{
    Vector3* p1;
    Vector3* p2;
    Vector3* p3;
    Edge* e1;
    Edge* e2;
    Edge* e3;
    std::vector<Edge> new_edges;
    
    //creates 3 edges for each face
    for (unsigned int k = 0; k < pfaces.size(); ++k)
    {
        p1 = pfaces[k].p1;
        p2 = pfaces[k].p2;
        p3 = pfaces[k].p3;
        
        e1 = new Edge;
        e2 = new Edge;
        e3 = new Edge;
        
        e1->p1 = p1;
        e1->p2 = p2;

        e2->p1 = p2;
        e2->p2 = p3;

        e3->p1 = p1;
        e3->p2 = p3;
		
        new_edges.push_back(*e1);
        new_edges.push_back(*e2);
        new_edges.push_back(*e3);
    }
    edges = new_edges;
}

void read_file(std::string filename)
{
    std::ifstream file;
    file.open(filename.c_str());
    if(!file.good())
    {
        printf("Can't find file.\n");   
        exit(0);
    }

    std::vector<int> v;
    std::string reader;
    while(file >> reader)
    {
        int n = atoi(reader.c_str());
        v.push_back(n);
    }
    
    int i = 2;
    int j = 0;
    int num_verts = v[0];
    int num_faces = v[1];
    
    i = 2;
    for(int j = 0; j < num_verts; ++j)
    {
        Vector3 *temp_point = new Vector3((double)v[i], (double)v[i+1], (double)v[i+2]);
        points.push_back(temp_point);
        i = i + 3;
    }
    for(int j = 0; j < num_faces; ++j)
    {
        Vector3 temp_point = Vector3((double)v[i], (double)v[i+1], (double)v[i+2]);
        vert_list.push_back(temp_point);
        i = i + 3;
    }
    
    connect_faces();
    create_edges(faces);
}

void connect_faces()
{
    std::vector<Face> new_faces;
    for(int j = 0; j < vert_list.size(); ++j)
    {
    	Face f_temp(points[(int)vert_list[j].x], points[(int)vert_list[j].y], points[(int)vert_list[j].z]);
    	f_temp.s = &obj;
    	new_faces.push_back(f_temp);
    }
    gnd_f1.s = &gnd;
    gnd_f2.s = &gnd;
    //gnd_f3.s = &gnd;
    //gnd_f4.s = &gnd;
    new_faces.push_back(gnd_f1);
    new_faces.push_back(gnd_f2);
    //new_faces.push_back(gnd_f3);
    //new_faces.push_back(gnd_f4);
    
    faces = new_faces;
    calculate_normals();
}

void draw_object()
{
	if(view == 0)
	{
    	for(int i = 0; i < edges.size(); ++i)
    	{
			Edge e1 = edges[i];
			DDA(scale*e1.p1->x, scale*e1.p1->y, scale*e1.p2->x, scale*e1.p2->y);
    	}
    }
	else if(view == 1)
	{
    	for(int i = 0; i < edges.size(); ++i)
    	{
			Edge e1 = edges[i];
			DDA(scale*e1.p1->y, scale*e1.p1->z, scale*e1.p2->y, scale*e1.p2->z);
    	}
    }
	else if(view == 2)
	{
    	for(int i = 0; i < edges.size(); ++i)
    	{
			Edge e1 = edges[i];
			DDA(scale*e1.p1->x, scale*e1.p1->z, scale*e1.p2->x, scale*e1.p2->z);
    	}
    }
}

void DDA(double x_0, double y_0, double x_n, double y_n)
{
    double d_x = x_n - x_0;
    double d_y = y_n - y_0;
    
    int steps = 0;
    double x = x_0;
    double y = y_0;
    
    if(fabs(d_x) > fabs(d_y))
        steps = abs((int)d_x);
    else
        steps = abs((int)d_y);
    
    double x_i = (double) d_x / (double)steps;
    double y_i = (double) d_y / (double)steps;
    
    for(int i = 0; i < steps; ++i)
    {
        renderPixel(x, y);
        x = x + x_i;
        y = y + y_i;
    }
}

void renderPixel(double x, double y, double r, double g, double b)
{
    glColor3f(r, g, b);
	glBegin(GL_POINTS);
	glVertex2f(x, y);
	glEnd();
}

//keyboard state machine. controls mesh and calls appropriate functions
enum keyboard_sm {wait, rotation, translation, subdivision, render} keyboard_state;
void Keyboard(unsigned char key, int x, int y)
{
    switch(keyboard_state)
    {
        case wait:						
            if(key == 'r') { printf("rotate\n"); keyboard_state = rotation; }
            else if(key == 't') { printf("transform\n"); keyboard_state = translation; }
            else if(key == 'l') { keyboard_state = subdivision; }
            else if(key == 'i') { keyboard_state = render; }
            else keyboard_state = wait;
            break;
            
        case rotation:						//rotation
            if(key == 'w' || key == 's' || key == 'a' || key == 'd') keyboard_state = rotation;
            else { printf("end rotate\n"); keyboard_state = wait; }
            break;		
            				
        case translation:						//translate
            if(key == 'w' || key == 's' || key == 'a' || key == 'd') keyboard_state = translation;
            else { printf("end transform\n"); keyboard_state = wait; }
            break;
            
        case subdivision:						//subdivide
            keyboard_state = wait;
            break;
            
        case render:						//render
            keyboard_state = wait;
            break;
            
		default:
			keyboard_state = wait;
            break;
    }
    
    switch(keyboard_state)
    {
        case wait:
        	if(key == '1')
        	{
        		view = 0;
        		draw_object();
        		glutPostRedisplay();
        	}
        	else if(key == '2')
        	{
        		view = 1;
        		draw_object();
        		glutPostRedisplay();
        	}
        	else if(key == '3')
        	{
        		view = 2;
        		draw_object();
        		glutPostRedisplay();
        	}
        	else if(key == '4')
        	{
        	    antialias = (antialias + 1)%2;
        	    printf("antialiasing: %d\n", antialias);
        	}
            break;
            
        case rotation:						//rotation
            if(view == 0)
            {
                if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 1.0, 0.0, 0.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, -1.0, 0.0, 0.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, -1.0, 0.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 1.0, 0.0, *points[i]);
               	}
           	}
           	else if(view == 1)
           	{
               	if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 1.0, 0.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, -1.0, 0.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 0.0, -1.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 0.0, 1.0, *points[i]);
               	}
            }
            else if(view == 2)
            {
                if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 1.0, 0.0, 0.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, -1.0, 0.0, 0.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 0.0, -1.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		rotate(2.0, 0.0, 0.0, 1.0, *points[i]);
               	}
            }
           	connect_faces();
           	create_edges(faces);
           	draw_object();
           	glutPostRedisplay();
            break;	
            					
        case translation:						//transform
            if(view == 0)
            {
                if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 1.0, 0.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, -1.0, 0.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(-1.0, 0.0, 0.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(1.0, 0.0, 0.0, *points[i]);
               	}
           	}
           	else if(view == 1)
           	{
               	if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 0.0, 1.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 0.0, -1.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, -1.0, 0.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 1.0, 0.0, *points[i]);
               	}
            }
            else if(view == 2)
            {
                if(key == 'w')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 0.0, 1.0, *points[i]);
               	}
                else if(key == 's')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(0.0, 0.0, -1.0, *points[i]);
               	} 
                else if(key == 'a')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(-1.0, 0.0, 0.0, *points[i]);
               	}
                else if(key == 'd')
                {
                	for(int i = 0; i < points.size(); ++i)
                		transform(1.0, 0.0, 0.0, *points[i]);
               	}
            }
           	connect_faces();
           	create_edges(faces);
           	draw_object();
           	glutPostRedisplay();
           	break;
            
        case render:							//render
        	if(toggle_raytrace % 2 == 0) printf("rendering\n");
        	else printf("switching to wireframe\n");
        	toggle_raytrace = ++toggle_raytrace % 2;
        	connect_faces();
           	create_edges(faces);
           	//raytrace();
        	glutPostRedisplay();
            break;
            
		default:
            break;
    }
}

void GL_render()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glClear(GL_COLOR_BUFFER_BIT);
    if(toggle_raytrace >= 1 && antialias <= 0)
    {
    	raytrace();
    	//printf("done. May take a while to display\n");
   	}
    else if(toggle_raytrace >= 1 && antialias >= 1)
    {
    	raytrace_aa();
    	//printf("done. May take a while to display\n");
   	}
    else draw_object();
    glutSwapBuffers();
}

void GLInit(int* argc, char** argv)
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

	glutCreateWindow("Raytracer");
	glMatrixMode(GL_PROJECTION_MATRIX);
	glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1);
}


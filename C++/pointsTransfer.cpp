#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <CGAL/Point_3.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>

#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <CGAL/bounding_box.h>

#include <CGAL/Timer.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Memory_sizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "Point.h"  // Defines type Point, Construct_coord_iterator
#include "Distance.h"

#include <utility>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

typedef CGAL::Dimension_tag<3>                                                          D;
typedef CGAL::Search_traits<double, Point, const double*, Construct_coord_iterator, D>  Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits, Distance>                            K_neighbor_search;
typedef K_neighbor_search::Tree                                                         Tree;

typedef CGAL::Exact_predicates_exact_constructions_kernel   Kernel;
typedef Kernel::FT                                          Scalar;

typedef CGAL::Point_2<Kernel>                                           Point_2;
typedef CGAL::Point_3<Kernel>                                           Point_3;
typedef CGAL::Triangle_3<Kernel>                                        Triangle_3;
typedef CGAL::Plane_3<Kernel>                                           Plane_3;
typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<Kernel>   Triangle_coordinates;

typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel>       Vertex_base;
typedef CGAL::Triangulation_data_structure_2<Vertex_base>                       Triangulation_data_structure;
typedef CGAL::Delaunay_triangulation_2<Kernel, Triangulation_data_structure>    Delaunay;
typedef Delaunay::Finite_vertices_iterator                                      Finite_vertices_iterator;
typedef Delaunay::Finite_faces_iterator                                         Finite_faces_iterator;

typedef cv::Mat     Mat;
typedef cv::Vec4b   Vec4b;

Point_3 point_to_point_3(Point &p)
{
    return Point_3 (p.x(), p.y(), p.z());
}

void draw_triangle(Point triangle[], int resolution, Mat *texture)
{
    Point_2 img_coords[3];
    for(int i = 0; i < 3; i++)
    {
        img_coords[i] = Point_2(triangle[i].u() * resolution, triangle[i].v() * resolution);
    }
    
    // Compute axis-aligned bounding box
    Kernel::Iso_rectangle_2 bb = CGAL::bounding_box(std::begin(img_coords), std::end(img_coords));
    
    // Triangle Rasterization
    // For each pixel in bounding box
    for(int i = (int)std::floor(CGAL::to_double(bb.xmin())); i <= std::floor(CGAL::to_double(bb.xmax())); i++)
    {
        for(int j = (int)std::floor(CGAL::to_double(bb.ymin())); j <= std::floor(CGAL::to_double(bb.ymax())); j++)
        {
            int x = i;
            int y = j;
            // Boundary check
            if(x >= resolution){ x = resolution - 1; }
            if(y >= resolution){ y = resolution - 1; }
            
            // Compute barycentric coordiates
            Triangle_coordinates triangle_coordinates(img_coords[0], img_coords[1], img_coords[2]);
            std::vector<Scalar> bc;
            triangle_coordinates(Point_2(x,y), bc);
            
            // If in triangle
            if(bc[0] >= 0 && bc[1] >= 0 && bc[2] >= 0)
            {
                // Compute pixel color
                float r = CGAL::to_double(bc[0]) * triangle[0].r() + CGAL::to_double(bc[1]) * triangle[1].r() + CGAL::to_double(bc[2]) * triangle[2].r();
                float g = CGAL::to_double(bc[0]) * triangle[0].g() + CGAL::to_double(bc[1]) * triangle[1].g() + CGAL::to_double(bc[2]) * triangle[2].g();
                float b = CGAL::to_double(bc[0]) * triangle[0].b() + CGAL::to_double(bc[1]) * triangle[1].b() + CGAL::to_double(bc[2]) * triangle[2].b();

                // Set Pixel
                texture->at<Vec4b>(j,i)[0] = b;
                texture->at<Vec4b>(j,i)[1] = g;
                texture->at<Vec4b>(j,i)[2] = r;
                texture->at<Vec4b>(j,i)[3] = 255;
            }
        }
    }
}

int main(int argc, char** argv){
    
    // Parse filenames from argument
    std::string usage_str = "Usage: ./pointTransfer <input-point-cloud> <input-mesh>";
    std::string pc_file_name = "";
    std::string mesh_file_name = "";
    
    if(argc < 3)
    {
        std::cout << usage_str << std::endl;
        return 0;
    }
    else
    {
        pc_file_name = argv[1];
        mesh_file_name = argv[2];
    }
    
    const int N = 1000;
    const unsigned int K = 20; // Search range
    const int RESOLUTION = 8192;

    CGAL::Timer task_timer; task_timer.start();
    CGAL::Real_timer real_total_timer; real_total_timer.start();

    // Read original point set
    std::ifstream pc_file_stream(pc_file_name, std::ios_base::in); // Open point cloud file

	if(!pc_file_stream.is_open())
	{ 
		std::cerr << "Cannot read or find point cloud file: " << pc_file_name << std::endl;
		return 0;
    }
    
	std::string str;
    
    std::vector<Point> points;
	int point_count;
	char ch;
    
    // Read header
	while(!pc_file_stream.eof())
	{
		pc_file_stream.get(ch);
		if(ch != ' ' && ch != '\t' && ch != '\n')
		{
			str.push_back(ch);
		}
        else
        {
            // Get vertex count
			if(str == "vertex")
			{
				str.clear();
				getline(pc_file_stream, str, '\n');
				point_count = atoi(str.c_str());
            }
            else if(str == "end_header")
            {
				str.clear();
				break;     
			}
            else
            {
                str.clear();
            }
        }
    }

    std::cout << "PC Point count: " << point_count << std::endl;

    int pos = 0;
	int counter = 0;
	double x, y, z, nx, ny, nz;
    int r, g, b;
 
	// Read vertices
	while (!pc_file_stream.eof())
	{
		pc_file_stream.get(ch);
        if(ch != ' ' && ch != '\t' && ch != '\n')
        {
			str.push_back(ch);
        }
        else
		{ 
            if(counter == point_count)
            {
                break;
            }
			// Save vertex data
            if(str == "")
            {
                continue;
            }
			else if(pos % 9 == 0)
			{
				x = atof(str.c_str());              
				str.clear();
			}
			else if(pos % 9 == 1)
			{
				y = atof(str.c_str());                    
				str.clear();
			}
			else if(pos % 9 == 2)
			{
				z = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 3)
			{
				nx = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 4)
			{
				ny = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 5)
			{
				nz = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 6)
			{
				r = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 7)
			{
				g = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 9 == 8)
			{
				b = atof(str.c_str());                   
				str.clear();
        		points.push_back(Point(x, y, z, nx, ny, nz, r, g, b));
				counter++;     
			}
			pos++;    
		}   
	}

    std::cout << "Read point set in: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    // Insert number_of_data_points in the tree
    Tree tree(points.begin(), points.end());

    std::cout << "Built Kd tree in: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    pc_file_stream.close();

    //Read mesh and vertices
    std::ifstream mesh_file_stream(mesh_file_name, std::ios_base::in); // Open mesh file

    if(!mesh_file_stream.is_open())
    {
        std::cerr << "Cannot read or find mesh file: " << mesh_file_name << std::endl;
        return 0;
    }
    
    std::vector<Point> vertices;
    int vertex_count, face_count;
   
    // Read header
	while(!mesh_file_stream.eof())
	{
		mesh_file_stream.get(ch);
		if(ch != ' ' && ch != '\t' && ch != '\n')
		{
			str.push_back(ch);
		}
		else
		{
		    // Get vertex count
			if(str == "vertex")
			{
				str.clear();
				getline(mesh_file_stream, str, '\n');
				vertex_count = atoi(str.c_str());
			}
            // Get face count
			else if(str == "face")
			{
				str.clear();
				getline(mesh_file_stream, str, '\n');
				face_count = atoi(str.c_str());
			}
			else if(str == "end_header")
			{
				str.clear();
				break;     
			}
            else
            {
                str.clear();
            }
		}
	} 

    std::cout << "Mesh vertex count: " << vertex_count << std::endl;
    std::cout << "Mesh face count: " << face_count << std::endl;

    pos = 0;
	counter = 0;
	double u,v;
 
	// Read vertices
	while(!mesh_file_stream.eof())
	{
        if(counter == vertex_count)
        {
            break;
        }
		mesh_file_stream.get(ch);
        if(ch != ' ' && ch != '\t' && ch != '\n')
        {
            str.push_back(ch);
        }
		else
		{ 
			// Save vertex data
            if(str == "")
            {
                continue;
            }
			else if(pos % 11 == 0)
			{
				x = atof(str.c_str());              
				str.clear();
			}
			else if(pos % 11 == 1)
			{
				y = atof(str.c_str());                    
				str.clear();
			}
			else if(pos % 11 == 2)
			{
				z = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 3)
			{
				nx = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 4)
			{
				ny = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 5)
			{
				nz = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 6)
			{
				u = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 7)
			{
				v = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 8)
			{
				r = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 9)
			{
				g = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 11 == 10)
			{
				b = atof(str.c_str());                   
				str.clear();
        		vertices.push_back(Point(x, y, z, nx, ny, nz, r, g, b, u, v));
				counter++;     
			}
			pos++;    
		}   
	}
    
    // Output image
    Mat texture(RESOLUTION, RESOLUTION, CV_8UC4);

    counter = 0;
    pos = 0;
    Point triangle_vertices[3];
	// Read faces
	while(!mesh_file_stream.eof())
	{
        if(counter == face_count)
        {
            break;
        }
		mesh_file_stream.get(ch);
        if(ch != ' ' && ch != '\t' && ch != '\n')
        {
            str.push_back(ch);
        }
		else
		{
            if(str == "")
            {
                continue;
            }
            else if(pos % 4 == 0)
			{
                // Ignore polygon vertex count
                // Assume all polygons are triangles
				str.clear();
			}
			else if(pos % 4 == 1)
			{
				triangle_vertices[0] = vertices[atof(str.c_str())];
				str.clear();
			}
			else if(pos % 4 == 2)
			{
				triangle_vertices[1] = vertices[atof(str.c_str())];
				str.clear();
			}
            else if(pos % 4 == 3)
			{
				triangle_vertices[2] = vertices[atof(str.c_str())];
				str.clear();
                
                counter++;
                
                // Find nearest points to the triangle
                std::set<Point, point_set_comparator> neighbors;
                for(int i = 0; i < 3; i++)
                {
                    K_neighbor_search search(tree, triangle_vertices[i], K);
                    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
                    {
                        neighbors.insert(it->first);
                    }
                }
                
                // Create CGAL Plane_3 of triangle vertices
                Point_3 r = point_to_point_3(triangle_vertices[0]);
                Point_3 p = point_to_point_3(triangle_vertices[1]);
                Point_3 q = point_to_point_3(triangle_vertices[2]);
                Plane_3 plane(r, p, q);
                
                // Triangle vertices to 2D
                Point_2 r_2 = plane.to_2d(r);
                Point_2 p_2 = plane.to_2d(p);
                Point_2 q_2 = plane.to_2d(q);
                Triangle_coordinates triangle_coordinates(r_2, p_2, q_2);
                
                // Save points for triangulation
                std::vector<std::pair<Point_2, unsigned>> triangulation_pts;
                triangulation_pts.push_back(std::make_pair(r_2, 0));
                triangulation_pts.push_back(std::make_pair(p_2, 1));
                triangulation_pts.push_back(std::make_pair(q_2, 2));
                
                // Save points that are in the triangle
                std::vector<Point> pts_in_tri;
                std::vector<std::vector<Scalar>> pts_bc_in_tri;
                
                // For each neighboring point
                int triangulation_index = 3;
                for(auto it = neighbors.begin(); it != neighbors.end(); it++)
                {
                    // Get neighboring point
                    Point n_point = *it;
                    
                    // Get CGAL Point_3 of point
                    Point_3 n_point_3 = point_to_point_3(n_point);
                    
                    // Find orthogonal projection of point onto plane
                    Point_3 n_proj_point_3 = plane.projection(n_point_3);
                    
                    // Projected point to 2D
                    Point_2 n_point_2 = plane.to_2d(n_proj_point_3);
                    
                    // Compute Barycentric Coordinate
                    std::vector<Scalar> bc;
                    triangle_coordinates(n_point_2, bc);
                    
                    // Check if point in triangle
                    if(bc[0] >= 0 && bc[1] >= 0 && bc[2] >= 0)
                    {
                        // Add point
                        pts_in_tri.push_back(n_point);
                        pts_bc_in_tri.push_back(bc);
                        triangulation_pts.push_back(std::make_pair(n_point_2, triangulation_index++));
                    }
                }
                
                // If there are no points in triangle
                if(triangulation_pts.size() == 3)
                {
                    // Draw triangle
                    draw_triangle(triangle_vertices, RESOLUTION, &texture);
                }
                // Else triangulate
                else
                {
                    Delaunay delaunay;
                    delaunay.insert(triangulation_pts.begin(), triangulation_pts.end());
                    
                    // For each face
                    for(Finite_faces_iterator it = delaunay.finite_faces_begin(); it != delaunay.finite_faces_end(); it++)
                    {
                        Point triangle[3];
                        for(int i = 0; i < 3; i++)
                        {
                            int index = it->vertex(i)->info();
                            switch (index) {
                                case 0:
                                    triangle[i] = triangle_vertices[0];
                                    break;
                                case 1:
                                    triangle[i] = triangle_vertices[1];
                                    break;
                                case 2:
                                    triangle[i] = triangle_vertices[2];
                                    break;
                                default:
                                    Point pt = pts_in_tri[index-3];
                                    std::vector<Scalar> pt_bc = pts_bc_in_tri[index-3];
                                    // Compute UV
                                    pt.U = CGAL::to_double(pt_bc[0]) * triangle_vertices[0].u() + CGAL::to_double(pt_bc[1]) * triangle_vertices[1].u() + CGAL::to_double(pt_bc[2]) * triangle_vertices[2].u();
                                    pt.V = CGAL::to_double(pt_bc[0]) * triangle_vertices[0].v() + CGAL::to_double(pt_bc[1]) * triangle_vertices[1].v() + CGAL::to_double(pt_bc[2]) * triangle_vertices[2].v();
                                    triangle[i] = pt;
                                    break;
                            }
                        }
                        // Draw Triangle
                        draw_triangle(triangle, RESOLUTION, &texture);
                    }
                }
            }
            pos++;
        }
	}
    mesh_file_stream.close();

    std::cout << "Nearest neighbors search total time: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();
    
    // Output
    // Split into channels
    Mat bgra[4];
    split(texture, bgra);
    // Create Dilate Kernel
    
    // Write Image
    cv::imwrite("texture.png", texture);
    std::cout << "Output time: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    std::cout << "Total real time: " << real_total_timer.time() << " seconds" << std::endl;
    real_total_timer.reset();
    
    std::cout << "VIRT: " << (CGAL::Memory_sizer().virtual_size() >> 20)  << " MiB" << std::endl;
    std::cout << "RES:  " << (CGAL::Memory_sizer().resident_size() >> 20) << " MiB" << std::endl;

    return 0;
}

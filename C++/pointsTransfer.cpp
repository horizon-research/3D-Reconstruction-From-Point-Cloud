#include <CGAL/Search_traits.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

#include <CGAL/Simple_cartesian.h>

#include <CGAL/Point_2.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>

#include <CGAL/Timer.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Memory_sizer.h>

#include "Point.h"  // Defines type Point, Construct_coord_iterator
#include "Distance.h"

#include <utility>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

typedef CGAL::Creator_uniform_3<double,Point> Point_creator;
typedef CGAL::Random_points_in_cube_3<Point, Point_creator> Random_points_iterator;
typedef CGAL::Counting_iterator<Random_points_iterator> N_Random_points_iterator;
typedef CGAL::Dimension_tag<3> D;
typedef CGAL::Search_traits<double, Point, const double*, Construct_coord_iterator, D> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits, Distance> K_neighbor_search;
typedef K_neighbor_search::Tree Tree;

typedef CGAL::Simple_cartesian<double> Kernel;

typedef Kernel::FT Scalar;

typedef CGAL::Point_2<Kernel> Point_2;
typedef CGAL::Point_3<Kernel> Point_3;
typedef CGAL::Triangle_3<Kernel> Triangle_3;
typedef CGAL::Plane_3<Kernel> Plane_3;
typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<Kernel> Triangle_coordinates;

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
                
                // Create CGAL Triangle_3
                Point_3 r = triangle_vertices[0].get_point_3();
                Point_3 p = triangle_vertices[1].get_point_3();
                Point_3 q = triangle_vertices[2].get_point_3();
                
                // Create CGAL Plane_3 of triangle vertices
                Plane_3 plane(r, p, q);
                
                // Triangle to 2D
                Point_2 r_2 = plane.to_2d(r);
                Point_2 p_2 = plane.to_2d(p);
                Point_2 q_2 = plane.to_2d(q);
                Triangle_coordinates triangle_coordinates(r_2, p_2, q_2);
                
                // Save points that are in the triangle
                std::vector<Point> ptsInTriangle;
                std::vector<std::vector<Scalar>> ptsBCInTriangle;
                
                // For each neighboring point
                for(auto it = neighbors.begin(); it != neighbors.end(); it++)
                {
                    // Get neighboring point
                    Point n_point = *it;
                    
                    // Get CGAL Point_3 of point
                    Point_3 n_point_3 = n_point.get_point_3();
                    
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
                        ptsInTriangle.push_back(n_point);
                        ptsBCInTriangle.push_back(bc);
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
    
    std::cout << "Output time: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    std::cout << "Total real time: " << real_total_timer.time() << " seconds" << std::endl;
    real_total_timer.reset();
    
    std::cout << "VIRT: " << (CGAL::Memory_sizer().virtual_size() >> 20)  << " MiB" << std::endl;
    std::cout << "RES:  " << (CGAL::Memory_sizer().resident_size() >> 20) << " MiB" << std::endl;

    return 0;
}

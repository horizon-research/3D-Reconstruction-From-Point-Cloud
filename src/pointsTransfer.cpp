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
#include <opencv2/imgproc/imgproc.hpp>

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

typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel;
typedef Kernel::FT                                              Scalar;

typedef CGAL::Point_2<Kernel>                                           Point_2;
typedef CGAL::Point_3<Kernel>                                           Point_3;
typedef CGAL::Triangle_2<Kernel>                                        Triangle_2;
typedef CGAL::Triangle_3<Kernel>                                        Triangle_3;
typedef CGAL::Plane_3<Kernel>                                           Plane_3;
typedef CGAL::Vector_3<Kernel>                                           Vector_3;  
typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<Kernel>   Triangle_coordinates;
typedef CGAL::Aff_transformation_3<Kernel>                              Transform3; 

#include "Quaternion.h"//Defines Quaternion

typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel>       Vertex_base;
typedef CGAL::Triangulation_data_structure_2<Vertex_base>                       Triangulation_data_structure;
typedef CGAL::Delaunay_triangulation_2<Kernel, Triangulation_data_structure>    Delaunay;
typedef Delaunay::Finite_vertices_iterator                                      Finite_vertices_iterator;
typedef Delaunay::Finite_faces_iterator                                         Finite_faces_iterator;

typedef cv::Mat     Mat;
typedef cv::Vec4b   Vec4b;
typedef cv::Vec3b   Vec3b;
typedef cv::Scalar Scalar_cv;

Point_3 point_to_point_3(Point &p)
{
    return Point_3 (p.x(), p.y(), p.z());
}

Vector_3 normal_to_color(Vector_3 normal)
{
    Vector_3 color = Vector_3(0.5, 0.5, 0.5) - normal/2;
    return color;
}

Vector_3 rotate(Vector_3 normal, Quaternion q){
    Transform3 tf = q.translate();
    return normal.transform(tf);
}

void draw_triangle(Point triangle[], int resolution, Mat texture_color, Quaternion rotation, Mat texture_normal) // add Quaternion and texture_normal
{
    Point_2 p_2(triangle[0].u() * resolution, triangle[0].v() * resolution);
    Point_2 q_2(triangle[1].u() * resolution, triangle[1].v() * resolution);
    Point_2 r_2(triangle[2].u() * resolution, triangle[2].v() * resolution);

    // Create CGAL Point_3 of triangle vertices
    Point_3 r = point_to_point_3(triangle[0]);
    Point_3 p = point_to_point_3(triangle[1]);
    Point_3 q = point_to_point_3(triangle[2]);
    
    // Create plane
    Plane_3 plane(r, p, q);

    Vector_3 triangle_normal;

    //Computer triangle normal and normalize
    Vector_3 plane_normal = plane.orthogonal_vector();
    double normal_length = sqrt(plane_normal.squared_length());
    if((triangle[0].nx() * plane_normal.x() + triangle[0].ny() * plane_normal.y() + triangle[0].nz() * plane_normal.z() < 0)
        || (triangle[1].nx() * plane_normal.x() + triangle[1].ny() * plane_normal.y() + triangle[1].nz() * plane_normal.z()< 0) 
        || (triangle[2].nx() * plane_normal.x() + triangle[2].ny() * plane_normal.y() + triangle[2].nz() * plane_normal.z()< 0)){
        triangle_normal = Vector_3 (-plane_normal.x()/normal_length, -plane_normal.y()/normal_length, -plane_normal.z()/normal_length);
    }
    else{
        triangle_normal = Vector_3 (plane_normal.x()/normal_length, plane_normal.y()/normal_length, plane_normal.z()/normal_length);
    }

    Vector_3 color = normal_to_color(rotate(triangle_normal, rotation));
    
    CGAL::Bbox_2 bb = Triangle_2(p_2,q_2,r_2).bbox();
    
    // Triangle Rasterization
    // For each pixel in bounding box
    for(int i = (int)std::floor(bb.xmin()); i <= std::floor(bb.xmax()); i++)
    {
        for(int j = (int)std::floor(bb.ymin()); j <= std::floor(bb.ymax()); j++)
        {
            int x = i;
            int y = j;
            // Boundary check
            if(x >= resolution){ x = resolution - 1; }
            if(y >= resolution){ y = resolution - 1; }
            
            // Compute barycentric coordiates
            Triangle_coordinates triangle_coordinates(p_2, q_2, r_2);
            std::vector<Scalar> bc;
            triangle_coordinates(Point_2(x,y), bc);
            
            // If in triangle
            if(bc[0] >= 0 && bc[1] >= 0 && bc[2] >= 0)
            {
                // Compute pixel color for color map
                float r = CGAL::to_double(bc[0]) * triangle[0].r() + CGAL::to_double(bc[1]) * triangle[1].r() + CGAL::to_double(bc[2]) * triangle[2].r();
                float g = CGAL::to_double(bc[0]) * triangle[0].g() + CGAL::to_double(bc[1]) * triangle[1].g() + CGAL::to_double(bc[2]) * triangle[2].g();
                float b = CGAL::to_double(bc[0]) * triangle[0].b() + CGAL::to_double(bc[1]) * triangle[1].b() + CGAL::to_double(bc[2]) * triangle[2].b();

                // Set Pixel for color map
                texture_color.at<Vec4b>(resolution - j, i)[0] = b;
                texture_color.at<Vec4b>(resolution - j, i)[1] = g;
                texture_color.at<Vec4b>(resolution - j, i)[2] = r;
                texture_color.at<Vec4b>(resolution - j, i)[3] = 255;

                // // Compute pixel color for normal map 
                // float nr = (CGAL::to_double(bc[0]) * color0.x() + CGAL::to_double(bc[1]) * color1.x() + CGAL::to_double(bc[2]) * color2.x()) * 255;
                // float ng = (CGAL::to_double(bc[0]) * color0.y() + CGAL::to_double(bc[1]) * color1.y() + CGAL::to_double(bc[2]) * color2.y()) * 255;
                // float nb = (CGAL::to_double(bc[0]) * color0.z() + CGAL::to_double(bc[1]) * color1.z() + CGAL::to_double(bc[2]) * color2.z()) * 255;
                float nr = color.x() * 255;
                float ng = color.y() * 255;
                float nb = color.z() * 255;

                // Set Pixel for normal map
                texture_normal.at<Vec3b>(resolution - j, i)[0] = nb;
                texture_normal.at<Vec3b>(resolution - j, i)[1] = ng;
                texture_normal.at<Vec3b>(resolution - j, i)[2] = nr;
            }
        }
    }
}

void draw_triangle(Point triangle[], int resolution, Mat texture_color, Mat texture_normal) // add Quaternion and texture_normal
{
    Point_2 p_2(triangle[0].u() * resolution, triangle[0].v() * resolution);
    Point_2 q_2(triangle[1].u() * resolution, triangle[1].v() * resolution);
    Point_2 r_2(triangle[2].u() * resolution, triangle[2].v() * resolution);
    
    CGAL::Bbox_2 bb = Triangle_2(p_2,q_2,r_2).bbox();
    
    // Triangle Rasterization
    // For each pixel in bounding box
    for(int i = (int)std::floor(bb.xmin()); i <= std::floor(bb.xmax()); i++)
    {
        for(int j = (int)std::floor(bb.ymin()); j <= std::floor(bb.ymax()); j++)
        {
            int x = i;
            int y = j;
            // Boundary check
            if(x >= resolution){ x = resolution - 1; }
            if(y >= resolution){ y = resolution - 1; }
            
            // Compute barycentric coordiates
            Triangle_coordinates triangle_coordinates(p_2, q_2, r_2);
            std::vector<Scalar> bc;
            triangle_coordinates(Point_2(x,y), bc);
            
            // If in triangle
            if(bc[0] >= 0 && bc[1] >= 0 && bc[2] >= 0)
            {
                // Compute pixel color for color map
                float r = CGAL::to_double(bc[0]) * triangle[0].r() + CGAL::to_double(bc[1]) * triangle[1].r() + CGAL::to_double(bc[2]) * triangle[2].r();
                float g = CGAL::to_double(bc[0]) * triangle[0].g() + CGAL::to_double(bc[1]) * triangle[1].g() + CGAL::to_double(bc[2]) * triangle[2].g();
                float b = CGAL::to_double(bc[0]) * triangle[0].b() + CGAL::to_double(bc[1]) * triangle[1].b() + CGAL::to_double(bc[2]) * triangle[2].b();

                // Set Pixel for color map
                texture_color.at<Vec4b>(resolution - j, i)[0] = b;
                texture_color.at<Vec4b>(resolution - j, i)[1] = g;
                texture_color.at<Vec4b>(resolution - j, i)[2] = r;
                texture_color.at<Vec4b>(resolution - j, i)[3] = 255;

                // // Set Pixel for normal map
                // texture_normal.at<Vec4b>(resolution - j, i)[0] = 255;
                // texture_normal.at<Vec4b>(resolution - j, i)[1] = 128;
                // texture_normal.at<Vec4b>(resolution - j, i)[2] = 128;
                // texture_normal.at<Vec4b>(resolution - j, i)[3] = 255;
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
    Mat texture_color(RESOLUTION, RESOLUTION, CV_8UC4);
    Mat texture_normal(RESOLUTION, RESOLUTION, CV_8UC3, Scalar_cv(255, 128, 128));

    counter = 0;
    pos = 0;
    int faces[face_count][3];
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
                int temp = std::stoi(str.c_str());
                faces[counter][0] = temp;
				str.clear();
			}
			else if(pos % 4 == 2)
			{
                int temp = std::stoi(str.c_str());
                faces[counter][1] = temp;
				str.clear();
			}
            else if(pos % 4 == 3)
			{
                int temp = std::stoi(str.c_str());
                faces[counter][2] = temp;
				str.clear();
                
                counter++;              
                
            }
            pos++;
        }
	}
    mesh_file_stream.close();
    std::cout << "Read mesh faces: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    double total_neighbor_search_time = 0;
    double total_triangle_draw_time = 0;
    
    Point triangle_vertices[3];
    Vector_3 face_normal;
    for(int j = 0; j < face_count; j++){

        // Find nearest points to the triangle
        std::set<Point, point_set_comparator> neighbors;
        for(int i = 0; i < 3; i++)
        {
            triangle_vertices[i] = vertices[faces[j][i]];
            K_neighbor_search search(tree, triangle_vertices[i], K);
            for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
            {
                neighbors.insert(it->first);
            }
        }
        
        total_neighbor_search_time += task_timer.time();
        task_timer.reset();
        
        // Create CGAL Point_3 of triangle vertices
        Point_3 r = point_to_point_3(triangle_vertices[0]);
        Point_3 p = point_to_point_3(triangle_vertices[1]);
        Point_3 q = point_to_point_3(triangle_vertices[2]);
        
        // Create plane
        Plane_3 plane(r, p, q);

        //Computer face normal and normalize
        Vector_3 plane_normal = plane.orthogonal_vector();
        double normal_length = sqrt(plane_normal.squared_length());
        if((triangle_vertices[0].nx() * plane_normal.x() + triangle_vertices[0].ny() * plane_normal.y() + triangle_vertices[0].nz() * plane_normal.z() < 0)
            || (triangle_vertices[1].nx() * plane_normal.x() + triangle_vertices[1].ny() * plane_normal.y() + triangle_vertices[1].nz() * plane_normal.z()< 0) 
            || (triangle_vertices[2].nx() * plane_normal.x() + triangle_vertices[2].ny() * plane_normal.y() + triangle_vertices[2].nz() * plane_normal.z()< 0)){
            face_normal = Vector_3 (-plane_normal.x()/normal_length, -plane_normal.y()/normal_length, -plane_normal.z()/normal_length);
        }
        else{
            face_normal = Vector_3 (plane_normal.x()/normal_length, plane_normal.y()/normal_length, plane_normal.z()/normal_length);
        }
        Quaternion rotation(face_normal, Vector_3(0, 0, 1));

        // Project to 2D
        Point_2 r_2 = plane.to_2d(r);
        Point_2 p_2 = plane.to_2d(p);
        Point_2 q_2 = plane.to_2d(q);

        // For computing barycentric coordinates
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
            draw_triangle(triangle_vertices, RESOLUTION, texture_color, texture_normal);
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
                draw_triangle(triangle, RESOLUTION, texture_color, rotation, texture_normal);
            }
        }
        
        total_triangle_draw_time += task_timer.time();
        task_timer.reset();
    }
    
    std::cout << "Neighbor search total time: " << total_neighbor_search_time << " seconds" << std::endl;
    task_timer.reset();
    
    std::cout << "Draw triangles total time: " << total_triangle_draw_time << " seconds" << std::endl;
    task_timer.reset();
    
    //Output color map
    // Create dilate kernel
    Mat dilate_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
    // Dilate Image
    Mat dilated;
    dilate(texture_color, dilated, dilate_kernel);
    // Split into 4 channels
    Mat bgra[4];
    split(texture_color, bgra);
    // Create alpha mask
    Mat all_alpha;
    Mat color_alphas[4] = {bgra[3], bgra[3], bgra[3], bgra[3]};
    merge(color_alphas, 4, all_alpha);
    Mat alpha_mask;
    bitwise_not(all_alpha, alpha_mask);
    // Extract dilated edges
    Mat edges;
    bitwise_and(dilated, alpha_mask, edges);
    // Append edges to original
    Mat padded;
    add(texture_color, edges, padded);
    // Write Image
    cv::imwrite("textureColor.png", padded);
    std::cout << "Color map output time: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    //Output normal map
    // Write Image
    cv::imwrite("textureNormal.png", texture_normal);
    std::cout << "Normal map output time: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    std::cout << "Total real time: " << real_total_timer.time() << " seconds" << std::endl;
    real_total_timer.reset();
    
    std::cout << "VIRT: " << (CGAL::Memory_sizer().virtual_size() >> 20)  << " MiB" << std::endl;
    std::cout << "RES:  " << (CGAL::Memory_sizer().resident_size() >> 20) << " MiB" << std::endl;

    return 0;
}

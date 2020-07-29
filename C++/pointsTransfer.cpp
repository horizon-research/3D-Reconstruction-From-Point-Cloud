#include <CGAL/Search_traits.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include "Point.h"  // Defines type Point, Construct_coord_iterator
#include "Distance.h"

#include <utility>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Timer.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

typedef CGAL::Creator_uniform_3<double,Point> Point_creator;
typedef CGAL::Random_points_in_cube_3<Point, Point_creator> Random_points_iterator;
typedef CGAL::Counting_iterator<Random_points_iterator> N_Random_points_iterator;
typedef CGAL::Dimension_tag<3> D;
typedef CGAL::Search_traits<double, Point, const double*, Construct_coord_iterator, D> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits, Distance> K_neighbor_search;
typedef K_neighbor_search::Tree Tree;

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

    std::cout << "Point count: " << point_count << std::endl;

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

    std::cout << "Read point set cost: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();

    // Insert number_of_data_points in the tree
    Tree tree(points.begin(), points.end());
    Distance tr_dist;

    std::cout << "Build Kd tree cost: " << task_timer.time() << " seconds" << std::endl;
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

    std::cout << "Vertex count: " << vertex_count << std::endl;
    std::cout << "Face count: " << face_count << std::endl;

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

    // boost::property_tree::ptree pt, output, child1, ans;
    // child1.put("sd", "sdf");
    // pt.push_back(std::make_pair("", child1));
    // pt.push_back(std::make_pair("", child1));
    // pt.push_back(std::make_pair("", child1));
    // pt.push_back(std::make_pair("", child1));
    // for(int index = 0; index < 2; index++){
    //   output.push_back(std::make_pair("", pt));
    // }
    // ans.add_child("res", output);
    // child1.put("sd", "---");

    // std::stringstream ss;
    // boost::property_tree::json_parser::write_json(ss, ans);
    // std::cout << ss.str() << std::endl;

    boost::property_tree::ptree pt, triangle, pts, ptList, single, data, output;

    int ver[4];
    int i = 1;
    Point v1, v2, v3;
    counter = 0;
    pos = 0;
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
				ver[0] = atof(str.c_str());              
				str.clear();
			}
			else if(pos % 4 == 1)
			{
				ver[1] = atof(str.c_str());                    
				str.clear ( );      
			}
			else if(pos % 4 == 2)
			{
				ver[2] = atof(str.c_str());                   
				str.clear();
			}
            else if(pos % 4 == 3)
			{
				ver[3] = atof(str.c_str());                   
				str.clear();
            
                counter++;
        
                v1 = vertices[ver[1]];
                v2 = vertices[ver[2]];
                v3 = vertices[ver[3]];
                
                for(int index = 1; index <= 3; index++){
                    // Store the vertices of the triangle
                    pt.put("v", vertices[ver[index]].detailWithUV());
                    triangle.push_back(std::make_pair("", pt));

                    // Look for points near the vertices
                    K_neighbor_search search(tree, vertices[ver[index]], K);
                    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
                        pts.put("p", (it->first).detail());
                        ptList.push_back(std::make_pair("", pts));
                    }
                }
                
                single.add_child("triangle", triangle);
                triangle.clear();
                single.add_child("points", ptList);
                ptList.clear();
                data.push_back(std::make_pair("",single));
                single.clear();
            }
            pos++;
        }
	}
    mesh_file_stream.close();
    
    output.add_child("data", data);

    std::cout << "Search near points cost: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();
    
    std::stringstream ss;
    boost::property_tree::json_parser::write_json(ss, output);
    std::ofstream outFile;
    outFile.open("data/pointsMesh.json");
    outFile << ss.str();
    outFile.close();

    std::cout << "Writing output cost: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();


//    // search K nearest neighbours
//    K_neighbor_search search(tree, vertices[0], K);
//    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
//        std::cout << " d(q, nearest neighbor)=  " << (it->first).x()
//                  << tr_dist.inverse_of_transformed_distance(it->second) << std::endl;
//    }

//    // Display points read
//    for (std::size_t i = 0; i < 3; ++ i)
//    {
//      const double x = points[i].x();
//      const double y = points[i].y();
//      const double z = points[i].z();
//      std::cerr << "Point (" << x << "  " << y << " "<< z << ") "<< std::endl;
//    }

    return 0;
}

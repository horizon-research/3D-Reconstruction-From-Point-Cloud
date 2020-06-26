#include <CGAL/Search_traits.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include "Point.h"  // defines types Point, Construct_coord_iterator
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

int main() {
  const int N = 1000;
  const unsigned int K = 20; //the number of search range

  CGAL::Timer task_timer; task_timer.start();

  //Read origin point set
  std::vector<Point> points; 
  std::string file_name = "data/RRLibStatueWithNormalX.ply";
  std::ifstream fin ( file_name, std::ios_base::in );//開檔 

 
	if ( !fin.is_open ( ) )
	{ 
		std::cout << "Cannot read the file." << std::endl;
		std::cout << "Please check again." << std::endl;
		exit(0);
        }
 
	std::string str;
	int numberOfPoints;
	char ch;
 /*讀取header*/

	while ( !fin.eof ( ) )
	{
		fin.get ( ch );
		if( ch != ' ' && ch != '\t' && ch != '\n' )
		{
			str.push_back ( ch );   
		}
		else
		{
		//取得vertex個數 
			if(str == "vertex")
			{
				str.clear ( );
				getline ( fin, str, '\n' ); 
				numberOfPoints = atoi(str.c_str());       
      }
			else if(str == "end_header")
			{
				str.clear ( );     
				break;     
			}
			else
				str.clear ( );             
		}
  } 

  std::cerr << "the size of points is: " << numberOfPoints << std::endl;

  int pos = 0;
	int counter = 0;
	double x , y, z, nx, ny, nz;
  int r, g, b;
 
	/*讀取Vertex*/ 
	while ( !fin.eof ( ) )
	{
		fin.get ( ch );
		if( ch != ' ' && ch != '\t' && ch != '\n' )
			str.push_back ( ch );
 
		else
		{ 
			if(counter == numberOfPoints)	break;  
			/*儲存vertex資料*/
			if(str == "")	continue;
 
			else if(pos%9 == 0)
			{
				x = atof(str.c_str());              
				str.clear ( );        
			}
			else if(pos%9 == 1)
			{
				y = atof(str.c_str());                    
				str.clear ( );      
			}
			else if(pos%9 == 2)
			{
				z = atof(str.c_str());                   
				str.clear ( );     
			}
      else if(pos%9 == 3)
			{
				nx = atof(str.c_str());                   
				str.clear ( );   
			}
      else if(pos%9 == 4)
			{
				ny = atof(str.c_str());                   
				str.clear ( );   
			}
      else if(pos%9 == 5)
			{
				nz = atof(str.c_str());                   
				str.clear ( );     
			}
      else if(pos%9 == 6)
			{
				r = atof(str.c_str());                   
				str.clear ( );      
			}
      else if(pos%9 == 7)
			{
				g = atof(str.c_str());                   
				str.clear ( );    
			}
      else if(pos%9 == 8)
			{
				b = atof(str.c_str());                   
				str.clear ( );  
        points.push_back(Point(x, y, z, nx, ny, nz, r, g, b));
				counter++;     
			}
			pos++;    
		}   
	}

  std::cerr << "Reads origin point set " << file_name << ": " << points.size() << " points, "
                                                         << task_timer.time() << " seconds"
                                                         << std::endl;
  task_timer.reset();

    // Insert number_of_data_points in the tree
  Tree tree(points.begin(), points.end());
  Distance tr_dist;

  std::cerr << "Build Kd tree cost: " << task_timer.time() << " seconds" << std::endl;
  task_timer.reset();

  fin.close();  

  //Read mesh and vertice
  std::vector<Point> vertices; 
  std::string file_name2 = "data/125KWithUV.ply";
  std::ifstream fin2 ( file_name2, std::ios_base::in );//開檔 

  int vertex, face;
 /*讀取header*/
 
	while ( !fin2.eof ( ) )
	{
		fin2.get ( ch );
		if( ch != ' ' && ch != '\t' && ch != '\n' )
		{
			str.push_back ( ch );   
		}
		else
		{
		//取得vertex個數 
			if(str == "vertex")
			{
				str.clear ( );
				getline ( fin2, str, '\n' ); 
				vertex = atoi(str.c_str());       
			}
		//取得face個數 
			else if(str == "face")
			{
				str.clear ( );
				getline ( fin2, str, '\n' );  
				face = atoi(str.c_str());             
			}
			else if(str == "end_header")
			{
				str.clear ( );     
				break;     
			}
			else
				str.clear ( );             
		}
	} 

  std::cerr << "the size of vertices is: " << vertex << std::endl;
  std::cerr << "the size of face is: " << face << std::endl;

  pos = 0;
	counter = 0;
	double u,v;
 
	/*讀取Vertex*/ 
	while ( !fin2.eof ( ) )
	{
    if(counter == vertex)	break;  
		fin2.get ( ch );
		if( ch != ' ' && ch != '\t' && ch != '\n' )
			str.push_back ( ch );
 
		else
		{ 
			/*儲存vertex資料*/
			if(str == "")	continue;
 
			else if(pos%11 == 0)
			{
				x = atof(str.c_str());              
				str.clear ( );        
			}
			else if(pos%11 == 1)
			{
				y = atof(str.c_str());                    
				str.clear ( );      
			}
			else if(pos%11 == 2)
			{
				z = atof(str.c_str());                   
				str.clear ( );     
			}
      else if(pos%11 == 3)
			{
				nx = atof(str.c_str());                   
				str.clear ( );   
			}
      else if(pos%11 == 4)
			{
				ny = atof(str.c_str());                   
				str.clear ( );   
			}
      else if(pos%11 == 5)
			{
				nz = atof(str.c_str());                   
				str.clear ( );     
			}
      else if(pos%11 == 6)
			{
				u = atof(str.c_str());                   
				str.clear ( );      
			}
      else if(pos%11 == 7)
			{
				v = atof(str.c_str());                   
				str.clear ( );    
			}
      else if(pos%11 == 8)
			{
				r = atof(str.c_str());                   
				str.clear ( );    
			}
      else if(pos%11 == 9)
			{
				g = atof(str.c_str());                   
				str.clear ( );    
			}
      else if(pos%11 == 10)
			{
				b = atof(str.c_str());                   
				str.clear ( );  
        vertices.push_back(Point(x, y, z, nx, ny, nz, r, g, b, u, v));
				counter++;     
			}
			pos++;    
		}   
	}

  std::cout<< counter << std::endl;

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
	/*畫Polygon*/  
	while ( !fin2.eof ( ) )
	{
    if(counter == face)		break;  
		fin2.get ( ch );
    // std::cout<< ch << std::endl;
		if( ch != ' ' && ch != '\t' && ch != '\n' )
			str.push_back ( ch );
		else
		{
      if(str == "")	continue; 
      else if(pos%4 == 0)
			{
				ver[0] = atof(str.c_str());              
				str.clear ( );        
			}
			else if(pos%4 == 1)
			{
				ver[1] = atof(str.c_str());                    
				str.clear ( );      
			}
			else if(pos%4 == 2)
			{
				ver[2] = atof(str.c_str());                   
				str.clear ( );     
			}
      else if(pos%4 == 3)
			{
				ver[3] = atof(str.c_str());                   
				str.clear ( );     
        counter++; 
        
        v1 = vertices[ver[1]];
        v2 = vertices[ver[2]];
        v3 = vertices[ver[3]];
        for(int index = 0; index < 3; index++){

          //store the vertex of triangle
          pt.put("x", vertices[ver[index]].x());
          pt.put("y", vertices[ver[index]].y());
          pt.put("z", vertices[ver[index]].z());
          pt.put("nx", vertices[ver[index]].nx());
          pt.put("ny", vertices[ver[index]].ny());
          pt.put("nz", vertices[ver[index]].nz());
          pt.put("r", vertices[ver[index]].r());
          pt.put("g", vertices[ver[index]].g());
          pt.put("b", vertices[ver[index]].b());
          pt.put("u", vertices[ver[index]].u());
          pt.put("v", vertices[ver[index]].v());
          triangle.push_back(std::make_pair("",pt));

          //look for points near vertex
          K_neighbor_search search(tree, vertices[ver[index]], K);
          for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
            pts.put("x", (it->first).x());
            pts.put("y", (it->first).y());
            pts.put("z", (it->first).z());
            pts.put("nx", (it->first).nx());
            pts.put("ny", (it->first).ny());
            pts.put("nz", (it->first).nz());
            pts.put("r", (it->first).r());
            pts.put("g", (it->first).g());
            pts.put("b", (it->first).b());
            ptList.push_back(std::make_pair("",pts));
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
    fin2.close();    
    output.add_child("data", data);

    std::cerr << "Search near points cost: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();


    std::stringstream ss;
    boost::property_tree::json_parser::write_json(ss, output);
    std::ofstream outFile;
    outFile.open("data/pointsMesh.json");
    outFile << ss.str();
    outFile.close();

    std::cerr << "Writing output cost: " << task_timer.time() << " seconds" << std::endl;
    task_timer.reset();


  // // search K nearest neighbours
  // K_neighbor_search search(tree, vertices[0], K);
  // for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
  //   std::cout << " d(q, nearest neighbor)=  " << (it->first).x()
	//       << tr_dist.inverse_of_transformed_distance(it->second) << std::endl;
  // }

  // // Display points read
  for (std::size_t i = 0; i < 3; ++ i)
    {
      const double x = points[i].x();
      const double y = points[i].y();
      const double z = points[i].z();                                                                                                                                                                                                                                                                                                                                             
      std::cerr << "Point (" << x << "  " << y << " "<< z << ") "<< std::endl;
    }

  return 0;
}
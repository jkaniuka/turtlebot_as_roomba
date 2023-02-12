#include "package_303762/path_generator_alg.hpp"

using namespace std;

// robot radius determined using RViz Measure Tool
float radius = 0.3;

// path generating algorithm (diagram in pdf report)
vector<point_t> generate_discrete_path(float x_start,float y_start,float x_end,float y_end) {
    int cols = floor(abs(x_end-x_start)/radius);
    int rows = floor(abs(y_end-y_start)/radius);
    int arr_size = (cols+1)*(rows+1);
    vector< point_t > points;

    for(int i=0; i <(rows+1); i++){
                vector< point_t > local_list;
            for (int j=0; j <(cols+1); j++){
                
            point_t mypoint;
            if(x_start > x_end && y_start > y_end){
                mypoint = {x_start-j*radius, y_start-i*radius};
            }
            else if (x_start < x_end && y_start < y_end){
                mypoint = {x_start+j*radius, y_start+i*radius};
            }
            else if (x_start > x_end && y_start < y_end){
                mypoint = {x_start-j*radius, y_start+i*radius};
            }
            else if (x_start < x_end && y_start > y_end){
                mypoint = {x_start+j*radius, y_start-i*radius};
            }               
              local_list.push_back(mypoint);
                
            }
            if (x_end > x_start)
            {
                if (i%2==0){    
                    for(int i =0; i<(cols+1);i++){
                        local_list[i].z = 0;
                        local_list[i].w = 1;
                    }
                    points.insert(points.end(), local_list.begin(), local_list.end());
                }
                else{
                    for(int i = 0; i<(cols+1); i++){
                        local_list[i].z = -1;
                        local_list[i].w = 0;
                    }
                    reverse(local_list.begin(), local_list.end());
                    points.insert(points.end(), local_list.begin(), local_list.end());
                }
            }
            else
            {
                if (i%2==0){    
                    for(int i =0; i<(cols+1);i++){
                        local_list[i].z = -1;
                        local_list[i].w = 0;
                    }
                    points.insert(points.end(), local_list.begin(), local_list.end());
                }
                else{
                    for(int i = 0; i<(cols+1); i++){
                        local_list[i].z = 0;
                        local_list[i].w = 1;
                    }
                    reverse(local_list.begin(), local_list.end());
                    points.insert(points.end(), local_list.begin(), local_list.end());
                }
            }

    }        
    cout<< "X Y Z W"<<endl;
    for (int i = 0; i < arr_size; i++) {
        cout<< points[i].x<<" "<<points[i].y<<" "<<points[i].z<<" "<<points[i].w<<endl;
    }
    return points;
}


// simple test function
void sayHello()
{
    ROS_INFO("Hello!");
}
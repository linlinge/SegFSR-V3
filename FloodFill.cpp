#include "FloodFill.h"

// void FloodFill::floodFillUtil(cv::Mat& screen, int x, int y, Vertices& ant) 
// { 
// 	// cout<<screen.rows<<","<<screen.cols<<","<<x<<","<<y<<endl;
//     // Base cases	
//     if ( x < 0 || x >= screen.rows || y < 0 || y >= screen.cols) 
//         return; 
//     if (screen.at<uchar>(x,y) != 0) 
//         return; 
  
//     // Replace the color at (x, y) 
//     screen.at<uchar>(x,y) = 100; 
// 	Vertex* vtmp=active_vertices_.Remove(x,y);
// 	ant.Insert(vtmp);
  
//     // Recur for north, east, south and west 
//     floodFillUtil(screen, x+1, y,ant); 
//     floodFillUtil(screen, x-1, y,ant); 
//     floodFillUtil(screen, x, y+1,ant); 
//     floodFillUtil(screen, x, y-1,ant); 
// }

static int wokaka=0;
void FloodFill::floodFillUtil(cv::Mat& screen, int x, int y, Vertices& ant) 
{	
	// cv::imwrite("wokaka.png",screen);
    uchar init = screen.at<uchar>(x,y);
	char r=200;
    // if (init != 0) return;
    // if (x < 0 || x >= screen.rows || y < 0 || y >= screen.cols) return; //Index out of bounds. 
    std::queue<std::pair<int, int>> q;
    q.push(std::make_pair(x, y)); 	//Push the initial position into the queue.    
    while (!q.empty()) {			//Keep looking at relevant neighbours so long as there is something in the queue. 
        auto pt = q.front(); 		//Collect the first entry.
        q.pop(); 					//Remove it, we don't want to keep processing the same point. 
		screen.at<uchar>(pt.first,pt.second) = r; 		//Replace the 	initial point. 	
		active_vertices_.remove(pt.first,pt.second);	
		ant.insert(pt.first,pt.second);
        //Now add neighbours if they match our initial point. 	
        if(pt.first-1 >= 0 && screen.at<uchar>(pt.first - 1,pt.second) == 0){
			q.push(std::make_pair(pt.first - 1, pt.second));
            screen.at<uchar>(pt.first - 1,pt.second) = r; 		//Replace the value here to avoid pushing the same point twice. 
		}

        if(pt.first + 1 < screen.rows && screen.at<uchar>(pt.first + 1,pt.second) == 0){
			q.push(std::make_pair(pt.first + 1, pt.second));
            screen.at<uchar>(pt.first + 1,pt.second) = r;
		}

        if(pt.second-1 >= 0 && screen.at<uchar>(pt.first,pt.second - 1) == 0){
			q.push(std::make_pair(pt.first, pt.second - 1));
            screen.at<uchar>(pt.first,pt.second - 1) = r;
		}

        if(pt.second + 1 < screen.cols && screen.at<uchar>(pt.first,pt.second + 1) == 0){
			q.push(std::make_pair(pt.first, pt.second + 1));
            screen.at<uchar>(pt.first,pt.second + 1) = r;
		}
    }
}

FloodFill::FloodFill(cv::Mat& img) 
{ 
	for(int i=0;i<img.rows;i++){
		//#pragma omp parallel shared(active_vertices_)
		for(int j=0;j<img.cols;j++){
			if(img.at<uchar>(i,j)==0)
				active_vertices_.insert(i,j);		
		}
	}
		
	// Find all result
	while(active_vertices_.size()!=0){
			list<pair<int,int>>::iterator it=active_vertices_.dat_.begin();
			Vertices ant;
			floodFillUtil(img,(*it).first,(*it).second ,ant);
			// if(ant.size()>100)
			// 	int a=1;			
			result_.push_back(ant);
	}
	
	// cv::imwrite("1.png", img);

	// cout<<wokaka++<<endl;
	// collate result
	sort(result_.begin(),result_.end(),[](Vertices& e1, Vertices& e2){return e1.size()>e2.size();});	
} 
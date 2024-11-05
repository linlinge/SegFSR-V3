#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <omp.h>
#include <utility>
#include <queue>
#include <list>
using namespace std;
class Vertices{
	public:
		list<pair<int,int>> dat_;
		list<pair<int,int>>::iterator it_;
	
	void insert(int x,int y){
		dat_.insert(dat_.end(),make_pair(x,y));
	}
	int size(){
		return dat_.size();
	}
	void remove(int x,int y){
		for(list<pair<int,int>>::iterator it=dat_.begin();it!=dat_.end();it++){
			if((*it).first==x && (*it).second==y){
				it_=dat_.erase(it);
				break;
			}
		}
	}

	Vertices() {};

	// 拷贝构造函数（深度拷贝）
    Vertices(const Vertices& other) {
        // 深拷贝 dat_（由于list自动处理深拷贝，因此直接赋值即可）
        dat_ = other.dat_;

        it_=other.it_;
    }

};
class FloodFill
{
	public:
		Vertices active_vertices_;	// mark the vertices which are not be used
		vector<Vertices> result_;
		
		// External Function
		FloodFill(cv::Mat& img);
		
		// Internal Function
		void floodFillUtil(cv::Mat& screen, int x, int y, Vertices& ant);
};
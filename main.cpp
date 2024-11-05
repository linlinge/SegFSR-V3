#include <iostream>
#include "SegFSR.h"
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <ctime>
#include <stdio.h>
#include "PCLExtend.h"
using namespace std;
vector<double> StatisticNearestDistance(pcl::PointCloud<PointType>::Ptr cloud)
{
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	vector<double> rst;
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{ 
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{			
			rst.push_back(sqr_distances[1]);
			++n_points;
		}
	}	
	
	return rst;
}
double mean(vector<double>& dat)
{
	double sum=0;
	#pragma omp parallel for reduction(+:sum)
	for(int i=0;i<dat.size();i++) sum+=dat[i];
	
	return sum/(double)dat.size();
}

int main(int argc, char **argv)
{	
	cout<<"------------------------------------------------------------------"<<endl;

	string ipath_of_ply, opath;
	for(int i=1; i<argc; i++){
		string s = argv[i];
		if(s=="--ipath_of_ply"){
			ipath_of_ply = argv[i+1];
		}
		else if(s=="--opath"){
			opath = argv[i+1];
		}
	}

	// Get filename
	string fname = ipath_of_ply;
	while(fname.find("/")!=-1)
		fname=fname.substr(1, fname.size()-1);
	fname = fname.substr(0, fname.size()-4);

	cout<<fname<<endl;

	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);	
	if (pcl::io::loadPLYFile<PointType>(ipath_of_ply, *cloud) == -1){
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}	
	
 	SegFSR alg;
	alg.Init(cloud);
	alg.Run();
	alg.ExtractLabels( opath + "/" + fname +"_segfsr.txt");
	alg.ExtractFilterCloud(opath + "/" + fname + "_segfsr.ply");
	alg.PrintLogs(ipath_of_ply);

	return 0;
}
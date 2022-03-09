#include <iostream>
#include <cstdlib>
#include <ctime>
#include <toposlam/TopoGraph.h>
//#include "/home/mingfeng/catkin_ws/src/toposlam/include/toposlam/TopoGraph.h"

typedef VertexProperties<unsigned long> TV;
typedef EdgeProperties TE;
typedef unsigned int TI;

int main(int argc, char **argv)
{
	TopoGraph<TV, TE, TI> mygraph;
	
	std::srand(std::time(0));
	
	for(int i=0; i<10; i++) {
		std::cout << "Add node " << i << ":" << std::endl;
		TV node(i);
		double weight = (std::rand() % 100 + 1.0)/20.0; // in (0.05, 5.0]
		TE edge(weight);
		bool bNode = mygraph.AddNode(node, edge, edge);
		std::cout << (bNode? "Success!" : "Fail!") << std::endl;
	}
	
	double weight = (std::rand() % 100 + 1.0)/20.0; // in (0.05, 5.0]
	TE edge1(weight);
	bool bEdge1 = mygraph.AddEdge(TI(1), TI(3), edge1);
	weight = (std::rand() % 100 + 1.0)/20.0; // in (0.05, 5.0]
	TE edge2(weight);
	bool bEdge2 = mygraph.AddEdge(TI(1), TI(4), edge2);
	weight = (std::rand() % 100 + 1.0)/20.0; // in (0.05, 5.0]
	TE edge3(weight);
	bool bEdge3 = mygraph.AddEdge(TI(3), TI(9), edge3);
	
	std::vector<TI> Path = mygraph.FindPath(TI(0), TI(8));
	
	mygraph.SaveGraph("test.dat");
	
	TopoGraph<TV, TE, TI>  newGraph;
	
	//mygraph.LoadGraph("test.dat", newGraph);
	
	newGraph.LoadGraph("test.dat");
	
	mygraph.GetNeighbors(1);
	mygraph.GetNeighbors(3);
	
	std::cout << "In new graph: " << std::endl;
	
	std::vector<TI> newPath = newGraph.FindPath(TI(0), TI(8));
	newGraph.GetNeighbors(TI(1));
	newGraph.GetNeighbors(TI(3));
	
	
	//TopoGraph<TV, TE, TI>::Vertex_t temp;
	
	return 0;
}

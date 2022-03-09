//#include "/home/mingfeng/catkin_ws/src/toposlam/include/toposlam/TopoGraph.h"
#include <toposlam/TopoGraph.h>
//
//template <typename TV, typename TE, typename TI>
//TopoGraph<TV, TE, TI>::TopoGraph( ) 
//{
 
//}

//template <typename TV, typename TE, typename TI>
//TopoGraph<TV, TE, TI>::~TopoGraph()
//{
//}

//template <typename TV, typename TE, typename TI>
//bool TopoGraph<TV, TE, TI>::AddNode(const TV& nodeData, const TE& edgeData)
//{
	//bool bSuccess = true;
	
	//Vertex_t newVertex = boost::add_vertex(m_Graph); // add a new vertex to the graph
	
	//m_Graph[newVertex] = nodeData; // set node properties, needs to define an assignment operator/copy constructor in class TV
	
	//if(boost::num_vertices(m_Graph) != 0) // if it's not the first vertex, add an edge to connect the current active vertex
	//{
		//bSuccess = AddEdge(m_CurrentVertex, newVertex, edgeData);
	//}
		
	//m_CurrentVertex = newVertex;
	//m_IDMap[nodeData.GetID()] = m_CurrentVertex;	
	
	//return bSuccess;
//}

//template <typename TV, typename TE, typename TI>
//bool TopoGraph<TV, TE, TI>::AddEdge(Vertex_t vertextStart, Vertex_t vertextEnd, const TE& edgeData)
//{
	//Edge_t newEdge;
	//bool bSuccess;
	//boost::tie(newEdge,bSuccess) = boost::add_edge(vertextStart, vertextEnd, m_Graph); // add an edge
	//m_Graph[newEdge] = edgeData; // set edge properties, needs to define an assignment operator/copy constructor in class TE	
	
	//return bSuccess;
//}

//template <typename TV, typename TE, typename TI>
//bool TopoGraph<TV, TE, TI>::AddEdge(TI idStart, TI idEnd, const TE& edgeData)
//{
	//Vertex_t vertextStart, vertextEnd; 
	//try {
		//vertextStart = m_IDMap.at(idStart);
		//vertextEnd = m_IDMap.at(idEnd);
	//}
	//catch (const std::out_of_range& oor) {
		//std::cerr << "Out of Range (TopoGraph/AddEdge/IDMap) error: " << oor.what() << std::endl;
	//}
	
	//return AddEdge(vertextStart, vertextEnd, edgeData);
//}

//template <typename TV, typename TE, typename TI>
//std::vector<TI> TopoGraph::FindPath(TI idStart, TI idEnd)
//{
	//try {
		//Vertex_t vertextStart = m_IDMap.at(idStart);
		//Vertex_t vertextEnd = m_IDMap.at(idEnd);
	//}
	//catch (const std::out_of_range& oor) {
		//std::cerr << "Out of Range (TopoGraph/FindPath/IDMap) error: " << oor.what() << std::endl;
	//}
	
	//return FindPath(vertextStart, vertextEnd);
//}

//template <typename TV, typename TE, typename TI>
//std::vector<Vertex_t> TopoGraph::FindPath(const Vertex_t& vStart, const Vertex_t& vEnd)
//{
	//// Create things for Dijkstra
	//std::vector<Vertex_t> parents(boost::num_vertices(m_Graph)); // To store parents
	//std::vector<double> distances(boost::num_vertices(m_Graph)); // To store distances	
	
	//VertexIndexMap Vertex_Map = boost::get(boost::vertex_index, m_Graph); 
	
	//boost::dijkstra_shortest_paths(m_Graph, vStart,
		//boost::weight_map(boost::get(typename &TE::WEIGHT, m_Graph)).
			//predecessor_map(boost::make_iterator_property_map(&parents[0], Vertex_Map)).
			//distance_map(boost::make_iterator_property_map(&distances[0], Vertex_Map)) );
			
	//std::vector<Vertex_t> path;
	//Vertex_Iter vi = vEnd;
	//while ( vi ! = vStart) {
		//path.push_back(vi);
		//vi = parents[vi];
	//}
	//path.push_back(vStart);
		
	//return path;
//}

//template <typename TV, typename TE, typename TI>
//void TopoGraph::PrintPath(std::vector<Vertex_t> path)
//{
	////This prints the path reversed use reverse_iterator and rbegin/rend
	//std::vector<Vertex_t>::iterator it;
	
	//for (it=path.begin(); it != path.end(); ++it) {
	
	    //std::cout << m_Graph[*it].GetID() << "->";
	//}
	//std::cout << std::endl;	
//}

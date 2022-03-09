#ifndef TOPOGRAPH_
#define TOPOGRAPH_

#include <ros/ros.h>
#include <ros/console.h>
#include <TooN/TooN.h>
#include <iostream>
#include <fstream>
#include <algorithm>    // std::reverse
#include <vector>
#include <map>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/archive_exception.hpp>

/// properties of each node/vertex
/// TI: type of vertiex (place/node in VocTree) ID
template <typename TI>
class VertexProperties
{
public:
	VertexProperties(TI nID = 0 ) : m_nID(nID) { }
	VertexProperties(TI nID, TooN::Vector<3> v3Pos) : m_nID(nID), m_v3Position(v3Pos){ }
	VertexProperties(TI nID, TooN::Vector<6> v6Pose) : m_nID(nID), m_v6Pose(v6Pose){ }
	~VertexProperties() { }
	
	TI GetID() const {return m_nID;}
	TooN::Vector<3> GetPosition() const {return m_v3Position; }
	TooN::Vector<6> GetPose() const {return m_v6Pose; }
	
private:
	TI m_nID; // unique ID of each vertex
	TooN::Vector<3> m_v3Position;
	TooN::Vector<6> m_v6Pose;
			
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & m_nID & m_v3Position & m_v6Pose;
    }
};

namespace boost {
namespace serialization {

template<class Archive, int nSize>
void serialize(Archive & ar, TooN::Vector<nSize> & vN, const unsigned int version)
{
	for (int i=0; i < nSize; i++ )
	{
		ar & vN[i];
	}
    //ar & v3[1];
    //ar & v3[2];
}

} // namespace serialization
} // namespace boost

/// properties of edges
class EdgeProperties
{
public:
	EdgeProperties(double dWeight = 1.0) : WEIGHT(dWeight) { }
	~EdgeProperties() {}
	
	double WEIGHT; //weight of each edge, default vaue 1.0, used to find the shortest path between two vertex

private:	
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar &  WEIGHT;
    }
};
 
/// Template of graph class 
/// TV: type of vertiex properties
/// TE: type of edge properties
/// TI: type of vertiex (place/node in VocTree) ID
template <typename TV, typename TE, typename TI>  
class TopoGraph
{
public:	
	typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS, TV, TE > Graph; // Define the type of the graph
	typedef typename boost::graph_traits< Graph >::vertex_descriptor 	Vertex_t;
	typedef typename boost::graph_traits< Graph >::vertex_iterator 	Vertex_Iter;
	typedef typename boost::graph_traits< Graph >::edge_descriptor 	Edge_t;
	typedef typename boost::graph_traits< Graph >::adjacency_iterator 	Adjacency_iterator;
	//typedef typename boost::graph_traits< Graph >::vertices_size_type 	VertexCount;
	typedef std::map< TI, Vertex_t > 	VertexIDMap;
	//typedef std::map< Edge_t, TE> EdgeInfoMap;
	
	typedef typename boost::property_map < Graph, boost::vertex_index_t >::type VertexIndexMap;
	//typedef boost::iterator_property_map < Vertex_t*, VertexIndexMap, Vertex_t, Vertex_t& > PredecessorMap;
	
	TopoGraph(){ ROS_INFO("Create Topological Graph"); }
	~TopoGraph(){ }
	
    /// Add a node to the graph by connecting it to the current active node by in & out edges
    
	bool AddNode(const TV& nodeData, const TE& edgeData_in, const TE& edgeData_out)
	{
		bool bSuccess = true;
		
		Vertex_t newVertex = boost::add_vertex(m_Graph); // add a new vertex to the graph
		
		std::cout << "Add a node ..." << std::endl;
		//std::cout << "ID: " << nodeData.GetID() << std::endl;
		//std::cout << "Position: " << nodeData.GetPosition() << std::endl;
		
		m_Graph[newVertex] = nodeData; // set node properties, needs to define an assignment operator/copy constructor in class TV
		
		if(boost::num_vertices(m_Graph) > 1) // if it's not the first vertex, add an edge to connect the current active vertex
		{
			bSuccess = AddEdge(m_CurrentVertex, newVertex, edgeData_in) && 
							AddEdge(newVertex, m_CurrentVertex, edgeData_out);
		}
			
		m_CurrentVertex = newVertex;
		m_IDMap[nodeData.GetID()] = m_CurrentVertex;	
		
		return bSuccess;
	}
	
	bool AddEdge(Vertex_t vertextStart, Vertex_t vertextEnd, const TE& edgeData)
	{
		Edge_t newEdge;
		bool bSuccess;
		boost::tie(newEdge,bSuccess) = boost::add_edge(vertextStart, vertextEnd, m_Graph); // add an edge
		m_Graph[newEdge] = edgeData; // set edge properties, needs to define an assignment operator/copy constructor in class TE	
	
		std::cout << "Add an edge from V" << m_Graph[vertextStart].GetID() << " to V"<< m_Graph[vertextEnd].GetID() << " W = " << edgeData.WEIGHT << std::endl;
		
		return bSuccess;
	}
	
	bool AddEdge(TI idStart, TI idEnd, const TE& edgeData)
	{
		Vertex_t vertextStart, vertextEnd; 
		try {
			vertextStart = m_IDMap.at(idStart);
			vertextEnd = m_IDMap.at(idEnd);
		}
		catch (const std::out_of_range& oor) {
			std::cerr << "Out of Range (TopoGraph/AddEdge/IDMap) error: " << oor.what() << std::endl;
		}
	
		return AddEdge(vertextStart, vertextEnd, edgeData) && AddEdge(vertextEnd, vertextStart, edgeData);
	}
	
	std::vector<TI> FindPath(TI idStart, TI idEnd)
	{
		std::cout << "Find path from V" << idStart << " to V" << idEnd << " :" << std::endl;
		Vertex_t vertextStart, vertextEnd;
		try {
			vertextStart = m_IDMap.at(idStart);
			vertextEnd = m_IDMap.at(idEnd);
		}
		catch (const std::out_of_range& oor) {
			std::cerr << "Out of Range (TopoGraph/FindPath/IDMap) error: " << oor.what() << std::endl;
		}
	
		std::vector<Vertex_t> path = FindPath(vertextStart, vertextEnd);
		
		std::vector<TI> path_id;
		typename std::vector<Vertex_t>::iterator it; //typename is required because Vertex_t is type dependent
		for (it=path.begin(); it != path.end(); ++it) {
			path_id.push_back(m_Graph[*it].GetID());
		}

		return path_id;
	}
	
	std::vector<Vertex_t> FindPath(const Vertex_t& vStart, const Vertex_t& vEnd)
	{
		// Create things for Dijkstra
		std::vector<Vertex_t> parents(boost::num_vertices(m_Graph)); // To store parents
		std::vector<double> distances(boost::num_vertices(m_Graph)); // To store distances	
		
		VertexIndexMap Vertex_Map = boost::get(boost::vertex_index, m_Graph); 
		
		dijkstra_shortest_paths(m_Graph, vStart,
			boost::weight_map(boost::get(&TE::WEIGHT, m_Graph)).
				predecessor_map(boost::make_iterator_property_map(&parents[0], Vertex_Map)).
				distance_map(boost::make_iterator_property_map(&distances[0], Vertex_Map)) );
				
		std::vector<Vertex_t> path;
		Vertex_t vi = vEnd; 
		while ( vi != vStart) {
			path.push_back(vi);
			vi = parents[vi];
		}
		path.push_back(vStart);
		
		std::reverse(path.begin(),path.end());
			
		PrintPath(path);
			
		return path;		
	}
	
	void PrintPath(std::vector<Vertex_t> path)
	{
		typename std::vector<Vertex_t>::iterator it; //typename is required because Vertex_t is type dependent
		
		for (it=path.begin(); it != path.end(); ++it) {
		
		    std::cout << m_Graph[*it].GetID() << "->";
		}
		std::cout << std::endl;	
	}
	
	std::vector<TI> GetNeighbors(TI idVertex)
	{
		Vertex_t vertext;
		try {
			vertext = m_IDMap.at(idVertex);
		}
		catch (const std::out_of_range& oor) {
			std::cerr << "Out of Range (TopoGraph/GetBeighbors/IDMap) error: " << oor.what() << std::endl;
		}
		
		std::pair< Adjacency_iterator, Adjacency_iterator > neighbors = 
			boost::adjacent_vertices(vertext, m_Graph);
			
		std::vector<TI> vID;
		
		for(; neighbors.first != neighbors.second; ++neighbors.first)
		{
			vID.push_back(m_Graph[*neighbors.first].GetID());
		}
		
		std::cout << "Neighbors of V" << idVertex << ": " ;
		typename std::vector<TI>::iterator it;
		for(it = vID.begin(); it != vID.end(); ++it)
		{
			std::cout << "V" << *it << " ";
		}
		std::cout << std::endl;
		
		return vID;
	}

	void SaveGraph(std::string filename)
	{
		std::ofstream fout(filename.c_str());
		
		if (!fout)
		{
			std::cerr << filename << " could not be opened for writing!" << std::endl;
			return;
		}
		
		boost::archive::text_oarchive ar(fout);	
		try {	
			ar << *this;
		}
		catch (const boost::archive::archive_exception& oor) {
			std::cerr << "Exception in SaveGraph(): " << oor.what() << std::endl;
		}
		
		//boost::write_graphviz(fout, m_Graph);
	}
	
	void LoadGraph(std::string filename) 
	{
		std::ifstream fin(filename.c_str());
		
		if (!fin)
		{
			std::cerr << filename << " could not be opened for reading!" << std::endl;
			return;
		}
 
		boost::archive::text_iarchive ar(fin);
		try {
			ar >> *this;
		}
		catch (const boost::archive::archive_exception& oor) {
			std::cerr << "Exception in LoadGraph(): " << oor.what() << std::endl;
		}
	}
	
	VertexProperties<TI> GetVertexProperties(TI idNode)
	{
		Vertex_t vertext; 
		try {
			vertext = m_IDMap.at(idNode);
		}
		catch (const std::out_of_range& oor) {
			std::cerr << "Out of Range (TopoGraph/GetVertexProperties/IDMap) error: " << oor.what() << std::endl;
		}	
		
		return 	m_Graph[vertext];
	}
	
private:
	Graph m_Graph;
	//VertexCount m_nVertexCount;
	Vertex_t m_CurrentVertex;
	VertexIDMap m_IDMap;
	//EdgeInfoMap m_EdgeInfoMap;


    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, Graph & g, const unsigned int version)
    {
        ar & g.graph();
    }
    
    template<class Archive> 
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & m_Graph & m_IDMap;
    }
};
#endif

/*
 * boost_graphs.h
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#ifndef ATOM_GRAPHS_H_
#define ATOM_GRAPHS_H_

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>
#include <boost/config.hpp>
#include <boost/version.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/static_assert.hpp>


struct EdgeProperties;

using namespace boost;

/* definition of basic boost::graph properties */
enum vertex_properties_t {
	vertex_properties
};
enum edge_properties_t {
	edge_properties
};
namespace boost {
BOOST_INSTALL_PROPERTY(vertex, properties);BOOST_INSTALL_PROPERTY(edge, properties);}

/*
 * Class for sorting the edges according to their angle
 */
template<class T> struct Sorted_list
{
  std::list<T> v;

public:
  typedef T value_type;
  typedef typename std::list<T>::size_type size_type;
  typedef typename std::list<T>::iterator iterator;



  iterator insert(const T& x)
  {
    Sorted_list<T>::iterator i = v.begin();


    string edgePropName = typeid(EdgeProperties).name();
    auto& element_i = (*i).get_property().m_value;
    auto& element_x = x.get_property().m_value;


    //cout <<"angle="<<element.angle<<endl;
    while(i != v.end() && element_x > element_i)
    {
      i++;
    }

    return v.insert(i, x);
  }

  iterator begin() { return v.begin(); }
  iterator end() { return v.end(); }

  size_type size() const { return v.size(); }
};

struct SlistS {};

namespace boost {
  template <class ValueType> struct container_gen<SlistS, ValueType>
  {
    typedef Sorted_list<ValueType> type;
  };

  struct sorted_list_tag {};

  template<class T> sorted_list_tag container_category(Sorted_list<T>&)
  {
    return sorted_list_tag();
  }

  template<class T> std::pair<typename Sorted_list<T>::iterator, bool>
  push_dispatch(Sorted_list<T>& v, const T& x, sorted_list_tag)
  {
    return std::make_pair(v.insert(x), true);
  }

  template <> struct parallel_edge_traits<SlistS> {
    typedef allow_parallel_edge_tag type;
  };
}
/*
 * END OF Class for sorting the edges according to their angle
 */


/* the graph base class template */
template<typename VERTEXPROPERTIES, typename EDGEPROPERTIES>
class Graph {
public:

	/* an adjacency_list like we need it */
	typedef adjacency_list<
			setS, // disallow parallel edges
			//SlistS, //sorted edges by angle?
			listS, // vertex container
			undirectedS, // UNdirected graph
			property<vertex_properties_t, VERTEXPROPERTIES>,
			property<edge_properties_t, EDGEPROPERTIES> > GraphContainer;

	/* a bunch of graph-specific typedefs */
	typedef typename graph_traits<GraphContainer>::vertex_descriptor Vertex;
	typedef typename graph_traits<GraphContainer>::edge_descriptor Edge;
	typedef std::pair<Edge, Edge> EdgePair;

	typedef typename graph_traits<GraphContainer>::vertex_iterator vertex_iter;
	typedef typename graph_traits<GraphContainer>::edge_iterator edge_iter;
	typedef typename graph_traits<GraphContainer>::adjacency_iterator adjacency_iter;
	typedef typename graph_traits<GraphContainer>::out_edge_iterator out_edge_iter;

	typedef typename graph_traits<GraphContainer>::degree_size_type degree_t;

	typedef std::pair<adjacency_iter, adjacency_iter> adjacency_vertex_range_t;
	typedef std::pair<out_edge_iter, out_edge_iter> out_edge_range_t;
	typedef std::pair<vertex_iter, vertex_iter> vertex_range_t;
	typedef std::pair<edge_iter, edge_iter> edge_range_t;

	/* constructors etc. */
	Graph() {
	}

	Graph(const Graph& g) :
			graph(g.graph) {
	}

	virtual ~Graph() {
	}

	/* structure modification methods */
	void Clear() {
		graph.clear();
	}

	Vertex AddVertex(const VERTEXPROPERTIES& prop) {
		Vertex v = add_vertex(graph);
		properties(v) = prop;
		return v;
	}

	void RemoveVertex(const Vertex& v) {
		clear_vertex(v, graph);
		remove_vertex(v, graph);
	}

	EdgePair AddEdge(const Vertex& v1, const Vertex& v2,
			const EDGEPROPERTIES& prop_12, const EDGEPROPERTIES& prop_21) {
		/* TODO: maybe one wants to check if this edge could be inserted */
		Edge addedEdge1 = add_edge(v1, v2, graph).first;
		Edge addedEdge2 = add_edge(v2, v1, graph).first;

		properties(addedEdge1) = prop_12;
		properties(addedEdge2) = prop_21;

		return EdgePair(addedEdge1, addedEdge2);
	}

	EdgePair AddEdge(const Vertex& v1, const Vertex& v2,
			const EDGEPROPERTIES& prop) {
		/* TODO: maybe one wants to check if this edge could be inserted */
		Edge addedEdge = add_edge(v1, v2, graph).first;

		properties(addedEdge) = prop;

		return EdgePair(addedEdge, addedEdge);
	}

	/* property access */
	VERTEXPROPERTIES& properties(const Vertex& v) {
		typename property_map<GraphContainer, vertex_properties_t>::type param =
				get(vertex_properties, graph);
		return param[v];
	}

	const VERTEXPROPERTIES& properties(const Vertex& v) const {
		typename property_map<GraphContainer, vertex_properties_t>::const_type param =
				get(vertex_properties, graph);
		return param[v];
	}

	EDGEPROPERTIES& properties(const Edge& v) {
		typename property_map<GraphContainer, edge_properties_t>::type param =
				get(edge_properties, graph);
		return param[v];
	}

	const EDGEPROPERTIES& properties(const Edge& v) const {
		typename property_map<GraphContainer, edge_properties_t>::const_type param =
				get(edge_properties, graph);
		return param[v];
	}

	/* selectors and properties */
	const GraphContainer& getGraph() const {
		return graph;
	}

	vertex_range_t getVertices() const {
		return vertices(graph);
	}

	edge_range_t getEdges() const {
		return edges(graph);
	}

	adjacency_vertex_range_t getAdjacentVertices(const Vertex& v) const {
		return adjacent_vertices(v, graph);
	}

	int getVertexCount() const {
		return num_vertices(graph);
	}

	int getVertexDegree(const Vertex& v) const {
		return out_degree(v, graph);
	}

	/* operators */
	Graph& operator=(const Graph &rhs) {
		graph = rhs.graph;
		return *this;
	}

protected:
	GraphContainer graph;
};

#endif /* ATOM_GRAPHS_H_ */

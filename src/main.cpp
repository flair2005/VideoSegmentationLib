/*
 * main.cpp
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "atom.h"
//#include "boost_graphs.h"

using namespace boost;

int example_1() {
	// create a typedef for the Graph type
	typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;

	// Make convenient labels for the vertices
	enum {
		A, B, C, D, E, N
	};
	const int num_vertices = N;
	const char* name = "ABCDE";

	// writing out the edges in the graph
	typedef std::pair<int, int> Edge;
	Edge edge_array[] = { Edge(A, B), Edge(A, D), Edge(C, A), Edge(D, C), Edge(
			C, E), Edge(B, D), Edge(D, E) };
	const int num_edges = sizeof(edge_array) / sizeof(edge_array[0]);

	// declare a graph object
	Graph g(num_vertices);

	// add the edges to the graph object
	for (int i = 0; i < num_edges; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, g);

	return 0;
}

//Define a class that has the data you want to associate to every vertex and edge
struct Vertex {
	int foo;
};
struct Edge {
	std::string blah;
};
//Define the graph using those classes
typedef adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
//Some typedefs for simplicity
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor edge_t;

/*
 * AtomGraph
 */
struct AtomVertex {
	videoseg::Atom atom;
};
//Define the graph using those classes
typedef adjacency_list<boost::listS, boost::vecS, boost::directedS, AtomVertex,
		Edge> AtomGraph;
//Some typedefs for simplicity
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor edge_t;

void example_0() {

	//Instanciate a graph
	Graph g;

	// Create two vertices in that graph
	vertex_t u = add_vertex(g);
	vertex_t v = add_vertex(g);

	// Create an edge conecting those two vertices
	edge_t e;
	bool b;
	tie(e, b) = boost::add_edge(u, v, g);

	// Set the properties of a vertex and the edge
	g[u].foo = 42;
	g[e].blah = "Hello world";
}

void example_2() {

	//Instanciate a graph
	Graph g;

	// Create two vertices in that graph
	vertex_t u = add_vertex(g);
	vertex_t v = add_vertex(g);

	// Create an edge conecting those two vertices
	edge_t e;
	bool b;
	tie(e, b) = boost::add_edge(u, v, g);

	// Set the properties of a vertex and the edge
	g[u].foo = 42;
	g[e].blah = "Hello world";
}

int main(int, char*[]) {

	example_2();
}


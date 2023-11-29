#ifndef GRAPH_HPP
# define GRAPH_HPP

# include <vector>
# include <iostream>

typedef std::vector<std::vector<std::pair<size_t, double>>>	adjList_t;

class Graph {

private:
	size_t		_nbVertices;
	adjList_t	_adjList;

public:
	Graph();
	Graph(Graph const & other);

	Graph&	operator=(Graph const & other);

	size_t		getNbVertices() const;
	adjList_t	getAdjList() const;
	void 		setNbVertices(size_t nbVertices);
	void		addEdge(size_t right, size_t left, double range);
	size_t 		addVertex();
	void 		printGraph();
	void		clear();
};

#endif
/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Graph.cpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: edelage <edelage@student.42lyon.fr>        +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/11/12 16:16:00 by edelage           #+#    #+#             */
/*   Updated: 2023/11/12 16:16:00 by edelage          ###   ########lyon.fr   */
/*                                                                            */
/* ************************************************************************** */
#include "Graph.hpp"

Graph::Graph(size_t nbVertices) : _nbVertices(nbVertices), _adjList(nbVertices) {}

Graph::Graph(const Graph& other) :
	_nbVertices(other._nbVertices),
	_adjList(other._nbVertices) {
	for (size_t i = 0; i != other._nbVertices; ++i) {
		for (auto edge : other._adjList[i]) {
			_adjList[i].push_back(edge);
		}
	}
}

void Graph::addEdge(size_t right, size_t left, double range) {
	std::pair<int, double>	dest;

	dest.first = left;
	dest.second = range;
	_adjList[right].push_back(dest);
	dest.first = right;
	_adjList[left].push_back(dest);
}

/**
 * @return Index of the new vertex
 */
size_t Graph::addVertex() {
	std::vector<std::pair<int, double>>	vertexAdjList;

	_adjList.push_back(vertexAdjList);
	++_nbVertices;
	return (_nbVertices - 1);
}

size_t Graph::getNbVertices() const {return (_nbVertices);}

adjList_t Graph::getAdjList() const {return (_adjList);}

void Graph::printGraph() {
	for (size_t i = 0; i < _nbVertices; ++i) {
		std::cout << "Adjacency list of vertex " << i << ": ";
		for (const auto &neighbor : _adjList[i]) {
			std::cout << '[' << neighbor.first << ": " << neighbor.second << "], ";
		}
		std::cout << std::endl;
	}
}

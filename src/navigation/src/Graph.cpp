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

Graph::Graph(size_t nbVertices) : _nbVertices(nbVertices), _adjList(nbVertices) {
}

void Graph::addEdge(size_t right, size_t left, double range) {
	std::pair<int, double>	dest;

	dest.first = left;
	dest.second = range;
	_adjList[right].push_back(dest);
	dest.first = right;
	_adjList[left].push_back(dest);
}

size_t Graph::getNbVertices() const {return (_nbVertices);}

std::vector<std::vector<std::pair<int, double>>> Graph::getAdjList() const {return (_adjList);}

void Graph::printGraph() {
	for (size_t i = 0; i < _nbVertices; ++i) {
		std::cout << "Adjacency list of vertex " << i << ": ";
		for (const auto &neighbor : _adjList[i]) {
			std::cout << '[' << neighbor.first << ": " << neighbor.second << "], ";
		}
		std::cout << std::endl;
	}
}

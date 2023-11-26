#include "Pathfinding.hpp"
#include <algorithm>
#include <limits>

std::vector<std::pair<size_t, double>> Pathfinding::djikstra(size_t start, size_t end, Graph const & graph) {
	adjList_t 								adjList;
	std::vector<size_t>						visited(1, start);
	std::pair<size_t, double>				current(start, 0);
	std::vector<std::pair<size_t, double>>	path(graph.getNbVertices(), std::pair<int, double>(-1, std::numeric_limits<double>::infinity()));

	adjList = graph.getAdjList();
	path[start] = current;
	while (current.first != end) {
		for (auto node : adjList[current.first]) {
			if (isVisited(node.first, visited) || node.first == current.first)
				continue;
			if (current.second + node.second < path[node.first].second)
				path[node.first] = std::pair<int, double>(current.first, current.second + node.second);
		}
		current = getMinNode(path, visited);
		visited.push_back(current.first);
	}
	return (path);
}

bool Pathfinding::isVisited(size_t index, const std::vector<size_t>& visited) {
	return (std::find(visited.begin(), visited.end(), index) != visited.end());
}

std::pair<size_t, double> Pathfinding::getMinNode(
	std::vector<std::pair<size_t, double>> const & path,
	std::vector<size_t> const & visited) {

	double					minDist = std::numeric_limits<double>::infinity();
	std::pair<int, double>	min;
	size_t					index = 0;

	for (auto node : path) {
		if ((node.second < minDist || minDist == std::numeric_limits<double>::infinity())
			&& !isVisited(index, visited)) {
			min.second = node.second;
			min.first = index;
			minDist = min.second;
		}
		++index;
	}
	return (min);
}

std::vector<point_t> Pathfinding::convertDjikstraToPoint(std::vector<std::pair<size_t, double>> reversePath, size_t start, size_t end) {
	std::pair<size_t, double>				current;
	std::list<size_t>						path;

	current = reversePath[end];

	path.push_front(_buoy.graphIndex);
	while (current.first != start) {
		path.push_front(current.first);
		current = reversePath[current.first];
	}
	return (convertNodeToPoint(path));
}

std::vector<point_t> Pathfinding::convertNodeToPoint(std::list<size_t> nodePath) {
	std::vector<point_t>	path;

	for (auto node : nodePath) {
		if (node != _buoy.graphIndex)
			path.push_back(_obstacles[node / 4].point[node % 4]);
		else
			path.push_back(_buoy.position);
	}
	return (path);
}

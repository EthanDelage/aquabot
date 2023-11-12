/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Graph.hpp                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: edelage <edelage@student.42lyon.fr>        +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/11/12 16:16:00 by edelage           #+#    #+#             */
/*   Updated: 2023/11/12 16:16:00 by edelage          ###   ########lyon.fr   */
/*                                                                            */
/* ************************************************************************** */
#ifndef GRAPH_HPP
# define GRAPH_HPP

# include <iostream>
# include <vector>

class Graph {

private:
	const size_t										_nbVertices;
	std::vector<std::vector<std::pair<int, double>>>	_adjList;

public:
	Graph(size_t nbVertices);

	size_t												getNbVertices() const;
	std::vector<std::vector<std::pair<int, double>>>	getAdjList() const;
	void												addEdge(size_t right, size_t left, double range);
	void 												printGraph();
};

#endif
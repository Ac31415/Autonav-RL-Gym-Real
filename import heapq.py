import heapq

def dijkstra(graph, start, end):
    # Initialize distances and visited set
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    visited = set()

    # Priority queue to store nodes to visit
    queue = [(0, start)]

    while queue:
        # Pop the node with the smallest distance
        current_distance, current_node = heapq.heappop(queue)

        # Skip if already visited
        if current_node in visited:
            continue

        # Mark node as visited
        visited.add(current_node)

        # Check if reached the end node
        if current_node == end:
            break

        # Explore neighbors
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight

            # Update distance if shorter path found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))

    # Return the shortest distance
    return distances[end]

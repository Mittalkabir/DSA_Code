# Dijkstra's algorithm (Python) — easy explanation + code

**What it does (in simple words):**
Dijkstra finds the shortest path (minimum total weight) from a starting node to every other node in a graph that has **non-negative** edge weights.

Think of it like this: you’re at a station and want the quickest way to every other station. Dijkstra explores the nearest unvisited station first, updates distances, and repeats.

---

## Key ideas (short)

* Keep the **best known distance** from the start to every node (initially `∞` except start = 0).
* Use a **priority queue** (min-heap) to always take the node with the smallest known distance next.
* For that node, try to **relax** each outgoing edge (see if going through this node gives a shorter path).
* Repeat until all reachable nodes are finalized.

**Important:** Works only when edge weights are **non-negative**.

---

## Python implementation (adjacency list + path reconstruction)

```python
import heapq
from typing import Dict, List, Tuple, Any, Optional

def dijkstra(graph: Dict[Any, List[Tuple[Any, float]]],
             start: Any
            ) -> Tuple[Dict[Any, float], Dict[Any, Optional[Any]]]:
    """
    graph: adjacency list where graph[u] = [(v, weight), (v2, weight2), ...]
    start: starting node

    Returns:
      distances: dict of shortest distances from start to each node (inf if unreachable)
      prev: dict to reconstruct shortest paths; prev[node] is the previous node on the path
    """
    # Initialize
    INF = float('inf')
    distances = {node: INF for node in graph}
    prev = {node: None for node in graph}
    distances[start] = 0

    # Min-heap of (distance, node)
    heap = [(0, start)]
    visited = set()

    while heap:
        dist_u, u = heapq.heappop(heap)

        # If we popped a stale pair (larger than best-known), skip
        if dist_u > distances[u]:
            continue

        # Optional: mark visited (not strictly necessary with the stale check)
        if u in visited:
            continue
        visited.add(u)

        # Relax neighbors
        for v, weight in graph.get(u, []):
            if weight < 0:
                raise ValueError("Dijkstra's algorithm cannot handle negative weights.")

            new_dist = dist_u + weight
            if new_dist < distances.get(v, INF):
                distances[v] = new_dist
                prev[v] = u
                heapq.heappush(heap, (new_dist, v))

    return distances, prev

def reconstruct_path(prev: Dict[Any, Optional[Any]],
                     start: Any,
                     end: Any) -> List[Any]:
    """
    Reconstructs path from start to end using prev map returned by dijkstra.
    Returns empty list if end is unreachable.
    """
    path = []
    cur = end
    while cur is not None:
        path.append(cur)
        if cur == start:
            break
        cur = prev.get(cur)
    path.reverse()
    if not path or path[0] != start:
        return []   # unreachable
    return path

# Example usage
if __name__ == "__main__":
    # directed graph example (can be used as undirected by adding both directions)
    graph_example = {
        'A': [('B', 5), ('C', 1)],
        'B': [('A', 5), ('C', 2), ('D', 1)],
        'C': [('A', 1), ('B', 2), ('D', 4), ('E', 8)],
        'D': [('B', 1), ('C', 4), ('E', 3)],
        'E': [('C', 8), ('D', 3)]
    }

    start_node = 'A'
    distances, prev = dijkstra(graph_example, start_node)

    print("Shortest distances from", start_node)
    for node in sorted(distances):
        print(f"  {node}: {distances[node]}")

    # reconstruct path from A to E
    path_A_to_E = reconstruct_path(prev, start_node, 'E')
    print("Path from A to E:", path_A_to_E)
```

---

## Walkthrough on the example graph

Graph edges (weights):

* A → B (5), A → C (1)
* B → C (2), B → D (1)
* C → D (4), C → E (8)
* D → E (3)

Start from `A`:

1. Distances init: A=0, B=∞, C=∞, D=∞, E=∞. Heap: (0, A)
2. Pop A (0). Relax neighbors:

   * C: 0+1 = 1 → update C=1, prev[C]=A
   * B: 0+5 = 5 → update B=5, prev[B]=A
     Heap: (1,C), (5,B)
3. Pop C (1). Relax neighbors:

   * B: 1+2 = 3 → update B=3, prev[B]=C
   * D: 1+4 = 5 → update D=5, prev[D]=C
   * E: 1+8 = 9 → update E=9, prev[E]=C
     Heap: (3,B), (5,B stale), (5,D), (9,E)
4. Pop B (3). Relax neighbors:

   * D: 3+1 = 4 → update D=4, prev[D]=B
     Heap: (4,D), (5,D stale), (9,E)
5. Pop D (4). Relax neighbors:

   * E: 4+3 = 7 → update E=7, prev[E]=D
     Heap: (7,E), (9,E stale)
6. Pop E (7). No neighbors improve. Done.

Final distances: A=0, B=3, C=1, D=4, E=7. Path A→E is `A -> C -> B -> D -> E`? Actually reconstructing prev gives A→C→B→D→E only if prev mapping matched—(in our run prev[B]=C, prev[D]=B, prev[E]=D) → path A→C→B→D→E.

---

## Time & space complexity

* Time: `O((V + E) log V)` using a binary heap (V = nodes, E = edges). Practically often written `O(E log V)`.
* Space: `O(V)` for distances + `O(E)` for the graph storage.

---

## Notes & tips

* If your graph is **undirected**, add edges in both directions when building the adjacency list.
* For graphs with **negative weights**, use Bellman–Ford instead.
* If you need the *shortest path between just two nodes* and want more speed in some cases, there are optimized or bidirectional versions — but classic Dijkstra is simple and reliable for most uses.


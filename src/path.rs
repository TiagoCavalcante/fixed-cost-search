use graphs::WeightedGraph;
use ordered_float::NotNan;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

/// This is used only in the binary heap, if we could create
/// a heap with a custom sorting key we would do.
#[derive(Eq)]
struct Vertex {
  index: usize,
  cost: NotNan<f32>,
}

impl PartialEq for Vertex {
  fn eq(&self, other: &Self) -> bool {
    (self.index, self.cost) == (other.index, other.cost)
  }
}

impl Ord for Vertex {
  fn cmp(&self, other: &Self) -> Ordering {
    // Smaller costs first.
    other.cost.cmp(&self.cost)
  }
}

impl PartialOrd for Vertex {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

/// Test whether `a` differs from `b` for a amount that is
/// less than `threshold`.
fn equal(a: f32, b: f32, threshold: f32) -> bool {
  (a - b).abs() < threshold
}

/// Returns whether `vertex` is in the path to the `to`
/// vertex given the `predecessor` vector.
fn in_start_path(
  predecessor: &Vec<usize>,
  to: usize,
  vertex: usize,
) -> bool {
  let mut current = to;
  while predecessor[current] != usize::MAX {
    current = predecessor[current];
    if current == vertex {
      return true;
    }
  }
  return false;
}

/// Returns whether both paths share a vertex.
/// `predecessor_list` is a vector with only the
/// predecessors of a given node, while `predecessors` is
/// a predecessor vector of all vertices in the graph, so
/// we need to iterate over it to find a path.
/// We start in iterating over `predecessors` in `start`.
///
/// Note that it uses `current` as the 0th vertex of the
/// `predecessors`, not `predecessors[current]`.
fn shared_paths(
  predecessor_list: &Vec<usize>,
  predecessors: &Vec<usize>,
  current: usize,
) -> bool {
  let mut current = current;

  while current != usize::MAX {
    if predecessor_list
      .iter()
      .any(|&vertex| vertex == current)
    {
      return true;
    }
    current = predecessors[current];
  }

  return false;
}

/// Fixed cost search algorithm.
/// For understanding this algorithm I recommend you to
/// study first how the BFS algorithm works.
/// See https://en.wikipedia.org/wiki/Breadth-first_search
///
/// The idea behind this algorithm is to first find the
/// shortest path from the start to then end, and then make
/// the reverse path trying to increase its cost, but
/// without exceding the desired cost, and stop when a
/// path with the desired cost is reached.
/// ```
/// let path = path::fixed_cost_search(
///   &graph,
///   start,
///   end,
///   cost,
///   threshold
/// );
/// println!("{:?}", path.unwrap_or(vec![]));
/// ```
pub fn fixed_cost_search(
  graph: &WeightedGraph,
  start: usize,
  end: usize,
  cost: f32,
  threshold: f32,
) -> Option<Vec<usize>> {
  // Predecessor vector as in a normal Dijkstra algorithm.
  let mut predecessor_from_start =
    vec![usize::MAX; graph.size];
  // Distance vector as in a normal Dijkstra algorithm.
  let mut distance_to_start = vec![f32::MAX; graph.size];

  // Differently from the BFS algorithm we need to keep the
  // predecessors from both the start and the end.
  // Also differently from the BFS algorithm we save the
  // predeecessors of each vertex in its own array, this is
  // necessary to avoid paths with wrong costs because
  // another iteration has modified the predecessors of a
  // vertex.
  let mut predecessor_from_end = vec![vec![]; graph.size];
  let mut distance_to_end = vec![f32::MAX; graph.size];

  // A heap to maintain the vertices whose adjacency list
  // is to be scanned as per normal Dijkstra algorithm.
  let mut heap = BinaryHeap::from(
    (0..graph.size)
      .map(|index| Vertex {
        index,
        cost: NotNan::new(f32::MAX).unwrap(),
      })
      .collect::<Vec<_>>(),
  );
  let mut in_heap = vec![true; graph.size];

  // The distance from the start to itself is 0.
  distance_to_start[start] = 0.0;

  // Standard Dijkstra algorithm.
  // See https://en.wikipedia.org/wiki/Breadth-first_search.
  // Note that in the Dijkstra algorithm the queue must be
  // first in first out.
  while let Some(current) = heap.pop() {
    in_heap[current.index] = false;

    // Possible optimization for graphs where all vertex are
    // reachable from the start: keep count on how many
    // vertices were visited and stop once that number is
    // equal to the total number of vertices.
    for &(vertex, weight) in
      graph.get_neighbors(current.index)
    {
      // If the distance is usize::MAX then that vertex was
      // never reached before.
      if in_heap[vertex] {
        let distance =
          distance_to_start[current.index] + weight;

        if distance < distance_to_start[vertex] {
          distance_to_start[vertex] = distance;
          predecessor_from_start[vertex] = current.index;
        }
      }
    }
  }

  // Return early if this node can't be reached or if its
  // shortest path cost is bigger than the desired cost.
  // Note that we don't need to directly check if
  // distance_to_start[end] == usize::MAX because if it is
  // equal to usize::MAX then it is bigger than the
  // distance.
  if distance_to_start[end] >= cost + threshold {
    return None;
  }

  let mut queue = VecDeque::new();

  distance_to_end[end] = 0.0;
  // Here we are starting from the end and going to the
  // start.
  queue.push_front(end);

  // Here the magic happens.
  // Instead of finding the smallest path we are trying to
  // find the biggest path that is no bigger than the
  // cost.
  // We want it to be exactly equal to the cost, but we
  // won't get there so easy.
  // In the first versions of this algorithm the queue
  // needed to be first in last out, but in the latest
  // version it doesn't need to be anymore.
  while let Some(current) = queue.pop_front() {
    for &(neighbor, weight) in graph.get_neighbors(current)
    {
      // If we never visited this vertex or the size of the
      // path is bigger than the last path but still not
      // bigger than the cost and that neighbor is not in
      // the path to the current vertex.
      // Note: if the vertex has no predecessors then it
      // was never reached.
      if (distance_to_end[neighbor] == f32::MAX
        || (distance_to_end[current] + weight
          > distance_to_end[neighbor]
          && distance_to_end[current]
            + distance_to_start[neighbor]
						+ weight
            < cost + threshold))
        // If it is already in path then we won't go to
        // this neighbor as we can't use any vertex more
        // than once.
        // && !in_end_path(&predecessor_from_end, current, neighbor)
        // The check above is implicity in shared_paths.
        // The contrary may also happen.
        && !in_start_path(&predecessor_from_start, neighbor, current)
        // This is the slowest test, but if we remove this
        // the algorithm may fail in small graphs.
        // Possible optimization: Move the above check to
        // inside the shared_paths function.
        && !shared_paths(&predecessor_from_end[current],&predecessor_from_start, neighbor)
      {
        // The code bellow is equivalent to:
        // predecessor_from_end[neighbor] =
        //   current_path + current;
        predecessor_from_end[neighbor].clear();
        let current_path =
          predecessor_from_end[current].clone();
        predecessor_from_end[neighbor].extend(current_path);
        predecessor_from_end[neighbor].push(current);

        distance_to_end[neighbor] =
          distance_to_end[current] + weight;

        if equal(
          cost,
          distance_to_start[neighbor]
            + distance_to_end[neighbor],
          threshold,
        ) {
          // First find the path between the end and the
          // current vertex.
          let mut path =
            predecessor_from_end[neighbor].clone();

          // Then append the path between the current vertex
          // and the start.
          let mut current = neighbor;

          path.push(current);

          while predecessor_from_start[current]
            != usize::MAX
          {
            current = predecessor_from_start[current];
            path.push(current);
          }

          // And then reverse the path.
          path.reverse();

          return Some(path);
        }

        // Using push_front here instead of push_back makes
        // the algorithm up to 3x faster for big costs.
        queue.push_front(neighbor);
      }
    }
  }

  return None;
}

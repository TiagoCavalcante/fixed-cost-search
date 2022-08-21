use graphs::WeightedGraph;
use std::time::Instant;

mod path;

fn main() {
  let size = 10_000;
  let start = 0;
  let end = 1;
  let cost = 10.0;
  let threshold = 0.01;

  let mut graph = WeightedGraph::new(size);

  let now = Instant::now();
  graph.fill_undirected(0.1, 1.0, 0.5);
  println!("Fill the graph - {:.2?}", now.elapsed());

  let now = Instant::now();
  let path = path::fixed_cost_search(
    &graph, start, end, cost, threshold,
  );
  println!("Fixed cost search - {:.2?}", now.elapsed());

  // Test if the path is valid.
  if let Some(path) = path {
    assert_eq!(*path.first().unwrap(), start);
    assert_eq!(*path.last().unwrap(), end);

    let mut measured_cost = 0.0;

    // Check if the path is made only by real edges.
    for index in 0..path.len() - 1 {
      if let Some(weight) =
        graph.get_edge(path[index], path[index + 1])
      {
        measured_cost += weight;
      } else {
        panic!("The path contains unvalid edges");
      }
    }

    assert!((cost - measured_cost).abs() < threshold);

    // Ensure that the path contain no loops.
    let mut unique = path.clone();
    // We need a sorted vector to use dedup.
    unique.sort();
    unique.dedup();
    // If the path had loops then the length of the unique
    // vector would be smaller than the length of the path.
    assert_eq!(path.len(), unique.len());

    println!("The path is valid");
  } else {
    panic!("Couldn't find a valid path")
  }
}

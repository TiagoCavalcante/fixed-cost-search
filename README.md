# fixed-cost-search

Fixed cost search on weighted graphs

## How to run?

```sh
$ cargo run --release
```

## How fast is it?

Here is the output of the benchmark of the algorithm for a graph with 10 thousand vertices, density of 0.1 and cost 10.0 (with a maximum error of 0.01):
```
Fill the graph - 658.09ms
Fixed cost search - 23.04ms
The path is valid
```

Yep, that is milliseconds, not seconds.

## Where did this come from?

This is a modification of the [fixed length search](https://github.com/TiagoCavalcante/fixed-length-search) for weighted graphs.

## Directed graphs?

This algorithm can be easily adaptated to directed graphs just by switching the `get_neighbors` by a `is_neghbored_by_these` or equivalent.

## Best path?

This algorithm can also be easily updated so instead of stoping when the threshold is reached, only stop when the best path is found. To do this you just need to remove the threshold and save the path with the closest cost.

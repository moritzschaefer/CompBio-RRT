# Algorithm

## RRTCon

We use RRTCon as base for two reasons:

1. It's the only one of the provided one, finding paths between start and goal.
2. It has a good trade off between extending our start tree (which is the only tree) and looking if we can connect our tree to the goal (5% of the time) and as such doesn't waste time on extending the goal-tree.

To further improve our algorithm and exploit knowledge about the environment we improved RRTCon in several ways:


## Extension

While the strategy of RRTCon of only using one Tree, is advantegous in our environment, sampling either inside the protein or outside of it (but NOT inside the pocket) will lead to collisions. As such we always use extensions instead of connections except when we try to connect to the goal and get fewer collisions when building the tree inside of the pocket.

## Sampling

Finding a path to the goal when we escaped out of the pocket is fairly simple. As such it is not necessary to sample too much outside of the pocket and we use a gaussian distribution. The mean is picked depending on the last node we created and the neighbour it was connected to. Knowning that we could connect into a given direction (from the last neighbour to the node it was connected to), we assume that we can go further in that direction by the same distance and picked that point as the mean. We take a relatively small variance when sampling. If an extend-step results in a collision we increase the sampling-variance. If an extend-step succeeds, we halve the sampling-variance. Using this strategy we found a good balance between exploration and exploitation.

## Recovering

If the variance gets too high we choose another point as the mean for the sampling and reset the variance to a low value. We choose a point close to the goal with a high probability. When we can go in straight line (e.g. when we escaped from the pocket) our sampling variance will become very small (as it halfs on each successfull extend step). We use this sign as a signal to search the space very broadly and set the variance to a very high value to find the goal as quickly as possible.

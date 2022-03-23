# OMPRRT: Open-MP based rapidly exploring random trees 
### *(pronounced Ompert. Like Robert with Omp.)*

## Summary
We plan to implement Rapidly Exploring Random Trees via different parallelization techniques. Using OpenMP, we hope to explore various strategies and compare performance.

## Background
[Rapidly exploring random trees (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) is an algorithm commonly used in path-planning. They are often used to simplify the search of a massive state space that would be too slow/inefficient to explore completely. Since RRTs are often deployed in real time robotics applications, their runtime speed optimization is critical. RRTs work by randomly exploring nodes and checking for collissions and there is massive potential for parallelization in exploring this state space. 

There are a few outlets that we could take when trying to parallelize RRTs. To start, we can add and process multiple nodes to the tree at the same time. Or, we can parallelize individual steps within the loop, such as finding the closest node/nodes in the tree to a certain point in the workspace when 'pruning' the tree. Or a combination of both.

## The Challenge
Graphs/trees have inherently poor spatial locality. This means that we will quickly approach memory bottlenecks when parallelizing; it will be interesting to see how various approaches best tackle this.

It will also be a challenge to effectively profile our code, since the algorithm has lots of little steps that can be parallelized, and hence must be timed/tested individually to see which combination works best.

We might also run into work allocation imbalances resulting from an asymmetrical workspace (from the randomness in the trees, and also in obstacles)—this could result in some workers processing vastly more nodes than others.

## Resources
The only resources we will need is CPU time on the PSC machines. That way, we can evaluate the performance of our approaches over a wide variety of worker counts.

## Goals and Deliverables

### 100% (Main goal)
- Implement RRT* and visualizer and a few heuristics
- Parallelize node generation
- Parallelize tree pruning
- Parallelize by segmenting workspace into a grid
- Analyze performance of each of these approaches and heuristics 
### 125% (Stretch goal)
- Implement GPU-based approach using CUDA

### 75% (Minimal product)
- Sucessfully implement two of the three pathways and analyze their performance

### Demo 
Lots of graphs detailing various speedups for each parallel approach and various parameters (thread count, window size, tree pruning frequency, etc.)

We would also love to show a working visualizer for OMPRRT in the demo (*OMP-art*).

## Platform Choice
We chose OpenMP because it serves as a frictionless pathway to parallelization. This is useful since multicore is getting cheaper and more power-efficient; so, such parallelization could make RRTs more feasible in an online setting (locally computed on the robot itself in real-time).

## Schedule
**Week 1:** 
- Sequential implementation of RRT*

**Week 2:** 
- One parallel approach to RRT*
- Work on visualizer

**Week 3:** 
- Second and third parallel approaches to RRT*

**Week 4:**
- Parameter-tuning
- Analysis

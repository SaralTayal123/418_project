# OMPRRT: Open-MP based rapidly exploring random trees 
### *(pronounced Ompert. Like Robert with Omp.)*

## Final Report

The final report is available at the 'Final_report.pdf' file in the root of the repository. 

Our presentation is available here as an unlisted YouTube video: https://youtu.be/GD27k0zCJVk


## Milestone Report

So far we have gotten our implementation of RRT an d RRT* working in serial execution. We have also developed a method for storing our graphs and the best path it takes in a standard data format. We have also started work on a web based visualizer for the graph to help us visually debug/demonstrate an optimal path and the statespace that the algorithm has found. 

We are perfectly on track to hitting our 100% deliverable and are currently around the end of week 2 of our 4 week plan. It is still difficult to comment on the strech goal target since we haven't fully started exploring avenues of parallelizatin that we intend to explore and upon doing that later this week, we hope to have a better sense of understanding for how far we can expect to get with our stretch goals

As for what we intend to demo, we intend to demo various visualizations of the output of the algorithm (state space & optimal path) and also display graphs for speedup vs cores for the various parallelization techniques we explore. 

We currently don't have any preliminary results as of now other than a working serial implementation. This is simply due to how we structured the timeline of the project. Additionally, there are no major concerns we have for the project.


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

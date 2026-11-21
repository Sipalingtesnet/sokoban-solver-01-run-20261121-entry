# Sokoban Solver (push-optimal)

A push-optimal Sokoban solver for the make.fun competition
`sokoban-solver-01-run-20261121` ("Build the best Sokoban solver, 21 November 2026").

## What it does

The solver reads XSB levels, finds the central hallway, decomposes the level into
rooms (one per door off the hallway), and solves each room **push-optimally** with
A* over push states. It uses an admissible minimum-cost box-to-goal assignment
heuristic, dead-square pruning via reverse reachability from goals, and 2x2-freeze
pruning. Between pushes it expands the push sequence into full player walks with
deterministic BFS. If decomposition fails it falls back to solving the whole level
as a single room. Every instance always gets a solution file written.

## Badges

- [Competition badge](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)
- [Rank badge](https://make.fun/badge/sokoban-solver-01-run-20261121/Sipalingtesnet/sokoban-solver-01-run-20261121-entry/rank.svg)
- [Reward badge](https://make.fun/badge/sokoban-solver-01-run-20261121/Sipalingtesnet/sokoban-solver-01-run-20261121-entry/reward.svg)

## Bounty

https://make.fun/bounty/sokoban-solver-01-run-20261121
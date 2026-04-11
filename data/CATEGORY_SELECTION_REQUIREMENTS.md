# Category Selection Requirements

This document defines the bar for adding or keeping categories in the current
repo taxonomy under `data/categories/`.

A category is not just an interesting prompt idea.
It is a durable taxonomy entry that should support repeated prompt batches,
multiple believable records, and a clear `target_sdk_version`.

## Core Standard

A good category should be:

- representable as a believable articulated object with explicit URDF joints
- buildable reliably with `sdk`
- specific enough to have a clear object identity
- broad enough to support more than a single one-off design
- distinct from categories we already have

If a category fails any of those tests, it is probably the wrong taxonomy unit.

## 1. Articulation Fit

The category must map cleanly to the articulation model supported by this repo:

- rigid parts connected by explicit joints
- `revolute`, `continuous`, `prismatic`, or `fixed` motion
- tree-structured kinematics

Reject categories whose defining behavior depends on mechanisms we cannot
represent honestly with explicit tree joints, including:

- closed kinematic loops when the loop is essential
- cable, belt, or chain coupling
- pulley routing
- gear coupling that requires linked motion constraints
- helical screw motion as the primary defining mechanism
- ratcheting, indexing, or intermittent locking as the main behavior
- contact-driven motion that only works through simulation tricks

Examples of risky or usually bad fits:

- true four-bar or scissor mechanisms when the closed loop is the whole point
- ratchet-and-pawl devices
- Geneva drives
- cable-and-pulley lifts
- differentials
- true cam-follower systems

If we would need to fake the defining motion, it should not be a category.

## 2. SDK Fit

The category must be practical to author with the tools and geometry patterns
the repo already supports.

Good fits usually have:

- explicit rigid members, housings, shells, rails, hubs, brackets, forks,
  yokes, arms, trays, panels, or frames
- hard-surface geometry with clear support relationships
- readable parent/child structure
- geometry that can be built from primitives, profiles, sweeps, revolves,
  booleans, and straightforward CadQuery solids

Bad fits usually depend on:

- soft-body behavior
- cloth, cables, springs, chains, or deformable materials as the main identity
- highly organic freeform surfacing
- exotic lofted bodywork or compound sculpted shells as the main visual story
- mechanisms that only work with special hacks or coupled constraints

If a category would be fragile in compile, QC, or testing even when prompted
well, it is not a good category for this repo.

## 3. Category Scope

A category should be a stable object class or mechanism class, not a vague
umbrella and not a microscopic variant.

Good category scope:

- clearly names one coherent class of object or mechanism
- has a recognizably shared structure across examples
- supports multiple prompt variants without changing the core identity
- has a predictable set of common-sense articulations

Bad category scope:

- too broad: mixes several object families that want different prompts and
  structures
- too narrow: basically one exact design instance disguised as a category
- too ambiguous: the likely outputs would vary wildly in topology or support
  structure

If you cannot picture several believable examples that still read as the same
category, the scope is wrong.

## 4. Distinctness

A new category must earn its place in the taxonomy.
It should not just be a rename or a thin extension of something we already
cover.

Usually reject categories that are only:

- an existing category with one extra joint
- a count variant of an existing topology
- a subset of an existing category
- an attachment variant of an existing mechanism
- a relabeling of an already-covered object class
- an umbrella name that overlaps heavily with several existing categories

A strong addition usually introduces at least one of:

- a meaningfully different articulation topology
- a meaningfully different support structure
- a meaningfully different mechanism archetype
- a clearly different object identity
- a different prompt regime that deserves its own durable category

## 5. Current Repo Placement

Think in terms of current category metadata, not legacy tracker blocks.

When a category is accepted, it should be reasonable to assign:

- a stable slug
- a readable title
- a coherent prompt batch strategy
- a `target_sdk_version`

Current default mapping:

- use `base` for object-first categories where product identity, housing, body
  shell, and visible finished form matter more than CAD-style mechanism detail
- use `base` for all current categories in this repo

## Decision Checklist

Before adding a category, check:

1. Is the defining motion representable with explicit URDF joints and tree
   kinematics?
2. Can we build multiple believable examples cleanly with `sdk`?
3. Is the category scope stable and coherent?
4. Is it meaningfully distinct from the categories already under
   `data/categories/`?
5. Can we assign a clear `target_sdk_version` based on how the geometry should
   actually be authored?
6. Would this category still make sense after several prompt batches and
   several accepted records, not just one prompt?

If any answer is no, the category should usually be rejected, merged into an
existing category, or renamed to a better taxonomy unit.

## Practical Defaults

- Prefer durable object or mechanism classes over clever one-off concepts.
- Prefer categories with clear support structure and common-sense articulation.
- Prefer categories that can produce several good prompts, not one lucky prompt.
- Prefer categories whose main geometry is achievable with the current SDK
  stack.
- When in doubt, choose the simpler, clearer category boundary.

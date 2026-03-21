# Category Prompt Guide

This document defines how to write category prompts for prompt batches under
`data/`.

The goal is to produce detailed, realistic articulated objects that are
visually strong, mechanically plausible, and achievable with `sdk` or
`sdk_hybrid`.

The prompt should push the agent toward visually rich, believable results, not
just technically valid ones.

## Core Standard

A good category prompt should:

- describe a recognizable object in concrete visual terms
- name the main rigid structures and support relationships
- include the common-sense articulations a believable instance should have
- push toward visually rich, believable object design
- stay inside geometry and mechanisms our SDK stack can actually author

We do not want bare mechanism sketches.
We also do not want prompts that depend on exotic sculpting or fragile hacks.

## Prompt Policy

- Describe the object, not just the joints.
- Include articulation as part of the object design.
- Do not artificially reduce a category to one motion if the real object
  normally has several canonical articulations.
- Do not add obscure or gimmicky motions that are not central to the category.
- Choose one coherent variant when the category has several common forms.

## What To Include

Every category prompt should cover:

- the object type and chosen variant
- the main body, frame, housing, or support structure
- three to six concrete visual cues
- material or finish cues when they help the object read correctly
- the standard articulations for that object
- the supports, hinges, rails, forks, hubs, brackets, or guides for those
  motions
- a short realism and buildability clause

## Visual Detail

Prompts should be fairly detailed.
Give enough structure that the agent does not have to invent the core design.
They should encourage rich, believable visual form, not bare schematic output.

Good detail usually includes:

- silhouette and overall proportions
- body shell, frame, arm, rail, hub, fork, bracket, or panel structure
- visible subassemblies
- mounting relationships
- materials and finish

Avoid:

- vague adjective piles
- ornamental detail as the main design signal
- freeform surfacing that dominates the object identity

## Articulation Policy

Include the articulations a reasonable person would expect on a normal example
of the object.
That often means more than one motion.

Common-sense articulations include things like:

- wheels spinning on axles or forks
- doors, lids, flaps, and covers hinging from a body
- drawers, trays, and sleeves sliding on guides
- handles pivoting from brackets or telescoping through nested members
- arms, mounts, heads, and columns panning, tilting, swiveling, or extending
- knobs, levers, pedals, and cranks rotating or pivoting when they matter to
  the category

If the articulation is canonical, visible, and buildable, prompt it.

## Tie Motion To Structure

Every named articulation should be tied to a physical support.

Prefer wording like:

- `wheel mounted in a fork`
- `lid hinged from the rear edge of the housing`
- `handle guided by nested rectangular tubes`
- `head carried by a yoke`

If a motion is named without its support, the result is more likely to produce
floating parts or weak geometry.

## SDK Fit

Category prompts must stay inside what the repo can model reliably with `sdk`
or `sdk_hybrid`.

Good fits:

- rigid housings, shells, frames, rails, brackets, hubs, forks, yokes, arms,
  trays, drawers, wheels, and panels
- hard-surface products and mechanisms with clear support structure
- geometry that can be built from primitives, profiles, sweeps, revolves,
  booleans, and straightforward CadQuery solids

Bad fits:

- soft goods, cloth, cables, belts, springs, or chains as the main identity
- highly organic freeform surfacing
- exotic lofted bodywork or compound sculpted shells as the core visual story
- mechanisms whose defining behavior depends on coupled constraints

If the prompt only really works with heroic surfacing or mechanism hacks, it is
the wrong prompt for this repo.

## Prompt Shape

Aim for one coherent paragraph or two to four dense sentences.

In general, prompts should:

1. establish the object and chosen variant
2. describe the main body and support structure
3. add concrete visual and material cues
4. name the canonical articulations and how they are mounted
5. reinforce realism and buildability at the end

This is guidance on information order, not a fixed template.
The exact wording should vary by category.

## Avoid

- `if needed`
- `if useful`
- `optional`
- `extra articulation`
- `mechanically expressive`
- `keep the body simple`
- vague prompts that name motions but not structure

## Review Checklist

Before accepting a category prompt, check:

- Does it describe a specific object or variant?
- Are the main rigid structures and supports named?
- Does it include the common-sense articulations for that object?
- Is every moving part tied to a believable mount or guide?
- Does it control silhouette and visible structure, not just motion?
- Would the requested geometry be achievable with `sdk` or `sdk_hybrid`?
- Does it avoid optional branching and gimmick motions?

If any answer is no, rewrite the prompt before running the batch.

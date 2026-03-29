# Category Prompt Guide

This document defines how to write category prompts for batch specs under
`data/batch_specs/`.

## Core Principle

**Write the prompt a mechanical engineer would write on a napkin sketch — enough
to build from, not enough to fill a spec sheet.**

The system prompt already handles realism, material choices, validation, SDK
tool selection, and the build process. The user prompt should specify *what* to
build, *what parts it has*, and *how it moves*.

More prompt detail = more geometry = more turns = more cost = more error
surface. Every sentence you add should earn its place by telling the agent
something it cannot infer from the object name alone.

## What the System Prompt Already Covers

The agent's system prompt enforces these — **do not repeat them in prompts**:

- Realistic geometry and appropriate SDK/CadQuery tool selection
- No floating parts, no unintentional overlaps
- Incremental build process with compile feedback
- Testing and validation of placement, connectivity, and articulation
- Material and color realism
- Mechanical plausibility

These phrases are **wasted tokens** — never include them:

- "realistic, highly detailed"
- "standalone mechanical study assembly"
- "for the sdk_hybrid pipeline"
- "emphasize rigid brackets, exposed bearing hardware"
- "disciplined machined-or-fabricated hard-surface geometry"
- "avoid product styling, decorative housings"
- "keep it as a pure mechanical assembly with explicit joints"
- "a short realism and buildability clause"

## Prompt Template

Every prompt should cover these three things, in roughly this order:

### 1. Object identity (required — 1 sentence)

What the object is, at a level a person would recognize. Name a specific
variant when the category has multiple common forms.

### 2. Part structure (required — 1-2 sentences)

Name the major parts and describe how they physically connect. This gives the
agent a part tree to scaffold from. Focus on the parts that define the
silhouette and the parts that move — not every sub-detail.

### 3. Articulation spec (required — 1-2 sentences)

Joint types (revolute, prismatic, continuous), which parts they connect, and
axis directions. **This is the most important section.** Be explicit about
every joint. Tie every motion to a physical support. Do not specify numeric
range-of-motion values — the agent will choose mechanically plausible limits.

### 4. Scale or proportion hint (optional — 1 sentence)

Only needed when the default would be ambiguous. E.g., "desktop-scale" vs
"industrial floor-mounted" or "arm span roughly 0.5m".

## Complexity Budget

| Object complexity | Target parts | Prompt length | Suggested max turns |
|---|---|---|---|
| Simple (1-2 joints) | 3-6 parts | 2-3 sentences | 25 |
| Medium (3-5 joints) | 6-12 parts | 3-4 sentences | 35 |
| Complex (6+ joints) | 12-20 parts | 4-6 sentences | 50 |

If you find yourself writing more than 6 sentences, you are probably
over-specifying. Split into two objects or simplify.

## Examples

### Good

**Single revolute hinge:**

> A heavy-duty door hinge. Two rectangular leaf plates connected by a
> barrel-and-pin hinge with alternating knuckles. One revolute joint along the
> barrel axis.

**Yaw-pitch module:**

> A two-axis gimbal mount. A yaw turntable base carries a U-shaped fork, which
> supports a pitch cradle between its arms. Yaw is continuous rotation about
> vertical; pitch is revolute about the horizontal fork axis.

**Telescoping boom:**

> A three-stage telescoping boom. Three nested rectangular tube sections that
> extend from a fixed root mount. Each stage slides prismatically along the boom
> axis.

**Orthogonal XY stage:**

> A two-axis positioning stage. A base carries an X-axis rail with sliding
> carriage; on top of that carriage sits a Y-axis rail with its own carriage,
> oriented 90 degrees to the first. Both axes are prismatic.

**TV wall mount:**

> An articulated TV wall mount (no screen attached). A wall plate connects to a
> folding two-link arm via revolute joints, ending in a tilt-and-swivel head
> plate. Arm folds flat against the wall. Joints: two arm-fold revolute joints,
> one head tilt revolute, one head swivel revolute.

**Shoulder-elbow-wrist arm:**

> A three-joint robotic arm. A base pedestal supports a shoulder revolute
> joint (vertical axis), an upper arm link to an elbow revolute, and a forearm
> link to a wrist revolute (roll axis).

### Bad — too verbose

> Design a realistic, highly detailed single revolute hinge as a standalone
> mechanical study assembly for the sdk_hybrid pipeline, with one clear rotary
> axis joining two rigid leaves or clevis members in a heavy-duty hinge. The
> hinge pin, barrel segments, and support cheeks should be prominent and
> mechanically plausible. Emphasize rigid brackets, exposed hinge hardware,
> removable access covers, and disciplined machined-or-fabricated hard-surface
> geometry, plus grease-cap details, stop lugs, and bolted leaf plates. Keep it
> as a pure mechanical assembly with explicit joints, guides, bearings,
> brackets, and support structure. Avoid product styling, decorative housings,
> or recognizable household or device forms.

Why it's bad: ~50% of the tokens repeat what the system prompt already says.
The agent gets no explicit articulation spec, so it has to infer joint types
and ranges. The detail wishlist (grease caps, stop lugs, removable access
covers) inflates geometry complexity and error surface without improving the
recognizable form.

### Bad — too terse

> A hinge.

Why it's bad: No part structure, no articulation spec, no scale. The agent
will produce something, but it may not match what you wanted.

## Articulation Policy

Include the articulations a reasonable person would expect on a normal instance
of the object. That often means more than one motion.

- Do not artificially reduce a category to one motion if the real object
  normally has several canonical articulations.
- Do not ignore motions of small parts (e.g., knobs, buttons, caster joints of
  the wheel, etc.).
- Do not add obscure or gimmicky motions that are not central to the category.
- Every named articulation must be tied to a physical support:
  - "wheel mounted in a fork" not "wheel spins"
  - "lid hinged from the rear edge of the housing" not "lid opens"
  - "carriage slides on twin rails" not "carriage moves"

## Batch Diversity

Prompt batches for the same category should contain meaningful variation.
That variation should come from believable differences inside the category,
not from drifting into a different object class.

Across prompts for one category, vary:

- the chosen real-world variant or subtype
- proportions and silhouette
- number and arrangement of sub-parts
- axis orientations or joint layout

Keep the category boundary stable. Avoid rows that are the same prompt with
adjectives swapped.

## SDK Fit

Category prompts must stay inside what the SDK stack can model reliably. The SDK enforces a **strict tree structure**: every part has exactly one parent, and no cycles are allowed.

Good fits: rigid housings, shells, frames, rails, brackets, hubs, forks,
yokes, arms, trays, drawers, wheels, panels — hard-surface products and
mechanisms built from primitives, profiles, sweeps, revolves, booleans, and
CadQuery solids.

Bad fits: scissor linkages, four-bar linkages, parallel linkages, differential gearsets, soft goods, cloth, cables, belts, springs, chains, highly organic
freeform surfacing, compound sculpted shells, mechanisms whose behavior depends on coupled constraints.

## Checklist

Before submitting a batch prompt:

- [ ] Does it name the object clearly?
- [ ] Does it list the key parts and how they connect?
- [ ] Does it specify every joint type and axis?
- [ ] Does it avoid specifying numeric range-of-motion values?
- [ ] Is it 6 sentences or fewer?
- [ ] Does it avoid repeating system-prompt concerns?
- [ ] Would a mechanical engineer understand what to build from this alone?

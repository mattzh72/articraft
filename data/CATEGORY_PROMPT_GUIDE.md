# Category Prompt Guide

This document defines how to write category prompts for batch specs under
`data/batch_specs/`.

Current batch specs use the unified default modeling stack. Do not mention
tooling variants or pipeline variants in prompts, and in normal batch CSVs you
should not need extra stack-selection fields.

## Core Principle

**Write the prompt a mechanical engineer would write on a napkin sketch — enough
to build from, not enough to fill a spec sheet.**

The system prompt already handles realism, material choices, validation, tool
selection, and the build process. The user prompt should specify *what* to
build, *what parts it has*, and *how it moves*.

More prompt detail = more geometry = more turns = more cost = more error
surface. Every sentence you add should earn its place by telling the agent
something it cannot infer from the object name alone.

## What the System Prompt Already Covers

The agent's system prompt enforces these — **do not repeat them in prompts**:

- Realistic geometry and appropriate modeling/CadQuery tool selection
- No floating parts, no unintentional overlaps
- Incremental build process with compile feedback
- Testing and validation of placement, connectivity, and articulation
- Material and color realism
- Mechanical plausibility

These phrases are **wasted tokens** — never include them:

- "realistic, highly detailed"
- "standalone mechanical study assembly"
- "for a specific tooling or pipeline variant"
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
| Simple (1-2 joints) | 3-6 parts | 2-3 sentences | 100 |
| Medium (3-5 joints) | 6-12 parts | 3-4 sentences | 140 |
| Complex (6+ joints) | 12-20 parts | 4-6 sentences | 180 |

If you find yourself writing more than 6 sentences, you are probably
over-specifying. Split into two objects or simplify.

These are current working budgets, not hard limits. New-category exploration in
this repo commonly runs in the `100-200` turn range. Raise the budget when the
object has real geometric or articulation complexity; do not lower prompt
quality just to fit an outdated turn target.

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
> mechanical study assembly for a special pipeline variant, with one clear rotary
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
- When several parts are really one rigid supported assembly in the prompt
  (e.g., one top tray carried by matched left/right supports, one seat carried
  by matched hangers), say so explicitly. Treat the shared body as a single
  rigid part, and describe duplicated supports as matched counterparts that
  stay aligned rather than as unrelated independently posed parts.
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

Category prompts must stay inside what the SDK stack can model reliably. Two
hard constraints shape everything below:

1. **Strict tree structure** — every part has exactly one parent joint and no
   cycles. Any mechanism requiring a part to be driven by two independent paths
   (a closed kinematic chain) cannot be modeled.
2. **URDF joint model** — every joint is single-axis (one revolute, one
   prismatic, or one continuous). There is no built-in concept of one joint
   driving another; coupled or geared motions are not expressible.

### Geometry: what models well

Listed roughly from most to least reliable:

- **Box, cylinder, sphere primitives** — exported directly as URDF primitives
  (most compact, exact, zero risk of mesh errors). Use these wherever the shape
  permits.
- **Extruded profiles and panels** — flat plates, L-brackets, C-channels,
  panels with rectangular cutouts or slots. Reliable and common.
- **Revolved / lathe shapes** — any axisymmetric part: knobs, handles, caps,
  barrels, nozzles, turret domes. Efficient and exact.
- **Boolean-carved primitives** — cylinders subtracted from cylinders (hinge
  knuckles, bored shafts), boxes with rectangular cutouts. Solid and
  well-tested.
- **Multi-branch tree assemblies** — one parent with several independent
  children (multi-drawer cabinet, fan hub with blades, robot torso with two
  arms). Each branch is independent; they do not couple.
- **Sequential link chains** — telescoping stages, robot arm segments, folding
  arm links. Arbitrary depth, fully supported.


### Geometry: what models poorly

- **Organic freeform surfaces** — compound-curved housings (car body panels,
  ergonomic grip shells) require `section_loft` with many guide curves and are
  fragile. Stick to objects with identifiable flat faces, cylinders, or
  revolved profiles.
- **Thin-wall shells without a clear inner/outer profile** — must be authored
  explicitly; no automatic offset. If the wall thickness is not a meaningful
  design feature, describe the part as solid.
- **Soft goods and flexible elements** — cloth, rubber seals, belts, cables,
  springs, and chains have no representation in URDF. Describe only the rigid
  structural parts.
- **Surface finish and texture** — URDF supports color (RGBA) only. Describing
  knurling, brushed metal, or embossed logos is wasted tokens.

### Kinematics: what URDF cannot represent

These patterns appear reasonable but fail at the model level — avoid prompting
objects whose core function depends on them:

| Pattern | Why it fails | Usable stand-in |
|---|---|---|
| **Scissor / crossed linkages** (scissor jack, lazy tongs, pantograph arms, scissor lift) | Crossed bars must share both the center pivot *and* the end pivots — a closed chain | Telescoping prismatic column, or a simple two-link fold arm |
| **Four-bar and parallel linkages** (parallelogram arms, parallel-jaw gripper with two driven jaws) | Coupler link has two parents | Model one moving jaw as prismatic child; fix the mirror jaw to the body |
| **Coupled / synchronized joints** (rack-and-pinion driving a second part, cable-driven gripper, worm gear + output gear) | URDF has no joint-drives-joint semantics; each joint moves independently | Describe only the output member's motion, omit the transmission |
| **Differential gearsets** (spider + two side gears sharing a carrier) | Carrier is both parent and child of side gears | Not modeled; pick an object with independent joints |
| **Spring/damper return mechanisms** (cam + spring-loaded follower) | Springs have no URDF primitive; cam geometry produces no kinematics | Omit the return spring; describe only the joint axis |

When a real object uses paired left/right supports or mirrored legs to carry
one shared rigid body, prefer prompt wording that makes the shared body primary
("a single rigid tray supported by matched left and right side assemblies") and
that keeps the counterparts aligned. Do not describe those repeated supports as
separately drifting or freely independent unless that independence is actually
part of the category.

### Articulation types

- **Revolute** — bounded rotation about one axis. Use for hinges, flaps, lids,
  arm joints. Always has motion limits.
- **Prismatic** — bounded translation along one axis. Use for drawers, slides,
  telescoping stages, jaws.
- **Continuous** — unbounded rotation (no limits). Use for wheels, fans,
  turntables, and any part that spins freely. *Do not use revolute for a
  freely-spinning wheel.*
- **Fixed** — zero-DOF rigid connection. Use to attach sub-parts that never
  move relative to their parent (mounting flanges, caps, trim rings).

Every joint is single-axis. A ball-and-socket wrist needs three separate
revolute joints stacked in a chain (yaw → pitch → roll links). State each axis
separately in the articulation spec.

## Checklist

Before submitting a batch prompt:

- [ ] Does it name the object clearly?
- [ ] Does it list the key parts and how they connect?
- [ ] Does it specify every joint type and axis?
- [ ] Does it make rigid grouped parts and matched counterpart supports explicit
      when alignment matters?
- [ ] Does it avoid specifying numeric range-of-motion values?
- [ ] Is it 6 sentences or fewer?
- [ ] Does it avoid repeating system-prompt concerns?
- [ ] Would a mechanical engineer understand what to build from this alone?

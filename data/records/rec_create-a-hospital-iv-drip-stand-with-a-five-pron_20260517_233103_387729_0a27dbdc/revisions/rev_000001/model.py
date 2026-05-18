from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ── Geometry constants ─────────────────────────────────────────────────────────
N_PRONGS = 5
ARM_LENGTH = 0.290          # hub-centre to arm-tip (m)
HUB_R = 0.045               # hub cylinder radius
ARM_LEN = ARM_LENGTH - HUB_R  # arm length beyond hub edge = 0.245
ARM_W = 0.024               # arm width
ARM_H = 0.018               # arm height
ARM_Z = 0.055               # arm centre-line height above floor

HUB_H = 0.070               # hub cylinder height
HUB_CENTER_Z = ARM_Z        # hub at same height as arms
HUB_TOP = HUB_CENTER_Z + HUB_H / 2   # ≈ 0.090
HUB_BOTTOM = HUB_CENTER_Z - HUB_H / 2  # ≈ 0.020

CASTER_R = 0.025            # caster wheel radius (50 mm dia)
CASTER_H = 0.020            # caster wheel thickness
CASTER_Z = CASTER_R         # wheel centre z (bottom at z = 0)

LOWER_POLE_R = 0.016
LOWER_POLE_H = 1.040
LOWER_POLE_BOTTOM = HUB_TOP  # ≈ 0.090
LOWER_POLE_TOP = LOWER_POLE_BOTTOM + LOWER_POLE_H  # ≈ 1.130

COLLAR_R = 0.023
COLLAR_H = 0.028
COLLAR_Z = LOWER_POLE_BOTTOM + 0.090  # just above base collar

OVERLAP = 0.260             # upper pole inside lower pole at rest
                            # must satisfy: 2*OVERLAP - SLIDE_RANGE >= 0.030
UPPER_EXPOSED = 0.500       # visible above lower pole at rest
UPPER_TOTAL = OVERLAP + UPPER_EXPOSED   # 0.760
SLIDE_RANGE = 0.420         # prismatic upper travel limit
                            # retention at max extension = 2*0.260 - 0.420 = 0.100 m ✓

JOINT_Z = LOWER_POLE_TOP - OVERLAP   # ≈ 0.870 — joint in parent frame

HOOK_HUB_R = 0.022
HOOK_HUB_H = 0.038
HOOK_R = 0.006              # hook wire radius
HOOK_POST_H = 0.095
HOOK_SPAN = 0.080           # radial extent of horizontal arm


# ── Shape helpers ──────────────────────────────────────────────────────────────

def _make_base_shape() -> cq.Workplane:
    """Five-pronged base + lower pole + collar."""
    # Hub
    shape = (
        cq.Workplane("XY")
        .cylinder(HUB_H, HUB_R)
        .translate((0, 0, HUB_CENTER_Z))
    )

    # Five arms: build template along +X, then rotate copies
    arm_tpl = (
        cq.Workplane("XY")
        .box(ARM_LEN, ARM_W, ARM_H)
        .translate((HUB_R + ARM_LEN / 2, 0, ARM_Z))
    )
    for i in range(N_PRONGS):
        shape = shape.union(arm_tpl.rotate((0, 0, 0), (0, 0, 1), i * 72.0))

    # Caster wheels (cylinder axis along Y for each arm along +X, then rotated)
    caster_tpl = (
        cq.Workplane("XY")
        .cylinder(CASTER_H, CASTER_R)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)   # Y-axis cylinder
        .translate((ARM_LENGTH, 0, CASTER_Z))
    )
    for i in range(N_PRONGS):
        shape = shape.union(caster_tpl.rotate((0, 0, 0), (0, 0, 1), i * 72.0))

    # Caster swivel column (small vertical pin from arm-tip down to wheel)
    col_bottom_z = CASTER_Z + CASTER_R * 0.6
    col_top_z = ARM_Z - ARM_H / 2
    col_h = max(col_top_z - col_bottom_z, 0.003)
    swivel_tpl = (
        cq.Workplane("XY")
        .cylinder(col_h, 0.007)
        .translate((ARM_LENGTH, 0, col_bottom_z + col_h / 2))
    )
    for i in range(N_PRONGS):
        shape = shape.union(swivel_tpl.rotate((0, 0, 0), (0, 0, 1), i * 72.0))

    # Lower pole
    pole = (
        cq.Workplane("XY")
        .cylinder(LOWER_POLE_H, LOWER_POLE_R)
        .translate((0, 0, LOWER_POLE_BOTTOM + LOWER_POLE_H / 2))
    )
    shape = shape.union(pole)

    # Height-adjustment collar ring
    collar = (
        cq.Workplane("XY")
        .cylinder(COLLAR_H, COLLAR_R)
        .translate((0, 0, COLLAR_Z + COLLAR_H / 2))
    )
    shape = shape.union(collar)

    return shape


def _make_upper_pole_shape() -> cq.Workplane:
    """Telescoping inner section.  Child frame is at JOINT_Z in world space."""
    # The cylinder extends from -OVERLAP to +UPPER_EXPOSED in child-frame z.
    # Centre = (UPPER_EXPOSED - OVERLAP) / 2
    centre_z = (UPPER_EXPOSED - OVERLAP) / 2
    return (
        cq.Workplane("XY")
        .cylinder(UPPER_TOTAL, UPPER_POLE_R := 0.011)
        .translate((0, 0, centre_z))
    )


def _make_hook_shape() -> cq.Workplane:
    """Chrome hub + four J-shaped hooks in hook-assembly frame (z=0 at joint).

    Connectivity rules (all verified by geometry):
    - post_x is placed INSIDE the hub rim so the post cylinder overlaps the hub
      cylinder in both XY (post inner edge < hub_r) and Z (post starts below hub top).
    - The horizontal arm is centred on the same x as the post top so they share
      a z-overlap zone at the post–arm junction.
    - The downward tip is centred at the arm's outer end so they share that x band.
    """
    # Hub cylinder: z from 0 to HOOK_HUB_H
    shape = (
        cq.Workplane("XY")
        .cylinder(HOOK_HUB_H, HOOK_HUB_R)
        .translate((0, 0, HOOK_HUB_H / 2))
    )

    # post_x INSIDE hub rim → inner edge (post_x - HOOK_R) is well inside hub radius
    post_x = HOOK_HUB_R - HOOK_R - 0.002   # ≈ 0.014  (inner edge at 0.008 m)
    # Post starts 5 mm below hub top so it overlaps the hub in z too
    post_start_z = HOOK_HUB_H - 0.005
    post_full_h = HOOK_POST_H + 0.005      # compensates for the 5 mm dip
    post_top_z = post_start_z + post_full_h   # = HOOK_HUB_H + HOOK_POST_H
    post_cz = (post_start_z + post_top_z) / 2

    # Horizontal arm radiates outward from post_x to tip_x
    tip_x = post_x + HOOK_SPAN + 0.014     # ≈ 0.108 m from axis
    arm_len = tip_x - post_x
    arm_cx = (post_x + tip_x) / 2
    # arm_cz = post_top_z so the arm box overlaps the post's top ±HOOK_R in z
    arm_cz = post_top_z

    # Downward tip: centred at arm_end, hanging from arm_cz level
    tip_h = 0.052
    tip_cz = arm_cz - tip_h / 2            # top of tip at arm_cz

    for i in range(4):
        ang = i * 90.0

        # Vertical post — overlaps hub in XY (post_x < HOOK_HUB_R) and z (dips 5 mm in)
        post = (
            cq.Workplane("XY")
            .cylinder(post_full_h, HOOK_R)
            .translate((post_x, 0, post_cz))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )

        # Horizontal arm — box oriented along +X in template, rotated per hook
        horiz = (
            cq.Workplane("XY")
            .box(arm_len, HOOK_R * 2, HOOK_R * 2)
            .translate((arm_cx, 0, arm_cz))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )

        # Downward tip — cylinder whose top edge meets arm_cz level
        tip = (
            cq.Workplane("XY")
            .cylinder(tip_h, HOOK_R)
            .translate((tip_x, 0, tip_cz))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )

        shape = shape.union(post).union(horiz).union(tip)

    return shape


# ── Model builder ──────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="iv_drip_stand")

    # Materials
    stainless = model.material("stainless_steel", rgba=(0.76, 0.78, 0.82, 1.0))
    chrome = model.material("chrome", rgba=(0.88, 0.91, 0.95, 1.0))

    # ── Base (root) ──
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_body"),
        material=stainless,
        name="base_body",
    )

    # ── Upper pole (telescoping) ──
    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        mesh_from_cadquery(_make_upper_pole_shape(), "upper_pole_body"),
        material=stainless,
        name="upper_pole_body",
    )

    pole_slide = model.articulation(
        "pole_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.20, lower=0.0, upper=SLIDE_RANGE
        ),
    )

    # ── Hook assembly (rotating) ──
    hook_assembly = model.part("hook_assembly")
    hook_assembly.visual(
        mesh_from_cadquery(_make_hook_shape(), "hook_body"),
        material=chrome,
        name="hook_body",
    )

    # Hook joint sits at the top of the upper pole in upper-pole frame.
    # Upper pole top in its own frame = UPPER_EXPOSED (shape centre + half-length).
    hook_rotate = model.articulation(
        "hook_rotate",
        ArticulationType.REVOLUTE,
        parent=upper_pole,
        child=hook_assembly,
        origin=Origin(xyz=(0.0, 0.0, UPPER_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-math.pi, upper=math.pi
        ),
    )

    return model


# ── Tests ──────────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_pole = object_model.get_part("upper_pole")
    hook_assembly = object_model.get_part("hook_assembly")
    pole_slide = object_model.get_articulation("pole_slide")
    hook_rotate = object_model.get_articulation("hook_rotate")

    # Upper pole overlaps lower pole at rest — intentional telescoping insertion
    ctx.allow_overlap(
        upper_pole,
        base,
        elem_a="upper_pole_body",
        elem_b="base_body",
        reason=(
            "Upper pole is retained inside the lower pole sleeve at q=0; "
            "the 260 mm overlap is the insertion section of the telescoping mechanism."
        ),
    )

    # At rest: upper pole should overlap with base along z (retained insertion)
    ctx.expect_overlap(
        upper_pole,
        base,
        axes="z",
        min_overlap=0.10,
        name="upper pole retained inside lower sleeve at rest",
    )

    # Upper pole should be concentric with base pole (xy centred)
    ctx.expect_within(
        upper_pole,
        base,
        axes="xy",
        margin=0.005,
        name="upper pole centred in lower pole",
    )

    # At max extension the pole still has retained insertion
    with ctx.pose({pole_slide: SLIDE_RANGE}):
        ctx.expect_overlap(
            upper_pole,
            base,
            axes="z",
            min_overlap=0.030,
            name="upper pole still inserted at full extension",
        )
        extended_pos = ctx.part_world_position(upper_pole)

    rest_pos = ctx.part_world_position(upper_pole)
    ctx.check(
        "upper pole moves upward on extension",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Hook assembly rotates: check it moves when given 90° input
    hook_rest = ctx.part_world_position(hook_assembly)
    with ctx.pose({hook_rotate: math.pi / 2}):
        hook_rotated = ctx.part_world_position(hook_assembly)
    # Position should be essentially unchanged (rotation around axis through its centre)
    ctx.check(
        "hook assembly position stable under rotation",
        hook_rest is not None
        and hook_rotated is not None
        and abs(hook_rest[0] - hook_rotated[0]) < 0.02
        and abs(hook_rest[1] - hook_rotated[1]) < 0.02,
        details=f"rest={hook_rest}, rotated={hook_rotated}",
    )

    return ctx.report()


object_model = build_object_model()

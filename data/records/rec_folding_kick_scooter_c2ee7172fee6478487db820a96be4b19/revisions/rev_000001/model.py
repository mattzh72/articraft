from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WHEEL_RADIUS = 0.108
WHEEL_WIDTH = 0.052
FRONT_AXLE_X = 0.56
REAR_AXLE_X = -0.48
AXLE_Z = 0.108


def _curved_fender_mesh(
    *,
    center_x: float,
    center_z: float,
    radius: float,
    width: float,
    thickness: float,
    start_deg: float,
    end_deg: float,
    name: str,
    segments: int = 24,
):
    """Return a closed, thin curved fender shell in the root scooter frame."""
    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []
    half_w = width / 2.0
    inner_r = radius - thickness
    for i in range(segments + 1):
        t = math.radians(start_deg + (end_deg - start_deg) * i / segments)
        ca, sa = math.cos(t), math.sin(t)
        outer_left = geom.add_vertex(center_x + radius * ca, -half_w, center_z + radius * sa)
        outer_right = geom.add_vertex(center_x + radius * ca, half_w, center_z + radius * sa)
        inner_left = geom.add_vertex(center_x + inner_r * ca, -half_w, center_z + inner_r * sa)
        inner_right = geom.add_vertex(center_x + inner_r * ca, half_w, center_z + inner_r * sa)
        rings.append((outer_left, outer_right, inner_left, inner_right))

    for a, b in zip(rings[:-1], rings[1:]):
        ao_l, ao_r, ai_l, ai_r = a
        bo_l, bo_r, bi_l, bi_r = b
        # Outer curved skin
        geom.add_face(ao_l, bo_l, bo_r)
        geom.add_face(ao_l, bo_r, ao_r)
        # Inner surface
        geom.add_face(ai_l, ai_r, bi_r)
        geom.add_face(ai_l, bi_r, bi_l)
        # Side faces
        geom.add_face(ao_l, ai_l, bi_l)
        geom.add_face(ao_l, bi_l, bo_l)
        geom.add_face(ao_r, bo_r, bi_r)
        geom.add_face(ao_r, bi_r, ai_r)

    # Close both ends.
    for ring in (rings[0], rings[-1]):
        o_l, o_r, i_l, i_r = ring
        geom.add_face(o_l, o_r, i_r)
        geom.add_face(o_l, i_r, i_l)
    return mesh_from_geometry(geom, name)


def _deck_mesh():
    return mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.74, 0.19, 0.045)
        .edges("|Z")
        .fillet(0.040),
        "rounded_flat_deck",
        tolerance=0.0015,
    )


def _outer_stem_mesh():
    sleeve = cq.Workplane("XY").circle(0.026).circle(0.020).extrude(0.480)
    top_collar = (
        cq.Workplane("XY")
        .circle(0.032)
        .circle(0.020)
        .extrude(0.048)
        .translate((0.0, 0.0, 0.432))
    )
    lower_collar = (
        cq.Workplane("XY")
        .circle(0.030)
        .circle(0.020)
        .extrude(0.040)
        .translate((0.0, 0.0, 0.020))
    )
    return mesh_from_cadquery(sleeve.union(top_collar).union(lower_collar), "hollow_outer_stem")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_electric_commuter_scooter")

    deck_black = model.material("satin_black_deck", rgba=(0.025, 0.027, 0.030, 1.0))
    grip = model.material("rubber_grip", rgba=(0.006, 0.006, 0.006, 1.0))
    graphite = model.material("dark_graphite_frame", rgba=(0.09, 0.10, 0.11, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    tire_black = model.material("pneumatic_tire_black", rgba=(0.015, 0.015, 0.016, 1.0))
    rim_silver = model.material("machined_wheel_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    red = model.material("rear_reflector_red", rgba=(0.85, 0.04, 0.02, 1.0))

    deck = model.part("deck")
    deck.visual(Box((0.74, 0.19, 0.045)), origin=Origin(xyz=(0.02, 0.0, 0.170)), material=deck_black, name="deck_shell")
    deck.visual(Box((0.62, 0.145, 0.006)), origin=Origin(xyz=(0.02, 0.0, 0.194)), material=grip, name="grip_pad")
    deck.visual(Box((0.70, 0.020, 0.024)), origin=Origin(xyz=(0.02, 0.096, 0.162)), material=brushed, name="side_rail_0")
    deck.visual(Box((0.70, 0.020, 0.024)), origin=Origin(xyz=(0.02, -0.096, 0.162)), material=brushed, name="side_rail_1")
    deck.visual(Box((0.44, 0.135, 0.035)), origin=Origin(xyz=(0.04, 0.0, 0.132)), material=graphite, name="battery_belly")

    # Rear dropout, fender, and the latch catch are rigid with the deck.
    for idx, y in enumerate((0.049, -0.049)):
        deck.visual(Box((0.165, 0.014, 0.050)), origin=Origin(xyz=(-0.410, y, 0.128)), material=graphite, name=f"rear_dropout_{idx}")
        deck.visual(Box((0.026, 0.014, 0.078)), origin=Origin(xyz=(-0.438, y, 0.190)), material=graphite, name=f"rear_fender_strut_{idx}")
    deck.visual(Cylinder(radius=0.005, length=0.128), origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="rear_axle_pin")
    deck.visual(
        _curved_fender_mesh(
            center_x=REAR_AXLE_X,
            center_z=AXLE_Z,
            radius=0.124,
            width=0.108,
            thickness=0.010,
            start_deg=22.0,
            end_deg=164.0,
            name="rear_curved_fender",
        ),
        material=graphite,
        name="rear_fender",
    )
    deck.visual(Box((0.026, 0.018, 0.036)), origin=Origin(xyz=(-0.442, 0.040, 0.235)), material=brushed, name="rear_catch")
    deck.visual(Box((0.035, 0.082, 0.026)), origin=Origin(xyz=(-0.560, 0.0, 0.210)), material=red, name="rear_reflector")

    # Front fork and folding-neck support.
    deck.visual(Box((0.120, 0.086, 0.034)), origin=Origin(xyz=(0.410, 0.0, 0.205)), material=graphite, name="neck_base")
    deck.visual(Box((0.085, 0.110, 0.024)), origin=Origin(xyz=(0.555, 0.0, 0.238)), material=graphite, name="fork_crown")
    for idx, y in enumerate((0.050, -0.050)):
        deck.visual(Box((0.160, 0.014, 0.018)), origin=Origin(xyz=(0.480, y, 0.232)), material=graphite, name=f"front_neck_rail_{idx}")
        deck.visual(Box((0.032, 0.014, 0.176)), origin=Origin(xyz=(FRONT_AXLE_X, y, 0.158)), material=graphite, name=f"front_fork_{idx}")
        deck.visual(Box((0.026, 0.014, 0.070)), origin=Origin(xyz=(FRONT_AXLE_X, y, 0.199)), material=graphite, name=f"front_fender_strut_{idx}")
        deck.visual(
            Cylinder(radius=0.018, length=0.025),
            origin=Origin(xyz=(0.430, y, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"fold_knuckle_{idx}",
        )
    deck.visual(Cylinder(radius=0.005, length=0.118), origin=Origin(xyz=(0.430, 0.0, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="fold_hinge_pin")
    deck.visual(Cylinder(radius=0.005, length=0.128), origin=Origin(xyz=(FRONT_AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="front_axle_pin")
    deck.visual(
        _curved_fender_mesh(
            center_x=FRONT_AXLE_X,
            center_z=AXLE_Z,
            radius=0.124,
            width=0.108,
            thickness=0.010,
            start_deg=16.0,
            end_deg=148.0,
            name="front_curved_fender",
        ),
        material=graphite,
        name="front_fender",
    )

    front_wheel = model.part("front_wheel")
    rear_wheel = model.part("rear_wheel")
    for part_obj, prefix in ((front_wheel, "front"), (rear_wheel, "rear")):
        part_obj.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.085,
                    0.037,
                    rim=WheelRim(inner_radius=0.060, flange_height=0.007, flange_thickness=0.003),
                    hub=WheelHub(
                        radius=0.022,
                        width=0.028,
                        cap_style="domed",
                        bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.0035),
                    ),
                    face=WheelFace(dish_depth=0.004, front_inset=0.002),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.007),
                    bore=WheelBore(style="round", diameter=0.010),
                ),
                f"{prefix}_rim",
            ),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_silver,
            name="rim",
        )
        part_obj.visual(
            mesh_from_geometry(
                TireGeometry(
                    WHEEL_RADIUS,
                    WHEEL_WIDTH,
                    inner_radius=0.086,
                    tread=TireTread(style="circumferential", depth=0.004, count=3),
                    grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
                    sidewall=TireSidewall(style="rounded", bulge=0.055),
                ),
                f"{prefix}_pneumatic_tire",
            ),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=tire_black,
            name="tire",
        )

    lower_stem = model.part("lower_stem")
    lower_stem.visual(Cylinder(radius=0.017, length=0.052), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="center_knuckle")
    lower_stem.visual(Box((0.055, 0.038, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.022)), material=graphite, name="hinge_block")
    lower_stem.visual(_outer_stem_mesh(), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=brushed, name="outer_sleeve")
    lower_stem.visual(Box((0.068, 0.020, 0.035)), origin=Origin(xyz=(0.0, -0.035, 0.468)), material=graphite, name="quick_clamp")

    upper_stem = model.part("upper_stem")
    upper_stem.visual(Cylinder(radius=0.016, length=0.680), origin=Origin(xyz=(0.0, 0.0, 0.090)), material=brushed, name="inner_mast")
    upper_stem.visual(Cylinder(radius=0.020, length=0.020), origin=Origin(xyz=(0.0, 0.0, -0.180)), material=graphite, name="lower_bushing")
    upper_stem.visual(Cylinder(radius=0.020, length=0.020), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=graphite, name="upper_bushing")
    upper_stem.visual(Cylinder(radius=0.017, length=0.430), origin=Origin(xyz=(0.0, 0.0, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="handlebar")
    upper_stem.visual(Cylinder(radius=0.020, length=0.085), origin=Origin(xyz=(0.0, 0.250, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)), material=grip, name="grip_0")
    upper_stem.visual(Cylinder(radius=0.020, length=0.085), origin=Origin(xyz=(0.0, -0.250, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)), material=grip, name="grip_1")
    upper_stem.visual(Box((0.060, 0.040, 0.018)), origin=Origin(xyz=(0.020, 0.0, 0.405)), material=deck_black, name="display_pod")
    upper_stem.visual(Box((0.022, 0.012, 0.022)), origin=Origin(xyz=(0.0, 0.021, 0.300)), material=graphite, name="latch_mount")

    hook_latch = model.part("hook_latch")
    hook_latch.visual(Cylinder(radius=0.007, length=0.026), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed, name="pivot_barrel")
    hook_latch.visual(Box((0.014, 0.012, 0.095)), origin=Origin(xyz=(0.0, 0.0, 0.0475)), material=graphite, name="latch_arm")
    hook_latch.visual(Box((0.042, 0.012, 0.014)), origin=Origin(xyz=(-0.014, 0.0, 0.097)), material=graphite, name="hook_tip")

    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lower_stem,
        origin=Origin(xyz=(0.430, 0.0, 0.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.5708, upper=0.150, effort=80.0, velocity=1.2),
    )
    model.articulation(
        "stem_slide",
        ArticulationType.PRISMATIC,
        parent=lower_stem,
        child=upper_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.220, effort=70.0, velocity=0.25),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_stem,
        child=hook_latch,
        origin=Origin(xyz=(0.0, 0.040, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.75, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    lower_stem = object_model.get_part("lower_stem")
    upper_stem = object_model.get_part("upper_stem")
    hook_latch = object_model.get_part("hook_latch")
    stem_slide = object_model.get_articulation("stem_slide")
    stem_fold = object_model.get_articulation("stem_fold")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        deck,
        front_wheel,
        elem_a="front_axle_pin",
        elem_b="rim",
        reason="The axle pin is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle_pin",
        elem_b="rim",
        reason="The axle pin is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        lower_stem,
        elem_a="fold_hinge_pin",
        elem_b="center_knuckle",
        reason="The fold hinge pin intentionally passes through the stem-base knuckle.",
    )
    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a="outer_sleeve",
        elem_b="lower_bushing",
        reason="The lower bushing is a close sliding guide represented seated inside the hollow sleeve.",
    )
    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a="outer_sleeve",
        elem_b="upper_bushing",
        reason="The upper bushing is a close sliding guide represented seated inside the hollow sleeve.",
    )

    ctx.check(
        "primary_articulations_present",
        all(
            object_model.get_articulation(name) is not None
            for name in ("front_axle", "rear_axle", "stem_fold", "stem_slide", "latch_pivot")
        ),
        "Expected wheel axles, folding stem hinge, telescoping slide, and hook latch pivot.",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        positive_elem="tire",
        negative_elem="deck_shell",
        min_gap=0.015,
        name="front wheel sits ahead of the flat deck",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="tire",
        min_gap=0.010,
        name="rear wheel sits behind the flat deck",
    )
    ctx.expect_overlap(
        deck,
        front_wheel,
        axes="y",
        elem_a="front_axle_pin",
        elem_b="rim",
        min_overlap=0.030,
        name="front axle pin spans the wheel hub",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle_pin",
        elem_b="rim",
        min_overlap=0.030,
        name="rear axle pin spans the wheel hub",
    )
    ctx.expect_overlap(
        deck,
        lower_stem,
        axes="y",
        elem_a="fold_hinge_pin",
        elem_b="center_knuckle",
        min_overlap=0.040,
        name="fold hinge pin spans the center knuckle",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a="lower_bushing",
        elem_b="outer_sleeve",
        min_overlap=0.015,
        name="lower bushing remains seated in outer sleeve",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a="upper_bushing",
        elem_b="outer_sleeve",
        min_overlap=0.015,
        name="upper bushing remains seated in outer sleeve",
    )
    ctx.expect_within(
        upper_stem,
        lower_stem,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="telescoping inner mast stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.200,
        name="collapsed telescoping stem remains inserted",
    )

    rest_upper = ctx.part_world_position(upper_stem)
    with ctx.pose({stem_slide: 0.220}):
        ctx.expect_within(
            upper_stem,
            lower_stem,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended telescoping stem remains centered",
        )
        ctx.expect_overlap(
            upper_stem,
            lower_stem,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="extended telescoping stem retains sleeve insertion",
        )
        extended_upper = ctx.part_world_position(upper_stem)
    ctx.check(
        "telescoping stem extends upward",
        rest_upper is not None and extended_upper is not None and extended_upper[2] > rest_upper[2] + 0.18,
        details=f"rest={rest_upper}, extended={extended_upper}",
    )

    rest_latch = ctx.part_world_position(hook_latch)
    with ctx.pose({stem_fold: -1.5708, stem_slide: 0.0, latch_pivot: 0.0}):
        folded_latch = ctx.part_world_position(hook_latch)
        ctx.expect_overlap(
            hook_latch,
            deck,
            axes="xz",
            elem_a="hook_tip",
            elem_b="rear_catch",
            min_overlap=0.006,
            name="folded hook latch reaches rear fender catch",
        )
        ctx.expect_overlap(
            hook_latch,
            deck,
            axes="y",
            elem_a="hook_tip",
            elem_b="rear_catch",
            min_overlap=0.006,
            name="folded hook latch aligns laterally with rear catch",
        )
    ctx.check(
        "fold hinge swings the stem toward the rear fender",
        rest_latch is not None and folded_latch is not None and folded_latch[0] < rest_latch[0] - 0.70,
        details=f"rest={rest_latch}, folded={folded_latch}",
    )

    return ctx.report()


object_model = build_object_model()

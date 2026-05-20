from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_effects_unit")

    # Realistic rack proportions: about 19 in wide at the ears, shallow body,
    # and a tall enough front face for a large jog encoder and transport cover.
    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("dark_graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    model.material("brushed_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("rubber_black", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("white_marking", rgba=(0.92, 0.92, 0.86, 1.0))
    model.material("blue_display", rgba=(0.05, 0.18, 0.35, 1.0))
    model.material("smoked_polycarbonate", rgba=(0.10, 0.12, 0.14, 0.32))

    housing = model.part("housing")
    housing.visual(
        Box((0.430, 0.220, 0.126)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material="matte_black",
        name="shallow_chassis",
    )
    housing.visual(
        Box((0.483, 0.010, 0.150)),
        origin=Origin(xyz=(0.0, -0.113, 0.075)),
        material="dark_graphite",
        name="front_plate",
    )
    housing.visual(
        Box((0.462, 0.047, 0.012)),
        origin=Origin(xyz=(0.0, -0.136, 0.006)),
        material="matte_black",
        name="bottom_hinge_lip",
    )
    housing.visual(
        Box((0.455, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.154, 0.006)),
        material="brushed_metal",
        name="hinge_mount_strip",
    )

    # Rack-ear screw bosses and a non-moving status display are seated into the
    # front plate so they read as decals/recesses rather than separate controls.
    for i, (x, z) in enumerate(
        (
            (-0.224, 0.120),
            (-0.224, 0.030),
            (0.224, 0.120),
            (0.224, 0.030),
        )
    ):
        housing.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(x, -0.1185, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="rubber_black",
            name=f"rack_screw_{i}",
        )
    housing.visual(
        Box((0.082, 0.003, 0.030)),
        origin=Origin(xyz=(-0.145, -0.119, 0.108)),
        material="blue_display",
        name="status_display",
    )
    housing.visual(
        Box((0.098, 0.002, 0.0025)),
        origin=Origin(xyz=(-0.145, -0.1185, 0.087)),
        material="white_marking",
        name="display_label_line",
    )

    # Main jog/encoder wheel.  It is a separate rotating part with its face just
    # proud of the front panel and a visible fiducial mark that rotates with it.
    jog_wheel = model.part("jog_wheel")
    jog_wheel.visual(
        Cylinder(radius=0.058, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="outer_grip",
    )
    jog_wheel.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="touch_plate",
    )
    jog_wheel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_graphite",
        name="center_cap",
    )
    jog_wheel.visual(
        Box((0.006, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, -0.041, 0.034)),
        material="white_marking",
        name="index_mark",
    )
    model.articulation(
        "jog_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=jog_wheel,
        origin=Origin(xyz=(0.0, -0.118, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0, lower=-math.pi, upper=math.pi),
    )

    knob_meshes = []
    for i in range(4):
        knob_geom = KnobGeometry(
            0.031,
            0.020,
            body_style="faceted",
            base_diameter=0.033,
            top_diameter=0.024,
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0009, width=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            top_feature=KnobTopFeature(style="top_insert", diameter=0.010, height=0.0010),
            center=False,
        )
        knob_meshes.append(mesh_from_geometry(knob_geom, f"parameter_knob_mesh_{i}"))

    knob_positions = (
        (0.130, 0.112),
        (0.178, 0.112),
        (0.130, 0.058),
        (0.178, 0.058),
    )
    for i, (x, z) in enumerate(knob_positions):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_meshes[i],
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="rubber_black",
            name="cap",
        )
        model.articulation(
            f"knob_{i}_turn",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, -0.118, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=8.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    # Smoked protective cover hinged along the lower front edge.  At q=0 it is
    # latched in front of the controls with clearance; positive motion folds it
    # down and outward for use.
    cover = model.part("front_cover")
    cover.visual(
        Box((0.462, 0.006, 0.132)),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material="smoked_polycarbonate",
        name="pane",
    )
    cover.visual(
        Cylinder(radius=0.006, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_metal",
        name="hinge_barrel",
    )
    cover.visual(
        Box((0.462, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.001, 0.139)),
        material="rubber_black",
        name="top_latch_rail",
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(0.0, -0.166, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jog_wheel = object_model.get_part("jog_wheel")
    cover = object_model.get_part("front_cover")
    jog_joint = object_model.get_articulation("jog_wheel_spin")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.check(
        "four separate parameter knobs",
        all(object_model.get_part(f"knob_{i}") is not None for i in range(4)),
    )
    ctx.check(
        "every parameter knob has a limited revolute joint",
        all(
            object_model.get_articulation(f"knob_{i}_turn").articulation_type
            == ArticulationType.REVOLUTE
            and object_model.get_articulation(f"knob_{i}_turn").motion_limits is not None
            and object_model.get_articulation(f"knob_{i}_turn").motion_limits.lower < 0.0
            and object_model.get_articulation(f"knob_{i}_turn").motion_limits.upper > 0.0
            for i in range(4)
        ),
    )
    ctx.check(
        "jog wheel is revolute about the front-panel normal",
        jog_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(jog_joint.axis) == (0.0, -1.0, 0.0),
    )
    ctx.expect_gap(
        housing,
        jog_wheel,
        axis="y",
        positive_elem="front_plate",
        negative_elem="outer_grip",
        max_gap=0.001,
        max_penetration=0.001,
        name="jog wheel is mounted flush to the front panel",
    )
    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        elem_a="pane",
        elem_b="front_plate",
        min_overlap=0.12,
        name="closed cover spans the rack front",
    )
    ctx.expect_gap(
        jog_wheel,
        cover,
        axis="y",
        negative_elem="pane",
        max_gap=0.030,
        max_penetration=0.0,
        name="closed cover clears protruding jog wheel",
    )

    closed_cover = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.35}):
        opened_cover = ctx.part_world_aabb(cover)
    ctx.check(
        "cover hinge folds the panel outward and down",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[0][1] < closed_cover[0][1] - 0.035
        and opened_cover[1][2] < closed_cover[1][2] - 0.080,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    return ctx.report()


object_model = build_object_model()

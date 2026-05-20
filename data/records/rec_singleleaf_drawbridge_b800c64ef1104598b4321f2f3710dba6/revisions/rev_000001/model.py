from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.55, 0.57, 0.55, 1.0))
    dark_water = model.material("dark_channel_water", rgba=(0.05, 0.12, 0.16, 1.0))
    steel = model.material("painted_steel", rgba=(0.18, 0.24, 0.28, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.06, 0.07, 0.08, 1.0))
    asphalt = model.material("worn_asphalt", rgba=(0.08, 0.085, 0.085, 1.0))
    yellow = model.material("faded_yellow_stripe", rgba=(0.92, 0.73, 0.18, 1.0))

    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.150, tube=0.040, radial_segments=20, tubular_segments=56),
        "shore_bearing_ring",
    )

    frame = model.part("shore_frame")
    frame.visual(
        Box((3.12, 1.86, 0.030)),
        origin=Origin(xyz=(1.62, 0.0, 0.015)),
        material=dark_water,
        name="channel_water",
    )
    for side_index, y in enumerate((-1.05, 1.05)):
        frame.visual(
            Box((3.78, 0.26, 0.30)),
            origin=Origin(xyz=(1.48, y, 0.18)),
            material=concrete,
            name=f"side_curb_{side_index}",
        )
    frame.visual(
        Box((0.70, 2.36, 0.32)),
        origin=Origin(xyz=(-0.37, 0.0, 0.20)),
        material=concrete,
        name="shore_abutment",
    )
    frame.visual(
        Box((0.22, 2.18, 0.20)),
        origin=Origin(xyz=(-0.03, 0.0, 0.22)),
        material=concrete,
        name="hinge_sill",
    )
    frame.visual(
        Box((0.28, 1.86, 0.25)),
        origin=Origin(xyz=(3.24, 0.0, 0.17)),
        material=concrete,
        name="far_landing_sill",
    )
    for side_index, y in enumerate((-0.94, 0.94)):
        frame.visual(
            Box((0.46, 0.34, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.27)),
            material=concrete,
            name=f"bearing_plinth_{side_index}",
        )
        frame.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_ring_{side_index}",
        )
        frame.visual(
            Box((0.08, 0.28, 0.18)),
            origin=Origin(xyz=(-0.19, y, 0.42)),
            material=steel,
            name=f"bearing_backplate_{side_index}",
        )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Cylinder(radius=0.075, length=2.02),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    leaf.visual(
        Box((3.00, 1.50, 0.10)),
        origin=Origin(xyz=(1.55, 0.0, -0.075)),
        material=steel,
        name="steel_leaf_panel",
    )
    leaf.visual(
        Box((2.86, 1.28, 0.026)),
        origin=Origin(xyz=(1.62, 0.0, -0.014)),
        material=asphalt,
        name="road_surface",
    )
    for side_index, y in enumerate((-0.69, 0.69)):
        leaf.visual(
            Box((2.80, 0.12, 0.18)),
            origin=Origin(xyz=(1.65, y, -0.175)),
            material=dark_steel,
            name=f"side_girder_{side_index}",
        )
    for rib_index, x in enumerate((0.40, 1.05, 1.70, 2.35, 2.90)):
        leaf.visual(
            Box((0.10, 1.34, 0.11)),
            origin=Origin(xyz=(x, 0.0, -0.180)),
            material=dark_steel,
            name=f"cross_rib_{rib_index}",
        )
    leaf.visual(
        Box((0.18, 1.46, 0.13)),
        origin=Origin(xyz=(0.08, 0.0, -0.055)),
        material=dark_steel,
        name="hinge_web",
    )
    for side_index, y in enumerate((-0.835, 0.835)):
        leaf.visual(
            Cylinder(radius=0.118, length=0.23),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"shaft_collar_{side_index}",
        )
    for stripe_index, y in enumerate((-0.16, 0.16)):
        leaf.visual(
            Box((2.46, 0.035, 0.006)),
            origin=Origin(xyz=(1.70, y, 0.001)),
            material=yellow,
            name=f"lane_stripe_{stripe_index}",
        )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35000.0, velocity=0.35, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    for side_index in (0, 1):
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a=f"shaft_collar_{side_index}",
            elem_b=f"bearing_ring_{side_index}",
            reason="The trunnion collar is intentionally seated into the heavy side bearing ring.",
        )

    ctx.check(
        "single revolute leaf",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.child == "bridge_leaf"
        and hinge.parent == "shore_frame",
        details=f"type={hinge.articulation_type}, parent={hinge.parent}, child={hinge.child}",
    )
    ctx.check(
        "horizontal shore-side hinge axis",
        tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="xy",
        min_overlap=1.20,
        elem_a="steel_leaf_panel",
        elem_b="channel_water",
        name="closed leaf spans the channel opening",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        min_gap=0.28,
        positive_elem="steel_leaf_panel",
        negative_elem="channel_water",
        name="leaf deck is shallow above water",
    )
    for side_index in (0, 1):
        ctx.expect_overlap(
            leaf,
            frame,
            axes="y",
            min_overlap=0.025,
            elem_a=f"shaft_collar_{side_index}",
            elem_b=f"bearing_ring_{side_index}",
            name=f"bearing collar {side_index} is retained in side bearing",
        )

    closed_aabb = ctx.part_element_world_aabb(leaf, elem="steel_leaf_panel")
    with ctx.pose({hinge: 1.25}):
        raised_aabb = ctx.part_element_world_aabb(leaf, elem="steel_leaf_panel")
    ctx.check(
        "leaf raises away from the opening",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 1.5,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

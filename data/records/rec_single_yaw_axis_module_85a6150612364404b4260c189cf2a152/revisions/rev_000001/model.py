from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_only_turntable")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_metal = model.material("brushed_aluminum", rgba=(0.60, 0.62, 0.60, 1.0))
    bolt_black = model.material("blackened_fastener", rgba=(0.01, 0.01, 0.012, 1.0))
    index_white = model.material("white_index_paint", rgba=(0.92, 0.92, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_black,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.158, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_metal,
        name="upper_step",
    )
    base.visual(
        Cylinder(radius=0.128, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=brushed_metal,
        name="bearing_race",
    )
    # Low rubber pads keep the rotary base visibly grounded without adding
    # another articulation.
    for i, (x, y) in enumerate(
        (
            (0.105, 0.105),
            (-0.105, 0.105),
            (-0.105, -0.105),
            (0.105, -0.105),
        )
    ):
        base.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber_black,
            name=f"foot_{i}",
        )

    top_plate = model.part("top_plate")
    top_plate.visual(
        Cylinder(radius=0.145, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_metal,
        name="top_disk",
    )
    top_plate.visual(
        Cylinder(radius=0.116, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_metal,
        name="lower_rotor_shadow",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        x = 0.108 * math.cos(angle)
        y = 0.108 * math.sin(angle)
        top_plate.visual(
            Cylinder(radius=0.0065, length=0.0012),
            origin=Origin(xyz=(x, y, 0.0166)),
            material=bolt_black,
            name=f"bolt_socket_{i}",
        )
    top_plate.visual(
        Box((0.050, 0.010, 0.0015)),
        origin=Origin(xyz=(0.103, 0.0, 0.01675)),
        material=index_white,
        name="index_mark",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.065, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.032, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=dark_metal,
        name="center_post",
    )
    pedestal.visual(
        Box((0.016, 0.110, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=dark_metal,
        name="long_gusset",
    )
    pedestal.visual(
        Box((0.110, 0.016, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=dark_metal,
        name="cross_gusset",
    )
    pedestal.visual(
        Box((0.150, 0.095, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=dark_metal,
        name="payload_plate",
    )
    for i, (x, y) in enumerate(
        (
            (0.052, 0.030),
            (-0.052, 0.030),
            (-0.052, -0.030),
            (0.052, -0.030),
        )
    ):
        pedestal.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.131)),
            material=brushed_metal,
            name=f"threaded_boss_{i}",
        )
        pedestal.visual(
            Cylinder(radius=0.0045, length=0.001),
            origin=Origin(xyz=(x, y, 0.1345)),
            material=bolt_black,
            name=f"boss_socket_{i}",
        )

    pan = model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    pan.meta["description"] = "Single vertical yaw axis for pan-only sensor pointing."

    model.articulation(
        "pedestal_mount",
        ArticulationType.FIXED,
        parent=top_plate,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
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

    base = object_model.get_part("base")
    top_plate = object_model.get_part("top_plate")
    pedestal = object_model.get_part("pedestal")
    pan = object_model.get_articulation("pan_axis")

    ctx.check(
        "pan joint is continuous yaw",
        pan.articulation_type == ArticulationType.CONTINUOUS and tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )

    with ctx.pose({pan: 0.0}):
        ctx.expect_contact(
            top_plate,
            base,
            elem_a="top_disk",
            elem_b="bearing_race",
            contact_tol=0.001,
            name="top plate is supported on base bearing race",
        )
        ctx.expect_contact(
            pedestal,
            top_plate,
            elem_a="base_flange",
            elem_b="top_disk",
            contact_tol=0.001,
            name="pedestal flange is bolted to top plate",
        )
        ctx.expect_within(
            pedestal,
            top_plate,
            axes="xy",
            inner_elem="base_flange",
            outer_elem="top_disk",
            margin=0.0,
            name="pedestal flange sits within circular plate",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({pan: 0.0}):
        rest_mark = aabb_center(ctx.part_element_world_aabb(top_plate, elem="index_mark"))
    with ctx.pose({pan: math.pi / 2.0}):
        turned_mark = aabb_center(ctx.part_element_world_aabb(top_plate, elem="index_mark"))
    ctx.check(
        "top plate visibly pans about vertical axis",
        rest_mark is not None
        and turned_mark is not None
        and rest_mark[0] > 0.095
        and abs(rest_mark[1]) < 0.020
        and turned_mark[1] > 0.095
        and abs(turned_mark[0]) < 0.020,
        details=f"rest_mark={rest_mark}, turned_mark={turned_mark}",
    )

    return ctx.report()


object_model = build_object_model()

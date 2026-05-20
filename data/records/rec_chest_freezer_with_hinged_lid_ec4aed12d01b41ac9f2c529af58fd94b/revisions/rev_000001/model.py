from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_lid_chest_cooler")

    blue = model.material("blue_plastic", rgba=(0.03, 0.24, 0.58, 1.0))
    white = model.material("white_liner", rgba=(0.94, 0.96, 0.92, 1.0))
    light = model.material("light_lid", rgba=(0.88, 0.93, 0.94, 1.0))
    gray = model.material("gray_rail", rgba=(0.42, 0.44, 0.46, 1.0))
    dark = model.material("dark_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    steel = model.material("pin_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.80, 0.42, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue,
        name="bottom_shell",
    )
    body.visual(
        Box((0.80, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, 0.20, 0.185)),
        material=blue,
        name="side_wall_0",
    )
    body.visual(
        Box((0.80, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, -0.20, 0.185)),
        material=blue,
        name="side_wall_1",
    )
    body.visual(
        Box((0.04, 0.42, 0.32)),
        origin=Origin(xyz=(0.38, 0.0, 0.185)),
        material=blue,
        name="end_wall_0",
    )
    body.visual(
        Box((0.04, 0.42, 0.32)),
        origin=Origin(xyz=(-0.38, 0.0, 0.185)),
        material=blue,
        name="end_wall_1",
    )

    # Bright liner surfaces inside the open chest make the body read as hollow.
    body.visual(
        Box((0.69, 0.29, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=white,
        name="liner_floor",
    )
    body.visual(
        Box((0.69, 0.010, 0.22)),
        origin=Origin(xyz=(0.0, 0.1755, 0.185)),
        material=white,
        name="liner_side_0",
    )
    body.visual(
        Box((0.69, 0.010, 0.22)),
        origin=Origin(xyz=(0.0, -0.1755, 0.185)),
        material=white,
        name="liner_side_1",
    )
    body.visual(
        Box((0.010, 0.30, 0.22)),
        origin=Origin(xyz=(0.3555, 0.0, 0.185)),
        material=white,
        name="liner_end_0",
    )
    body.visual(
        Box((0.010, 0.30, 0.22)),
        origin=Origin(xyz=(-0.3555, 0.0, 0.185)),
        material=white,
        name="liner_end_1",
    )

    # Top rim and low-friction guide tracks along the long opening edges.
    body.visual(
        Box((0.82, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, 0.195, 0.360)),
        material=blue,
        name="top_rim_0",
    )
    body.visual(
        Box((0.82, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.195, 0.360)),
        material=blue,
        name="top_rim_1",
    )
    body.visual(
        Box((0.055, 0.42, 0.030)),
        origin=Origin(xyz=(0.382, 0.0, 0.360)),
        material=blue,
        name="end_rim_0",
    )
    body.visual(
        Box((0.055, 0.42, 0.030)),
        origin=Origin(xyz=(-0.382, 0.0, 0.360)),
        material=blue,
        name="end_rim_1",
    )
    body.visual(
        Box((0.74, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.145, 0.384)),
        material=gray,
        name="rail_0",
    )
    body.visual(
        Box((0.74, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.145, 0.384)),
        material=gray,
        name="rail_1",
    )
    body.visual(
        Box((0.74, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, 0.167, 0.393)),
        material=gray,
        name="rail_lip_0",
    )
    body.visual(
        Box((0.74, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, -0.167, 0.393)),
        material=gray,
        name="rail_lip_1",
    )

    for x in (-0.29, 0.29):
        for y in (-0.145, 0.145):
            body.visual(
                Box((0.075, 0.045, 0.020)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=dark,
                name=f"foot_{x}_{y}",
            )
    body.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.407, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="drain_plug",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.82, 0.44, 0.40)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.62, 0.32, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=light,
        name="lid_panel",
    )
    lid.visual(
        Box((0.58, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.145, -0.021)),
        material=gray,
        name="runner_0",
    )
    lid.visual(
        Box((0.58, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.145, -0.021)),
        material=gray,
        name="runner_1",
    )
    lid.visual(
        Box((0.52, 0.22, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=white,
        name="recess_panel",
    )
    for sx in (-1.0, 1.0):
        # Cheek blocks carry the small transverse pivot pins without touching the rotating eyelets.
        lid.visual(
            Box((0.026, 0.040, 0.074)),
            origin=Origin(xyz=(sx * 0.305, -0.145, 0.037)),
            material=light,
            name=f"pin_block_{sx}",
        )
        lid.visual(
            Cylinder(radius=0.011, length=0.055),
            origin=Origin(
                xyz=(sx * 0.270, -0.145, 0.055),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"pivot_pin_{sx}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.32, 0.075)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    handle_path = [
        (-0.270, 0.020, 0.006),
        (-0.270, 0.080, 0.009),
        (-0.270, 0.245, 0.010),
        (-0.135, 0.268, 0.010),
        (0.0, 0.272, 0.010),
        (0.135, 0.268, 0.010),
        (0.270, 0.245, 0.010),
        (0.270, 0.080, 0.009),
        (0.270, 0.020, 0.006),
    ]
    handle_geom = tube_from_spline_points(
        handle_path,
        radius=0.010,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    for sx in (-1.0, 1.0):
        handle_geom.merge(
            TorusGeometry(radius=0.018, tube=0.007, radial_segments=28, tubular_segments=12)
            .rotate_y(math.pi / 2.0)
            .translate(sx * 0.270, 0.0, 0.0)
        )
    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(handle_geom, "swing_handle"),
        origin=Origin(),
        material=dark,
        name="handle_bar",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.58, 0.30, 0.07)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.14, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.4220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.30),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -0.145, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("body_to_lid")
    swing = object_model.get_articulation("lid_to_handle")

    for pin_name in ("pivot_pin_-1.0", "pivot_pin_1.0"):
        ctx.allow_overlap(
            handle,
            lid,
            elem_a="handle_bar",
            elem_b=pin_name,
            reason="The swing handle eye is intentionally captured around the lid pivot pin.",
        )
        ctx.expect_overlap(
            handle,
            lid,
            axes="x",
            elem_a="handle_bar",
            elem_b=pin_name,
            min_overlap=0.010,
            name=f"{pin_name} passes through the handle eye",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="liner_floor",
        min_overlap=0.20,
        name="closed lid spans the cooler opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="runner_0",
        negative_elem="rail_0",
        max_gap=0.006,
        max_penetration=0.0005,
        name="near runner rides just above guide rail",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="runner_1",
        negative_elem="rail_1",
        max_gap=0.006,
        max_penetration=0.0005,
        name="far runner rides just above guide rail",
    )

    rest_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({slide: 0.30}):
        extended_lid_pos = ctx.part_world_position(lid)
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="runner_0",
            elem_b="rail_0",
            min_overlap=0.25,
            name="slid lid retains near rail insertion",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="runner_1",
            elem_b="rail_1",
            min_overlap=0.25,
            name="slid lid retains far rail insertion",
        )
    ctx.check(
        "lid translates along the rail direction",
        rest_lid_pos is not None
        and extended_lid_pos is not None
        and extended_lid_pos[0] > rest_lid_pos[0] + 0.25,
        details=f"rest={rest_lid_pos}, extended={extended_lid_pos}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({swing: 1.35}):
        raised_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle swings upward from pivot pins",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.18,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

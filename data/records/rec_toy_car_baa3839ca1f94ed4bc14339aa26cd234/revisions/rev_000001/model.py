from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_xz_beam(
    part,
    *,
    name: str,
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    y: float,
    width_y: float,
    width_z: float,
    material,
) -> None:
    dx = x1 - x0
    dz = z1 - z0
    length = sqrt(dx * dx + dz * dz)
    angle = atan2(dz, dx)
    part.visual(
        Box((length, width_y, width_z)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y, (z0 + z1) * 0.5),
            rpy=(0.0, -angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback_car")

    paint = model.material("paint_red", rgba=(0.84, 0.14, 0.12, 1.0))
    trim = model.material("trim_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    glass = model.material("smoke_glass", rgba=(0.35, 0.45, 0.58, 0.78))
    tire = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hub = model.material("hub_silver", rgba=(0.76, 0.78, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.28, 0.15, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=trim,
        name="floor_pan",
    )
    body.visual(
        Box((0.03, 0.16, 0.03)),
        origin=Origin(xyz=(0.155, 0.0, 0.05)),
        material=trim,
        name="front_bumper",
    )
    body.visual(
        Box((0.11, 0.16, 0.04)),
        origin=Origin(xyz=(0.095, 0.0, 0.07)),
        material=paint,
        name="nose_block",
    )
    body.visual(
        Box((0.10, 0.15, 0.024)),
        origin=Origin(xyz=(0.08, 0.0, 0.101)),
        material=paint,
        name="hood",
    )
    body.visual(
        Box((0.028, 0.15, 0.052)),
        origin=Origin(xyz=(0.022, 0.0, 0.115)),
        material=paint,
        name="cowl",
    )
    body.visual(
        Box((0.205, 0.15, 0.012)),
        origin=Origin(xyz=(-0.03, 0.0, 0.149)),
        material=paint,
        name="roof",
    )
    body.visual(
        Box((0.24, 0.012, 0.056)),
        origin=Origin(xyz=(-0.01, 0.069, 0.078)),
        material=paint,
        name="left_sill",
    )
    body.visual(
        Box((0.24, 0.012, 0.056)),
        origin=Origin(xyz=(-0.01, -0.069, 0.078)),
        material=paint,
        name="right_sill",
    )
    body.visual(
        Box((0.012, 0.012, 0.08)),
        origin=Origin(xyz=(-0.086, 0.069, 0.11)),
        material=paint,
        name="left_b_pillar",
    )
    body.visual(
        Box((0.012, 0.012, 0.08)),
        origin=Origin(xyz=(-0.086, -0.069, 0.11)),
        material=paint,
        name="right_b_pillar",
    )
    body.visual(
        Box((0.024, 0.15, 0.052)),
        origin=Origin(xyz=(-0.105, 0.0, 0.09)),
        material=paint,
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.04, 0.16, 0.045)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0575)),
        material=trim,
        name="rear_bumper",
    )
    body.visual(
        Box((0.07, 0.012, 0.055)),
        origin=Origin(xyz=(0.12, 0.069, 0.1025)),
        material=paint,
        name="left_front_fender",
    )
    body.visual(
        Box((0.07, 0.012, 0.055)),
        origin=Origin(xyz=(0.12, -0.069, 0.1025)),
        material=paint,
        name="right_front_fender",
    )
    body.visual(
        Box((0.07, 0.012, 0.055)),
        origin=Origin(xyz=(-0.12, 0.069, 0.1025)),
        material=paint,
        name="left_rear_fender",
    )
    body.visual(
        Box((0.07, 0.012, 0.055)),
        origin=Origin(xyz=(-0.12, -0.069, 0.1025)),
        material=paint,
        name="right_rear_fender",
    )
    body.visual(
        Box((0.05, 0.15, 0.012)),
        origin=Origin(xyz=(-0.106, 0.0, 0.149)),
        material=paint,
        name="rear_header",
    )
    for axle_name, axle_x, axle_y in (
        ("front_left_axle_stub", 0.095, 0.082),
        ("front_right_axle_stub", 0.095, -0.082),
        ("rear_left_axle_stub", -0.095, 0.082),
        ("rear_right_axle_stub", -0.095, -0.082),
    ):
        body.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(axle_x, axle_y, 0.035), rpy=(pi * 0.5, 0.0, 0.0)),
            material=trim,
            name=axle_name,
        )

    _add_xz_beam(
        body,
        name="left_a_pillar",
        x0=0.03,
        z0=0.10,
        x1=-0.02,
        z1=0.149,
        y=0.069,
        width_y=0.012,
        width_z=0.012,
        material=paint,
    )
    _add_xz_beam(
        body,
        name="right_a_pillar",
        x0=0.03,
        z0=0.10,
        x1=-0.02,
        z1=0.149,
        y=-0.069,
        width_y=0.012,
        width_z=0.012,
        material=paint,
    )
    _add_xz_beam(
        body,
        name="left_c_pillar",
        x0=-0.09,
        z0=0.149,
        x1=-0.145,
        z1=0.114,
        y=0.069,
        width_y=0.012,
        width_z=0.012,
        material=paint,
    )
    _add_xz_beam(
        body,
        name="right_c_pillar",
        x0=-0.09,
        z0=0.149,
        x1=-0.145,
        z1=0.114,
        y=-0.069,
        width_y=0.012,
        width_z=0.012,
        material=paint,
    )

    body.visual(
        Box((0.006, 0.136, 0.078)),
        origin=Origin(xyz=(0.004, 0.0, 0.12), rpy=(0.0, 0.74, 0.0)),
        material=glass,
        name="windshield",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.084, 0.008, 0.086)),
        origin=Origin(xyz=(-0.042, 0.0, 0.043)),
        material=paint,
        name="door_shell",
    )
    left_door.visual(
        Box((0.05, 0.004, 0.03)),
        origin=Origin(xyz=(-0.046, 0.0015, 0.056)),
        material=glass,
        name="door_window",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.084, 0.008, 0.086)),
        origin=Origin(xyz=(-0.042, 0.0, 0.043)),
        material=paint,
        name="door_shell",
    )
    right_door.visual(
        Box((0.05, 0.004, 0.03)),
        origin=Origin(xyz=(-0.046, -0.0015, 0.056)),
        material=glass,
        name="door_window",
    )

    liftgate = model.part("liftgate")
    liftgate.visual(
        Box((0.016, 0.122, 0.01)),
        origin=Origin(xyz=(-0.012, 0.0, -0.018)),
        material=paint,
        name="liftgate_top_rail",
    )
    _add_xz_beam(
        liftgate,
        name="liftgate_left_rail",
        x0=-0.014,
        z0=-0.02,
        x1=-0.014,
        z1=-0.064,
        y=0.057,
        width_y=0.008,
        width_z=0.01,
        material=paint,
    )
    _add_xz_beam(
        liftgate,
        name="liftgate_right_rail",
        x0=-0.014,
        z0=-0.02,
        x1=-0.014,
        z1=-0.064,
        y=-0.057,
        width_y=0.008,
        width_z=0.01,
        material=paint,
    )
    liftgate.visual(
        Box((0.016, 0.118, 0.01)),
        origin=Origin(xyz=(-0.014, 0.0, -0.064)),
        material=paint,
        name="liftgate_lower_rail",
    )
    liftgate.visual(
        Box((0.008, 0.104, 0.058)),
        origin=Origin(xyz=(-0.022, 0.0, -0.039), rpy=(0.0, 0.55, 0.0)),
        material=paint,
        name="liftgate_shell",
    )
    liftgate.visual(
        Box((0.004, 0.092, 0.03)),
        origin=Origin(xyz=(-0.016, 0.0, -0.032), rpy=(0.0, 0.55, 0.0)),
        material=glass,
        name="liftgate_window",
    )
    liftgate.visual(
        Box((0.01, 0.014, 0.007)),
        origin=Origin(xyz=(-0.012, 0.043, -0.0095)),
        material=paint,
        name="liftgate_left_hinge_leaf",
    )
    liftgate.visual(
        Box((0.01, 0.014, 0.007)),
        origin=Origin(xyz=(-0.012, -0.043, -0.0095)),
        material=paint,
        name="liftgate_right_hinge_leaf",
    )

    wheel_specs = (
        ("front_left_wheel", 0.095, 0.10),
        ("front_right_wheel", 0.095, -0.10),
        ("rear_left_wheel", -0.095, 0.10),
        ("rear_right_wheel", -0.095, -0.10),
    )
    for wheel_name, wheel_x, wheel_y in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.035, length=0.022),
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
            material=tire,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
            material=hub,
            name="hub",
        )
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(wheel_x, wheel_y, 0.035)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    left_hinge = model.articulation(
        "body_to_left_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(0.005, 0.079, 0.055)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    right_hinge = model.articulation(
        "body_to_right_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(0.005, -0.079, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    liftgate_hinge = model.articulation(
        "body_to_liftgate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=liftgate,
        origin=Origin(xyz=(-0.108, 0.0, 0.149)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.2),
    )

    body.meta["primary_articulations"] = (
        left_hinge.name,
        right_hinge.name,
        liftgate_hinge.name,
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
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    liftgate = object_model.get_part("liftgate")
    wheel_names = (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )
    wheel_joints = [
        object_model.get_articulation(f"body_to_{wheel_name}")
        for wheel_name in wheel_names
    ]
    left_hinge = object_model.get_articulation("body_to_left_door")
    right_hinge = object_model.get_articulation("body_to_right_door")
    liftgate_hinge = object_model.get_articulation("body_to_liftgate")

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        max_gap=0.01,
        max_penetration=1e-5,
        negative_elem="left_sill",
        name="left door sits flush to body side",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        max_gap=0.01,
        max_penetration=1e-5,
        positive_elem="right_sill",
        name="right door sits flush to body side",
    )
    ctx.expect_overlap(
        left_door,
        body,
        axes="xz",
        min_overlap=0.055,
        name="left door covers its side opening",
    )
    ctx.expect_overlap(
        right_door,
        body,
        axes="xz",
        min_overlap=0.055,
        name="right door covers its side opening",
    )
    ctx.expect_overlap(
        liftgate,
        body,
        axes="yz",
        min_overlap=0.06,
        name="liftgate spans the rear opening",
    )
    ctx.expect_contact(
        liftgate,
        body,
        elem_a="liftgate_left_hinge_leaf",
        elem_b="rear_header",
        contact_tol=1e-6,
        name="liftgate is supported at the roof hinge line",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(left_door, elem="door_shell"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(right_door, elem="door_shell"))
    hatch_rest = _aabb_center(ctx.part_element_world_aabb(liftgate, elem="liftgate_shell"))

    with ctx.pose({left_hinge: 0.9}):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_door, elem="door_shell"))
    ctx.check(
        "left door opens outward",
        left_rest is not None
        and left_open is not None
        and left_open[1] > left_rest[1] + 0.02,
        details=f"rest={left_rest}, open={left_open}",
    )

    with ctx.pose({right_hinge: 0.9}):
        right_open = _aabb_center(ctx.part_element_world_aabb(right_door, elem="door_shell"))
    ctx.check(
        "right door opens outward",
        right_rest is not None
        and right_open is not None
        and right_open[1] < right_rest[1] - 0.02,
        details=f"rest={right_rest}, open={right_open}",
    )

    with ctx.pose({liftgate_hinge: 1.0}):
        hatch_open = _aabb_center(ctx.part_element_world_aabb(liftgate, elem="liftgate_shell"))
    ctx.check(
        "liftgate swings upward",
        hatch_rest is not None
        and hatch_open is not None
        and hatch_open[2] > hatch_rest[2] + 0.02
        and hatch_open[0] < hatch_rest[0] - 0.01,
        details=f"rest={hatch_rest}, open={hatch_open}",
    )

    for wheel_name, wheel_joint in zip(wheel_names, wheel_joints):
        limits = wheel_joint.motion_limits
        ctx.check(
            f"{wheel_name} is a continuous axle joint",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=(
                f"type={wheel_joint.articulation_type}, "
                f"axis={wheel_joint.axis}, "
                f"limits={limits}"
            ),
        )

    ctx.check(
        "door hinges are vertical",
        tuple(left_hinge.axis) == (0.0, 0.0, -1.0)
        and tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "liftgate hinge is roof mounted and horizontal",
        tuple(liftgate_hinge.axis) == (0.0, 1.0, 0.0)
        and abs(liftgate_hinge.origin.xyz[2] - 0.149) < 1e-6,
        details=f"axis={liftgate_hinge.axis}, origin={liftgate_hinge.origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

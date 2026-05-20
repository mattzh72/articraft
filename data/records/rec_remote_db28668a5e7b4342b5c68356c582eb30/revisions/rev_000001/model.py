from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _managed_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drone_flight_controller")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    metal_mid = model.material("metal_mid", rgba=(0.44, 0.46, 0.50, 1.0))
    cover_plastic = model.material("cover_plastic", rgba=(0.19, 0.20, 0.22, 1.0))

    top_shell_mesh = _managed_mesh(
        "controller_top_shell",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.206, 0.104, 0.019, corner_segments=8),
            0.012,
            cap=True,
            closed=True,
        ),
    )

    controller_body = model.part("controller_body")
    controller_body.visual(
        top_shell_mesh,
        origin=Origin(xyz=(0.0, -0.002, 0.034)),
        material=body_plastic,
        name="top_shell",
    )
    controller_body.visual(
        Box((0.164, 0.062, 0.028)),
        origin=Origin(xyz=(0.0, 0.010, 0.014)),
        material=body_plastic,
        name="front_core",
    )
    controller_body.visual(
        Box((0.164, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.021, 0.012)),
        material=body_plastic,
        name="rear_bridge",
    )
    controller_body.visual(
        Box((0.128, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, 0.005)),
        material=trim_dark,
        name="bottom_plate",
    )
    for x_sign, name in ((-1.0, "left_grip"), (1.0, "right_grip")):
        controller_body.visual(
            Cylinder(radius=0.022, length=0.090),
            origin=Origin(
                xyz=(0.082 * x_sign, -0.002, 0.018),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=grip_rubber,
            name=name,
        )
        controller_body.visual(
            Cylinder(radius=0.0155, length=0.090),
            origin=Origin(
                xyz=(0.082 * x_sign, -0.002, 0.026),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=body_plastic,
            name=f"{name}_upper",
        )

    controller_body.visual(
        Box((0.092, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.034, 0.020)),
        material=trim_dark,
        name="battery_bay_inner_wall",
    )
    controller_body.visual(
        Box((0.006, 0.020, 0.028)),
        origin=Origin(xyz=(-0.047, -0.043, 0.020)),
        material=trim_dark,
        name="battery_bay_left_wall",
    )
    controller_body.visual(
        Box((0.006, 0.020, 0.028)),
        origin=Origin(xyz=(0.047, -0.043, 0.020)),
        material=trim_dark,
        name="battery_bay_right_wall",
    )
    controller_body.visual(
        Box((0.100, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.043, 0.006)),
        material=trim_dark,
        name="battery_bay_floor",
    )
    controller_body.visual(
        Box((0.100, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.039, 0.033)),
        material=trim_dark,
        name="battery_bay_lintel",
    )

    for x_pos, prefix in ((-0.050, "left"), (0.050, "right")):
        controller_body.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(x_pos, 0.008, 0.040)),
            material=trim_dark,
            name=f"{prefix}_gimbal_bezel",
        )
        controller_body.visual(
            Cylinder(radius=0.0115, length=0.006),
            origin=Origin(xyz=(x_pos, 0.008, 0.041)),
            material=metal_mid,
            name=f"{prefix}_gimbal_socket",
        )

    controller_body.inertial = Inertial.from_geometry(
        Box((0.214, 0.110, 0.052)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    left_joystick = model.part("left_joystick")
    left_joystick.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal_dark,
        name="left_stick_shaft",
    )
    left_joystick.visual(
        Cylinder(radius=0.0135, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=metal_mid,
        name="left_stick_cap",
    )
    left_joystick.visual(
        Box((0.016, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0065, 0.037)),
        material=trim_dark,
        name="left_stick_nub",
    )
    left_joystick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.040),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    right_joystick = model.part("right_joystick")
    right_joystick.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal_dark,
        name="right_stick_shaft",
    )
    right_joystick.visual(
        Cylinder(radius=0.0135, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=metal_mid,
        name="right_stick_cap",
    )
    right_joystick.visual(
        Box((0.016, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0065, 0.037)),
        material=trim_dark,
        name="right_stick_nub",
    )
    right_joystick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.040),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    battery_cover = model.part("battery_cover")
    battery_cover.visual(
        Box((0.100, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.002, -0.020)),
        material=cover_plastic,
        name="cover_panel",
    )
    battery_cover.visual(
        Box((0.030, 0.0025, 0.006)),
        origin=Origin(xyz=(0.0, -0.005, -0.033)),
        material=trim_dark,
        name="cover_latch_ridge",
    )
    battery_cover.inertial = Inertial.from_geometry(
        Box((0.100, 0.006, 0.040)),
        mass=0.040,
        origin=Origin(xyz=(0.0, -0.003, -0.020)),
    )

    model.articulation(
        "left_joystick_yaw",
        ArticulationType.CONTINUOUS,
        parent=controller_body,
        child=left_joystick,
        origin=Origin(xyz=(-0.050, 0.008, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "right_joystick_yaw",
        ArticulationType.CONTINUOUS,
        parent=controller_body,
        child=right_joystick,
        origin=Origin(xyz=(0.050, 0.008, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "battery_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=controller_body,
        child=battery_cover,
        origin=Origin(xyz=(0.0, -0.054, 0.040)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.75),
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

    controller_body = object_model.get_part("controller_body")
    left_joystick = object_model.get_part("left_joystick")
    right_joystick = object_model.get_part("right_joystick")
    battery_cover = object_model.get_part("battery_cover")

    left_joint = object_model.get_articulation("left_joystick_yaw")
    right_joint = object_model.get_articulation("right_joystick_yaw")
    cover_joint = object_model.get_articulation("battery_cover_hinge")

    ctx.check(
        "left joystick uses a continuous vertical articulation",
        left_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_joint.axis) == (0.0, 0.0, 1.0)
        and left_joint.motion_limits is not None
        and left_joint.motion_limits.lower is None
        and left_joint.motion_limits.upper is None,
        details=f"type={left_joint.articulation_type}, axis={left_joint.axis}, limits={left_joint.motion_limits}",
    )
    ctx.check(
        "right joystick uses a continuous vertical articulation",
        right_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_joint.axis) == (0.0, 0.0, 1.0)
        and right_joint.motion_limits is not None
        and right_joint.motion_limits.lower is None
        and right_joint.motion_limits.upper is None,
        details=f"type={right_joint.articulation_type}, axis={right_joint.axis}, limits={right_joint.motion_limits}",
    )
    ctx.check(
        "battery cover hinge is top mounted",
        cover_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(cover_joint.axis) == (-1.0, 0.0, 0.0)
        and cover_joint.motion_limits is not None
        and cover_joint.motion_limits.lower == 0.0
        and cover_joint.motion_limits.upper is not None
        and cover_joint.motion_limits.upper >= 1.5,
        details=f"type={cover_joint.articulation_type}, axis={cover_joint.axis}, limits={cover_joint.motion_limits}",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_overlap(
            left_joystick,
            controller_body,
            axes="xy",
            min_overlap=0.020,
            name="left joystick sits within the top-face footprint",
        )
        ctx.expect_gap(
            left_joystick,
            controller_body,
            axis="z",
            max_gap=0.002,
            max_penetration=1e-5,
            name="left joystick seats onto the top face",
        )
        ctx.expect_overlap(
            right_joystick,
            controller_body,
            axes="xy",
            min_overlap=0.020,
            name="right joystick sits within the top-face footprint",
        )
        ctx.expect_gap(
            right_joystick,
            controller_body,
            axis="z",
            max_gap=0.002,
            max_penetration=1e-5,
            name="right joystick seats onto the top face",
        )
        ctx.expect_overlap(
            battery_cover,
            controller_body,
            axes="xz",
            min_overlap=0.030,
            name="battery cover matches the rear compartment opening",
        )
        ctx.expect_gap(
            controller_body,
            battery_cover,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            name="battery cover closes flush to the rear face",
        )

        left_nub_rest = _aabb_center(
            ctx.part_element_world_aabb(left_joystick, elem="left_stick_nub")
        )
        right_nub_rest = _aabb_center(
            ctx.part_element_world_aabb(right_joystick, elem="right_stick_nub")
        )
        cover_center_rest = _aabb_center(
            ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
        )

    with ctx.pose({left_joint: pi / 2.0}):
        left_nub_turned = _aabb_center(
            ctx.part_element_world_aabb(left_joystick, elem="left_stick_nub")
        )

    with ctx.pose({right_joint: pi / 2.0}):
        right_nub_turned = _aabb_center(
            ctx.part_element_world_aabb(right_joystick, elem="right_stick_nub")
        )

    with ctx.pose({cover_joint: 1.40}):
        cover_center_open = _aabb_center(
            ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
        )

    ctx.check(
        "left joystick nub rotates around the vertical stick axis",
        left_nub_rest is not None
        and left_nub_turned is not None
        and abs(left_nub_turned[0] - left_nub_rest[0]) > 0.004
        and abs(left_nub_turned[1] - left_nub_rest[1]) > 0.004
        and abs(left_nub_turned[2] - left_nub_rest[2]) < 0.001,
        details=f"rest={left_nub_rest}, turned={left_nub_turned}",
    )
    ctx.check(
        "right joystick nub rotates around the vertical stick axis",
        right_nub_rest is not None
        and right_nub_turned is not None
        and abs(right_nub_turned[0] - right_nub_rest[0]) > 0.004
        and abs(right_nub_turned[1] - right_nub_rest[1]) > 0.004
        and abs(right_nub_turned[2] - right_nub_rest[2]) < 0.001,
        details=f"rest={right_nub_rest}, turned={right_nub_turned}",
    )
    ctx.check(
        "battery cover swings outward and upward when opened",
        cover_center_rest is not None
        and cover_center_open is not None
        and cover_center_open[1] < cover_center_rest[1] - 0.012
        and cover_center_open[2] > cover_center_rest[2] + 0.012,
        details=f"closed={cover_center_rest}, open={cover_center_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

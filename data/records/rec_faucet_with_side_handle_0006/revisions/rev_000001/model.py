from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

_X_AXIS_CYL = (0.0, math.pi / 2.0, 0.0)
_Y_AXIS_CYL = (math.pi / 2.0, 0.0, 0.0)

_HANDLE_CENTER_X = 0.112
_HANDLE_JOINT_Y = 0.034


def _rod(part, *, name: str, radius: float, length: float, center, axis: str, material) -> None:
    rpy = (0.0, 0.0, 0.0)
    if axis == "x":
        rpy = _X_AXIS_CYL
    elif axis == "y":
        rpy = _Y_AXIS_CYL
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _cross_handle(part, *, metal, cap) -> None:
    _rod(
        part,
        name="hub",
        radius=0.015,
        length=0.015,
        center=(0.0, 0.0075, 0.0),
        axis="y",
        material=metal,
    )
    part.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=metal,
        name="center_boss",
    )
    _rod(
        part,
        name="arm_x",
        radius=0.0042,
        length=0.060,
        center=(0.0, 0.018, 0.0),
        axis="x",
        material=metal,
    )
    _rod(
        part,
        name="arm_z",
        radius=0.0042,
        length=0.060,
        center=(0.0, 0.018, 0.0),
        axis="z",
        material=metal,
    )
    part.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=_Y_AXIS_CYL),
        material=cap,
        name="face_cap",
    )
    for name, xyz in (
        ("x_plus_tip", (0.030, 0.018, 0.0)),
        ("x_minus_tip", (-0.030, 0.018, 0.0)),
        ("z_plus_tip", (0.0, 0.018, 0.030)),
        ("z_minus_tip", (0.0, 0.018, -0.030)),
    ):
        part.visual(Sphere(radius=0.0052), origin=Origin(xyz=xyz), material=cap, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_bridge_faucet")

    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    brushed = model.material("brushed_nickel", rgba=(0.73, 0.75, 0.78, 1.0))
    porcelain = model.material("porcelain", rgba=(0.96, 0.96, 0.94, 1.0))

    body = model.part("body")
    for side, x_pos in (("left", -_HANDLE_CENTER_X), ("right", _HANDLE_CENTER_X)):
        _rod(
            body,
            name=f"{side}_escutcheon",
            radius=0.030,
            length=0.008,
            center=(x_pos, 0.004, 0.0),
            axis="y",
            material=brushed,
        )
        body.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=(x_pos, 0.016, 0.0)),
            material=brushed,
            name=f"{side}_escutcheon_bell",
        )
        _rod(
            body,
            name=f"{side}_inlet_sleeve",
            radius=0.013,
            length=0.020,
            center=(x_pos, 0.017, 0.0),
            axis="y",
            material=chrome,
        )
        _rod(
            body,
            name=f"{side}_valve_seat",
            radius=0.020,
            length=0.010,
            center=(x_pos, 0.029, 0.0),
            axis="y",
            material=chrome,
        )

    _rod(
        body,
        name="bridge_tube",
        radius=0.009,
        length=0.188,
        center=(0.0, 0.029, 0.0),
        axis="x",
        material=chrome,
    )
    for side, x_pos in (("left", -0.060), ("right", 0.060)):
        body.visual(
            Sphere(radius=0.0115),
            origin=Origin(xyz=(x_pos, 0.029, 0.0)),
            material=chrome,
            name=f"{side}_bridge_knuckle",
        )
    _rod(
        body,
        name="spout_base",
        radius=0.017,
        length=0.020,
        center=(0.0, 0.038, 0.0),
        axis="y",
        material=chrome,
    )
    body.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.0, 0.039, 0.0)),
        material=chrome,
        name="spout_knuckle",
    )
    _rod(
        body,
        name="spout_tube",
        radius=0.009,
        length=0.062,
        center=(0.0, 0.078, 0.0),
        axis="y",
        material=chrome,
    )
    body.visual(
        Sphere(radius=0.0115),
        origin=Origin(xyz=(0.0, 0.106, 0.0)),
        material=chrome,
        name="aerator_collar",
    )
    _rod(
        body,
        name="aerator_tip",
        radius=0.011,
        length=0.010,
        center=(0.0, 0.114, 0.0),
        axis="y",
        material=chrome,
    )
    _rod(
        body,
        name="outlet_drop",
        radius=0.006,
        length=0.012,
        center=(0.0, 0.119, -0.008),
        axis="z",
        material=chrome,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.08)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
    )

    left_handle = model.part("left_handle")
    _cross_handle(left_handle, metal=chrome, cap=porcelain)
    left_handle.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.070)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    right_handle = model.part("right_handle")
    _cross_handle(right_handle, metal=chrome, cap=porcelain)
    right_handle.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.070)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    model.articulation(
        "left_handle_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_handle,
        origin=Origin(xyz=(-_HANDLE_CENTER_X, _HANDLE_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.1, upper=1.1),
    )
    model.articulation(
        "right_handle_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_handle,
        origin=Origin(xyz=(_HANDLE_CENTER_X, _HANDLE_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.1, upper=1.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_joint")
    right_joint = object_model.get_articulation("right_handle_joint")

    left_escutcheon = body.get_visual("left_escutcheon")
    right_escutcheon = body.get_visual("right_escutcheon")
    left_seat = body.get_visual("left_valve_seat")
    right_seat = body.get_visual("right_valve_seat")
    bridge = body.get_visual("bridge_tube")
    spout = body.get_visual("spout_tube")
    aerator = body.get_visual("aerator_tip")
    left_hub = left_handle.get_visual("hub")
    right_hub = right_handle.get_visual("hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "left_handle_joint_axis",
        tuple(round(value, 6) for value in left_joint.axis) == (0.0, 1.0, 0.0),
        details=f"left handle axis was {left_joint.axis}",
    )
    ctx.check(
        "right_handle_joint_axis",
        tuple(round(value, 6) for value in right_joint.axis) == (0.0, 1.0, 0.0),
        details=f"right handle axis was {right_joint.axis}",
    )
    ctx.check(
        "left_handle_joint_limits",
        left_joint.motion_limits is not None
        and left_joint.motion_limits.lower == -1.1
        and left_joint.motion_limits.upper == 1.1,
        details=f"left handle limits were {left_joint.motion_limits}",
    )
    ctx.check(
        "right_handle_joint_limits",
        right_joint.motion_limits is not None
        and right_joint.motion_limits.lower == -1.1
        and right_joint.motion_limits.upper == 1.1,
        details=f"right handle limits were {right_joint.motion_limits}",
    )

    ctx.expect_overlap(
        body,
        body,
        axes="z",
        elem_a=left_escutcheon,
        elem_b=bridge,
        min_overlap=0.018,
        name="left escutcheon and bridge share the same outlet height",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="z",
        elem_a=right_escutcheon,
        elem_b=bridge,
        min_overlap=0.018,
        name="right escutcheon and bridge share the same outlet height",
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        positive_elem=spout,
        negative_elem=bridge,
        min_gap=0.008,
        max_gap=0.012,
        name="short horizontal spout projects just ahead of the bridge tube",
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        positive_elem=aerator,
        negative_elem=spout,
        min_gap=0.0,
        max_gap=0.002,
        name="aerator continues directly from the short spout tube",
    )
    ctx.expect_gap(
        body,
        left_handle,
        axis="x",
        positive_elem=spout,
        negative_elem=left_hub,
        min_gap=0.065,
        max_gap=0.090,
        name="spout stays centered clear of the left handle",
    )
    ctx.expect_gap(
        right_handle,
        body,
        axis="x",
        positive_elem=right_hub,
        negative_elem=spout,
        min_gap=0.065,
        max_gap=0.090,
        name="spout stays centered clear of the right handle",
    )

    for pose_name, left_angle, right_angle in (
        ("rest", 0.0, 0.0),
        ("left_open_right_closed", 1.1, -1.1),
        ("left_closed_right_open", -1.1, 1.1),
    ):
        with ctx.pose({left_joint: left_angle, right_joint: right_angle}):
            ctx.expect_contact(
                left_handle,
                body,
                elem_a=left_hub,
                elem_b=left_seat,
                name=f"{pose_name}_left_handle_seated",
            )
            ctx.expect_gap(
                left_handle,
                body,
                axis="y",
                positive_elem=left_hub,
                negative_elem=left_seat,
                min_gap=0.0,
                max_gap=0.0005,
                name=f"{pose_name}_left_handle_base_gap",
            )
            ctx.expect_contact(
                right_handle,
                body,
                elem_a=right_hub,
                elem_b=right_seat,
                name=f"{pose_name}_right_handle_seated",
            )
            ctx.expect_gap(
                right_handle,
                body,
                axis="y",
                positive_elem=right_hub,
                negative_elem=right_seat,
                min_gap=0.0,
                max_gap=0.0005,
                name=f"{pose_name}_right_handle_base_gap",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

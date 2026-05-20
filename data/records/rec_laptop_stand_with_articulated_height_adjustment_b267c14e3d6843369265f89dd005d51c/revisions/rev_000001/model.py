from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_L = 0.56
BASE_W = 0.36
BASE_T = 0.035
PIVOT_X = -0.23
PIVOT_Y = 0.135
PIVOT_Z = BASE_T + 0.030
ARM_YAW = -0.45
LOWER_ELBOW = (0.120, 0.0, 0.270)
UPPER_HEAD = (0.100, 0.0, 0.055)
TRAY_L = 0.42
TRAY_W = 0.30
TRAY_T = 0.012
TRAY_X0 = 0.040


def _shifted_profile(width: float, height: float, radius: float, x: float, y: float):
    return [(px + x, py + y) for px, py in rounded_rect_profile(width, height, radius)]


def _box_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    y_size: float,
    z_size: float,
    material: str,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    pitch = -math.atan2(dz, math.sqrt(dx * dx + dy * dy))
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((length, y_size, z_size)),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) / 2.0,
                (start[1] + end[1]) / 2.0,
                (start[2] + end[2]) / 2.0,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_laptop_stand")
    model.material("base_powdercoat", rgba=(0.075, 0.080, 0.085, 1.0))
    model.material("dark_anodized", rgba=(0.015, 0.018, 0.020, 1.0))
    model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("hinge_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    base = model.part("base")
    base_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BASE_L, BASE_W, 0.035),
        [],
        BASE_T,
        center=True,
    )
    base.visual(
        mesh_from_geometry(base_plate, "rounded_base"),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="base_powdercoat",
        name="rounded_base",
    )
    base.visual(
        Box((0.45, 0.25, 0.008)),
        origin=Origin(xyz=(-0.015, 0.0, BASE_T + 0.004)),
        material="dark_anodized",
        name="top_weight",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, BASE_T + 0.015)),
        material="base_powdercoat",
        name="pivot_socket",
    )
    for idx, (fx, fy) in enumerate(
        (
            (-0.22, -0.13),
            (0.22, -0.13),
            (-0.22, 0.13),
            (0.22, 0.13),
        )
    ):
        base.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(fx, fy, -0.002)),
            material="rubber",
            name=f"foot_{idx}",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.046, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material="dark_anodized",
        name="turntable",
    )
    lower_arm.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="brushed_aluminum",
        name="pivot_column",
    )
    lower_arm.visual(
        Box((0.070, 0.115, 0.018)),
        origin=Origin(xyz=(0.022, 0.0, 0.040)),
        material="brushed_aluminum",
        name="base_bridge",
    )
    for side, y in enumerate((-0.050, 0.050)):
        _box_between(
            lower_arm,
            (0.018, y, 0.030),
            (0.116, y, 0.260),
            y_size=0.014,
            z_size=0.016,
            material="brushed_aluminum",
            name=f"lower_bar_{side}",
        )
    lower_arm.visual(
        Box((0.054, 0.094, 0.014)),
        origin=Origin(xyz=(LOWER_ELBOW[0] - 0.030, 0.0, LOWER_ELBOW[2] - 0.036)),
        material="brushed_aluminum",
        name="elbow_bridge",
    )
    for side, y in enumerate((-0.046, 0.046)):
        lower_arm.visual(
            Box((0.056, 0.012, 0.076)),
            origin=Origin(xyz=(LOWER_ELBOW[0] - 0.004, y, LOWER_ELBOW[2])),
            material="brushed_aluminum",
            name=f"elbow_cheek_{side}",
        )
    lower_arm.visual(
        Cylinder(radius=0.009, length=0.088),
        origin=Origin(xyz=LOWER_ELBOW, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hinge_steel",
        name="elbow_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.045, 0.042, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_anodized",
        name="elbow_lug",
    )
    upper_arm.visual(
        Box((0.024, 0.030, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.031)),
        material="dark_anodized",
        name="lug_bridge",
    )
    _box_between(
        upper_arm,
        (0.035, 0.0, 0.014),
        (0.091, 0.0, 0.049),
        y_size=0.026,
        z_size=0.020,
        material="brushed_aluminum",
        name="upper_link",
    )
    upper_arm.visual(
        Box((0.070, 0.040, 0.058)),
        origin=Origin(xyz=(UPPER_HEAD[0] - 0.030, 0.0, UPPER_HEAD[2])),
        material="brushed_aluminum",
        name="tilt_plate",
    )
    upper_arm.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(xyz=UPPER_HEAD, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hinge_steel",
        name="tray_pin",
    )

    tray = model.part("tray")
    vent_holes = []
    for x in (0.130, 0.225, 0.320):
        for y in (-0.060, 0.0, 0.060):
            vent_holes.append(_shifted_profile(0.066, 0.014, 0.006, x, y))
    tray_plate = ExtrudeWithHolesGeometry(
        _shifted_profile(TRAY_L, TRAY_W, 0.025, TRAY_X0 + TRAY_L / 2.0, 0.0),
        vent_holes,
        TRAY_T,
        center=True,
    )
    tray.visual(
        mesh_from_geometry(tray_plate, "vented_tray_plate"),
        material="dark_anodized",
        name="vented_plate",
    )
    tray.visual(
        Box((0.024, TRAY_W, 0.040)),
        origin=Origin(xyz=(TRAY_X0 + TRAY_L - 0.012, 0.0, 0.023)),
        material="dark_anodized",
        name="front_lip",
    )
    for side, y in enumerate((-(TRAY_W / 2.0 - 0.007), TRAY_W / 2.0 - 0.007)):
        tray.visual(
            Box((TRAY_L * 0.82, 0.014, 0.026)),
            origin=Origin(xyz=(TRAY_X0 + TRAY_L * 0.53, y, 0.014)),
            material="dark_anodized",
            name=f"side_lip_{side}",
        )
    for barrel_name, tab_name, y in (
        ("tray_barrel_0", "hinge_tab_0", -0.055),
        ("tray_barrel_1", "hinge_tab_1", 0.055),
    ):
        tray.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_anodized",
            name=barrel_name,
        )
        tray.visual(
            Box((0.052, 0.052, 0.012)),
            origin=Origin(xyz=(0.026, y, -0.007)),
            material="dark_anodized",
            name=tab_name,
        )
    for idx, (x, y) in enumerate(
        (
            (0.115, -0.088),
            (0.115, 0.088),
            (0.295, -0.088),
            (0.295, 0.088),
        )
    ):
        tray.visual(
            Box((0.100, 0.020, 0.004)),
            origin=Origin(xyz=(TRAY_X0 + x, y, 0.008)),
            material="rubber",
            name=f"pad_{idx}",
        )
    tray.visual(
        Box((0.014, 0.235, 0.008)),
        origin=Origin(xyz=(TRAY_X0 + TRAY_L - 0.026, 0.0, 0.045)),
        material="rubber",
        name="front_bumper",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z), rpy=(0.0, 0.0, ARM_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=LOWER_ELBOW),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.9, lower=-0.40, upper=0.75),
    )
    model.articulation(
        "upper_to_tray",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tray,
        origin=Origin(xyz=UPPER_HEAD),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8, lower=-0.35, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    tray = object_model.get_part("tray")
    lower_joint = object_model.get_articulation("base_to_lower")
    elbow_joint = object_model.get_articulation("lower_to_upper")
    tray_joint = object_model.get_articulation("upper_to_tray")

    ctx.allow_overlap(
        lower,
        upper,
        elem_a="elbow_pin",
        elem_b="elbow_lug",
        reason="The elbow pin is intentionally captured inside the upper lug bore proxy.",
    )
    for elem in ("tray_barrel_0", "tray_barrel_1", "hinge_tab_0", "hinge_tab_1"):
        ctx.allow_overlap(
            upper,
            tray,
            elem_a="tray_pin",
            elem_b=elem,
            reason="The tray hinge pin intentionally passes through the tray hinge knuckle or leaf proxy.",
        )

    ctx.expect_contact(
        base,
        lower,
        elem_a="pivot_socket",
        elem_b="turntable",
        contact_tol=0.001,
        name="turntable sits on base pivot socket",
    )
    ctx.expect_overlap(
        lower,
        upper,
        elem_a="elbow_pin",
        elem_b="elbow_lug",
        axes="xyz",
        min_overlap=0.010,
        name="elbow pin is captured in lug",
    )
    ctx.expect_overlap(
        upper,
        tray,
        elem_a="tray_pin",
        elem_b="tray_barrel_0",
        axes="xyz",
        min_overlap=0.006,
        name="tray hinge barrel captures pin",
    )
    ctx.expect_within(
        tray,
        base,
        axes="xy",
        margin=0.24,
        name="tray is carried over base footprint",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.26,
        name="tray clears heavy desk base",
    )

    ctx.check("lower pivot axis is vertical", tuple(lower_joint.axis) == (0.0, 0.0, 1.0))
    ctx.check("elbow axis is horizontal", abs(elbow_joint.axis[2]) < 1e-9)
    ctx.check("tray hinge axis is horizontal", abs(tray_joint.axis[2]) < 1e-9)
    tray_joint_count = sum(
        1
        for joint in object_model.articulations
        if getattr(joint.parent, "name", joint.parent) == tray.name
        or getattr(joint.child, "name", joint.child) == tray.name
    )
    ctx.check(
        "tray is a single rigid link",
        tray_joint_count == 1,
        details=f"tray_joint_count={tray_joint_count}",
    )

    rest_tray_aabb = ctx.part_world_aabb(tray)
    rest_tray_z = None if rest_tray_aabb is None else (rest_tray_aabb[0][2] + rest_tray_aabb[1][2]) / 2.0
    with ctx.pose({elbow_joint: elbow_joint.motion_limits.upper}):
        raised_aabb = ctx.part_world_aabb(tray)
        raised_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    ctx.check(
        "elbow raises arm head",
        rest_tray_z is not None and raised_z is not None and raised_z > rest_tray_z + 0.045,
        details=f"rest_z={rest_tray_z}, raised_z={raised_z}",
    )

    rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    rest_front_z = None if rest_front is None else (rest_front[0][2] + rest_front[1][2]) / 2.0
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        tilted_front_z = (
            None
            if tilted_front is None
            else (tilted_front[0][2] + tilted_front[1][2]) / 2.0
        )
    ctx.check(
        "tray hinge tilts front lip upward",
        rest_front_z is not None
        and tilted_front_z is not None
        and tilted_front_z > rest_front_z + 0.10,
        details=f"rest_front_z={rest_front_z}, tilted_front_z={tilted_front_z}",
    )

    return ctx.report()


object_model = build_object_model()

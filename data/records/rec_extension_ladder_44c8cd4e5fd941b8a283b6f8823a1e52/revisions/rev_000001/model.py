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
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_channel_rail(
    part,
    *,
    x_sign: float,
    rail_center_x: float,
    rail_bottom_z: float,
    rail_length: float,
    web_size_x: float,
    web_size_y: float,
    flange_size_x: float,
    flange_size_y: float,
    lip_size_x: float,
    lip_size_y: float,
    flange_center_x: float,
    lip_center_x: float,
    material,
    prefix: str,
) -> None:
    z_center = rail_bottom_z + rail_length * 0.5
    flange_center_y = web_size_y * 0.5 + flange_size_y * 0.5 - 0.001
    lip_center_y = flange_center_y + flange_size_y * 0.5 + lip_size_y * 0.5 - 0.001
    part.visual(
        Box((web_size_x, web_size_y, rail_length)),
        origin=Origin(xyz=(x_sign * rail_center_x, 0.0, z_center)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((flange_size_x, flange_size_y, rail_length)),
        origin=Origin(xyz=(x_sign * flange_center_x, flange_center_y, z_center)),
        material=material,
        name=f"{prefix}_flange",
    )
    part.visual(
        Box((lip_size_x, lip_size_y, rail_length)),
        origin=Origin(xyz=(x_sign * lip_center_x, lip_center_y, z_center)),
        material=material,
        name=f"{prefix}_guide_lip",
    )


def _add_fly_rail(
    part,
    *,
    x_sign: float,
    rail_center_x: float,
    rail_bottom_z: float,
    rail_length: float,
    material,
    prefix: str,
) -> None:
    z_center = rail_bottom_z + rail_length * 0.5
    part.visual(
        Box((0.038, 0.018, rail_length)),
        origin=Origin(xyz=(x_sign * rail_center_x, 0.0, z_center)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((0.056, 0.006, rail_length)),
        origin=Origin(xyz=(x_sign * 0.177, 0.012, z_center)),
        material=material,
        name=f"{prefix}_flange",
    )
    part.visual(
        Box((0.010, 0.004, rail_length)),
        origin=Origin(xyz=(x_sign * 0.145, 0.015, z_center)),
        material=material,
        name=f"{prefix}_guide_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="firefighter_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.50, 0.54, 1.0))
    rubber = model.material("rubber", rgba=(0.15, 0.15, 0.15, 1.0))
    safety_red = model.material("safety_red", rgba=(0.72, 0.10, 0.08, 1.0))

    outer_width = 0.50
    base_length = 4.20
    fly_length = 3.85
    base_rail_x = 0.223
    fly_rail_x = 0.186
    base_rung_length = 0.430
    fly_rung_length = 0.300
    pawl_pivot_x = 0.248
    pawl_mount_y = -0.028
    rung_pitch = 0.30
    fly_joint_z = 0.76
    fly_front_offset = 0.0
    fly_travel = 2.00

    base = model.part("base_section")
    for x_sign, prefix in ((-1.0, "left_base_rail"), (1.0, "right_base_rail")):
        _add_channel_rail(
            base,
            x_sign=x_sign,
            rail_center_x=base_rail_x,
            rail_bottom_z=0.0,
            rail_length=base_length,
            web_size_x=0.028,
            web_size_y=0.034,
            flange_size_x=0.082,
            flange_size_y=0.012,
            lip_size_x=0.010,
            lip_size_y=0.022,
            flange_center_x=0.196,
            lip_center_x=0.160,
            material=aluminum,
            prefix=prefix,
        )

    base_rung_zs = [0.25 + rung_pitch * index for index in range(13)]
    for index, rung_z in enumerate(base_rung_zs):
        if index == 2:
            base.visual(
                Cylinder(radius=0.016, length=base_rung_length),
                origin=Origin(
                    xyz=(0.0, -0.024, rung_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=steel,
                name="lock_rung",
            )
        else:
            base.visual(
                Cylinder(radius=0.016, length=base_rung_length),
                origin=Origin(
                    xyz=(0.0, -0.024, rung_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=steel,
                name=f"base_rung_{index}",
            )

    for x_sign, prefix in ((-1.0, "left_foot"), (1.0, "right_foot")):
        base.visual(
            Box((0.060, 0.028, 0.100)),
            origin=Origin(xyz=(x_sign * base_rail_x, -0.005, 0.050)),
            material=rubber,
            name=prefix,
        )

    base.inertial = Inertial.from_geometry(
        Box((outer_width, 0.11, base_length)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.01, base_length * 0.5)),
    )

    fly = model.part("fly_section")
    for x_sign, prefix in ((-1.0, "left_fly_rail"), (1.0, "right_fly_rail")):
        _add_fly_rail(
            fly,
            x_sign=x_sign,
            rail_center_x=fly_rail_x,
            rail_bottom_z=0.0,
            rail_length=fly_length,
            material=aluminum,
            prefix=prefix,
        )
        fly.visual(
            Box((0.020, 0.014, 0.110)),
            origin=Origin(xyz=(x_sign * 0.203, pawl_mount_y, 0.186)),
            material=steel,
            name=f"{prefix}_pawl_hanger_plate",
        )
        fly.visual(
            Box((0.014, 0.014, 0.072)),
            origin=Origin(xyz=(x_sign * 0.263, pawl_mount_y, 0.205)),
            material=steel,
            name=f"{prefix}_pawl_outer_cheek",
        )
        fly.visual(
            Box((0.084, 0.014, 0.018)),
            origin=Origin(xyz=(x_sign * 0.235, pawl_mount_y, 0.203)),
            material=steel,
            name=f"{prefix}_pawl_top_bridge",
        )
        fly.visual(
            Box((0.026, 0.018, 0.050)),
            origin=Origin(xyz=(x_sign * 0.196, -0.018, 0.216)),
            material=steel,
            name=f"{prefix}_pawl_hanger_doubler",
        )
        fly.visual(
            Box((0.022, 0.010, 0.090)),
            origin=Origin(xyz=(x_sign * fly_rail_x, 0.004, 0.145)),
            material=steel,
            name=f"{prefix}_guide_shoe",
        )

    fly_rung_zs = [0.22 + rung_pitch * index for index in range(12)]
    for index, rung_z in enumerate(fly_rung_zs):
        fly.visual(
            Cylinder(radius=0.014, length=fly_rung_length),
            origin=Origin(
                xyz=(0.0, 0.003, rung_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"fly_rung_{index}",
        )

    for x_sign, prefix in ((-1.0, "left_tip_cap"), (1.0, "right_tip_cap")):
        fly.visual(
            Box((0.042, 0.026, 0.080)),
            origin=Origin(xyz=(x_sign * fly_rail_x, -0.002, fly_length - 0.040)),
            material=rubber,
            name=prefix,
        )

    fly.inertial = Inertial.from_geometry(
        Box((0.42, 0.08, fly_length)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.01, fly_length * 0.5)),
    )

    fly_joint = model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, fly_front_offset, fly_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.60,
            lower=0.0,
            upper=fly_travel,
        ),
    )

    def add_pawl(name: str, x_pos: float) -> None:
        x_sign = 1.0 if x_pos >= 0.0 else -1.0
        pawl = model.part(name)
        pawl.visual(
            Cylinder(radius=0.008, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pawl_barrel",
        )
        pawl.visual(
            Box((0.022, 0.012, 0.064)),
            origin=Origin(xyz=(-x_sign * 0.014, 0.0, -0.034)),
            material=safety_red,
            name="pawl_arm",
        )
        pawl.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(
                xyz=(-x_sign * 0.025, 0.0, -0.072), rpy=(0.0, math.pi / 2.0, 0.0)
            ),
            material=steel,
            name="pawl_hook",
        )
        pawl.inertial = Inertial.from_geometry(
            Box((0.040, 0.024, 0.090)),
            mass=0.30,
            origin=Origin(xyz=(-x_sign * 0.013, 0.0, -0.036)),
        )
        model.articulation(
            f"fly_to_{name}",
            ArticulationType.REVOLUTE,
            parent=fly,
            child=pawl,
            origin=Origin(xyz=(x_pos, pawl_mount_y, 0.186)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=2.0,
                lower=0.0,
                upper=1.10,
            ),
        )

    add_pawl("left_pawl", -pawl_pivot_x)
    add_pawl("right_pawl", pawl_pivot_x)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    left_pawl = object_model.get_part("left_pawl")
    right_pawl = object_model.get_part("right_pawl")
    fly_joint = object_model.get_articulation("base_to_fly")
    left_pawl_joint = object_model.get_articulation("fly_to_left_pawl")
    right_pawl_joint = object_model.get_articulation("fly_to_right_pawl")

    ctx.check("base section present", base is not None)
    ctx.check("fly section present", fly is not None)
    ctx.check("left pawl present", left_pawl is not None)
    ctx.check("right pawl present", right_pawl is not None)

    ctx.check(
        "fly joint extends upward",
        tuple(fly_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={fly_joint.axis}",
    )
    ctx.check(
        "pawl joints swing about ladder width axis",
        tuple(left_pawl_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(right_pawl_joint.axis) == (1.0, 0.0, 0.0),
        details=f"left_axis={left_pawl_joint.axis}, right_axis={right_pawl_joint.axis}",
    )

    with ctx.pose({fly_joint: 0.0, left_pawl_joint: 0.0, right_pawl_joint: 0.0}):
        ctx.expect_contact(
            left_pawl,
            base,
            elem_a="pawl_hook",
            elem_b="lock_rung",
            contact_tol=0.003,
            name="left pawl seats on lock rung",
        )
        ctx.expect_contact(
            right_pawl,
            base,
            elem_a="pawl_hook",
            elem_b="lock_rung",
            contact_tol=0.003,
            name="right pawl seats on lock rung",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=3.0,
            name="collapsed fly remains deeply nested in base",
        )

    fly_rest_pos = ctx.part_world_position(fly)
    with ctx.pose({fly_joint: 2.00}):
        fly_extended_pos = ctx.part_world_position(fly)
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.35,
            name="extended fly retains guide engagement in base",
        )

    ctx.check(
        "fly section raises when extended",
        fly_rest_pos is not None
        and fly_extended_pos is not None
        and fly_extended_pos[2] > fly_rest_pos[2] + 1.9,
        details=f"rest={fly_rest_pos}, extended={fly_extended_pos}",
    )

    with ctx.pose({left_pawl_joint: 1.00, right_pawl_joint: 1.00}):
        ctx.expect_gap(
            left_pawl,
            base,
            axis="y",
            positive_elem="pawl_hook",
            negative_elem="lock_rung",
            min_gap=0.010,
            name="left pawl clears forward when released",
        )
        ctx.expect_gap(
            right_pawl,
            base,
            axis="y",
            positive_elem="pawl_hook",
            negative_elem="lock_rung",
            min_gap=0.010,
            name="right pawl clears forward when released",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

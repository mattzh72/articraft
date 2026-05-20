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
)


BASE_BOTTOM_Z = 0.08
BASE_TOP_Z = 3.35
BASE_BOTTOM_HALF_WIDTH = 0.31
BASE_TOP_HALF_WIDTH = 0.22
BASE_RAIL_WIDTH = 0.036
BASE_RAIL_DEPTH = 0.088

FLY_JOINT_Z = 1.05
FLY_JOINT_Y = 0.10
FLY_RAIL_HALF_WIDTH = 0.16
FLY_RAIL_WIDTH = 0.028
FLY_RAIL_DEPTH = 0.072
FLY_LOWER_Z = -0.58
FLY_UPPER_Z = 2.52
FLY_TRAVEL = 1.40

TRAY_HINGE_Z = 2.28
TRAY_HINGE_Y = 0.078
TRAY_WIDTH = 0.34
TRAY_DEPTH = 0.24
TRAY_THICKNESS = 0.016
TRAY_WALL_HEIGHT = 0.050


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _member_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_member_rpy(a, b)),
        material=material,
        name=name,
    )


def _add_cylinder_member(
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
        origin=Origin(xyz=_midpoint(a, b), rpy=_member_rpy(a, b)),
        material=material,
        name=name,
    )


def _base_half_width_at(z: float) -> float:
    t = (z - BASE_BOTTOM_Z) / (BASE_TOP_Z - BASE_BOTTOM_Z)
    return BASE_BOTTOM_HALF_WIDTH + (BASE_TOP_HALF_WIDTH - BASE_BOTTOM_HALF_WIDTH) * t


def _build_base_section(model: ArticulatedObject, rail_material, tread_material, rubber_material):
    base = model.part("base_section")

    left_bottom = (-BASE_BOTTOM_HALF_WIDTH, 0.0, BASE_BOTTOM_Z)
    left_top = (-BASE_TOP_HALF_WIDTH, 0.0, BASE_TOP_Z)
    right_bottom = (BASE_BOTTOM_HALF_WIDTH, 0.0, BASE_BOTTOM_Z)
    right_top = (BASE_TOP_HALF_WIDTH, 0.0, BASE_TOP_Z)

    _add_box_member(
        base,
        left_bottom,
        left_top,
        width=BASE_RAIL_WIDTH,
        depth=BASE_RAIL_DEPTH,
        material=rail_material,
        name="base_rail_0",
    )
    _add_box_member(
        base,
        right_bottom,
        right_top,
        width=BASE_RAIL_WIDTH,
        depth=BASE_RAIL_DEPTH,
        material=rail_material,
        name="base_rail_1",
    )

    rung_zs = [0.40 + 0.285 * i for i in range(10)]
    for index, z in enumerate(rung_zs):
        half_width = _base_half_width_at(z) - 0.010
        _add_box_member(
            base,
            (-half_width, 0.0, z),
            (half_width, 0.0, z),
            width=0.032,
            depth=0.030,
            material=tread_material,
            name=f"base_rung_{index}",
        )

    for sign, rail_name in ((-1.0, "left"), (1.0, "right")):
        for index, z in enumerate((0.95, 1.55, 2.15, 2.75)):
            rail_center_x = sign * _base_half_width_at(z)
            fly_outer_face_x = FLY_RAIL_HALF_WIDTH + (FLY_RAIL_WIDTH * 0.5)
            guide_outer_face = abs(rail_center_x) - (BASE_RAIL_WIDTH * 0.5) + 0.004
            guide_width = guide_outer_face - fly_outer_face_x
            guide_center_x = sign * (fly_outer_face_x + guide_width * 0.5)
            base.visual(
                Box((guide_width, 0.023, 0.120)),
                origin=Origin(xyz=(guide_center_x, 0.0525, z)),
                material=rail_material,
                name=f"{rail_name}_guide_{index}",
            )

    base.visual(
        Box((0.410, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - 0.02)),
        material=tread_material,
        name="top_cap",
    )

    for sign, name in ((-1.0, "foot_0"), (1.0, "foot_1")):
        base.visual(
            Box((0.110, 0.095, 0.090)),
            origin=Origin(xyz=(sign * 0.305, 0.0, 0.045)),
            material=rubber_material,
            name=name,
        )

    return base


def _build_fly_section(model: ArticulatedObject, rail_material, tread_material, tray_material):
    fly = model.part("fly_section")

    _add_box_member(
        fly,
        (-FLY_RAIL_HALF_WIDTH, 0.0, FLY_LOWER_Z),
        (-FLY_RAIL_HALF_WIDTH, 0.0, FLY_UPPER_Z),
        width=FLY_RAIL_WIDTH,
        depth=FLY_RAIL_DEPTH,
        material=rail_material,
        name="fly_rail_0",
    )
    _add_box_member(
        fly,
        (FLY_RAIL_HALF_WIDTH, 0.0, FLY_LOWER_Z),
        (FLY_RAIL_HALF_WIDTH, 0.0, FLY_UPPER_Z),
        width=FLY_RAIL_WIDTH,
        depth=FLY_RAIL_DEPTH,
        material=rail_material,
        name="fly_rail_1",
    )

    rung_zs = [-0.26 + 0.280 * i for i in range(10)]
    for index, z in enumerate(rung_zs):
        _add_box_member(
            fly,
            (-(FLY_RAIL_HALF_WIDTH - 0.008), 0.0, z),
            (FLY_RAIL_HALF_WIDTH - 0.008, 0.0, z),
            width=0.030,
            depth=0.026,
            material=tread_material,
            name=f"fly_rung_{index}",
        )

    fly.visual(
        Box((0.300, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.038, TRAY_HINGE_Z - 0.05)),
        material=rail_material,
        name="tray_mount",
    )
    for sign, index in ((-1.0, 0), (1.0, 1)):
        fly.visual(
            Box((0.024, 0.040, 0.052)),
            origin=Origin(
                xyz=(sign * (FLY_RAIL_HALF_WIDTH - 0.004), 0.050, TRAY_HINGE_Z)
            ),
            material=tray_material,
            name=f"tray_bracket_{index}",
        )

    fly.visual(
        Box((0.304, 0.022, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, FLY_UPPER_Z - 0.012)),
        material=tread_material,
        name="fly_top_cap",
    )

    return fly


def _build_tray(model: ArticulatedObject, tray_material, hinge_material):
    tray = model.part("work_tray")

    tray.visual(
        Box((TRAY_WIDTH, TRAY_THICKNESS, TRAY_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, -TRAY_DEPTH * 0.5)),
        material=tray_material,
        name="tray_deck",
    )
    tray.visual(
        Box((0.016, TRAY_WALL_HEIGHT, TRAY_DEPTH - 0.012)),
        origin=Origin(
            xyz=(-TRAY_WIDTH * 0.5 + 0.008, TRAY_WALL_HEIGHT * 0.5 - 0.004, -TRAY_DEPTH * 0.5)
        ),
        material=tray_material,
        name="side_wall_0",
    )
    tray.visual(
        Box((0.016, TRAY_WALL_HEIGHT, TRAY_DEPTH - 0.012)),
        origin=Origin(
            xyz=(TRAY_WIDTH * 0.5 - 0.008, TRAY_WALL_HEIGHT * 0.5 - 0.004, -TRAY_DEPTH * 0.5)
        ),
        material=tray_material,
        name="side_wall_1",
    )
    tray.visual(
        Box((TRAY_WIDTH - 0.032, TRAY_WALL_HEIGHT, 0.016)),
        origin=Origin(
            xyz=(0.0, TRAY_WALL_HEIGHT * 0.5 - 0.004, -0.008),
        ),
        material=tray_material,
        name="rear_wall",
    )
    tray.visual(
        Box((TRAY_WIDTH - 0.040, TRAY_WALL_HEIGHT, 0.016)),
        origin=Origin(
            xyz=(0.0, TRAY_WALL_HEIGHT * 0.5 - 0.004, -TRAY_DEPTH + 0.008),
        ),
        material=tray_material,
        name="front_lip",
    )
    tray.visual(
        Box((0.012, TRAY_WALL_HEIGHT, TRAY_DEPTH - 0.052)),
        origin=Origin(
            xyz=(0.0, TRAY_WALL_HEIGHT * 0.5 - 0.004, -TRAY_DEPTH * 0.5 - 0.014),
        ),
        material=tray_material,
        name="tray_divider",
    )

    for sign, index in ((-1.0, 0), (1.0, 1)):
        tray.visual(
            Cylinder(radius=0.008, length=0.040),
            origin=Origin(
                xyz=(sign * (TRAY_WIDTH * 0.5 - 0.028), 0.0, -0.004),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_material,
            name=f"hinge_barrel_{index}",
        )

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder_with_work_tray")

    rail_material = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    tread_material = model.material("tread_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber_material = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_material = model.material("tray_plastic", rgba=(0.92, 0.56, 0.12, 1.0))
    hinge_material = model.material("hinge_steel", rgba=(0.43, 0.45, 0.48, 1.0))

    base = _build_base_section(model, rail_material, tread_material, rubber_material)
    fly = _build_fly_section(model, rail_material, tread_material, tray_material)
    tray = _build_tray(model, tray_material, hinge_material)

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, FLY_JOINT_Y, FLY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLY_TRAVEL,
            effort=350.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_HINGE_Y, TRAY_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.62,
            effort=18.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    tray = object_model.get_part("work_tray")
    fly_slide = object_model.get_articulation("fly_slide")
    tray_hinge = object_model.get_articulation("tray_hinge")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        margin=0.055,
        name="left fly rail stays under the base side guides",
    )
    ctx.expect_within(
        fly,
        base,
        axes="x",
        margin=0.055,
        name="right fly rail stays under the base side guides",
    )

    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.70,
        name="collapsed fly remains deeply nested in the base section",
    )

    collapsed_origin = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: FLY_TRAVEL}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.45,
            name="extended fly still retains substantial insertion in the base",
        )
        extended_origin = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        collapsed_origin is not None
        and extended_origin is not None
        and extended_origin[2] > collapsed_origin[2] + 1.35,
        details=f"collapsed_origin={collapsed_origin}, extended_origin={extended_origin}",
    )

    closed_deck = ctx.part_element_world_aabb(tray, elem="tray_deck")
    with ctx.pose({tray_hinge: 1.50}):
        open_deck = ctx.part_element_world_aabb(tray, elem="tray_deck")

    ctx.check(
        "tray swings outward into a working shelf position",
        closed_deck is not None
        and open_deck is not None
        and open_deck[1][1] > closed_deck[1][1] + 0.20
        and open_deck[0][2] > closed_deck[0][2] + 0.10,
        details=f"closed_deck={closed_deck}, open_deck={open_deck}",
    )

    return ctx.report()


object_model = build_object_model()

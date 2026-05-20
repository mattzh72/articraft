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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_points(radius: float, *, segments: int) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            0.0,
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _handwheel_rim_mesh(name: str, *, radius: float, tube_radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            _circle_points(radius, segments=18),
            radius=tube_radius,
            samples_per_segment=5,
            closed_spline=True,
            radial_segments=18,
            cap_ends=False,
            up_hint=(0.0, 1.0, 0.0),
        ),
        name,
    )


def _add_guide_rail(part, *, side_sign: float, material) -> None:
    rail_height = 4.38

    part.visual(
        Box((0.09, 0.04, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_height * 0.5)),
        material=material,
        name="rail_web",
    )
    part.visual(
        Box((0.09, 0.02, rail_height - 0.14)),
        origin=Origin(
            xyz=(-side_sign * 0.06, 0.085, (rail_height - 0.14) * 0.5 + 0.07),
        ),
        material=material,
        name="guide_face",
    )
    part.visual(
        Box((0.03, 0.07, rail_height - 0.18)),
        origin=Origin(
            xyz=(-side_sign * 0.03, 0.045, (rail_height - 0.18) * 0.5 + 0.09),
        ),
        material=material,
        name="guide_flange",
    )
    for index, z in enumerate((0.48, 2.12, 3.76)):
        part.visual(
            Box((0.12, 0.06, 0.28)),
            origin=Origin(xyz=(0.0, -0.05, z)),
            material=material,
            name=f"mount_pad_{index}",
        )
    part.visual(
        Box((0.14, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.015, rail_height - 0.06)),
        material=material,
        name="top_head",
    )


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    coated_plate = model.material("coated_plate", rgba=(0.22, 0.30, 0.39, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.80, 0.66, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.60, 0.36, 5.00)),
        origin=Origin(xyz=(-1.00, 0.0, 2.50)),
        material=concrete,
        name="pier_0",
    )
    frame.visual(
        Box((0.60, 0.36, 5.00)),
        origin=Origin(xyz=(1.00, 0.0, 2.50)),
        material=concrete,
        name="pier_1",
    )
    frame.visual(
        Box((1.40, 0.36, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=concrete,
        name="sill",
    )
    frame.visual(
        Box((2.60, 0.30, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 4.86)),
        material=concrete,
        name="top_cap",
    )
    frame.visual(
        Box((0.06, 0.08, 2.00)),
        origin=Origin(xyz=(-0.73, 0.14, 1.30)),
        material=steel,
        name="seat_0",
    )
    frame.visual(
        Box((0.06, 0.08, 2.00)),
        origin=Origin(xyz=(0.73, 0.14, 1.30)),
        material=steel,
        name="seat_1",
    )
    frame.visual(
        Box((1.52, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.14, 2.08)),
        material=steel,
        name="head_seat",
    )

    guide_0 = model.part("guide_0")
    _add_guide_rail(guide_0, side_sign=-1.0, material=steel)

    guide_1 = model.part("guide_1")
    _add_guide_rail(guide_1, side_sign=1.0, material=steel)

    lift_plate = model.part("lift_plate")
    lift_plate.visual(
        Box((1.36, 0.04, 2.25)),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=coated_plate,
        name="leaf",
    )
    lift_plate.visual(
        Box((1.34, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="bottom_edge",
    )
    for index, x in enumerate((-0.655, 0.655)):
        lift_plate.visual(
            Box((0.07, 0.06, 2.15)),
            origin=Origin(xyz=(x, 0.0, 1.075)),
            material=dark_steel,
            name=f"edge_bar_{index}",
        )
    for index, z in enumerate((0.58, 1.18, 1.78)):
        lift_plate.visual(
            Box((1.18, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, 0.03, z)),
            material=dark_steel,
            name=f"stiffener_{index}",
        )
    lift_plate.visual(
        Box((0.20, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.33)),
        material=dark_steel,
        name="yoke",
    )
    lift_plate.visual(
        Box((0.34, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.015, 2.45)),
        material=dark_steel,
        name="yoke_cap",
    )

    operator_housing = model.part("operator_housing")
    operator_housing.visual(
        Box((1.85, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=steel,
        name="crossbeam",
    )
    operator_housing.visual(
        Box((0.48, 0.24, 0.28)),
        origin=Origin(xyz=(0.0, 0.06, 0.34)),
        material=dark_steel,
        name="gearbox",
    )
    operator_housing.visual(
        Box((0.22, 0.14, 0.70)),
        origin=Origin(xyz=(0.0, -0.06, -0.25)),
        material=dark_steel,
        name="bonnet",
    )
    operator_housing.visual(
        Box((0.24, 0.12, 0.16)),
        origin=Origin(xyz=(0.46, 0.00, 0.28)),
        material=dark_steel,
        name="wheel_support",
    )
    operator_housing.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.56, 0.04, 0.28), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="shaft_boss",
    )
    operator_housing.visual(
        Box((0.08, 0.10, 0.22)),
        origin=Origin(xyz=(0.82, 0.15, 0.42)),
        material=dark_steel,
        name="pawl_bracket",
    )
    operator_housing.visual(
        Box((0.34, 0.16, 0.08)),
        origin=Origin(xyz=(0.66, 0.08, 0.37)),
        material=dark_steel,
        name="pawl_bridge",
    )
    operator_housing.visual(
        Box((0.12, 0.10, 0.16)),
        origin=Origin(xyz=(-0.46, 0.03, 0.28)),
        material=dark_steel,
        name="service_box",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        _handwheel_rim_mesh("sluice_handwheel_rim", radius=0.28, tube_radius=0.018),
        material=warning_yellow,
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.065, length=0.06),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    handwheel.visual(
        Cylinder(radius=0.025, length=0.20),
        origin=Origin(xyz=(0.0, -0.10, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    for index, angle in enumerate((0.0, math.pi / 3.0, -math.pi / 3.0)):
        handwheel.visual(
            Box((0.54, 0.016, 0.028)),
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=steel,
            name=f"spoke_{index}",
        )
    handwheel.visual(
        Box((0.035, 0.03, 0.035)),
        origin=Origin(xyz=(0.202, 0.005, 0.202)),
        material=dark_steel,
        name="grip_stem",
    )
    handwheel.visual(
        Cylinder(radius=0.016, length=0.09),
        origin=Origin(
            xyz=(0.19, 0.04, 0.19),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="grip",
    )

    locking_pawl = model.part("locking_pawl")
    locking_pawl.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    locking_pawl.visual(
        Box((0.04, 0.022, 0.06)),
        origin=Origin(xyz=(-0.01, 0.0, -0.02), rpy=(0.0, -0.55, 0.0)),
        material=dark_steel,
        name="pawl_root",
    )
    locking_pawl.visual(
        Box((0.024, 0.022, 0.18)),
        origin=Origin(xyz=(-0.018, 0.0, -0.075), rpy=(0.0, -0.18, 0.0)),
        material=dark_steel,
        name="pawl_arm",
    )
    locking_pawl.visual(
        Box((0.032, 0.03, 0.036)),
        origin=Origin(xyz=(-0.035, 0.0, -0.154), rpy=(0.0, -0.18, 0.0)),
        material=dark_steel,
        name="pawl_tooth",
    )
    locking_pawl.visual(
        Box((0.022, 0.022, 0.05)),
        origin=Origin(xyz=(-0.026, 0.0, -0.122), rpy=(0.0, -0.18, 0.0)),
        material=dark_steel,
        name="tooth_neck",
    )
    locking_pawl.visual(
        Box((0.018, 0.018, 0.08)),
        origin=Origin(xyz=(0.028, 0.0, 0.042), rpy=(0.0, 0.45, 0.0)),
        material=dark_steel,
        name="pawl_handle",
    )

    model.articulation(
        "guide_0_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=guide_0,
        origin=Origin(xyz=(-0.795, 0.26, 0.18)),
    )
    model.articulation(
        "guide_1_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=guide_1,
        origin=Origin(xyz=(0.795, 0.26, 0.18)),
    )
    model.articulation(
        "plate_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_plate,
        origin=Origin(xyz=(0.0, 0.31, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.18,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=operator_housing,
        origin=Origin(xyz=(0.0, 0.27, 4.58)),
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=operator_housing,
        child=handwheel,
        origin=Origin(xyz=(0.56, 0.28, 0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=8.0),
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=operator_housing,
        child=locking_pawl,
        origin=Origin(xyz=(0.82, 0.22, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    guide_0 = object_model.get_part("guide_0")
    guide_1 = object_model.get_part("guide_1")
    lift_plate = object_model.get_part("lift_plate")
    handwheel = object_model.get_part("handwheel")
    locking_pawl = object_model.get_part("locking_pawl")

    plate_lift = object_model.get_articulation("plate_lift")
    handwheel_spin = object_model.get_articulation("handwheel_spin")
    pawl_pivot = object_model.get_articulation("pawl_pivot")

    lift_limits = plate_lift.motion_limits
    pawl_limits = pawl_pivot.motion_limits

    with ctx.pose({plate_lift: 0.0}):
        ctx.expect_contact(
            lift_plate,
            guide_0,
            name="lift plate bears in guide_0 at rest",
        )
        ctx.expect_contact(
            guide_1,
            lift_plate,
            name="lift plate bears in guide_1 at rest",
        )
        ctx.expect_gap(
            lift_plate,
            frame,
            axis="y",
            min_gap=0.075,
            max_gap=0.110,
            name="lift plate stands proud of the wall frame",
        )

    if lift_limits is not None and lift_limits.upper is not None:
        rest_pos = ctx.part_world_position(lift_plate)
        with ctx.pose({plate_lift: lift_limits.upper}):
            ctx.expect_contact(
                lift_plate,
                guide_0,
                name="lift plate bears in guide_0 when raised",
            )
            ctx.expect_contact(
                guide_1,
                lift_plate,
                name="lift plate bears in guide_1 when raised",
            )
            ctx.expect_overlap(
                lift_plate,
                guide_0,
                axes="z",
                min_overlap=2.20,
                name="lift plate remains captured by guide_0",
            )
            ctx.expect_overlap(
                lift_plate,
                guide_1,
                axes="z",
                min_overlap=2.20,
                name="lift plate remains captured by guide_1",
            )
            raised_pos = ctx.part_world_position(lift_plate)
        ctx.check(
            "lift plate moves upward",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[2] > rest_pos[2] + 1.80,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    spoke_rest = ctx.part_element_world_aabb(handwheel, elem="spoke_0")
    with ctx.pose({handwheel_spin: math.pi * 0.5}):
        spoke_turn = ctx.part_element_world_aabb(handwheel, elem="spoke_0")
    spoke_rest_size = _aabb_size(spoke_rest)
    spoke_turn_size = _aabb_size(spoke_turn)
    ctx.check(
        "handwheel rotates about its shaft",
        spoke_rest_size is not None
        and spoke_turn_size is not None
        and spoke_rest_size[0] > spoke_rest_size[2]
        and spoke_turn_size[2] > spoke_turn_size[0],
        details=f"rest_size={spoke_rest_size}, turn_size={spoke_turn_size}",
    )

    if pawl_limits is not None and pawl_limits.upper is not None:
        tooth_rest = ctx.part_element_world_aabb(locking_pawl, elem="pawl_tooth")
        with ctx.pose({pawl_pivot: pawl_limits.upper}):
            tooth_released = ctx.part_element_world_aabb(locking_pawl, elem="pawl_tooth")
        tooth_rest_center = _aabb_center(tooth_rest)
        tooth_released_center = _aabb_center(tooth_released)
        ctx.check(
            "locking pawl swings on its pivot",
            tooth_rest_center is not None
            and tooth_released_center is not None
            and tooth_released_center[2] > tooth_rest_center[2] + 0.03,
            details=f"rest_center={tooth_rest_center}, released_center={tooth_released_center}",
        )

    return ctx.report()


object_model = build_object_model()

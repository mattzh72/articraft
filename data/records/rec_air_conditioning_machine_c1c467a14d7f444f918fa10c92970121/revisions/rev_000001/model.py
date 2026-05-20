from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.49
BODY_DEPTH = 0.29
BODY_HEIGHT = 0.32
FASCIA_DEPTH = 0.04
FRONT_Y = -BODY_DEPTH / 2.0

OUTLET_WIDTH = 0.23
OUTLET_HEIGHT = 0.10
OUTLET_BOTTOM = 0.11
OUTLET_TOP = OUTLET_BOTTOM + OUTLET_HEIGHT
OUTLET_CENTER_Z = (OUTLET_BOTTOM + OUTLET_TOP) * 0.5

CAVITY_WALL = 0.01
CAVITY_LIP = 0.01
OUTLET_INNER_WIDTH = OUTLET_WIDTH - 2.0 * CAVITY_WALL
OUTLET_BACKDROP_HEIGHT = OUTLET_HEIGHT - 0.014

KNOB_CENTER_Z = 0.258
KNOB_X = 0.115


def _build_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.033,
            0.020,
            body_style="skirted",
            top_diameter=0.028,
            skirt=KnobSkirt(0.038, 0.004, flare=0.05),
            grip=KnobGrip(style="fluted", count=14, depth=0.0010),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0007,
                angle_deg=0.0,
            ),
            center=False,
        ),
        "window_ac_control_knob",
    )


def _aabb_center(aabb, axis: str) -> float | None:
    if aabb is None:
        return None
    idx = "xyz".index(axis)
    mins, maxs = aabb
    return 0.5 * (float(mins[idx]) + float(maxs[idx]))


def _aabb_max(aabb, axis: str) -> float | None:
    if aabb is None:
        return None
    idx = "xyz".index(axis)
    _, maxs = aabb
    return float(maxs[idx])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_air_conditioner")

    case_white = model.material("case_white", rgba=(0.92, 0.93, 0.91, 1.0))
    trim_white = model.material("trim_white", rgba=(0.96, 0.97, 0.95, 1.0))
    control_grey = model.material("control_grey", rgba=(0.82, 0.84, 0.82, 1.0))
    vent_shadow = model.material("vent_shadow", rgba=(0.18, 0.20, 0.21, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    louver_white = model.material("louver_white", rgba=(0.89, 0.90, 0.87, 1.0))
    vane_grey = model.material("vane_grey", rgba=(0.76, 0.78, 0.78, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH - FASCIA_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, FASCIA_DEPTH * 0.5, BODY_HEIGHT * 0.5)),
        material=case_white,
        name="rear_body",
    )
    cabinet.visual(
        Box((BODY_WIDTH, FASCIA_DEPTH, OUTLET_BOTTOM)),
        origin=Origin(xyz=(0.0, FRONT_Y + FASCIA_DEPTH * 0.5, OUTLET_BOTTOM * 0.5)),
        material=trim_white,
        name="fascia_bottom",
    )
    cabinet.visual(
        Box((BODY_WIDTH, FASCIA_DEPTH, BODY_HEIGHT - OUTLET_TOP)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y + FASCIA_DEPTH * 0.5,
                OUTLET_TOP + (BODY_HEIGHT - OUTLET_TOP) * 0.5,
            )
        ),
        material=trim_white,
        name="fascia_top",
    )

    side_width = (BODY_WIDTH - OUTLET_WIDTH) * 0.5
    side_center_x = OUTLET_WIDTH * 0.5 + side_width * 0.5
    cabinet.visual(
        Box((side_width, FASCIA_DEPTH, OUTLET_HEIGHT)),
        origin=Origin(xyz=(-side_center_x, FRONT_Y + FASCIA_DEPTH * 0.5, OUTLET_CENTER_Z)),
        material=trim_white,
        name="fascia_side_0",
    )
    cabinet.visual(
        Box((side_width, FASCIA_DEPTH, OUTLET_HEIGHT)),
        origin=Origin(xyz=(side_center_x, FRONT_Y + FASCIA_DEPTH * 0.5, OUTLET_CENTER_Z)),
        material=trim_white,
        name="fascia_side_1",
    )
    cabinet.visual(
        Box((BODY_WIDTH * 0.80, 0.004, 0.066)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.002, KNOB_CENTER_Z)),
        material=control_grey,
        name="control_strip",
    )
    cabinet.visual(
        Box((OUTLET_WIDTH, FASCIA_DEPTH * 0.30, 0.006)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.008, OUTLET_TOP + 0.003)),
        material=trim_white,
        name="vent_brow",
    )

    cavity_center_y = FRONT_Y + FASCIA_DEPTH * 0.5
    cabinet.visual(
        Box((CAVITY_WALL, FASCIA_DEPTH, OUTLET_HEIGHT)),
        origin=Origin(xyz=(-(OUTLET_INNER_WIDTH * 0.5 + CAVITY_WALL * 0.5), cavity_center_y, OUTLET_CENTER_Z)),
        material=vent_shadow,
        name="outlet_side_0",
    )
    cabinet.visual(
        Box((CAVITY_WALL, FASCIA_DEPTH, OUTLET_HEIGHT)),
        origin=Origin(xyz=(OUTLET_INNER_WIDTH * 0.5 + CAVITY_WALL * 0.5, cavity_center_y, OUTLET_CENTER_Z)),
        material=vent_shadow,
        name="outlet_side_1",
    )
    cabinet.visual(
        Box((OUTLET_INNER_WIDTH, FASCIA_DEPTH, CAVITY_LIP)),
        origin=Origin(xyz=(0.0, cavity_center_y, OUTLET_BOTTOM + CAVITY_LIP * 0.5)),
        material=vent_shadow,
        name="outlet_floor",
    )
    cabinet.visual(
        Box((OUTLET_INNER_WIDTH, FASCIA_DEPTH, CAVITY_LIP)),
        origin=Origin(xyz=(0.0, cavity_center_y, OUTLET_TOP - CAVITY_LIP * 0.5)),
        material=vent_shadow,
        name="outlet_hood",
    )
    cabinet.visual(
        Box((OUTLET_INNER_WIDTH, 0.008, OUTLET_BACKDROP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y + FASCIA_DEPTH + 0.004,
                OUTLET_CENTER_Z,
            )
        ),
        material=vent_shadow,
        name="outlet_backdrop",
    )

    knob_mesh = _build_knob_mesh()
    for index, x_pos in enumerate((-KNOB_X, KNOB_X)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=control_grey,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_dark,
            name="knob_shell",
        )
        model.articulation(
            f"cabinet_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x_pos, FRONT_Y, KNOB_CENTER_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=10.0),
        )

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Box((OUTLET_INNER_WIDTH - 0.006, 0.038, 0.003)),
        origin=Origin(xyz=(0.0, 0.019, -0.0015)),
        material=louver_white,
        name="flap_blade",
    )
    vent_flap.visual(
        Cylinder(radius=0.003, length=OUTLET_INNER_WIDTH - 0.01),
        origin=Origin(xyz=(0.0, 0.004, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=louver_white,
        name="flap_lip",
    )
    model.articulation(
        "cabinet_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=vent_flap,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, OUTLET_TOP - 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=math.radians(-32.0),
            upper=math.radians(28.0),
        ),
    )

    vane_x_positions = (-0.046, 0.046)
    for index, x_pos in enumerate(vane_x_positions):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Box((0.004, 0.024, 0.076)),
            origin=Origin(xyz=(0.0, 0.012, 0.0)),
            material=vane_grey,
            name="vane_blade",
        )
        vane.visual(
            Cylinder(radius=0.0018, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.038)),
            material=vane_grey,
            name="top_pin",
        )
        vane.visual(
            Cylinder(radius=0.0018, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=vane_grey,
            name="bottom_pin",
        )
        model.articulation(
            f"cabinet_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=vane,
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.015, OUTLET_CENTER_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=2.5,
                lower=math.radians(-24.0),
                upper=math.radians(24.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    flap = object_model.get_part("vent_flap")
    vane_0 = object_model.get_part("vane_0")
    vane_1 = object_model.get_part("vane_1")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    flap_joint = object_model.get_articulation("cabinet_to_vent_flap")
    vane_0_joint = object_model.get_articulation("cabinet_to_vane_0")
    vane_1_joint = object_model.get_articulation("cabinet_to_vane_1")

    for joint_name in ("cabinet_to_knob_0", "cabinet_to_knob_1"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={joint.articulation_type!r}, "
                f"lower={None if limits is None else limits.lower}, "
                f"upper={None if limits is None else limits.upper}"
            ),
        )

    ctx.expect_contact(knob_0, cabinet, elem_b="control_strip", name="knob_0 seats on control strip")
    ctx.expect_contact(knob_1, cabinet, elem_b="control_strip", name="knob_1 seats on control strip")

    ctx.expect_within(
        vane_0,
        cabinet,
        axes="xz",
        outer_elem="outlet_backdrop",
        margin=0.002,
        name="vane_0 sits within outlet aperture",
    )
    ctx.expect_within(
        vane_1,
        cabinet,
        axes="xz",
        outer_elem="outlet_backdrop",
        margin=0.002,
        name="vane_1 sits within outlet aperture",
    )
    ctx.expect_overlap(
        flap,
        cabinet,
        axes="x",
        elem_b="outlet_backdrop",
        min_overlap=0.19,
        name="vent flap spans most of the outlet width",
    )

    neutral_flap = ctx.part_world_aabb(flap)
    low_flap = None
    high_flap = None
    with ctx.pose({flap_joint: flap_joint.motion_limits.lower}):
        low_flap = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        high_flap = ctx.part_world_aabb(flap)

    neutral_max_y = _aabb_max(neutral_flap, "y")
    low_max_y = _aabb_max(low_flap, "y")
    high_max_y = _aabb_max(high_flap, "y")
    neutral_min_z = None if neutral_flap is None else float(neutral_flap[0][2])
    low_min_z = None if low_flap is None else float(low_flap[0][2])
    neutral_max_z = None if neutral_flap is None else float(neutral_flap[1][2])
    high_max_z = None if high_flap is None else float(high_flap[1][2])
    ctx.check(
        "vent_flap_changes_pitch",
        neutral_min_z is not None
        and low_min_z is not None
        and neutral_max_z is not None
        and high_max_z is not None
        and neutral_max_y is not None
        and low_max_y is not None
        and high_max_y is not None
        and low_min_z < neutral_min_z - 0.010
        and high_max_z > neutral_max_z + 0.010
        and abs(low_max_y - neutral_max_y) > 0.003
        and abs(high_max_y - neutral_max_y) > 0.003,
        details=(
            f"neutral_min_z={neutral_min_z}, low_min_z={low_min_z}, "
            f"neutral_max_z={neutral_max_z}, high_max_z={high_max_z}, "
            f"neutral_max_y={neutral_max_y}, low_max_y={low_max_y}, high_max_y={high_max_y}"
        ),
    )

    neutral_vane_0 = ctx.part_world_aabb(vane_0)
    neutral_vane_1 = ctx.part_world_aabb(vane_1)
    with ctx.pose({vane_0_joint: math.radians(20.0)}):
        vane_0_moved = ctx.part_world_aabb(vane_0)
        vane_1_still = ctx.part_world_aabb(vane_1)
    with ctx.pose({vane_1_joint: math.radians(-20.0)}):
        vane_1_moved = ctx.part_world_aabb(vane_1)
        vane_0_still = ctx.part_world_aabb(vane_0)

    neutral_vane_0_x = _aabb_center(neutral_vane_0, "x")
    neutral_vane_1_x = _aabb_center(neutral_vane_1, "x")
    vane_0_moved_x = _aabb_center(vane_0_moved, "x")
    vane_1_still_x = _aabb_center(vane_1_still, "x")
    vane_1_moved_x = _aabb_center(vane_1_moved, "x")
    vane_0_still_x = _aabb_center(vane_0_still, "x")

    ctx.check(
        "vane_0_pivots_independently",
        neutral_vane_0_x is not None
        and neutral_vane_1_x is not None
        and vane_0_moved_x is not None
        and vane_1_still_x is not None
        and abs(vane_0_moved_x) > abs(neutral_vane_0_x) + 0.0025
        and abs(vane_1_still_x - neutral_vane_1_x) < 1e-6,
        details=(
            f"neutral_vane_0_x={neutral_vane_0_x}, vane_0_moved_x={vane_0_moved_x}, "
            f"neutral_vane_1_x={neutral_vane_1_x}, vane_1_still_x={vane_1_still_x}"
        ),
    )
    ctx.check(
        "vane_1_pivots_independently",
        neutral_vane_0_x is not None
        and neutral_vane_1_x is not None
        and vane_1_moved_x is not None
        and vane_0_still_x is not None
        and abs(vane_1_moved_x) > abs(neutral_vane_1_x) + 0.0025
        and abs(vane_0_still_x - neutral_vane_0_x) < 1e-6,
        details=(
            f"neutral_vane_1_x={neutral_vane_1_x}, vane_1_moved_x={vane_1_moved_x}, "
            f"neutral_vane_0_x={neutral_vane_0_x}, vane_0_still_x={vane_0_still_x}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
)


PLATTER_CENTER = (-0.16, -0.02, 0.266)
TONEARM_PIVOT = (0.235, 0.170, 0.305)


def _ring_band(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 72,
):
    """Solid annular band with a visible through-hole."""
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=segments,
    )
    return boolean_difference(outer, inner)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center_xy(aabb) -> tuple[float, float] | None:
    if aabb is None:
        return None
    return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_turntable")

    concrete = model.material("sealed_concrete", rgba=(0.56, 0.56, 0.53, 1.0))
    marine_polymer = model.material("marine_black_polymer", rgba=(0.045, 0.052, 0.058, 1.0))
    deck_finish = model.material("powdercoated_aluminum", rgba=(0.18, 0.21, 0.23, 1.0))
    anodized = model.material("hard_anodized_platter", rgba=(0.50, 0.53, 0.55, 1.0))
    rubber = model.material("uv_black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    gasket = model.material("matte_epdm_gasket", rgba=(0.015, 0.017, 0.016, 1.0))
    stainless = model.material("316_stainless", rgba=(0.76, 0.78, 0.76, 1.0))
    cartridge_body = model.material("sealed_cartridge", rgba=(0.10, 0.12, 0.13, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.90, 0.70, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=concrete,
        name="concrete_anchor_slab",
    )
    plinth.visual(
        Box((0.76, 0.56, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=marine_polymer,
        name="sealed_plinth_body",
    )
    plinth.visual(
        Box((0.84, 0.64, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=deck_finish,
        name="overhanging_deck",
    )
    # Downturned lips under the deck create a rain-shedding drip break.
    plinth.visual(
        Box((0.84, 0.024, 0.055)),
        origin=Origin(xyz=(0.0, -0.332, 0.197)),
        material=deck_finish,
        name="front_drip_lip",
    )
    plinth.visual(
        Box((0.84, 0.024, 0.055)),
        origin=Origin(xyz=(0.0, 0.332, 0.197)),
        material=deck_finish,
        name="rear_drip_lip",
    )
    plinth.visual(
        Box((0.024, 0.64, 0.055)),
        origin=Origin(xyz=(-0.432, 0.0, 0.197)),
        material=deck_finish,
        name="side_drip_lip_0",
    )
    plinth.visual(
        Box((0.024, 0.64, 0.055)),
        origin=Origin(xyz=(0.432, 0.0, 0.197)),
        material=deck_finish,
        name="side_drip_lip_1",
    )
    plinth.visual(
        Box((0.71, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.285, 0.244)),
        material=gasket,
        name="front_deck_gasket",
    )
    plinth.visual(
        Box((0.71, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.285, 0.244)),
        material=gasket,
        name="rear_deck_gasket",
    )
    plinth.visual(
        Box((0.018, 0.50, 0.010)),
        origin=Origin(xyz=(-0.365, 0.0, 0.244)),
        material=gasket,
        name="side_deck_gasket_0",
    )
    plinth.visual(
        Box((0.018, 0.50, 0.010)),
        origin=Origin(xyz=(0.365, 0.0, 0.244)),
        material=gasket,
        name="side_deck_gasket_1",
    )

    # Platter support: a raised, sealed bearing sleeve with a real central hole.
    plinth.visual(
        mesh_from_geometry(_ring_band(0.076, 0.022, 0.026), "platter_bearing_collar"),
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], 0.253)),
        material=stainless,
        name="platter_bearing_collar",
    )
    plinth.visual(
        mesh_from_geometry(TorusGeometry(radius=0.190, tube=0.006), "platter_deck_gasket"),
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], 0.246)),
        material=gasket,
        name="platter_deck_gasket",
    )

    # Tonearm pivot support: fixed pedestal, boot, and stainless thrust land.
    plinth.visual(
        Cylinder(radius=0.060, length=0.065),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], 0.2725)),
        material=stainless,
        name="tonearm_pedestal",
    )
    plinth.visual(
        mesh_from_geometry(TorusGeometry(radius=0.068, tube=0.006), "tonearm_boot_gasket"),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], 0.246)),
        material=gasket,
        name="tonearm_boot_gasket",
    )
    plinth.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], 0.300)),
        material=stainless,
        name="tonearm_thrust_land",
    )

    for index, (x, y) in enumerate(
        [(-0.34, -0.25), (0.34, -0.25), (-0.34, 0.25), (0.34, 0.25)]
    ):
        plinth.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(x, y, 0.093)),
            material=stainless,
            name=f"anchor_washer_{index}",
        )
        plinth.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(x, y, 0.100)),
            material=stainless,
            name=f"anchor_bolt_{index}",
        )

    for index, (x, y) in enumerate(
        [(-0.32, -0.255), (-0.08, -0.255), (0.16, -0.255), (0.32, -0.255)]
    ):
        plinth.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.243)),
            material=stainless,
            name=f"deck_screw_{index}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.065, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stainless,
        name="thrust_hub",
    )
    platter.visual(
        Cylinder(radius=0.181, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=anodized,
        name="platter_disc",
    )
    platter.visual(
        mesh_from_geometry(TorusGeometry(radius=0.172, tube=0.008), "raised_platter_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=anodized,
        name="raised_platter_rim",
    )
    platter.visual(
        Cylinder(radius=0.154, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=rubber,
        name="drainable_rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.115, -0.080, 0.0575)),
        material=stainless,
        name="platter_index_dot",
    )
    platter.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=stainless,
        name="record_spindle",
    )

    tonearm_stage = model.part("tonearm_stage")
    tonearm_stage.visual(
        Cylinder(radius=0.067, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=stainless,
        name="pivot_cap",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.028, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=stainless,
        name="sealed_pivot_post",
    )
    tonearm_stage.visual(
        Box((0.110, 0.038, 0.026)),
        origin=Origin(xyz=(-0.040, -0.010, 0.076)),
        material=deck_finish,
        name="tonearm_stage_block",
    )
    _add_tube(
        tonearm_stage,
        (-0.055, -0.014, 0.078),
        (-0.330, -0.120, 0.066),
        0.008,
        stainless,
        name="tonearm_tube",
    )
    arm_yaw = math.atan2(-0.120 + 0.014, -0.330 + 0.055)
    tonearm_stage.visual(
        Box((0.070, 0.036, 0.011)),
        origin=Origin(xyz=(-0.356, -0.130, 0.064), rpy=(0.0, 0.0, arm_yaw)),
        material=deck_finish,
        name="sealed_headshell",
    )
    tonearm_stage.visual(
        Box((0.024, 0.020, 0.020)),
        origin=Origin(xyz=(-0.382, -0.140, 0.049), rpy=(0.0, 0.0, arm_yaw)),
        material=cartridge_body,
        name="sealed_cartridge",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0025, length=0.020),
        origin=Origin(xyz=(-0.388, -0.142, 0.030)),
        material=stainless,
        name="stylus_guard_pin",
    )

    model.articulation(
        "platter_spindle",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=PLATTER_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.3),
    )
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm_stage,
        origin=Origin(xyz=TONEARM_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.55, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm_stage = object_model.get_part("tonearm_stage")
    platter_spindle = object_model.get_articulation("platter_spindle")
    tonearm_pivot = object_model.get_articulation("tonearm_pivot")

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="thrust_hub",
        elem_b="platter_bearing_collar",
        contact_tol=0.001,
        name="platter hub seats on bearing collar",
    )
    ctx.expect_contact(
        tonearm_stage,
        plinth,
        elem_a="pivot_cap",
        elem_b="tonearm_thrust_land",
        contact_tol=0.001,
        name="tonearm cap seats on thrust land",
    )
    ctx.expect_gap(
        tonearm_stage,
        platter,
        axis="z",
        positive_elem="stylus_guard_pin",
        negative_elem="drainable_rubber_mat",
        min_gap=0.0005,
        max_gap=0.010,
        name="protected stylus clears the weather mat",
    )

    rest_platter_position = ctx.part_world_position(platter)
    rest_dot = _aabb_center_xy(ctx.part_element_world_aabb(platter, elem="platter_index_dot"))
    with ctx.pose({platter_spindle: math.pi / 2.0}):
        ctx.expect_contact(
            platter,
            plinth,
            elem_a="thrust_hub",
            elem_b="platter_bearing_collar",
            contact_tol=0.001,
            name="rotated platter remains bearing-supported",
        )
        rotated_platter_position = ctx.part_world_position(platter)
        rotated_dot = _aabb_center_xy(ctx.part_element_world_aabb(platter, elem="platter_index_dot"))
    ctx.check(
        "platter spins about fixed spindle",
        rest_platter_position is not None
        and rotated_platter_position is not None
        and all(
            abs(rest_platter_position[i] - rotated_platter_position[i]) < 1e-6
            for i in range(3)
        ),
        details=f"rest={rest_platter_position}, rotated={rotated_platter_position}",
    )
    ctx.check(
        "platter index dot follows rotation",
        rest_dot is not None
        and rotated_dot is not None
        and math.hypot(rest_dot[0] - rotated_dot[0], rest_dot[1] - rotated_dot[1]) > 0.12,
        details=f"rest_dot={rest_dot}, rotated_dot={rotated_dot}",
    )

    rest_arm_position = ctx.part_world_position(tonearm_stage)
    rest_stylus = _aabb_center_xy(ctx.part_element_world_aabb(tonearm_stage, elem="stylus_guard_pin"))
    with ctx.pose({tonearm_pivot: 0.55}):
        ctx.expect_contact(
            tonearm_stage,
            plinth,
            elem_a="pivot_cap",
            elem_b="tonearm_thrust_land",
            contact_tol=0.001,
            name="pivoted tonearm remains on protected bearing",
        )
        pivoted_arm_position = ctx.part_world_position(tonearm_stage)
        pivoted_stylus = _aabb_center_xy(
            ctx.part_element_world_aabb(tonearm_stage, elem="stylus_guard_pin")
        )
    ctx.check(
        "tonearm pivots about fixed stage post",
        rest_arm_position is not None
        and pivoted_arm_position is not None
        and all(abs(rest_arm_position[i] - pivoted_arm_position[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_arm_position}, pivoted={pivoted_arm_position}",
    )
    ctx.check(
        "tonearm stylus sweeps on pivot",
        rest_stylus is not None
        and pivoted_stylus is not None
        and math.hypot(rest_stylus[0] - pivoted_stylus[0], rest_stylus[1] - pivoted_stylus[1])
        > 0.10,
        details=f"rest_stylus={rest_stylus}, pivoted_stylus={pivoted_stylus}",
    )

    return ctx.report()


object_model = build_object_model()

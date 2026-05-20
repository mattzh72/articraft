from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_disc_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
) -> object:
    return mesh_from_geometry(
        LatheGeometry(
            [
                (inner_radius, -thickness * 0.5),
                (outer_radius, -thickness * 0.5),
                (outer_radius, thickness * 0.5),
                (inner_radius, thickness * 0.5),
            ],
            segments=128,
        ),
        name,
    )


def _add_index_marks(
    part,
    *,
    count: int,
    radius: float,
    mark_radius: float,
    z: float,
    material,
    prefix: str,
) -> None:
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        part.visual(
            Cylinder(radius=mark_radius, length=0.0025),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z - 0.00125,
                )
            ),
            material=material,
            name=f"{prefix}_{i:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_ring_coaxial_indexing_stack")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.18, 0.23, 0.28, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.56, 0.55, 0.50, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    blackened = model.material("blackened", rgba=(0.045, 0.045, 0.045, 1.0))
    amber = model.material("amber_index_mark", rgba=(1.0, 0.63, 0.12, 1.0))
    red = model.material("red_index_mark", rgba=(0.85, 0.08, 0.04, 1.0))

    # Stationary base, spindle, bearing collars, and the fixed index pointer.
    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.45, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=blackened,
        name="base_plinth",
    )
    spindle.visual(
        Cylinder(radius=0.040, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=brushed_steel,
        name="center_spindle",
    )
    spindle.visual(
        Cylinder(radius=0.120, length=0.0125),
        origin=Origin(xyz=(0.0, 0.0, 0.04625)),
        material=brushed_steel,
        name="lower_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.105, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=brushed_steel,
        name="middle_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.090, length=0.0125),
        origin=Origin(xyz=(0.0, 0.0, 0.17875)),
        material=brushed_steel,
        name="top_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.088, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.2325)),
        material=dark_steel,
        name="top_clamp_nut",
    )
    spindle.visual(
        Box((0.035, 0.045, 0.220)),
        origin=Origin(xyz=(0.455, 0.0, 0.150)),
        material=blackened,
        name="pointer_post",
    )
    spindle.visual(
        Box((0.074, 0.018, 0.012)),
        origin=Origin(xyz=(0.402, 0.0, 0.102)),
        material=amber,
        name="lower_pointer",
    )
    spindle.visual(
        Box((0.180, 0.016, 0.010)),
        origin=Origin(xyz=(0.352, 0.0, 0.150)),
        material=amber,
        name="middle_pointer",
    )
    spindle.visual(
        Box((0.260, 0.014, 0.010)),
        origin=Origin(xyz=(0.310, 0.0, 0.2025)),
        material=amber,
        name="top_pointer",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.45, length=0.270),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        _annular_disc_mesh(
            "lower_turntable_plate",
            outer_radius=0.360,
            inner_radius=0.075,
            thickness=0.065,
        ),
        material=blued_steel,
        name="turntable_plate",
    )
    _add_index_marks(
        lower_stage,
        count=24,
        radius=0.306,
        mark_radius=0.012,
        z=0.0325,
        material=blackened,
        prefix="lower_index_socket",
    )
    lower_stage.visual(
        _annular_disc_mesh(
            "lower_heavy_skirt",
            outer_radius=0.350,
            inner_radius=0.130,
            thickness=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=dark_steel,
        name="heavy_lower_skirt",
    )
    lower_stage.visual(
        Box((0.090, 0.014, 0.012)),
        origin=Origin(xyz=(0.310, 0.0, 0.0385)),
        material=amber,
        name="lower_zero_mark",
    )
    lower_stage.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(-0.310, 0.0, 0.0395)),
        material=red,
        name="lower_detent_pin",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.075),
        mass=24.0,
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        _annular_disc_mesh(
            "middle_index_ring",
            outer_radius=0.260,
            inner_radius=0.075,
            thickness=0.045,
        ),
        material=warm_steel,
        name="middle_ring",
    )
    _add_index_marks(
        middle_stage,
        count=18,
        radius=0.215,
        mark_radius=0.010,
        z=0.0225,
        material=blackened,
        prefix="middle_index_socket",
    )
    middle_stage.visual(
        Box((0.070, 0.012, 0.010)),
        origin=Origin(xyz=(0.185, 0.0, 0.0275)),
        material=amber,
        name="middle_zero_mark",
    )
    middle_stage.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(-0.185, 0.0, 0.0285)),
        material=red,
        name="middle_detent_pin",
    )
    middle_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.26, length=0.052),
        mass=8.5,
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        _annular_disc_mesh(
            "top_flange_plate",
            outer_radius=0.170,
            inner_radius=0.075,
            thickness=0.035,
        ),
        material=brushed_steel,
        name="top_flange",
    )
    _add_index_marks(
        top_stage,
        count=12,
        radius=0.130,
        mark_radius=0.007,
        z=0.0175,
        material=blackened,
        prefix="top_index_socket",
    )
    top_stage.visual(
        _annular_disc_mesh(
            "top_annular_cap",
            outer_radius=0.150,
            inner_radius=0.095,
            thickness=0.008,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=dark_steel,
        name="top_flange_cap",
    )
    top_stage.visual(
        Box((0.050, 0.010, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, 0.0275)),
        material=amber,
        name="top_zero_mark",
    )
    top_stage.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.112, 0.0, 0.0285)),
        material=red,
        name="top_detent_pin",
    )
    top_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.043),
        mass=3.5,
    )

    model.articulation(
        "lower_rotation",
        ArticulationType.REVOLUTE,
        parent=spindle,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "middle_rotation",
        ArticulationType.REVOLUTE,
        parent=spindle,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "top_rotation",
        ArticulationType.REVOLUTE,
        parent=spindle,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.2025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spindle = object_model.get_part("spindle")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    top_stage = object_model.get_part("top_stage")
    lower_joint = object_model.get_articulation("lower_rotation")
    middle_joint = object_model.get_articulation("middle_rotation")
    top_joint = object_model.get_articulation("top_rotation")

    for joint in (lower_joint, middle_joint, top_joint):
        ctx.check(
            f"{joint.name} is a vertical revolute axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and abs(joint.origin.xyz[0]) < 1.0e-9
            and abs(joint.origin.xyz[1]) < 1.0e-9,
            details=f"type={joint.articulation_type}, axis={joint.axis}, origin={joint.origin}",
        )

    ctx.expect_gap(
        lower_stage,
        spindle,
        axis="z",
        positive_elem="turntable_plate",
        negative_elem="lower_bearing_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower turntable rests on lower bearing collar",
    )
    ctx.expect_gap(
        middle_stage,
        spindle,
        axis="z",
        positive_elem="middle_ring",
        negative_elem="middle_bearing_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="middle ring rests on middle bearing collar",
    )
    ctx.expect_gap(
        top_stage,
        spindle,
        axis="z",
        positive_elem="top_flange",
        negative_elem="top_bearing_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="top flange rests on top bearing collar",
    )

    rest_positions = {
        "lower_stage": ctx.part_world_position(lower_stage),
        "middle_stage": ctx.part_world_position(middle_stage),
        "top_stage": ctx.part_world_position(top_stage),
    }
    with ctx.pose({lower_joint: 0.65, middle_joint: -0.45, top_joint: 0.90}):
        moved_positions = {
            "lower_stage": ctx.part_world_position(lower_stage),
            "middle_stage": ctx.part_world_position(middle_stage),
            "top_stage": ctx.part_world_position(top_stage),
        }

    ctx.check(
        "all three stages rotate without leaving the common centerline",
        all(
            rest_positions[name] is not None
            and moved_positions[name] is not None
            and abs(rest_positions[name][0] - moved_positions[name][0]) < 1.0e-9
            and abs(rest_positions[name][1] - moved_positions[name][1]) < 1.0e-9
            and abs(rest_positions[name][2] - moved_positions[name][2]) < 1.0e-9
            for name in rest_positions
        ),
        details=f"rest={rest_positions}, moved={moved_positions}",
    )

    return ctx.report()


object_model = build_object_model()

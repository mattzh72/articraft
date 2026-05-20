from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_STEP_Z = 0.080


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    center_distance: float,
    radius: float,
    *,
    segments_per_end: int = 24,
) -> list[tuple[float, float]]:
    """Dogbone side-plate outline with pivot centers at x=0 and x=center_distance."""
    points: list[tuple[float, float]] = []
    for i in range(segments_per_end + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments_per_end
        points.append(
            (
                center_distance + radius * math.cos(angle),
                radius * math.sin(angle),
            )
        )
    for i in range(segments_per_end + 1):
        angle = math.pi / 2.0 + math.pi * i / segments_per_end
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _dogbone_plate_mesh(
    center_distance: float,
    outer_radius: float,
    bore_radius: float,
    thickness: float,
    mesh_name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _capsule_profile(center_distance, outer_radius),
            [
                _circle_profile(bore_radius, center=(0.0, 0.0), segments=36),
                _circle_profile(bore_radius, center=(center_distance, 0.0), segments=36),
            ],
            thickness,
            center=True,
        ),
        mesh_name,
    )


def _add_rotary_module(
    part,
    *,
    module_name: str,
    offset_xy: tuple[float, float],
    plate_material: Material,
    steel_material: Material,
    spacer_material: Material,
) -> None:
    """Add a two-side-plate rotary link from its input axis to an offset output axis."""
    dx, dy = offset_xy
    span = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    plate_radius = 0.046
    bore_radius = 0.018
    plate_thickness = 0.010
    plate_mesh = _dogbone_plate_mesh(
        span,
        plate_radius,
        bore_radius,
        plate_thickness,
        f"{module_name}_side_plate",
    )

    # Two rigid side plates form the visible stack layer.  The holes are real mesh
    # openings, while press-fit bushings and spacer sleeves tie the plates together.
    part.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.017), rpy=(0.0, 0.0, yaw)),
        material=plate_material,
        name="lower_side_plate",
    )
    part.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.062), rpy=(0.0, 0.0, yaw)),
        material=plate_material,
        name="upper_side_plate",
    )

    # Vertical bearing sleeves at the input and output pivots.  The literal
    # visual names are referenced by exact tests below.
    part.visual(
        Cylinder(radius=0.021, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=steel_material,
        name="input_bushing",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel_material,
        name="input_lower_flange",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=steel_material,
        name="input_upper_flange",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.068),
        origin=Origin(xyz=(dx, dy, 0.034)),
        material=steel_material,
        name="output_bushing",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(dx, dy, 0.007)),
        material=steel_material,
        name="output_lower_flange",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(dx, dy, 0.065)),
        material=steel_material,
        name="output_upper_flange",
    )

    # The output cap is the mechanical seat for the next serial module.
    part.visual(
        Cylinder(radius=0.037, length=0.012),
        origin=Origin(xyz=(dx, dy, 0.074)),
        material=steel_material,
        name="output_thrust_cap",
    )

    # Four standoffs at the dogbone lobes make the side plates read as a bracketed
    # spacer assembly instead of as unsupported parallel sheets.
    nx = -dy / span
    ny = dx / span
    for axis_index, (base_x, base_y) in enumerate(((0.0, 0.0), (dx, dy))):
        for side_index, sign in enumerate((-1.0, 1.0)):
            x = base_x + sign * nx * 0.030
            y = base_y + sign * ny * 0.030
            part.visual(
                Cylinder(radius=0.0085, length=0.058),
                origin=Origin(xyz=(x, y, 0.040)),
                material=spacer_material,
                name=f"spacer_{axis_index}_{side_index}",
            )
            # Small rectangular feet at the standoff ends show the bolted bracket pads.
            part.visual(
                Box((0.026, 0.014, 0.006)),
                origin=Origin(xyz=(x, y, 0.017), rpy=(0.0, 0.0, yaw)),
                material=spacer_material,
                name=f"lower_pad_{axis_index}_{side_index}",
            )
            part.visual(
                Box((0.026, 0.014, 0.006)),
                origin=Origin(xyz=(x, y, 0.062), rpy=(0.0, 0.0, yaw)),
                material=spacer_material,
                name=f"upper_pad_{axis_index}_{side_index}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack")

    black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.24, 0.25, 0.26, 1.0))
    brushed_steel = model.material("brushed_spacer_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    blue = model.material("blue_anodized_plate", rgba=(0.05, 0.22, 0.62, 1.0))
    orange = model.material("orange_anodized_plate", rgba=(0.95, 0.42, 0.08, 1.0))
    green = model.material("green_anodized_plate", rgba=(0.08, 0.45, 0.22, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.02, 0.02, 0.018, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.78, 0.48, 0.035)),
        origin=Origin(xyz=(0.270, -0.025, 0.0175)),
        material=black,
        name="base_plate",
    )
    for i, (x, y) in enumerate(((-0.075, -0.205), (-0.075, 0.155), (0.615, -0.205), (0.615, 0.155))):
        base_frame.visual(
            Cylinder(radius=0.026, length=0.012),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=f"foot_{i}",
        )
    base_frame.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_steel,
        name="root_pedestal",
    )
    base_frame.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=brushed_steel,
        name="root_bearing_cap",
    )
    for index, y in enumerate((-0.072, 0.072)):
        base_frame.visual(
            Box((0.115, 0.012, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.052)),
            material=dark_steel,
            name=f"root_side_plate_{index}",
        )
        base_frame.visual(
            Box((0.070, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=brushed_steel,
            name=f"root_bracket_foot_{index}",
        )

    module_0 = model.part("module_0")
    module_1 = model.part("module_1")
    module_2 = model.part("module_2")

    offset_0 = (0.240, 0.060)
    offset_1 = (0.180, -0.160)
    offset_2 = (0.145, 0.120)
    _add_rotary_module(
        module_0,
        module_name="module_0",
        offset_xy=offset_0,
        plate_material=blue,
        steel_material=dark_steel,
        spacer_material=brushed_steel,
    )
    _add_rotary_module(
        module_1,
        module_name="module_1",
        offset_xy=offset_1,
        plate_material=orange,
        steel_material=dark_steel,
        spacer_material=brushed_steel,
    )
    _add_rotary_module(
        module_2,
        module_name="module_2",
        offset_xy=offset_2,
        plate_material=green,
        steel_material=dark_steel,
        spacer_material=brushed_steel,
    )
    module_2.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(offset_2[0], offset_2[1], 0.086)),
        material=brushed_steel,
        name="terminal_flange",
    )
    module_2.visual(
        Box((0.095, 0.040, 0.012)),
        origin=Origin(xyz=(offset_2[0], offset_2[1], 0.098), rpy=(0.0, 0.0, math.atan2(offset_2[1], offset_2[0]))),
        material=green,
        name="terminal_mount_bar",
    )

    model.articulation(
        "base_to_module_0",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=module_0,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "module_0_to_module_1",
        ArticulationType.REVOLUTE,
        parent=module_0,
        child=module_1,
        origin=Origin(xyz=(offset_0[0], offset_0[1], FRAME_STEP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "module_1_to_module_2",
        ArticulationType.REVOLUTE,
        parent=module_1,
        child=module_2,
        origin=Origin(xyz=(offset_1[0], offset_1[1], FRAME_STEP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-1.85, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("base_to_module_0"),
        object_model.get_articulation("module_0_to_module_1"),
        object_model.get_articulation("module_1_to_module_2"),
    ]
    ctx.check(
        "serial joints are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.articulation_type for j in joints]),
    )
    ctx.check(
        "rotary axes are parallel vertical",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=str([j.axis for j in joints]),
    )

    module_0 = object_model.get_part("module_0")
    module_1 = object_model.get_part("module_1")
    module_2 = object_model.get_part("module_2")

    pos_0 = ctx.part_world_position(module_0)
    pos_1 = ctx.part_world_position(module_1)
    pos_2 = ctx.part_world_position(module_2)
    ctx.check(
        "successive axes are laterally displaced",
        (
            pos_0 is not None
            and pos_1 is not None
            and pos_2 is not None
            and math.hypot(pos_1[0] - pos_0[0], pos_1[1] - pos_0[1]) > 0.20
            and math.hypot(pos_2[0] - pos_1[0], pos_2[1] - pos_1[1]) > 0.20
        ),
        details=f"axis positions: {pos_0}, {pos_1}, {pos_2}",
    )
    ctx.expect_gap(
        module_0,
        "base_frame",
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="input_bushing",
        negative_elem="root_bearing_cap",
        name="first rotary module seats on root bearing",
    )
    ctx.expect_gap(
        module_1,
        module_0,
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="input_bushing",
        negative_elem="output_thrust_cap",
        name="second module seats on offset thrust cap",
    )
    ctx.expect_gap(
        module_2,
        module_1,
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="input_bushing",
        negative_elem="output_thrust_cap",
        name="third module seats on offset thrust cap",
    )

    rest_pos_2 = ctx.part_world_position(module_2)
    with ctx.pose({"base_to_module_0": 0.55}):
        swung_pos_2 = ctx.part_world_position(module_2)
    ctx.check(
        "base rotation carries the serial stack laterally",
        (
            rest_pos_2 is not None
            and swung_pos_2 is not None
            and math.hypot(swung_pos_2[0] - rest_pos_2[0], swung_pos_2[1] - rest_pos_2[1]) > 0.10
        ),
        details=f"rest={rest_pos_2}, swung={swung_pos_2}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_spur_reduction")

    painted_steel = model.material("painted_steel", rgba=(0.10, 0.13, 0.16, 1.0))
    bearing_iron = model.material("bearing_iron", rgba=(0.20, 0.22, 0.24, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.74, 0.76, 0.72, 1.0))
    input_bronze = model.material("input_bronze", rgba=(0.83, 0.55, 0.18, 1.0))
    idler_steel = model.material("idler_steel", rgba=(0.58, 0.61, 0.64, 1.0))
    output_brass = model.material("output_brass", rgba=(0.76, 0.63, 0.28, 1.0))
    black_bakelite = model.material("black_bakelite", rgba=(0.015, 0.014, 0.013, 1.0))
    red_handle = model.material("red_handle", rgba=(0.55, 0.06, 0.04, 1.0))

    frame = model.part("plate_frame")

    # Two narrow side-plate frames hold the shafts while leaving the gear faces open.
    frame_half_width = 0.290
    frame_y = 0.072
    frame_z = 0.275
    bottom_z = 0.090
    top_z = 0.460
    rail_thick = 0.030
    plate_thick = 0.012
    for side_y, side_name in ((-frame_y, "rear"), (frame_y, "front")):
        frame.visual(
            Box((0.590, plate_thick, rail_thick)),
            origin=Origin(xyz=(0.0, side_y, bottom_z)),
            material=painted_steel,
            name=f"{side_name}_bottom_rail",
        )
        frame.visual(
            Box((0.590, plate_thick, rail_thick)),
            origin=Origin(xyz=(0.0, side_y, top_z)),
            material=painted_steel,
            name=f"{side_name}_top_rail",
        )
        for x, post_name in ((-frame_half_width, "post_0"), (frame_half_width, "post_1")):
            frame.visual(
                Box((rail_thick, plate_thick, top_z - bottom_z + rail_thick)),
                origin=Origin(xyz=(x, side_y, frame_z)),
                material=painted_steel,
                name=f"{side_name}_{post_name}",
            )

    # Corner spacers tie the two side plates into one rigid frame.
    for x, x_name in ((-frame_half_width, "post_0"), (frame_half_width, "post_1")):
        for z, z_name in ((bottom_z, "lower"), (top_z, "upper")):
            frame.visual(
                Box((0.034, 2.0 * frame_y + plate_thick, 0.034)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=painted_steel,
                name=f"{z_name}_{x_name}_spacer",
            )

    shaft_z = 0.285
    gear_width = 0.024
    shaft_radius = 0.011
    shaft_length = 0.245
    centers = {
        "input": -0.176,
        "idler": -0.020,
        "output": 0.160,
    }

    for side_y, side_name in ((-frame_y, "rear"), (frame_y, "front")):
        for shaft_name, x in centers.items():
            frame.visual(
                Box((0.024, plate_thick + 0.004, 0.152)),
                origin=Origin(xyz=(x, side_y, 0.180)),
                material=painted_steel,
                name=f"{side_name}_{shaft_name}_web",
            )
            frame.visual(
                Box((0.060, plate_thick + 0.010, 0.012)),
                origin=Origin(xyz=(x, side_y, shaft_z - 0.024)),
                material=bearing_iron,
                name=f"{side_name}_{shaft_name}_bearing",
            )
            for dx, cheek_name in ((-0.028, "cheek_0"), (0.028, "cheek_1")):
                frame.visual(
                    Box((0.010, plate_thick + 0.010, 0.056)),
                    origin=Origin(xyz=(x + dx, side_y, shaft_z + 0.005)),
                    material=bearing_iron,
                    name=f"{side_name}_{shaft_name}_{cheek_name}",
                )
            frame.visual(
                Box((0.066, plate_thick + 0.010, 0.010)),
                origin=Origin(xyz=(x, side_y, shaft_z + 0.038)),
                material=bearing_iron,
                name=f"{side_name}_{shaft_name}_cap",
            )

    def spur_profile(teeth: int, module: float = 0.004) -> list[tuple[float, float]]:
        pitch_radius = 0.5 * module * teeth
        root_radius = pitch_radius - 1.10 * module
        tip_radius = pitch_radius + 1.00 * module
        tooth_pitch = 2.0 * math.pi / teeth
        profile: list[tuple[float, float]] = []
        for i in range(teeth):
            tooth_center = i * tooth_pitch
            for frac, radius in (
                (-0.50, root_radius),
                (-0.23, root_radius),
                (-0.13, tip_radius),
                (0.13, tip_radius),
                (0.23, root_radius),
                (0.50, root_radius),
            ):
                angle = tooth_center + frac * tooth_pitch
                profile.append((radius * math.cos(angle), radius * math.sin(angle)))
        return profile

    def spur_mesh(name: str, teeth: int, _material: Material, _hub_diameter: float, _spoke_count: int):
        return mesh_from_geometry(
            ExtrudeGeometry(spur_profile(teeth), gear_width, center=True),
            name,
        )

    input_shaft = model.part("input_shaft")
    idler_shaft = model.part("idler_shaft")
    output_shaft = model.part("output_shaft")

    def add_shaft_core(shaft_part):
        shaft_part.visual(
            Cylinder(shaft_radius, shaft_length),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name="shaft_bar",
        )
        shaft_part.visual(
            Cylinder(0.018, 0.018),
            origin=Origin(xyz=(0.0, frame_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name="front_journal",
        )
        shaft_part.visual(
            Cylinder(0.018, 0.018),
            origin=Origin(xyz=(0.0, -frame_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name="rear_journal",
        )

    add_shaft_core(input_shaft)
    input_shaft.visual(
        spur_mesh("input_gear", 22, input_bronze, 0.042, 4),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=input_bronze,
        name="input_gear",
    )
    add_shaft_core(idler_shaft)
    idler_shaft.visual(
        spur_mesh("idler_gear", 50, idler_steel, 0.056, 6),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=idler_steel,
        name="idler_gear",
    )
    add_shaft_core(output_shaft)
    output_shaft.visual(
        spur_mesh("output_gear", 34, output_brass, 0.048, 5),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=output_brass,
        name="output_gear",
    )

    # Handwheel mounted on the input shaft, outside the front bearing.
    handwheel_y = 0.132
    input_shaft.visual(
        mesh_from_geometry(TorusGeometry(0.055, 0.006, radial_segments=40, tubular_segments=12), "handwheel_rim"),
        origin=Origin(xyz=(0.0, handwheel_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_bakelite,
        name="handwheel_rim",
    )
    input_shaft.visual(
        Cylinder(0.019, 0.042),
        origin=Origin(xyz=(0.0, handwheel_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_bakelite,
        name="handwheel_hub",
    )
    input_shaft.visual(
        Box((0.102, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, handwheel_y, 0.0)),
        material=black_bakelite,
        name="handwheel_spoke_x",
    )
    input_shaft.visual(
        Box((0.010, 0.010, 0.102)),
        origin=Origin(xyz=(0.0, handwheel_y, 0.0)),
        material=black_bakelite,
        name="handwheel_spoke_z",
    )
    input_shaft.visual(
        Cylinder(0.0065, 0.060),
        origin=Origin(xyz=(0.0, handwheel_y + 0.027, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="spinner_stem",
    )
    input_shaft.visual(
        Cylinder(0.011, 0.038),
        origin=Origin(xyz=(0.0, handwheel_y + 0.065, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_handle,
        name="spinner_grip",
    )

    joint_limits = MotionLimits(effort=8.0, velocity=10.0, lower=-2.0 * math.pi, upper=2.0 * math.pi)
    model.articulation(
        "frame_to_input",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(centers["input"], 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "frame_to_idler",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=idler_shaft,
        origin=Origin(xyz=(centers["idler"], 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
        mimic=Mimic("frame_to_input", multiplier=-(22.0 / 50.0)),
    )
    model.articulation(
        "frame_to_output",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(centers["output"], 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
        mimic=Mimic("frame_to_input", multiplier=(22.0 / 34.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("plate_frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_joint = object_model.get_articulation("frame_to_input")
    idler_joint = object_model.get_articulation("frame_to_idler")
    output_joint = object_model.get_articulation("frame_to_output")

    ctx.check(
        "three_supported_revolute_shafts",
        all(j is not None and tuple(j.axis) == (0.0, 1.0, 0.0) for j in (input_joint, idler_joint, output_joint)),
        "Expected three parallel shaft joints about the side-to-side Y axis.",
    )

    for shaft, shaft_name in (
        (input_shaft, "input"),
        (idler_shaft, "idler"),
        (output_shaft, "output"),
    ):
        ctx.expect_contact(
            shaft,
            frame,
            elem_a="front_journal",
            elem_b=f"front_{shaft_name}_bearing",
            contact_tol=0.001,
            name=f"{shaft_name} shaft seated in front bearing",
        )
        ctx.expect_contact(
            shaft,
            frame,
            elem_a="rear_journal",
            elem_b=f"rear_{shaft_name}_bearing",
            contact_tol=0.001,
            name=f"{shaft_name} shaft seated in rear bearing",
        )

    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        positive_elem="idler_gear",
        negative_elem="input_gear",
        min_gap=0.002,
        max_gap=0.007,
        name="input gear is close to idler without collision",
    )
    ctx.expect_overlap(
        idler_shaft,
        input_shaft,
        axes="yz",
        elem_a="idler_gear",
        elem_b="input_gear",
        min_overlap=0.020,
        name="input and idler gears share the same mesh plane",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        positive_elem="output_gear",
        negative_elem="idler_gear",
        min_gap=0.002,
        max_gap=0.007,
        name="idler gear is close to output without collision",
    )
    ctx.expect_overlap(
        output_shaft,
        idler_shaft,
        axes="yz",
        elem_a="output_gear",
        elem_b="idler_gear",
        min_overlap=0.020,
        name="idler and output gears share the same mesh plane",
    )

    def element_size(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(float(maxs[i] - mins[i]) for i in range(3))

    input_size = element_size(input_shaft, "input_gear")
    idler_size = element_size(idler_shaft, "idler_gear")
    output_size = element_size(output_shaft, "output_gear")
    ctx.check(
        "gear_diameters_show_reduction_steps",
        input_size is not None
        and idler_size is not None
        and output_size is not None
        and idler_size[0] > input_size[0] * 1.9
        and idler_size[0] > output_size[0] * 1.35
        and output_size[0] > input_size[0] * 1.35,
        details=f"input={input_size}, idler={idler_size}, output={output_size}",
    )

    rest_positions = tuple(ctx.part_world_position(p) for p in (input_shaft, idler_shaft, output_shaft))
    with ctx.pose({input_joint: 1.0}):
        posed_positions = tuple(ctx.part_world_position(p) for p in (input_shaft, idler_shaft, output_shaft))
    ctx.check(
        "gear_train_spins_about_fixed_parallel_axes",
        rest_positions == posed_positions and all(p is not None for p in posed_positions),
        details=f"rest={rest_positions}, posed={posed_positions}",
    )

    return ctx.report()


object_model = build_object_model()

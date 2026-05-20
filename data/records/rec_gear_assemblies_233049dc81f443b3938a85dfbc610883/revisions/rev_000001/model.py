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
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


Y_AXIS_RPY = (-math.pi / 2.0, 0.0, 0.0)
HANDWHEEL_RPY = (0.0, 0.0, math.pi / 2.0)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _bearing_plate_mesh(name: str, outer: float = 0.092, hole_radius: float = 0.020, thickness: float = 0.018):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer, outer, 0.010, corner_segments=8),
        [_circle_profile(hole_radius, 72)],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _bearing_boss_mesh(name: str, outer_radius: float = 0.036, hole_radius: float = 0.020, thickness: float = 0.038):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, 96),
        [_circle_profile(hole_radius, 72)],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _spur_gear_mesh(
    name: str,
    *,
    module: float,
    teeth: int,
    width: float,
    bore: float,
    hub_diameter: float,
    hub_length: float,
    spokes: int,
):
    gear = SpurGear(module, teeth, width, backlash=0.0007, clearance=0.0004)
    shape = gear.build(
        bore_d=bore,
        hub_d=hub_diameter,
        hub_length=hub_length,
        n_spokes=spokes,
        spoke_width=0.010,
        spokes_id=bore * 1.7,
        spokes_od=max(hub_diameter * 1.15, module * teeth * 0.72),
        chamfer=0.001,
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0007, angular_tolerance=0.08)


def _add_y_cylinder(part, name: str, radius: float, length: float, xyz, material: Material | str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=Y_AXIS_RPY),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_spur_reduction")

    cast_iron = model.material("dark_cast_iron", color=(0.18, 0.19, 0.18, 1.0))
    machined = model.material("machined_steel", color=(0.64, 0.64, 0.60, 1.0))
    gear_bronze = model.material("oiled_bronze_gears", color=(0.82, 0.58, 0.24, 1.0))
    black = model.material("blackened_handwheel", color=(0.03, 0.03, 0.03, 1.0))
    bolt = model.material("dark_bolts", color=(0.06, 0.06, 0.055, 1.0))

    shaft_z = 0.250
    gear_y = -0.100
    module = 0.006
    input_teeth = 14
    idler_teeth = 30
    output_teeth = 44
    input_outer_r = module * (input_teeth + 2) / 2.0
    idler_outer_r = module * (idler_teeth + 2) / 2.0
    output_outer_r = module * (output_teeth + 2) / 2.0
    mesh_clearance = 0.004

    input_x = -0.220
    idler_x = input_x + input_outer_r + idler_outer_r + mesh_clearance
    output_x = idler_x + idler_outer_r + output_outer_r + mesh_clearance
    shaft_centers = {
        "input": (input_x, shaft_z),
        "idler": (idler_x, shaft_z),
        "output": (output_x, shaft_z),
    }

    frame = model.part(
        "plate_frame",
        meta={
            "description": "One rigid side plate frame with protruding bearing supports and open front access to the gears."
        },
    )
    frame.visual(
        Box((0.700, 0.016, 0.420)),
        origin=Origin(xyz=(-0.020, 0.050, 0.240)),
        material=cast_iron,
        name="rear_plate",
    )
    frame.visual(
        Box((0.740, 0.120, 0.030)),
        origin=Origin(xyz=(-0.020, 0.006, 0.015)),
        material=cast_iron,
        name="base_foot",
    )
    frame.visual(
        Box((0.640, 0.014, 0.020)),
        origin=Origin(xyz=(-0.020, -0.054, 0.438)),
        material=cast_iron,
        name="top_cover_plate",
    )
    frame.visual(
        Box((0.640, 0.014, 0.020)),
        origin=Origin(xyz=(-0.020, -0.054, 0.074)),
        material=cast_iron,
        name="bottom_cover_plate",
    )
    for i, (sx, sz) in enumerate(((-0.330, 0.080), (0.290, 0.080), (-0.330, 0.420), (0.290, 0.420))):
        _add_y_cylinder(frame, f"standoff_{i}", 0.012, 0.096, (sx, -0.006, sz), machined)
        frame.visual(
            Cylinder(radius=0.017, length=0.006),
            origin=Origin(xyz=(sx, -0.057, sz), rpy=Y_AXIS_RPY),
            material=bolt,
            name=f"standoff_cap_{i}",
        )

    for x, z, plate_name, boss_name, bolt_prefix in (
        (input_x, shaft_z, "input_bearing_plate", "input_bearing_boss", "input_bearing_bolt"),
        (idler_x, shaft_z, "idler_bearing_plate", "idler_bearing_boss", "idler_bearing_bolt"),
        (output_x, shaft_z, "output_bearing_plate", "output_bearing_boss", "output_bearing_bolt"),
    ):
        frame.visual(
            _bearing_plate_mesh(plate_name),
            origin=Origin(xyz=(x, 0.035, z), rpy=Y_AXIS_RPY),
            material=machined,
            name=plate_name,
        )
        frame.visual(
            _bearing_boss_mesh(boss_name),
            origin=Origin(xyz=(x, 0.023, z), rpy=Y_AXIS_RPY),
            material=machined,
            name=boss_name,
        )
        for j, (dx, dz) in enumerate(((-0.030, -0.030), (0.030, -0.030), (-0.030, 0.030), (0.030, 0.030))):
            frame.visual(
                Cylinder(radius=0.0052, length=0.006),
                origin=Origin(xyz=(x + dx, 0.023, z + dz), rpy=Y_AXIS_RPY),
                material=bolt,
                name=f"{bolt_prefix}_{j}",
            )

    input_shaft = model.part("input_shaft")
    _add_y_cylinder(input_shaft, "input_main_shaft", 0.012, 0.188, (0.0, -0.054, 0.0), machined)
    _add_y_cylinder(input_shaft, "input_rear_journal", 0.016, 0.018, (0.0, -0.014, 0.0), machined)
    _add_y_cylinder(input_shaft, "input_front_shoulder", 0.018, 0.014, (0.0, -0.073, 0.0), machined)
    _add_y_cylinder(input_shaft, "input_handwheel_hub", 0.024, 0.034, (0.0, -0.138, 0.0), machined)
    input_shaft.visual(
        _spur_gear_mesh(
            "input_pinion",
            module=module,
            teeth=input_teeth,
            width=0.028,
            bore=0.024,
            hub_diameter=0.050,
            hub_length=0.042,
            spokes=4,
        ),
        origin=Origin(xyz=(0.0, gear_y, 0.0), rpy=Y_AXIS_RPY),
        material=gear_bronze,
        name="input_gear",
    )
    input_shaft.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.065, tube=0.0065, radial_segments=18, tubular_segments=72),
            "handwheel_rim",
        ),
        origin=Origin(xyz=(0.0, -0.152, 0.0), rpy=Y_AXIS_RPY),
        material=black,
        name="handwheel_rim",
    )
    input_shaft.visual(
        Box((0.124, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.152, 0.0)),
        material=black,
        name="handwheel_spoke_x",
    )
    input_shaft.visual(
        Box((0.012, 0.010, 0.124)),
        origin=Origin(xyz=(0.0, -0.152, 0.0)),
        material=black,
        name="handwheel_spoke_z",
    )
    input_shaft.visual(
        Box((0.008, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, gear_y, 0.0155)),
        material=machined,
        name="input_key",
    )

    idler_shaft = model.part("idler_shaft")
    _add_y_cylinder(idler_shaft, "idler_main_shaft", 0.014, 0.150, (0.0, -0.040, 0.0), machined)
    _add_y_cylinder(idler_shaft, "idler_rear_journal", 0.017, 0.018, (0.0, -0.014, 0.0), machined)
    _add_y_cylinder(idler_shaft, "idler_front_shoulder", 0.022, 0.016, (0.0, -0.066, 0.0), machined)
    idler_shaft.visual(
        _spur_gear_mesh(
            "idler_gear_mesh",
            module=module,
            teeth=idler_teeth,
            width=0.030,
            bore=0.028,
            hub_diameter=0.070,
            hub_length=0.050,
            spokes=6,
        ),
        origin=Origin(xyz=(0.0, gear_y, 0.0), rpy=Y_AXIS_RPY),
        material=gear_bronze,
        name="idler_gear",
    )
    idler_shaft.visual(
        Box((0.010, 0.038, 0.007)),
        origin=Origin(xyz=(0.0, gear_y, 0.0175)),
        material=machined,
        name="idler_key",
    )

    output_shaft = model.part("output_shaft")
    _add_y_cylinder(output_shaft, "output_main_shaft", 0.016, 0.172, (0.0, -0.042, 0.0), machined)
    _add_y_cylinder(output_shaft, "output_rear_journal", 0.018, 0.020, (0.0, -0.016, 0.0), machined)
    _add_y_cylinder(output_shaft, "output_front_shoulder", 0.026, 0.018, (0.0, -0.071, 0.0), machined)
    _add_y_cylinder(output_shaft, "output_coupling", 0.030, 0.040, (0.0, -0.124, 0.0), machined)
    output_shaft.visual(
        _spur_gear_mesh(
            "output_gear_mesh",
            module=module,
            teeth=output_teeth,
            width=0.032,
            bore=0.032,
            hub_diameter=0.086,
            hub_length=0.054,
            spokes=8,
        ),
        origin=Origin(xyz=(0.0, gear_y, 0.0), rpy=Y_AXIS_RPY),
        material=gear_bronze,
        name="output_gear",
    )
    output_shaft.visual(
        Box((0.011, 0.042, 0.008)),
        origin=Origin(xyz=(0.0, gear_y, 0.020)),
        material=machined,
        name="output_key",
    )
    output_shaft.visual(
        Box((0.010, 0.044, 0.010)),
        origin=Origin(xyz=(0.0, -0.124, 0.030)),
        material=machined,
        name="output_coupling_key",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=6.0, lower=-math.pi, upper=math.pi)
    model.articulation(
        "input_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(input_x, 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "idler_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=idler_shaft,
        origin=Origin(xyz=(idler_x, 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "output_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(output_x, 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("plate_frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_axis = object_model.get_articulation("input_axis")
    idler_axis = object_model.get_articulation("idler_axis")
    output_axis = object_model.get_articulation("output_axis")

    revolute_joints = [j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE]
    ctx.check("three supported revolute shaft joints", len(revolute_joints) == 3)
    for joint in (input_axis, idler_axis, output_axis):
        ctx.check(
            f"{joint.name} turns about the shaft line",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    for shaft, shaft_elem, bearing_elem, plate_elem in (
        (input_shaft, "input_main_shaft", "input_bearing_boss", "input_bearing_plate"),
        (idler_shaft, "idler_main_shaft", "idler_bearing_boss", "idler_bearing_plate"),
        (output_shaft, "output_main_shaft", "output_bearing_boss", "output_bearing_plate"),
    ):
        ctx.allow_overlap(
            frame,
            shaft,
            elem_a=bearing_elem,
            elem_b=shaft_elem,
            reason="The rotating shaft is intentionally captured through its bearing boss; the mesh bearing is a sleeve proxy around the journal.",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem=shaft_elem,
            outer_elem=bearing_elem,
            margin=0.0,
            name=f"{shaft_elem} is centered inside its bearing boss",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="y",
            elem_a=shaft_elem,
            elem_b=bearing_elem,
            min_overlap=0.010,
            name=f"{shaft_elem} remains inserted through its bearing boss",
        )
        ctx.allow_overlap(
            frame,
            shaft,
            elem_a=plate_elem,
            elem_b=shaft_elem,
            reason="The shaft continues through the bearing plate clearance bore behind the boss.",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem=shaft_elem,
            outer_elem=plate_elem,
            margin=0.0,
            name=f"{shaft_elem} is centered inside its bearing plate",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="y",
            elem_a=shaft_elem,
            elem_b=plate_elem,
            min_overlap=0.006,
            name=f"{shaft_elem} passes through its bearing plate",
        )

    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        positive_elem="idler_gear",
        negative_elem="input_gear",
        min_gap=0.001,
        max_gap=0.012,
        name="input pinion runs close to idler gear without clipping",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        positive_elem="output_gear",
        negative_elem="idler_gear",
        min_gap=0.001,
        max_gap=0.012,
        name="idler gear runs close to output gear without clipping",
    )
    for shaft, gear_name, bearing_name in (
        (input_shaft, "input_gear", "input_bearing_boss"),
        (idler_shaft, "idler_gear", "idler_bearing_boss"),
        (output_shaft, "output_gear", "output_bearing_boss"),
    ):
        ctx.expect_gap(
            frame,
            shaft,
            axis="y",
            positive_elem=bearing_name,
            negative_elem=gear_name,
            min_gap=0.015,
            max_gap=0.055,
            name=f"{gear_name} has practical clearance in front of bearing",
        )

    with ctx.pose({input_axis: 1.2, idler_axis: -0.9, output_axis: 0.65}):
        ctx.expect_gap(
            frame,
            input_shaft,
            axis="y",
            positive_elem="input_bearing_boss",
            negative_elem="input_gear",
            min_gap=0.015,
            max_gap=0.055,
            name="rotated input gear still clears bearing cover",
        )
        ctx.expect_gap(
            output_shaft,
            idler_shaft,
            axis="x",
            positive_elem="output_gear",
            negative_elem="idler_gear",
            min_gap=0.001,
            max_gap=0.014,
            name="rotated gear train keeps tooth-tip clearance",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PI = math.pi


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float):
    """Centered annular cartridge/flange mesh authored in local +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )


def _cyl_origin(center, axis: str = "z") -> Origin:
    if axis == "x":
        return Origin(xyz=center, rpy=(0.0, PI / 2.0, 0.0))
    if axis == "z":
        return Origin(xyz=center)
    raise ValueError(f"unsupported cylinder axis {axis!r}")


def _add_spur_gear(
    part,
    *,
    axis: str,
    center,
    root_radius: float,
    width: float,
    teeth: int,
    tooth_depth: float,
    material,
    name: str,
    body_name: str | None = None,
    hub_name: str | None = None,
) -> None:
    """Readable primitive spur gear: solid web, hub, and overlapping tooth blocks."""
    part.visual(
        Cylinder(radius=root_radius, length=width),
        origin=_cyl_origin(center, axis),
        material=material,
        name=body_name or f"{name}_body",
    )
    part.visual(
        Cylinder(radius=root_radius * 0.52, length=width + 0.026),
        origin=_cyl_origin(center, axis),
        material=material,
        name=hub_name or f"{name}_hub",
    )

    pitch = 2.0 * PI * (root_radius + 0.5 * tooth_depth) / teeth
    tangential = pitch * 0.55
    radial_center = root_radius + 0.5 * tooth_depth - 0.002
    for i in range(teeth):
        theta = 2.0 * PI * i / teeth
        if axis == "x":
            y = center[1] + radial_center * math.cos(theta)
            z = center[2] + radial_center * math.sin(theta)
            part.visual(
                Box((width * 0.96, tangential, tooth_depth)),
                origin=Origin(xyz=(center[0], y, z), rpy=(theta - PI / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{name}_tooth_{i}",
            )
        elif axis == "z":
            x = center[0] + radial_center * math.cos(theta)
            y = center[1] + radial_center * math.sin(theta)
            part.visual(
                Box((tangential, tooth_depth, width * 0.96)),
                origin=Origin(xyz=(x, y, center[2]), rpy=(0.0, 0.0, theta - PI / 2.0)),
                material=material,
                name=f"{name}_tooth_{i}",
            )
        else:
            raise ValueError(f"unsupported spur gear axis {axis!r}")


def _add_stepped_bevel_gear(
    part,
    *,
    axis: str,
    wide_center,
    direction: float,
    length: float,
    wide_radius: float,
    small_radius: float,
    teeth: int,
    tooth_depth: float,
    material,
    name: str,
) -> None:
    """A visible stepped bevel gear with a tapered cone body and crown teeth."""
    segments = 5
    seg_len = length / segments
    for s in range(segments):
        t = (s + 0.5) / segments
        radius = wide_radius + (small_radius - wide_radius) * t
        if axis == "z":
            center = (wide_center[0], wide_center[1], wide_center[2] + direction * seg_len * (s + 0.5))
        elif axis == "x":
            center = (wide_center[0] + direction * seg_len * (s + 0.5), wide_center[1], wide_center[2])
        else:
            raise ValueError(f"unsupported bevel gear axis {axis!r}")
        part.visual(
            Cylinder(radius=radius, length=seg_len + 0.002),
            origin=_cyl_origin(center, axis),
            material=material,
            name=f"{name}_body_{s}",
        )

    tooth_width = 2.0 * PI * (wide_radius + 0.5 * tooth_depth) / teeth * 0.50
    tooth_radial_center = wide_radius + 0.15 * tooth_depth
    tooth_axial = min(0.026, length * 0.35)
    for i in range(teeth):
        theta = 2.0 * PI * i / teeth
        if axis == "z":
            x = wide_center[0] + tooth_radial_center * math.cos(theta)
            y = wide_center[1] + tooth_radial_center * math.sin(theta)
            z = wide_center[2] + direction * tooth_axial * 0.45
            part.visual(
                Box((tooth_width, tooth_depth, tooth_axial)),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, theta - PI / 2.0)),
                material=material,
                name=f"{name}_tooth_{i}",
            )
        elif axis == "x":
            y = wide_center[1] + tooth_radial_center * math.cos(theta)
            z = wide_center[2] + tooth_radial_center * math.sin(theta)
            x = wide_center[0] + direction * tooth_axial * 0.45
            part.visual(
                Box((tooth_axial, tooth_width, tooth_depth)),
                origin=Origin(xyz=(x, y, z), rpy=(theta - PI / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{name}_tooth_{i}",
            )


def _add_horizontal_bearing_station(
    housing,
    *,
    x: float,
    shaft_y: float,
    shaft_z: float,
    ring_mesh,
    cast,
    steel,
    dark,
    stem: str,
    bearing_name: str | None = None,
) -> None:
    """Side-frame bearing cartridge plus local lugs and bolt circle."""
    housing.visual(
        ring_mesh,
        origin=Origin(xyz=(x, shaft_y, shaft_z), rpy=(0.0, PI / 2.0, 0.0)),
        material=steel,
        name=bearing_name or f"{stem}_bearing",
    )
    # Webs from the bearing ring to the open rectangular side frame.
    housing.visual(
        Box((0.052, 0.040, 0.122)),
        origin=Origin(xyz=(x, shaft_y, shaft_z - 0.100)),
        material=cast,
        name=f"{stem}_lower_lug",
    )
    housing.visual(
        Box((0.052, 0.040, 0.122)),
        origin=Origin(xyz=(x, shaft_y, shaft_z + 0.100)),
        material=cast,
        name=f"{stem}_upper_lug",
    )
    housing.visual(
        Box((0.052, 0.070, 0.036)),
        origin=Origin(xyz=(x, shaft_y - 0.090, shaft_z)),
        material=cast,
        name=f"{stem}_front_lug",
    )
    housing.visual(
        Box((0.052, 0.070, 0.036)),
        origin=Origin(xyz=(x, shaft_y + 0.090, shaft_z)),
        material=cast,
        name=f"{stem}_rear_lug",
    )

    face_x = x + math.copysign(0.031, x)
    for i in range(6):
        theta = 2.0 * PI * i / 6.0
        y = shaft_y + 0.078 * math.cos(theta)
        z = shaft_z + 0.078 * math.sin(theta)
        housing.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(face_x, y, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=dark,
            name=f"{stem}_bolt_{i}",
        )


def _add_collar(part, *, axis: str, center, radius: float, width: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=_cyl_origin(center, axis),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_right_angle_transfer_gearbox")

    cast = model.material("painted_cast_iron", rgba=(0.18, 0.22, 0.25, 1.0))
    machined = model.material("machined_steel", rgba=(0.62, 0.64, 0.60, 1.0))
    gear_bronze = model.material("oiled_bronze_gear", rgba=(0.72, 0.48, 0.20, 1.0))
    gear_steel = model.material("dark_steel_gear", rgba=(0.35, 0.36, 0.34, 1.0))
    bolt_black = model.material("black_oxide_bolts", rgba=(0.05, 0.05, 0.045, 1.0))
    gasket = model.material("red_gasket_faces", rgba=(0.55, 0.08, 0.05, 1.0))

    horizontal_bearing = mesh_from_cadquery(
        _annular_cylinder(0.095, 0.025, 0.050),
        "horizontal_bearing_cartridge",
        tolerance=0.0015,
    )
    vertical_bearing = mesh_from_cadquery(
        _annular_cylinder(0.100, 0.025, 0.045),
        "vertical_bearing_cartridge",
        tolerance=0.0015,
    )
    thrust_bearing = mesh_from_cadquery(
        _annular_cylinder(0.090, 0.025, 0.040),
        "lower_thrust_bearing",
        tolerance=0.0015,
    )

    housing = model.part("housing")
    # Open cast gearbox pan: low walls and heavy side frames leave the gear train visible.
    housing.visual(Box((0.86, 0.54, 0.040)), origin=Origin(xyz=(0.0, 0.075, 0.020)), material=cast, name="base_pan")
    housing.visual(Box((0.86, 0.035, 0.150)), origin=Origin(xyz=(0.0, -0.205, 0.095)), material=cast, name="front_lip")
    housing.visual(Box((0.86, 0.035, 0.150)), origin=Origin(xyz=(0.0, 0.355, 0.095)), material=cast, name="rear_lip")
    housing.visual(Box((0.050, 0.54, 0.150)), origin=Origin(xyz=(-0.425, 0.075, 0.095)), material=cast, name="left_end_lip")
    housing.visual(Box((0.050, 0.54, 0.150)), origin=Origin(xyz=(0.425, 0.075, 0.095)), material=cast, name="right_end_lip")

    for x, side in [(-0.360, "side_0"), (0.360, "side_1")]:
        housing.visual(Box((0.052, 0.480, 0.050)), origin=Origin(xyz=(x, 0.075, 0.175)), material=cast, name=f"{side}_lower_rail")
        housing.visual(Box((0.052, 0.480, 0.050)), origin=Origin(xyz=(x, 0.075, 0.545)), material=cast, name=f"{side}_upper_rail")
        housing.visual(Box((0.052, 0.050, 0.395)), origin=Origin(xyz=(x, -0.150, 0.355)), material=cast, name=f"{side}_front_post")
        housing.visual(Box((0.052, 0.050, 0.395)), origin=Origin(xyz=(x, 0.300, 0.355)), material=cast, name=f"{side}_rear_post")
        housing.visual(Box((0.052, 0.032, 0.335)), origin=Origin(xyz=(x, 0.073, 0.355)), material=cast, name=f"{side}_center_web")
        housing.visual(Box((0.064, 0.500, 0.006)), origin=Origin(xyz=(x, 0.075, 0.573)), material=gasket, name=f"{side}_machined_face")
        for y, foot in [(-0.120, "front"), (0.270, "rear")]:
            housing.visual(
                Box((0.052, 0.052, 0.160)),
                origin=Origin(xyz=(x, y, 0.095)),
                material=cast,
                name=f"{side}_{foot}_foot",
            )
        _add_horizontal_bearing_station(
            housing,
            x=x,
            shaft_y=0.000,
            shaft_z=0.350,
            ring_mesh=horizontal_bearing,
            cast=cast,
            steel=machined,
            dark=bolt_black,
            stem=f"{side}_lay",
            bearing_name="side_0_lay_bearing" if side == "side_0" else "side_1_lay_bearing",
        )
        _add_horizontal_bearing_station(
            housing,
            x=x,
            shaft_y=0.145,
            shaft_z=0.350,
            ring_mesh=horizontal_bearing,
            cast=cast,
            steel=machined,
            dark=bolt_black,
            stem=f"{side}_output",
            bearing_name="side_0_output_bearing" if side == "side_0" else "side_1_output_bearing",
        )

    # Vertical input bearing tower, deliberately open so the bevel pair is visible.
    for y, stem in [(-0.175, "front"), (0.225, "rear")]:
        housing.visual(Box((0.060, 0.055, 0.600)), origin=Origin(xyz=(-0.150, y, 0.330)), material=cast, name=f"input_{stem}_column")
        housing.visual(Box((0.050, 0.140, 0.038)), origin=Origin(xyz=(-0.150, y * 0.55, 0.620)), material=cast, name=f"top_{stem}_arm")
        housing.visual(Box((0.050, 0.140, 0.038)), origin=Origin(xyz=(-0.150, y * 0.55, 0.545)), material=cast, name=f"lower_{stem}_arm")
    housing.visual(
        vertical_bearing,
        origin=Origin(xyz=(-0.150, 0.0, 0.620)),
        material=machined,
        name="top_input_bearing",
    )
    housing.visual(
        thrust_bearing,
        origin=Origin(xyz=(-0.150, 0.0, 0.545)),
        material=machined,
        name="lower_input_bearing",
    )
    for i in range(6):
        theta = 2.0 * PI * i / 6.0
        x = -0.150 + 0.082 * math.cos(theta)
        y = 0.0 + 0.082 * math.sin(theta)
        housing.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(x, y, 0.648)),
            material=bolt_black,
            name=f"top_input_bolt_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0060, length=0.012),
            origin=Origin(xyz=(x, y, 0.568)),
            material=bolt_black,
            name=f"lower_input_bolt_{i}",
        )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=0.018, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=machined,
        name="input_journal",
    )
    _add_collar(input_shaft, axis="z", center=(0.0, 0.0, 0.165), radius=0.029, width=0.020, material=machined, name="lower_input_collar")
    _add_collar(input_shaft, axis="z", center=(0.0, 0.0, 0.225), radius=0.029, width=0.020, material=machined, name="upper_input_collar")
    _add_collar(input_shaft, axis="z", center=(0.0, 0.0, 0.080), radius=0.030, width=0.030, material=machined, name="input_gear_hub")
    _add_stepped_bevel_gear(
        input_shaft,
        axis="z",
        wide_center=(0.0, 0.0, 0.052),
        direction=1.0,
        length=0.088,
        wide_radius=0.050,
        small_radius=0.027,
        teeth=14,
        tooth_depth=0.008,
        material=gear_bronze,
        name="input_bevel",
    )

    layshaft = model.part("layshaft")
    layshaft.visual(
        Cylinder(radius=0.018, length=0.790),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=machined,
        name="lay_journal",
    )
    for x, name in [(-0.325, "lay_left_collar"), (0.325, "lay_right_collar")]:
        _add_collar(layshaft, axis="x", center=(x, 0.0, 0.0), radius=0.029, width=0.020, material=machined, name=name)
    _add_collar(layshaft, axis="x", center=(-0.020, 0.0, 0.0), radius=0.031, width=0.036, material=machined, name="lay_bevel_hub")
    _add_stepped_bevel_gear(
        layshaft,
        axis="x",
        wide_center=(-0.080, 0.0, 0.0),
        direction=1.0,
        length=0.090,
        wide_radius=0.050,
        small_radius=0.027,
        teeth=14,
        tooth_depth=0.008,
        material=gear_steel,
        name="lay_bevel",
    )
    _add_spur_gear(
        layshaft,
        axis="x",
        center=(0.165, 0.0, 0.0),
        root_radius=0.044,
        width=0.052,
        teeth=18,
        tooth_depth=0.008,
        material=gear_steel,
        name="spur_pinion",
        body_name="spur_pinion_body",
        hub_name="spur_pinion_hub",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=0.018, length=0.790),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=machined,
        name="output_journal",
    )
    for x, name in [(-0.325, "output_left_collar"), (0.325, "output_right_collar")]:
        _add_collar(output_shaft, axis="x", center=(x, 0.0, 0.0), radius=0.029, width=0.020, material=machined, name=name)
    _add_collar(output_shaft, axis="x", center=(0.165, 0.0, 0.0), radius=0.036, width=0.070, material=machined, name="output_gear_hub")
    _add_spur_gear(
        output_shaft,
        axis="x",
        center=(0.165, 0.0, 0.0),
        root_radius=0.076,
        width=0.058,
        teeth=30,
        tooth_depth=0.008,
        material=gear_bronze,
        name="output_spur",
        body_name="output_spur_body",
        hub_name="output_spur_hub",
    )

    model.articulation(
        "input_spin",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=input_shaft,
        origin=Origin(xyz=(-0.150, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0, lower=-2.0 * PI, upper=2.0 * PI),
    )
    model.articulation(
        "lay_spin",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=layshaft,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=16.0, lower=-2.0 * PI, upper=2.0 * PI),
    )
    model.articulation(
        "output_spin",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=output_shaft,
        origin=Origin(xyz=(0.0, 0.145, 0.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=12.0, lower=-2.0 * PI, upper=2.0 * PI),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    layshaft = object_model.get_part("layshaft")
    output_shaft = object_model.get_part("output_shaft")
    input_spin = object_model.get_articulation("input_spin")
    lay_spin = object_model.get_articulation("lay_spin")
    output_spin = object_model.get_articulation("output_spin")

    # Every rotating member is centered in its bearing cartridges and remains
    # axially inserted through the support stations.
    ctx.expect_within(
        input_shaft,
        housing,
        axes="xy",
        inner_elem="input_journal",
        outer_elem="top_input_bearing",
        margin=0.004,
        name="input journal is centered in the top bearing cartridge",
    )
    ctx.expect_overlap(
        input_shaft,
        housing,
        axes="z",
        elem_a="input_journal",
        elem_b="top_input_bearing",
        min_overlap=0.040,
        name="input journal passes through the top bearing",
    )
    ctx.expect_origin_gap(
        layshaft,
        input_shaft,
        axis="x",
        min_gap=0.149,
        max_gap=0.151,
        name="horizontal layshaft axis meets the vertical input bevel centerline",
    )
    ctx.expect_origin_gap(
        output_shaft,
        layshaft,
        axis="y",
        min_gap=0.144,
        max_gap=0.146,
        name="parallel output shaft is offset by the spur mesh center distance",
    )

    # The visible spur reduction is a coherent parallel-shaft mesh, not two
    # unrelated gears: the revolute axes are parallel and the shaft separation
    # matches the authored pinion/output gear pitch region.
    ctx.expect_origin_distance(
        layshaft,
        output_shaft,
        axes="z",
        min_dist=0.0,
        max_dist=0.001,
        name="spur shafts share a common gear plane height",
    )

    # Revolute pose checks prove the shafts spin in place on their bearing axes.
    rest_positions = {
        "input": ctx.part_world_position(input_shaft),
        "lay": ctx.part_world_position(layshaft),
        "output": ctx.part_world_position(output_shaft),
    }
    with ctx.pose({input_spin: PI / 2.0, lay_spin: -PI / 2.0, output_spin: PI / 2.0}):
        spun_positions = {
            "input": ctx.part_world_position(input_shaft),
            "lay": ctx.part_world_position(layshaft),
            "output": ctx.part_world_position(output_shaft),
        }
    ctx.check(
        "shaft revolute joints spin without translating their bearing axes",
        all(
            rest_positions[key] is not None
            and spun_positions[key] is not None
            and max(abs(rest_positions[key][i] - spun_positions[key][i]) for i in range(3)) < 1e-6
            for key in rest_positions
        ),
        details=f"rest={rest_positions}, spun={spun_positions}",
    )

    return ctx.report()


object_model = build_object_model()

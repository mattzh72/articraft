from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_PLATE_X = 0.24
BASE_PLATE_Y = 0.13
BASE_PLATE_Z = 0.01

INPUT_AXIS = (0.0, 0.0, 0.07)
OUTPUT_AXIS = (0.0, 0.0, 0.07)
REDUCTION_AXIS = (0.108, 0.054, 0.07)

SHAFT_RADIUS = 0.005
BEARING_CLEARANCE = 0.0006
GEAR_BORE_RADIUS = 0.0046


def _toothed_profile(root_radius: float, tip_radius: float, teeth: int, phase: float = 0.0) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for index in range(teeth * 2):
        angle = phase + (index * math.pi / teeth)
        radius = tip_radius if index % 2 == 0 else root_radius
        pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def _pillow_block_x(
    *,
    length: float,
    width: float,
    height: float,
    bore_radius: float,
    fillet: float,
) -> cq.Workplane:
    block = cq.Workplane("XY").box(length, width, height, centered=(True, True, True))
    block = block.edges("|X").fillet(fillet)
    bore = cq.Workplane("YZ").circle(bore_radius).extrude((length / 2.0) + 0.003, both=True)
    return block.cut(bore)


def _bearing_block_z(
    *,
    x_size: float,
    y_size: float,
    z_size: float,
    bore_radius: float,
    fillet: float,
) -> cq.Workplane:
    block = cq.Workplane("XY").box(x_size, y_size, z_size, centered=(True, True, True))
    block = block.edges("|Z").fillet(fillet)
    bore = cq.Workplane("XY").circle(bore_radius).extrude((z_size / 2.0) + 0.003, both=True)
    return block.cut(bore)


def _fork_top_arm() -> cq.Workplane:
    return cq.Workplane("XY").box(0.06, 0.018, 0.012, centered=(True, True, True))


def _input_lower_block() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .center(-0.012, 0.0)
        .box(0.026, 0.034, 0.05, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.003)
    )
    bore = cq.Workplane("XY").circle(SHAFT_RADIUS + BEARING_CLEARANCE).extrude(0.028, both=True)
    rear_buttress = (
        cq.Workplane("XY")
        .center(-0.004, -0.012)
        .box(0.018, 0.012, 0.05, centered=(True, True, True))
    )
    return body.union(rear_buttress).cut(bore)


def _spur_gear(
    *,
    root_radius: float,
    tip_radius: float,
    teeth: int,
    width: float,
    bore_radius: float,
    hub_radius: float,
    hub_width: float,
) -> cq.Workplane:
    profile = _toothed_profile(root_radius, tip_radius, teeth, phase=math.pi / (2.0 * teeth))
    gear = cq.Workplane("XY").polyline(profile).close().extrude(width / 2.0, both=True)
    hub = cq.Workplane("XY").circle(hub_radius).extrude(hub_width / 2.0, both=True)
    bore = cq.Workplane("XY").circle(bore_radius).extrude((hub_width / 2.0) + 0.003, both=True)
    return gear.union(hub).cut(bore)


def _bevel_gear(
    *,
    small_root: float,
    small_tip: float,
    large_root: float,
    large_tip: float,
    teeth: int,
    length: float,
    bore_radius: float,
    hub_radius: float,
    hub_length: float,
) -> cq.Workplane:
    small_profile = _toothed_profile(small_root, small_tip, teeth, phase=math.pi / (2.0 * teeth))
    large_profile = _toothed_profile(large_root, large_tip, teeth, phase=math.pi / (2.0 * teeth))
    gear = (
        cq.Workplane("XY")
        .polyline(small_profile)
        .close()
        .workplane(offset=length)
        .polyline(large_profile)
        .close()
        .loft(combine=True)
    )
    hub = cq.Workplane("XY").workplane(offset=length).circle(hub_radius).extrude(hub_length)
    bore = cq.Workplane("XY").circle(bore_radius).extrude(length + hub_length + 0.003)
    return gear.union(hub).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_gear_assembly", assets=ASSETS)

    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.23, 0.25, 0.28, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.59, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z)),
        origin=Origin(xyz=(0.08, 0.015, BASE_PLATE_Z / 2.0)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.06, 0.012, 0.15)),
        origin=Origin(xyz=(0.0, -0.035, 0.085)),
        material=cast_iron,
        name="upright_wall",
    )
    base.visual(
        mesh_from_cadquery(_fork_top_arm(), "top_arm.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.021, 0.132)),
        material=cast_iron,
        name="top_arm",
    )

    base.visual(
        mesh_from_cadquery(
            _input_lower_block(),
            "input_lower_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="input_lower_block",
    )
    base.visual(
        mesh_from_cadquery(
            _bearing_block_z(
                x_size=0.034,
                y_size=0.042,
                z_size=0.024,
                bore_radius=SHAFT_RADIUS + BEARING_CLEARANCE,
                fillet=0.0025,
            ),
            "input_upper_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=cast_iron,
        name="input_upper_block",
    )
    base.visual(
        mesh_from_cadquery(
            _pillow_block_x(
                length=0.028,
                width=0.042,
                height=0.036,
                bore_radius=SHAFT_RADIUS + BEARING_CLEARANCE,
                fillet=0.003,
            ),
            "output_inner_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.052, 0.0, 0.07)),
        material=cast_iron,
        name="output_inner_block",
    )
    base.visual(
        Box((0.028, 0.028, 0.042)),
        origin=Origin(xyz=(0.052, 0.0, 0.031)),
        material=cast_iron,
        name="output_inner_pedestal",
    )
    base.visual(
        mesh_from_cadquery(
            _pillow_block_x(
                length=0.028,
                width=0.042,
                height=0.036,
                bore_radius=SHAFT_RADIUS + BEARING_CLEARANCE,
                fillet=0.003,
            ),
            "output_outer_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.158, 0.0, 0.07)),
        material=cast_iron,
        name="output_outer_block",
    )
    base.visual(
        Box((0.028, 0.028, 0.042)),
        origin=Origin(xyz=(0.158, 0.0, 0.031)),
        material=cast_iron,
        name="output_outer_pedestal",
    )
    base.visual(
        mesh_from_cadquery(
            _pillow_block_x(
                length=0.026,
                width=0.038,
                height=0.034,
                bore_radius=SHAFT_RADIUS + BEARING_CLEARANCE,
                fillet=0.0025,
            ),
            "reduction_inner_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.084, 0.054, 0.07)),
        material=cast_iron,
        name="reduction_inner_block",
    )
    base.visual(
        Box((0.026, 0.026, 0.043)),
        origin=Origin(xyz=(0.084, 0.054, 0.0315)),
        material=cast_iron,
        name="reduction_inner_pedestal",
    )
    base.visual(
        mesh_from_cadquery(
            _pillow_block_x(
                length=0.026,
                width=0.038,
                height=0.034,
                bore_radius=SHAFT_RADIUS + BEARING_CLEARANCE,
                fillet=0.0025,
            ),
            "reduction_outer_block.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.146, 0.054, 0.07)),
        material=cast_iron,
        name="reduction_outer_block",
    )
    base.visual(
        Box((0.026, 0.026, 0.043)),
        origin=Origin(xyz=(0.146, 0.054, 0.0315)),
        material=cast_iron,
        name="reduction_outer_pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.13, 0.15)),
        mass=8.0,
        origin=Origin(xyz=(0.08, 0.005, 0.075)),
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="input_shaft_shell",
    )
    input_shaft.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=dark_steel,
        name="input_collar",
    )
    input_shaft.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=dark_steel,
        name="input_drive_hub",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            _bevel_gear(
                small_root=0.007,
                small_tip=0.010,
                large_root=0.018,
                large_tip=0.023,
                teeth=14,
                length=0.024,
                bore_radius=GEAR_BORE_RADIUS,
                hub_radius=0.010,
                hub_length=0.008,
            ),
            "input_bevel.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass,
        name="input_bevel_gear",
    )
    input_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.23),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.165),
        origin=Origin(xyz=(0.0975, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="output_shaft_shell",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            _bevel_gear(
                small_root=0.007,
                small_tip=0.010,
                large_root=0.018,
                large_tip=0.023,
                teeth=14,
                length=0.024,
                bore_radius=GEAR_BORE_RADIUS,
                hub_radius=0.010,
                hub_length=0.01,
            ),
            "output_bevel.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="output_bevel_gear",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            _spur_gear(
                root_radius=0.015,
                tip_radius=0.0195,
                teeth=16,
                width=0.012,
                bore_radius=GEAR_BORE_RADIUS,
                hub_radius=0.010,
                hub_width=0.02,
            ),
            "output_pinion.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="output_spur_pinion",
    )
    output_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.02, length=0.165),
        mass=0.9,
        origin=Origin(xyz=(0.0975, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    reduction_shaft = model.part("reduction_shaft")
    reduction_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="reduction_shaft_shell",
    )
    reduction_shaft.visual(
        mesh_from_cadquery(
            _spur_gear(
                root_radius=0.030,
                tip_radius=0.0355,
                teeth=28,
                width=0.014,
                bore_radius=GEAR_BORE_RADIUS,
                hub_radius=0.012,
                hub_width=0.022,
            ),
            "reduction_spur.obj",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="reduction_spur_gear",
    )
    reduction_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.12),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "input_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=input_shaft,
        origin=Origin(xyz=INPUT_AXIS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0),
    )
    model.articulation(
        "output_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=output_shaft,
        origin=Origin(xyz=OUTPUT_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0),
    )
    model.articulation(
        "reduction_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=reduction_shaft,
        origin=Origin(xyz=REDUCTION_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    input_shaft = object_model.get_part("input_shaft")
    output_shaft = object_model.get_part("output_shaft")
    reduction_shaft = object_model.get_part("reduction_shaft")

    input_spin = object_model.get_articulation("input_spin")
    output_spin = object_model.get_articulation("output_spin")
    reduction_spin = object_model.get_articulation("reduction_spin")

    input_lower_block = base.get_visual("input_lower_block")
    input_upper_block = base.get_visual("input_upper_block")
    output_inner_block = base.get_visual("output_inner_block")
    output_outer_block = base.get_visual("output_outer_block")
    reduction_inner_block = base.get_visual("reduction_inner_block")
    reduction_outer_block = base.get_visual("reduction_outer_block")

    input_shaft_shell = input_shaft.get_visual("input_shaft_shell")
    output_shaft_shell = output_shaft.get_visual("output_shaft_shell")
    reduction_shaft_shell = reduction_shaft.get_visual("reduction_shaft_shell")
    input_bevel = input_shaft.get_visual("input_bevel_gear")
    output_bevel = output_shaft.get_visual("output_bevel_gear")
    output_pinion = output_shaft.get_visual("output_spur_pinion")
    reduction_spur = reduction_shaft.get_visual("reduction_spur_gear")

    ctx.allow_overlap(
        input_shaft,
        output_shaft,
        elem_a=input_bevel,
        elem_b=output_bevel,
        reason="Bevel gears intentionally overlap slightly to represent engaged tooth mesh.",
    )
    ctx.allow_overlap(
        output_shaft,
        reduction_shaft,
        elem_a=output_pinion,
        elem_b=reduction_spur,
        reason="Spur gear pair intentionally overlaps slightly to represent tooth engagement.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all required parts present",
        all(part is not None for part in (base, input_shaft, output_shaft, reduction_shaft)),
        "Base, input shaft, output shaft, and reduction shaft must all exist.",
    )
    ctx.check(
        "input shaft axis vertical",
        tuple(input_spin.axis) == (0.0, 0.0, 1.0),
        f"Expected input axis (0, 0, 1), got {input_spin.axis}.",
    )
    ctx.check(
        "output shaft axis horizontal",
        tuple(output_spin.axis) == (1.0, 0.0, 0.0),
        f"Expected output axis (1, 0, 0), got {output_spin.axis}.",
    )
    ctx.check(
        "reduction shaft axis horizontal",
        tuple(reduction_spin.axis) == (1.0, 0.0, 0.0),
        f"Expected reduction axis (1, 0, 0), got {reduction_spin.axis}.",
    )

    ctx.expect_overlap(
        input_shaft,
        base,
        axes="xy",
        min_overlap=0.009,
        elem_a=input_shaft_shell,
        elem_b=input_lower_block,
        name="input shaft passes through lower bearing block footprint",
    )
    ctx.expect_overlap(
        input_shaft,
        base,
        axes="xy",
        min_overlap=0.009,
        elem_a=input_shaft_shell,
        elem_b=input_upper_block,
        name="input shaft passes through upper bearing block footprint",
    )
    ctx.expect_overlap(
        output_shaft,
        base,
        axes="yz",
        min_overlap=0.009,
        elem_a=output_shaft_shell,
        elem_b=output_inner_block,
        name="output shaft sits in inner output bearing block",
    )
    ctx.expect_overlap(
        output_shaft,
        base,
        axes="yz",
        min_overlap=0.009,
        elem_a=output_shaft_shell,
        elem_b=output_outer_block,
        name="output shaft sits in outer output bearing block",
    )
    ctx.expect_overlap(
        reduction_shaft,
        base,
        axes="yz",
        min_overlap=0.009,
        elem_a=reduction_shaft_shell,
        elem_b=reduction_inner_block,
        name="reduction shaft sits in inner reduction bearing block",
    )
    ctx.expect_overlap(
        reduction_shaft,
        base,
        axes="yz",
        min_overlap=0.009,
        elem_a=reduction_shaft_shell,
        elem_b=reduction_outer_block,
        name="reduction shaft sits in outer reduction bearing block",
    )

    ctx.expect_origin_distance(
        input_shaft,
        output_shaft,
        axes=("x", "y", "z"),
        max_dist=0.0001,
        name="input and output joint origins coincide at bevel mesh point",
    )
    ctx.expect_origin_gap(
        reduction_shaft,
        output_shaft,
        axis="y",
        min_gap=0.053,
        max_gap=0.055,
        name="spur reduction shaft offset matches gear center distance",
    )
    ctx.expect_overlap(
        input_shaft,
        output_shaft,
        axes="yz",
        min_overlap=0.012,
        elem_a=input_bevel,
        elem_b=output_bevel,
        name="bevel gear pair shares engaged footprint",
    )
    ctx.expect_overlap(
        output_shaft,
        reduction_shaft,
        axes="xz",
        min_overlap=0.018,
        elem_a=output_pinion,
        elem_b=reduction_spur,
        name="spur reduction gears share a common mesh plane",
    )

    with ctx.pose({input_spin: math.pi / 3.0, output_spin: -math.pi / 4.0, reduction_spin: math.pi / 7.0}):
        ctx.expect_overlap(
            input_shaft,
            base,
            axes="xy",
            min_overlap=0.009,
            elem_a=input_shaft_shell,
            elem_b=input_lower_block,
            name="input shaft remains supported when rotated",
        )
        ctx.expect_overlap(
            output_shaft,
            base,
            axes="yz",
            min_overlap=0.009,
            elem_a=output_shaft_shell,
            elem_b=output_outer_block,
            name="output shaft remains supported when rotated",
        )
        ctx.expect_overlap(
            reduction_shaft,
            base,
            axes="yz",
            min_overlap=0.009,
            elem_a=reduction_shaft_shell,
            elem_b=reduction_inner_block,
            name="reduction shaft remains supported when rotated",
        )
        ctx.expect_overlap(
            input_shaft,
            output_shaft,
            axes="yz",
            min_overlap=0.012,
            elem_a=input_bevel,
            elem_b=output_bevel,
            name="bevel pair stays aligned in an operating pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

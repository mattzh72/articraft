from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cylinder_origin_between(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux) if abs(math.sin(pitch)) > 1e-9 else 0.0
    origin = Origin(
        xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_tube(part, p0, p1, radius, *, material, name):
    origin, length = _cylinder_origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_dolly")

    blue = model.material("blue_powder_coat", rgba=(0.035, 0.18, 0.62, 1.0))
    dark_blue = model.material("dark_blue_shadow", rgba=(0.02, 0.08, 0.24, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    plate_mat = model.material("galvanized_nose_plate", rgba=(0.48, 0.50, 0.50, 1.0))
    tire_mat = model.material("matte_black_rubber", rgba=(0.012, 0.011, 0.010, 1.0))
    grip_mat = model.material("black_foam_grip", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")

    # Main load plate and scuffed anti-slip treads.
    frame.visual(
        Box((0.56, 0.46, 0.026)),
        origin=Origin(xyz=(0.06, 0.0, 0.140)),
        material=plate_mat,
        name="nose_plate",
    )
    frame.visual(
        Box((0.035, 0.47, 0.055)),
        origin=Origin(xyz=(0.342, 0.0, 0.162)),
        material=plate_mat,
        name="front_lip",
    )
    for i, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        frame.visual(
            Box((0.36, 0.016, 0.004)),
            origin=Origin(xyz=(0.075, y, 0.155)),
            material=dark_blue,
            name=f"plate_tread_{i}",
        )

    # Upright tubular frame.
    rail_bottom_z = 0.150
    rail_top_z = 1.230
    for i, y in enumerate((-0.225, 0.225)):
        _add_tube(
            frame,
            (-0.215, y, rail_bottom_z),
            (-0.345, y, rail_top_z),
            0.018,
            material=blue,
            name=f"upright_rail_{i}",
        )
        _add_tube(
            frame,
            (-0.345, y, rail_top_z),
            (-0.352, y, 1.270),
            0.018,
            material=blue,
            name=f"handle_upright_{i}",
        )
        _add_tube(
            frame,
            (-0.210, y, 0.150),
            (-0.070, y, 0.140),
            0.016,
            material=blue,
            name=f"plate_strut_{i}",
        )
        frame.visual(
            Box((0.115, 0.018, 0.018)),
            origin=Origin(xyz=(-0.115, y, 0.143)),
            material=blue,
            name=f"toe_weld_{i}",
        )

    for name, z in (("lower_crossbar", 0.300), ("middle_crossbar", 0.700), ("top_crossbar", 1.135)):
        t = (z - rail_bottom_z) / (rail_top_z - rail_bottom_z)
        x = -0.215 + (-0.345 + 0.215) * t
        _add_tube(frame, (x, -0.255, z), (x, 0.255, z), 0.015, material=blue, name=name)

    _add_tube(frame, (-0.352, -0.275, 1.270), (-0.352, 0.275, 1.270), 0.017, material=blue, name="handle_bar")
    for i, y in enumerate((-0.325, 0.325)):
        frame.visual(
            Cylinder(radius=0.024, length=0.115),
            origin=Origin(xyz=(-0.352, y, 1.270), rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
            material=grip_mat,
            name=f"hand_grip_{i}",
        )

    # Rear axle tube and under-plate hinge lugs that capture the folding caster bar.
    _add_tube(frame, (-0.220, -0.430, 0.178), (-0.220, 0.430, 0.178), 0.014, material=steel, name="rear_axle")
    for i, y in enumerate((-0.245, 0.245)):
        frame.visual(
            Box((0.075, 0.052, 0.055)),
            origin=Origin(xyz=(-0.100, y, 0.105)),
            material=blue,
            name=f"hinge_mount_{i}",
        )
        frame.visual(
            Cylinder(radius=0.029, length=0.018),
            origin=Origin(xyz=(-0.100, y * 1.10, 0.105), rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
            material=steel,
            name=f"hinge_boss_{i}",
        )

    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.170,
            0.075,
            inner_radius=0.124,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.008, radius=0.003),
        ),
        "rear_tire",
    )
    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.126,
            0.068,
            rim=WheelRim(inner_radius=0.085, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.034,
                width=0.050,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.046, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.007, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.040),
        ),
        "rear_rim",
    )

    rear_wheels = []
    for i, y in enumerate((-0.355, 0.355)):
        wheel = model.part(f"rear_wheel_{i}")
        wheel.visual(rear_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=tire_mat, name="tire")
        wheel.visual(rear_wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=steel, name="rim")
        wheel.visual(
            Cylinder(radius=0.0205, length=0.060),
            origin=Origin(rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
            material=steel,
            name="bearing_sleeve",
        )
        rear_wheels.append(wheel)
        model.articulation(
            f"rear_wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.220, y, 0.178)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    caster_bar = model.part("caster_bar")
    _add_tube(caster_bar, (0.000, -0.245, 0.000), (0.000, 0.245, 0.000), 0.018, material=blue, name="hinge_sleeve")
    _add_tube(caster_bar, (0.000, 0.000, 0.000), (0.430, 0.000, 0.000), 0.014, material=blue, name="center_spine")
    _add_tube(caster_bar, (0.430, -0.230, 0.000), (0.430, 0.230, 0.000), 0.014, material=blue, name="front_crossbar")
    _add_tube(caster_bar, (0.015, -0.200, 0.000), (0.430, -0.170, 0.000), 0.010, material=blue, name="diagonal_brace_0")
    _add_tube(caster_bar, (0.015, 0.200, 0.000), (0.430, 0.170, 0.000), 0.010, material=blue, name="diagonal_brace_1")
    for i, y in enumerate((-0.170, 0.170)):
        caster_bar.visual(
            Box((0.070, 0.070, 0.010)),
            origin=Origin(xyz=(0.430, y, -0.011)),
            material=steel,
            name=f"caster_pad_{i}",
        )

    model.articulation(
        "caster_bar_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=caster_bar,
        origin=Origin(xyz=(-0.100, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.035,
            0.026,
            inner_radius=0.024,
            carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.03),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.032,
            0.026,
            rim=WheelRim(inner_radius=0.021, flange_height=0.004, flange_thickness=0.002, bead_seat_depth=0.0015),
            hub=WheelHub(radius=0.013, width=0.020, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "caster_rim",
    )

    for i, y in enumerate((-0.170, 0.170)):
        stem = model.part(f"caster_stem_{i}")
        stem.visual(Cylinder(radius=0.008, length=0.050), origin=Origin(xyz=(0.0, 0.0, -0.032)), material=steel, name="vertical_stem")
        stem.visual(Cylinder(radius=0.032, length=0.014), origin=Origin(xyz=(0.0, 0.0, -0.007)), material=steel, name="swivel_bearing")
        stem.visual(Box((0.082, 0.070, 0.014)), origin=Origin(xyz=(0.040, 0.0, -0.014)), material=steel, name="fork_bridge")
        stem.visual(Box((0.058, 0.010, 0.070)), origin=Origin(xyz=(0.055, -0.026, -0.056)), material=steel, name="fork_cheek_0")
        stem.visual(Box((0.058, 0.010, 0.070)), origin=Origin(xyz=(0.055, 0.026, -0.056)), material=steel, name="fork_cheek_1")
        _add_tube(stem, (0.055, -0.040, -0.058), (0.055, 0.040, -0.058), 0.005, material=steel, name="caster_axle")

        model.articulation(
            f"caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=caster_bar,
            child=stem,
            origin=Origin(xyz=(0.430, y, -0.016)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(caster_tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=tire_mat, name="tire")
        wheel.visual(
            Cylinder(radius=0.025, length=0.028),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.0075, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
            material=steel,
            name="bearing_sleeve",
        )
        model.articulation(
            f"caster_wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=stem,
            child=wheel,
            origin=Origin(xyz=(0.055, 0.0, -0.058)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=22.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    caster_bar = object_model.get_part("caster_bar")
    hinge = object_model.get_articulation("caster_bar_hinge")

    for idx in (0, 1):
        rear = object_model.get_part(f"rear_wheel_{idx}")
        ctx.allow_overlap(
            frame,
            rear,
            elem_a="rear_axle",
            elem_b="bearing_sleeve",
            reason="The rear wheel bearing sleeve is intentionally captured around the fixed rear axle.",
        )
        ctx.expect_overlap(
            frame,
            rear,
            axes="xyz",
            elem_a="rear_axle",
            elem_b="bearing_sleeve",
            min_overlap=0.020,
            name=f"rear wheel {idx} is retained on the axle",
        )

        stem = object_model.get_part(f"caster_stem_{idx}")
        caster = object_model.get_part(f"caster_wheel_{idx}")
        ctx.allow_overlap(
            stem,
            caster,
            elem_a="caster_axle",
            elem_b="bearing_sleeve",
            reason="The small caster wheel bearing sleeve is intentionally captured around its fork axle.",
        )
        ctx.allow_overlap(
            stem,
            caster,
            elem_a="caster_axle",
            elem_b="rim",
            reason="The caster rim is a simplified hub proxy with the axle passing through its central bore.",
        )
        ctx.expect_overlap(
            stem,
            caster,
            axes="xyz",
            elem_a="caster_axle",
            elem_b="bearing_sleeve",
            min_overlap=0.009,
            name=f"front caster wheel {idx} is retained on its axle",
        )
        ctx.expect_overlap(
            stem,
            caster,
            axes="xyz",
            elem_a="caster_axle",
            elem_b="rim",
            min_overlap=0.009,
            name=f"front caster axle {idx} passes through the wheel hub",
        )

        ctx.allow_overlap(
            frame,
            caster_bar,
            elem_a=f"hinge_mount_{idx}",
            elem_b="hinge_sleeve",
            reason="The folding caster bar sleeve is clipped through the under-plate hinge mount proxy.",
        )
        ctx.expect_overlap(
            frame,
            caster_bar,
            axes="xyz",
            elem_a=f"hinge_mount_{idx}",
            elem_b="hinge_sleeve",
            min_overlap=0.020,
            name=f"caster bar hinge sleeve is captured by mount {idx}",
        )

        ctx.expect_gap(
            caster_bar,
            object_model.get_part(f"caster_stem_{idx}"),
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=f"caster_pad_{idx}",
            negative_elem="swivel_bearing",
            name=f"caster swivel {idx} seats under the caster pad",
        )

    with ctx.pose({hinge: 1.35}):
        folded_aabb = ctx.part_element_world_aabb(caster_bar, elem="front_crossbar")
    with ctx.pose({hinge: 0.0}):
        deployed_aabb = ctx.part_element_world_aabb(caster_bar, elem="front_crossbar")

    folded_z = folded_aabb[1][2] if folded_aabb is not None else None
    deployed_z = deployed_aabb[1][2] if deployed_aabb is not None else None
    ctx.check(
        "caster bar folds upward from deployed cart mode",
        folded_z is not None and deployed_z is not None and folded_z > deployed_z + 0.25,
        details=f"deployed_front_crossbar_max_z={deployed_z}, folded_front_crossbar_max_z={folded_z}",
    )

    return ctx.report()


object_model = build_object_model()

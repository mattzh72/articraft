from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SHAFT_Z = 0.260
BASE_TOP_Z = 0.040
BORE_Z = SHAFT_Z - BASE_TOP_Z


def _cyl_x(part, radius, length, center, material, name):
    """Attach a cylinder whose axis is the assembly X shaft axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _safe_fillet(shape, selector: str, radius: float):
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _safe_chamfer(shape, selector: str, distance: float):
    try:
        return shape.edges(selector).chamfer(distance)
    except Exception:
        return shape


def _pillow_block_body(name: str):
    """Machined pillow-block station with a true through-bore for the shaft."""
    foot_x, foot_y, foot_z = 0.240, 0.340, 0.025
    neck_x, neck_y, neck_z = 0.155, 0.175, 0.160
    cap_x, cap_y, cap_z = 0.145, 0.255, 0.180

    foot = cq.Workplane("XY").box(foot_x, foot_y, foot_z).translate((0.0, 0.0, foot_z / 2.0))
    neck = cq.Workplane("XY").box(neck_x, neck_y, neck_z).translate(
        (0.0, 0.0, foot_z + neck_z / 2.0)
    )
    cap = cq.Workplane("XY").box(cap_x, cap_y, cap_z).translate((0.0, 0.0, BORE_Z))

    # Wide side lugs make the foot read as a bolted pedestal rather than a floating block.
    lug_0 = cq.Workplane("XY").box(0.095, 0.045, 0.040).translate((0.060, 0.145, 0.045))
    lug_1 = cq.Workplane("XY").box(0.095, 0.045, 0.040).translate((-0.060, 0.145, 0.045))
    lug_2 = cq.Workplane("XY").box(0.095, 0.045, 0.040).translate((0.060, -0.145, 0.045))
    lug_3 = cq.Workplane("XY").box(0.095, 0.045, 0.040).translate((-0.060, -0.145, 0.045))

    body = foot.union(neck).union(cap).union(lug_0).union(lug_1).union(lug_2).union(lug_3)

    cutter = cq.Workplane("YZ").center(0.0, BORE_Z).circle(0.044).extrude(0.42, both=True)
    body = body.cut(cutter)
    body = _safe_fillet(body, "|Z", 0.006)
    body = _safe_chamfer(body, ">Z", 0.003)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.06)


def _end_housing_body(name: str):
    """End seal housing with a vertical bored plate and bolted base foot."""
    foot = cq.Workplane("XY").box(0.175, 0.320, 0.025).translate((0.0, 0.0, 0.0125))
    plate = cq.Workplane("XY").box(0.090, 0.230, 0.275).translate((0.0, 0.0, 0.025 + 0.1375))
    boss = cq.Workplane("YZ").center(0.0, BORE_Z).circle(0.073).extrude(0.025, both=True)
    rib_0 = cq.Workplane("XY").box(0.075, 0.026, 0.185).translate((0.0, 0.108, 0.115))
    rib_1 = cq.Workplane("XY").box(0.075, 0.026, 0.185).translate((0.0, -0.108, 0.115))
    body = foot.union(plate).union(boss).union(rib_0).union(rib_1)

    cutter = cq.Workplane("YZ").center(0.0, BORE_Z).circle(0.043).extrude(0.32, both=True)
    body = body.cut(cutter)
    body = _safe_fillet(body, "|Z", 0.005)
    body = _safe_chamfer(body, ">Z", 0.0025)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.06)


def _annular_cylinder(name: str, outer_radius: float, inner_radius: float, length: float):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(ring, name)


def _hex_nut(name: str, outer_radius: float, inner_radius: float, length: float):
    nut = cq.Workplane("XY").polygon(6, outer_radius * 2.0).extrude(length).translate(
        (0.0, 0.0, -length / 2.0)
    )
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length + 0.010).translate(
        (0.0, 0.0, -(length + 0.010) / 2.0)
    )
    nut = nut.cut(bore)
    nut = _safe_chamfer(nut, "|Z", 0.002)
    return mesh_from_cadquery(nut, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="continuous_rotary_shaft_study")

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    bed_blue = model.material("blued_fabricated_steel", rgba=(0.10, 0.13, 0.15, 1.0))
    machined = model.material("machined_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    polished = model.material("polished_journal_steel", rgba=(0.86, 0.87, 0.84, 1.0))
    black_oxide = model.material("black_oxide_fasteners", rgba=(0.015, 0.014, 0.012, 1.0))
    bronze = model.material("bronze_bearing_liner", rgba=(0.78, 0.52, 0.22, 1.0))
    gasket = model.material("dark_gasket", rgba=(0.025, 0.022, 0.020, 1.0))

    bearing_ring_mesh = _annular_cylinder("split_bearing_ring", 0.070, 0.036, 0.016)
    seal_cover_mesh = _annular_cylinder("seal_cover_ring", 0.064, 0.036, 0.018)
    locknut_mesh = _hex_nut("shaft_hex_locknut", 0.062, 0.032, 0.028)

    base = model.part("base_frame")
    base.visual(
        Box((1.960, 0.360, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bed_blue,
        name="bed_plate",
    )
    for y, name in ((0.190, "side_rail_0"), (-0.190, "side_rail_1")):
        base.visual(
            Box((1.980, 0.042, 0.062)),
            origin=Origin(xyz=(0.0, y, 0.071)),
            material=bed_blue,
            name=name,
        )
    for x, name in ((-0.72, "cross_tie_0"), (0.0, "cross_tie_1"), (0.72, "cross_tie_2")):
        base.visual(
            Box((0.090, 0.430, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=bed_blue,
            name=name,
        )
    for x in (-0.88, 0.88):
        for y in (-0.145, 0.145):
            base.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(x, y, 0.044)),
                material=black_oxide,
                name=f"anchor_bolt_{x}_{y}",
            )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.032, length=1.840),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="shaft_core",
    )
    station_xs = (-0.460, 0.0, 0.460)
    for idx, x in enumerate(station_xs):
        _cyl_x(shaft, 0.034, 0.135, (x, 0.0, 0.0), polished, f"journal_{idx}")
        for side, suffix in ((-1.0, "inboard"), (1.0, "outboard")):
            _cyl_x(
                shaft,
                0.052,
                0.028,
                (x + side * 0.128, 0.0, 0.0),
                machined,
                f"collar_{idx}_{suffix}",
            )
    for side, x in ((-1.0, -0.665), (1.0, 0.665)):
        shaft.visual(
            locknut_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_oxide,
            name=f"locknut_{int((side + 1) / 2)}",
        )
    for side, x in ((-1.0, -0.870), (1.0, 0.870)):
        _cyl_x(shaft, 0.090, 0.055, (x, 0.0, 0.0), machined, f"coupling_flange_{int((side + 1) / 2)}")
        _cyl_x(shaft, 0.025, 0.075, (x + side * 0.065, 0.0, 0.0), polished, f"pilot_stub_{int((side + 1) / 2)}")
        for bolt_idx in range(6):
            angle = bolt_idx * math.tau / 6.0
            y = 0.064 * math.cos(angle)
            z = 0.064 * math.sin(angle)
            _cyl_x(
                shaft,
                0.007,
                0.012,
                (x + side * 0.033, y, z),
                black_oxide,
                f"flange_{int((side + 1) / 2)}_bolt_{bolt_idx}",
            )
    shaft.visual(
        Box((0.155, 0.012, 0.008)),
        origin=Origin(xyz=(0.705, 0.0, 0.035)),
        material=black_oxide,
        name="drive_key",
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=45.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    stations = []
    for idx, x in enumerate(station_xs):
        station = model.part(f"station_{idx}")
        station.visual(
            _pillow_block_body(f"pillow_block_{idx}"),
            origin=Origin(),
            material=cast_iron,
            name="housing_body",
        )
        station.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(-0.0805, 0.0, BORE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name="bearing_face_0",
        )
        station.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(0.0805, 0.0, BORE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name="bearing_face_1",
        )
        station.visual(
            Box((0.120, 0.012, 0.040)),
            origin=Origin(xyz=(0.0, 0.038, BORE_Z)),
            material=bronze,
            name="bearing_shoe",
        )
        for bx in (-0.073, 0.073):
            for by in (-0.130, 0.130):
                station.visual(
                    Cylinder(radius=0.011, length=0.010),
                    origin=Origin(xyz=(bx, by, 0.030)),
                    material=black_oxide,
                    name=f"foot_bolt_{bx}_{by}",
                )
        for by, name in ((0.112, "torque_arm_0"), (-0.112, "torque_arm_1")):
            station.visual(
                Box((0.035, 0.028, 0.230)),
                origin=Origin(xyz=(0.0, by, 0.137), rpy=(0.0, 0.0, 0.0)),
                material=cast_iron,
                name=name,
            )
        model.articulation(
            f"base_to_station_{idx}",
            ArticulationType.FIXED,
            parent=base,
            child=station,
            origin=Origin(xyz=(x, 0.0, BASE_TOP_Z)),
        )
        stations.append(station)

        cover = model.part(f"cover_{idx}")
        cover.visual(
            Box((0.152, 0.245, 0.018)),
            origin=Origin(),
            material=cast_iron,
            name="cover_plate",
        )
        cover.visual(
            Box((0.118, 0.160, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.0115)),
            material=gasket,
            name="inspection_gasket",
        )
        for bx in (-0.050, 0.050):
            for by in (-0.085, 0.085):
                cover.visual(
                    Cylinder(radius=0.0085, length=0.010),
                    origin=Origin(xyz=(bx, by, 0.014)),
                    material=black_oxide,
                    name=f"cover_screw_{bx}_{by}",
                )
        model.articulation(
            f"station_{idx}_to_cover",
            ArticulationType.PRISMATIC,
            parent=station,
            child=cover,
            origin=Origin(xyz=(0.0, 0.0, 0.319)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=60.0, velocity=0.12, lower=0.0, upper=0.080),
        )

    for idx, x in enumerate((-0.745, 0.745)):
        end_housing = model.part(f"end_housing_{idx}")
        end_housing.visual(
            _end_housing_body(f"end_housing_body_{idx}"),
            origin=Origin(),
            material=cast_iron,
            name="housing_body",
        )
        for bx in (-0.055, 0.055):
            for by in (-0.122, 0.122):
                end_housing.visual(
                    Cylinder(radius=0.010, length=0.010),
                    origin=Origin(xyz=(bx, by, 0.030)),
                    material=black_oxide,
                    name=f"base_screw_{bx}_{by}",
                )
        model.articulation(
            f"base_to_end_housing_{idx}",
            ArticulationType.FIXED,
            parent=base,
            child=end_housing,
            origin=Origin(xyz=(x, 0.0, BASE_TOP_Z)),
        )

        side = -1.0 if idx == 0 else 1.0
        seal_cover = model.part(f"seal_cover_{idx}")
        seal_cover.visual(
            seal_cover_mesh,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name="seal_retainer",
        )
        seal_cover.visual(
            _annular_cylinder(f"seal_gasket_{idx}", 0.052, 0.036, 0.004),
            origin=Origin(xyz=(-side * 0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gasket,
            name="gasket_ring",
        )
        for bolt_idx in range(4):
            angle = math.pi / 4.0 + bolt_idx * math.pi / 2.0
            seal_cover.visual(
                Cylinder(radius=0.0055, length=0.006),
                origin=Origin(
                    xyz=(side * 0.012, 0.044 * math.cos(angle), 0.044 * math.sin(angle)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_oxide,
                name=f"seal_screw_{bolt_idx}",
            )
        model.articulation(
            f"end_housing_{idx}_to_seal_cover",
            ArticulationType.PRISMATIC,
            parent=end_housing,
            child=seal_cover,
            origin=Origin(xyz=(side * 0.054, 0.0, BORE_Z)),
            axis=(side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=0.08, lower=0.0, upper=0.060),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    base = object_model.get_part("base_frame")
    shaft_joint = object_model.get_articulation("base_to_shaft")

    ctx.check(
        "single continuous shaft joint",
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(shaft_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft_joint.articulation_type}, axis={shaft_joint.axis}",
    )
    ctx.expect_origin_gap(
        shaft,
        base,
        axis="z",
        min_gap=0.24,
        name="shaft centerline is elevated above fabricated bed",
    )

    for idx in range(3):
        station = object_model.get_part(f"station_{idx}")
        cover = object_model.get_part(f"cover_{idx}")
        lift = object_model.get_articulation(f"station_{idx}_to_cover")
        ctx.expect_gap(
            station,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="housing_body",
            negative_elem="bed_plate",
            name=f"station_{idx} foot is seated on bed",
        )
        ctx.expect_overlap(
            shaft,
            station,
            axes="x",
            min_overlap=0.100,
            elem_a=f"journal_{idx}",
            elem_b="housing_body",
            name=f"journal_{idx} passes through bearing station",
        )
        ctx.expect_within(
            shaft,
            station,
            axes="yz",
            margin=0.004,
            inner_elem=f"journal_{idx}",
            outer_elem="bearing_face_0",
            name=f"journal_{idx} is centered inside bearing ring",
        )
        ctx.expect_gap(
            cover,
            station,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cover_plate",
            negative_elem="housing_body",
            name=f"cover_{idx} is seated on access pad",
        )
        with ctx.pose({lift: 0.080}):
            ctx.expect_origin_gap(
                cover,
                station,
                axis="z",
                min_gap=0.075,
                name=f"cover_{idx} lifts clear for removal",
            )

    for idx in range(2):
        end_housing = object_model.get_part(f"end_housing_{idx}")
        seal_cover = object_model.get_part(f"seal_cover_{idx}")
        slide = object_model.get_articulation(f"end_housing_{idx}_to_seal_cover")
        ctx.expect_contact(
            seal_cover,
            end_housing,
            contact_tol=0.001,
            elem_a="seal_retainer",
            elem_b="housing_body",
            name=f"seal cover {idx} is seated on end housing",
        )
        ctx.expect_within(
            shaft,
            seal_cover,
            axes="yz",
            margin=0.006,
            inner_elem="shaft_core",
            outer_elem="seal_retainer",
            name=f"shaft runs through seal cover {idx}",
        )
        with ctx.pose({slide: 0.050}):
            if idx == 0:
                ctx.expect_origin_gap(
                    end_housing,
                    seal_cover,
                    axis="x",
                    min_gap=0.045,
                    name="seal cover 0 slides off the end housing",
                )
            else:
                ctx.expect_origin_gap(
                    seal_cover,
                    end_housing,
                    axis="x",
                    min_gap=0.045,
                    name="seal cover 1 slides off the end housing",
                )

    return ctx.report()


object_model = build_object_model()

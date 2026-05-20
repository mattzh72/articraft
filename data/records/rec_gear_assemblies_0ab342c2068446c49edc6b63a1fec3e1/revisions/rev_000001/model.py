from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BASE_THICKNESS = 0.035
BEVEL_PAIR_X = -0.180
INPUT_Y = -0.055
BEVEL_PAIR_Z = 0.130
SPUR_X = 0.155
SHAFT_RADIUS = 0.018
GEAR_MODULE = 0.004
LAY_SPUR_TEETH = 28
OUTPUT_SPUR_TEETH = 36


def _add_box(part, size, center, material, name):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_x_cylinder(part, radius, length, center, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_z_cylinder(part, radius, length, center, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _annular_toothed_gear_geometry(
    *,
    teeth: int,
    root_radius: float,
    outer_radius: float,
    width: float,
    bore_radius: float,
    segments_per_tooth: int = 4,
) -> MeshGeometry:
    """Simple robust spur-gear mesh with alternating tooth tips and root gaps."""
    geom = MeshGeometry()
    count = teeth * segments_per_tooth
    radii = []
    for i in range(count):
        phase = i % segments_per_tooth
        radii.append(outer_radius if phase in (1, 2) else root_radius)

    z0 = -width / 2.0
    z1 = width / 2.0
    outer0 = []
    outer1 = []
    inner0 = []
    inner1 = []
    for i, radius in enumerate(radii):
        angle = 2.0 * math.pi * i / count
        c = math.cos(angle)
        s = math.sin(angle)
        outer0.append(geom.add_vertex(radius * c, radius * s, z0))
        outer1.append(geom.add_vertex(radius * c, radius * s, z1))
        inner0.append(geom.add_vertex(bore_radius * c, bore_radius * s, z0))
        inner1.append(geom.add_vertex(bore_radius * c, bore_radius * s, z1))

    for i in range(count):
        j = (i + 1) % count
        # Outside tooth/rim wall
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Bore wall
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        # Front and rear annular faces
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
    return geom


def _bevel_gear_geometry(
    *,
    teeth: int,
    base_root_radius: float,
    base_outer_radius: float,
    tip_root_radius: float,
    tip_outer_radius: float,
    face_width: float,
    bore_radius: float,
    segments_per_tooth: int = 4,
) -> MeshGeometry:
    """Toothed conical frustum for the right-angle bevel mesh."""
    geom = MeshGeometry()
    count = teeth * segments_per_tooth
    base_radii = []
    tip_radii = []
    for i in range(count):
        phase = i % segments_per_tooth
        if phase in (1, 2):
            base_radii.append(base_outer_radius)
            tip_radii.append(tip_outer_radius)
        else:
            base_radii.append(base_root_radius)
            tip_radii.append(tip_root_radius)

    base_outer = []
    tip_outer = []
    base_inner = []
    tip_inner = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        c = math.cos(angle)
        s = math.sin(angle)
        base_outer.append(geom.add_vertex(base_radii[i] * c, base_radii[i] * s, 0.0))
        tip_outer.append(geom.add_vertex(tip_radii[i] * c, tip_radii[i] * s, face_width))
        base_inner.append(geom.add_vertex(bore_radius * c, bore_radius * s, 0.0))
        tip_inner.append(geom.add_vertex(bore_radius * c, bore_radius * s, face_width))

    for i in range(count):
        j = (i + 1) % count
        geom.add_face(base_outer[i], base_outer[j], tip_outer[j])
        geom.add_face(base_outer[i], tip_outer[j], tip_outer[i])
        geom.add_face(base_inner[j], base_inner[i], tip_inner[i])
        geom.add_face(base_inner[j], tip_inner[i], tip_inner[j])
        geom.add_face(base_outer[j], base_outer[i], base_inner[i])
        geom.add_face(base_outer[j], base_inner[i], base_inner[j])
        geom.add_face(tip_outer[i], tip_outer[j], tip_inner[j])
        geom.add_face(tip_outer[i], tip_inner[j], tip_inner[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_transfer_gearbox")

    cast = model.material("blue_gray_cast_housing", rgba=(0.17, 0.22, 0.25, 1.0))
    dark_cast = model.material("dark_bearing_caps", rgba=(0.06, 0.07, 0.075, 1.0))
    shaft_steel = model.material("polished_shaft_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    gear_bronze = model.material("oiled_bronze_gears", rgba=(0.78, 0.58, 0.24, 1.0))
    bolt_steel = model.material("dark_socket_heads", rgba=(0.025, 0.028, 0.030, 1.0))

    # Procedural toothed meshes keep each gear on its own articulated shaft.
    # They use true annular/bored bodies instead of plain cylinders so the open
    # housing clearly exposes both the bevel and spur meshes.
    layshaft_z = BEVEL_PAIR_Z + 0.047

    lay_pitch_radius = GEAR_MODULE * LAY_SPUR_TEETH / 2.0
    output_pitch_radius = GEAR_MODULE * OUTPUT_SPUR_TEETH / 2.0
    output_y = lay_pitch_radius + output_pitch_radius + 0.008

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.035, tube=0.009, radial_segments=32, tubular_segments=16),
        "bearing_race_torus",
    )
    input_bevel_mesh = mesh_from_geometry(
        _bevel_gear_geometry(
            teeth=22,
            base_root_radius=0.040,
            base_outer_radius=0.048,
            tip_root_radius=0.016,
            tip_outer_radius=0.023,
            face_width=0.030,
            bore_radius=0.0175,
        ),
        "vertical_bevel_gear",
    )
    lay_bevel_mesh = mesh_from_geometry(
        _bevel_gear_geometry(
            teeth=22,
            base_root_radius=0.040,
            base_outer_radius=0.048,
            tip_root_radius=0.016,
            tip_outer_radius=0.023,
            face_width=0.030,
            bore_radius=0.0175,
        ),
        "horizontal_bevel_pinion",
    )
    lay_spur_mesh = mesh_from_geometry(
        _annular_toothed_gear_geometry(
            teeth=LAY_SPUR_TEETH,
            root_radius=lay_pitch_radius - 0.0045,
            outer_radius=lay_pitch_radius + 0.0040,
            width=0.045,
            bore_radius=0.0175,
        ),
        "layshaft_spur_gear",
    )
    output_spur_mesh = mesh_from_geometry(
        _annular_toothed_gear_geometry(
            teeth=OUTPUT_SPUR_TEETH,
            root_radius=output_pitch_radius - 0.0045,
            outer_radius=output_pitch_radius + 0.0040,
            width=0.045,
            bore_radius=0.0175,
        ),
        "output_spur_gear",
    )

    housing = model.part("housing")
    _add_box(housing, (0.700, 0.470, BASE_THICKNESS), (0.0, 0.0, BASE_THICKNESS / 2.0), cast, "base_sump")
    _add_box(housing, (0.700, 0.035, 0.290), (0.0, 0.232, 0.175), cast, "rear_wall")
    _add_box(housing, (0.700, 0.030, 0.045), (0.0, -0.222, 0.055), cast, "front_lower_rail")
    _add_box(housing, (0.700, 0.030, 0.055), (0.0, -0.222, 0.300), cast, "front_top_rail")

    for x, suffix in ((-0.335, "0"), (0.335, "1")):
        _add_box(housing, (0.035, 0.470, 0.075), (x, 0.0, 0.072), cast, f"end_bottom_rail_{suffix}")
        _add_box(housing, (0.035, 0.470, 0.075), (x, 0.0, 0.282), cast, f"end_top_rail_{suffix}")
        _add_box(housing, (0.035, 0.038, 0.265), (x, -0.216, 0.167), cast, f"front_corner_post_{suffix}")
        _add_box(housing, (0.035, 0.038, 0.265), (x, 0.216, 0.167), cast, f"rear_corner_post_{suffix}")

    # Pillow-block bearing races for the two horizontal shafts.  The torus holes
    # clear the moving shafts while the pedestal blocks tie the races into the
    # open housing frame.
    for y, shaft_name in ((0.0, "lay"), (output_y, "output")):
        for x, side in ((-0.305, "0"), (0.305, "1")):
            _add_box(
                housing,
                (0.060, 0.068, layshaft_z - 0.080),
                (x, y, 0.040 + 0.5 * (layshaft_z - 0.080)),
                cast,
                f"{shaft_name}_pedestal_{side}",
            )
            housing.visual(
                bearing_mesh,
                origin=Origin(xyz=(x, y, layshaft_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_cast,
                name=f"{shaft_name}_bearing_{side}",
            )
            _add_box(
                housing,
                (0.052, 0.040, 0.032),
                (x, y, layshaft_z - SHAFT_RADIUS - 0.016),
                dark_cast,
                f"{shaft_name}_shaft_saddle_{side}",
            )
            for bolt_y in (y - 0.021, y + 0.021):
                _add_z_cylinder(
                    housing,
                    0.0055,
                    0.006,
                    (x, bolt_y, layshaft_z - 0.042),
                    bolt_steel,
                    f"{shaft_name}_cap_screw_{side}_{'a' if bolt_y < y else 'b'}",
                )

    # Vertical input-shaft bearings: a thrust race on the base and a square top
    # bridge, leaving the center open so the shaft and bevel gear are visible.
    housing.visual(
        bearing_mesh,
        origin=Origin(xyz=(BEVEL_PAIR_X, INPUT_Y, 0.044)),
        material=dark_cast,
        name="input_lower_bearing",
    )
    housing.visual(
        bearing_mesh,
        origin=Origin(xyz=(BEVEL_PAIR_X, INPUT_Y, 0.360)),
        material=dark_cast,
        name="input_top_bearing",
    )
    for y, label in ((-0.050, "front"), (0.050, "rear")):
        _add_box(housing, (0.220, 0.026, 0.018), (BEVEL_PAIR_X, INPUT_Y + y, 0.360), cast, f"input_top_{label}_bar")
    for dx, label in ((-0.085, "near"), (0.085, "far")):
        _add_box(housing, (0.026, 0.145, 0.018), (BEVEL_PAIR_X + dx, INPUT_Y, 0.360), cast, f"input_top_{label}_bar")
    for dx in (-0.085, 0.085):
        for y in (-0.060, 0.025):
            _add_box(
                housing,
                (0.022, 0.016, 0.325),
                (BEVEL_PAIR_X + dx, INPUT_Y + y, 0.198),
                cast,
                f"input_bridge_post_{'n' if dx < 0 else 'p'}_{'f' if y < 0 else 'r'}",
            )

    # Low feet and oil drain plug make the part read as a cast gearbox case
    # rather than a generic open frame.
    for x in (-0.265, 0.265):
        for y in (-0.165, 0.165):
            _add_box(housing, (0.105, 0.070, 0.020), (x, y, -0.010), cast, f"mount_foot_{x:.2f}_{y:.2f}")
            _add_z_cylinder(housing, 0.010, 0.006, (x, y, 0.024), bolt_steel, f"foot_bolt_{x:.2f}_{y:.2f}")
    _add_z_cylinder(housing, 0.012, 0.012, (0.285, -0.222, 0.079), dark_cast, "drain_plug")

    input_shaft = model.part("input_shaft")
    _add_z_cylinder(
        input_shaft,
        SHAFT_RADIUS,
        0.405,
        (0.0, 0.0, 0.105),
        shaft_steel,
        "input_core",
    )
    input_shaft.visual(
        input_bevel_mesh,
        origin=Origin(),
        material=gear_bronze,
        name="input_bevel",
    )
    _add_z_cylinder(input_shaft, 0.027, 0.020, (0.0, 0.0, 0.036), shaft_steel, "input_lock_collar")
    _add_z_cylinder(input_shaft, 0.025, 0.030, (0.0, 0.0, 0.240), shaft_steel, "input_top_coupling")

    layshaft = model.part("layshaft")
    _add_x_cylinder(layshaft, SHAFT_RADIUS, 0.690, (0.0, 0.0, 0.0), shaft_steel, "lay_core")
    layshaft.visual(
        lay_bevel_mesh,
        origin=Origin(xyz=(BEVEL_PAIR_X - 0.047, 0.0, BEVEL_PAIR_Z - layshaft_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gear_bronze,
        name="lay_bevel",
    )
    layshaft.visual(
        lay_spur_mesh,
        origin=Origin(xyz=(SPUR_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gear_bronze,
        name="lay_spur",
    )
    _add_x_cylinder(layshaft, 0.025, 0.030, (SPUR_X - 0.052, 0.0, 0.0), shaft_steel, "lay_spur_collar")
    _add_x_cylinder(layshaft, 0.025, 0.030, (SPUR_X + 0.052, 0.0, 0.0), shaft_steel, "lay_spur_nut")

    output_shaft = model.part("output_shaft")
    _add_x_cylinder(output_shaft, SHAFT_RADIUS, 0.760, (0.015, 0.0, 0.0), shaft_steel, "output_core")
    output_shaft.visual(
        output_spur_mesh,
        origin=Origin(xyz=(SPUR_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gear_bronze,
        name="output_spur",
    )
    _add_x_cylinder(output_shaft, 0.036, 0.080, (0.395, 0.0, 0.0), shaft_steel, "output_coupling")
    _add_x_cylinder(output_shaft, 0.025, 0.030, (SPUR_X - 0.058, 0.0, 0.0), shaft_steel, "output_spur_collar")
    _add_x_cylinder(output_shaft, 0.025, 0.030, (SPUR_X + 0.058, 0.0, 0.0), shaft_steel, "output_spur_nut")

    shaft_motion = MotionProperties(damping=0.01, friction=0.003)
    shaft_limits = MotionLimits(effort=12.0, velocity=45.0)
    model.articulation(
        "housing_to_input",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=input_shaft,
        origin=Origin(xyz=(BEVEL_PAIR_X, INPUT_Y, BEVEL_PAIR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=shaft_limits,
        motion_properties=shaft_motion,
    )
    model.articulation(
        "housing_to_layshaft",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=layshaft,
        origin=Origin(xyz=(0.0, 0.0, layshaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=shaft_limits,
        motion_properties=shaft_motion,
        mimic=Mimic(joint="housing_to_input", multiplier=-1.0),
    )
    model.articulation(
        "housing_to_output",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=output_shaft,
        origin=Origin(xyz=(0.0, output_y, layshaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=shaft_limits,
        motion_properties=shaft_motion,
        mimic=Mimic(joint="housing_to_input", multiplier=LAY_SPUR_TEETH / OUTPUT_SPUR_TEETH),
    )

    model.meta["gearbox_layout"] = {
        "bevel_pair_center": (BEVEL_PAIR_X, 0.0, BEVEL_PAIR_Z),
        "input_y_offset": INPUT_Y,
        "layshaft_z": layshaft_z,
        "output_y": output_y,
        "spur_center_x": SPUR_X,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    layshaft = object_model.get_part("layshaft")
    output_shaft = object_model.get_part("output_shaft")
    input_joint = object_model.get_articulation("housing_to_input")
    lay_joint = object_model.get_articulation("housing_to_layshaft")
    output_joint = object_model.get_articulation("housing_to_output")

    ctx.check("three_supported_shafts", all((input_shaft, layshaft, output_shaft)), "Expected input, lay, and output shaft parts.")
    ctx.check(
        "shaft_joints_on_bearing_axes",
        input_joint.axis == (0.0, 0.0, 1.0) and lay_joint.axis == (1.0, 0.0, 0.0) and output_joint.axis == (1.0, 0.0, 0.0),
        details=f"axes={input_joint.axis}, {lay_joint.axis}, {output_joint.axis}",
    )
    ctx.check(
        "gear_train_is_coupled",
        lay_joint.mimic is not None and output_joint.mimic is not None,
        "The layshaft and output shaft should follow the input through the visible gear train.",
    )

    ctx.allow_overlap(
        input_shaft,
        layshaft,
        elem_a="input_bevel",
        elem_b="lay_bevel",
        reason="The simplified bevel-tooth meshes deliberately interdigitate at the exposed right-angle mesh.",
    )
    ctx.expect_overlap(
        input_shaft,
        layshaft,
        axes="yz",
        elem_a="input_bevel",
        elem_b="lay_bevel",
        min_overlap=0.030,
        name="bevel gears share a visible mesh envelope",
    )
    ctx.expect_overlap(
        layshaft,
        output_shaft,
        axes="xz",
        elem_a="lay_spur",
        elem_b="output_spur",
        min_overlap=0.040,
        name="spur gears share a visible mesh envelope",
    )
    ctx.expect_origin_gap(
        output_shaft,
        layshaft,
        axis="y",
        min_gap=0.090,
        max_gap=0.180,
        name="parallel output shaft is offset by the spur center distance",
    )
    ctx.expect_origin_gap(
        layshaft,
        housing,
        axis="z",
        min_gap=0.150,
        max_gap=0.190,
        name="horizontal shafts sit on the raised bearing centerline",
    )

    return ctx.report()


object_model = build_object_model()

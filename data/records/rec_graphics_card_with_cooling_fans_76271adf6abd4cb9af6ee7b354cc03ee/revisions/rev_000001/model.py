from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CARD_LENGTH = 0.290
CARD_THICKNESS = 0.040
CARD_HEIGHT = 0.112
FAN_CENTER_Z = 0.058
BLOWER_CENTER_X = 0.048
AXIAL_CENTER_X = 0.212


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _annular_shell(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    *,
    center_depth: float = 0.0,
    segments: int = 48,
) -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * depth),
            (outer_radius, 0.5 * depth),
        ],
        [
            (inner_radius, -0.5 * depth),
            (inner_radius, 0.5 * depth),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    if center_depth:
        shell.translate(0.0, 0.0, center_depth)
    return shell


def _radial_pattern(base_geom: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_z(angle_offset + index * math.tau / count))
    return patterned


def _blower_rotor_mesh() -> MeshGeometry:
    rotor = MeshGeometry()
    rotor.merge(_annular_shell(0.010, 0.0050, 0.010, center_depth=0.013, segments=40))
    rotor.merge(_annular_shell(0.034, 0.0315, 0.010, center_depth=0.013, segments=56))
    rotor.merge(_annular_shell(0.034, 0.010, 0.0016, center_depth=0.0083, segments=56))
    rotor.merge(_annular_shell(0.034, 0.010, 0.0016, center_depth=0.0177, segments=56))

    blade = (
        BoxGeometry((0.022, 0.0026, 0.010))
        .translate(0.021, 0.0, 0.013)
        .rotate_z(math.radians(7.0))
    )
    rotor.merge(_radial_pattern(blade, 14, angle_offset=math.pi / 14.0))
    return rotor


def _axial_rotor_mesh() -> MeshGeometry:
    rotor = MeshGeometry()
    rotor.merge(_annular_shell(0.014, 0.0055, 0.008, center_depth=0.015, segments=40))
    rotor.merge(CylinderGeometry(radius=0.016, height=0.0024, radial_segments=36).translate(0.0, 0.0, 0.015))
    rotor.merge(CylinderGeometry(radius=0.0145, height=0.003, radial_segments=36).translate(0.0, 0.0, 0.0195))

    blade = (
        BoxGeometry((0.026, 0.010, 0.0022))
        .translate(0.019, 0.0, 0.015)
        .rotate_y(math.radians(18.0))
        .rotate_z(math.radians(18.0))
    )
    rotor.merge(_radial_pattern(blade, 11, angle_offset=math.pi / 11.0))
    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hybrid_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.11, 0.12, 0.13, 1.0))
    backplate_black = model.material("backplate_black", rgba=(0.18, 0.19, 0.21, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.10, 0.28, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.07, 0.08, 0.09, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.77, 0.64, 0.24, 1.0))

    card_body = model.part("card_body")
    card_body.visual(
        Box((0.286, 0.0015, 0.104)),
        origin=Origin(xyz=(0.144, -0.01825, 0.056)),
        material=backplate_black,
        name="backplate",
    )
    card_body.visual(
        Box((0.286, 0.036, 0.004)),
        origin=Origin(xyz=(0.144, 0.0, 0.108)),
        material=shroud_black,
        name="top_cover",
    )
    card_body.visual(
        Box((0.286, 0.036, 0.004)),
        origin=Origin(xyz=(0.144, 0.0, 0.002)),
        material=shroud_black,
        name="bottom_cover",
    )
    card_body.visual(
        Box((0.160, 0.004, 0.010)),
        origin=Origin(xyz=(0.208, 0.018, 0.105)),
        material=shroud_black,
        name="front_top_rail",
    )
    card_body.visual(
        Box((0.160, 0.004, 0.010)),
        origin=Origin(xyz=(0.208, 0.018, 0.005)),
        material=shroud_black,
        name="front_bottom_rail",
    )
    card_body.visual(
        Box((0.016, 0.004, 0.068)),
        origin=Origin(xyz=(0.126, 0.018, FAN_CENTER_Z)),
        material=shroud_black,
        name="front_center_bridge",
    )
    card_body.visual(
        Box((0.004, 0.036, 0.100)),
        origin=Origin(xyz=(0.288, 0.0, 0.056)),
        material=shroud_black,
        name="front_end_wall",
    )
    card_body.visual(
        Box((0.048, 0.022, 0.024)),
        origin=Origin(xyz=(0.106, 0.006, 0.094)),
        material=shroud_black,
        name="blower_duct_cover",
    )
    card_body.visual(
        Box((0.020, 0.016, 0.024)),
        origin=Origin(xyz=(BLOWER_CENTER_X, -0.010, FAN_CENTER_Z)),
        material=dark_metal,
        name="blower_bearing_mount",
    )
    card_body.visual(
        _mesh("blower_housing_band", _annular_shell(0.048, 0.037, 0.028, center_depth=0.004, segments=64)),
        origin=Origin(xyz=(BLOWER_CENTER_X, 0.0, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="blower_housing",
    )
    card_body.visual(
        _mesh("axial_opening_ring", _annular_shell(0.048, 0.040, 0.006, center_depth=0.017, segments=64)),
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.0, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="axial_opening_ring",
    )
    card_body.visual(
        _mesh("axial_rear_shroud", _annular_shell(0.048, 0.040, 0.012, center_depth=0.006, segments=64)),
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.0, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axial_rear_shroud",
    )
    card_body.visual(
        Box((0.044, 0.003, 0.006)),
        origin=Origin(xyz=(AXIAL_CENTER_X + 0.024, 0.006, FAN_CENTER_Z)),
        material=dark_metal,
        name="axial_right_strut",
    )
    card_body.visual(
        Box((0.044, 0.003, 0.006)),
        origin=Origin(xyz=(AXIAL_CENTER_X - 0.024, 0.006, FAN_CENTER_Z)),
        material=dark_metal,
        name="axial_left_strut",
    )
    card_body.visual(
        Box((0.006, 0.003, 0.044)),
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.006, FAN_CENTER_Z + 0.024)),
        material=dark_metal,
        name="axial_upper_strut",
    )
    card_body.visual(
        Box((0.006, 0.003, 0.044)),
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.006, FAN_CENTER_Z - 0.024)),
        material=dark_metal,
        name="axial_lower_strut",
    )
    card_body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.006, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axial_stator_core",
    )
    card_body.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(BLOWER_CENTER_X, 0.000, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="blower_support_spindle",
    )
    card_body.visual(
        Box((0.278, 0.0016, 0.098)),
        origin=Origin(xyz=(0.141, -0.0122, 0.050)),
        material=pcb_green,
        name="pcb",
    )
    card_body.visual(
        Box((0.120, 0.0016, 0.008)),
        origin=Origin(xyz=(0.145, -0.0122, 0.004)),
        material=connector_gold,
        name="pcie_edge_connector",
    )
    card_body.visual(
        Box((0.002, CARD_THICKNESS, CARD_HEIGHT)),
        origin=Origin(xyz=(0.001, 0.0, 0.056)),
        material=dark_metal,
        name="io_bracket",
    )
    card_body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_THICKNESS, CARD_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.145, 0.0, 0.056)),
    )

    blower_rotor = model.part("blower_rotor")
    blower_rotor.visual(
        _mesh("blower_rotor_mesh", _blower_rotor_mesh()),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rotor_black,
        name="blower_rotor_shell",
    )
    blower_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.012),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
    )

    axial_rotor = model.part("axial_rotor")
    axial_rotor.visual(
        _mesh("axial_rotor_mesh", _axial_rotor_mesh()),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rotor_black,
        name="axial_rotor_shell",
    )
    axial_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.012),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )

    model.articulation(
        "body_to_blower_rotor",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=blower_rotor,
        origin=Origin(xyz=(BLOWER_CENTER_X, 0.0, FAN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=220.0),
    )
    model.articulation(
        "body_to_axial_rotor",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=axial_rotor,
        origin=Origin(xyz=(AXIAL_CENTER_X, 0.0, FAN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=220.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("card_body")
    blower = object_model.get_part("blower_rotor")
    axial = object_model.get_part("axial_rotor")
    blower_joint = object_model.get_articulation("body_to_blower_rotor")
    axial_joint = object_model.get_articulation("body_to_axial_rotor")
    blower_housing = body.get_visual("blower_housing")
    axial_opening = body.get_visual("axial_opening_ring")

    ctx.check(
        "blower rotor uses continuous articulation",
        blower_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in blower_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={blower_joint.articulation_type}, axis={blower_joint.axis}",
    )
    ctx.check(
        "axial rotor uses continuous articulation",
        axial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in axial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={axial_joint.articulation_type}, axis={axial_joint.axis}",
    )
    ctx.check(
        "blower pod sits at one end and axial opening at the other",
        blower_housing.origin.xyz[0] < axial_opening.origin.xyz[0] - 0.12,
        details=f"blower_x={blower_housing.origin.xyz[0]}, axial_x={axial_opening.origin.xyz[0]}",
    )
    ctx.expect_origin_distance(
        blower,
        axial,
        axes="x",
        min_dist=0.14,
        max_dist=0.20,
        name="blower rotor and axial rotor are widely separated along card length",
    )

    with ctx.pose({blower_joint: 1.1, axial_joint: 0.9}):
        ctx.expect_overlap(
            blower,
            body,
            axes="xz",
            min_overlap=0.060,
            name="blower rotor stays nested inside the graphics card silhouette",
        )
        ctx.expect_overlap(
            axial,
            body,
            axes="xz",
            min_overlap=0.065,
            name="axial rotor stays nested inside the graphics card silhouette",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

HUB_RADIUS = 0.045
HUB_HEIGHT = 0.07
HINGE_ORIGIN_RADIUS = 0.057
LOWER_SLEEVE_OUTER_RADIUS = 0.022
LOWER_SLEEVE_INNER_RADIUS = 0.018
LOWER_SLEEVE_LENGTH = 0.58
LOWER_SLEEVE_CENTER_Z = 0.32
UPPER_TUBE_RADIUS = 0.014
UPPER_TUBE_LENGTH = 0.56
UPPER_TUBE_CENTER_Z = 0.28
COLUMN_JOINT_Z = 0.15
BRACKET_OFFSET_Z = 0.84
LEG_REST_ANGLE = math.radians(61.0)
LEG_STRUT_LENGTH = 0.54
LEG_FOOT_DISTANCE = 0.56
RING_OUTER_RADIUS = 0.22
RING_INNER_RADIUS = 0.15
RING_DEPTH = 0.03


def _ring_band_mesh(
    filename: str,
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
    radial_segments: int = 64,
):
    outer = CylinderGeometry(radius=outer_radius, height=depth, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=depth + 0.004,
        radial_segments=radial_segments,
    )
    return mesh_from_geometry(boolean_difference(outer, inner), ASSETS.mesh_path(filename))


def _leg_point(distance: float) -> tuple[float, float, float]:
    return (
        distance * math.cos(LEG_REST_ANGLE),
        0.0,
        -distance * math.sin(LEG_REST_ANGLE),
    )


def _radial_xy(radius: float, yaw: float) -> tuple[float, float]:
    return (radius * math.cos(yaw), radius * math.sin(yaw))


def _add_tripod_leg(model: ArticulatedObject, *, name: str, material: str, foot_material: str) -> None:
    leg = model.part(name)

    strut_center = _leg_point(LEG_STRUT_LENGTH * 0.5)
    strut_pitch = (math.pi * 0.5) + LEG_REST_ANGLE
    leg.visual(
        Cylinder(radius=0.010, length=LEG_STRUT_LENGTH),
        origin=Origin(xyz=strut_center, rpy=(0.0, strut_pitch, 0.0)),
        material=material,
        name="main_strut",
    )
    brace_center = _leg_point(0.47)
    leg.visual(
        Cylinder(radius=0.0072, length=0.16),
        origin=Origin(xyz=brace_center, rpy=(0.0, strut_pitch, 0.0)),
        material=material,
        name="lower_extension",
    )
    leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=material,
        name="hinge_knuckle",
    )
    leg.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=_leg_point(LEG_STRUT_LENGTH), rpy=(0.0, strut_pitch, 0.0)),
        material=foot_material,
        name="foot_tip",
    )
    leg.inertial = Inertial.from_geometry(
        Box((0.58, 0.04, 0.56)),
        mass=0.32,
        origin=Origin(xyz=_leg_point(0.28)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light_tripod", assets=ASSETS)

    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.95, 0.96, 0.97, 0.92))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))

    lower_sleeve_mesh = _ring_band_mesh(
        "tripod_lower_sleeve.obj",
        outer_radius=LOWER_SLEEVE_OUTER_RADIUS,
        inner_radius=LOWER_SLEEVE_INNER_RADIUS,
        depth=LOWER_SLEEVE_LENGTH,
    )
    receiver_collar_mesh = _ring_band_mesh(
        "tripod_receiver_collar.obj",
        outer_radius=0.028,
        inner_radius=0.0185,
        depth=0.05,
    )
    ring_housing_mesh = _ring_band_mesh(
        "ring_housing.obj",
        outer_radius=RING_OUTER_RADIUS,
        inner_radius=RING_INNER_RADIUS,
        depth=RING_DEPTH,
        radial_segments=80,
    )
    diffuser_mesh = _ring_band_mesh(
        "ring_diffuser.obj",
        outer_radius=0.213,
        inner_radius=0.163,
        depth=0.006,
        radial_segments=80,
    )

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(),
        material=dark_metal,
        name="hub_shell",
    )
    tripod_base.visual(
        lower_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SLEEVE_CENTER_Z)),
        material=stand_black,
        name="lower_sleeve",
    )
    tripod_base.visual(
        receiver_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=dark_metal,
        name="receiver_collar",
    )
    tripod_base.visual(
        Box((0.018, 0.040, 0.018)),
        origin=Origin(xyz=(0.031, 0.0, 0.57)),
        material=satin_black,
        name="receiver_knob",
    )
    for leg_name, yaw in (
        ("leg_a_mount", 0.0),
        ("leg_b_mount", math.tau / 3.0),
        ("leg_c_mount", (2.0 * math.tau) / 3.0),
    ):
        mount_x, mount_y = _radial_xy(0.037, yaw)
        tripod_base.visual(
            Box((0.016, 0.034, 0.028)),
            origin=Origin(xyz=(mount_x, mount_y, -0.005), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=leg_name,
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.70)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=UPPER_TUBE_RADIUS, length=UPPER_TUBE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, UPPER_TUBE_CENTER_Z)),
        material=stand_black,
        name="upper_tube",
    )
    center_column.visual(
        Cylinder(radius=0.010, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=dark_metal,
        name="top_spigot",
    )
    center_column.inertial = Inertial.from_geometry(
        Box((0.035, 0.035, 0.64)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Cylinder(radius=0.010, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
        material=dark_metal,
        name="stem",
    )
    tilt_bracket.visual(
        Box((0.060, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=dark_metal,
        name="cross_block",
    )
    tilt_bracket.visual(
        Box((0.454, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=dark_metal,
        name="lower_bridge",
    )
    tilt_bracket.visual(
        Box((0.014, 0.020, 0.156)),
        origin=Origin(xyz=(-0.227, 0.0, -0.077)),
        material=dark_metal,
        name="left_arm",
    )
    tilt_bracket.visual(
        Box((0.014, 0.020, 0.156)),
        origin=Origin(xyz=(0.227, 0.0, -0.077)),
        material=dark_metal,
        name="right_arm",
    )
    tilt_bracket.inertial = Inertial.from_geometry(
        Box((0.47, 0.04, 0.28)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    light_ring = model.part("light_ring")
    light_ring.visual(
        ring_housing_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_black,
        name="ring_housing",
    )
    light_ring.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=diffuser_white,
        name="diffuser_ring",
    )
    light_ring.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.207, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_metal,
        name="left_pivot",
    )
    light_ring.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.207, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_metal,
        name="right_pivot",
    )
    light_ring.visual(
        Box((0.020, 0.030, 0.050)),
        origin=Origin(xyz=(-0.207, 0.015, 0.0)),
        material=dark_metal,
        name="left_yoke_lug",
    )
    light_ring.visual(
        Box((0.020, 0.030, 0.050)),
        origin=Origin(xyz=(0.207, 0.015, 0.0)),
        material=dark_metal,
        name="right_yoke_lug",
    )
    light_ring.visual(
        Box((0.045, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.030, -0.165)),
        material=dark_metal,
        name="control_box",
    )
    light_ring.inertial = Inertial.from_geometry(
        Box((0.46, 0.05, 0.46)),
        mass=1.8,
        origin=Origin(),
    )

    _add_tripod_leg(model, name="leg_a", material=stand_black, foot_material=rubber_black)
    _add_tripod_leg(model, name="leg_b", material=stand_black, foot_material=rubber_black)
    _add_tripod_leg(model, name="leg_c", material=stand_black, foot_material=rubber_black)

    model.articulation(
        "base_to_center_column",
        ArticulationType.PRISMATIC,
        parent=tripod_base,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.45,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "center_column_to_tilt_bracket",
        ArticulationType.FIXED,
        parent=center_column,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_OFFSET_Z)),
    )
    model.articulation(
        "tilt_bracket_to_light_ring",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=light_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-math.radians(60.0),
            upper=math.radians(60.0),
        ),
    )

    for leg_name, articulation_name, yaw in (
        ("leg_a", "base_to_leg_a", 0.0),
        ("leg_b", "base_to_leg_b", math.tau / 3.0),
        ("leg_c", "base_to_leg_c", (2.0 * math.tau) / 3.0),
    ):
        hinge_x, hinge_y = _radial_xy(HINGE_ORIGIN_RADIUS, yaw)
        model.articulation(
            articulation_name,
            ArticulationType.REVOLUTE,
            parent=tripod_base,
            child=leg_name,
            origin=Origin(xyz=(hinge_x, hinge_y, -0.005), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.5,
                lower=-1.0,
                upper=0.20,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    tripod_base = object_model.get_part("tripod_base")
    center_column = object_model.get_part("center_column")
    tilt_bracket = object_model.get_part("tilt_bracket")
    light_ring = object_model.get_part("light_ring")
    leg_a = object_model.get_part("leg_a")
    leg_b = object_model.get_part("leg_b")
    leg_c = object_model.get_part("leg_c")

    base_to_center_column = object_model.get_articulation("base_to_center_column")
    tilt_joint = object_model.get_articulation("tilt_bracket_to_light_ring")
    base_to_leg_a = object_model.get_articulation("base_to_leg_a")

    lower_sleeve = tripod_base.get_visual("lower_sleeve")
    leg_a_mount = tripod_base.get_visual("leg_a_mount")
    leg_b_mount = tripod_base.get_visual("leg_b_mount")
    leg_c_mount = tripod_base.get_visual("leg_c_mount")
    upper_tube = center_column.get_visual("upper_tube")
    top_spigot = center_column.get_visual("top_spigot")
    bracket_stem = tilt_bracket.get_visual("stem")
    left_arm = tilt_bracket.get_visual("left_arm")
    right_arm = tilt_bracket.get_visual("right_arm")
    left_pivot = light_ring.get_visual("left_pivot")
    right_pivot = light_ring.get_visual("right_pivot")
    leg_a_knuckle = leg_a.get_visual("hinge_knuckle")
    leg_b_knuckle = leg_b.get_visual("hinge_knuckle")
    leg_c_knuckle = leg_c.get_visual("hinge_knuckle")
    leg_a_foot = leg_a.get_visual("foot_tip")
    leg_b_foot = leg_b.get_visual("foot_tip")
    leg_c_foot = leg_c.get_visual("foot_tip")

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
    ctx.allow_overlap(
        center_column,
        tripod_base,
        reason="Upper telescoping tube intentionally slides within the lower sleeve.",
        elem_a=upper_tube,
        elem_b=lower_sleeve,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        center_column,
        tripod_base,
        axes="xy",
        inner_elem=upper_tube,
        outer_elem=lower_sleeve,
    )
    ctx.expect_overlap(
        center_column,
        tripod_base,
        axes="xy",
        min_overlap=0.028,
        elem_a=upper_tube,
        elem_b=lower_sleeve,
    )
    ctx.expect_contact(center_column, tilt_bracket, elem_a=top_spigot, elem_b=bracket_stem)
    ctx.expect_origin_distance(light_ring, center_column, axes="xy", max_dist=0.002)
    ctx.expect_origin_gap(light_ring, tripod_base, axis="z", min_gap=0.97, max_gap=1.01)
    ctx.expect_contact(light_ring, tilt_bracket, elem_a=left_pivot, elem_b=left_arm)
    ctx.expect_contact(light_ring, tilt_bracket, elem_a=right_pivot, elem_b=right_arm)

    ctx.expect_contact(leg_a, tripod_base, elem_a=leg_a_knuckle, elem_b=leg_a_mount)
    ctx.expect_contact(leg_b, tripod_base, elem_a=leg_b_knuckle, elem_b=leg_b_mount)
    ctx.expect_contact(leg_c, tripod_base, elem_a=leg_c_knuckle, elem_b=leg_c_mount)

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ring_rest_aabb = ctx.part_world_aabb(light_ring)
    assert ring_rest_aabb is not None
    ring_rest_y = ring_rest_aabb[1][1] - ring_rest_aabb[0][1]
    ring_rest_z = ring_rest_aabb[1][2] - ring_rest_aabb[0][2]

    center_rest = ctx.part_world_position(center_column)
    ring_rest = ctx.part_world_position(light_ring)
    assert center_rest is not None
    assert ring_rest is not None

    for leg_part, foot_visual in ((leg_a, leg_a_foot), (leg_b, leg_b_foot), (leg_c, leg_c_foot)):
        foot_aabb = ctx.part_element_world_aabb(leg_part, elem=foot_visual)
        assert foot_aabb is not None
        foot_center = _aabb_center(foot_aabb)
        assert math.hypot(foot_center[0], foot_center[1]) > 0.23
        assert foot_center[2] < -0.45

    with ctx.pose({base_to_center_column: 0.30}):
        center_extended = ctx.part_world_position(center_column)
        ring_extended = ctx.part_world_position(light_ring)
        assert center_extended is not None
        assert ring_extended is not None
        assert center_extended[2] > center_rest[2] + 0.29
        assert ring_extended[2] > ring_rest[2] + 0.29
        ctx.expect_within(
            center_column,
            tripod_base,
            axes="xy",
            inner_elem=upper_tube,
            outer_elem=lower_sleeve,
        )

    with ctx.pose({tilt_joint: math.radians(60.0)}):
        ring_tilted = ctx.part_world_aabb(light_ring)
        assert ring_tilted is not None
        ring_tilt_y = ring_tilted[1][1] - ring_tilted[0][1]
        ring_tilt_z = ring_tilted[1][2] - ring_tilted[0][2]
        assert ring_tilt_y > ring_rest_y + 0.12
        assert ring_tilt_z < ring_rest_z - 0.09
        ctx.expect_contact(light_ring, tilt_bracket, elem_a=left_pivot, elem_b=left_arm)
        ctx.expect_contact(light_ring, tilt_bracket, elem_a=right_pivot, elem_b=right_arm)

    with ctx.pose({tilt_joint: -math.radians(60.0)}):
        ctx.expect_contact(light_ring, tilt_bracket, elem_a=left_pivot, elem_b=left_arm)
        ctx.expect_contact(light_ring, tilt_bracket, elem_a=right_pivot, elem_b=right_arm)

    leg_rest_foot = ctx.part_element_world_aabb(leg_a, elem=leg_a_foot)
    assert leg_rest_foot is not None
    leg_rest_center = _aabb_center(leg_rest_foot)
    with ctx.pose({base_to_leg_a: 0.20}):
        leg_folded_foot = ctx.part_element_world_aabb(leg_a, elem=leg_a_foot)
        assert leg_folded_foot is not None
        leg_folded_center = _aabb_center(leg_folded_foot)
        assert leg_folded_center[2] < leg_rest_center[2] - 0.03
        assert math.hypot(leg_folded_center[0], leg_folded_center[1]) < math.hypot(
            leg_rest_center[0],
            leg_rest_center[1],
        ) - 0.09
        ctx.expect_contact(leg_a, tripod_base, elem_a=leg_a_knuckle, elem_b=leg_a_mount)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

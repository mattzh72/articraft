from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width_y,
        height_z,
        radius=min(radius, width_y * 0.45, height_z * 0.45),
        corner_segments=8,
    )
    return [(x_pos, y, z + z_center) for y, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_model_plane_rig")

    stand_gray = model.material("stand_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    airframe_white = model.material("airframe_white", rgba=(0.93, 0.94, 0.90, 1.0))
    datum_orange = model.material("datum_orange", rgba=(0.92, 0.42, 0.10, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.35, 0.48, 0.56, 0.92))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))

    stand_base = model.part("stand_base")

    base_plate_geom = ExtrudeGeometry(
        rounded_rect_profile(0.260, 0.200, 0.022, corner_segments=10),
        0.018,
    )
    stand_base.visual(
        _save_mesh("stand_base_plate", base_plate_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_gray,
        name="base_plate",
    )
    stand_base.visual(
        Box((0.044, 0.056, 0.126)),
        origin=Origin(xyz=(-0.078, 0.0, 0.081)),
        material=stand_gray,
        name="mast",
    )
    stand_base.visual(
        Box((0.090, 0.036, 0.022)),
        origin=Origin(xyz=(-0.044, 0.0, 0.060), rpy=(0.0, -0.62, 0.0)),
        material=stand_gray,
        name="rear_brace",
    )
    stand_base.visual(
        Box((0.060, 0.052, 0.016)),
        origin=Origin(xyz=(-0.060, 0.0, 0.149)),
        material=stand_gray,
        name="trunnion_bridge",
    )
    stand_base.visual(
        Box((0.018, 0.008, 0.066)),
        origin=Origin(xyz=(-0.060, 0.022, 0.181)),
        material=dark_steel,
        name="left_cheek",
    )
    stand_base.visual(
        Box((0.018, 0.008, 0.066)),
        origin=Origin(xyz=(-0.060, -0.022, 0.181)),
        material=dark_steel,
        name="right_cheek",
    )
    stand_base.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.060, 0.031, 0.181), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="adjust_knob",
    )
    stand_base.visual(
        Box((0.014, 0.006, 0.024)),
        origin=Origin(xyz=(-0.060, 0.038, 0.181)),
        material=dark_steel,
        name="knob_grip",
    )
    stand_base.visual(
        Box((0.120, 0.004, 0.0015)),
        origin=Origin(xyz=(0.018, 0.0, 0.01875)),
        material=datum_orange,
        name="datum_centerline_x",
    )
    stand_base.visual(
        Box((0.004, 0.120, 0.0015)),
        origin=Origin(xyz=(0.018, 0.0, 0.01875)),
        material=datum_orange,
        name="datum_centerline_y",
    )
    stand_base.visual(
        Box((0.018, 0.0015, 0.003)),
        origin=Origin(xyz=(-0.060, 0.01875, 0.181)),
        material=datum_orange,
        name="left_zero_mark",
    )
    stand_base.visual(
        Box((0.012, 0.0015, 0.003)),
        origin=Origin(xyz=(-0.072, 0.01875, 0.193)),
        material=datum_orange,
        name="left_plus_mark",
    )
    stand_base.visual(
        Box((0.012, 0.0015, 0.003)),
        origin=Origin(xyz=(-0.072, 0.01875, 0.169)),
        material=datum_orange,
        name="left_minus_mark",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.260, 0.200, 0.214)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_barrel",
    )
    carriage.visual(
        Box((0.076, 0.010, 0.010)),
        origin=Origin(xyz=(0.038, 0.011, 0.015), rpy=(0.0, -0.32, 0.0)),
        material=dark_steel,
        name="left_support_arm",
    )
    carriage.visual(
        Box((0.076, 0.010, 0.010)),
        origin=Origin(xyz=(0.038, -0.011, 0.015), rpy=(0.0, -0.32, 0.0)),
        material=dark_steel,
        name="right_support_arm",
    )
    carriage.visual(
        Box((0.028, 0.034, 0.026)),
        origin=Origin(xyz=(0.079, 0.0, 0.019)),
        material=dark_steel,
        name="mount_block",
    )
    carriage.visual(
        Box((0.072, 0.032, 0.006)),
        origin=Origin(xyz=(0.095, 0.0, 0.034)),
        material=stand_gray,
        name="saddle_pad",
    )
    carriage.visual(
        Box((0.018, 0.002, 0.008)),
        origin=Origin(xyz=(-0.004, 0.016, 0.0)),
        material=datum_orange,
        name="pointer_tab",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.125, 0.034, 0.052)),
        mass=0.35,
        origin=Origin(xyz=(0.050, 0.0, 0.018)),
    )

    model.articulation(
        "stand_pitch",
        ArticulationType.REVOLUTE,
        parent=stand_base,
        child=carriage,
        origin=Origin(xyz=(-0.060, 0.0, 0.181)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.2,
            lower=math.radians(-8.0),
            upper=math.radians(18.0),
        ),
    )

    airframe = model.part("airframe")

    fuselage_sections = [
        _yz_section(-0.155, 0.010, 0.018, 0.031, 0.004),
        _yz_section(-0.120, 0.020, 0.024, 0.033, 0.006),
        _yz_section(-0.050, 0.036, 0.040, 0.037, 0.010),
        _yz_section(0.000, 0.044, 0.052, 0.040, 0.012),
        _yz_section(0.080, 0.040, 0.048, 0.040, 0.011),
        _yz_section(0.150, 0.024, 0.028, 0.037, 0.007),
        _yz_section(0.188, 0.014, 0.018, 0.036, 0.005),
    ]
    fuselage_geom = section_loft(fuselage_sections)
    airframe.visual(
        _save_mesh("airframe_fuselage", fuselage_geom),
        material=airframe_white,
        name="fuselage_shell",
    )

    left_wing_geom = ExtrudeGeometry(
        [(-0.058, 0.0), (0.040, 0.0), (0.010, 0.225), (-0.030, 0.225)],
        0.006,
    )
    right_wing_geom = ExtrudeGeometry(
        [(-0.058, 0.0), (-0.030, -0.225), (0.010, -0.225), (0.040, 0.0)],
        0.006,
    )
    airframe.visual(
        _save_mesh("left_wing_panel", left_wing_geom),
        origin=Origin(xyz=(-0.004, 0.0, 0.043), rpy=(math.radians(4.0), 0.0, 0.0)),
        material=airframe_white,
        name="left_wing",
    )
    airframe.visual(
        _save_mesh("right_wing_panel", right_wing_geom),
        origin=Origin(xyz=(-0.004, 0.0, 0.043), rpy=(math.radians(-4.0), 0.0, 0.0)),
        material=airframe_white,
        name="right_wing",
    )
    airframe.visual(
        Box((0.090, 0.050, 0.012)),
        origin=Origin(xyz=(-0.004, 0.0, 0.043)),
        material=airframe_white,
        name="wing_center_section",
    )

    tailplane_geom = ExtrudeGeometry(
        [(-0.050, 0.0), (-0.035, -0.080), (0.020, -0.080), (0.030, 0.0), (0.020, 0.080), (-0.035, 0.080)],
        0.004,
    )
    airframe.visual(
        _save_mesh("tailplane_panel", tailplane_geom),
        origin=Origin(xyz=(-0.122, 0.0, 0.045)),
        material=airframe_white,
        name="tailplane",
    )

    fin_geom = ExtrudeGeometry(
        [(-0.030, 0.0), (0.006, 0.0), (0.010, 0.050), (-0.014, 0.075), (-0.028, 0.032)],
        0.006,
    ).rotate_x(math.pi / 2.0)
    airframe.visual(
        _save_mesh("vertical_fin", fin_geom),
        origin=Origin(xyz=(-0.122, 0.0, 0.045)),
        material=airframe_white,
        name="vertical_fin",
    )

    airframe.visual(
        Box((0.070, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stand_gray,
        name="mount_shoe",
    )
    airframe.visual(
        Box((0.088, 0.032, 0.024)),
        origin=Origin(xyz=(0.002, 0.0, 0.015)),
        material=airframe_white,
        name="belly_fairing",
    )
    airframe.visual(
        Box((0.050, 0.034, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.066)),
        material=canopy_tint,
        name="canopy",
    )
    airframe.visual(
        Box((0.030, 0.008, 0.018)),
        origin=Origin(xyz=(0.006, 0.022, 0.040)),
        material=datum_orange,
        name="left_datum_flat",
    )
    airframe.visual(
        Box((0.030, 0.008, 0.018)),
        origin=Origin(xyz=(0.006, -0.022, 0.040)),
        material=datum_orange,
        name="right_datum_flat",
    )
    airframe.visual(
        Box((0.070, 0.004, 0.0015)),
        origin=Origin(xyz=(-0.014, 0.0, 0.05075)),
        material=datum_orange,
        name="wing_index_longitudinal",
    )
    airframe.visual(
        Box((0.004, 0.116, 0.0015)),
        origin=Origin(xyz=(-0.014, 0.0, 0.05075)),
        material=datum_orange,
        name="wing_index_transverse",
    )
    airframe.visual(
        Cylinder(radius=0.0105, length=0.014),
        origin=Origin(xyz=(0.195, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="nose_shaft",
    )
    airframe.visual(
        Cylinder(radius=0.0115, length=0.002),
        origin=Origin(xyz=(0.203, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=datum_orange,
        name="nose_bulkhead",
    )
    airframe.visual(
        Sphere(radius=0.0025),
        origin=Origin(xyz=(0.202, 0.0, 0.036)),
        material=dark_steel,
        name="shaft_tip",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((0.360, 0.460, 0.110)),
        mass=0.55,
        origin=Origin(xyz=(0.010, 0.0, 0.048)),
    )

    model.articulation(
        "carriage_to_airframe",
        ArticulationType.FIXED,
        parent=carriage,
        child=airframe,
        origin=Origin(xyz=(0.095, 0.0, 0.037)),
    )

    propeller = model.part("propeller")
    spinner_geom = ConeGeometry(radius=0.013, height=0.028).rotate_y(math.pi / 2.0).translate(0.018, 0.0, 0.0)
    propeller.visual(
        Sphere(radius=0.0025),
        material=dark_steel,
        name="hub_pivot",
    )
    propeller.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="backplate",
    )
    propeller.visual(
        _save_mesh("prop_spinner", spinner_geom),
        material=prop_black,
        name="spinner",
    )
    propeller.visual(
        Box((0.004, 0.082, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(math.radians(12.0), 0.0, math.radians(6.0))),
        material=prop_black,
        name="blade_bar",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.032),
        mass=0.06,
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "prop_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.207, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    carriage = object_model.get_part("carriage")
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    stand_pitch = object_model.get_articulation("stand_pitch")
    prop_spin = object_model.get_articulation("prop_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.check(
        "explicit_joint_axes",
        stand_pitch.axis == (0.0, -1.0, 0.0) and prop_spin.axis == (1.0, 0.0, 0.0),
        details=f"stand_pitch axis={stand_pitch.axis}, prop_spin axis={prop_spin.axis}",
    )
    ctx.check(
        "joint_limits_are_plausible",
        stand_pitch.motion_limits is not None
        and prop_spin.motion_limits is not None
        and stand_pitch.motion_limits.lower is not None
        and stand_pitch.motion_limits.upper is not None
        and stand_pitch.motion_limits.lower < 0.0 < stand_pitch.motion_limits.upper,
        details="Expected a limited pitch cradle and an unlimited prop spin joint.",
    )

    with ctx.pose({stand_pitch: 0.0}):
        ctx.expect_contact(
            carriage,
            stand_base,
            elem_a="trunnion_barrel",
            elem_b="left_cheek",
            name="carriage_is_supported_by_trunnion_cheek",
        )
        ctx.expect_contact(
            airframe,
            carriage,
            elem_a="mount_shoe",
            elem_b="saddle_pad",
            name="airframe_mount_is_seated",
        )
        ctx.expect_gap(
            airframe,
            carriage,
            axis="z",
            positive_elem="mount_shoe",
            negative_elem="saddle_pad",
            max_gap=0.0005,
            max_penetration=0.0,
            name="mount_stack_has_controlled_seat_gap",
        )
        ctx.expect_contact(
            propeller,
            airframe,
            elem_a="hub_pivot",
            elem_b="shaft_tip",
            name="propeller_hub_is_supported",
        )
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="yz",
            min_overlap=0.020,
            elem_a="backplate",
            elem_b="nose_bulkhead",
            name="propeller_is_coaxial_with_nose",
        )
        ctx.expect_gap(
            stand_base,
            carriage,
            axis="y",
            positive_elem="left_zero_mark",
            negative_elem="pointer_tab",
            min_gap=0.0008,
            max_gap=0.0012,
            name="pointer_has_controlled_mark_gap",
        )
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="backplate",
            negative_elem="nose_bulkhead",
            min_gap=0.0054,
            max_gap=0.0066,
            name="backplate_bulkhead_clearance_gap",
        )
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            positive_elem="spinner",
            negative_elem="nose_bulkhead",
            min_gap=0.0065,
            max_gap=0.0085,
            name="spinner_nose_clearance_gap",
        )

    with ctx.pose({stand_pitch: stand_pitch.motion_limits.upper}):
        ctx.expect_gap(
            airframe,
            stand_base,
            axis="z",
            min_gap=0.001,
            name="pitched_airframe_clears_stand",
        )

    with ctx.pose({stand_pitch: 0.0}):
        rest_prop = ctx.part_world_position(propeller)
    with ctx.pose({stand_pitch: stand_pitch.motion_limits.upper}):
        pitched_prop = ctx.part_world_position(propeller)
    ctx.check(
        "positive_pitch_raises_nose",
        rest_prop is not None
        and pitched_prop is not None
        and pitched_prop[2] > rest_prop[2] + 0.05,
        details=f"rest_prop={rest_prop}, pitched_prop={pitched_prop}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

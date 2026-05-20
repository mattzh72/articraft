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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for z_pos, y_pos in rounded_rect_profile(height_z, width_y, radius)
    ]


def _build_fuselage_shell_mesh():
    z_center = 0.024
    return section_loft(
        [
            _yz_section(-0.106, 0.006, 0.006, 0.0015, z_center),
            _yz_section(-0.092, 0.014, 0.012, 0.0030, z_center),
            _yz_section(-0.074, 0.022, 0.018, 0.0045, z_center),
            _yz_section(-0.046, 0.030, 0.024, 0.0060, z_center),
            _yz_section(-0.006, 0.036, 0.030, 0.0080, z_center),
            _yz_section(0.036, 0.034, 0.028, 0.0070, z_center),
            _yz_section(0.072, 0.028, 0.024, 0.0060, z_center),
            _yz_section(0.098, 0.018, 0.016, 0.0040, z_center),
            _yz_section(0.112, 0.005, 0.005, 0.0010, z_center),
        ]
    )


def _build_canopy_mesh():
    return section_loft(
        [
            _yz_section(-0.006, 0.004, 0.004, 0.0010, 0.031),
            _yz_section(0.014, 0.012, 0.010, 0.0025, 0.034),
            _yz_section(0.034, 0.008, 0.006, 0.0015, 0.032),
        ]
    )


def _build_wing_mesh():
    wing_profile = [
        (0.054, 0.0),
        (0.022, 0.120),
        (-0.038, 0.110),
        (-0.052, 0.032),
        (-0.054, 0.0),
        (-0.052, -0.032),
        (-0.038, -0.110),
        (0.022, -0.120),
    ]
    return ExtrudeGeometry(wing_profile, 0.006, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_model_plane")

    body_red = model.material("body_red", rgba=(0.82, 0.16, 0.14, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.30, 0.33, 0.37, 1.0))
    canopy_clear = model.material("canopy_clear", rgba=(0.60, 0.76, 0.90, 0.70))
    prop_yellow = model.material("prop_yellow", rgba=(0.94, 0.79, 0.18, 1.0))

    stand = model.part("stand")
    stand_base_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.110, 0.070, 0.012), 0.010, center=True),
        "display_base",
    )
    stand.visual(
        stand_base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=stand_gray,
        name="display_base",
    )
    stand.visual(
        Cylinder(radius=0.005, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stand_gray,
        name="stand_stem",
    )
    stand.visual(
        Box((0.018, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=stand_gray,
        name="stand_crosshead",
    )
    stand.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.011, 0.101)),
        material=stand_gray,
        name="yoke_left",
    )
    stand.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.011, 0.101)),
        material=stand_gray,
        name="yoke_right",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.110, 0.070, 0.111)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0555)),
    )

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_geometry(_build_fuselage_shell_mesh(), "fuselage_shell"),
        material=body_red,
        name="fuselage_shell",
    )
    fuselage.visual(
        mesh_from_geometry(_build_canopy_mesh(), "canopy"),
        material=canopy_clear,
        name="canopy",
    )
    fuselage.visual(
        Box((0.018, 0.018, 0.010)),
        origin=Origin(),
        material=body_red,
        name="pivot_lug",
    )
    fuselage.visual(
        Box((0.024, 0.012, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.012)),
        material=body_red,
        name="pivot_pylon",
    )
    fuselage.visual(
        Box((0.034, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=body_red,
        name="wing_saddle",
    )
    fuselage.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.016, 0.041)),
        material=body_red,
        name="wing_clip_left",
    )
    fuselage.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.016, 0.041)),
        material=body_red,
        name="wing_clip_right",
    )
    fuselage.visual(
        Box((0.022, 0.016, 0.004)),
        origin=Origin(xyz=(-0.095, 0.0, 0.025)),
        material=body_red,
        name="tail_pad",
    )
    fuselage.visual(
        Box((0.014, 0.003, 0.006)),
        origin=Origin(xyz=(-0.094, 0.0070, 0.030)),
        material=body_red,
        name="tail_clip_left",
    )
    fuselage.visual(
        Box((0.014, 0.003, 0.006)),
        origin=Origin(xyz=(-0.094, -0.0070, 0.030)),
        material=body_red,
        name="tail_clip_right",
    )
    fuselage.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.1125, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="nose_bushing",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.224, 0.040, 0.060)),
        mass=0.18,
        origin=Origin(xyz=(0.003, 0.0, 0.024)),
    )

    wing = model.part("wing")
    wing.visual(
        mesh_from_geometry(_build_wing_mesh(), "wing_shell"),
        material=trim_gray,
        name="wing_shell",
    )
    wing.inertial = Inertial.from_geometry(
        Box((0.108, 0.240, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    tail = model.part("tail")
    tail.visual(
        Box((0.046, 0.090, 0.004)),
        origin=Origin(xyz=(-0.029, 0.0, 0.014)),
        material=trim_gray,
        name="tailplane",
    )
    tail.visual(
        Box((0.010, 0.012, 0.012)),
        origin=Origin(xyz=(-0.005, 0.0, 0.006)),
        material=trim_gray,
        name="tail_mount_block",
    )
    tail.visual(
        Box((0.024, 0.004, 0.028)),
        origin=Origin(xyz=(-0.022, 0.0, 0.030)),
        material=trim_gray,
        name="vertical_fin",
    )
    tail.inertial = Inertial.from_geometry(
        Box((0.052, 0.090, 0.044)),
        mass=0.025,
        origin=Origin(xyz=(-0.026, 0.0, 0.022)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_yellow,
        name="prop_hub",
    )
    propeller.visual(
        Box((0.004, 0.108, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.22, 0.0, 0.0)),
        material=prop_yellow,
        name="prop_blade_pair",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.016, 0.108, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_fuselage",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=fuselage,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=-0.30,
            upper=0.55,
        ),
    )
    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
    )
    model.articulation(
        "fuselage_to_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail,
        origin=Origin(xyz=(-0.106, 0.0, 0.027)),
    )
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.114, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    tail = object_model.get_part("tail")
    propeller = object_model.get_part("propeller")
    display_pitch = object_model.get_articulation("stand_to_fuselage")
    prop_spin = object_model.get_articulation("fuselage_to_propeller")

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

    ctx.expect_contact(fuselage, stand, name="stand_yoke_supports_fuselage")
    ctx.expect_contact(wing, fuselage, name="wing_seats_on_snap_saddle")
    ctx.expect_contact(tail, fuselage, name="tail_tongue_seats_in_rear_socket")
    ctx.expect_contact(propeller, fuselage, name="propeller_hub_seats_on_nose_bushing")
    ctx.expect_overlap(wing, fuselage, axes="xy", min_overlap=0.020, name="wing_root_centered_on_body")

    with ctx.pose({display_pitch: 0.55}):
        ctx.expect_origin_gap(
            propeller,
            tail,
            axis="z",
            min_gap=0.05,
            name="display_pitch_raises_nose",
        )

    with ctx.pose({prop_spin: math.pi / 2.0}):
        ctx.expect_contact(propeller, fuselage, name="propeller_spin_keeps_hub_supported")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

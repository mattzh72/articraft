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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    corner_radius = min(radius, width * 0.48, height * 0.48)
    return [
        (x_pos, y_pos, z_pos)
        for y_pos, z_pos in rounded_rect_profile(
            width,
            height,
            corner_radius,
            corner_segments=8,
        )
    ]


def _airfoil_section(
    y_pos: float,
    chord: float,
    thickness: float,
    *,
    x_offset: float = 0.0,
    camber: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_offset + chord * 0.50, y_pos, camber),
        (x_offset + chord * 0.18, y_pos, camber + thickness * 0.52),
        (x_offset - chord * 0.12, y_pos, camber + thickness * 0.30),
        (x_offset - chord * 0.50, y_pos, camber),
        (x_offset - chord * 0.18, y_pos, camber - thickness * 0.56),
        (x_offset + chord * 0.20, y_pos, camber - thickness * 0.34),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_toy_utility_plane", assets=ASSETS)

    molded_olive = model.material("molded_olive", rgba=(0.33, 0.41, 0.22, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.84, 0.42, 0.10, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.52, 0.62, 0.45))
    pale_grey = model.material("pale_grey", rgba=(0.78, 0.79, 0.80, 1.0))

    fuselage_shell_mesh = _save_mesh(
        "rugged_plane_fuselage_shell.obj",
        section_loft(
            [
                _rounded_yz_section(0.136, 0.022, 0.024, 0.010),
                _rounded_yz_section(0.094, 0.062, 0.066, 0.024),
                _rounded_yz_section(0.018, 0.074, 0.084, 0.028),
                _rounded_yz_section(-0.066, 0.068, 0.076, 0.024),
                _rounded_yz_section(-0.146, 0.040, 0.050, 0.014),
                _rounded_yz_section(-0.186, 0.020, 0.028, 0.008),
            ]
        ),
    )
    wing_shell_mesh = _save_mesh(
        "rugged_plane_wing_shell.obj",
        section_loft(
            [
                _airfoil_section(-0.170, 0.056, 0.011, x_offset=-0.003, camber=0.0008),
                _airfoil_section(-0.070, 0.070, 0.016, x_offset=-0.006, camber=0.0010),
                _airfoil_section(0.070, 0.070, 0.016, x_offset=-0.006, camber=0.0010),
                _airfoil_section(0.170, 0.056, 0.011, x_offset=-0.003, camber=0.0008),
            ]
        ),
    )
    tailplane_mesh = _save_mesh(
        "rugged_plane_tailplane.obj",
        section_loft(
            [
                _airfoil_section(-0.075, 0.024, 0.007, x_offset=-0.022, camber=0.0004),
                _airfoil_section(-0.018, 0.037, 0.010, x_offset=-0.0155, camber=0.0006),
                _airfoil_section(0.018, 0.037, 0.010, x_offset=-0.0155, camber=0.0006),
                _airfoil_section(0.075, 0.024, 0.007, x_offset=-0.022, camber=0.0004),
            ]
        ),
    )

    fuselage = model.part("fuselage")
    fuselage.visual(
        fuselage_shell_mesh,
        material=molded_olive,
        name="fuselage_shell",
    )
    fuselage.visual(
        Cylinder(radius=0.032, length=0.064),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=molded_olive,
        name="engine_cowling",
    )
    fuselage.visual(
        Box((0.090, 0.040, 0.050)),
        origin=Origin(xyz=(-0.145, 0.0, -0.002)),
        material=molded_olive,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.070, 0.052, 0.032)),
        origin=Origin(xyz=(-0.005, 0.0, 0.038)),
        material=molded_olive,
        name="cabin_roof",
    )
    fuselage.visual(
        Box((0.036, 0.050, 0.019)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0455)),
        material=molded_olive,
        name="wing_pylon",
    )
    fuselage.visual(
        Box((0.090, 0.050, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.044)),
        material=matte_black,
        name="belly_skid",
    )
    fuselage.visual(
        Box((0.030, 0.002, 0.022)),
        origin=Origin(xyz=(0.010, -0.032, 0.018)),
        material=smoked_glass,
        name="left_window",
    )
    fuselage.visual(
        Box((0.030, 0.002, 0.022)),
        origin=Origin(xyz=(0.010, 0.032, 0.018)),
        material=smoked_glass,
        name="right_window",
    )
    fuselage.visual(
        Box((0.034, 0.034, 0.002)),
        origin=Origin(xyz=(0.030, 0.0, 0.037)),
        material=warning_orange,
        name="roof_panel",
    )
    fuselage.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.045, -0.026, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="left_exhaust_stack",
    )
    fuselage.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.045, 0.026, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="right_exhaust_stack",
    )
    fuselage.visual(
        Box((0.010, 0.014, 0.014)),
        origin=Origin(xyz=(0.071, -0.026, -0.005)),
        material=steel,
        name="left_exhaust_mount_block",
    )
    fuselage.visual(
        Box((0.010, 0.014, 0.014)),
        origin=Origin(xyz=(0.071, 0.026, -0.005)),
        material=steel,
        name="right_exhaust_mount_block",
    )
    fuselage.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.072, 0.0, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="nose_bumper",
    )
    fuselage.visual(
        Box((0.010, 0.120, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, -0.022)),
        material=steel,
        name="gear_spreader",
    )
    fuselage.visual(
        Box((0.012, 0.010, 0.055)),
        origin=Origin(xyz=(-0.020, -0.060, -0.0365)),
        material=steel,
        name="left_main_gear_leg",
    )
    fuselage.visual(
        Box((0.012, 0.010, 0.055)),
        origin=Origin(xyz=(-0.020, 0.060, -0.0365)),
        material=steel,
        name="right_main_gear_leg",
    )
    fuselage.visual(
        Cylinder(radius=0.025, length=0.016),
        origin=Origin(xyz=(-0.020, -0.060, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="left_wheel",
    )
    fuselage.visual(
        Cylinder(radius=0.025, length=0.016),
        origin=Origin(xyz=(-0.020, 0.060, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="right_wheel",
    )
    fuselage.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.020, -0.060, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_grey,
        name="left_wheel_hub",
    )
    fuselage.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.020, 0.060, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_grey,
        name="right_wheel_hub",
    )
    fuselage.visual(
        Box((0.032, 0.010, 0.040)),
        origin=Origin(xyz=(-0.160, 0.0, -0.028)),
        material=steel,
        name="tail_wheel_strut",
    )
    fuselage.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.160, 0.0, -0.053), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="tail_wheel",
    )

    for y_pos in (-0.017, 0.017):
        fuselage.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(0.125, y_pos, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"cowling_bolt_{'left' if y_pos < 0.0 else 'right'}_upper",
        )
        fuselage.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(0.125, y_pos, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"cowling_bolt_{'left' if y_pos < 0.0 else 'right'}_lower",
        )

    for y_pos in (-0.022, 0.022):
        fuselage.visual(
            Cylinder(radius=0.003, length=0.003),
            origin=Origin(xyz=(-0.004, y_pos, 0.056), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"wing_root_fastener_{'left' if y_pos < 0.0 else 'right'}",
        )

    for side in (-1.0, 1.0):
        y_sign = "left" if side < 0.0 else "right"
        fuselage.visual(
            Box((0.012, 0.046, 0.028)),
            origin=Origin(
                xyz=(-0.010, side * 0.054, 0.031),
            ),
            material=steel,
            name=f"{y_sign}_wing_strut",
        )

    fuselage.inertial = Inertial.from_geometry(
        Box((0.33, 0.14, 0.13)),
        mass=0.80,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    wing = model.part("wing")
    wing.visual(
        wing_shell_mesh,
        material=warning_orange,
        name="main_wing_shell",
    )
    wing.visual(
        Box((0.020, 0.180, 0.008)),
        origin=Origin(xyz=(-0.022, 0.0, -0.005)),
        material=molded_olive,
        name="wing_trailing_reinforcement",
    )
    wing.visual(
        Box((0.020, 0.030, 0.022)),
        origin=Origin(xyz=(0.000, -0.155, 0.002)),
        material=molded_olive,
        name="left_tip_block",
    )
    wing.visual(
        Box((0.020, 0.030, 0.022)),
        origin=Origin(xyz=(0.000, 0.155, 0.002)),
        material=molded_olive,
        name="right_tip_block",
    )
    wing.visual(
        Box((0.030, 0.050, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.014)),
        material=matte_black,
        name="wing_center_spar_cap",
    )
    wing.inertial = Inertial.from_geometry(
        Box((0.078, 0.340, 0.022)),
        mass=0.22,
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
    )

    left_aileron = model.part("left_aileron")
    left_aileron.visual(
        Box((0.028, 0.110, 0.012)),
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
        material=warning_orange,
        name="left_aileron_surface",
    )
    left_aileron.visual(
        Cylinder(radius=0.003, length=0.104),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_aileron_hinge_barrel",
    )
    left_aileron.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0, -0.007)),
        material=matte_black,
        name="left_aileron_trim_tab",
    )
    left_aileron.inertial = Inertial.from_geometry(
        Box((0.032, 0.110, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )

    right_aileron = model.part("right_aileron")
    right_aileron.visual(
        Box((0.028, 0.110, 0.012)),
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
        material=warning_orange,
        name="right_aileron_surface",
    )
    right_aileron.visual(
        Cylinder(radius=0.003, length=0.104),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_aileron_hinge_barrel",
    )
    right_aileron.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0, -0.007)),
        material=matte_black,
        name="right_aileron_trim_tab",
    )
    right_aileron.inertial = Inertial.from_geometry(
        Box((0.032, 0.110, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )

    tail = model.part("tail")
    tail.visual(
        tailplane_mesh,
        material=molded_olive,
        name="horizontal_stabilizer",
    )
    tail.visual(
        Box((0.026, 0.010, 0.064)),
        origin=Origin(xyz=(-0.010, 0.0, 0.038)),
        material=molded_olive,
        name="vertical_fin",
    )
    tail.visual(
        Box((0.022, 0.040, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.006)),
        material=steel,
        name="tail_root_reinforcement",
    )
    tail.visual(
        Box((0.010, 0.012, 0.036)),
        origin=Origin(xyz=(-0.016, 0.0, 0.020)),
        material=matte_black,
        name="tail_service_post",
    )
    tail.inertial = Inertial.from_geometry(
        Box((0.050, 0.150, 0.078)),
        mass=0.10,
        origin=Origin(xyz=(-0.018, 0.0, 0.028)),
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Box((0.026, 0.056, 0.010)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=warning_orange,
        name="left_elevator_surface",
    )
    left_elevator.visual(
        Cylinder(radius=0.003, length=0.052),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_elevator_hinge_barrel",
    )
    left_elevator.inertial = Inertial.from_geometry(
        Box((0.028, 0.056, 0.012)),
        mass=0.018,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Box((0.026, 0.056, 0.010)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=warning_orange,
        name="right_elevator_surface",
    )
    right_elevator.visual(
        Cylinder(radius=0.003, length=0.052),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_elevator_hinge_barrel",
    )
    right_elevator.inertial = Inertial.from_geometry(
        Box((0.028, 0.056, 0.012)),
        mass=0.018,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.022, 0.008, 0.056)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=warning_orange,
        name="rudder_surface",
    )
    rudder.visual(
        Cylinder(radius=0.003, length=0.054),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=steel,
        name="rudder_hinge_barrel",
    )
    rudder.inertial = Inertial.from_geometry(
        Box((0.024, 0.010, 0.056)),
        mass=0.02,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="propeller_shaft",
    )
    propeller.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=matte_black,
        name="propeller_hub",
    )
    for blade_index, roll in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        propeller.visual(
            Box((0.008, 0.096, 0.016)),
            origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(roll, 0.0, 0.0)),
            material=matte_black,
            name=f"blade_{blade_index}",
        )
    propeller.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub_retainer",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.032, 0.100, 0.100)),
        mass=0.025,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=Origin(xyz=(-0.004, 0.0, 0.064)),
    )
    model.articulation(
        "wing_to_left_aileron",
        ArticulationType.REVOLUTE,
        parent=wing,
        child=left_aileron,
        origin=Origin(xyz=(-0.039, -0.112, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "wing_to_right_aileron",
        ArticulationType.REVOLUTE,
        parent=wing,
        child=right_aileron,
        origin=Origin(xyz=(-0.039, 0.112, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    model.articulation(
        "fuselage_to_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail,
        origin=Origin(xyz=(-0.190, 0.0, 0.010)),
    )
    model.articulation(
        "tail_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=tail,
        child=left_elevator,
        origin=Origin(xyz=(-0.031, -0.047, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=1.5,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "tail_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=tail,
        child=right_elevator,
        origin=Origin(xyz=(-0.031, 0.047, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=1.5,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "tail_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail,
        child=rudder,
        origin=Origin(xyz=(-0.020, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=1.5,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.152, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    left_aileron = object_model.get_part("left_aileron")
    right_aileron = object_model.get_part("right_aileron")
    tail = object_model.get_part("tail")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    rudder = object_model.get_part("rudder")
    propeller = object_model.get_part("propeller")

    wing_joint = object_model.get_articulation("fuselage_to_wing")
    left_aileron_joint = object_model.get_articulation("wing_to_left_aileron")
    right_aileron_joint = object_model.get_articulation("wing_to_right_aileron")
    tail_joint = object_model.get_articulation("fuselage_to_tail")
    left_elevator_joint = object_model.get_articulation("tail_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("tail_to_right_elevator")
    rudder_joint = object_model.get_articulation("tail_to_rudder")
    propeller_joint = object_model.get_articulation("fuselage_to_propeller")

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(contact_tol=0.0012)
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
        "plane_joint_axes",
        tuple(round(v, 3) for v in left_aileron_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in right_aileron_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in left_elevator_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in right_elevator_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in rudder_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 3) for v in propeller_joint.axis) == (1.0, 0.0, 0.0),
        details="Control surfaces or propeller use the wrong articulation axis.",
    )

    ctx.expect_contact(wing, fuselage, name="wing_root_contact")
    ctx.expect_contact(tail, fuselage, name="tail_mount_contact")
    ctx.expect_contact(propeller, fuselage, name="propeller_shaft_contact")
    ctx.expect_contact(left_aileron, wing, contact_tol=0.0012, name="left_aileron_contact")
    ctx.expect_contact(right_aileron, wing, contact_tol=0.0012, name="right_aileron_contact")
    ctx.expect_contact(left_elevator, tail, contact_tol=0.0012, name="left_elevator_contact")
    ctx.expect_contact(right_elevator, tail, contact_tol=0.0012, name="right_elevator_contact")
    ctx.expect_contact(rudder, tail, contact_tol=0.0012, name="rudder_contact")

    ctx.expect_overlap(wing, fuselage, axes="xy", min_overlap=0.030, name="wing_over_root")
    ctx.expect_overlap(tail, fuselage, axes="yz", min_overlap=0.020, name="tail_overlap_root")
    ctx.expect_within(left_aileron, wing, axes="y", margin=0.005, name="left_aileron_span_within")
    ctx.expect_within(right_aileron, wing, axes="y", margin=0.005, name="right_aileron_span_within")
    ctx.expect_within(left_elevator, tail, axes="y", margin=0.005, name="left_elevator_span_within")
    ctx.expect_within(right_elevator, tail, axes="y", margin=0.005, name="right_elevator_span_within")
    ctx.expect_within(rudder, tail, axes="z", margin=0.005, name="rudder_height_within")
    ctx.expect_origin_distance(propeller, fuselage, axes="yz", max_dist=0.001, name="propeller_centered")

    ctx.check(
        "fixed_mount_parts_present",
        wing_joint.parent == "fuselage"
        and wing_joint.child == "wing"
        and tail_joint.parent == "fuselage"
        and tail_joint.child == "tail",
        details="Fixed wing or tail mount is not wired to the fuselage.",
    )

    control_joints = (
        left_aileron_joint,
        right_aileron_joint,
        left_elevator_joint,
        right_elevator_joint,
        rudder_joint,
    )
    for joint, parent_part, child_part in (
        (left_aileron_joint, wing, left_aileron),
        (right_aileron_joint, wing, right_aileron),
        (left_elevator_joint, tail, left_elevator),
        (right_elevator_joint, tail, right_elevator),
        (rudder_joint, tail, rudder),
    ):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0012, name=f"{joint.name}_lower_no_floating")
            ctx.expect_contact(child_part, parent_part, contact_tol=0.0012, name=f"{joint.name}_lower_contact")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0012, name=f"{joint.name}_upper_no_floating")
            ctx.expect_contact(child_part, parent_part, contact_tol=0.0012, name=f"{joint.name}_upper_contact")

    for angle, label in ((0.0, "rest"), (math.pi / 2.0, "quarter_turn"), (math.pi, "half_turn")):
        with ctx.pose({propeller_joint: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"propeller_{label}_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0012, name=f"propeller_{label}_no_floating")
            ctx.expect_contact(propeller, fuselage, name=f"propeller_{label}_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

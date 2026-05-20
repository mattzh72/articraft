from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _z_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stick_vacuum")

    painted_metal = model.material("painted_metal", rgba=(0.74, 0.73, 0.70, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.17, 0.18, 0.20, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.42, 0.47, 0.52, 0.34))
    elastomer = model.material("elastomer", rgba=(0.08, 0.08, 0.09, 1.0))
    bright_hardware = model.material("bright_hardware", rgba=(0.68, 0.70, 0.72, 1.0))

    body_shell_mesh = _mesh(
        "vacuum_body_shell",
        section_loft(
            [
                _z_section(0.076, 0.058, 0.018, 0.342, x_shift=0.010),
                _z_section(0.114, 0.076, 0.024, 0.468, x_shift=0.032),
                _z_section(0.100, 0.070, 0.022, 0.628, x_shift=0.018),
                _z_section(0.066, 0.050, 0.016, 0.802, x_shift=-0.008),
            ]
        ),
    )
    dust_cup_mesh = _mesh(
        "vacuum_dust_cup",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.024, -0.058),
                (0.029, -0.052),
                (0.032, 0.030),
                (0.031, 0.050),
                (0.026, 0.060),
            ],
            inner_profile=[
                (0.021, -0.051),
                (0.028, 0.028),
                (0.027, 0.047),
                (0.023, 0.053),
            ],
            segments=56,
        ),
    )
    upper_wand_mesh = _mesh(
        "upper_wand_tube",
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.018),
                (0.004, 0.000, 0.170),
                (0.012, 0.000, 0.340),
            ],
            radius=0.017,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    lower_wand_mesh = _mesh(
        "lower_wand_tube",
        tube_from_spline_points(
            [
                (0.004, 0.000, -0.018),
                (0.010, 0.000, -0.210),
                (0.016, 0.000, -0.405),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    floor_head_shell_mesh = _mesh(
        "floor_head_shell",
        section_loft(
            [
                _z_section(0.066, 0.040, 0.012, 0.002, x_shift=0.046),
                _z_section(0.204, 0.068, 0.018, -0.010, x_shift=0.114),
                _z_section(0.270, 0.086, 0.021, -0.024, x_shift=0.146),
                _z_section(0.286, 0.092, 0.022, -0.036, x_shift=0.152),
            ]
        ),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(upper_wand_mesh, material=wand_metal, name="upper_wand_tube")
    upper_assembly.visual(
        Cylinder(radius=0.022, length=0.080),
        origin=Origin(xyz=(0.000, 0.000, 0.347)),
        material=graphite_polymer,
        name="wand_socket",
    )
    upper_assembly.visual(body_shell_mesh, material=painted_metal, name="main_body_shell")
    upper_assembly.visual(
        dust_cup_mesh,
        origin=Origin(xyz=(0.066, 0.000, 0.492)),
        material=smoke_clear,
        name="dust_cup_shell",
    )
    upper_assembly.visual(
        Box((0.050, 0.052, 0.050)),
        origin=Origin(xyz=(0.038, 0.000, 0.484)),
        material=graphite_polymer,
        name="dust_cup_bridge",
    )
    upper_assembly.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.066, 0.000, 0.550)),
        material=painted_metal,
        name="cyclone_band",
    )
    upper_assembly.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(-0.016, 0.000, 0.817)),
        material=graphite_polymer,
        name="rear_filter_cap",
    )
    upper_assembly.visual(
        Box((0.024, 0.040, 0.180)),
        origin=Origin(xyz=(-0.026, 0.000, 0.670)),
        material=elastomer,
        name="handle_grip",
    )
    upper_assembly.visual(
        Box((0.028, 0.046, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.027)),
        material=graphite_polymer,
        name="fold_bridge",
    )
    upper_assembly.visual(
        Box((0.012, 0.008, 0.032)),
        origin=Origin(xyz=(0.000, 0.023, 0.004)),
        material=graphite_polymer,
        name="fold_cheek_left",
    )
    upper_assembly.visual(
        Box((0.012, 0.008, 0.032)),
        origin=Origin(xyz=(0.000, -0.023, 0.004)),
        material=graphite_polymer,
        name="fold_cheek_right",
    )
    upper_assembly.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.000, 0.029, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="fold_pin_cap_left",
    )
    upper_assembly.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.000, -0.029, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="fold_pin_cap_right",
    )
    upper_assembly.inertial = Inertial.from_geometry(
        Box((0.180, 0.100, 0.880)),
        mass=2.2,
        origin=Origin(xyz=(0.020, 0.000, 0.440)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.0105, length=0.038),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="fold_hinge_barrel",
    )
    lower_wand.visual(
        Box((0.024, 0.026, 0.028)),
        origin=Origin(xyz=(0.007, 0.000, -0.020)),
        material=graphite_polymer,
        name="fold_hinge_web",
    )
    lower_wand.visual(lower_wand_mesh, material=wand_metal, name="lower_wand_tube")
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(0.014, 0.000, -0.390)),
        material=graphite_polymer,
        name="nozzle_cuff",
    )
    lower_wand.visual(
        Box((0.028, 0.046, 0.016)),
        origin=Origin(xyz=(0.014, 0.000, -0.406)),
        material=graphite_polymer,
        name="nozzle_bridge",
    )
    lower_wand.visual(
        Box((0.010, 0.008, 0.032)),
        origin=Origin(xyz=(0.014, 0.023, -0.430)),
        material=graphite_polymer,
        name="nozzle_fork_left",
    )
    lower_wand.visual(
        Box((0.010, 0.008, 0.032)),
        origin=Origin(xyz=(0.014, -0.023, -0.430)),
        material=graphite_polymer,
        name="nozzle_fork_right",
    )
    lower_wand.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.014, 0.029, -0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="nozzle_pin_cap_left",
    )
    lower_wand.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.014, -0.029, -0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="nozzle_pin_cap_right",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.520)),
        mass=0.55,
        origin=Origin(xyz=(0.012, 0.000, -0.240)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.0105, length=0.038),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_hardware,
        name="neck_barrel",
    )
    floor_head.visual(
        Box((0.026, 0.026, 0.036)),
        origin=Origin(xyz=(0.014, 0.000, -0.018)),
        material=graphite_polymer,
        name="neck_tower",
    )
    floor_head.visual(floor_head_shell_mesh, material=graphite_polymer, name="head_shell")
    floor_head.visual(
        Box((0.116, 0.010, 0.008)),
        origin=Origin(xyz=(0.144, 0.000, -0.009)),
        material=painted_metal,
        name="head_accent_strip",
    )
    floor_head.visual(
        Box((0.064, 0.094, 0.010)),
        origin=Origin(xyz=(0.186, 0.000, -0.030)),
        material=elastomer,
        name="front_bumper",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.290, 0.100, 0.050)),
        mass=0.85,
        origin=Origin(xyz=(0.080, 0.000, -0.020)),
    )

    model.articulation(
        "upper_to_lower_fold",
        ArticulationType.REVOLUTE,
        parent=upper_assembly,
        child=lower_wand,
        origin=Origin(),
        # The lower wand extends along local -Z from the fold axis.
        # -Y makes positive q swing the nozzle forward toward +X.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "lower_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.014, 0.000, -0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.45,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_assembly = object_model.get_part("upper_assembly")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("upper_to_lower_fold")
    nozzle = object_model.get_articulation("lower_to_floor_head")

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

    ctx.check(
        "primary parts exist",
        all(part is not None for part in (upper_assembly, lower_wand, floor_head)),
        "Upper assembly, lower wand, and floor head must all be present.",
    )
    ctx.check(
        "fold joint uses forward hinge convention",
        fold.axis == (0.0, -1.0, 0.0)
        and fold.motion_limits is not None
        and fold.motion_limits.lower == 0.0
        and fold.motion_limits.upper is not None
        and fold.motion_limits.upper >= 1.0,
        "Fold joint should hinge about +Y from a straight wand pose into a forward fold.",
    )
    ctx.check(
        "nozzle joint has usable pitch range",
        nozzle.axis == (0.0, -1.0, 0.0)
        and nozzle.motion_limits is not None
        and nozzle.motion_limits.lower is not None
        and nozzle.motion_limits.upper is not None
        and nozzle.motion_limits.lower < 0.0 < nozzle.motion_limits.upper,
        "Floor head should pitch around a lateral pin with both down and up travel.",
    )
    ctx.expect_contact(
        upper_assembly,
        lower_wand,
        elem_a="fold_cheek_left",
        elem_b="fold_hinge_barrel",
        name="fold pin is bracket-supported",
    )
    ctx.expect_contact(
        lower_wand,
        floor_head,
        elem_a="nozzle_fork_left",
        elem_b="neck_barrel",
        name="nozzle pin is bracket-supported",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold: 0.95}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint advances floor head forward",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > rest_head_pos[0] + 0.22,
        "Opening the fold joint should move the floor head noticeably forward.",
    )

    rest_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    with ctx.pose({nozzle: 0.50}):
        pitched_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    ctx.check(
        "nozzle articulation lifts the nose",
        rest_bumper is not None
        and pitched_bumper is not None
        and pitched_bumper[0][2] > rest_bumper[0][2] + 0.03,
        "Positive nozzle pitch should lift the front bumper upward.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

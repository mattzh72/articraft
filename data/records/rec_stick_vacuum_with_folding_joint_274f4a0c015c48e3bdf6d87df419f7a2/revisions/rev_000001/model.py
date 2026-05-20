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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width: float,
    depth: float,
    z: float,
    radius: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _yz_section(
    width: float,
    height: float,
    x: float,
    radius: float,
    *,
    y_shift: float = 0.0,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z + z_shift)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_stick_vacuum")

    shell_gray = model.material("shell_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    wand_gray = model.material("wand_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    wear_black = model.material("wear_black", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    service_orange = model.material("service_orange", rgba=(0.88, 0.47, 0.12, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.72, 0.79, 0.84, 0.35))

    body = model.part("body_module")
    body_shell = section_loft(
        [
            _xy_section(0.090, 0.072, 0.045, 0.018, x_shift=-0.006),
            _xy_section(0.102, 0.086, 0.180, 0.020, x_shift=-0.010),
            _xy_section(0.088, 0.074, 0.340, 0.018, x_shift=-0.014),
            _xy_section(0.060, 0.050, 0.480, 0.013, x_shift=-0.008),
        ]
    )
    handle_loop = tube_from_spline_points(
        [
            (-0.032, 0.0, 0.320),
            (-0.046, 0.0, 0.385),
            (-0.030, 0.0, 0.505),
            (0.008, 0.0, 0.525),
            (0.045, 0.0, 0.420),
            (0.030, 0.0, 0.335),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    body.visual(_mesh("body_shell", body_shell), material=shell_gray, name="body_shell")
    body.visual(
        Box((0.028, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, 0.023, 0.018)),
        material=steel,
        name="hinge_plate_pos",
    )
    body.visual(
        Box((0.028, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, -0.023, 0.018)),
        material=steel,
        name="hinge_plate_neg",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_orange,
        name="hinge_pin_cap_pos",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_orange,
        name="hinge_pin_cap_neg",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.200),
        origin=Origin(xyz=(0.034, 0.0, 0.228)),
        material=clear_smoke,
        name="dust_bin",
    )
    body.visual(
        Box((0.072, 0.070, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, 0.128)),
        material=shell_gray,
        name="cyclone_collar",
    )
    body.visual(
        Box((0.056, 0.060, 0.260)),
        origin=Origin(xyz=(-0.022, 0.0, 0.300)),
        material=charcoal,
        name="rear_spine",
    )
    body.visual(
        Box((0.082, 0.058, 0.140)),
        origin=Origin(xyz=(-0.050, 0.0, 0.158)),
        material=charcoal,
        name="battery_pack",
    )
    body.visual(
        Box((0.024, 0.030, 0.020)),
        origin=Origin(xyz=(-0.088, 0.0, 0.232)),
        material=service_orange,
        name="battery_release",
    )
    body.visual(
        Box((0.050, 0.056, 0.012)),
        origin=Origin(xyz=(0.044, 0.0, 0.380)),
        material=service_orange,
        name="service_hatch",
    )
    body.visual(_mesh("body_handle_loop", handle_loop), material=charcoal, name="handle_loop")
    body.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(-0.002, 0.0, 0.500)),
        material=charcoal,
        name="top_exhaust",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.150, 0.100, 0.560)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
    )

    wand = model.part("folding_wand")
    wand.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    wand.visual(
        Box((0.036, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=service_orange,
        name="fold_collar",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, -0.260)),
        material=wand_gray,
        name="wand_tube",
    )
    wand.visual(
        Box((0.018, 0.022, 0.360)),
        origin=Origin(xyz=(-0.020, 0.0, -0.270)),
        material=charcoal,
        name="service_rail",
    )
    wand.visual(
        Box((0.018, 0.034, 0.026)),
        origin=Origin(xyz=(0.023, 0.0, -0.070)),
        material=service_orange,
        name="fold_release",
    )
    wand.visual(
        Box((0.042, 0.050, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.490)),
        material=charcoal,
        name="neck_mount",
    )
    wand.visual(
        Box((0.028, 0.008, 0.054)),
        origin=Origin(xyz=(0.0, 0.026, -0.545)),
        material=steel,
        name="neck_plate_pos",
    )
    wand.visual(
        Box((0.028, 0.008, 0.054)),
        origin=Origin(xyz=(0.0, -0.026, -0.545)),
        material=steel,
        name="neck_plate_neg",
    )
    wand.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.033, -0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_orange,
        name="neck_pin_cap_pos",
    )
    wand.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.033, -0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_orange,
        name="neck_pin_cap_neg",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.580)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
    )

    head = model.part("floor_head")
    head_shell = section_loft(
        [
            _yz_section(0.140, 0.052, -0.020, 0.014, z_shift=-0.078),
            _yz_section(0.220, 0.056, 0.050, 0.018, z_shift=-0.082),
            _yz_section(0.260, 0.050, 0.150, 0.016, z_shift=-0.084),
            _yz_section(0.210, 0.036, 0.240, 0.012, z_shift=-0.086),
        ]
    )
    head.visual(
        Cylinder(radius=0.013, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    head.visual(
        Box((0.040, 0.044, 0.060)),
        origin=Origin(xyz=(0.018, 0.0, -0.030)),
        material=charcoal,
        name="pivot_block",
    )
    head.visual(
        Box((0.060, 0.080, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, -0.055)),
        material=service_orange,
        name="neck_cover",
    )
    head.visual(
        _mesh("head_shell", head_shell),
        material=charcoal,
        name="head_shell",
    )
    head.visual(
        Box((0.084, 0.160, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, -0.056)),
        material=wand_gray,
        name="upper_hump",
    )
    head.visual(
        Box((0.032, 0.240, 0.028)),
        origin=Origin(xyz=(0.165, 0.0, -0.078)),
        material=wear_black,
        name="front_bumper",
    )
    head.visual(
        Box((0.090, 0.180, 0.040)),
        origin=Origin(xyz=(0.108, 0.0, -0.086)),
        material=charcoal,
        name="bumper_carrier",
    )
    head.visual(
        Box((0.052, 0.220, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, -0.080)),
        material=wear_black,
        name="rear_skid",
    )
    head.visual(
        Box((0.230, 0.185, 0.010)),
        origin=Origin(xyz=(0.066, 0.0, -0.106)),
        material=wear_black,
        name="service_plate",
    )
    head.visual(
        Box((0.250, 0.014, 0.010)),
        origin=Origin(xyz=(0.070, 0.090, -0.105)),
        material=wear_black,
        name="skid_left",
    )
    head.visual(
        Box((0.250, 0.014, 0.010)),
        origin=Origin(xyz=(0.070, -0.090, -0.105)),
        material=wear_black,
        name="skid_right",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.036, 0.112, -0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_black,
        name="rear_wheel_pos",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.036, -0.112, -0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_black,
        name="rear_wheel_neg",
    )
    head.visual(
        Box((0.030, 0.040, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.042)),
        material=service_orange,
        name="brush_latch",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.310, 0.270, 0.110)),
        mass=1.0,
        origin=Origin(xyz=(0.080, 0.0, -0.070)),
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, -0.545)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=math.radians(-22.0),
            upper=math.radians(32.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_module")
    wand = object_model.get_part("folding_wand")
    head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("fold_joint")
    nozzle_pitch = object_model.get_articulation("nozzle_pitch")

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
        "fold_joint_axis",
        tuple(fold_joint.axis) == (0.0, -1.0, 0.0),
        f"expected fold axis (0,-1,0), got {fold_joint.axis}",
    )
    ctx.check(
        "nozzle_pitch_axis",
        tuple(nozzle_pitch.axis) == (0.0, -1.0, 0.0),
        f"expected nozzle axis (0,-1,0), got {nozzle_pitch.axis}",
    )
    ctx.expect_contact(
        body,
        wand,
        elem_a="hinge_plate_pos",
        elem_b="hinge_barrel",
        contact_tol=1e-5,
        name="fold_joint_contact_pos",
    )
    ctx.expect_contact(
        body,
        wand,
        elem_a="hinge_plate_neg",
        elem_b="hinge_barrel",
        contact_tol=1e-5,
        name="fold_joint_contact_neg",
    )
    ctx.expect_contact(
        wand,
        head,
        elem_a="neck_plate_pos",
        elem_b="pivot_barrel",
        contact_tol=1e-5,
        name="nozzle_joint_contact_pos",
    )
    ctx.expect_contact(
        wand,
        head,
        elem_a="neck_plate_neg",
        elem_b="pivot_barrel",
        contact_tol=1e-5,
        name="nozzle_joint_contact_neg",
    )
    ctx.expect_gap(
        body,
        head,
        axis="z",
        min_gap=0.45,
        name="upright_body_clears_head",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({fold_joint: math.radians(78.0)}):
        folded_head_pos = ctx.part_world_position(head)
        ctx.expect_gap(
            head,
            body,
            axis="x",
            min_gap=0.40,
            name="folded_head_swings_forward_of_body",
        )
    ctx.check(
        "fold_joint_swings_head_forward",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > rest_head_pos[0] + 0.48,
        f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    flat_front = ctx.part_element_world_aabb(head, elem="front_bumper")
    with ctx.pose({nozzle_pitch: math.radians(22.0)}):
        pitched_front = ctx.part_element_world_aabb(head, elem="front_bumper")
    ctx.check(
        "nozzle_pitch_lifts_front_bumper",
        flat_front is not None
        and pitched_front is not None
        and pitched_front[1][2] > flat_front[1][2] + 0.04,
        f"flat={flat_front}, pitched={pitched_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

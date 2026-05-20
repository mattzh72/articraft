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
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_fasteners(
    part,
    *,
    prefix: str,
    centers: list[tuple[float, float, float]],
    axis: str,
    radius: float,
    length: float,
    material,
) -> None:
    rpy = _axis_rpy(axis)
    for index, center in enumerate(centers):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=rpy),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_fold_stick_vacuum")

    shell_dark = model.material("shell_dark", rgba=(0.17, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.47, 0.49, 0.50, 1.0))
    fastener = model.material("fastener", rgba=(0.72, 0.73, 0.74, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.71, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.09, 1.0))

    upper_body = model.part("upper_body")
    upper_body.inertial = Inertial.from_geometry(
        Box((0.22, 0.12, 0.78)),
        mass=3.8,
        origin=Origin(xyz=(0.06, 0.0, 0.38)),
    )
    body_shell_geom = section_loft(
        [
            _xy_section(0.112, 0.082, 0.018, 0.120, x_shift=0.070),
            _xy_section(0.118, 0.086, 0.019, 0.280, x_shift=0.072),
            _xy_section(0.142, 0.092, 0.024, 0.470, x_shift=0.082),
            _xy_section(0.120, 0.084, 0.020, 0.610, x_shift=0.080),
        ]
    )
    upper_body.visual(
        _mesh("vacuum_body_shell", body_shell_geom),
        material=shell_dark,
        name="body_shell",
    )
    upper_body.visual(
        Box((0.046, 0.030, 0.290)),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=steel,
        name="upper_wand_tube",
    )
    upper_body.visual(
        Box((0.006, 0.026, 0.220)),
        origin=Origin(xyz=(0.026, 0.0, 0.195)),
        material=steel,
        name="wand_strap_right",
    )
    upper_body.visual(
        Box((0.006, 0.026, 0.220)),
        origin=Origin(xyz=(-0.026, 0.0, 0.195)),
        material=steel,
        name="wand_strap_left",
    )
    upper_body.visual(
        Cylinder(radius=0.036, length=0.230),
        origin=Origin(xyz=(0.115, 0.0, 0.305)),
        material=shell_dark,
        name="filter_canister",
    )
    upper_body.visual(
        Box((0.050, 0.060, 0.080)),
        origin=Origin(xyz=(0.085, 0.0, 0.210)),
        material=steel,
        name="canister_cradle",
    )
    upper_body.visual(
        Box((0.090, 0.020, 0.095)),
        origin=Origin(xyz=(0.066, 0.0, 0.565)),
        material=safety_yellow,
        name="service_guard",
    )
    handle_loop = tube_from_spline_points(
        [
            (0.025, 0.0, 0.480),
            (0.000, 0.0, 0.565),
            (0.025, 0.0, 0.665),
            (0.085, 0.0, 0.715),
            (0.120, 0.0, 0.640),
            (0.108, 0.0, 0.545),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=20,
    )
    upper_body.visual(
        _mesh("vacuum_handle_loop", handle_loop),
        material=steel,
        name="handle_loop",
    )
    upper_body.visual(
        Box((0.048, 0.034, 0.060)),
        origin=Origin(xyz=(0.100, 0.0, 0.515)),
        material=safety_yellow,
        name="trigger_guard_block",
    )
    upper_body.visual(
        Box((0.044, 0.018, 0.050)),
        origin=Origin(xyz=(0.030, 0.0, 0.043)),
        material=steel,
        name="fold_bridge_block",
    )
    upper_body.visual(
        Box((0.054, 0.006, 0.086)),
        origin=Origin(xyz=(0.0, 0.016, 0.015)),
        material=steel,
        name="fold_plate_left",
    )
    upper_body.visual(
        Box((0.054, 0.006, 0.086)),
        origin=Origin(xyz=(0.0, -0.016, 0.015)),
        material=steel,
        name="fold_plate_right",
    )
    upper_body.visual(
        Box((0.032, 0.020, 0.012)),
        origin=Origin(xyz=(-0.016, 0.0, 0.041)),
        material=safety_yellow,
        name="fold_stop_pad",
    )
    upper_body.visual(
        Box((0.028, 0.032, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.055)),
        material=safety_yellow,
        name="fold_lockout_bridge",
    )
    upper_body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=_axis_rpy("y")),
        material=fastener,
        name="fold_pin_head_left",
    )
    upper_body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=_axis_rpy("y")),
        material=fastener,
        name="fold_pin_head_right",
    )
    _add_fasteners(
        upper_body,
        prefix="body_plate_fastener_left",
        centers=[
            (0.067, 0.045, 0.265),
            (0.067, 0.045, 0.385),
            (0.067, 0.045, 0.505),
        ],
        axis="y",
        radius=0.0045,
        length=0.006,
        material=fastener,
    )
    _add_fasteners(
        upper_body,
        prefix="body_plate_fastener_right",
        centers=[
            (0.067, -0.045, 0.265),
            (0.067, -0.045, 0.385),
            (0.067, -0.045, 0.505),
        ],
        axis="y",
        radius=0.0045,
        length=0.006,
        material=fastener,
    )

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.58)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
    )
    lower_wand.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(rpy=_axis_rpy("y")),
        material=steel,
        name="hinge_barrel",
    )
    lower_wand.visual(
        Box((0.032, 0.018, 0.038)),
        origin=Origin(xyz=(-0.016, 0.0, 0.016)),
        material=safety_yellow,
        name="fold_stop_lug",
    )
    lower_wand.visual(
        Box((0.040, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="hinge_cheek",
    )
    lower_wand.visual(
        Box((0.040, 0.026, 0.446)),
        origin=Origin(xyz=(0.0, 0.0, -0.238)),
        material=steel,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.058, 0.038, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=safety_yellow,
        name="fold_collar_guard",
    )
    lower_wand.visual(
        Box((0.006, 0.022, 0.245)),
        origin=Origin(xyz=(0.023, 0.0, -0.230)),
        material=steel,
        name="lower_reinforcement_right",
    )
    lower_wand.visual(
        Box((0.006, 0.022, 0.245)),
        origin=Origin(xyz=(-0.023, 0.0, -0.230)),
        material=steel,
        name="lower_reinforcement_left",
    )
    lower_wand.visual(
        Box((0.014, 0.032, 0.285)),
        origin=Origin(xyz=(0.027, 0.0, -0.215)),
        material=safety_yellow,
        name="cable_guard_channel",
    )
    lower_wand.visual(
        Box((0.046, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.474)),
        material=steel,
        name="nozzle_bridge_block",
    )
    lower_wand.visual(
        Box((0.050, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, 0.016, -0.515)),
        material=steel,
        name="nozzle_plate_left",
    )
    lower_wand.visual(
        Box((0.050, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, -0.016, -0.515)),
        material=steel,
        name="nozzle_plate_right",
    )
    lower_wand.visual(
        Box((0.032, 0.020, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, -0.481)),
        material=safety_yellow,
        name="nozzle_stop_pad",
    )
    lower_wand.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.021, -0.515), rpy=_axis_rpy("y")),
        material=fastener,
        name="nozzle_pin_head_left",
    )
    lower_wand.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -0.021, -0.515), rpy=_axis_rpy("y")),
        material=fastener,
        name="nozzle_pin_head_right",
    )

    floor_head = model.part("floor_head")
    floor_head.inertial = Inertial.from_geometry(
        Box((0.13, 0.32, 0.09)),
        mass=0.9,
        origin=Origin(xyz=(0.015, 0.0, -0.035)),
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(rpy=_axis_rpy("y")),
        material=steel,
        name="head_knuckle",
    )
    floor_head.visual(
        Box((0.032, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, 0.0, 0.021)),
        material=safety_yellow,
        name="head_stop_lug",
    )
    floor_head.visual(
        Box((0.036, 0.022, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=steel,
        name="neck_spine",
    )
    floor_head.visual(
        Box((0.110, 0.300, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, -0.047)),
        material=shell_dark,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.020, 0.288, 0.020)),
        origin=Origin(xyz=(0.066, 0.0, -0.040)),
        material=rubber_black,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.018, 0.180, 0.018)),
        origin=Origin(xyz=(-0.038, 0.0, -0.040)),
        material=steel,
        name="rear_heel_guard",
    )
    floor_head.visual(
        Box((0.100, 0.004, 0.032)),
        origin=Origin(xyz=(0.010, 0.152, -0.046)),
        material=steel,
        name="side_skid_left",
    )
    floor_head.visual(
        Box((0.100, 0.004, 0.032)),
        origin=Origin(xyz=(0.010, -0.152, -0.046)),
        material=steel,
        name="side_skid_right",
    )
    floor_head.visual(
        Box((0.060, 0.110, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, -0.028)),
        material=safety_yellow,
        name="access_plate",
    )
    _add_fasteners(
        floor_head,
        prefix="access_plate_fastener",
        centers=[
            (-0.016, 0.036, -0.0245),
            (-0.016, -0.036, -0.0245),
            (0.026, 0.036, -0.0245),
            (0.026, -0.036, -0.0245),
        ],
        axis="z",
        radius=0.004,
        length=0.003,
        material=fastener,
    )

    model.articulation(
        "body_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=lower_wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "lower_wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.515)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=math.radians(-25.0),
            upper=math.radians(42.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_body = object_model.get_part("upper_body")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_lower_wand")
    nozzle_joint = object_model.get_articulation("lower_wand_to_floor_head")

    fold_plate_left = upper_body.get_visual("fold_plate_left")
    fold_stop_pad = upper_body.get_visual("fold_stop_pad")
    filter_canister = upper_body.get_visual("filter_canister")
    hinge_barrel = lower_wand.get_visual("hinge_barrel")
    fold_stop_lug = lower_wand.get_visual("fold_stop_lug")
    lower_tube = lower_wand.get_visual("lower_tube")
    nozzle_plate_left = lower_wand.get_visual("nozzle_plate_left")
    nozzle_stop_pad = lower_wand.get_visual("nozzle_stop_pad")
    head_knuckle = floor_head.get_visual("head_knuckle")
    head_stop_lug = floor_head.get_visual("head_stop_lug")

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
        "fold_joint_axis_and_limits",
        fold_joint.axis == (0.0, 1.0, 0.0)
        and fold_joint.motion_limits is not None
        and fold_joint.motion_limits.lower == 0.0
        and fold_joint.motion_limits.upper is not None
        and fold_joint.motion_limits.upper > 1.5,
        "Fold joint must pitch around the transverse pin and open to a deep fold.",
    )
    ctx.check(
        "nozzle_joint_axis_and_limits",
        nozzle_joint.axis == (0.0, -1.0, 0.0)
        and nozzle_joint.motion_limits is not None
        and nozzle_joint.motion_limits.lower is not None
        and nozzle_joint.motion_limits.lower < 0.0
        and nozzle_joint.motion_limits.upper is not None
        and nozzle_joint.motion_limits.upper > 0.6,
        "Nozzle joint should pitch on a pinned clevis with realistic fore-aft range.",
    )
    ctx.expect_contact(
        lower_wand,
        upper_body,
        elem_a=hinge_barrel,
        elem_b=fold_plate_left,
        name="fold_pin_is_bracketed",
    )
    ctx.expect_overlap(
        lower_wand,
        upper_body,
        elem_a=hinge_barrel,
        elem_b=fold_plate_left,
        axes="xz",
        min_overlap=0.025,
        name="fold_bracket_captures_barrel_projection",
    )
    ctx.expect_contact(
        lower_wand,
        upper_body,
        elem_a=fold_stop_lug,
        elem_b=fold_stop_pad,
        name="fold_stop_is_real_contact",
    )
    ctx.expect_contact(
        floor_head,
        lower_wand,
        elem_a=head_knuckle,
        elem_b=nozzle_plate_left,
        name="nozzle_pin_is_bracketed",
    )
    ctx.expect_overlap(
        floor_head,
        lower_wand,
        elem_a=head_knuckle,
        elem_b=nozzle_plate_left,
        axes="xz",
        min_overlap=0.020,
        name="nozzle_bracket_captures_knuckle_projection",
    )
    ctx.expect_contact(
        floor_head,
        lower_wand,
        elem_a=head_stop_lug,
        elem_b=nozzle_stop_pad,
        name="nozzle_stop_is_real_contact",
    )

    with ctx.pose({fold_joint: math.radians(95.0)}):
        ctx.expect_contact(
            lower_wand,
            upper_body,
            elem_a=hinge_barrel,
            elem_b=fold_plate_left,
            name="folded_pose_retains_hinge_support",
        )
        ctx.expect_gap(
            upper_body,
            lower_wand,
            axis="x",
            positive_elem=filter_canister,
            negative_elem=lower_tube,
            min_gap=0.030,
            name="folded_wand_clears_main_body",
        )

    with ctx.pose({nozzle_joint: math.radians(30.0)}):
        ctx.expect_contact(
            floor_head,
            lower_wand,
            elem_a=head_knuckle,
            elem_b=nozzle_plate_left,
            name="pitched_nozzle_retains_pin_support",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

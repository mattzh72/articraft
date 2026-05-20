from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_HEIGHT = 0.94
BODY_TOP_WIDTH = 0.58
BODY_TOP_DEPTH = 0.73
BODY_MID_WIDTH = 0.52
BODY_MID_DEPTH = 0.65
BODY_BOTTOM_WIDTH = 0.44
BODY_BOTTOM_DEPTH = 0.56
BODY_WALL = 0.012
BODY_BASE = 0.020
BODY_RIM = 0.024

AXLE_RADIUS = 0.011
AXLE_Y = -0.374
AXLE_Z = 0.112
WHEEL_RADIUS = 0.108
WHEEL_WIDTH = 0.052
WHEEL_CENTER_X = 0.287

HINGE_RADIUS = 0.011
HINGE_X = 0.274
HINGE_Y = -0.390
HINGE_Z = 0.928


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def _ring_on_x(inner_radius: float, outer_radius: float, length: float) -> MeshGeometry:
    half_length = 0.5 * length
    return LatheGeometry(
        [
            (inner_radius, -half_length),
            (outer_radius, -half_length),
            (outer_radius, half_length),
            (inner_radius, half_length),
            (inner_radius, -half_length),
        ],
        segments=48,
    ).rotate_y(pi / 2.0)


def _wheel_mesh(radius: float, width: float, bore_radius: float) -> MeshGeometry:
    half_width = 0.5 * width
    hub_radius = 0.032
    rim_radius = radius * 0.90
    profile = [
        (bore_radius, -half_width),
        (hub_radius, -half_width),
        (rim_radius, -half_width * 0.96),
        (radius * 0.985, -half_width * 0.52),
        (radius, 0.0),
        (radius * 0.985, half_width * 0.52),
        (rim_radius, half_width * 0.96),
        (hub_radius, half_width),
        (bore_radius, half_width),
        (bore_radius, half_width * 0.34),
        (hub_radius * 0.94, half_width * 0.18),
        (hub_radius * 0.94, -half_width * 0.18),
        (bore_radius, -half_width * 0.34),
        (bore_radius, -half_width),
    ]
    return LatheGeometry(profile, segments=64).rotate_y(pi / 2.0)


def _body_shell_mesh() -> MeshGeometry:
    outer_lower = _rounded_loop(BODY_BOTTOM_WIDTH, BODY_BOTTOM_DEPTH, 0.038, BODY_BASE, y_shift=-0.020)
    outer_mid = _rounded_loop(BODY_MID_WIDTH, BODY_MID_DEPTH, 0.050, 0.49, y_shift=-0.012)
    outer_top = _rounded_loop(BODY_TOP_WIDTH, BODY_TOP_DEPTH, 0.062, BODY_HEIGHT)

    inner_lower = _rounded_loop(
        BODY_BOTTOM_WIDTH - 2.0 * BODY_WALL,
        BODY_BOTTOM_DEPTH - 2.0 * BODY_WALL,
        0.026,
        BODY_BASE,
        y_shift=-0.020,
    )
    inner_mid = _rounded_loop(
        BODY_MID_WIDTH - 2.0 * BODY_WALL,
        BODY_MID_DEPTH - 2.0 * BODY_WALL,
        0.038,
        0.49,
        y_shift=-0.012,
    )
    inner_top = _rounded_loop(
        BODY_TOP_WIDTH - 2.0 * BODY_WALL,
        BODY_TOP_DEPTH - 2.0 * BODY_WALL,
        0.050,
        BODY_HEIGHT,
    )

    outer_walls = LoftGeometry([outer_lower, outer_mid, outer_top], cap=False, closed=True)
    inner_walls = LoftGeometry(
        [list(reversed(inner_lower)), list(reversed(inner_mid)), list(reversed(inner_top))],
        cap=False,
        closed=True,
    )

    bottom_profile = rounded_rect_profile(BODY_BOTTOM_WIDTH, BODY_BOTTOM_DEPTH, 0.038, corner_segments=8)
    bottom_plate = ExtrudeGeometry.from_z0(bottom_profile, BODY_BASE).translate(0.0, -0.020, 0.0)

    top_outer_profile = rounded_rect_profile(BODY_TOP_WIDTH, BODY_TOP_DEPTH, 0.062, corner_segments=8)
    top_inner_profile = rounded_rect_profile(
        BODY_TOP_WIDTH - 2.0 * BODY_WALL,
        BODY_TOP_DEPTH - 2.0 * BODY_WALL,
        0.050,
        corner_segments=8,
    )
    top_rim = ExtrudeWithHolesGeometry(
        top_outer_profile,
        [top_inner_profile],
        BODY_RIM,
        center=False,
    ).translate(0.0, 0.0, BODY_HEIGHT - BODY_RIM)

    return _merge_geometries(outer_walls, inner_walls, bottom_plate, top_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    body_green = model.material("body_green", rgba=(0.19, 0.27, 0.16, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.08, 0.08, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))

    body = model.part("body")
    body.visual(
        _mesh("wheelie_bin_body_shell", _body_shell_mesh()),
        material=body_green,
        name="shell",
    )
    body.visual(
        Box((0.32, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.282, 0.055)),
        material=body_green,
        name="front_toe",
    )
    body.visual(
        Box((0.100, 0.090, 0.070)),
        origin=Origin(xyz=(0.198, AXLE_Y + 0.040, 0.175)),
        material=body_green,
        name="left_axle_web",
    )
    body.visual(
        Box((0.100, 0.090, 0.070)),
        origin=Origin(xyz=(-0.198, AXLE_Y + 0.040, 0.175)),
        material=body_green,
        name="right_axle_web",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.160, AXLE_Y, AXLE_Z)),
        material=body_green,
        name="left_axle_boss",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(-0.160, AXLE_Y, AXLE_Z)),
        material=body_green,
        name="right_axle_boss",
    )
    body.visual(
        Cylinder(radius=0.0175, length=0.036),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_green,
        name="left_hinge_socket",
    )
    body.visual(
        Box((0.045, 0.060, 0.078)),
        origin=Origin(xyz=(HINGE_X - 0.033, HINGE_Y + 0.027, HINGE_Z - 0.056)),
        material=body_green,
        name="left_hinge_support",
    )
    body.visual(
        Cylinder(radius=0.0175, length=0.036),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_green,
        name="right_hinge_socket",
    )
    body.visual(
        Box((0.045, 0.060, 0.078)),
        origin=Origin(xyz=(-(HINGE_X - 0.033), HINGE_Y + 0.027, HINGE_Z - 0.056)),
        material=body_green,
        name="right_hinge_support",
    )
    body.visual(
        Box((0.055, 0.038, 0.160)),
        origin=Origin(xyz=(0.205, -0.376, 0.825)),
        material=body_green,
        name="left_handle_post",
    )
    body.visual(
        Box((0.055, 0.038, 0.160)),
        origin=Origin(xyz=(-0.205, -0.376, 0.825)),
        material=body_green,
        name="right_handle_post",
    )
    body.visual(
        Box((0.470, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.392, 0.825)),
        material=body_green,
        name="rear_handle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.76, BODY_HEIGHT)),
        mass=13.5,
        origin=Origin(xyz=(0.0, -0.01, BODY_HEIGHT * 0.50)),
    )

    axle = model.part("axle")
    axle.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.646),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod",
    )
    axle.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.256, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_inner_retainer",
    )
    axle.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.256, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_inner_retainer",
    )
    axle.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.318, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_end_cap",
    )
    axle.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(-0.318, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_end_cap",
    )
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.650),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        _mesh("wheelie_bin_left_wheel", _wheel_mesh(WHEEL_RADIUS, WHEEL_WIDTH, AXLE_RADIUS + 0.001)),
        material=wheel_black,
        name="wheel_shell",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        _mesh("wheelie_bin_right_wheel", _wheel_mesh(WHEEL_RADIUS, WHEEL_WIDTH, AXLE_RADIUS + 0.001)),
        material=wheel_black,
        name="wheel_shell",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    lid = model.part("lid")
    lid_top_outer = rounded_rect_profile(0.648, 0.800, 0.068, corner_segments=8)
    lid_skirt_outer = rounded_rect_profile(0.644, 0.794, 0.066, corner_segments=8)
    lid_skirt_inner = rounded_rect_profile(0.620, 0.770, 0.056, corner_segments=8)
    lid.visual(
        _mesh(
            "wheelie_bin_lid_top",
            ExtrudeGeometry.from_z0(lid_top_outer, 0.007).translate(0.0, 0.426, 0.012),
        ),
        material=body_green,
        name="lid_top",
    )
    lid.visual(
        Box((0.028, 0.717, 0.052)),
        origin=Origin(xyz=(0.308, 0.3785, -0.014)),
        material=body_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.028, 0.717, 0.052)),
        origin=Origin(xyz=(-0.308, 0.3785, -0.014)),
        material=body_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.008, 0.028, 0.032)),
        origin=Origin(xyz=(HINGE_X + 0.025, 0.016, 0.006)),
        material=body_green,
        name="left_hinge_bridge",
    )
    lid.visual(
        Box((0.008, 0.028, 0.032)),
        origin=Origin(xyz=(-(HINGE_X + 0.025), 0.016, 0.006)),
        material=body_green,
        name="right_hinge_bridge",
    )
    lid.visual(
        Box((0.220, 0.036, 0.022)),
        origin=Origin(xyz=(0.0, 0.806, 0.016)),
        material=body_green,
        name="front_pull",
    )
    lid.visual(
        Box((0.040, 0.300, 0.016)),
        origin=Origin(xyz=(0.100, 0.476, 0.004)),
        material=body_green,
        name="left_rib",
    )
    lid.visual(
        Box((0.040, 0.300, 0.016)),
        origin=Origin(xyz=(-0.100, 0.476, 0.004)),
        material=body_green,
        name="right_rib",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.042),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_green,
        name="left_hinge_pin",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.042),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_green,
        name="right_hinge_pin",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.76, 0.060)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.410, 0.002)),
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z)),
    )
    model.articulation(
        "axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=left_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=right_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    axle = object_model.get_part("axle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid = object_model.get_part("lid")
    left_spin = object_model.get_articulation("axle_to_left_wheel")
    right_spin = object_model.get_articulation("axle_to_right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

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
    ctx.allow_overlap(
        axle,
        body,
        elem_a="rod",
        elem_b="left_axle_boss",
        reason="The steel axle passes through the molded left bearing boss; the captured through-bore reads as overlap in rest-pose QC.",
    )
    ctx.allow_overlap(
        axle,
        body,
        elem_a="rod",
        elem_b="right_axle_boss",
        reason="The steel axle passes through the molded right bearing boss; the captured through-bore reads as overlap in rest-pose QC.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="left_hinge_socket",
        elem_b="left_hinge_pin",
        reason="The molded snap socket captures the left lid trunnion pin; the retained pin-in-socket fit is intentionally concentric.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="right_hinge_socket",
        elem_b="right_hinge_pin",
        reason="The molded snap socket captures the right lid trunnion pin; the retained pin-in-socket fit is intentionally concentric.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lid hinge axis runs across bin width",
        tuple(round(value, 6) for value in lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "wheel spin axes match rear axle",
        tuple(round(value, 6) for value in left_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(value, 6) for value in right_spin.axis) == (1.0, 0.0, 0.0),
        f"left={left_spin.axis}, right={right_spin.axis}",
    )

    ctx.expect_contact(axle, body, name="axle is clamped into body bosses")
    ctx.expect_contact(left_wheel, axle, name="left wheel is retained on axle")
    ctx.expect_contact(right_wheel, axle, name="right wheel is retained on axle")
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.55,
        max_dist=0.59,
        name="rear wheels span the bin width",
    )
    ctx.expect_origin_distance(
        left_wheel,
        axle,
        axes="yz",
        max_dist=0.001,
        name="left wheel centers on axle line",
    )
    ctx.expect_origin_distance(
        right_wheel,
        axle,
        axes="yz",
        max_dist=0.001,
        name="right wheel centers on axle line",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_hinge_pin",
            elem_b="left_hinge_socket",
            name="left lid hinge pin seats in socket",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_hinge_pin",
            elem_b="right_hinge_socket",
            name="right lid hinge pin seats in socket",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top",
            negative_elem="shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid closes down onto top rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.52,
            name="closed lid covers the full bin opening",
        )

    with ctx.pose({lid_hinge: 1.35}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_hinge_pin",
            elem_b="left_hinge_socket",
            name="left hinge stays engaged when lid is open",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_hinge_pin",
            elem_b="right_hinge_socket",
            name="right hinge stays engaged when lid is open",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_pull",
            negative_elem="shell",
            min_gap=0.22,
            name="front edge lifts clear when lid swings open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

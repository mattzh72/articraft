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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 0.58
AXLE_RADIUS = 0.024
WHEEL_OUTER_RADIUS = 0.42
WHEEL_WIDTH = 0.22
RIM_CENTER_X = 0.095
SUPPORT_X = 0.17
INNER_GUARD_X = 0.145


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _merge_meshes(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_wheel_core_mesh() -> MeshGeometry:
    hub_radius = 0.085
    hub_length = 0.120
    flange_radius = 0.104
    flange_length = 0.014
    rim_radius = 0.372
    rim_tube = 0.016
    spoke_length = 0.356
    spoke_center_radius = 0.228
    collar_radius = 0.060
    collar_length = 0.014
    collar_x = 0.126
    web_radius = 0.248
    web_length = 0.010
    web_x = 0.055

    geometries: list[MeshGeometry] = [
        CylinderGeometry(radius=hub_radius, height=hub_length, radial_segments=36).rotate_y(
            math.pi / 2.0
        ),
        CylinderGeometry(radius=flange_radius, height=flange_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(-0.045, 0.0, 0.0),
        CylinderGeometry(radius=flange_radius, height=flange_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(0.045, 0.0, 0.0),
        CylinderGeometry(radius=collar_radius, height=collar_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(-collar_x, 0.0, 0.0),
        CylinderGeometry(radius=collar_radius, height=collar_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(collar_x, 0.0, 0.0),
        CylinderGeometry(radius=web_radius, height=web_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(-web_x, 0.0, 0.0),
        CylinderGeometry(radius=web_radius, height=web_length, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(web_x, 0.0, 0.0),
        TorusGeometry(
            radius=rim_radius,
            tube=rim_tube,
            radial_segments=18,
            tubular_segments=72,
        )
        .rotate_y(math.pi / 2.0)
        .translate(-RIM_CENTER_X, 0.0, 0.0),
        TorusGeometry(
            radius=rim_radius,
            tube=rim_tube,
            radial_segments=18,
            tubular_segments=72,
        )
        .rotate_y(math.pi / 2.0)
        .translate(RIM_CENTER_X, 0.0, 0.0),
    ]

    for index in range(8):
        angle = (math.tau * index) / 8.0
        spoke = (
            BoxGeometry((0.182, 0.030, spoke_length))
            .rotate_x(angle)
            .translate(
                0.0,
                spoke_center_radius * math.sin(angle),
                spoke_center_radius * math.cos(angle),
            )
        )
        geometries.append(spoke)

    return _merge_meshes(geometries)


def _build_paddle_mesh() -> MeshGeometry:
    paddle_center_radius = 0.338
    paddle_height = 0.164
    paddle_thickness = 0.022
    paddle_pitch = 0.16

    geometries: list[MeshGeometry] = []
    for index in range(12):
        angle = (math.tau * index) / 12.0
        paddle = (
            BoxGeometry((WHEEL_WIDTH, paddle_thickness, paddle_height))
            .rotate_x(angle + paddle_pitch)
            .translate(
                0.0,
                paddle_center_radius * math.sin(angle),
                paddle_center_radius * math.cos(angle),
            )
        )
        geometries.append(paddle)
    return _merge_meshes(geometries)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_undershot_waterwheel")

    frame_paint = model.material("frame_paint", rgba=(0.34, 0.38, 0.43, 1.0))
    datum_gray = model.material("datum_gray", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    paddle_timber = model.material("paddle_timber", rgba=(0.53, 0.40, 0.24, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.78, 0.14, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.08, 1.08, 0.06)),
        origin=Origin(xyz=(-0.18, 0.0, 0.03)),
        material=frame_paint,
        name="left_runner",
    )
    frame.visual(
        Box((0.08, 1.08, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.03)),
        material=frame_paint,
        name="right_runner",
    )
    frame.visual(
        Box((0.48, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.48, 0.12)),
        material=frame_paint,
        name="front_tie",
    )
    frame.visual(
        Box((0.48, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.48, 0.12)),
        material=frame_paint,
        name="rear_tie",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((0.010, 0.22, 0.438)),
            origin=Origin(xyz=(sign * INNER_GUARD_X, 0.0, 0.279)),
            material=datum_gray,
            name=f"{side_name}_inner_guard",
        )
        frame.visual(
            Box((0.040, 0.120, 0.680)),
            origin=Origin(xyz=(sign * 0.19, 0.14, 0.40)),
            material=frame_paint,
            name=f"{side_name}_front_post",
        )
        frame.visual(
            Box((0.040, 0.120, 0.680)),
            origin=Origin(xyz=(sign * 0.19, -0.14, 0.40)),
            material=frame_paint,
            name=f"{side_name}_rear_post",
        )
        frame.visual(
            Box((0.050, 0.280, 0.020)),
            origin=Origin(xyz=(sign * SUPPORT_X, 0.0, 0.398)),
            material=machined_steel,
            name=f"{side_name}_adjuster_plate",
        )
        frame.visual(
            Box((0.050, 0.280, 0.008)),
            origin=Origin(xyz=(sign * SUPPORT_X, 0.0, 0.502)),
            material=datum_gray,
            name=f"{side_name}_shim_pack",
        )
        frame.visual(
            Box((0.050, 0.280, 0.050)),
            origin=Origin(xyz=(sign * SUPPORT_X, 0.0, AXLE_Z - AXLE_RADIUS - 0.025)),
            material=machined_steel,
            name=f"{side_name}_saddle",
        )
        frame.visual(
            Box((0.050, 0.280, 0.020)),
            origin=Origin(xyz=(sign * SUPPORT_X, 0.0, AXLE_Z + AXLE_RADIUS + 0.014)),
            material=machined_steel,
            name=f"{side_name}_bearing_cap",
        )

        for y_pos, screw_name in ((0.075, "front"), (-0.075, "rear")):
            frame.visual(
                Cylinder(radius=0.006, length=0.090),
                origin=Origin(xyz=(sign * 0.185, y_pos, 0.453)),
                material=dark_steel,
                name=f"{side_name}_{screw_name}_jackscrew",
            )
            frame.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(sign * 0.185, y_pos, 0.488)),
                material=dark_steel,
                name=f"{side_name}_{screw_name}_locknut",
            )

        frame.visual(
            Box((0.004, 0.100, 0.004)),
            origin=Origin(xyz=(sign * 0.152, 0.09, AXLE_Z)),
            material=indicator_red if sign < 0.0 else datum_gray,
            name=f"{side_name}_zero_mark",
        )
        frame.visual(
            Box((0.004, 0.060, 0.004)),
            origin=Origin(xyz=(sign * 0.152, 0.09, AXLE_Z + 0.050)),
            material=datum_gray,
            name=f"{side_name}_upper_mark",
        )
        frame.visual(
            Box((0.004, 0.060, 0.004)),
            origin=Origin(xyz=(sign * 0.152, 0.09, AXLE_Z - 0.050)),
            material=datum_gray,
            name=f"{side_name}_lower_mark",
        )
        frame.visual(
            Box((0.008, 0.080, 0.172)),
            origin=Origin(xyz=(sign * 0.148, 0.09, 0.584)),
            material=datum_gray,
            name=f"{side_name}_scale_backer",
        )

    frame.visual(
        Box((0.008, 0.030, 0.030)),
        origin=Origin(xyz=(-0.154, 0.09, AXLE_Z + 0.072)),
        material=indicator_red,
        name="left_pointer",
    )
    frame.visual(
        Box((0.48, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, -0.50, 0.74)),
        material=datum_gray,
        name="rear_datum_bridge",
    )

    for name, a, b in (
        ("left_rear_brace", (-0.19, -0.14, 0.72), (-0.18, -0.50, 0.74)),
        ("right_rear_brace", (0.19, -0.14, 0.72), (0.18, -0.50, 0.74)),
    ):
        frame.visual(
            Cylinder(radius=0.016, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=frame_paint,
            name=name,
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.52, 1.10, 0.78)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(_build_wheel_core_mesh(), "undershot_wheel_core"),
        material=machined_steel,
        name="wheel_core",
    )
    wheel.visual(
        mesh_from_geometry(_build_paddle_mesh(), "undershot_wheel_paddles"),
        material=paddle_timber,
        name="wheel_paddles",
    )
    wheel.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.38),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Box((0.012, 0.012, 0.030)),
        origin=Origin(xyz=(-0.129, 0.0, 0.073)),
        material=indicator_red,
        name="reference_index",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_OUTER_RADIUS, length=WHEEL_WIDTH),
        mass=28.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=4.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("frame_to_wheel")

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
        "wheel_spin_is_explicit",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"expected continuous rotation about +X, got type={spin.articulation_type} axis={spin.axis}",
    )

    ctx.expect_contact(
        frame,
        wheel,
        elem_a="left_saddle",
        elem_b="axle_shaft",
        name="left_bearing_support_contacts_axle",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="right_saddle",
        elem_b="axle_shaft",
        name="right_bearing_support_contacts_axle",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="wheel_paddles",
        negative_elem="left_inner_guard",
        min_gap=0.028,
        max_gap=0.042,
        name="left_side_clearance_controlled",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="right_inner_guard",
        negative_elem="wheel_paddles",
        min_gap=0.028,
        max_gap=0.042,
        name="right_side_clearance_controlled",
    )

    reference_rest = ctx.part_element_world_aabb(wheel, elem="reference_index")
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_contact(
            frame,
            wheel,
            elem_a="left_saddle",
            elem_b="axle_shaft",
            name="left_bearing_support_stays_loaded_at_quarter_turn",
        )
        ctx.expect_contact(
            frame,
            wheel,
            elem_a="right_saddle",
            elem_b="axle_shaft",
            name="right_bearing_support_stays_loaded_at_quarter_turn",
        )
        reference_quarter = ctx.part_element_world_aabb(wheel, elem="reference_index")

    moved_as_expected = False
    if reference_rest is not None and reference_quarter is not None:
        rest_center = _aabb_center(reference_rest)
        quarter_center = _aabb_center(reference_quarter)
        moved_as_expected = (
            quarter_center[2] < rest_center[2] - 0.05
            and quarter_center[1] < rest_center[1] - 0.05
        )
    ctx.check(
        "reference_index_tracks_rotation",
        moved_as_expected,
        details="expected the wheel reference index to move from top-dead-center toward negative Y on a +90 deg spin",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

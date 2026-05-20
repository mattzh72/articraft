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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_LENGTH_TOP = 1.08
BODY_WIDTH_TOP = 0.34
BODY_LENGTH_BOTTOM = 1.00
BODY_WIDTH_BOTTOM = 0.28
BODY_HEIGHT = 0.42
BODY_RADIUS_TOP = 0.042
BODY_RADIUS_BOTTOM = 0.032
WALL_THICKNESS = 0.018
FLOOR_THICKNESS = 0.028
RIM_DROP = 0.026
DIVIDER_THICKNESS = 0.016
LID_THICKNESS = 0.036
LID_LENGTH = 1.10
LID_PANEL_DEPTH = (BODY_WIDTH_TOP * 0.5) - (DIVIDER_THICKNESS * 0.5) + 0.010
LID_RADIUS = 0.028


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _append_loop(
    geometry: MeshGeometry,
    points: list[tuple[float, float, float]],
) -> list[int]:
    return [geometry.add_vertex(*point) for point in points]


def _bridge_loops(geometry: MeshGeometry, loop_a: list[int], loop_b: list[int]) -> None:
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geometry,
            loop_a[index],
            loop_a[next_index],
            loop_b[next_index],
            loop_b[index],
        )


def _cap_loop(
    geometry: MeshGeometry,
    loop_ids: list[int],
    points: list[tuple[float, float, float]],
    *,
    upward: bool,
) -> None:
    center = (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        sum(point[2] for point in points) / len(points),
    )
    center_id = geometry.add_vertex(*center)
    for index, point_id in enumerate(loop_ids):
        next_id = loop_ids[(index + 1) % len(loop_ids)]
        if upward:
            geometry.add_face(center_id, point_id, next_id)
        else:
            geometry.add_face(center_id, next_id, point_id)


def _rounded_rect_loop(
    length: float,
    width: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(length, width, radius, corner_segments=8)
    ]


def _build_body_shell_mesh() -> MeshGeometry:
    geometry = MeshGeometry()

    outer_bottom_points = _rounded_rect_loop(
        BODY_LENGTH_BOTTOM,
        BODY_WIDTH_BOTTOM,
        BODY_RADIUS_BOTTOM,
        0.0,
    )
    outer_top_points = _rounded_rect_loop(
        BODY_LENGTH_TOP,
        BODY_WIDTH_TOP,
        BODY_RADIUS_TOP,
        BODY_HEIGHT,
    )
    inner_bottom_points = _rounded_rect_loop(
        BODY_LENGTH_BOTTOM - (2.0 * WALL_THICKNESS),
        BODY_WIDTH_BOTTOM - (2.0 * WALL_THICKNESS),
        max(BODY_RADIUS_BOTTOM - (0.65 * WALL_THICKNESS), 0.008),
        FLOOR_THICKNESS,
    )
    inner_top_points = _rounded_rect_loop(
        BODY_LENGTH_TOP - (2.0 * WALL_THICKNESS),
        BODY_WIDTH_TOP - (2.0 * WALL_THICKNESS),
        max(BODY_RADIUS_TOP - (0.65 * WALL_THICKNESS), 0.010),
        BODY_HEIGHT - RIM_DROP,
    )

    outer_bottom = _append_loop(geometry, outer_bottom_points)
    outer_top = _append_loop(geometry, outer_top_points)
    inner_top = _append_loop(geometry, inner_top_points)
    inner_bottom = _append_loop(geometry, inner_bottom_points)

    _bridge_loops(geometry, outer_bottom, outer_top)
    _bridge_loops(geometry, outer_top, inner_top)
    _bridge_loops(geometry, inner_top, inner_bottom)
    _cap_loop(geometry, outer_bottom, outer_bottom_points, upward=False)
    _cap_loop(geometry, inner_bottom, inner_bottom_points, upward=True)

    return geometry


def _lid_section(
    length: float,
    depth: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(length, depth, radius, corner_segments=8)
    ]


def _build_lid_shell_mesh() -> MeshGeometry:
    bottom = _lid_section(LID_LENGTH, LID_PANEL_DEPTH, LID_RADIUS, 0.0)
    shoulder = _lid_section(
        LID_LENGTH - 0.010,
        LID_PANEL_DEPTH - 0.010,
        LID_RADIUS - 0.004,
        LID_THICKNESS * 0.62,
    )
    top = _lid_section(
        LID_LENGTH - 0.028,
        LID_PANEL_DEPTH - 0.016,
        LID_RADIUS - 0.008,
        LID_THICKNESS,
    )
    return section_loft([bottom, shoulder, top])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slimline_split_lid_cooler")

    body_shell = model.material("body_shell", rgba=(0.86, 0.87, 0.84, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.21, 0.24, 0.28, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    liner_white = model.material("liner_white", rgba=(0.95, 0.96, 0.95, 1.0))

    body = model.part("cooler_body")
    body.visual(
        mesh_from_geometry(_build_body_shell_mesh(), "cooler_body_shell"),
        material=body_shell,
        name="body_shell",
    )
    body.visual(
        Box(
            (
                BODY_LENGTH_TOP - (2.0 * WALL_THICKNESS) - 0.020,
                DIVIDER_THICKNESS,
                BODY_HEIGHT - FLOOR_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FLOOR_THICKNESS + ((BODY_HEIGHT - FLOOR_THICKNESS) * 0.5),
            )
        ),
        material=liner_white,
        name="divider_wall",
    )
    for sign, label in ((-1.0, "left"), (1.0, "right")):
        body.visual(
            Box((BODY_LENGTH_TOP * 0.88, 0.024, 0.078)),
            origin=Origin(
                xyz=(0.0, sign * ((BODY_WIDTH_TOP * 0.5) - 0.005), BODY_HEIGHT * 0.54)
            ),
            material=handle_dark,
            name=f"{label}_side_band",
        )
        body.visual(
            Box((0.040, 0.180, 0.044)),
            origin=Origin(
                xyz=(sign * ((BODY_LENGTH_TOP * 0.5) - 0.010), 0.0, BODY_HEIGHT * 0.56)
            ),
            material=handle_dark,
            name=f"{label}_end_handle",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH_TOP, BODY_WIDTH_TOP, BODY_HEIGHT)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    lid_mesh = mesh_from_geometry(_build_lid_shell_mesh(), "cooler_lid_shell")

    left_lid = model.part("left_lid")
    left_lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, -(LID_PANEL_DEPTH * 0.5), 0.0)),
        material=lid_plastic,
        name="lid_shell",
    )
    left_lid.visual(
        Box((LID_LENGTH * 0.72, 0.018, 0.014)),
        origin=Origin(
            xyz=(0.0, -LID_PANEL_DEPTH + 0.012, (LID_THICKNESS * 0.5) + 0.005)
        ),
        material=handle_dark,
        name="outer_grip",
    )
    left_lid.inertial = Inertial.from_geometry(
        Box((LID_LENGTH, LID_PANEL_DEPTH, LID_THICKNESS)),
        mass=1.6,
        origin=Origin(
            xyz=(0.0, -(LID_PANEL_DEPTH * 0.5), LID_THICKNESS * 0.5),
        ),
    )

    right_lid = model.part("right_lid")
    right_lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, LID_PANEL_DEPTH * 0.5, 0.0)),
        material=lid_plastic,
        name="lid_shell",
    )
    right_lid.visual(
        Box((LID_LENGTH * 0.72, 0.018, 0.014)),
        origin=Origin(
            xyz=(0.0, LID_PANEL_DEPTH - 0.012, (LID_THICKNESS * 0.5) + 0.005)
        ),
        material=handle_dark,
        name="outer_grip",
    )
    right_lid.inertial = Inertial.from_geometry(
        Box((LID_LENGTH, LID_PANEL_DEPTH, LID_THICKNESS)),
        mass=1.6,
        origin=Origin(
            xyz=(0.0, LID_PANEL_DEPTH * 0.5, LID_THICKNESS * 0.5),
        ),
    )

    model.articulation(
        "body_to_left_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_lid,
        origin=Origin(xyz=(0.0, -(DIVIDER_THICKNESS * 0.5), BODY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "body_to_right_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_lid,
        origin=Origin(xyz=(0.0, DIVIDER_THICKNESS * 0.5, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cooler_body")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_hinge = object_model.get_articulation("body_to_left_lid")
    right_hinge = object_model.get_articulation("body_to_right_lid")

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
        "left lid hinge axis points outward",
        tuple(left_hinge.axis) == (-1.0, 0.0, 0.0),
        f"axis={left_hinge.axis}",
    )
    ctx.check(
        "right lid hinge axis points outward",
        tuple(right_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={right_hinge.axis}",
    )

    ctx.expect_gap(
        left_lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="left lid seats on cooler rim",
    )
    ctx.expect_gap(
        right_lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="right lid seats on cooler rim",
    )
    ctx.expect_overlap(
        left_lid,
        body,
        axes="x",
        min_overlap=0.96,
        name="left lid covers elongated body span",
    )
    ctx.expect_overlap(
        right_lid,
        body,
        axes="x",
        min_overlap=0.96,
        name="right lid covers elongated body span",
    )

    left_closed_aabb = ctx.part_world_aabb(left_lid)
    right_closed_aabb = ctx.part_world_aabb(right_lid)
    assert left_closed_aabb is not None
    assert right_closed_aabb is not None

    with ctx.pose({left_hinge: math.radians(72.0), right_hinge: 0.0}):
        left_open_aabb = ctx.part_world_aabb(left_lid)
        right_still_aabb = ctx.part_world_aabb(right_lid)
        assert left_open_aabb is not None
        assert right_still_aabb is not None
        ctx.check(
            "left lid opens independently",
            left_open_aabb[1][2] > left_closed_aabb[1][2] + 0.10,
            f"closed_max_z={left_closed_aabb[1][2]:.4f}, open_max_z={left_open_aabb[1][2]:.4f}",
        )
        ctx.check(
            "right lid stays closed while left opens",
            abs(right_still_aabb[1][2] - right_closed_aabb[1][2]) < 0.005,
            f"closed_max_z={right_closed_aabb[1][2]:.4f}, pose_max_z={right_still_aabb[1][2]:.4f}",
        )
        ctx.expect_gap(
            left_lid,
            body,
            axis="z",
            min_gap=0.08,
            positive_elem="outer_grip",
            name="left outer grip lifts clear of body",
        )
        ctx.expect_gap(
            right_lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="right lid remains seated when left lid opens",
        )

    with ctx.pose({left_hinge: 0.0, right_hinge: math.radians(72.0)}):
        right_open_aabb = ctx.part_world_aabb(right_lid)
        left_still_aabb = ctx.part_world_aabb(left_lid)
        assert right_open_aabb is not None
        assert left_still_aabb is not None
        ctx.check(
            "right lid opens independently",
            right_open_aabb[1][2] > right_closed_aabb[1][2] + 0.10,
            f"closed_max_z={right_closed_aabb[1][2]:.4f}, open_max_z={right_open_aabb[1][2]:.4f}",
        )
        ctx.check(
            "left lid stays closed while right opens",
            abs(left_still_aabb[1][2] - left_closed_aabb[1][2]) < 0.005,
            f"closed_max_z={left_closed_aabb[1][2]:.4f}, pose_max_z={left_still_aabb[1][2]:.4f}",
        )
        ctx.expect_gap(
            right_lid,
            body,
            axis="z",
            min_gap=0.08,
            positive_elem="outer_grip",
            name="right outer grip lifts clear of body",
        )
        ctx.expect_gap(
            left_lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="left lid remains seated when right lid opens",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

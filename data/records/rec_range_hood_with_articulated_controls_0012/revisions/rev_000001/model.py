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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _build_curved_front_strip(
    *,
    width: float,
    height: float,
    thickness: float,
    outer_radius: float,
    centerline_y: float,
    z_bottom: float,
    segments: int = 28,
) -> MeshGeometry:
    geometry = MeshGeometry()
    inner_radius = outer_radius - thickness
    half_angle = math.asin(width / (2.0 * outer_radius))
    circle_center_y = centerline_y - outer_radius

    rings: list[tuple[int, int, int, int]] = []
    for index in range(segments + 1):
        t = index / segments
        theta = -half_angle + (2.0 * half_angle * t)
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        outer_x = outer_radius * sin_t
        outer_y = circle_center_y + outer_radius * cos_t
        inner_x = inner_radius * sin_t
        inner_y = circle_center_y + inner_radius * cos_t

        rings.append(
            (
                geometry.add_vertex(outer_x, outer_y, z_bottom),
                geometry.add_vertex(outer_x, outer_y, z_bottom + height),
                geometry.add_vertex(inner_x, inner_y, z_bottom),
                geometry.add_vertex(inner_x, inner_y, z_bottom + height),
            )
        )

    for start, end in zip(rings[:-1], rings[1:]):
        ob0, ot0, ib0, it0 = start
        ob1, ot1, ib1, it1 = end
        _add_quad(geometry, ob0, ob1, ot1, ot0)
        _add_quad(geometry, ib0, it0, it1, ib1)
        _add_quad(geometry, ot0, ot1, it1, it0)
        _add_quad(geometry, ib0, ib1, ob1, ob0)

    first = rings[0]
    last = rings[-1]
    _add_quad(geometry, first[0], first[1], first[3], first[2])
    _add_quad(geometry, last[2], last[3], last[1], last[0])
    return geometry


def _button_mount_origin(
    *,
    theta: float,
    outer_radius: float,
    centerline_y: float,
    z: float,
) -> Origin:
    circle_center_y = centerline_y - outer_radius
    x = outer_radius * math.sin(theta)
    y = circle_center_y + outer_radius * math.cos(theta)
    yaw = (math.pi / 2.0) - theta
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_filter = model.material("filter_dark", rgba=(0.29, 0.31, 0.33, 1.0))
    glass_light = model.material("light_glass", rgba=(0.92, 0.94, 0.95, 0.70))
    button_black = model.material("button_black", rgba=(0.12, 0.12, 0.13, 1.0))

    hood_width = 0.90
    hood_depth = 0.50
    hood_height = 0.16
    sheet = 0.006

    strip_width = 0.884
    strip_height = 0.072
    strip_thickness = 0.010
    strip_outer_radius = 1.40
    strip_centerline_y = 0.474
    strip_z_bottom = 0.078
    button_z = strip_z_bottom + (strip_height * 0.52)

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((hood_width, hood_depth, sheet)),
        origin=Origin(xyz=(0.0, hood_depth * 0.5, hood_height - (sheet * 0.5))),
        material=steel,
        name="top_panel",
    )
    hood_body.visual(
        Box((hood_width, sheet, hood_height)),
        origin=Origin(xyz=(0.0, sheet * 0.5, hood_height * 0.5)),
        material=steel,
        name="back_panel",
    )
    hood_body.visual(
        Box((sheet, hood_depth, hood_height)),
        origin=Origin(xyz=(-(hood_width * 0.5) + (sheet * 0.5), hood_depth * 0.5, hood_height * 0.5)),
        material=steel,
        name="left_side_panel",
    )
    hood_body.visual(
        Box((sheet, hood_depth, hood_height)),
        origin=Origin(xyz=((hood_width * 0.5) - (sheet * 0.5), hood_depth * 0.5, hood_height * 0.5)),
        material=steel,
        name="right_side_panel",
    )
    hood_body.visual(
        Box((hood_width, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.458, 0.151)),
        material=satin_steel,
        name="front_upper_return",
    )
    hood_body.visual(
        mesh_from_geometry(
            _build_curved_front_strip(
                width=strip_width,
                height=strip_height,
                thickness=strip_thickness,
                outer_radius=strip_outer_radius,
                centerline_y=strip_centerline_y,
                z_bottom=strip_z_bottom,
            ),
            ASSETS.mesh_path("range_hood_front_strip.obj"),
        ),
        material=steel,
        name="front_control_strip",
    )
    hood_body.visual(
        Box((hood_width, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.484, 0.074)),
        material=satin_steel,
        name="front_lip",
    )
    hood_body.visual(
        Box((hood_width, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.095, 0.018)),
        material=satin_steel,
        name="rear_filter_rail",
    )
    hood_body.visual(
        Box((hood_width, 0.036, 0.015)),
        origin=Origin(xyz=(0.0, 0.452, 0.012)),
        material=satin_steel,
        name="front_filter_rail",
    )
    hood_body.visual(
        Box((0.028, 0.380, 0.015)),
        origin=Origin(xyz=(0.0, 0.272, 0.012)),
        material=satin_steel,
        name="center_filter_rail",
    )
    hood_body.visual(
        Box((0.340, 0.350, 0.010)),
        origin=Origin(xyz=(-0.185, 0.272, 0.005)),
        material=dark_filter,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.340, 0.350, 0.010)),
        origin=Origin(xyz=(0.185, 0.272, 0.005)),
        material=dark_filter,
        name="right_filter",
    )
    hood_body.visual(
        Box((0.090, 0.018, 0.005)),
        origin=Origin(xyz=(-0.245, 0.410, 0.006)),
        material=glass_light,
        name="left_light_panel",
    )
    hood_body.visual(
        Box((0.090, 0.018, 0.005)),
        origin=Origin(xyz=(0.245, 0.410, 0.006)),
        material=glass_light,
        name="right_light_panel",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((hood_width, hood_depth, hood_height)),
        mass=14.0,
        origin=Origin(xyz=(0.0, hood_depth * 0.5, hood_height * 0.5)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_width = 0.34
    chimney_depth = 0.26
    chimney_height = 0.40
    chimney_sheet = 0.005
    chimney_cover.visual(
        Box((chimney_width, chimney_sheet, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_sheet * 0.5, chimney_height * 0.5)),
        material=steel,
        name="chimney_back",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_sheet, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_depth - (chimney_sheet * 0.5), chimney_height * 0.5)
        ),
        material=steel,
        name="chimney_front",
    )
    chimney_cover.visual(
        Box((chimney_sheet, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -(chimney_width * 0.5) + (chimney_sheet * 0.5),
                chimney_depth * 0.5,
                chimney_height * 0.5,
            )
        ),
        material=steel,
        name="chimney_left",
    )
    chimney_cover.visual(
        Box((chimney_sheet, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                (chimney_width * 0.5) - (chimney_sheet * 0.5),
                chimney_depth * 0.5,
                chimney_height * 0.5,
            )
        ),
        material=steel,
        name="chimney_right",
    )
    chimney_cover.visual(
        Box((chimney_width, chimney_depth, chimney_sheet)),
        origin=Origin(
            xyz=(0.0, chimney_depth * 0.5, chimney_height - (chimney_sheet * 0.5))
        ),
        material=satin_steel,
        name="chimney_top_cap",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, chimney_depth * 0.5, chimney_height * 0.5)),
    )
    model.articulation(
        "hood_to_chimney_cover",
        ArticulationType.FIXED,
        parent=hood_body,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, 0.090, hood_height)),
    )

    button_angles = (-0.090, -0.045, 0.0, 0.045, 0.090)
    for index, theta in enumerate(button_angles, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=button_black,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.024, 0.024, 0.020)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=_button_mount_origin(
                theta=theta,
                outer_radius=strip_outer_radius + 0.001,
                centerline_y=strip_centerline_y,
                z=button_z,
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=-0.004,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    chimney_cover = object_model.get_part("chimney_cover")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    button_joints = [
        object_model.get_articulation(f"hood_to_button_{index}") for index in range(1, 6)
    ]
    hood_to_chimney = object_model.get_articulation("hood_to_chimney_cover")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    for button in buttons:
        ctx.allow_overlap(
            button,
            hood_body,
            reason="Round plunger stems slide inside concealed switch guides behind the fascia.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(chimney_cover, hood_body, name="chimney_cover_seats_on_hood")
    ctx.expect_gap(
        chimney_cover,
        hood_body,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        name="chimney_cover_bottom_flush",
    )
    ctx.expect_overlap(
        chimney_cover,
        hood_body,
        axes="xy",
        min_overlap=0.20,
        name="chimney_cover_centered_over_canopy",
    )

    hood_aabb = ctx.part_world_aabb(hood_body)
    chimney_aabb = ctx.part_world_aabb(chimney_cover)
    ctx.check("hood_body_present", hood_aabb is not None, "hood_body should have world geometry")
    ctx.check(
        "chimney_cover_present",
        chimney_aabb is not None,
        "chimney_cover should have world geometry",
    )
    if hood_aabb is not None:
        hood_size = (
            hood_aabb[1][0] - hood_aabb[0][0],
            hood_aabb[1][1] - hood_aabb[0][1],
            hood_aabb[1][2] - hood_aabb[0][2],
        )
        ctx.check(
            "hood_body_realistic_size",
            0.84 <= hood_size[0] <= 0.95
            and 0.47 <= hood_size[1] <= 0.52
            and 0.15 <= hood_size[2] <= 0.17,
            f"Unexpected hood body size: {hood_size}",
        )
    if chimney_aabb is not None:
        chimney_size = (
            chimney_aabb[1][0] - chimney_aabb[0][0],
            chimney_aabb[1][1] - chimney_aabb[0][1],
            chimney_aabb[1][2] - chimney_aabb[0][2],
        )
        ctx.check(
            "chimney_cover_realistic_size",
            0.30 <= chimney_size[0] <= 0.36
            and 0.24 <= chimney_size[1] <= 0.28
            and 0.39 <= chimney_size[2] <= 0.41,
            f"Unexpected chimney cover size: {chimney_size}",
        )

    button_positions: list[tuple[float, float, float]] = []
    for index, button in enumerate(buttons, start=1):
        ctx.expect_overlap(
            button,
            hood_body,
            axes="xz",
            elem_a="button_cap",
            elem_b="front_control_strip",
            min_overlap=0.012,
            name=f"button_{index}_registered_on_front_strip",
        )
        position = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_present",
            position is not None,
            f"button_{index} should resolve to a world position",
        )
        if position is not None:
            button_positions.append(position)

    if len(button_positions) == 5:
        ordered_x = all(
            left[0] < right[0] for left, right in zip(button_positions[:-1], button_positions[1:])
        )
        arc_forward = (
            button_positions[2][1] > button_positions[1][1] > button_positions[0][1]
            and button_positions[2][1] > button_positions[3][1] > button_positions[4][1]
        )
        level_row = max(abs(position[2] - button_positions[2][2]) for position in button_positions) <= 0.003
        ctx.check("buttons_ordered_left_to_right", ordered_x, f"Button x positions: {button_positions}")
        ctx.check("buttons_follow_gentle_arc", arc_forward, f"Button y positions: {button_positions}")
        ctx.check("buttons_share_common_height", level_row, f"Button z positions: {button_positions}")

    for index, (button, joint) in enumerate(zip(buttons, button_joints), start=1):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index}_has_motion_limits",
            limits is not None and limits.lower == -0.004 and limits.upper == 0.0,
            f"Unexpected limits on {joint.name}: {limits}",
        )
        rest = ctx.part_world_position(button)
        if rest is None or limits is None:
            continue
        with ctx.pose({joint: limits.lower}):
            pressed = ctx.part_world_position(button)
            ctx.fail_if_parts_overlap_in_current_pose(name=f"button_{index}_pressed_no_new_overlap")
            ctx.fail_if_isolated_parts(name=f"button_{index}_pressed_no_floating")
            if pressed is None:
                ctx.fail(f"button_{index}_pressed_position", "Pressed button position should exist")
                continue
            travel = math.dist(rest, pressed)
            ctx.check(
                f"button_{index}_travel_distance",
                0.0035 <= travel <= 0.0045,
                f"Unexpected travel on button_{index}: {travel}",
            )
            ctx.check(
                f"button_{index}_moves_inward",
                pressed[1] < rest[1] - 0.003,
                f"Button_{index} should move rearward when pressed: rest={rest}, pressed={pressed}",
            )
            if index < 3:
                ctx.check(
                    f"button_{index}_leans_toward_center",
                    pressed[0] > rest[0] + 0.0001,
                    f"Left-side button_{index} should move inward toward center: rest={rest}, pressed={pressed}",
                )
            elif index == 3:
                ctx.check(
                    f"button_{index}_center_stays_centered",
                    abs(pressed[0] - rest[0]) <= 0.0005,
                    f"Center button should plunge almost straight back: rest={rest}, pressed={pressed}",
                )
            else:
                ctx.check(
                    f"button_{index}_leans_toward_center",
                    pressed[0] < rest[0] - 0.0001,
                    f"Right-side button_{index} should move inward toward center: rest={rest}, pressed={pressed}",
                )

    ctx.check(
        "only_buttons_articulate",
        hood_to_chimney.articulation_type == ArticulationType.FIXED
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints)
        and len(object_model.articulations) == 6,
        "Expected one fixed chimney mount and five prismatic button plungers only.",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    enamel = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    trim = model.material("trim_grey", rgba=(0.80, 0.81, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.22, 1.0))
    black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    lens = model.material("lamp_lens", rgba=(0.93, 0.89, 0.72, 0.78))

    hood_body = model.part("hood_body")

    body_width = 0.760
    body_depth = 0.500
    body_height = 0.180
    shell_thickness = 0.012
    front_valance_depth = 0.030
    front_face_center_y = -body_depth * 0.5 + front_valance_depth * 0.5
    front_face_outer_y = -body_depth * 0.5

    hood_body.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - shell_thickness * 0.5)),
        material=enamel,
        name="top_panel",
    )
    hood_body.visual(
        Box((shell_thickness, body_depth, body_height - shell_thickness)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + shell_thickness * 0.5,
                0.0,
                (body_height - shell_thickness) * 0.5,
            )
        ),
        material=enamel,
        name="left_side_panel",
    )
    hood_body.visual(
        Box((shell_thickness, body_depth, body_height - shell_thickness)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - shell_thickness * 0.5,
                0.0,
                (body_height - shell_thickness) * 0.5,
            )
        ),
        material=enamel,
        name="right_side_panel",
    )
    hood_body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, body_height - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth * 0.5 - shell_thickness * 0.5,
                (body_height - shell_thickness) * 0.5,
            )
        ),
        material=enamel,
        name="back_panel",
    )

    opening_width = 0.230
    opening_height = 0.040
    opening_center_x = -0.200
    opening_center_z = 0.050
    front_span_width = body_width - 2.0 * shell_thickness
    front_inner_left = -front_span_width * 0.5
    front_inner_right = front_span_width * 0.5
    opening_left = opening_center_x - opening_width * 0.5
    opening_right = opening_center_x + opening_width * 0.5

    hood_body.visual(
        Box((front_span_width, front_valance_depth, 0.098)),
        origin=Origin(xyz=(0.0, front_face_center_y, 0.119)),
        material=enamel,
        name="front_upper_valance",
    )
    hood_body.visual(
        Box((front_span_width, front_valance_depth, 0.030)),
        origin=Origin(xyz=(0.0, front_face_center_y, 0.015)),
        material=enamel,
        name="front_lower_valance",
    )
    hood_body.visual(
        Box((opening_left - front_inner_left, front_valance_depth, opening_height)),
        origin=Origin(
            xyz=(
                (front_inner_left + opening_left) * 0.5,
                front_face_center_y,
                opening_center_z,
            )
        ),
        material=enamel,
        name="front_left_jamb",
    )
    hood_body.visual(
        Box((front_inner_right - opening_right, front_valance_depth, opening_height)),
        origin=Origin(
            xyz=(
                (opening_right + front_inner_right) * 0.5,
                front_face_center_y,
                opening_center_z,
            )
        ),
        material=enamel,
        name="front_right_field",
    )
    def rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (cx - half_w, cy - half_h),
            (cx + half_w, cy - half_h),
            (cx + half_w, cy + half_h),
            (cx - half_w, cy + half_h),
        ]

    def circle_profile(
        radius: float,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        segments: int = 24,
    ) -> list[tuple[float, float]]:
        return [
            (
                cx + radius * math.cos(2.0 * math.pi * index / segments),
                cy + radius * math.sin(2.0 * math.pi * index / segments),
            )
            for index in range(segments)
        ]

    filter_width = body_width - 2.0 * shell_thickness
    filter_depth = 0.446
    filter_thickness = 0.008
    filter_holes: list[list[tuple[float, float]]] = []
    for hole_x in (-0.255, -0.153, -0.051, 0.051, 0.153, 0.255):
        for hole_y in (-0.144, -0.048, 0.048, 0.144):
            filter_holes.append(rect_profile(0.052, 0.026, cx=hole_x, cy=hole_y))
    intake_filter_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rect_profile(filter_width, filter_depth),
            filter_holes,
            height=filter_thickness,
            center=True,
        ),
        ASSETS.mesh_path("range_hood_intake_filter.obj"),
    )
    hood_body.visual(
        intake_filter_mesh,
        origin=Origin(xyz=(0.0, 0.015, filter_thickness * 0.5)),
        material=charcoal,
        name="intake_filter",
    )
    hood_body.visual(
        Box((0.500, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, -0.214, 0.005)),
        material=trim,
        name="grease_rail",
    )
    hood_body.visual(
        Box((0.112, 0.050, 0.004)),
        origin=Origin(xyz=(-0.140, -0.040, -0.002)),
        material=lens,
        name="left_lamp_lens",
    )
    hood_body.visual(
        Box((0.112, 0.050, 0.004)),
        origin=Origin(xyz=(0.140, -0.040, -0.002)),
        material=lens,
        name="right_lamp_lens",
    )

    strip_width = 0.290
    strip_height = 0.055
    strip_thickness = 0.003
    knob_offset_x = -0.095
    button_light_offset_x = 0.025
    button_fan_offset_x = 0.060
    control_strip_geom = ExtrudeWithHolesGeometry(
        rect_profile(strip_width, strip_height),
        [
            circle_profile(0.012, cx=knob_offset_x, cy=0.0),
            rect_profile(0.012, 0.012, cx=button_light_offset_x, cy=0.0),
            rect_profile(0.012, 0.012, cx=button_fan_offset_x, cy=0.0),
        ],
        height=strip_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    control_strip_mesh = mesh_from_geometry(
        control_strip_geom,
        ASSETS.mesh_path("range_hood_control_strip.obj"),
    )
    hood_body.visual(
        control_strip_mesh,
        origin=Origin(
            xyz=(
                opening_center_x,
                front_face_outer_y - strip_thickness * 0.5,
                opening_center_z,
            )
        ),
        material=trim,
        name="control_strip_plate",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.0, 0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.004, 0.0025, 0.010)),
        origin=Origin(xyz=(0.013, -0.0160, 0.0)),
        material=trim,
        name="knob_pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.046, 0.036, 0.046)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
    )

    light_button = model.part("light_button")
    light_button.visual(
        Box((0.012, 0.009, 0.012)),
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
        material=charcoal,
        name="button_plunger",
    )
    light_button.inertial = Inertial.from_geometry(
        Box((0.012, 0.009, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
    )

    fan_button = model.part("fan_button")
    fan_button.visual(
        Box((0.012, 0.009, 0.012)),
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
        material=charcoal,
        name="button_plunger",
    )
    fan_button.inertial = Inertial.from_geometry(
        Box((0.012, 0.009, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
    )

    control_plane_y = front_face_outer_y - strip_thickness * 0.5
    model.articulation(
        "selector_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child=selector_knob,
        origin=Origin(
            xyz=(
                opening_center_x + knob_offset_x,
                control_plane_y,
                opening_center_z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "light_button_press",
        ArticulationType.PRISMATIC,
        parent=hood_body,
        child=light_button,
        origin=Origin(
            xyz=(
                opening_center_x + button_light_offset_x,
                control_plane_y,
                opening_center_z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0010,
        ),
    )
    model.articulation(
        "fan_button_press",
        ArticulationType.PRISMATIC,
        parent=hood_body,
        child=fan_button,
        origin=Origin(
            xyz=(
                opening_center_x + button_fan_offset_x,
                control_plane_y,
                opening_center_z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    selector_knob = object_model.get_part("selector_knob")
    light_button = object_model.get_part("light_button")
    fan_button = object_model.get_part("fan_button")

    knob_spin = object_model.get_articulation("selector_knob_spin")
    light_press = object_model.get_articulation("light_button_press")
    fan_press = object_model.get_articulation("fan_button_press")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    hood_aabb = ctx.part_world_aabb(hood_body)
    if hood_aabb is None:
        ctx.fail("hood_body_has_geometry", "hood_body AABB is missing")
    else:
        hood_width = hood_aabb[1][0] - hood_aabb[0][0]
        hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
        hood_height = hood_aabb[1][2] - hood_aabb[0][2]
        ctx.check(
            "range_hood_realistic_proportions",
            0.72 <= hood_width <= 0.82 and 0.46 <= hood_depth <= 0.54 and 0.16 <= hood_height <= 0.20,
            details=(
                f"got width={hood_width:.3f} depth={hood_depth:.3f} height={hood_height:.3f}"
            ),
        )

    ctx.expect_contact(
        selector_knob,
        hood_body,
        elem_a="knob_cap",
        elem_b="control_strip_plate",
        name="selector_knob_rest_contact",
    )
    ctx.expect_contact(light_button, hood_body, name="light_button_rest_contact")
    ctx.expect_contact(fan_button, hood_body, name="fan_button_rest_contact")
    ctx.expect_origin_gap(
        light_button,
        selector_knob,
        axis="x",
        min_gap=0.10,
        max_gap=0.14,
        name="light_button_right_of_knob",
    )
    ctx.expect_origin_gap(
        fan_button,
        light_button,
        axis="x",
        min_gap=0.025,
        max_gap=0.045,
        name="fan_button_grouped_right_of_light_button",
    )
    ctx.expect_within(
        selector_knob,
        hood_body,
        axes="xz",
        margin=0.0,
        name="selector_knob_within_front_envelope",
    )
    ctx.expect_within(
        light_button,
        hood_body,
        axes="xz",
        margin=0.0,
        name="light_button_within_front_envelope",
    )
    ctx.expect_within(
        fan_button,
        hood_body,
        axes="xz",
        margin=0.0,
        name="fan_button_within_front_envelope",
    )

    knob_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({knob_spin: math.pi / 2.0}):
        knob_quarter = ctx.part_world_position(selector_knob)
        if knob_rest is None or knob_quarter is None:
            ctx.fail("selector_knob_pose_measurement", "could not measure knob pose positions")
        else:
            ctx.check(
                "selector_knob_rotates_in_place",
                abs(knob_quarter[0] - knob_rest[0]) < 1e-6
                and abs(knob_quarter[1] - knob_rest[1]) < 1e-6
                and abs(knob_quarter[2] - knob_rest[2]) < 1e-6,
                details=f"rest={knob_rest}, quarter_turn={knob_quarter}",
            )
        ctx.expect_contact(
            selector_knob,
            hood_body,
            elem_a="knob_cap",
            elem_b="control_strip_plate",
            name="selector_knob_quarter_turn_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="selector_knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="selector_knob_quarter_turn_no_floating")

    for button, joint, label in (
        (light_button, light_press, "light_button"),
        (fan_button, fan_press, "fan_button"),
    ):
        limits = joint.motion_limits
        rest_position = ctx.part_world_position(button)
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.expect_contact(button, hood_body, name=f"{label}_lower_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                pressed_position = ctx.part_world_position(button)
                if rest_position is None or pressed_position is None:
                    ctx.fail(f"{label}_pressed_position_measured", "could not measure button pose")
                else:
                    ctx.check(
                        f"{label}_moves_inward",
                        pressed_position[1] > rest_position[1] + 0.0007,
                        details=f"rest={rest_position}, pressed={pressed_position}",
                    )
                ctx.expect_contact(button, hood_body, name=f"{label}_upper_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")

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

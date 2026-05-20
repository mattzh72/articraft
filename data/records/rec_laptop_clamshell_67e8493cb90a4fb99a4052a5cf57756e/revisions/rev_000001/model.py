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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rotate_x_point(
    point: tuple[float, float, float],
    angle: float,
) -> tuple[float, float, float]:
    x, y, z = point
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        x,
        y * cos_a - z * sin_a,
        y * sin_a + z * cos_a,
    )


def _build_keycap_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
):
    lower = rounded_rect_profile(width, depth, radius=min(width, depth) * 0.18, corner_segments=5)
    upper = rounded_rect_profile(
        width - 0.0018,
        depth - 0.0014,
        radius=min(width - 0.0018, depth - 0.0014) * 0.16,
        corner_segments=5,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _build_lid_shell_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
):
    lower = rounded_rect_profile(width, height, radius=0.011, corner_segments=6)
    upper = rounded_rect_profile(width - 0.004, height - 0.004, radius=0.009, corner_segments=6)
    geom = LoftGeometry(
        [
            [(x, y, -thickness * 0.5) for x, y in lower],
            [(x, y, thickness * 0.5) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _build_rear_foot_mesh(
    name: str,
    *,
    arm_span: float,
    arm_length: float,
):
    left_arm = BoxGeometry((0.010, arm_length, 0.006))
    left_arm.translate(-arm_span * 0.5, -arm_length * 0.5, -0.007)

    right_arm = BoxGeometry((0.010, arm_length, 0.006))
    right_arm.translate(arm_span * 0.5, -arm_length * 0.5, -0.007)

    crossbar = BoxGeometry((arm_span + 0.026, 0.008, 0.010))
    crossbar.translate(0.0, -arm_length + 0.004, -0.010)

    left_hinge_pad = BoxGeometry((0.016, 0.008, 0.006))
    left_hinge_pad.translate(-arm_span * 0.5, -0.003, -0.007)

    right_hinge_pad = BoxGeometry((0.016, 0.008, 0.006))
    right_hinge_pad.translate(arm_span * 0.5, -0.003, -0.007)

    foot = left_arm
    foot.merge(right_arm)
    foot.merge(crossbar)
    foot.merge(left_hinge_pad)
    foot.merge(right_hinge_pad)
    return mesh_from_geometry(foot, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="creator_laptop")

    chassis_mat = model.material("chassis_graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    deck_mat = model.material("deck_black", rgba=(0.14, 0.15, 0.17, 1.0))
    key_mat = model.material("key_charcoal", rgba=(0.07, 0.08, 0.09, 1.0))
    glass_mat = model.material("screen_glass", rgba=(0.05, 0.08, 0.12, 1.0))
    trackpad_mat = model.material("trackpad_glass", rgba=(0.33, 0.35, 0.38, 1.0))
    foot_mat = model.material("foot_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    camera_mat = model.material("camera_black", rgba=(0.02, 0.02, 0.03, 1.0))

    base_width = 0.356
    base_depth = 0.242
    lower_shell_height = 0.010
    deck_height = 0.006
    total_base_height = lower_shell_height + deck_height

    keyboard_width = 0.286
    keyboard_depth = 0.108
    keyboard_center_y = 0.020

    lid_width = 0.354
    lid_height = 0.224
    lid_thickness = 0.0075
    nominal_lid_open = 1.72

    key_width = 0.0182
    key_depth = 0.0170
    key_height = 0.0028
    key_travel = 0.0018
    keyboard_plate_thickness = 0.0008
    keyboard_plate_top = total_base_height - 0.0002

    key_mesh = _build_keycap_mesh(
        "creator_laptop_keycap",
        width=key_width,
        depth=key_depth,
        height=key_height,
    )
    lid_shell_mesh = _build_lid_shell_mesh(
        "creator_laptop_lid_shell",
        width=lid_width,
        height=lid_height,
        thickness=lid_thickness,
    )
    rear_foot_mesh = _build_rear_foot_mesh(
        "creator_laptop_rear_foot",
        arm_span=0.126,
        arm_length=0.040,
    )

    base = model.part("base")
    x_pitch = 0.023
    y_pitch = 0.021
    column_centers = [(-5.5 + column) * x_pitch for column in range(12)]
    row_centers = [0.056 - row * y_pitch for row in range(4)]
    row_offsets = (0.000, 0.004, 0.008, 0.012)

    key_hole_profiles = []
    for row, row_y in enumerate(row_centers):
        for column, column_x in enumerate(column_centers):
            key_hole_profiles.append(
                _shift_profile(
                    rounded_rect_profile(0.0136, 0.0118, radius=0.0020, corner_segments=4),
                    dx=column_x + row_offsets[row],
                    dy=row_y,
                )
            )

    base_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(base_width, base_depth, radius=0.014, corner_segments=8),
            lower_shell_height,
            cap=True,
            center=True,
            closed=True,
        ),
        "creator_laptop_base_shell",
    )
    keyboard_opening = _shift_profile(
        rounded_rect_profile(keyboard_width, keyboard_depth, radius=0.006, corner_segments=6),
        dy=keyboard_center_y,
    )
    deck_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(base_width, base_depth, radius=0.014, corner_segments=8),
            [keyboard_opening],
            deck_height,
            cap=True,
            center=True,
            closed=True,
        ),
        "creator_laptop_deck_frame",
    )
    keyboard_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _shift_profile(
                rounded_rect_profile(keyboard_width + 0.012, keyboard_depth + 0.012, radius=0.008, corner_segments=6),
                dy=keyboard_center_y,
            ),
            key_hole_profiles,
            keyboard_plate_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        "creator_laptop_keyboard_plate",
    )
    base.visual(
        base_shell,
        origin=Origin(xyz=(0.0, 0.0, lower_shell_height * 0.5)),
        material=chassis_mat,
        name="lower_shell",
    )
    base.visual(
        deck_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, lower_shell_height + deck_height * 0.5)),
        material=deck_mat,
        name="deck_frame",
    )
    base.visual(
        keyboard_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, keyboard_plate_top - keyboard_plate_thickness * 0.5)),
        material=deck_mat,
        name="keyboard_plate",
    )
    base.visual(
        Box((0.150, 0.092, 0.0012)),
        origin=Origin(xyz=(0.0, -0.058, total_base_height + 0.0006)),
        material=trackpad_mat,
        name="touchpad",
    )
    base.visual(
        Box((0.268, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, base_depth * 0.5 - 0.010, total_base_height + 0.001)),
        material=chassis_mat,
        name="rear_hinge_bar",
    )
    base.visual(
        Box((0.188, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, base_depth * 0.5 - 0.017, 0.002)),
        material=foot_mat,
        name="rear_foot_mount",
    )

    lid = model.part("lid")
    lid_center = (0.0, -lid_height * 0.5, lid_thickness * 0.5)
    screen_center = (0.0, -lid_height * 0.5, 0.0014)
    webcam_center = (0.0, -0.010, 0.0012)
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=lid_center),
        material=chassis_mat,
        name="lid_shell",
    )
    lid.visual(
        Box((0.326, 0.198, 0.0014)),
        origin=Origin(xyz=screen_center),
        material=glass_mat,
        name="screen_panel",
    )
    lid.visual(
        Box((0.334, 0.206, 0.0015)),
        origin=Origin(xyz=(0.0, -0.004, 0.00270)),
        material=deck_mat,
        name="display_carrier",
    )
    lid.visual(
        Box((0.010, 0.003, 0.0015)),
        origin=Origin(xyz=webcam_center),
        material=camera_mat,
        name="webcam_bar",
    )
    lid.visual(
        Box((0.286, 0.010, 0.0045)),
        origin=Origin(xyz=(0.0, 0.003, 0.00225)),
        material=chassis_mat,
        name="lid_hinge_web",
    )
    lid.visual(
        Box((0.248, 0.008, 0.0040)),
        origin=Origin(xyz=(0.0, 0.007, 0.0020)),
        material=chassis_mat,
        name="lid_hinge_sleeve",
    )

    rear_foot = model.part("rear_foot")
    rear_foot.visual(
        rear_foot_mesh,
        material=foot_mat,
        name="rear_foot_body",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, base_depth * 0.5 - 0.017, total_base_height + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "base_to_rear_foot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_foot,
        origin=Origin(xyz=(0.0, base_depth * 0.5 - 0.018, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    key_origin_z = keyboard_plate_top

    for row, row_y in enumerate(row_centers):
        for column, column_x in enumerate(column_centers):
            key_part = model.part(f"key_r{row}_c{column}")
            key_part.visual(
                key_mesh,
                origin=Origin(xyz=(0.0, 0.0, key_height * 0.5)),
                material=key_mat,
                name="keycap",
            )
            key_part.visual(
                Box((0.0110, 0.0100, 0.0036)),
                origin=Origin(xyz=(0.0, 0.0, 0.0002)),
                material=key_mat,
                name="plunger_stem",
            )
            key_part.visual(
                Box((0.0200, 0.0180, 0.0008)),
                origin=Origin(xyz=(0.0, 0.0, -0.0012)),
                material=key_mat,
                name="retainer_flange",
            )
            model.articulation(
                f"base_to_key_r{row}_c{column}",
                ArticulationType.PRISMATIC,
                parent=base,
                child=key_part,
                origin=Origin(
                    xyz=(
                        column_x + row_offsets[row],
                        row_y,
                        key_origin_z,
                    )
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.4,
                    velocity=0.05,
                    lower=0.0,
                    upper=key_travel,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    rear_foot = object_model.get_part("rear_foot")
    lid_hinge = object_model.get_articulation("base_to_lid")
    foot_hinge = object_model.get_articulation("base_to_rear_foot")

    key_a = object_model.get_part("key_r0_c0")
    key_b = object_model.get_part("key_r1_c5")
    key_c = object_model.get_part("key_r3_c11")
    key_joint_a = object_model.get_articulation("base_to_key_r0_c0")
    key_joint_b = object_model.get_articulation("base_to_key_r1_c5")
    key_joint_c = object_model.get_articulation("base_to_key_r3_c11")

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.0045,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="deck_frame",
        name="closed lid clears the keyboard deck without penetrating it",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.22,
        elem_a="lid_shell",
        name="closed lid covers the chassis footprint",
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="lid_hinge_web",
        elem_b="rear_hinge_bar",
        contact_tol=1e-6,
        name="lid hinge web is supported by the rear hinge bar in the closed pose",
    )
    with ctx.pose({lid_hinge: 1.24}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the closed pose",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    stowed_foot_aabb = ctx.part_element_world_aabb(rear_foot, elem="rear_foot_body")
    ctx.expect_gap(
        base,
        rear_foot,
        axis="z",
        max_gap=0.012,
        max_penetration=0.0002,
        negative_elem="rear_foot_body",
        name="stowed rear foot tucks just beneath the base",
    )
    with ctx.pose({foot_hinge: foot_hinge.motion_limits.upper}):
        deployed_foot_aabb = ctx.part_element_world_aabb(rear_foot, elem="rear_foot_body")
    ctx.check(
        "rear foot deploys downward from its tucked pose",
        stowed_foot_aabb is not None
        and deployed_foot_aabb is not None
        and deployed_foot_aabb[0][2] < stowed_foot_aabb[0][2] - 0.010,
        details=f"stowed={stowed_foot_aabb}, deployed={deployed_foot_aabb}",
    )

    def _check_key_press(key_part, key_joint, label: str) -> None:
        rest_pos = ctx.part_world_position(key_part)
        with ctx.pose({key_joint: key_joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(key_part)
        ctx.check(
            f"{label} plunges downward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.0012,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    _check_key_press(key_a, key_joint_a, "front-left key")
    _check_key_press(key_b, key_joint_b, "home-row key")
    _check_key_press(key_c, key_joint_c, "bottom-right key")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

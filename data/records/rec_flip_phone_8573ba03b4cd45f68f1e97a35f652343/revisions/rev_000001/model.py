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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LOWER_WIDTH = 0.064
LOWER_LENGTH = 0.090
LOWER_THICKNESS = 0.018
LOWER_RADIUS = 0.010

UPPER_WIDTH = 0.056
UPPER_LENGTH = 0.082
UPPER_THICKNESS = 0.015
UPPER_RADIUS = 0.009

HINGE_RADIUS = 0.0065
HINGE_AXIS_Y = LOWER_LENGTH * 0.5 + 0.0075
LOWER_BARREL_LENGTH = 0.017
UPPER_BARREL_LENGTH = 0.020
BARREL_OFFSET_X = 0.0185
UPPER_SHELL_CENTER_Y = 0.051


def _rounded_body_mesh(name: str, width: float, length: float, thickness: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, length, radius), thickness),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="durable_flip_phone")

    shell_dark = model.material("shell_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.28, 0.30, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.47, 0.49, 0.52, 1.0))
    keypad = model.material("keypad", rgba=(0.16, 0.17, 0.18, 1.0))
    key_textured = model.material("key_textured", rgba=(0.24, 0.26, 0.28, 1.0))
    call_green = model.material("call_green", rgba=(0.15, 0.46, 0.22, 1.0))
    end_red = model.material("end_red", rgba=(0.58, 0.16, 0.15, 1.0))
    display_glass = model.material("display_glass", rgba=(0.12, 0.22, 0.25, 0.70))
    cover_glass = model.material("cover_glass", rgba=(0.07, 0.09, 0.10, 0.85))
    speaker_metal = model.material("speaker_metal", rgba=(0.54, 0.56, 0.59, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        _rounded_body_mesh(
            "lower_shell",
            LOWER_WIDTH,
            LOWER_LENGTH,
            LOWER_THICKNESS,
            LOWER_RADIUS,
        ),
        material=shell_mid,
        name="lower_shell",
    )
    lower_body.visual(
        Box((LOWER_WIDTH - 0.010, LOWER_LENGTH - 0.016, 0.0028)),
        origin=Origin(xyz=(0.0, -0.003, LOWER_THICKNESS * 0.5 - 0.0010)),
        material=shell_dark,
        name="lower_front_plate",
    )
    lower_body.visual(
        Box((0.006, LOWER_LENGTH - 0.016, LOWER_THICKNESS + 0.001)),
        origin=Origin(xyz=(LOWER_WIDTH * 0.5 - 0.002, 0.0, 0.0)),
        material=rubber,
        name="lower_right_bumper",
    )
    lower_body.visual(
        Box((0.006, LOWER_LENGTH - 0.016, LOWER_THICKNESS + 0.001)),
        origin=Origin(xyz=(-LOWER_WIDTH * 0.5 + 0.002, 0.0, 0.0)),
        material=rubber,
        name="lower_left_bumper",
    )
    lower_body.visual(
        Box((0.052, 0.056, 0.0020)),
        origin=Origin(xyz=(0.0, -0.014, LOWER_THICKNESS * 0.5 - 0.0004)),
        material=keypad,
        name="keypad_deck",
    )
    lower_body.visual(
        Box((0.040, 0.018, 0.0022)),
        origin=Origin(xyz=(0.0, 0.022, LOWER_THICKNESS * 0.5 - 0.0003)),
        material=keypad,
        name="control_deck",
    )
    lower_body.visual(
        Cylinder(radius=0.011, length=0.0028),
        origin=Origin(xyz=(0.0, 0.019, LOWER_THICKNESS * 0.5 + 0.0003)),
        material=key_textured,
        name="nav_ring",
    )
    lower_body.visual(
        Cylinder(radius=0.0052, length=0.0030),
        origin=Origin(xyz=(0.0, 0.019, LOWER_THICKNESS * 0.5 + 0.00045)),
        material=shell_mid,
        name="nav_center",
    )
    lower_body.visual(
        Box((0.010, 0.006, 0.0028)),
        origin=Origin(xyz=(-0.017, 0.021, LOWER_THICKNESS * 0.5 + 0.00025)),
        material=key_textured,
        name="soft_key_left",
    )
    lower_body.visual(
        Box((0.010, 0.006, 0.0028)),
        origin=Origin(xyz=(0.017, 0.021, LOWER_THICKNESS * 0.5 + 0.00025)),
        material=key_textured,
        name="soft_key_right",
    )
    lower_body.visual(
        Box((0.011, 0.006, 0.0028)),
        origin=Origin(xyz=(-0.017, 0.010, LOWER_THICKNESS * 0.5 + 0.00025)),
        material=call_green,
        name="call_key",
    )
    lower_body.visual(
        Box((0.011, 0.006, 0.0028)),
        origin=Origin(xyz=(0.017, 0.010, LOWER_THICKNESS * 0.5 + 0.00025)),
        material=end_red,
        name="end_key",
    )

    key_centers_x = (-0.016, 0.0, 0.016)
    key_centers_y = (-0.006, -0.019, -0.032, -0.045)
    for row_index, key_y in enumerate(key_centers_y):
        for col_index, key_x in enumerate(key_centers_x):
            lower_body.visual(
                Box((0.0105, 0.0080, 0.0028)),
                origin=Origin(
                    xyz=(key_x, key_y, LOWER_THICKNESS * 0.5 + 0.00025)
                ),
                material=key_textured,
                name=f"number_key_{row_index}_{col_index}",
            )

    lower_body.visual(
        Box((0.018, 0.0035, 0.0018)),
        origin=Origin(xyz=(0.0, -0.039, LOWER_THICKNESS * 0.5 - 0.00015)),
        material=speaker_metal,
        name="microphone_slot",
    )
    lower_body.visual(
        Box((0.013, 0.012, 0.010)),
        origin=Origin(xyz=(-BARREL_OFFSET_X, HINGE_AXIS_Y - 0.0055, 0.0)),
        material=rubber,
        name="lower_left_hinge_support",
    )
    lower_body.visual(
        Box((0.013, 0.012, 0.010)),
        origin=Origin(xyz=(BARREL_OFFSET_X, HINGE_AXIS_Y - 0.0055, 0.0)),
        material=rubber,
        name="lower_right_hinge_support",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=LOWER_BARREL_LENGTH),
        origin=Origin(
            xyz=(-BARREL_OFFSET_X, HINGE_AXIS_Y, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="lower_left_barrel",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=LOWER_BARREL_LENGTH),
        origin=Origin(
            xyz=(BARREL_OFFSET_X, HINGE_AXIS_Y, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="lower_right_barrel",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((LOWER_WIDTH, LOWER_LENGTH, LOWER_THICKNESS + 0.008)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        Cylinder(radius=HINGE_RADIUS * 0.96, length=UPPER_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="upper_center_barrel",
    )
    upper_body.visual(
        Box((0.012, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=hinge_metal,
        name="upper_hinge_tongue",
    )
    upper_body.visual(
        _rounded_body_mesh(
            "upper_shell",
            UPPER_WIDTH,
            UPPER_LENGTH,
            UPPER_THICKNESS,
            UPPER_RADIUS,
        ),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y, 0.0)),
        material=shell_mid,
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.0055, UPPER_LENGTH - 0.014, UPPER_THICKNESS + 0.0008)),
        origin=Origin(xyz=(UPPER_WIDTH * 0.5 - 0.0018, UPPER_SHELL_CENTER_Y, 0.0)),
        material=rubber,
        name="upper_right_bumper",
    )
    upper_body.visual(
        Box((0.0055, UPPER_LENGTH - 0.014, UPPER_THICKNESS + 0.0008)),
        origin=Origin(xyz=(-UPPER_WIDTH * 0.5 + 0.0018, UPPER_SHELL_CENTER_Y, 0.0)),
        material=rubber,
        name="upper_left_bumper",
    )
    upper_body.visual(
        Box((0.048, 0.060, 0.0023)),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y - 0.001, UPPER_THICKNESS * 0.5 - 0.00055)),
        material=shell_dark,
        name="front_display_bezel",
    )
    upper_body.visual(
        Box((0.037, 0.046, 0.0016)),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y - 0.003, UPPER_THICKNESS * 0.5 + 0.00025)),
        material=display_glass,
        name="main_display_glass",
    )
    upper_body.visual(
        Box((0.018, 0.0038, 0.0018)),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y + 0.024, UPPER_THICKNESS * 0.5 - 0.00015)),
        material=speaker_metal,
        name="earpiece_slot",
    )
    upper_body.visual(
        Box((0.029, 0.019, 0.0016)),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y + 0.011, -UPPER_THICKNESS * 0.5 + 0.0002)),
        material=shell_dark,
        name="rear_window_bezel",
    )
    upper_body.visual(
        Box((0.022, 0.014, 0.0012)),
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y + 0.011, -UPPER_THICKNESS * 0.5 + 0.0006)),
        material=cover_glass,
        name="rear_secondary_window",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((UPPER_WIDTH, UPPER_LENGTH + 0.012, UPPER_THICKNESS)),
        mass=0.08,
        origin=Origin(xyz=(0.0, UPPER_SHELL_CENTER_Y, 0.0)),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=0.0,
            upper=2.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    flip_hinge = object_model.get_articulation("flip_hinge")

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

    ctx.expect_contact(
        lower_body,
        upper_body,
        elem_a="lower_left_barrel",
        elem_b="upper_center_barrel",
        contact_tol=1e-4,
        name="left hinge barrel touches center barrel",
    )
    ctx.expect_contact(
        lower_body,
        upper_body,
        elem_a="lower_right_barrel",
        elem_b="upper_center_barrel",
        contact_tol=1e-4,
        name="right hinge barrel touches center barrel",
    )
    ctx.expect_overlap(
        lower_body,
        upper_body,
        axes="x",
        min_overlap=0.040,
        name="open bodies share hinge width",
    )
    ctx.expect_within(
        upper_body,
        upper_body,
        axes="xy",
        inner_elem="rear_secondary_window",
        outer_elem="upper_shell",
        margin=0.001,
        name="rear display window sits within upper shell footprint",
    )

    lower_aabb = ctx.part_world_aabb(lower_body)
    upper_aabb = ctx.part_world_aabb(upper_body)
    upper_shell_aabb = ctx.part_element_world_aabb(upper_body, elem="upper_shell")
    rear_window_aabb = ctx.part_element_world_aabb(upper_body, elem="rear_secondary_window")
    screen_aabb_rest = ctx.part_element_world_aabb(upper_body, elem="main_display_glass")
    assert lower_aabb is not None
    assert upper_aabb is not None
    assert upper_shell_aabb is not None
    assert rear_window_aabb is not None
    assert screen_aabb_rest is not None

    lower_width = lower_aabb[1][0] - lower_aabb[0][0]
    upper_width = upper_aabb[1][0] - upper_aabb[0][0]
    ctx.check(
        "lower body wider than upper body",
        lower_width > upper_width + 0.006,
        f"lower width={lower_width:.4f}, upper width={upper_width:.4f}",
    )

    rear_face_flush = abs(rear_window_aabb[0][2] - upper_shell_aabb[0][2]) <= 0.0001
    ctx.check(
        "rear secondary window is flush with rear face",
        rear_face_flush,
        (
            f"rear window zmin={rear_window_aabb[0][2]:.4f}, "
            f"upper shell zmin={upper_shell_aabb[0][2]:.4f}"
        ),
    )

    rest_screen_center_z = (screen_aabb_rest[0][2] + screen_aabb_rest[1][2]) * 0.5
    rest_screen_center_y = (screen_aabb_rest[0][1] + screen_aabb_rest[1][1]) * 0.5
    with ctx.pose({flip_hinge: 1.65}):
        screen_aabb_folded = ctx.part_element_world_aabb(upper_body, elem="main_display_glass")
        upper_aabb_folded = ctx.part_world_aabb(upper_body)
        assert screen_aabb_folded is not None
        assert upper_aabb_folded is not None
        folded_screen_center_z = (screen_aabb_folded[0][2] + screen_aabb_folded[1][2]) * 0.5
        folded_screen_center_y = (screen_aabb_folded[0][1] + screen_aabb_folded[1][1]) * 0.5
        ctx.check(
            "hinge rotates display upward when closing",
            folded_screen_center_z > rest_screen_center_z + 0.030
            and folded_screen_center_y < rest_screen_center_y - 0.020,
            (
                f"rest(y,z)=({rest_screen_center_y:.4f}, {rest_screen_center_z:.4f}), "
                f"folded(y,z)=({folded_screen_center_y:.4f}, {folded_screen_center_z:.4f})"
            ),
        )
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="z",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            min_gap=-0.001,
            max_gap=0.060,
            name="folding upper shell stays near but not through lower shell",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

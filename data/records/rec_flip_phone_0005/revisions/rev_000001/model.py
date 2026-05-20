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


def _rounded_slab_mesh(name: str, *, width: float, depth: float, thickness: float, radius: float):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_flip_phone")

    body_rubber = model.material("body_rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    metal = model.material("hinge_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    screen_black = model.material("screen_black", rgba=(0.04, 0.05, 0.06, 1.0))
    glass = model.material("screen_glass", rgba=(0.13, 0.26, 0.33, 0.55))
    keypad_black = model.material("keypad_black", rgba=(0.06, 0.07, 0.08, 1.0))
    keypad_gray = model.material("keypad_gray", rgba=(0.21, 0.22, 0.24, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.09, 0.11, 0.13, 0.85))
    flash_lens = model.material("flash_lens", rgba=(0.80, 0.82, 0.72, 0.95))

    lower_w = 0.066
    lower_d = 0.102
    lower_t = 0.0148

    upper_w = 0.062
    upper_d = 0.094
    upper_t = 0.0138

    hinge_y = -0.057
    hinge_z = 0.0093
    hinge_radius = 0.0049
    center_barrel_len = 0.022
    sleeve_len = 0.014
    hinge_cap_len = 0.006

    shell_gap = 0.0032
    upper_rel_y = upper_d * 0.5 + 0.006
    upper_world_z = lower_t * 0.5 + shell_gap + upper_t * 0.5
    upper_rel_z = upper_world_z - hinge_z

    lower_body = model.part("lower_body")
    lower_body.visual(
        _rounded_slab_mesh(
            "lower_body_shell",
            width=lower_w,
            depth=lower_d,
            thickness=lower_t,
            radius=0.010,
        ),
        material=body_rubber,
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.006, lower_d * 0.74, lower_t * 0.72)),
        origin=Origin(xyz=(lower_w * 0.5 - 0.003, 0.0, 0.0)),
        material=grip_rubber,
        name="right_lower_grip",
    )
    lower_body.visual(
        Box((0.006, lower_d * 0.74, lower_t * 0.72)),
        origin=Origin(xyz=(-(lower_w * 0.5 - 0.003), 0.0, 0.0)),
        material=grip_rubber,
        name="left_lower_grip",
    )
    lower_body.visual(
        Box((lower_w * 0.52, 0.010, lower_t * 0.34)),
        origin=Origin(xyz=(0.0, lower_d * 0.5 - 0.005, -lower_t * 0.08)),
        material=grip_rubber,
        name="front_bumper",
    )
    lower_body.visual(
        Box((0.022, 0.012, 0.0066)),
        origin=Origin(xyz=(0.0, -0.054, 0.0072)),
        material=grip_rubber,
        name="hinge_bridge",
    )
    lower_body.visual(
        Cylinder(radius=hinge_radius, length=center_barrel_len),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="hinge_barrel",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((lower_w, lower_d, lower_t)),
        mass=0.12,
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        _rounded_slab_mesh(
            "upper_body_shell",
            width=upper_w,
            depth=upper_d,
            thickness=upper_t,
            radius=0.009,
        ),
        origin=Origin(xyz=(0.0, upper_rel_y, upper_rel_z)),
        material=body_rubber,
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.0055, upper_d * 0.73, upper_t * 0.70)),
        origin=Origin(
            xyz=(upper_w * 0.5 - 0.00275, upper_rel_y, upper_rel_z),
        ),
        material=grip_rubber,
        name="right_upper_grip",
    )
    upper_body.visual(
        Box((0.0055, upper_d * 0.73, upper_t * 0.70)),
        origin=Origin(
            xyz=(-(upper_w * 0.5 - 0.00275), upper_rel_y, upper_rel_z),
        ),
        material=grip_rubber,
        name="left_upper_grip",
    )
    upper_body.visual(
        Box((upper_w * 0.38, 0.008, upper_t * 0.24)),
        origin=Origin(
            xyz=(
                0.0,
                upper_rel_y + upper_d * 0.5 - 0.0045,
                upper_rel_z + (upper_t * 0.5) + 0.0004,
            ),
        ),
        material=grip_rubber,
        name="outer_top_ridge",
    )
    right_sleeve_x = (center_barrel_len * 0.5) + (sleeve_len * 0.5)
    left_sleeve_x = -right_sleeve_x
    upper_body.visual(
        Cylinder(radius=hinge_radius + 0.0005, length=sleeve_len),
        origin=Origin(xyz=(right_sleeve_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_rubber,
        name="right_hinge_sleeve",
    )
    upper_body.visual(
        Cylinder(radius=hinge_radius + 0.0005, length=sleeve_len),
        origin=Origin(xyz=(left_sleeve_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_rubber,
        name="left_hinge_sleeve",
    )
    cap_x = (center_barrel_len * 0.5) + sleeve_len + (hinge_cap_len * 0.5)
    upper_body.visual(
        Cylinder(radius=0.0062, length=hinge_cap_len),
        origin=Origin(xyz=(cap_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="right_hinge_cap",
    )
    upper_body.visual(
        Cylinder(radius=0.0062, length=hinge_cap_len),
        origin=Origin(xyz=(-cap_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="left_hinge_cap",
    )
    upper_body.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(right_sleeve_x, 0.009, 0.0069)),
        material=body_rubber,
        name="right_hinge_cheek",
    )
    upper_body.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(left_sleeve_x, 0.009, 0.0069)),
        material=body_rubber,
        name="left_hinge_cheek",
    )
    upper_body.visual(
        Box((0.010, 0.046, 0.006)),
        origin=Origin(xyz=(right_sleeve_x, 0.040, 0.0090)),
        material=body_rubber,
        name="right_hinge_arm",
    )
    upper_body.visual(
        Box((0.010, 0.046, 0.006)),
        origin=Origin(xyz=(left_sleeve_x, 0.040, 0.0090)),
        material=body_rubber,
        name="left_hinge_arm",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((upper_w, upper_d, upper_t)),
        mass=0.09,
        origin=Origin(xyz=(0.0, upper_rel_y, upper_rel_z)),
    )

    keypad = model.part("keypad_module")
    keypad.visual(
        Box((0.056, 0.076, 0.0009)),
        origin=Origin(xyz=(0.0, 0.0, 0.00045)),
        material=keypad_black,
        name="keypad_base",
    )
    keypad.visual(
        Box((0.030, 0.018, 0.0007)),
        origin=Origin(xyz=(0.0, 0.024, 0.00125)),
        material=keypad_gray,
        name="nav_pad",
    )
    keypad.visual(
        Box((0.010, 0.008, 0.0007)),
        origin=Origin(xyz=(0.0, 0.024, 0.00125)),
        material=keypad_black,
        name="nav_select",
    )
    keypad.visual(
        Box((0.012, 0.007, 0.0007)),
        origin=Origin(xyz=(-0.017, 0.024, 0.00125)),
        material=keypad_gray,
        name="call_key",
    )
    keypad.visual(
        Box((0.012, 0.007, 0.0007)),
        origin=Origin(xyz=(0.017, 0.024, 0.00125)),
        material=keypad_gray,
        name="end_key",
    )
    for row_index, y_pos in enumerate((0.008, -0.006, -0.020, -0.034)):
        for col_index, x_pos in enumerate((-0.018, 0.0, 0.018)):
            keypad.visual(
                Box((0.012, 0.008, 0.0007)),
                origin=Origin(xyz=(x_pos, y_pos, 0.00125)),
                material=keypad_gray,
                name=f"key_{row_index}_{col_index}",
            )
    keypad.inertial = Inertial.from_geometry(
        Box((0.056, 0.076, 0.0018)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
    )

    screen = model.part("screen_module")
    screen.visual(
        Box((0.052, 0.072, 0.0009)),
        origin=Origin(xyz=(0.0, 0.0, -0.00045)),
        material=screen_black,
        name="screen_bezel",
    )
    screen.visual(
        Box((0.041, 0.056, 0.0005)),
        origin=Origin(xyz=(0.0, -0.002, -0.00055)),
        material=glass,
        name="display_glass",
    )
    screen.visual(
        Box((0.014, 0.003, 0.0005)),
        origin=Origin(xyz=(0.0, 0.029, -0.00055)),
        material=keypad_gray,
        name="earpiece_slot",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.052, 0.072, 0.0012)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -0.0006)),
    )

    camera = model.part("camera_module")
    camera.visual(
        Box((0.020, 0.016, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=keypad_black,
        name="camera_island",
    )
    camera.visual(
        Cylinder(radius=0.0052, length=0.0005),
        origin=Origin(xyz=(0.0045, 0.0, 0.00145)),
        material=metal,
        name="lens_ring",
    )
    camera.visual(
        Cylinder(radius=0.0040, length=0.0016),
        origin=Origin(xyz=(0.0045, 0.0, 0.0020)),
        material=lens_glass,
        name="camera_lens",
    )
    camera.visual(
        Cylinder(radius=0.0017, length=0.0008),
        origin=Origin(xyz=(-0.0055, 0.0, 0.0016)),
        material=flash_lens,
        name="flash",
    )
    camera.inertial = Inertial.from_geometry(
        Box((0.020, 0.016, 0.0030)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )

    model.articulation(
        "phone_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=2.55,
        ),
    )
    model.articulation(
        "lower_to_keypad",
        ArticulationType.FIXED,
        parent=lower_body,
        child=keypad,
        origin=Origin(xyz=(0.0, 0.007, lower_t * 0.5)),
    )
    model.articulation(
        "upper_to_screen",
        ArticulationType.FIXED,
        parent=upper_body,
        child=screen,
        origin=Origin(xyz=(0.0, upper_rel_y + 0.003, upper_rel_z - upper_t * 0.5)),
    )
    model.articulation(
        "upper_to_camera",
        ArticulationType.FIXED,
        parent=upper_body,
        child=camera,
        origin=Origin(
            xyz=(0.017, upper_rel_y + 0.021, upper_rel_z + upper_t * 0.5),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    keypad = object_model.get_part("keypad_module")
    screen = object_model.get_part("screen_module")
    camera = object_model.get_part("camera_module")
    hinge = object_model.get_articulation("phone_hinge")

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

    limits = hinge.motion_limits
    ctx.check(
        "hinge axis follows phone width",
        tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {hinge.axis}",
    )
    ctx.check(
        "hinge opens to a realistic clamshell angle",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 2.4,
        details=f"unexpected hinge limits: {limits}",
    )

    ctx.expect_gap(
        upper_body,
        lower_body,
        axis="z",
        min_gap=0.0026,
        max_gap=0.0038,
        positive_elem="upper_shell",
        negative_elem="lower_shell",
        name="closed shell gap",
    )
    ctx.expect_overlap(
        upper_body,
        lower_body,
        axes="xy",
        min_overlap=0.05,
        name="closed halves overlap in plan",
    )
    ctx.expect_contact(keypad, lower_body, name="keypad mounted to lower body")
    ctx.expect_contact(screen, upper_body, name="screen mounted to upper body")
    ctx.expect_contact(camera, upper_body, name="camera mounted to upper rear")
    ctx.expect_contact(upper_body, lower_body, name="hinge physically connects both halves")
    ctx.expect_within(
        camera,
        upper_body,
        axes="xy",
        margin=0.0,
        name="camera footprint stays on upper body",
    )
    ctx.expect_gap(
        screen,
        keypad,
        axis="z",
        min_gap=0.0002,
        max_gap=0.0016,
        name="closed internals clear screen to keypad",
    )

    closed_upper_aabb = ctx.part_world_aabb(upper_body)
    lower_aabb = ctx.part_world_aabb(lower_body)
    assert closed_upper_aabb is not None
    assert lower_aabb is not None
    closed_upper_center_y = (closed_upper_aabb[0][1] + closed_upper_aabb[1][1]) * 0.5
    closed_upper_max_z = closed_upper_aabb[1][2]

    with ctx.pose({hinge: 2.2}):
        open_upper_aabb = ctx.part_world_aabb(upper_body)
        assert open_upper_aabb is not None
        open_upper_center_y = (open_upper_aabb[0][1] + open_upper_aabb[1][1]) * 0.5
        open_upper_max_z = open_upper_aabb[1][2]
        lower_center_y = (lower_aabb[0][1] + lower_aabb[1][1]) * 0.5

        ctx.check(
            "open pose lifts the display half",
            open_upper_max_z > closed_upper_max_z + 0.035,
            details=f"closed max z={closed_upper_max_z:.4f}, open max z={open_upper_max_z:.4f}",
        )
        ctx.check(
            "open pose swings the display half behind the hinge",
            open_upper_center_y < lower_center_y - 0.015 and open_upper_center_y < closed_upper_center_y - 0.015,
            details=(
                f"lower center y={lower_center_y:.4f}, "
                f"closed upper center y={closed_upper_center_y:.4f}, "
                f"open upper center y={open_upper_center_y:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

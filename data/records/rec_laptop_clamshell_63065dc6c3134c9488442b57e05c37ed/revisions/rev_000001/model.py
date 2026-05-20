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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_laptop")

    body_dark = model.material("body_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    body_mid = model.material("body_mid", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    display_black = model.material("display_black", rgba=(0.03, 0.03, 0.04, 1.0))
    glass = model.material("glass", rgba=(0.04, 0.05, 0.07, 1.0))
    key_dark = model.material("key_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    key_accent = model.material("key_accent", rgba=(0.58, 0.08, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.26, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=body_dark,
        name="bottom_pan",
    )
    base.visual(
        Box((0.018, 0.25, 0.017)),
        origin=Origin(xyz=(-0.171, 0.0, 0.0145)),
        material=body_mid,
        name="left_side_rail",
    )
    base.visual(
        Box((0.018, 0.25, 0.017)),
        origin=Origin(xyz=(0.171, 0.0, 0.0145)),
        material=body_mid,
        name="right_side_rail",
    )
    base.visual(
        Box((0.324, 0.095, 0.017)),
        origin=Origin(xyz=(0.0, -0.0825, 0.0145)),
        material=body_mid,
        name="front_palmrest_block",
    )
    base.visual(
        Box((0.36, 0.07, 0.024)),
        origin=Origin(xyz=(0.0, 0.095, 0.018)),
        material=body_mid,
        name="rear_exhaust_deck",
    )
    base.visual(
        Box((0.102, 0.050, 0.0012)),
        origin=Origin(xyz=(0.0, -0.096, 0.0200)),
        material=display_black,
        name="touchpad_plate",
    )
    base.visual(
        Box((0.310, 0.122, 0.0016)),
        origin=Origin(xyz=(0.0, -0.009, 0.0200)),
        material=body_dark,
        name="keyboard_deck",
    )
    for idx in range(8):
        x = -0.1365 + idx * 0.039
        base.visual(
            Box((0.024, 0.004, 0.010)),
            origin=Origin(xyz=(x, 0.126, 0.019)),
            material=hinge_dark,
            name=f"rear_vent_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.03)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    for side, x_pos in (("left", -0.115), ("right", 0.115)):
        hinge = model.part(f"{side}_hinge")
        hinge.visual(
            Box((0.052, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=hinge_dark,
            name="mount_foot",
        )
        hinge.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_dark,
            name="hinge_barrel",
        )
        model.articulation(
            f"base_to_{side}_hinge",
            ArticulationType.FIXED,
            parent=base,
            child=hinge,
            origin=Origin(xyz=(x_pos, 0.091, 0.030)),
        )

    screen = model.part("screen")
    screen.visual(
        Box((0.34, 0.008, 0.23)),
        origin=Origin(xyz=(0.0, -0.004, 0.115)),
        material=body_dark,
        name="screen_back_shell",
    )
    screen.visual(
        Box((0.012, 0.004, 0.205)),
        origin=Origin(xyz=(-0.158, -0.006, 0.111)),
        material=display_black,
        name="bezel_left",
    )
    screen.visual(
        Box((0.012, 0.004, 0.205)),
        origin=Origin(xyz=(0.158, -0.006, 0.111)),
        material=display_black,
        name="bezel_right",
    )
    screen.visual(
        Box((0.316, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.218)),
        material=display_black,
        name="bezel_top",
    )
    screen.visual(
        Box((0.316, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.011)),
        material=display_black,
        name="bezel_bottom",
    )
    screen.visual(
        Box((0.304, 0.001, 0.176)),
        origin=Origin(xyz=(0.0, -0.0075, 0.114)),
        material=glass,
        name="display_panel",
    )
    screen.visual(
        Box((0.032, 0.012, 0.010)),
        origin=Origin(xyz=(-0.115, 0.004, 0.012)),
        material=hinge_dark,
        name="left_hinge_ear",
    )
    screen.visual(
        Box((0.032, 0.012, 0.010)),
        origin=Origin(xyz=(0.115, 0.004, 0.012)),
        material=hinge_dark,
        name="right_hinge_ear",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.34, 0.01, 0.23)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -0.004, 0.115)),
    )
    model.articulation(
        "base_to_screen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=screen,
        origin=Origin(xyz=(0.0, 0.082, 0.040)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.28,
            upper=0.30,
        ),
    )

    key_specs = [
        ("key_q", -0.105, 0.038, 0.032, 0.024, False),
        ("key_w", -0.063, 0.038, 0.032, 0.024, True),
        ("key_e", -0.021, 0.038, 0.032, 0.024, False),
        ("key_r", 0.021, 0.038, 0.032, 0.024, False),
        ("key_t", 0.063, 0.038, 0.032, 0.024, False),
        ("key_y", 0.105, 0.038, 0.032, 0.024, False),
        ("key_a", -0.105, 0.004, 0.032, 0.024, True),
        ("key_s", -0.063, 0.004, 0.032, 0.024, True),
        ("key_d", -0.021, 0.004, 0.032, 0.024, True),
        ("key_f", 0.021, 0.004, 0.032, 0.024, False),
        ("key_g", 0.063, 0.004, 0.032, 0.024, False),
        ("key_h", 0.105, 0.004, 0.032, 0.024, False),
        ("key_z", -0.105, -0.030, 0.032, 0.024, False),
        ("key_x", -0.063, -0.030, 0.032, 0.024, False),
        ("key_c", -0.021, -0.030, 0.032, 0.024, False),
        ("key_v", 0.021, -0.030, 0.032, 0.024, False),
        ("key_b", 0.063, -0.030, 0.032, 0.024, False),
        ("key_n", 0.105, -0.030, 0.032, 0.024, False),
        ("key_space", 0.0, -0.056, 0.132, 0.022, False),
    ]
    for key_name, x_pos, y_pos, width, depth, accented in key_specs:
        key = model.part(key_name)
        key.visual(
            Box((width, depth, 0.0024)),
            origin=Origin(xyz=(0.0, 0.0, 0.0012)),
            material=key_accent if accented else key_dark,
            name="keycap",
        )
        model.articulation(
            f"base_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x_pos, y_pos, 0.0208)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.10,
                lower=0.0,
                upper=0.0012,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base = object_model.get_part("base")
    screen = object_model.get_part("screen")
    left_hinge = object_model.get_part("left_hinge")
    right_hinge = object_model.get_part("right_hinge")
    screen_hinge = object_model.get_articulation("base_to_screen")
    key_w = object_model.get_part("key_w")
    key_a = object_model.get_part("key_a")
    key_s = object_model.get_part("key_s")
    key_d = object_model.get_part("key_d")
    key_space = object_model.get_part("key_space")
    key_w_slide = object_model.get_articulation("base_to_key_w")
    key_space_slide = object_model.get_articulation("base_to_key_space")

    ctx.expect_contact(left_hinge, base, name="left hinge is mounted to the rear deck")
    ctx.expect_contact(right_hinge, base, name="right hinge is mounted to the rear deck")
    ctx.expect_contact(screen, left_hinge, name="screen shell bears on the left hinge")
    ctx.expect_contact(screen, right_hinge, name="screen shell bears on the right hinge")
    ctx.expect_overlap(
        screen,
        base,
        axes="x",
        min_overlap=0.28,
        name="screen spans nearly the full base width",
    )
    ctx.check(
        "screen hinge uses rear crosswise revolute axis",
        tuple(round(v, 3) for v in screen_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={screen_hinge.axis}",
    )

    rest_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({screen_hinge: -1.10}):
        closed_aabb = ctx.part_world_aabb(screen)
    ctx.check(
        "screen rotates forward toward the keyboard when closing",
        rest_aabb is not None
        and closed_aabb is not None
        and closed_aabb[0][1] < rest_aabb[0][1] - 0.10
        and closed_aabb[1][2] < rest_aabb[1][2] - 0.05,
        details=f"rest_aabb={rest_aabb}, closed_aabb={closed_aabb}",
    )
    ctx.expect_contact(key_w, base, name="W key rests on the keyboard deck")
    ctx.expect_contact(key_a, base, name="A key rests on the keyboard deck")
    ctx.expect_contact(key_s, base, name="S key rests on the keyboard deck")
    ctx.expect_contact(key_d, base, name="D key rests on the keyboard deck")
    ctx.expect_contact(key_space, base, name="space bar rests on the keyboard deck")
    ctx.check(
        "keyboard plungers move vertically downward",
        tuple(round(v, 3) for v in key_w_slide.axis) == (0.0, 0.0, -1.0)
        and tuple(round(v, 3) for v in key_space_slide.axis) == (0.0, 0.0, -1.0),
        details=f"key_w_axis={key_w_slide.axis}, key_space_axis={key_space_slide.axis}",
    )

    key_w_rest = ctx.part_world_aabb(key_w)
    with ctx.pose({key_w_slide: 0.0012, key_space_slide: 0.0012}):
        key_w_pressed = ctx.part_world_aabb(key_w)
        key_space_pressed = ctx.part_world_aabb(key_space)
    key_space_rest = ctx.part_world_aabb(key_space)
    ctx.check(
        "keys plunge by a short travel",
        key_w_rest is not None
        and key_w_pressed is not None
        and key_space_rest is not None
        and key_space_pressed is not None
        and key_w_pressed[1][2] < key_w_rest[1][2] - 0.0010
        and key_space_pressed[1][2] < key_space_rest[1][2] - 0.0010,
        details=(
            f"key_w_rest={key_w_rest}, key_w_pressed={key_w_pressed}, "
            f"key_space_rest={key_space_rest}, key_space_pressed={key_space_pressed}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

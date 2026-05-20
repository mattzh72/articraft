from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_thin_laptop")

    # Realistic compact laptop proportions in meters.
    base_w = 0.320
    base_d = 0.220
    base_t = 0.012
    rear_y = base_d / 2.0
    hinge_y = 0.106
    hinge_z = 0.020
    hinge_radius = 0.004

    lid_w = 0.306
    lid_d = 0.205
    lid_t = 0.008
    lid_hinge_gap = 0.006
    current_open = math.radians(112.0)
    max_open = math.radians(135.0)

    shell_mat = Material("anodized_space_gray", rgba=(0.42, 0.43, 0.44, 1.0))
    dark_mat = Material("matte_dark_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    key_mat = Material("black_keycaps", rgba=(0.025, 0.025, 0.028, 1.0))
    screen_mat = Material("glossy_black_screen", rgba=(0.0, 0.0, 0.0, 1.0))
    pad_mat = Material("slightly_darker_trackpad", rgba=(0.30, 0.31, 0.32, 1.0))
    hinge_mat = Material("dark_hinge_metal", rgba=(0.06, 0.06, 0.065, 1.0))

    def rx(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
        x, y, z = point
        ca = math.cos(angle)
        sa = math.sin(angle)
        return (x, y * ca - z * sa, y * sa + z * ca)

    def lid_pose(local_center: tuple[float, float, float]) -> Origin:
        """Place a closed-laptop local feature in the default open display pose."""
        return Origin(xyz=rx(local_center, -current_open), rpy=(-current_open, 0.0, 0.0))

    base = model.part("base")
    base_profile = rounded_rect_profile(base_w, base_d, 0.018, corner_segments=10)
    base.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(base_profile, base_t), "rounded_base_shell"),
        material=shell_mat,
        name="base_shell",
    )

    # Slightly recessed dark keyboard and touchpad beds are mounted into the metal deck.
    key_bed_h = 0.0006
    key_bed_top = base_t + 0.0005
    base.visual(
        Box((0.250, 0.083, key_bed_h)),
        origin=Origin(xyz=(0.0, 0.026, key_bed_top - key_bed_h / 2.0)),
        material=dark_mat,
        name="keyboard_well",
    )
    pad_bed_h = 0.0005
    pad_bed_top = base_t + 0.00045
    base.visual(
        Box((0.105, 0.064, pad_bed_h)),
        origin=Origin(xyz=(0.0, -0.058, pad_bed_top - pad_bed_h / 2.0)),
        material=shell_mat,
        name="touchpad_recess",
    )
    # Low speaker grilles flanking the keyboard, implemented as seated dark strips.
    for idx, x in enumerate((-0.139, 0.139)):
        base.visual(
            Box((0.020, 0.076, 0.0005)),
            origin=Origin(xyz=(x, 0.026, base_t + 0.00020)),
            material=dark_mat,
            name=f"speaker_grille_{idx}",
        )

    # Two compact hinge assemblies: each has two fixed outer barrels on the base.
    hinge_centers = (-0.104, 0.104)
    for hinge_i, cx in enumerate(hinge_centers):
        for seg_i, sx in enumerate((-0.015, 0.015)):
            base.visual(
                Box((0.013, 0.010, 0.0052)),
                origin=Origin(xyz=(cx + sx, hinge_y, base_t + 0.0026)),
                material=hinge_mat,
                name=f"hinge_block_{hinge_i}_{seg_i}",
            )
            base.visual(
                Cylinder(radius=hinge_radius, length=0.011),
                origin=Origin(xyz=(cx + sx, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_mat,
                name=f"base_barrel_{hinge_i}_{seg_i}",
            )

    # Individual keycaps are articulated small prismatic controls.  The deck is
    # static, but each key can travel down a short realistic amount.
    key_h = 0.0022
    key_bottom_z = key_bed_top
    key_rows = [
        (0.061, 12, 0.014, 0.011, 0.0175, 0.0),
        (0.043, 12, 0.015, 0.012, 0.0180, 0.0),
        (0.025, 11, 0.015, 0.012, 0.0180, 0.009),
        (0.007, 10, 0.016, 0.012, 0.0185, 0.0),
    ]
    for row_i, (y, count, kw, kd, pitch, x_shift) in enumerate(key_rows):
        start_x = -pitch * (count - 1) / 2.0 + x_shift
        for col_i in range(count):
            x = start_x + col_i * pitch
            key = model.part(f"key_{row_i}_{col_i}")
            key.visual(
                Box((kw, kd, key_h)),
                origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
                material=key_mat,
                name="keycap",
            )
            model.articulation(
                f"base_to_key_{row_i}_{col_i}",
                ArticulationType.PRISMATIC,
                parent=base,
                child=key,
                origin=Origin(xyz=(x, y, key_bottom_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=0.7, velocity=0.08, lower=0.0, upper=0.0018),
            )

    # Bottom row with realistic wide space bar and shorter modifier keys.
    bottom_y = -0.012
    bottom_keys = [
        (-0.101, 0.018),
        (-0.078, 0.018),
        (-0.052, 0.022),
        (0.000, 0.058),
        (0.052, 0.022),
        (0.078, 0.018),
        (0.101, 0.018),
    ]
    for col_i, (x, kw) in enumerate(bottom_keys):
        key = model.part(f"key_4_{col_i}")
        key.visual(
            Box((kw, 0.012, key_h)),
            origin=Origin(xyz=(0.0, 0.0, key_h / 2.0)),
            material=key_mat,
            name="keycap",
        )
        model.articulation(
            f"base_to_key_4_{col_i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x, bottom_y, key_bottom_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.9, velocity=0.08, lower=0.0, upper=0.0018),
        )

    touchpad = model.part("touchpad")
    touchpad_h = 0.0012
    touchpad.visual(
        Box((0.096, 0.056, touchpad_h)),
        origin=Origin(xyz=(0.0, 0.0, touchpad_h / 2.0)),
        material=pad_mat,
        name="touch_surface",
    )
    model.articulation(
        "base_to_touchpad",
        ArticulationType.PRISMATIC,
        parent=base,
        child=touchpad,
        origin=Origin(xyz=(0.0, -0.058, pad_bed_top)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0008),
    )

    lid = model.part("display_lid")
    lid_profile = rounded_rect_profile(lid_w, lid_d, 0.014, corner_segments=10)
    lid_shell_center = (0.0, -lid_hinge_gap - lid_d / 2.0, 0.0)
    lid.visual(
        mesh_from_geometry(ExtrudeGeometry.centered(lid_profile, lid_t), "rounded_display_lid"),
        origin=lid_pose(lid_shell_center),
        material=shell_mat,
        name="lid_shell",
    )
    screen_t = 0.0007
    screen_center = (0.0, -0.107, -lid_t / 2.0 - screen_t / 2.0 + 0.0001)
    lid.visual(
        Box((0.266, 0.154, screen_t)),
        origin=lid_pose(screen_center),
        material=screen_mat,
        name="screen_glass",
    )
    lid.visual(
        Box((0.008, 0.004, screen_t)),
        origin=lid_pose((0.0, -0.021, -lid_t / 2.0 - screen_t / 2.0 + 0.00008)),
        material=screen_mat,
        name="webcam_window",
    )

    # The moving center knuckle on each compact hinge is carried by the display lid.
    for hinge_i, cx in enumerate(hinge_centers):
        lid.visual(
            Cylinder(radius=hinge_radius, length=0.017),
            origin=Origin(xyz=(cx, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"lid_barrel_{hinge_i}",
        )
        lid.visual(
            Box((0.017, lid_hinge_gap + 0.002, 0.004)),
            origin=lid_pose((cx, -lid_hinge_gap / 2.0, 0.0)),
            material=hinge_mat,
            name=f"lid_hinge_leaf_{hinge_i}",
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed lid extends along local -Y, so rotation about -X opens it.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-current_open,
            upper=max_open - current_open,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("display_lid")
    touchpad = object_model.get_part("touchpad")
    sample_key = object_model.get_part("key_2_5")
    lid_joint = object_model.get_articulation("base_to_lid")
    key_joint = object_model.get_articulation("base_to_key_2_5")
    pad_joint = object_model.get_articulation("base_to_touchpad")

    ctx.expect_contact(
        sample_key,
        base,
        elem_a="keycap",
        elem_b="keyboard_well",
        contact_tol=0.00005,
        name="sample keycap rests on keyboard deck",
    )
    ctx.expect_contact(
        touchpad,
        base,
        elem_a="touch_surface",
        elem_b="touchpad_recess",
        contact_tol=0.00005,
        name="touchpad is seated in centered recess",
    )
    ctx.expect_within(
        touchpad,
        base,
        axes="xy",
        inner_elem="touch_surface",
        outer_elem="touchpad_recess",
        margin=0.006,
        name="touchpad stays centered in its deck recess",
    )

    lower = lid_joint.motion_limits.lower
    upper = lid_joint.motion_limits.upper
    ctx.check(
        "lid hinge spans closed to one hundred thirty five degrees",
        lower is not None
        and upper is not None
        and abs((upper - lower) - math.radians(135.0)) < math.radians(0.75),
        details=f"lower={lower}, upper={upper}",
    )

    with ctx.pose({lid_joint: lower}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="base_shell",
            min_overlap=0.19,
            name="closed lid covers the base footprint",
        )
        ctx.expect_gap(
            lid,
            sample_key,
            axis="z",
            positive_elem="screen_glass",
            negative_elem="keycap",
            min_gap=0.0002,
            name="closed screen clears raised keycaps",
        )

    with ctx.pose({lid_joint: upper}):
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "upper stop holds display high behind rear edge",
            lid_aabb is not None and lid_aabb[1][2] > 0.155 and lid_aabb[0][1] > -0.020,
            details=f"lid_shell_aabb={lid_aabb}",
        )

    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({key_joint: 0.0018, pad_joint: 0.0008}):
        key_down = ctx.part_world_position(sample_key)
        pad_down = ctx.part_world_position(touchpad)
    pad_rest = ctx.part_world_position(touchpad)
    ctx.check(
        "sample key and touchpad travel downward",
        key_rest is not None
        and key_down is not None
        and pad_rest is not None
        and pad_down is not None
        and key_down[2] < key_rest[2] - 0.001
        and pad_down[2] < pad_rest[2] - 0.0004,
        details=f"key_rest={key_rest}, key_down={key_down}, pad_rest={pad_rest}, pad_down={pad_down}",
    )

    return ctx.report()


object_model = build_object_model()

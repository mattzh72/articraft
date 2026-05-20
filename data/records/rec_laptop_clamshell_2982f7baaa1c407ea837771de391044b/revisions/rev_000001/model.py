from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_laptop")

    aluminum = model.material("satin_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.11, 0.12, 0.13, 1.0))
    key_black = model.material("soft_black_keys", rgba=(0.015, 0.016, 0.018, 1.0))
    key_side = model.material("key_side_shadow", rgba=(0.04, 0.042, 0.045, 1.0))
    glass = model.material("dark_glass", rgba=(0.015, 0.025, 0.040, 1.0))
    trackpad_mat = model.material("matte_trackpad", rgba=(0.45, 0.47, 0.49, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    base = model.part("base")
    # The base frame is centered on the rear 360-degree hinge axis.  The
    # keyboard deck sits 6 mm below that axis, like a thin convertible laptop.
    base.visual(
        Box((0.215, 0.315, 0.014)),
        origin=Origin(xyz=(-0.1075, 0.0, -0.013)),
        material=aluminum,
        name="base_shell",
    )
    base.visual(
        Box((0.105, 0.260, 0.0008)),
        origin=Origin(xyz=(-0.097, 0.0, -0.0056)),
        material=dark_aluminum,
        name="keyboard_recess",
    )
    base.visual(
        Box((0.056, 0.096, 0.0007)),
        origin=Origin(xyz=(-0.174, 0.0, -0.00555)),
        material=trackpad_mat,
        name="trackpad",
    )
    # Rubber feet and a short rear vent line are low, visible product details
    # that remain fused to the base shell rather than floating decorations.
    for i, y in enumerate((-0.125, 0.125)):
        base.visual(
            Box((0.032, 0.010, 0.0012)),
            origin=Origin(xyz=(-0.190, y, -0.0202)),
            material=rubber,
            name=f"front_foot_{i}",
        )
        base.visual(
            Box((0.030, 0.010, 0.0012)),
            origin=Origin(xyz=(-0.030, y, -0.0202)),
            material=rubber,
            name=f"rear_foot_{i}",
        )
    for i, y in enumerate((-0.070, -0.046, -0.022, 0.022, 0.046, 0.070)):
        base.visual(
            Box((0.004, 0.014, 0.0008)),
            origin=Origin(xyz=(-0.024, y, -0.0056)),
            material=rubber,
            name=f"rear_vent_{i}",
        )

    hinge_radius = 0.0042
    hinge_centers = (-0.095, 0.095)
    for h, y0 in enumerate(hinge_centers):
        for segment, offset in enumerate((-0.017, 0.017)):
            y = y0 + offset
            base.visual(
                Cylinder(radius=hinge_radius, length=0.014),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_aluminum,
                name=f"hinge_barrel_{h}_{segment}",
            )
            base.visual(
                Box((0.014, 0.014, 0.007)),
                origin=Origin(xyz=(-0.004, y, -0.0042)),
                material=dark_aluminum,
                name=f"hinge_saddle_{h}_{segment}",
            )

    display = model.part("display")
    display.visual(
        Box((0.008, 0.305, 0.199)),
        # At q=0 the laptop is open, so the display shell stands upright on the
        # rear hinge.  Negative rotation closes over the keyboard; positive
        # rotation folds it behind the deck.
        origin=Origin(xyz=(0.004, 0.0, 0.1055)),
        material=aluminum,
        name="display_shell",
    )
    display.visual(
        Box((0.0005, 0.236, 0.152)),
        origin=Origin(xyz=(-0.00025, 0.0, 0.106)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.0006, 0.270, 0.020)),
        origin=Origin(xyz=(-0.0003, 0.0, 0.190)),
        material=dark_aluminum,
        name="top_bezel",
    )
    display.visual(
        Box((0.0006, 0.270, 0.024)),
        origin=Origin(xyz=(-0.0003, 0.0, 0.018)),
        material=dark_aluminum,
        name="bottom_bezel",
    )
    for side, y in enumerate((-0.132, 0.132)):
        display.visual(
            Box((0.0006, 0.012, 0.156)),
            origin=Origin(xyz=(-0.0003, y, 0.104)),
            material=dark_aluminum,
            name=f"side_bezel_{side}",
        )
    display.visual(
        Cylinder(radius=0.0032, length=0.0008),
        origin=Origin(xyz=(-0.0004, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="webcam_lens",
    )
    for h, y0 in enumerate(hinge_centers):
        display.visual(
            Cylinder(radius=hinge_radius, length=0.020),
            origin=Origin(xyz=(0.0, y0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name=f"display_knuckle_{h}",
        )
        display.visual(
            Box((0.008, 0.018, 0.012)),
            origin=Origin(xyz=(0.002, y0, 0.006)),
            material=dark_aluminum,
            name=f"display_hinge_leaf_{h}",
        )

    display_hinge = model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=3.0 * math.pi / 2.0,
        ),
    )
    display_hinge.meta["description"] = (
        "A 360-degree rear hinge: -pi/2 is closed over the keyboard, 0 is "
        "laptop mode, and pi/2 folds the display behind the keyboard deck."
    )

    def add_key(row: int, col: int, x: float, y: float, width_y: float, depth_x: float = 0.014) -> None:
        key = model.part(f"key_{row}_{col}")
        key.visual(
            Box((depth_x * 0.62, width_y * 0.58, 0.0018)),
            origin=Origin(xyz=(0.0, 0.0, 0.0009)),
            material=key_side,
            name="plunger_stem",
        )
        key.visual(
            Box((depth_x, width_y, 0.0028)),
            origin=Origin(xyz=(0.0, 0.0, 0.0027)),
            material=key_black,
            name="key_cap",
        )
        model.articulation(
            f"base_to_key_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x, y, -0.0052)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.9,
                velocity=0.45,
                lower=0.0,
                upper=0.0032,
            ),
        )

    # A compact chiclet keyboard.  Every cap is its own short vertical prismatic
    # plunger, including the long space bar.
    key_pitch = 0.020
    row_specs = (
        (0, -0.062, 12, 0.000, 0.016),
        (1, -0.083, 11, 0.010, 0.016),
        (2, -0.104, 10, 0.020, 0.016),
    )
    for row, x, count, offset, key_width in row_specs:
        start = -0.5 * (count - 1) * key_pitch + offset
        for col in range(count):
            add_key(row, col, x, start + col * key_pitch, key_width)

    bottom_keys = (
        (-0.074, 0.016),
        (-0.052, 0.016),
        (-0.016, 0.052),
        (0.032, 0.016),
        (0.056, 0.016),
        (0.080, 0.016),
    )
    for col, (y, width) in enumerate(bottom_keys):
        add_key(3, col, -0.127, y, width)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    display = object_model.get_part("display")
    sample_key = object_model.get_part("key_1_5")
    display_hinge = object_model.get_articulation("base_to_display")
    sample_key_joint = object_model.get_articulation("base_to_key_1_5")

    ctx.check(
        "display hinge has full 360 degree travel",
        display_hinge.motion_limits is not None
        and display_hinge.motion_limits.lower is not None
        and display_hinge.motion_limits.upper is not None
        and (
            display_hinge.motion_limits.upper - display_hinge.motion_limits.lower
        )
        >= 2.0 * math.pi - 1.0e-6,
        details=f"limits={display_hinge.motion_limits}",
    )

    open_aabb = ctx.part_world_aabb(display)
    ok = open_aabb is not None and (open_aabb[1][2] - open_aabb[0][2]) > 0.18
    ctx.check(
        "display stands upright at laptop angle",
        ok,
        details=f"open_aabb={open_aabb}",
    )

    with ctx.pose({display_hinge: math.pi / 2.0}):
        folded_aabb = ctx.part_world_aabb(display)
        ok = folded_aabb is not None and folded_aabb[1][0] > 0.18
        ctx.check(
            "display folds behind keyboard deck",
            ok,
            details=f"folded_aabb={folded_aabb}",
        )

    rest_aabb = ctx.part_world_aabb(sample_key)
    with ctx.pose({sample_key_joint: 0.0032}):
        pressed_aabb = ctx.part_world_aabb(sample_key)
        ok = (
            rest_aabb is not None
            and pressed_aabb is not None
            and pressed_aabb[1][2] < rest_aabb[1][2] - 0.0025
        )
        ctx.check(
            "keyboard key plunges downward",
            ok,
            details=f"rest_aabb={rest_aabb}, pressed_aabb={pressed_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

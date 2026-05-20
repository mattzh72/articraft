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


def _cyl_y_origin(x: float, y: float, z: float) -> Origin:
    """URDF cylinders are local-Z; rotate them so their length runs along Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_laptop_service_door")

    gunmetal = model.material("dark_gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    deck_gray = model.material("keyboard_deck_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    key_black = model.material("keycap_black", rgba=(0.025, 0.026, 0.028, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.01, 0.015, 0.025, 1.0))
    blue_screen = model.material("soft_blue_display", rgba=(0.06, 0.12, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.380, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=gunmetal,
        name="lower_body",
    )
    base.visual(
        Box((0.162, 0.310, 0.002)),
        origin=Origin(xyz=(-0.015, 0.0, 0.027)),
        material=deck_gray,
        name="keyboard_deck",
    )
    base.visual(
        Box((0.050, 0.145, 0.002)),
        origin=Origin(xyz=(0.088, 0.0, 0.027)),
        material=black,
        name="trackpad",
    )
    # Rear display hinge: two fixed barrels on the base leave a central gap for
    # the rotating display barrel.
    for idx, y in enumerate((-0.123, 0.123)):
        base.visual(
            Cylinder(radius=0.007, length=0.086),
            origin=_cyl_y_origin(-0.128, y, 0.032),
            material=gunmetal,
            name=f"display_hinge_socket_{idx}",
        )
    # Low rubber feet make the underside read as a heavy workstation base.
    for idx, (x, y) in enumerate(
        ((-0.095, -0.148), (-0.095, 0.148), (0.105, -0.148), (0.105, 0.148))
    ):
        base.visual(
            Box((0.036, 0.052, 0.004)),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=f"foot_{idx}",
        )

    # Raised underside frame and hinge lugs around the service bay opening.
    base.visual(
        Box((0.096, 0.006, 0.002)),
        origin=Origin(xyz=(0.073, -0.060, -0.001)),
        material=black,
        name="service_frame_0",
    )
    base.visual(
        Box((0.096, 0.006, 0.002)),
        origin=Origin(xyz=(0.073, 0.060, -0.001)),
        material=black,
        name="service_frame_1",
    )
    base.visual(
        Box((0.006, 0.126, 0.002)),
        origin=Origin(xyz=(0.026, 0.0, -0.001)),
        material=black,
        name="service_frame_2",
    )
    base.visual(
        Box((0.006, 0.126, 0.002)),
        origin=Origin(xyz=(0.120, 0.0, -0.001)),
        material=black,
        name="service_frame_3",
    )
    for idx, y in enumerate((-0.064, 0.064)):
        base.visual(
            Box((0.018, 0.014, 0.014)),
            origin=Origin(xyz=(0.026, y, -0.007)),
            material=gunmetal,
            name=f"service_hinge_lug_{idx}",
        )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.006, length=0.154),
        origin=_cyl_y_origin(0.0, 0.0, 0.0),
        material=gunmetal,
        name="display_hinge_barrel",
    )
    display.visual(
        Box((0.012, 0.380, 0.250)),
        origin=Origin(xyz=(-0.006, 0.0, 0.130)),
        material=gunmetal,
        name="lid_shell",
    )
    display.visual(
        Box((0.0015, 0.326, 0.196)),
        origin=Origin(xyz=(-0.0002, 0.0, 0.144)),
        material=screen_glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.0018, 0.290, 0.158)),
        origin=Origin(xyz=(0.0008, 0.0, 0.138)),
        material=blue_screen,
        name="active_display",
    )
    display.visual(
        Box((0.0018, 0.012, 0.004)),
        origin=Origin(xyz=(0.0008, 0.0, 0.239)),
        material=black,
        name="webcam",
    )
    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(xyz=(-0.128, 0.0, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.20, upper=1.55),
    )

    service_door = model.part("service_door")
    door_angle = 0.55
    door_len = 0.086
    service_door.visual(
        Cylinder(radius=0.004, length=0.104),
        origin=_cyl_y_origin(0.0, 0.0, 0.0),
        material=gunmetal,
        name="door_hinge_barrel",
    )
    service_door.visual(
        Box((door_len, 0.112, 0.004)),
        origin=Origin(
            xyz=(0.5 * door_len * math.cos(door_angle), 0.0, -0.5 * door_len * math.sin(door_angle)),
            rpy=(0.0, door_angle, 0.0),
        ),
        material=gunmetal,
        name="door_panel",
    )
    service_door.visual(
        Box((0.032, 0.030, 0.002)),
        origin=Origin(
            xyz=(0.070 * math.cos(door_angle), 0.0, -0.070 * math.sin(door_angle) - 0.001),
            rpy=(0.0, door_angle, 0.0),
        ),
        material=black,
        name="pull_recess",
    )
    model.articulation(
        "base_to_service_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=service_door,
        origin=Origin(xyz=(0.026, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.4, lower=-0.55, upper=0.85),
    )

    key_pitch_y = 0.027
    key_pitch_x = 0.027
    key_size = (0.020, 0.022, 0.004)
    key_rows = (-0.056, -0.029, -0.002, 0.025)
    for row, x in enumerate(key_rows):
        for col in range(10):
            y = (col - 4.5) * key_pitch_y
            key = model.part(f"key_{row}_{col}")
            key.visual(
                Box(key_size),
                origin=Origin(xyz=(0.0, 0.0, 0.005)),
                material=key_black,
                name="keycap",
            )
            key.visual(
                Box((0.008, 0.008, 0.003)),
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
                material=black,
                name="key_stem",
            )
            model.articulation(
                f"base_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=base,
                child=key,
                origin=Origin(xyz=(x, y, 0.028)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=0.35, velocity=0.10, lower=0.0, upper=0.003),
            )

    space_key = model.part("space_key")
    space_key.visual(
        Box((0.020, 0.138, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=key_black,
        name="space_keycap",
    )
    space_key.visual(
        Box((0.008, 0.040, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=black,
        name="space_stem",
    )
    model.articulation(
        "base_to_space_key",
        ArticulationType.PRISMATIC,
        parent=base,
        child=space_key,
        origin=Origin(xyz=(0.052, 0.0, 0.028)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.50, velocity=0.10, lower=0.0, upper=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    display = object_model.get_part("display")
    service_door = object_model.get_part("service_door")
    sample_key = object_model.get_part("key_0_0")
    lid_hinge = object_model.get_articulation("base_to_display")
    door_hinge = object_model.get_articulation("base_to_service_door")
    key_plunger = object_model.get_articulation("base_to_key_0_0")

    ctx.expect_overlap(
        display,
        base,
        axes="xz",
        elem_a="display_hinge_barrel",
        elem_b="display_hinge_socket_0",
        min_overlap=0.006,
        name="display hinge barrel aligns with rear hinge sockets",
    )
    ctx.expect_gap(
        display,
        base,
        axis="y",
        positive_elem="display_hinge_barrel",
        negative_elem="display_hinge_socket_0",
        min_gap=0.001,
        max_gap=0.006,
        name="display hinge has clearance beside socket",
    )
    ctx.expect_gap(
        sample_key,
        base,
        axis="z",
        positive_elem="key_stem",
        negative_elem="keyboard_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="key stem is seated on the keyboard deck",
    )
    ctx.expect_gap(
        base,
        service_door,
        axis="z",
        positive_elem="lower_body",
        negative_elem="door_panel",
        min_gap=0.001,
        name="pop-open service door sits below the underside",
    )

    def aabb_center_z(part_name: str) -> float | None:
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def aabb_center_x(part_name: str) -> float | None:
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    display_rest_z = aabb_center_z("display")
    display_rest_x = aabb_center_x("display")
    with ctx.pose({lid_hinge: 1.30}):
        display_folded_z = aabb_center_z("display")
        display_folded_x = aabb_center_x("display")
    ctx.check(
        "display lid folds forward and downward about rear hinge",
        display_rest_z is not None
        and display_folded_z is not None
        and display_rest_x is not None
        and display_folded_x is not None
        and display_folded_z < display_rest_z - 0.03
        and display_folded_x > display_rest_x + 0.06,
        details=f"rest=({display_rest_x}, {display_rest_z}), folded=({display_folded_x}, {display_folded_z})",
    )

    door_rest_z = aabb_center_z("service_door")
    with ctx.pose({door_hinge: -0.55}):
        door_closed_z = aabb_center_z("service_door")
    ctx.check(
        "service door swings up toward the underside",
        door_rest_z is not None
        and door_closed_z is not None
        and door_closed_z > door_rest_z + 0.010,
        details=f"rest_z={door_rest_z}, closed_z={door_closed_z}",
    )

    key_rest = ctx.part_world_aabb(sample_key)
    with ctx.pose({key_plunger: 0.003}):
        key_pressed = ctx.part_world_aabb(sample_key)
    rest_min_z = key_rest[0][2] if key_rest is not None else None
    pressed_min_z = key_pressed[0][2] if key_pressed is not None else None
    ctx.check(
        "sample key plunges downward",
        rest_min_z is not None and pressed_min_z is not None and pressed_min_z < rest_min_z - 0.002,
        details=f"rest_min_z={rest_min_z}, pressed_min_z={pressed_min_z}",
    )

    return ctx.report()


object_model = build_object_model()

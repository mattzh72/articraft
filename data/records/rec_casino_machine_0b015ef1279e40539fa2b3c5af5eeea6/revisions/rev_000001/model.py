from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _tilted_origin(
    center: tuple[float, float, float],
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    *,
    pitch: float,
) -> Origin:
    """Place a visual in the shared tilted screen plane."""
    ox, oy, oz = local_offset
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    return Origin(
        xyz=(
            center[0] + cp * ox + sp * oz,
            center[1] + oy,
            center[2] - sp * ox + cp * oz,
        ),
        rpy=(0.0, pitch, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_video_poker_machine")

    model.material("cabinet_black", color=(0.015, 0.017, 0.020, 1.0))
    model.material("charcoal_plastic", color=(0.055, 0.060, 0.068, 1.0))
    model.material("blue_glass", color=(0.02, 0.11, 0.22, 0.82))
    model.material("screen_bezel", color=(0.0, 0.0, 0.0, 1.0))
    model.material("poker_card_white", color=(0.94, 0.93, 0.86, 1.0))
    model.material("red_trim", color=(0.55, 0.02, 0.03, 1.0))
    model.material("brushed_metal", color=(0.58, 0.58, 0.55, 1.0))
    model.material("button_red", color=(0.85, 0.02, 0.02, 1.0))
    model.material("button_white", color=(0.92, 0.90, 0.82, 1.0))
    model.material("button_gold", color=(0.98, 0.68, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.55, 0.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="cabinet_black",
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.46, 0.64, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material="charcoal_plastic",
        name="lower_body",
    )
    cabinet.visual(
        Box((0.04, 0.64, 0.22)),
        origin=Origin(xyz=(-0.21, 0.0, 0.82)),
        material="charcoal_plastic",
        name="shelf_mount_face",
    )
    cabinet.visual(
        Box((0.44, 0.64, 0.76)),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material="charcoal_plastic",
        name="upper_body",
    )
    cabinet.visual(
        Box((0.38, 0.62, 0.16)),
        origin=Origin(xyz=(0.02, 0.0, 1.61)),
        material="cabinet_black",
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.020, 0.58, 0.060)),
        origin=Origin(xyz=(-0.235, 0.0, 0.925)),
        material="red_trim",
        name="screen_shelf_reveal",
    )

    screen_center = (-0.255, 0.0, 1.18)
    screen_pitch = 0.24
    cabinet.visual(
        Box((0.065, 0.59, 0.48)),
        origin=_tilted_origin(screen_center, pitch=screen_pitch),
        material="cabinet_black",
        name="screen_housing",
    )
    cabinet.visual(
        Box((0.014, 0.46, 0.32)),
        origin=_tilted_origin(screen_center, (-0.010, 0.0, 0.0), pitch=screen_pitch),
        material=Material("lit_blue_glass", rgba=(0.01, 0.18, 0.36, 0.74)),
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.026, 0.55, 0.050)),
        origin=_tilted_origin(screen_center, (-0.020, 0.0, 0.215), pitch=screen_pitch),
        material="screen_bezel",
        name="screen_top_bezel",
    )
    cabinet.visual(
        Box((0.026, 0.55, 0.050)),
        origin=_tilted_origin(screen_center, (-0.020, 0.0, -0.215), pitch=screen_pitch),
        material="screen_bezel",
        name="screen_bottom_bezel",
    )
    for side, y in (("side_0", -0.275), ("side_1", 0.275)):
        cabinet.visual(
            Box((0.026, 0.050, 0.43)),
            origin=_tilted_origin(screen_center, (-0.020, y, 0.0), pitch=screen_pitch),
            material="screen_bezel",
            name=f"screen_{side}_bezel",
        )
    for idx, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        cabinet.visual(
            Box((0.004, 0.052, 0.074)),
            origin=_tilted_origin(screen_center, (-0.019, y, 0.020), pitch=screen_pitch),
            material="poker_card_white",
            name=f"card_{idx}",
        )

    control_shelf = model.part("control_shelf")
    control_shelf.visual(
        Box((0.23, 0.60, 0.055)),
        origin=Origin(),
        material="cabinet_black",
        name="shelf_deck",
    )
    control_shelf.visual(
        Box((0.055, 0.60, 0.085)),
        origin=Origin(xyz=(-0.090, 0.0, -0.030)),
        material="red_trim",
        name="front_lip",
    )
    control_shelf.visual(
        Box((0.018, 0.60, 0.070)),
        origin=Origin(xyz=(0.106, 0.0, -0.010)),
        material="charcoal_plastic",
        name="rear_mount_flange",
    )
    model.articulation(
        "cabinet_to_shelf",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_shelf,
        origin=Origin(xyz=(-0.345, 0.0, 0.765)),
    )

    button_specs = [
        ("button_0", -0.20, "button_red"),
        ("button_1", -0.10, "button_white"),
        ("button_2", 0.00, "button_gold"),
        ("button_3", 0.10, "button_white"),
        ("button_4", 0.20, "button_red"),
    ]
    for button_name, y, material in button_specs:
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=0.032, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"shelf_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_shelf,
            child=button,
            origin=Origin(xyz=(-0.020, y, 0.0275)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.010),
        )

    cash_panel = model.part("cash_panel")
    cash_panel.visual(
        Box((0.024, 0.42, 0.30)),
        origin=Origin(xyz=(0.004, 0.210, 0.0)),
        material="brushed_metal",
        name="door_plate",
    )
    cash_panel.visual(
        Cylinder(radius=0.016, length=0.32),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_barrel",
    )
    cash_panel.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(-0.0125, 0.300, 0.030), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="cabinet_black",
        name="round_lock",
    )
    cash_panel.visual(
        Box((0.006, 0.160, 0.018)),
        origin=Origin(xyz=(-0.0105, 0.220, -0.060)),
        material="cabinet_black",
        name="cash_slot",
    )
    cash_hinge = model.articulation(
        "cabinet_to_cash_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cash_panel,
        origin=Origin(xyz=(-0.250, -0.220, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    handle_bracket = model.part("handle_bracket")
    handle_bracket.visual(
        Box((0.16, 0.024, 0.22)),
        origin=Origin(),
        material="brushed_metal",
        name="bracket_plate",
    )
    handle_bracket.visual(
        Cylinder(radius=0.045, length=0.052),
        origin=Origin(xyz=(0.0, 0.038, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="pivot_boss",
    )
    for idx, (x, z) in enumerate(((-0.050, -0.075), (0.050, -0.075), (-0.050, 0.075), (0.050, 0.075))):
        handle_bracket.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(x, 0.018, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="cabinet_black",
            name=f"bolt_{idx}",
        )
    model.articulation(
        "cabinet_to_bracket",
        ArticulationType.FIXED,
        parent=cabinet,
        child=handle_bracket,
        origin=Origin(xyz=(-0.030, 0.332, 0.970)),
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="pivot_eye",
    )
    pull_handle.visual(
        Cylinder(radius=0.022, length=0.32),
        origin=Origin(xyz=(0.0, 0.045, -0.180)),
        material="brushed_metal",
        name="lever_stem",
    )
    pull_handle.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.045, -0.360)),
        material="button_red",
        name="handle_knob",
    )
    handle_pivot = model.articulation(
        "bracket_to_handle",
        ArticulationType.REVOLUTE,
        parent=handle_bracket,
        child=pull_handle,
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=-0.70, upper=0.35),
    )

    # Retain references in metadata for simple pose-aware tests without
    # hard-coding string limits in two places.
    model.meta["cash_hinge_upper"] = cash_hinge.motion_limits.upper
    model.meta["handle_lower"] = handle_pivot.motion_limits.lower
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    shelf = object_model.get_part("control_shelf")
    cash_panel = object_model.get_part("cash_panel")
    bracket = object_model.get_part("handle_bracket")
    handle = object_model.get_part("pull_handle")
    cash_hinge = object_model.get_articulation("cabinet_to_cash_panel")
    handle_pivot = object_model.get_articulation("bracket_to_handle")

    ctx.expect_gap(
        cabinet,
        shelf,
        axis="x",
        positive_elem="shelf_mount_face",
        negative_elem="shelf_deck",
        max_gap=0.002,
        max_penetration=0.0,
        name="control shelf rear edge mounts to front face",
    )
    ctx.expect_gap(
        cabinet,
        shelf,
        axis="z",
        positive_elem="screen_bottom_bezel",
        negative_elem="shelf_deck",
        min_gap=0.08,
        name="control shelf remains below the screen housing",
    )

    ctx.expect_gap(
        cabinet,
        cash_panel,
        axis="x",
        positive_elem="lower_body",
        negative_elem="door_plate",
        min_gap=0.002,
        max_gap=0.010,
        name="cash door sits just proud of the lower front",
    )
    ctx.expect_overlap(
        cash_panel,
        cabinet,
        axes="yz",
        elem_a="door_plate",
        elem_b="lower_body",
        min_overlap=0.25,
        name="cash door occupies the lower front panel area",
    )

    ctx.expect_gap(
        bracket,
        cabinet,
        axis="y",
        positive_elem="bracket_plate",
        negative_elem="upper_body",
        max_gap=0.002,
        max_penetration=0.0,
        name="handle bracket plate is mounted on cabinet side",
    )
    ctx.expect_contact(
        bracket,
        handle,
        elem_a="pivot_boss",
        elem_b="pivot_eye",
        contact_tol=0.003,
        name="pull handle eye bears on the side pivot boss",
    )

    closed_cash = ctx.part_element_world_aabb(cash_panel, elem="door_plate")
    with ctx.pose({cash_hinge: object_model.meta["cash_hinge_upper"]}):
        open_cash = ctx.part_element_world_aabb(cash_panel, elem="door_plate")
    ctx.check(
        "cash panel swings outward on vertical hinge",
        closed_cash is not None
        and open_cash is not None
        and open_cash[0][0] < closed_cash[0][0] - 0.08,
        details=f"closed={closed_cash}, open={open_cash}",
    )

    rest_handle = ctx.part_element_world_aabb(handle, elem="handle_knob")
    with ctx.pose({handle_pivot: object_model.meta["handle_lower"]}):
        pulled_handle = ctx.part_element_world_aabb(handle, elem="handle_knob")
    ctx.check(
        "pull handle rotates about side pivot",
        rest_handle is not None
        and pulled_handle is not None
        and abs(pulled_handle[0][0] - rest_handle[0][0]) > 0.10,
        details=f"rest={rest_handle}, pulled={pulled_handle}",
    )

    for idx in range(5):
        ctx.expect_gap(
            f"button_{idx}",
            shelf,
            axis="z",
            positive_elem="button_cap",
            negative_elem="shelf_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button_{idx} cap rests on control shelf",
        )

    return ctx.report()


object_model = build_object_model()

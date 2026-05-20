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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_WIDTH = 0.390
BASE_DEPTH = 0.290
BASE_HEIGHT = 0.045
BASE_FLOOR = 0.006
BASE_WALL = 0.012

LID_WIDTH = 0.370
LID_DEPTH = 0.275
LID_THICKNESS = 0.024

KEY_COLUMNS = tuple(-0.1155 + 0.033 * index for index in range(8))
KEY_ROWS = (-0.047, -0.020, 0.007)
KEY_SPECS = tuple(
    (
        f"key_r{row_index + 1}_c{col_index + 1}",
        x_pos,
        y_pos,
    )
    for row_index, y_pos in enumerate(KEY_ROWS)
    for col_index, x_pos in enumerate(KEY_COLUMNS)
)


def _translate_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _build_keyboard_deck():
    outer = rounded_rect_profile(0.366, 0.112, 0.010, corner_segments=8)
    hole_profile = rounded_rect_profile(0.020, 0.014, 0.0028, corner_segments=6)
    holes = [
        _translate_profile(hole_profile, x_pos, y_pos)
        for _, x_pos, y_pos in KEY_SPECS
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            0.003,
            cap=True,
            center=True,
            closed=True,
        ),
        "field_laptop_keyboard_deck",
    )


def _add_key(model: ArticulatedObject, base, name: str, x_pos: float, y_pos: float, key_material) -> None:
    key_part = model.part(name)
    key_part.visual(
        Box((0.026, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0063)),
        material=key_material,
        name="cap",
    )
    key_part.visual(
        Box((0.018, 0.012, 0.0048)),
        origin=Origin(xyz=(0.0, 0.0, 0.0024)),
        material=key_material,
        name="stem",
    )
    key_part.visual(
        Box((0.022, 0.016, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=key_material,
        name="retainer",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((0.026, 0.018, 0.0088)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
    )
    model.articulation(
        f"{name}_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key_part,
        origin=Origin(xyz=(x_pos, y_pos, 0.032)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=0.06,
            lower=0.0,
            upper=0.0015,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_field_laptop")

    body = model.material("body_olive", rgba=(0.28, 0.31, 0.26, 1.0))
    bumper = model.material("bumper_black", rgba=(0.08, 0.09, 0.10, 1.0))
    deck = model.material("deck_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    key_material = model.material("key_dark", rgba=(0.24, 0.25, 0.26, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.07, 0.12, 0.16, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.62, 0.66, 0.70, 1.0))
    trim = model.material("trim_dark", rgba=(0.17, 0.19, 0.18, 1.0))

    keyboard_deck_mesh = _build_keyboard_deck()

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, BASE_FLOOR * 0.5)),
        material=body,
        name="floor_pan",
    )
    base.visual(
        Box((BASE_WALL, BASE_DEPTH, BASE_HEIGHT - BASE_FLOOR)),
        origin=Origin(
            xyz=(BASE_WIDTH * 0.5 - BASE_WALL * 0.5, 0.0, BASE_FLOOR + (BASE_HEIGHT - BASE_FLOOR) * 0.5)
        ),
        material=body,
        name="right_wall",
    )
    base.visual(
        Box((BASE_WALL, BASE_DEPTH, BASE_HEIGHT - BASE_FLOOR)),
        origin=Origin(
            xyz=(-BASE_WIDTH * 0.5 + BASE_WALL * 0.5, 0.0, BASE_FLOOR + (BASE_HEIGHT - BASE_FLOOR) * 0.5)
        ),
        material=body,
        name="left_wall",
    )
    base.visual(
        Box((BASE_WIDTH - 2.0 * BASE_WALL, BASE_WALL, BASE_HEIGHT - BASE_FLOOR)),
        origin=Origin(
            xyz=(0.0, BASE_DEPTH * 0.5 - BASE_WALL * 0.5, BASE_FLOOR + (BASE_HEIGHT - BASE_FLOOR) * 0.5)
        ),
        material=body,
        name="front_wall",
    )
    base.visual(
        Box((BASE_WIDTH - 2.0 * BASE_WALL, BASE_WALL, BASE_HEIGHT - BASE_FLOOR)),
        origin=Origin(
            xyz=(0.0, -BASE_DEPTH * 0.5 + BASE_WALL * 0.5, BASE_FLOOR + (BASE_HEIGHT - BASE_FLOOR) * 0.5)
        ),
        material=body,
        name="rear_wall",
    )
    base.visual(
        Box((0.366, 0.112, 0.003)),
        origin=Origin(xyz=(0.0, 0.087, 0.0335)),
        material=deck,
        name="palmrest_plate",
    )
    base.visual(
        Box((0.090, 0.050, 0.0015)),
        origin=Origin(xyz=(0.0, 0.090, 0.03425)),
        material=screen_glass,
        name="touchpad",
    )
    base.visual(
        keyboard_deck_mesh,
        origin=Origin(xyz=(0.0, -0.020, 0.0335)),
        material=deck,
        name="keyboard_deck",
    )
    base.visual(
        Box((0.040, 0.032, 0.010)),
        origin=Origin(xyz=(-0.175, 0.116, 0.040)),
        material=trim,
        name="left_palm_boss",
    )
    base.visual(
        Box((0.040, 0.032, 0.010)),
        origin=Origin(xyz=(0.175, 0.116, 0.040)),
        material=trim,
        name="right_palm_boss",
    )
    base.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(-0.190, 0.133, 0.050)),
        material=bumper,
        name="left_pedestal",
    )
    base.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(0.190, 0.133, 0.050)),
        material=bumper,
        name="right_pedestal",
    )
    base.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(-0.186, -0.136, 0.013)),
        material=bumper,
        name="rear_left_bumper",
    )
    base.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(0.186, -0.136, 0.013)),
        material=bumper,
        name="rear_right_bumper",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, 0.004)),
        origin=Origin(xyz=(0.0, LID_DEPTH * 0.5, LID_THICKNESS - 0.002)),
        material=body,
        name="back_cover",
    )
    lid.visual(
        Box((0.012, LID_DEPTH, LID_THICKNESS - 0.004)),
        origin=Origin(xyz=(LID_WIDTH * 0.5 - 0.006, LID_DEPTH * 0.5, (LID_THICKNESS - 0.004) * 0.5)),
        material=body,
        name="right_shell",
    )
    lid.visual(
        Box((0.012, LID_DEPTH, LID_THICKNESS - 0.004)),
        origin=Origin(xyz=(-LID_WIDTH * 0.5 + 0.006, LID_DEPTH * 0.5, (LID_THICKNESS - 0.004) * 0.5)),
        material=body,
        name="left_shell",
    )
    lid.visual(
        Box((LID_WIDTH - 0.024, 0.012, LID_THICKNESS - 0.004)),
        origin=Origin(xyz=(0.0, 0.006, (LID_THICKNESS - 0.004) * 0.5)),
        material=body,
        name="rear_shell",
    )
    lid.visual(
        Box((LID_WIDTH - 0.024, 0.012, LID_THICKNESS - 0.004)),
        origin=Origin(xyz=(0.0, LID_DEPTH - 0.006, (LID_THICKNESS - 0.004) * 0.5)),
        material=body,
        name="front_shell",
    )
    lid.visual(
        Box((LID_WIDTH - 0.024, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.012, 0.002)),
        material=trim,
        name="rear_bezel",
    )
    lid.visual(
        Box((LID_WIDTH - 0.024, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, LID_DEPTH - 0.015, 0.002)),
        material=trim,
        name="front_bezel",
    )
    lid.visual(
        Box((0.028, 0.205, 0.004)),
        origin=Origin(xyz=(-0.171, 0.140, 0.002)),
        material=trim,
        name="left_bezel",
    )
    lid.visual(
        Box((0.028, 0.205, 0.004)),
        origin=Origin(xyz=(0.171, 0.140, 0.002)),
        material=trim,
        name="right_bezel",
    )
    lid.visual(
        Box((0.314, 0.221, 0.002)),
        origin=Origin(xyz=(0.0, 0.1345, 0.001)),
        material=screen_glass,
        name="display_panel",
    )
    lid.visual(
        Box((0.014, 0.010, 0.008)),
        origin=Origin(xyz=(-0.166, 0.262, 0.007)),
        material=latch_metal,
        name="left_keeper",
    )
    lid.visual(
        Box((0.014, 0.010, 0.008)),
        origin=Origin(xyz=(0.166, 0.262, 0.007)),
        material=latch_metal,
        name="right_keeper",
    )
    lid.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(-0.175, 0.264, 0.019)),
        material=bumper,
        name="left_corner_guard",
    )
    lid.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.175, 0.264, 0.019)),
        material=bumper,
        name="right_corner_guard",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=2.0,
        origin=Origin(xyz=(0.0, LID_DEPTH * 0.5, LID_THICKNESS * 0.5)),
    )

    left_hook = model.part("left_hook")
    left_hook.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=latch_metal,
        name="pivot",
    )
    left_hook.visual(
        Box((0.028, 0.024, 0.004)),
        origin=Origin(xyz=(0.014, -0.007, 0.003)),
        material=latch_metal,
        name="arm",
    )
    left_hook.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.024, -0.019, 0.0)),
        material=latch_metal,
        name="toe",
    )
    left_hook.inertial = Inertial.from_geometry(
        Box((0.032, 0.026, 0.014)),
        mass=0.04,
        origin=Origin(xyz=(0.014, -0.007, 0.004)),
    )

    right_hook = model.part("right_hook")
    right_hook.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=latch_metal,
        name="pivot",
    )
    right_hook.visual(
        Box((0.028, 0.024, 0.004)),
        origin=Origin(xyz=(-0.014, -0.007, 0.003)),
        material=latch_metal,
        name="arm",
    )
    right_hook.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(-0.024, -0.019, 0.0)),
        material=latch_metal,
        name="toe",
    )
    right_hook.inertial = Inertial.from_geometry(
        Box((0.032, 0.026, 0.014)),
        mass=0.04,
        origin=Origin(xyz=(-0.014, -0.007, 0.004)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -0.139, BASE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "base_to_left_hook",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_hook,
        origin=Origin(xyz=(-0.190, 0.133, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "base_to_right_hook",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_hook,
        origin=Origin(xyz=(0.190, 0.133, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.0,
        ),
    )

    for key_name, x_pos, y_pos in KEY_SPECS:
        _add_key(model, base, key_name, x_pos, y_pos, key_material)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    left_hook = object_model.get_part("left_hook")
    right_hook = object_model.get_part("right_hook")
    lid_hinge = object_model.get_articulation("base_to_lid")
    left_latch_joint = object_model.get_articulation("base_to_left_hook")
    right_latch_joint = object_model.get_articulation("base_to_right_hook")

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

    ctx.check(
        "lid hinge axis is lateral",
        all(abs(a - b) < 1e-9 for a, b in zip(lid_hinge.axis, (1.0, 0.0, 0.0))),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "latch joints rotate about vertical pivots",
        all(abs(a - b) < 1e-9 for a, b in zip(left_latch_joint.axis, (0.0, 0.0, 1.0)))
        and all(abs(a - b) < 1e-9 for a, b in zip(right_latch_joint.axis, (0.0, 0.0, 1.0))),
        details=f"left={left_latch_joint.axis} right={right_latch_joint.axis}",
    )

    ctx.expect_contact(lid, base, name="lid seats on lower chassis")
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.27, name="lid covers base footprint")
    ctx.expect_contact(left_hook, base, elem_a="pivot", elem_b="left_pedestal", name="left hook is mounted on pedestal")
    ctx.expect_contact(right_hook, base, elem_a="pivot", elem_b="right_pedestal", name="right hook is mounted on pedestal")
    ctx.expect_gap(
        left_hook,
        lid,
        axis="z",
        positive_elem="arm",
        negative_elem="left_keeper",
        max_gap=0.001,
        max_penetration=0.0,
        name="left hook arm rests on left keeper",
    )
    ctx.expect_gap(
        right_hook,
        lid,
        axis="z",
        positive_elem="arm",
        negative_elem="right_keeper",
        max_gap=0.001,
        max_penetration=0.0,
        name="right hook arm rests on right keeper",
    )
    ctx.expect_gap(
        lid,
        left_hook,
        axis="y",
        positive_elem="left_keeper",
        negative_elem="toe",
        max_gap=0.001,
        max_penetration=1e-5,
        name="left hook toe clamps keeper front face",
    )
    ctx.expect_gap(
        lid,
        right_hook,
        axis="y",
        positive_elem="right_keeper",
        negative_elem="toe",
        max_gap=0.001,
        max_penetration=1e-5,
        name="right hook toe clamps keeper front face",
    )
    ctx.expect_overlap(
        left_hook,
        lid,
        axes="xy",
        elem_a="arm",
        elem_b="left_keeper",
        min_overlap=0.006,
        name="left latch overlaps keeper in plan",
    )
    ctx.expect_overlap(
        right_hook,
        lid,
        axes="xy",
        elem_a="arm",
        elem_b="right_keeper",
        min_overlap=0.006,
        name="right latch overlaps keeper in plan",
    )

    key_sample = object_model.get_part("key_r2_c4")
    key_sample_joint = object_model.get_articulation("key_r2_c4_slide")
    key_sample_rest = ctx.part_world_position(key_sample)
    assert key_sample_rest is not None

    for key_name, _, _ in KEY_SPECS:
        key_part = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(f"{key_name}_slide")
        ctx.check(
            f"{key_name} has vertical plunger axis",
            all(abs(a - b) < 1e-9 for a, b in zip(key_joint.axis, (0.0, 0.0, -1.0))),
            details=f"axis={key_joint.axis}",
        )
        ctx.expect_gap(
            base,
            key_part,
            axis="z",
            positive_elem="keyboard_deck",
            negative_elem="retainer",
            max_gap=0.0002,
            max_penetration=0.0,
            name=f"{key_name} retainer seats under keyboard deck",
        )

    with ctx.pose({left_latch_joint: 1.0, right_latch_joint: -1.0, lid_hinge: 1.35}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_shell",
            min_gap=0.20,
            name="opened lid front edge lifts above base",
        )

    with ctx.pose({key_sample_joint: 0.0015}):
        key_sample_pressed = ctx.part_world_position(key_sample)
        assert key_sample_pressed is not None
        ctx.check(
            "sample key plunges downward when actuated",
            key_sample_pressed[2] < key_sample_rest[2] - 0.001,
            details=f"rest={key_sample_rest} pressed={key_sample_pressed}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

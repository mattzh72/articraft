from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_WIDTH = 0.348
BASE_DEPTH = 0.262
BASE_SHELL_HEIGHT = 0.016
BASE_TOP_HEIGHT = 0.044
HINGE_Y = 0.120
HINGE_Z = 0.045
LID_WIDTH = 0.328
LID_DEPTH = 0.252
LID_THICKNESS = 0.028


def _rounded_panel_mesh(
    *,
    name: str,
    width: float,
    depth: float,
    thickness: float,
    radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius),
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def _rounded_frame_mesh(
    *,
    name: str,
    width: float,
    depth: float,
    inner_width: float,
    inner_depth: float,
    thickness: float,
    outer_radius: float,
    inner_radius: float,
):
    outer = rounded_rect_profile(width, depth, outer_radius)
    inner = rounded_rect_profile(inner_width, inner_depth, inner_radius)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            thickness,
            cap=True,
            center=False,
            closed=True,
        ),
        name,
    )


def _add_key(
    model: ArticulatedObject,
    base_part,
    *,
    name: str,
    center_x: float,
    center_y: float,
    width: float,
    depth: float,
    top_z: float,
    key_material,
    stem_material,
) -> None:
    key_part = model.part(name)
    stem_width = min(width * 0.38, 0.015)
    stem_depth = min(depth * 0.42, 0.012)
    key_part.visual(
        Box((width, depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=key_material,
        name="key_cap",
    )
    key_part.visual(
        Box((stem_width, stem_depth, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=stem_material,
        name="key_stem",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((width, depth, 0.022)),
        mass=0.010 if width < 0.08 else 0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )
    model.articulation(
        f"base_to_{name}",
        ArticulationType.PRISMATIC,
        parent=base_part,
        child=key_part,
        origin=Origin(xyz=(center_x, center_y, top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0025,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_field_laptop")

    shell_dark = model.material("shell_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.25, 0.27, 0.29, 1.0))
    bumper = model.material("bumper", rgba=(0.08, 0.09, 0.10, 1.0))
    deck = model.material("deck", rgba=(0.12, 0.13, 0.14, 1.0))
    keycap = model.material("keycap", rgba=(0.14, 0.15, 0.16, 1.0))
    stem = model.material("stem", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.20, 0.24, 0.95))
    trackpad = model.material("trackpad", rgba=(0.23, 0.25, 0.27, 1.0))
    accent = model.material("accent", rgba=(0.48, 0.50, 0.52, 1.0))

    base = model.part("base_chassis")
    base.visual(
        _rounded_panel_mesh(
            name="base_shell_mesh",
            width=BASE_WIDTH,
            depth=BASE_DEPTH,
            thickness=BASE_SHELL_HEIGHT,
            radius=0.024,
        ),
        material=shell_dark,
        name="base_shell",
    )
    base.visual(
        Box((0.300, 0.074, BASE_TOP_HEIGHT - BASE_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.091, 0.030)),
        material=shell_mid,
        name="front_block",
    )
    base.visual(
        Box((0.300, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.106, 0.031)),
        material=shell_mid,
        name="rear_block",
    )
    base.visual(
        Box((0.024, 0.224, BASE_TOP_HEIGHT - BASE_SHELL_HEIGHT)),
        origin=Origin(xyz=(-0.162, 0.008, 0.030)),
        material=shell_mid,
        name="left_rail",
    )
    base.visual(
        Box((0.024, 0.224, BASE_TOP_HEIGHT - BASE_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.162, 0.008, 0.030)),
        material=shell_mid,
        name="right_rail",
    )
    base.visual(
        Box((0.300, 0.142, 0.002)),
        origin=Origin(xyz=(0.0, 0.013, 0.019)),
        material=deck,
        name="keyboard_floor",
    )
    base.visual(
        Box((0.086, 0.056, 0.0015)),
        origin=Origin(xyz=(0.0, -0.081, 0.04475)),
        material=trackpad,
        name="trackpad",
    )
    for side, x in (("left", -0.154), ("right", 0.154)):
        base.visual(
            Box((0.030, 0.030, 0.014)),
            origin=Origin(xyz=(x, -0.114, 0.023)),
            material=bumper,
            name=f"{side}_front_bumper",
        )
        base.visual(
            Box((0.030, 0.030, 0.014)),
            origin=Origin(xyz=(x, 0.114, 0.023)),
            material=bumper,
            name=f"{side}_rear_bumper",
        )
        base.visual(
            Box((0.022, 0.016, 0.004)),
            origin=Origin(xyz=(x * 0.935, -0.132, 0.046)),
            material=accent,
            name=f"{side}_hook_mount",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_TOP_HEIGHT)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_HEIGHT * 0.5)),
    )

    lid = model.part("lid_housing")
    lid.visual(
        _rounded_panel_mesh(
            name="lid_back_shell_mesh",
            width=LID_WIDTH,
            depth=LID_DEPTH,
            thickness=0.006,
            radius=0.020,
        ),
        origin=Origin(xyz=(0.0, -0.126, 0.022)),
        material=shell_mid,
        name="lid_back_shell",
    )
    lid.visual(
        Box((0.008, LID_DEPTH, 0.020)),
        origin=Origin(xyz=(-0.160, -0.126, 0.013)),
        material=shell_dark,
        name="left_lid_wall",
    )
    lid.visual(
        Box((0.008, LID_DEPTH, 0.020)),
        origin=Origin(xyz=(0.160, -0.126, 0.013)),
        material=shell_dark,
        name="right_lid_wall",
    )
    lid.visual(
        Box((0.312, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.248, 0.013)),
        material=shell_dark,
        name="front_lid_wall",
    )
    lid.visual(
        Box((0.312, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.005, 0.013)),
        material=shell_dark,
        name="rear_lid_wall",
    )
    lid.visual(
        _rounded_frame_mesh(
            name="lid_bezel_mesh",
            width=LID_WIDTH,
            depth=LID_DEPTH,
            inner_width=0.286,
            inner_depth=0.184,
            thickness=0.004,
            outer_radius=0.018,
            inner_radius=0.010,
        ),
        origin=Origin(xyz=(0.0, -0.126, 0.003)),
        material=shell_dark,
        name="front_bezel",
    )
    lid.visual(
        Box((0.016, 0.172, 0.014)),
        origin=Origin(xyz=(-0.146, -0.126, 0.015)),
        material=shell_dark,
        name="left_display_retainer",
    )
    lid.visual(
        Box((0.016, 0.172, 0.014)),
        origin=Origin(xyz=(0.146, -0.126, 0.015)),
        material=shell_dark,
        name="right_display_retainer",
    )
    lid.visual(
        Box((0.018, 0.010, 0.002)),
        origin=Origin(xyz=(-0.116, -0.241, 0.004)),
        material=accent,
        name="left_latch_plate",
    )
    lid.visual(
        Box((0.018, 0.010, 0.002)),
        origin=Origin(xyz=(0.116, -0.241, 0.004)),
        material=accent,
        name="right_latch_plate",
    )
    for side, x in (("left", -0.151), ("right", 0.151)):
        lid.visual(
            Box((0.024, 0.024, 0.010)),
            origin=Origin(xyz=(x, -0.118, 0.014)),
            material=bumper,
            name=f"{side}_front_lid_bumper",
        )
        lid.visual(
            Box((0.024, 0.024, 0.010)),
            origin=Origin(xyz=(x, -0.014, 0.014)),
            material=bumper,
            name=f"{side}_rear_lid_bumper",
        )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -0.126, 0.014)),
    )
    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=2.20,
        ),
    )

    display = model.part("display_panel")
    display.visual(
        Box((0.276, 0.174, 0.004)),
        material=glass,
        name="screen_glass",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.276, 0.174, 0.004)),
        mass=0.18,
    )
    model.articulation(
        "lid_to_display",
        ArticulationType.FIXED,
        parent=lid,
        child=display,
        origin=Origin(xyz=(0.0, -0.126, 0.009)),
    )

    left_hook = model.part("left_latch_hook")
    left_hook.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=bumper,
        name="pivot_barrel",
    )
    left_hook.visual(
        Box((0.030, 0.010, 0.008)),
        origin=Origin(xyz=(0.017, -0.006, 0.004)),
        material=bumper,
        name="hook_arm",
    )
    left_hook.visual(
        Box((0.008, 0.006, 0.010)),
        origin=Origin(xyz=(0.032, -0.014, -0.002)),
        material=accent,
        name="hook_tooth",
    )
    left_hook.inertial = Inertial.from_geometry(
        Box((0.038, 0.020, 0.016)),
        mass=0.035,
        origin=Origin(xyz=(0.016, -0.007, 0.003)),
    )
    left_hook_joint = model.articulation(
        "base_to_left_latch_hook",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_hook,
        origin=Origin(xyz=(-0.140, -0.140, 0.048)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    right_hook = model.part("right_latch_hook")
    right_hook.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=bumper,
        name="pivot_barrel",
    )
    right_hook.visual(
        Box((0.030, 0.010, 0.008)),
        origin=Origin(xyz=(-0.017, -0.006, 0.004)),
        material=bumper,
        name="hook_arm",
    )
    right_hook.visual(
        Box((0.008, 0.006, 0.010)),
        origin=Origin(xyz=(-0.032, -0.014, -0.002)),
        material=accent,
        name="hook_tooth",
    )
    right_hook.inertial = Inertial.from_geometry(
        Box((0.038, 0.020, 0.016)),
        mass=0.035,
        origin=Origin(xyz=(-0.016, -0.007, 0.003)),
    )
    right_hook_joint = model.articulation(
        "base_to_right_latch_hook",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_hook,
        origin=Origin(xyz=(0.140, -0.140, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    key_rows = [
        (-0.118, -0.071, -0.024, 0.024, 0.071, 0.118),
        (-0.118, -0.071, -0.024, 0.024, 0.071, 0.118),
        (-0.118, -0.071, -0.024, 0.024, 0.071, 0.118),
    ]
    for row_index, y in enumerate((0.056, 0.030, 0.004)):
        for col_index, x in enumerate(key_rows[row_index]):
            _add_key(
                model,
                base,
                name=f"key_r{row_index}_c{col_index}",
                center_x=x,
                center_y=y,
                width=0.038,
                depth=0.022,
                top_z=0.034,
                key_material=keycap,
                stem_material=stem,
            )

    bottom_row = [
        ("bottom_left_mod", -0.117, 0.032),
        ("bottom_left_alt", -0.078, 0.032),
        ("spacebar", 0.0, 0.128),
        ("bottom_right_alt", 0.078, 0.032),
        ("bottom_right_mod", 0.117, 0.032),
    ]
    for key_name, x, width in bottom_row:
        _add_key(
            model,
            base,
            name=key_name,
            center_x=x,
            center_y=-0.022,
            width=width,
            depth=0.022,
            top_z=0.034,
            key_material=keycap,
            stem_material=stem,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_chassis")
    lid = object_model.get_part("lid_housing")
    display = object_model.get_part("display_panel")
    left_hook = object_model.get_part("left_latch_hook")
    right_hook = object_model.get_part("right_latch_hook")
    center_key = object_model.get_part("key_r1_c2")
    spacebar = object_model.get_part("spacebar")

    lid_hinge = object_model.get_articulation("base_to_lid")
    left_hook_joint = object_model.get_articulation("base_to_left_latch_hook")
    right_hook_joint = object_model.get_articulation("base_to_right_latch_hook")
    center_key_joint = object_model.get_articulation("base_to_key_r1_c2")
    spacebar_joint = object_model.get_articulation("base_to_spacebar")

    ctx.expect_overlap(
        base,
        lid,
        axes="xy",
        min_overlap=0.220,
        name="lid covers most of the base footprint when closed",
    )
    ctx.expect_within(
        display,
        lid,
        axes="xy",
        margin=0.010,
        name="display panel stays within the lid bezel footprint",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.90}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_left_tooth = ctx.part_element_world_aabb(left_hook, elem="hook_tooth")
    with ctx.pose({left_hook_joint: 0.90}):
        open_left_tooth = ctx.part_element_world_aabb(left_hook, elem="hook_tooth")
    ctx.check(
        "left latch hook swings forward to release",
        closed_left_tooth is not None
        and open_left_tooth is not None
        and open_left_tooth[0][1] < closed_left_tooth[0][1] - 0.010,
        details=f"closed={closed_left_tooth}, open={open_left_tooth}",
    )

    closed_right_tooth = ctx.part_element_world_aabb(right_hook, elem="hook_tooth")
    with ctx.pose({right_hook_joint: 0.90}):
        open_right_tooth = ctx.part_element_world_aabb(right_hook, elem="hook_tooth")
    ctx.check(
        "right latch hook swings forward to release",
        closed_right_tooth is not None
        and open_right_tooth is not None
        and open_right_tooth[0][1] < closed_right_tooth[0][1] - 0.010,
        details=f"closed={closed_right_tooth}, open={open_right_tooth}",
    )

    center_key_rest = ctx.part_world_position(center_key)
    with ctx.pose({center_key_joint: 0.0022}):
        center_key_pressed = ctx.part_world_position(center_key)
    ctx.check(
        "center key plunges downward",
        center_key_rest is not None
        and center_key_pressed is not None
        and center_key_pressed[2] < center_key_rest[2] - 0.0015,
        details=f"rest={center_key_rest}, pressed={center_key_pressed}",
    )

    spacebar_rest = ctx.part_world_position(spacebar)
    with ctx.pose({spacebar_joint: 0.0022}):
        spacebar_pressed = ctx.part_world_position(spacebar)
    ctx.check(
        "spacebar plunges downward",
        spacebar_rest is not None
        and spacebar_pressed is not None
        and spacebar_pressed[2] < spacebar_rest[2] - 0.0015,
        details=f"rest={spacebar_rest}, pressed={spacebar_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

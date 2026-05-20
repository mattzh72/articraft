from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_W = 0.92
CABINET_D = 0.70
CABINET_H = 1.86
WALL = 0.045
DOOR_T = 0.070
FRONT_Y = -CABINET_D / 2.0
HINGE_Y = FRONT_Y - 0.008
CENTER_GAP = 0.014

UPPER_BOTTOM = 0.88
UPPER_H = 0.94
LOWER_BOTTOM = 0.11
LOWER_H = 0.72


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _vertical_handle(part, *, x, z0, z1, material, prefix="handle"):
    """A raised pull: two mounting pads tied to a vertical round grip."""
    z_mid = (z0 + z1) * 0.5
    length = z1 - z0
    y_bar = -DOOR_T - 0.045
    y_mount = -DOOR_T - 0.023

    part.visual(
        Cylinder(radius=0.018, length=length),
        origin=Origin(xyz=(x, y_bar, z_mid)),
        material=material,
        name=f"{prefix}_bar",
    )
    for i, z in enumerate((z0 + 0.075, z1 - 0.075)):
        _box(
            part,
            f"{prefix}_mount_{i}",
            (0.055, 0.056, 0.075),
            (x, y_mount, z),
            material,
        )


def _add_door_skin(part, *, width, height, center_x, material, trim_material):
    """Main slab plus small dark reveal strips seated on the front face."""
    _box(part, "door_panel", (width, DOOR_T, height), (center_x, -DOOR_T / 2.0, height / 2.0), material)

    sign = 1.0 if center_x > 0.0 else -1.0
    free_edge_x = center_x + sign * (width / 2.0 - 0.006)
    hinge_edge_x = center_x - sign * (width / 2.0 - 0.006)
    front_y = -DOOR_T - 0.003
    strip_z = height / 2.0

    _box(part, "center_reveal", (0.012, 0.006, height - 0.040), (free_edge_x, front_y, strip_z), trim_material)
    _box(part, "hinge_reveal", (0.010, 0.006, height - 0.060), (hinge_edge_x, front_y, strip_z), trim_material)
    _box(part, "top_reveal", (width - 0.030, 0.006, 0.012), (center_x, front_y, height - 0.016), trim_material)
    _box(part, "bottom_reveal", (width - 0.030, 0.006, 0.012), (center_x, front_y, 0.016), trim_material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hinged_french_door_refrigerator")

    stainless = Material("brushed_warm_stainless", rgba=(0.72, 0.72, 0.69, 1.0))
    side_silver = Material("satin_side_silver", rgba=(0.60, 0.62, 0.62, 1.0))
    white_liner = Material("white_plastic_liner", rgba=(0.92, 0.94, 0.93, 1.0))
    dark_gasket = Material("dark_rubber_gasket", rgba=(0.025, 0.027, 0.030, 1.0))
    black_shadow = Material("black_recess_shadow", rgba=(0.010, 0.012, 0.014, 1.0))
    glass = Material("pale_glass_shelf", rgba=(0.62, 0.82, 0.92, 0.55))
    handle_metal = Material("satin_handle_metal", rgba=(0.82, 0.82, 0.78, 1.0))

    cabinet = model.part("cabinet")

    # One connected cabinet shell: side walls, back, bottom/top, front rails, and
    # internal mullions that divide the two upper food compartments from the
    # larger lower freezer compartment.
    _box(cabinet, "back_panel", (CABINET_W, WALL, CABINET_H), (0.0, CABINET_D / 2.0 - WALL / 2.0, CABINET_H / 2.0), white_liner)
    _box(cabinet, "side_wall_0", (WALL, CABINET_D, CABINET_H), (-CABINET_W / 2.0 + WALL / 2.0, 0.0, CABINET_H / 2.0), side_silver)
    _box(cabinet, "side_wall_1", (WALL, CABINET_D, CABINET_H), (CABINET_W / 2.0 - WALL / 2.0, 0.0, CABINET_H / 2.0), side_silver)
    _box(cabinet, "top_wall", (CABINET_W, CABINET_D, WALL), (0.0, 0.0, CABINET_H - WALL / 2.0), side_silver)
    _box(cabinet, "bottom_wall", (CABINET_W, CABINET_D, WALL), (0.0, 0.0, WALL / 2.0), side_silver)

    _box(cabinet, "front_left_stile", (0.060, 0.050, CABINET_H), (-CABINET_W / 2.0 + 0.030, FRONT_Y + 0.025, CABINET_H / 2.0), side_silver)
    _box(cabinet, "front_right_stile", (0.060, 0.050, CABINET_H), (CABINET_W / 2.0 - 0.030, FRONT_Y + 0.025, CABINET_H / 2.0), side_silver)
    _box(cabinet, "front_top_rail", (CABINET_W, 0.050, 0.060), (0.0, FRONT_Y + 0.025, CABINET_H - 0.030), side_silver)
    _box(cabinet, "front_bottom_rail", (CABINET_W, 0.050, 0.070), (0.0, FRONT_Y + 0.025, 0.035), side_silver)
    _box(cabinet, "middle_rail", (CABINET_W, 0.050, 0.060), (0.0, FRONT_Y + 0.025, UPPER_BOTTOM - 0.030), side_silver)
    _box(cabinet, "upper_center_mullion", (0.038, 0.055, UPPER_H + 0.040), (0.0, FRONT_Y + 0.027, UPPER_BOTTOM + UPPER_H / 2.0), side_silver)

    _box(cabinet, "horizontal_divider", (CABINET_W - 2.0 * WALL, CABINET_D - 0.040, 0.042), (0.0, 0.020, UPPER_BOTTOM - 0.021), white_liner)
    _box(cabinet, "upper_vertical_divider", (0.035, CABINET_D - 0.055, CABINET_H - UPPER_BOTTOM - WALL), (0.0, 0.020, (UPPER_BOTTOM + CABINET_H - WALL) / 2.0), white_liner)
    _box(cabinet, "upper_shelf_0", (CABINET_W / 2.0 - WALL - 0.018, CABINET_D - 0.160, 0.014), (-CABINET_W / 4.0 - 0.009, 0.035, 1.30), glass)
    _box(cabinet, "upper_shelf_1", (CABINET_W / 2.0 - WALL - 0.018, CABINET_D - 0.160, 0.014), (CABINET_W / 4.0 + 0.009, 0.035, 1.30), glass)
    _box(cabinet, "freezer_floor_lip", (CABINET_W - 0.120, 0.050, 0.050), (0.0, FRONT_Y + 0.025, LOWER_BOTTOM - 0.010), white_liner)

    # Exposed vertical hinge barrels provide the visible side-hinged support
    # path. They are attached to the cabinet stiles and pass through the door
    # hinge edge in the simplified visual model.
    cabinet.visual(
        Cylinder(radius=0.014, length=UPPER_H - 0.120),
        origin=Origin(xyz=(-CABINET_W / 2.0, HINGE_Y, UPPER_BOTTOM + UPPER_H / 2.0)),
        material=handle_metal,
        name="upper_hinge_pin_0",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=UPPER_H - 0.120),
        origin=Origin(xyz=(CABINET_W / 2.0, HINGE_Y, UPPER_BOTTOM + UPPER_H / 2.0)),
        material=handle_metal,
        name="upper_hinge_pin_1",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=LOWER_H - 0.100),
        origin=Origin(xyz=(-CABINET_W / 2.0, HINGE_Y, LOWER_BOTTOM + LOWER_H / 2.0)),
        material=handle_metal,
        name="freezer_hinge_pin",
    )

    # A dark toe grille sits below the lower side-hinged freezer door.
    _box(cabinet, "toe_grille_panel", (CABINET_W - 0.130, 0.024, 0.055), (0.0, FRONT_Y + 0.003, 0.064), black_shadow)
    for i, z in enumerate((0.050, 0.064, 0.078)):
        _box(cabinet, f"toe_slat_{i}", (CABINET_W - 0.180, 0.010, 0.004), (0.0, FRONT_Y - 0.010, z), side_silver)

    upper_w = CABINET_W / 2.0 - CENTER_GAP / 2.0
    lower_w = CABINET_W - CENTER_GAP

    upper_door_0 = model.part("upper_door_0")
    _add_door_skin(upper_door_0, width=upper_w, height=UPPER_H, center_x=upper_w / 2.0, material=stainless, trim_material=dark_gasket)
    _vertical_handle(upper_door_0, x=upper_w - 0.070, z0=0.145, z1=UPPER_H - 0.145, material=handle_metal)

    upper_door_1 = model.part("upper_door_1")
    _add_door_skin(upper_door_1, width=upper_w, height=UPPER_H, center_x=-upper_w / 2.0, material=stainless, trim_material=dark_gasket)
    _vertical_handle(upper_door_1, x=-(upper_w - 0.070), z0=0.145, z1=UPPER_H - 0.145, material=handle_metal)

    freezer_door = model.part("freezer_door")
    _add_door_skin(freezer_door, width=lower_w, height=LOWER_H, center_x=lower_w / 2.0, material=stainless, trim_material=dark_gasket)
    _vertical_handle(freezer_door, x=lower_w - 0.080, z0=0.150, z1=LOWER_H - 0.110, material=handle_metal)

    model.articulation(
        "upper_hinge_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door_0,
        origin=Origin(xyz=(-CABINET_W / 2.0, HINGE_Y, UPPER_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.3, lower=0.0, upper=1.85),
    )
    model.articulation(
        "upper_hinge_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door_1,
        origin=Origin(xyz=(CABINET_W / 2.0, HINGE_Y, UPPER_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.3, lower=0.0, upper=1.85),
    )
    model.articulation(
        "freezer_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(-CABINET_W / 2.0, HINGE_Y, LOWER_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    upper_door_0 = object_model.get_part("upper_door_0")
    upper_door_1 = object_model.get_part("upper_door_1")
    freezer_door = object_model.get_part("freezer_door")
    upper_hinge_0 = object_model.get_articulation("upper_hinge_0")
    upper_hinge_1 = object_model.get_articulation("upper_hinge_1")
    freezer_hinge = object_model.get_articulation("freezer_hinge")

    ctx.allow_overlap(
        cabinet,
        upper_door_0,
        elem_a="upper_hinge_pin_0",
        elem_b="door_panel",
        reason="The visible hinge pin is intentionally captured in the door edge knuckle proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        upper_door_1,
        elem_a="upper_hinge_pin_1",
        elem_b="door_panel",
        reason="The visible hinge pin is intentionally captured in the door edge knuckle proxy.",
    )
    ctx.allow_overlap(
        cabinet,
        freezer_door,
        elem_a="freezer_hinge_pin",
        elem_b="door_panel",
        reason="The visible hinge pin is intentionally captured in the lower door edge knuckle proxy.",
    )

    ctx.expect_overlap(
        cabinet,
        upper_door_0,
        axes="z",
        min_overlap=0.75,
        elem_a="upper_hinge_pin_0",
        elem_b="door_panel",
        name="left upper hinge pin runs through the door edge",
    )
    ctx.expect_overlap(
        cabinet,
        upper_door_1,
        axes="z",
        min_overlap=0.75,
        elem_a="upper_hinge_pin_1",
        elem_b="door_panel",
        name="right upper hinge pin runs through the door edge",
    )
    ctx.expect_overlap(
        cabinet,
        freezer_door,
        axes="z",
        min_overlap=0.55,
        elem_a="freezer_hinge_pin",
        elem_b="door_panel",
        name="lower hinge pin runs through the door edge",
    )

    ctx.expect_gap(
        cabinet,
        upper_door_0,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="front_left_stile",
        negative_elem="door_panel",
        name="left upper door sits just proud of cabinet",
    )
    ctx.expect_gap(
        cabinet,
        upper_door_1,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="front_right_stile",
        negative_elem="door_panel",
        name="right upper door sits just proud of cabinet",
    )
    ctx.expect_gap(
        cabinet,
        freezer_door,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="front_left_stile",
        negative_elem="door_panel",
        name="lower door sits just proud of cabinet",
    )
    ctx.expect_gap(
        upper_door_1,
        upper_door_0,
        axis="x",
        min_gap=0.006,
        max_gap=0.024,
        positive_elem="door_panel",
        negative_elem="door_panel",
        name="two upper doors have a central closing reveal",
    )
    ctx.expect_gap(
        upper_door_0,
        freezer_door,
        axis="z",
        min_gap=0.030,
        max_gap=0.070,
        positive_elem="door_panel",
        negative_elem="door_panel",
        name="upper and lower doors are separated by the cabinet rail",
    )
    ctx.expect_overlap(
        upper_door_0,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="door_panel",
        elem_b="back_panel",
        name="left upper door covers a narrow upper compartment",
    )
    ctx.expect_overlap(
        upper_door_1,
        cabinet,
        axes="xz",
        min_overlap=0.35,
        elem_a="door_panel",
        elem_b="back_panel",
        name="right upper door covers a narrow upper compartment",
    )
    ctx.expect_overlap(
        freezer_door,
        cabinet,
        axes="xz",
        min_overlap=0.60,
        elem_a="door_panel",
        elem_b="back_panel",
        name="lower side-hinged door covers the wide freezer compartment",
    )

    rest_left = ctx.part_element_world_aabb(upper_door_0, elem="door_panel")
    rest_right = ctx.part_element_world_aabb(upper_door_1, elem="door_panel")
    rest_freezer = ctx.part_element_world_aabb(freezer_door, elem="door_panel")
    with ctx.pose({upper_hinge_0: 1.10, upper_hinge_1: 1.10, freezer_hinge: 1.05}):
        open_left = ctx.part_element_world_aabb(upper_door_0, elem="door_panel")
        open_right = ctx.part_element_world_aabb(upper_door_1, elem="door_panel")
        open_freezer = ctx.part_element_world_aabb(freezer_door, elem="door_panel")

    ctx.check(
        "left upper hinge swings outward",
        rest_left is not None and open_left is not None and open_left[0][1] < rest_left[0][1] - 0.10,
        details=f"rest={rest_left}, open={open_left}",
    )
    ctx.check(
        "right upper hinge swings outward",
        rest_right is not None and open_right is not None and open_right[0][1] < rest_right[0][1] - 0.10,
        details=f"rest={rest_right}, open={open_right}",
    )
    ctx.check(
        "lower freezer hinge swings outward",
        rest_freezer is not None and open_freezer is not None and open_freezer[0][1] < rest_freezer[0][1] - 0.10,
        details=f"rest={rest_freezer}, open={open_freezer}",
    )

    return ctx.report()


object_model = build_object_model()

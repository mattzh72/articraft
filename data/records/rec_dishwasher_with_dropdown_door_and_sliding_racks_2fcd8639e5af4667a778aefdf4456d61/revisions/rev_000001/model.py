from __future__ import annotations

from math import pi

import cadquery as cq
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
    mesh_from_cadquery,
)


BODY_W = 0.60
BODY_D = 0.58
BODY_H = 0.84
FRONT_Y = -BODY_D / 2


def _open_tub_shell() -> cq.Workplane:
    """Thin-walled open-front dishwasher carcass and stainless tub."""

    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2))
    )
    # The cutter deliberately runs through the front face, leaving visible side,
    # bottom, top, and rear thin walls rather than a solid block.
    inner = (
        cq.Workplane("XY")
        .box(0.535, 0.565, 0.675)
        .translate((0.0, -0.030, 0.420))
    )
    toe_recess = (
        cq.Workplane("XY")
        .box(0.50, 0.055, 0.105)
        .translate((0.0, FRONT_Y - 0.003, 0.052))
    )
    return outer.cut(inner).cut(toe_recess)


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, rpy, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_wire_rack(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    material: Material,
    prefix: str,
    tine_rows: int,
) -> None:
    """Build a connected open wire basket around the part origin."""

    bar = 0.010
    z0 = -height / 2 + bar / 2
    ztop = height / 2 - bar / 2
    x_side = width / 2 - bar / 2
    y_front = -depth / 2 + bar / 2
    y_back = depth / 2 - bar / 2

    _add_box(part, f"{prefix}_front_rail", (width, bar, bar), (0.0, y_front, z0), material)
    _add_box(part, f"{prefix}_rear_rail", (width, bar, bar), (0.0, y_back, z0), material)
    _add_box(part, f"{prefix}_side_rail_0", (bar, depth, bar), (-x_side, 0.0, z0), material)
    _add_box(part, f"{prefix}_side_rail_1", (bar, depth, bar), (x_side, 0.0, z0), material)

    # Bottom grate members intersect the perimeter rail, so the part remains a
    # single connected basket while still reading as an open volume.
    for i, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        _add_box(part, f"{prefix}_bottom_bar_{i}", (bar * 0.85, depth, bar * 0.8), (x, 0.0, z0), material)
    for i, y in enumerate((-0.145, -0.055, 0.035, 0.125)):
        _add_box(part, f"{prefix}_cross_bar_{i}", (width, bar * 0.75, bar * 0.8), (0.0, y, z0 + 0.003), material)
    _add_box(part, f"{prefix}_tine_base", (0.360, bar * 0.75, bar * 0.8), (0.0, -0.080, z0 + 0.004), material)

    # Upper rim and side hoops make the rack look deep instead of flat.
    _add_box(part, f"{prefix}_top_front", (width, bar, bar), (0.0, y_front, ztop), material)
    _add_box(part, f"{prefix}_top_rear", (width, bar, bar), (0.0, y_back, ztop), material)
    _add_box(part, f"{prefix}_top_side_0", (bar, depth, bar), (-x_side, 0.0, ztop), material)
    _add_box(part, f"{prefix}_top_side_1", (bar, depth, bar), (x_side, 0.0, ztop), material)
    for i, y in enumerate((-0.16, -0.05, 0.06, 0.17)):
        _add_box(part, f"{prefix}_upright_0_{i}", (bar, bar, height), (-x_side, y, 0.0), material)
        _add_box(part, f"{prefix}_upright_1_{i}", (bar, bar, height), (x_side, y, 0.0), material)

    # Closely spaced tines are tied through the bottom grate.
    for i in range(tine_rows):
        x = -0.16 + i * (0.32 / max(1, tine_rows - 1))
        _add_box(part, f"{prefix}_tine_{i}", (bar * 0.75, bar * 0.75, height * 0.78), (x, -0.08, -0.007), material)

    # Slide runners are carried by the side hoops and ride beside the fixed tub guides.
    _add_box(part, f"{prefix}_runner_0", (bar, depth + 0.055, bar * 1.2), (-(width / 2 + bar / 2), 0.0, z0), material)
    _add_box(part, f"{prefix}_runner_1", (bar, depth + 0.055, bar * 1.2), ((width / 2 + bar / 2), 0.0, z0), material)


def _add_cutlery_tray(part, material: Material) -> None:
    bar = 0.006
    width = 0.50
    depth = 0.425
    height = 0.050
    z0 = -height / 2 + bar / 2
    ztop = height / 2 - bar / 2
    _add_box(part, "tray_front_lip", (width, bar, height), (0.0, -depth / 2 + bar / 2, 0.0), material)
    _add_box(part, "tray_rear_lip", (width, bar, height), (0.0, depth / 2 - bar / 2, 0.0), material)
    _add_box(part, "tray_side_lip_0", (bar, depth, height), (-width / 2 + bar / 2, 0.0, 0.0), material)
    _add_box(part, "tray_side_lip_1", (bar, depth, height), (width / 2 - bar / 2, 0.0, 0.0), material)
    for i, x in enumerate((-0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18)):
        _add_box(part, f"tray_channel_{i}", (bar, depth, bar), (x, 0.0, z0), material)
    for i, y in enumerate((-0.135, -0.045, 0.045, 0.135)):
        _add_box(part, f"tray_slot_bar_{i}", (width, bar, bar), (0.0, y, ztop - 0.015), material)
    _add_box(part, "tray_runner_0", (bar, depth + 0.050, bar), (-(width / 2 + bar / 2), 0.0, z0), material)
    _add_box(part, "tray_runner_1", (bar, depth + 0.050, bar), ((width / 2 + bar / 2), 0.0, z0), material)


def _add_spray_arm(part, material: Material) -> None:
    _add_cylinder(part, "rotating_hub", 0.034, 0.028, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), material)
    _add_box(part, "main_arm", (0.435, 0.040, 0.012), (0.0, 0.0, 0.004), material)
    _add_box(part, "spray_slot_0", (0.130, 0.012, 0.015), (-0.115, 0.022, 0.012), material)
    _add_box(part, "spray_slot_1", (0.130, 0.012, 0.015), (0.115, -0.022, 0.012), material)
    _add_cylinder(part, "end_cap_0", 0.022, 0.014, (-0.222, 0.0, 0.004), (pi / 2, 0.0, 0.0), material)
    _add_cylinder(part, "end_cap_1", 0.022, 0.014, (0.222, 0.0, 0.004), (pi / 2, 0.0, 0.0), material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_european_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    white = model.material("flat_white_panel", rgba=(0.90, 0.91, 0.88, 1.0))
    dark = model.material("concealed_dark_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    wire = model.material("coated_gray_wire", rgba=(0.78, 0.80, 0.78, 1.0))
    bluegray = model.material("blue_gray_plastic", rgba=(0.33, 0.43, 0.50, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_open_tub_shell(), "open_tub_shell", tolerance=0.0015),
        material=stainless,
        name="thin_wall_tub",
    )
    # European built-in side flanges and plinth/toe-kick, tied into the shell.
    _add_box(cabinet, "top_install_flange", (0.64, 0.035, 0.030), (0.0, FRONT_Y + 0.010, BODY_H - 0.015), stainless)
    _add_box(cabinet, "front_plinth", (0.62, 0.030, 0.075), (0.0, FRONT_Y - 0.015, 0.0375), stainless)
    _add_box(cabinet, "rear_feed_channel", (0.050, 0.018, 0.560), (0.0, 0.250, 0.405), stainless)
    _add_cylinder(cabinet, "lower_boss_stem", 0.012, 0.052, (0.0, -0.015, 0.106), (0.0, 0.0, 0.0), stainless)
    _add_cylinder(cabinet, "lower_fixed_boss", 0.026, 0.020, (0.0, -0.015, 0.142), (0.0, 0.0, 0.0), stainless)
    _add_box(cabinet, "upper_feed_tube", (0.048, 0.265, 0.018), (0.0, 0.120, 0.366), stainless)
    _add_cylinder(cabinet, "upper_fixed_boss", 0.024, 0.018, (0.0, -0.010, 0.366), (0.0, 0.0, 0.0), stainless)

    guide_specs = (
        ("lower", 0.198, 0.26425, 0.0235),
        ("upper", 0.422, 0.25975, 0.0245),
        ("tray", 0.633, 0.26800, 0.0240),
    )
    for level, z, xcenter, xsize in guide_specs:
        _add_box(cabinet, f"{level}_guide_0", (xsize, 0.450, 0.016), (-xcenter, -0.020, z), stainless)
        _add_box(cabinet, f"{level}_guide_1", (xsize, 0.450, 0.016), (xcenter, -0.020, z), stainless)

    _add_cylinder(cabinet, "hinge_knuckle_0", 0.012, 0.020, (-0.315, FRONT_Y - 0.028, 0.075), (0.0, pi / 2, 0.0), stainless)
    _add_cylinder(cabinet, "hinge_knuckle_1", 0.012, 0.020, (0.315, FRONT_Y - 0.028, 0.075), (0.0, pi / 2, 0.0), stainless)

    door = model.part("door")
    _add_box(door, "flat_front_panel", (0.595, 0.044, 0.720), (0.0, -0.018, 0.360), white)
    _add_box(door, "inner_stainless_liner", (0.535, 0.012, 0.615), (0.0, 0.010, 0.380), stainless)
    _add_box(door, "top_control_strip", (0.520, 0.038, 0.016), (0.0, -0.005, 0.728), dark)
    _add_box(door, "black_top_gasket", (0.570, 0.010, 0.018), (0.0, 0.015, 0.690), rubber)
    _add_box(door, "outer_panel_shadow_gap", (0.560, 0.004, 0.010), (0.0, -0.0420, 0.700), dark)
    _add_cylinder(door, "door_hinge_barrel", 0.014, 0.430, (0.0, -0.018, 0.014), (0.0, pi / 2, 0.0), stainless)
    _add_box(door, "detergent_recess", (0.175, 0.006, 0.105), (0.115, 0.019, 0.340), dark)
    _add_box(door, "rinse_aid_cap", (0.050, 0.009, 0.050), (-0.055, 0.020, 0.350), bluegray)

    door_hinge = model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.028, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.72),
    )

    button_xs = (-0.205, -0.105, 0.0, 0.105, 0.205)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        _add_box(button, "button_cap", (0.052, 0.022, 0.008), (0.0, 0.0, 0.0), dark)
        _add_box(button, "button_glyph", (0.030, 0.003, 0.0015), (0.0, -0.011, 0.0038), white)
        model.articulation(
            f"door_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.005, 0.740)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    lower_rack = model.part("lower_rack")
    _add_wire_rack(lower_rack, width=0.485, depth=0.420, height=0.135, material=wire, prefix="lower", tine_rows=6)
    lower_slide = model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.020, 0.260)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.365),
    )

    upper_rack = model.part("upper_rack")
    _add_wire_rack(upper_rack, width=0.475, depth=0.415, height=0.125, material=wire, prefix="upper", tine_rows=5)
    # Pivot sockets for the fold-down stemware shelf are integral to the upper rack.
    _add_cylinder(upper_rack, "stemware_pivot_0", 0.011, 0.026, (0.244, -0.170, 0.030), (-pi / 2, 0.0, 0.0), wire)
    _add_cylinder(upper_rack, "stemware_pivot_1", 0.011, 0.026, (0.244, 0.170, 0.030), (-pi / 2, 0.0, 0.0), wire)
    upper_slide = model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.020, 0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=38.0, velocity=0.33, lower=0.0, upper=0.335),
    )

    cutlery_tray = model.part("cutlery_tray")
    _add_cutlery_tray(cutlery_tray, wire)
    tray_slide = model.articulation(
        "cabinet_to_cutlery_tray",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cutlery_tray,
        origin=Origin(xyz=(0.0, -0.020, 0.655)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=0.28, lower=0.0, upper=0.300),
    )

    stemware_shelf = model.part("stemware_shelf")
    _add_box(stemware_shelf, "shelf_hinge_rail", (0.012, 0.300, 0.012), (0.0, 0.0, 0.0), wire)
    _add_box(stemware_shelf, "shelf_front_rail", (0.140, 0.012, 0.009), (-0.070, -0.145, -0.0095), wire)
    _add_box(stemware_shelf, "shelf_rear_rail", (0.140, 0.012, 0.009), (-0.070, 0.145, -0.0095), wire)
    for i, y in enumerate((-0.105, -0.052, 0.0, 0.052, 0.105)):
        _add_box(stemware_shelf, f"glass_slot_{i}", (0.145, 0.007, 0.007), (-0.072, y, -0.0095), wire)
    model.articulation(
        "upper_rack_to_stemware_shelf",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=stemware_shelf,
        origin=Origin(xyz=(0.244, 0.0, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    lower_arm = model.part("lower_spray_arm")
    _add_spray_arm(lower_arm, bluegray)
    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_arm,
        origin=Origin(xyz=(0.0, -0.015, 0.166)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    upper_arm = model.part("upper_spray_arm")
    _add_spray_arm(upper_arm, bluegray)
    model.articulation(
        "cabinet_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=upper_arm,
        origin=Origin(xyz=(0.0, -0.010, 0.389)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    detergent_cover = model.part("detergent_cover")
    _add_box(detergent_cover, "cover_plate", (0.145, 0.008, 0.078), (0.0, 0.004, 0.039), bluegray)
    _add_cylinder(detergent_cover, "cover_hinge_roll", 0.007, 0.145, (0.0, 0.006, 0.000), (0.0, pi / 2, 0.0), bluegray)
    model.articulation(
        "door_to_detergent_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_cover,
        origin=Origin(xyz=(0.115, 0.023, 0.300)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=1.25),
    )

    # Keep a few handles for tests without relying on string typos in explanations.
    model.meta["primary_joints"] = {
        "door": door_hinge.name,
        "lower_rack": lower_slide.name,
        "upper_rack": upper_slide.name,
        "cutlery_tray": tray_slide.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    cabinet = object_model.get_part("cabinet")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cutlery_tray = object_model.get_part("cutlery_tray")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    lower_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_slide = object_model.get_articulation("cabinet_to_upper_rack")
    tray_slide = object_model.get_articulation("cabinet_to_cutlery_tray")

    # The door is a bottom-hinged drop-down panel: opening lowers the top edge
    # and moves the slab forward of the built-in cabinet.
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.55}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door drops downward on bottom hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.45
        and open_aabb[0][1] < closed_aabb[0][1] - 0.45,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # The hollow tub contains the rack volumes at rest, with the three rack
    # stages stacked vertically like a European built-in dishwasher.
    ctx.expect_within(lower_rack, cabinet, axes="xz", margin=0.010, name="lower rack fits inside tub width and height")
    ctx.expect_within(upper_rack, cabinet, axes="xz", margin=0.010, name="upper rack fits inside tub width and height")
    ctx.expect_within(cutlery_tray, cabinet, axes="xz", margin=0.010, name="cutlery tray fits inside upper tub")
    ctx.expect_gap(upper_rack, lower_rack, axis="z", min_gap=0.030, name="upper rack clears lower rack")
    ctx.expect_gap(cutlery_tray, upper_rack, axis="z", min_gap=0.030, name="cutlery tray clears upper rack")

    # Rack and tray slides extend out through the open front.
    for part, joint, travel, label in (
        (lower_rack, lower_slide, 0.34, "lower rack"),
        (upper_rack, upper_slide, 0.31, "upper rack"),
        (cutlery_tray, tray_slide, 0.27, "cutlery tray"),
    ):
        rest = ctx.part_world_position(part)
        with ctx.pose({door_hinge: 1.55, joint: travel}):
            extended = ctx.part_world_position(part)
        ctx.check(
            f"{label} slides outward on guides",
            rest is not None and extended is not None and extended[1] < rest[1] - travel + 0.015,
            details=f"rest={rest}, extended={extended}",
        )

    # Exposed controls are five independent push buttons.
    for i in range(5):
        button_joint = object_model.get_articulation(f"door_to_button_{i}")
        ctx.check(
            f"button {i} is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and button_joint.motion_limits is not None
            and button_joint.motion_limits.upper == 0.006,
        )

    for joint_name in ("cabinet_to_lower_spray_arm", "cabinet_to_upper_spray_arm"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS)

    return ctx.report()


object_model = build_object_model()

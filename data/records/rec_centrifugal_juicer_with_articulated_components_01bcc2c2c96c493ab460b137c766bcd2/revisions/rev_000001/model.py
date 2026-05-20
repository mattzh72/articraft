from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _rounded_pad(width: float, depth: float, height: float, radius: float) -> MeshGeometry:
    return ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        height,
        center=True,
    )


def _annular_shell(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _build_cutter_basket() -> MeshGeometry:
    basket = _annular_shell(
        outer_profile=[
            (0.040, 0.006),
            (0.074, 0.024),
            (0.102, 0.064),
            (0.108, 0.116),
        ],
        inner_profile=[
            (0.032, 0.011),
            (0.062, 0.030),
            (0.088, 0.066),
            (0.093, 0.106),
        ],
        segments=72,
    )
    basket.merge(CylinderGeometry(radius=0.035, height=0.022, radial_segments=48).translate(0, 0, 0.016))
    basket.merge(CylinderGeometry(radius=0.066, height=0.006, radial_segments=64).translate(0, 0, 0.045))

    for index in range(12):
        angle = index * math.tau / 12.0
        blade = BoxGeometry((0.070, 0.0045, 0.004)).translate(0.052, 0.0, 0.050)
        blade.rotate_z(angle)
        basket.merge(blade)

    # Slightly raised cutter teeth on the disk make the rotating basket read as a juicer cutter.
    for index in range(6):
        angle = index * math.tau / 6.0 + math.radians(12)
        tooth = BoxGeometry((0.034, 0.006, 0.012)).translate(0.036, 0.0, 0.052)
        tooth.rotate_z(angle)
        basket.merge(tooth)

    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="polished_centrifugal_juicer")

    white_plastic = model.material("white_plastic", rgba=(0.93, 0.94, 0.93, 1.0))
    pearl_highlight = model.material("pearl_highlight", rgba=(0.98, 0.99, 0.98, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.06, 0.065, 0.075, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.028, 1.0))
    clear_poly = model.material("clear_polycarbonate", rgba=(0.78, 0.94, 1.0, 0.38))
    smoke_clear = model.material("smoked_clear_plastic", rgba=(0.48, 0.62, 0.70, 0.44))
    brushed_metal = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.32, 1.0))
    button_blue = model.material("blue_button", rgba=(0.10, 0.32, 0.74, 1.0))
    button_gray = model.material("silver_button", rgba=(0.72, 0.75, 0.78, 1.0))

    base = model.part("body")
    base.visual(
        _mesh(_rounded_pad(0.54, 0.36, 0.210, 0.085), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=white_plastic,
        name="body_shell",
    )
    base.visual(
        _mesh(_rounded_pad(0.44, 0.30, 0.060, 0.060), "top_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.213)),
        material=pearl_highlight,
        name="top_shoulder",
    )
    top_well = _annular_shell(
        outer_profile=[(0.143, -0.010), (0.148, 0.000), (0.143, 0.014)],
        inner_profile=[(0.111, -0.008), (0.113, 0.000), (0.111, 0.012)],
        segments=80,
    )
    base.visual(
        _mesh(top_well, "stainless_top_well"),
        origin=Origin(xyz=(0.0, 0.0, 0.233)),
        material=brushed_metal,
        name="stainless_well",
    )
    base.visual(
        Box((0.008, 0.250, 0.120)),
        origin=Origin(xyz=(0.270, 0.0, 0.120)),
        material=dark_panel,
        name="front_panel",
    )
    for idx, y in enumerate((-0.046, 0.046)):
        base.visual(
            Box((0.026, 0.072, 0.042)),
            origin=Origin(xyz=(0.283, y, 0.120)),
            material=dark_panel,
            name=f"button_bezel_{idx}",
        )
    base.visual(
        Box((0.018, 0.070, 0.035)),
        origin=Origin(xyz=(0.275, 0.0, 0.048)),
        material=black_rubber,
        name="front_foot",
    )
    base.visual(
        Box((0.030, 0.130, 0.025)),
        origin=Origin(xyz=(-0.225, 0.0, 0.238)),
        material=dark_panel,
        name="rear_hinge_plinth",
    )
    for y in (-0.076, 0.076):
        base.visual(
            Cylinder(radius=0.014, length=0.055),
            origin=Origin(xyz=(-0.250, y, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"rear_hinge_knuckle_{'neg' if y < 0 else 'pos'}",
        )
    base.visual(
        Cylinder(radius=0.022, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.227)),
        material=dark_metal,
        name="drive_coupler",
    )

    basket = model.part("cutter_basket")
    basket.visual(
        _mesh(_build_cutter_basket(), "cutter_basket"),
        material=brushed_metal,
        name="perforated_basket",
    )
    basket.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_metal,
        name="drive_hub",
    )

    lid = model.part("lid")
    chamber_shell = _annular_shell(
        outer_profile=[
            (0.132, 0.002),
            (0.150, 0.022),
            (0.150, 0.115),
            (0.137, 0.150),
        ],
        inner_profile=[
            (0.111, 0.009),
            (0.119, 0.026),
            (0.120, 0.110),
            (0.109, 0.139),
        ],
        segments=88,
    ).translate(0.250, 0.0, 0.012)
    lid.visual(
        _mesh(chamber_shell, "clear_chamber_shell"),
        material=clear_poly,
        name="clear_chamber",
    )
    # Four clear deck webs leave an actual rectangular chute opening while tying the chute to the chamber shell.
    lid.visual(Box((0.065, 0.104, 0.010)), origin=Origin(xyz=(0.342, 0.0, 0.146)), material=clear_poly, name="front_deck_web")
    lid.visual(Box((0.065, 0.104, 0.010)), origin=Origin(xyz=(0.158, 0.0, 0.146)), material=clear_poly, name="rear_deck_web")
    lid.visual(Box((0.124, 0.070, 0.010)), origin=Origin(xyz=(0.250, 0.086, 0.146)), material=clear_poly, name="side_deck_web_0")
    lid.visual(Box((0.124, 0.070, 0.010)), origin=Origin(xyz=(0.250, -0.086, 0.146)), material=clear_poly, name="side_deck_web_1")

    # Wide rectangular feed chute with four separate walls so the sliding pusher has real clearance.
    lid.visual(Box((0.012, 0.104, 0.180)), origin=Origin(xyz=(0.306, 0.0, 0.235)), material=smoke_clear, name="chute_front")
    lid.visual(Box((0.012, 0.104, 0.180)), origin=Origin(xyz=(0.194, 0.0, 0.235)), material=smoke_clear, name="chute_rear")
    lid.visual(Box((0.100, 0.012, 0.180)), origin=Origin(xyz=(0.250, 0.046, 0.235)), material=smoke_clear, name="chute_side_0")
    lid.visual(Box((0.100, 0.012, 0.180)), origin=Origin(xyz=(0.250, -0.046, 0.235)), material=smoke_clear, name="chute_side_1")

    lid.visual(
        Cylinder(radius=0.013, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="lid_hinge_barrel",
    )
    lid.visual(Box((0.115, 0.020, 0.030)), origin=Origin(xyz=(0.056, 0.032, 0.022)), material=dark_panel, name="hinge_arm_0")
    lid.visual(Box((0.115, 0.020, 0.030)), origin=Origin(xyz=(0.056, -0.032, 0.022)), material=dark_panel, name="hinge_arm_1")
    for y in (-0.047, 0.047):
        lid.visual(
            Box((0.030, 0.016, 0.028)),
            origin=Origin(xyz=(0.394, y, 0.066)),
            material=dark_panel,
            name=f"spout_pivot_bridge_{'neg' if y < 0 else 'pos'}",
        )
        lid.visual(
            Box((0.018, 0.014, 0.036)),
            origin=Origin(xyz=(0.405, y, 0.066)),
            material=dark_panel,
            name=f"spout_pivot_lug_{'neg' if y < 0 else 'pos'}",
        )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Box((0.084, 0.064, 0.158)),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=pearl_highlight,
        name="pusher_column",
    )
    pusher.visual(
        Box((0.140, 0.112, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=white_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_panel,
        name="pusher_grip",
    )

    spout = model.part("juice_spout")
    spout.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    spout.visual(Box((0.096, 0.050, 0.008)), origin=Origin(xyz=(0.054, 0.0, -0.005)), material=clear_poly, name="spout_floor")
    spout.visual(Box((0.090, 0.006, 0.018)), origin=Origin(xyz=(0.057, 0.028, 0.004)), material=clear_poly, name="spout_side_0")
    spout.visual(Box((0.090, 0.006, 0.018)), origin=Origin(xyz=(0.057, -0.028, 0.004)), material=clear_poly, name="spout_side_1")
    spout.visual(Box((0.008, 0.050, 0.012)), origin=Origin(xyz=(0.104, 0.0, 0.001)), material=clear_poly, name="spout_lip")

    for idx, (y, material) in enumerate(((-0.046, button_blue), (0.046, button_gray))):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.020, 0.050, 0.030)),
            origin=Origin(xyz=(0.010, 0.0, 0.0)),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.030, 0.018)),
            origin=Origin(xyz=(-0.010, 0.0, 0.0)),
            material=dark_metal,
            name="plunger",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(0.296, y, 0.120)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.12, lower=0.0, upper=0.014),
        )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=80.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.250, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=math.radians(68.0)),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.250, 0.0, 0.325)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.35, lower=0.0, upper=0.110),
    )
    model.articulation(
        "lid_to_spout",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=spout,
        origin=Origin(xyz=(0.410, 0.0, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.2, lower=0.0, upper=math.radians(70.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    spout = object_model.get_part("juice_spout")
    basket = object_model.get_part("cutter_basket")

    basket_joint = object_model.get_articulation("body_to_basket")
    lid_joint = object_model.get_articulation("body_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    spout_joint = object_model.get_articulation("lid_to_spout")

    ctx.check(
        "basket rotates continuously about the vertical drive axis",
        basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in basket_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={basket_joint.articulation_type}, axis={basket_joint.axis}",
    )

    for idx in range(2):
        button = object_model.get_part(f"button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_button_{idx}")
        ctx.allow_overlap(
            button,
            base,
            elem_a="plunger",
            elem_b=f"button_bezel_{idx}",
            reason="The button plunger is intentionally captured inside its front-panel guide sleeve.",
        )
        ctx.expect_within(
            button,
            base,
            axes="yz",
            inner_elem="plunger",
            outer_elem=f"button_bezel_{idx}",
            margin=0.001,
            name=f"button_{idx} plunger stays inside its guide sleeve",
        )
        ctx.expect_overlap(
            button,
            base,
            axes="x",
            elem_a="plunger",
            elem_b=f"button_bezel_{idx}",
            min_overlap=0.010,
            name=f"button_{idx} plunger remains inserted",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.014}):
            pushed = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} moves inward on a short plunger",
            rest is not None and pushed is not None and pushed[0] < rest[0] - 0.010,
            details=f"rest={rest}, pushed={pushed}",
        )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_column",
        margin=0.0,
        name="rectangular pusher is contained by the chute footprint",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_column",
        elem_b="chute_front",
        min_overlap=0.120,
        name="pusher remains engaged in the vertical chute at rest",
    )
    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.110}):
        pusher_down = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_column",
            elem_b="chute_front",
            min_overlap=0.060,
            name="pusher remains guided when pressed down",
        )
    ctx.check(
        "feed pusher slides downward along the chute guide",
        pusher_rest is not None and pusher_down is not None and pusher_down[2] < pusher_rest[2] - 0.090,
        details=f"rest={pusher_rest}, pushed={pusher_down}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: math.radians(68.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the clear lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.080,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    down_spout_aabb = ctx.part_world_aabb(spout)
    with ctx.pose({spout_joint: math.radians(70.0)}):
        up_spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "front juice spout flips upward on its pivot",
        down_spout_aabb is not None
        and up_spout_aabb is not None
        and up_spout_aabb[1][2] > down_spout_aabb[1][2] + 0.060,
        details=f"down={down_spout_aabb}, up={up_spout_aabb}",
    )

    ctx.expect_within(
        basket,
        lid,
        axes="xy",
        inner_elem="perforated_basket",
        outer_elem="clear_chamber",
        margin=0.004,
        name="cutter basket sits inside the clear chamber wall",
    )

    return ctx.report()


object_model = build_object_model()

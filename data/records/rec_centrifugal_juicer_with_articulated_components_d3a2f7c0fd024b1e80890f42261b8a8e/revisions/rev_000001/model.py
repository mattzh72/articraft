from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _base_shell():
    """Rounded-rectangle motor base with a soft countertop-appliance footprint."""
    profile = superellipse_profile(0.34, 0.26, exponent=3.2, segments=72)
    return ExtrudeGeometry.from_z0(profile, 0.11, cap=True, closed=True)


def _chamber_wall():
    """Clear annular juice chamber wall, open through the middle."""
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.106, 0.000),
            (0.124, 0.018),
            (0.126, 0.118),
            (0.116, 0.145),
        ],
        inner_profile=[
            (0.084, 0.000),
            (0.094, 0.018),
            (0.098, 0.118),
            (0.094, 0.145),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _lid_cover():
    """Smoked transparent annular lid with a central feed opening."""
    cover = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.111, 0.000),
            (0.123, 0.012),
            (0.116, 0.029),
        ],
        inner_profile=[
            (0.047, 0.000),
            (0.046, 0.014),
            (0.044, 0.029),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    return cover.translate(0.145, 0.0, 0.0)


def _feed_chute_wall():
    return LatheGeometry.from_shell_profiles(
        outer_profile=[(0.046, 0.026), (0.046, 0.150)],
        inner_profile=[(0.033, 0.026), (0.033, 0.150)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).translate(0.145, 0.0, 0.0)


def _basket_shell():
    """Slightly conical stainless filter basket that clears the chamber wall."""
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.054, 0.000),
            (0.076, 0.070),
            (0.088, 0.106),
        ],
        inner_profile=[
            (0.044, 0.000),
            (0.066, 0.070),
            (0.078, 0.106),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_countertop_juicer")

    white_plastic = model.material("white_plastic", rgba=(0.88, 0.90, 0.86, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.50, 0.64, 0.70, 0.38))
    clear_plastic = model.material("clear_plastic", rgba=(0.72, 0.90, 0.95, 0.32))
    stainless = model.material("stainless", rgba=(0.72, 0.75, 0.76, 1.0))
    juice_orange = model.material("juice_orange", rgba=(1.0, 0.55, 0.16, 1.0))
    indicator_grey = model.material("indicator_grey", rgba=(0.30, 0.32, 0.34, 1.0))

    base = model.part("base")
    base.visual(_mesh(_base_shell(), "rounded_base"), material=white_plastic, name="rounded_base")
    base.visual(
        Box((0.26, 0.030, 0.010)),
        origin=Origin(xyz=(0.010, -0.132, 0.088)),
        material=indicator_grey,
        name="side_speed_panel",
    )
    base.visual(
        _mesh(_chamber_wall(), "clear_chamber_wall"),
        origin=Origin(xyz=(0.020, 0.0, 0.110)),
        material=clear_plastic,
        name="chamber_wall",
    )
    base.visual(
        Cylinder(radius=0.104, length=0.012),
        origin=Origin(xyz=(0.020, 0.0, 0.115)),
        material=dark_plastic,
        name="lower_chamber_gasket",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.020, 0.0, 0.120)),
        material=dark_plastic,
        name="drive_spindle",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.158, 0.0, 0.205), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="spout_socket",
    )
    base.visual(
        Box((0.044, 0.018, 0.052)),
        origin=Origin(xyz=(0.186, 0.034, 0.203)),
        material=white_plastic,
        name="spout_lug_0",
    )
    base.visual(
        Box((0.044, 0.018, 0.052)),
        origin=Origin(xyz=(0.186, -0.034, 0.203)),
        material=white_plastic,
        name="spout_lug_1",
    )
    base.visual(
        Box((0.070, 0.092, 0.016)),
        origin=Origin(xyz=(0.170, 0.0, 0.180)),
        material=white_plastic,
        name="spout_saddle",
    )
    base.visual(
        Box((0.036, 0.030, 0.090)),
        origin=Origin(xyz=(-0.118, 0.066, 0.210)),
        material=white_plastic,
        name="rear_hinge_ear_0",
    )
    base.visual(
        Box((0.036, 0.030, 0.090)),
        origin=Origin(xyz=(-0.118, -0.066, 0.210)),
        material=white_plastic,
        name="rear_hinge_ear_1",
    )
    base.visual(
        Box((0.040, 0.170, 0.026)),
        origin=Origin(xyz=(-0.106, 0.0, 0.174)),
        material=white_plastic,
        name="rear_hinge_bridge",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(-0.042, -0.136, 0.070), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="speed_pivot_boss",
    )

    basket = model.part("basket")
    basket.visual(_mesh(_basket_shell(), "filter_basket_shell"), material=stainless, name="basket_wall")
    basket.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_plastic,
        name="basket_hub",
    )
    for rib_index in range(6):
        basket.visual(
            Box((0.078, 0.006, 0.007)),
            origin=Origin(xyz=(0.052, 0.0, 0.038), rpy=(0.0, 0.0, rib_index * pi / 3.0)),
            material=stainless,
            name=f"basket_rib_{rib_index}",
        )

    lid = model.part("lid")
    lid.visual(_mesh(_lid_cover(), "annular_lid"), material=smoked_clear, name="lid_cover")
    lid.visual(_mesh(_feed_chute_wall(), "feed_chute_wall"), material=smoked_clear, name="chute_wall")
    lid.visual(
        Box((0.092, 0.140, 0.016)),
        origin=Origin(xyz=(0.044, 0.0, 0.008)),
        material=smoked_clear,
        name="rear_lid_leaf",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hinge_barrel",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.026, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=white_plastic,
        name="plunger",
    )
    pusher.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=white_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.060, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_plastic,
        name="thumb_pad",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="spout_pivot_barrel",
    )
    spout.visual(
        Cylinder(radius=0.017, length=0.096),
        origin=Origin(xyz=(0.062, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=juice_orange,
        name="spout_tube",
    )
    spout.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="spout_opening",
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="lever_pivot_cap",
    )
    speed_lever.visual(
        Box((0.020, 0.014, 0.078)),
        origin=Origin(xyz=(0.0, -0.018, -0.045)),
        material=juice_orange,
        name="lever_paddle",
    )

    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.020, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.120, 0.0, 0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.145, 0.0, 0.150)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.080),
    )
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.190, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "speed_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.042, -0.150, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.8, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    spout = object_model.get_part("spout")

    basket_joint = object_model.get_articulation("basket_spin")
    lid_joint = object_model.get_articulation("lid_hinge")
    pusher_joint = object_model.get_articulation("pusher_slide")
    spout_joint = object_model.get_articulation("spout_pivot")
    lever_joint = object_model.get_articulation("speed_lever_pivot")

    ctx.check(
        "primary juicer mechanisms are articulated",
        basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and lid_joint.articulation_type == ArticulationType.REVOLUTE
        and pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and spout_joint.articulation_type == ArticulationType.REVOLUTE
        and lever_joint.articulation_type == ArticulationType.REVOLUTE,
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="plunger",
        outer_elem="chute_wall",
        margin=0.003,
        name="feed pusher is centered in the chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="plunger",
        elem_b="chute_wall",
        min_overlap=0.100,
        name="pusher remains engaged in the chute at rest",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.080}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="plunger",
            outer_elem="chute_wall",
            margin=0.003,
            name="pressed pusher stays centered in the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="plunger",
            elem_b="chute_wall",
            min_overlap=0.035,
            name="pressed pusher retains insertion",
        )
        pressed_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides downward when pressed",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.060,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 0.90}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge lifts the feed chute upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.035,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_spout_aabb = ctx.part_world_aabb(spout)
    with ctx.pose({spout_joint: 0.75}):
        raised_spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "anti-drip spout pivots upward",
        rest_spout_aabb is not None
        and raised_spout_aabb is not None
        and raised_spout_aabb[1][2] > rest_spout_aabb[1][2] + 0.030,
        details=f"rest={rest_spout_aabb}, raised={raised_spout_aabb}",
    )

    ctx.expect_overlap(
        "basket",
        base,
        axes="xy",
        elem_a="basket_wall",
        elem_b="chamber_wall",
        min_overlap=0.050,
        name="basket is visibly nested inside the chamber footprint",
    )

    return ctx.report()


object_model = build_object_model()

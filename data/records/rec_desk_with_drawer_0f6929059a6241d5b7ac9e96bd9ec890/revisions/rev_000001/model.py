from __future__ import annotations

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
    model = ArticulatedObject(name="scandinavian_desk")

    oak = model.material("pale_oak", rgba=(0.78, 0.57, 0.35, 1.0))
    oak_light = model.material("pale_oak_light", rgba=(0.86, 0.68, 0.45, 1.0))
    oak_dark = model.material("oak_end_grain", rgba=(0.58, 0.38, 0.20, 1.0))
    painted = model.material("warm_white_lacquer", rgba=(0.86, 0.85, 0.80, 1.0))
    steel = model.material("satin_black_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_steel = model.material("brushed_drawer_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    shadow = model.material("drawer_shadow", rgba=(0.08, 0.075, 0.065, 1.0))

    # World frame: X = left/right along the desk, Y = depth (front is -Y),
    # Z = height from floor.  The top is a realistic full-size writing desk.
    desktop = model.part("desktop")
    desktop.visual(
        Box((1.40, 0.70, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
        material=oak,
        name="solid_wood_top",
    )
    # Slightly darker edge bands and subtle raised grain streaks make the slab
    # read as a solid pale-oak top rather than a plain rectangular proxy.
    desktop.visual(
        Box((1.405, 0.022, 0.047)),
        origin=Origin(xyz=(0.0, -0.361, 0.745)),
        material=oak_dark,
        name="front_end_grain",
    )
    desktop.visual(
        Box((1.405, 0.022, 0.047)),
        origin=Origin(xyz=(0.0, 0.361, 0.745)),
        material=oak_dark,
        name="rear_end_grain",
    )
    desktop.visual(
        Box((0.022, 0.70, 0.047)),
        origin=Origin(xyz=(-0.711, 0.0, 0.745)),
        material=oak_dark,
        name="end_grain_0",
    )
    desktop.visual(
        Box((0.022, 0.70, 0.047)),
        origin=Origin(xyz=(0.711, 0.0, 0.745)),
        material=oak_dark,
        name="end_grain_1",
    )
    for index, (y, width) in enumerate(
        [(-0.24, 0.010), (-0.13, 0.006), (0.02, 0.008), (0.16, 0.007), (0.28, 0.005)]
    ):
        desktop.visual(
            Box((1.22, width, 0.0012)),
            origin=Origin(xyz=(0.0, y, 0.7681)),
            material=oak_light if index % 2 == 0 else oak_dark,
            name=f"wood_grain_{index}",
        )

    def _make_leg(name: str, x: float, y: float) -> None:
        leg = model.part(name)
        leg.visual(
            Cylinder(radius=0.045, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=steel,
            name="leveling_foot",
        )
        leg.visual(
            Cylinder(radius=0.014, length=0.6965),
            origin=Origin(xyz=(0.0, 0.0, 0.36025)),
            material=steel,
            name="slim_steel_tube",
        )
        leg.visual(
            Box((0.090, 0.070, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.7155)),
            material=steel,
            name="top_mount_plate",
        )
        model.articulation(
            f"desktop_to_{name}",
            ArticulationType.FIXED,
            parent=desktop,
            child=leg,
            origin=Origin(xyz=(x, y, 0.0)),
        )

    _make_leg("leg_0", -0.59, -0.27)
    _make_leg("leg_1", -0.59, 0.27)
    _make_leg("leg_2", 0.59, -0.27)
    _make_leg("leg_3", 0.59, 0.27)

    # Floating side drawer unit: it hangs from the underside of the desk at the
    # user's right side and deliberately stops well above the floor.
    drawer_frame = model.part("drawer_frame")
    drawer_frame.visual(
        Box((0.380, 0.520, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.301)),
        material=painted,
        name="top_panel",
    )
    drawer_frame.visual(
        Box((0.380, 0.520, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=painted,
        name="bottom_panel",
    )
    drawer_frame.visual(
        Box((0.018, 0.520, 0.310)),
        origin=Origin(xyz=(-0.181, 0.0, 0.155)),
        material=painted,
        name="side_panel_0",
    )
    drawer_frame.visual(
        Box((0.018, 0.520, 0.310)),
        origin=Origin(xyz=(0.181, 0.0, 0.155)),
        material=painted,
        name="side_panel_1",
    )
    drawer_frame.visual(
        Box((0.380, 0.018, 0.310)),
        origin=Origin(xyz=(0.0, 0.251, 0.155)),
        material=painted,
        name="back_panel",
    )
    drawer_frame.visual(
        Box((0.344, 0.500, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=painted,
        name="center_divider",
    )
    drawer_frame.visual(
        Box((0.038, 0.120, 0.0125)),
        origin=Origin(xyz=(-0.115, -0.175, 0.31625)),
        material=steel,
        name="hanger_0",
    )
    drawer_frame.visual(
        Box((0.038, 0.120, 0.0125)),
        origin=Origin(xyz=(0.115, -0.175, 0.31625)),
        material=steel,
        name="hanger_1",
    )
    drawer_frame.visual(
        Box((0.038, 0.120, 0.0125)),
        origin=Origin(xyz=(-0.115, 0.175, 0.31625)),
        material=steel,
        name="hanger_2",
    )
    drawer_frame.visual(
        Box((0.038, 0.120, 0.0125)),
        origin=Origin(xyz=(0.115, 0.175, 0.31625)),
        material=steel,
        name="hanger_3",
    )

    lower_z = 0.0825
    upper_z = 0.2275
    for drawer_index, z_center in enumerate((upper_z, lower_z)):
        for side_index, x_center in enumerate((-0.166, 0.166)):
            drawer_frame.visual(
                Box((0.012, 0.410, 0.014)),
                origin=Origin(xyz=(x_center, -0.010, z_center)),
                material=rail_steel,
                name=f"fixed_rail_{drawer_index}_{side_index}",
            )

    model.articulation(
        "desktop_to_drawer_frame",
        ArticulationType.FIXED,
        parent=desktop,
        child=drawer_frame,
        origin=Origin(xyz=(0.36, 0.0, 0.400)),
    )

    def _make_drawer(name: str, z_center: float) -> None:
        drawer = model.part(name)
        drawer.visual(
            Box((0.300, 0.414, 0.088)),
            origin=Origin(xyz=(0.0, 0.202, -0.003)),
            material=shadow,
            name="drawer_box",
        )
        drawer.visual(
            Box((0.362, 0.024, 0.126)),
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
            material=oak,
            name="wood_front",
        )
        for side_index, x_center in enumerate((-0.154, 0.154)):
            drawer.visual(
                Box((0.010, 0.380, 0.010)),
                origin=Origin(xyz=(x_center, 0.210, 0.0)),
                material=rail_steel,
                name=f"moving_rail_{side_index}",
            )
        drawer.visual(
            Box((0.160, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, -0.042, 0.0)),
            material=steel,
            name="drawer_pull",
        )
        drawer.visual(
            Box((0.018, 0.012, 0.026)),
            origin=Origin(xyz=(-0.070, -0.030, 0.0)),
            material=steel,
            name="pull_standoff_0",
        )
        drawer.visual(
            Box((0.018, 0.012, 0.026)),
            origin=Origin(xyz=(0.070, -0.030, 0.0)),
            material=steel,
            name="pull_standoff_1",
        )
        model.articulation(
            f"drawer_frame_to_{name}",
            ArticulationType.PRISMATIC,
            parent=drawer_frame,
            child=drawer,
            origin=Origin(xyz=(0.0, -0.260, z_center)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.280),
        )

    _make_drawer("upper_drawer", upper_z)
    _make_drawer("lower_drawer", lower_z)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desktop = object_model.get_part("desktop")
    drawer_frame = object_model.get_part("drawer_frame")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_slide = object_model.get_articulation("drawer_frame_to_upper_drawer")
    lower_slide = object_model.get_articulation("drawer_frame_to_lower_drawer")

    ctx.check(
        "two independent drawer slides",
        upper_slide.articulation_type == ArticulationType.PRISMATIC
        and lower_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"upper={upper_slide.articulation_type}, lower={lower_slide.articulation_type}",
    )
    ctx.expect_gap(
        desktop,
        drawer_frame,
        axis="z",
        positive_elem="solid_wood_top",
        negative_elem="hanger_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="drawer unit hangs from desktop underside",
    )
    ctx.expect_gap(
        desktop,
        object_model.get_part("leg_0"),
        axis="z",
        positive_elem="solid_wood_top",
        negative_elem="top_mount_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="leg top plate seats against desktop",
    )
    ctx.expect_within(
        upper_drawer,
        drawer_frame,
        axes="x",
        inner_elem="drawer_box",
        margin=0.020,
        name="upper drawer centered between side panels",
    )
    ctx.expect_within(
        lower_drawer,
        drawer_frame,
        axes="x",
        inner_elem="drawer_box",
        margin=0.020,
        name="lower drawer centered between side panels",
    )
    ctx.expect_overlap(
        upper_drawer,
        drawer_frame,
        axes="y",
        elem_a="drawer_box",
        elem_b="top_panel",
        min_overlap=0.25,
        name="upper drawer box remains inserted when closed",
    )
    ctx.expect_overlap(
        lower_drawer,
        drawer_frame,
        axes="y",
        elem_a="drawer_box",
        elem_b="bottom_panel",
        min_overlap=0.25,
        name="lower drawer box remains inserted when closed",
    )

    closed_upper_pos = ctx.part_world_position(upper_drawer)
    with ctx.pose({upper_slide: 0.280, lower_slide: 0.280}):
        extended_upper_pos = ctx.part_world_position(upper_drawer)
        ctx.expect_overlap(
            upper_drawer,
            drawer_frame,
            axes="y",
            elem_a="drawer_box",
            elem_b="top_panel",
            min_overlap=0.09,
            name="upper drawer retains rail insertion when extended",
        )
        ctx.expect_overlap(
            lower_drawer,
            drawer_frame,
            axes="y",
            elem_a="drawer_box",
            elem_b="bottom_panel",
            min_overlap=0.09,
            name="lower drawer retains rail insertion when extended",
        )
    ctx.check(
        "drawers pull toward the front",
        closed_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[1] < closed_upper_pos[1] - 0.20,
        details=f"closed={closed_upper_pos}, extended={extended_upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()

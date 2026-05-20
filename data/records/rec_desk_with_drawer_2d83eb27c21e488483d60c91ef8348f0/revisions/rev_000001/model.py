from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trestle_desk")

    oak = model.material("oak", rgba=(0.70, 0.56, 0.38, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))

    top_length = 1.55
    top_depth = 0.75
    top_thickness = 0.036
    desk_height = 0.75
    trestle_height = desk_height - top_thickness

    top = model.part("top")
    top.visual(
        Box((top_length, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_thickness / 2.0)),
        material=oak,
        name="top_panel",
    )
    top.visual(
        Box((0.44, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, -0.26, -0.009)),
        material=graphite,
        name="front_mount_batten",
    )
    top.visual(
        Box((0.44, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, 0.26, -0.009)),
        material=graphite,
        name="rear_mount_batten",
    )
    top.inertial = Inertial.from_geometry(
        Box((top_length, top_depth, top_thickness + 0.018)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    def add_trestle(name: str) -> None:
        trestle = model.part(name)

        cap_width = 0.34
        cap_depth = 0.10
        cap_height = 0.060
        leg_length = 0.71
        leg_width = 0.055
        leg_depth = 0.095
        leg_angle = math.radians(16.0)
        lower_tie_length = 0.46
        lower_tie_height = 0.055
        lower_tie_z = -(trestle_height - 0.22)

        trestle.visual(
            Box((cap_width, cap_depth, cap_height)),
            origin=Origin(xyz=(0.0, 0.0, -cap_height / 2.0)),
            material=charcoal,
            name="cap_beam",
        )
        trestle.visual(
            Box((leg_width, leg_depth, leg_length)),
            origin=Origin(
                xyz=(-0.20, 0.0, -0.382),
                rpy=(0.0, leg_angle, 0.0),
            ),
            material=charcoal,
            name="left_leg",
        )
        trestle.visual(
            Box((leg_width, leg_depth, leg_length)),
            origin=Origin(
                xyz=(0.20, 0.0, -0.382),
                rpy=(0.0, -leg_angle, 0.0),
            ),
            material=charcoal,
            name="right_leg",
        )
        trestle.visual(
            Box((lower_tie_length, cap_depth, lower_tie_height)),
            origin=Origin(xyz=(0.0, 0.0, lower_tie_z)),
            material=graphite,
            name="lower_tie",
        )
        trestle.visual(
            Box((0.10, 0.11, 0.018)),
            origin=Origin(xyz=(-0.30, 0.0, -trestle_height + 0.009)),
            material=black,
            name="left_foot",
        )
        trestle.visual(
            Box((0.10, 0.11, 0.018)),
            origin=Origin(xyz=(0.30, 0.0, -trestle_height + 0.009)),
            material=black,
            name="right_foot",
        )
        trestle.inertial = Inertial.from_geometry(
            Box((0.62, 0.14, trestle_height)),
            mass=5.0,
            origin=Origin(xyz=(0.0, 0.0, -trestle_height / 2.0)),
        )

    add_trestle("left_trestle")
    add_trestle("right_trestle")

    drawer_unit = model.part("drawer_unit")
    drawer_unit.visual(
        Box((0.055, 0.31, 0.020)),
        origin=Origin(xyz=(-0.232, 0.0, -0.010)),
        material=graphite,
        name="left_mount_rail",
    )
    drawer_unit.visual(
        Box((0.055, 0.31, 0.020)),
        origin=Origin(xyz=(0.232, 0.0, -0.010)),
        material=graphite,
        name="right_mount_rail",
    )
    drawer_unit.visual(
        Box((0.018, 0.30, 0.130)),
        origin=Origin(xyz=(-0.251, 0.0, -0.085)),
        material=charcoal,
        name="left_side",
    )
    drawer_unit.visual(
        Box((0.018, 0.30, 0.130)),
        origin=Origin(xyz=(0.251, 0.0, -0.085)),
        material=charcoal,
        name="right_side",
    )
    drawer_unit.visual(
        Box((0.486, 0.018, 0.110)),
        origin=Origin(xyz=(0.0, 0.141, -0.077)),
        material=charcoal,
        name="back_panel",
    )
    drawer_unit.visual(
        Box((0.430, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.124, -0.026)),
        material=black,
        name="rear_slide_block",
    )
    drawer_unit.visual(
        Box((0.022, 0.240, 0.018)),
        origin=Origin(xyz=(-0.231, -0.003, -0.118)),
        material=black,
        name="left_slide_rail",
    )
    drawer_unit.visual(
        Box((0.022, 0.240, 0.018)),
        origin=Origin(xyz=(0.231, -0.003, -0.118)),
        material=black,
        name="right_slide_rail",
    )
    drawer_unit.inertial = Inertial.from_geometry(
        Box((0.52, 0.32, 0.15)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.012, 0.260, 0.072)),
        origin=Origin(xyz=(-0.214, -0.130, -0.102)),
        material=charcoal,
        name="left_wall",
    )
    drawer.visual(
        Box((0.012, 0.260, 0.072)),
        origin=Origin(xyz=(0.214, -0.130, -0.102)),
        material=charcoal,
        name="right_wall",
    )
    drawer.visual(
        Box((0.416, 0.012, 0.072)),
        origin=Origin(xyz=(0.0, -0.006, -0.102)),
        material=charcoal,
        name="back_wall",
    )
    drawer.visual(
        Box((0.416, 0.248, 0.010)),
        origin=Origin(xyz=(0.0, -0.130, -0.133)),
        material=graphite,
        name="bottom_panel",
    )
    drawer.visual(
        Box((0.470, 0.018, 0.096)),
        origin=Origin(xyz=(0.0, -0.269, -0.098)),
        material=oak,
        name="front_panel",
    )
    drawer.visual(
        Box((0.012, 0.022, 0.012)),
        origin=Origin(xyz=(-0.055, -0.289, -0.098)),
        material=black,
        name="left_pull_post",
    )
    drawer.visual(
        Box((0.012, 0.022, 0.012)),
        origin=Origin(xyz=(0.055, -0.289, -0.098)),
        material=black,
        name="right_pull_post",
    )
    drawer.visual(
        Box((0.160, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.301, -0.098)),
        material=black,
        name="pull_bar",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.47, 0.30, 0.10)),
        mass=2.6,
        origin=Origin(xyz=(0.0, -0.150, -0.100)),
    )

    model.articulation(
        "top_to_left_trestle",
        ArticulationType.FIXED,
        parent=top,
        child="left_trestle",
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
    )
    model.articulation(
        "top_to_right_trestle",
        ArticulationType.FIXED,
        parent=top,
        child="right_trestle",
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
    )
    model.articulation(
        "top_to_drawer_unit",
        ArticulationType.FIXED,
        parent=top,
        child=drawer_unit,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "drawer_unit_to_drawer",
        ArticulationType.PRISMATIC,
        parent=drawer_unit,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    top = object_model.get_part("top")
    left_trestle = object_model.get_part("left_trestle")
    right_trestle = object_model.get_part("right_trestle")
    drawer_unit = object_model.get_part("drawer_unit")
    drawer = object_model.get_part("drawer")
    drawer_slide = object_model.get_articulation("drawer_unit_to_drawer")

    top_panel = top.get_visual("top_panel")
    left_cap = left_trestle.get_visual("cap_beam")
    right_cap = right_trestle.get_visual("cap_beam")

    ctx.expect_gap(
        top,
        left_trestle,
        axis="z",
        positive_elem=top_panel,
        negative_elem=left_cap,
        max_gap=0.001,
        max_penetration=0.0,
        name="left trestle cap seats against the desktop underside",
    )
    ctx.expect_gap(
        top,
        right_trestle,
        axis="z",
        positive_elem=top_panel,
        negative_elem=right_cap,
        max_gap=0.001,
        max_penetration=0.0,
        name="right trestle cap seats against the desktop underside",
    )
    ctx.expect_gap(
        top,
        drawer_unit,
        axis="z",
        positive_elem=top_panel,
        max_gap=0.001,
        max_penetration=0.0,
        name="drawer unit hangs directly from the desktop underside",
    )

    ctx.expect_within(
        drawer,
        drawer_unit,
        axes="xz",
        margin=0.015,
        name="drawer stays centered within the suspended housing",
    )
    ctx.expect_overlap(
        drawer,
        drawer_unit,
        axes="y",
        min_overlap=0.12,
        name="closed drawer remains substantially inserted in the housing",
    )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_within(
            drawer,
            drawer_unit,
            axes="xz",
            margin=0.015,
            name="extended drawer stays centered within the suspended housing",
        )
        ctx.expect_overlap(
            drawer,
            drawer_unit,
            axes="y",
            min_overlap=0.07,
            name="extended drawer retains insertion in the housing",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer slides forward from the desk",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] < rest_pos[1] - 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

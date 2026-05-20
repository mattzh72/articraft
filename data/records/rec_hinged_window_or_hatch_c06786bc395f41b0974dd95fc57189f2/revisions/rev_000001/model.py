from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet_hatch")

    outer_width = 0.64
    outer_height = 0.84
    frame_depth = 0.07
    frame_border = 0.07
    opening_width = outer_width - 2.0 * frame_border
    opening_height = outer_height - 2.0 * frame_border

    panel_width = opening_width - 0.008
    panel_height = opening_height - 0.008
    panel_skin_thickness = 0.022
    panel_front_inset = 0.002

    cabinet_grey = model.material("cabinet_grey", rgba=(0.72, 0.74, 0.76, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.84, 0.85, 0.86, 1.0))
    support_steel = model.material("support_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.24, 0.27, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((outer_width, frame_depth, frame_border)),
        origin=Origin(xyz=(0.0, frame_depth * 0.5, -frame_border * 0.5)),
        material=cabinet_grey,
        name="frame_bottom",
    )
    frame.visual(
        Box((outer_width, frame_depth, frame_border)),
        origin=Origin(xyz=(0.0, frame_depth * 0.5, opening_height + frame_border * 0.5)),
        material=cabinet_grey,
        name="frame_top",
    )
    frame.visual(
        Box((frame_border, frame_depth, opening_height)),
        origin=Origin(
            xyz=(-(opening_width * 0.5 + frame_border * 0.5), frame_depth * 0.5, opening_height * 0.5)
        ),
        material=cabinet_grey,
        name="frame_left",
    )
    frame.visual(
        Box((frame_border, frame_depth, opening_height)),
        origin=Origin(
            xyz=((opening_width * 0.5 + frame_border * 0.5), frame_depth * 0.5, opening_height * 0.5)
        ),
        material=cabinet_grey,
        name="frame_right",
    )
    frame.visual(
        Box((0.022, 0.016, 0.060)),
        origin=Origin(xyz=(-0.239, 0.046, 0.520)),
        material=dark_hardware,
        name="left_stay_bracket",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(-0.240, 0.046, 0.520), rpy=(0.0, pi * 0.5, 0.0)),
        material=support_steel,
        name="left_stay_pivot_pin",
    )
    frame.visual(
        Box((0.022, 0.016, 0.060)),
        origin=Origin(xyz=(0.239, 0.046, 0.520)),
        material=dark_hardware,
        name="right_stay_bracket",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.240, 0.046, 0.520), rpy=(0.0, pi * 0.5, 0.0)),
        material=support_steel,
        name="right_stay_pivot_pin",
    )
    for hinge_x, hinge_len in ((-0.185, 0.108), (0.0, 0.136), (0.185, 0.108)):
        frame.visual(
            Cylinder(radius=0.009, length=hinge_len),
            origin=Origin(xyz=(hinge_x, 0.016, -0.009), rpy=(0.0, pi * 0.5, 0.0)),
            material=dark_hardware,
            name=f"hinge_knuckle_{str(hinge_x).replace('-', 'm').replace('.', '_')}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, frame_depth * 0.5, opening_height * 0.5)),
    )

    panel = model.part("panel")
    panel.visual(
        Box((panel_width, panel_skin_thickness, panel_height)),
        origin=Origin(
            xyz=(0.0, panel_front_inset + panel_skin_thickness * 0.5, panel_height * 0.5)
        ),
        material=panel_grey,
        name="panel_skin",
    )
    panel.visual(
        Box((0.440, 0.010, 0.580)),
        origin=Origin(xyz=(0.0, panel_front_inset + panel_skin_thickness + 0.005, 0.344)),
        material=cabinet_grey,
        name="panel_stiffener",
    )
    panel.visual(
        Box((0.020, 0.010, 0.580)),
        origin=Origin(xyz=(-0.226, panel_front_inset + panel_skin_thickness + 0.005, 0.344)),
        material=cabinet_grey,
        name="left_back_rib",
    )
    panel.visual(
        Box((0.020, 0.010, 0.580)),
        origin=Origin(xyz=(0.226, panel_front_inset + panel_skin_thickness + 0.005, 0.344)),
        material=cabinet_grey,
        name="right_back_rib",
    )
    panel.visual(
        Box((0.014, 0.014, 0.070)),
        origin=Origin(xyz=(-0.235, 0.041, 0.290)),
        material=dark_hardware,
        name="left_lug",
    )
    panel.visual(
        Box((0.014, 0.014, 0.070)),
        origin=Origin(xyz=(0.235, 0.041, 0.290)),
        material=dark_hardware,
        name="right_lug",
    )
    panel.visual(
        Box((0.180, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.007, panel_height - 0.050)),
        material=support_steel,
        name="pull_tab",
    )
    panel.visual(
        Box((panel_width * 0.84, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, panel_front_inset + panel_skin_thickness + 0.002, 0.020)),
        material=dark_hardware,
        name="hinge_leaf",
    )
    panel.inertial = Inertial.from_geometry(
        Box((panel_width, 0.034, panel_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.017, panel_height * 0.5)),
    )

    def add_stay_bar(name: str, side_sign: float) -> tuple[object, object]:
        stay = model.part(name)
        x_shift = -side_sign * 0.013
        stay.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(x_shift, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
            material=support_steel,
            name="stay_pivot",
        )
        stay.visual(
            Box((0.014, 0.008, 0.240)),
            origin=Origin(xyz=(x_shift, -0.003, -0.120)),
            material=support_steel,
            name="stay_bar",
        )
        stay.visual(
            Box((0.016, 0.010, 0.022)),
            origin=Origin(xyz=(x_shift, -0.005, -0.249)),
            material=dark_hardware,
            name="stay_tip",
        )
        stay.inertial = Inertial.from_geometry(
            Box((0.016, 0.010, 0.262)),
            mass=0.18,
            origin=Origin(xyz=(x_shift, -0.003, -0.124)),
        )

        joint = model.articulation(
            f"frame_to_{name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=stay,
            origin=Origin(xyz=(side_sign * 0.233, 0.046, 0.520)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=0.0,
                upper=0.85,
            ),
        )
        return stay, joint

    left_stay, left_joint = add_stay_bar("left_stay", -1.0)
    right_stay, right_joint = add_stay_bar("right_stay", 1.0)

    panel_joint = model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, panel_front_inset, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.25,
        ),
    )

    model.meta["named_parts"] = {
        "frame": frame.name,
        "panel": panel.name,
        "left_stay": left_stay.name,
        "right_stay": right_stay.name,
        "panel_joint": panel_joint.name,
        "left_joint": left_joint.name,
        "right_joint": right_joint.name,
    }
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

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")

    panel_joint = object_model.get_articulation("frame_to_panel")
    left_joint = object_model.get_articulation("frame_to_left_stay")
    right_joint = object_model.get_articulation("frame_to_right_stay")

    ctx.check(
        "panel hinge uses horizontal x axis",
        tuple(panel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={panel_joint.axis}",
    )
    ctx.check(
        "stay bars use horizontal support pivots",
        tuple(left_joint.axis) == (-1.0, 0.0, 0.0) and tuple(right_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"left={left_joint.axis}, right={right_joint.axis}",
    )

    with ctx.pose({panel_joint: 0.0, left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem="panel_skin",
            negative_elem="frame_left",
            min_gap=0.003,
            max_gap=0.0055,
            name="panel clears left frame rail",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem="frame_right",
            negative_elem="panel_skin",
            min_gap=0.003,
            max_gap=0.0055,
            name="panel clears right frame rail",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="frame_top",
            negative_elem="panel_skin",
            min_gap=0.006,
            max_gap=0.010,
            name="panel top edge stays below top rail",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="panel_skin",
            negative_elem="frame_bottom",
            min_gap=0.0,
            max_gap=0.002,
            name="panel sits on the lower hinge line without sinking into the frame",
        )

        ctx.expect_gap(
            left_stay,
            panel,
            axis="x",
            positive_elem="stay_tip",
            negative_elem="left_lug",
            max_gap=0.002,
            max_penetration=1e-5,
            name="left stay tip sits just inboard of the left lug",
        )
        ctx.expect_gap(
            panel,
            right_stay,
            axis="x",
            positive_elem="right_lug",
            negative_elem="stay_tip",
            max_gap=0.002,
            max_penetration=1e-5,
            name="right stay tip sits just inboard of the right lug",
        )

        left_tip_closed = ctx.part_element_world_aabb(left_stay, elem="stay_tip")
        right_tip_closed = ctx.part_element_world_aabb(right_stay, elem="stay_tip")
        panel_closed = ctx.part_element_world_aabb(panel, elem="panel_skin")

    with ctx.pose({panel_joint: 1.05}):
        panel_open = ctx.part_element_world_aabb(panel, elem="panel_skin")
        panel_opens_out = (
            panel_closed is not None
            and panel_open is not None
            and panel_open[0][1] < panel_closed[0][1] - 0.12
            and panel_open[1][2] < panel_closed[1][2] - 0.20
        )
        ctx.check(
            "panel opens downward and outward",
            panel_opens_out,
            details=f"closed={panel_closed}, open={panel_open}",
        )

    with ctx.pose({left_joint: 0.55, right_joint: 0.55}):
        left_tip_open = ctx.part_element_world_aabb(left_stay, elem="stay_tip")
        right_tip_open = ctx.part_element_world_aabb(right_stay, elem="stay_tip")
        left_swings = (
            left_tip_closed is not None
            and left_tip_open is not None
            and left_tip_open[0][1] < left_tip_closed[0][1] - 0.08
        )
        right_swings = (
            right_tip_closed is not None
            and right_tip_open is not None
            and right_tip_open[0][1] < right_tip_closed[0][1] - 0.08
        )
        ctx.check(
            "left stay bar swings outward from its side pivot",
            left_swings,
            details=f"closed={left_tip_closed}, open={left_tip_open}",
        )
        ctx.check(
            "right stay bar swings outward from its side pivot",
            right_swings,
            details=f"closed={right_tip_closed}, open={right_tip_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

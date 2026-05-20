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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

FRAME_WIDTH = 0.90
FRAME_FRONT_Y = 0.50
FRAME_REAR_Y = -0.66
FRAME_CENTER_Y = (FRAME_FRONT_Y + FRAME_REAR_Y) / 2.0
FRAME_LENGTH = FRAME_FRONT_Y - FRAME_REAR_Y

RAIL_CENTER_X = 0.31
RAIL_CENTER_Y = -0.12
RAIL_LENGTH = 1.02
RAIL_BASE_WIDTH = 0.054
RAIL_BASE_THICKNESS = 0.006
RAIL_FLANGE_WIDTH = 0.007
RAIL_FLANGE_HEIGHT = 0.010

PANEL_WIDTH = 0.74
PANEL_LENGTH = 0.82
PANEL_GLASS_THICKNESS = 0.005
SEAL_WIDTH = 0.018
SEAL_THICKNESS = 0.004

TILT_AXIS_Y = 0.39
TILT_AXIS_Z = 0.022
DRIVE_CONNECT_LOCAL_Y = -0.200
PANEL_FRONT_SEAL_Y = 0.011
PANEL_REAR_SEAL_Y = PANEL_FRONT_SEAL_Y - (PANEL_LENGTH - SEAL_WIDTH)
PANEL_CENTER_Y = (PANEL_FRONT_SEAL_Y + PANEL_REAR_SEAL_Y) / 2.0
PANEL_SIDE_SEAL_LENGTH = PANEL_LENGTH - SEAL_WIDTH

SLIDE_TRAVEL = 0.24
MAX_TILT = 0.07


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_sunroof_cassette")

    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.20, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    glass = model.material("glass", rgba=(0.28, 0.40, 0.48, 0.42))
    nylon = model.material("nylon", rgba=(0.14, 0.15, 0.17, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_WIDTH, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, 0.470, 0.010)),
        material=steel,
        name="front_crossmember",
    )
    frame.visual(
        Box((FRAME_WIDTH, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, -0.630, 0.010)),
        material=steel,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.070, 1.080, 0.020)),
        origin=Origin(xyz=(-0.415, FRAME_CENTER_Y, 0.010)),
        material=steel,
        name="left_side_member",
    )
    frame.visual(
        Box((0.070, 1.080, 0.020)),
        origin=Origin(xyz=(0.415, FRAME_CENTER_Y, 0.010)),
        material=steel,
        name="right_side_member",
    )
    frame.visual(
        Box((0.020, PANEL_SIDE_SEAL_LENGTH, 0.004)),
        origin=Origin(xyz=(-0.380, 0.000, 0.020)),
        material=dark_metal,
        name="left_seat_ledge",
    )
    frame.visual(
        Box((0.020, PANEL_SIDE_SEAL_LENGTH, 0.004)),
        origin=Origin(xyz=(0.380, 0.000, 0.020)),
        material=dark_metal,
        name="right_seat_ledge",
    )

    flange_offset = (RAIL_BASE_WIDTH / 2.0) - (RAIL_FLANGE_WIDTH / 2.0)
    for side_name, rail_x, sign in (
        ("left", -RAIL_CENTER_X, -1.0),
        ("right", RAIL_CENTER_X, 1.0),
    ):
        frame.visual(
            Box((RAIL_BASE_WIDTH, RAIL_LENGTH, RAIL_BASE_THICKNESS)),
            origin=Origin(xyz=(rail_x, RAIL_CENTER_Y, 0.007)),
            material=dark_metal,
            name=f"{side_name}_rail_base",
        )
        frame.visual(
            Box((RAIL_FLANGE_WIDTH, RAIL_LENGTH, RAIL_FLANGE_HEIGHT)),
            origin=Origin(xyz=(rail_x - sign * flange_offset, RAIL_CENTER_Y, 0.015)),
            material=dark_metal,
            name=f"{side_name}_rail_outer_flange",
        )
        frame.visual(
            Box((RAIL_FLANGE_WIDTH, RAIL_LENGTH, RAIL_FLANGE_HEIGHT)),
            origin=Origin(xyz=(rail_x + sign * flange_offset, RAIL_CENTER_Y, 0.015)),
            material=dark_metal,
            name=f"{side_name}_rail_inner_flange",
        )

    frame.visual(
        Box((PANEL_WIDTH, SEAL_WIDTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.401, 0.020)),
        material=rubber,
        name="front_seal_seat",
    )
    frame.visual(
        Box((PANEL_WIDTH, SEAL_WIDTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.401, 0.020)),
        material=rubber,
        name="rear_seal_seat",
    )
    frame.visual(
        Box((SEAL_WIDTH, PANEL_SIDE_SEAL_LENGTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(-0.361, 0.000, 0.020)),
        material=rubber,
        name="left_seal_seat",
    )
    frame.visual(
        Box((SEAL_WIDTH, PANEL_SIDE_SEAL_LENGTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.361, 0.000, 0.020)),
        material=rubber,
        name="right_seal_seat",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_LENGTH, 0.050)),
        mass=8.2,
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, 0.025)),
    )

    slider = model.part("slider")
    for shoe_name, shoe_x, shoe_y in (
        ("front_left", -RAIL_CENTER_X, 0.300),
        ("front_right", RAIL_CENTER_X, 0.300),
        ("rear_left", -RAIL_CENTER_X, -0.310),
        ("rear_right", RAIL_CENTER_X, -0.310),
    ):
        slider.visual(
            Box((0.030, 0.060, 0.006)),
            origin=Origin(xyz=(shoe_x, shoe_y, 0.013)),
            material=nylon,
            name=f"{shoe_name}_shoe_pad",
        )
        slider.visual(
            Box((0.018, 0.040, 0.008)),
            origin=Origin(xyz=(shoe_x, shoe_y, 0.018)),
            material=nylon,
            name=f"{shoe_name}_shoe_tower",
        )
        slider.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(shoe_x, shoe_y, 0.014),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=f"{shoe_name}_roller",
        )

    slider.visual(
        Box((0.014, 0.620, 0.004)),
        origin=Origin(xyz=(-RAIL_CENTER_X, -0.005, 0.014)),
        material=nylon,
        name="left_guide_link",
    )
    slider.visual(
        Box((0.014, 0.620, 0.004)),
        origin=Origin(xyz=(RAIL_CENTER_X, -0.005, 0.014)),
        material=nylon,
        name="right_guide_link",
    )
    slider.visual(
        Box((0.560, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.300, 0.014)),
        material=nylon,
        name="front_bridge",
    )
    slider.visual(
        Box((0.220, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.310, 0.010)),
        material=nylon,
        name="rear_bridge",
    )
    slider.visual(
        Box((0.046, 0.700, 0.006)),
        origin=Origin(xyz=(0.0, -0.005, 0.013)),
        material=dark_metal,
        name="center_drive_housing",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.640, 0.760, 0.060)),
        mass=1.9,
        origin=Origin(xyz=(0.0, -0.020, 0.030)),
    )

    panel = model.part("glass_panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_LENGTH, PANEL_GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, 0.0065)),
        material=glass,
        name="glass_panel",
    )
    panel.visual(
        Box((PANEL_WIDTH, SEAL_WIDTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.0, PANEL_FRONT_SEAL_Y, 0.002)),
        material=rubber,
        name="front_seal_lip",
    )
    panel.visual(
        Box((PANEL_WIDTH, SEAL_WIDTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.0, PANEL_REAR_SEAL_Y, 0.002)),
        material=rubber,
        name="rear_seal_lip",
    )
    panel.visual(
        Box((SEAL_WIDTH, PANEL_SIDE_SEAL_LENGTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(-0.361, PANEL_CENTER_Y, 0.002)),
        material=rubber,
        name="left_seal_lip",
    )
    panel.visual(
        Box((SEAL_WIDTH, PANEL_SIDE_SEAL_LENGTH, SEAL_THICKNESS)),
        origin=Origin(xyz=(0.361, PANEL_CENTER_Y, 0.002)),
        material=rubber,
        name="right_seal_lip",
    )
    panel.visual(
        Box((0.034, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, DRIVE_CONNECT_LOCAL_Y, 0.001)),
        material=steel,
        name="drive_pickup",
    )
    panel.visual(
        Box((0.020, 0.034, 0.006)),
        origin=Origin(xyz=(-0.240, -0.610, 0.001)),
        material=steel,
        name="left_pivot_mount",
    )
    panel.visual(
        Box((0.020, 0.034, 0.006)),
        origin=Origin(xyz=(0.240, -0.610, 0.001)),
        material=steel,
        name="right_pivot_mount",
    )
    panel.visual(
        Box((0.014, 0.100, 0.004)),
        origin=Origin(xyz=(-0.240, -0.660, -0.006), rpy=(0.17, 0.0, 0.0)),
        material=steel,
        name="left_pivot_arm",
    )
    panel.visual(
        Box((0.014, 0.100, 0.004)),
        origin=Origin(xyz=(0.240, -0.660, -0.006), rpy=(0.17, 0.0, 0.0)),
        material=steel,
        name="right_pivot_arm",
    )
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_LENGTH, 0.040)),
        mass=4.6,
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, 0.010)),
    )

    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.45,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "slider_to_panel",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=panel,
        origin=Origin(xyz=(0.0, TILT_AXIS_Y, TILT_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.60,
            lower=0.0,
            upper=MAX_TILT,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slider = object_model.get_part("slider")
    panel = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("frame_to_slider")
    tilt = object_model.get_articulation("slider_to_panel")

    left_rail_base = frame.get_visual("left_rail_base")
    right_rail_base = frame.get_visual("right_rail_base")
    front_seat = frame.get_visual("front_seal_seat")
    rear_seat = frame.get_visual("rear_seal_seat")
    left_seat = frame.get_visual("left_seal_seat")
    right_seat = frame.get_visual("right_seal_seat")

    front_left_pad = slider.get_visual("front_left_shoe_pad")
    front_right_pad = slider.get_visual("front_right_shoe_pad")
    rear_left_pad = slider.get_visual("rear_left_shoe_pad")
    rear_right_pad = slider.get_visual("rear_right_shoe_pad")
    center_drive_housing = slider.get_visual("center_drive_housing")

    glass_panel = panel.get_visual("glass_panel")
    front_seal = panel.get_visual("front_seal_lip")
    rear_seal = panel.get_visual("rear_seal_lip")
    left_seal = panel.get_visual("left_seal_lip")
    right_seal = panel.get_visual("right_seal_lip")
    drive_pickup = panel.get_visual("drive_pickup")
    left_pivot_mount = panel.get_visual("left_pivot_mount")
    right_pivot_mount = panel.get_visual("right_pivot_mount")
    left_pivot_arm = panel.get_visual("left_pivot_arm")
    right_pivot_arm = panel.get_visual("right_pivot_arm")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(panel, frame, axes="xy", min_overlap=0.42)
    ctx.expect_within(panel, frame, axes="x", inner_elem=glass_panel)
    ctx.expect_within(slider, frame, axes="x", inner_elem=center_drive_housing)

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem=front_seal,
        negative_elem=front_seat,
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_seal,
        negative_elem=rear_seat,
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_seal,
        negative_elem=left_seat,
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_seal,
        negative_elem=right_seat,
    )
    ctx.expect_within(panel, slider, axes="xy", inner_elem=drive_pickup, outer_elem=center_drive_housing)

    for shoe, rail in (
        (front_left_pad, left_rail_base),
        (rear_left_pad, left_rail_base),
        (front_right_pad, right_rail_base),
        (rear_right_pad, right_rail_base),
    ):
        ctx.expect_within(slider, frame, axes="xy", inner_elem=shoe, outer_elem=rail)
        ctx.expect_gap(
            slider,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem=shoe,
            negative_elem=rail,
        )

    ctx.expect_within(panel, slider, axes="xy", inner_elem=left_pivot_mount)
    ctx.expect_within(panel, slider, axes="xy", inner_elem=right_pivot_mount)
    ctx.expect_within(panel, frame, axes="xy", inner_elem=left_pivot_arm)
    ctx.expect_within(panel, frame, axes="xy", inner_elem=right_pivot_arm)

    with ctx.pose({tilt: 0.055}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            min_gap=0.030,
            positive_elem=rear_seal,
            negative_elem=rear_seat,
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            max_gap=0.003,
            max_penetration=0.002,
            positive_elem=front_seal,
            negative_elem=front_seat,
        )
        ctx.expect_within(panel, slider, axes="x", inner_elem=drive_pickup, outer_elem=center_drive_housing)

    with ctx.pose({slide: 0.220, tilt: 0.040}):
        ctx.expect_gap(
            frame,
            panel,
            axis="y",
            min_gap=0.190,
            positive_elem=front_seat,
            negative_elem=front_seal,
        )
        ctx.expect_within(slider, frame, axes="xy", inner_elem=front_left_pad, outer_elem=left_rail_base)
        ctx.expect_within(slider, frame, axes="xy", inner_elem=rear_left_pad, outer_elem=left_rail_base)
        ctx.expect_within(slider, frame, axes="xy", inner_elem=front_right_pad, outer_elem=right_rail_base)
        ctx.expect_within(slider, frame, axes="xy", inner_elem=rear_right_pad, outer_elem=right_rail_base)
        ctx.expect_within(panel, slider, axes="xy", inner_elem=drive_pickup, outer_elem=center_drive_housing)
        ctx.expect_within(panel, slider, axes="xy", inner_elem=left_pivot_mount)
        ctx.expect_within(panel, slider, axes="xy", inner_elem=right_pivot_mount)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

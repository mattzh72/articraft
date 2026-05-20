from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)

CABINET_WIDTH = 0.90
CABINET_DEPTH = 0.58
PLINTH_HEIGHT = 0.10
CARCASS_HEIGHT = 0.76

COUNTERTOP_WIDTH = 0.96
COUNTERTOP_DEPTH = 0.62
COUNTERTOP_THICKNESS = 0.04
COUNTERTOP_BOTTOM_Z = PLINTH_HEIGHT + CARCASS_HEIGHT

SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.012
BOTTOM_THICKNESS = 0.018
DOOR_THICKNESS = 0.019

OPENING_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
DOOR_CENTER_GAP = 0.004
DOOR_WIDTH = (OPENING_WIDTH - DOOR_CENTER_GAP) / 2.0
DOOR_HEIGHT = 0.62
DOOR_BOTTOM_Z = 0.12
DOOR_CENTER_Z = DOOR_BOTTOM_Z + DOOR_HEIGHT / 2.0
DOOR_AXIS_Y = -CABINET_DEPTH / 2.0
HINGE_RADIUS = 0.008
LEFT_DOOR_AXIS_X = -CABINET_WIDTH / 2.0 - HINGE_RADIUS
RIGHT_DOOR_AXIS_X = CABINET_WIDTH / 2.0 + HINGE_RADIUS

COOKTOP_OPENING_WIDTH = 0.57
COOKTOP_OPENING_DEPTH = 0.50
COOKTOP_GLASS_WIDTH = 0.60
COOKTOP_GLASS_DEPTH = 0.52
COOKTOP_GLASS_THICKNESS = 0.006
COOKTOP_BODY_WIDTH = 0.54
COOKTOP_BODY_DEPTH = 0.42
COOKTOP_BODY_HEIGHT = 0.09

CONTROL_STRIP_WIDTH = 0.34
CONTROL_STRIP_HEIGHT = 0.10
CONTROL_STRIP_DEPTH = 0.014
CONTROL_STRIP_CENTER_Y = -0.320
CONTROL_STRIP_CENTER_Z = -0.050
BUTTON_DEPTH = 0.006
GUIDE_THICKNESS = 0.004

BUTTON_WIDTH = 0.022
BUTTON_HEIGHT = 0.010
BUTTON_TRAVEL = 0.004
BUTTON_LAYOUT = (
    (-0.102, 0.020),
    (-0.102, -0.012),
    (-0.038, 0.040),
    (0.038, 0.040),
    (0.102, 0.020),
    (0.102, -0.012),
)
KNOB_CENTER = (0.0, 0.004)
KNOB_SHAFT_RADIUS = 0.017


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rect_profile(
    width: float,
    height: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 32,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _build_control_strip_mesh():
    hole_profiles = [_circle_profile(KNOB_SHAFT_RADIUS, center=KNOB_CENTER, segments=36)]
    for x_pos, z_pos in BUTTON_LAYOUT:
        hole_profiles.append(_rect_profile(BUTTON_WIDTH, BUTTON_HEIGHT, center=(x_pos, z_pos)))
    plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(CONTROL_STRIP_WIDTH, CONTROL_STRIP_HEIGHT, 0.010, corner_segments=8),
        hole_profiles,
        height=CONTROL_STRIP_DEPTH,
        center=True,
    )
    return plate.rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cooktop_cabinet", assets=ASSETS)

    model.material("cabinet_paint", rgba=(0.90, 0.91, 0.88, 1.0))
    model.material("cabinet_shadow", rgba=(0.75, 0.77, 0.73, 1.0))
    model.material("plinth_dark", rgba=(0.18, 0.18, 0.18, 1.0))
    model.material("counter_stone", rgba=(0.80, 0.81, 0.82, 1.0))
    model.material("cooktop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("cooktop_zone", rgba=(0.31, 0.32, 0.35, 1.0))
    model.material("button_light", rgba=(0.92, 0.91, 0.87, 1.0))
    model.material("knob_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    model.material("metal", rgba=(0.69, 0.71, 0.73, 1.0))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + CARCASS_HEIGHT / 2.0,
            )
        ),
        material="cabinet_paint",
        name="left_side",
    )
    cabinet_body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + CARCASS_HEIGHT / 2.0,
            )
        ),
        material="cabinet_paint",
        name="right_side",
    )
    cabinet_body.visual(
        Box((OPENING_WIDTH, BACK_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0 - BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + CARCASS_HEIGHT / 2.0,
            )
        ),
        material="cabinet_shadow",
        name="back_panel",
    )
    cabinet_body.visual(
        Box((OPENING_WIDTH, CABINET_DEPTH - BACK_THICKNESS, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + BOTTOM_THICKNESS / 2.0,
            )
        ),
        material="cabinet_paint",
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((0.13, 0.08, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                -OPENING_WIDTH / 2.0 + 0.065,
                CABINET_DEPTH / 2.0 - 0.040,
                COUNTERTOP_BOTTOM_Z - BOTTOM_THICKNESS / 2.0,
            )
        ),
        material="cabinet_paint",
        name="left_top_back_rail",
    )
    cabinet_body.visual(
        Box((0.13, 0.08, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH / 2.0 - 0.065,
                CABINET_DEPTH / 2.0 - 0.040,
                COUNTERTOP_BOTTOM_Z - BOTTOM_THICKNESS / 2.0,
            )
        ),
        material="cabinet_paint",
        name="right_top_back_rail",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH, 0.018, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH / 2.0 + 0.009, PLINTH_HEIGHT / 2.0)),
        material="plinth_dark",
        name="plinth_front",
    )
    cabinet_body.visual(
        Box((0.060, CABINET_DEPTH - 0.120, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + 0.030,
                0.0,
                PLINTH_HEIGHT / 2.0,
            )
        ),
        material="plinth_dark",
        name="plinth_left_return",
    )
    cabinet_body.visual(
        Box((0.060, CABINET_DEPTH - 0.120, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - 0.030,
                0.0,
                PLINTH_HEIGHT / 2.0,
            )
        ),
        material="plinth_dark",
        name="plinth_right_return",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, COUNTERTOP_BOTTOM_Z)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_BOTTOM_Z / 2.0)),
    )

    countertop = model.part("countertop")
    side_run_width = (COUNTERTOP_WIDTH - COOKTOP_OPENING_WIDTH) / 2.0
    front_back_depth = (COUNTERTOP_DEPTH - COOKTOP_OPENING_DEPTH) / 2.0
    countertop.visual(
        Box((side_run_width, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS)),
        origin=Origin(
            xyz=(
                -COUNTERTOP_WIDTH / 2.0 + side_run_width / 2.0,
                0.0,
                COUNTERTOP_THICKNESS / 2.0,
            )
        ),
        material="counter_stone",
        name="left_run",
    )
    countertop.visual(
        Box((side_run_width, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS)),
        origin=Origin(
            xyz=(
                COUNTERTOP_WIDTH / 2.0 - side_run_width / 2.0,
                0.0,
                COUNTERTOP_THICKNESS / 2.0,
            )
        ),
        material="counter_stone",
        name="right_run",
    )
    countertop.visual(
        Box((COOKTOP_OPENING_WIDTH, front_back_depth, COUNTERTOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -COUNTERTOP_DEPTH / 2.0 + front_back_depth / 2.0,
                COUNTERTOP_THICKNESS / 2.0,
            )
        ),
        material="counter_stone",
        name="front_run",
    )
    countertop.visual(
        Box((COOKTOP_OPENING_WIDTH, front_back_depth, COUNTERTOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                COUNTERTOP_DEPTH / 2.0 - front_back_depth / 2.0,
                COUNTERTOP_THICKNESS / 2.0,
            )
        ),
        material="counter_stone",
        name="back_run",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTERTOP_WIDTH, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_THICKNESS / 2.0)),
    )

    cooktop_body = model.part("cooktop_body")
    cooktop_body.visual(
        Box((COOKTOP_GLASS_WIDTH, COOKTOP_GLASS_DEPTH, COOKTOP_GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_GLASS_THICKNESS / 2.0)),
        material="cooktop_black",
        name="glass_top",
    )
    cooktop_body.visual(
        Box((COOKTOP_BODY_WIDTH, COOKTOP_BODY_DEPTH, COOKTOP_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.03, -COOKTOP_BODY_HEIGHT / 2.0)),
        material="cooktop_black",
        name="underslung_body",
    )
    for zone_name, x_pos, y_pos, radius in (
        ("zone_left_front", -0.175, -0.135, 0.090),
        ("zone_left_rear", -0.170, 0.135, 0.105),
        ("zone_center", 0.000, 0.015, 0.125),
        ("zone_right_front", 0.175, -0.135, 0.090),
        ("zone_right_rear", 0.170, 0.135, 0.105),
    ):
        cooktop_body.visual(
            Cylinder(radius=radius, length=0.0006),
            origin=Origin(xyz=(x_pos, y_pos, COOKTOP_GLASS_THICKNESS - 0.0003)),
            material="cooktop_zone",
            name=zone_name,
        )
    cooktop_body.inertial = Inertial.from_geometry(
        Box((COOKTOP_GLASS_WIDTH, COOKTOP_GLASS_DEPTH, COOKTOP_BODY_HEIGHT + COOKTOP_GLASS_THICKNESS)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((CONTROL_STRIP_WIDTH, 0.006, CONTROL_STRIP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material="cooktop_black",
        name="rear_mount",
    )
    control_strip.visual(
        Box((0.100, CONTROL_STRIP_DEPTH, 0.012)),
        origin=Origin(xyz=(-0.120, 0.003, 0.044)),
        material="cooktop_black",
        name="left_top_rail",
    )
    control_strip.visual(
        Box((0.100, CONTROL_STRIP_DEPTH, 0.012)),
        origin=Origin(xyz=(0.120, 0.003, 0.044)),
        material="cooktop_black",
        name="right_top_rail",
    )
    control_strip.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, 0.012)),
        origin=Origin(xyz=(0.0, 0.003, -0.044)),
        material="cooktop_black",
        name="bottom_rail",
    )
    control_strip.visual(
        Box((0.018, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT)),
        origin=Origin(xyz=(-0.161, 0.003, 0.0)),
        material="cooktop_black",
        name="left_rail",
    )
    control_strip.visual(
        Box((0.018, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT)),
        origin=Origin(xyz=(0.161, 0.003, 0.0)),
        material="cooktop_black",
        name="right_rail",
    )
    control_strip.visual(
        Box((GUIDE_THICKNESS, CONTROL_STRIP_DEPTH, 0.044)),
        origin=Origin(xyz=(-(KNOB_SHAFT_RADIUS + GUIDE_THICKNESS / 2.0), 0.003, KNOB_CENTER[1])),
        material="cooktop_black",
        name="knob_left_guide",
    )
    control_strip.visual(
        Box((GUIDE_THICKNESS, CONTROL_STRIP_DEPTH, 0.044)),
        origin=Origin(xyz=((KNOB_SHAFT_RADIUS + GUIDE_THICKNESS / 2.0), 0.003, KNOB_CENTER[1])),
        material="cooktop_black",
        name="knob_right_guide",
    )
    control_strip.visual(
        Box((KNOB_SHAFT_RADIUS * 2.0, CONTROL_STRIP_DEPTH, GUIDE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.003, KNOB_CENTER[1] + KNOB_SHAFT_RADIUS + GUIDE_THICKNESS / 2.0)),
        material="cooktop_black",
        name="knob_top_guide",
    )
    control_strip.visual(
        Box((KNOB_SHAFT_RADIUS * 2.0, CONTROL_STRIP_DEPTH, GUIDE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.003, KNOB_CENTER[1] - KNOB_SHAFT_RADIUS - GUIDE_THICKNESS / 2.0)),
        material="cooktop_black",
        name="knob_bottom_guide",
    )
    for index, (x_pos, z_pos) in enumerate(BUTTON_LAYOUT, start=1):
        control_strip.visual(
            Box((GUIDE_THICKNESS, CONTROL_STRIP_DEPTH, BUTTON_HEIGHT + 0.006)),
            origin=Origin(xyz=(x_pos - (BUTTON_WIDTH / 2.0 + GUIDE_THICKNESS / 2.0), 0.003, z_pos)),
            material="cooktop_black",
            name=f"button_{index}_left_guide",
        )
        control_strip.visual(
            Box((GUIDE_THICKNESS, CONTROL_STRIP_DEPTH, BUTTON_HEIGHT + 0.006)),
            origin=Origin(xyz=(x_pos + (BUTTON_WIDTH / 2.0 + GUIDE_THICKNESS / 2.0), 0.003, z_pos)),
            material="cooktop_black",
            name=f"button_{index}_right_guide",
        )
    control_strip.inertial = Inertial.from_geometry(
        Box((CONTROL_STRIP_WIDTH, 0.010, CONTROL_STRIP_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + SIDE_THICKNESS + HINGE_RADIUS, -DOOR_THICKNESS / 2.0, 0.0)),
        material="cabinet_paint",
        name="door_panel",
    )
    left_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(0.0, -HINGE_RADIUS, 0.0)),
        material="metal",
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.022, 0.006, 0.100)),
        origin=Origin(xyz=(0.019, -0.008, 0.190)),
        material="metal",
        name="upper_hinge_leaf",
    )
    left_door.visual(
        Box((0.022, 0.006, 0.100)),
        origin=Origin(xyz=(0.019, -0.008, -0.190)),
        material="metal",
        name="lower_hinge_leaf",
    )
    left_door.visual(
        Box((0.012, 0.012, 0.160)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.040 + SIDE_THICKNESS + HINGE_RADIUS, -DOOR_THICKNESS - 0.006, 0.0)),
        material="metal",
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + SIDE_THICKNESS + HINGE_RADIUS, -DOOR_THICKNESS / 2.0, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0 + SIDE_THICKNESS + HINGE_RADIUS), -DOOR_THICKNESS / 2.0, 0.0)),
        material="cabinet_paint",
        name="door_panel",
    )
    right_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_HEIGHT),
        origin=Origin(xyz=(0.0, -HINGE_RADIUS, 0.0)),
        material="metal",
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.022, 0.006, 0.100)),
        origin=Origin(xyz=(-0.019, -0.008, 0.190)),
        material="metal",
        name="upper_hinge_leaf",
    )
    right_door.visual(
        Box((0.022, 0.006, 0.100)),
        origin=Origin(xyz=(-0.019, -0.008, -0.190)),
        material="metal",
        name="lower_hinge_leaf",
    )
    right_door.visual(
        Box((0.012, 0.012, 0.160)),
        origin=Origin(xyz=(-DOOR_WIDTH + 0.040 - SIDE_THICKNESS - HINGE_RADIUS, -DOOR_THICKNESS - 0.006, 0.0)),
        material="metal",
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0 + SIDE_THICKNESS + HINGE_RADIUS), -DOOR_THICKNESS / 2.0, 0.0)),
    )

    for index in range(6):
        button = model.part(f"button_{index + 1}")
        button.visual(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
            material="button_light",
            name="button_body",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
            mass=0.02,
        )

    center_knob = model.part("center_knob")
    center_knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=BUTTON_DEPTH),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_dark",
        name="knob_shaft",
    )
    center_knob.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_dark",
        name="knob_grip",
    )
    center_knob.visual(
        Box((0.010, 0.003, 0.003)),
        origin=Origin(xyz=(0.018, -0.0135, 0.0)),
        material="button_light",
        name="indicator_mark",
    )
    center_knob.inertial = Inertial.from_geometry(
        Box((0.056, 0.016, 0.056)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
    )

    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_BOTTOM_Z)),
    )
    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop_body,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_THICKNESS)),
    )
    model.articulation(
        "countertop_to_control_strip",
        ArticulationType.FIXED,
        parent=countertop,
        child=control_strip,
        origin=Origin(xyz=(0.0, CONTROL_STRIP_CENTER_Y, CONTROL_STRIP_CENTER_Z)),
    )
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=left_door,
        origin=Origin(xyz=(LEFT_DOOR_AXIS_X, DOOR_AXIS_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=right_door,
        origin=Origin(xyz=(RIGHT_DOOR_AXIS_X, DOOR_AXIS_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    for index, (x_pos, z_pos) in enumerate(BUTTON_LAYOUT, start=1):
        model.articulation(
            f"control_strip_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=f"button_{index}",
            origin=Origin(xyz=(x_pos, -0.003, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )
    model.articulation(
        "control_strip_to_center_knob",
        ArticulationType.CONTINUOUS,
        parent=control_strip,
        child=center_knob,
        origin=Origin(xyz=(KNOB_CENTER[0], -0.003, KNOB_CENTER[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    cabinet_body = object_model.get_part("cabinet_body")
    countertop = object_model.get_part("countertop")
    cooktop_body = object_model.get_part("cooktop_body")
    control_strip = object_model.get_part("control_strip")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    center_knob = object_model.get_part("center_knob")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 7)]

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    knob_joint = object_model.get_articulation("control_strip_to_center_knob")
    button_joints = [
        object_model.get_articulation(f"control_strip_to_button_{index}") for index in range(1, 7)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(countertop, cabinet_body, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_contact(cooktop_body, countertop, elem_a="glass_top")
    ctx.expect_contact(control_strip, countertop, elem_a="rear_mount", elem_b="front_run")
    ctx.expect_contact(left_door, cabinet_body)
    ctx.expect_contact(right_door, cabinet_body)
    ctx.expect_gap(right_door, left_door, axis="x", min_gap=0.003, max_gap=0.007)
    ctx.expect_overlap(cooktop_body, countertop, axes="xy", min_overlap=0.50)

    ctx.check(
        "left_door_hinge_axis",
        left_hinge.axis == (0.0, 0.0, -1.0),
        f"Expected left hinge axis (0,0,-1), got {left_hinge.axis}",
    )
    ctx.check(
        "right_door_hinge_axis",
        right_hinge.axis == (0.0, 0.0, 1.0),
        f"Expected right hinge axis (0,0,1), got {right_hinge.axis}",
    )
    ctx.check(
        "knob_joint_setup",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and knob_joint.axis == (0.0, 1.0, 0.0),
        f"Expected continuous knob on Y axis, got {knob_joint.articulation_type} {knob_joint.axis}",
    )

    for button_index, button_joint in enumerate(button_joints, start=1):
        limits = button_joint.motion_limits
        ctx.check(
            f"button_{button_index}_joint_setup",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and button_joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == BUTTON_TRAVEL,
            f"Button {button_index} should press inward 4 mm on +Y; got axis={button_joint.axis}, limits={limits}",
        )

    button_rest_positions = []
    for button in buttons:
        position = ctx.part_world_position(button)
        assert position is not None
        button_rest_positions.append(position)

    for index, (button, joint, rest_position) in enumerate(zip(buttons, button_joints, button_rest_positions), start=1):
        limits = joint.motion_limits
        assert limits is not None and limits.upper is not None
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.check(
                f"button_{index}_travels_inward",
                pressed_position[1] > rest_position[1] + 0.0035,
                f"Button {index} should move inward on +Y. rest={rest_position}, pressed={pressed_position}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"button_{index}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"button_{index}_upper_no_floating")

    for door_name, door, hinge in (
        ("left", left_door, left_hinge),
        ("right", right_door, right_hinge),
    ):
        limits = hinge.motion_limits
        assert limits is not None and limits.upper is not None
        closed_aabb = ctx.part_world_aabb(door)
        assert closed_aabb is not None
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
            assert open_aabb is not None
            ctx.check(
                f"{door_name}_door_swings_out",
                open_aabb[0][1] < closed_aabb[0][1] - 0.15,
                f"{door_name} door should swing into negative Y. closed={closed_aabb}, open={open_aabb}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{door_name}_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{door_name}_door_upper_no_floating")

    with ctx.pose({knob_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_quarter_turn_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


PIVOT_RADIUS = 0.012
PIVOT_GAP = 0.018
EAR_THICKNESS = 0.006
LINK_BODY_WIDTH = 0.012
LINK_BODY_HEIGHT = 0.018
LOWER_LINK_LENGTH = 0.118
UPPER_LINK_LENGTH = 0.112
BRACKET_WIDTH = 0.072
BRACKET_HEIGHT = 0.150
PAN_HUB_RADIUS = 0.018
PAN_HUB_THICKNESS = 0.005
PAN_BASE_THICKNESS = 0.006
PAN_NECK_HEIGHT = 0.024
TILT_AXIS_X = 0.032
TILT_AXIS_Z = 0.016
YOKE_GAP = 0.230
DISPLAY_WIDTH = 0.210
DISPLAY_HEIGHT = 0.138
DISPLAY_DEPTH = 0.022
SCREEN_WIDTH = 0.182
SCREEN_HEIGHT = 0.108
SCREEN_THICKNESS = 0.002


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _y_cylinder(radius: float, length: float, *, x: float, z: float, y: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _back_bracket_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.008, BRACKET_WIDTH, BRACKET_HEIGHT).translate((-0.020, 0.0, 0.0))
    standoff = cq.Workplane("XY").box(0.024, 0.030, 0.052).translate((-0.010, 0.0, 0.0))
    rib = (
        cq.Workplane("XZ")
        .polyline([(-0.024, -0.028), (-0.004, -0.018), (0.004, 0.0), (-0.004, 0.018), (-0.024, 0.028)])
        .close()
        .extrude(0.010, both=True)
    )
    ear_offset = PIVOT_GAP / 2.0 + EAR_THICKNESS / 2.0
    ear_left = _y_cylinder(PIVOT_RADIUS, EAR_THICKNESS, x=0.0, z=0.0, y=ear_offset)
    ear_right = _y_cylinder(PIVOT_RADIUS, EAR_THICKNESS, x=0.0, z=0.0, y=-ear_offset)
    cheek_left = cq.Workplane("XY").box(0.018, EAR_THICKNESS, 0.024).translate((-0.009, ear_offset, 0.0))
    cheek_right = cq.Workplane("XY").box(0.018, EAR_THICKNESS, 0.024).translate((-0.009, -ear_offset, 0.0))
    return _combine(plate, standoff, rib, ear_left, ear_right, cheek_left, cheek_right)


def _lower_link_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XZ")
        .center((LOWER_LINK_LENGTH - 0.016) / 2.0, 0.0)
        .slot2D(LOWER_LINK_LENGTH - 0.034, LINK_BODY_HEIGHT)
        .extrude(LINK_BODY_WIDTH / 2.0, both=True)
    )
    rear_barrel = _y_cylinder(PIVOT_RADIUS, PIVOT_GAP, x=0.0, z=0.0)
    front_connector = cq.Workplane("XY").box(0.016, LINK_BODY_WIDTH, 0.018).translate((LOWER_LINK_LENGTH - 0.008, 0.0, 0.0))
    ear_offset = PIVOT_GAP / 2.0 + EAR_THICKNESS / 2.0
    front_ear_left = _y_cylinder(PIVOT_RADIUS, EAR_THICKNESS, x=LOWER_LINK_LENGTH, z=0.0, y=ear_offset)
    front_ear_right = _y_cylinder(PIVOT_RADIUS, EAR_THICKNESS, x=LOWER_LINK_LENGTH, z=0.0, y=-ear_offset)
    cheek_left = cq.Workplane("XY").box(0.014, EAR_THICKNESS, 0.022).translate((LOWER_LINK_LENGTH - 0.007, ear_offset, 0.0))
    cheek_right = cq.Workplane("XY").box(0.014, EAR_THICKNESS, 0.022).translate((LOWER_LINK_LENGTH - 0.007, -ear_offset, 0.0))
    return _combine(beam, rear_barrel, front_connector, front_ear_left, front_ear_right, cheek_left, cheek_right)


def _upper_link_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XZ")
        .center((UPPER_LINK_LENGTH - 0.016) / 2.0, 0.0)
        .slot2D(UPPER_LINK_LENGTH - 0.032, LINK_BODY_HEIGHT)
        .extrude(LINK_BODY_WIDTH / 2.0, both=True)
    )
    rear_barrel = _y_cylinder(PIVOT_RADIUS, PIVOT_GAP, x=0.0, z=0.0)
    front_pad = cq.Workplane("XY").box(0.022, 0.026, 0.018).translate((UPPER_LINK_LENGTH - 0.011, 0.0, 0.0))
    pan_hub = (
        cq.Workplane("XY")
        .center(UPPER_LINK_LENGTH, 0.0)
        .circle(PAN_HUB_RADIUS)
        .extrude(PAN_HUB_THICKNESS)
        .translate((0.0, 0.0, -PAN_HUB_THICKNESS))
    )
    return _combine(beam, rear_barrel, front_pad, pan_hub)


def _pan_head_shape() -> cq.Workplane:
    arm_y = YOKE_GAP / 2.0 + EAR_THICKNESS / 2.0
    hole_radius = 0.008
    swivel_drum = cq.Workplane("YZ").circle(0.012).extrude(0.014)
    swivel_neck = cq.Workplane("XY").box(0.020, 0.024, 0.024).translate((0.010, 0.0, 0.012))
    rear_bridge = cq.Workplane("XY").box(0.018, YOKE_GAP + 2.0 * EAR_THICKNESS, 0.016).translate((0.020, 0.0, TILT_AXIS_Z))
    left_arm = cq.Workplane("XY").box(0.030, EAR_THICKNESS, 0.032).translate((0.034, arm_y, TILT_AXIS_Z))
    right_arm = cq.Workplane("XY").box(0.030, EAR_THICKNESS, 0.032).translate((0.034, -arm_y, TILT_AXIS_Z))
    yoke = _combine(swivel_drum, swivel_neck, rear_bridge, left_arm, right_arm)
    tilt_bore = _y_cylinder(hole_radius, YOKE_GAP + 0.040, x=TILT_AXIS_X, z=TILT_AXIS_Z)
    return yoke.cut(tilt_bore)


def _display_plate_shape() -> cq.Workplane:
    stub_length = EAR_THICKNESS
    stub_center_y = YOKE_GAP / 2.0 + stub_length / 2.0
    left_stub = _y_cylinder(0.008, stub_length, x=0.0, z=0.0, y=stub_center_y)
    right_stub = _y_cylinder(0.008, stub_length, x=0.0, z=0.0, y=-stub_center_y)
    left_cheek = cq.Workplane("XY").box(0.012, 0.090, 0.018).translate((0.010, 0.060, 0.0))
    right_cheek = cq.Workplane("XY").box(0.012, 0.090, 0.018).translate((0.010, -0.060, 0.0))
    center_spine = cq.Workplane("XY").box(0.016, 0.060, 0.060).translate((0.018, 0.0, 0.0))
    housing = cq.Workplane("XY").box(DISPLAY_DEPTH, DISPLAY_WIDTH, DISPLAY_HEIGHT).translate((0.030, 0.0, 0.0))
    back_plate = cq.Workplane("XY").box(0.004, 0.194, 0.122).translate((0.019, 0.0, 0.0))
    return _combine(left_stub, right_stub, left_cheek, right_cheek, center_spine, housing, back_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_bay_monitor_arm")

    model.material("powder_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_gray", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("screen_glass", rgba=(0.18, 0.28, 0.34, 0.90))

    back_bracket = model.part("back_bracket")
    back_bracket.visual(
        Box((0.010, BRACKET_WIDTH, BRACKET_HEIGHT)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material="powder_black",
        name="mount_plate",
    )
    ear_offset = PIVOT_GAP / 2.0 + EAR_THICKNESS / 2.0
    for side in (-1.0, 1.0):
        back_bracket.visual(
            Box((0.026, EAR_THICKNESS, 0.026)),
            origin=Origin(xyz=(-0.013, side * ear_offset, 0.0)),
            material="powder_black",
            name=f"bracket_arm_{'right' if side < 0.0 else 'left'}",
        )
        back_bracket.visual(
            Cylinder(radius=PIVOT_RADIUS, length=EAR_THICKNESS),
            origin=Origin(xyz=(0.0, side * ear_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="powder_black",
            name=f"bracket_ear_{'right' if side < 0.0 else 'left'}",
        )
    back_bracket.visual(
        Box((0.010, 0.044, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material="powder_black",
        name="pivot_spine",
    )
    back_bracket.inertial = Inertial.from_geometry(
        Box((0.032, BRACKET_WIDTH, BRACKET_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        Box((0.088, LINK_BODY_WIDTH, 0.012)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material="machined_gray",
        name="lower_beam",
    )
    lower_link.visual(
        Box((0.024, LINK_BODY_WIDTH, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="machined_gray",
        name="rear_neck",
    )
    lower_link.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_gray",
        name="rear_barrel",
    )
    lower_link.visual(
        Box((0.020, 0.024, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material="machined_gray",
        name="front_bridge",
    )
    for side in (-1.0, 1.0):
        lower_link.visual(
            Box((0.024, EAR_THICKNESS, 0.020)),
            origin=Origin(xyz=(0.106, side * ear_offset, 0.0)),
            material="machined_gray",
            name=f"front_rail_{'right' if side < 0.0 else 'left'}",
        )
        lower_link.visual(
            Cylinder(radius=PIVOT_RADIUS, length=EAR_THICKNESS),
            origin=Origin(xyz=(LOWER_LINK_LENGTH, side * ear_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="machined_gray",
            name=f"front_ear_{'right' if side < 0.0 else 'left'}",
        )
    lower_link.inertial = Inertial.from_geometry(
        Box((0.140, 0.030, 0.028)),
        mass=0.45,
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Box((0.082, LINK_BODY_WIDTH, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material="machined_gray",
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.022, LINK_BODY_WIDTH, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material="machined_gray",
        name="rear_neck",
    )
    upper_link.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_gray",
        name="rear_barrel",
    )
    upper_link.visual(
        Box((0.016, 0.020, 0.012)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material="machined_gray",
        name="swivel_block",
    )
    upper_link.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.112, 0.0, -0.002)),
        material="machined_gray",
        name="swivel_collar",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.135, 0.040, 0.030)),
        mass=0.42,
        origin=Origin(xyz=(0.058, 0.0, -0.002)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="powder_black",
        name="swivel_drum",
    )
    pan_head.visual(
        Box((0.018, 0.030, 0.020)),
        origin=Origin(xyz=(0.009, 0.0, 0.010)),
        material="powder_black",
        name="swivel_spine",
    )
    pan_head.visual(
        Box((0.024, YOKE_GAP + 0.012, 0.014)),
        origin=Origin(xyz=(0.016, 0.0, TILT_AXIS_Z)),
        material="powder_black",
        name="yoke_bridge",
    )
    arm_inner_gap = YOKE_GAP / 2.0
    arm_center_y = arm_inner_gap + EAR_THICKNESS / 2.0
    for side in (-1.0, 1.0):
        pan_head.visual(
            Box((0.016, EAR_THICKNESS, 0.034)),
            origin=Origin(xyz=(0.020, side * arm_center_y, TILT_AXIS_Z)),
            material="powder_black",
            name=f"yoke_arm_{'right' if side < 0.0 else 'left'}",
        )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.055, 0.242, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(0.026, 0.0, 0.016)),
    )

    display_plate = model.part("display_plate")
    display_plate.visual(
        Box((0.012, YOKE_GAP - 0.006, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="graphite",
        name="tilt_bar",
    )
    lug_center_y = YOKE_GAP / 2.0 - EAR_THICKNESS / 2.0
    for side in (-1.0, 1.0):
        display_plate.visual(
            Box((0.008, EAR_THICKNESS, 0.020)),
            origin=Origin(xyz=(0.0, side * lug_center_y, 0.0)),
            material="graphite",
            name=f"tilt_lug_{'right' if side < 0.0 else 'left'}",
        )
    display_plate.visual(
        Box((0.016, 0.070, 0.060)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material="graphite",
        name="mount_block",
    )
    display_plate.visual(
        Box((0.004, 0.194, 0.122)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material="machined_gray",
        name="back_plate",
    )
    display_plate.visual(
        Box((DISPLAY_DEPTH, DISPLAY_WIDTH, DISPLAY_HEIGHT)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material="graphite",
        name="display_body",
    )
    display_plate.visual(
        Box((SCREEN_THICKNESS, SCREEN_WIDTH, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material="screen_glass",
        name="screen_glass",
    )
    display_plate.inertial = Inertial.from_geometry(
        Box((0.030, 0.230, 0.140)),
        mass=0.85,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_to_lower",
        ArticulationType.REVOLUTE,
        parent=back_bracket,
        child=lower_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.95, effort=28.0, velocity=1.8),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=upper_link,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.15, effort=22.0, velocity=2.0),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=pan_head,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.60, upper=1.60, effort=10.0, velocity=2.2),
    )
    model.articulation(
        "pan_to_display",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=display_plate,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.80, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_bracket = object_model.get_part("back_bracket")
    lower_link = object_model.get_part("lower_link")
    upper_link = object_model.get_part("upper_link")
    pan_head = object_model.get_part("pan_head")
    display_plate = object_model.get_part("display_plate")

    shoulder = object_model.get_articulation("bracket_to_lower")
    elbow = object_model.get_articulation("lower_to_upper")
    swivel = object_model.get_articulation("upper_to_pan")
    tilt = object_model.get_articulation("pan_to_display")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(back_bracket, lower_link, name="back bracket supports lower link")
    ctx.expect_contact(lower_link, upper_link, name="lower link supports upper link")
    ctx.expect_contact(upper_link, pan_head, name="upper link supports pan head")
    ctx.expect_contact(pan_head, display_plate, name="pan yoke supports display plate")
    ctx.expect_origin_gap(
        display_plate,
        back_bracket,
        axis="x",
        min_gap=0.24,
        name="display plate sits forward of grounded bracket",
    )

    rest_pan = ctx.part_world_position(pan_head)
    rest_display = ctx.part_world_position(display_plate)
    rest_screen = ctx.part_element_world_aabb(display_plate, elem="screen_glass")

    with ctx.pose({shoulder: 0.55, elbow: 0.45}):
        raised_pan = ctx.part_world_position(pan_head)
        ctx.check(
            "arm links raise the head assembly",
            rest_pan is not None and raised_pan is not None and raised_pan[2] > rest_pan[2] + 0.09,
            f"expected pan head to rise from {rest_pan} to above it, got {raised_pan}",
        )

    with ctx.pose({swivel: 0.75}):
        turned_display = ctx.part_world_position(display_plate)
        ctx.check(
            "swivel pans the display laterally",
            rest_display is not None and turned_display is not None and turned_display[1] > rest_display[1] + 0.02,
            f"expected positive swivel to move display in +Y, got rest={rest_display}, posed={turned_display}",
        )

    with ctx.pose({tilt: 0.55}):
        tilted_screen = ctx.part_element_world_aabb(display_plate, elem="screen_glass")
        tilted_screen_center_z = None
        if tilted_screen is not None:
            tilted_screen_center_z = (tilted_screen[0][2] + tilted_screen[1][2]) / 2.0
        rest_screen_center_z = None
        if rest_screen is not None:
            rest_screen_center_z = (rest_screen[0][2] + rest_screen[1][2]) / 2.0
        ctx.check(
            "tilt joint tips the screen upward",
            rest_screen_center_z is not None
            and tilted_screen_center_z is not None
            and tilted_screen_center_z > rest_screen_center_z + 0.01,
            (
                "expected positive tilt to raise the forward-offset screen center, "
                f"got rest_z={rest_screen_center_z}, posed_z={tilted_screen_center_z}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

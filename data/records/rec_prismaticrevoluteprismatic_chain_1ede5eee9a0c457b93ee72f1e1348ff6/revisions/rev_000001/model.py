from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.018
BASE_STIFFENER_LENGTH = 0.28
BASE_STIFFENER_WIDTH = 0.08
BASE_STIFFENER_HEIGHT = 0.010

RAIL_PAD_LENGTH = 0.31
RAIL_PAD_WIDTH = 0.028
RAIL_PAD_HEIGHT = 0.008
RAIL_LENGTH = 0.29
RAIL_WIDTH = 0.014
RAIL_HEIGHT = 0.010
RAIL_Y = 0.036

CARRIAGE_LENGTH = 0.14
SLEEVE_WIDTH = 0.034
SLEEVE_HEIGHT = 0.034
SLEEVE_GROOVE_WIDTH = 0.016
SLEEVE_GROOVE_HEIGHT = 0.016
REAR_BRIDGE_LENGTH = 0.086
REAR_BRIDGE_WIDTH = 0.088
REAR_BRIDGE_HEIGHT = 0.018
FRONT_BRIDGE_LENGTH = 0.036
FRONT_BRIDGE_WIDTH = 0.060
FRONT_BRIDGE_HEIGHT = 0.014
PIVOT_X = 0.044
PIVOT_Z = 0.083
EAR_THICKNESS = 0.012
EAR_HEIGHT = 0.058
EAR_DEPTH = 0.036
EAR_CENTER_Y = 0.036

FRAME_BODY_LENGTH = 0.112
FRAME_BODY_WIDTH = 0.044
FRAME_BODY_HEIGHT = 0.076
FRAME_BODY_X = 0.046
FRAME_BODY_Z = 0.014
FRAME_WINDOW_LENGTH = 0.070
FRAME_WINDOW_WIDTH = 0.028
FRAME_WINDOW_HEIGHT = 0.046
FRAME_WINDOW_X = 0.034
FRAME_WINDOW_Z = 0.012
GUIDE_HOUSING_LENGTH = 0.090
GUIDE_HOUSING_WIDTH = 0.052
GUIDE_HOUSING_HEIGHT = 0.050
GUIDE_HOUSING_X = 0.135
GUIDE_HOUSING_Z = 0.0
GUIDE_CAVITY_LENGTH = 0.096
GUIDE_CAVITY_WIDTH = 0.040
GUIDE_CAVITY_HEIGHT = 0.028
GUIDE_CAVITY_Z = 0.004
GUIDE_CENTER_RELIEF_WIDTH = 0.022
GUIDE_CENTER_RELIEF_HEIGHT = 0.006
GUIDE_CENTER_RELIEF_Z = -0.013
TRUNNION_RADIUS = 0.010
TRUNNION_LENGTH = 0.060
TRUNNION_COLLAR_RADIUS = 0.016
TRUNNION_COLLAR_LENGTH = 0.004
TRUNNION_COLLAR_OFFSET = 0.028

RAM_LENGTH = 0.078
RAM_WIDTH = 0.018
RAM_HEIGHT = 0.014
RAM_NOSE_LENGTH = 0.016
RAM_NOSE_WIDTH = 0.018
RAM_NOSE_HEIGHT = 0.014
RAM_NOSE_X = 0.086
RAM_GUIDE_ORIGIN_X = 0.124
RAM_GUIDE_ORIGIN_Z = 0.0

SLIDE_LOWER = -0.085
SLIDE_UPPER = 0.085
HINGE_LOWER = 0.0
HINGE_UPPER = 1.00
RAM_LOWER = 0.0
RAM_UPPER = 0.045


def _box_center(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((x, y, z))


def _box_bottom(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False)).translate((x, y, z))


def _cylinder_y(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -(length / 2.0)))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def _base_shape() -> cq.Workplane:
    base_plate = _box_bottom(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    base_plate = base_plate.edges("|Z").fillet(0.003)

    stiffener = _box_bottom(BASE_STIFFENER_LENGTH, BASE_STIFFENER_WIDTH, BASE_STIFFENER_HEIGHT)

    rail_pad_left = _box_bottom(RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT, y=RAIL_Y, z=BASE_THICKNESS)
    rail_pad_right = _box_bottom(RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT, y=-RAIL_Y, z=BASE_THICKNESS)

    rail_left = _box_bottom(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, y=RAIL_Y, z=BASE_THICKNESS + RAIL_PAD_HEIGHT)
    rail_right = _box_bottom(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, y=-RAIL_Y, z=BASE_THICKNESS + RAIL_PAD_HEIGHT)

    center_cover = _box_bottom(0.110, 0.040, 0.014, z=BASE_THICKNESS)
    end_pad_left = _box_bottom(0.026, 0.018, 0.014, x=0.150, y=0.056, z=BASE_THICKNESS)
    end_pad_right = _box_bottom(0.026, 0.018, 0.014, x=-0.150, y=-0.056, z=BASE_THICKNESS)

    shape = (
        base_plate
        .union(stiffener)
        .union(rail_pad_left)
        .union(rail_pad_right)
        .union(rail_left)
        .union(rail_right)
        .union(center_cover)
        .union(end_pad_left)
        .union(end_pad_right)
    )

    side_relief_left = _box_bottom(0.220, 0.022, 0.0065, y=0.054, z=BASE_THICKNESS - 0.0005)
    side_relief_right = _box_bottom(0.220, 0.022, 0.0065, y=-0.054, z=BASE_THICKNESS - 0.0005)
    center_relief = _box_bottom(0.090, 0.030, 0.004, z=BASE_THICKNESS + 0.004)

    return shape.cut(side_relief_left).cut(side_relief_right).cut(center_relief)


def _carriage_shape() -> cq.Workplane:
    sleeve_left = _box_bottom(CARRIAGE_LENGTH, SLEEVE_WIDTH, SLEEVE_HEIGHT, y=RAIL_Y)
    sleeve_right = _box_bottom(CARRIAGE_LENGTH, SLEEVE_WIDTH, SLEEVE_HEIGHT, y=-RAIL_Y)

    groove_left = _box_bottom(CARRIAGE_LENGTH - 0.010, SLEEVE_GROOVE_WIDTH, SLEEVE_GROOVE_HEIGHT, y=RAIL_Y)
    groove_right = _box_bottom(CARRIAGE_LENGTH - 0.010, SLEEVE_GROOVE_WIDTH, SLEEVE_GROOVE_HEIGHT, y=-RAIL_Y)

    sleeve_left = sleeve_left.cut(groove_left)
    sleeve_right = sleeve_right.cut(groove_right)

    rear_bridge = _box_bottom(REAR_BRIDGE_LENGTH, REAR_BRIDGE_WIDTH, REAR_BRIDGE_HEIGHT, x=-0.010, z=SLEEVE_HEIGHT)
    front_bridge = _box_bottom(0.044, 0.054, 0.012, x=0.026, z=SLEEVE_HEIGHT)
    center_web = _box_bottom(0.050, 0.022, 0.014, x=0.012, z=0.022)

    ear_left = _box_bottom(0.024, EAR_THICKNESS, EAR_HEIGHT, x=PIVOT_X, y=EAR_CENTER_Y, z=SLEEVE_HEIGHT)
    ear_right = _box_bottom(0.024, EAR_THICKNESS, EAR_HEIGHT, x=PIVOT_X, y=-EAR_CENTER_Y, z=SLEEVE_HEIGHT)
    ear_bridge = _box_bottom(0.020, 0.020, 0.012, x=PIVOT_X - 0.010, z=0.060)

    gusset_left = _box_bottom(0.054, 0.010, 0.028, x=0.018, y=0.026, z=SLEEVE_HEIGHT)
    gusset_right = _box_bottom(0.054, 0.010, 0.028, x=0.018, y=-0.026, z=SLEEVE_HEIGHT)

    carriage = (
        sleeve_left
        .union(sleeve_right)
        .union(rear_bridge)
        .union(front_bridge)
        .union(center_web)
        .union(ear_left)
        .union(ear_right)
        .union(ear_bridge)
        .union(gusset_left)
        .union(gusset_right)
    )

    top_window = _box_bottom(0.052, 0.050, 0.012, x=0.020, z=SLEEVE_HEIGHT + 0.006)
    ear_hole_left = _cylinder_y(0.008, EAR_THICKNESS + 0.004, x=PIVOT_X, y=EAR_CENTER_Y, z=PIVOT_Z)
    ear_hole_right = _cylinder_y(0.008, EAR_THICKNESS + 0.004, x=PIVOT_X, y=-EAR_CENTER_Y, z=PIVOT_Z)

    return carriage.cut(top_window).cut(ear_hole_left).cut(ear_hole_right)


def _hinge_frame_shape() -> cq.Workplane:
    left_journal = _cylinder_y(0.008, 0.012, x=0.0, y=0.036, z=0.0)
    right_journal = _cylinder_y(0.008, 0.012, x=0.0, y=-0.036, z=0.0)

    lift_bridge = _box_bottom(0.020, 0.016, 0.008, x=0.030, z=0.014)
    backbone = _box_bottom(0.084, 0.016, 0.010, x=0.096, z=0.016)
    rib_left = _box_bottom(0.064, 0.006, 0.012, x=0.118, y=0.010, z=0.010)
    rib_right = _box_bottom(0.064, 0.006, 0.012, x=0.118, y=-0.010, z=0.010)

    guide_outer = _box_bottom(0.108, 0.030, 0.024, x=0.170, z=0.000)
    guide_cavity = _box_bottom(0.096, 0.022, 0.016, x=0.170, z=0.004)
    rear_access = _box_bottom(0.018, 0.024, 0.016, x=0.118, z=0.004)
    guide = guide_outer.cut(guide_cavity).cut(rear_access)

    return (
        left_journal
        .union(right_journal)
        .union(lift_bridge)
        .union(backbone)
        .union(rib_left)
        .union(rib_right)
        .union(guide)
    )


def _ram_shape() -> cq.Workplane:
    main_bar = _box_bottom(RAM_LENGTH, RAM_WIDTH, RAM_HEIGHT, x=RAM_LENGTH / 2.0, z=0.005)
    nose = _box_bottom(RAM_NOSE_LENGTH, RAM_NOSE_WIDTH, RAM_NOSE_HEIGHT, x=RAM_NOSE_X, z=0.005)
    nose = nose.faces(">X").edges("|Z").chamfer(0.004)
    guide_shoe_left = _box_bottom(0.040, 0.002, 0.004, x=0.028, y=0.008, z=0.003)
    guide_shoe_right = _box_bottom(0.040, 0.002, 0.004, x=0.028, y=-0.008, z=0.003)
    return main_bar.union(nose).union(guide_shoe_left).union(guide_shoe_right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_axis")

    model.material("painted_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("ground_rail", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("anodized_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tool_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    base_stage = model.part("base_stage")
    base_stage.visual(
        mesh_from_cadquery(_base_shape(), "service_axis_base"),
        material="painted_steel",
        name="base_shell",
    )
    base_stage.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + RAIL_PAD_HEIGHT + RAIL_HEIGHT)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + RAIL_PAD_HEIGHT + RAIL_HEIGHT) / 2.0)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "service_axis_carriage"),
        material="anodized_dark",
        name="carriage_shell",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, 0.096, PIVOT_Z + 0.020)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    hinge_frame = model.part("hinge_frame")
    hinge_frame.visual(
        mesh_from_cadquery(_hinge_frame_shape(), "service_axis_hinge_frame"),
        material="machined_aluminum",
        name="frame_shell",
    )
    hinge_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.05, 0.05)),
        mass=1.4,
        origin=Origin(xyz=(0.11, 0.0, 0.022)),
    )

    terminal_ram = model.part("terminal_ram")
    terminal_ram.visual(
        mesh_from_cadquery(_ram_shape(), "service_axis_ram"),
        material="tool_steel",
        name="ram_shell",
    )
    terminal_ram.inertial = Inertial.from_geometry(
        Box((RAM_LENGTH + RAM_NOSE_LENGTH, RAM_NOSE_WIDTH, RAM_NOSE_HEIGHT)),
        mass=0.52,
        origin=Origin(xyz=((RAM_LENGTH + RAM_NOSE_LENGTH) / 2.0, 0.0, RAM_NOSE_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base_stage,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + RAIL_PAD_HEIGHT + RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=500.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_hinge",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=hinge_frame,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=HINGE_LOWER,
            upper=HINGE_UPPER,
            effort=40.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "ram_extension",
        ArticulationType.PRISMATIC,
        parent=hinge_frame,
        child=terminal_ram,
        origin=Origin(xyz=(RAM_GUIDE_ORIGIN_X, 0.0, RAM_GUIDE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=RAM_LOWER,
            upper=RAM_UPPER,
            effort=90.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_stage = object_model.get_part("base_stage")
    slide_carriage = object_model.get_part("slide_carriage")
    hinge_frame = object_model.get_part("hinge_frame")
    terminal_ram = object_model.get_part("terminal_ram")
    base_slide = object_model.get_articulation("base_slide")
    carriage_hinge = object_model.get_articulation("carriage_hinge")
    ram_extension = object_model.get_articulation("ram_extension")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        slide_carriage,
        hinge_frame,
        reason="captured trunnion journals are represented with simplified hidden bushing envelopes inside the clevis ears",
    )
    ctx.allow_overlap(
        hinge_frame,
        terminal_ram,
        reason="the enclosed ram guide is modeled as a tight hidden sleeve around the slider rather than exposing internal running clearance",
    )

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

    axes_ok = (
        base_slide.axis == (1.0, 0.0, 0.0)
        and carriage_hinge.axis == (0.0, -1.0, 0.0)
        and ram_extension.axis == (1.0, 0.0, 0.0)
    )
    limits_ok = (
        base_slide.motion_limits is not None
        and carriage_hinge.motion_limits is not None
        and ram_extension.motion_limits is not None
        and isclose(base_slide.motion_limits.lower or 0.0, SLIDE_LOWER)
        and isclose(base_slide.motion_limits.upper or 0.0, SLIDE_UPPER)
        and isclose(carriage_hinge.motion_limits.lower or 0.0, HINGE_LOWER)
        and isclose(carriage_hinge.motion_limits.upper or 0.0, HINGE_UPPER)
        and isclose(ram_extension.motion_limits.lower or 0.0, RAM_LOWER)
        and isclose(ram_extension.motion_limits.upper or 0.0, RAM_UPPER)
    )
    ctx.check(
        "joint_sequence_matches_slide_hinge_ram_prompt",
        axes_ok and limits_ok,
        details=(
            f"axes: slide={base_slide.axis}, hinge={carriage_hinge.axis}, ram={ram_extension.axis}; "
            f"limits: slide=({base_slide.motion_limits.lower if base_slide.motion_limits else None}, "
            f"{base_slide.motion_limits.upper if base_slide.motion_limits else None}), "
            f"hinge=({carriage_hinge.motion_limits.lower if carriage_hinge.motion_limits else None}, "
            f"{carriage_hinge.motion_limits.upper if carriage_hinge.motion_limits else None}), "
            f"ram=({ram_extension.motion_limits.lower if ram_extension.motion_limits else None}, "
            f"{ram_extension.motion_limits.upper if ram_extension.motion_limits else None})"
        ),
    )

    ctx.expect_contact(slide_carriage, base_stage, name="carriage_bears_on_machined_rail_pads")
    ctx.expect_contact(hinge_frame, slide_carriage, name="hinge_frame_seated_in_trunnion_supports")
    ctx.expect_contact(terminal_ram, hinge_frame, name="ram_supported_by_nose_guide")

    ctx.expect_overlap(
        slide_carriage,
        base_stage,
        axes="xy",
        min_overlap=0.08,
        name="slide_stage_stays_visibly_grounded_over_base",
    )
    ctx.expect_origin_gap(
        hinge_frame,
        slide_carriage,
        axis="z",
        min_gap=0.07,
        max_gap=0.09,
        name="hinge_axis_rides_above_carriage_structure",
    )
    ctx.expect_overlap(
        terminal_ram,
        hinge_frame,
        axes="yz",
        min_overlap=0.013,
        name="ram_cross_section_stays_captured_by_local_guide",
    )

    with ctx.pose({base_slide: SLIDE_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_slide_limit_clearance")
    with ctx.pose({base_slide: SLIDE_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="forward_slide_limit_clearance")
    with ctx.pose({base_slide: 0.060, carriage_hinge: 0.88, ram_extension: RAM_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_motion_clearance")
        ctx.expect_overlap(
            terminal_ram,
            hinge_frame,
            axes="yz",
            min_overlap=0.013,
            name="ram_remains_guided_when_extended_and_pitched",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

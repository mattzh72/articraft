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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OUTER_WIDTH = 0.340
OUTER_HEIGHT = 0.240
FRAME_DEPTH = 0.045
FRONT_PLATE_THICKNESS = 0.003
MAIN_DEPTH = FRAME_DEPTH - FRONT_PLATE_THICKNESS
SIDE_RAIL_WIDTH = 0.028
TOP_RAIL_HEIGHT = 0.034
INNER_WIDTH = OUTER_WIDTH - 2.0 * SIDE_RAIL_WIDTH
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * TOP_RAIL_HEIGHT
LOUVER_PITCH = 0.038
LOUVER_CENTERS_Y = (0.057, 0.019, -0.019, -0.057)
LOUVER_AXIS_Z = 0.024
LOUVER_CHORD = 0.033
LOUVER_THICKNESS = 0.006
LOUVER_BODY_SPAN = 0.252
LOUVER_HUB_LENGTH = 0.008
LOUVER_HUB_RADIUS = 0.006
LOUVER_PIN_LENGTH = 0.008
LOUVER_PIN_RADIUS = 0.0035


def _louver_profile(chord: float, thickness: float) -> list[tuple[float, float]]:
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (0.0, -half_chord),
        (half_thickness * 0.45, -half_chord * 0.74),
        (half_thickness * 0.92, -half_chord * 0.22),
        (half_thickness * 0.72, half_chord * 0.34),
        (half_thickness * 0.25, half_chord),
        (-half_thickness * 0.28, half_chord * 0.62),
        (-half_thickness * 0.82, -half_chord * 0.10),
        (-half_thickness * 0.42, -half_chord * 0.72),
    ]


def _make_louver_mesh(name: str) -> object:
    geom = ExtrudeGeometry(
        _louver_profile(LOUVER_CHORD, LOUVER_THICKNESS),
        LOUVER_BODY_SPAN,
        cap=True,
        center=True,
        closed=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_shutter_insert")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.83, 0.85, 0.87, 1.0))
    frame_shadow = model.material("frame_shadow", rgba=(0.62, 0.65, 0.68, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.76, 0.79, 0.82, 1.0))
    linkage_dark = model.material("linkage_dark", rgba=(0.25, 0.27, 0.30, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.11, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, OUTER_HEIGHT, FRAME_DEPTH)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, FRAME_DEPTH * 0.5)),
    )

    main_center_z = FRONT_PLATE_THICKNESS + MAIN_DEPTH * 0.5
    right_strip_center_x = OUTER_WIDTH * 0.5 - 0.024
    right_outer_center_x = OUTER_WIDTH * 0.5 - 0.010

    frame.visual(
        Box((SIDE_RAIL_WIDTH, OUTER_HEIGHT, MAIN_DEPTH)),
        origin=Origin(xyz=(-OUTER_WIDTH * 0.5 + SIDE_RAIL_WIDTH * 0.5, 0.0, main_center_z)),
        material=frame_aluminum,
        name="left_rail",
    )
    frame.visual(
        Box((INNER_WIDTH, TOP_RAIL_HEIGHT, MAIN_DEPTH)),
        origin=Origin(xyz=(0.0, OUTER_HEIGHT * 0.5 - TOP_RAIL_HEIGHT * 0.5, main_center_z)),
        material=frame_aluminum,
        name="top_rail",
    )
    frame.visual(
        Box((INNER_WIDTH, TOP_RAIL_HEIGHT, MAIN_DEPTH)),
        origin=Origin(xyz=(0.0, -OUTER_HEIGHT * 0.5 + TOP_RAIL_HEIGHT * 0.5, main_center_z)),
        material=frame_aluminum,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.008, OUTER_HEIGHT, FRAME_DEPTH - 0.014)),
        origin=Origin(xyz=(right_strip_center_x, 0.0, 0.014 + (FRAME_DEPTH - 0.014) * 0.5)),
        material=frame_shadow,
        name="right_inner_strip",
    )
    frame.visual(
        Box((0.020, 0.100, MAIN_DEPTH)),
        origin=Origin(xyz=(right_outer_center_x, 0.070, main_center_z)),
        material=frame_aluminum,
        name="right_outer_upper",
    )
    frame.visual(
        Box((0.020, 0.100, MAIN_DEPTH)),
        origin=Origin(xyz=(right_outer_center_x, -0.070, main_center_z)),
        material=frame_aluminum,
        name="right_outer_lower",
    )

    frame.visual(
        Box((OUTER_WIDTH, 0.018, FRONT_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, OUTER_HEIGHT * 0.5 - 0.009, FRONT_PLATE_THICKNESS * 0.5)),
        material=frame_shadow,
        name="top_flange",
    )
    frame.visual(
        Box((OUTER_WIDTH, 0.018, FRONT_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, -OUTER_HEIGHT * 0.5 + 0.009, FRONT_PLATE_THICKNESS * 0.5)),
        material=frame_shadow,
        name="bottom_flange",
    )
    frame.visual(
        Box((0.018, OUTER_HEIGHT - 0.036, FRONT_PLATE_THICKNESS)),
        origin=Origin(xyz=(-OUTER_WIDTH * 0.5 + 0.009, 0.0, FRONT_PLATE_THICKNESS * 0.5)),
        material=frame_shadow,
        name="left_flange",
    )
    frame.visual(
        Box((0.018, 0.070, FRONT_PLATE_THICKNESS)),
        origin=Origin(xyz=(OUTER_WIDTH * 0.5 - 0.009, 0.067, FRONT_PLATE_THICKNESS * 0.5)),
        material=frame_shadow,
        name="right_upper_flange",
    )
    frame.visual(
        Box((0.018, 0.070, FRONT_PLATE_THICKNESS)),
        origin=Origin(xyz=(OUTER_WIDTH * 0.5 - 0.009, -0.067, FRONT_PLATE_THICKNESS * 0.5)),
        material=frame_shadow,
        name="right_lower_flange",
    )

    frame.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(0.156, 0.014, 0.005)),
        material=frame_shadow,
        name="knob_bushing_upper",
    )
    frame.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(0.156, -0.014, 0.005)),
        material=frame_shadow,
        name="knob_bushing_lower",
    )

    for prefix, guide_y in (("upper", 0.068), ("lower", -0.068)):
        frame.visual(
            Box((0.008, 0.018, 0.002)),
            origin=Origin(xyz=(0.136, guide_y, 0.006)),
            material=frame_shadow,
            name=f"linkage_{prefix}_guide_front",
        )
        frame.visual(
            Box((0.008, 0.018, 0.003)),
            origin=Origin(xyz=(0.136, guide_y, 0.0125)),
            material=frame_shadow,
            name=f"linkage_{prefix}_guide_back",
        )
        frame.visual(
            Box((0.002, 0.018, 0.009)),
            origin=Origin(xyz=(0.141, guide_y, 0.0095)),
            material=frame_shadow,
            name=f"linkage_{prefix}_guide_rib",
        )

    for index, center_y in enumerate(LOUVER_CENTERS_Y, start=1):
        frame.visual(
            Box((0.007, 0.014, 0.004)),
            origin=Origin(xyz=(-0.1385, center_y, 0.016)),
            material=frame_shadow,
            name=f"left_clip_{index}",
        )
        frame.visual(
            Box((0.007, 0.014, 0.004)),
            origin=Origin(xyz=(0.1385, center_y, 0.016)),
            material=frame_shadow,
            name=f"right_clip_{index}",
        )

    linkage_bar = model.part("linkage_bar")
    linkage_bar.visual(
        Box((0.006, 0.156, 0.004)),
        origin=Origin(),
        material=linkage_dark,
        name="bar",
    )
    for index, center_y in enumerate(LOUVER_CENTERS_Y, start=1):
        linkage_bar.visual(
            Box((0.006, 0.008, 0.004)),
            origin=Origin(xyz=(-0.006, center_y, 0.0)),
            material=linkage_dark,
            name=f"paddle_{index}",
        )
    linkage_bar.visual(
        Box((0.008, 0.018, 0.004)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=linkage_dark,
        name="drive_tab",
    )
    linkage_bar.inertial = Inertial.from_geometry(
        Box((0.008, 0.156, 0.006)),
        mass=0.05,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_linkage_bar",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=linkage_bar,
        origin=Origin(xyz=(0.136, 0.0, 0.009)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.05,
            lower=-0.010,
            upper=0.010,
        ),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=knob_black,
        name="grip",
    )
    control_knob.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=linkage_dark,
        name="collar",
    )
    control_knob.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=linkage_dark,
        name="shaft",
    )
    control_knob.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(-0.001, 0.0, 0.010)),
        material=linkage_dark,
        name="drive_arm",
    )
    control_knob.visual(
        Box((0.008, 0.010, 0.003)),
        origin=Origin(xyz=(-0.005, 0.0, 0.010)),
        material=linkage_dark,
        name="drive_pin",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
    )
    model.articulation(
        "frame_to_knob",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=control_knob,
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    louver_mesh = _make_louver_mesh("vent_louver_blade")
    hub_center_x = LOUVER_BODY_SPAN * 0.5 + LOUVER_HUB_LENGTH * 0.5
    pin_center_x = LOUVER_BODY_SPAN * 0.5 + LOUVER_HUB_LENGTH + LOUVER_PIN_LENGTH * 0.5
    for index, center_y in enumerate(LOUVER_CENTERS_Y, start=1):
        louver = model.part(f"louver_{index}")
        louver.visual(
            louver_mesh,
            material=blade_finish,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=LOUVER_HUB_RADIUS, length=LOUVER_HUB_LENGTH),
            origin=Origin(xyz=(-hub_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_shadow,
            name="left_hub",
        )
        louver.visual(
            Cylinder(radius=LOUVER_HUB_RADIUS, length=LOUVER_HUB_LENGTH),
            origin=Origin(xyz=(hub_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_shadow,
            name="right_hub",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(xyz=(-pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=linkage_dark,
            name="left_pin",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(xyz=(pin_center_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=linkage_dark,
            name="right_pin",
        )
        louver.visual(
            Box((0.010, 0.004, 0.010)),
            origin=Origin(xyz=(0.121, 0.0, -0.010)),
            material=linkage_dark,
            name="drive_ear",
        )
        louver.inertial = Inertial.from_geometry(
            Box((INNER_WIDTH, LOUVER_CHORD, 0.020)),
            mass=0.08,
            origin=Origin(),
        )
        model.articulation(
            f"frame_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, center_y, LOUVER_AXIS_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=1.2,
                lower=-0.55,
                upper=0.70,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    linkage_bar = object_model.get_part("linkage_bar")
    control_knob = object_model.get_part("control_knob")
    knob_joint = object_model.get_articulation("frame_to_knob")
    linkage_joint = object_model.get_articulation("frame_to_linkage_bar")
    louvers = [object_model.get_part(f"louver_{index}") for index in range(1, 5)]
    louver_joints = [object_model.get_articulation(f"frame_to_louver_{index}") for index in range(1, 5)]

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

    ctx.check(
        "knob_rotates_about_front_axis",
        tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
        f"expected knob axis (0, 0, 1), got {knob_joint.axis}",
    )
    ctx.check(
        "linkage_slides_vertically",
        tuple(linkage_joint.axis) == (0.0, 1.0, 0.0),
        f"expected linkage axis (0, 1, 0), got {linkage_joint.axis}",
    )
    for index, joint in enumerate(louver_joints, start=1):
        ctx.check(
            f"louver_{index}_rotates_horizontally",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"expected louver axis (1, 0, 0), got {joint.axis}",
        )

    rest_pose = {knob_joint: 0.0, linkage_joint: 0.0}
    for joint in louver_joints:
        rest_pose[joint] = 0.0

    with ctx.pose(rest_pose):
        ctx.expect_contact(control_knob, frame, elem_a="collar", name="knob_seats_on_bushing")
        ctx.expect_contact(control_knob, linkage_bar, elem_a="drive_pin", elem_b="drive_tab", name="knob_drives_linkage")
        ctx.expect_contact(linkage_bar, frame, name="linkage_guided_by_frame")
        ctx.expect_gap(
            frame,
            linkage_bar,
            axis="x",
            positive_elem="right_inner_strip",
            negative_elem="bar",
            min_gap=0.0025,
            max_gap=0.0045,
            name="linkage_runs_just_inside_right_rail",
        )

        for index, louver in enumerate(louvers, start=1):
            ctx.expect_contact(louver, frame, elem_a="left_pin", name=f"louver_{index}_left_pin_captured")
            ctx.expect_contact(louver, frame, elem_a="right_pin", name=f"louver_{index}_right_pin_captured")
            ctx.expect_gap(
                linkage_bar,
                louver,
                axis="x",
                positive_elem=f"paddle_{index}",
                negative_elem="drive_ear",
                min_gap=0.0005,
                max_gap=0.0020,
                name=f"louver_{index}_ear_aligned_to_linkage",
            )

        for index in range(len(louvers) - 1):
            ctx.expect_gap(
                louvers[index],
                louvers[index + 1],
                axis="y",
                min_gap=0.0035,
                max_gap=0.0065,
                name=f"louver_gap_{index + 1}_{index + 2}",
            )

    open_pose = {knob_joint: 0.85, linkage_joint: 0.006}
    for joint in louver_joints:
        open_pose[joint] = 0.55

    with ctx.pose(open_pose):
        ctx.expect_gap(
            frame,
            louvers[0],
            axis="y",
            positive_elem="top_rail",
            min_gap=0.010,
            max_gap=0.020,
            name="top_louver_clears_top_rail_open",
        )
        ctx.expect_gap(
            louvers[-1],
            frame,
            axis="y",
            negative_elem="bottom_rail",
            min_gap=0.010,
            max_gap=0.020,
            name="bottom_louver_clears_bottom_rail_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

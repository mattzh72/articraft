from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


LINK1_LENGTH = 0.340
LINK2_LENGTH = 0.280

BRACKET_EAR_THICKNESS = 0.010
BRACKET_EAR_OFFSET = 0.060
BRACKET_INNER_SPAN = 2.0 * (BRACKET_EAR_OFFSET - BRACKET_EAR_THICKNESS / 2.0)

LINK1_PLATE_THICKNESS = 0.008
LINK1_PLATE_OFFSET = 0.040
LINK1_PLATE_HEIGHT = 0.058
LINK1_INNER_SPAN = 2.0 * (LINK1_PLATE_OFFSET - LINK1_PLATE_THICKNESS / 2.0)

LINK2_PLATE_THICKNESS = 0.007
LINK2_PLATE_OFFSET = 0.024
LINK2_PLATE_HEIGHT = 0.048
LINK2_INNER_SPAN = 2.0 * (LINK2_PLATE_OFFSET - LINK2_PLATE_THICKNESS / 2.0)

SHOULDER_RADIUS = 0.018
ELBOW_RADIUS = 0.015

END_TAB_LENGTH = 0.085
END_TAB_THICKNESS = 0.016
END_TAB_HEIGHT = 0.032
END_TAB_ROOT_LENGTH = 0.018
LINK2_TIP_BRIDGE_LENGTH = 0.032


def _capsule_plate(
    *,
    length: float,
    thickness: float,
    height: float,
    y_offset: float,
    window_length: float,
    window_height: float,
    window_x: float,
) -> cq.Workplane:
    core_length = max(length - height, 0.001)
    cap_center_x = height / 2.0

    core = (
        cq.Workplane("XY")
        .box(core_length, thickness, height)
        .translate((length / 2.0, y_offset, 0.0))
    )
    root_cap = (
        cq.Workplane("XZ")
        .circle(height / 2.0)
        .extrude(thickness / 2.0, both=True)
        .translate((cap_center_x, y_offset, 0.0))
    )
    tip_cap = (
        cq.Workplane("XZ")
        .circle(height / 2.0)
        .extrude(thickness / 2.0, both=True)
        .translate((length - cap_center_x, y_offset, 0.0))
    )

    plate = core.union(root_cap).union(tip_cap)

    if window_length > 0.0 and window_height > 0.0:
        window = (
            cq.Workplane("XY")
            .box(window_length, thickness * 3.0, window_height)
            .translate((window_x, y_offset, 0.0))
        )
        plate = plate.cut(window)

    return plate


def _make_root_bracket_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(0.090, 0.160, 0.014)
        .translate((-0.045, 0.0, -0.103))
    )
    rear_web = (
        cq.Workplane("XY")
        .box(0.030, 0.146, 0.102)
        .translate((-0.045, 0.0, -0.045))
    )
    crown = (
        cq.Workplane("XY")
        .box(0.018, 0.130, 0.020)
        .translate((-0.009, 0.0, 0.010))
    )
    pivot_bar = (
        cq.Workplane("XY")
        .box(0.020, 0.118, 0.022)
        .translate((-0.010, 0.0, 0.000))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.074)
        .translate((-0.010, 0.053, -0.007))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.074)
        .translate((-0.010, -0.053, -0.007))
    )
    shoulder_pad = (
        cq.Workplane("XY")
        .box(0.028, 0.070, 0.030)
        .translate((-0.024, 0.0, -0.012))
    )

    return (
        base_plate
        .union(rear_web)
        .union(crown)
        .union(pivot_bar)
        .union(left_ear)
        .union(right_ear)
        .union(shoulder_pad)
        .edges("|Y and >X")
        .fillet(0.004)
    )


def _make_link1_shape() -> cq.Workplane:
    root_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.096, 0.022)
        .translate((0.011, 0.0, 0.0))
    )
    left_plate = (
        cq.Workplane("XY")
        .box(0.292, LINK1_PLATE_THICKNESS, 0.048)
        .translate((0.164, LINK1_PLATE_OFFSET, 0.0))
    )
    right_plate = (
        cq.Workplane("XY")
        .box(0.292, LINK1_PLATE_THICKNESS, 0.048)
        .translate((0.164, -LINK1_PLATE_OFFSET, 0.0))
    )
    tip_bridge = (
        cq.Workplane("XY")
        .box(0.032, 0.072, 0.020)
        .translate((LINK1_LENGTH - 0.016, 0.0, 0.0))
    )
    left_window = (
        cq.Workplane("XY")
        .box(0.172, LINK1_PLATE_THICKNESS * 3.0, 0.018)
        .translate((0.178, LINK1_PLATE_OFFSET, 0.0))
    )
    right_window = (
        cq.Workplane("XY")
        .box(0.172, LINK1_PLATE_THICKNESS * 3.0, 0.018)
        .translate((0.178, -LINK1_PLATE_OFFSET, 0.0))
    )

    return (
        root_bridge
        .union(left_plate)
        .union(right_plate)
        .union(tip_bridge)
        .cut(left_window)
        .cut(right_window)
        .edges("|Y and >X")
        .fillet(0.003)
    )


def _make_link2_shape() -> cq.Workplane:
    root_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.072, 0.020)
        .translate((0.011, 0.0, 0.0))
    )
    left_plate = (
        cq.Workplane("XY")
        .box(0.202, LINK2_PLATE_THICKNESS, 0.040)
        .translate((0.119, LINK2_PLATE_OFFSET, 0.0))
    )
    right_plate = (
        cq.Workplane("XY")
        .box(0.202, LINK2_PLATE_THICKNESS, 0.040)
        .translate((0.119, -LINK2_PLATE_OFFSET, 0.0))
    )
    tip_block = (
        cq.Workplane("XY")
        .box(0.062, LINK2_INNER_SPAN, 0.024)
        .translate((LINK2_LENGTH - 0.031, 0.0, 0.0))
    )
    left_window = (
        cq.Workplane("XY")
        .box(0.112, LINK2_PLATE_THICKNESS * 3.0, 0.014)
        .translate((0.134, LINK2_PLATE_OFFSET, 0.0))
    )
    right_window = (
        cq.Workplane("XY")
        .box(0.112, LINK2_PLATE_THICKNESS * 3.0, 0.014)
        .translate((0.134, -LINK2_PLATE_OFFSET, 0.0))
    )

    return (
        root_bridge
        .union(left_plate)
        .union(right_plate)
        .union(tip_block)
        .cut(left_window)
        .cut(right_window)
        .edges("|Y and >X")
        .fillet(0.0025)
    )


def _make_end_tab_shape() -> cq.Workplane:
    root_block = (
        cq.Workplane("XY")
        .box(END_TAB_ROOT_LENGTH, LINK2_INNER_SPAN, 0.028)
        .translate((END_TAB_ROOT_LENGTH / 2.0, 0.0, 0.0))
    )
    blade_core_length = END_TAB_LENGTH - END_TAB_HEIGHT
    blade_core = (
        cq.Workplane("XY")
        .box(blade_core_length, END_TAB_THICKNESS, END_TAB_HEIGHT)
        .translate((END_TAB_HEIGHT / 2.0 + blade_core_length / 2.0, 0.0, 0.0))
    )
    tip_cap = (
        cq.Workplane("XZ")
        .circle(END_TAB_HEIGHT / 2.0)
        .extrude(END_TAB_THICKNESS / 2.0, both=True)
        .translate((END_TAB_LENGTH, 0.0, 0.0))
    )
    end_tab = root_block.union(blade_core).union(tip_cap)
    relief = (
        cq.Workplane("XY")
        .box(0.026, END_TAB_THICKNESS * 2.4, 0.010)
        .translate((0.050, 0.0, -0.011))
    )
    return end_tab.cut(relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_two_joint_chain")

    model.material("bracket_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("link_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("link_satin", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tip_oxide", rgba=(0.64, 0.39, 0.22, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket_shape(), "root_bracket"),
        material="bracket_charcoal",
        name="bracket_shell",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.130, 0.160, 0.113)),
        mass=2.4,
        origin=Origin(xyz=(-0.010, 0.0, -0.056)),
    )

    link1 = model.part("link1")
    link1.visual(
        mesh_from_cadquery(_make_link1_shape(), "link1"),
        material="link_steel",
        name="link1_shell",
    )
    link1.inertial = Inertial.from_geometry(
        Box((LINK1_LENGTH, BRACKET_INNER_SPAN, LINK1_PLATE_HEIGHT)),
        mass=0.86,
        origin=Origin(xyz=(LINK1_LENGTH / 2.0, 0.0, 0.0)),
    )

    link2 = model.part("link2")
    link2.visual(
        mesh_from_cadquery(_make_link2_shape(), "link2"),
        material="link_satin",
        name="link2_shell",
    )
    link2.inertial = Inertial.from_geometry(
        Box((LINK2_LENGTH, LINK1_INNER_SPAN, LINK2_PLATE_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(LINK2_LENGTH / 2.0, 0.0, 0.0)),
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab_shape(), "end_tab"),
        material="tip_oxide",
        name="end_tab_shell",
    )
    end_tab.inertial = Inertial.from_geometry(
        Box((END_TAB_LENGTH, LINK2_INNER_SPAN, END_TAB_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(END_TAB_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.65,
            upper=1.35,
        ),
    )

    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=-0.20,
            upper=1.95,
        ),
    )

    model.articulation(
        "forearm_to_end_tab",
        ArticulationType.FIXED,
        parent=link2,
        child=end_tab,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    end_tab = object_model.get_part("end_tab")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    tip_mount = object_model.get_articulation("forearm_to_end_tab")

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
        "two_primary_revolute_joints_present",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tip_mount.articulation_type == ArticulationType.FIXED,
        "Expected two serial revolute joints and a fixed tip mount.",
    )
    ctx.check(
        "parallel_joint_axes",
        shoulder.axis == elbow.axis == (0.0, -1.0, 0.0),
        f"Shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_contact(
            root_bracket,
            link1,
            contact_tol=0.0005,
            name="shoulder_joint_has_real_support_contact",
        )
        ctx.expect_contact(
            link1,
            link2,
            contact_tol=0.0005,
            name="elbow_joint_has_real_support_contact",
        )
        ctx.expect_contact(
            link2,
            end_tab,
            contact_tol=0.0005,
            name="end_tab_is_mounted_to_forearm",
        )
        closed_tip = ctx.part_world_position(end_tab)

    with ctx.pose({shoulder: 0.85, elbow: 0.70}):
        raised_tip = ctx.part_world_position(end_tab)

    tip_lifts = (
        closed_tip is not None
        and raised_tip is not None
        and raised_tip[2] > closed_tip[2] + 0.12
        and abs(raised_tip[1]) < 0.005
    )
    ctx.check(
        "positive_joint_motion_raises_tip_in_plane",
        tip_lifts,
        f"closed_tip={closed_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PROXIMAL_LENGTH = 0.135
MIDDLE_LENGTH = 0.112
END_TAB_LENGTH = 0.058

PROXIMAL_EYE_RADIUS = 0.015
PROXIMAL_EYE_WIDTH = 0.014
PROXIMAL_TOP_HOLE_RADIUS = 0.0046
PROXIMAL_BOTTOM_HOLE_RADIUS = 0.0041
PROXIMAL_FORK_GAP = 0.016
PROXIMAL_FORK_CHEEK = 0.009
PROXIMAL_FORK_RADIUS = 0.014

MIDDLE_EYE_RADIUS = 0.013
MIDDLE_EYE_WIDTH = 0.012
MIDDLE_TOP_HOLE_RADIUS = PROXIMAL_BOTTOM_HOLE_RADIUS
MIDDLE_BOTTOM_HOLE_RADIUS = 0.0036
MIDDLE_FORK_GAP = 0.014
MIDDLE_FORK_CHEEK = 0.008
MIDDLE_FORK_RADIUS = 0.012

END_EYE_RADIUS = 0.011
END_EYE_WIDTH = 0.010
END_TOP_HOLE_RADIUS = MIDDLE_BOTTOM_HOLE_RADIUS
END_DISTAL_HOLE_RADIUS = 0.0042

ROOT_CLEVIS_GAP = 0.018
ROOT_CLEVIS_CHEEK = 0.010
ROOT_CLEVIS_RADIUS = 0.016
ROOT_HOLE_RADIUS = PROXIMAL_TOP_HOLE_RADIUS

JOINT_AXIS = (0.0, -1.0, 0.0)


def _y_cylinder(radius: float, width: float, *, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(width / 2.0, both=True).translate((0.0, 0.0, z))


def _make_serial_link(
    *,
    length: float,
    top_eye_radius: float,
    top_eye_width: float,
    top_hole_radius: float,
    body_width_top: float,
    body_width_bottom: float,
    body_thickness: float,
    lower_fork_gap: float,
    lower_fork_cheek: float,
    lower_fork_radius: float,
    lower_hole_radius: float,
    child_eye_radius: float,
    fork_bridge_height: float,
) -> cq.Workplane:
    top_eye = _y_cylinder(top_eye_radius, top_eye_width)

    spine = (
        cq.Workplane("XZ")
        .moveTo(body_width_top / 2.0, -0.28 * top_eye_radius)
        .lineTo(body_width_bottom / 2.0, -(length - fork_bridge_height))
        .lineTo(-body_width_bottom / 2.0, -(length - fork_bridge_height))
        .lineTo(-body_width_top / 2.0, -0.28 * top_eye_radius)
        .close()
        .extrude(body_thickness / 2.0, both=True)
    )

    lower_outer_width = lower_fork_gap + 2.0 * lower_fork_cheek
    yoke_width_x = max(body_width_bottom + 0.006, lower_fork_radius * 2.25)
    yoke_height = fork_bridge_height + lower_fork_radius
    yoke_center_z = -length + (fork_bridge_height - lower_fork_radius) / 2.0

    lower_yoke = cq.Workplane("XY").box(yoke_width_x, lower_outer_width, yoke_height).translate(
        (0.0, 0.0, yoke_center_z)
    )
    lower_pad = _y_cylinder(lower_fork_radius, lower_outer_width, z=-length)

    link = top_eye.union(spine).union(lower_yoke).union(lower_pad)

    slot_top = -length + child_eye_radius + 0.006
    slot_bottom = -length - lower_fork_radius - 0.005
    slot_height = slot_top - slot_bottom
    slot_center_z = (slot_top + slot_bottom) / 2.0
    slot = cq.Workplane("XY").box(yoke_width_x + 0.020, lower_fork_gap, slot_height).translate(
        (0.0, 0.0, slot_center_z)
    )

    upper_hole = _y_cylinder(top_hole_radius, top_eye_width + 0.010)
    lower_hole = _y_cylinder(lower_hole_radius, lower_outer_width + 0.010, z=-length)
    lower_pin = _y_cylinder(lower_hole_radius, lower_outer_width, z=-length)

    return link.cut(slot).cut(upper_hole).cut(lower_hole).union(lower_pin)


def _make_end_tab_shape() -> cq.Workplane:
    top_eye = _y_cylinder(END_EYE_RADIUS, END_EYE_WIDTH)

    plate = (
        cq.Workplane("XZ")
        .moveTo(0.022 / 2.0, -0.24 * END_EYE_RADIUS)
        .lineTo(0.014 / 2.0, -END_TAB_LENGTH)
        .lineTo(-0.014 / 2.0, -END_TAB_LENGTH)
        .lineTo(-0.022 / 2.0, -0.24 * END_EYE_RADIUS)
        .close()
        .extrude(0.005, both=True)
    )

    pad = _y_cylinder(0.0105, 0.010, z=-END_TAB_LENGTH)
    top_hole = _y_cylinder(END_TOP_HOLE_RADIUS, END_EYE_WIDTH + 0.008)
    distal_hole = _y_cylinder(END_DISTAL_HOLE_RADIUS, 0.018, z=-END_TAB_LENGTH)

    return top_eye.union(plate).union(pad).cut(top_hole).cut(distal_hole)


def _make_bridge_support_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.050, 0.248, 0.028).translate((0.0, 0.0, 0.150))
    left_post = cq.Workplane("XY").box(0.040, 0.038, 0.126).translate((0.0, -0.102, 0.075))
    right_post = cq.Workplane("XY").box(0.040, 0.038, 0.126).translate((0.0, 0.102, 0.075))
    left_pad = cq.Workplane("XY").box(0.078, 0.054, 0.012).translate((0.0, -0.102, 0.006))
    right_pad = cq.Workplane("XY").box(0.078, 0.054, 0.012).translate((0.0, 0.102, 0.006))
    center_hanger = cq.Workplane("XY").box(0.030, ROOT_CLEVIS_GAP + 2.0 * ROOT_CLEVIS_CHEEK, 0.138).translate(
        (0.0, 0.0, 0.069)
    )
    root_pad = _y_cylinder(ROOT_CLEVIS_RADIUS, ROOT_CLEVIS_GAP + 2.0 * ROOT_CLEVIS_CHEEK)

    bridge = (
        beam.union(left_post)
        .union(right_post)
        .union(left_pad)
        .union(right_pad)
        .union(center_hanger)
        .union(root_pad)
    )

    slot_top = PROXIMAL_EYE_RADIUS + 0.006
    slot_bottom = -ROOT_CLEVIS_RADIUS - 0.005
    clevis_slot = cq.Workplane("XY").box(0.060, ROOT_CLEVIS_GAP, slot_top - slot_bottom).translate(
        (0.0, 0.0, (slot_top + slot_bottom) / 2.0)
    )
    root_hole = _y_cylinder(ROOT_HOLE_RADIUS, ROOT_CLEVIS_GAP + 2.0 * ROOT_CLEVIS_CHEEK + 0.010)
    root_pin = _y_cylinder(ROOT_HOLE_RADIUS, ROOT_CLEVIS_GAP + 2.0 * ROOT_CLEVIS_CHEEK)

    return bridge.cut(clevis_slot).cut(root_hole).union(root_pin)


def _make_proximal_link_shape() -> cq.Workplane:
    return _make_serial_link(
        length=PROXIMAL_LENGTH,
        top_eye_radius=PROXIMAL_EYE_RADIUS,
        top_eye_width=PROXIMAL_EYE_WIDTH,
        top_hole_radius=PROXIMAL_TOP_HOLE_RADIUS,
        body_width_top=0.032,
        body_width_bottom=0.025,
        body_thickness=0.014,
        lower_fork_gap=PROXIMAL_FORK_GAP,
        lower_fork_cheek=PROXIMAL_FORK_CHEEK,
        lower_fork_radius=PROXIMAL_FORK_RADIUS,
        lower_hole_radius=PROXIMAL_BOTTOM_HOLE_RADIUS,
        child_eye_radius=MIDDLE_EYE_RADIUS,
        fork_bridge_height=0.032,
    )


def _make_middle_link_shape() -> cq.Workplane:
    return _make_serial_link(
        length=MIDDLE_LENGTH,
        top_eye_radius=MIDDLE_EYE_RADIUS,
        top_eye_width=MIDDLE_EYE_WIDTH,
        top_hole_radius=MIDDLE_TOP_HOLE_RADIUS,
        body_width_top=0.026,
        body_width_bottom=0.020,
        body_thickness=0.012,
        lower_fork_gap=MIDDLE_FORK_GAP,
        lower_fork_cheek=MIDDLE_FORK_CHEEK,
        lower_fork_radius=MIDDLE_FORK_RADIUS,
        lower_hole_radius=MIDDLE_BOTTOM_HOLE_RADIUS,
        child_eye_radius=END_EYE_RADIUS,
        fork_bridge_height=0.028,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    model.material("bridge_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("proximal_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("middle_steel", rgba=(0.58, 0.61, 0.64, 1.0))
    model.material("end_tab_dark", rgba=(0.31, 0.33, 0.36, 1.0))

    bridge_support = model.part("bridge_support")
    bridge_support.visual(
        mesh_from_cadquery(_make_bridge_support_shape(), "bridge_support"),
        material="bridge_graphite",
        name="support_body",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(_make_proximal_link_shape(), "proximal_link"),
        material="proximal_steel",
        name="proximal_body",
    )

    middle_link = model.part("middle_link")
    middle_link.visual(
        mesh_from_cadquery(_make_middle_link_shape(), "middle_link"),
        material="middle_steel",
        name="middle_body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab_shape(), "end_tab"),
        material="end_tab_dark",
        name="end_tab_body",
    )

    model.articulation(
        "bridge_to_proximal",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=proximal_link,
        origin=Origin(),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=28.0, velocity=1.8),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=middle_link,
        origin=Origin(xyz=(0.0, 0.0, -PROXIMAL_LENGTH)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=20.0, velocity=2.1),
    )
    model.articulation(
        "middle_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -MIDDLE_LENGTH)),
        axis=JOINT_AXIS,
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=10.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    proximal_link = object_model.get_part("proximal_link")
    middle_link = object_model.get_part("middle_link")
    end_tab = object_model.get_part("end_tab")
    bridge_to_proximal = object_model.get_articulation("bridge_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_end_tab = object_model.get_articulation("middle_to_end_tab")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        bridge_support,
        proximal_link,
        reason="The root revolute joint is modeled as a captured pin passing through the proximal eye.",
    )
    ctx.allow_overlap(
        proximal_link,
        middle_link,
        reason="The second revolute joint uses a captured pin inside the middle link eye.",
    )
    ctx.allow_overlap(
        middle_link,
        end_tab,
        reason="The distal compact tab is carried on the middle link pin at the third revolute joint.",
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

    ctx.expect_origin_distance(
        bridge_support,
        proximal_link,
        axes="xy",
        max_dist=0.001,
        name="bridge and proximal joint stay on the same centerline",
    )
    ctx.expect_origin_gap(
        proximal_link,
        middle_link,
        axis="z",
        min_gap=PROXIMAL_LENGTH - 0.002,
        max_gap=PROXIMAL_LENGTH + 0.002,
        name="middle joint sits one proximal span below the first joint",
    )
    ctx.expect_origin_gap(
        middle_link,
        end_tab,
        axis="z",
        min_gap=MIDDLE_LENGTH - 0.002,
        max_gap=MIDDLE_LENGTH + 0.002,
        name="end tab pivots one middle span below the second joint",
    )

    ctx.check(
        "all three joints pitch about the bridge axis",
        bridge_to_proximal.axis == JOINT_AXIS
        and proximal_to_middle.axis == JOINT_AXIS
        and middle_to_end_tab.axis == JOINT_AXIS,
        details=(
            f"bridge_to_proximal={bridge_to_proximal.axis}, "
            f"proximal_to_middle={proximal_to_middle.axis}, "
            f"middle_to_end_tab={middle_to_end_tab.axis}"
        ),
    )

    rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose(
        {
            bridge_to_proximal: 0.60,
            proximal_to_middle: 0.52,
            middle_to_end_tab: 0.40,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="curled pose stays clear of self-intersection")
        curled_tip = ctx.part_world_position(end_tab)

    ctx.check(
        "positive joint motion curls the distal tab forward",
        rest_tip is not None
        and curled_tip is not None
        and curled_tip[0] > rest_tip[0] + 0.080
        and curled_tip[2] > rest_tip[2] + 0.040,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


ROOT_CHEEK_T = 0.014
ROOT_GAP = 0.040
ROOT_OUTER_W = ROOT_GAP + 2.0 * ROOT_CHEEK_T
SHOULDER_PIN_R = 0.0105

UPPER_WEB_T = 0.014
ELBOW_GAP = 0.028
ELBOW_CHEEK_T = 0.014
ELBOW_OUTER_W = ELBOW_GAP + 2.0 * ELBOW_CHEEK_T
ELBOW_PIN_R = 0.0085

FOREARM_T = 0.022
ELBOW_Y = 0.330
ELBOW_Z = 0.110
TIP_Y = 0.292
TIP_Z = -0.004


def _root_bracket_body() -> cq.Workplane:
    wall_plate = cq.Workplane("XY").box(0.240, 0.018, 0.380).translate((0.0, -0.100, 0.0))
    lower_block = cq.Workplane("XY").box(0.128, 0.056, 0.120).translate((0.0, -0.070, -0.082))
    cheek_left = cq.Workplane("XY").box(ROOT_CHEEK_T, 0.036, 0.118).translate(
        (ROOT_GAP / 2.0 + ROOT_CHEEK_T / 2.0, -0.006, 0.0)
    )
    cheek_right = cq.Workplane("XY").box(ROOT_CHEEK_T, 0.036, 0.118).translate(
        (-ROOT_GAP / 2.0 - ROOT_CHEEK_T / 2.0, -0.006, 0.0)
    )
    gusset_left = cq.Workplane("XY").box(ROOT_CHEEK_T, 0.060, 0.042).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -22
    ).translate(
        (ROOT_GAP / 2.0 + ROOT_CHEEK_T / 2.0, -0.050, -0.050)
    )
    gusset_right = cq.Workplane("XY").box(ROOT_CHEEK_T, 0.060, 0.042).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -22
    ).translate(
        (-ROOT_GAP / 2.0 - ROOT_CHEEK_T / 2.0, -0.050, -0.050)
    )
    cap_left = cq.Workplane("YZ").circle(0.018).extrude(0.004).translate((ROOT_OUTER_W / 2.0, 0.0, 0.0))
    cap_right = cq.Workplane("YZ").circle(0.018).extrude(0.004).translate((-ROOT_OUTER_W / 2.0 - 0.004, 0.0, 0.0))
    wall_screws = (
        cq.Workplane("XZ")
        .pushPoints([(-0.075, -0.120), (0.075, -0.120), (-0.075, 0.120), (0.075, 0.120)])
        .circle(0.009)
        .extrude(0.006)
        .translate((0.0, -0.091, 0.0))
    )
    return (
        wall_plate.union(lower_block)
        .union(gusset_left)
        .union(gusset_right)
        .union(cheek_left)
        .union(cheek_right)
        .union(cap_left)
        .union(cap_right)
        .union(wall_screws)
    )


def _root_shoulder_pin() -> cq.Workplane:
    pin = cq.Workplane("YZ").circle(SHOULDER_PIN_R).extrude(ROOT_OUTER_W / 2.0 + 0.004, both=True)
    left_bolt = cq.Workplane("YZ").circle(0.005).extrude(0.003).translate((ROOT_OUTER_W / 2.0 + 0.004, 0.0, 0.0))
    right_bolt = cq.Workplane("YZ").circle(0.005).extrude(0.003).translate((-ROOT_OUTER_W / 2.0 - 0.007, 0.0, 0.0))
    return pin.union(left_bolt).union(right_bolt)


def _upper_link_body() -> cq.Workplane:
    shoulder_eye = _upper_shoulder_eye()
    shoulder_link_left = cq.Workplane("XY").box(0.008, 0.072, 0.016).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12
    ).translate((0.014, 0.042, 0.016))
    shoulder_link_right = cq.Workplane("XY").box(0.008, 0.072, 0.016).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12
    ).translate((-0.014, 0.042, 0.016))
    rear_rail_left = cq.Workplane("XY").box(0.008, 0.110, 0.020).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -20
    ).translate((0.014, 0.082, 0.034))
    rear_rail_right = cq.Workplane("XY").box(0.008, 0.110, 0.020).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -20
    ).translate((-0.014, 0.082, 0.034))
    mid_riser_left = cq.Workplane("XY").box(0.008, 0.070, 0.018).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 6
    ).translate((0.014, 0.165, 0.072))
    mid_riser_right = cq.Workplane("XY").box(0.008, 0.070, 0.018).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 6
    ).translate((-0.014, 0.165, 0.072))
    approach_left = cq.Workplane("XY").box(0.008, 0.105, 0.018).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 10
    ).translate((0.014, 0.250, 0.104))
    approach_right = cq.Workplane("XY").box(0.008, 0.105, 0.018).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 10
    ).translate((-0.014, 0.250, 0.104))
    cheek_left = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).rect(0.058, 0.062).extrude(
        ELBOW_CHEEK_T / 2.0, both=True
    ).translate((ELBOW_GAP / 2.0 + ELBOW_CHEEK_T / 2.0, 0.0, 0.0))
    cheek_right = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).rect(0.058, 0.062).extrude(
        ELBOW_CHEEK_T / 2.0, both=True
    ).translate((-ELBOW_GAP / 2.0 - ELBOW_CHEEK_T / 2.0, 0.0, 0.0))
    top_bridge_left = cq.Workplane("XY").box(0.008, 0.074, 0.014).translate((0.021, 0.292, 0.136))
    top_bridge_right = cq.Workplane("XY").box(0.008, 0.074, 0.014).translate((-0.021, 0.292, 0.136))
    cap_left = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).circle(0.018).extrude(0.004).translate(
        (ELBOW_OUTER_W / 2.0, 0.0, 0.0)
    )
    cap_right = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).circle(0.018).extrude(0.004).translate(
        (-ELBOW_OUTER_W / 2.0 - 0.004, 0.0, 0.0)
    )
    return (
        shoulder_eye.union(shoulder_link_left)
        .union(shoulder_link_right)
        .union(rear_rail_left)
        .union(rear_rail_right)
        .union(mid_riser_left)
        .union(mid_riser_right)
        .union(approach_left)
        .union(approach_right)
        .union(cheek_left)
        .union(cheek_right)
        .union(top_bridge_left)
        .union(top_bridge_right)
        .union(cap_left)
        .union(cap_right)
    )


def _upper_shoulder_eye() -> cq.Workplane:
    eye = cq.Workplane("YZ").circle(0.016).extrude((ROOT_GAP - 0.002) / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(SHOULDER_PIN_R).extrude(ROOT_GAP / 2.0 + 0.003, both=True)
    return eye.cut(bore)


def _upper_elbow_pin() -> cq.Workplane:
    pin = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).circle(ELBOW_PIN_R).extrude(ELBOW_OUTER_W / 2.0 + 0.004, both=True)
    left_bolt = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).circle(0.0045).extrude(0.003).translate((ELBOW_OUTER_W / 2.0 + 0.004, 0.0, 0.0))
    right_bolt = cq.Workplane("YZ").center(ELBOW_Y, ELBOW_Z).circle(0.0045).extrude(0.003).translate((-ELBOW_OUTER_W / 2.0 - 0.007, 0.0, 0.0))
    return pin.union(left_bolt).union(right_bolt)


def _forearm_body() -> cq.Workplane:
    forearm_eye = _forearm_eye()
    root_neck = cq.Workplane("XY").box(FOREARM_T, 0.054, 0.016).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 4
    ).translate((0.0, 0.032, -0.004))
    beam_a = cq.Workplane("XY").box(FOREARM_T, 0.138, 0.016).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 6
    ).translate((0.0, 0.082, -0.006))
    beam_b = cq.Workplane("XY").box(FOREARM_T * 0.9, 0.084, 0.014).rotate(
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -4
    ).translate((0.0, 0.192, -0.012))
    terminal_stem = cq.Workplane("XY").box(FOREARM_T, 0.056, 0.018).translate((0.0, 0.262, -0.008))
    plate = cq.Workplane("XY").box(0.072, 0.012, 0.084).translate((0.0, TIP_Y, TIP_Z))
    plate_screws = (
        cq.Workplane("XZ")
        .pushPoints([(-0.018, 0.016), (0.018, 0.016)])
        .circle(0.0045)
        .extrude(0.004)
        .translate((0.0, TIP_Y + 0.006, TIP_Z))
    )
    return forearm_eye.union(root_neck).union(beam_a).union(beam_b).union(terminal_stem).union(plate).union(plate_screws)


def _forearm_eye() -> cq.Workplane:
    eye = cq.Workplane("YZ").circle(0.015).extrude(FOREARM_T / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(ELBOW_PIN_R).extrude(ELBOW_GAP / 2.0 + 0.004, both=True)
    return eye.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_transfer_arm")

    bracket_mat = model.material("bracket_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    upper_mat = model.material("upper_link_paint", rgba=(0.58, 0.40, 0.16, 1.0))
    forearm_mat = model.material("forearm_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    hardware_mat = model.material("hardware_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_root_bracket_body(), "root_bracket_body"),
        material=bracket_mat,
        name="bracket_body",
    )
    root_bracket.visual(
        mesh_from_cadquery(_root_shoulder_pin(), "root_shoulder_pin"),
        material=hardware_mat,
        name="shoulder_pin",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_body(), "upper_link_body"),
        material=upper_mat,
        name="upper_body",
    )
    upper_link.visual(
        mesh_from_cadquery(_upper_elbow_pin(), "upper_elbow_pin"),
        material=hardware_mat,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_body(), "forearm_body"),
        material=forearm_mat,
        name="forearm_body",
    )

    model.articulation(
        "root_to_upper",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=upper_link,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=1.8, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.0, ELBOW_Y, ELBOW_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.2, lower=-1.90, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        "root_bracket",
        "upper_link",
        elem_a="shoulder_pin",
        elem_b="upper_body",
        reason="Shoulder pin runs through the upper-link bore; tiny residual mesh overlap is acceptable at the revolute bearing.",
    )
    ctx.allow_overlap(
        "upper_link",
        "forearm",
        elem_a="elbow_pin",
        elem_b="forearm_body",
        reason="Elbow pin runs through the forearm eye; tiny residual mesh overlap is acceptable at the revolute bearing.",
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

    root = object_model.get_part("root_bracket")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("root_to_upper")
    elbow = object_model.get_articulation("upper_to_forearm")

    ctx.check(
        "serial revolute axes stay in one plane",
        tuple(shoulder.axis) == (1.0, 0.0, 0.0) and tuple(elbow.axis) == (1.0, 0.0, 0.0),
        details=f"shoulder_axis={shoulder.axis} elbow_axis={elbow.axis}",
    )
    ctx.expect_contact(root, upper, contact_tol=0.0015, name="shoulder joint is grounded")
    ctx.expect_contact(upper, forearm, contact_tol=0.0015, name="elbow joint is grounded")
    ctx.expect_origin_gap(forearm, root, axis="y", min_gap=0.28, name="rest reach projects forward from bracket")

    with ctx.pose({shoulder: 0.85, elbow: -1.70}):
        ctx.expect_origin_gap(forearm, root, axis="y", min_gap=0.05, name="folded pose still stays outboard of the wall bracket")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_fixed=True,
        name="transfer arm clears itself through its motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

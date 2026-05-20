from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BEAM_LENGTH = 2.4
BEAM_HEIGHT = 0.16
FLANGE_WIDTH = 0.18
WEB_THICKNESS = 0.014
FLANGE_THICKNESS = 0.018

SHUTTLE_LENGTH = 0.26
SHUTTLE_WIDTH = 0.16
SHUTTLE_TOP_HEIGHT = 0.022
SHUTTLE_SIDE_HEIGHT = 0.12
SHUTTLE_SIDE_THICKNESS = 0.012
PAD_LENGTH = 0.06
PAD_WIDTH = 0.028
PAD_HEIGHT = 0.012
PAD_X = 0.075
PAD_Y = 0.045
SHOULDER_Z = -0.14
SHOULDER_CLEVIS_GAP = 0.03
SHOULDER_CLEVIS_PLATE = 0.01
SHOULDER_HUB_THICKNESS = SHOULDER_CLEVIS_GAP

LINK1_LENGTH = 0.28
LINK2_LENGTH = 0.62
LINK_WEB_WIDTH = 0.05
LINK_WEB_THICKNESS = 0.018
HUB_RADIUS = 0.023
ELBOW_CLEVIS_GAP = 0.026
ELBOW_CLEVIS_PLATE = 0.009
ELBOW_HUB_THICKNESS = ELBOW_CLEVIS_GAP

END_PLATE_LENGTH = 0.18
END_PLATE_WIDTH = 0.12
END_PLATE_THICKNESS = 0.012


def fuse_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def make_i_beam(length: float, height: float, flange_width: float, web_thickness: float, flange_thickness: float) -> cq.Workplane:
    half_w = flange_width / 2.0
    half_h = height / 2.0
    half_web = web_thickness / 2.0
    ft = flange_thickness

    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-half_w, -half_h),
                (half_w, -half_h),
                (half_w, -half_h + ft),
                (half_web, -half_h + ft),
                (half_web, half_h - ft),
                (half_w, half_h - ft),
                (half_w, half_h),
                (-half_w, half_h),
                (-half_w, half_h - ft),
                (-half_web, half_h - ft),
                (-half_web, -half_h + ft),
                (-half_w, -half_h + ft),
            ]
        )
        .close()
        .extrude(length / 2.0, both=True)
    )
    return profile


def make_shuttle() -> cq.Workplane:
    top_body = cq.Workplane("XY").box(
        SHUTTLE_LENGTH, SHUTTLE_WIDTH, SHUTTLE_TOP_HEIGHT
    ).translate((0.0, 0.0, -SHUTTLE_TOP_HEIGHT / 2.0))
    left_side = cq.Workplane("XY").box(
        SHUTTLE_LENGTH, SHUTTLE_SIDE_THICKNESS, 0.05
    ).translate((0.0, SHUTTLE_WIDTH / 2.0 - SHUTTLE_SIDE_THICKNESS / 2.0, -0.047))
    right_side = cq.Workplane("XY").box(
        SHUTTLE_LENGTH, SHUTTLE_SIDE_THICKNESS, 0.05
    ).translate((0.0, -SHUTTLE_WIDTH / 2.0 + SHUTTLE_SIDE_THICKNESS / 2.0, -0.047))
    center_hanger = cq.Workplane("XY").box(0.055, 0.05, 0.07).translate((0.0, 0.0, -0.057))

    pads = []
    for x_center in (-PAD_X, PAD_X):
        for y_center in (-PAD_Y, PAD_Y):
            pads.append(
                cq.Workplane("XY")
                .box(PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT)
                .translate((x_center, y_center, -PAD_HEIGHT / 2.0))
            )

    support_height = 0.085
    yoke_height = 0.036
    left_support = cq.Workplane("XY").box(0.03, SHOULDER_CLEVIS_PLATE, support_height).translate(
        (
            0.0,
            SHOULDER_CLEVIS_GAP / 2.0 + SHOULDER_CLEVIS_PLATE / 2.0,
            -0.0975,
        )
    )
    right_support = cq.Workplane("XY").box(0.03, SHOULDER_CLEVIS_PLATE, support_height).translate(
        (
            0.0,
            -SHOULDER_CLEVIS_GAP / 2.0 - SHOULDER_CLEVIS_PLATE / 2.0,
            -0.0975,
        )
    )
    left_clevis = cq.Workplane("XY").box(0.04, SHOULDER_CLEVIS_PLATE, yoke_height).translate(
        (
            0.0,
            SHOULDER_CLEVIS_GAP / 2.0 + SHOULDER_CLEVIS_PLATE / 2.0,
            SHOULDER_Z + yoke_height / 2.0,
        )
    )
    right_clevis = cq.Workplane("XY").box(0.04, SHOULDER_CLEVIS_PLATE, yoke_height).translate(
        (
            0.0,
            -SHOULDER_CLEVIS_GAP / 2.0 - SHOULDER_CLEVIS_PLATE / 2.0,
            SHOULDER_Z + yoke_height / 2.0,
        )
    )

    return fuse_all(
        top_body,
        left_side,
        right_side,
        center_hanger,
        left_support,
        right_support,
        left_clevis,
        right_clevis,
        *pads,
    )


def make_upper_arm() -> cq.Workplane:
    top_lug = cq.Workplane("XY").box(0.04, SHOULDER_HUB_THICKNESS, 0.04).translate((0.0, 0.0, -0.02))
    neck = cq.Workplane("XY").box(0.032, LINK_WEB_THICKNESS, 0.02).translate((0.0, 0.0, -0.05))
    main_web = cq.Workplane("XY").box(0.04, LINK_WEB_THICKNESS, 0.18).translate((0.0, 0.0, -0.15))
    lower_yoke_block = cq.Workplane("XY").box(0.04, LINK_WEB_THICKNESS, 0.02).translate((0.0, 0.0, -0.25))

    elbow_plate_height = 0.04
    left_clevis = cq.Workplane("XY").box(0.036, ELBOW_CLEVIS_PLATE, elbow_plate_height).translate(
        (
            0.0,
            ELBOW_CLEVIS_GAP / 2.0 + ELBOW_CLEVIS_PLATE / 2.0,
            -LINK1_LENGTH + elbow_plate_height / 2.0,
        )
    )
    right_clevis = cq.Workplane("XY").box(0.036, ELBOW_CLEVIS_PLATE, elbow_plate_height).translate(
        (
            0.0,
            -ELBOW_CLEVIS_GAP / 2.0 - ELBOW_CLEVIS_PLATE / 2.0,
            -LINK1_LENGTH + elbow_plate_height / 2.0,
        )
    )

    return fuse_all(top_lug, neck, main_web, lower_yoke_block, left_clevis, right_clevis)


def make_forearm_with_plate() -> cq.Workplane:
    top_lug = cq.Workplane("XY").box(0.036, ELBOW_HUB_THICKNESS, 0.04).translate((0.0, 0.0, -0.02))
    neck = cq.Workplane("XY").box(0.03, LINK_WEB_THICKNESS, 0.02).translate((0.0, 0.0, -0.05))
    main_web = cq.Workplane("XY").box(0.038, LINK_WEB_THICKNESS, 0.52).translate((0.0, 0.0, -0.32))
    mount_block = cq.Workplane("XY").box(0.05, 0.04, 0.06).translate((0.0, 0.0, -0.59))
    end_plate = cq.Workplane("XY").box(
        END_PLATE_LENGTH, END_PLATE_WIDTH, END_PLATE_THICKNESS
    ).translate((0.0, 0.0, -LINK2_LENGTH - END_PLATE_THICKNESS / 2.0))

    return fuse_all(top_lug, neck, main_web, mount_block, end_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_rail_inspection_arm")

    model.material("beam_steel", rgba=(0.28, 0.30, 0.34, 1.0))
    model.material("shuttle_orange", rgba=(0.88, 0.50, 0.16, 1.0))
    model.material("arm_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("plate_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(
            make_i_beam(
                BEAM_LENGTH,
                BEAM_HEIGHT,
                FLANGE_WIDTH,
                WEB_THICKNESS,
                FLANGE_THICKNESS,
            ),
            "beam",
        ),
        name="beam_shell",
        material="beam_steel",
    )
    beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, FLANGE_WIDTH, BEAM_HEIGHT)),
        mass=45.0,
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(make_shuttle(), "shuttle"),
        name="shuttle_shell",
        material="shuttle_orange",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((SHUTTLE_LENGTH, SHUTTLE_WIDTH, 0.16)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(make_upper_arm(), "upper_arm"),
        name="upper_arm_shell",
        material="arm_graphite",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.06, 0.05, LINK1_LENGTH + 0.05)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH / 2.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(make_forearm_with_plate(), "forearm"),
        name="forearm_shell",
        material="plate_aluminum",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((END_PLATE_LENGTH, END_PLATE_WIDTH, LINK2_LENGTH + END_PLATE_THICKNESS)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, -(LINK2_LENGTH + END_PLATE_THICKNESS) / 2.0)),
    )

    travel = model.articulation(
        "beam_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, -BEAM_HEIGHT / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-0.90,
            upper=0.90,
        ),
    )
    shoulder = model.articulation(
        "shuttle_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-1.10,
            upper=1.10,
        ),
    )
    elbow = model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=-0.10,
            upper=2.20,
        ),
    )

    model.meta["primary_articulations"] = [travel.name, shoulder.name, elbow.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    shuttle = object_model.get_part("shuttle")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")

    travel = object_model.get_articulation("beam_to_shuttle")
    shoulder = object_model.get_articulation("shuttle_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")

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

    ctx.check("beam_present", beam is not None, "beam part is missing")
    ctx.check("shuttle_present", shuttle is not None, "shuttle part is missing")
    ctx.check("upper_arm_present", upper_arm is not None, "upper arm part is missing")
    ctx.check("forearm_present", forearm is not None, "forearm part is missing")

    ctx.check(
        "prismatic_axis_runs_along_beam",
        tuple(travel.axis) == (1.0, 0.0, 0.0),
        f"expected shuttle slide axis (1, 0, 0), got {travel.axis}",
    )
    ctx.check(
        "shoulder_is_pitch_joint",
        tuple(shoulder.axis) == (0.0, 1.0, 0.0),
        f"expected shoulder axis (0, 1, 0), got {shoulder.axis}",
    )
    ctx.check(
        "elbow_is_pitch_joint",
        tuple(elbow.axis) == (0.0, 1.0, 0.0),
        f"expected elbow axis (0, 1, 0), got {elbow.axis}",
    )

    ctx.expect_contact(beam, shuttle, name="shuttle_physically_rides_on_beam")
    ctx.expect_contact(shuttle, upper_arm, name="inspection_arm_hangs_from_shuttle")
    ctx.expect_contact(upper_arm, forearm, name="forearm_contacts_upper_arm_at_elbow")
    ctx.expect_gap(beam, forearm, axis="z", min_gap=0.05, name="arm_hangs_below_beam")

    with ctx.pose({travel: 0.72, shoulder: 0.55, elbow: 1.10}):
        ctx.expect_contact(beam, shuttle, name="shuttle_stays_attached_while_traveling")
        ctx.expect_contact(shuttle, upper_arm, name="shoulder_joint_remains_connected_in_pose")
        ctx.expect_contact(upper_arm, forearm, name="elbow_joint_remains_connected_in_pose")
        ctx.expect_gap(beam, forearm, axis="z", min_gap=0.02, name="folded_arm_clears_beam")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

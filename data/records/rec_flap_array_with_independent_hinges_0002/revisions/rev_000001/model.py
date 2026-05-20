from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
FRAME_W = 0.76
FRAME_D = 0.52
FRAME_T = 0.028
FRAME_BORDER_X = 0.055
FRAME_BORDER_Y = 0.055
MULLION_X = 0.032
MULLION_Y = 0.032

OPENING_W = (FRAME_W - 2.0 * FRAME_BORDER_X - MULLION_X) / 2.0
OPENING_D = (FRAME_D - 2.0 * FRAME_BORDER_Y - MULLION_Y) / 2.0
OPENING_X_OFFSET = (MULLION_X + OPENING_W) / 2.0
OPENING_Y_OFFSET = (MULLION_Y + OPENING_D) / 2.0

PANEL_T = 0.005
CLOSED_GAP = 0.0012
HINGE_AXIS_INSET = 0.002
PANEL_START_X = 0.0048
FLAP_W = OPENING_W - 0.014
FLAP_D = OPENING_D - 0.014
BARREL_R = 0.0055
BARREL_LEN = FLAP_D - 0.020
PANEL_Z_OFFSET = -(BARREL_R - PANEL_T / 2.0)
COLLISION_W = FLAP_W - 0.012
COLLISION_D = FLAP_D - 0.010
OPEN_LIMIT = 1.35

FLAP_LAYOUT = (
    ("flap_top_left", "frame_to_flap_top_left", (-OPENING_X_OFFSET, OPENING_Y_OFFSET)),
    ("flap_top_right", "frame_to_flap_top_right", (OPENING_X_OFFSET, OPENING_Y_OFFSET)),
    ("flap_bottom_left", "frame_to_flap_bottom_left", (-OPENING_X_OFFSET, -OPENING_Y_OFFSET)),
    ("flap_bottom_right", "frame_to_flap_bottom_right", (OPENING_X_OFFSET, -OPENING_Y_OFFSET)),
)


def _make_frame_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FRAME_W, FRAME_D, FRAME_T)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([center for _, _, center in FLAP_LAYOUT])
        .rect(OPENING_W, OPENING_D)
        .cutThruAll()
    )


def _make_flap_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(FLAP_W, FLAP_D, PANEL_T)
        .translate((PANEL_START_X + FLAP_W / 2.0, 0.0, 0.0))
    )
    panel = (
        panel.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(FLAP_W - 0.028, FLAP_D - 0.028)
        .cutBlind(-0.0014)
    )
    panel = (
        panel.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(FLAP_W * 0.20, 0.0)
        .slot2D(0.055, 0.012, angle=0.0)
        .cutBlind(-0.0010)
    )
    barrel = (
        cq.Workplane("XZ")
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0.0, -BARREL_LEN / 2.0, 0.0))
    )
    hinge_leaf = (
        cq.Workplane("XY")
        .box(BARREL_R * 1.6, BARREL_LEN, PANEL_T * 0.8)
        .translate((BARREL_R * 0.55, 0.0, -PANEL_T * 0.05))
    )
    return panel.union(barrel).union(hinge_leaf)


def _add_frame_collisions(frame_part) -> None:
    pass


def _configure_flap_part(flap_part) -> None:
    panel_origin_x = PANEL_START_X + FLAP_W / 2.0
    bead_inset = 0.014
    bead_width = 0.010
    bead_height = 0.0012
    inner_w = FLAP_W - 2.0 * bead_inset
    inner_d = FLAP_D - 2.0 * bead_inset
    bead_z = PANEL_Z_OFFSET + PANEL_T / 2.0 - bead_height / 2.0

    flap_part.visual(
        Box((FLAP_W, FLAP_D, PANEL_T)),
        origin=Origin(xyz=(panel_origin_x, 0.0, PANEL_Z_OFFSET)),
        material="flap_finish",
    )
    flap_part.visual(
        Box((inner_w, bead_width, bead_height)),
        origin=Origin(xyz=(panel_origin_x, FLAP_D / 2.0 - bead_inset - bead_width / 2.0, bead_z)),
        material="flap_finish",
    )
    flap_part.visual(
        Box((inner_w, bead_width, bead_height)),
        origin=Origin(xyz=(panel_origin_x, -FLAP_D / 2.0 + bead_inset + bead_width / 2.0, bead_z)),
        material="flap_finish",
    )
    flap_part.visual(
        Box((bead_width, inner_d, bead_height)),
        origin=Origin(xyz=(PANEL_START_X + bead_inset + bead_width / 2.0, 0.0, bead_z)),
        material="flap_finish",
    )
    flap_part.visual(
        Box((bead_width, inner_d, bead_height)),
        origin=Origin(xyz=(PANEL_START_X + FLAP_W - bead_inset - bead_width / 2.0, 0.0, bead_z)),
        material="flap_finish",
    )
    flap_part.visual(
        Cylinder(radius=BARREL_R, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="frame_finish",
    )
    flap_part.visual(
        Box((BARREL_R * 1.6, BARREL_LEN, PANEL_T * 0.8)),
        origin=Origin(xyz=(BARREL_R * 0.55, 0.0, PANEL_Z_OFFSET * 0.65)),
        material="frame_finish",
    )
    flap_part.inertial = Inertial.from_geometry(
        Box((FLAP_W, FLAP_D, PANEL_T)),
        mass=0.34,
        origin=Origin(xyz=(panel_origin_x, 0.0, PANEL_Z_OFFSET)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_flap_frame", assets=ASSETS)
    model.material("frame_finish", rgba=(0.23, 0.26, 0.30, 1.0))
    model.material("flap_finish", rgba=(0.84, 0.86, 0.89, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS), material="frame_finish"
    )
    _add_frame_collisions(frame)
    frame.inertial = Inertial.from_geometry(Box((FRAME_W, FRAME_D, FRAME_T)), mass=4.8)

    for part_name, joint_name, (center_x, center_y) in FLAP_LAYOUT:
        flap = model.part(part_name)
        _configure_flap_part(flap)
        hinge_x = center_x - OPENING_W / 2.0 + HINGE_AXIS_INSET
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(
                xyz=(
                    hinge_x,
                    center_y,
                    FRAME_T / 2.0 + BARREL_R + CLOSED_GAP,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=OPEN_LIMIT,
                effort=3.0,
                velocity=1.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    for flap_name, joint_name, _ in FLAP_LAYOUT:
        ctx.expect_origin_gap(flap_name, "frame", axis="z", min_gap=0.0)
        ctx.expect_aabb_gap(flap_name, "frame", axis="z", max_gap=0.006, max_penetration=0.0)
        ctx.expect_aabb_overlap(flap_name, "frame", axes="xy", min_overlap=0.035)
        ctx.expect_joint_motion_axis(
            joint_name,
            flap_name,
            world_axis="z",
            direction="positive",
            min_delta=0.045,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

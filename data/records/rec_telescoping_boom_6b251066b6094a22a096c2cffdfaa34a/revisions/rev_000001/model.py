from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.48
BASE_WIDTH = 0.34
BASE_THICKNESS = 0.03
PEDESTAL_LENGTH = 0.18
PEDESTAL_WIDTH = 0.24
PEDESTAL_HEIGHT = 0.24
GUSSET_THICKNESS = 0.016

BOOM_CENTER_Z = 0.30
ROOT_SLEEVE_START_X = 0.04

ROOT_LENGTH = 1.00
ROOT_OUTER_Y = 0.220
ROOT_OUTER_Z = 0.180
ROOT_WALL = 0.012

MID_LENGTH = 0.92
MID_OUTER_Y = 0.188
MID_OUTER_Z = 0.148
MID_WALL = 0.010

TIP_LENGTH = 0.88
TIP_OUTER_Y = 0.160
TIP_OUTER_Z = 0.120
TIP_WALL = 0.008

PAD_LENGTH = 0.12

MID_HOME_X = 0.22
MID_TRAVEL = 0.58
TIP_HOME_X = 0.18
TIP_TRAVEL = 0.46


def _rect_tube(
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    *,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> cq.Workplane:
    inner_y = outer_y - 2.0 * wall
    inner_z = outer_z - 2.0 * wall

    top = (
        cq.Workplane("XY")
        .box(length, outer_y, wall)
        .translate((0.5 * length, 0.0, 0.5 * outer_z - 0.5 * wall))
    )
    bottom = (
        cq.Workplane("XY")
        .box(length, outer_y, wall)
        .translate((0.5 * length, 0.0, -0.5 * outer_z + 0.5 * wall))
    )
    right = (
        cq.Workplane("XY")
        .box(length, wall, inner_z)
        .translate((0.5 * length, 0.5 * outer_y - 0.5 * wall, 0.0))
    )
    left = (
        cq.Workplane("XY")
        .box(length, wall, inner_z)
        .translate((0.5 * length, -0.5 * outer_y + 0.5 * wall, 0.0))
    )
    shell = top.union(bottom).union(right).union(left)

    if rear_cap > 0.0:
        rear_frame = _guide_cluster(
            0.5 * rear_cap,
            rear_cap,
            inner_y,
            inner_z,
            max(inner_y - 2.0 * wall, wall),
            max(inner_z - 2.0 * wall, wall),
        )
        shell = shell.union(rear_frame)

    if front_cap > 0.0:
        front_frame = _guide_cluster(
            length - 0.5 * front_cap,
            front_cap,
            inner_y,
            inner_z,
            max(inner_y - 2.0 * wall, wall),
            max(inner_z - 2.0 * wall, wall),
        )
        shell = shell.union(front_frame)

    return shell


def _guide_cluster(
    x_center: float,
    pad_length: float,
    inner_y: float,
    inner_z: float,
    target_y: float,
    target_z: float,
) -> cq.Workplane:
    side_thickness = 0.5 * (inner_y - target_y)
    top_thickness = 0.5 * (inner_z - target_z)

    top = (
        cq.Workplane("XY")
        .box(pad_length, inner_y, top_thickness)
        .translate((x_center, 0.0, 0.5 * target_z + 0.5 * top_thickness))
    )
    bottom = (
        cq.Workplane("XY")
        .box(pad_length, inner_y, top_thickness)
        .translate((x_center, 0.0, -0.5 * target_z - 0.5 * top_thickness))
    )
    right = (
        cq.Workplane("XY")
        .box(pad_length, side_thickness, target_z)
        .translate((x_center, 0.5 * target_y + 0.5 * side_thickness, 0.0))
    )
    left = (
        cq.Workplane("XY")
        .box(pad_length, side_thickness, target_z)
        .translate((x_center, -0.5 * target_y - 0.5 * side_thickness, 0.0))
    )
    return top.union(bottom).union(right).union(left)


def _root_mount_structure() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            PEDESTAL_LENGTH,
            PEDESTAL_WIDTH,
            PEDESTAL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    gusset_profile = [
        (-0.02, BASE_THICKNESS),
        (0.14, BASE_THICKNESS),
        (0.30, BOOM_CENTER_Z - 0.5 * ROOT_OUTER_Z + 0.01),
        (0.08, BOOM_CENTER_Z - 0.5 * ROOT_OUTER_Z + 0.01),
    ]
    gusset = cq.Workplane("XZ").polyline(gusset_profile).close().extrude(GUSSET_THICKNESS)
    right_gusset = gusset.translate((0.0, 0.5 * PEDESTAL_WIDTH - 0.5 * GUSSET_THICKNESS, 0.0))
    left_gusset = gusset.translate((0.0, -0.5 * PEDESTAL_WIDTH - 0.5 * GUSSET_THICKNESS, 0.0))
    return base_plate.union(pedestal).union(right_gusset).union(left_gusset)


def _root_sleeve_shape() -> cq.Workplane:
    sleeve = _rect_tube(
        ROOT_LENGTH,
        ROOT_OUTER_Y,
        ROOT_OUTER_Z,
        ROOT_WALL,
        rear_cap=0.10,
    ).translate((ROOT_SLEEVE_START_X, 0.0, BOOM_CENTER_Z))
    inner_y = ROOT_OUTER_Y - 2.0 * ROOT_WALL
    inner_z = ROOT_OUTER_Z - 2.0 * ROOT_WALL
    rear_guides = _guide_cluster(0.30, PAD_LENGTH, inner_y, inner_z, MID_OUTER_Y, MID_OUTER_Z)
    front_guides = _guide_cluster(0.90, PAD_LENGTH, inner_y, inner_z, MID_OUTER_Y, MID_OUTER_Z)
    guides = rear_guides.union(front_guides).translate((ROOT_SLEEVE_START_X, 0.0, BOOM_CENTER_Z))
    return sleeve.union(guides)


def _mid_stage_shape() -> cq.Workplane:
    shell = _rect_tube(
        MID_LENGTH,
        MID_OUTER_Y,
        MID_OUTER_Z,
        MID_WALL,
    )
    inner_y = MID_OUTER_Y - 2.0 * MID_WALL
    inner_z = MID_OUTER_Z - 2.0 * MID_WALL
    rear_guides = _guide_cluster(0.24, PAD_LENGTH, inner_y, inner_z, TIP_OUTER_Y, TIP_OUTER_Z)
    front_guides = _guide_cluster(0.74, PAD_LENGTH, inner_y, inner_z, TIP_OUTER_Y, TIP_OUTER_Z)
    return shell.union(rear_guides).union(front_guides)


def _tip_nose_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .box(0.035, TIP_OUTER_Y * 0.92, TIP_OUTER_Z * 0.92)
        .translate((TIP_LENGTH - 0.055, 0.0, 0.0))
    )
    nose_wedge = (
        cq.Workplane("XZ")
        .polyline(
            [
                (TIP_LENGTH - 0.11, -0.42 * TIP_OUTER_Z),
                (TIP_LENGTH - 0.04, -0.42 * TIP_OUTER_Z),
                (TIP_LENGTH, 0.0),
                (TIP_LENGTH - 0.04, 0.42 * TIP_OUTER_Z),
                (TIP_LENGTH - 0.11, 0.42 * TIP_OUTER_Z),
            ]
        )
        .close()
        .extrude(TIP_OUTER_Y * 0.62, both=True)
    )
    return collar.union(nose_wedge)


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, visual_name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_rect_tube_box_visuals(
    part,
    *,
    prefix: str,
    start_x: float,
    center_z: float,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    material: str,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> None:
    inner_z = outer_z - 2.0 * wall
    _add_box_visual(
        part,
        (length, outer_y, wall),
        (start_x + 0.5 * length, 0.0, center_z + 0.5 * outer_z - 0.5 * wall),
        material=material,
        name=f"{prefix}_top",
    )
    _add_box_visual(
        part,
        (length, outer_y, wall),
        (start_x + 0.5 * length, 0.0, center_z - 0.5 * outer_z + 0.5 * wall),
        material=material,
        name=f"{prefix}_bottom",
    )
    _add_box_visual(
        part,
        (length, wall, inner_z),
        (start_x + 0.5 * length, 0.5 * outer_y - 0.5 * wall, center_z),
        material=material,
        name=f"{prefix}_right",
    )
    _add_box_visual(
        part,
        (length, wall, inner_z),
        (start_x + 0.5 * length, -0.5 * outer_y + 0.5 * wall, center_z),
        material=material,
        name=f"{prefix}_left",
    )
    if rear_cap > 0.0:
        _add_box_visual(
            part,
            (rear_cap, outer_y, wall),
            (start_x + 0.5 * rear_cap, 0.0, center_z + 0.5 * outer_z - 0.5 * wall),
            material=material,
            name=f"{prefix}_rear_cap_top",
        )
        _add_box_visual(
            part,
            (rear_cap, outer_y, wall),
            (start_x + 0.5 * rear_cap, 0.0, center_z - 0.5 * outer_z + 0.5 * wall),
            material=material,
            name=f"{prefix}_rear_cap_bottom",
        )
        _add_box_visual(
            part,
            (rear_cap, wall, inner_z),
            (start_x + 0.5 * rear_cap, 0.5 * outer_y - 0.5 * wall, center_z),
            material=material,
            name=f"{prefix}_rear_cap_right",
        )
        _add_box_visual(
            part,
            (rear_cap, wall, inner_z),
            (start_x + 0.5 * rear_cap, -0.5 * outer_y + 0.5 * wall, center_z),
            material=material,
            name=f"{prefix}_rear_cap_left",
        )
    if front_cap > 0.0:
        _add_box_visual(
            part,
            (front_cap, outer_y, wall),
            (start_x + length - 0.5 * front_cap, 0.0, center_z + 0.5 * outer_z - 0.5 * wall),
            material=material,
            name=f"{prefix}_front_cap_top",
        )
        _add_box_visual(
            part,
            (front_cap, outer_y, wall),
            (start_x + length - 0.5 * front_cap, 0.0, center_z - 0.5 * outer_z + 0.5 * wall),
            material=material,
            name=f"{prefix}_front_cap_bottom",
        )
        _add_box_visual(
            part,
            (front_cap, wall, inner_z),
            (start_x + length - 0.5 * front_cap, 0.5 * outer_y - 0.5 * wall, center_z),
            material=material,
            name=f"{prefix}_front_cap_right",
        )
        _add_box_visual(
            part,
            (front_cap, wall, inner_z),
            (start_x + length - 0.5 * front_cap, -0.5 * outer_y + 0.5 * wall, center_z),
            material=material,
            name=f"{prefix}_front_cap_left",
        )


def _add_guide_cluster_visuals(
    part,
    *,
    prefix: str,
    x_center: float,
    center_z: float,
    pad_length: float,
    inner_y: float,
    inner_z: float,
    target_y: float,
    target_z: float,
    material: str,
) -> None:
    side_thickness = 0.5 * (inner_y - target_y)
    top_thickness = 0.5 * (inner_z - target_z)
    _add_box_visual(
        part,
        (pad_length, inner_y, top_thickness),
        (x_center, 0.0, center_z + 0.5 * target_z + 0.5 * top_thickness),
        material=material,
        name=f"{prefix}_top_pad",
    )
    _add_box_visual(
        part,
        (pad_length, inner_y, top_thickness),
        (x_center, 0.0, center_z - 0.5 * target_z - 0.5 * top_thickness),
        material=material,
        name=f"{prefix}_bottom_pad",
    )
    _add_box_visual(
        part,
        (pad_length, side_thickness, target_z),
        (x_center, 0.5 * target_y + 0.5 * side_thickness, center_z),
        material=material,
        name=f"{prefix}_right_pad",
    )
    _add_box_visual(
        part,
        (pad_length, side_thickness, target_z),
        (x_center, -0.5 * target_y - 0.5 * side_thickness, center_z),
        material=material,
        name=f"{prefix}_left_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_boom")

    model.material("mount_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("boom_outer_paint", rgba=(0.90, 0.75, 0.18, 1.0))
    model.material("boom_inner_paint", rgba=(0.96, 0.82, 0.24, 1.0))

    root_mount = model.part("root_mount")
    stage_mid = model.part("stage_mid")
    stage_tip = model.part("stage_tip")

    _add_mesh_visual(
        root_mount,
        _root_mount_structure(),
        "root_mount_structure",
        "mount_structure",
        "mount_steel",
    )
    _add_rect_tube_box_visuals(
        root_mount,
        prefix="root_sleeve",
        start_x=ROOT_SLEEVE_START_X,
        center_z=BOOM_CENTER_Z,
        length=ROOT_LENGTH,
        outer_y=ROOT_OUTER_Y,
        outer_z=ROOT_OUTER_Z,
        wall=ROOT_WALL,
        material="boom_outer_paint",
        rear_cap=0.10,
    )
    _add_guide_cluster_visuals(
        root_mount,
        prefix="root_rear_guides",
        x_center=ROOT_SLEEVE_START_X + 0.30,
        center_z=BOOM_CENTER_Z,
        pad_length=PAD_LENGTH,
        inner_y=ROOT_OUTER_Y - 2.0 * ROOT_WALL,
        inner_z=ROOT_OUTER_Z - 2.0 * ROOT_WALL,
        target_y=MID_OUTER_Y,
        target_z=MID_OUTER_Z,
        material="mount_steel",
    )
    _add_guide_cluster_visuals(
        root_mount,
        prefix="root_front_guides",
        x_center=ROOT_SLEEVE_START_X + 0.90,
        center_z=BOOM_CENTER_Z,
        pad_length=PAD_LENGTH,
        inner_y=ROOT_OUTER_Y - 2.0 * ROOT_WALL,
        inner_z=ROOT_OUTER_Z - 2.0 * ROOT_WALL,
        target_y=MID_OUTER_Y,
        target_z=MID_OUTER_Z,
        material="mount_steel",
    )
    _add_rect_tube_box_visuals(
        stage_mid,
        prefix="mid_stage",
        start_x=0.0,
        center_z=0.0,
        length=MID_LENGTH,
        outer_y=MID_OUTER_Y,
        outer_z=MID_OUTER_Z,
        wall=MID_WALL,
        material="boom_outer_paint",
    )
    _add_guide_cluster_visuals(
        stage_mid,
        prefix="mid_rear_guides",
        x_center=0.24,
        center_z=0.0,
        pad_length=PAD_LENGTH,
        inner_y=MID_OUTER_Y - 2.0 * MID_WALL,
        inner_z=MID_OUTER_Z - 2.0 * MID_WALL,
        target_y=TIP_OUTER_Y,
        target_z=TIP_OUTER_Z,
        material="mount_steel",
    )
    _add_guide_cluster_visuals(
        stage_mid,
        prefix="mid_front_guides",
        x_center=0.74,
        center_z=0.0,
        pad_length=PAD_LENGTH,
        inner_y=MID_OUTER_Y - 2.0 * MID_WALL,
        inner_z=MID_OUTER_Z - 2.0 * MID_WALL,
        target_y=TIP_OUTER_Y,
        target_z=TIP_OUTER_Z,
        material="mount_steel",
    )
    _add_rect_tube_box_visuals(
        stage_tip,
        prefix="tip_stage",
        start_x=0.0,
        center_z=0.0,
        length=TIP_LENGTH,
        outer_y=TIP_OUTER_Y,
        outer_z=TIP_OUTER_Z,
        wall=TIP_WALL,
        material="boom_inner_paint",
        front_cap=0.02,
    )
    _add_mesh_visual(
        stage_tip,
        _tip_nose_shape(),
        "tip_nose",
        "tip_nose",
        "boom_inner_paint",
    )

    model.articulation(
        "root_to_mid",
        ArticulationType.PRISMATIC,
        parent=root_mount,
        child=stage_mid,
        origin=Origin(xyz=(MID_HOME_X, 0.0, BOOM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.25,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "mid_to_tip",
        ArticulationType.PRISMATIC,
        parent=stage_mid,
        child=stage_tip,
        origin=Origin(xyz=(TIP_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.30,
            lower=0.0,
            upper=TIP_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_mount = object_model.get_part("root_mount")
    stage_mid = object_model.get_part("stage_mid")
    stage_tip = object_model.get_part("stage_tip")
    root_to_mid = object_model.get_articulation("root_to_mid")
    mid_to_tip = object_model.get_articulation("mid_to_tip")

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
        "boom_parts_present",
        root_mount is not None and stage_mid is not None and stage_tip is not None,
        "Expected root mount, middle stage, and tip stage parts.",
    )
    ctx.check(
        "prismatic_axes_follow_boom",
        root_to_mid.axis == (1.0, 0.0, 0.0) and mid_to_tip.axis == (1.0, 0.0, 0.0),
        f"Expected both prismatic axes to be +X, got {root_to_mid.axis} and {mid_to_tip.axis}.",
    )

    ctx.expect_contact(stage_mid, root_mount, name="mid_stage_guided_by_root")
    ctx.expect_contact(stage_tip, stage_mid, name="tip_stage_guided_by_mid")
    ctx.expect_overlap(stage_mid, root_mount, axes="yz", min_overlap=0.145, name="mid_stage_aligned_in_root")
    ctx.expect_overlap(stage_tip, stage_mid, axes="yz", min_overlap=0.118, name="tip_stage_aligned_in_mid")

    closed_mid_x = ctx.part_world_position(stage_mid)[0]
    closed_tip_x = ctx.part_world_position(stage_tip)[0]
    with ctx.pose({root_to_mid: MID_TRAVEL, mid_to_tip: TIP_TRAVEL}):
        ctx.expect_contact(
            stage_mid,
            root_mount,
            name="mid_stage_stays_supported_at_full_extension",
        )
        ctx.expect_contact(
            stage_tip,
            stage_mid,
            name="tip_stage_stays_supported_at_full_extension",
        )
        ctx.expect_overlap(
            stage_mid,
            root_mount,
            axes="yz",
            min_overlap=0.145,
            name="mid_stage_stays_aligned_in_root",
        )
        ctx.expect_overlap(
            stage_tip,
            stage_mid,
            axes="yz",
            min_overlap=0.118,
            name="tip_stage_stays_aligned_in_mid",
        )
        ctx.expect_overlap(
            stage_mid,
            root_mount,
            axes="x",
            min_overlap=0.20,
            name="mid_stage_remains_captured_in_root",
        )
        ctx.expect_overlap(
            stage_tip,
            stage_mid,
            axes="x",
            min_overlap=0.22,
            name="tip_stage_remains_captured_in_mid",
        )
        open_mid_x = ctx.part_world_position(stage_mid)[0]
        open_tip_x = ctx.part_world_position(stage_tip)[0]
        ctx.check(
            "middle_stage_extends_forward",
            open_mid_x > closed_mid_x + 0.50,
            f"Expected middle stage to move forward by >0.50 m, got {open_mid_x - closed_mid_x:.3f} m.",
        )
        ctx.check(
            "tip_stage_extends_forward",
            open_tip_x > closed_tip_x + 0.85,
            f"Expected tip stage to move forward by >0.85 m, got {open_tip_x - closed_tip_x:.3f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

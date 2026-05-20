from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lamp_shell_mesh():
    outer_profile = [
        (0.11, -0.26),
        (0.16, -0.22),
        (0.23, -0.16),
        (0.29, -0.02),
        (0.31, 0.16),
        (0.34, 0.34),
        (0.36, 0.44),
        (0.38, 0.49),
    ]
    inner_profile = [
        (0.08, -0.24),
        (0.13, -0.20),
        (0.20, -0.14),
        (0.26, -0.01),
        (0.28, 0.16),
        (0.31, 0.33),
        (0.33, 0.43),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56),
        "searchlight_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.59, 0.60, 0.61, 1.0))
    tower_gray = model.material("tower_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    galvanized = model.material("galvanized", rgba=(0.64, 0.66, 0.69, 1.0))
    lamp_olive = model.material("lamp_olive", rgba=(0.33, 0.38, 0.28, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.10, 0.11, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.83, 0.91, 0.98, 0.55))

    lamp_shell_mesh = _lamp_shell_mesh()

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.20, 1.20, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="foundation_block",
    )
    tower_base.visual(
        Box((0.74, 0.74, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=tower_gray,
        name="base_plinth",
    )
    tower_base.visual(
        Cylinder(radius=0.17, length=1.95),
        origin=Origin(xyz=(0.0, 0.0, 1.435)),
        material=galvanized,
        name="mast_tube",
    )
    tower_base.visual(
        Cylinder(radius=0.27, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 2.52)),
        material=tower_gray,
        name="pan_bearing_pedestal",
    )
    tower_base.visual(
        Box((0.22, 0.18, 0.92)),
        origin=Origin(xyz=(0.0, 0.34, 2.27)),
        material=tower_gray,
        name="left_stage_cheek",
    )
    tower_base.visual(
        Box((0.22, 0.18, 0.92)),
        origin=Origin(xyz=(0.0, -0.34, 2.27)),
        material=tower_gray,
        name="right_stage_cheek",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 2.80)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.23, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=lamp_black,
        name="slew_ring",
    )
    pan_stage.visual(
        Cylinder(radius=0.18, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=tower_gray,
        name="pan_turret",
    )
    pan_stage.visual(
        Box((0.76, 0.54, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=tower_gray,
        name="deck_plate",
    )
    pan_stage.visual(
        Box((0.16, 0.16, 0.18)),
        origin=Origin(xyz=(-0.22, 0.0, 0.53)),
        material=tower_gray,
        name="rear_drive_box",
    )
    pan_stage.visual(
        Box((0.34, 0.20, 0.24)),
        origin=Origin(xyz=(0.02, 0.36, 0.52)),
        material=tower_gray,
        name="left_arm_base",
    )
    pan_stage.visual(
        Box((0.34, 0.20, 0.24)),
        origin=Origin(xyz=(0.02, -0.36, 0.52)),
        material=tower_gray,
        name="right_arm_base",
    )
    pan_stage.visual(
        Box((0.58, 0.10, 0.70)),
        origin=Origin(xyz=(-0.11, 0.46, 0.83)),
        material=galvanized,
        name="left_yoke_arm",
    )
    pan_stage.visual(
        Box((0.58, 0.10, 0.70)),
        origin=Origin(xyz=(-0.11, -0.46, 0.83)),
        material=galvanized,
        name="right_yoke_arm",
    )
    pan_stage.visual(
        Box((0.16, 1.02, 0.12)),
        origin=Origin(xyz=(-0.33, 0.0, 1.18)),
        material=galvanized,
        name="yoke_crossbeam",
    )
    pan_stage.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.08, 0.47, 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_black,
        name="left_trunnion_housing",
    )
    pan_stage.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.08, -0.47, 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_black,
        name="right_trunnion_housing",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.82, 0.92, 1.06)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        lamp_shell_mesh,
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_olive,
        name="body_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.30, length=0.16),
        origin=Origin(xyz=(0.45, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_black,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.25, length=0.10),
        origin=Origin(xyz=(0.54, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.09, length=0.18),
        origin=Origin(xyz=(-0.23, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_black,
        name="rear_cap",
    )
    lamp_head.visual(
        Box((0.20, 0.34, 0.32)),
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
        material=lamp_black,
        name="rear_ballast_box",
    )
    lamp_head.visual(
        Box((0.16, 0.12, 0.22)),
        origin=Origin(xyz=(0.08, 0.31, 0.0)),
        material=lamp_black,
        name="left_trunnion_boss",
    )
    lamp_head.visual(
        Box((0.16, 0.12, 0.22)),
        origin=Origin(xyz=(0.08, -0.31, 0.0)),
        material=lamp_black,
        name="right_trunnion_boss",
    )
    lamp_head.visual(
        Cylinder(radius=0.05, length=0.09),
        origin=Origin(xyz=(0.08, 0.365, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_black,
        name="left_trunnion_journal",
    )
    lamp_head.visual(
        Cylinder(radius=0.05, length=0.09),
        origin=Origin(xyz=(0.08, -0.365, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lamp_black,
        name="right_trunnion_journal",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.88, 0.72, 0.72)),
        mass=95.0,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 2.63)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "pan_to_lamp",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.08, 0.0, 0.94)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=-0.55,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tower_base = object_model.get_part("tower_base")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("mast_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_lamp")

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            lamp_head,
            pan_stage,
            axis="z",
            positive_elem="body_shell",
            negative_elem="deck_plate",
            min_gap=0.10,
            name="lamp body clears the rotating deck",
        )
        ctx.expect_overlap(
            pan_stage,
            tower_base,
            axes="xy",
            elem_a="slew_ring",
            elem_b="pan_bearing_pedestal",
            min_overlap=0.18,
            name="pan stage stays centered on the mast pedestal",
        )

    rest_front = ctx.part_world_position(lamp_head)
    with ctx.pose({tilt_joint: 1.0}):
        tilted_front = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    with ctx.pose({tilt_joint: 0.0}):
        rest_front_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")

    rest_pan_pos = ctx.part_world_position(pan_stage)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_pan_pos = ctx.part_world_position(pan_stage)

    lens_raises = (
        tilted_front is not None
        and rest_front_aabb is not None
        and tilted_front[1][2] > rest_front_aabb[1][2] + 0.18
    )
    ctx.check(
        "tilt joint raises the lamp nose",
        lens_raises,
        details=f"rest_front={rest_front_aabb}, tilted_front={tilted_front}",
    )
    ctx.check(
        "pan rotation keeps the stage on the mast axis",
        rest_pan_pos is not None
        and turned_pan_pos is not None
        and abs(rest_pan_pos[0] - turned_pan_pos[0]) < 1e-6
        and abs(rest_pan_pos[1] - turned_pan_pos[1]) < 1e-6
        and abs(rest_pan_pos[2] - turned_pan_pos[2]) < 1e-6,
        details=f"rest={rest_pan_pos}, turned={turned_pan_pos}, lamp_rest={rest_front}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

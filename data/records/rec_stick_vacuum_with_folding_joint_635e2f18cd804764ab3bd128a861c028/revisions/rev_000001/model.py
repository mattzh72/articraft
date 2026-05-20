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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))
    red = model.material("red", rgba=(0.76, 0.18, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.82, 1.0))
    dark = model.material("dark", rgba=(0.10, 0.10, 0.11, 1.0))

    fold_height = 0.62
    fold_x = 0.05
    wand_angle = 1.16

    body = model.part("body")
    body.visual(
        Box((0.07, 0.06, 0.36)),
        origin=Origin(xyz=(-0.05, 0.0, 0.51)),
        material=graphite,
        name="spine",
    )
    body.visual(
        Box((0.060, 0.050, 0.100)),
        origin=Origin(xyz=(-0.010, 0.0, 0.570)),
        material=graphite,
        name="hinge_tower",
    )
    body.visual(
        Box((0.050, 0.018, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.610)),
        material=graphite,
        name="hinge_neck",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.055),
        origin=Origin(xyz=(fold_x, 0.0, fold_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="body_hinge_barrel",
    )
    body.visual(
        Box((0.12, 0.09, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.69)),
        material=graphite,
        name="motor_mount",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.18),
        origin=Origin(xyz=(-0.15, 0.0, 0.74), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="motor_barrel",
    )
    body.visual(
        Cylinder(radius=0.04, length=0.13),
        origin=Origin(xyz=(-0.055, 0.0, 0.61)),
        material=red,
        name="cyclone",
    )
    body.visual(
        Box((0.09, 0.055, 0.20)),
        origin=Origin(xyz=(-0.20, 0.0, 0.60)),
        material=graphite,
        name="battery_pack",
    )
    body.visual(
        Box((0.10, 0.035, 0.05)),
        origin=Origin(xyz=(-0.20, 0.0, 0.79)),
        material=graphite,
        name="handle_bridge",
    )
    body.visual(
        Box((0.04, 0.038, 0.18)),
        origin=Origin(xyz=(-0.26, 0.0, 0.78)),
        material=graphite,
        name="handle_stem",
    )

    wand = model.part("wand")
    wand.visual(
        Box((0.026, 0.006, 0.040)),
        origin=Origin(xyz=(0.022, 0.0305, 0.0)),
        material=dark,
        name="left_fold_cheek",
    )
    wand.visual(
        Box((0.026, 0.006, 0.040)),
        origin=Origin(xyz=(0.022, -0.0305, 0.0)),
        material=dark,
        name="right_fold_cheek",
    )
    wand.visual(
        Box((0.040, 0.055, 0.016)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=dark,
        name="fold_core",
    )
    wand.visual(
        Box((0.032, 0.058, 0.020)),
        origin=Origin(xyz=(0.043, 0.0, -0.014)),
        material=dark,
        name="fold_bridge",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.60),
        origin=Origin(xyz=(0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tube",
    )
    wand.visual(
        Box((0.04, 0.042, 0.04)),
        origin=Origin(xyz=(0.615, 0.0, 0.0)),
        material=dark,
        name="neck_socket",
    )
    wand.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.635, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="neck_pivot",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.028, 0.008, 0.030)),
        origin=Origin(xyz=(0.012, 0.021, -0.015)),
        material=dark,
        name="left_hinge_cheek",
    )
    floor_head.visual(
        Box((0.028, 0.008, 0.030)),
        origin=Origin(xyz=(0.012, -0.021, -0.015)),
        material=dark,
        name="right_hinge_cheek",
    )
    floor_head.visual(
        Box((0.050, 0.034, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, -0.026)),
        material=dark,
        name="neck_bridge",
    )
    floor_head.visual(
        Box((0.12, 0.06, 0.022)),
        origin=Origin(xyz=(0.085, 0.0, -0.005)),
        material=graphite,
        name="top_hump",
    )
    floor_head.visual(
        Box((0.30, 0.09, 0.026)),
        origin=Origin(xyz=(0.19, 0.0, -0.018)),
        material=graphite,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.04, 0.09, 0.014)),
        origin=Origin(xyz=(0.332, 0.0, -0.024)),
        material=red,
        name="front_lip",
    )
    floor_head.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.06, 0.044, -0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="left_wheel",
    )
    floor_head.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.06, -0.044, -0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="right_wheel",
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(fold_x, 0.0, fold_height), rpy=(0.0, wand_angle, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.635, 0.0, 0.0), rpy=(0.0, -wand_angle, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.55,
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

    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("body_to_wand_fold")
    head_pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    body_hinge_z = fold.origin.xyz[2]
    ctx.check(
        "fold hinge sits well above the floor line",
        body_hinge_z >= 0.55,
        details=f"fold hinge z={body_hinge_z:.3f} m",
    )

    head_aabb = ctx.part_world_aabb(floor_head)
    ctx.check(
        "floor head rests close to the ground plane",
        head_aabb is not None and 0.0 <= head_aabb[0][2] <= 0.01,
        details=f"floor head aabb={head_aabb}",
    )

    ctx.expect_gap(
        wand,
        floor_head,
        axis="z",
        min_gap=0.0,
        max_gap=0.02,
        positive_elem="neck_pivot",
        negative_elem="neck_bridge",
        name="floor head hinge bracket stays just below the wand pivot",
    )

    head_rest = ctx.part_world_aabb(floor_head)
    with ctx.pose({fold: fold.motion_limits.upper}):
        head_folded = ctx.part_world_aabb(floor_head)
    ctx.check(
        "fold joint lifts the floor head when folded",
        head_rest is not None
        and head_folded is not None
        and head_folded[0][2] > head_rest[0][2] + 0.18,
        details=f"rest={head_rest}, folded={head_folded}",
    )

    front_rest = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    with ctx.pose({head_pitch: head_pitch.motion_limits.upper}):
        front_raised = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    ctx.check(
        "floor head pitch raises the nozzle front",
        front_rest is not None
        and front_raised is not None
        and front_raised[0][2] > front_rest[0][2] + 0.02,
        details=f"rest={front_rest}, raised={front_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

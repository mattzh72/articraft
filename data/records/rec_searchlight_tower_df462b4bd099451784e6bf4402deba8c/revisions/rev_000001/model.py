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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    base_paint = model.material("base_paint", rgba=(0.30, 0.33, 0.36, 1.0))
    mast_paint = model.material("mast_paint", rgba=(0.77, 0.79, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.48, 0.52, 0.42, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.17, 1.0))

    brace_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.28, 0.0, 0.78),
                (0.22, 0.0, 1.46),
                (0.12, 0.0, 2.30),
            ],
            radius=0.028,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "mast_brace",
    )

    base_tower = model.part("base_tower")
    base_tower.visual(
        Box((1.40, 1.40, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=base_paint,
        name="foundation",
    )
    base_tower.visual(
        Box((0.82, 0.82, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=base_paint,
        name="equipment_plinth",
    )
    base_tower.visual(
        Cylinder(radius=0.12, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
        material=mast_paint,
        name="mast_column",
    )
    base_tower.visual(
        Cylinder(radius=0.22, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 2.60)),
        material=steel,
        name="pan_bearing_housing",
    )
    for index in range(4):
        base_tower.visual(
            brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, index * (math.pi / 2.0))),
            material=steel,
            name=f"mast_brace_{index}",
        )
    base_tower.inertial = Inertial.from_geometry(
        Box((1.40, 1.40, 2.72)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
    )

    yoke_stage = model.part("yoke_stage")
    yoke_stage.visual(
        Cylinder(radius=0.24, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_trim,
        name="turntable_drum",
    )
    yoke_stage.visual(
        Cylinder(radius=0.32, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=steel,
        name="turntable_deck",
    )
    yoke_stage.visual(
        Box((0.16, 0.78, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=steel,
        name="yoke_crossbeam",
    )
    yoke_stage.visual(
        Box((0.10, 0.08, 0.70)),
        origin=Origin(xyz=(0.0, 0.34, 0.67)),
        material=steel,
        name="left_arm",
    )
    yoke_stage.visual(
        Box((0.10, 0.08, 0.70)),
        origin=Origin(xyz=(0.0, -0.34, 0.67)),
        material=steel,
        name="right_arm",
    )
    yoke_stage.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, 0.34, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="left_bearing",
    )
    yoke_stage.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, -0.34, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="right_bearing",
    )
    yoke_stage.inertial = Inertial.from_geometry(
        Box((0.64, 0.84, 1.06)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.24, length=0.46),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="main_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.30, length=0.12),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.18, length=0.18),
        origin=Origin(xyz=(-0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="rear_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(-0.39, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="rear_cap",
    )
    lamp_head.visual(
        Box((0.22, 0.18, 0.09)),
        origin=Origin(xyz=(-0.02, 0.0, 0.265)),
        material=dark_trim,
        name="top_access_box",
    )
    lamp_head.visual(
        Cylinder(radius=0.06, length=0.10),
        origin=Origin(xyz=(0.0, 0.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.06, length=0.10),
        origin=Origin(xyz=(0.0, -0.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.82, 0.56, 0.62)),
        mass=160.0,
        origin=Origin(xyz=(-0.04, 0.0, 0.03)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=base_tower,
        child=yoke_stage,
        origin=Origin(xyz=(0.0, 0.0, 2.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=yoke_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.8,
            lower=math.radians(-25.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_tower = object_model.get_part("base_tower")
    yoke_stage = object_model.get_part("yoke_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_origin_gap(
        yoke_stage,
        base_tower,
        axis="z",
        min_gap=2.6,
        name="pan stage sits high above the base",
    )
    ctx.expect_contact(
        yoke_stage,
        lamp_head,
        elem_a="left_bearing",
        elem_b="left_trunnion",
        name="left trunnion contacts the left bearing housing",
    )
    ctx.expect_contact(
        lamp_head,
        yoke_stage,
        elem_a="right_trunnion",
        elem_b="right_bearing",
        name="right trunnion contacts the right bearing housing",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    with ctx.pose({pan_axis: math.pi / 2.0}):
        pan_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    ctx.check(
        "positive pan swings the lamp toward +Y",
        rest_front is not None
        and pan_front is not None
        and pan_front[1] > rest_front[1] + 0.22
        and abs(pan_front[0]) < rest_front[0] * 0.35,
        details=f"rest_front={rest_front}, pan_front={pan_front}",
    )

    with ctx.pose({tilt_axis: math.radians(45.0)}):
        tilt_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    ctx.check(
        "positive tilt raises the lamp nose",
        rest_front is not None
        and tilt_front is not None
        and tilt_front[2] > rest_front[2] + 0.18,
        details=f"rest_front={rest_front}, tilt_front={tilt_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

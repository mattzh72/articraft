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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    base_metal = model.material("base_metal", rgba=(0.25, 0.27, 0.29, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.56, 0.58, 0.52, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.78, 0.79, 0.74, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.16, 0.16, 0.18, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.84, 0.96, 0.65))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=base_metal,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.18, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=base_metal,
        name="pedestal",
    )
    tower.visual(
        Cylinder(radius=0.09, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=painted_steel,
        name="mast",
    )
    tower.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
        material=hub_dark,
        name="mast_cap",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.84, 0.84, 1.62)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    yoke_stage = model.part("yoke_stage")
    yoke_stage.visual(
        Cylinder(radius=0.28, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=hub_dark,
        name="turntable",
    )
    yoke_stage.visual(
        Cylinder(radius=0.12, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=hub_dark,
        name="bearing_housing",
    )
    yoke_stage.visual(
        Box((0.18, 0.035, 0.84)),
        origin=Origin(xyz=(0.0, 0.28, 0.52)),
        material=painted_steel,
        name="left_arm",
    )
    yoke_stage.visual(
        Box((0.18, 0.035, 0.84)),
        origin=Origin(xyz=(0.0, -0.28, 0.52)),
        material=painted_steel,
        name="right_arm",
    )
    yoke_stage.visual(
        Box((0.18, 0.60, 0.12)),
        origin=Origin(xyz=(-0.10, 0.0, 0.26)),
        material=painted_steel,
        name="rear_bridge",
    )
    yoke_stage.visual(
        Cylinder(radius=0.085, length=0.03),
        origin=Origin(xyz=(0.0, 0.312, 0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="left_hub",
    )
    yoke_stage.visual(
        Cylinder(radius=0.085, length=0.03),
        origin=Origin(xyz=(0.0, -0.312, 0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="right_hub",
    )
    yoke_stage.inertial = Inertial.from_geometry(
        Box((0.56, 0.64, 0.94)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )

    lamp_head = model.part("lamp_head")
    shell_outer_profile = [
        (0.075, -0.24),
        (0.150, -0.19),
        (0.220, -0.05),
        (0.228, 0.10),
        (0.246, 0.22),
        (0.282, 0.34),
    ]
    shell_inner_profile = [
        (0.0, -0.21),
        (0.125, -0.17),
        (0.182, -0.05),
        (0.188, 0.10),
        (0.212, 0.29),
    ]
    lamp_shell_geom = LatheGeometry.from_shell_profiles(
        shell_outer_profile,
        shell_inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    lamp_shell_geom.rotate_y(math.pi / 2.0)
    lamp_shell_mesh = mesh_from_geometry(lamp_shell_geom, "lamp_shell")
    lamp_head.visual(
        lamp_shell_mesh,
        material=lamp_paint,
        name="lamp_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.228, length=0.024),
        origin=Origin(xyz=(0.302, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.066, length=0.043),
        origin=Origin(xyz=(0.0, 0.241, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.066, length=0.043),
        origin=Origin(xyz=(0.0, -0.241, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="right_trunnion",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.62, 0.52, 0.52)),
        mass=46.0,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_yoke_stage",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.7,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    model.articulation(
        "yoke_stage_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=yoke_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=-math.radians(30.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    yoke_stage = object_model.get_part("yoke_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan = object_model.get_articulation("tower_to_yoke_stage")
    tilt = object_model.get_articulation("yoke_stage_to_lamp_head")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_gap(
        yoke_stage,
        tower,
        axis="z",
        positive_elem="turntable",
        negative_elem="mast_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="turntable seats on mast cap",
    )
    ctx.expect_overlap(
        yoke_stage,
        tower,
        axes="xy",
        elem_a="turntable",
        elem_b="mast_cap",
        min_overlap=0.24,
        name="turntable stays centered over mast support",
    )

    rest_lens_center = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    with ctx.pose({pan: math.radians(60.0)}):
        pan_lens_center = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "positive pan swings lens toward +Y",
        rest_lens_center is not None
        and pan_lens_center is not None
        and pan_lens_center[1] > rest_lens_center[1] + 0.18,
        details=f"rest={rest_lens_center}, pan60={pan_lens_center}",
    )

    with ctx.pose({tilt: math.radians(55.0)}):
        tilted_lens_center = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "positive tilt raises the beam",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.16,
        details=f"rest={rest_lens_center}, tilt55={tilted_lens_center}",
    )

    with ctx.pose({tilt: math.radians(-25.0)}):
        ctx.expect_gap(
            lamp_head,
            yoke_stage,
            axis="z",
            positive_elem="lamp_shell",
            negative_elem="turntable",
            max_penetration=0.0,
            name="lamp shell clears turntable at downward tilt",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

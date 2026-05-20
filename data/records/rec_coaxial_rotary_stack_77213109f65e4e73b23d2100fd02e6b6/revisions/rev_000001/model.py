from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def cylinder_section(radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def annular_section(
    outer_radius: float, inner_radius: float, height: float, z0: float
) -> cq.Workplane:
    outer = cylinder_section(outer_radius, height, z0)
    inner = cylinder_section(inner_radius, height + 0.002, z0 - 0.001)
    return outer.cut(inner)


def build_base_cup() -> cq.Workplane:
    foot = cylinder_section(0.105, 0.012, 0.0)
    body = cylinder_section(0.095, 0.080, 0.0)
    cavity = cylinder_section(0.076, 0.066, 0.014)
    return foot.union(body).cut(cavity)


def build_lower_stage() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    lower_shaft = cylinder_section(0.056, 0.036, -0.036)
    drum = (
        cylinder_section(0.086, 0.008, 0.000)
        .union(cylinder_section(0.073, 0.034, 0.008))
    )
    upper_sleeve = annular_section(0.058, 0.0405, 0.020, 0.042)
    return lower_shaft, drum, upper_sleeve


def build_upper_stage() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    upper_shaft = cylinder_section(0.036, 0.020, -0.020)
    upper_collar = cylinder_section(0.048, 0.010, 0.000)
    tooling_head = cylinder_section(0.054, 0.034, 0.010)
    return upper_shaft, upper_collar, tooling_head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collared_coaxial_rotary_head")

    base_color = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    stage_color = model.material("machined_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    base_cup = model.part("base_cup")
    base_cup.visual(
        mesh_from_cadquery(build_base_cup(), "base_cup_shell"),
        material=base_color,
        name="cup_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_shaft_shape, lower_drum_shape, upper_sleeve_shape = build_lower_stage()
    lower_stage.visual(
        mesh_from_cadquery(lower_shaft_shape, "lower_stage_shaft"),
        material=stage_color,
        name="lower_shaft",
    )
    lower_stage.visual(
        mesh_from_cadquery(lower_drum_shape, "lower_stage_drum"),
        material=stage_color,
        name="lower_drum",
    )
    lower_stage.visual(
        mesh_from_cadquery(upper_sleeve_shape, "lower_stage_upper_sleeve"),
        material=stage_color,
        name="upper_sleeve",
    )

    upper_stage = model.part("upper_stage")
    upper_shaft_shape, upper_collar_shape, tooling_head_shape = build_upper_stage()
    upper_stage.visual(
        mesh_from_cadquery(upper_shaft_shape, "upper_stage_shaft"),
        material=stage_color,
        name="upper_shaft",
    )
    upper_stage.visual(
        mesh_from_cadquery(upper_collar_shape, "upper_stage_collar"),
        material=stage_color,
        name="upper_collar",
    )
    upper_stage.visual(
        mesh_from_cadquery(tooling_head_shape, "upper_stage_tooling_head"),
        material=stage_color,
        name="tooling_head",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base_cup,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=5.0,
            lower=-math.pi,
            upper=math.pi,
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

    base_cup = object_model.get_part("base_cup")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("base_to_lower_stage")
    upper_joint = object_model.get_articulation("lower_to_upper_stage")

    ctx.check(
        "lower stage uses a vertical revolute axis",
        tuple(round(v, 6) for v in lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper stage uses a vertical revolute axis",
        tuple(round(v, 6) for v in upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_joint.axis}",
    )

    ctx.expect_overlap(
        lower_stage,
        base_cup,
        axes="xy",
        elem_a="lower_shaft",
        elem_b="cup_shell",
        min_overlap=0.10,
        name="lower shaft stays centered over the base cup",
    )
    ctx.expect_gap(
        lower_stage,
        base_cup,
        axis="z",
        positive_elem="lower_drum",
        negative_elem="cup_shell",
        min_gap=0.0,
        max_gap=0.02,
        name="lower drum sits just above the base cup rim",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_stage,
        axes="xy",
        elem_a="upper_shaft",
        elem_b="upper_sleeve",
        min_overlap=0.07,
        name="upper shaft stays centered within the lower stage collar",
    )
    ctx.expect_gap(
        upper_stage,
        lower_stage,
        axis="z",
        positive_elem="tooling_head",
        negative_elem="upper_sleeve",
        min_gap=0.0,
        max_gap=0.02,
        name="tooling head rises cleanly above the lower stage collar",
    )

    lower_rest = ctx.part_world_position(lower_stage)
    upper_rest = ctx.part_world_position(upper_stage)
    with ctx.pose({lower_joint: 1.1, upper_joint: -0.8}):
        lower_rotated = ctx.part_world_position(lower_stage)
        upper_rotated = ctx.part_world_position(upper_stage)

    def unchanged(rest: tuple[float, float, float] | None, moved: tuple[float, float, float] | None) -> bool:
        if rest is None or moved is None:
            return False
        return max(abs(a - b) for a, b in zip(rest, moved)) <= 1e-6

    ctx.check(
        "lower stage rotates in place about the shared centerline",
        unchanged(lower_rest, lower_rotated),
        details=f"rest={lower_rest}, rotated={lower_rotated}",
    )
    ctx.check(
        "upper stage rotates in place about the shared centerline",
        unchanged(upper_rest, upper_rotated),
        details=f"rest={upper_rest}, rotated={upper_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

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


SECTION_LEN = 0.74
SLIDE_TRAVEL = 0.46
COLLAPSED_STEP = 0.11
RUNG_Y = 0.052


def _add_stage(
    part,
    *,
    rail_x: float,
    stile_width: float,
    stile_depth: float,
    rung_z: float,
    rung_radius: float,
    aluminium,
    dark_plastic,
    rubber=None,
    is_base: bool = False,
) -> None:
    """Add one compact telescoping ladder section in the part's local frame."""
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((stile_width, stile_depth, SECTION_LEN)),
            origin=Origin(xyz=(sign * rail_x, 0.0, SECTION_LEN / 2.0)),
            material=aluminium,
            name=f"{side}_stile",
        )
        # Dark guide collars at the top make each stile read as a sleeve.
        part.visual(
            Box((stile_width + 0.012, stile_depth + 0.012, 0.050)),
            origin=Origin(xyz=(sign * rail_x, 0.0, SECTION_LEN - 0.025)),
            material=dark_plastic,
            name=f"{side}_collar",
        )

    inner_span = 2.0 * (rail_x - stile_width * 0.50) + 0.034
    part.visual(
        Cylinder(radius=rung_radius, length=inner_span),
        origin=Origin(xyz=(0.0, RUNG_Y, rung_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminium,
        name="rung",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((0.030, 0.040, 0.058)),
            origin=Origin(
                xyz=(sign * (rail_x - stile_width * 0.52), RUNG_Y * 0.50, rung_z)
            ),
            material=aluminium,
            name=f"{side}_rung_socket",
        )

    if is_base and rubber is not None:
        part.visual(
            Cylinder(radius=0.026, length=0.720),
            origin=Origin(xyz=(0.0, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminium,
            name="base_spreader",
        )
        for side, sign in (("left", -1.0), ("right", 1.0)):
            part.visual(
                Box((0.120, 0.085, 0.045)),
                origin=Origin(xyz=(sign * 0.360, 0.0, 0.025)),
                material=rubber,
                name=f"{side}_foot",
            )


def _add_catch(part, *, latch_x: float, yellow, dark_plastic) -> None:
    """A front release bar with two locking tabs, hinged about local X."""
    part.visual(
        Cylinder(radius=0.006, length=2.0 * latch_x + 0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="pivot_rod",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((0.038, 0.012, 0.075)),
            origin=Origin(xyz=(sign * latch_x, 0.010, -0.040)),
            material=yellow,
            name=f"{side}_tab",
        )
    part.visual(
        Box((0.105, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.006, -0.012)),
        material=yellow,
        name="release_paddle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_telescoping_aluminium_ladder")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.76, 0.78, 1.0))
    dark_plastic = model.material("black_plastic", rgba=(0.02, 0.022, 0.025, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.01, 0.012, 0.012, 1.0))
    yellow = model.material("yellow_locking_catch", rgba=(1.0, 0.72, 0.08, 1.0))

    stage_specs = (
        ("base_section", 0.285, 0.080, 0.060, 0.170, 0.019),
        ("section_1", 0.275, 0.060, 0.046, 0.330, 0.018),
        ("section_2", 0.268, 0.045, 0.035, 0.430, 0.017),
        ("section_3", 0.263, 0.034, 0.027, 0.665, 0.016),
    )

    parts = {}
    for idx, (name, rail_x, width, depth, rung_z, rung_radius) in enumerate(stage_specs):
        stage = model.part(name)
        _add_stage(
            stage,
            rail_x=rail_x,
            stile_width=width,
            stile_depth=depth,
            rung_z=rung_z,
            rung_radius=rung_radius,
            aluminium=aluminium,
            dark_plastic=dark_plastic,
            rubber=rubber,
            is_base=(idx == 0),
        )
        parts[name] = stage

    previous_name = "base_section"
    for idx, name in enumerate(("section_1", "section_2", "section_3"), start=1):
        joint = model.articulation(
            f"section_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=parts[previous_name],
            child=parts[name],
            origin=Origin(xyz=(0.0, 0.0, COLLAPSED_STEP)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=180.0, velocity=0.28, lower=0.0, upper=SLIDE_TRAVEL
            ),
        )
        joint.meta["qc_samples"] = [0.0, SLIDE_TRAVEL]
        previous_name = name

    # Spring catches sit on the front of each lower sleeve and flip outward to release.
    catch_mounts = (
        ("catch_1", "base_section", 0.055, RUNG_Y + 0.025, 0.170),
        ("catch_2", "section_1", 0.055, RUNG_Y + 0.024, 0.330),
        ("catch_3", "section_2", 0.055, RUNG_Y + 0.023, 0.430),
    )
    for idx, (catch_name, parent_name, latch_x, y_mount, z_mount) in enumerate(catch_mounts, start=1):
        catch = model.part(catch_name)
        _add_catch(catch, latch_x=latch_x, yellow=yellow, dark_plastic=dark_plastic)
        model.articulation(
            f"{catch_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=parts[parent_name],
            child=catch,
            origin=Origin(xyz=(0.0, y_mount, z_mount)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    nested_pairs = (
        ("base_section", "section_1", "section_1_slide"),
        ("section_1", "section_2", "section_2_slide"),
        ("section_2", "section_3", "section_3_slide"),
    )
    collapsed_pairs = (
        ("base_section", "section_1"),
        ("base_section", "section_2"),
        ("base_section", "section_3"),
        ("section_1", "section_2"),
        ("section_1", "section_3"),
        ("section_2", "section_3"),
    )
    for parent, child in collapsed_pairs:
        for elem in ("left_stile", "right_stile"):
            ctx.allow_overlap(
                parent,
                child,
                elem_a=elem,
                elem_b=elem,
                reason="Nested stile proxies intentionally represent one telescoping tube sliding inside the wider sleeve.",
            )
            ctx.expect_within(
                child,
                parent,
                axes="xy",
                inner_elem=elem,
                outer_elem=elem,
                margin=0.003,
                name=f"{child} {elem} centered in {parent}",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="z",
                elem_a=elem,
                elem_b=elem,
                min_overlap=0.25,
                name=f"{child} {elem} retained when collapsed",
            )
            ctx.allow_overlap(
                parent,
                child,
                elem_a=elem.replace("stile", "collar"),
                elem_b=elem,
                reason="A dark guide collar intentionally captures the nested inner stile at the sleeve mouth.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="z",
                elem_a=elem,
                elem_b=elem.replace("stile", "collar"),
                min_overlap=0.020,
                name=f"{child} {elem} passes through {parent} collar",
            )
            socket = elem.replace("stile", "rung_socket")
            ctx.allow_overlap(
                parent,
                child,
                elem_a=elem,
                elem_b=socket,
                reason="The rung socket is represented as passing through a simplified sleeve slot while the section is collapsed.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="x",
                elem_a=socket,
                elem_b=elem,
                min_overlap=0.005,
                name=f"{child} {socket} sits in {parent} sleeve slot",
            )
            ctx.allow_overlap(
                parent,
                child,
                elem_a=socket,
                elem_b=elem,
                reason="Nested inner stiles pass through the simplified rung-socket slot of the outer section.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="x",
                elem_a=elem,
                elem_b=socket,
                min_overlap=0.005,
                name=f"{child} {elem} clears {parent} rung socket slot",
            )

    for parent, child, joint_name in nested_pairs:
        joint = object_model.get_articulation(joint_name)
        with ctx.pose({joint: SLIDE_TRAVEL}):
            for elem in ("left_stile", "right_stile"):
                ctx.expect_within(
                    child,
                    parent,
                    axes="xy",
                    inner_elem=elem,
                    outer_elem=elem,
                    margin=0.003,
                    name=f"{child} {elem} centered when extended",
                )
                ctx.expect_overlap(
                    child,
                    parent,
                    axes="z",
                    elem_a=elem,
                    elem_b=elem,
                    min_overlap=SECTION_LEN - SLIDE_TRAVEL - COLLAPSED_STEP - 0.01,
                    name=f"{child} {elem} remains inserted when extended",
                )

    # The base is visibly the widest, while upper sections remain nested.
    base = object_model.get_part("base_section")
    top = object_model.get_part("section_3")
    ctx.expect_overlap(top, base, axes="xy", min_overlap=0.055, name="collapsed ladder is compact")

    with ctx.pose(
        {
            object_model.get_articulation("section_1_slide"): SLIDE_TRAVEL,
            object_model.get_articulation("section_2_slide"): SLIDE_TRAVEL,
            object_model.get_articulation("section_3_slide"): SLIDE_TRAVEL,
        }
    ):
        base_pos = ctx.part_world_position(base)
        top_pos = ctx.part_world_position(top)
        ctx.check(
            "upper sections extend upward",
            base_pos is not None
            and top_pos is not None
            and top_pos[2] > base_pos[2] + 1.30,
            details=f"base={base_pos}, top={top_pos}",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from math import pi

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
import cadquery as cq


HINGE_AXIS = (0.0, -1.0, 0.0)
HINGE_RPY = (-pi / 2.0, 0.0, 0.0)


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and orientation for a hinge-axis barrel."""
    return Cylinder(radius=radius, length=length), Origin(rpy=HINGE_RPY)


def _add_hinge_cylinder(
    part,
    *,
    name: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    length: float,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=HINGE_RPY),
        material=material,
        name=name,
    )


def _add_forked_link(part, *, length: float) -> None:
    """A box-section link with a central proximal eye and forked distal cheeks."""
    boss_r = 0.075
    body_w = 0.070
    body_h = 0.090
    lug_w = 0.026
    lug_y = 0.061

    # Deep rectangular web; it is set back from both pin eyes so the link reads
    # as a boxed industrial arm rather than a flat strap.
    part.visual(
        Box((length - 0.150, body_w, body_h)),
        origin=Origin(xyz=(length / 2.0 - 0.015, 0.0, 0.0)),
        material="safety_yellow",
        name="box_web",
    )

    _add_hinge_cylinder(
        part,
        name="proximal_boss",
        x=0.0,
        y=0.0,
        z=0.0,
        radius=boss_r,
        length=body_w,
        material="dark_steel",
    )
    # Shallow bearing washers on the central eye: visible but kept inside the
    # fork clearance of the neighboring link.
    for side, y in (("pos", 0.039), ("neg", -0.039)):
        _add_hinge_cylinder(
            part,
            name=f"proximal_washer_{side}",
            x=0.0,
            y=y,
            z=0.0,
            radius=0.048,
            length=0.008,
            material="zinc",
        )

    # A transverse web connects the central box to the two fork cheeks before
    # the next link's central eye begins, leaving real swing clearance at the
    # hinge.
    part.visual(
        Box((0.060, 0.145, 0.038)),
        origin=Origin(xyz=(length - boss_r - 0.035, 0.0, 0.0)),
        material="safety_yellow",
        name="distal_cross_web",
    )
    _add_hinge_cylinder(
        part,
        name="distal_pin",
        x=length,
        y=0.0,
        z=0.0,
        radius=0.026,
        length=0.170,
        material="zinc",
    )
    for side, y, cheek_name, boss_name, cap_name in (
        ("pos", lug_y, "distal_cheek_pos", "distal_boss_pos", "pin_cap_pos"),
        ("neg", -lug_y, "distal_cheek_neg", "distal_boss_neg", "pin_cap_neg"),
    ):
        part.visual(
            Box((0.135, lug_w, 0.064)),
            origin=Origin(xyz=(length - boss_r / 2.0, y, 0.0)),
            material="safety_yellow",
            name=cheek_name,
        )
        _add_hinge_cylinder(
            part,
            name=boss_name,
            x=length,
            y=y,
            z=0.0,
            radius=boss_r,
            length=lug_w,
            material="dark_steel",
        )
        # A proud cap on the outer face makes the hinge pin boss read visibly
        # from the side without occupying the central clevis gap.
        _add_hinge_cylinder(
            part,
            name=cap_name,
            x=length,
            y=y + (0.017 if y > 0.0 else -0.017),
            z=0.0,
            radius=0.052,
            length=0.008,
            material="zinc",
        )


def _add_final_link(part, *, length: float) -> None:
    boss_r = 0.075
    body_w = 0.070
    body_h = 0.086

    _add_hinge_cylinder(
        part,
        name="proximal_boss",
        x=0.0,
        y=0.0,
        z=0.0,
        radius=boss_r,
        length=body_w,
        material="dark_steel",
    )
    for side, y in (("pos", 0.039), ("neg", -0.039)):
        _add_hinge_cylinder(
            part,
            name=f"proximal_washer_{side}",
            x=0.0,
            y=y,
            z=0.0,
            radius=0.048,
            length=0.008,
            material="zinc",
        )

    part.visual(
        Box((length - 0.060, body_w, body_h)),
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
        material="safety_yellow",
        name="box_web",
    )
    part.visual(
        Box((0.120, 0.155, 0.070)),
        origin=Origin(xyz=(length, 0.0, 0.0)),
        material="dark_steel",
        name="end_pad_body",
    )
    part.visual(
        Box((0.014, 0.145, 0.060)),
        origin=Origin(xyz=(length + 0.067, 0.0, 0.0)),
        material="rubber_black",
        name="rubber_face",
    )
    # Four compact socket-head screws on the pad face.
    for i, (y, z) in enumerate(((-0.047, -0.020), (-0.047, 0.020), (0.047, -0.020), (0.047, 0.020))):
        part.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(length + 0.070, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="zinc",
            name=f"pad_screw_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_industrial_folding_arm_chain")

    model.material("safety_yellow", rgba=(0.95, 0.66, 0.10, 1.0))
    model.material("powder_grey", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("zinc", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.016, 0.014, 1.0))

    base = model.part("base")
    # Bolted plinth.
    base.visual(
        Box((0.34, 0.32, 0.040)),
        origin=Origin(xyz=(-0.020, 0.0, 0.020)),
        material="powder_grey",
        name="base_plinth",
    )
    for i, (x, y) in enumerate(((-0.125, -0.115), (-0.125, 0.115), (0.095, -0.115), (0.095, 0.115))):
        base.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, y, 0.043)),
            material="zinc",
            name=f"mount_bolt_{i}",
        )

    # Reinforced cheek pair around the first pin.  The gap is wide enough for
    # the central eye on link_0, with outer bosses and gussets tied back to the
    # plinth.
    for side, y in (("pos", 0.082), ("neg", -0.082)):
        base.visual(
            Box((0.170, 0.026, 0.235)),
            origin=Origin(xyz=(0.0, y, 0.150)),
            material="powder_grey",
            name=f"cheek_plate_{side}",
        )
        base.visual(
            Box((0.130, 0.020, 0.115)),
            origin=Origin(xyz=(-0.045, y, 0.090), rpy=(0.0, 0.48, 0.0)),
            material="powder_grey",
            name=f"cheek_gusset_{side}",
        )
        _add_hinge_cylinder(
            base,
            name=f"base_boss_{side}",
            x=0.0,
            y=y + (0.019 if y > 0.0 else -0.019),
            z=0.220,
            radius=0.067,
            length=0.014,
            material="dark_steel",
        )
    _add_hinge_cylinder(
        base,
        name="base_pin",
        x=0.0,
        y=0.0,
        z=0.220,
        radius=0.026,
        length=0.205,
        material="zinc",
    )

    link_0 = model.part("link_0")
    _add_forked_link(link_0, length=0.560)

    link_1 = model.part("link_1")
    _add_forked_link(link_1, length=0.500)

    end_link = model.part("end_link")
    _add_final_link(end_link, length=0.340)

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=95.0, velocity=1.2, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "middle_hinge",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.560, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=80.0, velocity=1.3, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_link,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.50, upper=1.50),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_link = object_model.get_part("end_link")

    hinges = [
        object_model.get_articulation("base_hinge"),
        object_model.get_articulation("middle_hinge"),
        object_model.get_articulation("wrist_hinge"),
    ]
    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_boss",
        reason="The hardened base pin is intentionally captured inside link_0's hinge boss.",
    )
    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_washer_pos",
        reason="The base pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_washer_neg",
        reason="The base pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The middle hinge pin is intentionally seated through link_1's proximal boss.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_washer_pos",
        reason="The middle hinge pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_washer_neg",
        reason="The middle hinge pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.allow_overlap(
        link_1,
        end_link,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The wrist hinge pin is intentionally seated through the end link boss.",
    )
    ctx.allow_overlap(
        link_1,
        end_link,
        elem_a="distal_pin",
        elem_b="proximal_washer_pos",
        reason="The wrist hinge pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.allow_overlap(
        link_1,
        end_link,
        elem_a="distal_pin",
        elem_b="proximal_washer_neg",
        reason="The wrist hinge pin is intentionally represented as passing through the pressed outer washer.",
    )
    ctx.check(
        "three parallel revolute hinges",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in hinges)
        and all(tuple(round(v, 6) for v in j.axis) == HINGE_AXIS for j in hinges),
        details=f"hinges={[j.name for j in object_model.articulations]}",
    )

    with ctx.pose({hinges[0]: 0.0, hinges[1]: 0.0, hinges[2]: 0.0}):
        ctx.expect_origin_gap(
            link_1,
            link_0,
            axis="x",
            min_gap=0.555,
            max_gap=0.565,
            name="middle hinge is at link_0 distal eye",
        )
        ctx.expect_origin_gap(
            end_link,
            link_1,
            axis="x",
            min_gap=0.495,
            max_gap=0.505,
            name="wrist hinge is at link_1 distal eye",
        )
        ctx.expect_overlap(
            base,
            link_0,
            axes="xyz",
            elem_a="base_pin",
            elem_b="proximal_boss",
            min_overlap=0.040,
            name="base pin passes through link_0 boss",
        )
        ctx.expect_overlap(
            base,
            link_0,
            axes="xyz",
            elem_a="base_pin",
            elem_b="proximal_washer_pos",
            min_overlap=0.006,
            name="base pin passes through outer washer",
        )
        ctx.expect_overlap(
            link_0,
            link_1,
            axes="xyz",
            elem_a="distal_pin",
            elem_b="proximal_boss",
            min_overlap=0.040,
            name="middle pin passes through link_1 boss",
        )
        ctx.expect_overlap(
            link_0,
            link_1,
            axes="xyz",
            elem_a="distal_pin",
            elem_b="proximal_washer_pos",
            min_overlap=0.006,
            name="middle pin passes through outer washer",
        )
        ctx.expect_overlap(
            link_1,
            end_link,
            axes="xyz",
            elem_a="distal_pin",
            elem_b="proximal_boss",
            min_overlap=0.040,
            name="wrist pin passes through end link boss",
        )
        ctx.expect_overlap(
            link_1,
            end_link,
            axes="xyz",
            elem_a="distal_pin",
            elem_b="proximal_washer_pos",
            min_overlap=0.006,
            name="wrist pin passes through outer washer",
        )
        ctx.expect_overlap(
            link_0,
            link_1,
            axes="xz",
            elem_a="distal_boss_pos",
            elem_b="proximal_boss",
            min_overlap=0.050,
            name="middle hinge bosses are coaxial",
        )
        ctx.expect_gap(
            link_0,
            link_1,
            axis="y",
            positive_elem="distal_boss_pos",
            negative_elem="proximal_boss",
            min_gap=0.006,
            max_gap=0.020,
            name="fork cheek clears central middle boss",
        )
        ctx.expect_overlap(
            link_1,
            end_link,
            axes="xz",
            elem_a="distal_boss_pos",
            elem_b="proximal_boss",
            min_overlap=0.050,
            name="wrist hinge bosses are coaxial",
        )

    rest_end = ctx.part_world_position(end_link)
    with ctx.pose({hinges[0]: 0.70, hinges[1]: 0.0, hinges[2]: 0.0}):
        raised_end = ctx.part_world_position(end_link)
    ctx.check(
        "positive base hinge folds chain upward",
        rest_end is not None and raised_end is not None and raised_end[2] > rest_end[2] + 0.35,
        details=f"rest={rest_end}, raised={raised_end}",
    )

    return ctx.report()


object_model = build_object_model()

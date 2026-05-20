from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


OUTER_LIP_Z = 0.336
DROPPER_TRAVEL = 0.150
STANCHION_INSERTION = 0.190


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_sprung_dropper_seatpost")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.019, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.047, 0.050, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    etched_grey = model.material("etched_grey", rgba=(0.62, 0.65, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.88, 0.66, 0.25, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    red_elastomer = model.material("red_elastomer", rgba=(0.75, 0.04, 0.02, 1.0))

    outer = model.part("outer_sleeve")
    outer_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0162, 0.000),
            (0.0165, 0.006),
            (0.0165, 0.026),
            (0.0158, 0.034),
            (0.0158, 0.292),
            (0.0174, 0.304),
            (0.0182, 0.326),
            (0.0172, OUTER_LIP_Z),
        ],
        [
            (0.0138, 0.000),
            (0.0138, OUTER_LIP_Z),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    outer.visual(
        mesh_from_geometry(outer_shell, "outer_sleeve_shell"),
        material=matte_black,
        name="outer_shell",
    )
    wiper_seal = LatheGeometry.from_shell_profiles(
        [
            (0.0182, 0.326),
            (0.0206, 0.331),
            (0.0206, 0.342),
            (0.0175, 0.348),
        ],
        [
            (0.0136, 0.326),
            (0.0136, 0.348),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    outer.visual(
        mesh_from_geometry(wiper_seal, "top_wiper_seal"),
        material=rubber,
        name="wiper_seal",
    )
    for name, z0, z1 in (("lower_bushing", 0.205, 0.217), ("upper_bushing", 0.318, 0.330)):
        bushing = LatheGeometry.from_shell_profiles(
            [(0.0138, z0), (0.0138, z1)],
            [(0.0120, z0), (0.0120, z1)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        )
        outer.visual(
            mesh_from_geometry(bushing, name),
            material=hard_anodized,
            name=name,
        )
    outer.visual(
        Cylinder(radius=0.0068, length=0.022),
        origin=Origin(xyz=(0.024, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="valve_boss",
    )
    outer.visual(
        Cylinder(radius=0.0031, length=0.014),
        origin=Origin(xyz=(0.041, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="schrader_stem",
    )
    outer.visual(
        Cylinder(radius=0.0036, length=0.003),
        origin=Origin(xyz=(0.0495, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="valve_cap_threads",
    )
    for index, z in enumerate((0.070, 0.095, 0.120, 0.145, 0.170)):
        outer.visual(
            Box((0.0008, 0.010 - 0.001 * (index % 2), 0.0015)),
            origin=Origin(xyz=(-0.0159, 0.0, z)),
            material=etched_grey,
            name=f"height_mark_{index}",
        )

    inner = model.part("inner_post")
    inner.visual(
        Cylinder(radius=0.0123, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hard_anodized,
        name="stanchion",
    )
    inner.visual(
        mesh_from_geometry(TorusGeometry(0.0128, 0.0009, radial_segments=36, tubular_segments=12), "travel_o_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=red_elastomer,
        name="travel_o_ring",
    )
    inner.visual(
        Cylinder(radius=0.0185, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
        material=satin_black,
        name="top_collar",
    )
    inner.visual(
        Box((0.070, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        material=satin_black,
        name="lower_cradle",
    )
    for x in (-0.024, 0.024):
        inner.visual(
            Cylinder(radius=0.006, length=0.046),
            origin=Origin(xyz=(x, 0.0, 0.236), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name=f"rail_saddle_{0 if x < 0.0 else 1}",
        )
    for y in (-0.011, 0.011):
        inner.visual(
            Cylinder(radius=0.0033, length=0.092),
            origin=Origin(xyz=(0.0, y, 0.244), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"saddle_rail_{0 if y < 0.0 else 1}",
        )
    inner.visual(
        Box((0.062, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.253)),
        material=satin_black,
        name="upper_clamp",
    )

    bolt_positions = (-0.022, 0.022)
    bolts = []
    for index, x in enumerate(bolt_positions):
        bolt = model.part(f"clamp_bolt_{index}")
        bolt.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=steel,
            name="bolt_head",
        )
        bolt.visual(
            Cylinder(radius=0.0021, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=steel,
            name="bolt_shank",
        )
        bolt.visual(
            mesh_from_geometry(
                CylinderGeometry(0.0024, 0.0008, radial_segments=6, closed=True),
                f"clamp_bolt_{index}_hex_socket",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.0064)),
            material=matte_black,
            name="hex_socket",
        )
        bolts.append((bolt, x))

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, OUTER_LIP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.70, lower=0.0, upper=DROPPER_TRAVEL),
        motion_properties=MotionProperties(damping=4.0, friction=1.5),
    )
    for index, (bolt, x) in enumerate(bolts):
        model.articulation(
            f"inner_to_bolt_{index}",
            ArticulationType.CONTINUOUS,
            parent=inner,
            child=bolt,
            origin=Origin(xyz=(x, 0.0, 0.259)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    inner = object_model.get_part("inner_post")
    slide = object_model.get_articulation("outer_to_inner")

    for bushing in ("lower_bushing", "upper_bushing"):
        ctx.allow_overlap(
            outer,
            inner,
            elem_a=bushing,
            elem_b="stanchion",
            reason=(
                "Dropper posts use low-friction guide bushings that lightly capture the pressurized "
                "stanchion; the tiny hidden radial interference represents that seated guide contact."
            ),
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="xy",
            elem_a="stanchion",
            elem_b=bushing,
            min_overlap=0.020,
            name=f"{bushing} surrounds the stanchion radially",
        )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_shell",
        margin=0.0,
        name="stanchion is centered inside the lower sleeve bore",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_shell",
        min_overlap=STANCHION_INSERTION - 0.010,
        name="collapsed stanchion retains deep insertion",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: DROPPER_TRAVEL}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.0,
            name="extended stanchion remains centered in the sleeve",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.020,
            name="extended stanchion stays retained in the sleeve",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="stanchion",
            elem_b="upper_bushing",
            min_overlap=0.010,
            name="extended post remains captured by the upper bushing",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "dropper post extends upward on the air-spring slider",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.140,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    for index in (0, 1):
        bolt = object_model.get_part(f"clamp_bolt_{index}")
        for clamp_elem in ("upper_clamp", "lower_cradle"):
            ctx.allow_overlap(
                bolt,
                inner,
                elem_a="bolt_shank",
                elem_b=clamp_elem,
                reason=(
                    "The saddle-clamp bolt shank intentionally passes through the two clamp plates "
                    "as the captured fastener that tensions the saddle collar."
                ),
            )
            ctx.expect_overlap(
                bolt,
                inner,
                axes="z",
                elem_a="bolt_shank",
                elem_b=clamp_elem,
                min_overlap=0.004,
                name=f"clamp bolt {index} passes through {clamp_elem}",
            )
        ctx.expect_contact(
            bolt,
            inner,
            elem_a="bolt_head",
            elem_b="upper_clamp",
            contact_tol=0.0005,
            name=f"clamp bolt {index} is seated on the upper clamp plate",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="highway_median_flood_mast")

    concrete = model.material("weathered_concrete", rgba=(0.48, 0.48, 0.43, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.63, 0.66, 0.67, 1.0))
    dark = model.material("powder_coated_black", rgba=(0.02, 0.025, 0.025, 1.0))
    glass = model.material("pale_prismatic_glass", rgba=(0.58, 0.82, 1.0, 0.55))
    led = model.material("warm_led_chips", rgba=(1.0, 0.82, 0.34, 1.0))
    hardware = model.material("dark_hardware", rgba=(0.10, 0.10, 0.09, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.36, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=concrete,
        name="concrete_pier",
    )
    mast.visual(
        Cylinder(radius=0.23, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=galvanized,
        name="base_flange",
    )
    mast.visual(
        Cylinder(radius=0.056, length=3.16),
        origin=Origin(xyz=(0.0, 0.0, 1.832)),
        material=galvanized,
        name="round_pole",
    )
    mast.visual(
        Cylinder(radius=0.068, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 3.432)),
        material=galvanized,
        name="pole_cap",
    )

    # Anchor studs and washers embedded in the circular steel base flange.
    for idx in range(6):
        angle = idx * math.tau / 6.0
        x = 0.172 * math.cos(angle)
        y = 0.172 * math.sin(angle)
        mast.visual(
            Cylinder(radius=0.020, length=0.007),
            origin=Origin(xyz=(x, y, 0.2595)),
            material=hardware,
            name=f"anchor_washer_{idx}",
        )
        mast.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(x, y, 0.291)),
            material=hardware,
            name=f"anchor_stud_{idx}",
        )

    # Welded stiffener plates from pole to base flange.
    for idx in range(4):
        angle = idx * math.pi / 2.0
        mast.visual(
            Box((0.135, 0.014, 0.235)),
            origin=Origin(
                xyz=(0.091 * math.cos(angle), 0.091 * math.sin(angle), 0.365),
                rpy=(0.0, 0.0, angle),
            ),
            material=galvanized,
            name=f"flange_gusset_{idx}",
        )

    arm_z = 3.430
    mast.visual(
        Cylinder(radius=0.035, length=1.44),
        origin=Origin(xyz=(0.72, 0.0, arm_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="side_crossarm",
    )
    mast.visual(
        Cylinder(radius=0.050, length=0.16),
        origin=Origin(xyz=(0.060, 0.0, arm_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="crossarm_root_sleeve",
    )

    # A diagonal brace under the side arm keeps the cantilever mechanically credible.
    brace_start = (0.030, 0.0, 3.190)
    brace_end = (0.455, 0.0, 3.395)
    dx = brace_end[0] - brace_start[0]
    dz = brace_end[2] - brace_start[2]
    brace_len = math.sqrt(dx * dx + dz * dz)
    brace_theta = math.atan2(dx, dz)
    mast.visual(
        Cylinder(radius=0.017, length=brace_len),
        origin=Origin(
            xyz=((brace_start[0] + brace_end[0]) / 2.0, 0.0, (brace_start[2] + brace_end[2]) / 2.0),
            rpy=(0.0, brace_theta, 0.0),
        ),
        material=galvanized,
        name="underarm_brace",
    )

    # Mesh-backed bracket ears have actual pin clearance holes rather than solid
    # proxy blocks, so the head hinge pins can pass through without collision.
    ear_shape = (
        cq.Workplane("XY")
        .box(0.026, 0.070, 0.230)
        .faces(">X")
        .workplane()
        .hole(0.044)
    )
    ear_mesh = mesh_from_cadquery(ear_shape, "tilt_bracket_ear")

    head_xs = (0.55, 1.10)
    hinge_z = 3.250
    for head_idx, x in enumerate(head_xs):
        mast.visual(
            Cylinder(radius=0.053, length=0.135),
            origin=Origin(xyz=(x, 0.0, arm_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"bracket_{head_idx}_clamp",
        )
        mast.visual(
            Box((0.430, 0.056, 0.032)),
            origin=Origin(xyz=(x, 0.0, hinge_z + 0.126)),
            material=galvanized,
            name=f"bracket_{head_idx}_top_bar",
        )
        mast.visual(
            Box((0.145, 0.048, 0.055)),
            origin=Origin(xyz=(x, 0.0, hinge_z + 0.161)),
            material=galvanized,
            name=f"bracket_{head_idx}_hanger",
        )
        for side_idx, side in enumerate((-1.0, 1.0)):
            mast.visual(
                ear_mesh,
                origin=Origin(xyz=(x + side * 0.195, 0.0, hinge_z)),
                material=galvanized,
                name=f"bracket_{head_idx}_ear_{side_idx}",
            )

    def add_flood_head(part_name: str) -> None:
        head = model.part(part_name)
        head.visual(
            Box((0.310, 0.120, 0.215)),
            origin=Origin(xyz=(0.0, 0.074, -0.158)),
            material=dark,
            name="housing",
        )
        head.visual(
            Box((0.262, 0.010, 0.158)),
            origin=Origin(xyz=(0.0, 0.139, -0.158)),
            material=glass,
            name="front_glass",
        )
        head.visual(
            Box((0.310, 0.016, 0.023)),
            origin=Origin(xyz=(0.0, 0.146, -0.058)),
            material=dark,
            name="top_bezel",
        )
        head.visual(
            Box((0.310, 0.016, 0.023)),
            origin=Origin(xyz=(0.0, 0.146, -0.258)),
            material=dark,
            name="bottom_bezel",
        )
        for side_idx, side in enumerate((-1.0, 1.0)):
            head.visual(
                Box((0.024, 0.016, 0.200)),
                origin=Origin(xyz=(side * 0.143, 0.146, -0.158)),
                material=dark,
                name=f"side_bezel_{side_idx}",
            )
            head.visual(
                Box((0.050, 0.050, 0.058)),
                origin=Origin(xyz=(side * 0.105, 0.034, -0.029)),
                material=dark,
                name=f"hinge_lug_{side_idx}",
            )

        head.visual(
            Cylinder(radius=0.013, length=0.420),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="hinge_pin",
        )
        for side_idx, side in enumerate((-1.0, 1.0)):
            head.visual(
                Cylinder(radius=0.030, length=0.012),
                origin=Origin(xyz=(side * 0.214, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hardware,
                name=f"hinge_washer_{side_idx}",
            )
        for fin_idx, x in enumerate((-0.120, -0.080, -0.040, 0.0, 0.040, 0.080, 0.120)):
            head.visual(
                Box((0.012, 0.042, 0.166)),
                origin=Origin(xyz=(x, -0.006, -0.158)),
                material=dark,
                name=f"rear_fin_{fin_idx}",
            )
        for row, z in enumerate((-0.194, -0.122)):
            for col, x in enumerate((-0.076, 0.0, 0.076)):
                head.visual(
                    Box((0.038, 0.006, 0.038)),
                    origin=Origin(xyz=(x, 0.144, z)),
                    material=led,
                    name=f"led_{row}_{col}",
                )

    add_flood_head("head_0")
    add_flood_head("head_1")

    for head_idx, x in enumerate(head_xs):
        model.articulation(
            f"tilt_{head_idx}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=f"head_{head_idx}",
            origin=Origin(xyz=(x, 0.0, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.65, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    head_0 = object_model.get_part("head_0")
    head_1 = object_model.get_part("head_1")
    tilt_0 = object_model.get_articulation("tilt_0")
    tilt_1 = object_model.get_articulation("tilt_1")

    ctx.check(
        "two independent flood head tilt hinges",
        tilt_0.articulation_type == ArticulationType.REVOLUTE
        and tilt_1.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt_0.axis) == (1.0, 0.0, 0.0)
        and tuple(tilt_1.axis) == (1.0, 0.0, 0.0),
        details=f"tilt_0={tilt_0.articulation_type}/{tilt_0.axis}, "
        f"tilt_1={tilt_1.articulation_type}/{tilt_1.axis}",
    )

    for head_idx, head in enumerate((head_0, head_1)):
        for side_idx in (0, 1):
            ctx.expect_overlap(
                head,
                mast,
                axes="x",
                elem_a="hinge_pin",
                elem_b=f"bracket_{head_idx}_ear_{side_idx}",
                min_overlap=0.006,
                name=f"head_{head_idx} pin passes through bracket ear {side_idx}",
            )
            ctx.expect_overlap(
                head,
                mast,
                axes="yz",
                elem_a="hinge_pin",
                elem_b=f"bracket_{head_idx}_ear_{side_idx}",
                min_overlap=0.020,
                name=f"head_{head_idx} pin aligns with bracket ear {side_idx} hole",
            )

    rest_aabb = ctx.part_element_world_aabb(head_0, elem="front_glass")
    with ctx.pose({tilt_0: 0.55}):
        raised_aabb = ctx.part_element_world_aabb(head_0, elem="front_glass")
    with ctx.pose({tilt_0: -0.55}):
        lowered_aabb = ctx.part_element_world_aabb(head_0, elem="front_glass")

    def z_center(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_z = z_center(rest_aabb)
    raised_z = z_center(raised_aabb)
    lowered_z = z_center(lowered_aabb)
    ctx.check(
        "tilt hinge pitches the flood head",
        rest_z is not None
        and raised_z is not None
        and lowered_z is not None
        and raised_z > rest_z + 0.035
        and lowered_z < rest_z - 0.035,
        details=f"rest_z={rest_z}, raised_z={raised_z}, lowered_z={lowered_z}",
    )

    return ctx.report()


object_model = build_object_model()

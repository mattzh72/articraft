from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    """Make an open through-bore sleeve centered on local Z."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_positioning_stage")

    dark = Material("anodized_black", rgba=(0.03, 0.035, 0.04, 1.0))
    blue = Material("blue_anodized_carriage", rgba=(0.05, 0.22, 0.62, 1.0))
    steel = Material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = Material("matte_black_pad", rgba=(0.008, 0.008, 0.008, 1.0))
    brass = Material("brass_bushing", rgba=(0.86, 0.62, 0.22, 1.0))

    x_bushing_mesh = _tube_mesh(
        "x_linear_bearing_sleeve",
        outer_radius=0.016,
        inner_radius=0.0077,
        length=0.090,
    )
    z_bushing_mesh = _tube_mesh(
        "z_linear_bearing_sleeve",
        outer_radius=0.012,
        inner_radius=0.0062,
        length=0.055,
    )

    base = model.part("base")
    base.visual(
        Box((0.46, 0.18, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark,
        name="base_plate",
    )
    base.visual(
        Box((0.030, 0.145, 0.074)),
        origin=Origin(xyz=(-0.200, 0.0, 0.057)),
        material=dark,
        name="end_support_0",
    )
    base.visual(
        Box((0.030, 0.145, 0.074)),
        origin=Origin(xyz=(0.200, 0.0, 0.057)),
        material=dark,
        name="end_support_1",
    )
    for y, rail_name in ((-0.045, "x_rail_0"), (0.045, "x_rail_1")):
        base.visual(
            Cylinder(radius=0.008, length=0.405),
            origin=Origin(xyz=(0.0, y, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=rail_name,
        )
    base.visual(
        Box((0.410, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=aluminum,
        name="center_scale",
    )
    for x in (-0.165, 0.165):
        for y in (-0.070, 0.070):
            base.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.026)),
                material=steel,
                name=f"mount_screw_{x}_{y}",
            )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.096, 0.138, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=blue,
        name="saddle_plate",
    )
    for y, sleeve_name in ((-0.045, "x_bushing_0"), (0.045, "x_bushing_1")):
        x_carriage.visual(
            x_bushing_mesh,
            origin=Origin(xyz=(0.0, y, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=sleeve_name,
        )
    x_carriage.visual(
        Box((0.092, 0.024, 0.260)),
        origin=Origin(xyz=(0.0, -0.012, 0.230)),
        material=blue,
        name="z_guide_backbone",
    )
    for x, rail_name in ((-0.028, "z_rail_0"), (0.028, "z_rail_1")):
        x_carriage.visual(
            Cylinder(radius=0.0065, length=0.255),
            origin=Origin(xyz=(x, 0.030, 0.230)),
            material=steel,
            name=rail_name,
        )
    x_carriage.visual(
        Box((0.108, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, -0.012, 0.370)),
        material=blue,
        name="top_travel_stop",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.086, 0.014, 0.176)),
        origin=Origin(xyz=(0.0, 0.047, 0.210)),
        material=aluminum,
        name="moving_plate",
    )
    for x, sleeve_name in (
        (-0.028, "lower_z_bushing_0"),
        (0.028, "lower_z_bushing_1"),
    ):
        z_slide.visual(
            z_bushing_mesh,
            origin=Origin(xyz=(x, 0.030, 0.170)),
            material=brass,
            name=sleeve_name,
        )
    for x, sleeve_name in (
        (-0.028, "upper_z_bushing_0"),
        (0.028, "upper_z_bushing_1"),
    ):
        z_slide.visual(
            z_bushing_mesh,
            origin=Origin(xyz=(x, 0.030, 0.250)),
            material=brass,
            name=sleeve_name,
        )
    for x, lug_name in (
        (-0.028, "lower_lug_0"),
        (0.028, "lower_lug_1"),
        (-0.028, "upper_lug_0"),
        (0.028, "upper_lug_1"),
    ):
        z = 0.170 if "lower" in lug_name else 0.250
        z_slide.visual(
            Box((0.020, 0.020, 0.032)),
            origin=Origin(xyz=(x, 0.052, z)),
            material=aluminum,
            name=lug_name,
        )
    z_slide.visual(
        Box((0.120, 0.095, 0.014)),
        origin=Origin(xyz=(0.0, 0.087, 0.306)),
        material=rubber,
        name="top_pad",
    )
    z_slide.visual(
        Box((0.050, 0.074, 0.040)),
        origin=Origin(xyz=(0.0, 0.069, 0.280)),
        material=aluminum,
        name="pad_riser",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=-0.100, upper=0.100),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_slide,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.150),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    for i in (0, 1):
        ctx.allow_overlap(
            base,
            x_carriage,
            elem_a=f"x_rail_{i}",
            elem_b=f"x_bushing_{i}",
            reason="The carriage linear bearings are intentionally captured around the X rail with a tiny modeled preload.",
        )
        ctx.allow_overlap(
            x_carriage,
            z_slide,
            elem_a=f"z_rail_{i}",
            elem_b=f"lower_z_bushing_{i}",
            reason="The Z slide lower bearings are intentionally captured around the guide rail with a tiny modeled preload.",
        )
        ctx.allow_overlap(
            x_carriage,
            z_slide,
            elem_a=f"z_rail_{i}",
            elem_b=f"upper_z_bushing_{i}",
            reason="The Z slide upper bearings are intentionally captured around the guide rail with a tiny modeled preload.",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a=f"x_bushing_{i}",
            elem_b=f"x_rail_{i}",
            min_overlap=0.080,
            name=f"x bearing {i} is retained on its rail",
        )
        for bushing in (f"lower_z_bushing_{i}", f"upper_z_bushing_{i}"):
            ctx.expect_overlap(
                z_slide,
                x_carriage,
                axes="z",
                elem_a=bushing,
                elem_b=f"z_rail_{i}",
                min_overlap=0.050,
                name=f"{bushing} is retained on its rail",
            )

    ctx.expect_within(
        x_carriage,
        base,
        axes="y",
        inner_elem="saddle_plate",
        outer_elem="base_plate",
        margin=0.010,
        name="x saddle stays over the base width",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        elem_a="x_bushing_0",
        elem_b="x_rail_0",
        min_overlap=0.080,
        name="x bearing remains engaged at center",
    )
    with ctx.pose({x_axis: 0.100}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_bushing_0",
            elem_b="x_rail_0",
            min_overlap=0.080,
            name="x bearing remains engaged at positive travel",
        )
    with ctx.pose({x_axis: -0.100}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_bushing_1",
            elem_b="x_rail_1",
            min_overlap=0.080,
            name="x bearing remains engaged at negative travel",
        )

    x_rest = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: 0.100}):
        x_positive = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: -0.100}):
        x_negative = ctx.part_world_position(x_carriage)
    ctx.check(
        "x carriage travels about 100 mm each way",
        x_rest is not None
        and x_positive is not None
        and x_negative is not None
        and x_positive[0] > x_rest[0] + 0.095
        and x_negative[0] < x_rest[0] - 0.095,
        details=f"negative={x_negative}, rest={x_rest}, positive={x_positive}",
    )

    z_rest = ctx.part_world_position(z_slide)
    with ctx.pose({z_axis: 0.150}):
        ctx.expect_overlap(
            z_slide,
            x_carriage,
            axes="z",
            elem_a="lower_z_bushing_0",
            elem_b="z_rail_0",
            min_overlap=0.050,
            name="z lower bushing remains on guide rail at full lift",
        )
        z_lifted = ctx.part_world_position(z_slide)

    ctx.check(
        "z slide travels upward about 150 mm",
        z_rest is not None and z_lifted is not None and z_lifted[2] > z_rest[2] + 0.145,
        details=f"rest={z_rest}, lifted={z_lifted}",
    )

    return ctx.report()


object_model = build_object_model()

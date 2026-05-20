from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_writing_table")

    walnut = model.material("polished_walnut", rgba=(0.36, 0.18, 0.08, 1.0))
    dark_walnut = model.material("dark_end_grain", rgba=(0.21, 0.10, 0.045, 1.0))
    shadow = model.material("shadowed_interior", rgba=(0.055, 0.040, 0.030, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.58, 0.24, 1.0))
    leather = model.material("green_writing_leather", rgba=(0.045, 0.18, 0.105, 1.0))

    body = model.part("body")

    # Wide writing surface with a slightly proud leather insert and darker breadboard edging.
    body.visual(
        Box((1.82, 0.78, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.7525)),
        material=walnut,
        name="top_slab",
    )
    body.visual(
        Box((1.34, 0.48, 0.004)),
        origin=Origin(xyz=(0.0, -0.025, 0.7820)),
        material=leather,
        name="writing_inset",
    )
    body.visual(
        Box((1.82, 0.035, 0.066)),
        origin=Origin(xyz=(0.0, -0.390, 0.747)),
        material=dark_walnut,
        name="front_edge_band",
    )
    body.visual(
        Box((1.82, 0.035, 0.066)),
        origin=Origin(xyz=(0.0, 0.390, 0.747)),
        material=dark_walnut,
        name="rear_edge_band",
    )
    body.visual(
        Box((0.035, 0.78, 0.066)),
        origin=Origin(xyz=(-0.910, 0.0, 0.747)),
        material=dark_walnut,
        name="end_band_0",
    )
    body.visual(
        Box((0.035, 0.78, 0.066)),
        origin=Origin(xyz=(0.910, 0.0, 0.747)),
        material=dark_walnut,
        name="end_band_1",
    )

    # Two side pedestal shells.  The fronts are framed but open so the moving
    # file drawer faces read as separate sliding assemblies.
    pedestal_centers = (-0.62, 0.62)
    for index, x_center in enumerate(pedestal_centers):
        body.visual(
            Box((0.026, 0.64, 0.700)),
            origin=Origin(xyz=(x_center - 0.220, 0.0, 0.375)),
            material=walnut,
            name=f"pedestal_side_{index}_0",
        )
        body.visual(
            Box((0.026, 0.64, 0.700)),
            origin=Origin(xyz=(x_center + 0.220, 0.0, 0.375)),
            material=walnut,
            name=f"pedestal_side_{index}_1",
        )
        body.visual(
            Box((0.440, 0.026, 0.700)),
            origin=Origin(xyz=(x_center, 0.307, 0.375)),
            material=walnut,
            name=f"pedestal_back_{index}",
        )
        body.visual(
            Box((0.440, 0.640, 0.050)),
            origin=Origin(xyz=(x_center, 0.0, 0.025)),
            material=dark_walnut,
            name=f"pedestal_plinth_{index}",
        )
        body.visual(
            Box((0.440, 0.640, 0.035)),
            origin=Origin(xyz=(x_center, 0.0, 0.7075)),
            material=walnut,
            name=f"pedestal_top_rail_{index}",
        )
        body.visual(
            Box((0.440, 0.036, 0.060)),
            origin=Origin(xyz=(x_center, -0.334, 0.650)),
            material=dark_walnut,
            name=f"file_top_rail_{index}",
        )
        body.visual(
            Box((0.440, 0.036, 0.060)),
            origin=Origin(xyz=(x_center, -0.334, 0.070)),
            material=dark_walnut,
            name=f"file_bottom_rail_{index}",
        )
        body.visual(
            Box((0.035, 0.040, 0.610)),
            origin=Origin(xyz=(x_center - 0.205, -0.334, 0.350)),
            material=dark_walnut,
            name=f"file_stile_{index}_0",
        )
        body.visual(
            Box((0.035, 0.040, 0.610)),
            origin=Origin(xyz=(x_center + 0.205, -0.334, 0.350)),
            material=dark_walnut,
            name=f"file_stile_{index}_1",
        )
        body.visual(
            Box((0.360, 0.020, 0.470)),
            origin=Origin(xyz=(x_center, 0.292, 0.340)),
            material=shadow,
            name=f"file_cavity_shadow_{index}",
        )
        # Fixed wooden/metal guide rails mounted to the inside of the pedestal.
        for side, side_x in enumerate((-0.188, 0.188)):
            body.visual(
                Box((0.026, 0.590, 0.020)),
                origin=Origin(xyz=(x_center + side_x, -0.020, 0.285)),
                material=brass,
                name=f"file_fixed_rail_{index}_{side}",
            )

    # Central knee-hole frieze and its long shallow drawer bay.
    body.visual(
        Box((0.820, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.334, 0.7075)),
        material=dark_walnut,
        name="frieze_top_rail",
    )
    body.visual(
        Box((0.820, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, -0.334, 0.565)),
        material=dark_walnut,
        name="frieze_lower_bead",
    )
    body.visual(
        Box((0.035, 0.040, 0.150)),
        origin=Origin(xyz=(-0.405, -0.334, 0.635)),
        material=dark_walnut,
        name="frieze_stile_0",
    )
    body.visual(
        Box((0.035, 0.040, 0.150)),
        origin=Origin(xyz=(0.405, -0.334, 0.635)),
        material=dark_walnut,
        name="frieze_stile_1",
    )
    body.visual(
        Box((0.660, 0.020, 0.110)),
        origin=Origin(xyz=(0.0, 0.297, 0.635)),
        material=shadow,
        name="frieze_cavity_shadow",
    )
    body.visual(
        Box((0.780, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, 0.320, 0.620)),
        material=walnut,
        name="rear_modesty_rail",
    )
    for side, side_x in enumerate((-0.335, 0.335)):
        body.visual(
            Box((0.024, 0.480, 0.018)),
            origin=Origin(xyz=(side_x, -0.080, 0.574)),
            material=brass,
            name=f"frieze_fixed_rail_{side}",
        )

    # Shallow central frieze drawer.
    frieze_drawer = model.part("frieze_drawer")
    frieze_drawer.visual(
        Box((0.660, 0.028, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=walnut,
        name="front",
    )
    frieze_drawer.visual(
        Box((0.560, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, -0.016, -0.004)),
        material=dark_walnut,
        name="inset_panel",
    )
    frieze_drawer.visual(
        Box((0.018, 0.480, 0.075)),
        origin=Origin(xyz=(-0.306, 0.240, -0.0025)),
        material=walnut,
        name="side_0",
    )
    frieze_drawer.visual(
        Box((0.018, 0.480, 0.075)),
        origin=Origin(xyz=(0.306, 0.240, -0.0025)),
        material=walnut,
        name="side_1",
    )
    frieze_drawer.visual(
        Box((0.600, 0.460, 0.012)),
        origin=Origin(xyz=(0.0, 0.242, -0.047)),
        material=dark_walnut,
        name="bottom",
    )
    frieze_drawer.visual(
        Box((0.610, 0.018, 0.075)),
        origin=Origin(xyz=(0.0, 0.472, -0.0025)),
        material=walnut,
        name="back",
    )
    for side, side_x in enumerate((-0.326, 0.326)):
        frieze_drawer.visual(
            Box((0.018, 0.480, 0.018)),
            origin=Origin(xyz=(side_x, 0.240, -0.043)),
            material=brass,
            name=f"runner_{side}",
        )
    _add_bar_pull(frieze_drawer, length=0.300, post_spacing=0.220, z=0.000, material=brass)

    model.articulation(
        "body_to_frieze_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=frieze_drawer,
        origin=Origin(xyz=(0.0, -0.356, 0.635)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.30, lower=0.0, upper=0.280),
    )

    # Two deep filing drawers, one per side pedestal.
    for index, x_center in enumerate(pedestal_centers):
        drawer = model.part(f"file_drawer_{index}")
        drawer.visual(
            Box((0.360, 0.032, 0.480)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=walnut,
            name="front",
        )
        drawer.visual(
            Box((0.285, 0.004, 0.360)),
            origin=Origin(xyz=(0.0, -0.018, -0.010)),
            material=dark_walnut,
            name="recessed_field",
        )
        drawer.visual(
            Box((0.018, 0.600, 0.400)),
            origin=Origin(xyz=(-0.170, 0.314, -0.020)),
            material=walnut,
            name="side_0",
        )
        drawer.visual(
            Box((0.018, 0.600, 0.400)),
            origin=Origin(xyz=(0.170, 0.314, -0.020)),
            material=walnut,
            name="side_1",
        )
        drawer.visual(
            Box((0.330, 0.600, 0.018)),
            origin=Origin(xyz=(0.0, 0.314, -0.225)),
            material=dark_walnut,
            name="bottom",
        )
        drawer.visual(
            Box((0.340, 0.018, 0.400)),
            origin=Origin(xyz=(0.0, 0.609, -0.020)),
            material=walnut,
            name="back",
        )
        for side, side_x in enumerate((-0.174, 0.174)):
            drawer.visual(
                Box((0.022, 0.550, 0.020)),
                origin=Origin(xyz=(side_x, 0.345, -0.035)),
                material=brass,
                name=f"runner_{side}",
            )
        _add_bar_pull(drawer, length=0.230, post_spacing=0.150, z=0.120, material=brass)

        model.articulation(
            f"body_to_file_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(x_center, -0.356, 0.340)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=85.0, velocity=0.28, lower=0.0, upper=0.420),
        )

    return model


def _add_bar_pull(part, *, length: float, post_spacing: float, z: float, material) -> None:
    """Add a connected rectangular bar-pull on the front face of a drawer part."""
    part.visual(
        Box((length, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.065, z)),
        material=material,
        name="bar_pull",
    )
    for index, x in enumerate((-post_spacing / 2.0, post_spacing / 2.0)):
        part.visual(
            Box((0.020, 0.038, 0.032)),
            origin=Origin(xyz=(x, -0.035, z)),
            material=material,
            name=f"pull_post_{index}",
        )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    frieze = object_model.get_part("frieze_drawer")
    frieze_joint = object_model.get_articulation("body_to_frieze_drawer")
    ctx.expect_contact(
        frieze,
        body,
        elem_a="runner_0",
        elem_b="frieze_fixed_rail_0",
        name="frieze drawer rests on a guide rail",
    )
    ctx.expect_overlap(
        frieze,
        body,
        axes="y",
        elem_a="runner_0",
        elem_b="frieze_fixed_rail_0",
        min_overlap=0.40,
        name="frieze drawer retained on rails when closed",
    )
    frieze_rest = ctx.part_world_position(frieze)
    with ctx.pose({frieze_joint: 0.280}):
        ctx.expect_overlap(
            frieze,
            body,
            axes="y",
            elem_a="runner_0",
            elem_b="frieze_fixed_rail_0",
            min_overlap=0.14,
            name="frieze drawer remains engaged at full travel",
        )
        frieze_extended = ctx.part_world_position(frieze)
    ctx.check(
        "frieze drawer extends toward the sitter",
        frieze_rest is not None
        and frieze_extended is not None
        and frieze_extended[1] < frieze_rest[1] - 0.25,
        details=f"rest={frieze_rest}, extended={frieze_extended}",
    )

    for index in (0, 1):
        drawer = object_model.get_part(f"file_drawer_{index}")
        joint = object_model.get_articulation(f"body_to_file_drawer_{index}")
        ctx.expect_contact(
            drawer,
            body,
            elem_a="runner_0",
            elem_b=f"file_fixed_rail_{index}_0",
            name=f"file drawer {index} rests on its guide rail",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="runner_0",
            elem_b=f"file_fixed_rail_{index}_0",
            min_overlap=0.55,
            name=f"file drawer {index} retained on rails when closed",
        )
        rest = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.420}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="runner_0",
                elem_b=f"file_fixed_rail_{index}_0",
                min_overlap=0.14,
                name=f"file drawer {index} remains on rails at full travel",
            )
            extended = ctx.part_world_position(drawer)
        ctx.check(
            f"file drawer {index} slides out on prismatic rails",
            rest is not None and extended is not None and extended[1] < rest[1] - 0.38,
            details=f"rest={rest}, extended={extended}",
        )

    return ctx.report()


object_model = build_object_model()

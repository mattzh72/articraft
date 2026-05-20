from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _bolt_head(
    part,
    *,
    xyz: tuple[float, float, float],
    radius: float = 0.008,
    height: float = 0.006,
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _guard_ring_mesh(name: str, radius: float, tube: float):
    return mesh_from_geometry(
        TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=48),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_revolute_prismatic_study")

    steel = _mat(model, "brushed_steel", (0.55, 0.57, 0.56, 1.0))
    dark_steel = _mat(model, "blackened_steel", (0.08, 0.085, 0.08, 1.0))
    anodized = _mat(model, "hard_anodized_link", (0.18, 0.22, 0.26, 1.0))
    bearing = _mat(model, "ground_bearing", (0.74, 0.72, 0.66, 1.0))
    cover = _mat(model, "satin_access_cover", (0.28, 0.31, 0.34, 1.0))
    rail = _mat(model, "polished_rail", (0.80, 0.82, 0.80, 1.0))

    # Root fixture: a welded/machined bed plate with the first rotary bearing
    # bracket built around the world Z axis at the part frame origin.
    base = model.part("base")
    base.visual(
        Box((1.72, 0.48, 0.040)),
        origin=Origin(xyz=(0.70, 0.0, -0.160)),
        material=dark_steel,
        name="bed_plate",
    )
    for i, y in enumerate((-0.215, 0.215)):
        base.visual(
            Box((1.52, 0.036, 0.030)),
            origin=Origin(xyz=(0.73, y, -0.125)),
            material=steel,
            name=f"bed_stiffener_{i}",
        )
    base.visual(
        Cylinder(radius=0.110, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=steel,
        name="pivot_pedestal",
    )
    base.visual(
        Cylinder(radius=0.120, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=bearing,
        name="first_bearing_block",
    )
    base.visual(
        Cylinder(radius=0.098, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=rail,
        name="first_thrust_washer",
    )
    for i, y in enumerate((-0.135, 0.135)):
        base.visual(
            Box((0.270, 0.036, 0.175)),
            origin=Origin(xyz=(0.0, y, -0.060)),
            material=steel,
            name=f"pivot_cheek_{i}",
        )
    for i, x in enumerate((-0.105, 0.105)):
        base.visual(
            Box((0.030, 0.250, 0.044)),
            origin=Origin(xyz=(x, 0.0, -0.005)),
            material=steel,
            name=f"cheek_bridge_{i}",
        )
    base.visual(
        _guard_ring_mesh("base_joint_guard", 0.136, 0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="first_guard_ring",
    )
    for idx, (x, y) in enumerate(((-0.105, -0.105), (-0.105, 0.105), (0.105, -0.105), (0.105, 0.105))):
        _bolt_head(
            base,
            xyz=(x, y, -0.136),
            radius=0.010,
            height=0.008,
            material=bearing,
            name=f"bed_bolt_{idx}",
        )

    # First rotary link: a short, rigid machined plate spanning the two rotary
    # centers, with raised bosses, thrust spacers, guard rings, and a removable
    # top inspection cover.
    primary = model.part("primary_link")
    primary.visual(
        Box((0.560, 0.082, 0.038)),
        origin=Origin(xyz=(0.310, 0.0, 0.061)),
        material=anodized,
        name="primary_web",
    )
    primary.visual(
        Cylinder(radius=0.082, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=anodized,
        name="first_hub",
    )
    primary.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=bearing,
        name="first_spacer",
    )
    primary.visual(
        _guard_ring_mesh("primary_first_guard", 0.087, 0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=dark_steel,
        name="first_guard_ring",
    )
    primary.visual(
        Cylinder(radius=0.082, length=0.052),
        origin=Origin(xyz=(0.620, 0.0, 0.061)),
        material=anodized,
        name="second_hub",
    )
    primary.visual(
        Cylinder(radius=0.056, length=0.027),
        origin=Origin(xyz=(0.620, 0.0, 0.1005)),
        material=bearing,
        name="second_spacer",
    )
    primary.visual(
        _guard_ring_mesh("primary_second_guard", 0.087, 0.008),
        origin=Origin(xyz=(0.620, 0.0, 0.089)),
        material=dark_steel,
        name="second_guard_ring",
    )
    primary.visual(
        Box((0.355, 0.058, 0.009)),
        origin=Origin(xyz=(0.310, 0.0, 0.083)),
        material=cover,
        name="primary_access_cover",
    )
    for idx, x in enumerate((0.165, 0.255, 0.365, 0.455)):
        for y in (-0.019, 0.019):
            _bolt_head(
                primary,
                xyz=(x, y, 0.090),
                radius=0.0055,
                height=0.006,
                material=bearing,
                name=f"primary_cover_bolt_{idx}_{0 if y < 0 else 1}",
            )
    for idx, x in enumerate((0.215, 0.405)):
        primary.visual(
            Box((0.055, 0.012, 0.026)),
            origin=Origin(xyz=(x, -0.047, 0.061)),
            material=dark_steel,
            name=f"side_gib_{idx}",
        )
        primary.visual(
            Box((0.055, 0.012, 0.026)),
            origin=Origin(xyz=(x, 0.047, 0.061)),
            material=dark_steel,
            name=f"side_gib_mirror_{idx}",
        )

    # Second rotary carrier: elevated over the primary link on a bearing stack
    # and ending in a rectangular guide sleeve for the output prismatic stage.
    secondary = model.part("secondary_link")
    secondary.visual(
        Cylinder(radius=0.088, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=bearing,
        name="second_lower_washer",
    )
    secondary.visual(
        Cylinder(radius=0.076, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=anodized,
        name="second_upper_hub",
    )
    secondary.visual(
        _guard_ring_mesh("secondary_pivot_guard", 0.081, 0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=dark_steel,
        name="second_guard_ring",
    )
    secondary.visual(
        Box((0.440, 0.074, 0.040)),
        origin=Origin(xyz=(0.220, 0.0, 0.155)),
        material=anodized,
        name="secondary_web",
    )
    secondary.visual(
        Box((0.150, 0.032, 0.034)),
        origin=Origin(xyz=(0.420, -0.047, 0.155)),
        material=anodized,
        name="guide_web_0",
    )
    secondary.visual(
        Box((0.150, 0.032, 0.034)),
        origin=Origin(xyz=(0.420, 0.047, 0.155)),
        material=anodized,
        name="guide_web_1",
    )
    secondary.visual(
        Box((0.430, 0.024, 0.074)),
        origin=Origin(xyz=(0.665, -0.060, 0.155)),
        material=steel,
        name="slide_sleeve_side_0",
    )
    secondary.visual(
        Box((0.430, 0.024, 0.074)),
        origin=Origin(xyz=(0.665, 0.060, 0.155)),
        material=steel,
        name="slide_sleeve_side_1",
    )
    secondary.visual(
        Box((0.430, 0.136, 0.018)),
        origin=Origin(xyz=(0.665, 0.0, 0.193)),
        material=steel,
        name="slide_sleeve_top",
    )
    secondary.visual(
        Box((0.430, 0.136, 0.018)),
        origin=Origin(xyz=(0.665, 0.0, 0.117)),
        material=steel,
        name="slide_sleeve_bottom",
    )
    secondary.visual(
        Box((0.250, 0.084, 0.008)),
        origin=Origin(xyz=(0.665, 0.0, 0.206)),
        material=cover,
        name="sleeve_access_cover",
    )
    for idx, x in enumerate((0.565, 0.665, 0.765)):
        for y in (-0.042, 0.042):
            _bolt_head(
                secondary,
                xyz=(x, y, 0.213),
                radius=0.005,
                height=0.006,
                material=bearing,
                name=f"sleeve_cover_bolt_{idx}_{0 if y < 0 else 1}",
            )
    for idx, x in enumerate((0.500, 0.830)):
        secondary.visual(
            Cylinder(radius=0.010, length=0.160),
            origin=Origin(xyz=(x, 0.0, 0.193), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rail,
            name=f"cross_pin_{idx}",
        )

    # Prismatic output: a retained inner telescoping rail running in the sleeve,
    # terminating in a plain bolted output flange and anti-rotation guide keys.
    slider = model.part("slider")
    slider.visual(
        Box((0.740, 0.050, 0.032)),
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        material=rail,
        name="inner_slide_bar",
    )
    slider.visual(
        Box((0.600, 0.023, 0.012)),
        origin=Origin(xyz=(0.250, -0.0365, 0.0)),
        material=dark_steel,
        name="side_key_0",
    )
    slider.visual(
        Box((0.600, 0.023, 0.012)),
        origin=Origin(xyz=(0.250, 0.0365, 0.0)),
        material=dark_steel,
        name="side_key_1",
    )
    slider.visual(
        Box((0.070, 0.155, 0.096)),
        origin=Origin(xyz=(0.705, 0.0, 0.0)),
        material=steel,
        name="output_flange",
    )
    slider.visual(
        Box((0.030, 0.095, 0.054)),
        origin=Origin(xyz=(0.655, 0.0, 0.0)),
        material=steel,
        name="flange_neck",
    )
    for idx, (y, z) in enumerate(((-0.050, -0.030), (-0.050, 0.030), (0.050, -0.030), (0.050, 0.030))):
        _bolt_head(
            slider,
            xyz=(0.745, y, z),
            radius=0.008,
            height=0.010,
            material=bearing,
            name=f"flange_bolt_{idx}",
            rpy=(0.0, pi / 2.0, 0.0),
        )
    slider.visual(
        Box((0.060, 0.070, 0.009)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0205)),
        material=cover,
        name="slider_stop_plate",
    )
    for idx, y in enumerate((-0.022, 0.022)):
        _bolt_head(
            slider,
            xyz=(-0.080, y, 0.0275),
            radius=0.0045,
            height=0.005,
            material=bearing,
            name=f"stop_bolt_{idx}",
        )

    model.articulation(
        "base_to_primary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.5, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "secondary_to_slider",
        ArticulationType.PRISMATIC,
        parent=secondary,
        child=slider,
        origin=Origin(xyz=(0.550, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.260),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    slider = object_model.get_part("slider")
    base = object_model.get_part("base")
    base_to_primary = object_model.get_articulation("base_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_slider = object_model.get_articulation("secondary_to_slider")

    ctx.check(
        "explicit rrp chain",
        base_to_primary.articulation_type == ArticulationType.REVOLUTE
        and primary_to_secondary.articulation_type == ArticulationType.REVOLUTE
        and secondary_to_slider.articulation_type == ArticulationType.PRISMATIC,
        details="Assembly must expose revolute-revolute-prismatic joints.",
    )
    ctx.expect_gap(
        primary,
        base,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="first_spacer",
        negative_elem="first_thrust_washer",
        name="first pivot thrust stack has a real running gap",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="second_lower_washer",
        negative_elem="second_spacer",
        name="second pivot bearing stack is seated",
    )
    ctx.expect_within(
        slider,
        secondary,
        axes="y",
        margin=0.002,
        inner_elem="inner_slide_bar",
        outer_elem="slide_sleeve_top",
        name="slide bar stays centered between sleeve sides",
    )
    ctx.expect_gap(
        secondary,
        slider,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="slide_sleeve_top",
        negative_elem="inner_slide_bar",
        name="upper sleeve clears moving bar",
    )
    ctx.expect_gap(
        slider,
        secondary,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="inner_slide_bar",
        negative_elem="slide_sleeve_bottom",
        name="lower sleeve clears moving bar",
    )
    ctx.expect_gap(
        secondary,
        slider,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slide_sleeve_side_1",
        negative_elem="side_key_1",
        name="positive guide key rides against sleeve",
    )
    ctx.expect_gap(
        slider,
        secondary,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="side_key_0",
        negative_elem="slide_sleeve_side_0",
        name="negative guide key rides against sleeve",
    )
    ctx.expect_overlap(
        slider,
        secondary,
        axes="x",
        min_overlap=0.280,
        elem_a="inner_slide_bar",
        elem_b="slide_sleeve_top",
        name="collapsed slider remains deeply engaged",
    )

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({secondary_to_slider: 0.260}):
        ctx.expect_overlap(
            slider,
            secondary,
            axes="x",
            min_overlap=0.170,
            elem_a="inner_slide_bar",
            elem_b="slide_sleeve_top",
            name="extended slider retains insertion in guide",
        )
        ctx.expect_within(
            slider,
            secondary,
            axes="y",
            margin=0.002,
            inner_elem="inner_slide_bar",
            outer_elem="slide_sleeve_top",
            name="extended slide remains centered laterally",
        )
        ctx.expect_gap(
            secondary,
            slider,
            axis="z",
            min_gap=0.006,
            max_gap=0.020,
            positive_elem="slide_sleeve_top",
            negative_elem="inner_slide_bar",
            name="extended upper sleeve clears moving bar",
        )
        extended_pos = ctx.part_world_position(slider)

    ctx.check(
        "prismatic output extends along positive local rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

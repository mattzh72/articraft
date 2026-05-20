from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laminated_glass_sunroof_cassette")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_groove = model.material("black_anodized_track", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("smoked_laminated_glass", rgba=(0.20, 0.34, 0.40, 0.38))
    glass_edge = model.material("green_laminate_edge", rgba=(0.42, 0.67, 0.62, 0.55))
    frit = model.material("ceramic_frit", rgba=(0.02, 0.025, 0.026, 0.88))
    slider = model.material("low_friction_slider", rgba=(0.09, 0.095, 0.10, 1.0))

    cassette = model.part("cassette")

    # Overall aluminium cassette frame: real car sunroof cassettes are a little
    # over a meter wide and use a shallow pressed perimeter frame.
    cassette.visual(
        Box((0.055, 1.30, 0.026)),
        origin=Origin(xyz=(-0.505, 0.0, 0.013)),
        material=aluminium,
        name="side_frame_0",
    )
    cassette.visual(
        Box((0.055, 1.30, 0.026)),
        origin=Origin(xyz=(0.505, 0.0, 0.013)),
        material=aluminium,
        name="side_frame_1",
    )
    cassette.visual(
        Box((1.065, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, -0.650, 0.013)),
        material=aluminium,
        name="front_frame",
    )
    cassette.visual(
        Box((1.065, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, 0.650, 0.013)),
        material=aluminium,
        name="rear_frame",
    )

    aperture_frame = BezelGeometry(
        opening_size=(0.900, 0.700),
        outer_size=(1.040, 0.840),
        depth=0.012,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.040,
        outer_corner_radius=0.060,
        center=False,
    )
    cassette.visual(
        mesh_from_geometry(aperture_frame, "aperture_frame"),
        origin=Origin(xyz=(0.0, -0.200, 0.026)),
        material=aluminium,
        name="aperture_frame",
    )

    for suffix, x in (("0", -0.425), ("1", 0.425)):
        cassette.visual(
            Box((0.068, 1.230, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=aluminium,
            name=f"guide_rail_{suffix}",
        )
        cassette.visual(
            Box((0.034, 1.160, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=dark_groove,
            name=f"glide_pad_{suffix}",
        )

    # Flush rubber seals around the roof aperture make the cassette read as a
    # weather-sealed sunroof, not a generic rectangular slider.
    cassette.visual(
        Box((0.850, 0.016, 0.007)),
        origin=Origin(xyz=(0.0, -0.550, 0.0455)),
        material=rubber,
        name="front_seal",
    )
    cassette.visual(
        Box((0.850, 0.016, 0.007)),
        origin=Origin(xyz=(0.0, 0.150, 0.0455)),
        material=rubber,
        name="rear_seal",
    )
    cassette.visual(
        Box((0.014, 0.700, 0.007)),
        origin=Origin(xyz=(-0.458, -0.200, 0.0435)),
        material=rubber,
        name="side_seal_0",
    )
    cassette.visual(
        Box((0.014, 0.700, 0.007)),
        origin=Origin(xyz=(0.458, -0.200, 0.0435)),
        material=rubber,
        name="side_seal_1",
    )

    # Small bolt tabs and drain gutters keep the perimeter frame grounded and
    # reinforce the cassette identity.
    for index, (x, y) in enumerate(
        (
            (-0.505, -0.420),
            (0.505, -0.420),
            (-0.505, 0.420),
            (0.505, 0.420),
        )
    ):
        cassette.visual(
            Box((0.080, 0.045, 0.008)),
            origin=Origin(xyz=(x, y, 0.030)),
            material=aluminium,
            name=f"mount_tab_{index}",
        )

    panel = model.part("glass_panel")
    glass_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.840, 0.620, 0.050, corner_segments=10),
            0.018,
        ),
        "laminated_glass_panel",
    )
    panel.visual(
        glass_panel_mesh,
        origin=Origin(),
        material=glass,
        name="glass_laminate",
    )

    # Dark ceramic frit ring and a faint green edge band are typical of
    # laminated automotive glass.
    panel.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.745, 0.525),
                outer_size=(0.838, 0.618),
                depth=0.002,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.038,
                outer_corner_radius=0.050,
                center=False,
            ),
            "ceramic_frit_border",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=frit,
        name="frit_border",
    )
    panel.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.820, 0.600),
                outer_size=(0.846, 0.626),
                depth=0.003,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.046,
                outer_corner_radius=0.052,
                center=False,
            ),
            "laminated_edge_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=glass_edge,
        name="laminated_edge",
    )

    for suffix, x in (("0", -0.425), ("1", 0.425)):
        panel.visual(
            Box((0.026, 0.480, 0.012)),
            origin=Origin(xyz=(x, 0.0, -0.006)),
            material=slider,
            name=f"slider_shoe_{suffix}",
        )
        panel.visual(
            Box((0.050, 0.150, 0.006)),
            origin=Origin(xyz=(x, -0.170, -0.003)),
            material=slider,
            name=f"front_carrier_{suffix}",
        )
        panel.visual(
            Box((0.050, 0.150, 0.006)),
            origin=Origin(xyz=(x, 0.170, -0.003)),
            material=slider,
            name=f"rear_carrier_{suffix}",
        )

    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=panel,
        # q=0 places the glass centered in the weather seal. Positive travel is
        # rearward along +Y until the rear edge reaches the back of the cassette.
        origin=Origin(xyz=(0.0, -0.200, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.540),
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

    cassette = object_model.get_part("cassette")
    panel = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("panel_slide")

    ctx.check(
        "panel uses rearward prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (0.0, 1.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.53,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    for suffix in ("0", "1"):
        ctx.expect_contact(
            panel,
            cassette,
            elem_a=f"slider_shoe_{suffix}",
            elem_b=f"glide_pad_{suffix}",
            contact_tol=0.0005,
            name=f"slider shoe {suffix} sits on its guide rail",
        )
        ctx.expect_within(
            panel,
            cassette,
            axes="x",
            inner_elem=f"slider_shoe_{suffix}",
            outer_elem=f"glide_pad_{suffix}",
            margin=0.001,
            name=f"slider shoe {suffix} is captured laterally",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            axes="y",
            elem_a=f"slider_shoe_{suffix}",
            elem_b=f"glide_pad_{suffix}",
            min_overlap=0.45,
            name=f"slider shoe {suffix} has retained rail engagement at rest",
        )

    rest_position = ctx.part_world_position(panel)
    with ctx.pose({slide: 0.540}):
        extended_position = ctx.part_world_position(panel)
        for suffix in ("0", "1"):
            ctx.expect_contact(
                panel,
                cassette,
                elem_a=f"slider_shoe_{suffix}",
                elem_b=f"glide_pad_{suffix}",
                contact_tol=0.0005,
                name=f"slider shoe {suffix} remains on rail at full rearward travel",
            )
            ctx.expect_overlap(
                panel,
                cassette,
                axes="y",
                elem_a=f"slider_shoe_{suffix}",
                elem_b=f"glide_pad_{suffix}",
                min_overlap=0.45,
                name=f"slider shoe {suffix} remains retained when rearward",
            )

    ctx.check(
        "glass panel moves fully rearward",
        rest_position is not None
        and extended_position is not None
        and extended_position[1] > rest_position[1] + 0.53,
        details=f"rest={rest_position}, rearward={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()

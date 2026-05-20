from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_camera_stand")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.024, 0.026, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.88, 0.58, 0.82))
    soft_white = model.material("soft_white", rgba=(0.96, 0.93, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.032), rpy=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="floor_hub",
    )

    # Three low tripod legs give the stand a camera-light footprint and keep the
    # vertical sleeve physically supported at floor level.
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        base.visual(
            Box((0.54, 0.035, 0.026)),
            origin=Origin(xyz=(0.245 * c, 0.245 * s, 0.023), rpy=(0.0, 0.0, yaw)),
            material=matte_black,
            name=f"tripod_leg_{idx}",
        )
        base.visual(
            Box((0.090, 0.060, 0.024)),
            origin=Origin(xyz=(0.510 * c, 0.510 * s, 0.020), rpy=(0.0, 0.0, yaw)),
            material=dark_plastic,
            name=f"rubber_foot_{idx}",
        )

    outer_sleeve_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.029, 96),
            [_circle_profile(0.020, 96)],
            0.800,
            center=True,
        ),
        "outer_sleeve_tube",
    )
    base.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.452)),
        material=satin_metal,
        name="outer_sleeve",
    )
    clamp_collar_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.039, 96),
            [_circle_profile(0.020, 96)],
            0.070,
            center=True,
        ),
        "upper_clamp_collar",
    )
    base.visual(
        clamp_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=matte_black,
        name="clamp_collar",
    )
    base.visual(
        Box((0.020, 0.020, 0.070)),
        origin=Origin(xyz=(0.036, 0.0, 0.840)),
        material=matte_black,
        name="clamp_lug",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.014, length=1.300),
        # The sliding tube extends below the joint frame, leaving hidden retained
        # insertion inside the hollow outer sleeve even at maximum height.
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=satin_metal,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=matte_black,
        name="top_collar",
    )
    mast.visual(
        Box((0.055, 0.044, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=matte_black,
        name="bracket_stem",
    )
    mast.visual(
        Box((0.190, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=matte_black,
        name="bracket_saddle",
    )
    mast.visual(
        Box((0.016, 0.052, 0.145)),
        origin=Origin(xyz=(-0.082, 0.0, 0.685)),
        material=matte_black,
        name="bracket_cheek_0",
    )
    mast.visual(
        Box((0.016, 0.052, 0.145)),
        origin=Origin(xyz=(0.082, 0.0, 0.685)),
        material=matte_black,
        name="bracket_cheek_1",
    )

    ring_housing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.185, 128),
            [_circle_profile(0.112, 128)],
            0.046,
            center=True,
        ),
        "ring_housing_annulus",
    )
    mast.visual(
        ring_housing_mesh,
        # The annulus is authored in local XY and extruded along local Z. Rotate
        # it so the light face is vertical and the rear is on +Y.
        origin=Origin(xyz=(0.0, 0.0, 0.815), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="ring_housing",
    )

    diffuser_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.166, 128),
            [_circle_profile(0.127, 128)],
            0.006,
            center=True,
        ),
        "front_diffuser_annulus",
    )
    mast.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.0, -0.026, 0.815), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_diffuser,
        name="front_diffuser",
    )
    inner_lip_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.122, 128),
            [_circle_profile(0.112, 128)],
            0.010,
            center=True,
        ),
        "inner_front_lip_ring",
    )
    mast.visual(
        inner_lip_mesh,
        origin=Origin(xyz=(0.0, -0.028, 0.815), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="inner_front_lip",
    )
    mast.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.029, 0.675), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="rear_knob_boss",
    )
    mast.visual(
        Box((0.030, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.029, 0.642)),
        material=matte_black,
        name="boss_web",
    )
    mast.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(0.112, 0.0, 0.815), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="side_fastener_0",
    )
    mast.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(-0.112, 0.0, 0.815), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_white,
        name="side_fastener_1",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="knob_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.026,
            body_style="faceted",
            top_diameter=0.046,
            base_diameter=0.052,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0020),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "rear_brightness_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_cap",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.855)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.350),
    )
    model.articulation(
        "mast_to_knob",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=knob,
        origin=Origin(xyz=(0.0, 0.035, 0.675)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=5.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    knob = object_model.get_part("knob")
    mast_slide = object_model.get_articulation("base_to_mast")
    knob_turn = object_model.get_articulation("mast_to_knob")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason=(
            "The telescoping mast is intentionally represented as a retained "
            "round tube sliding inside the hollow stand sleeve."
        ),
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="clamp_collar",
        elem_b="inner_tube",
        reason=(
            "The upper clamp collar encircles the same telescoping inner tube "
            "at the sleeve mouth."
        ),
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner mast is centered in the sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.45,
        name="collapsed mast has deep retained insertion",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="clamp_collar",
        margin=0.002,
        name="inner mast passes through the clamp collar",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.060,
        name="inner mast remains inside the clamp collar",
    )

    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.350}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast remains centered in the sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="extended mast remains captured in the sleeve",
        )
        extended_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slides upward at full travel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.34,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        knob,
        mast,
        axis="y",
        positive_elem="knob_shaft",
        negative_elem="rear_knob_boss",
        max_gap=0.001,
        max_penetration=0.001,
        name="brightness knob shaft seats on rear boss",
    )
    ctx.expect_overlap(
        knob,
        mast,
        axes="xz",
        elem_a="knob_shaft",
        elem_b="rear_knob_boss",
        min_overlap=0.018,
        name="brightness knob is centered on rear boss",
    )
    ctx.expect_gap(
        knob,
        knob,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="knob_shaft",
        max_gap=0.001,
        max_penetration=0.001,
        name="knob cap sits on its short shaft",
    )
    with ctx.pose({knob_turn: 2.5}):
        ctx.expect_gap(
            knob,
            mast,
            axis="y",
            positive_elem="knob_shaft",
            negative_elem="rear_knob_boss",
            max_gap=0.001,
            max_penetration=0.001,
            name="brightness knob remains seated while rotated",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    *,
    name: str,
    segments: int = 72,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [list(reversed(_circle_profile(inner_radius, segments=segments)))],
            depth,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_camera_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.63, 0.65, 0.68, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.96, 0.94, 0.94))

    stand_base = model.part("stand_base")
    stand_base.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=powder_black,
        name="tripod_hub",
    )
    stand_base.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=powder_black,
        name="lower_collar",
    )
    stand_base.visual(
        _annulus_mesh(0.0195, 0.0155, 0.700, name="stand_outer_tube_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=powder_black,
        name="outer_sleeve",
    )
    stand_base.visual(
        _annulus_mesh(0.028, 0.0162, 0.050, name="stand_top_socket_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=powder_black,
        name="top_socket",
    )
    stand_base.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.031, 0.0, 0.925), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="clamp_stem",
    )
    stand_base.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.044, 0.0, 0.925), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.030 * c, 0.030 * s, 0.122),
                (0.175 * c, 0.175 * s, 0.074),
                (0.375 * c, 0.375 * s, 0.020),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        stand_base.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_mesh_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        stand_base.visual(
            Cylinder(radius=0.013, length=0.018),
            origin=Origin(xyz=(0.375 * c, 0.375 * s, 0.011)),
            material=rubber_black,
            name=f"foot_{index}",
        )

    stand_base.inertial = Inertial.from_geometry(
        Box((0.80, 0.80, 1.00)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    telescoping_mast = model.part("telescoping_mast")
    telescoping_mast.visual(
        Cylinder(radius=0.0135, length=0.820),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_aluminum,
        name="inner_mast_tube",
    )
    telescoping_mast.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=powder_black,
        name="mast_stop_collar",
    )
    telescoping_mast.visual(
        Cylinder(radius=0.017, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.4325)),
        material=powder_black,
        name="mast_tip_collar",
    )
    telescoping_mast.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.800)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "stand_to_mast",
        ArticulationType.PRISMATIC,
        parent=stand_base,
        child=telescoping_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.950)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.220,
        ),
    )

    light_head = model.part("light_head")
    light_head.visual(
        _annulus_mesh(0.220, 0.140, 0.050, name="ring_light_housing_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="housing_ring",
    )
    light_head.visual(
        _annulus_mesh(0.214, 0.146, 0.008, name="ring_light_diffuser_mesh"),
        origin=Origin(xyz=(0.0, 0.021, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=diffuser_white,
        name="front_diffuser",
    )
    light_head.visual(
        Cylinder(radius=0.019, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_black,
        name="mount_stem",
    )
    light_head.visual(
        Box((0.050, 0.052, 0.220)),
        origin=Origin(xyz=(0.0, -0.010, 0.110)),
        material=powder_black,
        name="rear_bracket_spine",
    )
    light_head.visual(
        Box((0.090, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.006, 0.058)),
        material=powder_black,
        name="lower_mount_block",
    )
    light_head.inertial = Inertial.from_geometry(
        Box((0.470, 0.080, 0.520)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )

    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=telescoping_mast,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
    )

    brightness_knob = model.part("brightness_knob")
    brightness_knob.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="knob_shaft",
    )
    brightness_knob.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    brightness_knob.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="knob_rim",
    )
    brightness_knob.inertial = Inertial.from_geometry(
        Box((0.042, 0.036, 0.042)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )

    model.articulation(
        "head_to_brightness_knob",
        ArticulationType.REVOLUTE,
        parent=light_head,
        child=brightness_knob,
        origin=Origin(xyz=(0.115, -0.025, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
            lower=-2.4,
            upper=2.4,
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

    stand_base = object_model.get_part("stand_base")
    telescoping_mast = object_model.get_part("telescoping_mast")
    light_head = object_model.get_part("light_head")
    brightness_knob = object_model.get_part("brightness_knob")
    mast_slide = object_model.get_articulation("stand_to_mast")
    knob_joint = object_model.get_articulation("head_to_brightness_knob")

    ctx.check(
        "mast articulation axis is vertical",
        tuple(round(value, 6) for value in mast_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={mast_slide.axis}",
    )
    ctx.check(
        "brightness knob articulation axis is fore-aft",
        tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )

    ctx.expect_within(
        telescoping_mast,
        stand_base,
        axes="xy",
        inner_elem="inner_mast_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="mast stays centered in the outer sleeve at rest",
    )
    ctx.expect_overlap(
        telescoping_mast,
        stand_base,
        axes="z",
        elem_a="inner_mast_tube",
        elem_b="outer_sleeve",
        min_overlap=0.250,
        name="collapsed mast retains deep insertion in the sleeve",
    )
    ctx.expect_contact(
        light_head,
        brightness_knob,
        elem_a="housing_ring",
        elem_b="knob_shaft",
        contact_tol=0.0005,
        name="brightness knob shaft seats against the rear housing",
    )

    rest_mast_pos = ctx.part_world_position(telescoping_mast)
    with ctx.pose({mast_slide: 0.220}):
        ctx.expect_within(
            telescoping_mast,
            stand_base,
            axes="xy",
            inner_elem="inner_mast_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="mast stays centered in the outer sleeve when extended",
        )
        ctx.expect_overlap(
            telescoping_mast,
            stand_base,
            axes="z",
            elem_a="inner_mast_tube",
            elem_b="outer_sleeve",
            min_overlap=0.080,
            name="extended mast still retains insertion in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(telescoping_mast)

    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.18,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

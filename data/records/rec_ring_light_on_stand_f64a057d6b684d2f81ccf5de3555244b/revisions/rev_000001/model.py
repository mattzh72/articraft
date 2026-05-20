from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * index) / segments),
            radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_prism_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
    rotate_to_xz: bool = False,
):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        height=depth,
        center=True,
    )
    if rotate_to_xz:
        geometry.rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _add_caster(
    model: ArticulatedObject,
    base,
    *,
    index: int,
    angle: float,
    tip_radius: float,
    base_material,
    fork_material,
    wheel_material,
    hub_material,
) -> None:
    tip_x = tip_radius * cos(angle)
    tip_y = tip_radius * sin(angle)

    base.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(tip_x, tip_y, 0.084)),
        material=base_material,
        name=f"caster_socket_{index}",
    )

    fork = model.part(f"caster_{index}_fork")
    fork.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=fork_material,
        name="stem_collar",
    )
    fork.visual(
        Box((0.026, 0.030, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, -0.010)),
        material=fork_material,
        name="fork_crown",
    )
    fork.visual(
        Box((0.022, 0.004, 0.056)),
        origin=Origin(xyz=(0.016, 0.013, -0.033)),
        material=fork_material,
        name="left_plate",
    )
    fork.visual(
        Box((0.022, 0.004, 0.056)),
        origin=Origin(xyz=(0.016, -0.013, -0.033)),
        material=fork_material,
        name="right_plate",
    )

    wheel = model.part(f"caster_{index}_wheel")
    wheel.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )

    model.articulation(
        f"caster_{index}_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=fork,
        origin=Origin(xyz=(tip_x, tip_y, 0.072), rpy=(0.0, 0.0, angle)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        f"caster_{index}_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.016, 0.0, -0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_ring_light")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.25, 0.26, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.78, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.95, 0.91, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    ring_housing_mesh = _annulus_prism_mesh(
        "ring_light_housing",
        outer_radius=0.235,
        inner_radius=0.170,
        depth=0.038,
        rotate_to_xz=True,
    )
    ring_diffuser_mesh = _annulus_prism_mesh(
        "ring_light_diffuser",
        outer_radius=0.221,
        inner_radius=0.184,
        depth=0.006,
        rotate_to_xz=True,
    )
    hub_shell_mesh = _annulus_prism_mesh(
        "ring_light_hub_shell",
        outer_radius=0.095,
        inner_radius=0.034,
        depth=0.050,
    )
    hub_cap_mesh = _annulus_prism_mesh(
        "ring_light_hub_cap",
        outer_radius=0.060,
        inner_radius=0.018,
        depth=0.045,
    )
    outer_sleeve_mesh = _annulus_prism_mesh(
        "ring_light_outer_sleeve",
        outer_radius=0.024,
        inner_radius=0.018,
        depth=0.420,
    )
    upper_collar_mesh = _annulus_prism_mesh(
        "ring_light_upper_collar",
        outer_radius=0.032,
        inner_radius=0.020,
        depth=0.050,
    )

    base = model.part("base")
    base.visual(
        hub_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=matte_black,
        name="hub_shell",
    )
    base.visual(
        hub_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=graphite,
        name="hub_cap",
    )
    base.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=matte_black,
        name="outer_sleeve",
    )
    base.visual(
        upper_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=matte_black,
        name="upper_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.036, 0.0, 0.555), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_handle",
    )

    leg_tip_radius = 0.340
    for index in range(5):
        angle = (2.0 * pi * index) / 5.0
        c = cos(angle)
        s = sin(angle)
        leg_geom = tube_from_spline_points(
            [
                (0.040 * c, 0.040 * s, 0.081),
                (0.185 * c, 0.185 * s, 0.082),
                (leg_tip_radius * c, leg_tip_radius * s, 0.090),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_geom, f"ring_light_leg_{index}"),
            material=matte_black,
            name=f"leg_{index}",
        )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.016, length=1.220),
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        material=aluminum,
        name="inner_tube",
    )
    inner_mast.visual(
        Cylinder(radius=0.023, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.472)),
        material=graphite,
        name="lower_stop_collar",
    )
    inner_mast.visual(
        Cylinder(radius=0.028, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.980)),
        material=matte_black,
        name="yoke_neck",
    )
    inner_mast.visual(
        Box((0.060, 0.100, 0.100)),
        origin=Origin(xyz=(0.0, -0.050, 1.030)),
        material=matte_black,
        name="yoke_spine",
    )
    inner_mast.visual(
        Box((0.520, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.055, 1.020)),
        material=matte_black,
        name="yoke_bridge",
    )
    for side, x in (("left", -0.252), ("right", 0.252)):
        inner_mast.visual(
            Box((0.014, 0.080, 0.220)),
            origin=Origin(xyz=(x, -0.035, 1.130)),
            material=matte_black,
            name=f"{side}_yoke_side_plate",
        )

    light_head = model.part("light_head")
    light_head.visual(
        ring_housing_mesh,
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material=satin_black,
        name="ring_housing",
    )
    light_head.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.112, 0.0)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    light_head.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="left_trunnion",
    )
    light_head.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="right_trunnion",
    )
    light_head.visual(
        Box((0.026, 0.074, 0.060)),
        origin=Origin(xyz=(-0.212, 0.037, 0.0)),
        material=graphite,
        name="left_mount_ear",
    )
    light_head.visual(
        Box((0.026, 0.074, 0.060)),
        origin=Origin(xyz=(0.212, 0.037, 0.0)),
        material=graphite,
        name="right_mount_ear",
    )
    light_head.visual(
        Box((0.092, 0.030, 0.054)),
        origin=Origin(xyz=(0.0, 0.084, -0.198)),
        material=graphite,
        name="rear_driver_box",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.450),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=light_head,
        origin=Origin(xyz=(0.0, -0.035, 1.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.70, upper=0.95),
    )

    for index in range(5):
        _add_caster(
            model,
            base,
            index=index,
            angle=(2.0 * pi * index) / 5.0,
            tip_radius=leg_tip_radius,
            base_material=matte_black,
            fork_material=graphite,
            wheel_material=rubber,
            hub_material=aluminum,
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
    base = object_model.get_part("base")
    inner_mast = object_model.get_part("inner_mast")
    light_head = object_model.get_part("light_head")
    mast_slide = object_model.get_articulation("mast_slide")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "mast and head articulation axes are configured",
        mast_slide.axis == (0.0, 0.0, 1.0)
        and head_tilt.axis == (1.0, 0.0, 0.0)
        and mast_slide.motion_limits is not None
        and head_tilt.motion_limits is not None,
        details=f"mast_axis={mast_slide.axis}, head_axis={head_tilt.axis}",
    )

    caster_axes_ok = True
    caster_details: list[str] = []
    for index in range(5):
        swivel = object_model.get_articulation(f"caster_{index}_swivel")
        spin = object_model.get_articulation(f"caster_{index}_spin")
        swivel_ok = swivel.axis == (0.0, 0.0, 1.0)
        spin_ok = spin.axis == (0.0, 1.0, 0.0)
        caster_axes_ok = caster_axes_ok and swivel_ok and spin_ok
        caster_details.append(
            f"{index}: swivel={swivel.axis}, spin={spin.axis}"
        )
    ctx.check(
        "caster swivel and wheel spin axes are configured",
        caster_axes_ok,
        details="; ".join(caster_details),
    )

    for index in range(5):
        fork = object_model.get_part(f"caster_{index}_fork")
        ctx.expect_contact(
            fork,
            base,
            elem_a="stem_collar",
            elem_b=f"caster_socket_{index}",
            contact_tol=0.0025,
            name=f"caster {index} stem sits in its base socket",
        )

    with ctx.pose({mast_slide: 0.0}):
        ctx.expect_within(
            inner_mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="inner mast stays centered in outer sleeve at rest",
        )
        ctx.expect_overlap(
            inner_mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.350,
            name="inner mast remains deeply inserted at rest",
        )
        ctx.expect_contact(
            inner_mast,
            base,
            elem_a="lower_stop_collar",
            elem_b="upper_collar",
            contact_tol=0.0005,
            name="mast stop collar seats on the sleeve collar at rest",
        )
        rest_mast_position = ctx.part_world_position(inner_mast)

    with ctx.pose({mast_slide: 0.450}):
        ctx.expect_within(
            inner_mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast stays centered in outer sleeve",
        )
        ctx.expect_overlap(
            inner_mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.060,
            name="extended mast still retains insertion",
        )
        extended_mast_position = ctx.part_world_position(inner_mast)

    ctx.check(
        "mast extends upward",
        rest_mast_position is not None
        and extended_mast_position is not None
        and extended_mast_position[2] > rest_mast_position[2] + 0.30,
        details=f"rest={rest_mast_position}, extended={extended_mast_position}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    with ctx.pose({head_tilt: 0.0}):
        rest_driver = _aabb_center(ctx.part_element_world_aabb(light_head, elem="rear_driver_box"))
    with ctx.pose({head_tilt: 0.80}):
        tilted_driver = _aabb_center(ctx.part_element_world_aabb(light_head, elem="rear_driver_box"))

    ctx.check(
        "light head tilts upward around its horizontal axis",
        rest_driver is not None
        and tilted_driver is not None
        and tilted_driver[1] > rest_driver[1] + 0.10
        and tilted_driver[2] > rest_driver[2] + 0.03,
        details=f"rest={rest_driver}, tilted={tilted_driver}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

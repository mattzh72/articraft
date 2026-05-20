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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_rect_section(width: float, depth: float, corner_radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, corner_radius, corner_segments=8)]


def _sphere_profile(radius: float, z_samples: list[float]) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    for z in z_samples:
        radial_sq = max(radius * radius - z * z, 0.0)
        profile.append((math.sqrt(radial_sq), z))
    return profile


def _rounded_rect_xz_section(
    width: float,
    height: float,
    corner_radius: float,
    y: float,
    *,
    center_z: float,
):
    return [(x, y, z + center_z) for x, z in rounded_rect_profile(width, height, corner_radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_gumball_machine")

    cast_red = model.material("cast_red", rgba=(0.73, 0.10, 0.12, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.18, 0.18, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.82, 1.0))
    smoke = model.material("smoke", rgba=(0.10, 0.10, 0.10, 1.0))
    clear_globe = model.material("clear_globe", rgba=(0.84, 0.92, 1.0, 0.32))

    base_radius = 0.19
    base_thickness = 0.055
    stem_radius = 0.055
    stem_length = 0.50

    housing_base_z = 0.545
    housing_height = 0.185
    housing_sections = [
        _rounded_rect_section(0.235, 0.190, 0.026, 0.000),
        _rounded_rect_section(0.228, 0.186, 0.026, 0.040),
        _rounded_rect_section(0.212, 0.174, 0.024, 0.098),
        _rounded_rect_section(0.176, 0.144, 0.021, 0.150),
        _rounded_rect_section(0.132, 0.110, 0.017, housing_height),
    ]
    housing_mesh = mesh_from_geometry(section_loft(housing_sections), "housing_cast")
    chute_sections = [
        _rounded_rect_xz_section(0.090, 0.054, 0.010, 0.058, center_z=0.531),
        _rounded_rect_xz_section(0.101, 0.062, 0.012, 0.086, center_z=0.529),
        _rounded_rect_xz_section(0.108, 0.070, 0.013, 0.109, center_z=0.523),
    ]
    chute_mesh = mesh_from_geometry(section_loft(chute_sections), "chute_body")

    bowl_radius = 0.170
    bowl_thickness = 0.006
    bowl_center_z = 0.860
    bowl_top_z = 0.155
    bowl_bottom_z = -0.152
    bowl_z_samples = [
        bowl_bottom_z,
        -0.120,
        -0.085,
        -0.045,
        0.000,
        0.050,
        0.095,
        0.128,
        bowl_top_z,
    ]
    bowl_outer = _sphere_profile(bowl_radius, bowl_z_samples)
    bowl_inner = _sphere_profile(bowl_radius - bowl_thickness, bowl_z_samples)
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            bowl_outer,
            bowl_inner,
            segments=96,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "globe_shell",
    )

    crown_ring_top_z = bowl_center_z + bowl_top_z + 0.006
    lid_hinge_z = crown_ring_top_z + 0.012
    lid_hinge_y = -0.074
    lid_radius = 0.077

    body = model.part("body")
    body.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=cast_red,
        name="base_plinth",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.014)),
        material=cast_red,
        name="base_collar",
    )
    body.visual(
        Cylinder(radius=stem_radius, length=stem_length),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + stem_length / 2.0)),
        material=cast_red,
        name="pedestal_column",
    )
    body.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, housing_base_z)),
        material=cast_red,
        name="housing_cast",
    )
    body.visual(
        Cylinder(radius=0.062, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, bowl_center_z + bowl_bottom_z + 0.020)),
        material=chrome,
        name="bowl_neck",
    )
    body.visual(
        bowl_mesh,
        origin=Origin(xyz=(0.0, 0.0, bowl_center_z)),
        material=clear_globe,
        name="bowl_shell",
    )
    body.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, crown_ring_top_z)),
        material=chrome,
        name="crown_collar",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.0, 0.086, 0.602), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_boss",
    )
    body.visual(
        Box((0.118, 0.004, 0.150)),
        origin=Origin(xyz=(0.0, 0.081, 0.594)),
        material=chrome,
        name="front_escutcheon",
    )
    body.visual(
        chute_mesh,
        origin=Origin(),
        material=chrome,
        name="chute_body",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(-0.038, lid_hinge_y, lid_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_hinge_left",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.038, lid_hinge_y, lid_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_hinge_right",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(-0.052, 0.115, 0.498), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_left",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.052, 0.115, 0.498), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_right",
    )
    body.visual(
        Box((0.016, 0.006, 0.012)),
        origin=Origin(xyz=(-0.052, 0.112, 0.498)),
        material=chrome,
        name="door_hinge_left_bracket",
    )
    body.visual(
        Box((0.016, 0.006, 0.012)),
        origin=Origin(xyz=(0.052, 0.112, 0.498)),
        material=chrome,
        name="door_hinge_right_bracket",
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=lid_radius, length=0.008),
        origin=Origin(xyz=(0.0, 0.085, -0.002)),
        material=chrome,
        name="lid_cap",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.112, 0.011)),
        material=smoke,
        name="lid_handle",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.050, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.010, 0.000)),
        material=chrome,
        name="lid_hinge_strap",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.040, length=0.036),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="knob_wheel",
    )
    knob.visual(
        Box((0.074, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        material=smoke,
        name="knob_handle",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.088, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, -0.002, 0.030)),
        material=chrome,
        name="door_panel",
    )
    flap.visual(
        Box((0.046, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.005, 0.055)),
        material=smoke,
        name="door_pull",
    )
    flap.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.093, 0.602)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, 0.115, 0.498)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5, lower=0.0, upper=1.20),
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("knob")
    flap = object_model.get_part("flap")
    lid_hinge = object_model.get_articulation("body_to_lid")
    knob_joint = object_model.get_articulation("body_to_knob")
    flap_hinge = object_model.get_articulation("body_to_flap")

    def span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "articulations match prompt mechanisms",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and knob_joint.axis == (0.0, 1.0, 0.0)
        and flap_hinge.axis == (-1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper >= 1.3
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.upper is not None
        and flap_hinge.motion_limits.upper >= 1.0
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=(
            f"lid_axis={lid_hinge.axis}, knob_axis={knob_joint.axis}, flap_axis={flap_hinge.axis}, "
            f"lid_limits={lid_hinge.motion_limits}, flap_limits={flap_hinge.motion_limits}, "
            f"knob_limits={knob_joint.motion_limits}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0, knob_joint: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="crown_collar",
            max_gap=0.0025,
            max_penetration=0.0005,
            name="closed lid sits on crown collar",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_cap",
            elem_b="crown_collar",
            min_overlap=0.10,
            name="lid covers the refill opening",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_wheel",
            elem_b="knob_boss",
            contact_tol=0.002,
            name="knob seats against boss",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="chute_body",
            max_gap=0.002,
            max_penetration=0.0005,
            name="closed candy flap meets chute front",
        )
        ctx.expect_overlap(
            flap,
            body,
            axes="xz",
            elem_a="door_panel",
            elem_b="chute_body",
            min_overlap=0.05,
            name="closed candy flap covers chute",
        )
        lid_rest = ctx.part_element_world_aabb(lid, elem="lid_cap")
        flap_rest = ctx.part_element_world_aabb(flap, elem="door_panel")
        knob_rest = ctx.part_element_world_aabb(knob, elem="knob_handle")

    with ctx.pose({lid_hinge: 1.10}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_cap")

    with ctx.pose({flap_hinge: 0.95}):
        flap_open = ctx.part_element_world_aabb(flap, elem="door_panel")

    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_turned = ctx.part_element_world_aabb(knob, elem="knob_handle")

    ctx.check(
        "lid opens upward from the bowl crown",
        lid_rest is not None and lid_open is not None and lid_open[1][2] > lid_rest[1][2] + 0.06,
        details=f"closed_lid={lid_rest}, open_lid={lid_open}",
    )
    ctx.check(
        "candy flap swings outward",
        flap_rest is not None and flap_open is not None and flap_open[1][1] > flap_rest[1][1] + 0.03,
        details=f"closed_flap={flap_rest}, open_flap={flap_open}",
    )
    ctx.check(
        "knob handle changes orientation around horizontal shaft",
        knob_rest is not None
        and knob_turned is not None
        and span(knob_rest, 0) is not None
        and span(knob_rest, 2) is not None
        and span(knob_turned, 0) is not None
        and span(knob_turned, 2) is not None
        and span(knob_rest, 0) > span(knob_rest, 2) + 0.03
        and span(knob_turned, 2) > span(knob_turned, 0) + 0.03,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

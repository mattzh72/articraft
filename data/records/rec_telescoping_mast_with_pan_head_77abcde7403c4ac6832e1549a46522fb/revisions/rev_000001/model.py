from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _hollow_square_tube(name: str, outer: float, wall: float, length: float):
    """Return a managed mesh for a straight machined square tube."""
    inner = outer - 2.0 * wall
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer, outer, min(outer * 0.08, 0.010), corner_segments=5),
        [
            rounded_rect_profile(
                inner,
                inner,
                min(inner * 0.06, max(0.001, wall * 0.35)),
                corner_segments=5,
            )
        ],
        length,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _square_collar(name: str, outer: float, inner: float, height: float):
    """Return a short split-clamp style square collar mesh."""
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer, outer, min(outer * 0.07, 0.010), corner_segments=5),
        [rounded_rect_profile(inner, inner, min(inner * 0.05, 0.006), corner_segments=5)],
        height,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def _add_square_tube_walls(part, prefix: str, outer: float, wall: float, length: float, z_center: float, material: str):
    """Add a collision-friendly four-wall square tube made from touching plates."""
    half = outer / 2.0
    front_back_y = half - wall / 2.0
    side_x = half - wall / 2.0
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, -front_back_y, z_center)),
        material=material,
        name=f"{prefix}_front_skin",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, front_back_y, z_center)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((wall, outer, length)),
        origin=Origin(xyz=(-side_x, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_0",
    )
    part.visual(
        Box((wall, outer, length)),
        origin=Origin(xyz=(side_x, 0.0, z_center)),
        material=material,
        name=f"{prefix}_side_1",
    )


def _add_square_collar_walls(part, prefix: str, outer: float, inner: float, height: float, z_center: float, material: str):
    wall = (outer - inner) / 2.0
    _add_square_tube_walls(part, prefix, outer, wall, height, z_center, material)


def _eye_ring(name: str, radius: float = 0.024, tube: float = 0.004):
    return mesh_from_geometry(TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=36), name)


def _add_bolt_grid(part, material: str, *, x_span: float, y_span: float, z: float, radius: float = 0.006):
    for ix, x in enumerate((-x_span / 2.0, x_span / 2.0)):
        for iy, y in enumerate((-y_span / 2.0, y_span / 2.0)):
            part.visual(
                Cylinder(radius=radius, length=0.006),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"bolt_{ix}_{iy}",
            )


def _add_cover(part, material: str, screw_material: str):
    part.visual(
        Box((0.115, 0.008, 0.170)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=material,
        name="cover_plate",
    )
    for ix, x in enumerate((-0.043, 0.043)):
        for iz, z in enumerate((-0.060, 0.060)):
            part.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x, -0.010, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=screw_material,
                name=f"captive_screw_{ix}_{iz}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_pan_head_study")

    model.material("dark_anodized_aluminum", rgba=(0.07, 0.08, 0.085, 1.0))
    model.material("brushed_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    model.material("black_oxide", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    model.material("wear_pad", rgba=(0.09, 0.11, 0.10, 1.0))
    model.material("warning_red", rgba=(0.65, 0.06, 0.035, 1.0))

    # Root fabricated base and lower mast sleeve.
    lower = model.part("lower_stage")
    lower.visual(
        Box((0.52, 0.42, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="dark_anodized_aluminum",
        name="floor_plate",
    )
    lower.visual(
        Box((0.27, 0.23, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material="brushed_steel",
        name="socket_plate",
    )
    _add_square_tube_walls(lower, "lower_tube", 0.190, 0.014, 0.950, 0.528, "dark_anodized_aluminum")
    lower.visual(
        Box((0.190, 0.014, 0.950)),
        origin=Origin(xyz=(0.0, -0.088, 0.528)),
        material="dark_anodized_aluminum",
        name="lower_tube_front",
    )
    _add_square_collar_walls(lower, "bottom_collar", 0.236, 0.184, 0.070, 0.100, "machined_aluminum")
    _add_square_collar_walls(lower, "top_collar", 0.236, 0.184, 0.080, 0.963, "machined_aluminum")
    # Fabricated ribs tying the mast socket into the base plate.
    lower.visual(
        Box((0.018, 0.185, 0.150)),
        origin=Origin(xyz=(0.112, 0.0, 0.098)),
        material="dark_anodized_aluminum",
        name="side_rib_0",
    )
    lower.visual(
        Box((0.018, 0.185, 0.150)),
        origin=Origin(xyz=(-0.112, 0.0, 0.098)),
        material="dark_anodized_aluminum",
        name="side_rib_1",
    )
    lower.visual(
        Box((0.170, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.102, 0.098)),
        material="dark_anodized_aluminum",
        name="cross_rib_0",
    )
    lower.visual(
        Box((0.170, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, -0.102, 0.098)),
        material="dark_anodized_aluminum",
        name="cross_rib_1",
    )
    # Exposed anti-rotation guide rail and split clamp ears.
    lower.visual(
        Box((0.026, 0.012, 0.730)),
        origin=Origin(xyz=(0.103, 0.0, 0.545)),
        material="brushed_steel",
        name="side_guide_rail",
    )
    for i, x in enumerate((-0.035, 0.035)):
        lower.visual(
            Box((0.018, 0.046, 0.046)),
            origin=Origin(xyz=(x, -0.138, 0.980)),
            material="machined_aluminum",
            name=f"clamp_ear_{i}",
        )
    lower.visual(
        Cylinder(radius=0.0075, length=0.105),
        origin=Origin(xyz=(0.0, -0.153, 0.985), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="clamp_screw",
    )
    lower.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.052, -0.153, 0.985), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="clamp_nut",
    )
    # Guy attachment eyes are welded to the top collar, not floating rings.
    for i, (x, y, yaw) in enumerate(((0.154, 0.0, 0.0), (-0.154, 0.0, pi), (0.0, 0.154, pi / 2.0), (0.0, -0.154, -pi / 2.0))):
        lug_x = 0.118 if x > 0 else (-0.118 if x < 0 else 0.0)
        lug_y = 0.118 if y > 0 else (-0.118 if y < 0 else 0.0)
        lower.visual(
            Box((0.060 if x else 0.022, 0.022 if x else 0.060, 0.014)),
            origin=Origin(xyz=(lug_x, lug_y, 0.972)),
            material="brushed_steel",
            name=f"guy_lug_{i}",
        )
        lower.visual(
            _eye_ring(f"lower_guy_eye_{i}"),
            origin=Origin(xyz=(x, y, 0.972), rpy=(pi / 2.0, 0.0, yaw)),
            material="brushed_steel",
            name=f"guy_eye_{i}",
        )
    _add_bolt_grid(lower, "black_oxide", x_span=0.420, y_span=0.320, z=0.028, radius=0.0065)

    # First sliding mast stage.  The geometry extends below the joint frame to
    # keep retained insertion through the whole prismatic travel.
    mid = model.part("mid_stage")
    _add_square_tube_walls(mid, "mid_tube", 0.128, 0.010, 1.350, 0.175, "machined_aluminum")
    mid.visual(
        Box((0.128, 0.010, 1.350)),
        origin=Origin(xyz=(0.0, -0.059, 0.175)),
        material="machined_aluminum",
        name="mid_tube_front",
    )
    _add_square_collar_walls(mid, "top_collar", 0.172, 0.124, 0.066, 0.800, "dark_anodized_aluminum")
    mid.visual(
        Box((0.008, 0.042, 0.920)),
        origin=Origin(xyz=(0.069, 0.0, 0.320)),
        material="brushed_steel",
        name="stage_key",
    )
    for zi, z in enumerate((-0.410, -0.085)):
        for side, x in enumerate((-0.0725, 0.0725)):
            mid.visual(
                Box((0.017, 0.056, 0.050)),
                origin=Origin(xyz=(x, 0.0, z)),
                material="wear_pad",
                name=f"lower_guide_shoe_{zi}_{side}",
            )
        for side, y in enumerate((-0.0725, 0.0725)):
            mid.visual(
                Box((0.056, 0.017, 0.050)),
                origin=Origin(xyz=(0.0, y, z)),
                material="wear_pad",
                name=f"lower_guide_pad_{zi}_{side}",
            )
    for i, x in enumerate((-0.026, 0.026)):
        mid.visual(
            Box((0.014, 0.038, 0.038)),
            origin=Origin(xyz=(x, -0.105, 0.814)),
            material="dark_anodized_aluminum",
            name=f"clamp_ear_{i}",
        )
    mid.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.0, -0.118, 0.816), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="clamp_screw",
    )
    for i, (x, yaw) in enumerate(((0.112, 0.0), (-0.112, pi))):
        mid.visual(
            Box((0.045, 0.018, 0.010)),
            origin=Origin(xyz=(0.088 if x > 0 else -0.088, 0.0, 0.805)),
            material="brushed_steel",
            name=f"guy_lug_{i}",
        )
        mid.visual(
            _eye_ring(f"mid_guy_eye_{i}", radius=0.018, tube=0.0035),
            origin=Origin(xyz=(x, 0.0, 0.805), rpy=(pi / 2.0, 0.0, yaw)),
            material="brushed_steel",
            name=f"guy_eye_{i}",
        )

    # Second sliding mast stage, carrying the pan bearing.
    upper = model.part("upper_stage")
    _add_square_tube_walls(upper, "upper_tube", 0.084, 0.008, 1.100, 0.130, "dark_anodized_aluminum")
    upper.visual(
        Box((0.084, 0.008, 1.100)),
        origin=Origin(xyz=(0.0, -0.038, 0.130)),
        material="dark_anodized_aluminum",
        name="upper_tube_front",
    )
    _add_square_collar_walls(upper, "head_collar", 0.130, 0.080, 0.064, 0.655, "machined_aluminum")
    for zi, z in enumerate((-0.330, -0.080)):
        for side, x in enumerate((-0.048, 0.048)):
            upper.visual(
                Box((0.012, 0.038, 0.044)),
                origin=Origin(xyz=(x, 0.0, z)),
                material="wear_pad",
                name=f"mid_guide_shoe_{zi}_{side}",
            )
        for side, y in enumerate((-0.048, 0.048)):
            upper.visual(
                Box((0.038, 0.012, 0.044)),
                origin=Origin(xyz=(0.0, y, z)),
                material="wear_pad",
                name=f"mid_guide_pad_{zi}_{side}",
            )
    upper.visual(
        Cylinder(radius=0.076, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.696)),
        material="brushed_steel",
        name="bearing_flange",
    )
    upper.visual(
        Cylinder(radius=0.052, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.738)),
        material="black_oxide",
        name="bearing_stator",
    )
    _add_bolt_grid(upper, "black_oxide", x_span=0.105, y_span=0.105, z=0.708, radius=0.0048)

    pan = model.part("pan_head")
    pan.visual(
        Cylinder(radius=0.047, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="machined_aluminum",
        name="rotor_hub",
    )
    pan.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="machined_aluminum",
        name="hub_spacer",
    )
    pan.visual(
        _eye_ring("pan_index_ring", radius=0.055, tube=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material="brushed_steel",
        name="index_ring",
    )
    pan.visual(
        Box((0.180, 0.140, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material="dark_anodized_aluminum",
        name="pan_plate",
    )
    for i, x in enumerate((-0.065, 0.065)):
        pan.visual(
            Box((0.010, 0.108, 0.132)),
            origin=Origin(xyz=(x, 0.0, 0.136)),
            material="machined_aluminum",
            name=f"cheek_plate_{i}",
        )
    for i, y in enumerate((-0.053, 0.053)):
        pan.visual(
            Box((0.140, 0.010, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.092)),
            material="dark_anodized_aluminum",
            name=f"cheek_bridge_{i}",
        )
    pan.visual(
        Cylinder(radius=0.009, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.142), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_oxide",
        name="cross_pin",
    )
    for i, x in enumerate((-0.087, 0.087)):
        pan.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(x * 0.977, 0.0, 0.142), rpy=(0.0, pi / 2.0, 0.0)),
            material="black_oxide",
            name=f"pin_cap_{i}",
        )
    pan.visual(
        Box((0.140, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material="brushed_steel",
        name="head_saddle",
    )
    pan.visual(
        Box((0.118, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.066, 0.083)),
        material="warning_red",
        name="pan_lock_tab",
    )
    _add_bolt_grid(pan, "black_oxide", x_span=0.120, y_span=0.085, z=0.079, radius=0.0045)

    lower_cover = model.part("lower_cover")
    _add_cover(lower_cover, "machined_aluminum", "black_oxide")
    mid_cover = model.part("mid_cover")
    _add_cover(mid_cover, "dark_anodized_aluminum", "black_oxide")

    model.articulation(
        "lower_to_mid",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=mid,
        origin=Origin(xyz=(0.0, 0.0, 1.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.360),
        motion_properties=MotionProperties(damping=6.0, friction=3.0),
    )
    model.articulation(
        "mid_to_upper",
        ArticulationType.PRISMATIC,
        parent=mid,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.270),
        motion_properties=MotionProperties(damping=5.0, friction=2.5),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.CONTINUOUS,
        parent=upper,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, 0.771)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4),
        motion_properties=MotionProperties(damping=1.2, friction=0.6),
    )
    model.articulation(
        "lower_to_cover",
        ArticulationType.FIXED,
        parent=lower,
        child=lower_cover,
        origin=Origin(xyz=(0.0, -0.095, 0.515)),
    )
    model.articulation(
        "mid_to_cover",
        ArticulationType.FIXED,
        parent=mid,
        child=mid_cover,
        origin=Origin(xyz=(0.0, -0.064, 0.360)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_stage")
    mid = object_model.get_part("mid_stage")
    upper = object_model.get_part("upper_stage")
    pan = object_model.get_part("pan_head")
    lower_cover = object_model.get_part("lower_cover")
    mid_cover = object_model.get_part("mid_cover")
    lower_slide = object_model.get_articulation("lower_to_mid")
    upper_slide = object_model.get_articulation("mid_to_upper")
    pan_joint = object_model.get_articulation("upper_to_pan")

    ctx.expect_within(
        mid,
        lower,
        axes="x",
        inner_elem="mid_tube_front",
        outer_elem="lower_tube_front",
        margin=0.002,
        name="mid stage stays between lower sleeve side walls",
    )
    ctx.expect_overlap(
        mid,
        lower,
        axes="z",
        elem_a="mid_tube_front",
        elem_b="lower_tube_front",
        min_overlap=0.45,
        name="mid stage has retained insertion when collapsed",
    )
    ctx.expect_within(
        upper,
        mid,
        axes="x",
        inner_elem="upper_tube_front",
        outer_elem="mid_tube_front",
        margin=0.002,
        name="upper stage stays between mid sleeve side walls",
    )
    ctx.expect_overlap(
        upper,
        mid,
        axes="z",
        elem_a="upper_tube_front",
        elem_b="mid_tube_front",
        min_overlap=0.38,
        name="upper stage has retained insertion when collapsed",
    )
    ctx.expect_contact(
        pan,
        upper,
        elem_a="rotor_hub",
        elem_b="bearing_stator",
        contact_tol=0.002,
        name="pan rotor is supported by top bearing",
    )
    ctx.expect_contact(
        lower_cover,
        lower,
        elem_a="cover_plate",
        elem_b="lower_tube_front",
        contact_tol=0.003,
        name="lower access cover is mounted to sleeve face",
    )
    ctx.expect_contact(
        mid_cover,
        mid,
        elem_a="cover_plate",
        elem_b="mid_tube_front",
        contact_tol=0.003,
        name="mid access cover is mounted to sleeve face",
    )

    rest_upper_pos = ctx.part_world_position(upper)
    with ctx.pose({lower_slide: 0.360, upper_slide: 0.270, pan_joint: 1.2}):
        ctx.expect_overlap(
            mid,
            lower,
            axes="z",
            elem_a="mid_tube_front",
            elem_b="lower_tube_front",
            min_overlap=0.12,
            name="mid stage remains captured when extended",
        )
        ctx.expect_overlap(
            upper,
            mid,
            axes="z",
            elem_a="upper_tube_front",
            elem_b="mid_tube_front",
            min_overlap=0.10,
            name="upper stage remains captured when extended",
        )
        extended_upper_pos = ctx.part_world_position(upper)

    ctx.check(
        "telescoping stages extend upward",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.55,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )
    ctx.check(
        "pan joint is vertical and continuous",
        tuple(round(v, 3) for v in pan_joint.axis) == (0.0, 0.0, 1.0)
        and str(pan_joint.articulation_type).lower().endswith("continuous"),
        details=f"type={pan_joint.articulation_type}, axis={pan_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()

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
)


def _circle_profile(radius: float, *, segments: int = 72, clockwise: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
    axis: str,
    name: str,
    segments: int = 72,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments, clockwise=True)],
        height=depth,
        center=True,
    )
    if axis == "x":
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        geom.rotate_x(-math.pi / 2.0)
    elif axis != "z":
        raise ValueError(f"Unsupported annulus axis: {axis}")
    return mesh_from_geometry(geom, name)


def _rpy_for_z_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = direction
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-9:
        raise ValueError("Direction must be non-zero.")
    x /= length
    y /= length
    z /= length
    yaw = math.atan2(y, x)
    pitch = math.atan2(math.sqrt(x * x + y * y), z)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="selfie_ring_light_tripod")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.36, 0.38, 0.41, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.95, 0.92, 0.96))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    ring_shell_mesh = _annulus_mesh(
        outer_radius=0.19,
        inner_radius=0.13,
        depth=0.028,
        axis="y",
        name="ring_shell",
    )
    diffuser_mesh = _annulus_mesh(
        outer_radius=0.184,
        inner_radius=0.136,
        depth=0.004,
        axis="y",
        name="ring_diffuser",
    )
    hinge_z = 0.325
    hub_radius = 0.033
    leg_mount_radius = 0.045
    leg_length = 0.34
    leg_open_angle = math.radians(38.0)

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        _annulus_mesh(
            outer_radius=hub_radius,
            inner_radius=0.0152,
            depth=0.064,
            axis="z",
            name="hub_drum",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.317)),
        material=dark_aluminum,
        name="hub_drum",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        lower_mast.visual(
            Box((0.012, 0.012, 0.016)),
            origin=Origin(
                xyz=(0.039 * math.cos(angle), 0.039 * math.sin(angle), hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"leg_support_{index}",
        )
        lower_mast.visual(
            Box((0.010, 0.010, 0.632)),
            origin=Origin(
                xyz=(0.018 * math.cos(angle), 0.018 * math.sin(angle), 0.665)
            ),
            material=aluminum,
            name=f"guide_rib_{index}",
        )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        lower_mast.visual(
            Box((0.012, 0.012, 0.030)),
            origin=Origin(
                xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), 0.965)
            ),
            material=dark_aluminum,
            name=f"guide_cap_{index}",
        )
    lower_mast.visual(
        Box((0.018, 0.010, 0.040)),
        origin=Origin(xyz=(0.026, 0.0, 0.935)),
        material=matte_black,
        name="lock_knob",
    )
    lower_mast.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 1.02)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.0135, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=aluminum,
        name="upper_tube",
    )
    upper_mast.visual(
        Box((0.050, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.658)),
        material=dark_aluminum,
        name="stop_collar",
    )
    upper_mast.visual(
        Cylinder(radius=0.012, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=dark_aluminum,
        name="yoke_stem",
    )
    upper_mast.visual(
        Box((0.40, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, 0.69)),
        material=dark_aluminum,
        name="yoke_bridge",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        upper_mast.visual(
            Box((0.200, 0.012, 0.020)),
            origin=Origin(xyz=(0.100 * sign, -0.017, 0.79)),
            material=dark_aluminum,
            name=f"{side}_yoke_arm",
        )
        upper_mast.visual(
            Box((0.016, 0.012, 0.070)),
            origin=Origin(xyz=(0.202 * sign, -0.017, 0.825)),
            material=dark_aluminum,
            name=f"{side}_cheek_riser",
        )
        upper_mast.visual(
            Box((0.008, 0.012, 0.030)),
            origin=Origin(xyz=(0.209 * sign, -0.006, 0.855)),
            material=dark_aluminum,
            name=f"{side}_yoke_cheek",
        )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.46, 0.05, 0.92)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        ring_shell_mesh,
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=satin_black,
        name="ring_shell",
    )
    ring_head.visual(
        diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material=diffuser_white,
        name="diffuser",
    )
    ring_head.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(-0.201, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="left_trunnion",
    )
    ring_head.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.201, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="right_trunnion",
    )
    ring_head.visual(
        Box((0.036, 0.086, 0.032)),
        origin=Origin(xyz=(-0.179, 0.038, 0.0)),
        material=dark_aluminum,
        name="left_side_bracket",
    )
    ring_head.visual(
        Box((0.036, 0.086, 0.032)),
        origin=Origin(xyz=(0.179, 0.038, 0.0)),
        material=dark_aluminum,
        name="right_side_bracket",
    )
    ring_head.visual(
        Box((0.024, 0.018, 0.17)),
        origin=Origin(xyz=(0.0, 0.029, 0.0)),
        material=matte_black,
        name="clamp_spine",
    )
    ring_head.visual(
        Box((0.020, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, 0.029, -0.100)),
        material=matte_black,
        name="lower_support",
    )
    ring_head.visual(
        Box((0.144, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.029, 0.039)),
        material=dark_aluminum,
        name="upper_rail",
    )
    ring_head.visual(
        Box((0.144, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.029, -0.039)),
        material=dark_aluminum,
        name="lower_rail",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 0.42)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    def add_leg(name: str, angle: float) -> None:
        radial = (math.cos(angle), math.sin(angle), 0.0)
        tangent = (-math.sin(angle), math.cos(angle), 0.0)
        leg_direction = (
            math.sin(leg_open_angle) * radial[0],
            math.sin(leg_open_angle) * radial[1],
            -math.cos(leg_open_angle),
        )
        beam_rpy = _rpy_for_z_axis(leg_direction)
        tangent_rpy = _rpy_for_z_axis(tangent)
        radial_rpy = _rpy_for_z_axis(radial)
        knuckle_length = 0.055
        end_x = radial[0] * knuckle_length + leg_direction[0] * leg_length
        end_y = radial[1] * knuckle_length + leg_direction[1] * leg_length
        end_z = leg_direction[2] * leg_length

        leg = model.part(name)
        leg.visual(
            Cylinder(radius=0.0075, length=knuckle_length),
            origin=Origin(
                xyz=(radial[0] * knuckle_length * 0.5, radial[1] * knuckle_length * 0.5, 0.0),
                rpy=radial_rpy,
            ),
            material=dark_aluminum,
            name="hinge_spacer",
        )
        leg.visual(
            Cylinder(radius=0.0105, length=leg_length),
            origin=Origin(
                xyz=(
                    radial[0] * knuckle_length + leg_direction[0] * leg_length * 0.5,
                    radial[1] * knuckle_length + leg_direction[1] * leg_length * 0.5,
                    leg_direction[2] * leg_length * 0.5,
                ),
                rpy=beam_rpy,
            ),
            material=dark_aluminum,
            name="leg_tube",
        )
        leg.visual(
            Box((0.024, 0.018, 0.050)),
            origin=Origin(
                xyz=(
                    radial[0] * 0.040 + leg_direction[0] * 0.12,
                    radial[1] * 0.040 + leg_direction[1] * 0.12,
                    leg_direction[2] * 0.12,
                ),
                rpy=beam_rpy,
            ),
            material=matte_black,
            name="leg_strut",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.032),
            origin=Origin(
                xyz=(
                    radial[0] * knuckle_length + leg_direction[0] * (leg_length - 0.016),
                    radial[1] * knuckle_length + leg_direction[1] * (leg_length - 0.016),
                    leg_direction[2] * (leg_length - 0.016),
                ),
                rpy=beam_rpy,
            ),
            material=dark_aluminum,
            name="foot_adapter",
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(xyz=(end_x, end_y, end_z + 0.012)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.08, 0.08, 0.45)),
            mass=0.28,
            origin=Origin(
                xyz=(
                    radial[0] * 0.030 + leg_direction[0] * 0.16,
                    radial[1] * 0.030 + leg_direction[1] * 0.16,
                    leg_direction[2] * 0.18,
                )
            ),
        )
        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent=lower_mast,
            child=leg,
            origin=Origin(
                xyz=(leg_mount_radius * radial[0], leg_mount_radius * radial[1], hinge_z)
            ),
            axis=tangent,
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.2,
                lower=-0.80,
                upper=0.18,
            ),
        )

    add_leg("front_leg", 0.0)
    add_leg("rear_left_leg", 2.0 * math.pi / 3.0)
    add_leg("rear_right_leg", 4.0 * math.pi / 3.0)

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, 0.039)),
        material=dark_aluminum,
        name="upper_sleeve",
    )
    left_jaw.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, -0.039)),
        material=dark_aluminum,
        name="lower_sleeve",
    )
    left_jaw.visual(
        Box((0.018, 0.014, 0.074)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=matte_black,
        name="carriage",
    )
    left_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, 0.044)),
        material=dark_aluminum,
        name="upper_clip_top",
    )
    left_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, 0.034)),
        material=dark_aluminum,
        name="upper_clip_bottom",
    )
    left_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, -0.034)),
        material=dark_aluminum,
        name="lower_clip_top",
    )
    left_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, -0.044)),
        material=dark_aluminum,
        name="lower_clip_bottom",
    )
    left_jaw.visual(
        Box((0.010, 0.024, 0.094)),
        origin=Origin(xyz=(0.008, 0.040, 0.0)),
        material=matte_black,
        name="jaw_pad",
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.110)),
        mass=0.04,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, 0.039)),
        material=dark_aluminum,
        name="upper_sleeve",
    )
    right_jaw.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, -0.039)),
        material=dark_aluminum,
        name="lower_sleeve",
    )
    right_jaw.visual(
        Box((0.018, 0.014, 0.074)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=matte_black,
        name="carriage",
    )
    right_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, 0.044)),
        material=dark_aluminum,
        name="upper_clip_top",
    )
    right_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, 0.034)),
        material=dark_aluminum,
        name="upper_clip_bottom",
    )
    right_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, -0.034)),
        material=dark_aluminum,
        name="lower_clip_top",
    )
    right_jaw.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.031, -0.044)),
        material=dark_aluminum,
        name="lower_clip_bottom",
    )
    right_jaw.visual(
        Box((0.010, 0.024, 0.094)),
        origin=Origin(xyz=(-0.008, 0.040, 0.0)),
        material=matte_black,
        name="jaw_pad",
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.110)),
        mass=0.04,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.30,
            lower=0.0,
            upper=0.38,
        ),
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "left_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=ring_head,
        child=left_jaw,
        origin=Origin(xyz=(-0.043, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "right_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=ring_head,
        child=right_jaw,
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    lower_mast = object_model.get_part("lower_mast")
    upper_mast = object_model.get_part("upper_mast")
    ring_head = object_model.get_part("ring_head")
    front_leg = object_model.get_part("front_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")

    mast_slide = object_model.get_articulation("mast_slide")
    ring_tilt = object_model.get_articulation("ring_tilt")
    front_leg_hinge = object_model.get_articulation("front_leg_hinge")
    rear_left_leg_hinge = object_model.get_articulation("rear_left_leg_hinge")
    rear_right_leg_hinge = object_model.get_articulation("rear_right_leg_hinge")
    left_jaw_slide = object_model.get_articulation("left_jaw_slide")
    right_jaw_slide = object_model.get_articulation("right_jaw_slide")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mast_slide_axis_vertical",
        mast_slide.axis == (0.0, 0.0, 1.0),
        f"Expected vertical mast slide axis, got {mast_slide.axis!r}",
    )
    ctx.check(
        "ring_tilt_axis_horizontal",
        ring_tilt.axis == (1.0, 0.0, 0.0),
        f"Expected horizontal ring tilt axis, got {ring_tilt.axis!r}",
    )
    for joint in (front_leg_hinge, rear_left_leg_hinge, rear_right_leg_hinge):
        ctx.check(
            f"{joint.name}_axis_horizontal",
            abs(joint.axis[2]) < 1e-9,
            f"Tripod leg hinge axis should stay horizontal, got {joint.axis!r}",
        )
    ctx.check(
        "jaw_slides_are_opposed",
        left_jaw_slide.axis == (-1.0, 0.0, 0.0) and right_jaw_slide.axis == (1.0, 0.0, 0.0),
        f"Jaw slide axes should oppose along x, got {left_jaw_slide.axis!r} and {right_jaw_slide.axis!r}",
    )

    ctx.expect_contact(upper_mast, lower_mast, elem_a="stop_collar", elem_b="guide_cap_0")
    ctx.expect_contact(ring_head, upper_mast, elem_a="left_trunnion", elem_b="left_yoke_cheek")
    ctx.expect_contact(ring_head, upper_mast, elem_a="right_trunnion", elem_b="right_yoke_cheek")
    ctx.expect_contact(front_leg, lower_mast)
    ctx.expect_contact(rear_left_leg, lower_mast)
    ctx.expect_contact(rear_right_leg, lower_mast)
    ctx.expect_contact(left_jaw, ring_head, elem_a="upper_sleeve", elem_b="upper_rail")
    ctx.expect_contact(left_jaw, ring_head, elem_a="lower_sleeve", elem_b="lower_rail")
    ctx.expect_contact(right_jaw, ring_head, elem_a="upper_sleeve", elem_b="upper_rail")
    ctx.expect_contact(right_jaw, ring_head, elem_a="lower_sleeve", elem_b="lower_rail")

    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        positive_elem="jaw_pad",
        negative_elem="jaw_pad",
        min_gap=0.055,
        max_gap=0.065,
        name="closed_phone_clamp_gap",
    )

    ring_rest_aabb = ctx.part_world_aabb(ring_head)
    front_leg_rest_aabb = ctx.part_world_aabb(front_leg)
    assert ring_rest_aabb is not None
    assert front_leg_rest_aabb is not None

    with ctx.pose({mast_slide: 0.32}):
        ring_extended_aabb = ctx.part_world_aabb(ring_head)
        assert ring_extended_aabb is not None
        ctx.check(
            "mast_extension_raises_ring_head",
            ring_extended_aabb[0][2] > ring_rest_aabb[0][2] + 0.30,
            f"Expected mast extension to raise ring head, got rest min z {ring_rest_aabb[0][2]:.4f} and extended min z {ring_extended_aabb[0][2]:.4f}",
        )

    with ctx.pose({ring_tilt: 0.60}):
        tilted_ring_aabb = ctx.part_world_aabb(ring_head)
        assert tilted_ring_aabb is not None
        rest_y_span = ring_rest_aabb[1][1] - ring_rest_aabb[0][1]
        tilted_y_span = tilted_ring_aabb[1][1] - tilted_ring_aabb[0][1]
        ctx.check(
            "ring_head_visibly_tilts",
            tilted_y_span > rest_y_span + 0.10,
            f"Expected ring tilt to swing the head out of plane, got y spans {rest_y_span:.4f} and {tilted_y_span:.4f}",
        )
        ctx.expect_contact(ring_head, upper_mast, elem_a="left_trunnion", elem_b="left_yoke_cheek")
        ctx.expect_contact(ring_head, upper_mast, elem_a="right_trunnion", elem_b="right_yoke_cheek")

    with ctx.pose({front_leg_hinge: -0.60}):
        folded_leg_aabb = ctx.part_world_aabb(front_leg)
        assert folded_leg_aabb is not None
        ctx.check(
            "front_leg_folds_upward",
            folded_leg_aabb[0][2] > front_leg_rest_aabb[0][2] + 0.04,
            f"Expected folded leg to lift, got rest min z {front_leg_rest_aabb[0][2]:.4f} and folded min z {folded_leg_aabb[0][2]:.4f}",
        )
        ctx.expect_contact(front_leg, lower_mast)

    with ctx.pose({left_jaw_slide: 0.018, right_jaw_slide: 0.018}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="jaw_pad",
            negative_elem="jaw_pad",
            min_gap=0.091,
            max_gap=0.101,
            name="open_phone_clamp_gap",
        )
        ctx.expect_within(left_jaw, ring_head, axes="x", inner_elem="upper_sleeve", outer_elem="upper_rail")
        ctx.expect_within(left_jaw, ring_head, axes="x", inner_elem="lower_sleeve", outer_elem="lower_rail")
        ctx.expect_within(right_jaw, ring_head, axes="x", inner_elem="upper_sleeve", outer_elem="upper_rail")
        ctx.expect_within(right_jaw, ring_head, axes="x", inner_elem="lower_sleeve", outer_elem="lower_rail")
        ctx.expect_contact(left_jaw, ring_head, elem_a="upper_sleeve", elem_b="upper_rail")
        ctx.expect_contact(right_jaw, ring_head, elem_a="upper_sleeve", elem_b="upper_rail")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

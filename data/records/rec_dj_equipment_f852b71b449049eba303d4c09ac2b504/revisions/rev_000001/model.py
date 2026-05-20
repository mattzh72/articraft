from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hex_controller_profile(
    outer_half_width: float,
    outer_half_depth: float,
    shoulder_half_width: float,
) -> list[tuple[float, float]]:
    return [
        (outer_half_width, 0.0),
        (shoulder_half_width, outer_half_depth),
        (-shoulder_half_width, outer_half_depth),
        (-outer_half_width, 0.0),
        (-shoulder_half_width, -outer_half_depth),
        (shoulder_half_width, -outer_half_depth),
    ]


def _scale_profile(
    profile: list[tuple[float, float]],
    factor_x: float,
    factor_y: float,
) -> list[tuple[float, float]]:
    return [(x * factor_x, y * factor_y) for x, y in profile]


def _add_encoder_part(
    model: ArticulatedObject,
    *,
    name: str,
    knob_material,
    marker_material,
):
    encoder = model.part(name)
    encoder.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=knob_material,
        name="encoder_base",
    )
    encoder.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=knob_material,
        name="encoder_skirt",
    )
    encoder.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=knob_material,
        name="encoder_cap",
    )
    encoder.visual(
        Box((0.0035, 0.010, 0.0015)),
        origin=Origin(xyz=(0.0, 0.009, 0.02175)),
        material=marker_material,
        name="encoder_marker",
    )
    encoder.inertial = Inertial.from_geometry(
        Box((0.038, 0.038, 0.0225)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.01125)),
    )
    return encoder


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_platter_controller")

    housing_dark = model.material("housing_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    deck_black = model.material("deck_black", rgba=(0.08, 0.09, 0.10, 1.0))
    platter_black = model.material("platter_black", rgba=(0.10, 0.10, 0.11, 1.0))
    touch_surface = model.material("touch_surface", rgba=(0.18, 0.18, 0.20, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    marker_orange = model.material("marker_orange", rgba=(0.93, 0.48, 0.18, 1.0))
    rubber_foot = model.material("rubber_foot", rgba=(0.06, 0.06, 0.07, 1.0))

    outer_profile = _hex_controller_profile(
        outer_half_width=0.235,
        outer_half_depth=0.135,
        shoulder_half_width=0.145,
    )
    inner_profile = _scale_profile(outer_profile, 0.87, 0.86)

    wall_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            0.050,
            cap=True,
            center=False,
        ),
        "controller_wall_ring",
    )
    top_deck_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(outer_profile, 0.004),
        "controller_top_deck",
    )
    bottom_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(outer_profile, 0.004),
        "controller_bottom_plate",
    )

    housing = model.part("housing")
    housing.visual(
        wall_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=housing_dark,
        name="wall_ring",
    )
    housing.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=deck_black,
        name="top_deck",
    )
    housing.visual(
        bottom_plate_mesh,
        material=housing_dark,
        name="bottom_plate",
    )
    housing.visual(
        Cylinder(radius=0.124, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.05525)),
        material=accent_silver,
        name="platter_surround",
    )
    housing.visual(
        Cylinder(radius=0.036, length=0.0065),
        origin=Origin(xyz=(0.0, 0.0, 0.05675)),
        material=accent_silver,
        name="bearing_collar",
    )
    housing.visual(
        Cylinder(radius=0.021, length=0.0035),
        origin=Origin(xyz=(-0.165, 0.0, 0.05525)),
        material=accent_silver,
        name="left_encoder_pad",
    )
    housing.visual(
        Cylinder(radius=0.021, length=0.0035),
        origin=Origin(xyz=(0.165, 0.0, 0.05525)),
        material=accent_silver,
        name="right_encoder_pad",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.145, -0.085), (-0.145, 0.085), (0.145, -0.085), (0.145, 0.085))
    ):
        housing.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, -0.0015)),
            material=rubber_foot,
            name=f"foot_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((0.470, 0.270, 0.060)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=accent_silver,
        name="hub_boss",
    )
    platter.visual(
        Cylinder(radius=0.112, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=platter_black,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.118, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=accent_silver,
        name="rim_band",
    )
    platter.visual(
        Cylinder(radius=0.086, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=touch_surface,
        name="touch_disc",
    )
    platter.visual(
        Cylinder(radius=0.028, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.01775)),
        material=accent_silver,
        name="center_badge",
    )
    platter.inertial = Inertial.from_geometry(
        Box((0.236, 0.236, 0.0185)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.00925)),
    )

    left_encoder = _add_encoder_part(
        model,
        name="left_encoder",
        knob_material=platter_black,
        marker_material=marker_orange,
    )
    right_encoder = _add_encoder_part(
        model,
        name="right_encoder",
        knob_material=platter_black,
        marker_material=marker_orange,
    )

    model.articulation(
        "housing_to_platter",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_left_encoder",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_encoder,
        origin=Origin(xyz=(-0.165, 0.0, 0.057)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=30.0),
    )
    model.articulation(
        "housing_to_right_encoder",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_encoder,
        origin=Origin(xyz=(0.165, 0.0, 0.057)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    platter = object_model.get_part("platter")
    left_encoder = object_model.get_part("left_encoder")
    right_encoder = object_model.get_part("right_encoder")

    platter_spin = object_model.get_articulation("housing_to_platter")
    left_spin = object_model.get_articulation("housing_to_left_encoder")
    right_spin = object_model.get_articulation("housing_to_right_encoder")

    bearing_collar = housing.get_visual("bearing_collar")
    left_pad = housing.get_visual("left_encoder_pad")
    right_pad = housing.get_visual("right_encoder_pad")
    platter_hub = platter.get_visual("hub_boss")
    left_base = left_encoder.get_visual("encoder_base")
    right_base = right_encoder.get_visual("encoder_base")

    for check_name, joint in (
        ("platter spin is continuous", platter_spin),
        ("left encoder spin is continuous", left_spin),
        ("right encoder spin is continuous", right_spin),
    ):
        limits = joint.motion_limits
        ctx.check(
            check_name,
            joint.joint_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and joint.axis == (0.0, 0.0, 1.0),
            details=(
                f"type={joint.joint_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )

    with ctx.pose({platter_spin: 1.7, left_spin: 0.8, right_spin: -1.2}):
        ctx.expect_contact(
            platter,
            housing,
            elem_a=platter_hub,
            elem_b=bearing_collar,
            name="platter hub stays mounted on bearing collar",
        )
        ctx.expect_contact(
            left_encoder,
            housing,
            elem_a=left_base,
            elem_b=left_pad,
            name="left encoder stays seated on its mounting pad",
        )
        ctx.expect_contact(
            right_encoder,
            housing,
            elem_a=right_base,
            elem_b=right_pad,
            name="right encoder stays seated on its mounting pad",
        )
        ctx.expect_origin_gap(
            platter,
            left_encoder,
            axis="x",
            min_gap=0.150,
            max_gap=0.180,
            name="left encoder flanks platter from the left",
        )
        ctx.expect_origin_gap(
            right_encoder,
            platter,
            axis="x",
            min_gap=0.150,
            max_gap=0.180,
            name="right encoder flanks platter from the right",
        )
        ctx.expect_origin_distance(
            left_encoder,
            right_encoder,
            axes="y",
            max_dist=0.002,
            name="encoder pair stays aligned across the deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

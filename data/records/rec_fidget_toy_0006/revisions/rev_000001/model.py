from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    rounded_rect_profile,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)

END_DISC_CENTER_X = 0.029
END_DISC_RADIUS = 0.015
END_DISC_THICKNESS = 0.0066
ARM_LENGTH = 0.032
ARM_WIDTH = 0.0105
ARM_THICKNESS = 0.0048
ARM_CENTER_X = 0.0145
CENTER_BODY_RADIUS = 0.0145
CENTER_DISC_RADIUS = 0.0135
CENTER_DISC_THICKNESS = 0.0058
BEARING_OUTER_RADIUS = 0.0112
BEARING_INNER_RADIUS = 0.0046
BEARING_THICKNESS = 0.0070
AXLE_RADIUS = 0.0036
BUTTON_RADIUS = 0.0068
BUTTON_THICKNESS = 0.0022
BUTTON_CENTER_Z = 0.5 * (BEARING_THICKNESS + BUTTON_THICKNESS)
AXLE_LENGTH = BUTTON_CENTER_Z * 2.0


def _ring_mesh(filename: str, *, outer_radius: float, inner_radius: float, thickness: float):
    outer = superellipse_profile(
        outer_radius * 2.0,
        outer_radius * 2.0,
        exponent=2.0,
        segments=48,
    )
    inner = superellipse_profile(
        inner_radius * 2.0,
        inner_radius * 2.0,
        exponent=2.0,
        segments=48,
    )
    ring = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        thickness,
        center=True,
    )
    return mesh_from_geometry(ring, ASSETS.mesh_path(filename))


def _bridge_mesh(filename: str):
    bridge = ExtrudeGeometry(
        rounded_rect_profile(
            ARM_LENGTH,
            ARM_WIDTH,
            radius=min(ARM_WIDTH * 0.48, ARM_THICKNESS * 0.7),
            corner_segments=8,
        ),
        ARM_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(bridge, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_fidget_spinner", assets=ASSETS)

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    weight_steel = model.material("weight_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    cap_aluminum = model.material("cap_aluminum", rgba=(0.84, 0.85, 0.88, 1.0))

    grip_core = model.part("grip_core")
    grip_core.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        material=bearing_steel,
        name="axle",
    )
    grip_core.visual(
        Cylinder(radius=BUTTON_RADIUS, length=BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_CENTER_Z)),
        material=cap_aluminum,
        name="top_button",
    )
    grip_core.visual(
        Cylinder(radius=BUTTON_RADIUS, length=BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_CENTER_Z)),
        material=cap_aluminum,
        name="bottom_button",
    )
    grip_core.inertial = Inertial.from_geometry(
        Cylinder(radius=BUTTON_RADIUS, length=AXLE_LENGTH + BUTTON_THICKNESS),
        mass=0.014,
        origin=Origin(),
    )

    spinner = model.part("spinner")
    spinner.visual(
        _bridge_mesh("spinner_left_bridge.obj"),
        origin=Origin(xyz=(-ARM_CENTER_X, 0.0, 0.0)),
        material=body_black,
        name="left_bridge",
    )
    spinner.visual(
        _bridge_mesh("spinner_right_bridge.obj"),
        origin=Origin(xyz=(ARM_CENTER_X, 0.0, 0.0)),
        material=body_black,
        name="right_bridge",
    )
    spinner.visual(
        _ring_mesh(
            "spinner_center_body_pad.obj",
            outer_radius=CENTER_BODY_RADIUS,
            inner_radius=BEARING_OUTER_RADIUS + 0.0004,
            thickness=ARM_THICKNESS,
        ),
        material=body_black,
        name="center_body_pad",
    )
    spinner.visual(
        Cylinder(radius=END_DISC_RADIUS, length=END_DISC_THICKNESS),
        origin=Origin(xyz=(-END_DISC_CENTER_X, 0.0, 0.0)),
        material=weight_steel,
        name="left_weight_disc",
    )
    spinner.visual(
        Cylinder(radius=END_DISC_RADIUS, length=END_DISC_THICKNESS),
        origin=Origin(xyz=(END_DISC_CENTER_X, 0.0, 0.0)),
        material=weight_steel,
        name="right_weight_disc",
    )
    spinner.visual(
        _ring_mesh(
            "spinner_center_weight_disc.obj",
            outer_radius=CENTER_DISC_RADIUS,
            inner_radius=BEARING_OUTER_RADIUS + 0.0002,
            thickness=CENTER_DISC_THICKNESS,
        ),
        material=weight_steel,
        name="center_weight_disc",
    )
    spinner.visual(
        _ring_mesh(
            "spinner_bearing_ring.obj",
            outer_radius=BEARING_OUTER_RADIUS,
            inner_radius=BEARING_INNER_RADIUS,
            thickness=BEARING_THICKNESS,
        ),
        material=bearing_steel,
        name="bearing_housing",
    )
    spinner.inertial = Inertial.from_geometry(
        Box((0.088, 0.031, BEARING_THICKNESS)),
        mass=0.048,
        origin=Origin(),
    )

    model.articulation(
        "center_spin",
        ArticulationType.REVOLUTE,
        parent=grip_core,
        child=spinner,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=60.0,
            lower=-8.0 * math.pi,
            upper=8.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grip_core = object_model.get_part("grip_core")
    spinner = object_model.get_part("spinner")
    center_spin = object_model.get_articulation("center_spin")

    axle = grip_core.get_visual("axle")
    top_button = grip_core.get_visual("top_button")
    bottom_button = grip_core.get_visual("bottom_button")
    left_bridge = spinner.get_visual("left_bridge")
    right_bridge = spinner.get_visual("right_bridge")
    center_body_pad = spinner.get_visual("center_body_pad")
    left_disc = spinner.get_visual("left_weight_disc")
    right_disc = spinner.get_visual("right_weight_disc")
    center_disc = spinner.get_visual("center_weight_disc")
    bearing_housing = spinner.get_visual("bearing_housing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    limits = center_spin.motion_limits
    ctx.check(
        "center_spin_axis_is_vertical",
        tuple(center_spin.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical spin axis, got {center_spin.axis}",
    )
    ctx.check(
        "center_spin_has_free_spin_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -math.tau
        and limits.upper >= math.tau,
        details="spinner should be able to make multiple full turns around the center bearing",
    )

    ctx.expect_origin_distance(spinner, grip_core, axes="xy", max_dist=0.0005)
    ctx.expect_contact(
        spinner,
        grip_core,
        elem_a=bearing_housing,
        elem_b=top_button,
        name="top_button_contacts_bearing_housing",
    )
    ctx.expect_contact(
        spinner,
        grip_core,
        elem_a=bearing_housing,
        elem_b=bottom_button,
        name="bottom_button_contacts_bearing_housing",
    )
    ctx.expect_within(
        grip_core,
        spinner,
        axes="xy",
        inner_elem=axle,
        outer_elem=bearing_housing,
        name="axle_runs_inside_bearing_hole",
    )
    ctx.expect_gap(
        grip_core,
        spinner,
        axis="z",
        positive_elem=top_button,
        negative_elem=bearing_housing,
        max_gap=0.0002,
        max_penetration=1e-6,
        name="top_button_seats_on_upper_bearing_face",
    )
    ctx.expect_gap(
        spinner,
        grip_core,
        axis="z",
        positive_elem=bearing_housing,
        negative_elem=bottom_button,
        max_gap=0.0002,
        max_penetration=1e-6,
        name="bottom_button_seats_on_lower_bearing_face",
    )
    ctx.expect_within(
        spinner,
        spinner,
        axes="xy",
        inner_elem=bearing_housing,
        outer_elem=center_body_pad,
        name="bearing_housing_stays_inside_center_pad",
    )
    ctx.expect_overlap(
        spinner,
        spinner,
        axes="xy",
        elem_a=left_bridge,
        elem_b=left_disc,
        min_overlap=0.010,
        name="left_weight_disc_blends_into_bar_bridge",
    )
    ctx.expect_overlap(
        spinner,
        spinner,
        axes="xy",
        elem_a=right_bridge,
        elem_b=right_disc,
        min_overlap=0.010,
        name="right_weight_disc_blends_into_bar_bridge",
    )
    ctx.expect_overlap(
        spinner,
        spinner,
        axes="xy",
        elem_a=center_body_pad,
        elem_b=center_disc,
        min_overlap=0.024,
        name="center_weight_disc_is_concentric_with_body_pad",
    )
    ctx.expect_gap(
        spinner,
        spinner,
        axis="x",
        positive_elem=bearing_housing,
        negative_elem=left_disc,
        min_gap=0.002,
        name="left_weight_disc_sits_outboard_of_center_bearing",
    )
    ctx.expect_gap(
        spinner,
        spinner,
        axis="x",
        positive_elem=right_disc,
        negative_elem=bearing_housing,
        min_gap=0.002,
        name="right_weight_disc_sits_outboard_of_center_bearing",
    )

    with ctx.pose({center_spin: math.pi * 0.5}):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_no_floating")
        ctx.expect_origin_distance(spinner, grip_core, axes="xy", max_dist=0.0005)
        ctx.expect_contact(
            spinner,
            grip_core,
            elem_a=bearing_housing,
            elem_b=top_button,
            name="top_button_contact_persists_in_spin_pose",
        )
        ctx.expect_contact(
            spinner,
            grip_core,
            elem_a=bearing_housing,
            elem_b=bottom_button,
            name="bottom_button_contact_persists_in_spin_pose",
        )
        ctx.expect_within(
            grip_core,
            spinner,
            axes="xy",
            inner_elem=axle,
            outer_elem=bearing_housing,
            name="axle_stays_centered_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

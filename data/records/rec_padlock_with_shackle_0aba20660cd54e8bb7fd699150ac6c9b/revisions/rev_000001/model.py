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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


BODY_WIDTH = 0.042
BODY_DEPTH = 0.018
BODY_HEIGHT = 0.047
BODY_CORNER_RADIUS = 0.0045
BODY_MESH_NAME = "padlock_body_shell_v3"

SHACKLE_RADIUS = 0.0030
SHACKLE_PIVOT_RADIUS = 0.0026
SHACKLE_PIVOT_BARREL_LENGTH = 0.0080
SHACKLE_CHEEK_LENGTH = 0.0050
SHACKLE_LEG_SPAN = 0.028
SHACKLE_LEFT_X = -0.014
SHACKLE_AXIS_Z = BODY_HEIGHT + SHACKLE_PIVOT_RADIUS
SHACKLE_BOTTOM_LOCAL_Z = SHACKLE_RADIUS - SHACKLE_PIVOT_RADIUS
SHACKLE_MESH_NAME = "padlock_shackle_v2"

KEY_PLUG_RADIUS = 0.0064
KEY_PLUG_LENGTH = 0.0032
KEY_PLUG_CENTER_Z = 0.0185
KEY_SLOT_WIDTH = 0.0018
KEY_SLOT_HEIGHT = 0.0072
KEY_SLOT_DEPTH = 0.0008


def _build_body_mesh():
    def body_section(width: float, depth: float, z: float, radius: float):
        return [
            (x, y, z)
            for x, y in rounded_rect_profile(
                width,
                depth,
                radius,
                corner_segments=8,
            )
        ]

    return section_loft(
        [
            body_section(BODY_WIDTH, BODY_DEPTH, 0.0, BODY_CORNER_RADIUS),
            body_section(BODY_WIDTH * 0.992, BODY_DEPTH * 0.990, 0.018, BODY_CORNER_RADIUS * 0.96),
            body_section(BODY_WIDTH * 0.955, BODY_DEPTH * 0.970, 0.035, BODY_CORNER_RADIUS * 0.88),
            body_section(BODY_WIDTH * 0.905, BODY_DEPTH * 0.930, BODY_HEIGHT, BODY_CORNER_RADIUS * 0.72),
        ]
    )


def _build_shackle_mesh():
    return tube_from_spline_points(
        [
            (0.000, 0.000, SHACKLE_PIVOT_RADIUS),
            (0.000, 0.000, 0.024),
            (0.003, 0.000, 0.032),
            (0.010, 0.000, 0.038),
            (0.018, 0.000, 0.040),
            (0.025, 0.000, 0.038),
            (SHACKLE_LEG_SPAN, 0.000, 0.032),
            (SHACKLE_LEG_SPAN, 0.000, SHACKLE_BOTTOM_LOCAL_Z),
        ],
        radius=SHACKLE_RADIUS,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_keyed_padlock")

    brass = model.material("brass", rgba=(0.74, 0.60, 0.24, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.57, 0.43, 0.16, 1.0))
    steel = model.material("hardened_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.35, 0.38, 1.0))
    shadow = model.material("slot_shadow", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_body_mesh(), BODY_MESH_NAME),
        material=brass,
        name="body_shell",
    )
    body.visual(
        Box((0.024, 0.0022, 0.020)),
        origin=Origin(
            xyz=(0.0, -(BODY_DEPTH / 2.0) + 0.0002, KEY_PLUG_CENTER_Z),
        ),
        material=brass_dark,
        name="face_plate",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.0020),
        origin=Origin(
            xyz=(0.0, -(BODY_DEPTH / 2.0) + 0.0001, KEY_PLUG_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="key_escutcheon",
    )

    cheek_center_y = (BODY_DEPTH / 2.0) - (SHACKLE_CHEEK_LENGTH / 2.0)
    cheek_origin = Origin(
        xyz=(SHACKLE_LEFT_X, 0.0, SHACKLE_AXIS_Z),
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    body.visual(
        Cylinder(radius=SHACKLE_PIVOT_RADIUS, length=SHACKLE_CHEEK_LENGTH),
        origin=Origin(
            xyz=(SHACKLE_LEFT_X, -cheek_center_y, SHACKLE_AXIS_Z - 0.0008),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass_dark,
        name="pivot_cheek_front",
    )
    body.visual(
        Cylinder(radius=SHACKLE_PIVOT_RADIUS, length=SHACKLE_CHEEK_LENGTH),
        origin=Origin(
            xyz=(SHACKLE_LEFT_X, cheek_center_y, SHACKLE_AXIS_Z - 0.0008),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass_dark,
        name="pivot_cheek_rear",
    )
    body.visual(
        Box((0.0062, BODY_DEPTH - 0.003, 0.0032)),
        origin=Origin(
            xyz=(SHACKLE_LEFT_X, 0.0, BODY_HEIGHT - 0.0008),
        ),
        material=brass_dark,
        name="pivot_bridge",
    )
    body.visual(
        Box((0.0060, BODY_DEPTH - 0.003, 0.0028)),
        origin=Origin(
            xyz=(SHACKLE_LEFT_X + SHACKLE_LEG_SPAN, 0.0, BODY_HEIGHT - 0.0010),
        ),
        material=brass_dark,
        name="receiver_bridge",
    )

    shackle = model.part("shackle")
    shackle.visual(
        Cylinder(radius=SHACKLE_PIVOT_RADIUS, length=SHACKLE_PIVOT_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    shackle.visual(
        mesh_from_geometry(_build_shackle_mesh(), SHACKLE_MESH_NAME),
        material=steel,
        name="shackle_rod",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_RADIUS, length=0.010),
        origin=Origin(
            xyz=(SHACKLE_LEG_SPAN, 0.0, 0.0054),
        ),
        material=steel,
        name="release_leg_segment",
    )

    key_plug = model.part("key_plug")
    key_plug.visual(
        Cylinder(radius=KEY_PLUG_RADIUS, length=KEY_PLUG_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="plug_core",
    )
    key_plug.visual(
        Box((KEY_SLOT_WIDTH, KEY_SLOT_DEPTH, KEY_SLOT_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(KEY_PLUG_LENGTH / 2.0) + (KEY_SLOT_DEPTH / 2.0), 0.0),
        ),
        material=shadow,
        name="key_slot",
    )
    key_plug.visual(
        Box((0.0048, KEY_SLOT_DEPTH, 0.0016)),
        origin=Origin(
            xyz=(0.0, -(KEY_PLUG_LENGTH / 2.0) + (KEY_SLOT_DEPTH / 2.0), -0.0015),
        ),
        material=shadow,
        name="key_bit_ward",
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(SHACKLE_LEFT_X, 0.0, SHACKLE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_key_plug",
        ArticulationType.REVOLUTE,
        parent=body,
        child=key_plug,
        origin=Origin(
            xyz=(0.0, -(BODY_DEPTH / 2.0) - (KEY_PLUG_LENGTH / 2.0), KEY_PLUG_CENTER_Z),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    key_plug = object_model.get_part("key_plug")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    key_joint = object_model.get_articulation("body_to_key_plug")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        body is not None and shackle is not None and key_plug is not None,
        "Body, shackle, and key plug must all exist.",
    )
    ctx.check(
        "shackle_joint_axis",
        tuple(shackle_joint.axis) == (0.0, -1.0, 0.0),
        f"Expected shackle axis (0, -1, 0), got {shackle_joint.axis}.",
    )
    ctx.check(
        "key_joint_axis",
        tuple(key_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected key plug axis (0, 1, 0), got {key_joint.axis}.",
    )

    with ctx.pose({shackle_joint: 0.0, key_joint: 0.0}):
        ctx.expect_gap(
            shackle,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.0012,
            negative_elem="body_shell",
            name="shackle_closed_seated_on_body",
        )
        ctx.expect_overlap(
            shackle,
            body,
            axes="xy",
            min_overlap=0.005,
            elem_b="body_shell",
            name="shackle_closed_aligned_over_body",
        )
        ctx.expect_contact(
            shackle,
            body,
            contact_tol=0.001,
            name="shackle_closed_contact",
        )
        ctx.expect_contact(
            key_plug,
            body,
            contact_tol=0.0005,
            name="key_plug_mounted_to_face",
        )

    shackle_limits = shackle_joint.motion_limits
    if shackle_limits is not None and shackle_limits.lower is not None and shackle_limits.upper is not None:
        with ctx.pose({shackle_joint: shackle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shackle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shackle_lower_no_floating")

        shackle_rest_aabb = ctx.part_element_world_aabb(
            shackle,
            elem="release_leg_segment",
        )
        assert shackle_rest_aabb is not None
        with ctx.pose({shackle_joint: shackle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shackle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shackle_upper_no_floating")
            ctx.expect_contact(
                shackle,
                body,
                contact_tol=0.001,
                name="shackle_open_still_captive",
            )
            shackle_open_aabb = ctx.part_element_world_aabb(
                shackle,
                elem="release_leg_segment",
            )
            assert shackle_open_aabb is not None
            ctx.check(
                "shackle_opens_upward",
                shackle_open_aabb[0][2] > shackle_rest_aabb[0][2] + 0.015,
                "Open shackle should raise the released leg above its closed position.",
            )
            ctx.check(
                "shackle_released_side_lifts",
                shackle_open_aabb[0][0] < shackle_rest_aabb[0][0] - 0.008,
                "Open shackle should swing leftward around the captive leg pivot.",
            )

    key_limits = key_joint.motion_limits
    if key_limits is not None and key_limits.lower is not None and key_limits.upper is not None:
        with ctx.pose({key_joint: key_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="key_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="key_lower_no_floating")

        slot_rest_aabb = ctx.part_element_world_aabb(key_plug, elem="key_slot")
        assert slot_rest_aabb is not None
        with ctx.pose({key_joint: key_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="key_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="key_upper_no_floating")
            ctx.expect_contact(
                key_plug,
                body,
                contact_tol=0.0005,
                name="key_plug_stays_seated_when_turned",
            )
            slot_open_aabb = ctx.part_element_world_aabb(key_plug, elem="key_slot")
            assert slot_open_aabb is not None
            rest_x = slot_rest_aabb[1][0] - slot_rest_aabb[0][0]
            rest_z = slot_rest_aabb[1][2] - slot_rest_aabb[0][2]
            open_x = slot_open_aabb[1][0] - slot_open_aabb[0][0]
            open_z = slot_open_aabb[1][2] - slot_open_aabb[0][2]
            ctx.check(
                "key_slot_rotates_to_horizontal",
                open_x > rest_x + 0.004 and open_z < rest_z - 0.004,
                "Turning the plug should rotate the slot from vertical toward horizontal.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

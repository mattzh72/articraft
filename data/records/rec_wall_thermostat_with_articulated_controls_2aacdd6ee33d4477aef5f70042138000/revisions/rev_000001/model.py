from __future__ import annotations

from math import pi, sin, cos

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate_mesh(
    name: str,
    *,
    width: float,
    height: float,
    depth: float,
    radius: float,
):
    """Rounded rectangle extruded as a wall-mounted plate.

    The mesh local frame uses X for width, Z for height, and Y for wall depth.
    Its front and rear faces are centered about local Y=0; the caller places it
    by offsetting the visual origin in Y.
    """

    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        depth,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_wall_thermostat")

    aged_ivory = model.material("aged_ivory", rgba=(0.83, 0.78, 0.66, 1.0))
    warm_face = model.material("warm_face", rgba=(0.90, 0.86, 0.74, 1.0))
    dial_plastic = model.material("dial_plastic", rgba=(0.86, 0.82, 0.70, 1.0))
    dark_print = model.material("dark_print", rgba=(0.19, 0.18, 0.15, 1.0))
    slot_black = model.material("slot_black", rgba=(0.05, 0.055, 0.055, 1.0))
    red_mark = model.material("red_mark", rgba=(0.75, 0.12, 0.08, 1.0))
    blue_mark = model.material("blue_mark", rgba=(0.10, 0.27, 0.70, 1.0))
    amber_mark = model.material("amber_mark", rgba=(0.92, 0.48, 0.10, 1.0))
    slider_plastic = model.material("slider_plastic", rgba=(0.12, 0.12, 0.11, 1.0))

    front_y = 0.006
    dial_z = 0.012
    slider_z = -0.047

    housing = model.part("housing")
    housing.visual(
        _rounded_plate_mesh(
            "thermostat_back_shell",
            width=0.108,
            height=0.130,
            depth=0.024,
            radius=0.014,
        ),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=aged_ivory,
        name="back_shell",
    )
    housing.visual(
        _rounded_plate_mesh(
            "thermostat_front_cover",
            width=0.100,
            height=0.120,
            depth=0.006,
            radius=0.012,
        ),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=warm_face,
        name="front_cover",
    )
    housing.visual(
        Cylinder(radius=0.0375, length=0.0010),
        origin=Origin(xyz=(0.0, front_y + 0.0005, dial_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=warm_face,
        name="dial_recess_ring",
    )

    # Raised temperature tick marks around the dial.  The long axis of each
    # mark is radial in the X/Z front plane; they are printed/ridged on the
    # front cover rather than separate floating pieces.
    for index in range(13):
        theta = (-120.0 + index * 20.0) * pi / 180.0
        major = index in (0, 3, 6, 9, 12)
        mark_length = 0.0080 if major else 0.0055
        mark_width = 0.0016 if major else 0.0012
        radius = 0.0415
        housing.visual(
            Box((mark_width, 0.0010, mark_length)),
            origin=Origin(
                xyz=(radius * sin(theta), front_y + 0.0005, dial_z + radius * cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=dark_print,
            name=f"scale_tick_{index}",
        )

    # Short horizontal guide slot for mode selection at the bottom.
    housing.visual(
        _rounded_plate_mesh(
            "mode_guide_slot",
            width=0.052,
            height=0.008,
            depth=0.0008,
            radius=0.0035,
        ),
        origin=Origin(xyz=(0.0, front_y + 0.0004, slider_z)),
        material=slot_black,
        name="mode_slot",
    )
    for x, mat, dot_name in (
        (-0.018, blue_mark, "cool_dot"),
        (0.0, dark_print, "off_dot"),
        (0.018, amber_mark, "heat_dot"),
    ):
        housing.visual(
            Cylinder(radius=0.0020, length=0.0008),
            origin=Origin(xyz=(x, front_y + 0.0004, slider_z - 0.012), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=dot_name,
        )

    dial = model.part("dial")
    dial_profile = [
        (0.0000, 0.0000),
        (0.0285, 0.0000),
        (0.0315, 0.0016),
        (0.0315, 0.0075),
        (0.0290, 0.0115),
        (0.0230, 0.0140),
        (0.0000, 0.0140),
    ]
    dial.visual(
        mesh_from_geometry(LatheGeometry(dial_profile, segments=72), "thermostat_dial_cap"),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dial_plastic,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.0255, length=0.0015),
        origin=Origin(xyz=(0.0, 0.01475, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=warm_face,
        name="dial_face",
    )
    dial.visual(
        Box((0.0036, 0.0012, 0.019)),
        origin=Origin(xyz=(0.0, 0.0146, 0.0185)),
        material=red_mark,
        name="pointer_line",
    )

    slider = model.part("mode_slider")
    slider.visual(
        Box((0.012, 0.0020, 0.0058)),
        origin=Origin(xyz=(0.0, 0.0010, 0.0)),
        material=slider_plastic,
        name="slot_runner",
    )
    slider.visual(
        Box((0.016, 0.0060, 0.0110)),
        origin=Origin(xyz=(0.0, 0.0045, 0.0)),
        material=slider_plastic,
        name="thumb_tab",
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, front_y, dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=6.0),
    )
    model.articulation(
        "mode_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        origin=Origin(xyz=(0.0, front_y + 0.0008, slider_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.014, upper=0.014),
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

    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    slider = object_model.get_part("mode_slider")
    dial_joint = object_model.get_articulation("dial_spin")
    slide_joint = object_model.get_articulation("mode_slide")

    ctx.check(
        "dial uses continuous rotation",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(dial_joint, 'articulation_type', None)}",
    )
    ctx.check(
        "mode selector is prismatic",
        getattr(slide_joint, "articulation_type", None) == ArticulationType.PRISMATIC,
        details=f"type={getattr(slide_joint, 'articulation_type', None)}",
    )

    ctx.expect_contact(
        dial,
        housing,
        elem_a="dial_cap",
        elem_b="front_cover",
        contact_tol=0.0005,
        name="dial seats on front cover",
    )
    ctx.expect_contact(
        slider,
        housing,
        elem_a="slot_runner",
        elem_b="mode_slot",
        contact_tol=0.0005,
        name="slider rides in guide slot",
    )
    ctx.expect_within(
        slider,
        housing,
        axes="xz",
        inner_elem="slot_runner",
        outer_elem="mode_slot",
        margin=0.001,
        name="centered slider runner stays inside guide",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="pointer_line")
    with ctx.pose({dial_joint: pi / 2.0}):
        pointer_quarter = ctx.part_element_world_aabb(dial, elem="pointer_line")

    if pointer_rest is not None and pointer_quarter is not None:
        rest_center_x = (pointer_rest[0][0] + pointer_rest[1][0]) * 0.5
        quarter_center_x = (pointer_quarter[0][0] + pointer_quarter[1][0]) * 0.5
        ctx.check(
            "dial pointer rotates about central axis",
            abs(quarter_center_x - rest_center_x) > 0.012,
            details=f"rest_x={rest_center_x:.4f}, quarter_x={quarter_center_x:.4f}",
        )
    else:
        ctx.fail("dial pointer rotates about central axis", "pointer_line AABB unavailable")

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({slide_joint: 0.014}):
        ctx.expect_within(
            slider,
            housing,
            axes="xz",
            inner_elem="slot_runner",
            outer_elem="mode_slot",
            margin=0.001,
            name="slider remains captured at heat end",
        )
        heat_pos = ctx.part_world_position(slider)
    with ctx.pose({slide_joint: -0.014}):
        ctx.expect_within(
            slider,
            housing,
            axes="xz",
            inner_elem="slot_runner",
            outer_elem="mode_slot",
            margin=0.001,
            name="slider remains captured at cool end",
        )
        cool_pos = ctx.part_world_position(slider)
    ctx.check(
        "mode slider translates horizontally",
        rest_pos is not None
        and heat_pos is not None
        and cool_pos is not None
        and heat_pos[0] > rest_pos[0] + 0.010
        and cool_pos[0] < rest_pos[0] - 0.010,
        details=f"cool={cool_pos}, rest={rest_pos}, heat={heat_pos}",
    )

    return ctx.report()


object_model = build_object_model()

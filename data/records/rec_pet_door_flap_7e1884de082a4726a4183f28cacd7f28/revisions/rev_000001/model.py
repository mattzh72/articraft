from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_flap")

    frame_color = model.material("frame_white", color=(0.93, 0.93, 0.90))
    frame_trim = model.material("frame_trim", color=(0.18, 0.18, 0.20))
    flap_tint = model.material("flap_tint", rgba=(0.22, 0.28, 0.33, 0.58))

    frame_depth = 0.028
    outer_width = 0.34
    outer_height = 0.46
    border = 0.04
    opening_width = outer_width - 2.0 * border
    opening_height = outer_height - 2.0 * border

    frame = model.part("frame")
    frame.visual(
        Box((frame_depth, outer_width, border)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0 - border / 2.0)),
        material=frame_color,
        name="top_header",
    )
    frame.visual(
        Box((frame_depth, outer_width, border)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height / 2.0 + border / 2.0)),
        material=frame_color,
        name="bottom_sill",
    )
    frame.visual(
        Box((frame_depth, border, opening_height + 0.004)),
        origin=Origin(xyz=(0.0, -outer_width / 2.0 + border / 2.0, 0.0)),
        material=frame_color,
        name="left_jamb",
    )
    frame.visual(
        Box((frame_depth, border, opening_height + 0.004)),
        origin=Origin(xyz=(0.0, outer_width / 2.0 - border / 2.0, 0.0)),
        material=frame_color,
        name="right_jamb",
    )
    frame.visual(
        Box((0.008, opening_width + 0.028, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, outer_height / 2.0 - border - 0.009)),
        material=frame_trim,
        name="inner_stop",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(0.006, -0.135, opening_height / 2.0 - 0.008),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_trim,
        name="left_hinge_knuckle",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(0.006, 0.135, opening_height / 2.0 - 0.008),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_trim,
        name="right_hinge_knuckle",
    )
    frame.visual(
        Box((0.012, 0.016, 0.016)),
        origin=Origin(xyz=(0.006, -0.135, opening_height / 2.0 - 0.006)),
        material=frame_trim,
        name="left_hinge_bracket",
    )
    frame.visual(
        Box((0.012, 0.016, 0.016)),
        origin=Origin(xyz=(0.006, 0.135, opening_height / 2.0 - 0.006)),
        material=frame_trim,
        name="right_hinge_bracket",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_depth, outer_width, outer_height)),
        mass=1.4,
    )

    flap = model.part("flap")
    flap_width = 0.248
    panel_height = 0.352
    panel_thickness = 0.004
    cap_thickness = 0.010
    cap_height = 0.018
    hinge_radius = 0.006
    hinge_length = 0.252

    flap.visual(
        Box((panel_thickness, flap_width, panel_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.182)),
        material=flap_tint,
        name="panel",
    )
    flap.visual(
        Box((cap_thickness, hinge_length, cap_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=frame_trim,
        name="top_cap",
    )
    flap.visual(
        Cylinder(radius=hinge_radius, length=hinge_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_trim,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.012, flap_width, panel_height + 0.020)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.176)),
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.006, 0.0, opening_height / 2.0 - 0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=-1.10,
            upper=1.10,
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

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    left_jamb = frame.get_visual("left_jamb")
    right_jamb = frame.get_visual("right_jamb")
    bottom_sill = frame.get_visual("bottom_sill")
    top_header = frame.get_visual("top_header")
    left_hinge_knuckle = frame.get_visual("left_hinge_knuckle")
    right_hinge_knuckle = frame.get_visual("right_hinge_knuckle")
    panel = flap.get_visual("panel")
    hinge_barrel = flap.get_visual("hinge_barrel")

    limits = hinge.motion_limits
    ctx.check(
        "hinge axis is horizontal across the opening",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "flap swings both inward and outward",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < -0.5
        and limits.upper > 0.5,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            frame,
            axis="y",
            positive_elem=panel,
            negative_elem=left_jamb,
            min_gap=0.004,
            max_gap=0.012,
            name="closed flap clears left jamb",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="y",
            positive_elem=right_jamb,
            negative_elem=panel,
            min_gap=0.004,
            max_gap=0.012,
            name="closed flap clears right jamb",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem=panel,
            negative_elem=bottom_sill,
            min_gap=0.010,
            max_gap=0.020,
            name="closed flap stops just above the sill",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            positive_elem=top_header,
            negative_elem=hinge_barrel,
            min_gap=0.001,
            max_gap=0.004,
            name="hinge barrel nests under the header",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_barrel,
            elem_b=left_hinge_knuckle,
            contact_tol=1e-6,
            name="left hinge knuckle touches the barrel",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a=hinge_barrel,
            elem_b=right_hinge_knuckle,
            contact_tol=1e-6,
            name="right hinge knuckle touches the barrel",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(flap, elem=panel)

    upper = limits.upper if limits is not None and limits.upper is not None else 0.9
    lower = limits.lower if limits is not None and limits.lower is not None else -0.9

    with ctx.pose({hinge: upper}):
        open_outward_aabb = ctx.part_element_world_aabb(flap, elem=panel)

    with ctx.pose({hinge: lower}):
        open_inward_aabb = ctx.part_element_world_aabb(flap, elem=panel)

    ctx.check(
        "positive hinge motion swings the flap outward",
        closed_panel_aabb is not None
        and open_outward_aabb is not None
        and open_outward_aabb[1][0] > closed_panel_aabb[1][0] + 0.10,
        details=f"closed={closed_panel_aabb}, outward={open_outward_aabb}",
    )
    ctx.check(
        "negative hinge motion swings the flap inward",
        closed_panel_aabb is not None
        and open_inward_aabb is not None
        and open_inward_aabb[0][0] < closed_panel_aabb[0][0] - 0.10,
        details=f"closed={closed_panel_aabb}, inward={open_inward_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
